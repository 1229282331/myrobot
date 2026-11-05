"""Human logs, independent of the lossless audit and the device deadlines.

Producers only enqueue LogRecords. Formatting, traceback rendering and file IO
belong to a bounded, best-effort worker. No stdout is used (Servo stdout is IPC).
"""
import codecs
import logging
from logging.handlers import WatchedFileHandler
import os
import queue
import threading
from pathlib import Path


class _Utf8WatchedFile:
    """Writer-owned stream with stdlib's watched-file rotation algorithm.

    Deliberately not a Handler: logging.shutdown must not register this stream
    and flush/close blocked file IO on the control owner's exit thread.
    """
    def __init__(self,path):
        self.baseFilename=os.path.abspath(path)
        self.mode='a';self.stream=None;self.dev=-1;self.ino=-1

    _statstream=WatchedFileHandler._statstream
    reopenIfNeeded=WatchedFileHandler.reopenIfNeeded

    def _open(self):
        return open(self.baseFilename, self.mode, encoding='utf-8', newline='\n')

    def flush(self):
        if self.stream is not None:self.stream.flush()

    def close(self):
        if self.stream is not None:
            stream=self.stream;self.stream=None
            stream.close()


class AsyncLog(logging.Handler):
    def __init__(self, path, capacity=1024):
        super().__init__()
        self.path=Path(path)
        self.queue=queue.Queue(capacity)
        self.halt=threading.Event()
        self.dropped=0
        self.dropped_native_bytes=0
        self.write_errors=0
        self.error=None
        self.previous={}
        # Only the writer thread touches this delayed stream. Reuse stdlib's
        # inode/rotation handling instead of opening and closing for each ROS
        # message; on eMMC those metadata operations can stall for seconds.
        self.output=_Utf8WatchedFile(self.path)
        self.formatter=logging.Formatter('%(asctime)s %(levelname)-7s %(name)s | %(message)s',
                                         datefmt='%Y-%m-%dT%H:%M:%S%z')
        self.thread=threading.Thread(target=self._run,name='human-log-writer',daemon=True)
        self.thread.start()

    def emit(self, record):
        try:self.queue.put_nowait(record)
        except queue.Full:
            self.dropped+=1
            self.dropped_native_bytes+=getattr(record,'native_bytes',0)

    def native(self,text,byte_count):
        record=logging.LogRecord('e6.native',logging.INFO,'',0,'',(),None)
        record.native_text=text;record.native_bytes=byte_count
        self.emit(record)

    def _changed(self, key, value):
        if key in self.previous and self.previous[key]==value:return False
        self.previous[key]=value
        return True

    def _describe(self, record):
        event=getattr(record,'operator_event',None)
        if event is None:return record.getMessage()
        kind=event.get('event')
        if kind=='cycle':
            from .state_observation import observation_fields
            state=observation_fields(event['state']);messages=[]
            for key,value,label in (
                ('control',(state.get('state'),state.get('ready'),state.get('reason')),'Control status / ready / reason'),
                ('feedback',state.get('feedback',{}).get('connected'),'Feedback connected'),
                ('payload',state.get('arm',{}).get('payload_matches'),'Configured payload matches feedback'),
                ('gripper',(state.get('gripper',{}).get('phase'),state.get('gripper',{}).get('reason')),'Gripper phase / reason'),
                ('gripper_collision',state.get('gripper',{}).get('collision_blocked'),'Gripper geometry permit wait'),
                ('recording',(state.get('recording',{}).get('phase'),state.get('recording',{}).get('error')),'Recording phase / reason'),
                ('camera',(state.get('camera_restart',{}).get('phase'),state.get('camera_restart',{}).get('error')),'Camera recovery / reason'),
                ('boundary',state.get('model',{}).get('status') if state.get('model') else None,'Servo status'),
            ):
                if self._changed(key,value):messages.append(f'{label}: {value}')
            return '; '.join(messages) or None
        labels={
            'prepared':'Control prepared', 'control_refused':'Controller refused',
            'enable_requested':'Enable requested', 'enable_result':'Enable result',
            'enable_observed':'Enable observation completed', 'enable_uncertain':'Enable outcome uncertain',
            'camera_restart_requested':'Camera reconnect requested',
            'recording_request_result':'Recording request result',
            'release_deceleration_requested':'Input released; deceleration requested',
            'stop_requested':'Stop requested', 'stop_result':'Stop result',
            'workspace_blocked':'Workspace boundary reached', 'motion_uncertain':'Motion outcome uncertain',
            'control_direction_limited':'Direction limited; control stays prepared; reverse or release to resume',
            'shutdown_state':'Control resources released', 'feedback_transport':'Feedback transport',
            'gripper_connecting':'Gripper attaching without position command',
            'gripper_connected':'Gripper attached at observed position',
            'gripper_fault':'Gripper fault; no automatic retry',
            'gripper_contact':'Gripper obstructed; target frozen, feedback continues (not proof of successful grasp)',
            'gripper_object_dropped':'Gripper reports object dropped; observing only. Release input, then explicitly open; no automatic regrasp',
            'gripper_contact_released':'Gripper reverse motion confirmed (not proof of object release)',
            'gripper_hold':'Gripper input released; holding bounded target',
        }
        if kind=='workspace_blocked':
            reason=event.get('reason')
            if not self._changed('workspace_blocked',reason):return None
        elif kind=='motion_submitted':self.previous.pop('workspace_blocked',None)
        # Individual input/target/feedback events are DEBUG, never an INFO flood.
        if kind not in labels and record.levelno>logging.DEBUG:return None
        detail='; '.join(f'{key}={value}' for key,value in event.items()
                        if key not in ('event','at_ns','state','pre_action_observation'))
        return labels.get(kind,kind)+(': '+detail if detail else '')

    def _write(self, text):
        self._append(text+'\n')

    def _write_native(self,text):
        self._append(text)

    def _append(self,text):
        if self.output.stream is None:
            self.path.parent.mkdir(parents=True,exist_ok=True)
        self.output.reopenIfNeeded()
        if self.output.stream is None:
            self.output.stream=self.output._open()
            self.output._statstream()
        self.output.stream.write(text)

    def _failed_write(self,exc):
        self.write_errors+=1;self.error=str(exc)

    def _run(self):
        reported=0
        try:
            while not self.halt.is_set() or not self.queue.empty():
                try:record=self.queue.get(timeout=.1)
                except queue.Empty:continue
                batch=[record]
                # Drain an already queued, bounded burst. Never wait to fill a
                # batch or buffer behind a timer: sparse messages stay visible.
                for _ in range(63):
                    try:batch.append(self.queue.get_nowait())
                    except queue.Empty:break
                for record in batch:
                    try:
                        if hasattr(record,'native_text'):
                            self._write_native(record.native_text)
                        else:
                            message=self._describe(record)
                            if message is not None:
                                record.msg=message;record.args=()
                                self._write(self.formatter.format(record))
                        if self.dropped!=reported:
                            self._write(self.formatter.format(logging.LogRecord('logs',logging.WARNING,'',0,
                                'Display log queue overflow: %s records dropped; structured audit is independent',
                                (self.dropped-reported,),None)))
                            reported=self.dropped
                    except Exception as exc:self._failed_write(exc)
                try:self.output.flush()
                except Exception as exc:self._failed_write(exc)
                finally:
                    for _ in batch:self.queue.task_done()
        finally:
            # Closing a blocked file belongs here, never on the control owner.
            # close() still reports a live thread if the bounded join expires.
            try:self.output.close()
            except Exception as exc:self._failed_write(exc)

    def close(self):
        self.halt.set()
        self.thread.join(timeout=1)
        super().close()


class NativeLogDrain:
    """Drain native stderr without doing file IO or accumulating unbounded lines.

    The native child must disable ROS's duplicate external file logger. Its
    console output stays visible in the existing bounded human-log sink;
    per-cycle structured audit and control IPC are independent.
    """
    def __init__(self,source,sink):
        self.source=source;self.sink=sink
        self.bytes=0;self.error=None;self.eof=False
        self.thread=threading.Thread(target=self._run,name='native-log-drain',daemon=True)
        self.thread.start()

    def _run(self):
        decoder=codecs.getincrementaldecoder('utf-8')(errors='replace')
        try:
            while True:
                data=os.read(self.source.fileno(),4096)
                if not data:
                    tail=decoder.decode(b'',final=True)
                    if tail:self.sink.native(tail,0)
                    self.eof=True;return
                self.bytes+=len(data)
                self.sink.native(decoder.decode(data),len(data))
        except Exception as exc:self.error=f'{type(exc).__name__}: {exc}'

    def close(self):
        # Called after the child exits: its private stderr pipe should reach EOF.
        self.thread.join(timeout=1)
        if not self.thread.is_alive():self.source.close()
        return dict(bytes=self.bytes,eof=self.eof,error=self.error,
                    drain_alive=self.thread.is_alive(),dropped_chunks=self.sink.dropped,
                    dropped_native_bytes=self.sink.dropped_native_bytes)


def start_logging(path, name='e6', capacity=1024):
    sink=AsyncLog(path,capacity)
    logger=logging.getLogger(name)
    logger.setLevel(logging.INFO);logger.propagate=False;logger.addHandler(sink)
    return logger,sink


def audit_message(logger,event):
    kind=event.get('event')
    level=logging.DEBUG if kind in ('input','input_processed','motion_submitted','motion_confirmed',
                                    'can','clock_sample') else logging.INFO
    if kind=='feedback_transport' and event.get('data',{}).get('event')=='receive_timing':level=logging.DEBUG
    logger.log(level,'',extra={'operator_event':event})
    if kind=='cycle' and logger.isEnabledFor(logging.DEBUG):
        from .state_observation import observation_fields
        state=observation_fields(event['state'])
        logger.debug('Cycle actual / candidate / accepted: %s / %s / %s',
            state.get('actual'),state.get('candidate'),state.get('accepted'))
