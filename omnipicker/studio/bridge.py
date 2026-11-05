from __future__ import annotations

from datetime import datetime
from pathlib import Path
import threading
import uuid
from PySide6.QtCore import QObject, Property, Signal, Slot, QThread
from .control import ControlEngine
from .device import ROOT, LIMIT_NAMES
from .recording import Recorder


class Worker(QThread):
    event = Signal(str, object)

    def __init__(self, log_path, run_id):
        super().__init__()
        self.run_id = run_id
        self.engine = ControlEngine(self.dispatch)
        self.recorder = Recorder(log_path, lambda message:self.event.emit("recording_error", message))
        self.finished_event = threading.Event()

    def dispatch(self, kind, data):
        if kind in ("log", "telemetry", "observation"):
            state = self.engine.state
            metadata = dict(software="0.2.1", run_id=self.run_id, connection_id=state["connectionId"],
                            simulated=state["simulated"], firmware=state["firmware"], serial=state["serial"])
            self.recorder.submit(kind, data, metadata)
        if kind != "observation":
            self.event.emit(kind, data)

    def run(self):
        try:
            self.engine.publish()
            while not self.finished_event.is_set():
                self.engine.tick()
                if self.engine.quit_requested:
                    return
                self.msleep(5)
        finally:
            try:
                self.engine.release_resources()
            finally:
                self.recorder.close()
                if self.engine.quit_requested: self.event.emit("quit", {})


class Bridge(QObject):
    changed = Signal()
    targetChanged = Signal()
    logsChanged = Signal()
    quitReady = Signal()
    captureRequested = Signal()
    recordingChanged = Signal()

    def __init__(self):
        super().__init__()
        self._target = 15.0
        self._logs = []
        self._recording_error = ""
        directory = ROOT / "logs"
        directory.mkdir(exist_ok=True)
        run_id = uuid.uuid4().hex
        self.log_path = directory / (datetime.now().strftime("studio-%Y%m%d-%H%M%S-")+run_id[:8]+".jsonl")
        self.worker = Worker(self.log_path, run_id)
        self._state = dict(self.worker.engine.state)
        self.worker.event.connect(self.receive)
        self.worker.start()

    state = Property("QVariantMap", lambda self:self._state, notify=changed)
    target = Property(float, lambda self:self._target, notify=targetChanged)
    logs = Property("QVariantList", lambda self:self._logs, notify=logsChanged)
    logFile = Property(str, lambda self:str(self.log_path), constant=True)
    recordingError = Property(str, lambda self:self._recording_error or self.worker.recorder.error or "", notify=recordingChanged)

    @Slot(str,object)
    def receive(self, kind, data):
        if kind == "state":
            self._state = data
            self.changed.emit()
            if data["testActive"] and data["ackTarget"] >= 0:
                self._target = data["ackTarget"]
                self.targetChanged.emit()
        elif kind in ("log","telemetry"):
            data = dict(data, time=datetime.now().isoformat(timespec="milliseconds"))
            if kind == "log": self._logs = [data] + self._logs[:299]
            if kind == "log": self.logsChanged.emit()
        elif kind == "recording_error":
            self._recording_error = str(data)
            self._logs = [dict(time=datetime.now().isoformat(), message=str(data), level="error")] + self._logs[:299]
            self.recordingChanged.emit()
            self.logsChanged.emit()
        elif kind == "quit": self.quitReady.emit()

    @Slot(bool,str)
    def connectDevice(self, simulated, serial):
        self.worker.engine.submit("connect",(simulated,serial))

    @Slot(str)
    def action(self, name):
        if name in ("enable","stop","restore","clear","mock_fault","mock_timeout","mock_feedback_pause"):
            self.worker.engine.submit(name)

    @Slot(float,bool)
    def editTarget(self, value, send=True):
        if self._state["testActive"]: return
        if not 0 <= value <= 100: return
        self._target = round(value,1)
        self.targetChanged.emit()
        if (send and self._state["enableState"] == "enabled" and not self._state["locked"]
                and (not self._state["feedbackRequired"] or self._state["feedbackValid"])):
            self.worker.engine.submit("target",self._target)

    @Slot("QVariantMap")
    def applyLimits(self, values):
        try: values = {n:float(values[n]) for n in LIMIT_NAMES}
        except (ValueError,KeyError): return
        self.worker.engine.submit("apply",values)

    @Slot(float,float,float,int)
    def startTest(self,a,b,dwell,cycles):
        self.worker.engine.submit("test",(a,b,dwell,cycles))

    @Slot(str,str)
    def leave(self, action, mode):
        if action in ("close","disconnect") and mode in ("keep","disable"):
            self.worker.engine.submit(action,mode)

    @Slot()
    def capture(self): self.captureRequested.emit()

    def shutdown(self):
        self.worker.finished_event.set()
        self.worker.wait(5000)
