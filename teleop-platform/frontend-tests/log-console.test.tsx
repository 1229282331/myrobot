import React from 'react';
import {act,cleanup,fireEvent,render,screen} from '@testing-library/react';
import {afterEach,expect,it,vi} from 'vitest';
import {LogStream} from '@/components/custom/LogsStream';
import {LogConsole} from '../frontend/LogConsole';
import {boundedLog,MAX_LOG_BYTES,MAX_LOG_LINES} from '../frontend/logText';

afterEach(()=>{cleanup();vi.useRealTimers();vi.unstubAllGlobals();});
const tick=async(ms=0)=>act(async()=>{await vi.advanceTimersByTimeAsync(ms);});
const response=(chunks:Uint8Array[],cursor:string,reset=false)=>new Response(new ReadableStream({start(controller){
  chunks.forEach(chunk=>controller.enqueue(chunk));controller.close();
}}),{headers:{'X-Log-Cursor':cursor,'X-Log-Reset':reset?'1':'0'}});
const bytes=(text:string)=>new TextEncoder().encode(text);

it('bounds UTF8 bytes and lines without damaging multibyte characters',()=>{
  const text=boundedLog('中'.repeat(MAX_LOG_BYTES));
  expect(new TextEncoder().encode(text).length).toBeLessThanOrEqual(MAX_LOG_BYTES);
  expect(text.includes('\ufffd')).toBe(false);
  const lines=boundedLog(Array.from({length:7000},(_,i)=>'line '+i).join('\n'));
  expect(lines.split('\n')).toHaveLength(MAX_LOG_LINES);expect(lines.endsWith('line 6999')).toBe(true);
});

it('stays collapsed until opened and aborts on collapse without control connections',async()=>{
  vi.useFakeTimers();const socket=vi.fn();vi.stubGlobal('WebSocket',socket);
  let signal:AbortSignal|undefined;
  const fetch=vi.fn((_url,options)=>{signal=options.signal;return new Promise(()=>{});});vi.stubGlobal('fetch',fetch);
  const view=render(<LogConsole/>);expect(fetch).not.toHaveBeenCalled();
  fireEvent.click(screen.getByRole('button',{name:'Logs'}));await tick();
  expect(fetch).toHaveBeenCalledTimes(1);expect(socket).not.toHaveBeenCalled();
  expect(screen.getByLabelText('Program output').className).toContain('h-64');
  fireEvent.click(screen.getByRole('button',{name:'Logs'}));expect(signal?.aborted).toBe(true);
  await tick(1000);expect(fetch).toHaveBeenCalledTimes(1);view.unmount();
});

it('preserves split Chinese and multiline traces, resumes cursor, clears display only and resets on rotation',async()=>{
  vi.useFakeTimers();const text=bytes('中文\nTraceback:\n  original\n');
  const fetch=vi.fn().mockResolvedValueOnce(response([text.slice(0,2)],'one',true))
    .mockRejectedValueOnce(new Error('SSH offline'))
    .mockResolvedValueOnce(response([text.slice(2,7),text.slice(7)],'two'))
    .mockResolvedValueOnce(response([bytes('next\n')],'three'))
    .mockResolvedValueOnce(response([bytes('rotated\n')],'four',true));
  vi.stubGlobal('fetch',fetch);
  render(<LogStream logFile={null} isLoading={false} streamUrl="/e6/logs/control/stream"/>);
  await tick();await tick(500);expect(screen.getByRole('status').textContent).toContain('SSH offline');
  await tick(500);expect(screen.getByLabelText('Program output').textContent).toBe('中文\nTraceback:\n  original\n');
  expect(fetch.mock.calls[2][0]).toContain('cursor=one');
  fireEvent.click(screen.getByLabelText('Clear display'));await tick(500);
  expect(screen.getByLabelText('Program output').textContent).toBe('next\n');
  expect(fetch.mock.calls[3][0]).toContain('cursor=two');
  await tick(500);expect(screen.getByLabelText('Program output').textContent).toBe('rotated\n');
});

it('pause follow keeps receiving text while scroll stays fixed; follow resumes; unmount cancels timers',async()=>{
  vi.useFakeTimers();let n=0;
  const fetch=vi.fn(async()=>response([bytes('line '+n+'\n')],String(++n)));vi.stubGlobal('fetch',fetch);
  const view=render(<LogStream logFile={null} isLoading={false} streamUrl="/e6/logs/pc/stream"/>);await tick();
  const terminal=screen.getByLabelText('Program output');
  Object.defineProperty(terminal,'scrollHeight',{value:500,configurable:true});terminal.scrollTop=10;
  fireEvent.click(screen.getByRole('button',{name:'Pause follow'}));await tick(500);
  expect(terminal.scrollTop).toBe(10);expect(terminal.textContent).toContain('line 1');
  fireEvent.click(screen.getByRole('button',{name:'Follow'}));expect(terminal.scrollTop).toBe(500);
  view.unmount();const count=fetch.mock.calls.length;await tick(1000);expect(fetch).toHaveBeenCalledTimes(count);
});

it('switching sources aborts the old request and ignores its late response',async()=>{
  vi.useFakeTimers();let resolve:(r:Response)=>void=()=>{};let first:AbortSignal|undefined;
  const fetch=vi.fn().mockImplementationOnce((_url,options)=>{first=options.signal;return new Promise(r=>resolve=r);})
    .mockResolvedValueOnce(response([bytes('camera native')],'camera',true));vi.stubGlobal('fetch',fetch);
  const view=render(<LogStream logFile={null} isLoading={false} streamUrl="/e6/logs/control/stream"/>);await tick();
  view.rerender(<LogStream logFile={null} isLoading={false} streamUrl="/e6/logs/camera/stream"/>);await tick();
  expect(first?.aborted).toBe(true);
  await act(async()=>resolve(response([bytes('old control')],'old')));
  expect(screen.getByLabelText('Program output').textContent).toBe('camera native');
});

it('keeps the upstream training endpoint and plain stream compatible',async()=>{
  vi.useFakeTimers();const fetch=vi.fn(async()=>response([bytes('upstream training\n')],'unused'));vi.stubGlobal('fetch',fetch);
  render(<LogStream logFile="training.log" isLoading={false}/>);await tick();
  expect(fetch.mock.calls[0][0]).toBe('/training/logs/training.log');
  expect(screen.getByLabelText('Program output').textContent).toBe('upstream training\n');
  await tick(1500);expect(fetch).toHaveBeenCalledTimes(1);
});

it('times out a stalled finite poll and retries the same cursor without losing text',async()=>{
  vi.useFakeTimers();let stalled:AbortSignal|undefined;
  const fetcher=vi.fn().mockResolvedValueOnce(response([bytes('before\n')],'one'))
    .mockImplementationOnce((_url,options)=>new Promise((_resolve,reject)=>{
      stalled=options.signal;stalled!.addEventListener('abort',()=>reject(new DOMException('aborted','AbortError')));
    }))
    .mockResolvedValueOnce(response([bytes('after\n')],'two'));
  vi.stubGlobal('fetch',fetcher);
  render(<LogStream logFile={null} isLoading={false} streamUrl="/e6/logs/control/stream"/>);
  await tick();await tick(500);await tick(5000);
  expect(stalled?.aborted).toBe(true);expect(screen.getByRole('status').textContent).toContain('timed out');
  expect(screen.getByLabelText('Program output').textContent).toBe('before\n');
  await tick(500);expect(fetcher.mock.calls[2][0]).toContain('cursor=one');
  expect(screen.getByLabelText('Program output').textContent).toBe('before\nafter\n');
});

it('discards a partial timed-out response and reconstructs UTF8 on retry',async()=>{
  vi.useFakeTimers();let stalled:AbortSignal|undefined;
  const text=bytes('中文\n');
  const fetcher=vi.fn().mockResolvedValueOnce(response([bytes('before\n')],'one'))
    .mockImplementationOnce((_url,options)=>{
      stalled=options.signal;
      return Promise.resolve(new Response(new ReadableStream({start(controller){
        controller.enqueue(text.slice(0,2));
        stalled!.addEventListener('abort',()=>controller.error(new DOMException('aborted','AbortError')));
      }}),{headers:{'X-Log-Cursor':'uncommitted'}}));
    }).mockResolvedValueOnce(response([text],'two'));
  vi.stubGlobal('fetch',fetcher);
  const view=render(<LogStream logFile={null} isLoading={false} streamUrl="/e6/logs/control/stream"/>);
  await tick();await tick(5500);expect(stalled?.aborted).toBe(true);
  expect(screen.getByLabelText('Program output').textContent).toBe('before\n');
  await tick(500);expect(fetcher.mock.calls[2][0]).toContain('cursor=one');
  expect(screen.getByLabelText('Program output').textContent).toBe('before\n中文\n');
  view.unmount();expect(vi.getTimerCount()).toBe(0);
});
