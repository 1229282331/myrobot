import React from 'react';
import { act, cleanup, fireEvent, render, renderHook } from '@testing-library/react';
import { afterEach, beforeEach, expect, it, vi } from 'vitest';
import { SWRConfig } from 'swr';
import { useControlServerStatus, ServerStatusRecovery } from '../frontend/useControlServerStatus';

const good = {robot_status:[{device_name:'fixture'}]};
const reply = (value=good) => ({ok:true,json:async()=>value});
function boot() {
  const cache=new Map();
  return renderHook(()=>useControlServerStatus(), {wrapper:({children}) =>
    <SWRConfig value={{provider:()=>cache,revalidateOnFocus:false}}>{children}</SWRConfig>});
}
const flush=async()=>{await act(async()=>{await Promise.resolve();});};
beforeEach(()=>{vi.useFakeTimers();});
afterEach(()=>{cleanup();vi.useRealTimers();vi.unstubAllGlobals();});

it('recovers from connection refusal without reload and sends only status reads',async()=>{
  const fetch=vi.fn().mockRejectedValueOnce(new Error('offline')).mockResolvedValue(reply());
  vi.stubGlobal('fetch',fetch);
  const {result}=boot();await flush();
  expect(result.current.error).toBeTruthy();
  await act(async()=>{await vi.advanceTimersByTimeAsync(1000);});
  expect(result.current.error).toBeUndefined();expect(result.current.data).toEqual(good);
  expect(fetch).toHaveBeenCalledTimes(2);
  for(const [url,options] of fetch.mock.calls){expect(url).toBe('/status');expect(options.method).toBeUndefined();}
});

it('retains cached discovery through downtime and reads the recovered server',async()=>{
  const fetch=vi.fn().mockResolvedValueOnce(reply()).mockResolvedValueOnce({ok:false,status:503})
    .mockResolvedValue(reply({robot_status:[{device_name:'new-session'}]}));
  vi.stubGlobal('fetch',fetch);const {result}=boot();await flush();
  await act(async()=>{await result.current.mutate().catch(()=>{});});
  expect(result.current.error).toBeTruthy();expect(result.current.data).toEqual(good);
  await act(async()=>{await vi.advanceTimersByTimeAsync(1000);});
  expect(result.current.data?.robot_status[0].device_name).toBe('new-session');
});

it('times out a hung request before retrying, without concurrent requests',async()=>{
  let active=0,maximum=0;
  const fetch=vi.fn((_url,options)=>{active++;maximum=Math.max(maximum,active);
    return new Promise((_resolve,reject)=>options.signal.addEventListener('abort',()=>{
      active--;reject(new Error('aborted'));
    }));});
  vi.stubGlobal('fetch',fetch);const {result}=boot();await flush();
  await act(async()=>{await vi.advanceTimersByTimeAsync(1500);});
  expect(result.current.error).toBeTruthy();expect(active).toBe(0);
  await act(async()=>{await vi.advanceTimersByTimeAsync(1000);});
  expect(fetch).toHaveBeenCalledTimes(2);expect(maximum).toBe(1);
});

it('cancels automatic retry when the control page unmounts',async()=>{
  const fetch=vi.fn().mockRejectedValue(new Error('offline'));vi.stubGlobal('fetch',fetch);
  const {unmount}=boot();await flush();unmount();
  await act(async()=>{await vi.advanceTimersByTimeAsync(20000);});
  expect(fetch).toHaveBeenCalledTimes(1);
});

it('offers an explicit read retry and never labels discovery as prepared control',async()=>{
  const retry=vi.fn().mockResolvedValue(undefined);
  const v=render(<ServerStatusRecovery retry={retry} pending={false}/>);
  expect(v.getByRole('status').textContent).toContain('重新准备控制');
  fireEvent.click(v.getByRole('button',{name:'立即重试连接'}));await flush();
  expect(retry).toHaveBeenCalledTimes(1);
  v.rerender(<ServerStatusRecovery retry={retry} pending/>);
  expect((v.getByRole('button') as HTMLButtonElement).disabled).toBe(true);
});
