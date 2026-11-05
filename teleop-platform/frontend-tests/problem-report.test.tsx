import React from 'react';
import {act, cleanup, fireEvent, render} from '@testing-library/react';
import {afterEach, expect, it, vi} from 'vitest';
import {ProblemReport} from '../frontend/ProblemReport';

const receipt={name:`problem-${'a'.repeat(32)}.zip`,url:`/e6/diagnostics/problem/problem-${'a'.repeat(32)}.zip`,
  sha256:'b'.repeat(64),bytes:2048,files:3,omissions:2};
const reply=(result=receipt)=>({ok:true,json:async()=>result});
const flush=async()=>{await act(async()=>{await Promise.resolve();});};
afterEach(()=>{cleanup();vi.unstubAllGlobals();});

it('saves locally and offers only the verified same-origin download',async()=>{
  const fetch=vi.fn().mockResolvedValue(reply());vi.stubGlobal('fetch',fetch);
  const v=render(<ProblemReport/>);
  fireEvent.click(v.getByRole('button',{name:'Problem report'}));
  fireEvent.click(v.getByRole('button',{name:'Export problem report'}));await flush();
  expect(fetch.mock.calls).toEqual([['/e6/diagnostics/problem',{method:'POST'}]]);
  expect(v.getByRole('link',{name:'Download report'}).getAttribute('href')).toBe(receipt.url);
  expect(v.getByRole('status').textContent).toContain('2 KiB');
  expect(v.getByRole('status').textContent).toContain('2项未纳入');
});

it('prevents rapid repeated clicks while the write is pending',async()=>{
  let resolve:(value:unknown)=>void=()=>{};
  const fetch=vi.fn().mockImplementation(()=>new Promise(r=>{resolve=r;}));vi.stubGlobal('fetch',fetch);
  const v=render(<ProblemReport/>);fireEvent.click(v.getByRole('button',{name:'Problem report'}));
  const button=v.getByRole('button',{name:'Export problem report'});
  act(()=>{fireEvent.click(button);fireEvent.click(button);});
  expect(fetch).toHaveBeenCalledTimes(1);
  expect((button as HTMLButtonElement).disabled).toBe(true);
  await act(async()=>{resolve(reply());});
  expect(v.getByRole('link')).toBeTruthy();
});

it.each([409,507])('allows retry after HTTP %i and does not label an old receipt as new',async(status)=>{
  const fetch=vi.fn().mockResolvedValueOnce(reply()).mockResolvedValueOnce({ok:false,status}).mockResolvedValue(reply());
  vi.stubGlobal('fetch',fetch);
  const v=render(<ProblemReport/>);fireEvent.click(v.getByRole('button',{name:'Problem report'}));
  fireEvent.click(v.getByRole('button',{name:'Export problem report'}));await flush();
  expect(v.getByRole('link')).toBeTruthy();
  fireEvent.click(v.getByRole('button',{name:'Export problem report'}));await flush();
  expect(v.getByRole('alert')).toBeTruthy();expect(v.queryByRole('link')).toBeNull();
  fireEvent.click(v.getByRole('button',{name:'Export problem report'}));await flush();
  expect(v.queryByRole('alert')).toBeNull();expect(v.getByRole('link')).toBeTruthy();
});

it.each([{url:'https://other.example/report.zip'},{sha256:'wrong'},{omissions:-1},{files:1.2},{bytes:0}])(
  'rejects malformed or external download receipts: %j',async(patch)=>{
    vi.stubGlobal('fetch',vi.fn().mockResolvedValue(reply({...receipt,...patch})));
    const v=render(<ProblemReport/>);fireEvent.click(v.getByRole('button',{name:'Problem report'}));
    fireEvent.click(v.getByRole('button',{name:'Export problem report'}));await flush();
    expect(v.getByRole('alert').textContent).toContain('回执无效');expect(v.queryByRole('link')).toBeNull();
  });
