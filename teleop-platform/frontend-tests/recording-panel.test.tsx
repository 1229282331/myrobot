import React from 'react';
import {act,cleanup,fireEvent,render,renderHook,screen} from '@testing-library/react';
import {afterEach,expect,it,vi} from 'vitest';
import {SWRConfig} from 'swr';
import {Recorder} from '@/components/common/recorder';
import {EpisodeTransferProgress,useNativeRecording} from '../frontend/EpisodeBrowser';
vi.mock('react-router-dom',()=>({useNavigate:()=>vi.fn()}));
vi.mock('@/lib/hooks',()=>({useGlobalStore:()=>[]}));

afterEach(()=>{cleanup();localStorage.clear();vi.unstubAllGlobals();vi.useRealTimers();});

it('polls the record library only during transfer and refreshes the final receipt',async()=>{
  vi.useFakeTimers();const fetcher=vi.fn(async()=>new Response(JSON.stringify({episodes:[]})));
  vi.stubGlobal('fetch',fetcher);
  const cache=new Map();const view=renderHook(()=>useNativeRecording(),{wrapper:({children})=><SWRConfig value={{provider:()=>cache,
    revalidateOnMount:false,revalidateOnFocus:false,dedupingInterval:0,
    fallback:{'/e6/capabilities':{original_recording:true},'/e6/episodes':{episodes:[]}}}}>{children}</SWRConfig>});
  await act(async()=>{state('idle');await vi.advanceTimersByTimeAsync(10000);});
  expect(fetcher).not.toHaveBeenCalled();
  await act(async()=>window.dispatchEvent(new CustomEvent('e6-recording-state',{detail:{phase:'transfer_pending',episode:'test'}})));
  await act(async()=>vi.advanceTimersByTimeAsync(4100));
  expect(fetcher.mock.calls.length).toBeGreaterThanOrEqual(2);
  const before=fetcher.mock.calls.length;
  await act(async()=>window.dispatchEvent(new CustomEvent('e6-recording-state',{detail:{phase:'complete',episode:'test'}})));
  expect(fetcher.mock.calls.length).toBeGreaterThan(before);
  const finished=fetcher.mock.calls.length;
  await act(async()=>vi.advanceTimersByTimeAsync(10000));expect(fetcher).toHaveBeenCalledTimes(finished);
  view.unmount();
});
const state=(phase:string)=>window.dispatchEvent(new CustomEvent('e6-recording-state',{detail:{phase,samples:23}}));
function mount(enabled=true,episodes:unknown[]=[]){
  vi.stubGlobal('fetch',vi.fn(()=>Promise.reject(new Error('No fixture network'))));
  return render(<SWRConfig value={{provider:()=>new Map(),revalidateOnMount:false,revalidateOnFocus:false,
    fallback:{'/e6/capabilities':{enabled,original_recording:enabled},'/e6/episodes':{episodes},
      '/admin/settings':{task_instruction:'Upstream task',dataset_name:'fixture'}}}}>
    <Recorder showCamera={true} setShowCamera={()=>{}}/></SWRConfig>);
}
it('uses the upstream recorder handlers with a persisted E6 task and distinct outcomes',async()=>{
  const requests:any[]=[];const listener=(event:Event)=>requests.push((event as CustomEvent).detail);
  window.addEventListener('e6-recording-request',listener);
  let view=mount();
  fireEvent.click(screen.getByRole("button",{name:"Recording settings"}));
  fireEvent.change(screen.getByLabelText('Task'),{target:{value:'局部路线 · 采集一'}});
  expect(localStorage.getItem('e6-recording-task')).toBe('局部路线 · 采集一');
  fireEvent.change(screen.getByLabelText('Scene / notes'),{target:{value:'桌面空载'}});
  fireEvent.change(screen.getByLabelText('Operator'),{target:{value:'auto-v169'}});
  view.unmount();view=mount();
  fireEvent.click(screen.getByRole("button",{name:"Recording settings"}));
  expect((screen.getByLabelText('Task') as HTMLInputElement).value).toBe('局部路线 · 采集一');
  expect((screen.getByLabelText('Scene / notes') as HTMLInputElement).value).toBe('桌面空载');
  expect((screen.getByLabelText('Operator') as HTMLInputElement).value).toBe('auto-v169');
  await act(async()=>state('idle'));
  fireEvent.click(screen.getByRole('button',{name:'Record',exact:true}));
  expect(requests.at(-1)).toEqual({op:'record_start',result:undefined,task:'局部路线 · 采集一'});
  await act(async()=>state('capturing'));
  expect((screen.getByLabelText('Task') as HTMLInputElement).disabled).toBe(true);
  expect((screen.getByLabelText('Scene / notes') as HTMLInputElement).disabled).toBe(true);
  expect((screen.getByLabelText('Operator') as HTMLInputElement).disabled).toBe(true);
  expect((screen.getByRole('button',{name:'Record',exact:true}) as HTMLButtonElement).disabled).toBe(true);
  for(const [name,result] of [['Success','success'],['Failure','failure'],['Abort','aborted']]){
    fireEvent.click(screen.getByRole('button',{name,exact:true}));
    expect(requests.at(-1)).toMatchObject({op:'record_finish',result});
  }
  await act(async()=>state('transfer_pending'));
  expect((screen.getByLabelText('Task') as HTMLInputElement).disabled).toBe(true);
  expect(screen.getByRole('status').textContent).toContain('Transferring');
  expect(screen.getByRole('status').textContent).toContain('23');
  await act(async()=>state('complete'));
  expect((screen.getByLabelText('Task') as HTMLInputElement).disabled).toBe(false);
  expect(screen.queryByRole('button',{name:'REPLAY',exact:true})).toBeNull();
  window.removeEventListener('e6-recording-request',listener);
});
it('preserves upstream recorder UI when the E6 recording capability is absent',()=>{
  mount(false);
  expect(screen.getByRole('button',{name:'REC',exact:true})).toBeTruthy();
  expect(screen.getByRole('button',{name:'REPLAY',exact:true})).toBeTruthy();
  expect(screen.queryByLabelText('Task')).toBeNull();
});

it('shows the payload allowance and warns before exhaustion without disabling finish',async()=>{
  mount();
  expect(screen.queryByLabelText('Episode capacity')).toBeNull();
  const update=async(used:number,warning:boolean,phase='capturing')=>act(async()=>{
    window.dispatchEvent(new CustomEvent('e6-recording-state',{detail:{phase,samples:10,
      capacity:{limit_bytes:2*1024**3,used_bytes:used,remaining_bytes:2*1024**3-used,warning,accounting:'native_image_payload'}}}));
  });
  await update(1024**3,false);
  fireEvent.click(screen.getByRole("button",{name:"Recording settings"}));
  expect(screen.getByLabelText('Episode capacity').textContent).toContain('1.00 GiB / 2.00 GiB');
  expect(screen.queryByRole('alert')).toBeNull();
  await update(1.8*1024**3,true);
  expect(screen.getByRole('alert').textContent).toContain('nearly full');
  expect(screen.getByLabelText('Episode capacity').textContent).toContain('Raw images');
  expect((screen.getByRole('button',{name:'Success',exact:true}) as HTMLButtonElement).disabled).toBe(false);
  await update(1.8*1024**3,true,'complete');
  expect(screen.queryByRole('alert')).toBeNull();
});

it('separates transferred bytes from verification and keeps unknown totals indeterminate',()=>{
  const item={episode:'fixture',phase:'transferring',progress:{stage:'receiving',received_bytes:1024**2,total_bytes:4*1024**2}};
  const view=render(<EpisodeTransferProgress item={item}/>);
  expect(screen.getByLabelText('Transfer progress').textContent).toContain('1.0 MiB / 4.0 MiB · 25%');
  expect(screen.getByText('Receiving originals')).toBeTruthy();
  expect(screen.getByText('Background transfer · control available. Next recording after verification.')).toBeTruthy();
  view.rerender(<EpisodeTransferProgress item={{...item,progress:{...item.progress,stage:'verification',received_bytes:4*1024**2}}}/>);
  expect(screen.getByText('Verifying files and associations')).toBeTruthy();
  expect(screen.queryByText('Saved')).toBeNull();
  view.rerender(<EpisodeTransferProgress item={{...item,progress:{...item.progress,total_bytes:null}}}/>);
  expect(screen.queryByRole('progressbar')).toBeNull();
  expect(screen.getByLabelText('Transfer progress').textContent).toContain('1.0 MiB');
  view.rerender(<EpisodeTransferProgress item={{...item,phase:'failed'}}/>);
  expect(screen.getByText('Transfer paused')).toBeTruthy();
  view.rerender(<EpisodeTransferProgress item={{...item,phase:'verified'}}/>);
  expect(screen.queryByLabelText('Transfer progress')).toBeNull();
});

it('shows only the current episode transfer on the formal upstream recorder',async()=>{
  mount(true,[{episode:'current',phase:'transferring',progress:{stage:'receiving',received_bytes:1024**2,total_bytes:2*1024**2}},
    {episode:'other',phase:'failed',progress:{stage:'receiving',received_bytes:0,total_bytes:3*1024**2}}]);
  await act(async()=>window.dispatchEvent(new CustomEvent('e6-recording-state',{detail:{phase:'transfer_pending',episode:'current'}})));
  expect(screen.getByLabelText('Transfer progress').textContent).toContain('1.0 MiB / 2.0 MiB · 50%');
  expect(screen.queryByText('Transfer paused')).toBeNull();
  expect((screen.getByRole('button',{name:'Record',exact:true}) as HTMLButtonElement).disabled).toBe(true);
});
