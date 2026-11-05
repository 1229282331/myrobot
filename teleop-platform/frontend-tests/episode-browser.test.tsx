import React from 'react';
import {cleanup,fireEvent,render,screen,act,waitFor,within} from '@testing-library/react';
import {afterEach,expect,it,vi} from 'vitest';
import {SWRConfig,useSWRConfig} from 'swr';
import {EpisodeBrowser} from '../frontend/EpisodeBrowser';
import {isEpisodeBrowserOpen} from '../frontend/episodeBrowserState';
import {EpisodeInspector} from '../frontend/EpisodeInspector';

it('loads samples when an already-selected transfer becomes verified, never labels pending data empty',async()=>{
  const item={episode:'restored',phase:'transferring'};
  const fetch=vi.fn(async(url:string)=>({ok:true,json:async()=>url==='/e6/episodes'?{episodes:[item]}:
    {total:1,cycles:[{at_ns:10,state:{}}],media:[],events:[],associations:[]}}));
  vi.stubGlobal('fetch',fetch);
  let complete:()=>Promise<unknown>=async()=>{};
  function Complete(){const {mutate}=useSWRConfig();complete=()=>mutate('/e6/episodes',{episodes:[{...item,phase:'verified'}]},false);return null;}
  render(<SWRConfig value={{provider:()=>new Map(),revalidateOnFocus:false,
    fallback:{'/e6/episodes':{episodes:[item]}}}}><Complete/><EpisodeBrowser state={{phase:'idle'}} compact recoveryOnly/></SWRConfig>);
  fireEvent.click(screen.getByRole('button',{name:/Browse recordings/}));
  fireEvent.click(screen.getByRole('button',{name:'restored',exact:true}));
  expect(screen.getByText(/Waiting for file verification/)).toBeTruthy();
  expect(screen.queryByText(/No cycle samples/)).toBeNull();
  await waitFor(()=>expect(fetch).toHaveBeenCalledWith('/e6/episodes'));
  expect(fetch.mock.calls.some(([url])=>url.includes('/samples'))).toBe(false);
  await act(async()=>{await complete();});
  await waitFor(()=>expect(fetch).toHaveBeenCalledWith('/e6/episodes/restored/samples?offset=0&limit=1'));
  await waitFor(()=>expect(screen.getByText('Sample 1 / 1')).toBeTruthy());
});

it('filters by optional annotations, result and origin without changing the retained outcomes',()=>{
  const episodes=[{episode:'real-failed',phase:'verified',summary:{result:'failure',samples:2},metadata:{task:'failed task',dataset_kind:'hardware_demonstration',research_context:{scene:'桌面',operator:'pilot'}}},
    {episode:'shadow-success',phase:'verified',summary:{result:'success',samples:1},metadata:{task:'shadow task',dataset_kind:'hardware_shadow'}},
    {episode:'abort-retained',phase:'verified',summary:{result:'aborted',samples:1},metadata:{task:'abort task',dataset_kind:'hardware_demonstration'}}];
  render(<SWRConfig value={{provider:()=>new Map(),revalidateOnMount:false,revalidateOnFocus:false,fallback:{'/e6/episodes':{episodes}}}}>
    <EpisodeBrowser state={{phase:'idle'}} compact/></SWRConfig>);
  fireEvent.click(screen.getByRole('button',{name:'Browse recordings'}));
  fireEvent.change(screen.getByLabelText('Search recordings'),{target:{value:'pilot'}});
  expect(screen.getByRole('button',{name:'real-failed',exact:true}).closest('tr')!.textContent).toContain('Failure');
  expect(screen.queryByRole('button',{name:'shadow-success',exact:true})).toBeNull();
  fireEvent.change(screen.getByLabelText('Search recordings'),{target:{value:''}});
  fireEvent.click(screen.getByRole('combobox',{name:'Episode result'}));
  fireEvent.click(screen.getByRole('option',{name:'Aborted',exact:true}));
  expect(screen.getByRole('button',{name:'abort-retained',exact:true})).toBeTruthy();
  fireEvent.click(screen.getByRole('combobox',{name:'Episode result'}));
  fireEvent.click(screen.getByRole('option',{name:'All results',exact:true}));
  fireEvent.click(screen.getByRole('combobox',{name:'Episode source'}));
  fireEvent.click(screen.getByRole('option',{name:'Hardware shadow',exact:true}));
  expect(screen.getByRole('button',{name:'shadow-success',exact:true})).toBeTruthy();
  expect(screen.queryByRole('button',{name:'real-failed',exact:true})).toBeNull();
});
it('keeps candidate, accepted, actual, uncertainty and stale image association visibly distinct',()=>{
  const page={total:1,cycles:[{at_ns:200000000,state:{actual:{joints_rad:[1,2,3,4,5,6],gripper:null},candidate:{position_m:[.1,.2,.3]},pending:{sent_ns:199000000}}}],
    media:[{sources:{rgb:{device_frame_id:7,x5_receive_monotonic_ns:1,fresh:false,reused:true}}}],
    events:[{event:'motion_uncertain',at_ns:201000000,receipt:{sent_ns:199000000}}],
    associations:[{action_sent_ns:199000000,valid:false,feedback_valid:true,media:{rgb:{frame_id:7,valid:false,reused:true,age_ms:198}}}]};
  render(<EpisodeInspector page={page} episode="fixture"/>);
  const accepted=screen.getByRole('heading',{name:'Accepted target'}).closest('[data-slot=card]')!;
  expect(accepted.textContent).toContain('No accepted target recorded');
  expect(accepted.textContent).not.toContain('57.30'); // actual never fills an accepted target
  expect(screen.getByText(/Pending command：Unconfirmed/)).toBeTruthy();
  const actual=screen.getByRole('heading',{name:'Actual feedback'}).closest('[data-slot=card]')!;
  expect(within(actual).getByText('Unknown')).toBeTruthy();
  expect(screen.getByText(/motion_uncertain/,{selector:'li'}).textContent).toContain('Unconfirmed');
  expect(screen.getByText(/RGB #7/).textContent).toContain('Capture freshness Invalid');
  expect(screen.getByText('Stale: over 100 ms')).toBeTruthy();
  expect(screen.getByRole('table',{hidden:true}).textContent).toContain('198.00 ms · Reused');
});
it.each([
  [{acknowledged:false,accepted:false},'Unconfirmed'],
  [{acknowledged:true,accepted:false},'Rejected'],
  [{acknowledged:true,accepted:true},'Accepted'],
  [{acknowledged:true,accepted:true,uncertain:true},'Unconfirmed'],
])('distinguishes initial accepted=false from an actual rejection: %j', (receipt,label)=>{
  render(<EpisodeInspector episode="fixture" page={{total:1,cycles:[{at_ns:10,state:{}}],media:[],associations:[],
    events:[{event:'motion_submitted',at_ns:11,receipt}]}}/>);
  expect(screen.getByText(/motion_submitted/,{selector:'li'}).textContent).toContain(label);
});

it('labels derived SDK cycle selection, preserves raw sampling evidence and handles unknown frames',()=>{
  const page={total:1,cycles:[{at_ns:1_000_000_000,state:{}}],associations:[],
    media:[{selection_basis:'sdk_available_v2',recorded_sample:{sources:{rgb:{device_frame_id:1,fresh:false}}},
      sources:{rgb:{device_frame_id:2,x5_receive_monotonic_ns:970_000_000,sdk_available_ns:999_000_000,
        cycle_valid:true,cycle_reused:true},depth:{device_frame_id:null,cycle_valid:false,cycle_reused:false}}}]};
  const view=render(<EpisodeInspector page={page} episode="fixture"/>);
  expect(screen.getByRole('img').getAttribute('src')).toBe('/e6/episodes/fixture/rgb/2');
  expect(screen.getByText(/RGB #2/).textContent).toContain('Cycle association Valid · Reused across cycles Yes');
  expect(screen.getByText(/RGB #2/).textContent).toContain('SDK available before cycle 1.00 ms');
  expect(screen.getByText(/Selected from frames/).textContent).toContain('Original associations are retained');
  expect(screen.queryByText(/Capture freshness Valid/)).toBeNull();
  expect(screen.getByText(/"recorded_sample"/,{selector:'pre'})).toBeTruthy();
  view.rerender(<EpisodeInspector page={{...page,media:[{selection_basis:'sdk_available_v2',sources:{rgb:{device_frame_id:null}}}]}} episode="fixture"/>);
  expect(screen.queryByRole('img')).toBeNull(); // do not request /rgb/null
});

afterEach(()=>{cleanup();vi.unstubAllGlobals();window.history.replaceState({},'','/');});
it('reports a failed sample read without presenting the prior image or action association as the new sample',async()=>{
  vi.stubGlobal('fetch',vi.fn(()=>Promise.reject(new Error('Injected read failure'))));
  render(<SWRConfig value={{provider:()=>new Map(),revalidateOnMount:false,revalidateOnFocus:false,shouldRetryOnError:false,
    fallback:{'/e6/episodes':{episodes:[{episode:'fixture',phase:'verified'}]},
      '/e6/episodes/fixture/samples?offset=0&limit=1':{total:2,cycles:[{at_ns:10,state:{}}],
        media:[{sources:{rgb:{device_frame_id:7}}}],associations:[{fixture:'old sample association'}]}}}}>
    <EpisodeBrowser state={{phase:'idle'}} compact/></SWRConfig>);
  fireEvent.click(screen.getByRole('button',{name:'Browse recordings'}));
  fireEvent.click(screen.getByRole('button',{name:'fixture',exact:true}));
  expect(screen.getByRole('img').getAttribute('src')).toContain('/rgb/7');
  fireEvent.click(screen.getByRole('button',{name:'Next'}));
  await waitFor(()=>expect(screen.getByRole('alert').textContent).toContain('Could not read this sample'));
  expect(screen.queryByRole('img')).toBeNull();
  expect(screen.getByRole('dialog').textContent).not.toContain('old sample association');
});
it.each(['link','url'])('opens through a %s and retains the pause state for a late input connection',async mode=>{
  vi.stubGlobal('fetch',vi.fn(()=>Promise.reject(new Error('No network in fixture'))));
  if(mode==='url')window.history.replaceState({},'','/?panel=episodes');
  const view=render(<SWRConfig value={{provider:()=>new Map(),revalidateOnMount:false,
    fallback:{'/e6/episodes':{episodes:[]}}}}><EpisodeBrowser state={{phase:'idle'}} compact/></SWRConfig>);
  if(mode==='link')await act(async()=>window.dispatchEvent(new Event('e6-open-episodes')));
  expect(screen.getByRole('dialog')).toBeTruthy();
  expect(isEpisodeBrowserOpen()).toBe(true);
  await act(async()=>fireEvent.click(screen.getByRole('button',{name:'Close'})));
  expect(isEpisodeBrowserOpen()).toBe(false);
  await act(async()=>window.dispatchEvent(new Event('e6-open-episodes')));
  view.unmount();
  expect(isEpisodeBrowserOpen()).toBe(false);
});
it('uses the upstream dialog without expanding live layout, and announces open, close and unmount',async()=>{
  vi.stubGlobal('fetch',vi.fn(()=>Promise.reject(new Error('No network in fixture'))));
  const events:boolean[]=[];
  const listener=(e:Event)=>events.push((e as CustomEvent).detail.open);
  window.addEventListener('e6-episode-browser',listener);
  const episodes=Array.from({length:23},(_,i)=>({episode:`episode-${i}`,phase:'complete'}));
  const view=render(<SWRConfig value={{provider:()=>new Map(),revalidateOnMount:false,revalidateOnFocus:false,
    fallback:{'/e6/episodes':{episodes}}}}><EpisodeBrowser state={{phase:'idle'}}/></SWRConfig>);
  await act(async()=>fireEvent.click(screen.getByRole('button',{name:'Browse recordings'})));
  const dialog=screen.getByRole('dialog');
  expect(dialog.textContent).toContain('Control paused');
  expect(dialog.className).toContain('fixed');
  expect(dialog.className).toContain('overflow-y-auto');
  expect(view.container.contains(dialog)).toBe(false); // Radix portal, no inserted page rows.
  expect(events).toEqual([true]);
  await act(async()=>fireEvent.click(screen.getByRole('button',{name:'Close'})));
  expect(events).toEqual([true,false]);
  await act(async()=>fireEvent.click(screen.getByRole('button',{name:'Browse recordings'})));
  view.unmount();
  expect(events).toEqual([true,false,true,false]);
  window.removeEventListener('e6-episode-browser',listener);
});
