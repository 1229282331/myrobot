import React,{useEffect} from 'react';
import {act,cleanup,render,screen} from '@testing-library/react';
import {afterEach,expect,it,vi} from 'vitest';
import {MemoryRouter} from 'react-router-dom';
import {SWRConfig} from 'swr';
import {E6RouteGuard,DeviceConnectionBadge,DeviceDashboard,CameraPreview} from '../frontend/PlatformPages';
import {feedbackFresh,isUnavailableRoute} from '../frontend/WorkbenchChrome';
import {AppSidebar} from '@/components/common/app-sidebar';
import {SidebarProvider} from '@/components/ui/sidebar';
import {EpisodeLibrary} from '../frontend/EpisodeBrowser';
import {isEpisodeBrowserOpen} from '../frontend/episodeBrowserState';
vi.mock('@/components/common/camera-stream-card',()=>({CameraStreamCard:({streamPath}:{streamPath:string})=><div data-testid="stream">{streamPath}</div>}));
vi.mock('@/hooks/use-mobile',()=>({useIsMobile:()=>false}));
afterEach(()=>{cleanup();vi.useRealTimers();vi.unstubAllGlobals();});
const wrapper=(children:React.ReactNode,path='/')=><MemoryRouter initialEntries={[path]}><SWRConfig value={{provider:()=>new Map(),
  revalidateOnMount:false,revalidateOnFocus:false,fallback:{'/e6/capabilities':{enabled:true},'/e6/episodes':{episodes:[]}}}}>{children}</SWRConfig></MemoryRouter>;
it.each(['/calibration','/train','/inference','/admin','/network','/admin/settings','/sign-in'])('guards %s without mounting upstream actions',path=>{
  const mounted=vi.fn();function Upstream(){useEffect(mounted,[]);return <div>upstream action</div>;}
  render(wrapper(<E6RouteGuard><Upstream/></E6RouteGuard>,path));
  expect(screen.getByRole('alert').textContent).toContain('Not adapted for E6');expect(mounted).not.toHaveBeenCalled();
});
it('keeps official navigation clickable with no adaptation badges',()=>{
  render(wrapper(<SidebarProvider><AppSidebar/></SidebarProvider>));
  for(const name of ['Calibration','AI Training','AI Control','Admin Configuration','Network Management'])
    expect(screen.getByRole('link',{name}).hasAttribute('href')).toBe(true);
  expect(screen.queryByText('Not adapted')).toBeNull();
});
it.each([undefined,{feedback:{connected:true,age_ms:101}},{feedback:{connected:false,age_ms:1}},
  {feedback:{connected:true,age_ms:NaN}},{feedback:{connected:true,age_ms:-1}}])('rejects stale or invalid connection: %j',value=>{
  expect(feedbackFresh(value)).toBe(false);
});
it('accepts fresh raw feedback independently of model state',()=>{
  expect(feedbackFresh({feedback:{connected:true,age_ms:5}})).toBe(true);
  expect(isUnavailableRoute('/control')).toBe(false);expect(isUnavailableRoute('/browse')).toBe(false);
});
it('ages an HTTP snapshot locally while a refresh hangs; never opens a control socket',async()=>{
  vi.useFakeTimers();const socket=vi.fn();vi.stubGlobal('WebSocket',socket);
  const fetch=vi.fn().mockResolvedValueOnce({ok:true,json:async()=>({feedback:{connected:true,age_ms:1}})})
    .mockImplementation(()=>new Promise(()=>{}));vi.stubGlobal('fetch',fetch);
  render(<SWRConfig value={{provider:()=>new Map(),revalidateOnFocus:false}}><DeviceConnectionBadge/></SWRConfig>);
  await act(async()=>{await Promise.resolve();});expect(screen.getByLabelText('E6 connection').textContent).toContain('Connected');
  await act(async()=>{await vi.advanceTimersByTimeAsync(1500);});expect(screen.getByLabelText('E6 connection').textContent).toContain('Offline');
  expect(socket).not.toHaveBeenCalled();expect(fetch.mock.calls.every(([url])=>url==='/e6/state')).toBe(true);
});
it('shows empty camera and unknown gripper',()=>{
  render(wrapper(<><DeviceDashboard/><CameraPreview/></>));
  expect(screen.getByText('Unknown')).toBeTruthy();expect(screen.getByText('No camera stream')).toBeTruthy();
});
it('recovers discovery after a backend restart without remounting or opening a control socket',async()=>{
  vi.useFakeTimers();const socket=vi.fn();vi.stubGlobal('WebSocket',socket);
  let online=false;
  const fetch=vi.fn(async()=>({ok:online,json:async()=>({feedback:{connected:true,age_ms:3}})}));
  vi.stubGlobal('fetch',fetch);
  render(<SWRConfig value={{provider:()=>new Map(),revalidateOnFocus:false}}><DeviceConnectionBadge/></SWRConfig>);
  await act(async()=>{await Promise.resolve();});
  expect(screen.getByLabelText('E6 connection').textContent).toContain('Offline');
  online=true;
  await act(async()=>{await vi.advanceTimersByTimeAsync(1500);});
  expect(screen.getByLabelText('E6 connection').textContent).toContain('Connected');
  expect(socket).not.toHaveBeenCalled();
});

it('recovers a hung observation request after a bounded timeout',async()=>{
  vi.useFakeTimers();let pending:AbortSignal|undefined;const socket=vi.fn();vi.stubGlobal('WebSocket',socket);
  const fetcher=vi.fn().mockImplementationOnce((_url,options)=>new Promise((_resolve,reject)=>{
    pending=options?.signal;pending?.addEventListener('abort',()=>reject(new DOMException('aborted','AbortError')));
  })).mockResolvedValue({ok:true,json:async()=>({feedback:{connected:true,age_ms:2}})});
  vi.stubGlobal('fetch',fetcher);
  render(<SWRConfig value={{provider:()=>new Map(),revalidateOnFocus:false}}><DeviceConnectionBadge/></SWRConfig>);
  await act(async()=>vi.advanceTimersByTimeAsync(1600));expect(pending?.aborted).toBe(true);
  await act(async()=>vi.advanceTimersByTimeAsync(1500));
  expect(screen.getByLabelText('E6 connection').textContent).toContain('Connected');expect(socket).not.toHaveBeenCalled();
});
it('reuses the existing WebRTC component',()=>{
  render(<CameraPreview url="http://127.0.0.1:8889/gemini"/>);
  expect(screen.getByTestId('stream').textContent).toBe('webrtc:http://127.0.0.1:8889/gemini');
});
it('browse route announces pause for its lifetime and retains empty state',()=>{
  const view=render(wrapper(<EpisodeLibrary/>));expect(isEpisodeBrowserOpen()).toBe(true);
  expect(screen.getByText('No recordings found.')).toBeTruthy();view.unmount();expect(isEpisodeBrowserOpen()).toBe(false);
});
