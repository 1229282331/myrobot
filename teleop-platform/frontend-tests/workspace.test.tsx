import {act,cleanup,fireEvent,render,screen,waitFor} from '@testing-library/react';
import {afterEach,beforeEach,describe,expect,it,vi} from 'vitest';
const scene=vi.hoisted(()=>({setJoints:vi.fn(),setGripper:vi.fn(),layers:vi.fn(),reset:vi.fn(),dispose:vi.fn()}));
vi.mock('../frontend/workspaceRenderer',()=>({createWorkspaceScene:vi.fn(async()=>scene)}));
import {WorkspaceView} from '../frontend/WorkspaceView';

beforeEach(()=>{Object.values(scene).forEach(f=>f.mockClear());vi.stubGlobal('fetch',vi.fn(async()=>({ok:true,json:async()=>({model_id:'test',robot_url:'/e6/workspace/assets/robot.urdf',map_url:'/e6/workspace/map',map_available:true})})));});
afterEach(()=>{cleanup();vi.unstubAllGlobals();vi.useRealTimers();});
describe('read-only workspace view',()=>{
  it('can retry a failed model fetch without reconnecting control',async()=>{
    const socket=vi.fn();vi.stubGlobal('WebSocket',socket);
    const fetcher=vi.mocked(fetch);fetcher.mockRejectedValueOnce(new Error('Temporary offline'));
    render(<WorkspaceView joints={[1,2,3,4,5,6]} fresh mode="model" assemblyId="test"/>);
    await screen.findByText(/Temporary offline/);
    fireEvent.click(screen.getByRole('button',{name:'Reload workspace'}));
    await waitFor(()=>expect(scene.setJoints).toHaveBeenCalledWith([1,2,3,4,5,6]));
    expect(screen.queryByText(/Temporary offline/)).toBeNull();expect(socket).not.toHaveBeenCalled();
  });
  it('reloads metadata when the connected assembly changes and aborts the previous fetch',async()=>{
    const fetcher=vi.mocked(fetch);
    const r=render(<WorkspaceView fresh={false} mode="model" assemblyId="test"/>);await act(async()=>{});
    const oldSignal=(fetcher.mock.calls[0][1] as RequestInit).signal;
    fetcher.mockResolvedValueOnce({ok:true,json:async()=>({model_id:'new',robot_url:'/new/robot.urdf',map_url:'/new/map'})} as Response);
    r.rerender(<WorkspaceView fresh={false} mode="model" assemblyId="new"/>);await act(async()=>{});
    expect(oldSignal?.aborted).toBe(true);expect(fetcher).toHaveBeenCalledTimes(2);
    expect(screen.queryByText('Model mismatch')).toBeNull();expect(scene.dispose).toHaveBeenCalledTimes(1);
  });
  it('ages gripper telemetry independently and resumes only on a new CAN receipt',async()=>{
    vi.useFakeTimers();vi.spyOn(performance,'now').mockImplementation(()=>Date.now());
    const grip={valid:true,fallback:false,received_ns:10,joints_rad:{op_hand_narrow1_joint:.2}};
    const r=render(<WorkspaceView joints={[0,0,0,0,0,0]} fresh mode="model" assemblyId="test" observation={1} gripper={grip}/>);
    await act(async()=>{});expect(scene.setGripper).toHaveBeenLastCalledWith(grip);
    await act(async()=>vi.advanceTimersByTime(300));
    expect(scene.setGripper).toHaveBeenLastCalledWith({valid:false,fallback:true,joints_rad:null});
    expect(screen.getByText('Live feedback')).toBeTruthy();
    r.rerender(<WorkspaceView joints={[0,0,0,0,0,0]} fresh mode="model" assemblyId="test" observation={2} gripper={{...grip}}/>);
    expect(screen.getByText('Unknown · Full stroke')).toBeTruthy();
    const newGrip={...grip,received_ns:11};
    r.rerender(<WorkspaceView joints={[0,0,0,0,0,0]} fresh mode="model" assemblyId="test" observation={3} gripper={newGrip}/>);
    expect(scene.setGripper).toHaveBeenLastCalledWith(newGrip);
    expect(screen.getByText('Moving fingers')).toBeTruthy();
  });
  it('uses fresh feedback, freezes stale joints and disposes on unmount',async()=>{
    const q=[1,2,3,4,5,6];const r=render(<WorkspaceView joints={q} fresh mode="model" assemblyId="test"/>);
    await waitFor(()=>expect(scene.setJoints).toHaveBeenCalledWith(q));scene.setJoints.mockClear();
    r.rerender(<WorkspaceView joints={[0,0,0,0,0,0]} fresh={false} mode="model" assemblyId="test"/>);
    expect(scene.setJoints).not.toHaveBeenCalled();expect(screen.getByText('Feedback unavailable')).toBeTruthy();
    r.unmount();expect(scene.dispose).toHaveBeenCalledTimes(1);
  });
  it('freezes feedback when controller and display model identities differ',async()=>{
    render(<WorkspaceView joints={[1,2,3,4,5,6]} fresh mode="model" assemblyId="other"/>);
    await act(async()=>{});
    expect(screen.getByText('Model mismatch')).toBeTruthy();
    expect(scene.setJoints).not.toHaveBeenCalled();
  });
  it('marks a stalled state stream stale using only the browser receipt clock',async()=>{
    vi.useFakeTimers();vi.spyOn(performance,'now').mockImplementation(()=>Date.now());
    render(<WorkspaceView joints={[1,2,3,4,5,6]} fresh mode="model" assemblyId="test" observation={1}/>);
    await act(async()=>{});
    expect(screen.getByText('Live feedback')).toBeTruthy();
    await act(async()=>vi.advanceTimersByTime(650));
    expect(screen.getByText('Feedback unavailable')).toBeTruthy();
  });
  it('pauses and resumes rendering without opening a control connection',async()=>{
    const socket=vi.fn();vi.stubGlobal('WebSocket',socket);
    render(<WorkspaceView fresh={false} mode="local"/>);await act(async()=>{});
    fireEvent.click(screen.getByRole('button',{name:'Pause 3D view'}));
    expect(scene.dispose).toHaveBeenCalledTimes(1);expect(screen.getByText('3D paused')).toBeTruthy();
    fireEvent.click(screen.getByRole('button',{name:'Resume 3D view'}));await act(async()=>{});
    expect(socket).not.toHaveBeenCalled();
  });
  it('layers, expansion and camera reset remain display-only',async()=>{
    const ws=vi.fn();vi.stubGlobal('WebSocket',ws);
    render(<WorkspaceView fresh={false} mode="local"/>);
    await waitFor(()=>expect(fetch).toHaveBeenCalledWith('/e6/workspace',expect.anything()));await act(async()=>{});
    fireEvent.click(screen.getByRole('button',{name:'Collision bounds'}));expect(scene.layers).toHaveBeenCalledWith(true,true);
    fireEvent.click(screen.getByRole('button',{name:'Reset workspace view'}));expect(scene.reset).toHaveBeenCalled();
    fireEvent.click(screen.getByRole('button',{name:'Expand workspace'}));expect(screen.getByRole('button',{name:'Collapse workspace'})).toBeTruthy();
    expect(ws).not.toHaveBeenCalled();expect(screen.getByTestId('workspace-view').hasAttribute('data-e6-input-isolation')).toBe(true);
  });
  it('keeps a fixed empty area and a readable error when model loading fails',async()=>{
    vi.stubGlobal('fetch',vi.fn(async()=>({ok:false})));render(<WorkspaceView fresh={false} mode="model"/>);
    await screen.findByText(/Error: Workspace model unavailable/);
    expect(screen.getByLabelText('Robot workspace model').className).toContain('h-[320px]');
    expect(screen.getByText(/Environment not modeled/)).toBeTruthy();
  });
});
