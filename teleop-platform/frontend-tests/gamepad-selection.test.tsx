import {useState} from 'react';
import {act,cleanup,renderHook} from '@testing-library/react';
import {afterEach,beforeEach,expect,it,vi} from 'vitest';
import {useGamepadDetection} from '@/pages/control/GamepadComponents';
import type {ControllerArmPair,MultiArmGroup} from '@/pages/control/GamepadUtils';

let pads:(Gamepad|null)[]=[];
const pad=(index:number,id='Wireless Controller')=>({index,id,connected:true} as Gamepad);
function boot(index:number|null=0,chosen=true) {
  return renderHook(()=>{
    const [pairs,setPairs]=useState<ControllerArmPair[]>([{controller_index:index,
      controller_name:index===null?'':'Wireless Controller',robot_name:'E6'}]);
    const [groups,setGroups]=useState<MultiArmGroup[]>([]);
    const [selected,setSelected]=useState(chosen);
    const detection=useGamepadDetection('individual',pairs,groups,selected,setPairs,setGroups,setSelected);
    return {...detection,pairs};
  });
}
beforeEach(()=>{vi.useFakeTimers();pads=[];Object.defineProperty(navigator,'getGamepads',{configurable:true,value:()=>pads});});
afterEach(()=>{cleanup();vi.useRealTimers();delete (navigator as any).getGamepads;});

it('reassigns a stale slot even when the receiver reconnects between polls',()=>{
  pads=[null,pad(1)];const {result}=boot(0);
  expect(result.current.pairs[0]).toMatchObject({controller_index:1,robot_name:'E6'});
});
it('removes the disconnected selection while another controller remains present',()=>{
  pads=[pad(0),pad(1)];const {result}=boot(0);
  pads=[null,pad(1)];act(()=>vi.advanceTimersByTime(1000));
  expect(result.current.pairs[0].controller_index).toBe(1);
});
it('preserves a valid explicit selection when another controller appears',()=>{
  pads=[pad(0),pad(1)];const {result}=boot(1);
  act(()=>vi.advanceTimersByTime(2000));expect(result.current.pairs[0].controller_index).toBe(1);
});
it('clears a complete disconnect then selects the actual sparse reconnect slot',()=>{
  pads=[pad(0)];const {result}=boot(0);
  pads=[];act(()=>vi.advanceTimersByTime(1000));expect(result.current.pairs[0].controller_index).toBeNull();
  pads=[null,null,null,pad(3)];act(()=>vi.advanceTimersByTime(1000));
  expect(result.current.pairs[0].controller_index).toBe(3);
});
