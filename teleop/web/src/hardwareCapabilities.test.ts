import { describe, expect, it } from 'vitest';
import { fromKeyboard, command } from './input';
import { BrowserInputGate, blocksControlInput, restrictGripper } from './hardwareCapabilities';

describe('arm-only capabilities', () => {
  it('preserves arm input and raw keys while suppressing jaw commands', () => {
    const original=fromKeyboard(new Set(['Space','KeyW','KeyF']));
    const restricted=restrictGripper(original,false);
    expect(restricted.axes).toEqual(original.axes);
    expect(restricted.deadman).toBe(true);
    expect(restricted.grip).toBe(0);
    expect(restricted.raw).toMatchObject({...original.raw,gripper_control_disabled:true});
    expect(command(restricted,'translation','base',true).gripper_rate).toBe(0);
    expect(original.grip).not.toBe(0);
  });
  it('unused jaw keys do not prevent neutral preparation', () => {
    expect(restrictGripper(fromKeyboard(new Set(['KeyF'])),false).neutral).toBe(true);
  });
  it('leaves full capability input unchanged', () => {
    const original=fromKeyboard(new Set(['Space','KeyR']));
    expect(restrictGripper(original,true)).toBe(original);
  });
});

describe('browser control authority', () => {
  it('waits for Stop and neutral-device acknowledgements before a new active input', () => {
    const gate=new BrowserInputGate();
    gate.sentHandshake(1,'keyboard');gate.acknowledge(1,true);
    expect(gate.decision('keyboard',false,true)).toBe('stream');
    gate.stop(3);
    expect(gate.decision('gamepad',true,false)).toBe('wait');
    gate.acknowledge(2,true); // Late old input cannot release the Stop barrier.
    expect(gate.decision('gamepad',true,false)).toBe('wait');
    gate.acknowledge(3,true);
    expect(gate.decision('gamepad',false,true)).toBe('center');
    expect(gate.decision('gamepad',true,false)).toBe('handshake');
    gate.sentHandshake(4,'gamepad');
    expect(gate.decision('gamepad',false,true)).toBe('wait');
    gate.acknowledge(4,true);
    expect(gate.decision('gamepad',false,true)).toBe('stream');
  });
  it('does not regain authority from a late handshake after blur/Stop', () => {
    const gate=new BrowserInputGate();gate.sentHandshake(1,'pad');gate.stop(2);
    gate.acknowledge(1,true);gate.acknowledge(2,true);
    expect(gate.decision('pad',false,true)).toBe('center');
  });
  it('requires neutral again after reconnect or rejected handshake', () => {
    const gate=new BrowserInputGate();gate.sentHandshake(1,'pad');gate.acknowledge(1,false);
    expect(gate.decision('pad',false,true)).toBe('center');
    gate.sentHandshake(2,'pad');gate.acknowledge(2,true);gate.reset();
    expect(gate.decision('pad',false,true)).toBe('center');
  });
  it('allows gamepad with a selected dropdown while blocking text edits and keyboard dropdown shortcuts', () => {
    expect(blocksControlInput('gamepad','SELECT')).toBe(false);
    expect(blocksControlInput('keyboard','SELECT')).toBe(true);
    expect(blocksControlInput('gamepad','TEXTAREA')).toBe(true);
    expect(blocksControlInput('keyboard','INPUT')).toBe(true);
    expect(blocksControlInput('gamepad','DIV',true)).toBe(true);
  });
});
