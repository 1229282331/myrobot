import { describe, expect, it } from "vitest";
import { radial, fromGamepad, fromKeyboard, command } from "./input";
const pad = (axes = [0, 0, 0, 0], values = Array(17).fill(0)): Gamepad =>
  ({
    connected: true,
    id: "Xbox test",
    index: 0,
    mapping: "standard",
    axes,
    buttons: values.map((v) => ({
      value: v,
      pressed: v > 0.5,
      touched: v > 0,
    })),
    timestamp: 12,
  }) as unknown as Gamepad;
describe("input semantics", () => {
  it("maps both devices, all signed axes and triggers using backend capability rates", () => {
    const limits={translation_m_s:.002,rotation_rad_s:Math.PI/180,gripper_s:.05};
    for(const mode of ['translation','rotation'] as const) for(const frame of ['base','tool'] as const) for(let axis=0;axis<4;axis++) for(const sign of [-1,1]){
      const a=[0,0,0,0],b=Array(17).fill(0);b[4]=b[5]=1;
      if(axis<3)a[[0,1,3][axis]]=sign*(axis===0?1:-1);else b[sign>0?6:7]=1;
      const keys=new Set(['Space','ShiftLeft',(sign>0?['KeyD','KeyW','KeyE','KeyF']:['KeyA','KeyS','KeyQ','KeyR'])[axis]]);
      for(const i of [fromGamepad(pad(a,b)),fromKeyboard(keys)]){
        const c=command(i,mode,frame,true,limits);
        expect(c.deadman).toBe(true);expect(c.fine).toBe(true);expect(c.neutral).toBe(false);expect(c.frame).toBe(frame);
        if(axis<3){expect(c.twist[axis+(mode==='rotation'?3:0)]).toBeCloseTo(sign*(mode==='rotation'?limits.rotation_rad_s:limits.translation_m_s));expect(c.gripper_rate).toBe(0)}
        else {expect(c.gripper_rate).toBeCloseTo(sign*.05);expect(c.twist.every(v=>v===0)).toBe(true)}
      }
    }
  });
  it("remaps radial deadzone continuously and caps diagonal magnitude", () => {
    expect(radial(0.02, 0.01)).toEqual([0, 0]);
    expect(radial(0.05, 0)).toEqual([0, 0]);
    expect(radial(0.525, 0)[0]).toBeCloseTo(0.5);
    expect(Math.hypot(...radial(1, 1))).toBeCloseTo(1);
  });
  it("uses the same XYZ channels for rotation and does not exceed speed", () => {
    const input = fromGamepad(pad([1, -1, 0, -1]));
    const t = command(input, "translation", "base", true),
      r = command(input, "rotation", "tool", true);
    expect(Math.hypot(...t.twist.slice(0, 3))).toBeCloseTo(0.01);
    expect(Math.hypot(...r.twist.slice(3))).toBeCloseTo((5 * Math.PI) / 180);
    expect(t.twist.slice(3)).toEqual([0, 0, 0]);
  });
  it("retains full analog trigger information and refuses false neutral", () => {
    const v = Array(17).fill(0);
    v[6] = v[7] = 0.5;
    const input = fromGamepad(pad([0, 0, 0, 0], v));
    expect(input.grip).toBe(0);
    expect(input.neutral).toBe(false);
    v[4] = 1;
    v[7] = 0;
    expect(fromGamepad(pad([0, 0, 0, 0], v)).deadman).toBe(true);
    expect(fromGamepad(pad([0, 0, 0, 0], v)).grip).toBeCloseTo(0.46 / 0.96);
  });
  it("requires standard mapping and releases on disconnection", () => {
    expect(fromGamepad(null).deadman).toBe(false);
    expect(fromGamepad({ ...pad(), mapping: "" } as Gamepad).connected).toBe(
      false,
    );
  });
  it("suppresses keyboard motion during text editing, including opposite held keys", () => {
    const keys = new Set(["Space", "KeyD", "KeyA", "KeyF"]);
    expect(fromKeyboard(keys).neutral).toBe(false);
    expect(fromKeyboard(keys, true).axes).toEqual([0, 0, 0]);
    expect(fromKeyboard(keys, true).deadman).toBe(false);
  });
});
