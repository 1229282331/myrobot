import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import { describe, expect, it } from "vitest";
import { fromGamepad, command } from "./input";

type Native = {
  buttons: number;
  left_x: number;
  left_y: number;
  right_x: number;
  right_y: number;
  left_trigger: number;
  right_trigger: number;
  return_code: number;
};
const mask = [
  0x1000, 0x2000, 0x4000, 0x8000, 0x100, 0x200, 0, 0, 0x20, 0x10, 0x40, 0x80, 1,
  2, 4, 8, 0,
];
const norm = (x: number) => x / (x < 0 ? 32768 : 32767);
// XInput -> documented standard-browser layout; exercises the archived real inputs.
// This regression is distinct from physically testing every button in the browser.
const browserFixture = (s: Native): Gamepad =>
  ({
    connected: true,
    id: "Archived Flydigi XInput",
    index: 0,
    mapping: "standard",
    timestamp: 0,
    axes: [norm(s.left_x), -norm(s.left_y), norm(s.right_x), -norm(s.right_y)],
    buttons: mask.map((m, i) => {
      const v =
        i === 6
          ? s.left_trigger / 255
          : i === 7
            ? s.right_trigger / 255
            : Number(!!(s.buttons & m));
      return { value: v, pressed: v > 0.5, touched: v > 0 };
    }),
  }) as unknown as Gamepad;

describe("archived hardware inputs", () => {
  it("covers full analog ranges, direction and all requested buttons", () => {
    const files = [
      "gamepad_dynamic_probe_20261004.json",
      "gamepad_shoulders_probe_20261004.json",
    ];
    const samples: Native[] = files.flatMap((name) => {
      const archive = JSON.parse(
        readFileSync(
          resolve("../../research/teleop_survey_20261004", name),
          "utf8",
        ),
      );
      return (
        archive.changed_state_events ??
        archive.button_events.map((s: Partial<Native>) => ({
          left_x: 0,
          left_y: 0,
          right_x: 0,
          right_y: 0,
          return_code: 0,
          ...s,
        }))
      );
    });
    const seen = new Set<number>();
    let opened = 0,
      closed = 0;
    const directions = [
      new Set<number>(),
      new Set<number>(),
      new Set<number>(),
    ];
    for (const sample of samples) {
      if (sample.return_code !== 0) continue;
      const i = fromGamepad(browserFixture(sample));
      i.buttons.forEach((b, n) => {
        if (b) seen.add(n);
      });
      i.axes.forEach((v, n) => {
        if (Math.abs(v) > 0.95) directions[n].add(Math.sign(v));
      });
      if (i.grip > 0.9) opened++;
      if (i.grip < -0.9) closed++;
      expect(i.deadman).toBe(!!(sample.buttons & 0x100));
      expect(i.fine).toBe(!!(sample.buttons & 0x200));
      expect(i.stop).toBe(!!(sample.buttons & 0x20));
      expect(
        Math.hypot(
          ...command(i, "translation", "base", true).twist.slice(0, 3),
        ),
      ).toBeLessThanOrEqual(0.0100000001);
    }
    for (const index of [0, 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13, 14, 15])
      expect(seen.has(index)).toBe(true);
    directions.forEach((signs) => expect([...signs].sort()).toEqual([-1, 1]));
    expect(opened).toBeGreaterThan(0);
    expect(closed).toBeGreaterThan(0);
  });
});
