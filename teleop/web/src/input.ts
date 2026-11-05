// Standard-layout extraction adapted from phosphobot GamepadUtils.ts (MIT).
// Pinned upstream and license are recorded in THIRD_PARTY_NOTICES.md.
export type Mode = "translation" | "rotation";
export type Frame = "base" | "tool";
export type Source = "gamepad" | "keyboard";
export type Input = {
  connected: boolean;
  identity: string;
  neutral: boolean;
  deadman: boolean;
  fine: boolean;
  stop: boolean;
  axes: number[];
  grip: number;
  buttons: boolean[];
  raw: Record<string, unknown>;
};
export const BUTTON_NAMES = [
  "A",
  "B",
  "X",
  "Y",
  "LB",
  "RB",
  "LT",
  "RT",
  "Back",
  "Start",
  "L3",
  "R3",
  "↑",
  "↓",
  "←",
  "→",
  "Guide",
];

export function radial(
  x: number,
  y: number,
  deadzone = 0.05,
): [number, number] {
  const norm = Math.hypot(x, y);
  if (norm <= deadzone) return [0, 0];
  const scale = Math.min(1, (norm - deadzone) / (1 - deadzone)) / norm;
  return [x * scale, y * scale];
}

export function getAvailableGamepads(): Gamepad[] {
  return Array.from(navigator.getGamepads()).filter(
    (pad): pad is Gamepad => !!pad,
  );
}

export function fromGamepad(pad: Gamepad | null): Input {
  if (
    !pad ||
    !pad.connected ||
    pad.mapping !== "standard" ||
    pad.axes.length < 4 ||
    pad.buttons.length < 16
  ) {
    return {
      connected: false,
      identity: pad?.id ?? "No gamepad",
      neutral: true,
      deadman: false,
      fine: false,
      stop: false,
      axes: [0, 0, 0],
      grip: 0,
      buttons: [],
      raw: {},
    };
  }
  const [lx, ly] = radial(pad.axes[0], pad.axes[1]);
  const [rx, ry] = radial(pad.axes[2], pad.axes[3]);
  const lt = pad.buttons[6].value,
    rt = pad.buttons[7].value;
  const trigger = (v: number) => (v <= 0.04 ? 0 : (v - 0.04) / 0.96);
  return {
    connected: true,
    identity: pad.id,
    neutral: [lx, ly, rx, ry].every((v) => v === 0) && lt <= 0.04 && rt <= 0.04,
    deadman: pad.buttons[4].pressed,
    fine: pad.buttons[5].pressed,
    stop: pad.buttons[8].pressed,
    axes: [lx, -ly, -ry],
    grip: trigger(lt) - trigger(rt),
    buttons: pad.buttons.map((b) => b.pressed),
    raw: {
      axes: [...pad.axes],
      buttons: pad.buttons.map((b) => b.value),
      browser_timestamp_ms: pad.timestamp,
      browser_index: pad.index,
    },
  };
}

export function fromKeyboard(keys: Set<string>, editing = false): Input {
  const direction = (positive: string, negative: string) =>
    Number(keys.has(positive)) - Number(keys.has(negative));
  const axes = editing
    ? [0, 0, 0]
    : [
        direction("KeyD", "KeyA"),
        direction("KeyW", "KeyS"),
        direction("KeyE", "KeyQ"),
      ];
  const grip = editing ? 0 : direction("KeyF", "KeyR");
  const motionKeys = [
    "KeyA",
    "KeyD",
    "KeyW",
    "KeyS",
    "KeyQ",
    "KeyE",
    "KeyF",
    "KeyR",
  ];
  return {
    connected: true,
    identity: "Browser keyboard",
    neutral: !motionKeys.some((k) => keys.has(k)),
    deadman: !editing && keys.has("Space"),
    fine: keys.has("ShiftLeft") || keys.has("ShiftRight"),
    stop: keys.has("Escape"),
    axes,
    grip,
    buttons: [],
    raw: { keys: [...keys], editing },
  };
}

export function command(
  input: Input,
  mode: Mode,
  frame: Frame,
  focused: boolean,
  limits = {translation_m_s:0.01,rotation_rad_s:(5*Math.PI)/180,gripper_s:0.1},
) {
  const norm = Math.max(1, Math.hypot(...input.axes));
  const rate = mode === "translation" ? limits.translation_m_s : limits.rotation_rad_s;
  const v = input.axes.map((x) => (x / norm) * rate);
  return {
    operation: "input",
    twist: mode === "translation" ? [...v, 0, 0, 0] : [0, 0, 0, ...v],
    gripper_rate: input.grip * limits.gripper_s,
    connected: input.connected,
    identity: input.identity,
    neutral: input.neutral,
    focused,
    deadman: input.deadman,
    fine: input.fine,
    stop: input.stop,
    mode,
    frame,
    raw: { ...input.raw, browser_sample_monotonic_ms: performance.now() },
  };
}
