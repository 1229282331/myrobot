import type { Input } from './input';

/** Preserve physical input evidence while removing an unavailable actuator. */
export function restrictGripper(input: Input, enabled: boolean): Input {
  if (enabled) return input;
  return { ...input, grip: 0, neutral: input.axes.every(axis => axis === 0),
    raw: { ...input.raw, gripper_control_disabled: true } };
}

/** A stop or device transition must finish before streaming a new lease. */
export class BrowserInputGate {
  identity: string | null = null;
  private stopRequest: number | null = null;
  private handshake: {request: number; identity: string} | null = null;
  reset() { this.identity = null; this.stopRequest = null; this.handshake = null; }
  stop(request: number) {
    this.identity = null;
    this.handshake = null;
    this.stopRequest = request;
  }
  acknowledge(request: number, ok: boolean) {
    if (request === this.stopRequest) this.stopRequest = null;
    if (request === this.handshake?.request) {
      this.identity = ok ? this.handshake.identity : null;
      this.handshake = null;
    }
  }
  decision(identity: string, neutral: boolean, deadman: boolean) {
    if (this.stopRequest !== null || this.handshake !== null) return 'wait';
    if (identity === this.identity) return 'stream';
    return neutral && !deadman ? 'handshake' : 'center';
  }
  sentHandshake(request: number, identity: string) { this.handshake = {request, identity}; }
}

/** A source selector is not text editing for a physical gamepad. */
export function blocksControlInput(source: 'keyboard' | 'gamepad', activeTag: string,
                                   contentEditable = false) {
  return contentEditable || ['INPUT', 'TEXTAREA'].includes(activeTag) ||
    source === 'keyboard' && activeTag === 'SELECT';
}
