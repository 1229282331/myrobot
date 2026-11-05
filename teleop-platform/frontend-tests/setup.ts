// jsdom lacks layout observers and pointer capture used by the upstream Radix controls.
class ResizeObserverFixture {observe(){} unobserve(){} disconnect(){}}
globalThis.ResizeObserver=ResizeObserverFixture as typeof ResizeObserver;
HTMLElement.prototype.scrollIntoView=()=>{};
HTMLElement.prototype.hasPointerCapture=()=>false;
HTMLElement.prototype.setPointerCapture=()=>{};
HTMLElement.prototype.releasePointerCapture=()=>{};
