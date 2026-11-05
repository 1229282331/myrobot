// Only the explicitly enabled synthetic-input fixture uses this panel.
// Keep the real video visible: Chromium can suspend presentation callbacks
// for an off-screen video, which must still trigger the existing health stop.
export function revealAcceptanceViews(event: {currentTarget: HTMLDetailsElement}) {
  if (event.currentTarget.open) {
    document.querySelector<HTMLElement>('.e6-observation-row')?.scrollIntoView?.({block:'start',behavior:'instant'});
  }
}
