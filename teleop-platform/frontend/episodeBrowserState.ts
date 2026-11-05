// The recorder can mount before capabilities connect the input hook. Retain the
// modal state so that a late connection also starts with control suspended.
let open = false;
export const isEpisodeBrowserOpen = () => open;
export function announceEpisodeBrowser(value:boolean) {
  open = value;
  window.dispatchEvent(new CustomEvent('e6-episode-browser',{detail:{open:value}}));
}
