export const MAX_LOG_LINES=5000;
export const MAX_LOG_BYTES=2*1024*1024;
// UTF-8 bytes and physical lines, including a final partial line.
export function boundedLog(text:string):string {
  const bytes=new TextEncoder().encode(text);
  if(bytes.length>MAX_LOG_BYTES){
    let start=bytes.length-MAX_LOG_BYTES;
    while(start<bytes.length&&(bytes[start]&0xc0)===0x80)start++;
    text=new TextDecoder().decode(bytes.subarray(start));
  }
  const lines=text.split('\n');
  if(lines.length>MAX_LOG_LINES)text=lines.slice(-MAX_LOG_LINES).join('\n');
  return text;
}
