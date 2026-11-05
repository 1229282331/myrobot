// Raw standard Gamepad samples for the explicitly enabled commissioning UI.
// This substitutes only the browser device read; mapping and transport stay real.
export function acceptanceGamepad(axes:number[]=[0,0,0,0],values:number[]=[]) : Gamepad {
  if(axes.length!==4 || axes.some(x=>!Number.isFinite(x)||Math.abs(x)>1) ||
     values.length>17 || values.some(x=>!Number.isFinite(x)||x<0||x>1))
    throw new Error('Invalid raw gamepad sample');
  return {id:'E6 synthetic standard gamepad (not physical)',index:-1,connected:true,
    mapping:'standard',timestamp:performance.now(),axes:[...axes],
    buttons:Array.from({length:17},(_,i)=>({value:values[i]||0,pressed:(values[i]||0)>.5,touched:(values[i]||0)>0})),
    vibrationActuator:null} as unknown as Gamepad;
}
