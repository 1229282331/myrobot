/** Keep upstream pair/group configuration, clearing only vanished input slots. */
export function clearMissingGamepads<T extends {controller_index:number|null;controller_name:string}>(
  assignments:T[], available:ReadonlyArray<{index:number}>,
):T[] {
  const connected=new Set(available.map(pad=>pad.index));
  if(!assignments.some(item=>item.controller_index!==null&&!connected.has(item.controller_index)))return assignments;
  return assignments.map(item=>item.controller_index!==null&&!connected.has(item.controller_index)
    ?{...item,controller_index:null,controller_name:''}:item);
}
