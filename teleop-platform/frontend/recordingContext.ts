export type ResearchContext={scene?:string;operator?:string};
export const contextStorageKey='e6-recording-context';
export function readRecordingContext():ResearchContext {
  try {
    const value=JSON.parse(localStorage.getItem(contextStorageKey)||'{}');
    return {scene:typeof value?.scene==='string'?value.scene.slice(0,500):'',
      operator:typeof value?.operator==='string'?value.operator.slice(0,80):''};
  } catch {return {};}
}
