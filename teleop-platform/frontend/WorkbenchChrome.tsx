import useSWR from 'swr';
import {useEffect,useState} from 'react';

// Presentation capability only; this never implies a connected robot.
export function useWorkbenchMode() {
  const {data} = useSWR('/e6/capabilities', async (url:string) => {
    const response = await fetch(url);
    if (response.status===404) return {enabled:false};
    if (!response.ok) throw new Error('Capabilities unavailable');
    return response.json();
  });
  return data===undefined?undefined:!!(data.enabled||data.recordings_only);
}
export const unavailableRoutes = ['/calibration','/train','/training','/inference','/admin','/network','/auth','/sign-in','/sign-up'];
export const isUnavailableRoute = (path:string) => unavailableRoutes.some(route=>path===route||path.startsWith(route+'/'));
export type Observation = {stage?:string;state?:string;feedback?:{connected:boolean;age_ms?:number};
  camera?:{valid:boolean;frame_id?:number;age_ms?:number};arm?:{enabled:boolean|null};gripper?:{valid:boolean;opening?:number|null;stroke?:{revision:string}};
  actual?:{joints_rad:number[];position_m:number[];gripper:number|null};capabilities?:{video_url?:string};};
export function feedbackFresh(value?:Observation) {
  const age=value?.feedback?.age_ms;
  return value?.feedback?.connected===true && typeof age==='number' && Number.isFinite(age) && age>=0 && age<=100;
}
export function useDeviceObservation<T extends Observation=Observation>() {
  const [now,setNow]=useState(()=>performance.now());
  useEffect(()=>{const timer=setInterval(()=>setNow(performance.now()),250);return()=>clearInterval(timer);},[]);
  const {data,error,isLoading,isValidating,mutate}=useSWR<{observation:T;receivedAt:number}>('/e6/state',async url=>{
    const abort=new AbortController();const timer=setTimeout(()=>abort.abort(),1500);
    try {
      const response=await fetch(url,{signal:abort.signal});if(!response.ok)throw new Error('Device observation unavailable');
      const observation=await response.json();return {observation,receivedAt:performance.now()};
    } finally {clearTimeout(timer);}
  },{refreshInterval:500,dedupingInterval:250,shouldRetryOnError:false});
  // SWR pauses interval refresh after a failed request. Backend replacement
  // must recover this read-only view without preparing/replaying control.
  useEffect(()=>{
    if(!error||isValidating)return;
    const timer=setTimeout(()=>{void mutate().catch(()=>{});},1000);
    return()=>clearTimeout(timer);
  },[error,isValidating,mutate]);
  // PC time measures only the age of this HTTP snapshot, not an X5 clock.
  const value=error||!data||now-data.receivedAt>1000?undefined:data.observation;
  return {value,connected:feedbackFresh(value),isLoading};
}
