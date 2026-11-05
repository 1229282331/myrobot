import {ReactNode} from 'react';
import {Link,useLocation} from 'react-router-dom';
import {Bot,Camera,FolderOpen,Play,Info} from 'lucide-react';
import {Card,CardHeader,CardTitle,CardContent} from '@/components/ui/card';
import {Badge} from '@/components/ui/badge';
import {Button} from '@/components/ui/button';
import {Alert,AlertTitle} from '@/components/ui/alert';
import {CameraStreamCard} from '@/components/common/camera-stream-card';
import {useWorkbenchMode,useDeviceObservation,isUnavailableRoute} from './WorkbenchChrome';

export function E6RouteGuard({children}:{children:ReactNode}) {
  const e6=useWorkbenchMode();const location=useLocation();
  if(e6===undefined)return <p role="status">Loading…</p>;
  if(e6&&isUnavailableRoute(location.pathname))return <Alert><Info className="size-4"/><AlertTitle>Not adapted for E6</AlertTitle></Alert>;
  return children;
}
export function DeviceConnectionBadge() {
  const {connected,isLoading}=useDeviceObservation();
  return <Badge variant="outline" className="gap-2 py-2" aria-label="E6 connection">
    <span className={`size-2 rounded-full ${connected?'bg-green-500':'bg-muted-foreground'}`}/>
    E6 · {isLoading?'Loading':connected?'Connected':'Offline'}</Badge>;
}
export function CameraPreview({url,footer}:{url?:string;footer?:ReactNode}) {
  return url?<CameraStreamCard id={0} title="Gemini E · RGB" streamPath={'webrtc:'+url} footer={footer}/>:
    <Card className="e6-camera-card"><CardHeader><CardTitle className="flex items-center gap-2"><Camera className="size-5"/>Gemini E</CardTitle></CardHeader>
      <CardContent className="flex min-h-32 items-center justify-center text-muted-foreground text-sm">No camera stream</CardContent></Card>;
}
export function CameraOverview() {
  const {value}=useDeviceObservation();
  return <div className="max-w-4xl"><CameraPreview url={value?.capabilities?.video_url}/></div>;
}
export function DeviceDashboard() {
  const {value,connected}=useDeviceObservation();
  const opening=value?.gripper?.stroke?value.gripper.opening:value?.actual?.gripper;
  const items=[['Robot',connected?'Connected':'Offline',Bot],['Camera',value?.camera?.valid?'Streaming':'No stream',Camera],
    ['Gripper',connected&&value?.gripper?.valid===true&&opening!=null?`${(opening*100).toFixed(1)}%`:'Unknown',Bot]] as const;
  return <div className="space-y-6">
    <div className="grid gap-6 md:grid-cols-3">{items.map(([name,status,Icon])=><Card key={name}>
      <CardHeader><CardTitle className="flex items-center gap-2"><Icon className="size-5"/>{name}</CardTitle></CardHeader>
      <CardContent><Badge variant="outline">{status}</Badge></CardContent></Card>)}</div>
    <div className="flex flex-wrap gap-3"><Button asChild><Link to="/control"><Play/>Control Robot</Link></Button>
      <Button variant="outline" asChild><Link to="/browse"><FolderOpen/>Browse Datasets</Link></Button>
      <Button variant="outline" asChild><Link to="/viz"><Camera/>Camera Overview</Link></Button></div>
  </div>;
}
