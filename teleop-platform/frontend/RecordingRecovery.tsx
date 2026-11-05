import useSWR from 'swr';
import {FolderOpen, ShieldCheck} from 'lucide-react';
import {EpisodeBrowser} from './EpisodeBrowser';

const read=async(url:string)=>{
  const response=await fetch(url);
  if(!response.ok)throw new Error('历史记录服务暂不可用');
  return response.json();
};

export function useRecordingRecovery() {
  const {data}=useSWR('/e6/capabilities',read);
  return !!data?.recordings_only;
}

export function RecordingRecovery() {
  const {data,error}=useSWR('/e6/episodes',read,{refreshInterval:2000});
  return <section className="mx-auto max-w-4xl space-y-6 py-8" aria-label="历史记录恢复">
    <div className="e6-control-header"><div><h2><FolderOpen className="mr-2 inline size-6"/>历史记录恢复</h2>
      <p>找回已结束会话中的原始示教，包括中止与失败片段。</p></div></div>
    <div className="rounded-xl border bg-card p-6 space-y-4">
      <p className="flex items-center gap-2"><ShieldCheck className="size-5"/>仅回传与浏览 · 设备连接关闭</p>
      <p className="text-sm text-muted-foreground">通过独立无线媒体通道恢复已封存文件，校验原图、反馈与动作关联。原始结果标签保留。退出后使用普通启动入口恢复遥操作。</p>
      <p role="status">{data?.transferring?`正在恢复 ${data.transferring}`:'正在检查保留记录；已校验的片段可以直接浏览。'}</p>
      {(error||data?.error)&&<p role="alert" className="text-destructive">{error?.message||data?.error}</p>}
      <EpisodeBrowser state={{phase:'idle'}} compact recoveryOnly/>
    </div>
  </section>;
}
