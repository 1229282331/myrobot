import {useRef,useState} from 'react';
import {Button} from '@/components/ui/button';
import {Popover,PopoverTrigger,PopoverContent} from '@/components/ui/popover';

type Receipt={name:string;url:string;sha256:string;bytes:number;files:number;omissions:number};

export function ProblemReport() {
  const [saving,setSaving]=useState(false);
  const [receipt,setReceipt]=useState<Receipt|null>(null);
  const [error,setError]=useState('');
  const pending=useRef(false);
  const save=async()=>{
    if(pending.current)return;
    pending.current=true;setSaving(true);setError('');setReceipt(null);
    try {
      const response=await fetch('/e6/diagnostics/problem',{method:'POST'});
      if(!response.ok)throw new Error(response.status===409?'已有问题包正在保存，请稍后查看。':'问题包未保存，请检查本PC磁盘空间后重试。');
      const result:Receipt=await response.json();
      if(!/^problem-[0-9a-f]{32}\.zip$/.test(result.name)||result.url!==`/e6/diagnostics/problem/${result.name}`
          ||!/^[0-9a-f]{64}$/.test(result.sha256)||!Number.isSafeInteger(result.bytes)||result.bytes<=0
          ||!Number.isSafeInteger(result.files)||result.files<0||!Number.isSafeInteger(result.omissions)||result.omissions<0)
        throw new Error('保存回执无效，请检查本机服务。');
      setReceipt(result);
    } catch(error) {setError(error instanceof Error?error.message:'问题包未保存。');}
    finally {pending.current=false;setSaving(false);}
  };
  return <Popover><PopoverTrigger asChild><Button size="sm" variant="ghost">Problem report</Button></PopoverTrigger><PopoverContent data-e6-input-isolation className="space-y-3 text-sm">
    <Button variant="outline" disabled={saving} onClick={save}>{saving?'Saving…':'Export problem report'}</Button>
    {error&&<p role="alert">{error}</p>}
    {receipt&&<div role="status" className="text-sm break-all">
      <p>已保存：{receipt.name} · {(receipt.bytes/1024).toFixed(0)} KiB</p>
      <a className="underline" href={receipt.url} download={receipt.name}>Download report</a>
      <p>{receipt.files}份诊断文件 · {receipt.omissions}项未纳入（详见清单）</p>
    </div>}
  </PopoverContent></Popover>;
}
