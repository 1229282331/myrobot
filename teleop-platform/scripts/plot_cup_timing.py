"""Plot retained isolated timing evidence; never produces device commands."""
from pathlib import Path
import argparse,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
 ap=argparse.ArgumentParser();ap.add_argument('root',type=Path);a=ap.parse_args();root=a.root
 fig,axes=plt.subplots(2,2,figsize=(13,8),constrained_layout=True)
 colors={'baseline':'#64748b','candidate':'#16866e'}
 tables={}
 for variant in colors:
  path=root/('timing-'+variant)/'frames.json'
  if not path.exists():continue
  rows=[r for r in json.loads(path.read_text()) if r['active']]
  values=np.array([r['reply']['callback_us']/1000 for r in rows])
  axes[0,0].plot(np.arange(len(values)),values,label=variant,color=colors[variant],alpha=.85,lw=.8)
  axes[0,1].plot(np.sort(values),np.arange(1,len(values)+1)/len(values),label=variant,color=colors[variant])
  tables[variant]=dict(active=len(rows),callback_ms=np.percentile(values,[50,95,99,100]).tolist(),over_42ms=int((values>=42).sum()))
 axes[0,0].axhline(42,color='#b91c1c',ls='--',label='42 ms cutoff');axes[0,0].set(xlabel='Active sample',ylabel='Callback (ms)',title='Real Servo calculation, synthetic plant')
 axes[0,1].axvline(42,color='#b91c1c',ls='--');axes[0,1].set(xlabel='Callback (ms)',ylabel='Cumulative fraction',title='Callback distribution')
 for variant in colors:
  for journals,style in ((0,':'),(2,'-')):
   path=root/f'pipeline-{variant}-{journals}'/'accepted.json'
   if not path.exists():continue
   values=np.array([r['take']['sample_age_ms'] for r in json.loads(path.read_text())]);values.sort()
   if len(values):axes[1,0].plot(values,np.arange(1,len(values)+1)/len(values),color=colors[variant],ls=style,label=f'{variant}, {journals} journals')
 axes[1,0].set(xlabel='Sample age when consumed (ms)',ylabel='Cumulative fraction',title='Worker + IPC + audit; not physical acceptance')
 labels=[];p95=[];maximum=[]
 for path in sorted(root.glob('audit-load-prod-*/summary.json')):
  row=json.loads(path.read_text());labels.append(path.parent.name.removeprefix('audit-load-prod-'));p95.append(row['scheduler_lateness_ms']['p95']);maximum.append(row['scheduler_lateness_ms']['max'])
 x=np.arange(len(labels));axes[1,1].bar(x-.18,p95,.36,label='P95');axes[1,1].bar(x+.18,maximum,.36,label='max')
 axes[1,1].set(xticks=x,xticklabels=labels,ylabel='Wakeup lateness (ms)',title='Dual audit ABBA, production 1 ms handoff')
 for ax in axes.flat:
  ax.grid(True,alpha=.2);ax.legend(fontsize=8)
 fig.suptitle('E6 v296 isolated timing qualification — original deadlines unchanged',fontsize=13)
 fig.savefig(root/'timing-curves.png',dpi=160);plt.close(fig)
 (root/'timing-plot-summary.json').write_text(json.dumps(tables,indent=2));print(json.dumps(tables))


if __name__=='__main__':main()
