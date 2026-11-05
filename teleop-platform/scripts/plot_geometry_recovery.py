"""Plot isolated timing evidence, never label it as physical motion."""
from pathlib import Path
import argparse,json,collections
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
 p=argparse.ArgumentParser();p.add_argument('evidence',type=Path);a=p.parse_args();root=a.evidence/'x5'
 sources=[('Deployed baseline',root/'attempt-a/baseline-smoke/frames.json'),
          ('Reuse only (A)',root/'attempt-a/candidate-smoke/frames.json'),
          ('Reuse + early negative proof (B)',root/'candidate-smoke/frames.json')]
 colors=['#c64545','#df9b20','#2874af'];fig,axes=plt.subplots(3,1,figsize=(11,8),layout='constrained')
 for index,((label,path),color) in enumerate(zip(sources,colors)):
  if not path.exists():continue
  expected={x['case']:x['active_frames'] for x in json.loads((path.parent/'cases.json').read_text())};seen=collections.Counter();rows=[]
  for x in json.loads(path.read_text()):
   if x['reply']['active']:
    seen[x['case']]+=1
    if seen[x['case']]<=expected[x['case']]:rows.append(x)
  ms=np.array([x['reply']['callback_us']/1000 for x in rows]);finger=np.array([x['reply']['gripper_check']['elapsed_us']/1000 for x in rows])
  axes[0].plot(np.arange(len(rows))*.05,ms,label=label,color=color,lw=1.3)
  axes[1].plot(np.arange(len(rows))*.05,finger,label=label,color=color,lw=1.3)
  accepted=sum(x['accepted'] for x in rows);axes[2].bar(index,accepted/len(rows)*100,color=color)
  axes[2].text(index,accepted/len(rows)*100+1,f'{accepted}/{len(rows)}',ha='center')
 axes[2].set_xticks(range(len(sources)),[label for label,_ in sources])
 axes[0].axhline(42,color='#b91c1c',ls='--',lw=1,label='42 ms acceptance deadline')
 axes[0].axhline(50,color='#555555',ls=':',lw=1,label='50 ms nominal cycle')
 axes[0].set_title('Isolated X5 replay at captured stuck pose (synthetic feedback)');axes[0].set_ylabel('Whole callback (ms)');axes[0].legend(fontsize=8,ncol=2)
 axes[1].set_ylabel('Gripper geometry (ms)');axes[1].set_xlabel('Active frame index × 50 ms (case gaps omitted)')
 axes[2].set_ylabel('Fresh permitted targets (%)');axes[2].set_ylim(0,112)
 for ax in axes:ax.grid(axis='y',alpha=.2);ax.spines[['top','right']].set_visible(False)
 fig.savefig(a.evidence/'isolated-timing.png',dpi=160);plt.close(fig)

if __name__=='__main__':main()
