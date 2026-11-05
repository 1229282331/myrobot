"""Plot recorded controller tracking; performs no device IO."""
from pathlib import Path
import base64
import gzip
import json
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

root = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'upstream/dobot-sdk'))
from dobot_sdk.protocol.data_types import MyType
base=Path(sys.argv[1])
report=json.loads((base/'continuous.json').read_text(encoding='utf8'))
group=report['groups'][int(sys.argv[2]) if len(sys.argv)>2 else -1]
start=group['start_ns']; finish=group['end_ns']+1_500_000_000
rows=[]
with gzip.open(base/'session/x5/audit/raw.jsonl.gz','rt') as stream:
    for line in stream:
        event=json.loads(line)
        if not start<=event['received_ns']<finish:continue
        row=np.frombuffer(base64.b64decode(event['raw_b64']),dtype=MyType)[0]
        rows.append([(event['received_ns']-start)/1e9,*row['ToolVectorActual'],row['QActual'][3],row['QTarget'][3]])
raw=np.array(rows)
commands=[]
with gzip.open(base/'session/x5/audit/events.jsonl.gz','rt',encoding='utf8') as stream:
    for line in stream:
        event=json.loads(line)
        if event['event']!='motion_submitted' or not start<=event['at_ns']<finish:continue
        c=event['candidate']; r=Rotation.from_quat(c['quaternion_xyzw'])
        commands.append([(event['at_ns']-start)/1e9,*np.array(c['position_m'])*1000,*r.as_euler('xyz',degrees=True)])
commands=np.array(commands)
fig,axes=plt.subplots(4,1,figsize=(11,9),sharex=True,layout='constrained')
axes[0].plot(raw[:,0],raw[:,2],label='Actual Y (controller feedback)',color='#1565c0')
if len(commands):axes[0].plot(commands[:,0],commands[:,2],'.-',label='Sent candidate Y',color='#f57c00',markersize=3)
axes[0].set_ylabel('Y (mm)');axes[0].legend(loc='best')
axes[1].plot(raw[:,0],raw[:,7],label='J4 actual',color='#1565c0')
axes[1].plot(raw[:,0],raw[:,8],label='J4 internal target',color='#f57c00')
axes[1].set_ylabel('J4 (deg)');axes[1].legend(loc='best')
axes[2].plot(raw[:,0],raw[:,7]-raw[:,8],color='#c62828',label='J4 actual minus internal target')
axes[2].set_ylabel('Error (deg)');axes[2].legend(loc='best')
r0=Rotation.from_euler('xyz',raw[0,4:7],degrees=True)
axes[3].plot(raw[:,0],(Rotation.from_euler('xyz',raw[:,4:7],degrees=True)*r0.inv()).magnitude()*180/np.pi,label='Actual orientation change')
if len(commands):axes[3].plot(commands[:,0],(Rotation.from_euler('xyz',commands[:,4:7],degrees=True)*r0.inv()).magnitude()*180/np.pi,'.-',label='Sent orientation change',markersize=3)
axes[3].set_ylabel('Angle (deg)');axes[3].set_xlabel('Seconds since X5 received held input');axes[3].legend(loc='best')
for ax in axes:
    for stamp in group['intermediate_stops']:ax.axvline((stamp-start)/1e9,color='#555',ls='--',alpha=.6)
    ax.axvline((group['end_ns']-start)/1e9,color='#2e7d32',ls=':',label='Release')
    ax.grid(alpha=.2)
fig.suptitle('30 mm/s continuous Y trial: failed following / boundary stop\nDashed: Stop requests; dotted: final input release. All samples retained.')
fig.savefig(base/'continuous-following.png',dpi=150)
