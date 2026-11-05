"""Read-only timeline of the first device collision, preserving all source frames.

Consumes pinned-SDK readback from analyze_servoj_following, never connects to
hardware. Position errors are diagnostics, not new control thresholds. Neither
current traces nor a timing correlation establishes contact or a firmware cause.
"""
import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('session',type=Path);p.add_argument('following',type=Path)
    a=p.parse_args()
    tracking=json.loads((a.following/'analysis.json').read_text(encoding='utf8'))
    audit=a.session/'session/x5/audit'
    assert all(digest(audit/n)==sha for n,sha in tracking['source_hashes'].items())
    g=tracking['groups'][0]
    raw=a.following/g['raw_csv']['file'];assert digest(raw)==g['raw_csv']['sha256']
    with raw.open(newline='',encoding='utf8') as f: rows=list(csv.DictReader(f))
    col=lambda k: np.array([float(r[k]) for r in rows])
    first=int(np.flatnonzero(col('CollisionState'))[0])
    received=np.array([int(r['received_ns']) for r in rows],dtype=np.int64)
    collision_ns=int(received[first]);times=col('since_input_ms')
    qa=np.array([col('QActual_'+str(j)) for j in range(1,7)]).T
    qt=np.array([col('QTarget_'+str(j)) for j in range(1,7)]).T
    before=(received<collision_ns)&(received>=g['start_ns'])
    errors=np.max(np.abs(qa[before]-qt[before]),axis=0)
    joint=int(np.argmax(errors));commands=g['commands']
    ct=np.array([c['sent_ns'] for c in commands],dtype=np.int64)
    cq=np.array([c['joints_deg'] for c in commands])
    with gzip.open(audit/'events.jsonl.gz','rt',encoding='utf8') as f:events=[json.loads(line) for line in f]
    stops=[e for e in events if e['event']=='stop_result' and e['receipt']['sent_ns']>=collision_ns]
    receipt=stops[0]['receipt']
    result=dict(source_hashes=tracking['source_hashes'],tracking_sha256=digest(a.following/'analysis.json'),
        script_sha256=digest(Path(__file__)),device_connected=False,
        first_collision=dict(sequence=int(rows[first]['sequence']),received_ns=collision_ns,
            since_input_ms=times[first],joints_deg=qa[first].tolist(),internal_target_deg=qt[first].tolist()),
        max_error_strictly_before_collision_deg=errors.tolist(),
        selected_joint=joint+1,stop_receipt=receipt,
        collision_receive_to_stop_send_ms=(receipt['send_completed_ns']-collision_ns)/1e6,
        collision_receive_to_stop_reply_ms=(receipt['reply_ns']-collision_ns)/1e6,
        sends_after_first_collision=int(sum(ct>=collision_ns)),
        sent_gap_ms_p50_p95_p99_max=np.percentile(np.diff(ct)/1e6,[50,95,99,100]).tolist(),
        reply_ms_p50_p95_p99_max=np.percentile([(c['receipt']['reply_ns']-c['sent_ns'])/1e6 for c in commands],[50,95,99,100]).tolist(),
        finite_difference_sent_velocity_deg_s_max=np.max(np.abs(np.diff(cq,axis=0)/(np.diff(ct)[:,None]/1e9)),axis=0).tolist(),
        sdk_options=commands[0]['sdk_options'],
        interpretation='All times on X5 monotonic clock. Pre-collision excludes collision/retreat frames. No inference of physical contact, motor limit or sole timing cause. Collision Stop is not a normal release test.')
    (a.following/'collision.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
    for ax in axes[:1]:
        ax.plot(times,qa[:,joint],label=f'J{joint+1} actual',lw=1.4)
        ax.plot(times,qt[:,joint],label='controller target',lw=1.1)
        ax.plot((ct-g['start_ns'])/1e6,cq[:,joint],'.-',label='sent ServoJ target',lw=1)
        ax.axvline(times[first],color='red',ls='--',label='first collision frame')
        ax.axvline((receipt['sent_ns']-g['start_ns'])/1e6,color='black',ls=':',label='Stop sent')
        ax.set_ylabel('Joint position (degrees)');ax.grid(alpha=.25)
    axes[0].legend(loc='best',fontsize=8)
    axes[0].set_title('Original X5 timestamps: sent, controller target, actual')
    axes[0].set_xlim(times[0],times[-1])
    axes[1].plot(times,qa[:,joint]-qt[:,joint],label='actual minus internal target')
    axes[1].axvline(times[first],color='red',ls='--',label='first collision frame')
    axes[1].axvline((receipt['sent_ns']-g['start_ns'])/1e6,color='black',ls=':',label='Stop sent')
    axes[1].set_ylabel('Following difference (degrees)');axes[1].set_xlabel('Time since input received (ms)')
    axes[1].grid(alpha=.25);axes[1].legend(fontsize=8)
    fig.tight_layout();fig.savefig(a.following/'collision.png',dpi=150);plt.close(fig)
    print(json.dumps(result))


if __name__=='__main__':main()
