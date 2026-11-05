"""Production local calibration, native preparation, drift and domain guards."""
import base64
import copy
import gzip
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import time
from unittest.mock import patch
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
from e6_observer.kinematics_profile import load_profile,source_sha
from e6_observer.control_guardian import DashboardLink,LocalDashboardLink
from e6_observer.control_math import controller_pose,pose,angle
from e6_observer.protocol import SDK_COMMIT
from e6_observer.local_kinematics import features,POWERS,LocalKinematics,logarithm,inverse
from e6_observer.control_math import multiply

CAPTURE=Path(__file__).resolve().parents[2]/'docs/evidence/teleop_local_kinematics_20261005/sample/samples.json.gz'


@pytest.fixture
def bundle(tmp_path):
    target=tmp_path/'capture.json.gz';shutil.copyfile(CAPTURE,target)
    p=dict(schema_version=1,dataset_kind='controller_kinematics_calibration',robot_ip='192.168.1.4',user=0,tool=0,
           layout_source_commit=SDK_COMMIT,source_sha256=source_sha(),capture_file=target.name,
           capture_sha256=hashlib.sha256(target.read_bytes()).hexdigest())
    path=tmp_path/'profile.json';path.write_text(json.dumps(p),encoding='utf-8')
    return path


def rewrite(bundle,edit):
    p=json.loads(bundle.read_text());target=bundle.parent/p['capture_file']
    with gzip.open(target,'rt',encoding='utf-8') as f:record=json.load(f)
    edit(record)
    with gzip.open(target,'wt',encoding='utf-8') as f:json.dump(record,f)
    p['capture_sha256']=hashlib.sha256(target.read_bytes()).hexdigest();bundle.write_text(json.dumps(p))


def test_raw_capture_refit_and_independent_boundaries(bundle):
    m,s=load_profile(bundle)
    assert s['fit_samples']==525 and s['holdout_samples']==352
    assert s['holdout_max_position_m']<.001 and not s['physical_accuracy_verified']
    # It is a new production calibration, not a relabelled experiment profile.
    assert m.serializable()['dataset_kind']=='controller_kinematics_calibration'


def test_sparse_polynomial_and_derivatives_match_independent_definitions():
    rng=np.random.default_rng(817);x=rng.uniform(-1,1,(25,6))
    expected=np.prod(x[:,None,:]**POWERS[None,:,:],axis=-1)
    np.testing.assert_allclose(features(x),expected,atol=2e-16,rtol=2e-15)
    coefficients=rng.normal(0,.002,(len(POWERS),6));coefficients[0]=0
    m=LocalKinematics(np.zeros(6),[100,-200,400,179.9,-5,90],.17,coefficients)
    for q in rng.uniform(-.14,.14,(10,6)):
        jac=m.jacobian(q)
        for axis in range(6):
            step=np.eye(6)[axis]*1e-6;pp,rp=m.fk(q+step);pm,rm=m.fk(q-step)
            reference=np.r_[(pp-pm)/2e-6,logarithm(multiply(rp,inverse(rm)))/2e-6]
            np.testing.assert_allclose(jac[:,axis],reference,atol=1e-8,rtol=1e-6)


@pytest.mark.parametrize('field,value',[('robot_ip','192.168.1.5'),('tool',1),('source_sha256',{}),('capture_file','../capture.json.gz'),('capture_sha256','0'*64)])
def test_profile_identity_source_path_and_checksum_refused(bundle,field,value):
    p=json.loads(bundle.read_text());p[field]=value;bundle.write_text(json.dumps(p))
    with pytest.raises(ValueError):load_profile(bundle)


@pytest.mark.parametrize('kind',['motion','raw','duplicate','missing-corners','unacknowledged','wrong-fk','holdout-error'])
def test_capture_attestation_refuses_invalid_or_reused_evidence(bundle,kind):
    def edit(c):
        if kind=='motion':c['motion_tx']=1
        elif kind=='raw':c['rows'][0]['input_feedback']['decoded']['q_actual_deg'][0]+=1
        elif kind=='duplicate':c['rows'][-1]=copy.deepcopy(c['rows'][0])
        elif kind=='missing-corners':c['rows']=[v for v in c['rows'] if not all(abs(abs(x)-10)<1e-7 for x in v['request']['delta_deg'])]
        elif kind=='unacknowledged':c['rows'][0]['events'][-1]['acknowledged']=False
        elif kind=='wrong-fk':c['rows'][0]['pose_mm_deg'][0]+=1
        else:
            row=next(v for v in c['rows'] if v['request']['split']=='holdout');row['pose_mm_deg'][0]+=3
            e=row['events'][-1];e['raw_reply']='0,{'+','.join(str(v) for v in row['pose_mm_deg'])+'},'+e['command']+';'
    rewrite(bundle,edit)
    with pytest.raises(ValueError):load_profile(bundle)


def local_link(bundle):
    with patch.object(DashboardLink,'_connect'):
        link=LocalDashboardLink('192.168.1.4',bundle)
    # Stand in for a connection which has already completed resource setup.
    link.sock=object();link.connection_started_ns=time.perf_counter_ns()-2_000_000_000
    with gzip.open(CAPTURE,'rt',encoding='utf-8') as f:d=json.load(f)['first_feedback']['decoded']
    return link,d


def test_preparation_native_recheck_then_cycle_has_no_queries(bundle):
    link,d=local_link(bundle)
    with pytest.raises(ValueError,match='未完成'):link.ik(d['controller_pose_raw_mm_deg'],d['q_actual_deg'],time.perf_counter_ns()+50_000_000)
    with patch.object(DashboardLink,'_connect'),patch.object(DashboardLink,'ik',return_value=d['q_actual_deg']) as ik,patch.object(DashboardLink,'fk',return_value=d['controller_pose_raw_mm_deg']) as fk:
        link.prepare(d,time.perf_counter_ns()+50_000_000)
        assert ik.call_count==fk.call_count==1
    target_q=link.model.anchor_q+np.radians([.005,0,0,0,0,0])
    tp,tr=link.model.fk(target_q);raw=controller_pose(tp,tr,d['controller_pose_raw_mm_deg'])
    with patch.object(DashboardLink,'query',side_effect=AssertionError('Cycle queried native IO')):
        q=link.ik(raw,d['q_actual_deg'],time.perf_counter_ns()+50_000_000)
        pp,rr=pose(link.fk(q,time.perf_counter_ns()+50_000_000))
    assert np.linalg.norm(tp-pp)<1e-8 and angle(tr,rr)<1e-7
    assert link.motion_tx==link.stop_tx==0 and link.gripper is None


def test_native_preparation_branch_jump_refused(bundle):
    link,d=local_link(bundle);bad=np.asarray(d['q_actual_deg'])+[0,0,0,0,0,.1]
    with patch.object(DashboardLink,'_connect'),patch.object(DashboardLink,'ik',return_value=bad.tolist()),patch.object(DashboardLink,'fk',return_value=d['controller_pose_raw_mm_deg']):
        with pytest.raises(ValueError,match='原生'):link.prepare(d,time.perf_counter_ns()+50_000_000)
    assert not link.kinematics_status['prepared'] and link.motion_tx==link.stop_tx==0


def test_stationary_native_proof_reuse_is_separate_from_control_ownership(bundle):
    link,d=local_link(bundle)
    with patch.object(DashboardLink,'ik',return_value=d['q_actual_deg']),patch.object(DashboardLink,'fk',return_value=d['controller_pose_raw_mm_deg']):
        link.prepare(d,time.perf_counter_ns()+50_000_000)
    proof=copy.deepcopy(link.native_preparation_reference)
    with patch.object(DashboardLink,'ik',side_effect=AssertionError('Repeated same-pose native query')):
        link.prepare(d,time.perf_counter_ns()+50_000_000)
    assert link.kinematics_status['native_preparation_reused']
    assert link.native_preparation_reference==proof
    changed=copy.deepcopy(d);changed['q_actual_deg'][0]+=.011
    with patch.object(DashboardLink,'ik',side_effect=TimeoutError('Fresh native deadline')):
        with pytest.raises(TimeoutError,match='Fresh native'):link.prepare(changed,time.perf_counter_ns()+50_000_000)
    assert not link.kinematics_status['prepared']


def test_native_proof_never_masks_feedback_drift_or_controller_restart(bundle):
    link,d=local_link(bundle)
    with patch.object(DashboardLink,'ik',return_value=d['q_actual_deg']),patch.object(DashboardLink,'fk',return_value=d['controller_pose_raw_mm_deg']):
        link.prepare(d,time.perf_counter_ns()+50_000_000)
    for field,change in (('controller_pose_raw_mm_deg',None),('controller_timestamp_raw',-1)):
        bad=copy.deepcopy(d)
        if change is None:bad[field][0]+=2
        else:bad[field]+=change
        with patch.object(DashboardLink,'ik',side_effect=TimeoutError('Must not accept old proof')):
            with pytest.raises((ValueError,TimeoutError)):link.prepare(bad,time.perf_counter_ns()+50_000_000)
        assert not link.kinematics_status['prepared']


@pytest.mark.parametrize('kind',['drift','domain','restart'])
def test_actual_feedback_mismatch_domain_and_restart_refused(bundle,kind):
    link,d=local_link(bundle);link.validate_feedback(d,streamed=True);d=copy.deepcopy(d)
    if kind=='drift':d['controller_pose_raw_mm_deg'][0]+=2
    elif kind=='domain':d['joints_rad'][0]+=math.radians(11)
    else:d['controller_timestamp_raw']-=1
    with pytest.raises(ValueError):link.validate_feedback(d,streamed=True)


def test_older_selected_feedback_is_not_mistaken_for_controller_restart(bundle):
    link,d=local_link(bundle);old=copy.deepcopy(d);d['controller_timestamp_raw']+=1
    link.validate_feedback(d,streamed=True);link.validate_feedback(old)
    with pytest.raises(ValueError,match='时钟重置'):link.validate_feedback(old,streamed=True)


def test_neutral_connection_staging_has_no_device_queries_and_needs_warmup(bundle):
    link,d=local_link(bundle);link.connection_started_ns=time.perf_counter_ns()-11_000_000_000
    with patch.object(link,'close_socket',side_effect=lambda:setattr(link,'sock',None)),patch.object(link,'_connect',side_effect=lambda timeout:setattr(link,'sock',object())) as connect,patch.object(link,'query',side_effect=AssertionError('Resource setup sent a device command')):
        link.stage_neutral_connection();assert connect.call_count==0
        link.stage_neutral_connection();link.connection_staging_thread.join(2);assert connect.call_count==1
        with pytest.raises(ValueError,match='连接预备'):link.prepare(d,time.perf_counter_ns()+50_000_000)
    assert link.motion_tx==link.stop_tx==0 and not link.kinematics_status['prepared']


def test_slow_neutral_tcp_acquisition_does_not_block_control_or_retry(bundle):
    import threading
    link,d=local_link(bundle);link.sock=None
    release=threading.Event();entered=threading.Event()
    def acquire(timeout):
        assert timeout==1;entered.set();assert release.wait(2)
        link.sock=object()
    with patch.object(link,'_connect',side_effect=acquire) as connect:
        start=time.perf_counter();link.stage_neutral_connection()
        assert time.perf_counter()-start<.05 and entered.wait(1)
        assert not link.connection_ready() and link.kinematics_status['connection_staging']
        link.stage_neutral_connection();assert connect.call_count==1
        release.set();link.connection_staging_thread.join(2)
        assert not link.connection_ready()  # The one-second warmup still applies.
    assert link.motion_tx==link.stop_tx==0


def test_failed_neutral_tcp_acquisition_is_terminal_in_this_session(bundle):
    link,d=local_link(bundle);link.sock=None
    with patch.object(link,'_connect',side_effect=TimeoutError('synthetic SYN delay')) as connect:
        link.stage_neutral_connection();link.connection_staging_thread.join(2)
        assert not link.connection_ready()
        with pytest.raises(ValueError,match='Neutral connection staging failed'):link.stage_neutral_connection()
        assert connect.call_count==1
    assert link.motion_tx==link.stop_tx==0
