"""Dependency reuse cannot unlock changed control code or altered evidence."""
import hashlib,json,sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import control_acceptance as harness

def review(tmp_path,monkeypatch):
    old={'device_gateway/e6_observer/protocol.py':'old','device_gateway/e6_observer/control_core.py':'same'}
    new=dict(old,**{'device_gateway/e6_observer/protocol.py':'new'})
    monkeypatch.setattr(harness,'source_hashes',lambda:new)
    proof=tmp_path/'proof.json';proof.write_text('{"passed":true}')
    value=dict(passed=True,scope='lossless_observation_header_only',original_source_sha256=old,
        current_source_sha256=new,control_limits_or_deadlines_changed=False,
        proofs=[dict(file=proof.name,sha256=hashlib.sha256(proof.read_bytes()).hexdigest())])
    path=tmp_path/'review.json';path.write_text(json.dumps(value))
    return old,new,proof,value,path

def test_old_shadow_requires_explicit_exact_framing_review(tmp_path,monkeypatch):
    old,new,proof,value,path=review(tmp_path,monkeypatch)
    monkeypatch.delenv('E6_TELEOP_SHADOW_COMPATIBILITY',raising=False)
    assert not harness.shadow_sources_match({'source_sha256':old})
    assert harness.shadow_sources_match({'source_sha256':old},path)
    proof.write_text('changed')
    assert not harness.shadow_sources_match({'source_sha256':old},path)

@pytest.mark.parametrize('invalid',['control-code','scope','deadline','empty-proofs','original-source'])
def test_changed_control_or_missing_provenance_cannot_reuse(tmp_path,monkeypatch,invalid):
    old,new,proof,value,path=review(tmp_path,monkeypatch)
    if invalid=='control-code':new['device_gateway/e6_observer/control_core.py']='changed'
    if invalid=='scope':value['scope']='all'
    if invalid=='deadline':value['control_limits_or_deadlines_changed']=True
    if invalid=='empty-proofs':value['proofs']=[]
    if invalid=='original-source':value['original_source_sha256']={}
    path.write_text(json.dumps(value))
    assert not harness.shadow_sources_match({'source_sha256':old},path)
