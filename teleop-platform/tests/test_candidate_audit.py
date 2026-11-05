"""Offline audit identity checks; no devices or network."""
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

spec=importlib.util.spec_from_file_location('page_audit',Path(__file__).resolve().parents[1]/'scripts/audit_moveit_page.py')
audit=importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def event(at,connection,intent):
    return dict(at_ns=at,connection_id=connection,data=dict(seq=17,intent=intent))


def test_sequence_reuse_selects_only_the_connection_receipt_before_dispatch():
    old=event(100_000_000,1,'neutral')
    active=event(500_000_000,2,'active')
    new=event(900_000_000,3,'neutral')
    assert audit.recorded_input({17:[new,old,active]},dict(sequence=17,at_ns=550_000_000)) is active


def test_future_and_stale_receipts_cannot_prove_source():
    inputs={17:[event(100_000_000,1,'active'),event(900_000_000,2,'active')]}
    assert audit.recorded_input(inputs,dict(sequence=17,at_ns=99_999_999)) is None
    assert audit.recorded_input(inputs,dict(sequence=17,at_ns=350_000_001)) is None
    assert audit.recorded_input(inputs,dict(sequence=18,at_ns=110_000_000)) is None


def test_matching_payload_does_not_override_a_newer_contradictory_receipt():
    inputs={17:[event(100_000_000,1,'expected'),event(120_000_000,2,'different')]}
    found=audit.recorded_input(inputs,dict(sequence=17,at_ns=125_000_000,intent='expected'))
    assert found['data']['intent']=='different'  # Caller must report input_mismatch.


def test_empty_candidate_audit_keeps_report_but_returns_failure(tmp_path):
    (tmp_path/'raw.jsonl').write_text('')
    (tmp_path/'events.jsonl').write_text('')
    output=tmp_path/'report.json'
    result=subprocess.run([sys.executable,spec.origin,str(tmp_path),'--output',str(output)],capture_output=True)
    assert result.returncode==1
    assert json.loads(output.read_text())['audit_passed'] is False
