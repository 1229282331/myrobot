import importlib.util
from pathlib import Path

spec=importlib.util.spec_from_file_location('release_classification',Path(__file__).parents[1]/'scripts/release_classification.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_fault_stop_after_received_release_is_not_normal_release():
    # v171: release was on the socket before a locked tick suspended the
    # session. Its later Stop must stay a fault despite this timestamp order.
    processed={100:dict(received_ns=100,at_ns=160,ready=False,reason='candidate stream stale')}
    assert module.classify_release(100,{'at_ns':150},[],processed)==(False,'candidate stream stale')


def test_normal_release_requires_exact_processing_identity_and_ready_session():
    processed={100:dict(ready=True,reason=None)}
    assert module.classify_release(100,{'at_ns':150},[],processed)==(True,None)
    assert module.classify_release(101,{'at_ns':150},[],processed)==(False,'release_processing_unobserved')
    assert module.classify_release(100,{'at_ns':150},[{'at_ns':90}],processed)==(False,'stop_preceded_release')
    assert module.classify_release(100,None,[],processed)==(False,'no_stop_observed')


def test_workspace_stop_between_receipt_and_processing_is_not_normal_release():
    # v182: normal release cleared the displayed workspace reason, but the
    # preceding Stop was already requested by a rejected outward candidate.
    processed={100:dict(at_ns=160,ready=True,reason=None)}
    assert module.classify_release(100,{'at_ns':150},[],processed,[{'at_ns':120}])==(False,'workspace_boundary_before_stop')
    assert module.classify_release(100,{'at_ns':150},[],processed,[{'at_ns':90},{'at_ns':180}])==(True,None)
