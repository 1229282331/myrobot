"""Changing displayed connection must never discard a slow earlier connection."""
import importlib.util
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('video_audit',Path(__file__).resolve().parents[1]/'scripts/analyze_video_timing.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


def report():
    samples=[]
    for connection,at,delay in [(1,1000,400),(2,2000,100)]:
        error=1.1
        samples.append(dict(connection=connection,presented=at,sourceMs=at-delay,
            basis='uvc_kernel_receipt',captureEpoch=str(connection),frameId=1,
            clock=dict(startedAt=at-100,at=at-50,lower=0,upper=0),
            driftAndQuantizationMs=error,lowerMs=delay-error,upperMs=delay+error,clockWidthMs=2*error))
    return dict(schema=2,method='test',samples=samples,attempts=2,decodeFailures=0,
        unsynchronized=0,invalidBounds=0,truncated=0,connections=2,
        presentationWindows=[dict(connection=1,startMs=0,endMs=1000,frames=20,fps=20,dropped=0)],
        summary=dict(connection=2,samples=1,p95Lower=98.9,p95Upper=101.1))


def test_current_display_does_not_hide_slow_history_from_acceptance():
    result=audit.analyze(report())
    assert result['samples']==2 and result['displayed_connection_samples']==1
    assert result['upper_ms']['p95']==401.1
    assert not result['measured_checks']['sampled_p95_upper_at_most_300ms']


def test_legacy_all_connection_summary_remains_readable():
    data=report();data['summary']=dict(samples=2,p95Lower=398.9,p95Upper=401.1)
    assert audit.analyze(data)['displayed_connection'] is None


def test_incorrect_active_summary_is_rejected():
    data=report();data['summary']['samples']=2
    with pytest.raises(AssertionError):audit.analyze(data)
