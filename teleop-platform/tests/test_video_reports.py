import hashlib
import json

from fastapi import APIRouter, FastAPI
from fastapi.testclient import TestClient
import pytest

from e6_phosphobot import video_reports


@pytest.fixture
def reports(tmp_path):
    app=FastAPI();router=APIRouter()
    video_reports.install_video_reports(router,tmp_path)
    app.include_router(router)
    return TestClient(app),tmp_path


def test_report_is_saved_verbatim_and_downloadable_without_devices(reports):
    client,directory=reports
    body=json.dumps(dict(schema=2,samples=[dict(frameId=17)],presentationWindows=[],
                         decodeFailures=3,method='原始测量'),ensure_ascii=False).encode('utf8')
    response=client.post('/e6/diagnostics/video',content=body,headers={'content-type':'application/json'})
    assert response.status_code==200
    receipt=response.json()
    assert (directory/receipt['name']).read_bytes()==body
    assert receipt['sha256']==hashlib.sha256(body).hexdigest()
    assert receipt['bytes']==len(body) and receipt['pc_saved_monotonic_ns']>0
    result=client.get(receipt['url'])
    assert result.content==body and 'attachment' in result.headers['content-disposition']
    assert len(list(directory.iterdir()))==1


@pytest.mark.parametrize('body',[b'{}',b'null',b'bad',b'{"schema":2,"samples":[],"presentationWindows":[],"n":NaN}'])
def test_invalid_reports_do_not_create_files(reports,body):
    client,directory=reports
    assert client.post('/e6/diagnostics/video',content=body,headers={'content-type':'application/json'}).status_code==422
    assert not list(directory.iterdir())


def test_limits_apply_before_writing_and_names_cannot_escape(reports,monkeypatch):
    client,directory=reports
    monkeypatch.setattr(video_reports,'MAX_REPORT_BYTES',20)
    assert client.post('/e6/diagnostics/video',content=b'X'*21,headers={'content-type':'application/json'}).status_code==413
    assert client.post('/e6/diagnostics/video',content=b'{}',headers={'content-type':'text/plain'}).status_code==415
    assert client.get('/e6/diagnostics/video/auto.key').status_code==404
    assert client.get('/e6/diagnostics/video/video-'+'0'*32+'.json').status_code==404
    assert not list(directory.iterdir())


def test_cross_origin_site_cannot_write_local_diagnostics(reports):
    client,directory=reports
    body=dict(schema=2,samples=[],presentationWindows=[])
    assert client.post('/e6/diagnostics/video',json=body,headers={'origin':'https://other.example'}).status_code==403
    assert not list(directory.iterdir())
    assert client.post('/e6/diagnostics/video',json=body,headers={'origin':'http://testserver'}).status_code==200


def test_disk_failure_does_not_claim_success_or_leave_a_partial_report(reports,monkeypatch):
    from pathlib import Path
    client,directory=reports
    def fail(*args):raise OSError('injected disk failure')
    monkeypatch.setattr(Path,'replace',fail)
    assert client.post('/e6/diagnostics/video',json=dict(schema=2,samples=[],presentationWindows=[])).status_code==507
    assert not list(directory.iterdir())
