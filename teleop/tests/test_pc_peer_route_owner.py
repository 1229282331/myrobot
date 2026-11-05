"""Route creation failure cannot silently leak or remove another owner's route."""
import importlib.util
import json
from pathlib import Path
from unittest.mock import patch
import pytest

spec=importlib.util.spec_from_file_location('peer_route',Path(__file__).parents[1]/'scripts/configure_pc_peer_route.py')
route=importlib.util.module_from_spec(spec);spec.loader.exec_module(route)
selected=dict(dev='eth0',gateway='172.24.208.1',prefsrc='172.24.219.228')
created=dict(dst=route.PEER,**selected,protocol=242,metrics=[dict(rto_min=20)])


def test_create_records_verified_route(tmp_path):
    calls=[]
    def command(*args):
        calls.append(args)
        return json.dumps([selected]) if args[:4]==('ip','-j','route','get') else ''
    with patch.object(route,'inactive'),patch.object(route,'exact',side_effect=[[],[created]]),patch.object(route,'command',command):
        route.create(tmp_path)
    assert json.loads((tmp_path/'owner.json').read_text())['created']==[created]
    assert sum('add' in a for a in calls)==1


@pytest.mark.parametrize('changed',[False,True])
def test_readback_failure_rolls_back_only_identical_owned_route(tmp_path,changed):
    calls=[]
    current=dict(created,protocol=241) if changed else created
    def command(*args):
        calls.append(args)
        return json.dumps([selected]) if args[:4]==('ip','-j','route','get') else ''
    with patch.object(route,'inactive'),patch.object(route,'exact',side_effect=[[],RuntimeError('readback failed'),[current],[]]),patch.object(route,'command',command):
        with pytest.raises(RuntimeError):route.create(tmp_path)
    result=json.loads((tmp_path/'create_failure.json').read_text())
    assert result['restored'] is (not changed)
    assert sum('del' in a for a in calls)==int(not changed)
    assert not (tmp_path/'owner.json').exists()


def test_failed_add_never_deletes_concurrent_route(tmp_path):
    calls=[]
    def command(*args):
        calls.append(args)
        if 'add' in args:raise RuntimeError('route already exists')
        return json.dumps([selected])
    with patch.object(route,'inactive'),patch.object(route,'exact',side_effect=[[],[created]]),patch.object(route,'command',command):
        with pytest.raises(RuntimeError):route.create(tmp_path)
    assert not any('del' in a for a in calls)
    assert json.loads((tmp_path/'create_failure.json').read_text())['manual_recovery_required']
