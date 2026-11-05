import importlib.util
import json
from pathlib import Path


source=Path(__file__).resolve().parents[1]/'scripts/qualification_cleanup.py'
spec=importlib.util.spec_from_file_location('qualification_cleanup',source)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_failure_does_not_skip_next_release_or_original_error(tmp_path):
    c=module.Cleanup(tmp_path/'cleanup.json')
    completed=[]
    def fail():raise TimeoutError('original wait timeout')
    assert not c.run('worker wait',fail)
    assert c.run('release next resource',lambda:completed.append('released'))
    rows=json.loads(c.path.read_text())
    assert completed==['released'] and len(c.failures)==1
    assert 'TimeoutError: original wait timeout' in rows[0]['error']
    assert rows[1]['error'] is None


def test_persistence_failure_does_not_stop_cleanup(tmp_path):
    c=module.Cleanup(tmp_path/'missing'/'cleanup.json')
    completed=[]
    for i in range(2):assert not c.run(str(i),lambda:completed.append(1))
    assert completed==[1,1] and len(c.failures)==2
    assert all(row['persistence_error'] for row in c.steps)
