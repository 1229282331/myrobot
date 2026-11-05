from pathlib import Path
import os
import pytest
from e6_phosphobot.jazzy_host import isolated_runtime_argv


def test_runtime_paths_are_argv_and_mount_failure_prevents_child(tmp_path):
    root=tmp_path/'root with spaces';(root/'dev/shm').mkdir(parents=True)
    runtime=tmp_path/'session';runtime.mkdir()
    child=['chroot',str(root),'/bin/bash','-c','echo literal $variable; exit 9']
    argv=isolated_runtime_argv(root,runtime,child)
    assert argv[-len(child):]==child
    assert str(root/'dev/shm') not in argv[7]
    assert argv[:6]==['unshare','--mount','--propagation','private','--','/bin/bash']
    assert argv[7].startswith('set -e; mount ')
    assert 'exec "$@"' in argv[7]


def test_reject_host_root_and_missing_directory(tmp_path):
    with pytest.raises(ValueError):isolated_runtime_argv(Path(tmp_path.anchor),tmp_path,['true'])
    with pytest.raises(ValueError):isolated_runtime_argv(tmp_path/'missing',tmp_path,['true'])


@pytest.mark.skipif(os.name=='nt',reason='Unix symlink escape check; run on Linux')
def test_reject_symlink_outside_runtime(tmp_path):
    root=tmp_path/'root';(root/'dev').mkdir(parents=True)
    outside=tmp_path/'outside';outside.mkdir();(root/'dev/shm').symlink_to(outside,target_is_directory=True)
    with pytest.raises(ValueError):isolated_runtime_argv(root,tmp_path,['true'])
