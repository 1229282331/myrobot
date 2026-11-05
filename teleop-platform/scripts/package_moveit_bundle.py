"""Package pinned Servo sources, reviewed model and mesh artifacts for X5."""
import argparse
import io
from pathlib import Path
import tarfile


def package(root,output):
    sources=[(root/'config/e6-controller-model','model'),
        (root/'runtime/e6-collision-hulls-v2','collision'),
        (root/'upstream/dobot-ros2/dobot_rviz/meshes/me6','meshes'),
        (root/'scripts/kdl_model_probe.cpp','kdl_model_probe.cpp'),
        (root/'upstream/moveit2/moveit_ros/moveit_servo','vendor/moveit_servo'),
        (root/'patches/moveit/0001-input-provenance.patch','provenance.patch'),
        (root/'upstream/moveit2/moveit_ros/moveit_servo/config/panda_simulated_config.yaml','servo_config.yaml')]
    with tarfile.open(output,'w:gz') as tar:
        for source,name in sources:
            for path in sorted(source.rglob('*')) if source.is_dir() else [source]:
                if not path.is_file():continue
                target=name+'/'+path.relative_to(source).as_posix() if source.is_dir() else name
                data=path.read_bytes()
                # Git checkout CRLF is normalized only in text build sources;
                # model/profile checksums and STL bytes remain unchanged.
                if target.startswith('vendor/') or target in ('provenance.patch','kdl_model_probe.cpp'):
                    if b'\0' not in data:data=data.replace(b'\r\n',b'\n')
                info=tarfile.TarInfo(target);info.size=len(data);info.mode=0o644
                tar.addfile(info,io.BytesIO(data))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    package(Path(__file__).resolve().parents[1],a.output)
