"""Install the isolated, pinned PC launcher and verified MediaMTX artifact."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import urllib.request
import zipfile

root = Path(__file__).resolve().parents[1]
uv = shutil.which('uv')
if not uv:
    raise SystemExit('Install uv first, then rerun this entry point.')
python = root/'.venv-launcher/Scripts/python.exe'
if not python.exists():
    subprocess.run([uv, 'venv', str(root/'.venv-launcher'), '--python', '3.12'], check=True)
subprocess.run([uv, 'pip', 'sync', '--require-hashes', '--python', str(python),
                str(root/'locks/launcher.txt')], check=True)
config = json.loads((root/'config/workstation.json').read_text())
lock = json.loads((root/'upstream.lock.json').read_text())['mediamtx']
binary = root/'runtime/mediamtx/mediamtx.exe'
if binary.exists():
    if hashlib.sha256(binary.read_bytes()).hexdigest() != config['mediamtx_executable_sha256']:
        raise SystemExit('Existing MediaMTX differs; retained for inspection. No overwrite performed.')
else:
    binary.parent.mkdir(parents=True, exist_ok=True)
    archive = binary.parent/('mediamtx_'+lock['version']+'_windows_amd64.zip')
    if not archive.exists():
        url = lock['url']+'/releases/download/'+lock['version']+'/'+archive.name
        urllib.request.urlretrieve(url, archive)
    if hashlib.sha256(archive.read_bytes()).hexdigest() != lock['windows_amd64_sha256']:
        raise SystemExit('Downloaded archive checksum differs; original retained.')
    with zipfile.ZipFile(archive) as package:
        data = package.read('mediamtx.exe')
    if hashlib.sha256(data).hexdigest() != config['mediamtx_executable_sha256']:
        raise SystemExit('MediaMTX executable checksum differs; archive retained.')
    binary.write_bytes(data)
print('Launcher dependencies and MediaMTX verified. Build the full app with scripts/setup.py if needed.')
