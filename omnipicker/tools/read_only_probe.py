"""Read the native USB identity and runtime configuration, never write endpoints."""
import json
from datetime import datetime
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from studio.device import NativeDevice, ROOT
from studio.protocol import crc16
import hashlib

if __name__ == '__main__':
    if Path(sys.prefix).name != 'omnipicker': raise SystemExit('Use the omnipicker Conda environment')
    d=NativeDevice()
    try:
        d.open()
        result=dict(time=datetime.now().astimezone().isoformat(),usb_serial=d.serial,
            serial=f'{d.internal_serial:012X}',firmware=d.firmware,compatible=d.compatible,
            device_type=d.read('device_type'),can_id=d.can_id,fault=d.fault(),
            ctrl_mode=d.read('motor.config.ctrl_mode'),calib_valid=d.read('motor.config.calib_valid'),
            limits=d.limits(),schema_sha256=hashlib.sha256(d.raw_schema).hexdigest(),
            schema_crc16=crc16(d.raw_schema),last_rtt_ms=d.rtt_ms,writes_performed=False)
        print(json.dumps(result,ensure_ascii=False,indent=2))
        (ROOT/'logs').mkdir(exist_ok=True)
        (ROOT/'logs'/('readonly-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    finally: d.close()
