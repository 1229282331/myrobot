"""Repeat official local readback checks without re-encoding or uploading."""
import argparse
import json
from pathlib import Path
from e6_teleop.exporter import load_episode, validate_dataset

parser=argparse.ArgumentParser(__doc__);parser.add_argument("dataset",type=Path)
args=parser.parse_args()
path=args.dataset.resolve();file=path/"simulation_validation.json"
report=json.loads(file.read_text(encoding="utf-8"))
report.update(validate_dataset(path,[load_episode(source) for source in report["raw_sources"]]))
file.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
print(json.dumps(report,ensure_ascii=False,indent=2))
