"""Failure-preserving lifecycle helpers for isolated software qualification.

This module owns no device connections or process groups. Callers supply only
cleanup operations for resources they created. Every failed step is retained;
subsequent steps and evidence collection still run.
"""
import json
import time
import traceback
from pathlib import Path


class Cleanup:
    def __init__(self,path):
        self.path=Path(path)
        self.steps=[]

    def run(self,name,operation):
        row=dict(step=name,start_ns=time.perf_counter_ns(),error=None)
        try:operation()
        except Exception:row['error']=traceback.format_exc()
        row['end_ns']=time.perf_counter_ns()
        self.steps.append(row)
        # A broken evidence volume must not prevent the next resource release.
        try:self.path.write_text(json.dumps(self.steps,indent=2),encoding='utf8')
        except Exception:
            row['persistence_error']=traceback.format_exc()
        return row['error'] is None and 'persistence_error' not in row

    @property
    def failures(self):
        return [row for row in self.steps if row['error'] or row.get('persistence_error')]
