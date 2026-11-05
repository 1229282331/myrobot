"""Optional session-scoped Wi-Fi latency setting with explicit restoration."""
import subprocess
from pathlib import Path


class WifiLatency:
    def __init__(self,enabled,reorder_ms=None):
        if reorder_ms is not None and (type(reorder_ms) is not int or not 5<=reorder_ms<=50):raise ValueError('Reorder diagnostic must be between 5 and 50 ms')
        self.enabled=enabled;self.changed=False
        self.reorder_ms=reorder_ms;self.reorder_changed=False
        self.reorder_path=Path('/sys/module/aic8800_fdrv/parameters/reorder_timeout')
        self.driver_ps_path=Path('/sys/module/aic8800_fdrv/parameters/ps_on')
        self.report=dict(requested=enabled,changed=False,original=None,reorder_requested_ms=reorder_ms,
                         setting_scope='cfg80211 iw request; optional temporary AIC8800 wlan0 reorder parameter',firmware_power_save_verified=False)

    def command(self,*args):
        return subprocess.run(['iw','dev','wlan0',*args],capture_output=True,text=True,timeout=3,check=True)

    def __enter__(self):
        try:
            # On the installed AIC module, cfg80211's power-mgmt callback is
            # a success-returning stub. Keep its API state separate from the
            # module parameter supplied to firmware initialization.
            self.report['driver_ps_on']=self.driver_ps_path.read_text().strip() if self.driver_ps_path.exists() else None
            if self.report['driver_ps_on'] not in (None,'Y','N'):raise ValueError('Unknown driver PS module parameter')
            self.report['driver_ps_semantics']='module initialization parameter; not independent firmware telemetry'
            if self.enabled:
                state=self.command('get','power_save').stdout.strip()
                if state not in ('Power save: on','Power save: off'):raise ValueError('Unknown wlan0 power-save status')
                self.report['original']=state.split(': ')[1]
                if self.report['original']=='on':
                    self.command('set','power_save','off');self.changed=True;self.report['changed']=True
            if self.reorder_ms is not None:
                original=self.reorder_path.read_text().strip()
                if not original.isdigit() or not 5<=int(original)<=1000:raise ValueError('Unrecognized AIC8800 reorder setting')
                self.report['reorder_original_ms']=int(original)
                self.reorder_path.write_text(str(self.reorder_ms)+'\n')
                self.reorder_changed=True
                if int(self.reorder_path.read_text())!=self.reorder_ms:raise ValueError('Reorder setting did not apply')
                self.report['reorder_effective_ms']=int(self.reorder_path.read_text())
        except BaseException:
            self.__exit__();raise
        return self

    def __exit__(self,*args):
        try:
            if self.reorder_changed:
                if int(self.reorder_path.read_text())!=self.reorder_ms:raise ValueError('Reorder setting changed outside this session; refuse to overwrite it')
                self.reorder_path.write_text(str(self.report['reorder_original_ms'])+'\n')
                if int(self.reorder_path.read_text())!=self.report['reorder_original_ms']:raise ValueError('Reorder restoration failed')
                self.reorder_changed=False
                print('Session AIC8800 reorder setting restored',flush=True)
        finally:
            if self.changed:
                self.command('set','power_save','on');self.changed=False
                print('Session Wi-Fi power save restored to on',flush=True)
