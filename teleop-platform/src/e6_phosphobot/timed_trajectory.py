"""Sample an upstream timed trajectory, without extrapolation or optimization.

All timestamps here belong to one X5 ROS clock. The caller maps the input
stamps to its retained input records; this module does not own a control lease.
"""
from bisect import bisect_left
from dataclasses import dataclass
import math
import numpy as np
from scipy.interpolate import CubicHermiteSpline


@dataclass(frozen=True)
class PointSource:
    point_ns: int
    kind: str
    input_ns: int


def parse_provenance(value):
    parts=value.split(';')
    if parts[0]!='e6_trace_v1' or not 3<=len(parts)<=130:
        raise ValueError('Missing or oversized trajectory provenance')
    sources=[]
    for part in parts[1:]:
        fields=part.split(',')
        if len(fields)!=3:raise ValueError('Malformed trajectory provenance')
        point,kind,stamp=fields
        if not point.isdecimal() or not stamp.isdecimal() or kind not in ('j','t','p','h','s'):
            raise ValueError('Invalid trajectory source')
        source=PointSource(int(point),kind,int(stamp))
        if source.point_ns<=0 or (source.input_ns==0)!=(kind=='s'):
            raise ValueError('Invalid source timestamp')
        if sources and source.point_ns<=sources[-1].point_ns:
            raise ValueError('Reordered or repeated point timestamp')
        sources.append(source)
    return tuple(sources)


class TimedTrajectory:
    def __init__(self,times_ns,positions,velocities,provenance):
        self.sources=parse_provenance(provenance)
        self.times=tuple(times_ns)
        q=np.asarray(positions,dtype=float);v=np.asarray(velocities,dtype=float)
        if len(self.times)<2 or len(self.times)>128 or q.shape!=(len(self.times),6) or v.shape!=q.shape:
            raise ValueError('Invalid six-joint trajectory shape')
        if not all(isinstance(t,int) and t>0 for t in self.times) or any(b<=a for a,b in zip(self.times,self.times[1:])):
            raise ValueError('Invalid trajectory timestamps')
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(v)):
            raise ValueError('Nonfinite trajectory')
        # Upstream retains one extra future point to set the preceding velocity.
        if len(self.sources)!=len(self.times)+1 or tuple(s.point_ns for s in self.sources[:-1])!=self.times:
            raise ValueError('Trajectory/provenance point mismatch')
        if self.sources[-1].point_ns-self.times[0]>1_000_000_000:
            raise ValueError('Trajectory window unexpectedly long')
        self.q=q;self.v=v
        self.curve=CubicHermiteSpline(np.array([t-self.times[0] for t in self.times])/1e9,q,v,axis=0,extrapolate=False)

    def latest_complete(self):
        """Newest published endpoint for immediate position consumers.

        The upstream final source is intentionally omitted from the position
        array until its velocity is finalized. It is retained for provenance,
        but must not be selected as an executable point.
        """
        return dict(joints_rad=self.q[-1].tolist(),sample_time_ns=self.times[-1],
                    bracket_times_ns=[self.times[-2],self.times[-1]],
                    bracket_sources=list(self.sources[-3:-1]),window_sources=self.sources)

    def sample(self,at_ns):
        if not isinstance(at_ns,int) or not self.times[0]<=at_ns<=self.times[-1]:return None
        upper=bisect_left(self.times,at_ns)
        lower=max(0,upper-1)
        values=self.curve((at_ns-self.times[0])/1e9).tolist()
        if not all(math.isfinite(x) for x in values):raise ValueError('Interpolation returned nonfinite joints')
        # Positions in these brackets came from these exact upstream input
        # stamps. Retain the whole window too: velocities and smoothing can
        # depend on neighboring/prior inputs, not only the upper bracket.
        return dict(joints_rad=values,sample_time_ns=at_ns,
                    bracket_times_ns=[self.times[lower],self.times[upper]],
                    bracket_sources=[self.sources[lower],self.sources[upper]],
                    window_sources=self.sources)
