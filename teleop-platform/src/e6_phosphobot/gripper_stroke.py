"""Installed gripper's usable stroke, distinct from firmware position units.

v283: user observed full closure at raw 11; target 12 subsequently settled
at raw 12/state 0 with the original force 64. Raw 255 was reached unloaded.
This is an installation-specific soft endpoint, not an encoder recalibration
or a force-free/linear fingertip-distance calibration. Do not remap geometry.
"""
CLOSED_RAW = 12
OPEN_RAW = 255
SPAN = (OPEN_RAW-CLOSED_RAW)/255
REVISION = 'installed-soft-stroke-v283'


def opening(native):
    if native is None:return None
    return max(0.,min(1.,(native*255-CLOSED_RAW)/(OPEN_RAW-CLOSED_RAW)))


def metadata():
    return dict(revision=REVISION,closed_raw=CLOSED_RAW,open_raw=OPEN_RAW,
                minimum=0.,maximum=1.,units='usable_stroke_fraction',
                basis='observed_closed_soft_endpoint',tolerance_counts=1,
                geometry_units='raw_can_position',linear_jaw_distance=False)
