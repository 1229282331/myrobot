from types import SimpleNamespace
import pytest
from e6_phosphobot.camera_probe import ClockWitness


def test_clock_witness_does_not_replace_rejected_timestamp_or_mask_error():
    def add(value):raise ValueError('duplicate')
    original=SimpleNamespace(last=123,hz=15_000_000,extended=123,wraps=0,add=add)
    witness=ClockWitness(original)
    with pytest.raises(ValueError,match='duplicate'):witness.add(123)
    assert original.last==123
    assert witness.rows[-1]['previous']==123 and witness.rows[-1]['pts']==123
    assert witness.rows[-1]['error']=='duplicate'


def test_clock_witness_returns_exact_upstream_result():
    result={'uvc_pts_ns':999}
    original=SimpleNamespace(last=123,hz=15_000_000,extended=123,wraps=0,add=lambda _:result)
    assert ClockWitness(original).add(124) is result
