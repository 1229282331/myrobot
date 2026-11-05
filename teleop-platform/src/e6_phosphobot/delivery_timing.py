"""Pinned delivery slack, separate from live feedback and integration time.

One outstanding endpoint may wait for its writer slot; it is never retimed,
extrapolated or used to advance the solver without a physical write receipt.
"""

PROFILE = dict(version=1, source_lifetime_ms=150, stream_timeout_ms=200)


def validate_delivery(value):
    if (not isinstance(value, dict) or set(value) != set(PROFILE) or
            any(type(value[k]) is not int for k in PROFILE) or value != PROFILE):
        raise ValueError('Unsupported candidate delivery timing')
    return dict(value)


def configure_delivery(manifest, writer_paced):
    value = manifest.get('delivery_timing')
    if value is None:
        return None
    if not writer_paced or not manifest.get('solver_timing'):
        raise ValueError('Delivery timing requires pinned solver timing and writer pacing')
    return validate_delivery(value)


def source_lifetime_ns(delivery):
    return (validate_delivery(delivery)['source_lifetime_ms'] if delivery is not None else 100) * 1_000_000


def candidate_lifetime_ns(row):
    delivery = row.get('cpp', {}).get('delivery_timing')
    if delivery is not None:
        if (row.get('dispatch') or {}).get('version') != 1:
            raise ValueError('Delivery timing requires a writer-paced candidate')
        stamps = (row.get('received_ns'), row.get('pre_action_received_ns'),
                  (row.get('source_input') or {}).get('received_ns'))
        if any(type(stamp) is not int or stamp < 0 for stamp in stamps):
            raise ValueError('Delivery timing requires original source timestamps')
    return source_lifetime_ns(delivery)


def candidate_deadline_ns(row):
    limit = candidate_lifetime_ns(row)
    deadline = min(row.get('valid_until_ns', row['received_ns'] + limit),
                   row['received_ns'] + limit)
    # Keep both original sources bound even if an IPC producer accidentally
    # supplies a deadline based on computation completion or publication.
    if 'pre_action_received_ns' in row:
        deadline = min(deadline, row['pre_action_received_ns'] + limit)
    source = row.get('source_input') or {}
    if 'received_ns' in source:
        deadline = min(deadline, source['received_ns'] + 250_000_000)
    return deadline


def stream_timeout_ns(info):
    delivery = info.get('delivery_timing')
    if delivery is not None:
        if info.get('writer_pacing_version') != 1:
            raise ValueError('Delivery timing requires writer pacing')
        return validate_delivery(delivery)['stream_timeout_ms'] * 1_000_000
    return 150_000_000
