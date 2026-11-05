"""Versioned timing contract shared by the pinned native worker and its caller."""

FIELDS=('version','period_ms','callback_budget_ms','request_timeout_ms',
        'retreat_budget_ms','geometry_budget_ms')
PARAMETERS={key:'e6_'+key for key in FIELDS if key not in ('version','period_ms')}


def validate_timing(value):
    if (not isinstance(value,dict) or set(value)!=set(FIELDS) or
            any(type(value[k]) is not int for k in FIELDS)):
        raise ValueError('Incomplete or non-integer solver timing contract')
    if value['version']!=1 or value['period_ms']!=50:
        raise ValueError('Unsupported solver timing version or integration period')
    if not (0 < value['geometry_budget_ms'] < value['retreat_budget_ms'] <
            value['callback_budget_ms'] < value['request_timeout_ms'] <= 100):
        raise ValueError('Invalid solver timing order or source lifetime')
    return dict(value)


def configure_timing(params,manifest):
    """Only a pinned binary manifest selects non-legacy timing."""
    timing=manifest.get('solver_timing')
    if any(key in params for key in PARAMETERS.values()):
        raise ValueError('Solver timing must come from the pinned native manifest')
    if timing is None:
        return None
    timing=validate_timing(timing)
    if (params['moveit_servo']['publish_period']!=.05 or params['update_period']!=.05):
        raise ValueError('Solver timing does not match integration configuration')
    params.update({parameter:timing[key] for key,parameter in PARAMETERS.items()})
    return timing


def cycle_options(timing):
    if timing is None:return {}
    timing=validate_timing(timing)
    return {key:timing[key] for key in ('callback_budget_ms','request_timeout_ms')}


def verify_timing_reply(reply,timing):
    actual=reply.get('solver_timing')
    if timing is None:
        if actual is not None:raise ValueError('Unattested native timing reply')
    elif validate_timing(actual)!=timing:
        raise ValueError('Native timing reply differs from the pinned contract')
