"""Conservative release qualification from original processing evidence."""


def classify_release(release_ns, stop, early_stops, processed, boundaries=()):
    if early_stops:
        return False, 'stop_preceded_release'
    if stop is None:
        return False, 'no_stop_observed'
    # A workspace rejection can Stop inside the control mutex after socket
    # receipt, then normal release processing clears its reason. Keep that
    # causal boundary event even when the later snapshot is ready/reason-free.
    if any(release_ns<=event['at_ns']<=stop['at_ns'] for event in boundaries):
        return False, 'workspace_boundary_before_stop'
    # A fault inside a locked tick can send Stop after the socket receives a
    # release but before that release can be applied. Timestamp ordering alone
    # must not relabel such a fault as a normal operator release.
    evidence = processed.get(release_ns)
    if evidence is None:
        return False, 'release_processing_unobserved'
    if evidence.get('ready') is not True or evidence.get('reason'):
        return False, evidence.get('reason') or 'session_not_ready_at_release'
    return True, None
