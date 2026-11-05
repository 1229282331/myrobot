"""Bounded operator state; full solver provenance stays in the X5 audit.

Sending trajectory windows twice at 20 Hz needlessly competes with intents on
Wi-Fi. The controller gets current actual state and receipts at 20 Hz; the
independent diagnostic observer retains full snapshots at 5 Hz.
"""

def select(value, keys):
    return {key: value[key] for key in keys if key in value} if value is not None else None


def for_operator(state):
    result = select(state, (
        'kind', 'state', 'ready', 'reason', 'input_paused', 'direction_limited', 'arm', 'echo_ns', 'actual', 'camera',
        'gripper', 'receipt', 'stage', 'pending', 'stop_acknowledged', 'stationary',
        'stopping', 'stop_uncertain', 'recording', 'capabilities', 'camera_restart'))
    result['detail_level'] = 'control'
    target_keys = ('kind', 'position_m', 'quaternion_xyzw', 'joints_rad',
                   'input_sequence', 'pre_action_received_ns', 'receipt', 'pose_frame', 'flange_pose', 'tcp')
    result['candidate'] = select(state.get('candidate'), target_keys)
    result['accepted'] = select(state.get('accepted'), target_keys)
    result['model'] = select(state.get('model'), ('ready', 'phase', 'error', 'status'))
    feedback = state.get('feedback')
    result['feedback'] = select(feedback, ('connected', 'age_ms', 'received_ns',
                                         'generation', 'sequence', 'error'))
    if feedback is not None:
        result['feedback']['status'] = select(feedback.get('status'), (
            'robot_mode', 'enable_status', 'error_status', 'collision_state',
            'run_queued_cmd', 'current_command_id', 'user_coordinate', 'tool_coordinate'))
    return result
