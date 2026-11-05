"""Read-only inspection of original events; never reconstruct actions from state."""
import bisect
from .episode_transfer import rows,association_policy,AvailableCameraFrames


def sample_page(directory,offset,limit):
    events=rows(directory/'audit/events.jsonl')
    cycles=sorted((r for r in events if r.get('event')=='cycle'),key=lambda r:r['at_ns'])
    media=sorted(rows(directory/'media/camera_samples.jsonl'),key=lambda r:r['x5_sample_ns'])
    times=[r['x5_sample_ns'] for r in media]
    manifest=directory/'media/manifest.jsonl'
    available=(AvailableCameraFrames(rows(manifest))
               if association_policy(directory)=='sdk_available_v2' and manifest.is_file() else None)
    inputs=sorted((r for r in events if r.get('event')=='input' and r.get('data',{}).get('op')=='intent'),key=lambda r:r['at_ns'])
    input_times=[r['at_ns'] for r in inputs]
    page=cycles[offset:offset+limit];selected=[];received=[]
    previous={}
    if available is not None and offset>0 and page:
        # Reuse refers to the preceding cycle, including across browser pages.
        previous={name:available.select(name,cycles[offset-1]['at_ns'])[1].get('device_frame_id')
                  for name in ('rgb','depth')}
    for row in page:
        index=bisect.bisect_right(times,row['at_ns'])-1
        source=media[index] if index>=0 else None
        if available is None:
            selected.append(dict(source or dict(sources={}),association_sample_ns=row['at_ns']))
        else:
            sources={}
            for name in ('rgb','depth'):
                when,src=available.select(name,row['at_ns'])
                fid=src.get('device_frame_id');received_ns=src.get('x5_receive_monotonic_ns')
                sources[name]=dict(src,device_frame_id=fid,sdk_available_ns=when,
                    cycle_valid=received_ns is not None and 0<=row['at_ns']-received_ns<=100_000_000,
                    cycle_reused=fid is not None and previous.get(name)==fid)
                previous[name]=fid
            selected.append(dict(sources=sources,association_sample_ns=row['at_ns'],
                                 selection_basis='sdk_available_v2',recorded_sample=source))
        index=bisect.bisect_right(input_times,row['at_ns'])-1
        received.append(inputs[index] if index>=0 else None)
    actions=rows(directory/'associations.jsonl')
    begin=page[0]['at_ns'] if page else 0
    # Include the final sample's tail and receipts. The former route ended at
    # the last cycle itself, silently hiding its sends and associations.
    end=(cycles[offset+limit]['at_ns'] if offset+limit<len(cycles) else
         max([r.get('at_ns',0) for r in events]+[r['action_sent_ns'] for r in actions]+[begin])+1) if page else 0
    event_names={'motion_submitted','motion_confirmed','motion_uncertain','stop_requested','stop_result','can','workspace_blocked'}
    return dict(total=len(cycles),cycles=page,media=selected,received_inputs=received,
                window=dict(begin_ns=begin,end_ns=end),
                events=[r for r in events if r.get('event') in event_names and begin<=r.get('at_ns',0)<end],
                associations=[r for r in actions if begin<=r['action_sent_ns']<end])
