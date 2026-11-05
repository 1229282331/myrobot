"""Build one sample in a worker. No SDK handles, publishing, or disk IO."""
import hashlib


def build_sample(meta,original,profile,decode_color,decode_depth):
    import cv2
    import numpy as np
    metadata=dict(meta)
    metadata.pop('_original_bytes',None)
    stream=metadata['stream'];width=metadata['width'];height=metadata['height']
    name='rgb' if stream=='color' else 'depth' if stream=='depth' else None
    if name is None:raise ValueError('Only native RGB/depth samples supported')
    value=dict(source='Gemini E:'+stream,device_frame_id=metadata['index'],
               x5_receive_monotonic_ns=metadata['host_perf_counter_ns'],metadata=metadata,
               width=width,height=height)
    if name=='rgb':
        image=decode_color(original,width,height,metadata['format_id'])
        value['jpeg']=original
        small=cv2.resize(image,tuple(profile['rgb']),interpolation=cv2.INTER_AREA)
    else:
        scale=metadata['depth_scale_mm']
        raw,mm=decode_depth(original,width,height,metadata['format_id'],scale)
        value.update(depth_scale_mm=scale,uint16_raw=original,valid_fraction=float(np.mean(raw>0)),
                     data_valid=bool(np.any(raw>0)))
        gray=np.clip(mm/1500*255,0,255).astype('uint8')
        small=cv2.resize(gray,tuple(profile['depth']),interpolation=cv2.INTER_NEAREST)
    ok,jpeg=cv2.imencode('.jpg',small,[cv2.IMWRITE_JPEG_QUALITY,profile['jpeg_quality']])
    if not ok:raise ValueError('Preview JPEG encoding failed')
    value.update(preview_jpeg=jpeg.tobytes(),preview_width=small.shape[1],preview_height=small.shape[0],
                 original_sha256=hashlib.sha256(original).hexdigest(),original_bytes=len(original))
    return value
