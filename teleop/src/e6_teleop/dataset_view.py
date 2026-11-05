"""On-demand official LeRobot/PyAV reading; called in the PC HTTP worker thread."""
from functools import lru_cache
from io import BytesIO
import base64
import threading
import numpy as np
from PIL import Image

LOCK = threading.Lock()


@lru_cache(maxsize=4)
def dataset(path):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    return LeRobotDataset("local/e6_simulation", root=path, video_backend="pyav")


def read_frame(path, index):
    with LOCK:
        ds = dataset(str(path))
        if not 0 <= index < len(ds): raise ValueError("Dataset frame index out of range")
        item = ds[index]
        pixels = np.round(item["observation.images.wrist"].permute(1,2,0).numpy()*255).clip(0,255).astype(np.uint8)
        stream = BytesIO(); Image.fromarray(pixels).save(stream, "JPEG", quality=90)
        state, action = item["observation.state"].tolist(), item["action"].tolist()
        return dict(index=index, total=len(ds), episode_index=int(item["episode_index"]), frame_index=int(item["frame_index"]),
                    timestamp_s=float(item["timestamp"]), task=item["task"], state=state, action=action,
                    image_jpeg=base64.b64encode(stream.getvalue()).decode(), reader="official LeRobot Dataset v3 / PyAV",
                    simulated=True)
