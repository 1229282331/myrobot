import sys,unittest,queue,threading,time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from rdk_healthcheck import decode_item,ordered_frames

class OrderedProcessing(unittest.TestCase):
    def frames(self, corrupt=False):
        q=queue.Queue()
        for i in range(12):
            m={'stream':'color','index':i,'width':2,'height':2,'format_id':23,'host_perf_counter_ns':i*1234}
            q.put((m,b'bad' if corrupt and i==5 else np.full((2,2,3),i,np.uint8).tobytes()))
        done=threading.Event();done.set()
        return q,done

    def test_out_of_order_completion_preserves_source_frame_pair(self):
        q,done=self.frames();r={'decode_pending_max':0}
        def slow_first(m,b,decode):
            if m['index']==0:time.sleep(.03)
            return decode_item(m,b,decode)
        with ThreadPoolExecutor(max_workers=2) as pool:
            frames=list(ordered_frames(q,done,pool,2,r,1,slow_first))
        self.assertEqual([m['index'] for m,_,_,_ in frames],list(range(12)))
        for m,im,_,_ in frames:
            self.assertTrue(np.all(im==m['index']))
            self.assertEqual(m['host_perf_counter_ns'],m['index']*1234)
        self.assertLessEqual(r['decode_pending_max'],4)

    def test_corrupt_frame_fails_instead_of_being_silently_skipped(self):
        q,done=self.frames(corrupt=True);r={'decode_pending_max':0}
        with ThreadPoolExecutor(max_workers=2) as pool:
            with self.assertRaises(ValueError):list(ordered_frames(q,done,pool,2,r,1))

    def test_sampled_decoding_is_explicit_without_losing_metadata(self):
        q,done=self.frames();r={'decode_pending_max':0}
        with ThreadPoolExecutor(max_workers=2) as pool:
            frames=list(ordered_frames(q,done,pool,2,r,3))
        self.assertEqual(len(frames),12)
        self.assertEqual([m['index'] for m,im,_,_ in frames if im is not None],[0,3,6,9])

if __name__=='__main__':unittest.main()
