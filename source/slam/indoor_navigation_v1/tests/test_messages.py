import sys,unittest
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from indoor.messages import convert
class MessageTests(unittest.TestCase):
    def make(self,order='<',padded=False):
        width,height=1000,100
        row=width*16+(16 if padded else 0);buffer=bytearray(row*height)
        for i,k in enumerate(['x','y','z','time']):
            a=np.ndarray((height,width),dtype=order+'f4',buffer=buffer,offset=i*4,strides=(row,16))
            a[:]=np.arange(width*height).reshape(height,width)*(.1/(width*height-1) if k=='time' else .001)
        return S(header=S(stamp=S(sec=10,nanosec=0),frame_id='wym_front_lidar'),fields=[S(name=k,datatype=7,count=1,offset=i*4) for i,k in enumerate(['x','y','z','time'])],
            is_bigendian=order=='>',height=height,width=width,row_step=row,point_step=16,data=bytes(buffer))
    def test_cap_and_full_scan_bounds(self):
        for order in ['<','>']:
            for padded in [False,True]:
                e=convert('/wym/slam/front/points',self.make(order,padded),1.)
                self.assertLessEqual(len(e['points']),5000);self.assertAlmostEqual(e['rel'].min(),0.);self.assertAlmostEqual(e['rel'].max(),.1)
                self.assertTrue(np.isfinite(e['points']).all())
    def test_malformed_buffer_rejected(self):
        m=self.make();m.data=m.data[:-1]
        with self.assertRaises(ValueError):convert('/wym/slam/front/points',m,1.)
if __name__=='__main__':unittest.main(verbosity=2)
