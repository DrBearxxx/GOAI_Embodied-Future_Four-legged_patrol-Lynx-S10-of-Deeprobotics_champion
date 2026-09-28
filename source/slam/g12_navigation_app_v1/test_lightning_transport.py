"""Transport callback units (not a ROS/DDS system test)."""
import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np

ROOT=Path(__file__).parent
spec=importlib.util.spec_from_file_location('airy_contract',ROOT/'lightning/airy_contract.py')
contract=importlib.util.module_from_spec(spec);spec.loader.exec_module(contract)


def method(name):
    tree=ast.parse((ROOT/'lightning/bridge.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Bridge')
    node=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
    module=ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[]))
    scope={'np':np,'stamp':lambda m:m.native_stamp}
    exec(compile(module,'bridge.py','exec'),scope)
    return scope[name]


class TransportTests(unittest.TestCase):
    def test_single_estimator_is_sufficient(self):
        state=NS(cloud_pub=NS(get_subscription_count=lambda:1),imu_pub=NS(get_subscription_count=lambda:1))
        self.assertTrue(method('ready')(state))

    def test_high_dynamic_imu_is_passed_unchanged(self):
        from collections import Counter
        published=[]
        m=NS(native_stamp=1_800_000_000_000_000_000,
             angular_velocity=NS(x=8.,y=-6.,z=10.),linear_acceleration=NS(x=60.,y=-30.,z=25.))
        state=NS(counts=Counter(),ready=lambda:True,ordered=lambda *args:True,imu_pub=NS(publish=published.append))
        method('imu')(state,m)
        self.assertIs(published[0],m)
        self.assertEqual(state.counts['imu_forwarded'],1)

    def test_native_clock_offset_is_not_a_gate(self):
        from collections import Counter
        state=NS(last_stamp={},counts=Counter())
        ordered=method('ordered')
        for i in range(200):
            self.assertTrue(ordered(state,'imu',1_234_567_890_000_000_000+i*5_000_000))
        self.assertFalse(ordered(state,'imu',state.last_stamp['imu']))
        with self.assertRaisesRegex(RuntimeError,'SENSOR_CLOCK_RESTART'):
            ordered(state,'imu',20_000_000)

    def test_padded_rows_big_endian_and_point_time_contract(self):
        dtype=np.dtype({'names':['x','y','z','intensity','ring','time'],
            'formats':['>f4','>f4','>f4','>f4','>u2','>f8'],
            'offsets':[0,4,8,12,16,24],'itemsize':32})
        raw=bytearray(144)
        arr=np.ndarray((2,2),dtype=dtype,buffer=raw,strides=(72,32))
        for name in ('x','y','z','intensity'):arr[name]=1
        arr['ring']=[[0,1],[2,3]]
        arr['time']=[[.09,.01],[.05,.08]]
        fields=[NS(name=n,datatype=k,count=1,offset=o) for n,k,o in
            [('x',7,0),('y',7,4),('z',7,8),('intensity',7,12),('ring',4,16),('time',8,24)]]
        msg=NS(fields=fields,is_bigendian=True,point_step=32,width=2,height=2,row_step=72,data=raw)
        normalized,stats=contract.normalize_points(msg)
        self.assertEqual(normalized.dtype.itemsize,24)
        np.testing.assert_allclose(normalized['time'],[.01,.05,.08,.09],atol=1e-8)
        np.testing.assert_array_equal(normalized['ring'],[1,2,3,0])
        self.assertEqual(stats['invalid_points'],0)


if __name__=='__main__':unittest.main()
