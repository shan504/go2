"""Pack dense XYZ clouds without constructing millions of Python point lists."""
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def xyz_message(header, points):
    xyz = np.ascontiguousarray(points,dtype='<f4').reshape(-1,3)
    fields = [PointField(name=name,offset=index*4,datatype=PointField.FLOAT32,count=1)
              for index,name in enumerate(('x','y','z'))]
    return PointCloud2(header=header,height=1,width=len(xyz),fields=fields,
        is_bigendian=False,point_step=12,row_step=len(xyz)*12,
        is_dense=bool(np.isfinite(xyz).all()),data=xyz.tobytes())
