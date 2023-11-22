# import rospy
# import numpy as np
# import open3d as o3d
# import ros_numpy
# import struct
# from sensor_msgs.msg import PointCloud2

# def bin_to_pcd(binFileName):
#     size_float = 4
#     list_pcd = []
#     with open(binFileName, "rb") as f:
#         byte = f.read(size_float * 4)
#         while byte:
#             x, y, z, intensity = struct.unpack("ffff", byte)
#             list_pcd.append([x, y, z])
#             byte = f.read(size_float * 4)
#     np_pcd = np.asarray(list_pcd)
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(np_pcd)
#     return pcd

# def callback(data):
#     rospy.loginfo('Received a PointCloud2 message')
    
#     pc = ros_numpy.numpify(data)

#     print(pc.shape)
#     points=np.zeros((pc.shape[0],pc.shape[1],4))
  
#     points[:,:,0]=pc['x']
#     points[:,:,1]=pc['y']
#     points[:,:,2]=pc['z']
#     points[:,:,3]=pc['intensity']

#     # Add to
#     p = np.array(points, dtype=np.float32)

#     global pc_number
#     pc_number += 1

#     # Save files
#     bin_file = f'/home/nvidia/BEVfusion/data/nuscenes/samples/LIDAR_TOP/LIDAR_TOP.bin'       # TODO: Replace with your desired folder for .bin files
#     pcd_file = f'/home/nvidia/BEVfusion/data/nuscenes/samples/LIDAR_TOP/LIDAR_TOP.pcd' # TODO: Replace with your desired folder for .pcd files

#     p.astype('float32').tofile(bin_file)
#     pcd = bin_to_pcd(bin_file)

#     o3d.io.write_point_cloud(pcd_file, pcd)

# def listener():
#     global pub
#     global pc_number
    
#     # Init pc number
#     pc_number = 0
    
#     rospy.init_node('pointcloud_to_bin_file', anonymous=True)

#     # Subscribe to the input PointCloud2 topic
#     input_cloud = 'ouster/points' # TODO: Change this to your pc topic
#     rospy.Subscriber(input_cloud, PointCloud2, callback)

#     # Create a publisher for the output PointCloud2 topic
#     pub = rospy.Publisher('/output_cloud', PointCloud2, queue_size=10)

#     rospy.spin()

# if __name__ == '__main__':
#     listener()


import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
from pypcd import pypcd
# import ros_numpy
import struct
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import PointField
import ctypes
import roslib.message
import math
import sys
# from collections.abc import Sequence

# def bin_to_pcd(binFileName):
#     size_float = 4
#     list_pcd = []
#     with open(binFileName, "rb") as f:
#         byte = f.read(size_float * 4)
#         while byte:
#             x, y, z, intensity = struct.unpack("ffff", byte)
#             list_pcd.append([x, y, z])
#             byte = f.read(size_float * 4)
#     np_pcd = np.asarray(list_pcd)
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(np_pcd)
#     return pcd

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    assert isinstance(cloud, pc2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step

class PointCloudToPCD(Node):

    def __init__(self):
        super().__init__('pointcloud_to_pcd')

        # self.publisher = self.create_publisher(PointCloud2, 'output_topic', 10)
        self.subscription = self.create_subscription(
            pc2,
            '/lidar_top',
            self.callback,
            10)

    def callback(self, ros_point_cloud):
        self.get_logger().info("Callback is started.")
        xyz = np.array([[0,0,0]])
        rgb = np.array([[0,0,0]])
        #self.lock.acquire()
        # gen = pc2.read_points(ros_point_cloud, skip_nans=True)
        gen = read_points(ros_point_cloud, skip_nans=True)
        int_data = list(gen)

        for x in int_data:
            test = x[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0)>> 16
            g = (pack & 0x0FF00)>> 8
            b = (pack & 0xFF)
            # prints r,g,b values in the 0-255 range
                        # x,y,z can be retrieved from the x[0],x[1],x[2]
            xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
            rgb = np.append(rgb,[[r,g,b]], axis = 0)

        out_pcd = o3d.geometry.PointCloud()    
        out_pcd.points = o3d.utility.Vector3dVector(xyz)
        out_pcd.colors = o3d.utility.Vector3dVector(rgb)
        o3d.io.write_point_cloud("/home/nvidia/BEVfusion/ROS/LIDAR_TOP.pcd",out_pcd)
        self.get_logger().info("PCD is saved.")

if __name__ == "__main__":
    rclpy.init()
    node = PointCloudToPCD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()