import rclpy
import sys
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import struct
import message_filters
from sensor_msgs.msg import Image, PointCloud2, PointField
from yolov8_msgs.msg import BoundingBox2D, BoundingBoxCenter, BoundingBoxCenterArray, DetectionArray, Detection
from cluster_msgs.msg import ClusterArray, Cluster
from rclpy.node import Node
from cv_bridge import CvBridge
import math

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

# bridge = CvBridge() 

class PROJmodule(Node) :
    def __init__(self) :
        super().__init__('projection_module')
        self.cv_bridge = CvBridge()

        #<------------------Calibration file load------------------>
        #P2 -->calib_cam_to_cam: P_rect_02
        #R0_rect -->calib_cam_to_cam: R_rect_00
        #Tr_velo_to_cam -->calib_imu_to_velo: R
        calib_path='/home/hwi/lidar_ws/src/yolov8_ros/yolov8_ros/yolov8_ros/0000000123.txt'
        with open(calib_path,'r') as f:
            calib = f.readlines()

        self.P2 = np.array([float(x) for x in calib[0].strip('\n').split(' ')[1:]]).reshape(3,4)
        self.R0_rect = np.array([float(x) for x in calib[1].strip('\n').split(' ')[1:]]).reshape(3,3)
        # Add a 1 in bottom-right, reshape to 4 x 4
        self.R0_rect = np.insert(self.R0_rect,3,values=[0,0,0],axis=0)
        self.R0_rect = np.insert(self.R0_rect,3,values=[0,0,0,1],axis=1)
        self.Tr_velo_to_cam = np.array([float(x) for x in calib[2].strip('\n').split(' ')[1:]]).reshape(3,4)
        self.Tr_velo_to_cam = np.insert(self.Tr_velo_to_cam,3,values=[0,0,0,1],axis=0)

       #<------------------------Subscriber------------------------>
        self.camera_sub_ = message_filters.Subscriber(self, Image, "/kitti/camera_color_left/image_raw")
        self.lidar_sub_ = message_filters.Subscriber(self, PointCloud2, "/kitti/velo/pointcloud")
        self.camera_center2d_sub_ = message_filters.Subscriber(self, BoundingBoxCenterArray, "/yolo/bbox_center2d_arr")
        self.lidar_center2d_sub_ = message_filters.Subscriber(self, ClusterArray, "/lidar_detection/center2d_arr")
        # self.bbox_sub_ = message_filters.Subscriber(self, DetectionArray, "/yolo/detections")
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
           [self.camera_sub_, 
            self.lidar_sub_, 
            self.camera_center2d_sub_,
            self.lidar_center2d_sub_],
            # self.bbox_sub_], 
            1,
            .2, 
            allow_headerless=True
            )
        self.ts.registerCallback(self.proj_callback)
        self.get_logger().info("PROJmodule is started")

    def _get_struct_fmt(self, is_bigendian, fields, field_names=None):
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

    def read_points(self, cloud, field_names=None, skip_nans=False, uvs=[]):
        assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
        fmt = self._get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
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

    # https://github.com/azureology/kitti-velo2cam
    # Z[u v 1]T = P2 * R0_rect * Tr_velo_to_cam * [x y z 1]T  -------->Equation
    def projection(self, img_msg, point_msg) :
        print('PROJECTION is started')
        P2=self.P2
        R0_rect=self.R0_rect
        Tr_velo_to_cam=self.Tr_velo_to_cam

        #<------------------pointcloud2_msg to array------------------>
        xyz = np.array([0,0,0])
        points = []
        gen = self.read_points(point_msg, skip_nans=True)
        int_data = list(gen)

        for x in int_data:
            xyz = [x[0],x[1],x[2]]
            points.append(xyz)
        #<-------------------------projection------------------------->
        points=np.array(points)
        # TODO: use fov filter? 
        velo = np.insert(points,3,1,axis=1).T
        velo = np.delete(velo,np.where(velo[0,:]<0),axis=1)
        cam = P2.dot(R0_rect.dot(Tr_velo_to_cam.dot(velo)))
        cam = np.delete(cam,np.where(cam[2,:]<0),axis=1)
        cam[:2] /= cam[2,:]
        plt.figure(figsize=(12,5),dpi=96,tight_layout=True)
        png = self.cv_bridge.imgmsg_to_cv2(img_msg)
        IMG_H,IMG_W,_ = png.shape
        plt.axis([0,IMG_W,IMG_H,0])
        u,v,z = cam
        u_out = np.logical_or(u<0, u>IMG_W)
        v_out = np.logical_or(v<0, v>IMG_H)
        outlier = np.logical_or(u_out, v_out)
        cam = np.delete(cam,np.where(outlier),axis=1)
        u,v,z = cam
        plt.scatter([u],[v],c=[z],cmap='rainbow_r',alpha=0.5,s=2) #x, y= u, v // z=color
        
    def matching_check(self, camera_center2d_msg, lidar_center2d_msg):
        #<--------------------Visualization checking-------------------->
        # 2D bounding box center coodinate is matched with projection result?
        # YES!! 
        i=0
        # center: BoundingBoxCenter
        center: Cluster
        # for center in center2d_msg.centers:
        for center in lidar_center2d_msg.centers:
            plt.scatter(
                [lidar_center2d_msg.centers[i].x_center], 
                [lidar_center2d_msg.centers[i].y_center],
                marker='o',
                s=30,
                c='red',
                edgecolors='black'
                )
            i=i+1

        j=0
        center: BoundingBoxCenter
        for center in camera_center2d_msg.centers:
            plt.scatter(
                [camera_center2d_msg.centers[j].x_center], 
                [camera_center2d_msg.centers[j].y_center],
                marker='o',
                s=30,
                c='lightgreen',
                edgecolors='black'
                )
            j=j+1
        plt.savefig('/home/hwi/result.png',bbox_inches='tight')

    #CALL_BACK
    def proj_callback(self, img_msg, point_msg, camera_center2d_msg, lidar_center2d_msg) :
        self.get_logger().info('Callback is started')
        self.projection(img_msg, point_msg)
        self.matching_check(camera_center2d_msg, lidar_center2d_msg)

        

def main() -> None:
    rclpy.init()

    node = PROJmodule()
    # rclpy.spin(node)
    try :
        rclpy.spin(node)
    except KeyboardInterrupt :
        node.get_logger().info('Stopped by Keyboard')
    finally :
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()