#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointField, PointCloud2
from geometry_msgs.msg import Twist
import numpy as np
from cv_bridge import CvBridge
import cv2
from collections import namedtuple
import ctypes
import math
import struct
bridge = CvBridge()
template_img_path = get_package_share_directory('crx_description') + '/images/' + 'template.png'
bounding_box_img_path = get_package_share_directory('crx_description') + '/images/' + 'bounding_box.jpg'
found_img_coords = False
img_coords = None
image_height = None
image_width = None

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

class CalcCoords(Node):

    def __init__(self):
        super().__init__('calc_coords')
        qos_profile_image = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE,liveliness=LivelinessPolicy.AUTOMATIC,depth=10)
        qos_profile_points = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE,liveliness=LivelinessPolicy.AUTOMATIC,depth=10)
        self.publisher_twist = self.create_publisher(Twist,'coords',10)
        self.subscription_img = self.create_subscription(Image,'/camera_depth/image_raw',self.img_callback,qos_profile_image)
        self.subscription_pointcloud = self.create_subscription(PointCloud2,'/camera_depth/points',self.pointcloud_callback,qos_profile_points)
    
    def find_template(self,image,template):
        global found_img_coords, img_coords, image_height, image_width
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        image_height = image_gray.shape[0]
        image_width = image_gray.shape[1]
        h, w = template_gray.shape
        scale_results = []
        scales = np.linspace(0.1,3,30)[::-1]
        for scale in scales:
            resized_template = cv2.resize(template_gray, (int(w*scale), int(h*scale)))
            result = cv2.matchTemplate(image_gray, resized_template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(result)
            scale_results.append((max_val, max_loc, scale))
        best_scale_result = max(scale_results, key=lambda x: x[0])
        max_val, max_loc, best_scale = best_scale_result
        top_left = max_loc
        bottom_right = (int(top_left[0] + w*best_scale), int(top_left[1] + h*best_scale))
        img_coords = (int(top_left[0] + (w/2)*best_scale), int(top_left[1] + (h/2)*best_scale))
        return top_left, bottom_right

    def img_callback(self, msg):
        global found_img_coords
        large_image = bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
        template = cv2.imread(template_img_path)
        top_left, bottom_right = self.find_template(large_image, template)
        cv2.rectangle(large_image, top_left, bottom_right, (0, 255, 0), 2)
        cv2.imshow("test",large_image)
        cv2.waitKey(0) 
        found_img_coords = True
        self.get_logger().info("Successfully analyzed the raw image")

    def _get_struct_fmt(self,is_bigendian,fields,field_names=None):
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

    def read_points(self,cloud,field_names=None,skip_nans=False,uvs=[]):
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
    
    def find_plane(self,point_array,idx):
        point1 = point_array[idx]
        point2 = point_array[idx-1]
        point3 = point_array[idx+1]
        point4 = point_array[idx-image_width]
        point5 = point_array[idx+image_width]
        A = np.ones((5,3))
        B = np.ones((5,1))
        A[0,0:2] = point1[0:2]
        A[1,0:2] = point2[0:2]
        A[2,0:2] = point3[0:2]
        A[3,0:2] = point4[0:2]
        A[4,0:2] = point5[0:2]
        B[0,0] = point1[2]
        B[1,0] = point2[2]
        B[2,0] = point3[2]
        B[3,0] = point4[2]
        B[4,0] = point5[2]
        plane_normal = np.dot(np.dot(np.linalg.inv(np.dot(np.transpose(A),A)),np.transpose(A)),B)
        return plane_normal[0,0], plane_normal[1,0], -1
    
    def pointcloud_callback(self,msg):
        if found_img_coords == True:
            point_array = np.array(list(self.read_points(msg)))
            print(img_coords)
            print(point_array[-1])
            idx = (img_coords[1]*image_width) + img_coords[0]
            a1, a2, a3 = self.find_plane(point_array,idx)
            magnitude = math.sqrt((a1**2)+(a2**2)+(a3**2))
            coord_data = Twist()
            coord_data.linear.x = point_array[idx,0]
            coord_data.linear.y = point_array[idx,1]
            coord_data.linear.z = point_array[idx,2]
            coord_data.angular.x = math.acos(-a1/magnitude)
            coord_data.angular.y = math.acos(-a2/magnitude)
            coord_data.angular.z = math.acos(-a3/magnitude)
            self.publisher_twist.publish(coord_data)
            self.get_logger().info("Successfully published the 3D coordinate data to the coords topic")

def main(args=None):
    rclpy.init(args=args)
    calc_coords = CalcCoords()
    rclpy.spin(calc_coords)
    calc_coords.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()