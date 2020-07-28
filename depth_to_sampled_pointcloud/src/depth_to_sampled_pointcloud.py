#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import image_geometry

from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField


CONV_FACTOR=0.001 # for 16UC1 depth image

class Depth2SampledPC(object):
    """docstring for Depth2SampledPC"""
    def __init__(self):
        rospy.init_node('depth_to_sampled_pointcloud')
        self.bridge = CvBridge()

        # Parameters
        self.n_points    = rospy.get_param('~num_points', 1000)
        max_range   = rospy.get_param('~max_range', 2.)
        self.max_range = int(max_range*1000)
        self.null_range = self.max_range

        # Get camera parameters
        camera_info_msg = rospy.wait_for_message("camera_info",CameraInfo)
        self.model = image_geometry.PinholeCameraModel()
        self.model.fromCameraInfo(camera_info_msg)
        constant_x = CONV_FACTOR / self.model.fx();
        constant_y = CONV_FACTOR / self.model.fy();
        self.cam_frame = camera_info_msg.header.frame_id

        # Configure image space
        x = (np.arange(camera_info_msg.width,dtype=np.float32)-self.model.cx())*constant_x
        y = (np.arange(camera_info_msg.height,dtype=np.float32)-self.model.cy())*constant_y
        xx, yy = np.meshgrid(x, y, copy=False)
        self.xx = xx.ravel()
        self.yy = yy.ravel()

        # Configure pointCloud
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1)
                      ]

        # setup subscriber and publisher
        self.pub = rospy.Publisher("point_cloud", PointCloud2, queue_size=2)
        rospy.Subscriber("depth_image_rect", Image, self.depthCallback)
        rospy.spin()

    def depthCallback(self, msg):
        
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1").ravel()
            # Find valid points
            valid_points = np.logical_and(depth_image>0, depth_image<self.max_range)
            valid_index = np.nonzero(valid_points)[0]
            if len(valid_index) == 0:
                rospy.logwarn("No valid points")
                return
            # Change not valid to null range
            null_index = np.nonzero(np.logical_not(valid_points))[0]
            depth_image[null_index] = self.null_range
            # Take sample from valid points
            n_samples = min(self.n_points, valid_index.size)
            sample_valid = np.random.choice(valid_index,n_samples-200,replace=False)
            sample_null = np.random.choice(null_index,200,replace=False)
            sample = np.hstack([sample_valid,sample_null])

            # Calculate points position
            depth = depth_image[sample].astype(np.float32)
            X= self.xx[sample]*depth
            Y= self.yy[sample]*depth
            Z = depth*CONV_FACTOR

            # Create PC2 message
            header = Header()
            header.frame_id = self.cam_frame
            pc2 = point_cloud2.create_cloud(header, self.fields, np.stack([X,Y,Z],axis=1))
            self.pub.publish(pc2)


        except CvBridgeError, e:
            print e
        

if __name__ == '__main__':
    Depth2SampledPC()