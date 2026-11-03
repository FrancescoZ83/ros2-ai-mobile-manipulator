#!/usr/bin/env python3
"""Depth from the stereo pair: block matching on the left and right images, published in the left frame.

On demand by default: the matching costs a lot of CPU (running at the rate of the cameras, the simulation falls
to a real time factor of 0.3), so the node keeps the last pair and computes when /stereo_depth/compute is called.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger


class StereoDepth(Node):
    def __init__(self):
        super().__init__('stereo_depth')
        self.declare_parameter('baseline', 0.05)             # [m] distance between the two cameras
        self.declare_parameter('focal_length', 233.9)        # [px] used until the camera_info of the left camera arrives
        # 128 disparities: the nearest depth is f * baseline / 128 = 0.09 m, below the near clip (0.1 m) of the cameras
        self.declare_parameter('num_disparities', 128)       # multiple of 16
        self.declare_parameter('block_size', 5)              # odd
        self.declare_parameter('rate', 0.0)                  # [Hz] 0 = only when /stereo_depth/compute is called
        self.baseline = self.get_parameter('baseline').value
        self.pad = self.get_parameter('num_disparities').value
        self.focal = self.get_parameter('focal_length').value
        nd, bs = self.get_parameter('num_disparities').value, self.get_parameter('block_size').value
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=nd, blockSize=bs, P1=8 * bs * bs, P2=32 * bs * bs,
            disp12MaxDiff=1, uniquenessRatio=10, speckleWindowSize=60, speckleRange=2, preFilterCap=31,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/stereo/depth', 5)
        self.create_subscription(CameraInfo, '/stereo/left/camera_info', self.on_info, 1)
        left = Subscriber(self, Image, '/stereo/left/image', qos_profile=qos_profile_sensor_data)
        right = Subscriber(self, Image, '/stereo/right/image', qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([left, right], queue_size=5, slop=0.02)
        self.sync.registerCallback(self.on_pair)
        self.latest = None
        self.create_service(Trigger, '~/compute', self.on_compute)
        rate = self.get_parameter('rate').value
        if rate > 0.0:
            self.create_timer(1.0 / rate, self.compute)
        self.get_logger().info(f'stereo depth ready: baseline {self.baseline} m, {nd} disparities, block {bs}, '
                               f'{"on demand" if rate <= 0.0 else f"{rate} Hz"}')

    def on_info(self, msg):
        self.focal = msg.k[0]

    def on_pair(self, left, right):
        self.latest = (left, right)         # cheap: the matching happens in compute()

    def on_compute(self, _, response):
        response.success = self.compute()
        response.message = 'depth published' if response.success else 'no image pair yet'
        return response

    def compute(self):
        if self.latest is None:
            return False
        left, right = self.latest
        gray = [cv2.cvtColor(self.bridge.imgmsg_to_cv2(m, 'rgb8'), cv2.COLOR_RGB2GRAY) for m in (left, right)]
        # The matcher gives nothing in the first numDisparities columns; a border on the left of both images
        # moves that blind strip out of the picture (a pixel at x can match up to disparity x)
        padded = [cv2.copyMakeBorder(g, 0, 0, self.pad, 0, cv2.BORDER_REPLICATE) for g in gray]
        disparity = self.matcher.compute(padded[0], padded[1])[:, self.pad:].astype(np.float32) / 16.0    # [px], <= 0 where no match
        depth = np.full(disparity.shape, np.nan, dtype=np.float32)
        ok = disparity > 0.5
        depth[ok] = self.focal * self.baseline / disparity[ok]
        out = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        out.header = left.header
        self.pub.publish(out)
        return True


def main():
    rclpy.init()
    node = StereoDepth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
