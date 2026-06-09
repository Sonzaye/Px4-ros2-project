"""
ArUco Detector

Subscribes to the downward camera, detects ArUco markers, and publishes the
detected marker's center pixel on /perception/marker_detection (PointStamped).
Also publishes an annotated image on /perception/marker_image for RViz.

frame_id of the PointStamped = the marker ID as a string (e.g. "0").
point.x, point.y = center pixel of the marker.
point.z = marker area in pixels (rough proximity proxy).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

import cv2
import cv2.aruco as aruco
import numpy as np


class ArucoDetector(Node):
    CAMERA_TOPIC = (
        '/world/default/model/x500_mono_cam_0/link/down_camera_link/'
        'sensor/down_camera/image'
    )

    def __init__(self):
        super().__init__('aruco_detector')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.bridge = CvBridge()

        # Must match the dictionary used to generate the marker (DICT_4X4_50)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.create_subscription(Image, self.CAMERA_TOPIC, self.image_cb, qos)
        self.detection_pub = self.create_publisher(
            PointStamped, '/perception/marker_detection', qos)
        self.overlay_pub = self.create_publisher(
            Image, '/perception/marker_image', qos)

        self.frame_count = 0
        self.get_logger().info('ArUco detector ready.')

    def image_cb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        overlay = bgr.copy()
        h, w = bgr.shape[:2]
        # draw image center crosshair
        cv2.line(overlay, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (255, 255, 255), 1)
        cv2.line(overlay, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (255, 255, 255), 1)

        best = None  # (marker_id, cx, cy, area)

        if ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(overlay, corners, ids)
            # pick the largest marker (closest / most reliable)
            for i, corner in enumerate(corners):
                pts = corner.reshape(4, 2)
                cx = float(pts[:, 0].mean())
                cy = float(pts[:, 1].mean())
                area = float(cv2.contourArea(pts.astype(np.float32)))
                marker_id = int(ids[i][0])
                if best is None or area > best[3]:
                    best = (marker_id, cx, cy, area)
                cv2.circle(overlay, (int(cx), int(cy)), 6, (0, 255, 0), -1)

        # publish overlay
        try:
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)
        except Exception as e:
            self.get_logger().error(f'overlay publish failed: {e}')

        # publish best detection
        if best is not None:
            marker_id, cx, cy, area = best
            det = PointStamped()
            det.header = msg.header
            det.header.frame_id = str(marker_id)   # marker ID as string
            det.point.x = cx
            det.point.y = cy
            det.point.z = area
            self.detection_pub.publish(det)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            if best is not None:
                self.get_logger().info(
                    f'frame {self.frame_count}: marker {best[0]} at '
                    f'({best[1]:.0f}, {best[2]:.0f}) area={best[3]:.0f}')
            else:
                self.get_logger().info(f'frame {self.frame_count}: no marker')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
