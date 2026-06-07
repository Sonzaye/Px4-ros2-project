"""
PX4 Perception Node: YOLO Object Detection

Subscribes to the drone's camera, runs YOLOv8 inference on each frame,
draws bounding boxes, and republishes the annotated image.

First run downloads YOLOv8n model weights (~6 MB) automatically.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO


class YoloDetector(Node):
    CAMERA_TOPIC = (
        '/world/default/model/x500_mono_cam_0/link/camera_link/'
        'sensor/camera/image'
    )
    OUTPUT_TOPIC = '/perception/image_annotated'

    def __init__(self):
        super().__init__('yolo_detector')

        # Load YOLOv8n. 'n' = nano = fastest. First call downloads weights.
        self.get_logger().info('Loading YOLOv8n model...')
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('Model loaded')

        self.bridge = CvBridge()

        # ros_gz_image bridge publishes camera with default RELIABLE QoS
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Image, self.CAMERA_TOPIC, self.image_callback, image_qos)

        self.annotated_pub = self.create_publisher(
            Image, self.OUTPUT_TOPIC, image_qos)

        self.frame_count = 0

    def image_callback(self, msg):
        # Convert ROS Image -> OpenCV BGR numpy array
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Run inference. verbose=False suppresses per-frame stdout spam.
        results = self.model(cv_image, verbose=False)

        # results[0].plot() draws boxes + labels onto a numpy array
        annotated = results[0].plot()

        # Convert back to ROS Image, preserving timestamp/frame
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        # Log detection count every ~2 seconds
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            n = len(results[0].boxes)
            classes = [self.model.names[int(c)] for c in results[0].boxes.cls] if n else []
            self.get_logger().info(
                f'frame {self.frame_count}: {n} detections {classes}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
