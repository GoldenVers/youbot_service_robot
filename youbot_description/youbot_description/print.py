import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
# from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        self.bridge = CvBridge()
        
        # Load YOLO model
        # self.model = YOLO('yolov8n.pt')
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publish detections
        self.detection_pub = self.create_publisher(String, '/detections', 10)
        
        self.get_logger().info('YOLO Detector started!')

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Run YOLO detection
        # results = self.model(cv_image)
        
        # Process results and publish
        detection_msg = String()
        detection_msg.data = 'Detected objects: ...'
        self.detection_pub.publish(detection_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()