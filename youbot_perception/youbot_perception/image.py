# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# from youbot_perception.yolo_detector import YOLODetector
# import numpy as np

# class ImageSubscriber(Node):
#     def __init__(self):
#         super().__init__('image_subscriber')
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/image_raw',
#             self.listener_callback,
#             10)
#         self.bridge = CvBridge()
#         self.detector = YOLODetector(model_path='yolov8s.pt')  # Update path as needed

#     def listener_callback(self, msg):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             # Run YOLO detection
#             results = self.detector.detect(cv_image)
#             # Render results on image
#             results.render()
#             detected_img = results.ims[0]
#             cv2.imshow('YOLO Detection', detected_img)
#             cv2.waitKey(1)
#         except Exception as e:
#             self.get_logger().error(f'Could not convert image or run detection: {e}')

# def main(args=None):
#     rclpy.init(args=args)
#     image_subscriber = ImageSubscriber()
#     try:
#         rclpy.spin(image_subscriber)
#     except KeyboardInterrupt:
#         pass
#     image_subscriber.destroy_node()
#     rclpy.shutdown()
#     cv2.destroyAllWindows()