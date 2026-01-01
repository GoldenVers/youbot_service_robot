# YOLOv8 weights and model loading utility
# Place your YOLOv8 .pt file in the appropriate directory or specify the path below.

# import torch
# import cv2
# import numpy as np

# class YOLODetector:
#     def __init__(self, model_path='yolov8s.pt', device='cpu'):
#         self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)
#         self.device = device
#         self.model.to(self.device)

#     def detect(self, image):
#         # image: numpy array (BGR)
#         results = self.model(image)
#         return results
