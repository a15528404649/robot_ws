#!/usr/bin/env python3
"""ROS 2 wrapper for the original YZZ YOLOv5 detector."""

from pathlib import Path
import sys
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from yzz_detect.msg import ImageResults, Result


PACKAGE_LIB = Path(__file__).resolve().parent
sys.path.insert(0, str(PACKAGE_LIB))


class YoloDetect(Node):
    def __init__(self):
        super().__init__('yolo_detect')
        self.declare_parameter('weights', str(PACKAGE_LIB / 'yolov5s.pt'))
        self.declare_parameter('input_topic', 'usb_cam/image_raw')
        self.declare_parameter('result_topic', '/detect_result_out')
        self.declare_parameter('image_topic', '/detect_image_out')
        self.declare_parameter('detection_period', 3.0)
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('classes', [0, 1, 2, 3, 5, 7])

        try:
            import torch
            import torch.backends.cudnn as cudnn
            from models.experimental import attempt_load
            from utils.general import check_img_size, non_max_suppression, scale_coords, plot_one_box
            from utils.torch_utils import select_device
        except ImportError as exc:
            raise RuntimeError(
                'yzz_detect requires the Jetson-compatible PyTorch runtime and the '
                'bundled YOLOv5 Python modules.'
            ) from exc

        self.torch = torch
        self.cudnn = cudnn
        self.check_img_size = check_img_size
        self.non_max_suppression = non_max_suppression
        self.scale_coords = scale_coords
        self.plot_one_box = plot_one_box
        self.device = select_device('')
        self.half = self.device.type != 'cpu'
        self.model = attempt_load(self.get_parameter('weights').value, map_location=self.device)
        self.image_size = int(self.check_img_size(640, s=self.model.stride.max()))
        if self.half:
            self.model.half()
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.cudnn.benchmark = True

        self.bridge = CvBridge()
        self.last_detect_time = 0.0
        self.result_pub = self.create_publisher(
            ImageResults, self.get_parameter('result_topic').value, 3)
        self.image_pub = self.create_publisher(
            Image, self.get_parameter('image_topic').value, 3)
        self.image_sub = self.create_subscription(
            Image, self.get_parameter('input_topic').value, self.image_callback, 3)

    def preprocess(self, image):
        height, width = image.shape[:2]
        ratio = min(self.image_size / height, self.image_size / width)
        resized = cv2.resize(image, (round(width * ratio), round(height * ratio)))
        pad_w = self.image_size - resized.shape[1]
        pad_h = self.image_size - resized.shape[0]
        padded = cv2.copyMakeBorder(
            resized, pad_h // 2, pad_h - pad_h // 2, pad_w // 2, pad_w - pad_w // 2,
            cv2.BORDER_CONSTANT, value=(114, 114, 114))
        return np.ascontiguousarray(padded[:, :, ::-1].transpose(2, 0, 1))

    def image_callback(self, image_msg):
        now = time.monotonic()
        if now - self.last_detect_time < self.get_parameter('detection_period').value:
            return
        self.last_detect_time = now
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            self.detect(image, image_msg.header)
        except Exception as exc:
            self.get_logger().error(f'Detection failed: {exc}')

    def detect(self, source_image, header):
        tensor = self.torch.from_numpy(self.preprocess(source_image)).to(self.device)
        tensor = tensor.half() if self.half else tensor.float()
        tensor /= 255.0
        if tensor.ndimension() == 3:
            tensor = tensor.unsqueeze(0)
        with self.torch.no_grad():
            prediction = self.model(tensor, augment=False)[0]
        detections = self.non_max_suppression(
            prediction,
            self.get_parameter('confidence_threshold').value,
            self.get_parameter('iou_threshold').value,
            classes=list(self.get_parameter('classes').value),
            agnostic=False,
        )[0]

        annotated = source_image.copy()
        output = ImageResults()
        output.image = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        output.image.header = header
        if detections is not None and len(detections):
            detections[:, :4] = self.scale_coords(tensor.shape[2:], detections[:, :4], source_image.shape).round()
            for *xyxy, confidence, cls in reversed(detections):
                class_id = int(cls)
                self.plot_one_box(xyxy, annotated, label=f'{self.names[class_id]} {float(confidence):.2f}', color=[0, 255, 0], line_thickness=3)
                result = Result()
                result.xyxy = [int(value) for value in xyxy]
                result.type = class_id
                result.label = self.names[class_id]
                result.score = float(confidence)
                output.results.append(result)
            output.image = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            output.image.header = header
            self.result_pub.publish(output)
            self.image_pub.publish(output.image)


def main():
    rclpy.init()
    node = None
    try:
        node = YoloDetect()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
