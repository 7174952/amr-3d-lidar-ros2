#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
import torch
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import signal
import sys

class RealsenseYoloNode(Node):
    def __init__(self):
        super().__init__('person_detect_node')

        # 发布器
        self.detection_pub = self.create_publisher(String, '/person_detect', 10)
        self.image_pub = self.create_publisher(Image, '/color_image', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # RealSense初始化
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        # 加载YOLO模型
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # 可替换为 yolov5m/l/x
        if torch.cuda.is_available():
            self.model.cuda()
            print("use cuda")

        # 设置定时器：10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # 目标检测
        results = self.model(color_image)
        person_count = 0
        detection_info = []

        for *box, conf, cls in results.xyxy[0]:
            if self.model.names[int(cls)] == 'person':
                person_count += 1
                x1, y1, x2, y2 = map(int, box)
                color = (0, 255, 0)
                cv2.rectangle(color_image, (x1, y1), (x2, y2), color, 2)

                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                distance = depth_frame.get_distance(center_x, center_y)
                if distance < 0.01:
                    person_count -= 1
                    continue
                label = f'{self.model.names[int(cls)]} {distance:.2f}'
                cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                detection_info.append(f'num_{person_count}:{distance:.2f}')

        # 发布消息
        detection_message = f'total:{person_count};' + '; '.join(detection_info) + ';'
        self.get_logger().info(detection_message)
        self.detection_pub.publish(String(data=detection_message))

        # 发布图像
        image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.image_pub.publish(image_msg)

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def shutdown_handler(signum, frame):
    print(f"[Signal] Caught signal {signum}, shutting down ROS 2 node.")
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RealsenseYoloNode()

    # 注册信号处理器
    signal.signal(signal.SIGINT, shutdown_handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, shutdown_handler)  # QProcess::terminate()

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Exception: {e}")
    finally:
        node.destroy_node()
        print("[Shutdown] ROS node destroyed.")
        rclpy.shutdown()
        print("[Shutdown] rclpy shutdown complete.")

if __name__ == '__main__':
    main()
