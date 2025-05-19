#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import torch
import os
import sys
import numpy as np
import logging
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from torch.utils.data import Dataset, DataLoader
from PIL import Image as PILImage
import pyrealsense2 as rs

import net
from face_alignment import align

LOGGER = logging.getLogger(__name__)

# 数据集类：加载目标注册图片
class ImageDataset(Dataset):
    def __init__(self, root_dir, target_label):
        self.root_dir = root_dir
        self.image_paths = []
        class_dir = os.path.join(root_dir, target_label)
        if os.path.isdir(class_dir):
            for img_file in os.listdir(class_dir):
                img_path = os.path.join(class_dir, img_file)
                self.image_paths.append(img_path)

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, idx):
        return self.image_paths[idx]

# 加载预训练模型
def load_pretrained_model(architecture='ir_50'):
    adaface_models = {
        'ir_50': "pretrained/adaface_ir50_ms1mv2.ckpt",
    }
    assert architecture in adaface_models.keys()
    model = net.build_model(architecture)
    statedict = torch.load(adaface_models[architecture], weights_only=False)['state_dict']
    # 去除 "model." 前缀
    model_statedict = {key[6:]: val for key, val in statedict.items() if key.startswith('model.')}
    model.load_state_dict(model_statedict)
    model.eval()
    return model

# 将 PIL 图像转换为模型输入张量
def to_input(pil_rgb_image):
    np_img = np.array(pil_rgb_image)
    bgr_img = ((np_img[:, :, ::-1] / 255.) - 0.5) / 0.5
    tensor = torch.tensor(np.array(bgr_img.transpose(2, 0, 1))).unsqueeze(0).float()
    return tensor

# 提取人脸特征
def embedding_face_features(model, aligned_img):
    bgr_tensor_input = to_input(aligned_img)
    embedding, _ = model(bgr_tensor_input)
    return embedding

# 绘制边框并标注（可附带距离信息）
def draw_bbox(img, bbox, class_name, distance=None):
    x, y, w, h, _ = bbox
    x, y, w, h = map(int, [x, y, w, h])
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.8
    thickness = 1
    label = f"{class_name}"
    if distance is not None:
        label += f" ({distance:.2f}m)"
    text_position = (x, y + 24)
    font_color = (0, 0, 0)
    bbox_color = [0, 0, 255] if class_name != "None" else [0, 0, 0]
    cv2.rectangle(img, (x, y), (w, h), bbox_color, 2)
    (text_width, text_height), baseline = cv2.getTextSize(label, font, font_scale, thickness)
    bg_start = (x, y + 26 - text_height - baseline)
    bg_end = (x + text_width, y + 26)
    cv2.rectangle(img, bg_start, bg_end, (255, 255, 255), -1)
    cv2.putText(img, label, text_position, font, font_scale, font_color, thickness, cv2.LINE_AA)
    return img

# ROS2 节点类：使用 RealSense 摄像头读取彩色和深度图像，并发布识别信息
class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__("face_recognition_node")
        # 声明并读取参数
        self.declare_parameter("threshold", 0.4)
        self.declare_parameter("target_dir", "./TARGETIMG")
        self.declare_parameter("annotated_topic", "color_image")
        self.declare_parameter("face_info_topic", "person_detect")
        self.threshold = self.get_parameter("threshold").value
        self.target_dir = self.get_parameter("target_dir").value
        self.annotated_topic = self.get_parameter("annotated_topic").value
        self.face_info_topic = self.get_parameter("face_info_topic").value

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, self.annotated_topic, 10)
        self.face_info_pub = self.create_publisher(String, self.face_info_topic, 10)

        self.get_logger().info("Loading pretrained model...")
        self.model = load_pretrained_model('ir_50')

        # 加载目标图片并提取人脸特征
        self.training_features = {}
        self.class_list = os.listdir(self.target_dir)
        self.get_logger().info(f"Found classes: {self.class_list}")
        for class_name in self.class_list:
            dataset = ImageDataset(root_dir=self.target_dir, target_label=class_name)
            data_loader = DataLoader(dataset, batch_size=1, shuffle=False)
            self.training_features[class_name] = []
            for img_paths in data_loader:
                for img_path in img_paths:
                    aligned_rgb_imgs, _ = align.get_aligned_face(img_path)
                    for aligned_img in aligned_rgb_imgs:
                        embedding = embedding_face_features(self.model, aligned_img)
                        self.training_features[class_name].append(embedding)
            self.get_logger().info(f"[{class_name}]: Registered {len(self.training_features[class_name])} faces")

        # 初始化 RealSense 流水线（彩色与深度流）
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.get_logger().info("RealSense camera pipeline started.")

        # 创建定时器，10Hz调用一次回调函数
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            self.get_logger().error("未获取到深度或彩色图像")
            return

        color_image = np.asanyarray(color_frame.get_data())
        cv_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(cv_rgb)
        # 注意：检测时传入 None 表示直接使用 PIL 图像
        aligned_rgb_imgs, bboxes = align.get_aligned_face(None, pil_image)

        num_recognized = 0
        face_info_str = ""
        if len(aligned_rgb_imgs) > 0:
            cam_features = []
            for aligned_img in aligned_rgb_imgs:
                embedding = embedding_face_features(self.model, aligned_img)
                cam_features.append(embedding)
            num_face = len(cam_features)
            similarity_scores = {}
            for class_name in self.class_list:
                num_training = len(self.training_features[class_name])
                if num_face == 0:
                    similarity_scores[class_name] = np.zeros((num_training, 1))
                else:
                    training_features_matrix = torch.cat(self.training_features[class_name])
                    cam_features_matrix = torch.cat(cam_features)
                    similarity_scores[class_name] = torch.mm(training_features_matrix, cam_features_matrix.T).detach().numpy()

            face_info_str = f"total:{num_face};"
            for i in range(num_face):
                best_class = "None"
                best_score = 0.0
                for class_name in self.class_list:
                    score = np.max(similarity_scores[class_name][:, i])
                    if score > best_score:
                        best_score = score
                        best_class = class_name
                if best_score < self.threshold:
                    best_class = "None"
                bbox = bboxes[i]
                x, y, w, h, _ = bbox
                center_x = int((x + w) / 2)
                center_y = int((y + h) / 2)
                dist_total = 0.0
                cnt = 0
                for nx in range(-4,4,1):
                    for ny in range(-4,4,1):
                        dist_total += depth_frame.get_distance(center_x + int(nx*(w/100)), center_y + int(ny*(h/100)))
                        cnt +=1
                # distance = depth_frame.get_distance(center_x, center_y)
                if cnt > 0:
                    distance = dist_total/cnt;
                else:
                    distance = 0.0

                color_image = draw_bbox(color_image, bbox, best_class, distance)
                if best_class != "None":
                    num_recognized += 1
                    face_info_str += f"num_{num_recognized}:{distance:.2f};"
        else:
            num_face = 0
            face_info_str = f"total:{num_recognized};"

        # 发布带标注的图像
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            self.image_pub.publish(annotated_msg)
        except CvBridgeError as e:
            self.get_logger().error("CvBridge 错误: " + str(e))

        # 发布人脸识别信息
        face_info_msg = String()
        face_info_msg.data = face_info_str
        self.face_info_pub.publish(face_info_msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

