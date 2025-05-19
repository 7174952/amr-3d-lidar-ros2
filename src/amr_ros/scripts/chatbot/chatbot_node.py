import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json

class ChatBotNode(Node):
    def __init__(self):
        super().__init__('chatboot_node')

        # Publisher
        self.publisher_ = self.create_publisher(String, 'chatbot_state', 10)

        # Subscriber (取代原 SetBool 服务)
        self.subscription = self.create_subscription(
            Bool,
            'chatbot_wakeup',
            self.wakeup_callback,
            10)
        self.subscription  # 防止未使用警告

        self.req_wakeup = False  # 默认状态

    def wakeup_callback(self, msg: Bool):
        self.req_wakeup = msg.data
        self.get_logger().info(f'接收到唤醒指令: {self.req_wakeup}')

    def publish_text(self, text: str):
        msg = String()
        if isinstance(text, dict):
            msg.data = text.get('result', 'わかりませんでした。')
        else:
            msg.data = str(text)
        self.publisher_.publish(msg)
