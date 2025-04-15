import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool  # 改为SetBool
import json

class ChatBotNode(Node):
    def __init__(self):
        super().__init__('chatboot_node')

        # Publisher
        self.publisher_ = self.create_publisher(String, 'chatbot_state', 10)

        # Service (SetBool)
        self.srv = self.create_service(
            SetBool, 'chatbot_wakeup',
            self.set_bool_callback)
        self.req_wakeup = False

    def set_bool_callback(self, request, response):
        self.get_logger().info(f'收到布尔请求: {request.data}')
        response.success = True
        response.message = ''
        self.req_wakeup = request.data
        return response

    def publish_text(self, text: str):
        msg = String()
        if isinstance(text, dict):
            msg.data = text.get('result', 'わかりませんでした。')
        else:
            msg.data = str(text)
        self.publisher_.publish(msg)
