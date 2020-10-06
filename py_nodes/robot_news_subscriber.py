import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

class SubscriberNode(Node):
	def __init__(self):
		super().__init__("news_subscriber")
		self.subscriber = self.create_subscription(String, "robot_news",  self.callback_robot_news, 10)
		self.get_logger().info("news subscriber started!")

	def callback_robot_news(self, msg):
		self.get_logger().info(msg.data)


def main(args=None):
	rclpy.init(args=args)
	node = SubscriberNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()