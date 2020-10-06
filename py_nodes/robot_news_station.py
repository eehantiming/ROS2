import rclpy
from rclpy.node import Node

from example_interfaces.msg import String


class PublisherNode(Node):
	def __init__(self):
		super().__init__('robot_news_station')
		self.count = 1
		self.publisher = self.create_publisher(String, "robot_news", 10)
		self.timer = self.create_timer(1, self.publish_news)
		self.get_logger().info("robot news station started!")

	def publish_news(self):
		msg = String()
		msg.data = f"publishing: {self.count}"
		self.publisher.publish(msg)
		self.count += 1



def main(args=None):
	rclpy.init(args=args)
	node = PublisherNode()
	rclpy.spin(node)
	rclpy.shutdown()


if __name__ == "__main__":
	main()