# pub: number constantly to /number

import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64


class PublisherNode(Node):
	def __init__(self):
		super().__init__("number_publisher")
		self.number = 3
		self.publisher = self.create_publisher(Int64, "number", 10)
		self.timer = self.create_timer(1, self.publish_number)
		self.get_logger().info("number_publisher publishing on /number!")

	def publish_number(self):
		msg = Int64()
		msg.data = self.number
		self.publisher.publish(msg)


def main(args=None):
	rclpy.init(args=args)
	node = PublisherNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()