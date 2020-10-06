# sub: to /number_count and prints
# service client: call /reset_counter with True to reset counter

from functools import partial

import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class ServerClientNode(Node):
	def __init__(self):
		super().__init__("reset_counter_client")
		self.create_subscription(Int64, "number_count", self.callback_number_count, 10)
		# self.reset_counter(True)

	def callback_number_count(self, msg):
		self.get_logger().info(f"{msg.data}")
		if msg.data > 30:
			self.get_logger().info("number is >30, resetting to 0!")
			self.reset_counter(True)

	def reset_counter(self, action):
		client = self.create_client(SetBool, "reset_counter")
		while not client.wait_for_service(3.0):
			self.get_logger().warn("waiting for service /reset_counter")
		request = SetBool.Request()
		request.data = action
		future = client.call_async(request)
		future.add_done_callback(partial(self.callback_reset_counter, data=action))

	def callback_reset_counter(self, future, data):
		response = future.result()
		self.get_logger().info(f"{data} -> {response.success}; {response.message}")


def main(args=None):
	rclpy.init(args=args)
	node = ServerClientNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()