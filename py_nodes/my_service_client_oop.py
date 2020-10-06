from functools import partial

import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


class ClientNode(Node):
	def __init__(self):
		super().__init__("my_service_client_oop")
		self.response = None
		self.call_add_two_ints(4, 2)

	def call_add_two_ints(self, a, b):
		client = self.create_client(AddTwoInts, "add_two_ints")
		while not client.wait_for_service(3.0):
			self.get_logger().warn("Waiting for service add_two_ints")
		request = AddTwoInts.Request()
		request.a = a
		request.b = b
		future = client.call_async(request)
		future.add_done_callback(partial(self.callback_add_two_ints, input_a=a, input_b=b)) # callback when future completes.

	def callback_add_two_ints(self, future, input_a, input_b):
		self.response = future.result()
		self.get_logger().info(f'{input_a} + {input_b} = {self.response.sum}')


def main(args=None):
	rclpy.init(args=args)
	node = ClientNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()