# sub: to /number 
# pub: adds number to current sum publish to /number_counter
# service: /reset_counter resets current sum to 0

import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounter(Node):
	def __init__(self):
		super().__init__("number_counter")
		self.create_subscription(Int64, 'number', self.callback_number, 10)

		self.counter = 0
		self.publisher = self.create_publisher(Int64, 'number_count', 10)
		self.get_logger().info('subscribing number and publishing to /number_count!')

		self.create_service(SetBool, 'reset_counter', self.callback_reset_counter)
		self.get_logger().info('started service /reset_counter!')

	def callback_number(self, msg):
		self.counter += msg.data

		new_msg = Int64()
		new_msg.data = self.counter
		self.publisher.publish(new_msg)

	def callback_reset_counter(self, request, response):
		self.get_logger().info(f'service received {request.data}')
		if request.data == True:
			self.counter = 0
			response.success = True
			response.message = "reset to 0!"
			return response
		response.success = False
		response.message = "nothing really happened"
		return response


def main(args=None):
	rclpy.init(args=args)
	node = NumberCounter()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()