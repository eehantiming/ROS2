import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


class MyServiceServer(Node):
	def __init__(self):
		super().__init__("my_service_server")
		self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints) # name of srv
		self.get_logger().info("add_two_ints server has been started!")

	def callback_add_two_ints(self, request, response):
		response.sum = request.a + request.b
		self.get_logger().info(f"received {request.a} and {request.b}. returning {response.sum}")
		return response


def main(args=None):
	rclpy.init(args=args)
	node = MyServiceServer()
	rclpy.spin(node)
	rclpy.shutdown()


if __name__ == "__main__":
	main()