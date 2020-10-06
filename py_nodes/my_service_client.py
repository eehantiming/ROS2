import sys
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


def main(args=None):
	rclpy.init(args=args)
	node = Node("my_service_client")
	my_client = node.create_client(AddTwoInts, "add_two_ints")
	while not my_client.wait_for_service(3.0): # wait for s, returns true when service up
		node.get_logger().warn("waiting for service add_two_ints")
	request = AddTwoInts.Request()
	request.a = int(sys.argv[1]) # from command line args
	request.b = int(sys.argv[2])

	le_future = my_client.call_async(request) # future obj from python
	rclpy.spin_until_future_complete(node, le_future) # node continues to spin and call callbacks until future is done ## problem is node will be killed when future completes. there may be other functions inside the node. see _oop for spin implementation

	try:
		response = le_future.result()
		node.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
	except Exception as e:
		node.get_logger().error(e)
	rclpy.shutdown()


if __name__ == "__main__":
	main()