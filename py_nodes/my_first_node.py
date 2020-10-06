import rclpy
from rclpy.node import Node

class MyNode(Node):
	def __init__(self):
		super().__init__('py_test') # argument is name of node
		self.counter = 0
		self.get_logger().info('Hello ROS2')
		self.create_timer(0.5, self.timer_callback) # every 0.5s callback

	def timer_callback(self):
		self.counter += 1 # everytime this callback is called
		self.get_logger().info(f'Hello {self.counter}')

def main(args=None): # point to this function in setup.py
	rclpy.init(args=args)
	node = MyNode()
	# node = Node('py_test') # can do this directly without own class
	#node.get_logger().info('Hello ROS2')
	rclpy.spin(node) # keeps the node alive like a while loop
	rclpy.shutdown()


if __name__ == '__main__':
	main()