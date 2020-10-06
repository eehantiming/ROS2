# timer: change state
# service client: call to set led 

import time

import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import SetLed # bool data --- string data


class BatteryNode(Node):
	def __init__(self):
		super().__init__("battery_node")
		self.battery_state = 1 # for full
		self.create_timer(5, self.toggle_state) # battery state changes every 5s
		self.get_logger().info("battery node started, battery is full!")

	def toggle_state(self): # simulate battery level
		self.battery_state = 1 - self.battery_state
		if self.battery_state == 0:
			self.get_logger().warn("Battery is flat. charging now...")
			self.set_led(True)
		else:
			self.get_logger().warn("charging complete!")
			self.set_led(False)
		
	def set_led(self, action):
		client = self.create_client(SetLed, "set_led")
		while not client.wait_for_service(3.0):
			self.get_logger().warn("Waiting for service set_led")
		request = SetLed.Request()
		request.data = action # True or False to toggle LED
		future = client.call_async(request)
		future.add_done_callback(self.callback_set_led)

	def callback_set_led(self, future):
		response = future.result()
		self.get_logger().info(f"{response.data}")


def main(args=None):
	rclpy.init(args=args)
	node = BatteryNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()