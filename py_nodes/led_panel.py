# service: /set_led 
# pub: to /led_states

import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import LedState #int8[] data
from my_robot_interfaces.srv import SetLed # bool data --- string data


class LEDPanel(Node):
	def __init__(self):
		super().__init__("led_panel")
		self.publisher = self.create_publisher(LedState, "led_states", 10)
		self.state = [0,0,0]
		self.create_timer(1, self.publish_state)
		self.get_logger().info("publishing on topic led_states!")

		self.create_service(SetLed, "set_led", self.callback_set_led)
		self.get_logger().info("started service set_led!")

	def publish_state(self):
		msg = LedState()
		msg.data = self.state
		self.publisher.publish(msg)

	def callback_set_led(self, request, response):
		# should check data type of request first and return warning
		if request.data: # true if charging
			self.get_logger().info(f"received {request.data} -> light on")
			response.data = "set_led: light on"
			self.state = [0,0,1]
		else: # false for not charging
			self.get_logger().info(f"received {request.data} -> light off")
			response.data = "set_led: light off"
			self.state = [0,0,0]
		# can publish here to show when there is a change
		return response


def main(args=None):
	rclpy.init(args=args)
	node = LEDPanel()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()