'''
sub: /alive_turtles #Turtles, /turtle1/pose #Pose
pub: /turtle1/cmd_vel #Twist
client: /catch_turtle to kill turtle
'''
import math
from collections import namedtuple

import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import Turtles
from my_robot_interfaces.srv import CatchTurtle
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleController(Node):
	def __init__(self):
		super().__init__('turtle_controller')
		self.declare_parameter('xmin', 0.5)
		self.xmin = self.get_parameter('xmin').value
		self.declare_parameter('p', 1)
		self.P = self.get_parameter('p').value # proportionality constant

		self.create_subscription(Pose, 'turtle1/pose', self.callback_turtle1, 10)
		self.create_subscription(Turtles, 'alive_turtles', self.callback_alive_turtles, 20)
		self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 20)

	def callback_alive_turtles(self, msg):
		# subscribes to list of alive turtles
		shortest_dist = 250
		dist_threshold = 2
		nearest = None
		for turtle in msg.turtles_pose:
			dist = (turtle.x - self.turtle1.x)**2 + (turtle.y - self.turtle1.y)**2

			if dist < dist_threshold:
				self.call_catch_turtle(turtle)
			elif dist < shortest_dist:
				shortest_dist = dist
				nearest = turtle
		# self.get_logger().info(f"nearest is {nearest}")
		if nearest is not None:
			x = 1 * (nearest.x - self.turtle1.x)
			y = 1 * (nearest.y - self.turtle1.y)
			raw_angle = (math.atan2(y, x) - self.turtle1.theta)
			# self.get_logger().info(f" angle is {math.degrees(raw_angle)}")

			if raw_angle < 0:
				raw_angle += 2 * math.pi
			if raw_angle < 0.1:
				angle = 0.0
			elif raw_angle > math.pi:
				angle = -1.0
			else: 
				angle = 1.0
			self.publish_vel(self.xmin + shortest_dist/2, 0.0, self.P * angle)
		# self.publish_vel(3.0, 0.0, 0.0)

	def callback_turtle1(self, msg):
		# updates live position of turtle1
		turtles_pose = namedtuple('turtles_pose', ['x', 'y', 'theta'])
		self.turtle1 = turtles_pose(msg.x, msg.y, msg.theta)
		# self.get_logger().info(f'turtle1 is at {self.turtle1}')

	def call_catch_turtle(self, turtle):
		self.get_logger().info("catching turtle!")
		client = self.create_client(CatchTurtle, 'catch_turtle')
		while not client.wait_for_service(3.0):
			self.get_logger().warn("waiting for service catch_turtle")
		request = CatchTurtle.Request()
		request.name = turtle.name
		future = client.call_async(request)
		future.add_done_callback(self.callback_catch_turtle)

	def callback_catch_turtle(self, future):
		self.response = future.result()
		self.get_logger().info(self.response.reply)


	def publish_vel(self, x, y, angle):
		msg = Twist()
		msg.linear.x = x
		msg.linear.y = y
		msg.angular.z = angle
		self.publisher.publish(msg)



def main(args=None):
	rclpy.init(args=args)
	node = TurtleController()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()