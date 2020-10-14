'''
sub: /alive_turtles #Turtles, /turtle1/pose #Pose
pub: /turtle1/cmd_vel #Twist
client: /catch_turtle to kill turtle
'''
import math
from collections import namedtuple

import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import Turtles # turtles_pose
from my_robot_interfaces.srv import CatchTurtle # name --- reply
from turtlesim.msg import Pose # x y theta linear angular
from geometry_msgs.msg import Twist # linear angular


class TurtleController(Node):
	def __init__(self):
		super().__init__('turtle_controller')
		self.declare_parameter('xmin', 3)
		self.xmin = self.get_parameter('xmin').value
		self.declare_parameter('xp', 10)
		self.xp = self.get_parameter('xp').value
		self.declare_parameter('anglep', 7)
		self.anglep = self.get_parameter('anglep').value # proportionality constant
		self.nearest = None

		self.create_subscription(Pose, 'turtle1/pose', self.callback_turtle1, 10) # pose of main turtle

		self.create_subscription(Turtles, 'alive_turtles', self.callback_alive_turtles, 20) 

		self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 20) # to control main turtle
		self.create_timer(0.02, self.publish_vel)

	def callback_alive_turtles(self, msg):
		# subscribes to list of alive turtles. Updates nearest turtle
		shortest_dist = 250 # map is ~11*11
		for turtle in msg.turtles_pose:
			dist = (turtle.x - self.turtle1.x)**2 + (turtle.y - self.turtle1.y)**2
			if dist < shortest_dist:
				shortest_dist = dist
				self.nearest = turtle
		# self.get_logger().info(f"nearest is {nearest}")

	def callback_turtle1(self, msg):
		# updates live position of turtle1
		turtles_pose = namedtuple('turtles_pose', ['x', 'y', 'theta'])
		self.turtle1 = turtles_pose(msg.x, msg.y, msg.theta)
		# self.get_logger().info(f'turtle1 is at {self.turtle1}')

	def publish_vel(self):
		msg = Twist()
		msg.linear.x = 0.0
		msg.angular.z = 0.0	
		if self.nearest is not None:
			x = self.nearest.x - self.turtle1.x
			y = self.nearest.y - self.turtle1.y
			dist = x**2 + y**2
			
			dist_threshold = 2 
			if dist < dist_threshold: # close enough to kill
				self.call_catch_turtle(self.nearest)
			else:
				raw_angle = (math.atan2(y, x) - self.turtle1.theta)
				# self.get_logger().info(f" angle is {math.degrees(raw_angle)}")
				if raw_angle < 0:
					raw_angle += 2 * math.pi # makes raw_angle always positive
				if raw_angle < 0.4: # close enough to not turn
					angle = 0.0
					msg.linear.x = self.xmin + self.xp * math.sqrt(dist)/2

				elif raw_angle > math.pi: # cw is nearer
					angle = -1.0
				else: 
					angle = 1.0

				msg.angular.z = self.anglep * angle
		self.publisher.publish(msg)


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


def main(args=None):
	rclpy.init(args=args)
	node = TurtleController()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()