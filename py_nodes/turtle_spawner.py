'''
client: /spawn to spawn new turtle
pub: /alive_turtles
server: /catch_turtle to kill and delete turtle
client: kill turtle
'''

import random
from functools import partial

import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn # x y theta string --- string
from turtlesim.srv import Kill
from my_robot_interfaces.msg import Turtle, Turtles
from my_robot_interfaces.srv import CatchTurtle


class TurtleSpawner(Node):
	def __init__(self):
		super().__init__('turtle_spawner')
		self.msg = Turtles()

		self.create_timer(1, self.create_turtle)

		self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)

		self.publisher = self.create_publisher(Turtles, 'alive_turtles', 20)
		self.create_timer(0.1, self.publish_turtles)

	def create_turtle(self):
		# periodically calls service to spawn turtle
		spawn_client = self.create_client(Spawn, 'spawn')
		while not spawn_client.wait_for_service(3.0):
			self.get_logger().warn("Waiting for srv Spawn")
		request = Spawn.Request()
		request.x = random.uniform(0, 11)
		request.y = random.uniform(0, 11)
		request.theta = random.uniform(0, 6.28)
		future = spawn_client.call_async(request)
		future.add_done_callback(partial(self.callback_spawn, request=request))

	def callback_spawn(self, future, request):
		self.response = future.result()
		self.get_logger().info(f"spawned {self.response.name}!")
		turtle = Turtle()
		turtle.name = self.response.name
		turtle.x = request.x
		turtle.y = request.y
		turtle.theta = request.theta
		self.msg.turtles_pose.append(turtle)

	def callback_catch_turtle(self, request, response):
		# delete turtle from list and kill turtle
		for turtle in self.msg.turtles_pose:
			if turtle.name == request.name:
				self.msg.turtles_pose.remove(turtle)
				response.reply = "Killed"

		kill_client = self.create_client(Kill, 'kill')
		while not kill_client.wait_for_service(3.0):
			self.get_logger().warn("waiting for service Kill")
		kill_request = Kill.Request()
		kill_request.name = request.name
		kill_client.call_async(kill_request)
		self.get_logger().info(f'killed {request.name}')
		return response

	def publish_turtles(self):
		# publish list of alive turtles and their pose
		# for turtle in self.msg.turtles_pose:
		# 	if turtle.name == 'turtle2':
		# 		self.msg.turtles_pose.remove(turtle)
		self.publisher.publish(self.msg)


def main(args=None):
	rclpy.init(args=args)
	node = TurtleSpawner()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()