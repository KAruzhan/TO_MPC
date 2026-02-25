import rclpy
from rclpy.node import Node

import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import json

position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

class MinimalPublisher(Node):

	def __init__(self):
		super().__init__('test_move')
		self.subscription = self.create_subscription(
			JointState,
			'/joint_states',
			self.listener_callback,
			1)
		self.subscription 
		self.publisher_ = self.create_publisher(String, 'ur_driver/URScript', 1)
		timer_period = 0.008  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.max_vel_name = 1.2
		self.max_vel = self.max_vel_name
		self.joint_index = 5
		self.Q1 = [0.0, -1.5708, 0.0, -1.5708, 0, 0]
		self.i = 0
		
		self.start = time.time()
		self.vel = [0,0,0,0,0,0]
		self.vel[self.joint_index] = self.max_vel
		print('starting_loop')
		self.homming = 1
		self.data_array = []

	def listener_callback(self, data):
		global position
		global velocity
		for i in range(6):
			position[i]=data.position[i]
			velocity[i]=data.velocity[i] 


	def timer_callback(self):
		global position
		global velocity
		# time.sleep(10)
		if self.homming:
		    #hello_str = "servoj(["+str(Q1[0])+","+str(Q1[1])+","+str(Q1[2])+","+str(Q1[3])+","+str(Q1[4])+","+str(Q1[5])+"], 0, 0, 0.1, 0.1, 500)"
		    hello_str = String()
		    hello_str.data = "servoj(["+str(self.Q1[0])+","+str(self.Q1[1])+","+str(self.Q1[2])+","+str(self.Q1[3])+","+str(self.Q1[4])+","+str(self.Q1[5])+"], 0, 0, 0.1)" #, 0.1, 500)"
		    self.publisher_.publish(hello_str)
		    if (time.time() - self.start)>10.0:
		        self.homming = 0
		else:
			self.i+=1
			#print(i, vel[joint_index], velocity[joint_index])
			#new_data.append([i, vel[joint_index], velocity[joint_index]])
			if self.i%100==0 and self.i%200!=0:
				self.max_vel = - self.max_vel
				self.vel = [0,0,0,0,0,0]
				self.vel[self.joint_index] = self.max_vel
			if self.i%200==0:
				self.vel = [0,0,0,0,0,0]
			hello_str = String()
			hello_str.data = "speedj(["+str(self.vel[0])+","+str(self.vel[1])+","+str(self.vel[2])+","+str(self.vel[3])+","+str(self.vel[4])+","+str(self.vel[5])+"],"+"5.0"+",0.02)" 
			self.publisher_.publish(hello_str)
			current_time = time.time()
			self.data_array.append([current_time, self.vel[self.joint_index], position[self.joint_index], velocity[self.joint_index]])
			if (current_time - self.start)>17:
				print('finished')
				with open("real_j"+str(self.joint_index)+"_"+str(self.max_vel_name)+"data.json", "w") as fp:
						json.dump(self.data_array, fp)
				#with open("j"+str(self.joint_index)+"_"+str(max_vel_name)+"new_data.json", "w") as fp:
				#        json.dump(new_data, fp)
				rclpy.shutdown()

				
							
def main(args=None):
	rclpy.init(args=args)

	minimal_publisher = MinimalPublisher()

	rclpy.spin(minimal_publisher)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()
