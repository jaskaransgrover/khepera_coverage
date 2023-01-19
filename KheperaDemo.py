import numpy as np
import rospy
import matlab.engine

# Import message file
from khepera_communicator.msg import K4_controls, SensorReadings
from geometry_msgs.msg import TransformStamped


from constants import *
from helpers import quaternion_to_euler


class KheperaDemo:
	def __init__(self):
		pass


	def initialize_publishers(self):
		self.pubs = []
		for i in range(self.N_active_robots):
			self.pubs.append(rospy.Publisher(
				'K4_controls_' + str(self.robot_ips[i]),
				K4_controls,
				queue_size = 10
			))


	def initialize_subscribers(self):
		def callback(data, args):
			theta = quaternion_to_euler(
				data.transform.rotation.x,
				data.transform.rotation.y,
				data.transform.rotation.z,
				data.transform.rotation.w
			)
			self.robot_poses[args, 0] = data.transform.translation.x
			self.robot_poses[args, 1] = data.transform.translation.y
			self.robot_poses[args, 2] = theta

		self.subs = []
		for i in range(self.N_active_robots):
			self.subs.append(rospy.Subscriber(
				'vicon/k' + self.robot_ips[i] + '/k' + self.robot_ips[i],
				TransformStamped,
				callback, i
			))
	

	def connect_to_robots(self, robot_ips):
		self.robot_ips = robot_ips
		self.N_active_robots = len(self.robot_ips)
		self.robot_poses = np.zeros((self.N_active_robots, 3))

		rospy.init_node('Central_Algorithm', anonymous=True)

		self.initialize_publishers()
		self.initialize_subscribers()

		self.rate = rospy.Rate(20)
	

	def alive(self):
		return not rospy.is_shutdown()
	

	def get_robot_poses(self):
		"Get current x, y, and theta for all robots (shape is 3 x N)"
		return self.robot_poses.T.copy()


	def send_control_signals(self, unicycle_controls):
		"""
		Send control signals to the robots.
		Argument `unicycle_controls` should have shape 2 x N (1st row speeds, 2nd row thetas)
		"""
		for i in range(self.N_active_robots):
			V, W = unicycle_controls[:,i]
			control_msgs = K4_controls()
			control_msgs.ctrl_V = V * 1000		# convert meters/s to mm/s (presumably)
			control_msgs.ctrl_W = W
			self.pubs[i].publish(control_msgs)
	

	def step(self):
		self.rate.sleep()
	

	# def run_controller(self, all_robot_control):
	# 	"""
	# 	Run the system using the specified controller.
		
	# 	This function handles all ROS initialization and interfacing.
	# 	"""	
		
	# 	# Break out `all_robot_control` into a function for a single robot (apparently useful for distributed controllers)
	# 	def control_for_one_robot(i, dog_flag):	
	# 		"""
	# 		Compute control for robot `i` using `all_robot_control(...)`.
	# 		If `dog_flag` is set, the cached values in `all_velocities` will be used instead.
	# 		"""			
	# 		if not dog_flag:
	# 			all_poses = np.vstack((self.XR, self.YR, self.THR))
	# 			self.all_velocities = all_robot_control(all_poses)
	# 			dog_flag = True
			
	# 		V = self.all_velocities[0,i]
	# 		W = self.all_velocities[1,i]
	# 		return V, W, dog_flag


	# 	# Main control loop
	# 	while not rospy.is_shutdown():
	# 		control_msgs = K4_controls()

	# 		# update poses (synchronized)
	# 		for i in range(self.N_active_robots):
	# 			self.XR[i] 	= self.last_data[i,0]
	# 			self.YR[i] 	= self.last_data[i,1]
	# 			self.THR[i]	= self.last_data[i,2]

	# 		# compute control signals and send to ROS (via publishers)
	# 		dog_flag = False	# track if all_robot_control has already been called this loop 
	# 		for j in range(self.N_active_robots):
	# 			V, W , dog_flag	 = control_for_one_robot(j, dog_flag)
	# 			control_msgs.ctrl_V = V * 1000
	# 			control_msgs.ctrl_W = W
	# 			self.pubs[j].publish(control_msgs)
			
			
