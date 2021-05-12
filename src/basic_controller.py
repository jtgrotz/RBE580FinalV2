#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from ambf_msgs.msg import RigidBodyState
from ambf_msgs.msg import RigidBodyCmd
from geometry_msgs.msg import Pose
#from evdev import InputDevice, categorize, ecodes
from sensor_msgs.msg import Joy
from averaging_filter import Averaging_filter
from inverse_kin import inverse_kin
from jacobian import jacobian

class Controller:

	def __init__(self):
		self.error = [0,0,0,0,0,0,0]
		self.prev_error = [0,0,0,0,0,0,0]
		self.errordot = [0,0,0,0,0,0,0]
		self.errorsum = [0,0,0,0,0,0,0]
		self.previous_pos = [0,0,0,0,0,0,0]
		self.current_pos = [0,0,0,0,0,0,0]
		self.kp = 0
		self.ki = 0
		self.kd = 0
		self.joint_setpoints = [0, 0.7, 0, 0.7, 0, 0.7,0] #index zero = joint1
		self.joint_controls = [2, 2, 2, 2, 1, 0, 0] #0 effort control, 1 position control, 2 velocity control
		self.x_filt = Averaging_filter(1000,1)
		self.y_filt = Averaging_filter(1000,1)
		self.z_filt = Averaging_filter(1000,1)
		self.last_x = 0
		self.last_y = 0
		self.last_z = 0
		self.my_jac = jacobian(0.103,0.403,0.404,0.257) #link lengths
		self.my_invk = inverse_kin(0.103,0.403,0.404) #link lengths
		self.max_velocity = 3
		self.control_mode = 1 #0 for torque, 1 for position, 2 for velocity

	
		rospy.init_node('Controller', anonymous=True)

		rospy.Subscriber('/ambf/env/base/State', RigidBodyState, self.update_effort)
		rospy.Subscriber('/joy', Joy, self.get_press)

		self.cmd_effort = rospy.Publisher('/ambf/env/base/Command',RigidBodyCmd, queue_size =10)
		
		#self.update_pos()
		print('Basic Controller Initialized')

		rospy.sleep(1)

	def update_effort(self, msg):
		#save pos
		pos = msg.joint_positions
		self.update_pos(pos)
		#updating stick value
		self.update_sticks(self.last_x,self.last_y,self.last_z)
	
		msg_cmd_effort = RigidBodyCmd()
		msg_cmd_effort.joint_cmds = self.joint_setpoints	#[joints[0],joints[0],0,-joints[2],0,0,0]
		msg_cmd_effort.joint_cmds_types = [1,1,1,1,1,1,1]#self.joint_controls
		msg_cmd_effort.publish_children_names = True
		msg_cmd_effort.publish_joint_names = True
		msg_cmd_effort.publish_joint_positions = True
		self.cmd_effort.publish(msg_cmd_effort)


	def calc_error(self,positions):
		for x in range(0,6):
			self.error[x] = positions[x]-previous_pos[x]
			self.errordot[x] = self.error[x]-self.prev_error[x]
			self.errorsum[x] = self.errorsum[x]+self.error[x]
			self.prev_error[x] = self.error[x]

	def update_pos(self,positions):
		self.previous_pos = self.current_pos
		self.current_pos = positions
		print(self.current_pos)

	def calc_effort(self):
		pass

	def inv_kin(self,positions):
		pass

	def get_press(self,msg):
		stick_valuex = msg.axes[3] #right left
		stick_valuey = msg.axes[4] #up down
		stick_valuez = msg.axes[1] #up down
		buttons = msg.buttons #LT 6, RT 7

		if buttons[4] == 1:
			self.joint_setpoints[6] = -0.5
			print('Left Bumper')
		elif buttons[5] == 1:
			self.joint_setpoints[6] = 0.5
			print('Right Bumper')
		else:
			self.joint_setpoints[6] = 0

		if buttons[6] == 1:
			self.joint_setpoints[5] = -0.5
			print('Left Trigger')
		elif buttons[7] == 1:
			self.joint_setpoints[5] = 0.5
			print('Right Trigger')
		else:
			self.joint_setpoints[5] = 0
			

		if msg.axes[6] == 1:
			self.control_mode = 1
		elif msg.axes[6] == -1:
			self.control_mode = 0
		elif msg.axes[7] == 1: 
			self.control_mode = 2
		elif msg.axes[7] == -1:
			self.joint_setpoints = aslist(self.current_pos)
			return

		self.update_sticks(stick_valuex,stick_valuey,stick_valuez)

		
	def update_sticks(self,x,y,z):
		xset = self.x_filt.filter(x)
		if abs(xset) < 0.02:
			xset = 0
		self.joint_setpoints[0] = xset
		self.last_x = xset

		yset = self.y_filt.filter(y)
		if abs(yset) < 0.02:
			yset = 0
		#self.joint_setpoints[1] = yset
		self.last_y = yset

		zset = self.z_filt.filter(z)
		if abs(zset) < 0.02:
			zset = 0
		#self.joint_setpoints[0] = zset
		self.last_z = zset

		joints = self.my_invk.calc(0,0,zset)
		#print(joints)
		self.joint_setpoints[1] = joints[1]
		self.joint_setpoints[3] = joints[2]
		#self.joint_setpoints[0] = joints[0]

	def calc_stick_vector(self):
		vector = [self.xlast,self.ylast,self.last_z]
		return vector

	def calc_joint_vel(self):
		vector = self.calc_stick_vector()
		ijac = my_jac.inv_jacob_calc(q[1],q[2],q[3],q[4])
		qvs = np.matmul(ijac,vector);

		
	def initialize(self):
		pass

	def run(self):
		rospy.spin()
	
if __name__ == '__main__':
	Controller().run()
