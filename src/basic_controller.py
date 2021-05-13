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
		self.joint_setpoints = [0, 0, 0, 0, 0, 0, 0] #index zero = joint1
		self.joint_controls = [1, 1, 1, 1, 1, 1, 1] #0 effort control, 1 position control, 2 velocity control
		self.rot_filt = Averaging_filter(50,1)
		self.y_filt = Averaging_filter(50,0.5)
		self.z_filt = Averaging_filter(50,0.5)
		self.last_rot = 0
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
		#self.update_sticks(self.last_x,self.last_y,self.last_z)
	
		msg_cmd_effort = RigidBodyCmd()
		msg_cmd_effort.joint_cmds = self.joint_setpoints
		msg_cmd_effort.joint_cmds_types = self.joint_controls
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
		self.current_pos = np.asarray(positions)
		#print(self.current_pos)

	def calc_effort(self):
		pass

	def inv_kin(self,positions):
		pass

	def get_press(self,msg):
		stick_valuerot = msg.axes[1] #up down
		stick_valuey = msg.axes[4] #up down
		stick_valuez = msg.axes[3] #right left
		buttons = msg.buttons #LT 6, RT 7

		#base case
		self.joint_setpoints[1] = self.current_pos[1]
		self.joint_setpoints[3] = self.current_pos[3]
		#self.joint_setpoints[5] = self.current_pos[5]
		#self.joint_setpoints[6] = self.current_pos[6]


		if buttons[4] == 1:
			self.joint_setpoints[6] += -0.15
			print('Left Bumper')
		elif buttons[5] == 1:
			self.joint_setpoints[6] += 0.15
			print('Right Bumper')
		else:
			self.joint_setpoints[6] += 0

		if msg.axes[2] == -1:
			self.joint_setpoints[5] += -0.15
			print('Left Trigger')
		elif msg.axes[5] == -1:
			self.joint_setpoints[5] += 0.15
			print('Right Trigger')
		else:
			self.joint_setpoints[5] += 0
			

		if msg.axes[6] == 1:
			self.control_mode = 1
			self.joint_controls = [1, 1, 1, 1, 1, 1, 1]
		elif msg.axes[6] == -1:
			self.control_mode = 0
			self.joint_controls = [1, 0, 1, 0, 1, 0, 2]
		elif msg.axes[7] == 1: 
			self.control_mode = 2
			self.joint_controls = [1, 2, 1, 2, 1, 2, 2]
		elif msg.axes[7] == -1:
			self.update_sticks(stick_valuerot,stick_valuey,stick_valuez)
			
			return


		
		
	def update_sticks(self,x,y,z):
		rotset = self.rot_filt.filter(x)
		if abs(rotset) < 0.02:
			rotset = 0
		self.joint_setpoints[0] = rotset
		self.last_rot = rotset

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
		if self.control_mode == 1:
			joints = self.my_invk.calc(0,yset,zset)
			#limit joint 1 motion		
			if(joints[1]) > 0.35:
				joints[1] = 0.35
			self.joint_setpoints[1] = joints[1]
			self.joint_setpoints[3] = joints[2]
			#self.joint_setpoints[0] = joints[0]
		elif self.control_mode == 0:
			vels = self.calc_joint_vel()
			self.joint_setpoints[1] = vels[1]
			self.joint_setpoints[3] = vels[3]
			

	def calc_stick_vector(self):
		vector = [self.last_rot,self.last_y,self.last_z, 0, 0, 0]
		return np.array(vector).T

	def calc_joint_vel(self):
		vector = self.calc_stick_vector()
		ijac = self.my_jac.inv_jacob_calc(self.current_pos[1],self.current_pos[2],self.current_pos[3],self.current_pos[4])
		qvs = np.matmul(ijac,vector);
		return qvs

		
	def initialize(self):
		pass

	def run(self):
		rospy.spin()
	
if __name__ == '__main__':
	Controller().run()
