#!/usr/bin/env python
#import evdev
#from evdev import InputDevice, categorize, ecodes
from inverse_kin import inverse_kin
import numpy as np
from jacobian import jacobian


my_kin = inverse_kin(0,10,10)

ans = my_kin.calc(0,18.32,2.41)
print(ans)

a= np.random.randn(9,6)
print(a)

b = [[1, 2, 3, 4,],[2, 4, 5, 6]]
print(b)

c = np.linalg.pinv(b)
print(c)

d = np.transpose(b)
print(d)

my_jac = jacobian(10,10,10,10)
e = my_jac.calc(0.7,0.7,0.7,0.7)
print(e)
#gamepad = InputDevice('/dev/input/event9')

#print(gamepad)

#for event in gamepad.read_loop():
	#print(categorize(event))
	#if event.value == 1: #ecodes.EV_ABS:
		#if event.code == 3:
		#print(event.value)


#for event in self.gamepad.read_loop():
			#print(event)
			#if event.value == ecodes.EV_KEY:
				#print(event.code)
				#if event.code == 1:
					#print(event.code)
				#	if event.code == 312: #Left Trigger
					#	self.joint_setpoints[4] = -10
					#	print('Left Trigger')
					#elif event.code == 313: #Right trigger
					#	self.joint_setpoints[4] = 10
					#	print('Right Trigger')
					#else: 
					#	self.joint_setpoints[4] = 0
			#break


