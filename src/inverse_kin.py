import numpy as np

class inverse_kin:
	
	def __init__(self, d1,d2,d3): #d1 is height of base to link1; d2 is height from link1-3, link2 is height from 3-5
		self.d1 = d1
		self.d2 = d2
		self.d3 = d3


	def calc(self,x,y,z):
		
		r = np.sqrt((x*x)+(y*y)) #horizontal displacement in frame of arm
		#print(r)
		theta1 = np.arctan2(x,y)
		if np.isnan(theta1):
			theta1 = 0

		dz = self.d1-z
		#print(dz)
		dc = np.sqrt((dz*dz)+(r*r))

		lc = ((self.d2*self.d2)+(self.d3*self.d3)-(dc*dc))/(2*self.d2*self.d3)

		theta3 = 3.14159-np.arccos(lc)
		if np.isnan(theta3):
			theta3 = 0

		alpha = 1.5708-np.arctan2(z,r)
		#print(alpha)
		beta = ((dc*dc)+(self.d2*self.d2)-(self.d3*self.d3))/(2*dc*self.d2)
		#print(beta)

		theta2 = alpha-beta
		if np.isnan(theta2):
			theta1 = 0

		return [theta1,theta2,theta3] 

if __name__ == '__main__':
	inverse_kin().run()
