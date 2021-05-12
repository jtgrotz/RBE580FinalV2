import numpy as np

class jacobian:
	
	def __init__(self, d1,d2,d3,d4):
		self.d02 = d1
		self.d24 = d2
		self.d46 = d3
		self.d67 = d4


	def calc(self,q1,q2,q3,q4):
		
		J11 = -(np.sin(q1)*(self.d24*np.sin(q2) + self.d46*np.sin(q2-q4) + self.d67*np.sin(q2 - q4)))
		J12 = np.cos(q1)*(self.d24*np.cos(q2) + self.d46*np.cos((q2 - q4)) + self.d67*np.cos((q2 - q4)))
		J13 = np.sin(q1)*np.sin(q4)*(self.d46 + self.d67)
		J14 = -(np.cos(q1)*np.cos((q2 - q4))*(self.d46 + self.d67))
		J15 = 0 
		J16 = (self.d67*np.cos(q1)*np.cos((q2 - q4)))
		J17 = 0

		J21 = (np.cos(q1)*(self.d24*np.sin(q2) + self.d46*np.sin((q2 - q4)) + self.d67*np.sin((q2 - q4))))
		J22 =  (np.sin(q1)*(self.d24*np.cos(q2) + self.d46*np.cos((q2 - q4)) + self.d67*np.cos((q2 - q4))))
		J23 =  -(np.cos(q1)*np.sin(q4)*(self.d46 + self.d67))
		J24 = -(np.sin(q1)*np.cos((q2 - q4))*(self.d46 + self.d67))
		J25 = 0
		J26 = (self.d67*np.sin(q1)*np.cos((q2 - q4)))
		J27 = 0

		J31 = 0 
		J32 = -(self.d24*np.sin(q2) + self.d46*np.sin((q2 - q4)) + self.d67*np.sin((q2 - q4)))
		J33 = 0
		J34 =  (np.sin((q2 - q4))*(self.d46 + self.d67))
		J35 = 0
		J36 = -(self.d67*np.sin((q2 - q4)))
		J37 = 0

		J41 = 0
		J42 = 0
		J43 = 0
		J44 = 0
		J45 = 0
		J46 = 0
		J47 = 0

		J51 = 1
		J52 = 0
		J53 = -1
		J54 = 0 
		J55 = 1
		J56 = 0
		J57 = 0

		J61 = 0
		J62 = 1
		J63 = 0
		J64 = 1
		J65 = 0
		J66 = 1
		J67 = 1

		Jac = [[J11,J12,J13,J14,J15,J16,J17],[J21,J22,J23,J24,J25,J26,J27],[J31,J32,J33,J34,J35,J36,J37],[J41,J42,J43,J44,J45,J46,J47],[J51,J52,J53,J54,J55,J56,J57],[J61,J62,J63,J64,J65,J66,J67]]

		return Jac


	def inv_jacob_calc(self,q1,q2,q3,q4):
		a = self.calc(q1,q2,q3,q4)
		b = np.linalg.pinv(a)

		return b


	def jacob_transpose(self,q1,q2,q3,q4):
		a = self.calc(q1,q2,q3,q4)
		b = np.transpose(a)
 
		

if __name__ == '__main__':
	inverse_kin().run()
