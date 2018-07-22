import numpy as np

class KF():
	def __init__(self,n):
		self.n = n
		self.I = np.matrix(np.eye(n))
		self.x = None
		self.P = None
		self.F = None
		self.Q = None

	def predict(self):
	        
		self.x = self.F * self.x
		self.P = self.F * self.P * self.F.T + self.Q
	    
	def update(self, z, H, Hx, R):

		y = z - Hx
		PHt = self.P * H.T
		S = H * PHt + R    
		K = PHt * (S.I)

		self.x = self.x + K * y
		self.P = (self.I - K * H) * self.P