
class Averaging_filter:
	
	def __init__(self,size,gain):
		self.gain = gain
		self.length = size
		self.input_buffer = [0]*size
		self.current_index = 0

	def filter(self,value):
		self.input_buffer[self.current_index] = value
		self.current_index += 1
		if self.current_index >= self.length:
			self.current_index = 0
		return self.gain*sum(self.input_buffer)/self.length

if __name__ == '__main__':
	averaging_filter().run()
