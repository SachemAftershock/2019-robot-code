import numpy as np
import threading
import base64
import atexit
import struct
import socket
import cv2

DEBUG = True

def dbgprint(string):
	if DEBUG:
		print(string)

class Communication:
	def __init__(self, port, address = '', cam_index = 0, max_packet_size = 65000, height = 640, width = 480):
		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.server_socket.bind((address, port))
		self.packet_size = max_packet_size

		self.frame = None

		self.height = height
		self.width = width
		self.index = cam_index

		atexit.register(self.release)

		dbgprint("Server Initialized at port {0} and address {1}".format(port, address))

	def start_streamer(self):
		threading.Thread(target = self.frame_thread).start()

	def itob(self, value):
		return struct.pack('!i', value)

	def receive_frame(self, length):
		recieved = 0
		message = b''
		while recieved < length - self.packet_size:
			_message, address = self.server_socket.recvfrom(self.packet_size)
			message += _message
			recieved += self.packet_size

		_message, address = self.server_socket.recvfrom(length % self.packet_size)
		message += _message
		
		return message 

	def frame_thread(self):
		while True:
			try:
				b_frame_length, address = self.server_socket.recvfrom(4)
				frame_length = struct.unpack('!i', b_frame_length)[0]
				dbgprint('Length recieved: {0}'.format(frame_length))

				if frame_length > 0 and frame_length < (2**31 - 1):
					self.frame = self.receive_frame(frame_length)

			except Exception as e:
				dbgprint('Exception caught in frame_thread: {0}'.format(str(e)))

	def display_thread(self):
		if not self.frame:
			return
		
		#decoded = base64.b64decode(frame)
		decoded = self.frame
		string_decoded = np.fromstring(decoded, dtype = np.uint8)
		img = cv2.imdecode(string_decoded, 1)
		try:
			if img is not None:
				img = cv2.resize(img, (self.height, self.width))
				cv2.imshow('{0}'.format(self.index), img)
			else:
				print('Frame was none')
		except Exception as e:
			dbgprint('Exception caught in display_thread: {0}'.format(str(e)))

	def release(self):
		self.server_socket.close()
		cv2.destroyAllWindows()

def main():
	comm = Communication(5804)
	
	comm.start_streamer()
	while True:
		comm.display_thread()

		if cv2.waitKey(10) & 0xFF == 113:
				break

if __name__ == '__main__':
	main()