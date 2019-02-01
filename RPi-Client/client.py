import numpy as np
import base64
import atexit
import struct
import socket
import cv2

class Camera:
	def __init__(self, index, width, height):
		self.cam = cv2.VideoCapture(index)
		self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
		self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

	def get_frame(self):
		ret, frame = self.cam.read()
		if ret:
			return frame
		else:
			return None

	def set_prop(self, prop, value):
		self.cam.set(prop, value)

	def release(self):
		self.cam.release()

class Communication:
	def __init__(self, address, port):
		self.comm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.comm_socket.settimeout(1.0)
		self.server = (address, port)
		self.packet_size = 65000

	def compress(self, frame):
		#return base64.b64encode(cv2.imencode('.jpg', frame)[1])
		if frame is None:
			return None
		return cv2.imencode('.jpg', frame)[1].tostring()
		
	def itob(self, value):
		return struct.pack('!i', value)

	def send_frame(self, frame):
		compressed_frame = self.compress(frame)
		if compressed_frame is None:
			return
			
		length = self.itob(len(compressed_frame))
		print(len(compressed_frame))
		#streams = self.itob(len(frame))
		#print('Client length:', length, len(compressed_frame))
		self.comm_socket.sendto(length, self.server)
		#self.comm_socket.sendto(streams, self.server)

		try:
			data, server = self.comm_socket.recvfrom(1)
			#print(data, 'recieved from', server)
		except socket.timeout:
			#print('timeout')
			return
		except KeyboardInterrupt:
			return;

		sent = 0
		while sent < len(compressed_frame):
			self.comm_socket.sendto(compressed_frame[sent:sent + self.packet_size], self.server)
			sent += self.packet_size

	def release(self):
		self.comm_socket.close()

def main():
	cam = Camera(0, 320, 240)
	cam1 = Camera(1, 320, 240)
	cam2 = Camera(2, 320, 240)

	comm = Communication('10.2.63.5', 5809)
	comm1 = Communication('10.2.63.5', 5810)
	comm2 = Communication('10.2.63.5', 5808)

	atexit.register(cam.release)
	atexit.register(comm.release)

	atexit.register(cam1.release)
	atexit.register(comm1.release)

	atexit.register(cam2.release)
	atexit.register(comm2.release)

	while True:
		comm.send_frame(cam.get_frame())
		comm1.send_frame(cam1.get_frame())
		comm2.send_frame(cam2.get_frame())

if __name__ == '__main__':
	main()
