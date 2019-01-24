import numpy as np
import base64
import atexit
import struct
import socket
import cv2

DEBUG = True

def dbgprint(string):
	if DEBUG:
		print(string)

class Camera:
	def __init__(self, index, height, width):
		self.cam = cv2.VideoCapture(index)
		self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
		self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)

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
	def __init__(self, address, port, cam_index, max_packet_index = 65000, stream_height = 320, stream_width = 240):
		self.comm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.comm_socket.settimeout(1.0)
		self.server = (address, port)

		self.packet_size = max_packet_index

		self.cam = Camera(cam_index, stream_height, stream_width)
		self.stream_height = stream_height
		self.stream_width = stream_width

		atexit.register(self.release)

	def get_compressed_frame(self):
		return cv2.imencode('.jpg', self.cam.get_frame())[1].tostring()

	def itob(self, value):
		return struct.pack('!i', value)

	def send_frame(self, compressed_frame):
		sent = 0
		while sent < len(compressed_frame):
			self.comm_socket.sendto(compressed_frame[sent:sent + self.packet_size], self.server)
			sent += self.packet_size

	def sender_thread(self):
		try:
			compressed_frame = self.get_compressed_frame()
			b_frame_length = self.itob(len(compressed_frame))

			self.comm_socket.sendto(b_frame_length, self.server)
			self.send_frame(compressed_frame)
		except Exception as e:
			dbgprint('Exception caught in sender_thread: {0}'.format(str(e)))

	def release(self):
		self.comm_socket.close()
		self.cam.release()

def main():
	comm = Communication('10.2.63.5', 5804, 0)
	while True:
		comm.sender_thread()

if __name__ == '__main__':
	main()