from win32api import GetSystemMetrics
import numpy as np
import threading
import base64
import atexit
import struct
import socket
import math
import cv2

class Communication:
	def __init__(self, port):
		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.server_socket.bind(('', port))
		self.frame = None
		self.MAX_INT = (2 ** 31) - 1
		self.packet_size = 65000

	def recv_frame(self):
		while True:
			try:
				_length, address = self.server_socket.recvfrom(4)
				#print(message, 'recieved from', address)
				length = struct.unpack("!i", _length)[0]

				if length > 0 and length < self.MAX_INT:
					self.server_socket.sendto(b'a', address)

					recieved = 0
					message = b''
					while recieved < length - self.packet_size:
						_message, address = self.server_socket.recvfrom(self.packet_size)
						message += _message
						recieved += self.packet_size

					_message, address = self.server_socket.recvfrom(length % self.packet_size)
					message += _message
					print(len(message))
					self.frame = message

				else:
					#print(length, 'is not good')
					continue
			except Exception as e:
				#print('exception', e)
				continue
				

	def get_frame(self):
		return self.frame

	def release(self):
		self.server_socket.close()

class Viewer:
	def __init__(self, index, height, width):
		self.index = index
		self.height = int(height)
		self.width = int(width)

	def process_frame(self, frame):
		if not frame:
			return
		
		#decoded = base64.b64decode(frame)
		decoded = frame
		string_decoded = np.fromstring(decoded, dtype = np.uint8)
		img = cv2.imdecode(string_decoded, 1)
		'''
		try:
			if img is not None:
				img = cv2.resize(img, (self.height, self.width))
				cv2.imshow('{0}'.format(self.index), img)
			else:
				print('Frame was none')
		except Exception as e:
			print('Exception caught in display_thread: {0}'.format(str(e)))
		'''

		return img

def stitch_frames(_frames):

	frames = []
	for _frame in _frames:
		if _frame is not None:
			frames.append(_frame)

	if len(frames) == 0:
		return None

	num_rows = round(math.sqrt(len(frames)))
	num_columns = math.ceil(len(frames) / num_rows)
	resolution = (int(GetSystemMetrics(0) / num_columns), int(GetSystemMetrics(1) / num_rows)) 
	resized = [cv2.resize(frame, resolution) for frame in frames]

	rows = []
	for i in range(num_rows - 1):
		rows.append(np.hstack(tuple(resized[i * num_columns:(i+1) * num_columns])))

	dif = num_rows * num_columns - len(frames)
	if dif > 0:
		for x in range(dif):
			resized.append(np.zeros_like(resized[-1]))
	rows.append(np.hstack(tuple(resized[(num_rows - 1) * num_columns:num_rows * num_columns])))

	return np.vstack(tuple(rows))

def main():
	comm = Communication(5809)
	comm1 = Communication(5810)
	comm2 = Communication(5808)

	viewer = Viewer(0, 640, 480)
	viewer1 = Viewer(1, 640, 480)
	viewer2 = Viewer(2, 640, 480)

	atexit.register(comm.release)
	atexit.register(comm1.release)
	atexit.register(comm2.release)

	threading.Thread(target = comm.recv_frame).start()
	threading.Thread(target = comm1.recv_frame).start()
	threading.Thread(target = comm2.recv_frame).start()

	while True:
		try:
			frame = viewer.process_frame(comm.get_frame())
			frame1 = viewer1.process_frame(comm1.get_frame())
			frame2 = viewer2.process_frame(comm2.get_frame())


			stitched_frame = stitch_frames([frame, frame1, frame2])
			if stitched_frame is not None:
				cv2.imshow('Robot Frames', stitched_frame)

			if cv2.waitKey(10) & 0xFF == 113:
				break
		except KeyboardInterrupt:
			return;

if __name__ == '__main__':
	main()