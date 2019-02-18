from StreamUtils import *
import numpy as np
import threading
import base64
import atexit
import struct
import socket
import cv2
import sys

class Camera:
	def __init__(self, index, width, height):
		self.cam = cv2.VideoCapture(index)
		self.width = width
		self.height = height
		self.index = index
		self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
		self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
		atexit.register(self.release)

	def get_frame(self):
		ret, frame = self.cam.read()
		if ret:
			return frame
		else:
			return None

	def set_prop(self, prop, value):
		self.cam.set(prop, value)

	def set_res(self, width, height):
		self.cam.release()
		self.cam = cv2.VideoCapture(self.index)
		self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
		self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

	def release(self):
		self.cam.release()

class ConfigClient:
	def __init__(self, server_address = 'localhost', server_port = 5807, packet_size = 1000):
		self.comm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.comm_socket.bind(('', 0))
		self.comm_socket.settimeout(1.0)
		self.server = (server_address, server_port)
		self.packet_size = packet_size
		self.cmd_string = ''

		atexit.register(self.release)

	def create_streamer(self):
		ret = 1
		while ret == 1:
			ret = StreamUtils.send_packet(self.comm_socket, self.server, StreamUtils.compress_string('camera'), timeout_kill = True)
		b_message, address = StreamUtils.recieve_packet(self.comm_socket)
		message = StreamUtils.decompress_string(b_message)
		items = message.split('|')
		return ClientStreamer(address[0], int(items[0].split('#')[1]), int(items[1].split('#')[1]))

	def send_message(self):
		StreamUtils.send_packet(self.comm_socket, self.server,  StreamUtils.compress_string(self.cmd_string))

	def release(self):
		self.comm_socket.close()

class ClientStreamer:
	def __init__(self, server_address, server_port, cam_index, packet_size = 1000, cap_width = 320, cap_height = 240):
		#self.comm_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		#self.comm_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, True)
		#self.comm_socket.connect((server_address, server_port))

		self.comm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.comm_socket.bind(('', 0))
		self.comm_socket.settimeout(1.0)
		self.server = (server_address, server_port)
		self.packet_size = packet_size
		self.camera = Camera(cam_index, cap_width, cap_height)
		self.cmd_string = 'width#%d|height#%d' % (cap_width, cap_height)
		self.running = True
		
		atexit.register(self.release)
		threading.Thread(target = self.streamer_thread).start()

	def streamer_thread(self):

		while self.running:
			try:
				'''
				frame = self.camera.get_frame()
				if frame is None:
					continue

				frame_bytes = StreamUtils.compress_frame(frame)
				print(len(frame_bytes))
				bframe_length = StreamUtils.itob(len(frame_bytes))
				#frame_bytes = StreamUtils.compress_frame(frame)

				self.comm_socket.sendall(StreamUtils.itob(263))
				self.comm_socket.sendall(bframe_length)
				self.comm_socket.sendall(frame_bytes)
				'''

				frame = self.camera.get_frame()
				if frame is None:
					continue

				frame_bytes = StreamUtils.compress_frame(frame)
				bframe_length = StreamUtils.itob(len(frame_bytes))

				self.comm_socket.sendto(bframe_length, self.server)

				bserver_info_length, address = self.comm_socket.recvfrom(4)
				server_info_length = StreamUtils.btoi(bserver_info_length)

				bserver_info, address = self.comm_socket.recvfrom(server_info_length)
				server_info = StreamUtils.decompress_string(bserver_info)

				sent = 0
				while sent < len(frame_bytes):
					self.comm_socket.sendto(frame_bytes[sent:sent + self.packet_size], self.server)
					sent += self.packet_size

				if len(server_info) > 0:
					self.parse_commands(server_info.split('|'))


				#self.comm_socket.sendto(self.p, self.server)
				#frame_bytes = StreamUtils.compress_frame(frame)
				#self.comm_socket.sendto(frame_bytes, self.server)
				'''	
				frame_bytes = StreamUtils.compress_frame(frame)
				info_bytes = StreamUtils.compress_string(self.cmd_string)

				bframe_length = StreamUtils.itob(len(frame_bytes))
				self.comm_socket.sendto(bframe_length, self.server)

				bserver_info_length, address = self.comm_socket.recvfrom(4)
				server_info_length = StreamUtils.btoi(bserver_info_length)

				binfo_length = StreamUtils.itob(len(info_bytes))
				self.comm_socket.sendto(binfo_length, self.server)

				bserver_info, address = self.comm_socket.recvfrom(server_info_length)
				server_info = StreamUtils.decompress_string(bserver_info)

				if len(server_info) > 0:
					self.parse_commands(server_info.split('|'))

				bpayload = info_bytes + frame_bytes
				sent = 0
				while sent < len(bpayload):
					self.comm_socket.sendto(bpayload[sent:sent + self.packet_size], self.server)
					sent += self.packet_size
				'''
			except Exception as e:
				if str(e) == 'timed out':
					pass
				else:
					StreamUtils.dbgprint('Exception caught in ClientStreamer::streamer_thread:', str(e))

	def parse_commands(self, commands):
		res = [self.camera.width, self.camera.height]
		for command in commands:
			data = command.split('#')
			if data[0] == 'width':
				res[0] = int(data[1])
			elif data[0] == 'height':
				res[1] = int(data[1])
			elif data[0] == 'kill':
				self.running = False
		self.camera.set_res(res[0], res[1])

	def release(self):
		self.comm_socket.close()


def main():
	config = ConfigClient()
	streams = [config.create_streamer()]#[config.create_streamer(), config.create_streamer(), config.create_streamer(), config.create_streamer(), config.create_streamer()]
	running = True
	while running:
		config.send_message()

		for stream in streams:
			if not stream.running:
				running = False
	

if __name__ == '__main__':
	main()
