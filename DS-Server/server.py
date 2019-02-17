from StreamUtils import *
import numpy as np
import threading
import base64
import atexit
import struct
import socket
import bimpy
import math
import time
import cv2

servers = []
available_ports = [(5810, 0)]

class ServerStreamer:
	def __init__(self, port, cam_id):
		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.server_socket.bind(('', port))
		self.server_socket.settimeout(5.0)
		self.frame = None
		self.MAX_INT = (2 ** 31) - 1
		self.packet_size = 1000
		self.port = port
		self.cam_id = cam_id
		self.first = True
		self.running = True
		self.cmd_string = ''

		self.stream_width = bimpy.Float(320)
		self.stream_height = bimpy.Float(240)

		atexit.register(self.release)
		threading.Thread(target = self.reciever_thread).start()

	def reciever_thread(self):
		while self.running:
			try:
				bframe_length, address = self.server_socket.recvfrom(4)
				frame_length = struct.unpack('!i', bframe_length)[0]

				recieved = 0
				bframe = b''
				while recieved < frame_length - self.packet_size:
					byte_message, address = self.server_socket.recvfrom(self.packet_size)
					bframe += byte_message
					recieved += self.packet_size

				btail_message, address = self.server_socket.recvfrom(frame_length % self.packet_size)
				bframe += btail_message

				self.frame = StreamUtils.decompress_frame(bframe)
				self.first = False

			except Exception as e:
				if str(e) == 'timed out':
					break
				else:
					StreamUtils.dbgprint('Exception caught in ServerStreamer::reciever_thread:', str(e))

		self.destroy()

	def update_settings(self):
		self.cmd_string += 'width#%d|height#%d' % (int(self.stream_width.value), int(self.stream_height.value))

	def reset_settings(self):
		for cmd in self.client_string.split('|'):
			data = cmd.split('#')
			if len(data) != 2:
				return
			elif data[0] == 'width':
				self.stream_width.value = int(data[1])
			elif data[0] == 'height':
				self.stream_height.value = int(data[1])

	def destroy(self):
		self.running = False
		servers.remove(self)
		available_ports.append((self.port, self.cam_id))
		atexit.unregister(self.release)
		self.release()

	def release(self):
		self.server_socket.close()
		cv2.destroyAllWindows()

class ConfigServer:
	def __init__(self, port = 5807, packet_size = 1000):
		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.server_socket.bind(('', port))
		self.server_socket.settimeout(0.001)
		self.packet_size = packet_size

		atexit.register(self.release)
		self.is_running = True

	def process_command(self):
		while True:
			command, address = StreamUtils.recieve_packet(self.server_socket, timeout_kill = True)
			if command == 1 and address == 1:
				continue

			command_string = StreamUtils.decompress_string(command)

			if len(command_string) > 0:
				self.handle_command(command_string.split('|'), address)

	def handle_command(self, info, address):
		if info[0] == 'camera':
			if len(available_ports) > 0:
				ports = available_ports.pop()
			else:
				StreamUtils.dbgprint('No available ports')
				return
			b_payload = StreamUtils.compress_string("port#%d|cam#%d" % ports)
			servers.append(ServerStreamer(ports[0], ports[1]))
			StreamUtils.send_packet(self.server_socket, address, b_payload)
		else:
			StreamUtils.dbgprint('Unknown command recieved:', info)

	def release(self):
		self.server_socket.close()

class GUI:
	def __init__(self, width, height, title):
		self.width = width
		self.height = height

		self.ctx = bimpy.Context()
		self.ctx.init(width, height, title)

	def render(self, config_server):
		if self.ctx.should_close():
			return

		self.ctx.new_frame()
		bimpy.set_next_window_pos(bimpy.Vec2(0, 0), bimpy.Condition.Once)
		bimpy.set_next_window_size(bimpy.Vec2(self.width, self.height), bimpy.Condition.Once)
		bimpy.begin("", flags = bimpy.WindowFlags.NoResize | bimpy.WindowFlags.NoTitleBar | bimpy.WindowFlags.NoMove)

		self.draw_gui(config_server)

		bimpy.end()
		self.ctx.render()

	def draw_gui(self, config_server):

		frames = []

		for server in servers:
			bimpy.slider_float("Stream Width %d" % server.cam_id, server.stream_width, 320, 640)
			bimpy.slider_float("Stream Height %d" % server.cam_id, server.stream_height, 240, 480)

			if bimpy.button("Update %d" % server.cam_id):
				server.update_settings()

			bimpy.same_line()
			if bimpy.button("Reset %d" % server.cam_id):
				server.reset_settings()

			bimpy.new_line()
			frames.append(server.frame)
		stitched = StreamUtils.stitch_frames(frames)
		if stitched is not None:
			cv2.imshow('Stitched', stitched)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				exit(1)


def main():
	gui = GUI(500, 400, "Sachem Aftershock Camera Streamer")
	config_server = ConfigServer()
	
	threading.Thread(target = config_server.process_command).start()
	while config_server.is_running:
		gui.render(config_server)
		

if __name__ == '__main__':
	main()
