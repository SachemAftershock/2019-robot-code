from StreamUtils import *
from scipy import ndimage
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
available_ports = [(5809, 0), (5810, 1), (5811, 2)]

class ServerStreamer:
	def __init__(self, port, cam_id):
		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.server_socket.bind(('', port))
		self.server_socket.settimeout(5.0)
		self.frame = None
		self.orientation = 0
		self.MAX_INT = (2 ** 31) - 1
		self.packet_size = 1000
		self.port = port
		self.cam_id = cam_id
		self.first = True
		self.running = True
		self.cmd_string = ''
		self.last_timestamp = 0
		self.process = False
		#self.V = 0
		#self.H = 0

		self.stream_width = bimpy.Float(320)
		self.stream_height = bimpy.Float(240)

		atexit.register(self.release)
		threading.Thread(target = self.reciever_thread).start()

	def reciever_thread(self):
		while self.running:
			try:
				bframe_length, address = self.server_socket.recvfrom(4)
				frame_length = struct.unpack('!i', bframe_length)[0]

				info_bytes = StreamUtils.compress_string(self.cmd_string)
				binfo_length = StreamUtils.itob(len(info_bytes))
				self.cmd_string = ''

				self.server_socket.sendto(binfo_length, address)
				self.server_socket.sendto(info_bytes, address)

				recieved = 0
				bframe = b''
				while recieved < frame_length - self.packet_size:
					byte_message, address = self.server_socket.recvfrom(self.packet_size)
					bframe += byte_message
					recieved += self.packet_size

				btail_message, address = self.server_socket.recvfrom(frame_length % self.packet_size)
				bframe += btail_message

				self.frame = StreamUtils.decompress_frame(bframe)
				self.orientation %= 360
				self.last_timestamp = time.time()
				self.first = False

			except Exception as e:
				if str(e) == 'timed out':
					break
				else:
					StreamUtils.dbgprint('Exception caught in ServerStreamer::reciever_thread:', str(e))

		self.destroy()

	def update_settings(self):
		self.cmd_string += '|width#%d|height#%d' % (int(self.stream_width.value), int(self.stream_height.value))

	def destroy(self):
		self.running = False
		servers.remove(self)
		available_ports.append((self.port, self.cam_id))
		self.release()

	def kill(self):
		tmp = self.last_timestamp
		self.cmd_string += '|kill'
		while tmp == self.last_timestamp:
			pass

		self.running = False

	def release(self):
		self.server_socket.close()
		cv2.destroyAllWindows()

class FrameViewer:
	def __init__(self):
		Ty = [19.0, 0.3, -11.71]
		d_measured = [58.25, 8.0, 0.0]
		Tx_measured = [0.0, 6.0, 18.0]
		Fy = [0, 50, 100]
		Fx_measured = [45, 25, 10]

		self.tape_coefficients = np.polyfit(Fy, Fx_measured, 2)
		self.tape_polynomial_function = np.poly1d(self.tape_coefficients)

		self.limelight_Tx_coefficients = np.polyfit(Ty, Tx_measured, 2)
		self.Tx_polynomial_function = np.poly1d(self.limelight_Tx_coefficients)

		self.limelight_distance_coefficients = np.polyfit(Ty, d_measured, 2)
		self.distance_polynomial_function = np.poly1d(self.limelight_distance_coefficients)

	def get_min_distance(centroids, frame_height):
		sorted_centroids = sorted(centroids, key = lambda pt: abs(pt[0] - polynomial_function(pt[1] / frame_height)))
		return abs(sorted_centroids[0][0] - polynomial_function(sorted_centroids[0][1] / frame_height))

	def render(self, identify, min_aspect_ratio):
		frames = []
		for server in servers:
			if server.frame is not None:
				frame = ndimage.rotate(server.frame, server.orientation)
				if identify:
					cv2.putText(frame, '%d' % server.cam_id, (int(frame.shape[0] / 2.), int(frame.shape[1] / 2.)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
				
				frames.append(frame)

				if server.process:
					overlay, centroids = StreamUtils.process_image(frame, min_aspect_ratio.value)
					frames.append(overlay)
					print(centroids)
			
		stitched = StreamUtils.stitch_frames(frames)
		if stitched is not None:
			cv2.imshow('Stitched', stitched)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				exit(1)

class ConfigServer:
	def __init__(self, port = 5807, packet_size = 1000):
		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.server_socket.bind(('', port))
		self.server_socket.settimeout(0.001)
		self.packet_size = packet_size

		atexit.register(self.release)
		self.running = True

	def process_command(self):
		while self.running:
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
		self.identify = False

		self.ctx = bimpy.Context()
		self.ctx.init(width, height, title)

		self.min_aspect_ratio = bimpy.Float(0.0)

	def render(self, config_server):
		if self.ctx.should_close():
			return 0

		self.ctx.new_frame()
		bimpy.set_next_window_pos(bimpy.Vec2(0, 0), bimpy.Condition.Once)
		bimpy.set_next_window_size(bimpy.Vec2(self.width, self.height), bimpy.Condition.Once)
		bimpy.begin("", flags = bimpy.WindowFlags.NoResize | bimpy.WindowFlags.NoTitleBar | bimpy.WindowFlags.NoMove)

		self.draw_gui(config_server)

		bimpy.end()
		self.ctx.render()

		return 1

	def draw_gui(self, config_server):

		frames = []

		if bimpy.button("Load Profile"):
			with open('config.txt', 'r') as f:
				lines = f.readlines()
				for index, server in enumerate(servers):
					items = lines[index].split(' ')
					server.orientation = int(items[0])
					server.process = int(items[1])
		bimpy.same_line()

		if bimpy.button("Save Profile"):
			with open('config.txt', 'w') as f:
				for server in servers:
					f.write('%d %d\n' % (server.orientation, server.process))
		bimpy.same_line()

		if bimpy.button("Identify"):
				self.identify = not self.identify
		bimpy.same_line()

		if bimpy.button("Print ports"):
			print(available_ports)

		bimpy.slider_float("Min Aspect Ratio", self.min_aspect_ratio, 0.0, 10.0)
		bimpy.new_line()

		for server in servers:
			bimpy.slider_float("Stream Width %d" % server.cam_id, server.stream_width, 0, 640)
			bimpy.slider_float("Stream Height %d" % server.cam_id, server.stream_height, 0, 480)

			if bimpy.button("Update %d" % server.cam_id):
				server.update_settings()
			bimpy.same_line()

			if bimpy.button("Add Processing %d" % server.cam_id):
				server.process = True
			bimpy.same_line()

			if bimpy.button("Remove Processing %d" % server.cam_id):
				server.process = False

			if bimpy.button("Rotate 90 %d" % server.cam_id):
				server.orientation += 90
			bimpy.same_line()

			if bimpy.button("Rotate -90 %d" % server.cam_id):
				server.orientation -= 90
			bimpy.same_line()

			if bimpy.button("Rotate 180 %d" % server.cam_id):
				server.orientation += 180

			'''
			if bimpy.button("V+ %d" % server.cam_id):
				server.V += 1
			bimpy.same_line()

			if bimpy.button("V- %d" % server.cam_id):
				server.V -= 1
			bimpy.same_line()

			if bimpy.button("H+ %d" % server.cam_id):
				server.H += 1
			bimpy.same_line()

			if bimpy.button("H- %d" % server.cam_id):
				server.H -= 1
			'''

			bimpy.new_line()
			bimpy.new_line()

def main():
	gui = GUI(600, 600, "Sachem Aftershock Camera Streamer")
	viewer = FrameViewer()
	config_server = ConfigServer()
	
	threading.Thread(target = config_server.process_command).start()
	while config_server.running:
		if gui.render(config_server) == 0:
			break
		viewer.render(gui.identify, gui.min_aspect_ratio)

	for server in servers:
		server.kill()
	config_server.running = False
	cv2.destroyAllWindows()
		

if __name__ == '__main__':
	main()
