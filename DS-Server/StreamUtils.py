from win32api import GetSystemMetrics
import numpy as np
import struct
import socket
import math
import cv2

DEBUG = True

class StreamUtils:
	@staticmethod
	def recieve_packet(socket, packet_size = 65000, timeout_kill = False):
		message = b''
		address = ()
		try:
			byte_length, address = socket.recvfrom(4)
			length = struct.unpack('!i', byte_length)[0]

			if length > 0:
				socket.sendto(byte_length, address)

				recieved = 0
				while recieved < length - packet_size:
					byte_message, address = socket.recvfrom(packet_size)
					message += _message
					recieved += packet_size

				_message, address = socket.recvfrom(length % packet_size)
				message += _message
		except Exception as e:
			if str(e) == 'timed out':
				if timeout_kill:
					return 1, 1
			else:
				StreamUtils.dbgprint('Exception caught in StreamUtils::recieve_packet:', str(e), )

		return message, address

	@staticmethod
	def send_packet(socket, server, byte_data, packet_size = 65000, timeout_kill = False):
		try:
			byte_length = StreamUtils.itob(len(byte_data))
			socket.sendto(byte_length, server)

			data, server = socket.recvfrom(4)
			sent = 0
			while sent < len(byte_data):
				socket.sendto(byte_data[sent:sent + packet_size], server)
				sent += packet_size

		except Exception as e:
			if str(e) == 'timed out':
				if timeout_kill:
					return 1, 1
			else:
				StreamUtils.dbgprint('Exception caught in StreamUtils::send_packet:', str(e))

	@staticmethod
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

	@staticmethod
	def dbgprint(msg, *args):
		if DEBUG:
			print(msg, args)

	@staticmethod
	def itob(value):
		return struct.pack('!i', value)
		
	@staticmethod
	def btoi(value):
		return struct.unpack('!i', value)[0]

	@staticmethod
	def compress_frame(frame):
		return cv2.imencode('.jpg', frame)[1].tostring()

	@staticmethod
	def decompress_frame(frame):
		string_decoded = np.fromstring(frame, dtype = np.uint8)
		return cv2.imdecode(string_decoded, 1)

	@staticmethod
	def compress_string(string):
		return string.encode()

	@staticmethod
	def decompress_string(string):
		return string.decode()
