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
	def process_image(frame, MIN_ASPECT_RATIO):
		centroids = []
		overlay = frame.copy()
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		gray = cv2.threshold(gray, 230, 255, cv2.THRESH_BINARY)[1]
		thresh = cv2.threshold(frame, 225, 255, cv2.THRESH_BINARY)[1]
		sobel = np.uint8(np.absolute(cv2.Sobel(thresh, cv2.CV_64F, 0, 1, ksize = 1)))
		hsv = cv2.cvtColor(thresh, cv2.COLOR_BGR2HSV)
		flt = cv2.inRange(hsv, np.array([0, 0, 255]), np.array([0, 0, 255]))
		masked = cv2.bitwise_and(thresh, thresh, mask = flt)
		_contours, _ = cv2.findContours(flt, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		for contour in _contours:
			epsilon = cv2.arcLength(contour, True)
			approx = cv2.approxPolyDP(contour, epsilon, True)
			area = cv2.contourArea(contour)
			rect = cv2.minAreaRect(contour)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			a, b, c, d = box[0], box[1], box[2], box[3]

			aspect_ratio = np.linalg.norm(a - b) / np.linalg.norm(c - b)
			if area > 100 and (aspect_ratio > MIN_ASPECT_RATIO or 1. / aspect_ratio > MIN_ASPECT_RATIO):
				cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
				centroids.append(np.mean(box, axis = 0, dtype = np.uint32))
				cv2.circle(frame, tuple(centroids[-1]), 3, (0, 0, 255), -1)
		return overlay, centroids

	@staticmethod
	def process_centroids(centroids, center, frame_height):
		return

		'''
		Predict USBCamera fX values from detected centroids fYs 
		'''
		# Limelight empirical data
		Ty = [19.0, 0.3, -11.71]  # Independent variable during empirical collection.
		d_measured = [58.25, 8.0, 0.0]
		Tx_measured = [0.0, 6.0, 18.0]
		# Floor Tape Camera empirical data
		#(Assume is converted into percent of axis, origin at upper left of image)
		Fy = [0, 50, 100]
		Fx_measured = [45, 25, 10]

		tape_coefficients = np.polyfit(Fy, Fx_measured, 2)
		polynomial_function = np.poly1d(tape_coefficients)
		

		sorted_centroids = sorted(centroids, key = lambda pt: abs(pt[0] - polynomial_function(pt[1] / frame_height)))
		distance = abs(sorted_centroids[0][0] - polynomial_function(sorted_centroids[0][1] / frame_height))

		auto = NetworkTables.getTable('VisionData')
		auto.putString('floorTapeRumble', distance < 20)


		'''
		Predict Limelight fX values from its detected tYs
		'''

		limeTable = NetworkTables.getTable('limelight')

		#TODO: if tV lost for certain amount of time, flush circular queue
		if limeTable.getNumber('Tv', 0):

			limelight_Tx_coefficients = np.polyfit(Ty, Tx_measured, 2)
			polynomial_function = np.poly1d(limelight_Tx_coefficients)
			

			sorted_centroids = sorted(centroids, key = lambda pt: abs(pt[0] - polynomial_function(pt[1] / frame_height)))
			distance = abs(sorted_centroids[0][0] - polynomial_function(sorted_centroids[0][1] / frame_height))

			# add distance to circular queue
			auto = NetworkTables.getTable('VisionData')
			auto.putString('limelightRetroRumble', distance < 20)


			limelight_distance_coefficients = np.polyfit(Ty, d_measured, 2)
			polynomial_function = np.poly1d(limelight_distance_coefficients)
			

			sorted_centroids = sorted(centroids, key = lambda pt: abs(pt[0] - polynomial_function(pt[1] / frame_height)))
			distance = abs(sorted_centroids[0][0] - polynomial_function(sorted_centroids[0][1] / frame_height))

			# add distance to circular queue
			auto = NetworkTables.getTable('VisionData')
			auto.putString('limelightDistance', distance < 20)



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
