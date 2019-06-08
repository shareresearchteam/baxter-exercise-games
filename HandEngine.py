#!/usr/bin/env python

import rospy
import numpy as np
from scipy.signal import butter, lfilter

class HandEngine(object):

	def __init__(self, limb, data, gameType, fileName):
		# Declarations
		self._accelerations = {'left': [], 'right': []}		# List of accelerations detected
		self._dummy_accel = {'left': [], 'right': []}		# List of extraneous accelerations detected
		self.torques = {'left': [], 'right': []}
		self.totaltorques = []
		self._startTime = None								# Time at the beginning of a clap
		self._clap = {'left': False, 'right': False}		# Indicates whether a clap has been detected since last ping
		self._blocking = {'left': False, 'right': False}	# Indicates whether accelerometer data is being blocked
		self._blockTime = {'left': 0.0, 'right': 0.0}		# Used to keep track of how long the accelerometer data needs to be blocked for
		self._delayTime = 0.15
		self.threshold = 1.5								# The amount of time that the accelerometer is blocked after a clap is detected

		self.frequency = 1 / 1.5							# The frequency of arm motion
		self.collectData = data								# Flag determining whether or not to save data
		self._fileName = fileName							# Filename to dump acceleration data

		self.globalTime = rospy.get_rostime().to_sec()		# The starting time of the overall program

		self._b = []										# Paremeters that define the acceleration value filter
		self._a = []
		self._b, self._a = butter(1, 0.5, 'highpass')

		self.limb = limb
		self.gameType = gameType

		self.num = 1

		self.strength = 0.0

		self._init_time = rospy.get_rostime().to_sec()

	def ping(self, arm):
		"""Returns true if a clap was detected since last ping"""
		if self._clap[arm]:
			self._clap[arm] = False
			return True
		return False

	def clear(self):
		for key in self._clap.keys():
			self._clap[key] = False
		#print self._clap

	def clearIntervals(self):
		"""Deletes recorded clap intervals"""
		self._intervals = []

	def _clapDetected(self, arm):
		"""Helper function that handles a left hand clap"""

		# print "--- " + arm + " hand contact detected ---"

		self._clap[arm] = True

	def calculateAmplitude(self, interval):
		"""Calculates the desired displacement of clapping motion for a given time interval"""
		return 0.8*np.exp(-1.0*(1/interval))

	def _dump(self, extension, *arg):
		"""Appends the arguments as comma separated values in a new line of the file at self.fileName"""
		message = ""
		for string in arg:
			message += str(string)
			message += ","

		message = message[:-1]
		message += '\n'

		f = open(self._fileName + extension + ".csv", "a")
		f.write(message)
		f.close()

	def run(self, rate):
		sleeper = rospy.Rate(rate)
		while not rospy.is_shutdown():
			self.callback('right')
			self.callback('left')
			sleeper.sleep()

	def callback(self, msg, arm):
		"""Logs accelerometer data and looks for hand contact"""

		if self.collectData:
			realName = self._fileName[0:3] + self.gameType + self._fileName[-9:-4] + "_" + arm + ".csv"
			f = open(realName, "a")
			f.write(str(rospy.get_rostime().to_sec() - self.globalTime) + ',' + str(msg.linear_acceleration.x) + str("\n"))
			f.close()

		if self.gameType == 'stretch':
			self.threshold = 1.5
			self._delayTime = 0.2
		elif self.gameType == 'mimic':
			self.threshold = 2.0
			self._delayTime = 0.7
		elif self.gameType == 'agility':
			self.threshold = 1.5
			self._delayTime = 0.2
		elif self.gameType == 'handclap':
			self.threshold = 1.5
			self._delayTime = 0.7

		if not self._blocking[arm]:
			#self.torques[arm].append(self.limb.getTorqueDifference(arm + "_w1"))
			#elf.totaltorques.append(self.torques[arm][len(self.torques[arm]) - 1])
			self._accelerations[arm].append(msg.linear_acceleration.x)

			filt = lfilter(self._b, self._a, self._accelerations[arm])
			#threshold = 0.35
			#threshold = 0.52
			
			if max(filt) > self.threshold:
				self._blocking[arm] = True 							# sets flag to ignore acceleration signals
				self._clapDetected(arm) 							# informs the main thread that a clap was detected
				self.strength = max(np.amax(self._accelerations[arm]), -np.amin(self._accelerations[arm]))
				#print self.strength
				self._accelerations[arm] = []								# clears the acceleration buffer
				self._blockTime[arm] = rospy.get_rostime().to_sec()	# starts tracking how long to block for
		elif rospy.get_rostime().to_sec() < self._blockTime[arm] + self._delayTime:
			#print 'boop'
			pass
			self._dummy_accel[arm].append(msg.linear_acceleration.x)
			#self._blocking[arm] = False
		else:
			self._blocking[arm] = False
			self._dummy_accel[arm] = []