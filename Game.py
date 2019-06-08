#!/usr/bin/env python

# Import ROS, thread, math packages
import rospy
import thread
from threading import Thread
import random
import math
import numpy as np

# Import audio resources
from mingus.containers.note import Note
import mingus.core.notes as notes

# Import our classes for controlling Baxter things
from Head import Head
from ZeroG import ZeroG
from AudioManager import AudioManager
from ArmGenerator import ClappingArm
from MotionEngine import MotionEngine

class Game(object):
	def __init__(self, engine, arm):
		
		self._engine = engine						# The HandEngine object used for detecting hand hits
		self._arm = arm								# Motion object used to access the arms
		self.running = False						# Flag that indicates whether the game is running
		self._head = Head("faces/")					# The head object animates Baxter's face and also allows for nodding
		self.globalTime = rospy.get_rostime().to_sec()		# The starting time of the overall program

		##############################################################
		# The following are a list of parameters to be set on a person by person basis for the games
		# We will ask people for their height, administer a Box and Blocks manual dexterity test, and administer a Montreal Cognitive Assessment
		self._userHeight = 1.75
		self._boxNblocks = 63
		self._mocaScore = 25
		##############################################################

		# Determine length of memory sequence from MoCA score
		if self._mocaScore > 25:
			self._seqLen = 5
		elif self._mocaScore > 23:
			self._seqLen = 4
		elif self._mocaScore > 20:
			self._seqLen = 3
		else:
			self._seqLen = 2

		# Cap minimum and maximum heights for our users so the scaling works
		if self._userHeight < 1.5:
			self._userHeight = 1.5
		if self._userHeight > 2.15:
			self._userHeight = 2.15
		# Determine range of poses based on the user height for teach game
		self._limits = {'left_s0': {'min': -0.85, 'max': -0.7 + 0.2*(self._userHeight-1.5)/(2.15-1.5)},
						'right_s0': {'min': 0.7 - 0.2*(self._userHeight-1.5)/(2.15-1.5), 'max': 0.85},
						'left_s1': {'min': -0.2 - 0.2*(self._userHeight-1.5)/(2.15-1.5), 'max': 0.35},
						'right_s1': {'min': -0.2 - 0.2*(self._userHeight-1.5)/(2.15-1.5), 'max': 0.35}}
		# Determine ranges of poses based on the user height for stretch game
		self.S0_min = 0.55 - 0.45*(self._userHeight-1.5)/(2.15-1.5)
		self.S0_max = 0.95
		self.S1_min = -0.2 - 0.5*(self._userHeight-1.5)/(2.15-1.5)
		self.S1_max = 0.65

		# Cap max number of blocks we expect people to pass at 60
		if self._boxNblocks > 60:
			self._boxNblocks = 60
		# Scale allotted amount of stretch time based on some baseline time plus a box and blocks-related factor
		self.stretchTime = 3.0 + 5.0*(60.0-self._boxNblocks)/99.0	# The amount of time (in seconds) for a person to hit each pose in the stretch game
		
		# Decide on wake up (agility) game thresholds using box and blocks score
		self.maxFreq = 3.5 - 3.5*(60.0-self._boxNblocks)/59.0 # The target frequency over which participant must hit to wake up robot
		self.pauseTolerance = 3.5 + 1.5*(60.0-self._boxNblocks)/59.0

		# Decide on strength threshold using box and blocks score
		self.forceMax = 500.0 - 200.0*(60.0-self._boxNblocks)/59.0

		# Decide on handclap game error threshold and game length
		self.clapFailTolerance = 2 + 2.2*(60.0-self._boxNblocks)/59.0
		self.gameLength = 10

		# Determine roboga pose hold length
		self.holdLength = 8 - 5*(60.0-self._boxNblocks)/59.0

		# Determine flamenco dance length
		self.numDanceMoves = 10

		# Define universal end pose
		self.rEnd = {'right_s0': -0.75, 'right_s1': 0.8, 'right_w0': -1.5, 'right_w1': 0.0, 'right_w2': -3.0, 'right_e0': 1.3, 'right_e1': 0.0}
		self.lEnd = {'left_s0': 0.75, 'left_s1': 0.8, 'left_w0': 1.5, 'left_w1': 0.0, 'left_w2': 3.0, 'left_e0': -1.3, 'left_e1': 0.0}
		self.endPose = {'left': self.lEnd, 'right': self.rEnd}

	def execute(self):
		"""Begins the game"""
		self.running = True

	def stop(self):
		"""Ends the game"""
		self.running = False


class MimicGame(Game):
	def __init__(self, engine, arm, repNum):
		Game.__init__(self, engine, arm)

		thread.start_new_thread(self._head.run, (5,))


		self.steps = []							# List that keeps track of the hand claps detected
													# 0 - Left
													# 1 - Right
													# 2 - Both
		self.recordPhase = True					# Indicates whether the clap sequence is being recorded or played back
		self.waiting = True						# Indicates whether the robot is waiting to begin the record phase

		self.simdif = 0.2

		# List of arm positions
		rRetreat = {'right_s0': 0.9, 'right_s1': (self.S1_min + self.S1_max)/2, 'right_w0': -1.7, 'right_w1': 2.0, 'right_w2': -0.02070874061279297, 'right_e0': -1.5132720455200197, 'right_e1': 0.0}
		lRetreat = {'left_s0': -0.9, 'left_s1': (self.S1_min + self.S1_max)/2, 'left_w0': 1.7, 'left_w1': 2.0, 'left_w2': 0.02070874061279297, 'left_e0': 1.5132720455200197, 'left_e1': 0.0}
		rExtend = {'right_s0': 0.9, 'right_s1': (self.S1_min + self.S1_max)/2, 'right_w0': -1.7, 'right_w1': 1.6, 'right_w2': -0.02070874061279297, 'right_e0': -1.5132720455200197, 'right_e1': 0.0}
		lExtend = {'left_s0': -0.9, 'left_s1': (self.S1_min + self.S1_max)/2, 'left_w0': 1.7, 'left_w1': 1.6, 'left_w2': 0.02070874061279297, 'left_e0': 1.5132720455200197, 'left_e1': 0.0}
		rVictory = {'right_s0': 0.14, 'right_s1': -0.4, 'right_w0': -1.6, 'right_w1': 1.1, 'right_w2': 0.55, 'right_e0': -1.57, 'right_e1': -0.05}
		lVictory = {'left_s0': -0.14, 'left_s1': -0.4, 'left_w0': 1.6, 'left_w1': 1.1, 'left_w2': -0.55, 'left_e0': 1.57, 'left_e1': -0.05}
		rConfusion = {'right_s0': 0.165, 'right_s1': -0.664, 'right_w0': -3.05, 'right_w1': 2.07, 'right_w2': 3.00, 'right_e0': -0.35, 'right_e1': 1.81}
		lConfusion = {'left_s0': -0.165, 'left_s1': -0.664, 'left_w0': 3.05, 'left_w1': 2.07, 'left_w2': -3.00, 'left_e0': 0.35, 'left_e1': 1.81}
		
		self.positions = {'rightRetreat': rRetreat, 'leftRetreat': lRetreat, 'rightExtend': rExtend, 'leftExtend': lExtend, 'rightVictory': rVictory, 'leftVictory': lVictory, 'rightConfusion': rConfusion, 'leftConfusion': lConfusion}

		self.normal = 2.0						# The amount of time for a normal hand clap

		self.audio = AudioManager()

		self.sleep = rospy.Rate(100)

		self._seqLen += repNum

	def _playMotions(self, motions):
		"""Replays all recorded motions for the specified arm"""
		for step in motions:
			extend = dict()
			retreat = dict()
			if step == 0:
				arms = ["left"]
				extend["right"] = self.positions["rightRetreat"]
				retreat["right"] = self.positions["rightRetreat"]
			elif step == 1:
				arms = ["right"]
				extend["left"] = self.positions["leftRetreat"]
				retreat["left"] = self.positions["leftRetreat"]
			else:
				arms = ["right", "left"]
			
			for arm in arms:
				extend[arm] = self.positions[arm + 'Extend']
				retreat[arm] = self.positions[arm + 'Retreat']

			extend_move_ids = self._arm.move(extend, 0.5)
			self._playStep(step)
			retreat_move_ids = self._arm.move(retreat, 0.5)

		rospy.sleep(1.0)
		self._engine.clear()

	def _playStep(self, stepNum):
		# timpani is 47, toms are 118, synth drum is 117, woodblock is 115, agogo is 113
		if stepNum == 0:
			self.audio.setInstrument(113)
		elif stepNum == 1:
			self.audio.setInstrument(115)
		else:
			self.audio.setInstrument(118)
		thread.start_new_thread(self.audio.playNote, (Note(),))

	def _recordMotion(self, num):
		i = 0
		k = 0
		newMotion = 0
		oldSteps = list(self.steps)
		self.steps = []
		self._engine.clear()

		while i < num and self.running:
			if self._engine.ping('left'):
				end = rospy.get_rostime().to_sec() + self.simdif
				while rospy.get_rostime().to_sec() < end:
					if self._engine.ping('right'):
						self.steps.append(2)
						newMotion = 2
						self._playStep(2)
						self._head.switchEmotion('Joy', 'Happy', 0.4)
						rospy.sleep(0.5)
						self._engine.clear()
						break
				else:
					self.steps.append(0)
					newMotion = 0
					self._playStep(0)
					self._head.switchEmotion('Joy', 'Happy', 0.4)
					rospy.sleep(0.5)
					self._engine.clear()
				i += 1

				# Save pattern of taught motions
				writeFileName = 'Mimic' + str(self._seqLen) + '.csv'
				f = open(writeFileName, "a")
				f.write(str(rospy.get_rostime().to_sec()-self.globalTime) + ',' + str(newMotion) + str("\n"))
				f.close()

			elif self._engine.ping('right'):
				end = rospy.get_rostime().to_sec() + self.simdif
				while rospy.get_rostime().to_sec() < end:
					if self._engine.ping('left'):
						self.steps.append(2)
						newMotion = 2
						self._playStep(2)
						self._head.switchEmotion('Joy', 'Happy', 0.4)
						rospy.sleep(0.5)
						self._engine.clear()
						break
				else:
					self.steps.append(1)
					newMotion = 1
					self._playStep(1)
					self._head.switchEmotion('Joy', 'Happy', 0.4)
					rospy.sleep(0.5)
					self._engine.clear()
				i += 1

				# Save pattern of taught motions
				writeFileName = 'Mimic' + str(self._seqLen) + '.csv'
				f = open(writeFileName, "a")
				f.write(str(rospy.get_rostime().to_sec()-self.globalTime) + ',' + str(newMotion) + str("\n"))
				f.close()

			if not k == i:
				k = i
				if i <= len(oldSteps):
					if not self.steps == oldSteps[0:i]:
						return False

		self._head.changeColor('Blue')
		return True

	def _evaluateSequence(self):
		num = 0
		for i in range(1, len(self.steps)):
			num += 1
			if self.steps[i] == self.steps[i - 1]:
				num -= 0.5
		return num

	def execute(self):
		Game.execute(self)

		# Slowly assume starting positions
		self._arm.move({'left': self.positions['leftExtend'],'right': self.positions['rightExtend']},6.0)

		print "Waiting - Press s to start"

		# Wait for starting key press
		while self.waiting:
			pass
		
		# Record three initial motions
		self._head.changeColor('Green')
		self._head.nod()
		self._engine.clear()
		self._recordMotion(1)

		while self.running and not rospy.is_shutdown() and self._evaluateSequence() < self._seqLen:
			self._arm.move({'left': self.positions['leftRetreat'], 'right': self.positions['rightRetreat']}, 2.0)
			self._playMotions(self.steps)

			self._arm.move({"left": self.positions['leftExtend'],'right': self.positions['rightExtend']},1.0)
			self._head.changeColor('Green')
			self._head.nod()
			self._engine.clear()

			if not self._recordMotion(len(self.steps) + 1):
				self._head.switchEmotion('Silly','Silly',3.0)
				self._head.switchColor('Red','Red',3.0)
				self._arm.safeMove({'left': self.positions['leftVictory'],'right': self.positions['rightVictory']})
				self._head.changeColor('Red')
				self._head.changeEmotion('Silly')
				rospy.sleep(1)
				break
		else:
			self._arm.move({'left': self.positions['leftRetreat'], 'right': self.positions['rightRetreat']}, 2.0)
			length = int(round(.5 * (len(self.steps) - 1)))
			partial = self.steps[0:length]
			badMotion = self.steps[length + 1] - 1
			if badMotion < 0:
				badMotion = 2
			partial.append(badMotion)
			self._playMotions(partial)
			self._head.switchEmotion('Worried','Worried',3.0)
			self._head.switchColor('Purple','Purple',3.0)
			self._arm.safeMove({'left': self.positions['leftConfusion'],'right': self.positions['rightConfusion']})
			self._head.changeColor('Purple')
			self._head.changeEmotion('Worried')

		self.waiting = True

		print "Waiting - Press s to end"

		# Wait for ending key press
		while self.waiting:
			pass

		# Slowly assume ending positions
		self._head.changeColor('Blue')
		self._head.changeEmotion('Happy')
		self._arm.move(self.endPose,6.0)

class StretchGame(Game):
	def __init__(self, engine, arm, filename, arms):
		Game.__init__(self, engine, arm)

		thread.start_new_thread(self._head.run, (5,))
		self.waiting = True

		# Defines the starting positions for the arms
		posR = {'right_s0': 0.0, 'right_s1': (self.S1_min + self.S1_max)/2, 'right_w0': -1.57, 'right_w1': 1.5, 'right_w2': 1.0, 'right_e0': -1.57, 'right_e1': 0.0}
		posL = {'left_s0': 0.0, 'left_s1': (self.S1_min + self.S1_max)/2, 'left_w0': 1.57, 'left_w1': 1.5, 'left_w2': -1.0, 'left_e0': 1.57, 'left_e1': 0.0}
		
		self.pos = {'right': posR, 'left': posL}

		self.audio = AudioManager()

		self.song = filename
		self.arms = arms

	def execute(self):
		Game.execute(self)

		song = self.song
		arms = self.arms

		# Move to initial position
		self._arm.move(self.pos,6.0)

		# Load the song in single or double hand mode, depending
		self.audio.loadSong(song, len(arms) > 1)

		pinged = dict()
		hit = False
		badNotes = []

		print "Waiting - Press s to start"

		# Wait for starting key press
		while self.waiting:
			pass

		# Loop until the song is complete
		while not self.audio.songComplete():
			# Play the next note
			self._head.changeColor("Blue")
			for i in range(0, len(arms)):
				# For each arm, move to the appropriate position for the note
				S0, S1 = self.positionFromNote(self.audio.getCurrentNote()[i])

				W1 = self._calculateW1(S1)
				W2 = self._calculateW2(S0)

				if arms[i] == 'left':
					S0 = -S0
					W2 = -W2

				self.pos[arms[i]][arms[i] + "_s0"] = S0
				self.pos[arms[i]][arms[i] + "_s1"] = S1
				self.pos[arms[i]][arms[i] + "_w1"] = W1
				self.pos[arms[i]][arms[i] + "_w2"] = W2

				self._engine.clear()
				pinged[arms[i]] = False

			self._head.setAngle(0.0)
			rospy.sleep(0.5)
			self._arm.waitForMove(self._arm.move(self.pos,1.5))
			self._engine.clear()

			self.audio.playNextNote()

			# Save time that note played and 0
			writeFileName = 'Stretch' + self.song[-3:-1] + '.csv'
			f = open(writeFileName, "a")
			f.write(str(rospy.get_rostime().to_sec()-self.globalTime) + ',' + str(0) + str("\n"))
			f.close()

			start = rospy.get_rostime().to_sec()
			timeout = False
			quarter = self.stretchTime / 4.0
			while not hit:
				for arm in arms:
					if self._engine.ping(arm):
						pinged[arm] = True
				hit = pinged.values()[0]
				for val in pinged.values():
					hit = hit and val
				if hit:
					self._head.changeColor("Blue")
					self._head.switchEmotion("Joy", "Happy", 1)
					
				dif = rospy.get_rostime().to_sec() - start

				if dif < quarter:
					self._head.changeColor("Green")
					self._head.setAngle(0.15)
				elif dif < 2 * quarter:
					self._head.changeColor("Yellow")
					self._head.setAngle(-0.15)
				elif dif < 3 * quarter:
					self._head.changeColor("Orange")
					self._head.setAngle(0.15)
				elif dif <= self.stretchTime:
					self._head.changeColor("Red")
					self._head.setAngle(-0.15)
				elif dif > self.stretchTime:
					self._head.changeColor("Blue")
					self._head.switchEmotion("Sassy", "Happy", 1)
					hit = True
					n = Note(self.audio.getCurrentNote()[0])
					n.augment()
					n2 = Note()
					n2.from_int(int(n))
					n2.augment()
					ns = None
					if len(arms) > 1:
						ns = [n, n2]
					else:
						ns = n
					badNotes.append(self.audio.changeNote(ns))
			# Save time impact felt and 1
			writeFileName = 'Stretch' + self.song[-3:-1] + '.csv'
			f = open(writeFileName, "a")
			f.write(str(rospy.get_rostime().to_sec()-self.globalTime) + ',' + str(1) + str("\n"))
			f.close()

			self.audio.advance()
			hit = False

		# Set face color for final song replay
		self._head.changeColor("Blue")
		rospy.sleep(1.0)					# pause briefly before replaying notes
		self._head.changeColor("White")

		# Audio resets and Baxter plays the created song
		self.audio.reset()
		i = 0
		
		# Defines the starting positions for the arms
		posRend = {'right_s0': 0.0, 'right_s1': 0.0, 'right_w0': -1.57, 'right_w1': 1.5, 'right_w2': -1.2, 'right_e0': -1.57, 'right_e1': 0.0}
		posLend = {'left_s0': 0.0, 'left_s1': 0.0, 'left_w0': 1.57, 'left_w1': 1.5, 'left_w2': 1.2, 'left_e0': 1.57, 'left_e1': 0.0}
		self.posend = {'right': posRend, 'left': posLend}
		self._arm.move(self.posend,2.0)

		while not self.audio.songComplete():
			if i in badNotes:
				self._head.changeColor("Red")
				self._head.changeEmotion("Sassy")
			else:
				self._head.changeColor("Blue")
				self._head.changeEmotion("Joy")
			i = self.audio.playNextNote() + 1
			self.audio.advance()
			rospy.sleep(0.2)

		self._head.changeColor("Blue")
		self._head.changeEmotion("Happy")

		self.waiting = True

		print "Waiting - Press s to end"

		# Wait for ending key press
		while self.waiting:
			pass

		# Slowly assume ending positions
		self._arm.move(self.endPose,6.0)

	def _calculateW1(self, S1):
		return (1.048 * S1) + 1.57

	def _calculateW2(self, S0):
		return (-1.364 * S0) + 1.023

	def positionFromNote(self, note):
		yStep = (self.S1_max - self.S1_min) / 5.0
		xStep = (self.S0_max - self.S0_min) / 3.0

		index = int(Note(note)) - 36

		y = math.floor(index / 4.0)
		x = index - (4 * y)

		xamount = x * xStep
		yamount = y * yStep

		y = self.S1_max - yamount
		x = self.S0_min + xamount

		return (x, y)

class StrengthGame(Game):

	def __init__(self, engine, arm, song, punchSeq, repetition, repNum):
		# Run super initializer
		Game.__init__(self, engine, arm)

		thread.start_new_thread(self._head.run, (5,))

		if song[10:15]=='Anoth':
			self.introTime = 4.0
			self.songTime = 192.5
		elif song[10:15]=='BackI': 
			self.introTime = 5.0
			self.songTime = 170.5
		elif song[10:15]=='EyeOf':
			self.introTime = 8.0
			self.songTime = 190.0
		elif song[10:15]=='IronM':
			self.introTime = 8.0
			self.songTime =  170.0
		elif song[10:15]=='Rocky':
			self.introTime = 8.0
			if repetition == 'train':
				self.songTime = 85.7 
			else:
				self.songTime = 165.5
		elif song[10:15]=='SoWha':
			self.introTime = 8.0
			self.songTime = 194.5
		elif song[10:15]=='Welco':
			self.introTime = 4.0
			self.songTime = 171.0

		# List of robot locations that indicate different punches
		straightL = {'left_s0': -0.9, 'left_s1': (self.S1_min + self.S1_max)/2, 'left_w0': 1.7, 'left_w1': 1.6, 'left_w2': 0.02070874061279297, 'left_e0': 1.5132720455200197, 'left_e1': 0.0}
		straightR = {'right_s0': 0.9, 'right_s1': (self.S1_min + self.S1_max)/2, 'right_w0': -1.7, 'right_w1': 1.6, 'right_w2': -0.02070874061279297, 'right_e0': -1.5132720455200197, 'right_e1': 0.0}
		innerL = {'left_s0': -0.9, 'left_s1': (self.S1_min + self.S1_max)/2, 'left_w0': 2.05, 'left_w1': 1.58, 'left_w2': 0.5, 'left_e0': 1.5132720455200197, 'left_e1': 0.0}
		innerR = {'right_s0': 0.9, 'right_s1': (self.S1_min + self.S1_max)/2, 'right_w0': -2.05, 'right_w1': 1.58, 'right_w2': -0.5, 'right_e0': -1.5132720455200197, 'right_e1': 0.0}
		outerL = {'left_s0': -0.9, 'left_s1': (self.S1_min + self.S1_max)/2, 'left_w0': 1.0, 'left_w1': 1.5, 'left_w2': -0.5, 'left_e0': 1.5132720455200197, 'left_e1': 0.0}
		outerR = {'right_s0': 0.9, 'right_s1': (self.S1_min + self.S1_max)/2, 'right_w0': -1.0, 'right_w1': 1.5, 'right_w2': 0.5, 'right_e0': -1.5132720455200197, 'right_e1': 0.0}
		lowerL = {'left_s0': -0.9, 'left_s1': (self.S1_min + self.S1_max)/2, 'left_w0': -1.4, 'left_w1': 0.9, 'left_w2': 0.02070874061279297, 'left_e0': 1.5132720455200197, 'left_e1': 0.0}
		lowerR = {'right_s0': 0.9, 'right_s1': (self.S1_min + self.S1_max)/2, 'right_w0': 1.4, 'right_w1': 0.9, 'right_w2': -0.02070874061279297, 'right_e0': -1.5132720455200197, 'right_e1': 0.0}
		innerLowL = {'left_s0': -0.9, 'left_s1': (self.S1_min + self.S1_max)/2, 'left_w0': -1.95, 'left_w1': 1.1, 'left_w2': -0.5, 'left_e0': 1.5132720455200197, 'left_e1': 0.0}
		innerLowR = {'right_s0': 0.9, 'right_s1': (self.S1_min + self.S1_max)/2, 'right_w0': 1.95, 'right_w1': 1.1, 'right_w2': 0.5, 'right_e0': -1.5132720455200197, 'right_e1': 0.0}
		outerLowL = {'left_s0': -0.9, 'left_s1': (self.S1_min + self.S1_max)/2, 'left_w0': -0.5, 'left_w1': 1.0, 'left_w2': 0.2, 'left_e0': 1.5132720455200197, 'left_e1': 0.0}
		outerLowR = {'right_s0': 0.9, 'right_s1': (self.S1_min + self.S1_max)/2, 'right_w0': 0.5, 'right_w1': 1.0, 'right_w2': -0.2, 'right_e0': -1.5132720455200197, 'right_e1': 0.0}
		self.rVictory = {'right_s0': 0.14, 'right_s1': -0.4, 'right_w0': -1.6, 'right_w1': 1.1, 'right_w2': 0.55, 'right_e0': -1.57, 'right_e1': -0.05}
		self.lVictory = {'left_s0': -0.14, 'left_s1': -0.4, 'left_w0': 1.6, 'left_w1': 1.1, 'left_w2': -0.55, 'left_e0': 1.57, 'left_e1': -0.05}

		self.pos = {'left_Straight': straightL, 'right_Straight': straightR, 'left_Uppercut': innerL, 'right_Uppercut': innerR, 'left_Hook': outerL, 'right_Hook': outerR, 'left_Lower': lowerL, 'right_Lower': lowerR, 'left_LowUppercut': innerLowL, 'right_LowUppercut': innerLowR, 'left_LowHook': outerLowL, 'right_LowHook': outerLowR}

		self.waiting = True
		self.force = self.forceMax*0.5 + 20.0

		self.drainAmount = self.forceMax/2000.0 + 0.004*repNum
		self.repNum = repNum

		self.audio = AudioManager()
		self.audio.loadMP3(song)

		# Note punch options
		punchOptions = [{'right': 'right_Hook', 'left': 'left_Hook'}, {'right': 'right_Uppercut', 'left': 'left_Uppercut'}, {'right': 'right_Straight', 'left': 'left_Straight'}, {'right': 'right_Lower', 'left': 'left_Lower'}, {'right': 'right_LowUppercut', 'left': 'left_LowUppercut'}, {'right': 'right_LowHook', 'left': 'left_LowHook'}]

		# Find correct routine based on game function input
		if punchSeq == 'g1':
			randSet = [2, 3, 4, 5, 4, 5, 6, 1, 4, 3, 1, 4, 3, 4, 1, 4, 3, 6, 4, 1, 5, 2, 1, 6, 5, 3, 4, 6, 1, 3, 6, 3, 6, 1, 2, 6, 2, 3, 2, 6, 3, 6, 5, 1, 6, 4, 3, 2, 6, 1, 3, 6, 5, 4, 5, 1, 2, 5, 6, 1, 5, 3, 6, 2, 4, 6, 4, 5, 6, 1, 5, 4, 3, 2, 3, 6, 5, 3, 2, 4, 2, 5, 2, 6, 3, 4, 1, 5, 3, 4, 5, 1, 5, 4, 1, 5, 3, 4, 3, 1, 3, 5, 1, 5, 6, 4, 2, 5, 2, 5, 4, 2, 3, 1, 6, 1, 2, 5, 1, 6, 4, 6, 2, 6, 4, 6, 5, 2, 5, 6, 4, 3, 5, 1, 3, 4, 3, 2, 4, 1, 3, 6, 5, 3, 5, 2, 6, 3, 4, 6, 4, 1, 3, 1, 4, 6, 1, 6, 5, 6, 5, 3, 4, 1, 2, 3, 1, 5]
		elif punchSeq == 'g2':
			randSet = [1, 3, 5, 3, 4, 5, 2, 4, 3, 6, 3, 2, 1, 5, 2, 4, 3, 5, 3, 1, 2, 4, 6, 3, 1, 2, 3, 2, 6, 5, 6, 2, 3, 4, 6, 1, 5, 6, 1, 2, 6, 3, 1, 3, 2, 4, 6, 5, 2, 6, 3, 2, 5, 1, 5, 2, 3, 1, 2, 4, 5, 6, 4, 5, 6, 1, 4, 5, 2, 6, 2, 4, 6, 1, 3, 5, 2, 5, 3, 4, 1, 3, 2, 5, 4, 6, 1, 5, 4, 3, 1, 4, 5, 3, 1, 3, 2, 3, 4, 3, 1, 2, 4, 2, 6, 1, 6, 1, 6, 4, 2, 1, 6, 1, 4, 6, 1, 2, 4, 6, 5, 1, 4, 2, 5, 4, 1, 4, 6, 3, 2, 6, 1, 6, 4, 6, 1, 3, 6, 5, 6, 1, 2, 4, 1, 5, 4, 2, 6, 5, 1, 6, 3, 2, 5, 4, 6, 4, 2, 3, 2, 1, 4, 3, 6, 5, 2, 6]
		elif punchSeq == 'g3':
			randSet = [1, 2, 3, 4, 5, 2, 4, 3, 5, 6, 2, 5, 6, 5, 2, 3, 6, 4, 1, 4, 6, 2, 1, 4, 3, 5, 3, 4, 1, 5, 2, 1, 6, 2, 4, 1, 5, 6, 2, 5, 4, 1, 6, 3, 5, 4, 3, 2, 5, 6, 4, 1, 2, 4, 5, 6, 5, 4, 1, 5, 6, 2, 3, 6, 5, 6, 1, 3, 4, 3, 4, 2, 1, 4, 3, 4, 3, 1, 3, 4, 6, 4, 2, 3, 6, 4, 5, 6, 1, 5, 3, 4, 6, 4, 6, 4, 3, 6, 2, 5, 6, 4, 2, 6, 5, 3, 1, 3, 6, 1, 5, 2, 3, 5, 1, 5, 4, 6, 4, 2, 3, 5, 2, 3, 2, 3, 4, 6, 2, 6, 3, 1, 5, 6, 3, 1, 3, 2, 4, 1, 5, 2, 4, 5, 3, 5, 2, 6, 5, 1, 4, 2, 5, 4, 3, 4, 2, 1, 5, 4, 5, 1, 4, 3, 4, 1, 6, 2]
		elif punchSeq == 'free':
			randSet = []
			for i in range(0, 200):
				if i == 0:
					randInt = random.randint(1,6)
					randIntOld = randInt
					randSet.append(randInt)
				else:
					randInt = random.randint(1,6)
					if randInt == randIntOld:
						randInt = random.randint(1,6)
					else:
						randSet.append(randInt)
						randIntOld = randInt

		# Generate boxing routine
		self._randomRoutine = []
		for randNum in randSet:
			self._randomRoutine.append(punchOptions[randNum-1])

		self.gameStartTime = 0

		self.sleep = rospy.Rate(100)

	def execute(self):
		Game.execute(self)									# Run super of function

		# Slowly move robot to starting pose
		self._arm.move({'left': self.pos['left_Straight'],'right': self.pos['right_Straight']},6.0)

		print "Waiting - Press s to start"

		# Wait for starting key press
		while self.waiting:
			pass

		# Load boxing routine and start playing power song
		routine = self._randomRoutine
		rospy.sleep(0.1)
		self.audio.playMP3()

		# Stay still for power song intro
		self.gameStartTime = rospy.get_rostime().to_sec()
		rospy.sleep(self.introTime)

		# Then mark start time and begin routine
		self._runRoutine(routine)							# Run the routine

		self.waiting = True

		# Ending shutdown protocol
		print "Waiting - Press s to end"
		# Wait for ending key press
		while self.waiting:
			pass

		# Slowly assume ending positions
		self._head.changeColor('Blue')
		self._head.changeEmotion('Happy')
		self._arm.move(self.endPose,6.0)

	def _drain(self):
		self.force -= self.drainAmount
		if self.force < 0.0:
			self.force = 0.0

	def _updateFace(self):
		quarter = self.forceMax / 4.0
		if self.force > quarter * 3:
			self._head.changeEmotion("Joy")
			self.audio.unpauseMP3()
		elif self.force > quarter * 2:
			self._head.changeEmotion("Happy")
			self.audio.unpauseMP3()
		elif self.force > quarter:
			self._head.changeEmotion("Neutral")
			self.audio.unpauseMP3()
		else:
			self._head.changeEmotion("Afraid")
			self.audio.pauseMP3()

	def _runRoutine(self, routine):
		"""Causes the robot to move into position to indicate a series of punches"""

		# Go through moves in routine one by one
		for punch in routine:

			# If song isn't over yet, keep doing boxing routine
			if rospy.get_rostime().to_sec() - self.gameStartTime < self.songTime:

				# Identify move, move to pose, clear detected contacts, and cue user with sound and face color
				self._head.changeColor("Blue")
				punchPos = {'left': self.pos[punch['left']], 'right': self.pos[punch['right']]}
				self._arm.move(punchPos, 1.5)			# Otherwise move into position
				self._engine.clear()
				self._head.changeColor("Green")
				self.audio.setInstrument(113)
				thread.start_new_thread(self.audio.playNote, (Note(),))
				# Save time cue played, 0, 0
				writeFileName = 'Strength' + str(self.repNum) + '.csv'
				f = open(writeFileName, "a")
				f.write(str(rospy.get_rostime().to_sec()-self.globalTime) + ',' + str(0) + ',' + str(0) + str("\n"))
				f.close()
				pinged = {'left': False, 'right': False}

				# When Baxter hasn't felt contact on both of its end-effectors yet
				while (not pinged['left'] or not pinged['right']) and self.running and rospy.get_rostime().to_sec() - self.gameStartTime < self.songTime:
					
					# Sleep, drain points, make Baxter sadder (if changes)
					self.sleep.sleep()
					self._drain()
					self._updateFace()

					# Wait for contact on arms
					for arm in ['left', 'right']:
						if self._engine.ping(arm):
							pinged[arm] = True
							self.force += self._engine.strength
							if self.force > self.forceMax:
								self.force = self.forceMax

							# Save time impact felt, arm, and strength of impact
							writeFileName = 'Strength' + str(self.repNum) + '.csv'
							f = open(writeFileName, "a")
							f.write(str(rospy.get_rostime().to_sec()-self.globalTime) + ',' + arm + ',' + str(self._engine.strength) + str("\n"))
							f.close()

			else:
				# Break and stop audio if time is up for play
				self.audio.pauseMP3()
				self._head.changeColor("Blue")

				quarter = self.forceMax / 4.0
				if self.force > quarter * 3:
					self._head.changeEmotion("Joy")
					self._arm.safeMove({'left': self.lVictory,'right': self.rVictory})
				elif self.force > quarter * 2:
					self._head.changeEmotion("Happy")
					self.lVictory['left_s1'] = -0.2
					self.rVictory['right_s1'] = -0.2
					self.lVictory['left_w1'] = 1.0
					self.rVictory['right_w1'] = 1.0
					self._arm.safeMove({'left': self.lVictory,'right': self.rVictory})
				elif self.force > quarter:
					self._head.changeEmotion("Neutral")
					self.lVictory['left_s1'] = 0.0
					self.rVictory['right_s1'] = 0.0
					self.lVictory['left_w1'] = -0.5
					self.rVictory['right_w1'] = -0.5
					self._arm.safeMove({'left': self.lVictory,'right': self.rVictory})
				else:
					self._head.changeEmotion("Afraid")
					self.lVictory['left_s1'] = 0.2
					self.rVictory['right_s1'] = 0.2
					self.lVictory['left_w1'] = -1.0
					self.rVictory['right_w1'] = -1.0
					self._arm.safeMove({'left': self.lVictory,'right': self.rVictory})
				break

class AgilityGame(Game):
	def __init__(self, engine, arm, repNum):
		Game.__init__(self, engine, arm)

		self.numHits = {'left': 0.0, 'right': 0.0}
		self.freqTicks = 5
		self.waiting = True

		self.goal = False

		self.rVictory = {'right_s0': 0.14, 'right_s1': -0.4, 'right_w0': -1.6, 'right_w1': 1.1, 'right_w2': 0.55, 'right_e0': -1.57, 'right_e1': -0.05}
		self.lVictory = {'left_s0': -0.14, 'left_s1': -0.4, 'left_w0': 1.6, 'left_w1': 1.1, 'left_w2': -0.55, 'left_e0': 1.57, 'left_e1': -0.05}
		self.startright = {'right_s0': 0.9, 'right_s1': (self.S1_min + self.S1_max)/2, 'right_w0': -1.7, 'right_w1': 1.6, 'right_w2': -0.02070874061279297, 'right_e0': -1.5132720455200197, 'right_e1': 0.0}
		self.startleft = {'left_s0': -0.9, 'left_s1': (self.S1_min + self.S1_max)/2, 'left_w0': 1.7, 'left_w1': 1.6, 'left_w2': 0.02070874061279297, 'left_e0': 1.5132720455200197, 'left_e1': 0.0}

		allpos = dict(self.startleft)
		allpos.update(self.startright)

		self.repNum = repNum

		self.audio = AudioManager()
		self.audio.loadMP3("songs/SoundEffects/Snore2Quiet.mp3")

		self.open = 0

	def _avg(self, freqTime):
		# Find overall frequency of hand contacts
		return (self.numHits['left'] + self.numHits['right']) / (rospy.get_rostime().to_sec() - freqTime)

	def execute(self):
		Game.execute(self)

		# Turn face to sleeping and move robot to starting pose
		self._arm.move({'left': self.startleft, 'right': self.startright},6.0)
		self._head.sleep()

		print "Waiting - Press s to start"
		# Wait for starting key press
		while self.waiting:
			pass

		# Set frequency for running loop
		rate = rospy.Rate(100)

		# Start playing snore track and record when track/snore face beginning
		freqTime = rospy.get_rostime().to_sec()
		self._head.sleepOpen()
		rospy.sleep(0.1)
		self.audio.playMP3()
		snoreDuration = 4.55
		lastSnoreTime = rospy.get_rostime().to_sec()
		mouthTime = rospy.get_rostime().to_sec()

		# Decouple landmark difficulty
		timeInterval = 6.0 + 1.0*self.repNum

		# Generate some random numbers to keep the facial animation fresh
		randTime1 = 2.0*timeInterval + random.uniform(0.2,0.8)*timeInterval
		randTime2 = 3.0*timeInterval + random.uniform(0.15,0.45)*timeInterval
		randTime3 = 3.0*timeInterval + random.uniform(0.55,0.85)*timeInterval
		randTime4 = 4.0*timeInterval + random.uniform(0.1,0.3)*timeInterval
		randTime5 = 4.0*timeInterval + random.uniform(0.4,0.6)*timeInterval
		randTime6 = 4.0*timeInterval + random.uniform(0.7,0.9)*timeInterval

		# Note robot states for waking up animation sequence
		states = [
			{
				'bother_time': timeInterval,
				'color': 'Green'
			},
			{
				'bother_time': 2.0*timeInterval,
				'color': 'Yellow'
			},
			{
				'bother_time': randTime1,
				'emotion': 'Neutral',
				'gaze': 'Blink'
			},
			{
				'bother_time': randTime1 + 0.3,
				'emotion': 'sleep'
			},
			{
				'bother_time': 3.0*timeInterval,
				'color': 'Orange'
			},
			{
				'bother_time': randTime2,
				'emotion': 'Neutral',
				'gaze': 'Blink'
			},
			{
				'bother_time': randTime2 + 0.3,
				'emotion': 'sleep'
			},
			{
				'bother_time': randTime3,
				'emotion': 'Neutral',
				'gaze': 'Blink'
			},
			{
				'bother_time': randTime3 + 0.3,
				'emotion': 'sleep'
			},
			{
				'bother_time': 4.0*timeInterval,
				'color': 'Red'
			},
			{
				'bother_time': randTime4,
				'emotion': 'Neutral',
				'gaze': 'Blink'
			},
			{
				'bother_time': randTime4 + 0.3,
				'emotion': 'sleep'
			},
			{
				'bother_time': randTime5,
				'emotion': 'Neutral',
				'gaze': 'NE'
			},
			{
				'bother_time': randTime5 + 0.3,
				'emotion': 'sleep'
			},
			{
				'bother_time': randTime6,
				'emotion': 'Neutral',
				'gaze': 'Blink'
			},
			{
				'bother_time': randTime6 + 0.3,
				'emotion': 'sleep'
			},
			{
				'bother_time': 5.0*timeInterval,
				'color': 'Purple'
			},
			{
				'bother_time': 5.0*timeInterval + 0.1,
				'emotion': 'Joy',
				'gaze': 'NE'
			}
		]
		active_state_index = 0
		now = rospy.get_rostime().to_sec()

		while not rospy.is_shutdown() and self.running:

			# If contact is felt on arm, add to running number of contacts
			for arm in ['left', 'right']:
				if self._engine.ping(arm):
					self.numHits[arm] += 1
					now = rospy.get_rostime().to_sec()
					# Save time impact felt and 1
					writeFileName = 'Agility' + str(self.repNum) + '.csv'
					f = open(writeFileName, "a")
					f.write(str(rospy.get_rostime().to_sec()-self.globalTime) + ',' + arm + str("\n"))
					f.close()

			# Figure out how long user has been adequately bothering Baxter
			bother_time = rospy.get_rostime().to_sec() - freqTime

			# Restart audio if user has not made sufficient progress
			if rospy.get_rostime().to_sec() - lastSnoreTime > snoreDuration and bother_time < 2.23*timeInterval:
				lastSnoreTime = rospy.get_rostime().to_sec()
				self.audio.playMP3()

			# Open and close mouth in time with snoring sound effect
			if rospy.get_rostime().to_sec() - mouthTime > snoreDuration/2 and self.open == 0 and bother_time < 2.23*timeInterval:
				self._head.sleep()
				self.open = 1
				mouthTime = rospy.get_rostime().to_sec()
			elif rospy.get_rostime().to_sec() - mouthTime > snoreDuration/2 and self.open == 1 and bother_time < 2.23*timeInterval:
				self._head.sleepOpen()
				self.open = 0
				mouthTime = rospy.get_rostime().to_sec()

			# Check if user has made progress and animate squinty faces accordingly
			if self._avg(freqTime) > self.maxFreq and rospy.get_rostime().to_sec() - now < self.pauseTolerance:
				bother_time = rospy.get_rostime().to_sec() - freqTime
				active_state = states[active_state_index]
				if bother_time > active_state['bother_time'] and rospy.get_rostime().to_sec() - now < self.pauseTolerance/7:
					if 'color' in active_state:
						self._head.changeColor(active_state['color'])
					elif active_state['emotion'] == 'sleep':
						self._head.sleep()
					else:
						self._head.changeEmotion(active_state['emotion'])
						self._head.changeGaze(active_state['gaze'])
						self._head.updateFace()
					active_state_index += 1
					if active_state_index == len(states):
						break
			else:
				active_state_index = 0
				freqTime = rospy.get_rostime().to_sec()
				self.numHits = {'left': 0.0, 'right': 0.0}
				self._head.changeColor('Blue')

			rate.sleep()

		self.audio.loadMP3("songs/SoundEffects/Yawn1Quiet.mp3")
		rospy.sleep(0.1)
		self.audio.playMP3()
		self._arm.move({'left': self.lVictory, 'right': self.rVictory},3.5)

		# Ending sequence
		self.waiting = True

		print "Waiting - Press s to end"

		# Wait for ending key press
		while self.waiting:
			pass

		# Slowly assume ending positions
		self._head.changeColor('Blue')
		self._head.changeEmotion('Happy')
		self._head.updateFace()
		self._arm.move(self.endPose,6.0)
		
class TeachGame(Game):

	def __init__(self, engine, arm):
		# Run super initializer
		Game.__init__(self, engine, arm)
		thread.start_new_thread(self._head.run, (5,))
		self.waiting = True

		self.pausedTime = 0.0
		self.pausedTicks = 100

		self.notes = []
		self.noteRegistered = {"left": True, "right": True}
		self.rightPoses = []
		self.leftPoses = []

		self.audio = AudioManager()

	def execute(self):
		Game.execute(self)

		noteGapTimer = rospy.get_rostime().to_sec()

		self._arm.move({'left':{'left_s0': -0.85, 'left_s1': 0.0, 'left_w0': 1.57, 'left_w1': 0.0, 'left_w2': 0, 'left_e0': 1.57, 'left_e1': 0.0},'right':{'right_s0': 0.85, 'right_s1': 0.0, 'right_w0': -1.57, 'right_w1': 0.0, 'right_w2': 0.0, 'right_e0': -1.57, 'right_e1': 0.0}},6.0)

		zero = ZeroG({'right_w0': -1.57, 'right_w1': 0.0, 'right_e0': -1.57, 'right_e1': 0.0, 'left_w0': 1.57, 'left_w1': 0.0, 'left_e0': 1.57, 'left_e1': 0.0})
		
		print "Waiting - Press s to start"

		# Wait for starting key press
		while self.waiting:
			zero._control_rate.sleep()

		poses = {'left': ([],[]), 'right': ([],[])}
		state = {'left': 0, 'right': 0}
		new_state = {'left': 0, 'right': 0}

		print "Press q for final song replay"

		while self.running and not rospy.is_shutdown():
			zero._update_torques()
			zero._control_rate.sleep()

			angles = self._arm.get_angles()

			new_state['left'] = self._wrist(angles['left_w2'])
			new_state['right'] = self._wrist(angles['right_w2'])

			if new_state['left'] == 1:
				if new_state['right'] == 1 and state['right'] == 0:
					nL = self.noteFromPosition('left', angles['left_s0'], angles['left_s1'])
					nR = self.noteFromPosition('right', angles['right_s0'], angles['right_s1'])
					nR.change_octave(1)
					self.notes.append([nL, nR])
					self.rightPoses.append({'right_s0':angles['right_s0'], 'right_s1':angles['right_s1'], 'right_w0': -1.57, 'right_w1': 0.0, 'right_w2': 0.0, 'right_e0': -1.57, 'right_e1': 0.0})
					self.leftPoses.append({'left_s0':angles['left_s0'], 'left_s1':angles['left_s1'], 'left_w0': 1.57, 'left_w1': 0.0, 'left_w2': 0, 'left_e0': 1.57, 'left_e1': 0.0})
					thread.start_new_thread(self.audio.playChord, (nL, nR))
					self._head.switchEmotion('Joy', 'Happy', 0.4)
					self._head.switchColor('Green', 'Blue', 1.5)
					noteGapTimer = rospy.get_rostime().to_sec()
				elif new_state['right'] == 0 and rospy.get_rostime().to_sec() - noteGapTimer > 1.0:
					nL = self.noteFromPosition('left', angles['left_s0'], angles['left_s1'])
					nR = self.noteFromPosition('right', angles['right_s0'], angles['right_s1'])
					nL.change_octave(1)
					thread.start_new_thread(self.audio.playChord, (nL, nL))
					noteGapTimer = rospy.get_rostime().to_sec()

			if new_state['right'] == 1:
				if new_state['left'] == 1 and state['left'] == 0:
					nL = self.noteFromPosition('left', angles['left_s0'], angles['left_s1'])
					nR = self.noteFromPosition('right', angles['right_s0'], angles['right_s1'])
					nR.change_octave(1)
					self.notes.append([nL, nR])
					self.rightPoses.append({'right_s0':angles['right_s0'], 'right_s1':angles['right_s1'], 'right_w0': -1.57, 'right_w1': 0.0, 'right_w2': 0.0, 'right_e0': -1.57, 'right_e1': 0.0})
					self.leftPoses.append({'left_s0':angles['left_s0'], 'left_s1':angles['left_s1'], 'left_w0': 1.57, 'left_w1': 0.0, 'left_w2': 0, 'left_e0': 1.57, 'left_e1': 0.0})
					thread.start_new_thread(self.audio.playChord, (nL, nR))
					self._head.switchEmotion('Joy', 'Happy', 0.4)
					self._head.switchColor('Green', 'Blue', 1.5)
					noteGapTimer = rospy.get_rostime().to_sec()
				elif new_state['left'] == 0 and rospy.get_rostime().to_sec() - noteGapTimer > 1.0:
					nL = self.noteFromPosition('left', angles['left_s0'], angles['left_s1'])
					nR = self.noteFromPosition('right', angles['right_s0'], angles['right_s1'])
					nL.change_octave(1)
					thread.start_new_thread(self.audio.playChord, (nR, nR))
					noteGapTimer = rospy.get_rostime().to_sec()

			state = new_state.copy()

		self.rVictory = {'right_s0': 0.0, 'right_s1': -0.5, 'right_w0': -1.2, 'right_w1': -1.1, 'right_w2': 3.0, 'right_e0': 1.4, 'right_e1': 0.3}
		self.lVictory = {'left_s0': 0.0, 'left_s1': -0.5, 'left_w0': 1.2, 'left_w1': -1.1, 'left_w2': -3.0, 'left_e0': -1.4, 'left_e1': 0.3}
		counter = 0
		# Save time impact felt and 1
		writeFileName = 'Teach' + '.csv'
		f = open(writeFileName, "a")
		f.write('right_s0' + ',' + 'right_s1' + ',' + 'right_w0' + ',' + 'right_s1' + ',' + 'right_w2' + ',' + 'right_e0' + ',' + 'right_e1' + ',' + 'left_s0' + ',' + 'left_s1' + ',' + 'left_w0' + ',' + 'left_s1' + ',' + 'left_w2' + ',' + 'left_e0' + ',' + 'left_e1' + str("\n"))
		f.close()
		for chord in self.notes:
			# Save time impact felt and 1
			writeFileName = 'Teach' + '.csv'
			f = open(writeFileName, "a")
			f.write(str(self.rightPoses[counter]['right_s0']) + ',' + str(self.rightPoses[counter]['right_s1']) + ',' + str(self.rightPoses[counter]['right_w0']) + ',' + str(self.rightPoses[counter]['right_s1']) + ',' + str(self.rightPoses[counter]['right_w2']) + ',' + str(self.rightPoses[counter]['right_e0']) + ',' + str(self.rightPoses[counter]['right_e1']) + ',' + str(self.leftPoses[counter]['left_s0']) + ',' + str(self.leftPoses[counter]['left_s1']) + ',' + str(self.leftPoses[counter]['left_w0']) + ',' + str(self.leftPoses[counter]['left_s1']) + ',' + str(self.leftPoses[counter]['left_w2']) + ',' + str(self.leftPoses[counter]['left_e0']) + ',' + str(self.leftPoses[counter]['left_e1']) + str("\n"))
			f.close()

			self._arm.waitForMove(self._arm.move({'left':self.leftPoses[counter],'right':self.rightPoses[counter]},1.5))
			thread.start_new_thread(self.audio.playChord, (chord[0],chord[1]))
			counter = counter + 1

		self.waiting = True

		print "Waiting - Press s to end"

		# Wait for ending key press
		while self.waiting:
			pass

		# Slowly assume ending positions
		self._head.changeColor('Blue')
		self._head.changeEmotion('Happy')
		self._arm.move(self.endPose,6.0)
			
	def noteFromPosition(self, arm, S0, S1):

		if arm == 'left':
			if S0 > self._limits["left_s0"]["max"]:
				S0 = self._limits["left_s0"]["max"]
			if S0 < self._limits["left_s0"]["min"]:
				S0 = self._limits["left_s0"]["min"]
			if S1 > self._limits["left_s1"]["max"]:
				S1 = self._limits["left_s1"]["max"]
			if S1 < self._limits["left_s1"]["min"]:
				S1 = self._limits["left_s1"]["min"]

			yStep = (self._limits["left_s1"]["max"] - self._limits["left_s1"]["min"]) / 3.0
			xStep = (self._limits["left_s0"]["max"] - self._limits["left_s0"]["min"]) / 3.0

			x = math.floor((S0 - self._limits["left_s0"]["min"]) / xStep)
			y = math.floor((self._limits["left_s1"]["max"] - S1) / yStep)

		elif arm == 'right':
			if S0 > self._limits["right_s0"]["max"]:
				S0 = self._limits["right_s0"]["max"]
			if S0 < self._limits["right_s0"]["min"]:
				S0 = self._limits["right_s0"]["min"]
			if S1 > self._limits["right_s1"]["max"]:
				S1 = self._limits["right_s1"]["max"]
			if S1 < self._limits["right_s1"]["min"]:
				S1 = self._limits["right_s1"]["min"]

			yStep = (self._limits["right_s1"]["max"] - self._limits["right_s1"]["min"]) / 3.0
			xStep = (self._limits["right_s0"]["max"] - self._limits["right_s0"]["min"]) / 3.0

			x = math.ceil((S0 - self._limits["right_s0"]["min"]) / xStep)
			y = math.floor((self._limits["right_s1"]["max"] - S1) / yStep)

		index = int(4 * y + x) + 36

		n = Note()
		n.from_int(index)

		return n

	def _wrist(self, W2):
		if W2 > 0.785 or W2 < -0.785:
			return 1
		else:
			return 0

class HandclapGame(Game):

	def __init__(self, engine, arm, gameid, mode, repetition):
		# Run super initializer
		Game.__init__(self, engine, arm)
		self.gameid = gameid
		self.mode = mode
		thread.start_new_thread(self._head.run, (5,))
		self._move = {}
		self.waiting = True
		self.failClaps = 0

		if repetition == 'train':
			self.gameLength -= 4

		self.rVictory = {'right_s0': 0.0, 'right_s1': -0.5, 'right_w0': -1.2, 'right_w1': -1.1, 'right_w2': 3.0, 'right_e0': 1.4, 'right_e1': 0.3}
		self.lVictory = {'left_s0': 0.0, 'left_s1': -0.5, 'left_w0': 1.2, 'left_w1': -1.1, 'left_w2': -3.0, 'left_e0': -1.4, 'left_e1': 0.3}

		self.rFailure = {'right_s0': -0.4, 'right_s1': 0.27, 'right_w0': -1.4, 'right_w1': 0.93, 'right_w2': 2.75, 'right_e0': 1.1, 'right_e1': 0.65}
		self.lFailure = {'left_s0': 0.4, 'left_s1': 0.27, 'left_w0': 1.4, 'left_w1': 0.93, 'left_w2': -2.75, 'left_e0': -1.1, 'left_e1': 0.65}

		# Ready pose targets
		self._move['ready'] = {'left': {'left_s0':-0.03, 'left_s1':0.14, 'left_w0':1.664, 'left_w1':-1.57, 'left_w2':-3.00, 'left_e0':-1.28, 'left_e1':1.875}, 'right': {'right_s0': 0.03, 'right_s1': 0.14, 'right_w0': -1.664, 'right_w1': -1.57, 'right_w2': 3.00, 'right_e0': 1.28, 'right_e1': 1.875}}
		self._move['ready_prep'] = {'left': {'left_s0':-0.03, 'left_s1':0.14, 'left_w0':1.664, 'left_w1':-1.57, 'left_w2':-3.00, 'left_e0':-1.28, 'left_e1':1.875}, 'right': {'right_s0': 0.03, 'right_s1': 0.14, 'right_w0': -1.664, 'right_w1': -1.57, 'right_w2': 3.00, 'right_e0': 1.28, 'right_e1': 1.875}}

		# Front five targets (1)
		self._move['front_five'] = {'left': {'left_s0': -0.1, 'left_s1': 0.225, 'left_w0': 1.664, 'left_w1': -1.54, 'left_w2': -2.58, 'left_e0': -1.5, 'left_e1': 1.25}, 'right': {'right_s0': 0.1, 'right_s1': 0.225, 'right_w0': -1.664, 'right_w1': -1.54, 'right_w2': 2.58, 'right_e0': 1.5, 'right_e1': 1.25}}
		self._move['front_five_prep'] = {'left': {'left_s0': 0.23, 'left_s1': 0.12, 'left_w0': 1.68, 'left_w1': -1.57, 'left_w2': -3.0, 'left_e0': -1.25, 'left_e1': 2.0}, 'right': {'right_s0': -0.23, 'right_s1': 0.12, 'right_w0': -1.68, 'right_w1': -1.57, 'right_w2': 3.0, 'right_e0': 1.25, 'right_e1': 2.0}}

		# Left five targets (2)
		self._move['left_five'] = {'left': {'left_s0': -0.44, 'left_s1': 0.139, 'left_w0': 1.717, 'left_w1': -1.484, 'left_w2': -2.914, 'left_e0': -1.494, 'left_e1': 1.0}, 'right': {'right_s0': -0.13, 'right_s1': 0.14, 'right_w0': -1.664, 'right_w1': -1.57, 'right_w2': 3.00, 'right_e0': 1.28, 'right_e1': 1.875}}
		self._move['left_five_prep'] = {'left': {'left_s0': 0.23, 'left_s1': 0.12, 'left_w0': 1.68, 'left_w1': -1.57, 'left_w2': -3.0, 'left_e0': -1.25, 'left_e1': 2.0}, 'right': {'right_s0': -0.13, 'right_s1': 0.14, 'right_w0': -1.664, 'right_w1': -1.57, 'right_w2': 3.00, 'right_e0': 1.28, 'right_e1': 1.875}}

		# Right five targets (3)
		self._move['right_five'] = {'left': {'left_s0':0.13, 'left_s1':0.14, 'left_w0':1.664, 'left_w1':-1.57, 'left_w2':-3.00, 'left_e0':-1.28, 'left_e1':1.875}, 'right': {'right_s0':0.44, 'right_s1':0.139, 'right_w0':-1.717, 'right_w1':-1.484, 'right_w2':2.914, 'right_e0':1.494, 'right_e1':1.0}}
		self._move['right_five_prep'] = {'left': {'left_s0':0.13, 'left_s1':0.14, 'left_w0':1.664, 'left_w1':-1.57, 'left_w2':-3.00, 'left_e0':-1.28, 'left_e1':1.875}, 'right': {'right_s0': -0.23, 'right_s1': 0.12, 'right_w0': -1.68, 'right_w1': -1.57, 'right_w2': 3.0, 'right_e0': 1.25, 'right_e1': 2.0}}

		# Left straight targets (4)
		self._move['left_straight'] = {'left': {'left_s0': -0.1, 'left_s1': 0.225, 'left_w0': 1.664, 'left_w1': -1.54, 'left_w2': -2.58, 'left_e0': -1.5, 'left_e1': 1.25}, 'right': {'right_s0': -0.23, 'right_s1': 0.12, 'right_w0': -1.68, 'right_w1': -1.57, 'right_w2': 3.0, 'right_e0': 1.25, 'right_e1': 2.0}}
		self._move['left_straight_prep'] = {'left': {'left_s0': 0.23, 'left_s1': 0.12, 'left_w0': 1.68, 'left_w1': -1.57, 'left_w2': -3.0, 'left_e0': -1.25, 'left_e1': 2.0}, 'right': {'right_s0': -0.23, 'right_s1': 0.12, 'right_w0': -1.68, 'right_w1': -1.57, 'right_w2': 3.0, 'right_e0': 1.25, 'right_e1': 2.0}}

		# Right straight targets (5)
		self._move['right_straight'] = {'left': {'left_s0': 0.23, 'left_s1': 0.12, 'left_w0': 1.68, 'left_w1': -1.57, 'left_w2': -3.0, 'left_e0': -1.25, 'left_e1': 2.0}, 'right': {'right_s0': 0.1, 'right_s1': 0.225, 'right_w0': -1.664, 'right_w1': -1.54, 'right_w2': 2.58, 'right_e0': 1.5, 'right_e1': 1.25}}
		self._move['right_straight_prep'] = {'left': {'left_s0': 0.23, 'left_s1': 0.12, 'left_w0': 1.68, 'left_w1': -1.57, 'left_w2': -3.0, 'left_e0': -1.25, 'left_e1': 2.0}, 'right': {'right_s0': -0.23, 'right_s1': 0.12, 'right_w0': -1.68, 'right_w1': -1.57, 'right_w2': 3.0, 'right_e0': 1.25, 'right_e1': 2.0}}

		# Clap targets (currently not used)
		self._move['clap'] = {'left': {'left_s0': -0.25, 'left_s1': 0.15, 'left_w0': 1.55, 'left_w1': -1.5, 'left_w2': 2.45, 'left_e0': -1.3, 'left_e1': 1.6}, 'right': {'right_s0': 0.25, 'right_s1': 0.15, 'right_w0': -1.55, 'right_w1': -1.5, 'right_w2': -2.45, 'right_e0': 1.3, 'right_e1': 1.6}}
		self._move['clap_prep'] = {'left': {'left_s0': -0.05, 'left_s1': 0.16, 'left_w0': 1.56, 'left_w1': -1.57, 'left_w2': 2.45, 'left_e0': -1.28, 'left_e1': 1.55}, 'right': {'right_s0': 0.05, 'right_s1': 0.16, 'right_w0': -1.56, 'right_w1': -1.57, 'right_w2': -2.45, 'right_e0': 1.28, 'right_e1': 1.55}}

	def execute(self):

		self._head.switchEmotion('Joy', 'Happy', 1)

		# Move arms to start pose and put starting face
		self._arm.move(self._move['ready_prep'], 6.0)
		
		print "Waiting - Press s to start"

		# Wait for starting key press
		while self.waiting:
			pass

		# Robot will do set of motions 3 times, once in demonstration and 2 times in collaborative play
		reps = 3
		# Game 1: 
		if self.gameid == 'g1':
			sequenceTemp = [5, 2, 4, 3, 1, 3, 4, 5, 1, 1]
		# Game 2:
		elif self.gameid == 'g2':
			sequenceTemp = [1, 3, 5, 5, 3, 5, 2, 4, 1, 3]
		# Game 3: 
		elif self.gameid == 'g3':
			sequenceTemp = [3, 2, 3, 3, 5, 1, 2, 5, 5, 4]
		# Free-play game: Baxter makes up a game based on allowed moves
		elif self.gameid == 'free':
			if self.mode == 'new':
				# In the free play mode, Baxter will invent a new game that can be reloaded later with 'load'
				sequenceTemp = [] 
				motion_set = [1, 2, 3, 4, 5]
				for num in range(10):
					# Pick motion to append
					sequenceTemp.append(random.choice(motion_set))
				# Name file for saving motion pattern
				file_name = 'RandomRobotClapSequence.csv'
				f = open(file_name, "a")
				f.writelines(["%s\n" % item  for item in sequenceTemp])
				f.close()
			# Game rep 2
			elif self.mode == 'load':
				# In the game load mode, Baxter will load game that was just taught to play again
				sequenceTemp = []
				# Import previous random data
				file_name = 'Subject' + subject + 'RandomRobotClapSequence.csv'
				with open(file_name, 'rb') as csvfile:
					csvReader = csv.reader(csvfile)
					for column in csvReader:
						for elem in column:
							sequenceTemp.append(int(elem))

		# cycle count used to control how many times Baxter plays each game sequence
		cycle = 1

		# number of moves in game
		gameMoves = 3

		# Identify motion period for observed tempo
		tempo_period = 1.5

		while not rospy.is_shutdown():

			self.waiting = True

			if cycle == 1:
				sequence = sequenceTemp[0:gameMoves]

			# Baxter stops after 1 demonstration and 2 cycles of gameplay
			if self.failClaps > self.clapFailTolerance:
				self._head.switchEmotion("Sad","Sad",6.0)
				self._head.switchColor('Red','Red',3.0)
				self._arm.move({'left': self.lFailure,'right': self.rFailure},6.0)
				self._head.changeColor('Red')
				self._head.changeEmotion('Sad')
				rospy.sleep(1)
				break				

			elif cycle > reps and gameMoves < self.gameLength:
				cycle = 1
				gameMoves += 1
				self._arm.move(self._move['ready_prep'], 1.0)
				self._head.changeEmotion('Happy')
				self._head.changeColor('Blue')
				self.failClaps = 0
				print "Waiting - Press s for next game repetition"
				while self.waiting:
					pass

			elif cycle > reps and gameMoves > self.gameLength - 1:
				self._head.switchEmotion("Joy","Joy",6.0)
				self._head.switchColor('Blue','Blue',6.0)
				self._arm.move({'left': self.lVictory,'right': self.rVictory},6.0)
				self._head.changeEmotion('Joy')
				self._head.changeColor('Blue')
				rospy.sleep(1)
				break

			else:

				if cycle == 1:
					self._head.changeEmotion('Happy')
					self._head.changeColor('Yellow')
				elif cycle == 2:
					self._head.changeEmotion('Happy')
					self._head.changeColor('Purple')

				motion_dict = {1:'front_five',2:'left_five',3:'right_five',4:'left_straight',5:'right_straight'}
				# Only activate after a motion sequence has been defined
				if len(sequence) != 0:
					for i in sequence: # sequence is the list of all the class labels that correspond to handclapping motions
						desired_move = motion_dict[i]
						prep_name = desired_move + '_prep'
						self._engine.clear()
						self._arm._motion_engine.do_handClapMove(self._arm._left_arm, self._arm._right_arm, self._move[prep_name], self._move[desired_move], tempo_period, True)

						rospy.sleep(0.2)
						clapsDetected = int(self._engine._clap['left']) + int(self._engine._clap['right'])

						if cycle > 1 and clapsDetected == 0:
							self.failClaps += 1
						elif cycle > 1:
							self._head.switchEmotion('Joy','Happy',1)

					cycle += 1

		self.waiting = True

		print "Waiting - Press s to end"

		# Wait for ending key press
		while self.waiting:
			pass

		# Slowly assume ending positions
		self._head.changeColor('Blue')
		self._head.changeEmotion('Happy')
		self._arm.move(self.endPose,6.0)

class RobogaGame(Game):
	def __init__(self, engine, arm, gameid, mode):
		# Run super initializer
		Game.__init__(self, engine, arm)
		thread.start_new_thread(self._head.run, (5,))
		self._move = {}
		self.gameid = gameid
		self.mode = mode
		self.waiting = True

		# Define relax pose and other mid and high side and front of body arm poses
		self._move['relax'] = {'left':{'left_s0': 0.95, 'left_s1': 1.0, 'left_w0': 1.9, 'left_w1': -0.01, 'left_w2': 3.0, 'left_e0': -1.5, 'left_e1': 0.2}, 'right': {'right_s0': -0.95, 'right_s1': 1.0, 'right_w0': -1.9, 'right_w1': -0.01, 'right_w2': -3.0, 'right_e0': 1.5, 'right_e1': 0.2}}
		self._move['midSide'] = {'left':{'left_s0': 0.9, 'left_s1': 0.0, 'left_w0': 1.7, 'left_w1': 0.0, 'left_w2': 3.0, 'left_e0': -1.6, 'left_e1': 0.2}, 'right': {'right_s0': -0.9, 'right_s1': 0.0, 'right_w0': -1.7, 'right_w1': 0.0, 'right_w2': -3.0, 'right_e0': 1.6, 'right_e1': 0.2}}
		self._move['topSide'] = {'left': {'left_s0': 0.8, 'left_s1': -1.0, 'left_w0': 1.55, 'left_w1': 0.0, 'left_w2': 0.15, 'left_e0': -1.4, 'left_e1': 0.1}, 'right': {'right_s0': -0.8, 'right_s1': -1.0, 'right_w0': -1.55, 'right_w1': 0.0, 'right_w2': 0.15, 'right_e0': 1.4, 'right_e1': 0.1}}
		self._move['midFront'] = {'left':{'left_s0': -0.65, 'left_s1': 0.0, 'left_w0': 1.7, 'left_w1': 0.0, 'left_w2': 3.0, 'left_e0': -1.6, 'left_e1': 0.2}, 'right': {'right_s0': 0.65, 'right_s1': 0.0, 'right_w0': -1.7, 'right_w1': 0.0, 'right_w2': -3.0, 'right_e0': 1.6, 'right_e1': 0.2}}
		self._move['topFront'] = {'left':{'left_s0': -0.65, 'left_s1': -0.7, 'left_w0': 1.7, 'left_w1': 0.0, 'left_w2': 3.0, 'left_e0': -1.6, 'left_e1': 0.2}, 'right': {'right_s0': 0.65, 'right_s1': -0.7, 'right_w0': -1.7, 'right_w1': 0.0, 'right_w2': -3.0, 'right_e0': 1.6, 'right_e1': 0.2}}

	def execute(self):

		print "Waiting - Press s to start"

		# Wait for starting key press
		while self.waiting:
			pass

		# Training game
		if self.gameid == 'train':
			sequence = [5, 2, 3, 2, 5, 4, 3, 1, 5, 3, 2, 4, 5]
		# Game 1:
		elif self.gameid == 'g1':
			sequence = [5, 2, 3, 2, 5, 4, 3, 1, 5, 3, 2, 4, 5, 3, 1, 3, 5, 1, 2, 1, 5]
		# Game 2:
		elif self.gameid == 'g2':
			sequence = [5, 1, 3, 2, 5, 1, 2, 1, 5, 3, 1, 4, 5, 4, 2, 1, 5, 3, 1, 3, 5]
		# Game 3: 
		elif self.gameid == 'g3':
			sequence = [5, 1, 4, 3, 5, 2, 3, 1, 5, 4, 1, 3, 5, 1, 4, 1, 5, 1, 4, 1, 5]
		# Free-play game: Baxter makes up a game based on allowed moves
		elif self.gameid == 'free':
			if self.mode == 'new':
				# In the free play mode, Baxter will invent a new game that can be reloaded later with 'load'
				sequence = [] 
				oldMotion = 0
				motion_set = [1, 2, 3, 4]
				for num in range(21):
					if num % 4 == 0:
						# Put rest time in at fixed intervals
						sequence.append(5)
						oldMotion = sequence[num]
					else:
						# Pick motion to append
						randPick = random.choice(motion_set)
						while randPick == oldMotion:
							randPick = random.choice(motion_set)
						sequence.append(randPick)
						oldMotion = sequence[num]

				# Name file for saving motion pattern
				file_name = 'RandomRobotYogaSequence.csv'
				f = open(file_name, "a")
				f.writelines(["%s\n" % item  for item in sequenceTemp])
				f.close()
			# Game rep 2
			elif self.mode == 'load':
				# In the game load mode, Baxter will load game that was just taught to play again
				sequence = []
				# Import previous random data
				file_name = 'Subject' + subject + 'RandomRobotYogaSequence.csv'
				with open(file_name, 'rb') as csvfile:
					csvReader = csv.reader(csvfile)
					for column in csvReader:
						for elem in column:
							sequence.append(int(elem))

		motion_dict = {1:'midSide',2:'topSide',3:'midFront',4:'topFront',5:'relax'}
		# Only activate after a motion sequence has been defined
		if len(sequence) != 0:
			for i in sequence: # sequence is the list of all the class labels that correspond to handclapping motions
				self._arm.move(self._move[motion_dict[i]],3.5)

				if i == 5:
					relaxTime = 3.0
					now = rospy.get_rostime().to_sec()
					fifth = relaxTime/5
					self._head.switchEmotion('Joy', 'Happy', fifth)

					while rospy.get_rostime().to_sec() - now < relaxTime + 0.1:
						if rospy.get_rostime().to_sec() - now > fifth*5.0:
							self._head.changeColor("Blue")
						elif rospy.get_rostime().to_sec() - now > fifth*4.0:
							self._head.changeColor("Red")
						elif rospy.get_rostime().to_sec() - now > fifth*3.0:
							self._head.changeColor("Orange")
						elif rospy.get_rostime().to_sec() - now > fifth*2.0:
							self._head.changeColor("Yellow")
						elif rospy.get_rostime().to_sec() - now > fifth:
							self._head.changeColor("Green")

				else:
					now = rospy.get_rostime().to_sec()
					fifth = self.holdLength/5

					while rospy.get_rostime().to_sec() - now < self.holdLength + 0.1:
						if rospy.get_rostime().to_sec() - now > fifth*5.0:
							self._head.changeColor("Blue")
						elif rospy.get_rostime().to_sec() - now > fifth*4.0:
							self._head.changeColor("Red")
						elif rospy.get_rostime().to_sec() - now > fifth*3.0:
							self._head.changeColor("Orange")
						elif rospy.get_rostime().to_sec() - now > fifth*2.0:
							self._head.changeColor("Yellow")
						elif rospy.get_rostime().to_sec() - now > fifth:
							self._head.changeColor("Green")

		# End game sequence
		self.waiting = True

		print "Waiting - Press s to end"

		# Wait for ending key press
		while self.waiting:
			pass

		# Slowly assume ending positions
		self._head.changeColor('Blue')
		self._head.changeEmotion('Happy')
		self._arm.move(self.endPose,6.0)
	
class FlamencoGame(Game):
	def __init__(self, engine, arm, gameid, repetition):
		# Run super initializer
		Game.__init__(self, engine, arm)
		thread.start_new_thread(self._head.run, (5,))
		self._move = {}
		self.gameid = gameid
		self.waiting = True

		if repetition == 'train':
			self.numDanceMoves -= 5

		# Ready pose targets
		self._move['ready'] = {'left': {'left_s0': -0.25, 'left_s1': 0.15, 'left_w0': 1.55, 'left_w1': -1.5, 'left_w2': 2.45, 'left_e0': -1.3, 'left_e1': 1.6}, 'right': {'right_s0': 0.25, 'right_s1': 0.15, 'right_w0': -1.55, 'right_w1': -1.5, 'right_w2': -2.45, 'right_e0': 1.3, 'right_e1': 1.6}}
		self._move['ready_prep'] = {'left': {'left_s0': -0.05, 'left_s1': 0.16, 'left_w0': 1.56, 'left_w1': -1.57, 'left_w2': 2.45, 'left_e0': -1.28, 'left_e1': 1.55}, 'right': {'right_s0': 0.05, 'right_s1': 0.16, 'right_w0': -1.56, 'right_w1': -1.57, 'right_w2': -2.45, 'right_e0': 1.28, 'right_e1': 1.55}}

		# Left ole
		self._move['left_ole'] = {'left': {'left_s0': -0.036, 'left_s1': -0.041, 'left_w0': 0.681, 'left_w1': 0.899, 'left_w2': 2.99, 'left_e0': -2.897, 'left_e1': 1.69}, 'right': {'right_s0': -0.179, 'right_s1': -0.011, 'right_w0': -2.81, 'right_w1': -0.661, 'right_w2': 1.0, 'right_e0': 1.0, 'right_e1': 2.0}}
		self._move['left_ole_prep'] = {'left': {'left_s0': 0.087, 'left_s1': 0.249, 'left_w0': 0.914, 'left_w1': 0.706, 'left_w2': 2.333, 'left_e0': -2.101, 'left_e1': 1.79}, 'right': {'right_s0': -0.293, 'right_s1': -0.305, 'right_w0': -2.546, 'right_w1': -0.736, 'right_w2': 0.0, 'right_e0': 2.0, 'right_e1': 2.0}}

		# Right ole
		self._move['right_ole'] = {'left': {'left_s0': 0.179, 'left_s1': -0.011, 'left_w0': 2.81, 'left_w1': -0.661, 'left_w2': -1.0, 'left_e0': -1.0, 'left_e1': 2.0}, 'right': {'right_s0': -0.036, 'right_s1': -0.041, 'right_w0': -0.681, 'right_w1': 0.899, 'right_w2': -2.99, 'right_e0': 2.897, 'right_e1': 1.69}}
		self._move['right_ole_prep'] = {'left': {'left_s0': 0.293, 'left_s1': -0.305, 'left_w0': 2.546, 'left_w1': -0.736, 'left_w2': 0.0, 'left_e0': -2.0, 'left_e1': 2.0}, 'right': {'right_s0': -0.087, 'right_s1': 0.249, 'right_w0': -0.914, 'right_w1': 0.706, 'right_w2': -2.333, 'right_e0': 2.101, 'right_e1': 1.79}}

		# Left clap
		self._move['left_clap'] = {'left': {'left_s0': 0.331, 'left_s1': 0.023, 'left_w0': 1.565, 'left_w1': -1.356, 'left_w2': 2.145, 'left_e0': -1.75, 'left_e1': 1.672}, 'right': {'right_s0': 1.0, 'right_s1': -0.149, 'right_w0': -1.357, 'right_w1': -1.282, 'right_w2': -2.944, 'right_e0': 1.492, 'right_e1': 1.1}}
		self._move['left_clap_prep'] = {'left': {'left_s0': 0.478, 'left_s1': 0.176, 'left_w0': 1.734, 'left_w1': -1.227, 'left_w2': 1.839, 'left_e0': -1.82, 'left_e1': 1.664}, 'right': {'right_s0': 0.657, 'right_s1': 0.083, 'right_w0': -1.514, 'right_w1': -1.3, 'right_w2': -2.907, 'right_e0': 1.782, 'right_e1': 1.441}}

		# Right clap
		self._move['right_clap'] = {'left': {'left_s0': -1.0, 'left_s1': -0.149, 'left_w0': 1.357, 'left_w1': -1.282, 'left_w2': 2.944, 'left_e0': -1.492, 'left_e1': 1.1}, 'right': {'right_s0': -0.331, 'right_s1': 0.023, 'right_w0': -1.565, 'right_w1': -1.356, 'right_w2': -2.145, 'right_e0': 1.75, 'right_e1': 1.672}}
		self._move['right_clap_prep'] = {'left': {'left_s0': -0.657, 'left_s1': 0.083, 'left_w0': 1.514, 'left_w1': -1.3, 'left_w2': 2.907, 'left_e0': -1.782, 'left_e1': 1.441}, 'right': {'right_s0': -0.478, 'right_s1': 0.176, 'right_w0': -1.734, 'right_w1': -1.227, 'right_w2': -1.839, 'right_e0': 1.82, 'right_e1': 1.664}}
		
		self.audio = AudioManager()
		self.audio.loadMP3("songs/MP3/MalaguenaQuiet.ogg")

	def execute(self):

		self._arm.move(self._move['ready_prep'], 6.0)

		print "Waiting - Press s to start"

		# Game 1:
		if self.gameid == 'g1':
			sequenceTemp = [1, 4, 2, 3, 1, 3, 4, 4, 1, 4]
		# Game 2:
		elif self.gameid == 'g2':
			sequenceTemp = [4, 1, 3, 2, 3, 3, 4, 1, 4, 3]
		# Game 3: 
		elif self.gameid == 'g3':
			sequenceTemp = [3, 4, 3, 1, 4, 2, 3, 2, 3, 3]
		# Free-play game: Baxter makes up a game based on allowed moves
		elif self.gameid == 'free':
			if self.mode == 'new':
				# In the free play mode, Baxter will invent a new game that can be reloaded later with 'load'
				sequenceTemp = [] 
				oldMove = 0
				for num in range(10):
					# Pick motion to append
					if oldMove == 1:
						newMove = random.choice([3,4])
					elif oldMove == 2:
						newMove = random.choice([3,4])
					else:
						newMove = random.choice([1,2,3,4])
					sequenceTemp.append(newMove)
					oldMove = sequenceTemp[num]
				# Name file for saving motion pattern
				file_name = 'RandomRobotFlamencoSequence.csv'
				f = open(file_name, "a")
				f.writelines(["%s\n" % item  for item in sequenceTemp])
				f.close()
			# Game rep 2
			elif self.mode == 'load':
				# In the game load mode, Baxter will load game that was just taught to play again
				sequenceTemp = []
				# Import previous random data
				file_name = 'Subject' + subject + 'RandomRobotFlamencoSequence.csv'
				with open(file_name, 'rb') as csvfile:
					csvReader = csv.reader(csvfile)
					for column in csvReader:
						for elem in column:
							sequenceTemp.append(int(elem))

		motion_dict = {1:'left_ole',2:'right_ole',3:'left_clap',4:'right_clap'}
		
		gameMoves = 2
		
		for j in range(self.numDanceMoves - 1):
			sequence = sequenceTemp[0:gameMoves]
			clapInd = 0

			self.waiting = True
			print "Press s to start next dance repetition"
			# Wait for starting key press
			while self.waiting:
				pass

			self._head.changeColor('Yellow')
			self._head.changeEmotion('Happy')
			moveDuration = 1.5

			# Start music and do routine
			songStartTime = rospy.get_rostime().to_sec()
			self.audio.playMP3()
			if sequence[0] == 3 or sequence[0] == 4:
				rospy.sleep(1.0)
			else:
				rospy.sleep(0.5)
			
			if len(sequence) != 0:
				for i in sequence: # sequence is the list of all the class labels that correspond to handclapping motions
					desired_move = motion_dict[i]
					prep_name = desired_move + '_prep'
					if i == 3 or i == 4:
						self._arm._motion_engine.do_handClapMove(self._arm._left_arm, self._arm._right_arm, self._move[prep_name], self._move[desired_move], moveDuration, True)
						self._head.switchEmotion('Joy','Happy',0.4)
						if clapInd < gameMoves-1:
							if sequence[clapInd + 1] == 3 or sequence[clapInd + 1] == 4:
								rospy.sleep(0.5)
					else:
						self._arm.move(self._move[desired_move],moveDuration)
						if clapInd < gameMoves-1:
							if sequence[clapInd + 1] == 3 or sequence[clapInd + 1] == 4:
								rospy.sleep(0.5)
							else:
								rospy.sleep(0.5)
					clapInd += 1
			self.audio.pauseMP3()
			songEndTime = rospy.get_rostime().to_sec()
			rospy.sleep(0.4)

			self._head.changeColor('Blue')
			self._head.changeEmotion('Happy')

			self._arm.move(self._move['ready_prep'], 2.0)

			self._head.nod()

			self._head.changeColor('Purple')
			self._head.changeEmotion('Happy')

			now = rospy.get_rostime().to_sec()
			self.audio.playMP3()
			while rospy.get_rostime().to_sec() - now < songEndTime - songStartTime:
				pass

			self.audio.pauseMP3()

			self._head.changeColor('Blue')
			self._head.changeColor('Happy')

			gameMoves += 1

		self.waiting = True

		print "Waiting - Press s to end"

		# Wait for ending key press
		while self.waiting:
			pass

		# Slowly assume ending positions
		self._head.changeColor('Blue')
		self._head.changeEmotion('Happy')
		self._arm.move(self.endPose,6.0)

class Hello(Game):
	def __init__(self, engine, arm):
		# Run super initializer
		Game.__init__(self, engine, arm)
		thread.start_new_thread(self._head.run, (5,))
		self._move = {}
		self.waiting = True

		# Define relax pose and other mid and high side and front of body arm poses
		self._move['ready'] = {'left': {'left_s0': 0.75, 'left_s1': 0.8, 'left_w0': 1.5, 'left_w1': 0.0, 'left_w2': 3.0, 'left_e0': -1.3, 'left_e1': -0.0}, 'right': {'right_s0': -0.62, 'right_s1': 0.25, 'right_w0': -1.3, 'right_w1': -1.57, 'right_w2': -3.0, 'right_e0': 1.56, 'right_e1': 1.5}}
		self._move['wave'] = {'left': {'left_s0': 0.75, 'left_s1': 0.8, 'left_w0': 1.5, 'left_w1': 0.0, 'left_w2': 3.0, 'left_e0': -1.3, 'left_e1': -0.0}, 'right': {'right_s0': -0.62, 'right_s1': 0.25, 'right_w0': -2.3, 'right_w1': -1.57, 'right_w2': -3.0, 'right_e0': 1.56, 'right_e1': 1.55}}
		self._move['done'] = {'left': {'left_s0': 0.75, 'left_s1': 0.8, 'left_w0': 1.5, 'left_w1': 0.0, 'left_w2': 3.0, 'left_e0': -1.3, 'left_e1': -0.0}, 'right': {'right_s0': -0.95, 'right_s1': 1.0, 'right_w0': -1.9, 'right_w1': -0.01, 'right_w2': -3.0, 'right_e0': 1.5, 'right_e1': 0.2}}

	def execute(self):

		print "Waiting - Press s to start"

		# Wait for starting key press
		while self.waiting:
			pass

		self._arm.move(self._move['ready'], 6.0)
		self._head.changeEmotion('Joy')
		self._arm.move(self._move['wave'], 1.5)
		self._arm.move(self._move['ready'], 1.5)
		self._head.changeEmotion('Happy')
		self._arm.move(self._move['done'], 6.0)

		self.waiting = True

		print "Waiting - Press s to end"

		# Wait for ending key press
		while self.waiting:
			pass

		# Slowly assume ending positions
		self._head.changeColor('Blue')
		self._head.changeEmotion('Happy')
		self._arm.move(self.endPose,6.0)

class ResetAll(Game):
	def __init__(self, engine, arm):
		# Run super initializer
		Game.__init__(self, engine, arm)
		thread.start_new_thread(self._head.run, (5,))
		self._move = {}
		self.waiting = True

	def execute(self):

		# Tool for pulling out joint angles
		#self._arm.get_print_friendly_angles()

		print "Waiting - Press s to start"

		# Wait for starting key press
		while self.waiting:
			pass

		# Slowly assume the uniform default end pose
		self._head.changeColor('Blue')
		self._head.changeEmotion('Happy')
		self._arm.move(self.endPose,6.0)