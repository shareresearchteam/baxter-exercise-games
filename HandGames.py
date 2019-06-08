#!/usr/bin/env python

import cProfile

# Import typical ROS, OS, and command argument items
import rospy
import thread
from threading import Thread
import os
import sys
import argparse

# Import Baxter sensors, interface, etc.
import baxter_external_devices
from sensor_msgs.msg import (
	Imu, Image,								
)	
import baxter_interface
from baxter_interface import CHECK_VERSION

# Import packages for interfacing with robot wrist sensors,
# moving arms, playing games, etc.
from HandEngine import HandEngine
from Motion import CubicMotion
from Game import *
from AdapterClass import NaomiMove
from ArmGenerator import ClappingArm

def handle_keyboard(engine, mgame):
	'Funcion for allowing us to interface with processes via the keyboard'
	while not rospy.is_shutdown():
		# Get keyboard input only when it changes (-1 input facilitates this)
		c = baxter_external_devices.getch(-1)
		if c:
			if c == 'c':
				engine._clapDetected('right')
			if c == 'x':
				engine._clapDetected('left')
			if c == 's':
				mgame.waiting = False
			if c == 'f':
				os._exit(0)
			if c == 'q':
				mgame.stop()

def main():
	'Start lots of processes and game'

	print("Initializing node... ")
	rospy.init_node("baxter_smooth_arm_motion")
	print("Getting robot state... ")
	rs = baxter_interface.RobotEnable(CHECK_VERSION)
	print("Enabling robot... ")
	rs.enable()
	print("Running. Ctrl-c or f to quit")

	# Start MotionEngine object for interfacing with wrist accelerometers
	motion_engine = MotionEngine()

	# Make arm objects
	left_arm = ClappingArm('left',100)
	right_arm = ClappingArm('right',100)

	# Get ready to start threads to allow arms to move nicely and simultaneously
	left_arm_thread = Thread(target = left_arm.main, args = ())
	right_arm_thread = Thread(target = right_arm.main, args = ())   

	# Start threads for controlling arms
	left_arm_thread.start()
	right_arm_thread.start()

	game = None
	parser = argparse.ArgumentParser(description = 'Input game name. {mimic / stretch / strength / teach}')
	parser.add_argument('strs', metavar = 'N', type = str, nargs = '+', help = 'A string representing the game type')
	foo = parser.parse_args()
	gameType = (foo.strs[0])

	# Decide whether or not to save data
	collectData = True
	fileName = "S24_" + gameType + "_Rec1.csv"

	# Create arm adaptor for compatibility between Dylan code and my arm motions
	arm = NaomiMove(motion_engine, left_arm, right_arm)
	engine = HandEngine(arm, collectData, gameType.lower(), fileName)
	
	# Identify game selected in input arguments
	if gameType.lower() == "gamea": #mimic":
		game = MimicGame(engine, arm, 0) # enter rep through the game (add 1 each time person wins)
	elif gameType.lower() == "gameb": #"stretch":
		game = StretchGame(engine, arm, 'songs/TwoHand/sample', ['left', 'right']) #  enter indicator of what song to play
	elif gameType.lower() == "gamec": #"teach":
		game = TeachGame(engine, arm)
	elif gameType.lower() == "gamed": #"agility":
		game = AgilityGame(engine, arm, 0) # enter rep through the game (add 1 each time person wins)
	elif gameType.lower() == "gamee": #"strength":
		game = StrengthGame(engine, arm, "songs/MP3/RockyQuiet.mp3", 'g1', 'train', 0) # enter power song file, number of boxing routine ('g1','g2','g3','free'), whether we're doing 'train' or 'full' rep, and rep through the game (add 1 each time person wins)
	elif gameType.lower() == "gamef": #"handclap":
		game = HandclapGame(engine, arm, 'g1','new','train') # enter name of clapping game to be played ('g1','g2','g3','free'), and mode ('new' or 'load'), and whether we're doing 'train' or 'full' rep
	elif gameType.lower() == "gameg": #"roboga":
		game = RobogaGame(engine, arm, 'train', 'new') # enter number of yoga routine to be completed ('train','g1','g2','g3','free')
	elif gameType.lower() == "gameh": #"flamenco":
		game = FlamencoGame(engine, arm, 'g1', 'train') # enter name of flamenco routine to be completed ('g1','g2','g3') and whether we're doing 'train' or 'full' rep
	elif gameType.lower() == "hello":
		game = Hello(engine, arm)
	elif gameType.lower() == "resetarms":
		game = ResetAll(engine, arm)
	else:
		sys.exit(0)

	# Start left and right accelerometer callbacks
	rospy.Subscriber('/robot/accelerometer/right_accelerometer/state', Imu, engine.callback, "right")
	rospy.Subscriber('/robot/accelerometer/left_accelerometer/state', Imu, engine.callback, "left")
	
	# Start keyboard handler
	thread.start_new_thread(handle_keyboard, (engine, game))

	# Start game process
	game.execute()

	# Tell arm threads that we done
	left_arm.shutdown()
	right_arm.shutdown()

	# Wait for threads to properly shut down
	left_arm_thread.join()
	right_arm_thread.join()

if __name__ == "__main__":
    main()
