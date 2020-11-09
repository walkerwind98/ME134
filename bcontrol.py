### Walker and Owen HW5 
### Bluetooth Control for wireless robot
#11/7/2020

import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from adafruit_servokit import ServoKit
from evdev import InputDevice, categorize, ecodes

gamepad = InputDevice('/dev/input/event0')

yBtn = 307
bBtn = 306
xBtn = 305
aBtn = 304

kit = ServoKit(channels = 16)
kit.servo[0].set_pulse_width_range(400, 2600)
kit.servo[1].set_pulse_width_range(400,2600)
kit.servo[2].set_pulse_width_range(400,2600)
currentangle = 130
frontup0 = 0
frontup1 = 90
frontflat0 = 90
frontflat1 = 0
backflat = 0
backup1 = 60
backup2 = 120
delay = .5
###
kit.servo[0].angle = frontflat0
kit.servo[1].angle = frontflat1
kit.servo[2].angle = backflat

#make sure that every limb is flat to start


def Forward(kit):
	
	#raise front legs
	kit.servo[0].angle = frontup0
	kit.servo[1].angle = frontup1
	time.sleep(delay)

	#raise back
	kit.servo[2].angle = backup1
	time.sleep(delay)
	#lower front legs and raise back more
	kit.servo[0].angle = frontflat0
	kit.servo[1].angle = frontflat1
	kit.servo[2].angle = backup2
	time.sleep(delay)
	#lower back
	kit.servo[2].angle = backflat

	return kit
###Function to flail wildly

###Function to turn right
def Right(kit):
	
	#raise front legs
	kit.servo[0].angle = frontup0
	time.sleep(delay)

	#raise back
	#kit.servo[2].angle = backup1
	#time.sleep(delay)
	#lower front legs and raise back more
	kit.servo[0].angle = frontflat0
	#kit.servo[2].angle = backup2
	time.sleep(delay)
	#lower back
	#kit.servo[2].angle = backflat

	return kit

###Function to turn left
def Left(kit):
	
	#raise front legs
	kit.servo[1].angle = frontup1
	time.sleep(delay)

	#raise back
	kit.servo[2].angle = backup1
	time.sleep(delay)
	#lower front legs and raise back more
	kit.servo[1].angle = frontflat1
	kit.servo[2].angle = backup2
	time.sleep(delay)
	#lower back
	kit.servo[2].angle = backflat

	return kit

def FlailUncontrollably(kit):
	start = time.time()
	curr = time.time()
	while (curr - start) < 3:
		curr = time.time()
		kit.servo[0].angle = 0
		kit.servo[1].angle = 0
		kit.servo[2].angle = 0
		time.sleep(delay)
		kit.servo[0].angle = frontup
		kit.servo[1].angle = frontflat
		kit.servo[2].angle = backup1
		time.sleep(.05)


print(gamepad)

for event in gamepad.read_loop():
	if event.type == ecodes.EV_KEY:
		if event.code == yBtn:
			print('yBtn')
			if event.value is 1:
				Forward(kit)
		elif event.code == bBtn:
			Left(kit)
			print('bBtn')
		elif event.code == xBtn:
			Right(kit)
			print('xBtn')
		elif event.code == aBtn:
			FlailUncontrollably(kit)
			print('aBtn')
		print(event.value)
		time.sleep(.1)
		print('-------')
