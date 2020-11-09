### Walker and Owen HW5 
### Bluetooth Control for wireless robot
#11/7/2020

import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from adafruit_servokit import ServoKit
from evdev import InputDevice, categorize, ecodes

class Lobster:

    def __init__(self, event):    

        # Controller mapping
        self.gamepad = InputDevice('/dev/input/' + event)
        self.yBtn = 307
        self.bBtn = 306
        self.xBtn = 305
        self.aBtn = 304

        # Setting up the servo intercommunication
        self.kit = ServoKit(channels = 16)

        self.frontLeft = self.kit.servo[5]
        self.frontRight = self.kit.servo[4]
        self.back = self.kit.servo[6]

        self.frontRight.set_pulse_width_range(400, 2600)
        self.frontLeft.set_pulse_width_range(400,2600)
        self.back.set_pulse_width_range(400,2600)

        #
        self.frontUp = 0
        self.frontFlat = 0
        self.backFlat = 0
        self.backUp = 90
        self.delay = .75

    def forward(self):
	
        #raise front legs
        self.frontRight.angle = self.frontUp 
        self.frontLeft.angle = self.frontUp + 90
        
        time.sleep(self.delay)

        #raise back
        self.back.angle = self.backUp
        
        time.sleep(self.delay)
        #lower front legs and raise back more
        self.frontRight.angle = self.frontFlat + 100
        self.frontLeft.angle = self.frontFlat
        
        time.sleep(.3)

        self.back.angle = self.backUp + 10
        time.sleep(.1)
        self.back.angle = self.backUp + 40
        
        time.sleep(1)
        #lower back
        self.back.angle = self.backFlat

    def left(self):
        self.frontLeft.angle = self.frontUp + 100
        time.sleep(1)
        self.frontLeft.angle = self.frontFlat
        time.sleep(1)

    def right(self):
        self.frontRight.angle = self.frontUp
        time.sleep(1)
        self.frontRight.angle = self.frontFlat + 100
        time.sleep(1)



lobster = Lobster('event0')

for event in lobster.gamepad.read_loop():
    if event.type == ecodes.EV_KEY:
        if event.code == lobster.yBtn:
            if event.value is 1:
                lobster.forward()
        elif event.code == lobster.bBtn:
            if event.value is 1:
                lobster.left()   
        elif event.code == lobster.xBtn:
            if event.value is 1:
                lobster.right()
        time.sleep(.2)
