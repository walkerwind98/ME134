# Walker Wind and Evan Slack
# HW4 Balancing Robot
# 10/21/20

# import necessary libraries 
from board import SCL, SDA
import busio, numpy, math, csv, time, sys, numpy, os, smbus
from math import atan, acos, sin, cos, degrees, pi, atan2,sqrt

# import the PCA9685 module
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from adafruit_servokit import ServoKit

# import the IMU module
from MPU950 import MPU9250

# import the PID controller module
from simple_pid import PID

import matplotlib.pyplot as plt

# Converts the speed output of the PID to angle and time delay and controls servos
def Speed2Angle(sspeed, anglestep, kit, currentangle): 
	
	# If the magnitude of the speed written to the servo is over 325 deg/s, cap the speed
	if sspeed > 325 or sspeed < -325:
		sspeed = 325
	
	# Compute time
	deltat = abs(anglestep/sspeed)
	
	# If system is balanced, time delay will be too long, must break sooner
	if deltat > .25:
		return currentangle
		
	# Compute direction and new angle
	direction = numpy.abs(sspeed)/sspeed
	newangle = currentangle - anglestep*direction
	
	# Servo range is limited by the size of our linear track
	if newangle >= 180:
		newangle = 180 
	if newangle <= 90:
		newangle = 90
		
	# Set servo angles
	kit.servo[0].angle = newangle
	currentangle = newangle
	time.sleep(deltat)
	return currentangle


# Setting up the IMU
imu = MPU9250()

# Initialize Motors
kit = ServoKit(channels = 16)
kit.servo[0].set_pulse_width_range(400, 2600)

currentangle = 130

kit.servo[0].angle = currentangle
time.sleep(1)

# PID Controller Values

#kp = 6
#kd = .3
#ki = 0

kp = 6
kd = .3
ki = 0

#initialize the PID controller
pid = PID(kp,ki,kd,setpoint = 0)
pid.sample_time = .05 #puts a cap on how fast the samples will be processed

z = input("Ready to begin? Type anything to commence balancing.")

# Initialize variables to plot
count = 1 
speedPlot = []
gyroPlot = []
referenceSpeed = []


while count < 400:
	gyro = imu.readGyro()
	
	thetadot = (gyro['z'] - 4) # Gyro returns +4 with no motion, must get rid of that
	sspeed = pid(thetadot) # Get PID control
	currentangle = Speed2Angle(sspeed, 2, kit, currentangle) # Calculate servo angle from gyro velocity
	
	# Print variables
	print("sspeed is: ",sspeed)
	print("gyro angular speed is: ",gyro['z'])
	print("current angle is: ", currentangle)
	
	speedPlot.append(sspeed)
	gyroPlot.append(thetadot*10)
	referenceSpeed.append(0)
	
	count = count + 1
	
# Plot response to better determine PID values	
speedline, = plt.plot(speedPlot)
referencespeedline, = plt.plot(referenceSpeed)
gyrodataline, = plt.plot(gyroPlot)
plt.xlabel('Time')
plt.legend([speedline,referencespeedline,gyrodataline], ["Speed","Reference Speed","Angular Velocity"])
plt.show()
plt.savefig('Plot.png')



	
	
