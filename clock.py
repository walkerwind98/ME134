#Walker Wind and Audrey Balaska
#9/22/20
#HW2 - Clock

# The purpose of this project is to make a clock that is powered by 
# a non-continuous servo motor. In this code, I will be controlling two servos to
# control the rotation of two ratchet and pawl mechanism wheels. 
# The wheels are telescoping, and will have a minute and hour hand that show the analog time.

# While it is simple to trigger servos to move once per hour or per minute, it's a little bit harder
# to keep track of the exact time. In order to accurately do this, 

from board import SCL, SDA
import busio
import time as time
import math

# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

#######################################
# Functions used in the code
#######################################
#function to make a a greeting to the user based on the time of day
def greeting(hour,digitaldisplay):
    #make a greeting given the time of day
    greeting = ""
    if hour > 12 and hour <18:
        greeting = "Good Afternoon! The current time in Boston is "
    if hour > 18:
        greeting = "Good Evening! The current time in Boston is "
    if hour < 12:
        greeting = "Good Morning! The current time in Boston is "

    print(greeting + digitaldisplay)


######################################
# Initial Setup
######################################

#angles needed for each wheel to move one increment

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 50


# The pulse range is 1000 - 2000 by default.
servo0 = servo.Servo(pca.channels[0], min_pulse=600, max_pulse=2600)
servo1 = servo.Servo(pca.channels[15], min_pulse=600, max_pulse=2400)

#upon initialization, make sure that the minute hand and hour hand are at 6:30

x = input("Are you ready to begin? (type y when ready)")


#make a while loop that will set the time until it is accurate to the minute
minutesteps = 1
hoursteps = 1
loopcounter = 0
currenthour = 6
currentminute = 30
while(True):
    #get the current local time and parse 
    digitaldisplay = time.asctime(time.localtime()).split()
    digitaldisplay = digitaldisplay[3]
    digitalclock = digitaldisplay.split(":")

    #Save the current minute, hour, seconds to a variables as ints
    hour = int(digitalclock[0])
    minute = int(digitalclock[1])
    second = int(digitalclock[2])
    if loopcounter is 0:
        greeting(hour,digitaldisplay)

    if hour > 12:
        hour = hour -12
  
    #IMPORTANT: ASSUME THAT THE INITIAL POSITION OF THE CLOCK IS 6:30 or 18:30
    #Determine the difference between the hours and minutes and the current time (which starts off at 6:30)
    #assume that hours wheel has 48 teeth, 4 per hour
    #assume that minute wheel has 60 teeth, 1 per minute
    if hour >= currenthour:
        hoursteps = 4*(hour-currenthour) + math.floor((minute-currentminute)/15)
    else:
        hoursteps = 4*(hour+currenthour) + math.floor((minute-currentminute)/15)

    if minute > 30:
        
        minutesteps = minute - currentminute
    else:
        minutesteps = minute + currentminute

    #Move the servos at a fast rate until it reaches the time
    angle0 = 30
    angle1 = 30
    if hoursteps > 0:
        for i in range(hoursteps):
             #Determine the angle needed for the
            deltangle = 30
            servo0.angle=0
            time.sleep(.1) #delay for 
            servo0.angle=deltangle
            time.sleep(.1)
            #servo0.angle=0
            #time.sleep(.5)
           
    if minutesteps > 0:
        for i in range(minutesteps):
            deltangle = 30
            servo1.angle=0
            time.sleep(.1) #delay for 
            servo1.angle=deltangle
            time.sleep(.1)
            servo1.angle=0
            #time.sleep(.5)
          
    loopcounter = loopcounter + 1
    currenthour = hour
    currentminute = minute
   
    
print("We completely" + str(loopcounter) + "loops to get the right time")

#need to check the time again to adjust the minute hand after the servo actions finish
pca.deinit()
