#Walker Wind, Yanxi Piao, Dylan Wagman
# 10/10/20
# Initial Writing Robot 
#For this project, we are building a raspberry pi driven initial writing robotic arm
#using two MG996R servos and a micro servo.

#Import necessary libraries
from board import SCL, SDA
import busio, numpy, math, csv
import time
from math import atan, acos, sin, cos, degrees, pi, atan2,sqrt
import numpy
# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo




#Constants
xmax=180 # maximum writing distance in x-axis
ymax=60 # maximum writing distance in y-axis
current_x=1 # record the current pen position x
current_y=200 # record the current pen position y
font=20 # [milimiter]
list =[] # for coordinate lists
x_list=[] # import x axis data
y_list=[] # import y axis data
theta_list=[] # import angle data

######################
# Functions used in the code
#######################################


# The purpose of this function use inverse kinematics to convert an X Y position 
# to angles that the \
def xy_to_ang(X,Y):
    #global q
    q = [0,0] #initialize for now
    #move_to_cord(XE,YE) moves arm to coords of end effector, xe and ye
    L_1 = 145 #mm link length 1
    L_2 = 205 # link2 + end effector width
    #solve for angles (q1 and q2), analytical
    q[1] = atan2(sqrt(1-(((X**2)+(Y**2)-(L_1**2)-(L_2**2))/(2*L_1*L_2))**2),((X**2)+(Y**2)-(L_1**2)-(L_2**2))/(2*L_1*L_2))
    if q[1] < 0:
        q[1] = atan2(-1*sqrt(1-(((X**2)+(Y**2)-(L_1**2)-(L_2**2))/(2*L_1*L_2))**2),((X**2)+(Y**2)-(L_1**2)-(L_2**2))/(2*L_1*L_2))
    q[0] = atan2(Y,X)-atan2(L_2*sin(q[1]),L_1+L_2*cos(q[1]))

    # or
    #q[1] = -1*acos(((X**2)+(Y**2)-(L_1**2)-(L_2**2))/(2*L_1*L_2)) #q2
    #q[0] = atan(Y/X)+atan((L_2*sin(q[1]))/(L_1+L_2*cos(q[1]))) #q1
    
     #accounting for the reverse direction of servo1
    q = [int(degrees(q[0])),int(180-degrees(q[1]))]
    return q

#For the following functions assume
# y falls between 200 and 270
# x falls between 0 and 50 or 50 to 100           
def Dfunction(x0,y0,servo0,servo1,servo2):
    numvalues = 100
    line1x= [int(x0)]*numvalues
    line1y=numpy.linspace(y0,y0+50,numvalues)
    line2y=numpy.linspace(y0+50,y0,numvalues)
    line2x =line2y
    line2y=line1y
    line1y=numpy.linspace(y0+50,y0,numvalues)
    print(line2y)
    for i in range(len(line2y)):
        line2x[i]=sqrt(25**2-(line2y[i]-(25+y0))**2)+x0
    print(line2x)

    #x is a list of x values that are endpoints
    #y is a list of y values
    for j in range(len(line1x)): #this may not be the correct range
        angles = xy_to_ang(line1x[j],line1y[j])
        print(angles)
        if j is 1:
            servo2.angle = 45 #the pen will move down before it moves between points
            time.sleep(1)
        servo0.angle = angles[0]
        time.sleep(.05)
        servo1.angle = angles[1]
        time.sleep(.05)
        #print('next line')
    for j in range(len(line1x)): #this may not be the correct range
        angles = xy_to_ang(line2x[j],line2y[j])
        print(angles)
        
        servo0.angle = angles[0]
        time.sleep(.05)
        servo1.angle = angles[1]
        time.sleep(.05)
        if j is len(line1x):
            servo2.angle = 0 #whatever angle to go up

def WFunction(x0,y0,servo0,servo1,servo2):
    coords=[[81.00, 200.00],[82.58, 203.16],[84.16, 206.32],[85.74, 209.47],[87.32, 212.63],[88.89, 215.79],[90.47, 218.95],[92.05, 222.11],[93.63, 225.26],[95.21, 228.42],[96.79, 231.58],[98.37, 234.74],[99.95, 237.89],[101.53, 241.05],[103.11, 244.21],[104.68, 247.37],[106.26, 250.53],[107.84, 253.68],[109.42, 256.84],[111.00, 260.00],[111.00, 260.00],[111.53, 258.95],[112.05, 257.89],[112.58, 256.84],[113.11, 255.79],[113.63, 254.74],[114.16, 253.68],[114.68, 252.63],[115.21, 251.58],[115.74, 250.53],[116.26, 249.47],[116.79, 248.42],[117.32, 247.37],[117.84, 246.32],[118.37, 245.26],[118.89, 244.21],[119.42, 243.16],[119.95, 242.11],[120.47, 241.05],[121.00, 240.00],[121.00, 240.00],[121.53, 241.05],[122.05, 242.11],[122.58, 243.16],[123.11, 244.21],[123.63, 245.26],[124.16, 246.32],[124.68, 247.37],[125.21, 248.42],[125.74, 249.47],[126.26, 250.53],[126.79, 251.58],[127.32, 252.63],[127.84, 253.68],[128.37, 254.74],[128.89, 255.79],[129.42, 256.84],[129.95, 257.89],[130.47, 258.95],[131.00, 260.00],[131.00, 260.00],[132.58, 256.84],[134.16, 253.68],[135.74, 250.53],[137.32, 247.37],[138.89, 244.21],[140.47, 241.05],[142.05, 237.89],[143.63, 234.74],[145.21, 231.58],[146.79, 228.42],[148.37, 225.26],[149.95, 222.11],[151.53, 218.95],[153.11, 215.79],[154.68, 212.63],[156.26, 209.47],[157.84, 206.32],[159.42, 203.16],[161.00, 200.00]]
    if x0 is not 80:
        coords = [[1.00, 200.00],[2.58, 203.16],[4.16, 206.32],[5.74, 209.47],[7.32, 212.63],[8.89, 215.79],[10.47, 218.95],[12.05, 222.11],[13.63, 225.26],[15.21, 228.42],[16.79, 231.58],[18.37, 234.74],[19.95, 237.89],[21.53, 241.05],[23.11, 244.21],[24.68, 247.37],[26.26, 250.53],[27.84, 253.68],[29.42, 256.84],[31.00, 260.00],[31.00, 260.00],[31.53, 258.95],[32.05, 257.89],[32.58, 256.84],[33.11, 255.79],[33.63, 254.74],[34.16, 253.68],[34.68, 252.63],[35.21, 251.58],[35.74, 250.53],[36.26, 249.47],[36.79, 248.42],[37.32, 247.37],[37.84, 246.32],[38.37, 245.26],[38.89, 244.21],[39.42, 243.16],[39.95, 242.11],[40.47, 241.05],[41.00, 240.00],[41.00, 240.00],[41.53, 241.05],[42.05, 242.11],[42.58, 243.16],[43.11, 244.21],[43.63, 245.26],[44.16, 246.32],[44.68, 247.37],[45.21, 248.42],[45.74, 249.47],[46.26, 250.53],[46.79, 251.58],[47.32, 252.63],[47.84, 253.68],[48.37, 254.74],[48.89, 255.79],[49.42, 256.84],[49.95, 257.89],[50.47, 258.95],[51.00, 260.00],[51.00, 260.00],[52.58, 256.84],[54.16, 253.68],[55.74, 250.53],[57.32, 247.37],[58.89, 244.21],[60.47, 241.05],[62.05, 237.89],[63.63, 234.74],[65.21, 231.58],[66.79, 228.42],[68.37, 225.26],[69.95, 222.11],[71.53, 218.95],[73.11, 215.79],[74.68, 212.63],[76.26, 209.47],[77.84, 206.32],[79.42, 203.16],[81.00, 200.00]]

    for j in range(len(coords)): #this may not be the correct range
        angles = xy_to_ang(coords[j][0],coords[j][1])

        print(angles)
        if j is 1:
            servo2.angle = 45 #the pen will move down before it moves between points
            time.sleep(1)
        servo0.angle = angles[0]
        time.sleep(.05)
        servo1.angle = angles[1]
        time.sleep(.05)

def Pfunction(x0,y0,servo0,servo1,servo2):
    coords = [[40, 260],[40, 257],[40, 254],[40, 251],[40, 247],[40, 244],[40, 241],[40, 238],[40, 235],[40, 232],[40, 228],[40, 225],[40, 222],[40, 219],[40, 216],[40, 213],[40, 209],[40, 206],[40, 203],[40, 200],[40, 200],[39, 200],[38, 200],[37, 200],[36, 200],[35, 200],[34, 200],[33, 200],[32, 200],[31, 200],[29, 200],[28, 200],[27, 200],[26, 200],[25, 200],[24, 200],[23, 200],[22, 200],[21, 200],[20, 200],[20, 200],[20, 201],[20, 202],[20, 203],[20, 204],[20, 205],[20, 206],[20, 207],[20, 208],[20, 209],[20, 211],[20, 212],[20, 213],[20, 214],[20, 215],[20, 216],[20, 217],[20, 218],[20, 219],[20, 220],[20, 220],[21, 219],[22, 218],[23, 217],[24, 216],[25, 215],[26, 214],[27, 213],[28, 212],[29, 211],[31, 209],[32, 208],[33, 207],[34, 206],[35, 205],[36, 204],[37, 203],[38, 202],[39, 201],[40, 200]]
    #if x0 is not 80:
    #    coords = [[1.00, 200.00],[2.58, 203.16],[4.16, 206.32],[5.74, 209.47],[7.32, 212.63],[8.89, 215.79],[10.47, 218.95],[12.05, 222.11],[13.63, 225.26],[15.21, 228.42],[16.79, 231.58],[18.37, 234.74],[19.95, 237.89],[21.53, 241.05],[23.11, 244.21],[24.68, 247.37],[26.26, 250.53],[27.84, 253.68],[29.42, 256.84],[31.00, 260.00],[31.00, 260.00],[31.53, 258.95],[32.05, 257.89],[32.58, 256.84],[33.11, 255.79],[33.63, 254.74],[34.16, 253.68],[34.68, 252.63],[35.21, 251.58],[35.74, 250.53],[36.26, 249.47],[36.79, 248.42],[37.32, 247.37],[37.84, 246.32],[38.37, 245.26],[38.89, 244.21],[39.42, 243.16],[39.95, 242.11],[40.47, 241.05],[41.00, 240.00],[41.00, 240.00],[41.53, 241.05],[42.05, 242.11],[42.58, 243.16],[43.11, 244.21],[43.63, 245.26],[44.16, 246.32],[44.68, 247.37],[45.21, 248.42],[45.74, 249.47],[46.26, 250.53],[46.79, 251.58],[47.32, 252.63],[47.84, 253.68],[48.37, 254.74],[48.89, 255.79],[49.42, 256.84],[49.95, 257.89],[50.47, 258.95],[51.00, 260.00],[51.00, 260.00],[52.58, 256.84],[54.16, 253.68],[55.74, 250.53],[57.32, 247.37],[58.89, 244.21],[60.47, 241.05],[62.05, 237.89],[63.63, 234.74],[65.21, 231.58],[66.79, 228.42],[68.37, 225.26],[69.95, 222.11],[71.53, 218.95],[73.11, 215.79],[74.68, 212.63],[76.26, 209.47],[77.84, 206.32],[79.42, 203.16],[81.00, 200.00]]

    for j in range(len(coords)): #this may not be the correct range
        angles = xy_to_ang(coords[j][0],coords[j][1])

        print(angles)
        if j is 1:
            servo2.angle = 45 #the pen will move down before it moves between points
            time.sleep(1)
        servo0.angle = angles[0]
        time.sleep(.05)
        servo1.angle = angles[1]
        time.sleep(.05)

def Yfunction(x0,y0,servo0,servo1,servo2):
    coords = [[100, 200],[99, 202],[98, 203],[97, 205],[96, 206],[95, 208],[94, 209],[93, 211],[92, 213],[91, 214],[89, 216],[88, 217],[87, 219],[86, 221],[85, 222],[84, 224],[83, 225],[82, 227],[81, 228],[80, 230],[80, 230],[79, 228],[78, 227],[77, 225],[76, 224],[75, 222],[74, 221],[73, 219],[72, 217],[71, 216],[69, 214],[68, 213],[67, 211],[66, 209],[65, 208],[64, 206],[63, 205],[62, 203],[61, 202],[60, 200],[60, 200],[60, 203],[60, 206],[60, 209],[60, 213],[60, 216],[60, 219],[60, 222],[60, 225],[60, 228],[60, 232],[60, 235],[60, 238],[60, 241],[60, 244],[60, 247],[60, 251],[60, 254],[60, 257],[60, 260]]
    for j in range(len(coords)): #this may not be the correct range
        angles = xy_to_ang(coords[j][0],coords[j][1])

        print(angles)
        if j is 1:
            servo2.angle = 45 #the pen will move down before it moves between points
            time.sleep(1)
        servo0.angle = angles[0]
        time.sleep(.05)
        servo1.angle = angles[1]
        time.sleep(.05)

######################################
# Initial Setup
######################################

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 50


# The pulse range is 1000 - 2000 by default.
servo0 = servo.Servo(pca.channels[7], min_pulse=400, max_pulse=2400) #standard servo range
servo1 = servo.Servo(pca.channels[3], min_pulse=600, max_pulse=2400) #standard servo range
servo2 = servo.Servo(pca.channels[11], min_pulse=600, max_pulse=2400) #micro servo range


servo2.angle = 15 #whatever angle it needs to be to be up


x = input("Are you ready to begin? (type y when ready)")
Yfunction(50,200,servo0,servo1,servo2)


#Pfunction(50,200,servo0,servo1,servo2)
servo2.angle = 0
time.sleep(1)
Pfunction(80,200,servo0,servo1,servo2)

#WFunction(80,200,servo0,servo1,servo2)

pca.deinit()