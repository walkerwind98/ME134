#Walker Wind, Yanxi Piao, Dylan Wagman
# 10/10/20
# Initial Writing Robot 
#For this project, we are building a raspberry pi driven initial writing robotic arm
#using two MG996R servos and a micro servo.

#Import necessary libraries
from board import SCL, SDA
import busio, numpy, math, csv
import time
from math import atan, acos, sin, cos, degrees, pi

# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

#Constants
xmax=180 # maximum writing distance in x-axis
ymax=60 # maximum writing distance in y-axis
current_x=100 # record the current pen position x
current_y=50 # record the current pen position y
font=20 # [milimiter]
list =[] # for coordinate lists
x_list=[] # import x axis data
y_list=[] # import y axis data
theta_list=[] # import angle data

#######################################
# Functions used in the code
#######################################


# The purpose of this function use inverse kinematics to convert an X Y position 
# to angles that the 
def xy_to_ang(X,Y):
    #global q
    q = [0,0] #initialize for now
    #move_to_cord(XE,YE) moves arm to coords of end effector, xe and ye
    L_1 = 125.73 #mm link length 1
    L_2 = 186.94 + 35 # link2 + end effector width
    #solve for angles (q1 and q2), analytical
    q[1] = acos(((X**2)+(Y**2)-(L_1**2)-(L_2**2))/(2*L_1*L_2)) #q2
    q[0] = atan(Y/X)-atan((L_2*sin(q[1]))/(L_1+L_2*cos(q[1]))) #q1
    # or 
    ''' 
    q[1] = -1*acos((sqrt(X)+sqrt(Y)-sqrt(L_1)-sqrt(L_2))/(2*L_1*L_2)) #q2
    q[0] = atan(Y/X)+atan((L_2*sin(q[1]))/(L_1+L_2*cos(q[1]))) #q1
    '''
     #accounting for the reverse direction of servo1
    q = [int(degrees(q[0])),int(degrees(q[1]))]
    return q

# import data from csv file
def get_data(linput):
    global x_list
    global y_list
    global theta_list
    with open('/home/pi/ME134/HW3/alphabet.csv',"r") as f:
        reader=csv.DictReader(f)
        for row in reader:
            if row['char']==linput:
                for k in range(0,len(row['x']),2):
                    x_list.append(int(row['x'][k]))
                    y_list.append(int(row['y'][k]))
                for k in range(0,len(row['theta']),2):
                    theta_list.append(int(row['theta'][k]))
                    
def path_straight(x,y,servo0,servo1,servo2):
    global list
    global current_x
    global current_y
    list=[]
    for i in range(0, len(x), 2):
        a = x[i] * font
        b = y[i] * font
        print(a,'',b)
        dist = math.sqrt(math.pow((x[i] - x[i + 1]), 2) + math.pow((y[i] - y[i + 1]), 2))
        intervals_x = (x[i + 1] - x[i]) / (dist * font)
        intervals_y = (y[i + 1] - y[i]) / (dist * font)
        for k in range(int(dist * font)):
            a = intervals_x * font*k
            b = intervals_y * font*k
            list.append((current_x-a, current_y-b))
        print(list)
        print("...")

        #walker is drafting this section
        #the purpose of this section is to convert the xy coords into angles
        #and write it to the servo.
        #The x and y points should be adjusted to where it makes sense for the robot
        #define a zero point for x and y
        xoffset = 0
        yoffset = 0
        for j in range(len(list)): #this may not be the correct range
            angles = xy_to_ang(list[j][0] + xoffset,list[j][1] +yoffset)
            print(angles)
            if j is 1:
                servo2.angle = 0 #the pen will move down before it moves between points
                time.sleep(1)
            servo0.angle = angles[0]
            time.sleep(.1)
            servo1.angle = angles[1]
            time.sleep(.1)
            if j is len(list):
                servo2.angle = 45 #whatever angle to go up 
        list = [] 
        print('next line')

        # servo2.angle = 45 #whatever angle needed to move the pen down
        # pen up and move to first coor in list
        # pen down
        # draw using IK from list
        # pen up
        
def path_curve(theta,char,servo0,servo1,servo2):
    global list
    global current_x
    global current_y
    list=[]
    for i in range(0, len(theta), 5):
        if char=='s':
            degrees_travel=270
        elif theta_list[i+4] - theta_list[i+3]==0:
            degrees_travel=360
        else:
            degrees_travel=abs(theta_list[i+4] - theta_list[i+3]) * pi / 4 * 180 / pi
        number_of_travel=int(degrees_travel/font)
        increment=font*pi/180 # increase by font degrees each time
        radius=theta[i]*font
        
        center_x=theta[i+1]*font+current_x*font
        center_y=ymax*font-(ymax-current_y+4)*font+theta[i+2]*font

        print('radius',radius,'center',center_x,center_y)
        a =theta[i+3]*pi/4
        for k in range(number_of_travel):
            list.append(curve(center_x, center_y, radius, a))
            a += increment
        print(list)
        #walker is drafting this section
        #the purpose of this section is to convert the xy coords into angles
        #and write it to the servo.
        #The x and y points should be adjusted to where it makes sense for the robot
        #define a zero point for x and y
        xoffset = 0
        yoffset = 0
        for j in range(len(list)):
            angles = xy_to_ang(list[j][0] + xoffset,list[j][1] +yoffset)
            if j is 1:
                servo2.angle = 0 #the pen will move down before it moves between points
                time.sleep(1)
            servo0.angle = angles[0]
            time.sleep(.05)
            servo1.angle = angles[1]
            time.sleep(.05)
            if j is len(list):
                servo2.angle = 45 #whatever angle to go up
        list=[]
        print('next line')
    

# move to the next character
def next_character():
    global current_x
    global current_y
    if xmax-current_x*font<font*4:
        current_x=0
        current_y-=5
    else:
        current_x+=5
              #move pen to next (0,0)

# calculate circumference of a circle
def curve(x, y, r, theta):
    return x + cos(theta) * r, y + sin(theta) * r

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

linput='w'
for char in linput:
    get_data(char) # import x_list, y_list, and theta_list
    path_straight(x_list,y_list,servo0,servo1,servo2)
    path_curve(theta_list,char,servo0,servo1,servo2)
    next_character()
    x_list=[]
    y_list=[]
    theta_list=[]

pca.deinit()