# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math
#######################################################
# Creates Robot
#######################################################
robot = Robot()

#######################################################
# Sets the time step of the current world
#######################################################
timestep = int(robot.getBasicTimeStep())

#######################################################
# Gets Robots Distance Sensors
#######################################################
frontDistanceSensor = robot.getDevice('front distance sensor')
leftDistanceSensor = robot.getDevice('left distance sensor')
rightDistanceSensor = robot.getDevice('right distance sensor')
rearDistanceSensor = robot.getDevice('rear distance sensor')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)
rearDistanceSensor.enable(timestep)
#######################################################
# Gets Robots Camera
#######################################################
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)
#######################################################
# Gets Robots Motors
#######################################################
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)
#######################################################
# Gets Robot's the position sensors
#######################################################
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)

distBtwWhe = 2.28
distBtwWheR = distBtwWhe/2
w_dia = 1.6
w_r = w_dia/2
pi = math.pi

# Transform inches to rad - max wheel speed is 5.024 in/
def inToRad(v):
    v = v/w_r                # input=inches
    if v > 6.28:             # max rotational velocity
        print(f'ERROR: Invalid input ({v:.2f} > 6.28), setting it to 6.28')
        v = 6.28
    elif v < -6.28:
        print(f'ERROR: Invalid input ({v:.2f} < -6.28), setting it to -6.28')
        v = -6.28
    return v

# move either left, or right motor with speed v
def singleMotorMotionV(v, isLeft):
    if(isLeft):
        leftMotor.setVelocity(v)
    else:
        rightMotor.setVelocity(v)


# move in straing line at max speed = 5.024 in/s
def straightMotionD(d):
    v = 6.28
    time = d/5.024

    s_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break 
        leftMotor.setVelocity(v)
        rightMotor.setVelocity(v)

# move the left and right motors by a specified speed and period of time 
# def dualMotorMotionLRT(l, r, t):
#     l = inToRad(l, True)        
#     r = inToRad(r, False) 

#     print(f'Task [vl: {l:.2f}, vr: {r:.2f}, t: {t:.2f}] - Start-time: {robot.getTime()}')
#     singleMotorMotionV(l, True)
#     singleMotorMotionV(r, False)
#     robot.step(int(t*1000))
#     singleMotorMotionV(0, True)
#     singleMotorMotionV(0, False)
#     # return l, r

def rotationInPlace(direction):
    # quarter of a circle = 2pi/4  == pi/2
    # for this self rotation ICC R = d_mid
    s = pi*distBtwWheR / 2
    # s = v * t
    in_v = 0.4
    time = s/in_v
    # time = 15
    # in_v = s/time
    v = inToRad(in_v)

    s_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break 
        if direction == "left":
            leftMotor.setVelocity(-v)
            rightMotor.setVelocity(v)
        else:
            leftMotor.setVelocity(v)
            rightMotor.setVelocity(-v)

def circularMotion(direction):
    vr = 2
    R = 5   # radius of the circle, hardcoded for this task.
    
    omega = vr/(R+distBtwWheR)
    vl = omega*(R-distBtwWheR)
    v = (vr+vl)/2
    # 2.5 R of icc
    s = (pi/2) * R
    time = s/v

    vl_rad = inToRad(vl)
    vr_rad = inToRad(vr) 

    if direction == "right":
        vl_rad = inToRad(vr)
        vr_rad = inToRad(vl)

    s_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break 
        leftMotor.setVelocity(vl_rad)
        rightMotor.setVelocity(vr_rad)

def diffDrive(arr):
    pass


def robotPose(A):
    pass

# Cleans the IMU readings so that the are in degrees and in the
# range of [0,359]
def imu_cleaner(imu_reading):
    rad_out = imu_reading
    if rad_out < 0:
        rad_out = rad_out + 2*math.pi
    return math.degrees(rad_out)


def get_info():
    # Process sensor data here.   
    print("Left position sensor: " +str(leftposition_sensor.getValue()))
    print("Right position sensor: " +str(rightposition_sensor.getValue()))
    print("IMU: "+str(imu_cleaner(imu.getRollPitchYaw()[2])))
    print("--------------------------------------------------")


def getRobotCommands(A):
    cur_pose = "east"   # default starting position for this lab
    commands = []       # should start and change based on the IMU
    prev = []

    for i in range(len(A)):
        if i == 0:
            prev = A[i]
            continue
        # horizontal movement
        if prev[0] != A[i][0] and prev[1] == A[i][1]:
            # left to right
            if prev[0] < A[i][0]: # looking east 2
                if cur_pose == "north":
                    commands.append(0)
                elif cur_pose == "south":
                    commands.append(-1) 
                cur_pose = "east"          
            
            # right to left
            else: # looking west = 4, 
                if cur_pose == "north":
                    commands.append(-1)
                elif cur_pose == "south":
                    commands.append(0) 
                cur_pose = "west"          
            
            commands.append(abs(prev[0] - A[i][0]))               
        # vertical movement
        elif prev[0] == A[i][0] and prev[1] != A[i][1]:
            # going up
            if prev[1] < A[i][1]:   
                if cur_pose == "east":
                    commands.append(-1)
                elif cur_pose == "west":
                    commands.append(0) 
                cur_pose = "north"          
            # going down
            else:                   
                if cur_pose == "east":
                    commands.append(0)
                elif cur_pose == "west":
                    commands.append(-1) 
                cur_pose = "south"          
            
            commands.append(abs(prev[1] - A[i][1])) 
        # hardcoded moves that are not straight lines    
        # 1/4 circle moves
        elif prev[0] != A[i][0] and prev[1] != A[i][1]:
            if cur_pose == "west":
                commands.append(-3)
                cur_pose = "south"
            elif cur_pose == "east":
                commands.append(-2)
                cur_pose = "south"
            elif cur_pose == "south":
                commands.append(-2)
                cur_pose = "west"
        prev = A[i]

    return commands

# no 180 deg turns in this project
def robotWayPointNav(A):
    # commands = [20, -1, 30, -1, 25, -3, 5, -1, 15, -2, -2, 10, -3, 5]
    # ^ it should return those commands 
    commands = getRobotCommands(A)
    # -1 rotate in place left, 0 for right
    # -3 quarter of a circle left, -2 right
    # positive values X --> straight motion for X distance
    for it in commands:
        if it == -1:
            rotationInPlace("left")
        elif it == 0:
            rotationInPlace("right")
        elif it == -3:
            circularMotion("left")
        elif it == -2:
            circularMotion("right")
        else:
            straightMotionD(it)
        get_info()

# get the initial position of the robot 
def main():
    A = [[-5,-15],[15,-15],[15,15], [-10,15] ,[-15,10] ,[-15,5], [0,5] ,[5, 0] , [0,-5], [-10,-5] ,[-15,-10] ,[-15,-15]]
    robotWayPointNav(A)

    # spin :)
    while robot.step(timestep) != -1:
        leftMotor.setVelocity(3)
        rightMotor.setVelocity(-3)
        



if __name__ == "__main__":
    main()

#IGNORE THIS CODE ####################################################3
# Main loop:
# perform simulation steps until Webots is stopping the controller

# while robot.step(timestep) != -1:
#     # Read the sensors:
#     # Enter here functions to read sensor data, like:
#     # val = ds.getValue()

#     # print simulation time in sec (timesteps are in msec)
#     print("Time: " +str(robot.getTime()))
    
#     # Process sensor data here.   
#     print("Left position sensor: " +str(leftposition_sensor.getValue()))
#     print("Right position sensor: " +str(rightposition_sensor.getValue()))

#     # Enter here functions to send actuator commands, like:
#     #  motor.setPosition(10.0)
#     leftMotor.setVelocity(6.28)
#     rightMotor.setVelocity(6.28)
    
#     print("IMU: "+str(imu.getRollPitchYaw()[2]))
    
# # Enter here exit cleanup code.
