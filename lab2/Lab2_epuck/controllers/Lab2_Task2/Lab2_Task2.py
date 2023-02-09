# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np

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

# Main loop:
# perform simulation steps until Webots is stopping the controller


























###################################################################################
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
    
# Enter here exit cleanup code.
