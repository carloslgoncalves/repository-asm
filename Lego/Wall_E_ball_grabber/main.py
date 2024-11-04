#!/usr/bin/env pybricks-micropython

import math

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, UltrasonicSensor, GyroSensor)
from pybricks.nxtdevices import UltrasonicSensor as UltSensor
from pybricks.parameters import Port, Direction, Button
from pybricks.tools import wait, StopWatch,DataLog
from pybricks.robotics import DriveBase

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the Ultrasonic Sensor. It is used to detect
# obstacles as the robot drives around.
wall_sensor = UltSensor(Port.S3)
ball_sensor = UltrasonicSensor(Port.S1)



gyro=GyroSensor(Port.S4, Direction.COUNTERCLOCKWISE)

# Initialize two motors with default settings on Port B and Port C.
# These will be the left and right motors of the drive base.
right_motor = Motor(Port.B)
left_motor = Motor(Port.C)
claw_motor = Motor(Port.D)

states={0:"Inicialise" , 1:"Define target" , 2:"Path finder" , 3:"Map & drive" , 4:"Target atcheaved" , 5:"System reset"}
currentState = 0

# The wheel_diameter and axle_track values are used to make the motors
# move at the correct speed when you give a motor command.
# The axle track is the distance between the points where the wheels
# touch the ground.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=111)#149
robot.settings(100, 100, 100, 100) #straight_speed, straight_acceleration, turn_rate, turn_acceleration

#Defining the bounderies for the claw_motor
max_claw_angle = 10 #degrees
max_claw_speed = 1 #degrees/s

#Diemnsions of the working area
Feald_dimensions=(100,100)  #x,y in milimeters


pos_x=0
pos_y=0
ang=0 # atual angle from 0 to 359 degrees anticlockwise

# Play a sound to tell us when we are ready to start moving
ev3.speaker.beep()

while Button.CENTER not in ev3.buttons.pressed():
    wait(100)


def inicialise():
    gyro.reset_angle(0)
    frst_mesure = wall_sensor.distance()
    for i in range(4):
        if frst_mesure<wall_sensor.distance():
            robot.turn(3)
        elif frst_mesure>wall_sensor.distance():
            robot.turn(-3)
        frst_mesure=wall_sensor.distance()


while True:
    if currentState == 0:
        robot.turn(90)
        robot.stop()
        #inicialise()
            
    elif currentState == 1:
        define_target() 
            
    elif currentState == 2:
        path_finder()
        
    elif currentState == 3:
        map_drive() 
            
    elif currentState == 4:
        target_atcheaved()
            
    elif currentState == 5:
        system_reset()
    



def inicialise():
    gyro.reset_angle(0)
    frst_mesure = wall_sensor.distance()
    robot.turn(3)
    for i in range(4):
        if frst_mesure<wall_sensor.distance():
            robot.turn(3)
        elif frst_mesure>wall_sensor.distance():
            robot.turn(-3)
        frst_mesure=wall_sensor.distance()
    


def define_target():
    print("Defining target...")

def path_finder():
    print("Finding path...")

def map_drive():
    print("Mapping and driving...")

def target_atcheaved():
    print("Target achieved!")

def system_reset():
    print("System reset.")




def calculate_distance(X_p, Y_p, theta, X_m=Feald_dimensions[0], Y_m=Feald_dimensions[1]):
    # Convert theta to radians
    theta_rad = math.radians(theta)
    cos_theta = round(math.cos(theta_rad),5)
    sin_theta = round(math.sin(theta_rad),5)
    
    # Initialize an empty list to store valid distances
    distances = []
    
    # Check intersection with the left edge (x = 0)
    if cos_theta != 0:  # Avoid division by zero
        t_left = -X_p / cos_theta
        y_left = Y_p + t_left * sin_theta
        if t_left > 0 and 0 <= y_left <= Y_m:
            distances.append(t_left)
    
    # Check intersection with the right edge (x = X_m)
    if cos_theta != 0:  # Avoid division by zero
        t_right = (X_m - X_p) / cos_theta
        y_right = Y_p + t_right * sin_theta
        if t_right > 0 and 0 <= y_right <= Y_m:
            distances.append(t_right)
    
    # Check intersection with the bottom edge (y = 0)
    if sin_theta != 0:  # Avoid division by zero
        t_bottom = -Y_p / sin_theta
        x_bottom = X_p + t_bottom * cos_theta
        if t_bottom > 0 and 0 <= x_bottom <= X_m:
            distances.append(t_bottom)
            
    
    # Check intersection with the top edge (y = Y_m)
    if sin_theta != 0:  # Avoid division by zero
        t_top = (Y_m - Y_p) / sin_theta
        x_top = X_p + t_top * cos_theta
        if t_top > 0 and 0 <= x_top <= X_m:
            distances.append(t_top)
    
    # Return the minimum positive distance (closest intersection)
    if distances:
        return round(min(distances),3)
    else:
        return None  # No valid intersection found