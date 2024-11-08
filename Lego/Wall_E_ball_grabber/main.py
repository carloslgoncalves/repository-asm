#!/usr/bin/env pybricks-micropython

import math

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, UltrasonicSensor, GyroSensor)
from pybricks.nxtdevices import UltrasonicSensor as UltSensor
from pybricks.parameters import Port, Direction, Button, Stop
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
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=110.5)#111
robot.settings(100, 100, 100, 100) #straight_speed, straight_acceleration, turn_rate, turn_acceleration
#robot.straight(500)
#robot.turn(360)
print(robot.distance_control.pid())
print(robot.heading_control.pid())
#Defining the bounderies for the claw_motor
max_claw_angle = 10 #degrees
max_claw_speed = 1 #degrees/s

#Diemnsions of the working area
Feald_dimensions=(1100,705)  #x,y in milimeters

ofset_wall_sensor=-60 #ditance weals to wall sensor in mm
ofset_ball_sensor=+120 #ditance weals to ball sensor in mm

max_measure= (Feald_dimensions[0]**2+Feald_dimensions[1]**2)**0.5

pos_x=230
pos_y=200
ang=0 # atual angle from 0 to 359 degrees anticlockwise

have_ball=False
ball_locat=[]
obstacles=[]





def make_mesure(device,ofset_wall_sensor=ofset_wall_sensor,ofset_ball_sensor=ofset_ball_sensor):
    mesures=[]
    
    for n in range(5):
        mesures.append(device.distance())
            
        wait(200)
    
    mean=sum(mesures)/len(mesures)

    if mean==None:
        ev3.speaker.beep()
    sum_squared_diff = 0
    for value in mesures:
        sum_squared_diff += (value - mean) ** 2
    variance = sum_squared_diff / len(mesures)
    std_dev = variance ** 0.5  # Standard deviation is the square root of variance
    
    # Step 3: Filter out values that are beyond the threshold
    filtered_measures = []
    threshold=1
    for value in mesures:
        if abs(value - mean) <= threshold * std_dev:
            filtered_measures.append(value)
    
    # Step 4: Calculate the average of the filtered measurements
    if filtered_measures:
        adjusted_average = sum(filtered_measures) / len(filtered_measures)
    else:
        adjusted_average = None  # No valid measurements after filtering

    if adjusted_average>=max_measure:
        adjusted_average=calculate_distance(pos_x, pos_y,gyro.angle())

    if device==ball_sensor:
        return adjusted_average + ofset_ball_sensor
    elif device==wall_sensor:
        return adjusted_average + ofset_wall_sensor
    
def norm_ang(ang):
    if abs(ang)>=360 or ang <0:
        n=abs(ang)//360
        if abs(ang) % 360==0:
            ang=0
        elif ang >=0:
            ang=ang-360*(n)
        elif ang<0:
            ang=ang+(360*(n+1))
    else:
        ang=ang 
    return ang

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


def inicialise():
    gyro.reset_angle(0)
    claw_motor.run_until_stalled(50,Stop.HOLD,25)
    

    


def define_target(cycle,pos_x,pos_y):
    print("in define_target")
    target_x=0
    target_y=0
    if have_ball==False and cycle==1:  
        print("if have_ball==False and  cycle==0:")
        frst_mesure = make_mesure(wall_sensor)
    
        for i in range(15):
            new_mesure=make_mesure(wall_sensor)
            print(new_mesure)
            
            if new_mesure>=max_measure:
                break
            
            elif frst_mesure>new_mesure :
                robot.turn(5)
                
                frst_mesure=new_mesure
            

            elif frst_mesure<new_mesure or i==0:
                robot.turn(-5)
                 
            
        ang=gyro.angle()
        ang=norm_ang(ang)
        gyro.reset_angle(ang)
        print("ang=",ang)
        target_x= pos_x+(math.cos(math.radians(ang))*(new_mesure))/2
        target_y= pos_y+(math.sin(math.radians(ang))*(new_mesure))/2

        return [target_x,target_y]
        print(target_x,target_y)


    elif have_ball==True:
        print("reajusting")
        #Reajust
        atual_ang=gyro.angle()
        robot.turn(0-atual_ang)
        reajust=[]
        reajust.append(make_mesure(wall_sensor))
        for k in range(3):
            robot.turn(90)
            reajust.append(make_mesure(wall_sensor))
        pos_x=(reajust[0]+reajust[2])/2
        pos_y=(reajust[1]+reajust[3])/2
        ang=gyro.angle()
        ang=norm_ang(ang)
        gyro.reset_angle(ang)

        return [0,0]

        
    elif not ball_locat:
        print("locating ball")
        #claw_motor.run_until_stalled(-50,Stop.HOLD,40)
        min_dis=0
        while not ball_locat:
            robot.turn(-5)
            wait(200)
            dis=make_mesure(ball_sensor)
            
            print("mesured distance ",dis)
            ang=gyro.angle()
            ang=norm_ang(ang)
            gyro.reset_angle(ang)
            print("calculate distace ",calculate_distance(pos_x, pos_y, ang))
            print("Angulo",ang)
            if dis<calculate_distance(pos_x, pos_y, ang)*0.9:
                target_x= pos_x+(math.cos(math.radians(ang))*dis)
                target_y= pos_y+(math.sin(math.radians(ang))*dis)
                
                ball_locat.append([target_x,target_y])
                print(ball_locat)
        
        return ball_locat[0]

        


def path_finder(x0, y0, theta0,pos_t, final_angle=None):
    print("Atual position ",x0, y0)
    print("going to ",pos_t)
    
    xt, yt = pos_t
    # Calculate the difference in coordinates
    dx = xt - x0
    dy = yt - y0
    
    # Calculate the target angle to point towards (anticlockwise from x-axis)
    target_angle = math.degrees(math.atan2(dy, dx))
    
    # Calculate the rotation needed from initial angle to target angle
    rotation_to_target = target_angle - theta0
    
    # Normalize rotation angle to the range [-180, 180] for shortest path
    rotation_to_target = (rotation_to_target + 180) % 360 - 180
    
    # Calculate the distance to the target point
    distance = math.sqrt(dx**2 + dy**2)
    
    # Rotate to face the target point (anticlockwise if positive, clockwise if negative)
    robot.turn(rotation_to_target)
    
    # Move forward to the target distance
    robot.straight(distance)
    
    
    # If a final angle is specified, rotate to that angle
    if final_angle is not None:
        final_rotation = final_angle - target_angle
        # Normalize the final rotation to [-180, 180] for shortest rotation
        final_rotation = (final_rotation + 180) % 360 - 180
        robot.turn(final_rotation)


def map_drive():
    print("Mapping and driving...")

def target_atcheaved():
    print("Target achieved!")

def system_reset():
    print("System reset.")


cycle=0

# Play a sound to tell us when we are ready to start moving
ev3.speaker.beep()

while Button.CENTER not in ev3.buttons.pressed():
    wait(100)

#path_finder(0, 0, 30, 200, 100, final_angle=90)

while True: 

    print(cycle)   
    if currentState == 0:
        
        claw_motor.run_until_stalled(50,Stop.HOLD,30)
        inicialise()
        currentState = 1
                
    elif currentState == 1:
        target=[]
        target=define_target(cycle,pos_x,pos_y) 
        
        currentState = 2
        print(target)
        

         
    elif currentState == 2:
        ang=gyro.angle()
        path_finder(pos_x,pos_y,ang,target)
        pos_x=target[0]
        pos_y=target[1]
        print("Atual position ",pos_x, pos_y)
        if not ball_locat:
            
            currentState = 1
        elif ball_locat:
            
            currentState=3

        
    cycle+=1 
# elif currentState == 3:
#     map_drive() 
            
# elif currentState == 4:
#     target_atcheaved()
            
# elif currentState == 5:
#     system_reset()
    



   






