#!/usr/bin/env pybricks-micropython

import math
import time

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, UltrasonicSensor, GyroSensor)
from pybricks.nxtdevices import UltrasonicSensor as UltSensor
from pybricks.parameters import Port, Direction, Button, Stop
from pybricks.tools import wait, StopWatch,DataLog
from pybricks.robotics import DriveBase
from pybricks.messaging import BluetoothMailboxServer, TextMailbox

# Initialize the EV3 Brick.
ev3 = EV3Brick()
wall_sensor = UltSensor(Port.S3)
ball_sensor = UltrasonicSensor(Port.S1)
gyro = GyroSensor(Port.S4, Direction.COUNTERCLOCKWISE)

# Initialize motors and the robot's drive base
right_motor = Motor(Port.B)
left_motor = Motor(Port.C)
claw_motor = Motor(Port.D)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=110.5)
robot.settings(100, 100, 100, 100)


# State Machine Variables
states = {0: "Initialize", 1: "Define Target", 2: "Path Finder", 3: "Map & Drive", 4: "Target Achieved", 5: "System Reset"}
currentState = 0
cycle = 0

# Field Constants and Robot Position
Feald_dimensions = (1120, 708)  # in mm
max_measure = math.sqrt(Feald_dimensions[0]**2 + Feald_dimensions[1]**2)
pos_x, pos_y = 210, 180
ang = 0

server = BluetoothMailboxServer()
mailbox = TextMailbox('greeting', server)

print("Waiting for client...")
server.wait_for_connection()  #  if client does not connect
print("Client connected!")

def initialize_robot():
    claw_motor.run_until_stalled(-60, Stop.HOLD, 40)
    
    ev3.speaker.beep()
    



def make_measure(device, offset=0):
    mesures = [device.distance() for _ in range(10)]
    wait(100)

    mean = sum(mesures) / len(mesures) if mesures else None
    variance = sum((x - mean) ** 2 for x in mesures) / len(mesures)
    std_dev = variance ** 0.5
    threshold = 2

    filtered_measures = [x for x in mesures if abs(x - mean) <= threshold * std_dev]
    adjusted_average = sum(filtered_measures) / len(filtered_measures) if filtered_measures else None

    return adjusted_average + offset if adjusted_average else calculate_distance(pos_x, pos_y, ang)




def inicialise():
    
    claw_motor.run_until_stalled(-60,Stop.HOLD,40)
    
    
    







def system_reset():
    print("System reset.")
    global have_ball, at_rampa,parar
    have_ball=False
    at_rampa=False
    parar=False
    ev3.speaker.beep()
    while Button.CENTER not in ev3.buttons.pressed():
        wait(100)
        global currentState
        currentState == 0

ev3.speaker.beep()
while Button.CENTER not in ev3.buttons.pressed():
        wait(100)
       

have_ball=False
currentState=0
parar=False
at_rampa=False

while True: 
   
    if currentState == 0:
        print("Estado 0")
        initialize_robot()
        currentState = 1
                
    elif currentState == 1:
        print("Estado 1")
        robot.turn(-35)
        while not have_ball:
            robot.drive(60,0)

            dist_parede=make_measure(wall_sensor)
            dist_bola=make_measure(ball_sensor)
            if dist_parede < 230:
                parar=True
                robot.stop()
                break
            if dist_bola < 180:
                robot.stop()
                claw_motor.run_until_stalled(100,Stop.HOLD,40)
                robot.straight(85)
                claw_motor.run_until_stalled(-100,Stop.HOLD,40)
                have_ball= True
                


        if parar==True:
            print("Tem uma parede")
            break
        elif have_ball==True:
            robot.turn(180)
            while not at_rampa:
                robot.drive(60,0)
                dist_parede=make_measure(wall_sensor)
                dist_bola=make_measure(ball_sensor)
                if dist_parede < 300:
                    robot.stop()
                    at_rampa=True
            claw_motor.run_until_stalled(100,Stop.HOLD,40)
            robot.straight(-150)
            claw_motor.run_until_stalled(-100,Stop.HOLD,40)
            
            mailbox.send("Bola")
            print("Mensagem enviada")
            system_reset()
            currentState=0

            

    



   






