#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, Image, ImageFile, Font
from pybricks.messaging import BluetoothMailboxServer, BluetoothMailboxClient, LogicMailbox, NumericMailbox, TextMailbox
from threading import Thread
from random import choice
from math import fmod
import sys
import os
import math
import struct

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
# MIT License: Copyright (c) 2022 Mr Jos

#####################################################################
#####################################################################
##########~~~~~PROGRAM WRITTEN BY JOZUA VAN RAVENHORST~~~~~##########
##########~~~~~~~~~~~~~~4D T-BOT GANTRY ROBOT~~~~~~~~~~~~~~##########
##########~~~~~~~~~~~~~YOUTUBE CHANNEL: MR JOS~~~~~~~~~~~~~##########
#####################################################################
#####################################################################


##########~~~~~~~~~~MACHINE OVERVIEW AND AXIS NAMES~~~~~~~~~~##########
#       |===|===============================|===|
#       |   |Left                      Right|   |
#       | _ |\                              | _ |\
#       ||_|| \                             ||_|| \  ⤡ Z-direction 
#       |   |\ \                            |   |\ \
#       |   | \ \                           |   | \ \
#       | _ |                    Touch sens | _ |
#       ||_||                          ⌄    ||_||
#       |   |                               |   |
#       |   |                               |   |
#       |   |                               |   |
#       |   |               ^               |   |
#       |   |               |Y-direction    |   |
#       |   |               ⌄               |   |
#       |   |                               |   |
#       |===|===============================|===|
#         \ \        <-> X-direction          \ \
#          \ \                                 \ \
#           \ \                                 \ \
#            \ \                                 \ \


##########~~~~~~~~~~HARDWARE CONFIGURATION~~~~~~~~~~##########
ev3 = EV3Brick()
#   Motors definition
left_motor  = Motor(Port.B)                                                                 #Left  motor for driving the chain for X and Z movement
right_motor = Motor(Port.A)                                                                 #Right motor for driving the chain for X and Z movement
arm_motor   = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)                  #Motor used to open and close the gripper hand
drive_motor = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)                  #Motor used to drive the robot for Y movement
#   Sensor definition
touch_drive = TouchSensor(Port.S4)                                                          #Touch sensor for homing the driving motor (Y-homing)


##########~~~~~~~~~~HOMING POSITION ANGLES WHEN SENSOR ACTIVATED~~~~~~~~~~##########
left_motor_homing_angle     = -40                                                           #Motor angle that will be used when resetting the homing position
right_motor_homing_angle    = 0                                                             #Motor angle that will be used when resetting the homing position
gripper_motor_homing_angle  = 350                                                           #Motor angle that will be used when resetting the homing position
x_adjust                    = 0
z_adjust                    = 30


##########~~~~~~~~~~GEARING~~~~~~~~~~##########
chain_gear      = 24                                                                        #Gear used on the left and right chain motors


##########~~~~~~~~~~MAXIMUM SPEED, MAXIMUM ACCELERATION, MAXIMUM POWER~~~~~~~~~~##########
feed_speed      = 820                                                                       #Maximal speed for the left/right and drive motors (deg/sec)
gripper_speed   = 1400                                                                      #Speed for opening/closing the gripper
left_motor.control.limits   ( 820, 3600, 100)                                               #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
right_motor.control.limits  ( 820, 3600, 100)                                               #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
arm_motor.control.limits    (1400, 3600, 100)                                               #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
drive_motor.control.limits  ( 820, 3600, 100)                                               #Motor settings [Max speed allowed (deg/sec) / Acceleration (deg/sec²) / Power (%)]
drive_motor.control.target_tolerances(800, 2)                                               #Motor settings to define what is a finished movement [Max speed still(deg/sec) / Motor angle from target (deg)]
left_motor.control.target_tolerances (5  , 1)                                               #Motor settings to define what is a finished movement [Max speed still(deg/sec) / Motor angle from target (deg)]
right_motor.control.target_tolerances(5  , 1)                                               #Motor settings to define what is a finished movement [Max speed still(deg/sec) / Motor angle from target (deg)]
#Default these last 3 motor settings are at (50, 5). By setting a lower value the positioning will take longer, but is more accurate, and needed for a machine like this, to allow higher speeds


##########~~~~~~~~~~CREATING AND STARTING A TIMER~~~~~~~~~~##########
#timer_movement = StopWatch()                                                               #Creating a timer                       [Example]
#timer_movement.time()                                                                      #Reading  a timer's current value (ms)  [Example]
#timer_movement.pause()                                                                     #Stopping a timer                       [Example]
#timer_movement.resume()                                                                    #Resuming a timer                       [Example]
#timer_movement.reset()                                                                     #Putting  a timer back at 0, if not stopped it will just keep running but start from 0 again.   [Example]


##########~~~~~~~~~~BUILDING GLOBAL VARIABLES~~~~~~~~~~##########


##########~~~~~~~~~~BRICK STARTUP SETTINGS~~~~~~~~~~##########
ev3.speaker.set_volume(volume=80, which='_all_')                                            #Set the volume for all sounds (speaking and beeps etc)
ev3.speaker.set_speech_options(language='en', voice='m7', speed=None, pitch=None)           #Select speaking language, and a voice (male/female)
small_font  = Font(size=6)                                                                  #Creating a font with  6 pixel height for text on screen    [Not used]
normal_font = Font(size=10)                                                                 #Creating a font with 10 pixel height for text on screen
big_font    = Font(size=16)                                                                 #Creating a font with 16 pixel height for text on screen    [Not used]
ev3.screen.set_font(normal_font)                                                            #Choose a preset font for writing next texts
ev3.screen.clear()                                                                          #Make the screen empty (all pixels white)
ev3.speaker.beep()                                                                          #Brick will make a beep sound 1 time
#ev3.light.on(Color.GREEN)                                                                  #Turns the green lights on the brick on                     [Not used, example]
ev3.light.off()                                                                             #Turn the lights off on the brick
#ev3.screen.draw_text(4, 2, "", text_color=Color.BLACK, background_color=Color.WHITE)       #X/Y position for writing on the screen, textstring, text color, background color   [Example]


##########~~~~~~~~~~DEFINE SUB-ROUTINES [FUNCTIONS] FOR THE OPERATION OF THE GANTRY~~~~~~~~~~##########
def stud_pos(x_studs, z_studs, y_degrees, gripper, speed_factor):                           #Main function called for driving the gantry (4D) [X-pos(studs) / Z-pos(studs) / Y-pos(degrees) / Gripper force (0=open, 6=fully closed) / Speed (1=max, any higher value will slow down the movement, like 5 = 100/5 = 20%)]
    next_pos(- x_adjust - z_adjust - 870 / chain_gear * x_studs - 870 / chain_gear * z_studs, x_adjust - z_adjust + 870 / chain_gear * x_studs - 870 / chain_gear * z_studs, y_degrees, 50 * gripper, speed_factor)


def next_pos(l_pos, r_pos, y_pos, g_pos, speed_factor):                                     #Sub-function to calculate and send the motor speeds to a given position
    speed_next = feed_speed / speed_factor                                                  #Calculate the maximal speed for this movement
    l_move = math.fabs(left_motor.angle() - l_pos)                                          #Calculate the angle that the left  motor has to change to reach the next position
    r_move = math.fabs(right_motor.angle() - r_pos)                                         #Calculate the angle that the right motor has to change to reach the next position

    if y_pos >= 0: drive_motor.run_target(speed_next, y_pos, then=Stop.COAST, wait=False)   #If the driving position is bigger than 0, start driving there at the given speed (-1 will skip this). Dont wait for arriving
    if l_move >= r_move:                                                                    #If the left motor has to move a greater angle than the right one,
        if r_move < 2: left_motor.run_target(speed_next, l_pos, then=Stop.COAST, wait=True) #If the right motor almost doesn't have to move, start only moving the left. Wait for arriving
        else:
            left_motor.run_target(speed_next, l_pos, then=Stop.COAST, wait=False)           #Else move the left at full speed. Dont wait for arriving
            right_motor.run_target(speed_next * r_move / l_move, r_pos, then=Stop.COAST, wait=True) #And the right at a calculated speed to arrive at the same time, and move linear. Wait for arriving
    else:
        if l_move < 2: right_motor.run_target(speed_next, r_pos, then=Stop.COAST, wait=True) #If the left motor almost doesn't have to move, start only moving the right. Wait for arriving
        else:
            left_motor.run_target(speed_next * l_move / r_move, l_pos, then=Stop.COAST, wait=False) #Move the left at a calculated speed to arrive at the same time, and move linear. Dont wait for arriving
            right_motor.run_target(speed_next, r_pos, then=Stop.COAST, wait=True)           #And move the right at full speed. Wait for arriving
    if y_pos >= 0:                                                                          #If the drive motor had to move,
        while drive_motor.control.done() == False: continue                                 #Start a while loop until the motor has arrived at the correct position
    arm_motor.run_target(gripper_speed, g_pos)                                              #Change the gripper position with the given gripper force


##########~~~~~~~~~~EV3 HAS INCREMENTAL ENCODERS IN IT'S MOTORS, SO IT'S NEEDED TO CALIBRATE EVERY STARTUP~~~~~~~~~~##########
arm_motor.run_target(1400, -50)                                                             #Open the gripper a little bit, if it was full closed, stress on the axle will prevent good chain homing.
right_motor.run(-75)                                                                        #Run the right motor at a set speed to match the left one
left_motor.run_until_stalled(80, then=Stop.COAST, duty_limit= 25)                           #Run the left motor at max 80°/sec and 25% force, wait for stalling
right_motor.stop()                                                                          #Stop the constantly running right motor
ev3.speaker.beep()                                                                          #Beep to acknowledge the first chain part homing is finished (left motor)
wait(1000)                                                                                  #Wait 1 second to relax the chain
left_motor.run(80)                                                                          #Run the left motor at a set speed to match the right one
right_motor.run_until_stalled(80, then=Stop.COAST, duty_limit= 20)                          #Run the right motor at max 80°/sec and 20% force, wait for stalling
left_motor.stop()                                                                           #Stop the constantly running left motor
wait(1000)                                                                                  #Wait 1 second to relax the chain
ev3.speaker.beep()                                                                          #Beep to acknowledge the chain homing is finished (left and right motor)
left_motor.reset_angle(left_motor_homing_angle)                                             #Save the current angle as the homing angle
right_motor.reset_angle(right_motor_homing_angle)                                           #Save the current angle as the homing angle

drive_motor.run(-100)                                                                       #Start driving towards the front at a constant low speed
while touch_drive.pressed() == False: continue                                              #While the touch sensor is not pressed, continue this loop
drive_motor.hold()                                                                          #Block the driving motor in position
drive_motor.reset_angle(0)                                                                  #Save the current angle as 0
left_motor.run_target(feed_speed, 0, then=Stop.HOLD, wait=False)                            #Move the left chain motor to position 0°
right_motor.run_target(feed_speed, 0, then=Stop.HOLD, wait=True)                            #Move the left chain motor to position 0°                                                                         
ev3.speaker.beep()                                                                          #Beep to acknowledge the driving homing is finished (driving motor)

stud_pos(10, 3, 0, 0, 1)                                                                    #Move to a safe position for homing the gripper
arm_motor.run_until_stalled(300, then=Stop.COAST, duty_limit= 40)                           #Run the gripper motor at max 300°/sec and 40% force, wait for stalling
arm_motor.reset_angle(gripper_motor_homing_angle)                                           #Reset the gripper motor angle to the preset value
arm_motor.run_target(gripper_speed, 0)                                                      #Move the gripper to the open position



# ~~~~~~~~~~ You can make the 4D Gantry go to any position by using this standard for every point after this part of code~~~~~~~~~~
#stud_pos(X-position (1-20), Z-position (1-18), Y-position (1-25), Gripper closing (0-6, not higher than 6!! motor will stall), Speed(1=fast / 10=slow))
#Example: stud_pos( 0.6, 5.5, 1210, 0,  1)      #This will go to the left, almost full up, drive near the backside, with an open gripper at full speed



##########~~~~~~~~~~MAIN PROGRAM FOR MOVING TO ANY POINT~~~~~~~~~~##########
while True:                                                                                 #Copy this one in front of the main forever loop for continuous random positioning
    random_one   = choice([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20])             #Choosing a random X-position (studs)
    random_two   = choice([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18])                   #Choosing a random Z-position (studs)
    random_three = 50 * choice([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25]) #Choosing a random Y-position (per 50degrees)
    random_four  = choice([0,6])                                                            #Choosing a random gripper clamping force (0=open, 6= fully closed)
    
    stud_pos(random_one, random_two, random_three, random_four, 1)                          #Move to the random position, then restart the loop



##########~~~~~~~~~~EXTRA'S NOT USED IN NORMAL PROGRAM, FOR SAVING PURPOSES~~~~~~~~~~##########                                                                        
###Copy one of these "while True" loops in front of the current main program "while True" loop, to test movement without taking up pins, just preprogrammed positions
while True:                                                                                 #Copy this one in front of the main forever loop for simulating a pickup/dropoff
    stud_pos( 0.6, 5.5, 1210, 0,  1)                                                        #Move fast to the 'wait' position
    stud_pos( 0.6, 7.5, 1100, 5, 10)                                                        #Slowly lower and close the gripper
    stud_pos( 0.6, 5.5, 1100, 5, 10)                                                        #Quickly pull the head up to a safe position
    stud_pos(  19, 5.5,  200, 5,  1)                                                        #Quickly move above the dropoff zone
    stud_pos(  19,  23,  200, 0,  1)                                                        #Lower the head and open the gripper
    stud_pos(  19, 5.5, 1210, 0,  1)                                                        #Quickly pull the head up again



ev3.speaker.say("Thank you for watching, subscribe for more Technic Machines")              #The EV3 speaker will read out this textstring