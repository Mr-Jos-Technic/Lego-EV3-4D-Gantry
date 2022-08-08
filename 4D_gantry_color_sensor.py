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
#       | _ |\     ||                       | _ |\
#       ||_|| \    ||                       ||_|| \  ⤡ Z-direction 
#       |   |\ \   ||# Color sensor         |   |\ \
#       |   | \ \  ||- wait_pos_y           |   | \ \
#       | _ |      ||\           Touch sens | _ |
#       ||_||      || \                ⌄    ||_||
#       |   |      ||\                      |   |
#       |   |      || \                     |   |
#       |   |      ||- max_pos_y            |   |
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
pin_color   = ColorSensor(Port.S2)                                                          #Color sensor used for scanning the color and length of a pin (advanced setup)
touch_drive = TouchSensor(Port.S4)                                                          #Touch sensor for homing the driving motor (Y-homing)


##########~~~~~~~~~~HOMING POSITION ANGLES WHEN SENSOR ACTIVATED~~~~~~~~~~##########
left_motor_homing_angle     = -40                                                           #Motor angle that will be used when resetting the homing position
right_motor_homing_angle    = 0                                                             #Motor angle that will be used when resetting the homing position
gripper_motor_homing_angle  = 350                                                           #Motor angle that will be used when resetting the homing position
x_adjust                    = 0
z_adjust                    = 30


##########~~~~~~~~~~GEARING~~~~~~~~~~##########
chain_gear      = 24                                                                        #Gear used on the left and right chain motors
high_speed      = 1                                                                         #Dividing factor for the max speed
low_speed       = 10                                                                        #Dividing factor for the low speed
clamp_open      = 0                                                                         #Position for a opened headpiece
clamp_closed    = 5                                                                         #Force/distance setting for closing the headpiece
y_follow_speed  = -42                                                                       #(°/sec) Speed of the ganty driving motor when following the belt at the same speed [No belt in basic version]
no_drive_y      = -1                                                                        #No given speed for driving (if already another drive is given it continues at the old speed)
deg_per_ms      = 0.042                                                                     #(°/ms) Speed of the conveyor belt (and gantry when following the belt = 42/1000)   [No belt in basic version]
deg_per_mm      = 2.36559                                                                   #(°/mm) Motor angle to run for 1mm displacement in Y-direction
dist_wait_pos   = 20                                                                        #(mm) Center color sensor, to center pin at waiting position distance               [No belt in basic version]
time_wait_pos   = dist_wait_pos * deg_per_mm / deg_per_ms                                   #(20 * 2.36559 / 0.042 = 1126ms)


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
timer_pin = StopWatch()                                                                     #Creating a timer


##########~~~~~~~~~~BUILDING GLOBAL VARIABLES~~~~~~~~~~##########
pin_list        = []                                                                        #List that will hold all the data from pins scanned by the color sensor
pin_handled     = 0                                                                         #Current amount of pins being handled by the gantry (Needs to stay at 0!)

wait_pos_x      = 0.6                                                                       #Default position when there is no task. (studs) to the right, after homing left
wait_pos_y      = 1210                                                                      #Default position when there is no task. (°) after homing at the front
wait_pos_z      = 5.5                                                                       #Default position when there is no task. (studs) down after top homing, safe height waiting for a pin pickup

pick_pos_z      = 7.3                                                                       #Default height for a pickup.  (studs) down after top homing, for picking a pin from the conveyor belt   [No belt in basic version]
drop_pos_z      = 23                                                                        #Default height for a dropoff. (studs) down after top homing, dropoff height
max_pos_y       = 550                                                                       #Maximal pickup Y-position to pickup a pin from the belt (°) [Pickup range = wait_pos_y until max_pos_y] [No belt in basic version]
pre_lowering    = True                                                                      #For speeding up the gantry, the head can start lowering when X and Z are in position (and Y still moves)
drop_low        = True                                                                      #For real high speeds, the head will not go down for a dropoff, if this variable is set to False

##########~~~~~~~~~~PIN DATA, THIS LIST HOLDS FOR EVERY ITEM; NAME / HOWMANY HAVE BEEN SEEN / DROPOFF LOCATION(studs/studs/Y-pos in degrees) / LENGTH(ms) + RGB VALUES BETWEEN~~~~~~~~~~##########
                #Name           #Amount counted #Dropoff position             #Color sensor  < Length <         < Red <    < Green <   < Blue <
pins_scanned = {"ReScan"     : {"counter" : 0 , "location" : [10,  6,    0] , "dataset" : [    0,    0,         0,    0,    0,    0,    0,    0]} , \
                "Reject"     : {"counter" : 0 , "location" : [10,  6,    0] , "dataset" : [    0,    0,         0,    0,    0,    0,    0,    0]} , \
                "Axle 2L Red": {"counter" : 0 , "location" : [10,  6,  920] , "dataset" : [  900, 1100,      14.0, 22.0,  0.0,  2.2,  0.0,  1.0]} , \
                "Axle 3L RB" : {"counter" : 0 , "location" : [10,  6,  770] , "dataset" : [ 1150, 1350,       3.9,  8.0,  1.5,  4.5,  0.5,  6.0]} , \
                "Axle 3L Tan": {"counter" : 0 , "location" : [18,  6,  770] , "dataset" : [ 1450, 1600,       8.4, 11.0,  6.8,  8.5,  6.4,  9.0]} , \
                "Conn 3L Whi": {"counter" : 0 , "location" : [18,  6,  920] , "dataset" : [ 1600, 1800,      15.0, 20.0, 15.0, 21.0, 24.0, 35.0]} , \
                "Black 2L"   : {"counter" : 0 , "location" : [10,  6,  450] , "dataset" : [  150,  400,       0.5,  2.0,  0.5,  1.5,  0.0,  1.0]} , \
                "DBG 3L"     : {"counter" : 0 , "location" : [18,  6,  590] , "dataset" : [ 1350, 1500,       3.5,  4.7,  4.2,  5.3,  6.2,  8.3]} , \
                "DBG 1.5L"   : {"counter" : 0 , "location" : [10,  6,  590] , "dataset" : [  600,  750,       3.5,  5.1,  2.9,  5.1,  2.9,  6.6]} , \
                "Blue 3L"    : {"counter" : 0 , "location" : [18,  6,  300] , "dataset" : [ 1350, 1500,       0.7,  3.0,  4.2,  7.5, 22.0, 40.0]} , \
                "Blue 2L"    : {"counter" : 0 , "location" : [10,  6,  300] , "dataset" : [  900, 1100,       0.7,  3.0,  4.2,  7.5, 22.0, 40.0]} , \
                "Blue 1.25L" : {"counter" : 0 , "location" : [ 2,  6,  300] , "dataset" : [  600,  750,       0.7,  3.0,  4.2,  7.5, 18.0, 40.0]} , \
                "Tan 3L"     : {"counter" : 0 , "location" : [18,  6,   20] , "dataset" : [ 1400, 1500,      15.0, 22.0, 13.0, 19.0, 15.0, 24.0]} , \
                "Tan 2L"     : {"counter" : 0 , "location" : [10,  6,   20] , "dataset" : [  950, 1150,      15.0, 22.0, 12.0, 19.0, 12.0, 24.0]} , \
                "Tan 1.5L"   : {"counter" : 0 , "location" : [ 2,  6,   20] , "dataset" : [  750,  850,      15.0, 22.0, 12.0, 19.0, 13.0, 24.0]} , \
                "Red 3L"     : {"counter" : 0 , "location" : [18,  6,  450] , "dataset" : [ 1350, 1500,      13.0, 22.0,  1.5,  4.0,  3.0,  5.0]} , \
                "LBG 1.25L"  : {"counter" : 0 , "location" : [ 2,  6,  160] , "dataset" : [  600,  750,       7.0, 13.2,  8.5, 14.2, 13.4, 27.0]} , \
                "LBG 2L"     : {"counter" : 0 , "location" : [10,  6,  160] , "dataset" : [  950, 1100,       7.0, 13.2,  8.5, 12.6, 12.4, 24.0]} , \
                "LBG 3L"     : {"counter" : 0 , "location" : [18,  6,  160] , "dataset" : [ 1350, 1500,       7.0, 13.2,  8.5, 12.6, 13.4, 24.0]} }
#Black is a difficult color to scan with a 1 sensor setup


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


##########~~~~~~~~~~CREATING A FILE THAT IS SAVED OFFLINE~~~~~~~~~~##########   [Not used]
#Not used right now, it can be used to store coordinates of positions to go to, and call them back when restarting the program
#os.remove("saveddata.txt")                                                                 #Removing the wms file for whatever reason you might need to delete it  [KEEP # for normal operation!]
#create_file = open("saveddata.txt", "a")                                                   #Create a file if it does not exist and open it, if it does exist just open it
#create_file.write("")                                                                      #Write "Nothing" to the file to have atleast 1 line in the file
#create_file.close()                                                                        #Close the file again, to be able to call it later again


##########~~~~~~~~~~DEFINE SUB-ROUTINES [FUNCTIONS] FOR THE OPERATION OF THE GANTRY~~~~~~~~~~##########
def check_result_scans():                                                                   #This function will, when called, check the last entered data for a pin, and compare it with the data list, to determine the result
    for x in pins_scanned:                                                                  #It will automatically loop for every pin defined in the dictionary, so you can add your own pins at line 142
        if  pins_scanned[x]["dataset"][ 0] <  pin_list[-1][0] <  pins_scanned[x]["dataset"][ 1] and pins_scanned[x]["dataset"][ 2] <= pin_list[-1][1] <= pins_scanned[x]["dataset"][ 3] and \
            pins_scanned[x]["dataset"][ 4] <= pin_list[-1][2] <= pins_scanned[x]["dataset"][ 5] and pins_scanned[x]["dataset"][ 6] <= pin_list[-1][3] <= pins_scanned[x]["dataset"][ 7]: return x
    return "ReScan"                                                                         #If it does not match with any pin, it will send back that a rescan is needed


def stud_pos(x_studs, z_studs, y_degrees, gripper, speed_factor, pre_drop):                 #Main function called for driving the gantry (4D) [X-pos(studs) / Z-pos(studs) / Y-pos(degrees) / Gripper force (0=open, 6=fully closed) / Speed (1=max, any higher value will slow down the movement, like 5 = 100/5 = 20%) / Predrop (False = go to given XZ pos, True = go to belt-pickup position if XZ is finished and Y still isn't)]
    next_pos(- x_adjust - z_adjust - 870 / chain_gear * x_studs - 870 / chain_gear * z_studs, x_adjust - z_adjust + 870 / chain_gear * x_studs - 870 / chain_gear * z_studs, y_degrees, 50 * gripper, speed_factor, pre_drop)


def next_pos(l_pos, r_pos, y_pos, g_pos, speed_factor, pre_drop):                           #Sub-function to calculate and send the motor speeds to a given position
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
        while drive_motor.control.done() == False:                                          #Start a while loop until the motor has arrived at the correct position
            if pre_drop == True:                                                            #If the predrop setting is set to True,
                pre_drop = False                                                            #Cancel the variable so it performs this only once,
                stud_pos(wait_pos_x, pick_pos_z, no_drive_y, clamp_open, high_speed, False) #Move only XZ and open the gripper if needed, all at high speed
            else: continue                                                                  #If predrop was set to False, just wait for the Y-movement to be finished
    arm_motor.run_target(gripper_speed, g_pos)                                              #Change the gripper position with the given gripper force


def auto_scan():                                                                            #Function that constantly scans the color in front of the color sensor
    global pin_list                                                                         #Using this global variable in this local function (if not defined to be global, it will make a local variable)

    pin_length      = 0                                                                     #Defining a local variable
    pin_rgb_list    = []
    pin_values      = [0, 0, 0]

    while True:                                                                             #Start a forever loop
        clr_now = pin_color.rgb()                                                           #Measure the RGB values once and save them in a new local variable
        if clr_now != (0, 0, 0):                                                            #Nothing in front of the pin is (0, 0, 0), if this is the case, do nothing else, and go take a new sample
            pin_start = timer_pin.time()                                                    #If there was another color, save the current time in a new local variable
            while clr_now != (0, 0, 0):                                                     #Whilst the color is not (0, 0, 0) perform this loop
                pin_rgb_list.append(clr_now)                                                #Add the RGB values to the local variable list
                clr_now = pin_color.rgb()                                                   #Take a new RGB measurement
            pin_end = timer_pin.time()                                                      #As the color is now (0, 0, 0) again, the pin is fully passed by, save the time in a new local variable
            pin_length = pin_end - pin_start                                                #Calculate the pin length (ms) that it has been in front of the scanner
            if len(pin_rgb_list) <= 60:                                                     #Check the amount of RGB scans that have been, if not enough,
                print("To short detection" , pin_length, "ms long", len(pin_rgb_list), "measuring points")  #Print this feedback line when debugging
                pin_rgb_list = []                                                           #Clear the local list
                continue                                                                    #Restart the loop
            data_points = len(pin_rgb_list) - 60                                            #Save the amount of valuable measured data points (30 first and 30last are garbage)
            for x in range(data_points):                                                    #For every valuable measurement,
                for y in range(3):                                                          #For each, Red / Green and Blue values,
                    pin_values[y] += pin_rgb_list[30 + x][y]                                #Add the value to the local list that holds the total value for each color
            for x in range(3): pin_values[x] /= data_points                                 #Divide the total color number by the amount of valuable data points, the result is an average color measured over many scans
            pin_list.append([pin_length, round(pin_values[0], 2), round(pin_values[1], 2), round(pin_values[2], 2), pin_start + 300])   #Add all the gathered data to the pin data list [Length / R / G / B / Timestamp for where the pin is (needed to calculate future position on a moving belt)]
            pin_result = check_result_scans()                                               #Call the function that determines the pin sort, and save the answer in a new local variable
            pin_list[-1].extend([pin_result])                                               #Add to the last (current one) pin DATA, the name of the pin
            pins_scanned[pin_result]["counter"] += 1                                        #For the scanned pin, add 1 to its counter
            if pin_result == "ReScan": print(pin_list[-1])                                  #If the result was undetermined (ReScan), print all pin DATA for debugging. With these values you can adjust the DATA list, or add new pins.
            else: print(pin_result)                                                         #If there was a clear answer, print the name for debugging
            pin_rgb_list = []                                                               #Clear the local list
            pin_values = [0, 0, 0]                                                          #Clear the local list
            ev3.speaker.beep()                                                              #Beep sound once to show there has been a pin scanned
            wait(200)                                                                       #Wait 200ms before starting to scan again, to avoid false restarts TODO is this still needed with the 60+ points needed?


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

stud_pos(10, 3, 0, 0, 1, False)                                                             #Move to a safe position for homing the gripper
arm_motor.run_until_stalled(300, then=Stop.COAST, duty_limit= 40)                           #Run the gripper motor at max 300°/sec and 40% force, wait for stalling
arm_motor.reset_angle(gripper_motor_homing_angle)                                           #Reset the gripper motor angle to the preset value
arm_motor.run_target(gripper_speed, 0)                                                      #Move the gripper to the open position


##########~~~~~~~~~~CREATING MULTITHREADS~~~~~~~~~~##########
sub_auto_scan = Thread(target=auto_scan)                                                    #This creates a thread, made from a previously defined function. No arguments can be given
sub_auto_scan.start()                                                                       #This starts the loop thread that contantly scans for pins. Non-blocking


while True:                                                                                 #Main program, starting the forever loop
    if len(pin_list) <= pin_handled: stud_pos( wait_pos_x, wait_pos_z, wait_pos_y , clamp_open, high_speed, pre_lowering) #If there is no new pin being started scanning, go to default wait position
    else:
        while len(pin_list[pin_handled]) < 6: continue                                      #Waiting for a full argument list for the next pin
        if pin_list[pin_handled][5] == "ReScan":                                            #If the result is undefined,
            pin_handled += 1                                                                #Add 1 to handled pins, just let the pin run on the belt to the scrap bin.
            continue                                                                        #Restart the loop
        else:
            pickup_pos_y = wait_pos_y - ((timer_pin.time() - (pin_list[pin_handled][4] + time_wait_pos)) * deg_per_ms)  #Calculate the current position of the pin on a moving conveyor belt
            current_pos_y = drive_motor.angle()                                             #Save the current driving motor angle in a new local variable
            fly_distance_y = math.fabs(current_pos_y - pickup_pos_y)                        #Calculate the distance needed to move, to the moving pin
            projected_pickup_pos_y = pickup_pos_y - (fly_distance_y * 1.7 * deg_per_ms)     #Calculate the time needed to get to that position, and adjust with the distance the pin will move in this time
            #print(pickup_pos_y, current_pos_y, projected_pickup_pos_y)                     #Print this feedback line when debugging, pin positions on a moving belt + actual position + adjustment angle
                                     
            if projected_pickup_pos_y < max_pos_y:                                          #Prevent running to far, when the pin already fell of the belt
                pins_scanned[pin_list[pin_handled][5]]["counter"] -= 1                      #Remove a count from the pin counter
                pins_scanned["Reject"]["counter"] += 1                                      #Add one to the count of Reject
                pin_handled += 1                                                            #Add 1 to handled pins, as the pin ran on the belt to the scrap bin.
                continue                                                                    #Restart the loop
            else: stud_pos(wait_pos_x, wait_pos_z, projected_pickup_pos_y, clamp_open, high_speed, pre_lowering)    #Move to the projected position the pin will be at
    if len(pin_list) <= pin_handled:                                                        #If the next pin hasn't started scanning yet
        while len(pin_list) <= pin_handled: continue                                        #Waiting for the next pin started to scan
    while len(pin_list[pin_handled]) < 6: continue                                          #Waiting for a full argument list for the next pin
    if pin_list[pin_handled][5] == "ReScan":                                                #If the result is undetermined
        pin_handled += 1                                                                    #Add 1 to handled pins, just let the pin run on the belt to the scrap bin.
        continue                                                                            #Restart the loop
    else:
        while pin_list[pin_handled][4] + time_wait_pos > timer_pin.time(): continue         #Wait for the minimal time after the pin was scanned (for not touching the sensor)
        drive_motor.run_target(feed_speed, wait_pos_y - ((timer_pin.time() - (pin_list[pin_handled][4] + time_wait_pos)) * deg_per_ms), then=Stop.HOLD, wait=True)  #Drive a last time to accurate pin position
        drive_motor.run(y_follow_speed)                                                     #Start following the conveyor belt at the same speed in Y-direction
        stud_pos(wait_pos_x, pick_pos_z, no_drive_y, clamp_closed, low_speed , False)       #Slowly lower the head and then close the gripper arm
        stud_pos(wait_pos_x, wait_pos_z, no_drive_y, clamp_closed, high_speed, False)       #Quickly pull the head up with the pin in the gripper

        dropoff_pos = pins_scanned[pin_list[pin_handled][5]]["location"]                    #Save the dropoff loction in a new local variable
        if drop_low == True:                                                                #If the pin needs to be put down gently,
            stud_pos(dropoff_pos[0], dropoff_pos[1], dropoff_pos[2], clamp_closed, high_speed, False)   #Move to the dropoff location and keep the gripper closed
            stud_pos(dropoff_pos[0], drop_pos_z    , no_drive_y    , clamp_open  , high_speed, False)   #Lower the head and open the gripper
            stud_pos(dropoff_pos[0], dropoff_pos[1], no_drive_y    , clamp_open  , high_speed, False)   #Go back up at the dropoff location
        else:                                                                               #Just drop off at height
            stud_pos(dropoff_pos[0], dropoff_pos[1], dropoff_pos[2], clamp_open  , high_speed, False)   #Move to the dropoff location and open the gripper
        pin_handled += 1                                                                    #Add 1 to handled pins


##########~~~~~~~~~~EXTRA'S NOT USED IN NORMAL PROGRAM, FOR SAVING PURPOSES~~~~~~~~~~##########                                                                        
###Copy one of these "while True" loops in front of the current main program "while True" loop, to test movement without taking up pins, just preprogrammed positions
while True:                                                                                 #Copy this one in front of the main forever loop for continuous random positioning
    random_one   = choice([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20])             #Choosing a random X-position (studs)
    random_two   = choice([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18])                   #Choosing a random Z-position (studs)
    random_three = 50 * choice([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25]) #Choosing a random Y-position (per 50degrees)
    random_four  = choice([0,6])                                                            #Choosing a random gripper clamping force (0=open, 6= fully closed)
    
    stud_pos(random_one, random_two, random_three, random_four, 1, False)                   #Move to the random position, then restart the loop


while True:                                                                                 #Copy this one in front of the main forever loop for simulating a pickup/dropoff
    stud_pos( 0.6, 5.5, 1210, 0,  1, False)                                                 #Move fast to the 'wait' position
    stud_pos( 0.6, 7.5, 1100, 5, 10, False)                                                 #Slowly lower and close the gripper
    stud_pos( 0.6, 5.5, 1100, 5, 10, False)                                                 #Quickly pull the head up to a safe position
    stud_pos(  19, 5.5,  200, 5,  1, False)                                                 #Quickly move above the dropoff zone
    stud_pos(  19,  23,  200, 0,  1, False)                                                 #Lower the head and open the gripper
    stud_pos(  19, 5.5, 1210, 0,  1, False)                                                 #Quickly pull the head up again


ev3.speaker.say("Thank you for watching, subscribe for more Technic Machines")              #The EV3 speaker will read out this textstring