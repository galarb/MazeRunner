#!/usr/bin/env python3

# Import the necessary libraries
import time
import math
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.button import Button
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *


# Create the sensors and motors objects
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
left_motor = motorA
right_motor = motorB
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)

spkr = Sound()
btn = Button()
radio = Radio()

color_sensor_in1 = ColorSensor(INPUT_1)
ultrasonic_sensor_in2 = UltrasonicSensor(INPUT_2)
gyro_sensor_in3 = GyroSensor(INPUT_3)
gps_sensor_in4 = GPSSensor(INPUT_4)
pen_in5 = Pen(INPUT_5)
ultrasonic_sensor_in6 = UltrasonicSensor(INPUT_6)#left
ultrasonic_sensor_in7 = UltrasonicSensor(INPUT_7)#right

motorC = LargeMotor(OUTPUT_C) # Magnet

# Here is where your code starts

debugval = 20 #the rateError value that triggeres the debuggin 
error = None
previousTime = time.time() * 1000
cumError = 0  # Initialize cumulative error
lastError = 0  # Initialize last error
rateError = 0
seconddev = 0
rlF_pv = 0
flag = False
lastdev = 0
error_debug = []
pid_values_debug=[]
rateError_debug =[]
definiteang =0

# Describe this function...
def pidcalc(sp, pv, Kp, Ki, Kd):

    global error, previousTime, cumError, lastError, rateError, lastdev, flag  # Specify global variables

    currentTime = int(time.time() * 1000)             
    elapsedTime = (currentTime - previousTime)/1000
    error = sp - pv
    
    if elapsedTime > 0:
        cumError += error * elapsedTime
        rateError = (error - lastError) / elapsedTime
        seconddev = (rateError - lastdev) / elapsedTime
    else:
        cumError = 0  # Reset cumulative error and rate error if elapsedTime is zero
        rateError = 0
        
    out = Kp*error + Ki*cumError + Kd*rateError
    previousTime = currentTime
    if (rateError > 0 and lastdev < 0) or (rateError < 0 and lastdev > 0):
        flag = True
    else:
        flag = False
        #print('resetting flag')

    lasterror = error
    #if abs(rateError) > debugval: 
    #debug(error, cumError, rateError, lastdev)
    
    lastrlF_pv = rlF_pv
    lastdev = rateError
    error_debug.append(error)
    rateError_debug.append(rateError)

    #else: debug(0,0,0)
    return out

# track reflected_light_intensity. for how many itterations
def track(reflected_light_sp, times):
    for i in range(times):
        rlF_pv = color_sensor_in1.reflected_light_intensity # 100 = white, 0 = black
        ang_pv = gyro_sensor_in3.angle
        speed = 50 #defaut speed
        #if abs(rateError) > debugval:
            #print('-----------------------------------------------------')
           # print ('itteration number', i, 'RL F reading:', rlF_pv)
            
        delta = pidcalc(reflected_light_sp, rlF_pv, 1.6, 0, 0.0025)
        pid_values_debug.append(delta)

        if delta > 0: #and ang_pv < 45 and ang_pv > -45:
            #print('going left')
            speedR = speed + delta
            speedL = speed - delta
        elif delta < 0 : #and ang_pv < 45 and ang_pv > -45:
            #print('going right')
            speedR = speed - abs(delta)
            speedL = speed + abs(delta)
        else:
            #print('lossing tracking')
            ang = pidcalc(0, ang_pv, 2, 0, 0) #go in 0 direction, with special Kp negative value means left correction
            #speedR = speed#(speed - ang)*0.2
            #speedL = speed#(speed + ang)*0.2
            
        #if abs(rateError) > debugval:
        #print (flag)    
        if flag:
            #print('hyper tracking is ON')
            factor = 1.2
            speedR = factor * speedR
            speedL = factor * speedL  
            
        if speedR > 100:
            speedR = 100
        if speedL > 100:
            speedL = 100
        if speedR < -100:
            speedR = -100
        if speedL < -100:
            speedL = -100
        
        #print ('speed R:', int(speedR), 'speed L:', int(speedL))
        
        tank_drive.on(speedL, speedR)

def debug(error, cumError, rateError, lastde):
    pen_in5.setColor(0.5, 0, 0.95)
    pen_in5.setWidth(2)
    #print(error)#, 'cumError', int(cumError), 'rateError', int(rateError), 'lastdev', int(lastde))
    pen_in5.down()

def goUS(dist_sp, times, Kp):
    for i in range(times):
        dist_pv = ultrasonic_sensor_in2.distance_centimeters
        if dist_pv > dist_sp:
            speedR = abs(pidcalc(dist_sp, dist_pv, Kp, 0, 0))
            speedL = speedR
            if speedR > 100:
                speedR = 100
            if speedL > 100:
                speedL = 100
            if speedR < -100:
                speedR = -100
            if speedL < -100:
                speedL = -100
            tank_drive.on(speedL, speedR)
    print ("finished drive")

def steer(ang_sp, speed, Kp):
    while ultrasonic_sensor_in2.distance_centimeters > 15:
        ang_pv = gyro_sensor_in3.angle
        delta = pidcalc(ang_sp, ang_pv, Kp, 0, 0)
        speedR = speed - delta
        speedL = speed + delta
        if speedR > 100:
            speedR = 100
        if speedL > 100:
            speedL = 100
        if speedR < -100:
            speedR = -100
        if speedL < -100:
            speedL = -100
        #print("speedR =", speedR, "speeedL=", speedL)
        #print("delta =", delta)
        tank_drive.on(speedL, speedR)

    
def turn(ang_sp, times, Kp):
    print ("turning to ", ang_sp)
    for i in range(times):
        ang_pv = gyro_sensor_in3.angle
        speedR = -pidcalc(ang_sp, ang_pv, Kp, 0, 0)
        speedL = -speedR
       
        if speedR > 100:
            speedR = 100
        elif speedR < -100:
            speedR = -100
        if speedL > 100:
            speedL = 100
        elif speedL < -100:
            speedL = -100
        tank_drive.on(speedL, speedR)
        # Update ang_pv within the loop
        ang_pv = gyro_sensor_in3.angle
    #print("finished turning", ang_pv)

def decidealignment():
    global definiteang  # Declare definiteang as global
    ang_pv = gyro_sensor_in3.angle
    if -45 <= ang_pv < 45:
        definiteang = 0
        return 0  # North
    elif 45 <= ang_pv < 135:
        definiteang = 90
        return 90  # East
    elif -135 <= ang_pv < -45:
        definiteang = -90
        return -90  # West
    elif 135 <= ang_pv < 225:
        definiteang = 180
        return 180  # South
    elif 225 <= ang_pv < 315:
        definiteang = 270
        return 270  # South-West
    elif -45 <= ang_pv < -135:
        definiteang = -180
        return -180  # South
    elif -135 <= ang_pv < -225:
        definiteang = -270
        return -270  # South-East
    elif ang_pv >= -315 or ang_pv < -225:
        definiteang = -360
        return -360  # North-West
    else:
        print("error. ang_pv =", ang_pv, "definiteang =", definiteang)

        
def orient(times):
    turn(decidealignment(), times, 1)


def decidedirection():
    front = ultrasonic_sensor_in2.distance_centimeters
    right = ultrasonic_sensor_in6.distance_centimeters
    left = ultrasonic_sensor_in7.distance_centimeters
    if front > right and front > left:
        #print("go forward")
        return "forward"
    elif right > left:
        return "right"
    else:
        return "left"

        
def decideturn(times):
    global definiteang
    direction = decidedirection() 
    print ("direction is : ", direction)
    if direction == "right":  
        definiteang += 90
    elif direction == "left":  
        definiteang -= 90
    elif direction == "stuck":
        definiteang += 180

    # Ensure definiteang stays within the range of -315 to 315 degrees
    if definiteang > 315:
        definiteang -= 360
    elif definiteang < -315:
        definiteang += 360
    print("error message: definiteang =", definiteang, "times =", times)
    turn(definiteang, times, 2)


def navigate(speed):
    debug(2,2,2,2)
    if ultrasonic_sensor_in2.distance_centimeters > 5:
        if color_sensor_in1.color != 4: 
            print("starting navigation itteration")
            orient(100)#iterations 
            decideturn(100)#iterations
            print("head to (definiteang) = ", definiteang);
            steer(definiteang, speed, 1)
        else:
            print("xxxxxxxxx FIRE xxxxxxxxxx")
            return "fire"
            pen_in5.up()
    else:
        print ("stuck - reeversing")
        motorA.on_for_rotations(100, 2, block=False) #%power, rotations, don't wait for completion
        motorB.on_for_rotations(100, 2)#wait for completion

        
        

for i in range (20):
    navigate(60)
    
    
tank_drive.off(brake=True)

