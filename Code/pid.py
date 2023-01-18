# Code to stablize drone

import time
import board
import adafruit_mpu6050
import busio

sda_pin = board.GP14
scl_pin = board.GP15
i2c = busio.I2C(scl_pin, sda_pin)

mpu = adafruit_mpu6050.MPU6050(i2c)

int motor1pin = ()
int motor2pin = ()
int motor3pin = ()
int motor4pin = ()
int defaultSpeed = 
int motor1speed = defaultSpeed
int motor2speet = defaultSpeed
int motor3speed = defaultSpeed
int motor4speed = defaultSpeed
speedChange = 0
const float Kp = () # proportional gain
const float Ki = () # integral gain
const float Kd = () # derivative gain
float ingerror
float lastError

gain = () #how much the sensor changes (dy) with respect to dx (1 notch up) 


daccx = (the flat x value)
daacy = ()
daacz = ()

error = 0

startTime = time.monotonic
loopTime = 0

while True:
    elapsedTime = startTime - loopTime

    # Get acceleration in g's
    x = mpu.acceleration[0] / 9.8
    y = mpu.acceleration[1] / 9.8
    z = mpu.acceleration[2] / 9.8

    roll = 57.2958*atan2(y,z);
    pitch = 57.2958*atan2(-x,sqrt(y*y+z*z));

# PID controller to find speedChange

    joystick1 = () #read analog joystick value
    targetPitch = map(joystick1, 0, 1023, -45, 45 )
    elapsedTime =     #insert function to get time since last loop
    error = (targetPitch - currentPitch)
    ingerror = ingerror + error * elapsedTime
    dxerror = (error - lastError)/elapsedTime
    speedChange = Kp*error + Ki*ingerror + Kd*dxerror
    error = lastError

# Changes motor speeds using calculates speedChange variable
    motor1speed = defaultSpeed + speedChange
    motor2speet = motor1speed
    motor3speed = defaultSpeed + speedChange
    motor4speed = motor3speed
    
    loopTime = time.monotonic


    


