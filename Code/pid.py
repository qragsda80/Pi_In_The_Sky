# Code to stablize drone

import time
import math
import board
import adafruit_mpu6050
import busio
import pwmio

sda_pin = board.GP12
scl_pin = board.GP13
i2c = busio.I2C(scl_pin, sda_pin)

mpu = adafruit_mpu6050.MPU6050(i2c)

pwm_motor = pwmio.PWMOut(board.GP26, duty_cycle=2 ** 15, frequency=50)
pwm_motor2 = pwmio.PWMOut(board.GP21, duty_cycle=2 ** 15, frequency=50)
pwm_motor3 = pwmio.PWMOut(board.GP18, duty_cycle=2 ** 15, frequency=50)
pwm_motor4 = pwmio.PWMOut(board.GP28, duty_cycle=2 ** 15, frequency=50)



defaultSpeed = 65000
pwm_motor.duty_cycle = defaultSpeed
pwm_motor2.duty_cycle = defaultSpeed
pwm_motor3.duty_cycle = defaultSpeed
pwm_motor4.duty_cycle = defaultSpeed
speedChange = 0
Kp = 5 # proportional gain 
Ki = 0.1 # integral gain
Kd = 5 # derivative gain
ingerror = 0
lastError = 0

gain = 330 #how much the sensor changes (dy) with respect to dx (1 notch up) 


#daccx = (the flat x value)
#daacy = ()
#daacz = ()

error = 0
elapsedTime = 1
loopTime = 0

while True:
    startTime = time.monotonic()

    

    # Get acceleration in g's
    x = mpu.acceleration[0] / 9.8
    y = mpu.acceleration[1] / 9.8
    z = mpu.acceleration[2] / 9.8

    roll = 57.2958*math.atan2(y,z)
    currentPitch = 57.2958*math.atan2(-x,math.sqrt(y*y+z*z))

# PID controller to find speedChange   

    targetPitch = -2.35
    error = (targetPitch - currentPitch)
    ingerror = ingerror + error * elapsedTime
    dxerror = (error - lastError)/elapsedTime
    speedChange = Kp*error + Ki*ingerror + Kd*dxerror
    lastError = error
# Changes motor speeds using calculates speedChange variable
    pwm_motor.duty_cycle = int(round(defaultSpeed + speedChange))
    pwm_motor2.duty_cycle = int(round(defaultSpeed + speedChange))
    pwm_motor3.duty_cycle = int(round(defaultSpeed - speedChange))
    pwm_motor4.duty_cycle = int(round(defaultSpeed - speedChange))
    
    loopTime = time.monotonic()
    elapsedTime = startTime - loopTime
    #print(f"Speedchange: {speedChange} & MotorSpeed: {pwm_motor.duty_cycle}")
    print(f"currentpitch: {currentPitch} & speedchange: {speedChange}")
    
