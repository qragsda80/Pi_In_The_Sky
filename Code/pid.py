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



defaultSpeed = 45000
pwm_motor.duty_cycle = defaultSpeed
pwm_motor2.duty_cycle = defaultSpeed
pwm_motor3.duty_cycle = defaultSpeed
pwm_motor4.duty_cycle = defaultSpeed
speedChange = 0
Kp = 0.01 # proportional gain
Ki = 0 # integral gain
Kd = 0 # derivative gain
ingerror = 0
lastError = 0



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

    targetPitch = 0
    error = (targetPitch - currentPitch)
    ingerror = ingerror + (error * elapsedTime)
    dxerror = (error - lastError)/elapsedTime
    speedChange = (Kp*error + Ki*ingerror + Kd*dxerror)*defaultSpeed
    lastError = error

# checks if motor speeds are within limits
    motor1speed = int(round(defaultSpeed + speedChange))
    motor2speed = int(round(defaultSpeed + speedChange))
    motor3speed = int(round(defaultSpeed - speedChange))
    motor4speed = int(round(defaultSpeed - speedChange))

    if  motor1speed < 0:
        motor1speed = 0
    if  motor2speed < 0:
        motor2speed = 0
    if  motor3speed < 0:
        motor3speed = 0
    if  motor4speed < 0:
        motor4speed = 0
    if  motor1speed > 65535:
        motor1speed = 65535
    if  motor2speed > 65535:
        motor2speed = 65535
    if  motor3speed > 65535:
        motor3speed = 65535
    if  motor4speed > 65535:
        motor4speed = 65535
    if speedChange > 20000:
        speedChange = 20000

    pwm_motor.duty_cycle = motor1speed
    pwm_motor2.duty_cycle = motor2speed
    pwm_motor3.duty_cycle = motor3speed
    pwm_motor4.duty_cycle = motor4speed
    
    loopTime = time.monotonic()
    elapsedTime = startTime - loopTime
    print(f"Speedchange: {speedChange} & MotorSpeed: {pwm_motor.duty_cycle}")
    print(currentPitch)
    time.sleep(.23)
