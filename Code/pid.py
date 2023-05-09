import time
import math
import board
import adafruit_mpu6050
import busio
import pwmio
import simpleio

sda_pin = board.GP10
scl_pin = board.GP11
i2c = busio.I2C(scl_pin, sda_pin)

mpu = adafruit_mpu6050.MPU6050(i2c)

pwm_motor = pwmio.PWMOut(board.GP26, duty_cycle=2 ** 15, frequency=50)
pwm_motor2 = pwmio.PWMOut(board.GP21, duty_cycle=2 ** 15, frequency=50)
pwm_motor3 = pwmio.PWMOut(board.GP18, duty_cycle=2 ** 15, frequency=50)
pwm_motor4 = pwmio.PWMOut(board.GP28, duty_cycle=2 ** 15, frequency=50)

motor1speed = 40000
motor2speed = 40000
motor3speed = 40000
motor4speed = 40000

x = mpu.acceleration[0] / 9.8
y = mpu.acceleration[1] / 9.8
z = mpu.acceleration[2] / 9.8

Ki = 0.5
Kp = 0
Kd = 0

quadrant = 0
target = 0
integralPitch = 0
integralRoll = 0
derivativePitch = 0
derivativeRoll = 0
lastError = 0


while True:
    x = mpu.acceleration[0] / 9.8
    y = mpu.acceleration[1] / 9.8
    z = mpu.acceleration[2] / 9.8

    roll = 180 * math.atan(y/math.sqrt(x**2 + z**2))/(math.pi)
    pitch = 180 * math.atan(x/(math.sqrt(y**2 + z**2)))/(math.pi)

    hyp = math.sqrt(roll**2 + pitch**2)

    if roll > 0 and pitch > 0:
        quadrant = 1
    elif roll > 0 and pitch < 0:
        quadrant = 2
    elif roll < 0 and pitch < 0:
        quadrant = 3
    elif roll < 0 and pitch > 0:
        quadrant = 4  

    currentTime = time.monotonic()
    elapsedTime = currentTime - timeCheck

    error = target - hyp
    integral = integral + elapsedTime*error
    derivative = (error - lastError)/elapsedTime

    PID = Ki*error + Kp*integral + Kd*derivative
    pidmin = Kp*integral + Kd*derivative
    pidmax = Ki*45 + Kp*integral + Kd*derivative
    pidSpeed = simpleio.map_range(PID: pidmin, pidmax, 10000, 65535)

    pwm_motor.duty_cycle = motor1speed
    pwm_motor2.duty_cycle = motor2speed
    pwm_motor3.duty_cycle = motor3speed
    pwm_motor4.duty_cycle = motor4speed
    




    

    




    timeCheck = currentTime
    lastError = error
    print(f" Roll: {roll} & Pitch: {pitch} & Hyp: {hyp}")
    time.sleep(.5)
    
    
    
    
    
    
    
    
    '''    
    errorPitch = target - pitch
    errorRoll = target - roll
    
    integralPitch = integralPitch + elapsedTime * errorPitch
    integralRoll = integralRoll + elapsedTime * errorRoll

    derivativePitch = (errorPitch - lastErrorPitch)/elapsedTime
    derivativeRoll = (errorRoll - lastErrorRoll)/elapsedTime 
    '''
