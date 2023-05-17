import time
import math
import board
import adafruit_mpu6050
import busio
import pwmio
from PID_CPY import PID


sda_pin = board.GP10
scl_pin = board.GP11
i2c = busio.I2C(scl_pin, sda_pin)

mpu = adafruit_mpu6050.MPU6050(i2c)

pwm_motor = pwmio.PWMOut(board.GP26, duty_cycle=2 ** 15, frequency=50)
pwm_motor2 = pwmio.PWMOut(board.GP21, duty_cycle=2 ** 15, frequency=50)
pwm_motor3 = pwmio.PWMOut(board.GP18, duty_cycle=2 ** 15, frequency=50)
pwm_motor4 = pwmio.PWMOut(board.GP28, duty_cycle=2 ** 15, frequency=50)

defaultSpeed = 40000

motor1speed = defaultSpeed
motor2speed = defaultSpeed
motor3speed = defaultSpeed
motor4speed = defaultSpeed

target = 0
quadrant = []

x = mpu.acceleration[0] / 9.8
y = mpu.acceleration[1] / 9.8
z = mpu.acceleration[2] / 9.8


pid = PID(-900, 0, 0, setpoint=target)
pid.output_limits = (0, 65535)
#pid.sample_time = 0.01

while True:
    startTime = time.time()
    lastTime = startTime
    x = mpu.acceleration[0] / 9.8
    y = mpu.acceleration[1] / 9.8
    z = mpu.acceleration[2] / 9.8

    roll = 180 * math.atan(y/math.sqrt(x**2 + z**2))/(math.pi)
    pitch = 180 * math.atan(x/(math.sqrt(y**2 + z**2)))/(math.pi)
    hyp = math.sqrt(roll**2 + pitch**2)
    if hyp < 1.3:
        hyp = 0

    if roll > 0 and pitch > 0:
        quadrant = 1
    elif roll > 0 and pitch < 0:
        quadrant = 2
    elif roll < 0 and pitch < 0:
        quadrant = 3
    elif roll < 0 and pitch > 0:
        quadrant = 4
    #currentTime = time.time()
    
    #dt = currentTime - lastTime
    
    pidSpeed = pid(hyp)
    motorSpeed = abs(pidSpeed - defaultSpeed)
    if quadrant == 1:
        motor1speed = defaultSpeed + motorSpeed
        motor3speed = defaultSpeed - motorSpeed
    if quadrant == 3:
        motor3speed = defaultSpeed + motorSpeed
        motor1speed = defaultSpeed - motorSpeed
    
    if motor1speed > 65535: 
        motor1speed = 65535
    if motor3speed > 65535:
        motor3speed = 65535
    

    pwm_motor.duty_cycle = int(motor1speed)
    pwm_motor3.duty_cycle =int(motor3speed)



    #print(f" Roll: {roll} & Pitch: {pitch} & Hyp: {hyp} & Speed: {pidSpeed}")
    print(f"motor1: {motor1speed} & motor 3: {motor3speed} & pidspeed: {pidSpeed} & roll: {roll} & pitch: {pitch} & hyp: {hyp}")
    #time.sleep(.15)

'''
    currentTime = time.monotonic()
    elapsedTime = currentTime - timeCheck

    error = target - hyp
    integral = integral + elapsedTime*error
    derivative = (error - lastError)/elapsedTime

    PID = Ki*error + Kp*integral + Kd*derivative
    pidmin = Kp*integral + Kd*derivative
    pidmax = Ki*45 + Kp*integral + Kd*derivative
    pidSpeed = map_range(PID, pidmin, pidmax, 10000, 65535)

    # pwm_motor.duty_cycle = motor1speed
    # pwm_motor2.duty_cycle = motor2speed
    # pwm_motor3.duty_cycle = motor3speed
    # pwm_motor4.duty_cycle = motor4speed
    '''
    
