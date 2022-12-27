# Code when drone is started

import time
import board
import digitalio

led = digitalio.DigitalInOut(board.GP15) # put on any pin
led.direction = digitalio.Direction.OUTPUT
button = digitalio.DigitalInOut(board.GP11) # put on any pin
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP
motor1pin = ()
motor2pin = ()
motor3pin = ()
motor4pin = ()
int speed = 0
defaultSpeed = 10 # should be considerably above liftoff speed to allow slow & smooth landing
int liftoffSpeed = () #speed that drone will create enough force to take off

# this section is before the while True so it only happens once (before takeoff)
while speed < (liftoffSpeed - 5): #stops incrementing right before drone takes off to ready user
        setSpeed (motor1pin, speed)) #find proper motor speed syntax & replciate for all 4 motors
        speed = speed + 1 # slowly sets motorspeed probably need to change the increment
        led.value = True #turn on a green LED to indicate drone is ready for takeoff
liftoffCheck = 0

while True: 
    if not button.value:
        if liftoffCheck == 0:
            while speed < defaultSpeed:
                
            setSpeed (motor1pin, speed) # find proper syntax (look above)
            speed = speed + 1
            liftoffCheck = 1
        if liftoffCheck = 1:
            setSpeed (motor1pin, speed)
            
    # other drone code will fill the void
    if not button.value:

    


