## Code to stablize drone

import time
import board
import adafruit_mpu6050
import busio

sda_pin = board.GP14
scl_pin = board.GP15
i2c = busio.I2C(scl_pin, sda_pin)

mpu = adafruit_mpu6050.MPU6050(i2c)

error = 0



while True:
    x = mpu.acceleration[0]
    y = mpu.acceleration[1]
    z = mpu.acceleration[2]

# calculates error variable
    


# Converts acceleration values into voltage 0-5
    vx = x/(acc. output range) * 5
    vy = y/() * 5
    vz = z/() * 5

    time.sleep(1)   