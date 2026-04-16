import board
from adafruit_lsm6ds.ism330dhcx import ISM330DHCX
import time
import numpy as np

i2c = board.I2C()
sensor = ISM330DHCX(i2c) # you can add a second parameter for the address if needed

sensor.gyro_range = 4000 # set the gyro range to 4000 dps
sensor.accelerometer_range = 2 # set the accel range to 2 g

cameras = np.empty([3, 3])

# --- Step 1: Wait for the ball to be still, then measure gravity on the tilted surface ---
print("Hold the ball still on the surface...")
stable_count = 0
samples = []
while stable_count < 20:
    ax, ay, az = sensor.acceleration
    mag = np.linalg.norm([ax, ay, az])
    if abs(mag - 9.81) > 0.5:
        stable_count = 0
        samples.clear()
    else:
        stable_count += 1
        samples.append([ax, ay, az])
    time.sleep(0.01)