from camera_down_detector import CameraDownDetector
import board
from adafruit_lsm6ds.ism330dhcx import ISM330DHCX
import argparse
import time
import numpy as np
import sys
try:
    from picamera2 import Picamera2
    from picamera2.configuration import CameraConfiguration
except ImportError:
    print("ERROR: picamera2 not found. This must be run on a Raspberry Pi.")
    print("Install with: sudo apt install -y python3-picamera2")
    sys.exit(1)

i2c = board.I2C()
sensor = ISM330DHCX(i2c) # you can add a second parameter for the address if needed

sensor.gyro_range = 4000 # set the gyro range to 4000 dps
sensor.accelerometer_range = 2 # set the accel range to 2 g

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script description")
    parser.add_argument("-v", "--verbose", action="store_true", help="Toggle verbose")
    parser.add_argument("-a", "--angle", type=float, help="Set the angle threshold in degrees", default=20.0)
    parser.add_argument("--accel_gain", type=float, help="Set the accelerometer correction gain", default=0.02)
    parser.add_argument("-p", "--pictures", action="store_true", help="Save pictures taken at 1fps")
    args = parser.parse_args()

    
    detector = CameraDownDetector(
        facing_down_threshold_deg=args.angle,
        accel_correction_gain=args.accel_gain,
        accel_trust_tolerance=1.0,
    )

    
    detector.initialize_from_stationary(sensor.acceleration)  # call once while still
    gyro_bias = detector.calibrate_gyro_bias(sensor)  # call once while still, after initialization

    picture_rate = 1.0  # pictures per second
    last_picture_time = time.monotonic()
    if args.pictures:
        camera = Picamera2()
        config = camera.create_still_configuration(
                main={"size": (4608, 2592)},  # Camera Module 3 max resolution
                buffer_count=2
            )
        camera.configure(config)
        camera.start()

        # Give camera time to warm up
        time.sleep(2)

    prevInterval = time.monotonic()
    while True:
        corrected_gyro = tuple(np.array(sensor.gyro) - gyro_bias)
        curInterval = time.monotonic()
        dt = curInterval - prevInterval
        prevInterval = curInterval
        result = detector.update(sensor.acceleration, corrected_gyro, dt)
        if args.pictures and (curInterval - last_picture_time) >= 1.0 / picture_rate:
            last_picture_time = curInterval
            timestamp = int(time.time())
            filename = ""
            if result[0]:
                filename = "down"
            filename += f"{timestamp}.jpg"
            camera.capture_file(filename)
            print(f"Saved picture: {filename}", end=" ")
        for i in range(3):
            if args.verbose:
                print(f"Camera {i} facing down: {result[i]}", end=" ")
            else:
                if result[i]:
                    print(f"Camera {i} facing down", end=" ")
        print()
        # result[i] is True if camera i faces downward