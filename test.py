from camera_down_detector import CameraDownDetector
import board
from adafruit_lsm6ds.ism330dhcx import ISM330DHCX
import argparse

i2c = board.I2C()
sensor = ISM330DHCX(i2c) # you can add a second parameter for the address if needed

sensor.gyro_range = 4000 # set the gyro range to 4000 dps
sensor.accelerometer_range = 2 # set the accel range to 2 g

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script description")
    parser.add_argument("-v", "--verbose", action="store_true", help="Toggle verbose")
    args = parser.parse_args()

    
    detector = CameraDownDetector(
        facing_down_threshold_deg=20.0,
        accel_correction_gain=0.02,
        accel_trust_tolerance=1.0,
    )

    
    detector.initialize_from_stationary(sensor.acceleration)  # call once while still

    while True:
        result = detector.update(sensor.acceleration, sensor.gyro, dt=0.01)
        for i in range(3):
            if args.verbose:
                print(f"Camera {i} facing down: {result[i]}", end=" ")
            else:
                if result[i]:
                    print(f"Camera {i} facing down", end=" ")
        print()
        # result[i] is True if camera i faces downward