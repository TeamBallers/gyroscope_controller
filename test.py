from camera_down_detector import CameraDownDetector

if __name__ == "__main__":
    
    detector = CameraDownDetector(
        facing_down_threshold_deg=20.0,
        accel_correction_gain=0.02,
        accel_trust_tolerance=1.0,
    )

    
    detector.initialize_from_stationary(imu.acceleration)  # call once while still

    while True:
        result = detector.update(imu.acceleration, imu.gyro, dt=0.01)
        for i in range(3):
            print(f"Camera {i} facing down: {result[i]}", end=" ")
        # result[i] is True if camera i faces downward