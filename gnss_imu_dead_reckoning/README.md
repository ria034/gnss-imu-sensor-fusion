# GNSS + IMU Dead Reckoning

Combines the GPS and IMU drivers to drive-test dead reckoning: magnetometer
calibration, sensor fusion for heading, and a dead-reckoning
trajectory/velocity comparison against GPS ground truth.

## Contents

- `src/gps_driver/`, `src/gps_msgs/` — GPS driver/message (same as `gnss_standalone/`).
- `src/imu_driver/`, `src/imu_msg/` — IMU driver/message (same as `imu/`), plus
  a combined `imu_gps_launch.py` launch file to run both sensors together.
- `analysis/magnetic_correction_vnymr.py` — hard/soft-iron magnetometer
  calibration directly from raw `$VNYMR` strings in a bag (ellipse-fit style
  correction), based on the same approach as the Allan deviation script.
- `analysis/magnetic_correction_bag.py` — equivalent magnetometer calibration
  reading the IMU message fields back out of a ROS 2 bag via `rclpy`
  deserialization.
- `analysis/dead_reckoning_analysis.py` — full pipeline: corrected
  magnetometer heading, complementary-filtered yaw (mag + gyro), IMU-only
  velocity/displacement vs. GPS-derived velocity/displacement, and a
  dead-reckoning vs. GPS trajectory plot.
- `data/data_driving/` — primary driving-course recording.
- `data/data_going_in_circles/` — circular driving recording used for
  magnetometer calibration (figure-eight/circle maneuver).
- `data/lab4_bag/` — additional recorded bag.
- `report/gnss_imu_dead_reckoning_report.pdf` — write-up.

## Running the drivers

```bash
cd gnss_imu_dead_reckoning/src
colcon build --packages-select gps_msgs gps_driver imu_msg imu_driver
source install/setup.bash
ros2 launch gps_driver imu_gps_launch.py
```

## Reproducing the analysis

```bash
cd gnss_imu_dead_reckoning/analysis
python3 magnetic_correction_vnymr.py
python3 magnetic_correction_bag.py
python3 dead_reckoning_analysis.py
```
