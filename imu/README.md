# IMU

ROS 2 driver for a VectorNav IMU (parses `$VNYMR` ASCII output), plus Allan
variance noise characterization and motion-data analysis.

## Contents

- `src/imu_driver/` — `imu_driver` node: reads `$VNYMR` strings over serial,
  converts Euler angles to quaternion, and publishes a custom `IMUmsg`
  (orientation + raw IMU + magnetic field) on `/imu`.
- `src/imu_msg/` — `IMUmsg.msg` interface definition.
- `analysis/allan_deviation_analysis.py` — Allan deviation analysis on a
  long-duration stationary recording (`allantools`) to characterize gyro/accel
  noise (bias instability, random walk, etc.).
- `analysis/motion_data_analysis_group.py` — 3D trajectory/orientation
  analysis of the group motion-capture recording.
- `analysis/short_dataset_analysis.py` — same style of analysis on a shorter
  dataset.
- `analysis/plots/imu_3d_plots.png` — generated figure.
- `data/five_hour_imu_data/` — long stationary recording used for Allan
  deviation.
- `data/dataset1/` — short IMU dataset.
- `data/motion_data_set_r/` — group motion dataset.
- `data/motion_video.mp4` — video recorded alongside a motion dataset.
- `report/imu_report.pdf` — write-up.

## Running the driver

```bash
cd imu/src
colcon build --packages-select imu_msg imu_driver
source install/setup.bash
ros2 run imu_driver imu_driver --ros-args -p port:=/dev/ttyUSB0
```

## Reproducing the analysis

```bash
cd imu/analysis
python3 allan_deviation_analysis.py
python3 motion_data_analysis_group.py
python3 short_dataset_analysis.py
```

> `motion_data_analysis_group.py` and `short_dataset_analysis.py` originally
> hardcoded a path to a dataset recorded on a different machine
> (`ros_lab4/ruggles2_imu/...`) that wasn't part of this export. Their paths
> were repointed at the closest matching dataset included here
> (`data/motion_data_set_r/` and `data/dataset1/` respectively) — double check
> this is the intended recording before relying on the output.
