# Standalone GNSS

A custom ROS 2 driver for a standalone (non-RTK) GPS puck, plus open-loop
accuracy analysis in open vs. cluttered environments.

## Contents

- `src/gps_driver/` — `gps_publisher` node: reads GPGGA sentences over serial,
  converts lat/lon to UTM, and publishes a custom `GpsMsg` on `/gps`.
- `src/gps_msgs/` — `GpsMsg.msg` interface definition.
- `analysis/cluttered_open_data_analysis.py` — compares GPS fixes recorded in
  open vs. cluttered (multipath-heavy) conditions.
- `analysis/straight_line_data_analysis.py` — straight-line walk analysis
  (trajectory linearity / deviation).
- `analysis/plots/` — generated figures: `gps_plot.png`, `gps_trajectories.png`,
  `gps_error_histogram_cluttered.png`, `gps_error_histogram_noncluttered.png`.
- `data/` — recorded ROS 2 bags: `cluttered/`, `non_cluttered/`, `straight_line/`.
- `report/gnss_standalone_report.pdf` — write-up.

## Running the driver

```bash
cd gnss_standalone/src
colcon build --packages-select gps_msgs gps_driver
source install/setup.bash
ros2 run gps_driver gps_publisher --ros-args -p port:=/dev/ttyUSB0
```

## Reproducing the analysis

```bash
cd gnss_standalone/analysis
python3 cluttered_open_data_analysis.py
python3 straight_line_data_analysis.py
```

Each script reads the matching ROS 2 bag(s) under `../data/` (SQLite `.db3`
format) and extracts the `/gps` topic with `sqlite3`/`pandas`.
