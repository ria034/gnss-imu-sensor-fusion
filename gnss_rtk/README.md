# RTK GNSS

> This folder was originally the top-level `gnss/` directory in the repo. Its
> content (RTK driver + standalone-vs-RTK comparison) is this course's
> GNSS-RTK assignment.

Adds RTK correction support on top of the standalone GPS driver, and compares
standalone vs. RTK-corrected fix accuracy.

## Contents

- `src/gps_driver/`, `src/gps_msgs/` — the same standalone GPS driver/message
  used in `gnss_standalone/` (kept here so this folder builds on its own).
- `src/gps_rtk_driver/` — `gps_publisher` node variant that publishes the
  custom `Customrtk` message (adds RTK fix-quality fields) on `/gps`.
- `src/gps_rtk_msgs/` — `Customrtk.msg` interface definition.
- `analysis/standalone/` — three variants of the standalone-GPS accuracy
  analysis (reference-point error, centroid-based error, straight-line
  analysis).
- `analysis/rtk/` — matching analysis for the RTK-corrected fixes
  (open vs. occluded sky view, straight-line walk).
- `data/standalone/` — `cluttered/`, `non_cluttered/`, `straight_line/` bags
  (same recordings used in `gnss_standalone/`).
- `data/rtk/` — `open/`, `occluded/`, `walking/` RTK bags.

## Running the driver

```bash
cd gnss_rtk/src
colcon build --packages-select gps_msgs gps_driver gps_rtk_msgs gps_rtk_driver
source install/setup.bash
ros2 run gps_rtk_driver gps_publisher --ros-args -p port:=/dev/ttyUSB0
```

## Reproducing the analysis

```bash
cd gnss_rtk/analysis
python3 standalone/cluttered_open_data_analysis_known.py
python3 standalone/cluttered_open_data_analysis_centroid.py
python3 standalone/straight_line_data_analysis.py
python3 rtk/cluttered_open_data_analysis.py
python3 rtk/straight_line_data_analysis.py
```
