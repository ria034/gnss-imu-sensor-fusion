# EECE5554 — Robot Sensing & Navigation

Coursework for EECE5554 (Robot Sensing and Navigation), Northeastern University.
Each topic lives in its own folder with the same layout: `src/` (ROS 2
packages), `analysis/` (post-processing scripts and plots), `data/` (recorded
ROS 2 bags), and `report/` (the write-up submitted for that assignment).

**Author:** Ria Mendes (mendes.ri@northeastern.edu)

## Contents

| Folder | Topic | Summary |
|---|---|---|
| [ros2_intro](ros2_intro/) | ROS 2 fundamentals | First ROS 2 package (`py_pubsub`), workspace setup, Turtlesim. |
| [gnss_standalone](gnss_standalone/) | Standalone GNSS | Custom GPS puck driver (GPGGA → UTM), data collected in open/cluttered conditions, error analysis. |
| [gnss_rtk](gnss_rtk/) | RTK GNSS | RTK-corrected GPS driver + message set, standalone-vs-RTK accuracy comparison. |
| [imu](imu/) | IMU | VectorNav IMU driver ($VNYMR parsing), Allan variance / noise characterization, motion data analysis. |
| [gnss_imu_dead_reckoning](gnss_imu_dead_reckoning/) | GNSS + IMU dead reckoning | Combined GPS/IMU driving dataset, magnetometer hard/soft-iron correction, dead-reckoning vs. GPS velocity comparison. |
| [camera_calibration_mosaicing](camera_calibration_mosaicing/) | Camera calibration & mosaicing | MATLAB camera calibration, Harris corner detection, and image mosaicing/stitching. |

## Repository layout

```
EECE5554/
├── ros2_intro/
├── gnss_standalone/
├── gnss_rtk/
├── imu/
├── gnss_imu_dead_reckoning/
└── camera_calibration_mosaicing/
```

## Notes on this restructuring

This repo was reorganized from the original lab submission folders into a
consistent, browsable layout:

- Every topic now follows the same `src/ · analysis/ · data/ · report/`
  pattern instead of mixed `Analysis/`, `Data/`, `src/` naming per folder.
- The original per-lab numbering (`LAB0`, `LAB1`, …) and the old top-level
  `gnss/` folder (RTK GPS driver + comparison analysis) were replaced with
  plain, descriptive folder names.
- Several build files (`setup.cfg`, `CMakeLists.txt`) and the repo `README.md`
  had been accidentally saved as `.docx` — these were restored to plain text
  so the ROS 2 packages actually build again.
- A few analysis scripts were missing the `.py` extension or had names with
  spaces/parentheses/typos (`Analyis_motion_data(group)`,
  `Imgae_02.jpeg`, `BuildingDir-Cinder wall`, …) — these were renamed to
  consistent `snake_case` names.
- Every analysis script (Python and MATLAB) originally hardcoded an absolute
  path to data on the author's own machine — these were repointed at
  relative paths into each folder's own `data/`, so the scripts run
  out of the box from this repo.
- Duplicate GPS driver code that existed in three places (`gnss_standalone`,
  `gnss_imu_dead_reckoning`, and the old `gnss/`) is kept in each folder so
  every topic is still self-contained/buildable on its own.

## Building the ROS 2 packages

Each folder's `src/` is a normal ROS 2 workspace `src` directory. From inside
a folder:

```bash
colcon build --packages-select <package_name>
source install/setup.bash
```

## Large data files

`data/` folders contain `.db3` ROS 2 bag recordings (and one `.mp4`/several
`.tif` calibration images under `camera_calibration_mosaicing/`) and can be
large. `.gitignore` excludes `*.db3` by default — use `git add -f` if you want
to version them, or keep them as local-only data and share via a release/LFS
instead.
