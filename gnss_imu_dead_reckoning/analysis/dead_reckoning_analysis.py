#!/usr/bin/env python3
import os
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid
from scipy.signal import medfilt

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def extract_data(db3_file):
    db_path = db3_file
    conn    = sqlite3.connect(db_path)
    cursor  = conn.cursor()

    cursor.execute("SELECT id, name, type FROM topics")
    topics = {row[0]: (row[1], row[2]) for row in cursor.fetchall()}

    yaw_data, mag_data, yaw_rate_data, fwd_acc_data, timestamps_data, side_acc_data = [], [], [], [], [], []

    cursor.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp")
    for topic_id, timestamp, blob in cursor.fetchall():
        topic_name, topic_type = topics[topic_id]
        if topic_name != '/imu':
            continue
        topic_type = topic_type.replace('imu_msgs/', 'imu_msg/')
        msg_type = get_message(topic_type)
        msg      = deserialize_message(blob, msg_type)

        yaw_data.append(msg.imu.orientation.z)
        mag_data.append([
            msg.mag_field.magnetic_field.x,
            msg.mag_field.magnetic_field.y,
            msg.mag_field.magnetic_field.z,
        ])
        yaw_rate_data.append(msg.imu.angular_velocity.z)
        fwd_acc_data.append(msg.imu.linear_acceleration.x)
        side_acc_data.append(msg.imu.linear_acceleration.y)
        timestamps_data.append(timestamp * 1e-9)

    conn.close()

    if not yaw_data:
        print("ERROR: Topic /imu not found!")
        return None

    return (np.array(yaw_data), np.array(mag_data), np.array(yaw_rate_data),
            np.array(fwd_acc_data), np.array(timestamps_data), np.array(side_acc_data))


def extract_gps_data(db3_file):
    db_path = db3_file
    conn    = sqlite3.connect(db_path)
    cursor  = conn.cursor()

    cursor.execute("SELECT id, name, type FROM topics")
    topics = {row[0]: (row[1], row[2]) for row in cursor.fetchall()}

    utm_easting, utm_northing, gps_timestamps = [], [], []

    cursor.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp")
    for topic_id, timestamp, blob in cursor.fetchall():
        topic_name, topic_type = topics[topic_id]
        if topic_name != '/gps':
            continue
        msg_type = get_message(topic_type)
        msg      = deserialize_message(blob, msg_type)

        utm_easting.append(msg.utm_easting)
        utm_northing.append(msg.utm_northing)
        gps_timestamps.append(timestamp * 1e-9)

    conn.close()

    if not utm_easting:
        raise RuntimeError("No GPS topic found.")

    utm_e = np.array(utm_easting)
    utm_n = np.array(utm_northing)
    print(f"GPS points extracted: {len(utm_e)}")
    print(f"  Easting  {utm_e.min():.1f} → {utm_e.max():.1f}  (span {utm_e.max()-utm_e.min():.1f} m)")
    print(f"  Northing {utm_n.min():.1f} → {utm_n.max():.1f}  (span {utm_n.max()-utm_n.min():.1f} m)")

    return utm_e, utm_n, np.array(gps_timestamps)


def perform_magnetic_correction(raw_data):
    A = np.array([[4.18840892, 0.0252338,  1.52507168],
                  [0.0252338,  4.95039529, 1.08457237],
                  [1.52507168, 1.08457237, 12.61154806]])
    b = np.array([0.51458966, -0.53171244, 0.81041767])
    return (A @ (raw_data - b).T).T


def compute_yaw(corrected_mag):
    return np.degrees(np.arctan2(-corrected_mag[:, 1], corrected_mag[:, 0]))


def get_yaw_from_gyro(yaw_rate, timestamps_arr):
    time_s = timestamps_arr - timestamps_arr[0]
    return np.degrees(cumulative_trapezoid(yaw_rate, time_s, initial=0))


def complementary_filter(yaw_mag_deg, gyro_rate_rad, dt, alpha=0.98):
    comp_yaw    = np.zeros_like(yaw_mag_deg)
    comp_yaw[0] = yaw_mag_deg[0]
    for i in range(1, len(yaw_mag_deg)):
        gyro_delta  = np.rad2deg(gyro_rate_rad[i]) * dt[i]
        prediction  = (comp_yaw[i-1] + gyro_delta + 180) % 360 - 180
        mag_error   = (yaw_mag_deg[i] - prediction + 180) % 360 - 180
        comp_yaw[i] = prediction + (1 - alpha) * mag_error
    return comp_yaw


def estimate_forward_velocity_imu(fwd_acc, timestamps_arr, stop_threshold=1):
    time_s           = timestamps_arr - timestamps_arr[0]
    debiased_fwd_acc = fwd_acc - np.mean(fwd_acc)
    dt               = np.diff(time_s, prepend=(time_s[1] - time_s[0]))
    velocity         = np.zeros(len(debiased_fwd_acc))
    velocity_raw     = np.zeros(len(fwd_acc))
    for i in range(1, len(debiased_fwd_acc)):
        velocity[i]     = velocity[i-1] + debiased_fwd_acc[i] * dt[i]
        velocity_raw[i] = fwd_acc[i] * dt[i] + velocity_raw[i-1]
    return velocity, velocity_raw


def estimate_forward_velocity_gps(utm_easting, utm_northing, timestamps_arr_gps):
    dt_gps  = np.diff(timestamps_arr_gps)
    vel_gps = np.sqrt((np.diff(utm_easting) / dt_gps)**2 +
                      (np.diff(utm_northing) / dt_gps)**2)
    return np.insert(vel_gps, 0, 0.0)


def get_gps_speed_on_imu_grid(utm_easting, utm_northing,
                               timestamps_arr_gps, timestamps_arr_imu):
    gps_speed        = estimate_forward_velocity_gps(utm_easting, utm_northing,
                                                      timestamps_arr_gps)
    gps_speed_smooth = medfilt(gps_speed, kernel_size=5)
    mask  = (timestamps_arr_imu >= timestamps_arr_gps[0]) & \
            (timestamps_arr_imu <= timestamps_arr_gps[-1])

    speed_on_imu       = np.zeros(len(timestamps_arr_imu))
    speed_on_imu[mask] = np.interp(timestamps_arr_imu[mask],
                                   timestamps_arr_gps, gps_speed_smooth)

    print(f"GPS speed on IMU grid — mean: {speed_on_imu[mask].mean():.2f} m/s  "
          f"max: {speed_on_imu[mask].max():.2f} m/s")
    return speed_on_imu


def estimate_trajectory(forward_velocity, yaw_deg, timestamps_arr):
    time_s  = timestamps_arr - timestamps_arr[0]
    yaw_rad = np.deg2rad(yaw_deg)
    xe = cumulative_trapezoid(forward_velocity * np.cos(yaw_rad), time_s, initial=0)
    xn = cumulative_trapezoid(forward_velocity * np.sin(yaw_rad), time_s, initial=0)
    return xe, xn


def align_trajectories(xe, xn, utm_easting, utm_northing, window=50):
    xe_c  = xe - xe[0];  xn_c  = xn - xn[0]
    gps_e = utm_easting  - utm_easting[0]
    gps_n = utm_northing - utm_northing[0]
    w     = min(window, len(xe_c) - 1, len(gps_e) - 1)
    angle = np.arctan2(gps_n[w], gps_e[w]) - np.arctan2(xn_c[w], xe_c[w])
    cos_a, sin_a = np.cos(angle), np.sin(angle)
    xe_final = cos_a * xe_c - sin_a * xn_c + utm_easting[0]
    xn_final = sin_a * xe_c + cos_a * xn_c + utm_northing[0]
    return xe_final, xn_final


def plot_calibration(raw, corrected):
    fig = plt.figure(figsize=(12, 5))
    ax1 = fig.add_subplot(121)
    ax1.scatter(raw[:, 0], raw[:, 1], s=5, alpha=0.3, label='Raw')
    ax1.scatter(corrected[:, 0], corrected[:, 1], s=5, alpha=0.3,
                color='red', label='Corrected')
    ax1.set_title("Magnetometer XY"); ax1.axis('equal'); ax1.legend()
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.scatter(corrected[:, 0], corrected[:, 1], corrected[:, 2], s=2, color='red')
    ax2.set_title("Corrected 3D Sphere")
    plt.tight_layout(); plt.show()


def time_series_plot_magnetometer(raw, corrected, time_s_imu):
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    labels = ['X', 'Y', 'Z']
    for i, (ax, label) in enumerate(zip(axes, labels)):
        ax.plot(time_s_imu, raw[:, i],       'g--', alpha=0.5, label=f'Raw M{label.lower()}')
        ax.plot(time_s_imu, corrected[:, i], 'r',   alpha=0.7, label=f'Corrected M{label.lower()}')
        ax.set_title(f"Magnetometer {label}-axis")
        ax.set_ylabel("Magnetic Field")
        ax.legend()
        ax.grid(True)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Raw vs Corrected Magnetometer Data", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig("magnetometer_comparison.png")
    plt.show()


def plot_yaw_comparison(timestamps_arr, yaw_mag, yaw_gyro, yaw_comp, yaw_raw):
    time_s = timestamps_arr - timestamps_arr[0]
    plt.figure(figsize=(14, 7))
    plt.plot(time_s, yaw_raw,  'g--', alpha=0.4, label='IMU Internal Yaw')
    plt.plot(time_s, yaw_mag,  'r',   alpha=0.4, label='LP: Mag Yaw')
    plt.plot(time_s, yaw_gyro, 'b',   alpha=0.4, label='HP: Integrated Gyro')
    plt.plot(time_s, yaw_comp, 'k',   linewidth=1.5, label='Complementary Filter')
    plt.title("Yaw Comparison"); plt.xlabel("Time (s)"); plt.ylabel("Yaw (deg)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig("yaw_comparison.png"); plt.show()


def plot_velocities(time_s_imu, vel_imu, time_s_gps, vel_gps):
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=False)
    axes[0].plot(time_s_imu, vel_imu, 'b', alpha=0.7, label='IMU velocity')
    axes[0].set_title("IMU Velocity"); axes[0].set_ylabel("m/s")
    axes[0].legend(); axes[0].grid(True)
    axes[1].plot(time_s_gps, vel_gps, 'r', alpha=0.7, label='GPS speed')
    axes[1].set_title("GPS Speed"); axes[1].set_ylabel("m/s")
    axes[1].legend(); axes[1].grid(True)
    plt.tight_layout(); plt.show()


def plot_imu_velocities(time_s_imu, velocity_imu_raw, velocity_imu):
    plt.figure(figsize=(14, 7))
    plt.plot(time_s_imu, velocity_imu_raw, 'g--', alpha=0.4, label='Uncorrected IMU Velocity')
    plt.plot(time_s_imu, velocity_imu,     'r',   alpha=0.4, label='Corrected IMU Velocity')
    plt.title("IMU Forward Velocity Comparison"); plt.xlabel("Time (s)"); plt.ylabel("Velocity(m/s)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.show()


def plot_displacement(time_s_imu, disp_imu, time_s_gps, disp_gps):
    plt.figure(figsize=(14, 6))
    plt.plot(time_s_imu, disp_imu, 'b', alpha=0.6, label='IMU Displacement')
    plt.plot(time_s_gps, disp_gps, 'r', alpha=0.6, label='GPS Displacement')
    plt.title("1-D Displacement Comparison")
    plt.xlabel("Time (s)"); plt.ylabel("Distance (m)")
    plt.legend(); plt.grid(True); plt.tight_layout(); plt.show()


def plot_lateral_acceleration_comparison(lat_acc, side_acc_data, time_s_imu):
    plt.figure(figsize=(14, 6))
    plt.plot(time_s_imu, side_acc_data, 'b', alpha=0.6, label='Measured Lateral Acc (IMU)')
    plt.plot(time_s_imu, lat_acc,        'r', alpha=0.6, label='Estimated Lateral Acc (v * yaw_rate)')
    plt.title("Comparison of Lateral Acceleration")
    plt.xlabel("Time (s)"); plt.ylabel("Acceleration (m/s^2)")
    plt.legend(); plt.grid(True); plt.tight_layout(); plt.show()


def plot_trajectory_comparison(ax, xe, xn, utm_easting, utm_northing, title,
                               dr_color='b', dr_label='Dead Reckoning'):
    ax.plot(utm_easting, utm_northing, 'r',      linewidth=2,   label='GPS Track')
    ax.plot(xe,          xn,           dr_color, linewidth=1.5, alpha=0.8, label=dr_label)
    ax.scatter(utm_easting[0], utm_northing[0], c='red',    s=300, zorder=6, marker='*', label='GPS Start')
    ax.scatter(xe[0],          xn[0],           c=dr_color, s=300, zorder=5, marker='o', label='DR Start')
    ax.set_title(title); ax.set_xlabel("Easting (m)"); ax.set_ylabel("Northing (m)")
    ax.legend(); ax.grid(True); ax.axis('equal')


# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "data_driving", "data_driving_0.db3")

    data_tuple = extract_data(path)
    utm_easting, utm_northing, timestamps_arr_gps = extract_gps_data(path)

    if data_tuple is None:
        print("IMU extraction failed.")
    else:
        yaw_raw, raw_mag, yaw_rate, fwd_acc, timestamps_arr_imu, side_acc_data = data_tuple

        if np.all(raw_mag == 0):
            print("CRITICAL ERROR: Raw magnetometer data is all zeros.")
        else:
            # --- 1. Time base ---
            time_s_imu = timestamps_arr_imu - timestamps_arr_imu[0]
            time_s_gps = timestamps_arr_gps - timestamps_arr_gps[0]
            dt = np.diff(time_s_imu, prepend=(time_s_imu[1] - time_s_imu[0]))

            # --- 2. Yaw ---
            corrected_mag = perform_magnetic_correction(raw_mag)
            yaw_mag_deg   = compute_yaw(corrected_mag)
            yaw_gyro_deg  = get_yaw_from_gyro(yaw_rate, timestamps_arr_imu)
            yaw_fused     = complementary_filter(yaw_mag_deg, yaw_rate, dt, alpha=0.98)

            # --- 3. Velocity ---
            vel_imu, velocity_imu_raw = estimate_forward_velocity_imu(fwd_acc, timestamps_arr_imu)
            vel_gps   = estimate_forward_velocity_gps(utm_easting, utm_northing, timestamps_arr_gps)
            gps_speed = get_gps_speed_on_imu_grid(utm_easting, utm_northing,
                                                   timestamps_arr_gps, timestamps_arr_imu)

            # lateral acceleration = v * yaw_rate
            lat_acc = vel_imu * yaw_rate

            # --- 4. Displacement ---
            disp_imu = cumulative_trapezoid(vel_imu, time_s_imu, initial=0)
            disp_gps = cumulative_trapezoid(vel_gps, time_s_gps, initial=0)

            # --- 5. Dead reckoning trajectories ---
            # compute heading directly from GPS positions
            gps_heading = np.degrees(np.arctan2(np.diff(utm_northing, append=utm_northing[-1]),
                                                 np.diff(utm_easting,  append=utm_easting[-1])))
            # interpolate onto IMU grid
            gps_heading_on_imu = np.interp(timestamps_arr_imu, timestamps_arr_gps, gps_heading)
            # estimate trajectory with GPS speed and GPS heading
            xe_a, xn_a = estimate_trajectory(vel_imu, yaw_fused, timestamps_arr_imu)
            xe_a, xn_a = align_trajectories(xe_a, xn_a, utm_easting, utm_northing)

            # --- 6. Plots ---
            plot_calibration(raw_mag, corrected_mag)
            time_series_plot_magnetometer(raw_mag, corrected_mag, time_s_imu)
            plot_yaw_comparison(timestamps_arr_imu, yaw_mag_deg, yaw_gyro_deg, yaw_fused, yaw_raw)
            plot_velocities(time_s_imu, vel_imu, time_s_gps, vel_gps)
            plot_imu_velocities(time_s_imu, velocity_imu_raw, vel_imu)
            plot_displacement(time_s_imu, disp_imu, time_s_gps, disp_gps)
            plot_lateral_acceleration_comparison(lat_acc, side_acc_data, time_s_imu)

            fig, axes = plt.subplots(1, 1, figsize=(9, 9))
            plot_trajectory_comparison(
                axes, xe_a, xn_a, utm_easting, utm_northing,
                title="A: IMU velocity + Comp Yaw (drifts)",
                dr_color='b', dr_label='DR — IMU vel + Comp Yaw')
            plt.suptitle("DR vs GPS Trajectory Comparison", fontsize=14, fontweight='bold')
            plt.tight_layout()
            plt.savefig("trajectory_comparison.png")
            plt.show()