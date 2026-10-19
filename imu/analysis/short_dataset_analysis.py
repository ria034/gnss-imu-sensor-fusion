#!/usr/bin/env python3
import os
import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
import struct
import numpy as np


# ─────────────────────────────────────────────
# 1. DATA EXTRACTION
# ─────────────────────────────────────────────
def extract_imu_data(db3_file):
    conn   = sqlite3.connect(db3_file)
    cursor = conn.cursor()

    cursor.execute("SELECT id FROM topics WHERE name = '/imu/data'")
    topic_id = cursor.fetchone()[0]

    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (topic_id,)
    )
    messages = cursor.fetchall()
    conn.close()

    timestamps        = []
    Euler_angles      = []
    Magnetic_data     = []
    Acceleration_data = []
    Gyro_data         = []

    t0 = None
    for timestamp, data in messages:
        try:
            if t0 is None:
                t0 = timestamp
            # Convert nanosecond ROS timestamp → elapsed seconds
            t_sec = (timestamp - t0) * 1e-9

            roll  = struct.unpack('<d', data[-24:-16])[0]
            yaw   = struct.unpack('<d', data[-16:-8])[0]
            pitch = struct.unpack('<d', data[-8:])[0]

            mag_x = struct.unpack('<d', data[-120:-112])[0]
            mag_y = struct.unpack('<d', data[-112:-104])[0]
            mag_z = struct.unpack('<d', data[-104:-96])[0]

            acc_x = struct.unpack('<d', data[-232:-224])[0]
            acc_y = struct.unpack('<d', data[-224:-216])[0]
            acc_z = struct.unpack('<d', data[-216:-208])[0]

            gyr_x = struct.unpack('<d', data[-328:-320])[0]
            gyr_y = struct.unpack('<d', data[-320:-312])[0]
            gyr_z = struct.unpack('<d', data[-312:-304])[0]

            timestamps.append(t_sec)
            Euler_angles.append({'yaw': yaw, 'pitch': pitch, 'roll': roll})
            Magnetic_data.append({'mag_x': mag_x, 'mag_y': mag_y, 'mag_z': mag_z})
            Acceleration_data.append({'acc_x': acc_x, 'acc_y': acc_y, 'acc_z': acc_z})
            Gyro_data.append({'gyr_x': gyr_x, 'gyr_y': gyr_y, 'gyr_z': gyr_z})

        except Exception:
            continue

    time = np.array(timestamps)

    df_euler = pd.DataFrame(Euler_angles)
    df_mag   = pd.DataFrame(Magnetic_data)
    df_acc   = pd.DataFrame(Acceleration_data)
    df_gyro  = pd.DataFrame(Gyro_data)

    print(f"Extracted {len(df_euler)} data points from the file")
    print(f"Duration : {time[-1]:.2f} s")
    print(f"First message — yaw: {df_euler['yaw'].iloc[0]:.3f}, "
          f"pitch: {df_euler['pitch'].iloc[0]:.3f}, "
          f"roll:  {df_euler['roll'].iloc[0]:.3f}")

    return time, df_acc, df_euler, df_gyro, df_mag


# ─────────────────────────────────────────────
# 2. STATISTICS
# ─────────────────────────────────────────────
def calculate_means_and_medians(df_euler):
    stats = {}
    for axis in ('yaw', 'pitch', 'roll'):
        stats[f'mean_{axis}']   = df_euler[axis].mean()
        stats[f'median_{axis}'] = df_euler[axis].median()

    print("\n── Orientation Statistics ──")
    for axis in ('yaw', 'pitch', 'roll'):
        print(f"  {axis.capitalize():6s} │ mean: {stats[f'mean_{axis}']:+.3f}°  "
              f"median: {stats[f'median_{axis}']:+.3f}°")
    return stats


# ─────────────────────────────────────────────
# 3. HELPERS
# ─────────────────────────────────────────────
def _timeseries_axes(fig, time, cols, labels, colors, ylabel, title):
    """Plot multiple columns from a DataFrame on a single Axes."""
    ax = fig.add_subplot(111)
    for col, lbl, clr in zip(cols, labels, colors):
        # Ensure 1-D numpy array regardless of input type
        arr = np.asarray(col).ravel()
        ax.plot(time, arr, color=clr, linewidth=0.8, label=lbl)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    return ax


# ─────────────────────────────────────────────
# 4. PLOTTING
# ─────────────────────────────────────────────
def plot_data(time, df_euler, df_mag, df_acc, df_gyro, stats):

    COLORS = ['#FF6B6B', '#4ECDC4', '#6B93FF']   # X=red, Y=teal, Z=blue

    # ── (a) Gyroscope time series ─────────────────────────────────────────
    fig_g = plt.figure(figsize=(12, 5))
    _timeseries_axes(
        fig_g, time,
        cols   = [df_gyro['gyr_x'], df_gyro['gyr_y'], df_gyro['gyr_z']],
        labels = ['ω_x', 'ω_y', 'ω_z'],
        colors = COLORS,
        ylabel = 'Angular Velocity (rad/s)',
        title  = '(a) Gyroscope — Angular Velocity vs Time'
    )
    plt.tight_layout()
    plt.savefig('timeseries_gyro.png', dpi=150)
    print("Saved: timeseries_gyro.png")

    # ── (b) Accelerometer time series ────────────────────────────────────
    fig_a = plt.figure(figsize=(12, 5))
    _timeseries_axes(
        fig_a, time,
        cols   = [df_acc['acc_x'], df_acc['acc_y'], df_acc['acc_z']],
        labels = ['a_x', 'a_y', 'a_z'],
        colors = COLORS,
        ylabel = 'Acceleration (m/s²)',
        title  = '(b) Accelerometer — Linear Acceleration vs Time'
    )
    plt.tight_layout()
    plt.savefig('timeseries_accel.png', dpi=150)
    print("Saved: timeseries_accel.png")

    # ── (c) Magnetometer time series ─────────────────────────────────────
    fig_m = plt.figure(figsize=(12, 5))
    _timeseries_axes(
        fig_m, time,
        cols   = [df_mag['mag_x'], df_mag['mag_y'], df_mag['mag_z']],
        labels = ['B_x', 'B_y', 'B_z'],
        colors = COLORS,
        ylabel = 'Magnetic Field (µT)',
        title  = '(c) Magnetometer — Magnetic Field vs Time'
    )
    plt.tight_layout()
    plt.savefig('timeseries_mag.png', dpi=150)
    print("Saved: timeseries_mag.png")

    # ── (d) Euler angles time series ─────────────────────────────────────
    fig_e = plt.figure(figsize=(12, 5))
    _timeseries_axes(
        fig_e, time,
        cols   = [df_euler['yaw'], df_euler['pitch'], df_euler['roll']],
        labels = ['Yaw', 'Pitch', 'Roll'],
        colors = COLORS,
        ylabel = 'Angle (degrees)',
        title  = '(d) Orientation — Euler Angles vs Time'
    )
    plt.tight_layout()
    plt.savefig('timeseries_euler.png', dpi=150)
    print("Saved: timeseries_euler.png")

    # ── (e) Euler angle histograms ────────────────────────────────────────
    axes_info = [
        ('yaw',   '(e-1) Yaw Distribution',   COLORS[0]),
        ('pitch', '(e-2) Pitch Distribution',  COLORS[1]),
        ('roll',  '(e-3) Roll Distribution',   COLORS[2]),
    ]

    fig_h, hist_axes = plt.subplots(1, 3, figsize=(15, 5))

    for ax, (col, title_str, color) in zip(hist_axes, axes_info):
        mean_val   = stats[f'mean_{col}']
        median_val = stats[f'median_{col}']
        data       = df_euler[col]

        ax.hist(data, bins=30, alpha=0.6, color=color, edgecolor='white', linewidth=0.5)

        ax.axvline(mean_val,   color='red',  linestyle='--', linewidth=1.5,
                   label=f'Mean:   {mean_val:.2f}°')
        ax.axvline(median_val, color='navy', linestyle=':',  linewidth=1.5,
                   label=f'Median: {median_val:.2f}°')

        ax.set_title(title_str)
        ax.set_xlabel(f'{col.capitalize()} (degrees)')
        ax.set_ylabel('Sample Count')
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.2)

    fig_h.suptitle('(e) Euler Angle Output Distribution', fontsize=13, fontweight='bold')
    plt.tight_layout()
    plt.savefig('histograms_euler.png', dpi=150)
    print("Saved: histograms_euler.png")

    plt.show()


# ─────────────────────────────────────────────
# 5. MAIN
# ─────────────────────────────────────────────
def main():
    filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "dataset1", "DATASET1_0.db3")

    print("── Extracting IMU data ──")
    time, df_acc, df_euler, df_gyro, df_mag = extract_imu_data(filepath)

    stats = calculate_means_and_medians(df_euler)

    print("\n── Plotting ──")
    plot_data(time, df_euler, df_mag, df_acc, df_gyro, stats)


if __name__ == "__main__":
    main()