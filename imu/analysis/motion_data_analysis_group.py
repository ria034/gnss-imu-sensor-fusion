#!/usr/bin/env python3
import os
import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import struct
import numpy as np

def extract_imu_data(db3_file):

    conn = sqlite3.connect(db3_file)
    cursor = conn.cursor()

    cursor.execute("SELECT id FROM topics WHERE name = '/imu/data'")
    topic_id = cursor.fetchone()[0]

    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (topic_id,)
    )
    messages = cursor.fetchall()
    conn.close()

    # Message tail layout (reading from end, all float64 = 8 bytes):
    # [-8:]      = pitch
    # [-16:-8]   = yaw
    # [-24:-16]  = roll
    # [-96:-24]  = magnetic_field_covariance (9 x float64 = 72 bytes, skip)
    # [-104:-96] = magnetic_field.z
    # [-112:-104]= magnetic_field.y
    # [-120:-112]= magnetic_field.x
    #
    # Inside sensor_msgs/Imu (before mag_field), also reading from known offsets from end:
    # [-120 - mag_header_size - 72 - 24 - 72 - 24 - ...] = linear_acceleration, angular_velocity
    # We use the imu sub-message covariance blocks as anchors (all 9x float64 = 72 bytes each)
    #
    # Simpler: linear_acceleration and angular_velocity sit just before the mag_field block.
    # linear_acceleration_covariance[9] = 72 bytes  -> ends at [-120 - mag_header]
    # linear_acceleration (Vector3)     = 24 bytes
    # angular_velocity_covariance[9]    = 72 bytes
    # angular_velocity (Vector3)        = 24 bytes
    #
    # We assume frame_id strings are empty (common in IMU drivers), giving fixed header = 16 bytes

    Euler_angles      = []
    Magnetic_data     = []
    Acceleration_data = []
    Gyro_data         = []
    timestamps = []

    for timestamp, data in messages:
        try:
            # --- Euler angles (last 24 bytes) ---
            
            roll  = struct.unpack('<d', data[-24:-16])[0]
            yaw   = struct.unpack('<d', data[-16:-8])[0]
            pitch = struct.unpack('<d', data[-8:])[0]

            # --- Magnetic field ---
            mag_x = struct.unpack('<d', data[-120:-112])[0]
            mag_y = struct.unpack('<d', data[-112:-104])[0]
            mag_z = struct.unpack('<d', data[-104:-96])[0]

            # Layout from end of message:
            # [-24:]       = roll, yaw, pitch        (24 bytes)
            # [-96:-24]    = mag_field_covariance     (72 bytes)
            # [-120:-96]   = mag_field vector         (24 bytes)
            # [-136:-120]  = mag header               (16 bytes)
            # [-208:-136]  = linear_accel_covariance  (72 bytes)
            # [-232:-208]  = linear_acceleration      (24 bytes)
            # [-304:-232]  = angular_vel_covariance   (72 bytes)
            # [-328:-304]  = angular_velocity         (24 bytes)

            acc_x = struct.unpack('<d', data[-232:-224])[0]
            acc_y = struct.unpack('<d', data[-224:-216])[0]
            acc_z = struct.unpack('<d', data[-216:-208])[0]

            gyr_x = struct.unpack('<d', data[-328:-320])[0]
            gyr_y = struct.unpack('<d', data[-320:-312])[0]
            gyr_z = struct.unpack('<d', data[-312:-304])[0]
            timestamps.append(timestamp)
            Euler_angles.append({'yaw': yaw, 'pitch': pitch, 'roll': roll})
            Magnetic_data.append({'magnetic_x': mag_x, 'magnetic_y': mag_y, 'magnetic_z': mag_z})
            Acceleration_data.append({'acceleration_x': acc_x, 'acceleration_y': acc_y, 'acceleration_z': acc_z})
            Gyro_data.append({'angular_x': gyr_x, 'angular_y': gyr_y, 'angular_z': gyr_z})

        except:
            continue

    df_euler        = pd.DataFrame(Euler_angles)
    df_mag          = pd.DataFrame(Magnetic_data)
    df_acceleration = pd.DataFrame(Acceleration_data)
    df_gyro         = pd.DataFrame(Gyro_data)
    df_timestamps = np.array(timestamps, dtype=np.int64)
    print(f"Extracted {len(df_euler)} datapoints from the file")
    print(f"First message — yaw: {df_euler['yaw'].iloc[0]:.3f}, pitch: {df_euler['pitch'].iloc[0]:.3f}, roll: {df_euler['roll'].iloc[0]:.3f}")
    
    return df_acceleration, df_euler, df_gyro, df_mag, df_timestamps


def calculate_means_and_medians(df_euler):

    mean_yaw     = df_euler['yaw'].mean()
    mean_pitch   = df_euler['pitch'].mean()
    mean_roll    = df_euler['roll'].mean()
    median_yaw   = df_euler['yaw'].median()
    median_pitch = df_euler['pitch'].median()
    median_roll  = df_euler['roll'].median()

    print(f"Mean   — Yaw: {mean_yaw:.3f}, Pitch: {mean_pitch:.3f}, Roll: {mean_roll:.3f}")
    print(f"Median — Yaw: {median_yaw:.3f}, Pitch: {median_pitch:.3f}, Roll: {median_roll:.3f}")

    return mean_yaw, mean_roll, mean_pitch, median_yaw, median_roll, median_pitch


def plot_data(df_euler, df_mag, df_acceleration, df_gyro, df_timestamps):
    
    relative_time = (df_timestamps-df_timestamps[0])/1e9
    # df_acceleration['time'] = df_timestamps
    # --- 3D Plots ---
    fig = plt.figure(figsize=(18, 12))

   
    ax4 = fig.add_subplot(224)
    ax4.plot(df_acceleration['acceleration_x'].to_numpy(),df_acceleration['acceleration_y'].to_numpy(), color='#6B93FF')
    ax4.set_xlabel('Magnetic_X'); ax4.set_ylabel('Magnetic_Y')
    ax4.set_title('Magnetic Data')

    plt.suptitle('IMU Sensor Data (3D)')
    plt.tight_layout()
    plt.savefig('imu_3d_plots.png', dpi=150)
    print("Plot saved as 'imu_3d_plots.png'")
    plt.show()

   

def main():
    filepath =os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "motion_data_set_r", "Motion_data_set_r_0.db3")

    print("Extracting IMU data...")
    df_acceleration, df_euler, df_gyro, df_mag ,df_timestamps= extract_imu_data(filepath)

    print("\nCalculating means and medians...")
    mean_yaw, mean_roll, mean_pitch, median_yaw, median_roll, median_pitch = calculate_means_and_medians(df_euler)

    print("\nPlotting...")
    plot_data(df_euler, df_mag, df_acceleration, df_gyro, df_timestamps)


if __name__ == "__main__":
    main()