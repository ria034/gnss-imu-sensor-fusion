#!/usr/bin/env python3
import os
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
from mpl_toolkits.mplot3d import Axes3D
from scipy.integrate import cumulative_trapezoid
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def extract_data(db3_file):
    db_files = [f for f in os.listdir(db3_file) if f.endswith('.db3')]
    db_path  = os.path.join(db3_file, db_files[0])

    conn   = sqlite3.connect(db_path)
    cursor = conn.cursor()

    cursor.execute("SELECT id, name, type FROM topics")
    topics = {row[0]: (row[1], row[2]) for row in cursor.fetchall()}

    yaw_data        = []
    mag_data        = []
    yaw_rate_data   = []
    fwd_acc_data    = []
    timestamps_data = []

    cursor.execute(
        "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
    )
    for topic_id, timestamp, blob in cursor.fetchall():
        topic_name, topic_type = topics[topic_id]
        if topic_name != '/imu':
            continue

        topic_type = topic_type.replace('imu_msgs/', 'imu_msg/').replace('gps_msgs/', 'gps_msgs/')
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
        timestamps_data.append(timestamp * 1e-9)   # ns → s

    conn.close()

    yaw            = np.array(yaw_data)
    mag            = np.array(mag_data)
    yaw_rate       = np.array(yaw_rate_data)
    fwd_acc        = np.array(fwd_acc_data)
    timestamps_arr = np.array(timestamps_data)

    print(f"Extracted {len(mag)} samples")
    print(f"Mag range:      X=[{mag[:,0].min():.4f}, {mag[:,0].max():.4f}]  "
          f"Y=[{mag[:,1].min():.4f}, {mag[:,1].max():.4f}]  "
          f"Z=[{mag[:,2].min():.4f}, {mag[:,2].max():.4f}]")
    print(f"Yaw rate range: [{yaw_rate.min():.4f}, {yaw_rate.max():.4f}] rad/s")
    print(f"Fwd acc range:  [{fwd_acc.min():.4f}, {fwd_acc.max():.4f}] m/s²")

    return yaw, mag, yaw_rate, fwd_acc, timestamps_arr


def perform_magnetic_correction(raw_data):
    mx, my, mz = raw_data[:, 0], raw_data[:, 1], raw_data[:, 2]

    D = np.vstack([
        mx**2, my**2, mz**2,
        2*mx*my, 2*mx*mz, 2*my*mz,
        2*mx, 2*my, 2*mz
    ]).T

    beta, _, _, _ = np.linalg.lstsq(D, np.ones(len(mx)), rcond=None)

    Q = np.array([[beta[0], beta[3], beta[4]],
                  [beta[3], beta[1], beta[5]],
                  [beta[4], beta[5], beta[2]]])
    L = np.array([beta[6], beta[7], beta[8]])

    b         = -np.linalg.pinv(Q) @ L
    k         = b.T @ Q @ b + 1
    A         = np.real(linalg.sqrtm(Q / k))
    print(A)
    print(b)
    corrected_mag = (A @ (raw_data - b).T).T

    return corrected_mag, b, A


def plot_calibration(raw, corrected):
    fig = plt.figure(figsize=(16, 6))

    ax1 = fig.add_subplot(131)
    ax1.scatter(raw[:, 0],       raw[:, 1],       s=8, alpha=0.4, label='Raw')
    ax1.scatter(corrected[:, 0], corrected[:, 1], s=8, alpha=0.4, color='red', label='Corrected')
    ax1.set_title("XY"); ax1.set_xlabel("X"); ax1.set_ylabel("Y")
    ax1.axis('equal'); ax1.grid(True); ax1.legend()

    ax3 = fig.add_subplot(133, projection='3d')
    ax3.scatter(raw[:, 0],       raw[:, 1],       raw[:, 2],       s=5, alpha=0.2, label='Raw')
    ax3.scatter(corrected[:, 0], corrected[:, 1], corrected[:, 2], s=5, alpha=0.4, color='red', label='Corrected')
    ax3.set_title("3D"); ax3.set_xlabel("X"); ax3.set_ylabel("Y"); ax3.set_zlabel("Z")
    ax3.legend()

    plt.suptitle("Magnetometer Calibration", fontsize=13, fontweight='bold')
    plt.tight_layout()
    plt.savefig("mag_calibration.png", dpi=150)
    plt.show()


def compute_yaw(mag_data):
    return np.degrees(np.arctan2(-mag_data[:, 1], mag_data[:, 0]))


def get_yaw_from_gyro_yaw_rate(yaw_rate, timestamps_arr):
    yaw_gyro = cumulative_trapezoid(yaw_rate, timestamps_arr)
    return yaw_gyro


# def complimentary_filter(yaw_gyro, yaw_mag_filtered):
#     return high_pass_gyro, low_pass_mag, comp_yaw


if __name__ == "__main__":
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "data_going_in_circles")

    yaw_raw, raw_mag, yaw_rate, fwd_acc, timestamps_arr = extract_data(path)
    corrected_mag, b, A = perform_magnetic_correction(raw_mag)
    yaw_mag_filtered    = compute_yaw(corrected_mag)
    yaw_mag_raw         = compute_yaw(raw_mag)
    yaw_gyro            = get_yaw_from_gyro_yaw_rate(yaw_rate, timestamps_arr)
    # high_pass_gyro, low_pass_mag, comp_yaw = complimentary_filter(yaw_gyro, yaw_mag_filtered)

    raw_norms  = np.linalg.norm(raw_mag,       axis=1)
    corr_norms = np.linalg.norm(corrected_mag, axis=1)

    plot_calibration(raw_mag, corrected_mag)