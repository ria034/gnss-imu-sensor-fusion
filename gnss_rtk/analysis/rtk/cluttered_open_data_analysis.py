#!/usr/bin/env python3
import os
import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
import struct
import numpy as np


def extract_gps_data(db3_file):
    """Extract GPS data from ROS2 bag (SQLite) file."""
    conn = sqlite3.connect(db3_file)
    cursor = conn.cursor()

    # Get topic ID for /gps
    cursor.execute("SELECT id FROM topics WHERE name = '/gps'")
    topic_id = cursor.fetchone()[0]

    # Get messages
    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (topic_id,)
    )
    messages = cursor.fetchall()
    conn.close()

    if len(messages) == 0:
        raise ValueError("No GPS messages found in the database.")

    timestamp, data = messages[0]
    field_offset = None

    for offset in range(0, len(data) - 48):
        try:
            lat = struct.unpack('<d', data[offset:offset+8])[0]
            lon = struct.unpack('<d', data[offset+8:offset+16])[0]
            utm_e = struct.unpack('<d', data[offset+32:offset+40])[0]
            if (-90 <= lat <= 90 and -180 <= lon <= 180 and 100000 <= utm_e <= 900000):
                field_offset = offset
                break
        except:
            continue

    if field_offset is None:
        raise ValueError("Could not detect GPS field offset.")

    gps_data = []
    for timestamp, data in messages:
        try:
            alt = struct.unpack('<d', data[field_offset+16:field_offset+24])[0]
            utm_e = struct.unpack('<d', data[field_offset+32:offset+40])[0]
            utm_n = struct.unpack('<d', data[field_offset+40:offset+48])[0]

            if 100000 <= utm_e <= 900000 and 0 <= utm_n <= 10000000:
                gps_data.append({
                    'timestamp': timestamp / 1e9,
                    'utm_e': utm_e,
                    'utm_n': utm_n,
                    'alt': alt
                })
        except:
            continue

    df = pd.DataFrame(gps_data)
    print(f"Extracted {len(df)} GPS points from {db3_file}")
    return df


def plot_data(df_clut, df_open):
    # ---- Compute centroids ----
    clut_centroid_e = df_clut['utm_e'].mean()
    clut_centroid_n = df_clut['utm_n'].mean()
    open_centroid_e = df_open['utm_e'].mean()
    open_centroid_n = df_open['utm_n'].mean()

    print("\nCentroids:")
    print(f"Cluttered -> E={clut_centroid_e:.3f}, N={clut_centroid_n:.3f}")
    print(f"Open      -> E={open_centroid_e:.3f}, N={open_centroid_n:.3f}")

    # ---- Center data ----
    df_clut_rel = df_clut.copy()
    df_clut_rel['e_rel'] = df_clut['utm_e'] - clut_centroid_e
    df_clut_rel['n_rel'] = df_clut['utm_n'] - clut_centroid_n

    df_open_rel = df_open.copy()
    df_open_rel['e_rel'] = df_open['utm_e'] - open_centroid_e
    df_open_rel['n_rel'] = df_open['utm_n'] - open_centroid_n

    # ---- Total offset calculation (sum of absolute displacement) ----
    clut_total_offset_e = df_clut_rel['e_rel'].abs().sum()
    clut_total_offset_n = df_clut_rel['n_rel'].abs().sum()

    open_total_offset_e = df_open_rel['e_rel'].abs().sum()
    open_total_offset_n = df_open_rel['n_rel'].abs().sum()

    print("\nTotal Offsets (sum of absolute displacements):")
    print(f"Cluttered Easting Offset: {clut_total_offset_e:.3f} m")
    print(f"Cluttered Northing Offset: {clut_total_offset_n:.3f} m")
    print(f"Open Easting Offset: {open_total_offset_e:.3f} m")
    print(f"Open Northing Offset: {open_total_offset_n:.3f} m")

    # ---- Compute Euclidean errors (existing behavior unchanged) ----
    df_clut_rel['euclidean_error'] = np.sqrt(
        df_clut_rel['e_rel']**2 + df_clut_rel['n_rel']**2
    )

    df_open_rel['euclidean_error'] = np.sqrt(
        df_open_rel['e_rel']**2 + df_open_rel['n_rel']**2
    )

    clut_errors = df_clut_rel['euclidean_error']
    open_errors = df_open_rel['euclidean_error']

    # ---- Print statistics ----
    print("\nEuclidean Error Statistics (relative to centroid):")
    print("Cluttered:")
    print(f"  Mean  = {clut_errors.mean():.3f} m")
    print(f"  RMS   = {np.sqrt(np.mean(clut_errors**2)):.3f} m")
    print(f"  95%   = {np.percentile(clut_errors, 95):.3f} m")
    print("Open:")
    print(f"  Mean  = {open_errors.mean():.3f} m")
    print(f"  RMS   = {np.sqrt(np.mean(open_errors**2)):.3f} m")
    print(f"  95%   = {np.percentile(open_errors, 95):.3f} m")

    # ---- Northing vs Easting Scatter (UNCHANGED) ----
    plt.figure(figsize=(8, 6))
    plt.scatter(df_clut_rel['e_rel'], df_clut_rel['n_rel'], s=15, alpha=0.6, label='Cluttered')
    plt.scatter(df_open_rel['e_rel'], df_open_rel['n_rel'], s=15, alpha=0.6, label='Open')
    plt.scatter(0, 0, color='black', marker='x', s=100, label='Centroid')
    plt.xlabel('Easting Offset (m)')
    plt.ylabel('Northing Offset (m)')
    plt.title('Northing vs Easting (Centered)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig('gps_northing_easting.png', dpi=150)
    plt.show()

    # ---- Altitude vs Time (UNCHANGED) ----
    plt.figure(figsize=(8, 5))
    plt.plot(df_clut['timestamp'] - df_clut['timestamp'].iloc[0], df_clut['alt'], label='Cluttered')
    plt.plot(df_open['timestamp'] - df_open['timestamp'].iloc[0], df_open['alt'], label='Open')
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.title('Altitude vs Time')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig('gps_altitude_vs_time.png', dpi=150)
    plt.show()

    # =====================================================
    # Separate Euclidean Error Histograms
    # =====================================================

    bins = 40

    # Cluttered
    plt.figure(figsize=(8, 5))
    plt.hist(clut_errors, bins=bins, density=True, alpha=0.7)
    plt.xlabel('Euclidean Distance from Centroid (m)')
    plt.ylabel('Probability Density')
    plt.title('Cluttered Euclidean Error Histogram')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('gps_cluttered_euclidean_error_hist.png', dpi=150)
    plt.show()

    # Open
    plt.figure(figsize=(8, 5))
    plt.hist(open_errors, bins=bins, density=True, alpha=0.7)
    plt.xlabel('Euclidean Distance from Centroid (m)')
    plt.ylabel('Probability Density')
    plt.title('Open Euclidean Error Histogram')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('gps_open_euclidean_error_hist.png', dpi=150)
    plt.show()


def main():
    cluttered_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data", "rtk", "occluded", "occludedRTK_0.db3")
    open_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data", "rtk", "open", "openRTK_0.db3")

    print("Extracting Cluttered GPS data...")
    df_clut = extract_gps_data(cluttered_file)

    print("Extracting Open GPS data...")
    df_open = extract_gps_data(open_file)

    plot_data(df_clut, df_open)


if __name__ == "__main__":
    main()