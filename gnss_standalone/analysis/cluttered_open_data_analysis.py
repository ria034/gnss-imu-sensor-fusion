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
    
    # Find topic_id for /gps
    cursor.execute("SELECT id FROM topics WHERE name = '/gps'")
    topic_id = cursor.fetchone()[0]
    
    # Get messages
    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (topic_id,)
    )
    messages = cursor.fetchall()
    conn.close()
    
    # Detect GPS field offset using first message
    timestamp, data = messages[0]
    field_offset = None
    for offset in range(0, len(data) - 48):
        try:
            lat = struct.unpack('<d', data[offset:offset+8])[0]
            lon = struct.unpack('<d', data[offset+8:offset+16])[0]
            alt = struct.unpack('<d', data[offset+16:offset+24])[0]
            utm_e = struct.unpack('<d', data[offset+32:offset+40])[0]
            utm_n = struct.unpack('<d', data[offset+40:offset+48])[0]
            if (-90 <= lat <= 90 and -180 <= lon <= 180 and
                100000 <= utm_e <= 900000):
                field_offset = offset
                break
        except:
            continue

    if field_offset is None:
        raise ValueError("Could not detect GPS field offset.")
    
    # Extract all GPS points
    gps_data = []
    for timestamp, data in messages:
        try:
            alt = struct.unpack('<d', data[field_offset+16:field_offset+24])[0]
            utm_e = struct.unpack('<d', data[field_offset+32:field_offset+40])[0]
            utm_n = struct.unpack('<d', data[field_offset+40:field_offset+48])[0]
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
    print(df.head())
    return df


def calculate_errors(df, ref_e, ref_n):
    """Calculate 2D Euclidean errors from reference position."""
    errors = np.sqrt(
        (df['utm_e'] - ref_e) ** 2 +
        (df['utm_n'] - ref_n) ** 2
    )

    mean_error = errors.mean()
    median_error = np.median(errors)
    error_95 = np.percentile(errors, 95)
    
    print(f"Reference position: Easting={ref_e:.3f}, Northing={ref_n:.3f}")
    print(f"Mean error:   {mean_error:.3f} m")
    print(f"Median error: {median_error:.3f} m")
    print(f"95% error:    {error_95:.3f} m\n")
    
    return errors, mean_error


def plot_data(df_clut, df_open):
    """Plot trajectories and altitude for both datasets."""
    t_clut = df_clut['timestamp'].to_numpy() - df_clut['timestamp'].iloc[0]
    t_open = df_open['timestamp'].to_numpy() - df_open['timestamp'].iloc[0]
    
    # Relative positions
    df_clut_rel = df_clut.copy()
    df_clut_rel['e_rel'] = df_clut['utm_e'] - df_clut['utm_e'].iloc[0]
    df_clut_rel['n_rel'] = df_clut['utm_n'] - df_clut['utm_n'].iloc[0]
    
    df_open_rel = df_open.copy()
    df_open_rel['e_rel'] = df_open['utm_e'] - df_open['utm_e'].iloc[0]
    df_open_rel['n_rel'] = df_open['utm_n'] - df_open['utm_n'].iloc[0]
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Trajectories
    ax1.scatter(df_clut_rel['e_rel'], df_clut_rel['n_rel'],
                s=30, alpha=0.6, label='Cluttered', color='#FF6B6B')
    ax1.scatter(df_open_rel['e_rel'], df_open_rel['n_rel'],
                s=30, alpha=0.6, label='Non-Cluttered', color='#4ECDC4')
    ax1.set_xlabel('Relative UTM Easting (m)')
    ax1.set_ylabel('Relative UTM Northing (m)')
    ax1.set_title('GPS Trajectory Comparison')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Altitude
    ax2.scatter(t_clut, df_clut['alt'], label='Cluttered', color='#FF6B6B')
    ax2.scatter(t_open, df_open['alt'], label='Non-Cluttered', color='#4ECDC4')
    ax2.set_xlabel('Relative Time (s)')
    ax2.set_ylabel('Altitude (m)')
    ax2.set_title('Altitude vs Time Comparison')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('gps_trajectories.png', dpi=150)
    print("Plot saved as 'gps_trajectories.png'")
    plt.show()


def plot_error_histograms_separate(errors_clut, errors_open):
    """Plot separate histograms for cluttered and non-cluttered errors."""

    # Cluttered histogram
    plt.figure(figsize=(7,5))
    bins_clut = np.linspace(0, errors_clut.max(), 40)
    plt.hist(errors_clut, bins=bins_clut, color='#FF6B6B', edgecolor='black', alpha=0.7)
    plt.xlabel('2D Position Error (m)')
    plt.ylabel('Number of Samples')
    plt.title('Cluttered GPS Error Histogram')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('gps_error_histogram_cluttered.png', dpi=150)
    print("Cluttered histogram saved as 'gps_error_histogram_cluttered.png'")
    plt.show()

    # Non-Cluttered histogram (all points)
    plt.figure(figsize=(7,5))
    bins_open = np.linspace(0, errors_open.max(), len(errors_open)//2)  # very fine bins
    plt.hist(errors_open, bins=bins_open, color='#4ECDC4', edgecolor='black', alpha=0.7)
    plt.xlabel('2D Position Error (m)')
    plt.ylabel('Number of Samples')
    plt.title('Non-Cluttered GPS Error Histogram (All Points)')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('gps_error_histogram_noncluttered.png', dpi=150)
    print("Non-Cluttered histogram saved as 'gps_error_histogram_noncluttered.png'")
    plt.show()


def main():
    cluttered_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "cluttered", "open_hraday_0.db3")
    non_cluttered_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "non_cluttered", "open_ria_mendes_0.db3")
    
    ref_clut = (328330.0, 4689545.0)
    ref_open = (327984.1488, 4689511.170)
    
    print("Extracting Cluttered GPS data...")
    df_clut = extract_gps_data(cluttered_file)
    
    print("Extracting Non-Cluttered GPS data...")
    df_open = extract_gps_data(non_cluttered_file)
    
    print("\nCluttered Errors (known reference):")
    errors_clut, _ = calculate_errors(df_clut, *ref_clut)
    
    print("Non-Cluttered Errors (known reference):")
    errors_open, _ = calculate_errors(df_open, *ref_open)
    
    plot_data(df_clut, df_open)
    plot_error_histograms_separate(errors_clut, errors_open)


if __name__ == "__main__":
    main()
