#!/usr/bin/env python3
import os
import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import struct
import sys

def extract_gps_data(db3_file, topic_name='/gps'):
    """Extract GPS data from ROS2 bag file"""
    conn = sqlite3.connect(db3_file)
    cursor = conn.cursor()
    
    cursor.execute("SELECT id FROM topics WHERE name = ?", (topic_name,))
    result = cursor.fetchone()
    if not result:
        print(f"Topic '{topic_name}' not found!")
        conn.close()
        return None
    
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (result[0],))
    messages = cursor.fetchall()
    conn.close()
    
    print(f"Processing {len(messages)} GPS messages...")
    
    timestamp, data = messages[0]
    field_offsets = None
    
    for base_offset in range(0, len(data) - 48, 1):
        try:
            lat = struct.unpack('<d', data[base_offset:base_offset+8])[0]
            lon = struct.unpack('<d', data[base_offset+8:base_offset+16])[0]
            alt = struct.unpack('<d', data[base_offset+16:base_offset+24])[0]
            hdop = struct.unpack('<d', data[base_offset+24:base_offset+32])[0]
            utm_e = struct.unpack('<d', data[base_offset+32:base_offset+40])[0]
            utm_n = struct.unpack('<d', data[base_offset+40:base_offset+48])[0]
            
            if (-90 <= lat <= 90 and -180 <= lon <= 180 and 
                0 < hdop < 100 and
                100000 <= utm_e <= 900000 and 
                0 <= utm_n <= 10000000):
                
                field_offsets = {
                    'lat': base_offset,
                    'lon': base_offset + 8,
                    'alt': base_offset + 16,
                    'utm_e': base_offset + 32,
                    'utm_n': base_offset + 40
                }
                break
        except:
            continue
    
    if field_offsets is None:
        print("Could not find GPS pattern!")
        return None
    
    gps_data = []
    failed_count = 0
    
    for timestamp, data in messages:
        try:
            lat = struct.unpack('<d', data[field_offsets['lat']:field_offsets['lat']+8])[0]
            lon = struct.unpack('<d', data[field_offsets['lon']:field_offsets['lon']+8])[0]
            alt = struct.unpack('<d', data[field_offsets['alt']:field_offsets['alt']+8])[0]
            utm_e = struct.unpack('<d', data[field_offsets['utm_e']:field_offsets['utm_e']+8])[0]
            utm_n = struct.unpack('<d', data[field_offsets['utm_n']:field_offsets['utm_n']+8])[0]
            
            if (-90 <= lat <= 90 and -180 <= lon <= 180 and
                100000 <= utm_e <= 900000 and
                0 <= utm_n <= 10000000):
                
                gps_data.append({
                    'timestamp': timestamp / 1e9,
                    'altitude': alt,
                    'utm_easting': utm_e,
                    'utm_northing': utm_n
                })
            else:
                failed_count += 1
        except:
            failed_count += 1
    
    df = pd.DataFrame(gps_data)
    print(f"Extracted {len(df)} GPS points (failed: {failed_count})")
    return df


def make_relative(df):
    """Convert coordinates to relative values"""
    if df is None or len(df) == 0:
        return df
    
    df = df.copy()
    df['time_relative'] = df['timestamp'] - df['timestamp'].iloc[0]
    df['utm_easting_relative'] = df['utm_easting'] - df['utm_easting'].iloc[0]
    df['utm_northing_relative'] = df['utm_northing'] - df['utm_northing'].iloc[0]
    
    return df


def plot_gps(df, output_file='gps_plot.png'):
    """Create plots and compute line-of-best-fit error"""
    if df is None or len(df) == 0:
        print("No data to plot!")
        return
    
    df = make_relative(df)
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    # ---- Scatter Plot ----
    ax1.scatter(df['utm_easting_relative'].values,
                df['utm_northing_relative'].values,
                s=30, alpha=0.6, color='#4ECDC4',
                marker='o', label='GPS Data')
    
    if len(df) > 1:
        x = df['utm_easting_relative'].values
        y = df['utm_northing_relative'].values
        
        # Linear regression
        z = np.polyfit(x, y, 1)
        m, c = z
        p = np.poly1d(z)
        
        # Plot best fit line
        x_line = np.linspace(x.min(), x.max(), 100)
        ax1.plot(x_line, p(x_line), '--',
                 linewidth=2, color='#FF6B6B',
                 label=f'Best fit: y={m:.3f}x+{c:.3f}', alpha=0.8)
        
        # ---- Perpendicular distance error ----
        errors = np.abs(m*x - y + c) / np.sqrt(m**2 + 1)
        
        print("\nLine of Best Fit Error Statistics:")
        print(f"  Mean Error = {np.mean(errors):.4f} m")
        print(f"  RMS Error  = {np.sqrt(np.mean(errors**2)):.4f} m")
        print(f"  Max Error  = {np.max(errors):.4f} m")
        
        # ---- Histogram ----
        plt.figure(figsize=(6, 4))
        plt.hist(errors, bins=40, density=True, alpha=0.7, color='#FF6B6B')
        plt.xlabel("Perpendicular Distance to Best Fit Line (m)")
        plt.ylabel("Probability Density")
        plt.title("Line-of-Best-Fit Error Histogram")
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig("line_fit_error_histogram.png", dpi=150)
        plt.show()
    
    ax1.set_xlabel('Relative UTM Easting (m)')
    ax1.set_ylabel('Relative UTM Northing (m)')
    ax1.set_title('GPS Trajectory')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.axis('equal')
    
    # ---- Altitude vs Time ----
    ax2.plot(df['time_relative'].values,
             df['altitude'].values,
             '-', linewidth=1.5, color='#4ECDC4')
    
    ax2.set_xlabel('Relative Time (s)')
    ax2.set_ylabel('Altitude (m)')
    ax2.set_title('Altitude vs Time')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Plot saved as '{output_file}'")
    plt.show()


def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <file.db3>")
        print("\nUsing default file...")
        bag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data", "rtk", "walking", "walkingRTK_0.db3")
    else:
        bag_file = sys.argv[1]
    
    print("\n" + "="*60)
    print(f"Processing File: {bag_file}")
    print("="*60)
    
    df = extract_gps_data(bag_file)
    
    if df is not None:
        df.to_csv('gps_data.csv', index=False)
        print("\nData saved to 'gps_data.csv'")
        plot_gps(df)
    else:
        print("\nError: Could not extract data from file")


if __name__ == "__main__":
    main()