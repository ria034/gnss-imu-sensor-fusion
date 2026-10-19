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
    
    # Get topic ID
    cursor.execute("SELECT id FROM topics WHERE name = ?", (topic_name,))
    result = cursor.fetchone()
    if not result:
        print(f"Topic '{topic_name}' not found!")
        conn.close()
        return None
    
    # Get messages
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (result[0],))
    messages = cursor.fetchall()
    conn.close()
    
    print(f"Processing {len(messages)} GPS messages...")
    
    # Find GPS data pattern in first message
    timestamp, data = messages[0]
    field_offsets = None
    for base_offset in range(0, len(data) - 48):
        try:
            lat = struct.unpack('<d', data[base_offset:base_offset+8])[0]
            lon = struct.unpack('<d', data[base_offset+8:base_offset+16])[0]
            alt = struct.unpack('<d', data[base_offset+16:base_offset+24])[0]
            utm_e = struct.unpack('<d', data[base_offset+32:base_offset+40])[0]
            utm_n = struct.unpack('<d', data[base_offset+40:base_offset+48])[0]
            
            if (-90 <= lat <= 90 and -180 <= lon <= 180 and 
                100000 <= utm_e <= 900000 and 0 <= utm_n <= 10000000):
                field_offsets = {
                    'lat': base_offset, 'lon': base_offset + 8, 'alt': base_offset + 16,
                    'utm_e': base_offset + 32, 'utm_n': base_offset + 40
                }
                break
        except:
            continue
    
    if field_offsets is None:
        print("Could not find GPS pattern!")
        return None
    
    # Extract all data
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
                100000 <= utm_e <= 900000 and 0 <= utm_n <= 10000000):
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
    if len(df) > 0:
        print(f"  Time range: {df['timestamp'].min():.2f} to {df['timestamp'].max():.2f}")
        print(f"  UTM Easting range: {df['utm_easting'].min():.2f} to {df['utm_easting'].max():.2f}")
        print(f"  UTM Northing range: {df['utm_northing'].min():.2f} to {df['utm_northing'].max():.2f}")
        print(f"  Altitude range: {df['altitude'].min():.2f} to {df['altitude'].max():.2f}")
    return df

def make_relative(df):
    """Convert coordinates to relative values (starting from 0)"""
    if df is None or len(df) == 0:
        return df
    
    df = df.copy()
    df['time_relative'] = df['timestamp'] - df['timestamp'].iloc[0]
    df['utm_easting_relative'] = df['utm_easting'] - df['utm_easting'].iloc[0]
    df['utm_northing_relative'] = df['utm_northing'] - df['utm_northing'].iloc[0]
    return df

def compute_best_fit_error(df):
    """Compute perpendicular distance from best-fit line and mean error"""
    if df is None or len(df) < 2:
        return None, None, None
    
    x = df['utm_easting_relative'].values
    y = df['utm_northing_relative'].values
    
    # Linear fit y = m*x + b
    m, b = np.polyfit(x, y, 1)
    
    # Perpendicular distance from each point to line: |mx - y + b| / sqrt(m^2 + 1)
    errors = np.abs(m*x - y + b) / np.sqrt(m**2 + 1)
    mean_error = errors.mean()
    
    print(f"\nBest fit line: y = {m:.5f} * x + {b:.5f}")
    print(f"Mean perpendicular error from best-fit line: {mean_error:.3f} m")
    
    return m, b, errors

def plot_gps_with_fit(df, output_file='gps_plot.png'):
    """Create GPS plot with best-fit line and errors"""
    if df is None or len(df) == 0:
        print("No data to plot!")
        return
    
    df = make_relative(df)
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    # Plot UTM trajectory
    x = df['utm_easting_relative'].values
    y = df['utm_northing_relative'].values
    ax1.scatter(x, y, s=30, alpha=0.6, color='#4ECDC4', marker='o', label='GPS Data')
    
    # Best fit line
    m, b, errors = compute_best_fit_error(df)
    if m is not None:
        x_line = np.linspace(x.min(), x.max(), 100)
        y_line = m * x_line + b
        ax1.plot(x_line, y_line, '--', color='#FF6B6B', linewidth=2, label='Best Fit Line')
    
    ax1.set_xlabel('Relative UTM Easting (m)')
    ax1.set_ylabel('Relative UTM Northing (m)')
    ax1.set_title('GPS Trajectory with Best Fit Line')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Plot Altitude vs time
    ax2.plot(df['time_relative'].values, df['altitude'].values, '-', color='#4ECDC4', linewidth=1.5)
    ax2.set_xlabel('Relative Time (s)')
    ax2.set_ylabel('Altitude (m)')
    ax2.set_title('Altitude vs Time')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150)
    print(f"Plot saved as '{output_file}'")
    plt.show()
    
    return errors

def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <file.db3>")
        print("\nUsing default file...")
        bag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "straight_line", "straight_line_ria_mendes_0.db3")
    else:
        bag_file = sys.argv[1]
    
    print(f"\n{'='*60}")
    print(f"Processing File: {bag_file}")
    print(f"{'='*60}")
    
    df = extract_gps_data(bag_file)
    
    if df is not None:
        df.to_csv('gps_data.csv', index=False)
        print("Data saved to 'gps_data.csv'")
        
        # Plot GPS with best fit
        errors = plot_gps_with_fit(df)
        
        if errors is not None:
            # Optional: histogram of errors
            plt.figure(figsize=(7,5))
            plt.hist(errors, bins=30, color='#FF6B6B', edgecolor='black', alpha=0.7)
            plt.xlabel('Perpendicular Error to Best Fit Line (m)')
            plt.ylabel('Number of Points')
            plt.title('Histogram of GPS Errors from Best Fit')
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.savefig('gps_best_fit_error_histogram.png', dpi=150)
            print("Histogram saved as 'gps_best_fit_error_histogram.png'")
            plt.show()
        
    else:
        print("Error: Could not extract data from file")

if __name__ == "__main__":
    main()
