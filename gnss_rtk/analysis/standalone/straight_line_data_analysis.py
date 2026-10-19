#!/usr/bin/env python3
import os
import sqlite3
import pandas as pd
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
    for base_offset in range(0, len(data) - 24, 1):
        try:
            lat = struct.unpack('<d', data[base_offset:base_offset+8])[0]
            lon = struct.unpack('<d', data[base_offset+8:base_offset+16])[0]
            alt = struct.unpack('<d', data[base_offset+16:base_offset+24])[0]
            hdop = struct.unpack('<d', data[base_offset+24:base_offset+32])[0]
            utm_e = struct.unpack('<d', data[base_offset+32:base_offset+40])[0]
            utm_n = struct.unpack('<d', data[base_offset+40:base_offset+48])[0]
            
            if (-90 <= lat <= 90 and -180 <= lon <= 180 and 0 < hdop < 100 and 
                100000 <= utm_e <= 900000 and 0 <= utm_n <= 10000000):
                field_offsets = {
                    'lat': base_offset, 'lon': base_offset + 8, 'alt': base_offset + 16,
                    'hdop': base_offset + 24, 'utm_e': base_offset + 32, 'utm_n': base_offset + 40
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
            
            # Validate the data
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
    
    # Make time relative (starting from 0)
    df['time_relative'] = df['timestamp'] - df['timestamp'].iloc[0]
    
    # Make UTM coordinates relative (starting from 0, 0)
    df['utm_easting_relative'] = df['utm_easting'] - df['utm_easting'].iloc[0]
    df['utm_northing_relative'] = df['utm_northing'] - df['utm_northing'].iloc[0]
    
    return df

def calculate_point_errors(df):
    """Calculate per-point distance from GPS points to the best-fit line and average error"""
    if df is None or len(df) < 2:
        print("Not enough data points to fit a line.")
        return None

    df = make_relative(df)

    x = df['utm_easting_relative'].values
    y = df['utm_northing_relative'].values

    # Fit line y = m*x + b
    z = np.polyfit(x, y, 1)
    m, b = z[0], z[1]

    # Distance from each point to line: |mx - y + b| / sqrt(m^2 + 1)
    distances = np.abs(m*x - y + b) / np.sqrt(m**2 + 1)
    df['error_to_line'] = distances

    # Compute average (mean) and max error
    mean_error = distances.mean()
    max_error = distances.max()

    print(f"\nBest-fit line slope: {m:.6f}, intercept: {b:.6f}")
    print(f"Mean error to line: {mean_error:.6f} m")
    print(f"Max error to line: {max_error:.6f} m")

    return df, {'slope': m, 'intercept': b, 'mean_error': mean_error, 'max_error': max_error}

def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <file.db3>")
        print("\nUsing default file...")
        bag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data", "standalone", "straight_line", "straight_line_ria_mendes_0.db3")
    else:
        bag_file = sys.argv[1]
    
    print(f"\n{'='*60}")
    print(f"Processing File: {bag_file}")
    print(f"{'='*60}")
    
    df = extract_gps_data(bag_file)
    
    if df is not None:
        # Save CSV with raw GPS
        df.to_csv('gps_data.csv', index=False)
        print("\nData saved to 'gps_data.csv'")

        # Calculate per-point errors relative to best-fit line
        df_with_error, bestfit_info = calculate_point_errors(df)

        # Save CSV with per-point errors
        df_with_error.to_csv('gps_data_with_error.csv', index=False)
        print("\nData with per-point errors saved to 'gps_data_with_error.csv'")
    else:
        print("\nError: Could not extract data from file")

if __name__ == "__main__":
    main()