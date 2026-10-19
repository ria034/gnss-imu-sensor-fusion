import numpy as np
import matplotlib.pyplot as plt
import sqlite3
import pandas as pd
import re

# =============================================================
# STEP 1: EXTRACT DATA — VectorNav ASCII $VNYMR parser
# =============================================================
# $VNYMR format:
# $VNYMR,yaw,pitch,roll,magX,magY,magZ,accelX,accelY,accelZ,gyroX,gyroY,gyroZ*checksum

def extract_data(db3_file):

    conn   = sqlite3.connect(db3_file)
    cursor = conn.cursor()

    cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
    tables         = [t[0] for t in cursor.fetchall()]
    topics_table   = next((n for n in tables if 'topic'   in n.lower()), None)
    messages_table = next((n for n in tables if 'message' in n.lower()), None)

    cursor.execute(f"SELECT * FROM {topics_table}")
    print("Available topics:")
    for t in cursor.fetchall(): print("  ", t)

    topic_id = None
    for topic_name in ['/imu/data', '/imu', '/imu/data_raw', '/sensor/imu']:
        cursor.execute(f"SELECT id FROM {topics_table} WHERE name=?", (topic_name,))
        r = cursor.fetchone()
        if r:
            topic_id = r[0]
            print(f"\nFound IMU topic: '{topic_name}'  id={topic_id}")
            break

    if topic_id is None:
        conn.close()
        raise ValueError("IMU topic not found.")

    cursor.execute(
        f"SELECT timestamp, data FROM {messages_table} "
        f"WHERE topic_id=? ORDER BY timestamp", (topic_id,)
    )
    messages = cursor.fetchall()
    if len(messages) > 200:
        timestamps = [m[0] for m in messages[:200]]
        dt = np.diff(timestamps) / 1e9  # nanoseconds → seconds
        estimated_fs = 1.0 / np.mean(dt)
        print(f"\nEstimated sample rate from timestamps: {estimated_fs:.1f} Hz")

    conn.close()
    print(f"Total messages: {len(messages)}")

    # ── Parse ASCII $VNYMR sentences ─────────────────────────────
    magnetic_data=  [], []
    skipped = 0

    for timestamp, raw in messages:
        try:
            text  = raw.decode('ascii', errors='ignore')
            match = re.search(r'\$VNYMR,([^\*\r\n]+)', text)
            if not match:
                skipped += 1
                continue

            fields = match.group(1).split(',')
            if len(fields) < 12:
                skipped += 1
                continue

            mx, my, mz = float(fields[3]),  float(fields[4]),  float(fields[5])
            

            if any(not np.isfinite(v) for v in [mx,my,mz]):
                skipped += 1
                continue

            magnetic_data.append({"magnetic_x": mx,
                                       "magnetic_y": my,
                                       "magnetic_z": mz})
            

        except Exception:
            skipped += 1

    df_mag  = pd.DataFrame(magnetic_data)


  
