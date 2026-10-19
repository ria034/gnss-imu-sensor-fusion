import os
import allantools
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

    # ── Estimate sample rate from timestamps ─────────────────────
    if len(messages) > 200:
        timestamps = [m[0] for m in messages[:200]]
        dt = np.diff(timestamps) / 1e9  # nanoseconds → seconds
        estimated_fs = 1.0 / np.mean(dt)
        print(f"\nEstimated sample rate from timestamps: {estimated_fs:.1f} Hz")

    conn.close()
    print(f"Total messages: {len(messages)}")

    # ── Parse ASCII $VNYMR sentences ─────────────────────────────
    acceleration_data, gyro_data = [], []
    skipped = 0

    for timestamp, raw in messages:
        try:
            text  = raw.decode('ascii', errors='ignore')
            match = re.search(r'\$VNYMR,([^\*\r\n]+)', text)
            if not match:
                skipped += 1
                continue

            fields = match.group(1).split(',')
            # $VNYMR: yaw, pitch, roll, magX, magY, magZ, aX, aY, aZ, gX, gY, gZ
            if len(fields) < 12:
                skipped += 1
                continue

            ax, ay, az = float(fields[6]),  float(fields[7]),  float(fields[8])
            gx, gy, gz = float(fields[9]),  float(fields[10]), float(fields[11].split('*')[0])

            if any(not np.isfinite(v) for v in [ax, ay, az, gx, gy, gz]):
                skipped += 1
                continue

            acceleration_data.append({"acceleration_x": ax,
                                       "acceleration_y": ay,
                                       "acceleration_z": az})
            gyro_data.append({"gyro_x": gx,
                               "gyro_y": gy,
                               "gyro_z": gz})

        except Exception:
            skipped += 1

    df_acc  = pd.DataFrame(acceleration_data)
    df_gyro = pd.DataFrame(gyro_data)

    print(f"\nExtracted : {len(df_gyro)} samples  |  Skipped : {skipped}")

    if df_gyro.empty:
        raise ValueError("No data extracted — check topic name or message format.")

    # ── Sanity check ─────────────────────────────────────────────
    print("\n--- GYRO STATISTICS ---")
    print(df_gyro.describe().round(6).to_string())
    print("\n--- ACCEL STATISTICS ---")
    print(df_acc.describe().round(4).to_string())

    # ── Std ratio check (flag large Z asymmetry) ─────────────────
    rx = df_gyro['gyro_z'].std() / df_gyro['gyro_x'].std()
    ry = df_gyro['gyro_z'].std() / df_gyro['gyro_y'].std()
    print(f"\nGyro std ratio  Z/X = {rx:.2f}  |  Z/Y = {ry:.2f}")
    if rx > 10 or ry > 10:
        print("  WARNING: Z-axis std is much larger than X/Y — "
              "check raw data for spikes or offsets on Z channel.")

    return df_gyro, df_acc


# =============================================================
# STEP 2: NOISE PARAMETERS  (slope-fitting method, MATLAB-equivalent)
# =============================================================
#
#   N  — Angle Random Walk       slope = -0.5   read at τ = 1
#   K  — Rate  Random Walk       slope = +0.5   read at τ = 3
#   B  — Bias  Instability       slope =  0     scaled by sqrt(2·ln2/π) ≈ 0.664
#
# All three use line-fitting on the log-log Allan deviation curve,
# then the coefficient is evaluated at the canonical tau for that noise type.

def extract_noise_parameters(taus, adev):
    taus    = np.array(taus)
    adev    = np.array(adev)
    logtau  = np.log10(taus)
    logadev = np.log10(adev)
    slopes  = np.diff(logadev) / np.diff(logtau)

    # ── Angle Random Walk (N) ────────────────────────────────────
    # Slope = -0.5 region; read line at τ = 1
    i_N  = np.argmin(np.abs(slopes - (-0.5)))
    b_N  = logadev[i_N] - (-0.5) * logtau[i_N]
    logN = (-0.5) * np.log10(1.0) + b_N   # log10(1) = 0, so logN = b_N
    N    = 10 ** logN

    # ── Rate Random Walk (K) ─────────────────────────────────────
    # Slope = +0.5 region; read line at τ = 3
    i_K  = np.argmin(np.abs(slopes - 0.5))
    b_K  = logadev[i_K] - 0.5 * logtau[i_K]
    logK = 0.5 * np.log10(3.0) + b_K
    K    = 10 ** logK

    # ── Bias Instability (B) ─────────────────────────────────────
    # Slope = 0 region; scaling factor 0.664 applied
    i_B  = np.argmin(np.abs(slopes - 0.0))
    b_B  = logadev[i_B]                         # slope = 0 → b = logadev[i]
    scfB = np.sqrt(2.0 * np.log(2.0) / np.pi)   # ≈ 0.6627
    logB = b_B - np.log10(scfB)
    B    = 10 ** logB
    tau_B = taus[i_B]

    return N, B, K, tau_B, scfB, b_N, b_K


# =============================================================
# STEP 3: PLOT  (fitted lines + annotated markers, MATLAB-style)
# =============================================================

def plot_allan_axis(ax_plot, taus, adev, N, B, K, tau_B, scfB,
                    b_N, b_K, axis_label, color):

    logtau = np.log10(taus)

    # Fitted lines over the full tau range
    lineN = 10 ** ((-0.5) * logtau + b_N)
    lineK = 10 ** (  0.5  * logtau + b_K)
    lineB = np.full_like(taus, scfB * B)

    ax_plot.loglog(taus, adev,  color=color, linewidth=2,   label='Allan Dev')
    ax_plot.loglog(taus, lineN, 'r--',       linewidth=1.5, label=f'N = {N:.3e}')
    ax_plot.loglog(taus, lineK, 'm--',       linewidth=1.5, label=f'K = {K:.3e}')
    ax_plot.loglog(taus, lineB, 'g--',       linewidth=1.5, label=f'0.664B = {scfB*B:.3e}')

    # Canonical marker points
    ax_plot.plot(1.0,   N,        'ro', markersize=8)
    ax_plot.plot(3.0,   K,        'mo', markersize=8)
    ax_plot.plot(tau_B, scfB * B, 'go', markersize=8)

    ax_plot.annotate('N',      xy=(1.0,   N),        fontsize=9, color='red')
    ax_plot.annotate('K',      xy=(3.0,   K),        fontsize=9, color='purple')
    ax_plot.annotate('0.664B', xy=(tau_B, scfB * B), fontsize=9, color='green')

    ax_plot.set_title(axis_label, fontsize=11)
    ax_plot.set_xlabel('τ (s)')
    ax_plot.set_ylabel('Allan Deviation')
    ax_plot.legend(fontsize=7)
    ax_plot.grid(True, which='both', alpha=0.3)


# =============================================================
# STEP 4: ANALYSIS
# =============================================================

def allan_variance_analysis(df_gyro, df_acceleration):

    Fs         = 100    # ← verify against your VectorNav config / estimated_fs above
    gyro_axes  = ['gyro_x',         'gyro_y',         'gyro_z']
    accel_axes = ['acceleration_x', 'acceleration_y', 'acceleration_z']
    colors     = ['steelblue', 'tomato', 'seagreen']

    def run_axis(df, axes, fig_title, label, filename):
        fig, axs = plt.subplots(1, 3, figsize=(18, 5))
        fig.suptitle(fig_title, fontsize=14, fontweight='bold')

        print(f"\n{'='*65}")
        print(f"  {label} NOISE PARAMETERS")
        print(f"{'='*65}")
        print(f"  {'Axis':<18} {'N (ARW)':<18} {'B (Bias Stab)':<18} {'K (RRW)'}")
        print(f"  {'-'*65}")

        params = {}
        for ax_plot, col, color in zip(axs, axes, colors):
            data = df[col].dropna().values

            taus, adev, _, _ = allantools.oadev(
                data, rate=Fs, data_type="freq", taus='octave'
            )

            print(f"\n  [{col}]  tau range: {taus[0]:.3f}s – {taus[-1]:.1f}s  "
                  f"|  adev range: {adev.min():.3e} – {adev.max():.3e}")

            # Warn if tau range is too short for reliable K or B
            if taus[-1] < 3.0:
                print(f"  WARNING: max tau ({taus[-1]:.1f}s) < 3s — "
                      f"K estimate may be unreliable for {col}")
            if taus[-1] < 100.0:
                print(f"  WARNING: max tau ({taus[-1]:.1f}s) < 100s — "
                      f"B estimate may be unreliable for {col}")

            N, B, K, tau_B, scfB, b_N, b_K = extract_noise_parameters(taus, adev)
            params[col] = {"N": N, "B": B, "K": K}

            plot_allan_axis(ax_plot, taus, adev, N, B, K, tau_B, scfB,
                            b_N, b_K, col.upper(), color)

            print(f"  {col:<18} N={N:.4e}    B={B:.4e}    K={K:.4e}")

        plt.tight_layout()
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"\n  Saved: {filename}")
        plt.show()
        return params

    gyro_params  = run_axis(df_gyro,         gyro_axes,
                            'Allan Deviation — Gyroscope (rad/s)',
                            'GYROSCOPE',
                            'allan_gyroscope.png')

    accel_params = run_axis(df_acceleration, accel_axes,
                            'Allan Deviation — Accelerometer (m/s²)',
                            'ACCELEROMETER',
                            'allan_accelerometer.png')

    # ── Final summary table ───────────────────────────────────────
    print(f"\n{'='*65}")
    print("  FINAL SUMMARY")
    print(f"{'='*65}")
    print(f"  {'Sensor':<18} {'Axis':<12} {'N (ARW)':<16} {'B (Bias)':<16} {'K (RRW)'}")
    print(f"  {'-'*65}")
    for col, p in gyro_params.items():
        print(f"  {'Gyro':<18} {col:<12} {p['N']:.4e}      {p['B']:.4e}      {p['K']:.4e}")
    for col, p in accel_params.items():
        print(f"  {'Accel':<18} {col:<12} {p['N']:.4e}      {p['B']:.4e}      {p['K']:.4e}")

    return gyro_params, accel_params


# =============================================================
# MAIN
# =============================================================

if __name__ == "__main__":

    db3_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "five_hour_imu_data", "5_hour_imu_data_0.db3")

    df_gyro, df_acceleration = extract_data(db3_file)
    gyro_params, accel_params = allan_variance_analysis(df_gyro, df_acceleration)
