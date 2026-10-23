#!/usr/bin/env python3
"""
Adaptive Net: Data Analysis & Visualization Suite (8-Path Architecture)
Reads ONLY empirical measured simulation CSV outputs (Rule R3).
Computes mean and 95% confidence intervals across seeds.
Generates:
  1. metric_vs_time.png: True per-second latency, throughput, and active router path vs. time.
  2. bar_charts_vs_baselines.png: Measured QoS performance across Adaptive, Baseline 1, and Baseline 2.
  3. path_timeline.png: Active router path timeline per class derived from decisions.csv.
  4. weight_sensitivity.png: Multi-criteria cost contours across delay and bandwidth for all 3 classes.
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.patches import Patch

# Configure publication-quality aesthetic
plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
plt.rcParams['font.sans-serif'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.labelsize'] = 11
plt.rcParams['axes.titlesize'] = 12
plt.rcParams['xtick.labelsize'] = 9
plt.rcParams['ytick.labelsize'] = 9
plt.rcParams['legend.fontsize'] = 9
plt.rcParams['figure.titlesize'] = 13

WORKSPACE_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(WORKSPACE_DIR, "results")
PLOTS_DIR = os.path.join(WORKSPACE_DIR, "plots")
os.makedirs(PLOTS_DIR, exist_ok=True)

PATH_NAMES = [f"Path_{i}" for i in range(8)]
ROUTER_LABELS = [
    "R0 (100M/5ms)", "R1 (50M/10ms)", "R2 (25M/15ms)", "R3 (15M/25ms)",
    "R4 (10M/35ms)", "R5 (5M/50ms)", "R6 (2M/70ms)", "R7 (1M/100ms)"
]
PATH_COLORS = [
    '#1f77b4', '#aec7e8', '#ff7f0e', '#ffbb78',
    '#2ca02c', '#98df8a', '#d62728', '#9467bd'
]

def find_file(filename, fallback_dir=RESULTS_DIR):
    """Finds a metric file in RESULTS_DIR or WORKSPACE_DIR."""
    p1 = os.path.join(RESULTS_DIR, filename)
    if os.path.exists(p1) and os.path.getsize(p1) > 0:
        return p1
    p2 = os.path.join(WORKSPACE_DIR, filename)
    if os.path.exists(p2) and os.path.getsize(p2) > 0:
        return p2
    return None

def compute_ci(data):
    """Computes mean and 95% confidence interval half-width."""
    arr = np.array(data, dtype=float)
    arr = arr[~np.isnan(arr)]
    if len(arr) == 0:
        return np.nan, 0.0
    mean = np.mean(arr)
    if len(arr) < 2:
        return mean, 0.0
    se = np.std(arr, ddof=1) / np.sqrt(len(arr))
    ci = 1.96 * se
    return mean, ci

def parse_path_id(val):
    """Parses 'Path_X' or integer X to integer path index [0..7]."""
    if pd.isna(val):
        return np.nan
    s = str(val).strip()
    if s.startswith("Path_"):
        try:
            return int(s.split("_")[1])
        except (ValueError, IndexError):
            return np.nan
    try:
        return int(s)
    except ValueError:
        return np.nan

def plot_metric_vs_time():
    """Figure 1: Measured Latency, Throughput, and Active Router Path vs Time (Rule R3)."""
    ts_file = find_file("time_series_adaptive_nodefail_single.csv")
    if not ts_file:
        ts_file = find_file("time_series_adaptive_steady.csv")
    if not ts_file:
        ts_file = find_file("time_series.csv")

    if not ts_file:
        print("[Notice] time_series.csv not found yet. Run simulations first to generate Figure 1.")
        return

    df = pd.read_csv(ts_file)
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)

    # 1. Latency Subplot
    if 'EmergencyDelayMs' in df.columns:
        ax1.plot(df['Time'], df['EmergencyDelayMs'], 'r-', lw=2.2, label='Emergency Vitals (UDP 7000)')
    if 'VideoDelayMs' in df.columns:
        ax1.plot(df['Time'], df['VideoDelayMs'], 'b-', lw=1.8, label='HD Video Stream (UDP 5000)')
    ax1.axhline(y=150.0, color='darkred', linestyle='--', lw=1.5, label='Emergency QoS Deadline (150 ms)')
    ax1.set_ylabel('One-Way Delay (ms)')
    ax1.set_title('(A) Measured End-to-End Latency vs. Simulation Time', fontweight='bold')
    max_d = 180.0
    if 'EmergencyDelayMs' in df.columns and not df['EmergencyDelayMs'].isna().all():
        max_d = max(max_d, float(df['EmergencyDelayMs'].max()) + 20)
    ax1.set_ylim(0, max_d)
    ax1.legend(loc='upper right')
    ax1.grid(True, linestyle=':', alpha=0.6)

    # 2. Throughput Subplot
    if 'FileTputKbps' in df.columns:
        ax2.plot(df['Time'], df['FileTputKbps'] / 1000.0, 'g-', lw=2.0, label='Bulk File Transfer (TCP 6000)')
    if 'VideoTputKbps' in df.columns:
        ax2.plot(df['Time'], df['VideoTputKbps'] / 1000.0, 'b-', lw=1.8, label='HD Video Stream (2.5 Mbps Target)')
    if 'EmergencyTputKbps' in df.columns:
        ax2.plot(df['Time'], df['EmergencyTputKbps'] / 1000.0, 'r--', lw=1.5, label='Emergency Telemetry (64 Kbps Target)')
    ax2.set_ylabel('Throughput (Mbps)')
    ax2.set_title('(B) Measured Per-Class Throughput vs. Simulation Time', fontweight='bold')
    ax2.legend(loc='upper right')
    ax2.grid(True, linestyle=':', alpha=0.6)

    # 3. Active Router Path Subplot
    if 'ActiveEmergPath' in df.columns:
        p_emerg = df['ActiveEmergPath'].apply(parse_path_id)
        ax3.step(df['Time'], p_emerg, 'r-', where='post', lw=2.4, label='Active Path: Emergency (7000)')
    if 'ActiveVideoPath' in df.columns:
        p_video = df['ActiveVideoPath'].apply(parse_path_id)
        ax3.step(df['Time'], p_video, 'b--', where='post', lw=2.0, label='Active Path: Video (5000)')
    if 'ActiveFilePath' in df.columns:
        p_file = df['ActiveFilePath'].apply(parse_path_id)
        ax3.step(df['Time'], p_file, 'g:', where='post', lw=2.0, label='Active Path: File Transfer (6000)')

    ax3.set_xlabel('Simulation Time (seconds)')
    ax3.set_ylabel('Active Router Path')
    ax3.set_title('(C) Real-Time Path Steering across 8 Parallel Routers (R0..R7)', fontweight='bold')
    ax3.set_yticks(range(8))
    ax3.set_yticklabels(ROUTER_LABELS)
    ax3.set_ylim(-0.5, 7.5)
    ax3.legend(loc='upper right')
    ax3.grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout()
    out_path = os.path.join(PLOTS_DIR, "metric_vs_time.png")
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"[Generated] {out_path}")

def plot_bar_charts_vs_baselines():
    """Figure 2: Empirical Benchmark Comparison across Modes with 95% Confidence Intervals."""
    sum_file = find_file("summary_metrics.csv")
    if not sum_file:
        print("[Notice] summary_metrics.csv not found yet. Run simulations first to generate Figure 2.")
        return

    df = pd.read_csv(sum_file)
    # If summary_metrics contains nodefail runs, prioritize comparing under failure
    scenarios_present = df['Scenario'].unique()
    target_scenario = 'nodefail_single' if 'nodefail_single' in scenarios_present else (
        'steady' if 'steady' in scenarios_present else scenarios_present[0]
    )
    df_scen = df[df['Scenario'] == target_scenario]
    if df_scen.empty:
        df_scen = df

    modes = ['adaptive', 'shortest', 'baseline2', 'baseline1']
    labels = [
        'Adaptive\n(Class-Aware Cost)',
        'Shortest\n(Blind Re-route)',
        'Baseline 2\n(Blind Min-Delay)',
        'Baseline 1\n(Static Path R3)'
    ]
    colors = ['#2ca02c', '#9467bd', '#d62728', '#1f77b4']

    miss_means, miss_cis = [], []
    lat_means, lat_cis   = [], []
    sw_means, sw_cis     = [], []
    file_means, file_cis = [], []

    for m in modes:
        sub = df_scen[df_scen['Mode'] == m]
        if sub.empty:
            miss_means.append(np.nan); miss_cis.append(0.0)
            lat_means.append(np.nan); lat_cis.append(0.0)
            sw_means.append(np.nan); sw_cis.append(0.0)
            file_means.append(np.nan); file_cis.append(0.0)
        else:
            m_val, c_val = compute_ci(sub['DeadlineMissPct'])
            miss_means.append(m_val); miss_cis.append(c_val)

            m_val, c_val = compute_ci(sub['MeanDelayMs'])
            lat_means.append(m_val); lat_cis.append(c_val)

            m_val, c_val = compute_ci(sub['SwitchCount'])
            sw_means.append(m_val); sw_cis.append(c_val)

            m_val, c_val = compute_ci(sub['FileDurationSec'])
            file_means.append(m_val); file_cis.append(c_val)

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8.5))

    # 1. Deadline Miss Ratio
    bars1 = ax1.bar(labels, miss_means, yerr=miss_cis, color=colors, width=0.55, edgecolor='black', alpha=0.85, capsize=5)
    ax1.set_ylabel('Deadline Miss Ratio (%)')
    ax1.set_title(f'(A) Emergency Deadline Miss Ratio (<150 ms) [{target_scenario}]', fontweight='bold')
    ax1.grid(True, linestyle=':', alpha=0.6)
    for bar in bars1:
        yval = bar.get_height()
        if not np.isnan(yval):
            ax1.text(bar.get_x() + bar.get_width()/2.0, yval + 1.0, f"{yval:.1f}%", ha='center', va='bottom', fontweight='bold')

    # 2. Mean Delay
    bars2 = ax2.bar(labels, lat_means, yerr=lat_cis, color=colors, width=0.55, edgecolor='black', alpha=0.85, capsize=5)
    ax2.set_ylabel('Mean Latency (ms)')
    ax2.set_title(f'(B) Emergency Telemetry Mean Latency [{target_scenario}]', fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.6)
    for bar in bars2:
        yval = bar.get_height()
        if not np.isnan(yval):
            ax2.text(bar.get_x() + bar.get_width()/2.0, yval + 1.0, f"{yval:.1f} ms", ha='center', va='bottom', fontweight='bold')

    # 3. Route Switches Executed
    bars3 = ax3.bar(labels, sw_means, yerr=sw_cis, color=colors, width=0.55, edgecolor='black', alpha=0.85, capsize=5)
    ax3.set_ylabel('Switch Count')
    ax3.set_title('(C) Dynamic Route Switches Executed', fontweight='bold')
    ax3.grid(True, linestyle=':', alpha=0.6)
    for bar in bars3:
        yval = bar.get_height()
        if not np.isnan(yval):
            ax3.text(bar.get_x() + bar.get_width()/2.0, yval + 0.1, f"{yval:.1f}", ha='center', va='bottom', fontweight='bold')

    # 4. File Transfer Completion Duration
    bars4 = ax4.bar(labels, file_means, yerr=file_cis, color=colors, width=0.55, edgecolor='black', alpha=0.85, capsize=5)
    ax4.set_ylabel('File Transfer Duration (s)')
    ax4.set_title('(D) Bulk File Transfer Completion Time', fontweight='bold')
    ax4.grid(True, linestyle=':', alpha=0.6)
    for bar in bars4:
        yval = bar.get_height()
        if not np.isnan(yval):
            ax4.text(bar.get_x() + bar.get_width()/2.0, yval + 0.5, f"{yval:.1f} s", ha='center', va='bottom', fontweight='bold')

    plt.tight_layout()
    out_path = os.path.join(PLOTS_DIR, "bar_charts_vs_baselines.png")
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"[Generated] {out_path}")

def plot_path_timeline():
    """Figure 3: Active Router Path Timeline reconstructed from real decisions.csv with failure markers."""
    dec_file = find_file("decisions_adaptive_nodefail_single.csv")
    if not dec_file:
        dec_file = find_file("decisions_adaptive_steady.csv")
    if not dec_file:
        dec_file = find_file("decisions.csv")

    if not dec_file:
        print("[Notice] decisions.csv not found yet. Run simulations first to generate Figure 3.")
        return

    df = pd.read_csv(dec_file)
    fig, ax = plt.subplots(figsize=(11, 5.2))

    classes = ['Emergency', 'Video', 'FileTransfer']
    y_map = {'Emergency': 2, 'Video': 1, 'FileTransfer': 0}

    # Extract simulation duration from time series or decisions
    sim_end = 30.0
    ts_file = find_file("time_series.csv")
    if ts_file:
        tdf = pd.read_csv(ts_file)
        if 'Time' in tdf.columns and len(tdf) > 0:
            sim_end = float(tdf['Time'].max())

    for c in classes:
        sub = df[df['Class'] == c].sort_values(by='Time')
        cur_t = 0.0
        cur_p = 0 if c == 'Emergency' else (1 if c == 'Video' else 2)

        for _, row in sub.iterrows():
            t = float(row['Time'])
            dur = t - cur_t
            if dur > 0:
                ax.barh(y_map[c], dur, left=cur_t, height=0.55,
                        color=PATH_COLORS[cur_p % 8], edgecolor='black', alpha=0.85)
            cur_t = t
            cur_p = parse_path_id(row['NewPath'])
            if np.isnan(cur_p):
                cur_p = 0
            else:
                cur_p = int(cur_p)

        if cur_t < sim_end:
            ax.barh(y_map[c], sim_end - cur_t, left=cur_t, height=0.55,
                    color=PATH_COLORS[cur_p % 8], edgecolor='black', alpha=0.85)

    # Add failure markers from failure_detection.csv
    fail_file = find_file("failure_detection.csv")
    if fail_file:
        fdf = pd.read_csv(fail_file)
        added_label = False
        for _, frow in fdf.iterrows():
            try:
                rfail = float(frow['RealFailTime'])
                rrest = float(frow['RealRestoreTime']) if ('RealRestoreTime' in frow and not pd.isna(frow['RealRestoreTime'])) else sim_end
                pid = int(frow['PathId'])
                lbl = f"Router R{pid} Hardware Down" if not added_label else None
                ax.axvspan(rfail, rrest, color='#e74c3c', alpha=0.18, hatch='//', label=lbl)
                ax.axvline(x=rfail, color='darkred', linestyle='--', lw=1.8)
                ax.text(rfail, 2.38, f"R{pid} Fail\n({rfail:.1f}s)", color='darkred', fontsize=8.5, ha='center', va='bottom', fontweight='bold')
                if rrest < sim_end:
                    ax.axvline(x=rrest, color='darkgreen', linestyle=':', lw=1.8)
                    ax.text(rrest, 2.38, f"R{pid} Restored\n({rrest:.1f}s)", color='darkgreen', fontsize=8.5, ha='center', va='bottom', fontweight='bold')
                added_label = True
            except Exception:
                pass

    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(['File Transfer (TCP)', 'HD Video (UDP)', 'Emergency Vitals (UDP)'], fontweight='bold')
    ax.set_xlabel('Simulation Time (seconds)')
    ax.set_title('Reconstructed Active Router Path Timeline per Traffic Class with Failure Events', fontweight='bold')
    ax.set_xlim(0, sim_end)
    ax.set_ylim(-0.6, 2.8)
    ax.grid(True, linestyle=':', alpha=0.6)

    legend_elements = [
        Patch(facecolor=PATH_COLORS[i], edgecolor='black', label=f"Path {i}: {ROUTER_LABELS[i]}")
        for i in range(8)
    ]
    if fail_file and added_label:
        legend_elements.append(Patch(facecolor='#e74c3c', hatch='//', edgecolor='darkred', alpha=0.3, label='Hardware Down Period'))

    ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1.0, -0.18), ncol=3)

    plt.tight_layout()
    out_path = os.path.join(PLOTS_DIR, "path_timeline.png")
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"[Generated] {out_path}")

def plot_detection_reroute_latency():
    """Figure 4: Failure Detection Latency, Re-route Latency, and Outage Metrics."""
    fail_file = find_file("failure_detection.csv")
    if not fail_file:
        print("[Notice] failure_detection.csv not found yet. Run node-failure simulations first.")
        return

    df = pd.read_csv(fail_file)
    if df.empty:
        print("[Notice] failure_detection.csv is empty.")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.8))

    # Clean data
    events = []
    det_lats = []
    rec_lats = []
    outage_gaps = []
    reroute_lats = []

    for idx, row in df.iterrows():
        sc = str(row['Scenario'])
        pid = str(row['PathId'])
        event_name = f"Evt {idx+1}\n({sc}/R{pid})"
        events.append(event_name)

        det_lats.append(float(row['DetectLatencySec']) if not pd.isna(row['DetectLatencySec']) else 0.0)
        rec_lats.append(float(row['RecoverLatencySec']) if not pd.isna(row['RecoverLatencySec']) else 0.0)
        outage_gaps.append(float(row['EmergOutageGapSec']) if not pd.isna(row['EmergOutageGapSec']) else 0.0)
        reroute_lats.append(float(row['EmergRerouteLatencySec']) if (not pd.isna(row['EmergRerouteLatencySec']) and float(row['EmergRerouteLatencySec']) >= 0) else 0.0)

    x = np.arange(len(events))
    width = 0.35

    # Subplot A: Detection & Recovery Latencies
    bars1 = ax1.bar(x - width/2, det_lats, width, label='Failure Detection Latency', color='#e74c3c', edgecolor='black', alpha=0.85)
    bars2 = ax1.bar(x + width/2, rec_lats, width, label='Recovery Detection Latency', color='#2ecc71', edgecolor='black', alpha=0.85)
    ax1.set_ylabel('Latency (seconds)')
    ax1.set_title('(A) Probe-Based Autonomous Detection Latency', fontweight='bold')
    ax1.set_xticks(x)
    ax1.set_xticklabels(events, fontsize=8.5)
    ax1.legend(loc='upper right')
    ax1.grid(True, linestyle=':', alpha=0.6)

    for bar in bars1:
        yval = bar.get_height()
        if yval > 0:
            ax1.text(bar.get_x() + bar.get_width()/2.0, yval + 0.03, f"{yval:.2f}s", ha='center', va='bottom', fontsize=8, fontweight='bold')
    for bar in bars2:
        yval = bar.get_height()
        if yval > 0:
            ax1.text(bar.get_x() + bar.get_width()/2.0, yval + 0.03, f"{yval:.2f}s", ha='center', va='bottom', fontsize=8, fontweight='bold')

    # Subplot B: Emergency Outage Gap and Reroute Latency
    bars3 = ax2.bar(x - width/2, outage_gaps, width, label='Emergency Outage Gap (Max Inter-Packet)', color='#e67e22', edgecolor='black', alpha=0.85)
    bars4 = ax2.bar(x + width/2, reroute_lats, width, label='Emergency Reroute Latency', color='#3498db', edgecolor='black', alpha=0.85)
    ax2.set_ylabel('Duration (seconds)')
    ax2.set_title('(B) Emergency Service Outage & Reroute Latency', fontweight='bold')
    ax2.set_xticks(x)
    ax2.set_xticklabels(events, fontsize=8.5)
    ax2.legend(loc='upper right')
    ax2.grid(True, linestyle=':', alpha=0.6)

    for bar in bars3:
        yval = bar.get_height()
        if yval > 0:
            ax2.text(bar.get_x() + bar.get_width()/2.0, yval + 0.03, f"{yval:.2f}s", ha='center', va='bottom', fontsize=8, fontweight='bold')
    for bar in bars4:
        yval = bar.get_height()
        if yval > 0:
            ax2.text(bar.get_x() + bar.get_width()/2.0, yval + 0.03, f"{yval:.2f}s", ha='center', va='bottom', fontsize=8, fontweight='bold')

    plt.tight_layout()
    out_path = os.path.join(PLOTS_DIR, "detection_reroute_latency.png")
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"[Generated] {out_path}")

def plot_weight_sensitivity():
    """Figure 5: Decision Boundary and Cost Surface across Weights for All 3 Classes."""
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(14, 4.4))

    delays = np.linspace(5, 100, 50)
    bws = np.linspace(1, 100, 50)
    D, B = np.meshgrid(delays, bws)

    # Cost formula: wd * (D / 100) + wl * LossNorm + wb * (1.0 / B)
    # Emergency: wd=0.70, wl=0.25, wb=0.05
    C_emerg = 0.70 * (D / 100.0) + 0.05 * (1.0 / B)
    # Video: wd=0.35, wl=0.15, wb=0.50
    C_video = 0.35 * (D / 100.0) + 0.50 * (1.0 / B)
    # File: wd=0.10, wl=0.20, wb=0.70
    C_file  = 0.10 * (D / 100.0) + 0.70 * (1.0 / B)

    cp1 = ax1.contourf(D, B, C_emerg, levels=18, cmap='Reds_r')
    fig.colorbar(cp1, ax=ax1, label='Normalized Cost')
    ax1.set_xlabel('Path Delay (ms)')
    ax1.set_ylabel('Path Bandwidth (Mbps)')
    ax1.set_title('(A) Emergency (Delay Sensitive)\nwd=0.70, wl=0.25, wb=0.05', fontweight='bold')

    cp2 = ax2.contourf(D, B, C_video, levels=18, cmap='Blues_r')
    fig.colorbar(cp2, ax=ax2, label='Normalized Cost')
    ax2.set_xlabel('Path Delay (ms)')
    ax2.set_ylabel('Path Bandwidth (Mbps)')
    ax2.set_title('(B) HD Video (Balanced/Bandwidth)\nwd=0.35, wl=0.15, wb=0.50', fontweight='bold')

    cp3 = ax3.contourf(D, B, C_file, levels=18, cmap='Greens_r')
    fig.colorbar(cp3, ax=ax3, label='Normalized Cost')
    ax3.set_xlabel('Path Delay (ms)')
    ax3.set_ylabel('Path Bandwidth (Mbps)')
    ax3.set_title('(C) File Transfer (Throughput/BW)\nwd=0.10, wl=0.20, wb=0.70', fontweight='bold')

    plt.tight_layout()
    out_path = os.path.join(PLOTS_DIR, "weight_sensitivity.png")
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"[Generated] {out_path}")

def main():
    print("==================================================================")
    print(" Adaptive Net: Empirical Visualization & Analysis Suite (8-Path)  ")
    print("==================================================================")
    plot_metric_vs_time()
    plot_bar_charts_vs_baselines()
    plot_path_timeline()
    plot_detection_reroute_latency()
    plot_weight_sensitivity()
    print("Visualization processing completed.\n")

if __name__ == '__main__':
    main()
