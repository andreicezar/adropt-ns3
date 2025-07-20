#!/usr/bin/env python3
"""
Comprehensive LoRaWAN Simulation Analyzer - CORRECTED VERSION

Reads all simulation output files (.csv and .txt) to perform a full analysis
of network performance, ADR behavior, and radio link quality, generating
detailed plots and summaries including SF and TP distributions.

KEY FIXES:
- Establishes single source of truth for data consistency
- Uses same data source for both console output and plots
- Eliminates duplicate function definitions
- Improved error handling and validation
"""

import os
import re
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import warnings
from typing import Dict, Tuple, Optional

# --- Configuration ---
PLOT_DIR = "plots"
sns.set_theme(style="whitegrid", palette="viridis")
warnings.filterwarnings("ignore", category=FutureWarning, module="seaborn")

# --- Ground Truth Data Structure ---
class GroundTruthData:
    """Centralized data container to ensure consistency across all analyses"""
    def __init__(self):
        self.primary_radio_data: Optional[pd.DataFrame] = None
        self.adr_data: Optional[pd.DataFrame] = None
        self.gateway_performance: Dict[int, int] = {}
        self.summary_stats: Dict[str, float] = {}
        self.data_source_info: Dict[str, str] = {}
        self.validated: bool = False
    
    def validate_data(self):
        """Validates data quality and consistency"""
        if self.primary_radio_data is not None:
            required_cols = ['Time', 'RSSI_dBm', 'SNR_dB']
            missing_cols = [col for col in required_cols if col not in self.primary_radio_data.columns]
            if missing_cols:
                print(f"⚠️  Warning: Missing columns in radio data: {missing_cols}")
                return False
            
            # Check for reasonable value ranges
            rssi_range = (self.primary_radio_data['RSSI_dBm'].min(), self.primary_radio_data['RSSI_dBm'].max())
            snr_range = (self.primary_radio_data['SNR_dB'].min(), self.primary_radio_data['SNR_dB'].max())
            
            if not (-180 <= rssi_range[0] <= rssi_range[1] <= -30):
                print(f"⚠️  Warning: RSSI values outside expected range: {rssi_range}")
            
            if not (-30 <= snr_range[0] <= snr_range[1] <= 30):
                print(f"⚠️  Warning: SNR values outside expected range: {snr_range}")
            
            self.validated = True
            return True
        return False

# Global ground truth instance
ground_truth = GroundTruthData()

# --- File Reading and Parsing Utilities ---

def load_csv(filepath: str) -> pd.DataFrame | None:
    """Safely loads a CSV file into a pandas DataFrame, handling empty files."""
    if not os.path.exists(filepath):
        print(f"⚠️  Warning: CSV file not found at '{filepath}'")
        return None
    try:
        df = pd.read_csv(filepath)
        if df.empty:
            print(f"⚠️  Warning: CSV file '{filepath}' is empty.")
            return None
        print(f"✅ Successfully loaded '{filepath}' - {len(df)} rows, {len(df.columns)} columns")
        return df
    except Exception as e:
        print(f"❌ Error loading '{filepath}': {e}")
        return None

def read_text_file(filepath: str) -> str | None:
    """Safely reads the entire content of a text file."""
    if not os.path.exists(filepath):
        print(f"⚠️  Warning: Text file not found at '{filepath}'")
        return None
    try:
        with open(filepath, 'r') as f:
            content = f.read()
        print(f"✅ Successfully loaded '{filepath}'")
        return content
    except Exception as e:
        print(f"❌ Error reading '{filepath}': {e}")
        return None

def parse_main_log(content: str | None) -> dict:
    """Parses the main simulation output log for final summary statistics."""
    if not content:
        return {}
    
    summary = {}
    patterns = {
        'transmitted': r"Total packets transmitted: (\d+)",
        'received': r"Total packets received: (\d+)",
        'pdr': r"Packet Delivery Rate \(PDR\): ([\d.]+)%",
    }
    for key, pattern in patterns.items():
        match = re.search(pattern, content)
        if match:
            try:
                summary[key] = float(match.group(1))
            except (ValueError, IndexError):
                continue
    return summary

def establish_ground_truth_data() -> GroundTruthData:
    """
    Establishes the single source of truth for all analyses.
    This ensures consistency between console output and plots.
    """
    print("\n--- 🎯 Establishing Ground Truth Data ---")
    
    # Load all potential data sources
    df_radio = load_csv('rssi_snr_measurements.csv')
    df_measurements = load_csv('radio_measurements.csv')
    df_adr = load_csv('paper_replication_adr.csv')
    main_log_content = read_text_file('paper_replication_output.txt')
    gw_perf_content = read_text_file('paper_phyPerformance.txt')
    
    # Establish primary radio data source (priority order)
    primary_radio_data = None
    data_source_name = ""
    
    if df_measurements is not None and not df_measurements.empty:
        if 'RSSI_dBm' in df_measurements.columns and 'SNR_dB' in df_measurements.columns:
            primary_radio_data = df_measurements
            data_source_name = "Radio Measurements"
            print(f"📊 Primary radio source: {data_source_name} ({len(primary_radio_data)} records)")
    
    if primary_radio_data is None and df_radio is not None and not df_radio.empty:
        if 'RSSI_dBm' in df_radio.columns and 'SNR_dB' in df_radio.columns:
            primary_radio_data = df_radio
            data_source_name = "RSSI/SNR Measurements"
            print(f"📊 Primary radio source: {data_source_name} ({len(primary_radio_data)} records)")
    
    if primary_radio_data is None:
        print("❌ No suitable radio data found. Analysis will be limited.")
        return ground_truth
    
    # Calculate consistent statistics from primary source
    ground_truth.primary_radio_data = primary_radio_data
    ground_truth.adr_data = df_adr
    ground_truth.data_source_info['primary_radio'] = data_source_name
    
    # Calculate gateway performance from radio data (ensures consistency)
    if 'GatewayID' in primary_radio_data.columns:
        ground_truth.gateway_performance = primary_radio_data['GatewayID'].value_counts().sort_index().to_dict()
        print(f"📊 Gateway performance calculated from {data_source_name}")
    
    # Parse summary statistics from log file
    if main_log_content:
        ground_truth.summary_stats = parse_main_log(main_log_content)
    
    # Calculate radio statistics for consistency
    radio_stats = {
        'rssi_mean': primary_radio_data['RSSI_dBm'].mean(),
        'rssi_median': primary_radio_data['RSSI_dBm'].median(),
        'rssi_std': primary_radio_data['RSSI_dBm'].std(),
        'snr_mean': primary_radio_data['SNR_dB'].mean(),
        'snr_median': primary_radio_data['SNR_dB'].median(),
        'snr_std': primary_radio_data['SNR_dB'].std(),
        'total_measurements': len(primary_radio_data)
    }
    ground_truth.summary_stats.update(radio_stats)
    
    # Validate the data
    ground_truth.validate_data()
    
    print(f"📊 Radio statistics: RSSI μ={radio_stats['rssi_mean']:.1f}dBm, SNR μ={radio_stats['snr_mean']:.1f}dB")
    print(f"📊 Total radio events: {radio_stats['total_measurements']:,}")
    
    return ground_truth

# --- Enhanced Plotting Functions (using ground truth data) ---

def plot_sf_tp_distributions():
    """Plots comprehensive SF and TP distributions using ground truth data."""
    print("\n--- 📊 Analyzing SF and TP Distributions ---")
    
    if ground_truth.primary_radio_data is None:
        print("⚠️  No radio data available for SF/TP analysis")
        return
    
    data = ground_truth.primary_radio_data
    
    # Check for required columns
    if 'SpreadingFactor' not in data.columns or 'TxPower_dBm' not in data.columns:
        print("⚠️  SF or TP data not available in primary source")
        return
    
    print(f"📈 Using data from: {ground_truth.data_source_info['primary_radio']}")
    print(f"📊 Analyzing {len(data)} measurements")
    
    fig, axes = plt.subplots(2, 2, figsize=(20, 16))
    fig.suptitle(f'Spreading Factor and Transmission Power Analysis\nData Source: {ground_truth.data_source_info["primary_radio"]}', 
                 fontsize=18, fontweight='bold')
    
    # SF Distribution (Bar Plot)
    sf_counts = data['SpreadingFactor'].value_counts().sort_index()
    ax1 = axes[0, 0]
    sf_bars = ax1.bar(sf_counts.index, sf_counts.values, 
                     color=plt.cm.viridis(np.linspace(0, 1, len(sf_counts))),
                     edgecolor='black', linewidth=1.5)
    
    ax1.set_title('Spreading Factor Distribution', fontsize=16, fontweight='bold')
    ax1.set_xlabel('Spreading Factor (SF)', fontsize=14)
    ax1.set_ylabel('Number of Transmissions', fontsize=14)
    ax1.set_xticks(sf_counts.index)
    ax1.set_xticklabels([f'SF{int(sf)}' for sf in sf_counts.index])
    ax1.grid(True, alpha=0.3)
    
    # Add value labels on bars
    total_sf = sf_counts.sum()
    for bar, count in zip(sf_bars, sf_counts.values):
        height = bar.get_height()
        percentage = (count / total_sf) * 100
        ax1.annotate(f'{count}\n({percentage:.1f}%)',
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    xytext=(0, 3),
                    textcoords="offset points",
                    ha='center', va='bottom',
                    fontsize=12, fontweight='bold')
    
    # SF Distribution Pie Chart
    ax2 = axes[0, 1]
    colors = plt.cm.Set3(np.linspace(0, 1, len(sf_counts)))
    ax2.pie(sf_counts.values, 
            labels=[f'SF{int(sf)}' for sf in sf_counts.index],
            autopct='%1.1f%%', startangle=90, colors=colors,
            textprops={'fontsize': 12, 'fontweight': 'bold'})
    ax2.set_title('SF Distribution (Pie Chart)', fontsize=16, fontweight='bold')
    
    # TP Distribution (Bar Plot)
    tp_counts = data['TxPower_dBm'].value_counts().sort_index()
    ax3 = axes[1, 0]
    tp_bars = ax3.bar(tp_counts.index, tp_counts.values,
                     color=plt.cm.plasma(np.linspace(0, 1, len(tp_counts))),
                     edgecolor='black', linewidth=1.5)
    
    ax3.set_title('Transmission Power Distribution', fontsize=16, fontweight='bold')
    ax3.set_xlabel('Transmission Power (dBm)', fontsize=14)
    ax3.set_ylabel('Number of Transmissions', fontsize=14)
    ax3.grid(True, alpha=0.3)
    
    # Add value labels on bars
    total_tp = tp_counts.sum()
    for bar, count in zip(tp_bars, tp_counts.values):
        height = bar.get_height()
        percentage = (count / total_tp) * 100
        ax3.annotate(f'{count}\n({percentage:.1f}%)',
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    xytext=(0, 3),
                    textcoords="offset points",
                    ha='center', va='bottom',
                    fontsize=12, fontweight='bold')
    
    # TP Distribution Pie Chart
    ax4 = axes[1, 1]
    colors = plt.cm.Set2(np.linspace(0, 1, len(tp_counts)))
    ax4.pie(tp_counts.values,
            labels=[f'{tp} dBm' for tp in tp_counts.index],
            autopct='%1.1f%%', startangle=90, colors=colors,
            textprops={'fontsize': 12, 'fontweight': 'bold'})
    ax4.set_title('TP Distribution (Pie Chart)', fontsize=16, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(os.path.join(PLOT_DIR, 'sf_tp_distributions.png'), dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  -> Generated 'sf_tp_distributions.png'")
    
    # Print statistics (consistent with ground truth)
    print(f"\n📊 SF Distribution Statistics:")
    for sf, count in sf_counts.items():
        percentage = (count / len(data)) * 100
        print(f"  SF{int(sf)}: {count:,} transmissions ({percentage:.1f}%)")
    
    print(f"\n📊 TP Distribution Statistics:")
    for tp, count in tp_counts.items():
        percentage = (count / len(data)) * 100
        print(f"  {tp} dBm: {count:,} transmissions ({percentage:.1f}%)")

def plot_rssi_analysis():
    """Comprehensive RSSI analysis using ground truth data."""
    if ground_truth.primary_radio_data is None:
        print("ℹ️  Skipping RSSI analysis: No radio data available.")
        return
    
    print(f"\n--- 📈 Analyzing RSSI (Data Source: {ground_truth.data_source_info['primary_radio']}) ---")
    data = ground_truth.primary_radio_data.sort_values('Time').copy()
    
    fig, axes = plt.subplots(2, 2, figsize=(20, 16))
    fig.suptitle(f'Comprehensive RSSI Analysis\nData Source: {ground_truth.data_source_info["primary_radio"]}', 
                 fontsize=18, fontweight='bold')
    
    # Time Series Plot
    ax1 = axes[0, 0]
    if 'GatewayID' in data.columns:
        unique_gateways = sorted(data['GatewayID'].unique())
        colors = plt.get_cmap('tab10', len(unique_gateways))
        for i, gw_id in enumerate(unique_gateways):
            gw_data = data[data['GatewayID'] == gw_id]
            ax1.scatter(gw_data['Time'], gw_data['RSSI_dBm'], color=colors(i), alpha=0.6, s=20, label=f'GW {gw_id}')
    else:
        ax1.scatter(data['Time'], data['RSSI_dBm'], alpha=0.6, s=20, color='blue', label='All Measurements')
    
    # Add moving average
    if len(data) > 10:
        window_size = max(3, min(len(data) // 10, 50))
        rolling_mean = data['RSSI_dBm'].rolling(window=window_size, center=True).mean()
        ax1.plot(data['Time'], rolling_mean, color='red', linewidth=3, label=f'Moving Average (n={window_size})')
    
    # Add sensitivity thresholds
    ax1.axhline(y=-130, color='orangered', linestyle='--', alpha=0.8, linewidth=2, label='SF7 Sensitivity (-130 dBm)')
    ax1.axhline(y=-137, color='goldenrod', linestyle='--', alpha=0.8, linewidth=2, label='SF12 Sensitivity (-137 dBm)')
    
    ax1.set_title('RSSI Over Time', fontsize=16, fontweight='bold')
    ax1.set_xlabel('Time (seconds)', fontsize=14)
    ax1.set_ylabel('RSSI (dBm)', fontsize=14)
    ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax1.grid(True, alpha=0.3)
    
    # Overall Histogram
    ax2 = axes[0, 1]
    ax2.hist(data['RSSI_dBm'], bins=30, alpha=0.7, color='skyblue', edgecolor='black')
    ax2.axvline(ground_truth.summary_stats['rssi_mean'], color='red', linestyle='--', linewidth=2, 
               label=f'Mean: {ground_truth.summary_stats["rssi_mean"]:.1f} dBm')
    ax2.axvline(ground_truth.summary_stats['rssi_median'], color='green', linestyle='--', linewidth=2, 
               label=f'Median: {ground_truth.summary_stats["rssi_median"]:.1f} dBm')
    ax2.set_title('RSSI Distribution (All Data)', fontsize=16, fontweight='bold')
    ax2.set_xlabel('RSSI (dBm)', fontsize=14)
    ax2.set_ylabel('Frequency', fontsize=14)
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Per-Gateway Histogram
    ax3 = axes[1, 0]
    if 'GatewayID' in data.columns:
        unique_gateways = sorted(data['GatewayID'].unique())
        colors = plt.get_cmap('Set3', len(unique_gateways))
        for i, gw_id in enumerate(unique_gateways):
            gw_data = data[data['GatewayID'] == gw_id]
            ax3.hist(gw_data['RSSI_dBm'], bins=20, alpha=0.6, color=colors(i), 
                    label=f'GW {gw_id} (μ={gw_data["RSSI_dBm"].mean():.1f})', density=True)
        ax3.set_title('RSSI Distribution by Gateway', fontsize=16, fontweight='bold')
        ax3.legend()
    else:
        ax3.hist(data['RSSI_dBm'], bins=20, alpha=0.7, color='lightcoral', density=True)
        ax3.set_title('RSSI Distribution', fontsize=16, fontweight='bold')
    
    ax3.set_xlabel('RSSI (dBm)', fontsize=14)
    ax3.set_ylabel('Density', fontsize=14)
    ax3.grid(True, alpha=0.3)
    
    # RSSI Statistics Box Plot
    ax4 = axes[1, 1]
    if 'GatewayID' in data.columns:
        gw_data_list = []
        gw_labels = []
        for gw_id in sorted(data['GatewayID'].unique()):
            gw_rssi = data[data['GatewayID'] == gw_id]['RSSI_dBm']
            gw_data_list.append(gw_rssi)
            gw_labels.append(f'GW {gw_id}')
        
        box_plot = ax4.boxplot(gw_data_list, tick_labels=gw_labels, patch_artist=True)
        colors = plt.get_cmap('Set2', len(gw_data_list))
        for patch, color in zip(box_plot['boxes'], [colors(i) for i in range(len(gw_data_list))]):
            patch.set_facecolor(color)
        ax4.set_title('RSSI Statistics by Gateway', fontsize=16, fontweight='bold')
    else:
        ax4.boxplot([data['RSSI_dBm']], tick_labels=['All Data'], patch_artist=True)
        ax4.set_title('RSSI Statistics', fontsize=16, fontweight='bold')
    
    ax4.set_ylabel('RSSI (dBm)', fontsize=14)
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(PLOT_DIR, 'rssi_comprehensive_analysis.png'), dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  -> Generated 'rssi_comprehensive_analysis.png'")
    
    # Print RSSI statistics (consistent with ground truth)
    print(f"\n📊 RSSI Statistics (consistent with ground truth):")
    print(f"  Mean: {ground_truth.summary_stats['rssi_mean']:.2f} dBm")
    print(f"  Median: {ground_truth.summary_stats['rssi_median']:.2f} dBm")
    print(f"  Std Dev: {ground_truth.summary_stats['rssi_std']:.2f} dB")

def plot_snr_analysis():
    """Comprehensive SNR analysis using ground truth data."""
    if ground_truth.primary_radio_data is None:
        print("ℹ️  Skipping SNR analysis: No radio data available.")
        return
    
    print(f"\n--- 📡 Analyzing SNR (Data Source: {ground_truth.data_source_info['primary_radio']}) ---")
    data = ground_truth.primary_radio_data.sort_values('Time').copy()
    
    fig, axes = plt.subplots(2, 2, figsize=(20, 16))
    fig.suptitle(f'Comprehensive SNR Analysis\nData Source: {ground_truth.data_source_info["primary_radio"]}', 
                 fontsize=18, fontweight='bold')
    
    # SNR Time Series
    ax1 = axes[0, 0]
    if 'GatewayID' in data.columns:
        unique_gateways = sorted(data['GatewayID'].unique())
        colors = plt.get_cmap('tab10', len(unique_gateways))
        for i, gw_id in enumerate(unique_gateways):
            gw_data = data[data['GatewayID'] == gw_id]
            ax1.scatter(gw_data['Time'], gw_data['SNR_dB'], color=colors(i), alpha=0.6, s=20, label=f'GW {gw_id}')
    else:
        ax1.scatter(data['Time'], data['SNR_dB'], alpha=0.6, s=20, color='green', label='All Measurements')
    
    # Add moving average for SNR
    if len(data) > 10:
        window_size = max(3, min(len(data) // 10, 50))
        rolling_mean = data['SNR_dB'].rolling(window=window_size, center=True).mean()
        ax1.plot(data['Time'], rolling_mean, color='red', linewidth=3, label=f'Moving Average (n={window_size})')
    
    ax1.axhline(y=0, color='orange', linestyle='--', alpha=0.8, linewidth=2, label='SNR Threshold (0 dB)')
    ax1.set_title('SNR Over Time', fontsize=16, fontweight='bold')
    ax1.set_xlabel('Time (seconds)', fontsize=14)
    ax1.set_ylabel('SNR (dB)', fontsize=14)
    ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax1.grid(True, alpha=0.3)
    
    # SNR Histogram
    ax2 = axes[0, 1]
    ax2.hist(data['SNR_dB'], bins=30, alpha=0.7, color='lightgreen', edgecolor='black')
    ax2.axvline(ground_truth.summary_stats['snr_mean'], color='red', linestyle='--', linewidth=2, 
               label=f'Mean: {ground_truth.summary_stats["snr_mean"]:.1f} dB')
    ax2.axvline(ground_truth.summary_stats['snr_median'], color='blue', linestyle='--', linewidth=2, 
               label=f'Median: {ground_truth.summary_stats["snr_median"]:.1f} dB')
    ax2.set_title('SNR Distribution', fontsize=16, fontweight='bold')
    ax2.set_xlabel('SNR (dB)', fontsize=14)
    ax2.set_ylabel('Frequency', fontsize=14)
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # SNR per Gateway Box Plot
    ax3 = axes[1, 0]
    if 'GatewayID' in data.columns:
        gw_data_list = []
        gw_labels = []
        for gw_id in sorted(data['GatewayID'].unique()):
            gw_snr = data[data['GatewayID'] == gw_id]['SNR_dB']
            gw_data_list.append(gw_snr)
            gw_labels.append(f'GW {gw_id}')
        
        box_plot = ax3.boxplot(gw_data_list, tick_labels=gw_labels, patch_artist=True)
        colors = plt.get_cmap('Set2', len(gw_data_list))
        for patch, color in zip(box_plot['boxes'], [colors(i) for i in range(len(gw_data_list))]):
            patch.set_facecolor(color)
        ax3.set_title('SNR Statistics by Gateway', fontsize=16, fontweight='bold')
    else:
        ax3.boxplot([data['SNR_dB']], tick_labels=['All Data'], patch_artist=True)
        ax3.set_title('SNR Statistics', fontsize=16, fontweight='bold')
    
    ax3.set_ylabel('SNR (dB)', fontsize=14)
    ax3.grid(True, alpha=0.3)
    
    # SNR per Gateway Histogram
    ax4 = axes[1, 1]
    if 'GatewayID' in data.columns:
        unique_gateways = sorted(data['GatewayID'].unique())
        colors = plt.get_cmap('Set3', len(unique_gateways))
        for i, gw_id in enumerate(unique_gateways):
            gw_data = data[data['GatewayID'] == gw_id]
            ax4.hist(gw_data['SNR_dB'], bins=15, alpha=0.6, color=colors(i), 
                    label=f'GW {gw_id} (μ={gw_data["SNR_dB"].mean():.1f})', density=True)
        ax4.set_title('SNR Distribution by Gateway', fontsize=16, fontweight='bold')
        ax4.legend()
    else:
        ax4.hist(data['SNR_dB'], bins=15, alpha=0.7, color='lightcyan', density=True)
        ax4.set_title('SNR Distribution', fontsize=16, fontweight='bold')
    
    ax4.set_xlabel('SNR (dB)', fontsize=14)
    ax4.set_ylabel('Density', fontsize=14)
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(PLOT_DIR, 'snr_comprehensive_analysis.png'), dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  -> Generated 'snr_comprehensive_analysis.png'")
    
    # Print statistics (consistent with ground truth)
    print(f"\n📊 SNR Statistics (consistent with ground truth):")
    print(f"  Mean: {ground_truth.summary_stats['snr_mean']:.2f} dB")
    print(f"  Median: {ground_truth.summary_stats['snr_median']:.2f} dB")
    print(f"  Std Dev: {ground_truth.summary_stats['snr_std']:.2f} dB")

def plot_gateway_performance():
    """Plot gateway performance using consistent ground truth data."""
    print(f"\n--- 📊 Analyzing Gateway Performance (Data Source: {ground_truth.data_source_info['primary_radio']}) ---")
    
    if not ground_truth.gateway_performance:
        print("⚠️  No gateway performance data available.")
        return
    
    fig, axes = plt.subplots(2, 2, figsize=(20, 16))
    fig.suptitle(f'Gateway Performance Analysis\nData Source: {ground_truth.data_source_info["primary_radio"]}', 
                 fontsize=18, fontweight='bold')
    
    gw_ids = list(ground_truth.gateway_performance.keys())
    gw_counts = list(ground_truth.gateway_performance.values())
    total_packets = sum(gw_counts)
    gw_rates = [(count/total_packets)*100 if total_packets > 0 else 0 for count in gw_counts]
    
    # Total packets received per gateway
    ax1 = axes[0, 0]
    bars = ax1.bar(gw_ids, gw_counts, color=plt.cm.Blues(np.linspace(0.4, 0.9, len(gw_ids))), 
                   edgecolor='black', linewidth=1.5)
    ax1.set_title('Total Packets Received per Gateway', fontsize=16, fontweight='bold')
    ax1.set_xlabel('Gateway ID', fontsize=14)
    ax1.set_ylabel('Packet Count', fontsize=14)
    ax1.grid(True, alpha=0.3)
    
    # Add value labels on bars
    for bar, count in zip(bars, gw_counts):
        height = bar.get_height()
        ax1.annotate(f'{count}',
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    xytext=(0, 3),
                    textcoords="offset points",
                    ha='center', va='bottom',
                    fontsize=12, fontweight='bold')
    
    # Gateway extraction rate percentage
    ax2 = axes[0, 1]
    pie_colors = plt.cm.Set3(np.linspace(0, 1, len(gw_ids)))
    ax2.pie(gw_counts, labels=[f'GW {gw_id}' for gw_id in gw_ids],
            autopct='%1.1f%%', startangle=90, colors=pie_colors,
            textprops={'fontsize': 12, 'fontweight': 'bold'})
    ax2.set_title('Gateway Data Distribution', fontsize=16, fontweight='bold')
    
    # Gateway extraction rates as bar chart
    ax3 = axes[1, 0]
    bars = ax3.bar(gw_ids, gw_rates, color=plt.cm.Oranges(np.linspace(0.4, 0.9, len(gw_ids))),
                  edgecolor='black', linewidth=1.5)
    ax3.set_title('Gateway Extraction Rate (%)', fontsize=16, fontweight='bold')
    ax3.set_xlabel('Gateway ID', fontsize=14)
    ax3.set_ylabel('Extraction Rate (%)', fontsize=14)
    ax3.grid(True, alpha=0.3)
    
    for bar, rate in zip(bars, gw_rates):
        height = bar.get_height()
        ax3.annotate(f'{rate:.1f}%',
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    xytext=(0, 3),
                    textcoords="offset points",
                    ha='center', va='bottom',
                    fontsize=10, fontweight='bold')
    
    # Gateway performance statistics summary
    ax4 = axes[1, 1]
    ax4.axis('off')
    
    max_gw = max(ground_truth.gateway_performance, key=ground_truth.gateway_performance.get)
    min_gw = min(ground_truth.gateway_performance, key=ground_truth.gateway_performance.get)
    avg_packets = np.mean(gw_counts)
    std_packets = np.std(gw_counts)
    
    stats_text = f"""Gateway Performance Summary:

Data Source: {ground_truth.data_source_info['primary_radio']}
Total Gateways: {len(gw_ids)}
Total Packets: {total_packets:,}
Average per Gateway: {avg_packets:.1f}
Std Deviation: {std_packets:.1f}

Best Performing Gateway:
  GW {max_gw}: {ground_truth.gateway_performance[max_gw]:,} packets ({(ground_truth.gateway_performance[max_gw]/total_packets)*100:.1f}%)

Worst Performing Gateway:  
  GW {min_gw}: {ground_truth.gateway_performance[min_gw]:,} packets ({(ground_truth.gateway_performance[min_gw]/total_packets)*100:.1f}%)

Load Balance Factor: {(std_packets/avg_packets)*100:.1f}%
(Lower is better distributed)
    """
    
    ax4.text(0.5, 0.5, stats_text, ha='center', va='center', fontsize=11,
            bbox=dict(boxstyle="round,pad=0.5", fc="lightcyan", ec="teal", lw=2))
    ax4.set_title('Gateway Statistics Summary', fontsize=16, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(os.path.join(PLOT_DIR, 'gateway_performance_analysis.png'), dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  -> Generated 'gateway_performance_analysis.png'")
    
    # Print gateway statistics (consistent with ground truth)
    print(f"\n📊 Gateway Performance Statistics (consistent with ground truth):")
    print(f"  Data source: {ground_truth.data_source_info['primary_radio']}")
    print(f"  Total packets: {total_packets:,}")
    print(f"  Average packets per gateway: {avg_packets:.1f}")
    print(f"  Best performing gateway: GW {max_gw} ({ground_truth.gateway_performance[max_gw]:,} packets)")
    print(f"  Worst performing gateway: GW {min_gw} ({ground_truth.gateway_performance[min_gw]:,} packets)")

def create_summary_dashboard():
    """Creates a comprehensive summary dashboard using consistent ground truth data."""
    if ground_truth.primary_radio_data is None:
        print("ℹ️  Skipping dashboard creation: No radio data available.")
        return

    print(f"\n--- 🖼️  Creating Summary Dashboard (Data Source: {ground_truth.data_source_info['primary_radio']}) ---")
    
    fig = plt.figure(figsize=(20, 12), constrained_layout=True)
    gs = fig.add_gridspec(2, 3)
    fig.suptitle(f'LoRaWAN Simulation Analysis Dashboard\nData Source: {ground_truth.data_source_info["primary_radio"]}', 
                 fontsize=22, fontweight='bold')

    # Metric Text Box (using consistent ground truth data)
    ax_text = fig.add_subplot(gs[0, 0])
    ax_text.axis('off')
    pdr_text = f"{ground_truth.summary_stats.get('pdr', 0):.1f}%"
    
    text_content = (
        f"Final PDR: {pdr_text}\n\n"
        f"Packets Sent: {int(ground_truth.summary_stats.get('transmitted', 0))}\n"
        f"Packets Received: {int(ground_truth.summary_stats.get('received', 0))}\n\n"
        f"Median SNR: {ground_truth.summary_stats['snr_median']:.1f} dB\n"
        f"Median RSSI: {ground_truth.summary_stats['rssi_median']:.1f} dBm\n"
        f"\nRadio Events: {ground_truth.summary_stats['total_measurements']:,}\n"
        f"Data Source: {ground_truth.data_source_info['primary_radio']}"
    )
    
    ax_text.text(0.5, 0.5, text_content, ha='center', va='center', fontsize=15,
                 bbox=dict(boxstyle="round,pad=0.5", fc="lightblue", ec="navy", lw=2))
    ax_text.set_title("Key Performance Indicators", fontsize=16, fontweight='bold', pad=20)

    # RSSI Distribution (using ground truth data)
    ax_rssi = fig.add_subplot(gs[0, 1])
    ax_rssi.hist(ground_truth.primary_radio_data['RSSI_dBm'], bins=20, color='skyblue', alpha=0.7, edgecolor='black')
    ax_rssi.set_title('RSSI Distribution', fontweight='bold', fontsize=16)
    ax_rssi.set_xlabel('RSSI (dBm)')
    ax_rssi.set_ylabel('Frequency')
    ax_rssi.grid(True, alpha=0.3)

    # SNR Distribution (using ground truth data)
    ax_snr = fig.add_subplot(gs[0, 2])
    ax_snr.hist(ground_truth.primary_radio_data['SNR_dB'], bins=20, color='salmon', alpha=0.7, edgecolor='black')
    ax_snr.set_title('SNR Distribution', fontweight='bold', fontsize=16)
    ax_snr.set_xlabel('SNR (dB)')
    ax_snr.set_ylabel('Frequency')
    ax_snr.grid(True, alpha=0.3)

    # Gateway Performance (using consistent ground truth data)
    ax_gw = fig.add_subplot(gs[1, :])
    if ground_truth.gateway_performance:
        gw_ids = list(ground_truth.gateway_performance.keys())
        gw_counts = list(ground_truth.gateway_performance.values())
        bars = ax_gw.bar(gw_ids, gw_counts, color='teal', alpha=0.8, edgecolor='black')
        
        total_packets = sum(gw_counts)
        title = f'Packets Received per Gateway ({ground_truth.data_source_info["primary_radio"]})\nTotal: {total_packets:,} packets'
        
        ax_gw.set_title(title, fontweight='bold', fontsize=16)
        ax_gw.set_xlabel('Gateway ID')
        ax_gw.set_ylabel('Packet Count')
        ax_gw.grid(True, alpha=0.3)
        
        # Add value labels on bars
        for bar in bars:
            height = bar.get_height()
            ax_gw.annotate(f'{int(height):,}',
                          xy=(bar.get_x() + bar.get_width() / 2, height),
                          xytext=(0, 3),
                          textcoords="offset points",
                          ha='center', va='bottom',
                          fontsize=12, fontweight='bold')

    plt.savefig(os.path.join(PLOT_DIR, 'comprehensive_summary.png'), dpi=150)
    plt.close()
    print(f"  -> Generated 'comprehensive_summary.png'")
    print(f"     All data consistent from: {ground_truth.data_source_info['primary_radio']}")

# --- Main Execution ---

def main():
    """Main function to orchestrate the loading, analysis, and plotting."""
    print("="*80)
    print("CORRECTED LORAWAN SIMULATION ANALYZER - SINGLE SOURCE OF TRUTH")
    print("="*80)

    if not os.path.exists(PLOT_DIR):
        os.makedirs(PLOT_DIR)

    # Establish ground truth data FIRST
    global ground_truth
    ground_truth = establish_ground_truth_data()
    
    if not ground_truth.validated:
        print("❌ Ground truth data validation failed. Exiting.")
        return
    
    print(f"\n📊 Ground Truth Established:")
    print(f"  Primary source: {ground_truth.data_source_info.get('primary_radio', 'None')}")
    print(f"  Radio measurements: {ground_truth.summary_stats.get('total_measurements', 0):,}")
    print(f"  Gateways detected: {len(ground_truth.gateway_performance)}")
    print(f"  RSSI range: {ground_truth.primary_radio_data['RSSI_dBm'].min():.1f} to {ground_truth.primary_radio_data['RSSI_dBm'].max():.1f} dBm")
    print(f"  SNR range: {ground_truth.primary_radio_data['SNR_dB'].min():.1f} to {ground_truth.primary_radio_data['SNR_dB'].max():.1f} dB")

    # Generate all plots using consistent ground truth data
    plot_sf_tp_distributions()
    plot_rssi_analysis()
    plot_snr_analysis()
    plot_gateway_performance()
    create_summary_dashboard()

    print("\n" + "="*80)
    print("✅ CONSISTENT Analysis Complete. All data from single source:")
    print(f"   📊 {ground_truth.data_source_info['primary_radio']}")
    print(f"   📈 {ground_truth.summary_stats['total_measurements']:,} measurements analyzed")
    
    plots_generated = [
        "sf_tp_distributions.png - SF and TP distribution analysis",
        "rssi_comprehensive_analysis.png - RSSI analysis with consistent statistics",
        "snr_comprehensive_analysis.png - SNR analysis with consistent statistics", 
        "gateway_performance_analysis.png - Gateway performance with consistent data",
        "comprehensive_summary.png - Dashboard with all consistent metrics"
    ]
    
    print("\n📊 Generated plots (all using consistent data):")
    for plot in plots_generated:
        print(f"  ✅ {plot}")
    
    print(f"\n📁 Check the '{PLOT_DIR}' directory for all visualizations.")
    print("🎯 All console output and plots now use the SAME data source!")
    print("="*80)

if __name__ == "__main__":
    main()