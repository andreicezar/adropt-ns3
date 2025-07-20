#!/usr/bin/env python3
"""
Enhanced LoRaWAN Simulation Analyzer - V8 (Corrected for Current Simulation)

This script provides comprehensive analysis with robust data validation, debugging,
and quality assessment capabilities for LoRaWAN simulation results.

UPDATED FOR CURRENT SIMULATION CONFIGURATION:
- Matches reorganized C++ simulation file 
- Matches simplified shell script parameters
- Corrected file names and expected values
- Aligned with exact paper replication setup

TERMINOLOGY (consistent with Heusse et al. 2020 paper):
- FER (Frame Erasure Rate): Physical loss ratio between ED and a given GW
- PER (Packet Error Rate): Loss ratio between ED and Network Server (benefits from macrodiversity + repetitions)  
- PDR (Packet Delivery Rate): Success ratio at network level = 1 - PER
- DER (Data Error Rate): Loss ratio between ED and Application Server (benefits from inter-packet FEC)

ENHANCED FEATURES:
- Comprehensive data validation and quality assessment
- Simulation duration and packet count validation
- Data completeness warnings and debugging info
- Robust handling of limited or incomplete data
- ADRopt parameter evolution tracking
- Gateway performance and diversity analysis
- Channel model validation with statistical tests
- Research-grade output with confidence indicators
- Consistent terminology following the paper
"""

import os
import re
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
from typing import Dict, Optional, Tuple
from scipy import stats
import warnings

# --- Configuration (Updated for Current Simulation) ---
PLOT_DIR = "plots"
DEBUG_DIR = "debug"
sns.set_theme(style="whitegrid", palette="viridis")

# Expected simulation parameters (from current shell script and simulation)
EXPECTED_PACKET_INTERVAL = 144  # seconds (from simulation: 2.4 minutes)
EXPECTED_SIMULATION_DAYS = 7    # days (504 periods × 20 minutes = 7 days)
EXPECTED_GATEWAYS = 8           # exactly 8 gateways from paper
EXPECTED_FADING_STD = 8.0       # dB (paper's Rayleigh fading model)
PAPER_TARGET_DER = 1.0          # % (Data Error Rate target: DER < 0.01 = 1%)
PAPER_TARGET_PDR = 99.0         # % (Packet Delivery Rate for reference)

# Expected packet count calculation (from shell script):
# 7 days = 604,800 seconds, 144s intervals = ~4,200 packets
EXPECTED_TOTAL_PACKETS = int(EXPECTED_SIMULATION_DAYS * 24 * 3600 / EXPECTED_PACKET_INTERVAL)  # ~4200

# File names (matching current simulation output)
RADIO_FILES = ['rssi_snr_measurements.csv', 'radio_measurements.csv', 'radio_measurement_summary.csv']
ADR_FILE = 'paper_replication_adr.csv'
LOG_FILE = 'paper_replication_output.txt'  # May not exist if using shell redirect

class SimulationQuality:
    """Quality assessment container."""
    def __init__(self):
        self.data_completeness = "Unknown"
        self.duration_accuracy = "Unknown" 
        self.channel_model_accuracy = "Unknown"
        self.gateway_diversity = "Unknown"
        self.adropt_functioning = "Unknown"
        self.overall_confidence = "Unknown"
        self.warnings = []
        self.debug_info = []

class GroundTruthData:
    """Enhanced data container with validation."""
    def __init__(self):
        self.radio_data: Optional[pd.DataFrame] = None
        self.adr_data: Optional[pd.DataFrame] = None
        self.packets_sent_per_device: Dict[int, int] = {}
        self.packets_received_per_device: Dict[int, int] = {}
        self.simulation_duration: float = 0.0
        self.data_source_name: str = ""
        self.validated: bool = False
        self.quality: SimulationQuality = SimulationQuality()

    def validate(self) -> bool:
        """Enhanced validation with quality assessment."""
        self.quality = SimulationQuality()
        
        if self.radio_data is None or self.radio_data.empty:
            self.quality.warnings.append("❌ No radio measurement data available")
            return False
            
        # Data completeness check (more lenient for debugging)
        measurement_count = len(self.radio_data)
        # Expected: ~4200 packets × 8 gateways × 90% reception = ~30,000 measurements
        expected_full = EXPECTED_TOTAL_PACKETS * EXPECTED_GATEWAYS * 0.9
        expected_min = max(100, expected_full * 0.01)  # At least 1% of expected
        
        if measurement_count < expected_min:
            self.quality.data_completeness = "Very Limited"
            self.quality.warnings.append(f"❌ Very limited data: {measurement_count} measurements (expected >{expected_full:.0f})")
        elif measurement_count < expected_full * 0.1:
            self.quality.data_completeness = "Limited"
            self.quality.warnings.append(f"⚠️ Limited data: {measurement_count} measurements (expected ~{expected_full:.0f})")
        elif measurement_count > expected_full * 0.5:
            self.quality.data_completeness = "Excellent" 
            self.quality.debug_info.append(f"✅ Rich dataset: {measurement_count} measurements")
        else:
            self.quality.data_completeness = "Adequate"
            self.quality.debug_info.append(f"✅ Reasonable data: {measurement_count} measurements")

        # Duration validation
        if 'Time' in self.radio_data.columns:
            time_span = self.radio_data['Time'].max() - self.radio_data['Time'].min()
            expected_duration = EXPECTED_SIMULATION_DAYS * 24 * 3600
            duration_ratio = time_span / expected_duration
            
            self.simulation_duration = time_span / (24 * 3600)  # days
            
            if 0.8 <= duration_ratio <= 1.2:
                self.quality.duration_accuracy = "Accurate"
                self.quality.debug_info.append(f"✅ Duration: {self.simulation_duration:.1f} days (target: {EXPECTED_SIMULATION_DAYS})")
            else:
                self.quality.duration_accuracy = "Inaccurate"
                self.quality.warnings.append(f"⚠️ Duration: {self.simulation_duration:.1f} days (expected: {EXPECTED_SIMULATION_DAYS})")

        # Gateway diversity check
        if 'GatewayID' in self.radio_data.columns:
            unique_gateways = self.radio_data['GatewayID'].nunique()
            if unique_gateways == EXPECTED_GATEWAYS:
                self.quality.gateway_diversity = "Complete"
                self.quality.debug_info.append(f"✅ All {EXPECTED_GATEWAYS} gateways active")
            elif unique_gateways >= EXPECTED_GATEWAYS * 0.75:
                self.quality.gateway_diversity = "Good"
                self.quality.warnings.append(f"⚠️ {unique_gateways}/{EXPECTED_GATEWAYS} gateways active")
            else:
                self.quality.gateway_diversity = "Poor"
                self.quality.warnings.append(f"❌ Only {unique_gateways}/{EXPECTED_GATEWAYS} gateways active")

        # Channel model validation
        if 'Fading_dB' in self.radio_data.columns:
            fading_std = self.radio_data['Fading_dB'].std()
            std_error = abs(fading_std - EXPECTED_FADING_STD) / EXPECTED_FADING_STD
            
            if std_error < 0.1:  # Within 10%
                self.quality.channel_model_accuracy = "Excellent"
                self.quality.debug_info.append(f"✅ Fading model perfect: {fading_std:.2f}dB (target: {EXPECTED_FADING_STD}dB)")
            elif std_error < 0.25:  # Within 25%
                self.quality.channel_model_accuracy = "Good"
                self.quality.debug_info.append(f"✅ Fading model good: {fading_std:.2f}dB (target: {EXPECTED_FADING_STD}dB)")
            else:
                self.quality.channel_model_accuracy = "Poor"
                self.quality.warnings.append(f"⚠️ Fading model off: {fading_std:.2f}dB (target: {EXPECTED_FADING_STD}dB)")

        # ADRopt functioning check
        if 'SpreadingFactor' in self.radio_data.columns and 'TxPower_dBm' in self.radio_data.columns:
            sf_range = self.radio_data['SpreadingFactor'].max() - self.radio_data['SpreadingFactor'].min()
            power_range = self.radio_data['TxPower_dBm'].max() - self.radio_data['TxPower_dBm'].min()
            
            if sf_range >= 3 and power_range >= 5:  # Significant parameter changes
                self.quality.adropt_functioning = "Active"
                self.quality.debug_info.append(f"✅ ADRopt optimizing: SF range={sf_range}, Power range={power_range:.1f}dB")
            elif sf_range >= 1 or power_range >= 2:
                self.quality.adropt_functioning = "Limited"
                self.quality.warnings.append(f"⚠️ Limited ADRopt activity: SF range={sf_range}, Power range={power_range:.1f}dB")
            else:
                self.quality.adropt_functioning = "Inactive"
                self.quality.warnings.append(f"❌ No ADRopt optimization detected")

        # Overall confidence assessment - Updated for current simulation
        actual_data_rows = len(self.radio_data)
        
        # More lenient thresholds for debugging incomplete exports
        if actual_data_rows < 50:
            self.quality.overall_confidence = "Very Low"
            self.quality.warnings.append("❌ Confidence Very Low: Insufficient CSV data for reliable analysis")
            return True
        elif actual_data_rows < 500:
            self.quality.overall_confidence = "Low" 
            self.quality.warnings.append("⚠️ Confidence Low: Limited CSV data export detected")
            return True
        
        # Assess other quality factors
        quality_scores = [
            self.quality.data_completeness in ["Adequate", "Excellent"],
            self.quality.duration_accuracy == "Accurate", 
            self.quality.gateway_diversity in ["Good", "Complete"],
            self.quality.channel_model_accuracy in ["Good", "Excellent"],
            self.quality.adropt_functioning in ["Limited", "Active"]
        ]
        
        confidence_score = sum(quality_scores) / len(quality_scores)
        
        if confidence_score >= 0.8:
            self.quality.overall_confidence = "High"
        elif confidence_score >= 0.6:
            self.quality.overall_confidence = "Medium"
        else:
            self.quality.overall_confidence = "Low"

        self.validated = True
        return True

def establish_ground_truth() -> GroundTruthData:
    """Enhanced data loading with comprehensive validation."""
    print("\n" + "="*70)
    print("🔍 ENHANCED SIMULATION DATA ANALYSIS (Current Version)")
    print("="*70)
    
    gt = GroundTruthData()

    # Load radio measurement data (updated file priority)
    for radio_file in RADIO_FILES:
        if os.path.exists(radio_file):
            try:
                gt.radio_data = pd.read_csv(radio_file)
                gt.data_source_name = radio_file
                print(f"✅ Loaded radio data: {radio_file} ({len(gt.radio_data)} rows)")
                break
            except Exception as e:
                print(f"❌ Error loading {radio_file}: {e}")
    
    if gt.radio_data is None:
        print("❌ No radio measurement data found. Analysis cannot proceed.")
        print(f"   Expected files: {', '.join(RADIO_FILES)}")
        return gt

    # Load ADR statistics if available
    if os.path.exists(ADR_FILE):
        try:
            gt.adr_data = pd.read_csv(ADR_FILE)
            print(f"✅ Loaded ADR data: {ADR_FILE} ({len(gt.adr_data)} rows)")
        except Exception as e:
            print(f"⚠️ Could not load ADR data: {e}")

    # Extract packet counts from log (updated log file handling)
    log_files_to_try = [LOG_FILE, 'simulation_log.txt', 'output.log']
    log_content = ""
    
    for log_file in log_files_to_try:
        if os.path.exists(log_file):
            try:
                with open(log_file, 'r') as f:
                    log_content = f.read()
                print(f"✅ Loaded log file: {log_file}")
                break
            except Exception as e:
                print(f"⚠️ Could not read {log_file}: {e}")
    
    # Extract packet counts from log or estimate from radio data
    if log_content:
        # Extract transmission count from log
        sent_match = re.search(r"Total packets transmitted: (\d+)", log_content)
        received_match = re.search(r"Total packets received: (\d+)", log_content)
        
        if sent_match and received_match:
            total_sent = int(sent_match.group(1))
            total_received = int(received_match.group(1))
            print(f"✅ From log - Sent: {total_sent}, Received: {total_received}")
            
            # Assign to device(s)
            unique_devices = gt.radio_data['DeviceAddr'].unique() if 'DeviceAddr' in gt.radio_data.columns else [1]
            for device_id in unique_devices:
                gt.packets_sent_per_device[device_id] = total_sent
                gt.packets_received_per_device[device_id] = total_received
        else:
            print("⚠️ Could not parse packet counts from log")
    
    # Fallback: estimate from radio data if no log available
    if not gt.packets_sent_per_device and gt.radio_data is not None:
        print("📊 Estimating packet counts from radio data...")
        if 'DeviceAddr' in gt.radio_data.columns:
            unique_devices = gt.radio_data['DeviceAddr'].unique()
            # Estimate total sent based on expected simulation parameters
            estimated_sent = EXPECTED_TOTAL_PACKETS
            
            for device_id in unique_devices:
                device_receptions = len(gt.radio_data[gt.radio_data['DeviceAddr'] == device_id])
                # Estimate unique packets (divide by gateway count)
                estimated_received = device_receptions // max(1, gt.radio_data['GatewayID'].nunique())
                
                gt.packets_sent_per_device[device_id] = estimated_sent
                gt.packets_received_per_device[device_id] = estimated_received
                
                print(f"📊 Device {device_id} - Estimated sent: {estimated_sent}, received: {estimated_received}")

    # Clean and prepare data
    if gt.radio_data is not None:
        gt.radio_data.dropna(subset=['Time'], inplace=True)
        if 'DeviceAddr' in gt.radio_data.columns:
            gt.radio_data['DeviceAddr'] = gt.radio_data['DeviceAddr'].astype(int)
        if 'GatewayID' in gt.radio_data.columns:
            gt.radio_data['GatewayID'] = gt.radio_data['GatewayID'].astype(int)

    # Validate and assess quality
    gt.validate()
    
    # Print quality assessment
    print(f"\n📊 SIMULATION QUALITY ASSESSMENT:")
    print(f"  Data Source: {gt.data_source_name}")
    print(f"  Data Completeness: {gt.quality.data_completeness}")
    print(f"  Duration Accuracy: {gt.quality.duration_accuracy}")
    print(f"  Gateway Diversity: {gt.quality.gateway_diversity}")
    print(f"  Channel Model: {gt.quality.channel_model_accuracy}")
    print(f"  ADRopt Function: {gt.quality.adropt_functioning}")
    print(f"  Overall Confidence: {gt.quality.overall_confidence}")
    
    if gt.quality.warnings:
        print(f"\n⚠️ WARNINGS:")
        for warning in gt.quality.warnings:
            print(f"  {warning}")
    
    if gt.quality.debug_info:
        print(f"\n✅ DEBUG INFO:")
        for info in gt.quality.debug_info:
            print(f"  {info}")
    
    return gt

def plot_enhanced_performance_analysis(gt: GroundTruthData):
    """Enhanced performance analysis with proper paper terminology."""
    print("\n--- 📉 Enhanced Performance Analysis (PDR/DER) ---")
    
    if not gt.packets_sent_per_device:
        print("⚠️ Skipping: No transmission data available for performance calculation.")
        return

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Enhanced Performance Analysis: PDR and Estimated DER\n(Current Simulation Results)', fontsize=16, fontweight='bold')

    # Calculate network-level metrics
    device_pdrs = {}  # Packet Delivery Rate (network level)
    device_pers = {}  # Packet Error Rate (network level)
    device_estimated_ders = {}  # Estimated Data Error Rate (with FEC)
    
    for device_id, total_sent in gt.packets_sent_per_device.items():
        total_received = gt.packets_received_per_device.get(device_id, 0)
        
        if total_sent > 0:
            pdr = (total_received / total_sent) * 100  # Packet Delivery Rate
            per = 100 - pdr  # Packet Error Rate
            
            # Estimate DER (Data Error Rate) assuming FEC can recover from PER ≤ 30%
            # (Based on paper: "we need to have PER < 0.3" for FEC recovery)
            if per <= 30.0:
                estimated_der = max(0.01, per * 0.1)  # Conservative estimate
            else:
                estimated_der = per  # FEC cannot recover
                
            device_pdrs[device_id] = pdr
            device_pers[device_id] = per
            device_estimated_ders[device_id] = estimated_der
        else:
            device_pdrs[device_id] = 0.0
            device_pers[device_id] = 100.0
            device_estimated_ders[device_id] = 100.0

    dev_ids = list(device_pdrs.keys())
    pdr_values = list(device_pdrs.values())
    per_values = list(device_pers.values())
    der_values = list(device_estimated_ders.values())

    # 1. PDR (Packet Delivery Rate) at network level
    ax1 = axes[0, 0]
    bars = ax1.bar([f'Device {d}' for d in dev_ids], pdr_values, color='skyblue', alpha=0.7)
    ax1.axhline(y=PAPER_TARGET_PDR, color='green', linestyle='--', label=f'High PDR Target ({PAPER_TARGET_PDR}%)')
    ax1.set_title('Packet Delivery Rate (PDR) - Network Level')
    ax1.set_ylabel('PDR (%)')
    ax1.legend()
    ax1.grid(axis='y', alpha=0.3)

    for bar, per in zip(bars, per_values):
        height = bar.get_height()
        color = 'green' if per <= 5.0 else 'orange' if per <= 15.0 else 'red'
        ax1.text(bar.get_x() + bar.get_width()/2., height, f'{height:.1f}%\n(PER:{per:.1f}%)', 
                ha='center', va='bottom', fontweight='bold', color=color)

    # 2. Estimated DER (Data Error Rate) with FEC
    ax2 = axes[0, 1]
    bars = ax2.bar([f'Device {d}' for d in dev_ids], der_values, color='coral', alpha=0.7)
    ax2.axhline(y=PAPER_TARGET_DER, color='green', linestyle='--', label=f'Paper DER Target (<{PAPER_TARGET_DER}%)')
    ax2.set_title('Estimated Data Error Rate (DER) - Application Level with FEC')
    ax2.set_ylabel('DER (%)')
    ax2.legend()
    ax2.grid(axis='y', alpha=0.3)

    for bar, der in zip(bars, der_values):
        height = bar.get_height()
        color = 'green' if der <= PAPER_TARGET_DER else 'orange' if der <= 5.0 else 'red'
        ax2.text(bar.get_x() + bar.get_width()/2., height, f'{height:.2f}%', 
                ha='center', va='bottom', fontweight='bold', color=color)

    # 3. Confidence indicator
    ax3 = axes[1, 0]
    confidence_colors = {'High': 'green', 'Medium': 'orange', 'Low': 'red', 'Very Low': 'darkred'}
    conf_color = confidence_colors.get(gt.quality.overall_confidence, 'gray')
    
    ax3.pie([1], labels=[f'Analysis Confidence\n{gt.quality.overall_confidence}'], 
           colors=[conf_color], autopct='', startangle=90)
    ax3.set_title('Result Confidence Level')
    
    # Add warning if confidence is low
    if gt.quality.overall_confidence in ['Low', 'Very Low']:
        ax3.text(0, -1.5, '⚠️ Limited by\nincomplete CSV export', ha='center', va='center',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))

    # 4. Current simulation status
    ax4 = axes[1, 1]
    ax4.axis('off')
    
    # Get actual CSV data count
    actual_csv_rows = len(gt.radio_data) if gt.radio_data is not None else 0
    expected_measurements = EXPECTED_TOTAL_PACKETS * EXPECTED_GATEWAYS * 0.9
    csv_capture_rate = (actual_csv_rows / expected_measurements * 100) if expected_measurements > 0 else 0
    
    status_text = f"""
📊 CURRENT SIMULATION STATUS:

Expected (Shell Script):
• Duration: {EXPECTED_SIMULATION_DAYS} days (504 periods)
• Packets: ~{EXPECTED_TOTAL_PACKETS} (144s intervals)
• Measurements: ~{expected_measurements:.0f}

Actual Results:
• CSV Rows: {actual_csv_rows}
• CSV Capture: {csv_capture_rate:.1f}%
• Duration: {gt.simulation_duration:.1f} days

Network Performance:
• PDR: {pdr_values[0]:.1f}% (Target: >{PAPER_TARGET_PDR}%)
• PER: {per_values[0]:.1f}%

Estimated App Performance:
• DER: {der_values[0]:.2f}% (Target: <{PAPER_TARGET_DER}%)

✅ Confidence: {gt.quality.overall_confidence}
"""
    
    ax4.text(0.1, 0.9, status_text, transform=ax4.transAxes, fontsize=9,
            verticalalignment='top', bbox=dict(boxstyle='round,pad=0.5', facecolor='lightblue', alpha=0.8))

    plt.tight_layout()
    plt.savefig(os.path.join(PLOT_DIR, 'enhanced_performance_analysis.png'), dpi=150)
    plt.close()
    print("  -> Generated 'enhanced_performance_analysis.png'")

def plot_adropt_evolution(gt: GroundTruthData):
    """Plot ADRopt parameter evolution over time."""
    print("\n--- 🧠 ADRopt Parameter Evolution ---")
    
    if gt.radio_data is None or 'Time' not in gt.radio_data.columns:
        print("⚠️ Skipping: No time-series data available.")
        return

    # Check if we have parameter data
    has_sf = 'SpreadingFactor' in gt.radio_data.columns
    has_power = 'TxPower_dBm' in gt.radio_data.columns
    
    if not has_sf and not has_power:
        print("⚠️ Skipping: No ADR parameter data available.")
        return

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('ADRopt Parameter Evolution Analysis\n(Current Simulation)', fontsize=16, fontweight='bold')

    time_hours = gt.radio_data['Time'] / 3600  # Convert to hours

    # 1. Spreading Factor Evolution
    if has_sf:
        ax1 = axes[0, 0]
        sf_values = gt.radio_data['SpreadingFactor']
        scatter = ax1.scatter(time_hours, sf_values, alpha=0.6, c=sf_values, cmap='viridis')
        ax1.set_title('Spreading Factor Evolution')
        ax1.set_ylabel('Spreading Factor')
        ax1.set_xlabel('Time (hours)')
        ax1.grid(True, alpha=0.3)
        plt.colorbar(scatter, ax=ax1, label='SF Value')
        
        # Add trend line
        if len(time_hours) > 1:
            z = np.polyfit(time_hours, sf_values, 1)
            p = np.poly1d(z)
            ax1.plot(time_hours, p(time_hours), "r--", alpha=0.8, label=f'Trend: {z[0]:.3f}/hour')
            ax1.legend()

    # 2. Transmission Power Evolution
    if has_power:
        ax2 = axes[0, 1]
        power_values = gt.radio_data['TxPower_dBm']
        scatter = ax2.scatter(time_hours, power_values, alpha=0.6, c=power_values, cmap='plasma')
        ax2.set_title('Transmission Power Evolution')
        ax2.set_ylabel('TX Power (dBm)')
        ax2.set_xlabel('Time (hours)')
        ax2.grid(True, alpha=0.3)
        plt.colorbar(scatter, ax=ax2, label='Power (dBm)')
        
        # Add trend line
        if len(time_hours) > 1:
            z = np.polyfit(time_hours, power_values, 1)
            p = np.poly1d(z)
            ax2.plot(time_hours, p(time_hours), "r--", alpha=0.8, label=f'Trend: {z[0]:.3f}dBm/hour')
            ax2.legend()

    # 3. Parameter Distribution
    ax3 = axes[1, 0]
    if has_sf:
        sf_counts = gt.radio_data['SpreadingFactor'].value_counts().sort_index()
        ax3.bar(sf_counts.index, sf_counts.values, alpha=0.7, color='teal')
        ax3.set_title('SF Distribution')
        ax3.set_xlabel('Spreading Factor')
        ax3.set_ylabel('Count')
        ax3.grid(axis='y', alpha=0.3)

    # 4. Energy Efficiency Indicator
    ax4 = axes[1, 1]
    if has_sf and has_power:
        # Calculate relative energy consumption (simplified)
        # Energy ∝ TxPower × ToA, where ToA ∝ 2^SF
        relative_energy = gt.radio_data['TxPower_dBm'] + 10 * np.log10(2 ** gt.radio_data['SpreadingFactor'])
        
        ax4.hist(relative_energy, bins=20, alpha=0.7, color='coral', edgecolor='black')
        ax4.set_title('Relative Energy Consumption Distribution')
        ax4.set_xlabel('Relative Energy (dB)')
        ax4.set_ylabel('Frequency')
        ax4.axvline(relative_energy.mean(), color='red', linestyle='--', 
                   label=f'Mean: {relative_energy.mean():.1f}dB')
        ax4.legend()
        ax4.grid(axis='y', alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(PLOT_DIR, 'adropt_evolution.png'), dpi=150)
    plt.close()
    print("  -> Generated 'adropt_evolution.png'")

def plot_enhanced_fading_validation(gt: GroundTruthData):
    """Enhanced fading analysis with statistical validation."""
    print("\n--- 🌊 Enhanced Fading Model Validation ---")
    
    if gt.radio_data is None or 'Fading_dB' not in gt.radio_data.columns:
        print("⚠️ Skipping: No fading data available.")
        return

    fading_data = gt.radio_data['Fading_dB'].dropna()
    
    if len(fading_data) < 10:
        print("⚠️ Insufficient fading data for statistical analysis.")
        return

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Enhanced Channel Fading Model Validation\n(Current Simulation vs Paper Target)', fontsize=16, fontweight='bold')

    # 1. Distribution with normal overlay
    ax1 = axes[0, 0]
    sns.histplot(fading_data, bins=30, kde=True, ax=ax1, alpha=0.7, color='mediumpurple')
    
    # Overlay theoretical normal distribution
    x = np.linspace(fading_data.min(), fading_data.max(), 100)
    theoretical_normal = stats.norm.pdf(x, 0, EXPECTED_FADING_STD)
    actual_normal = stats.norm.pdf(x, fading_data.mean(), fading_data.std())
    
    ax1_twin = ax1.twinx()
    ax1_twin.plot(x, theoretical_normal, 'r-', label=f'Target: N(0, {EXPECTED_FADING_STD})', linewidth=2)
    ax1_twin.plot(x, actual_normal, 'g--', label=f'Actual: N({fading_data.mean():.2f}, {fading_data.std():.2f})', linewidth=2)
    ax1_twin.legend()
    
    ax1.set_title('Fading Distribution vs Target')
    ax1.set_xlabel('Fading (dB)')

    # 2. Q-Q plot for normality test
    ax2 = axes[0, 1]
    stats.probplot(fading_data, dist="norm", plot=ax2)
    ax2.set_title('Q-Q Plot (Normality Test)')
    ax2.grid(True, alpha=0.3)

    # 3. Statistical test results
    ax3 = axes[1, 0]
    ax3.axis('off')
    
    # Perform statistical tests
    shapiro_stat, shapiro_p = stats.shapiro(fading_data) if len(fading_data) <= 5000 else (None, None)
    ks_stat, ks_p = stats.kstest(fading_data, lambda x: stats.norm.cdf(x, 0, EXPECTED_FADING_STD))
    
    test_results = f"""
Statistical Validation Results:

Descriptive Statistics:
• Mean: {fading_data.mean():.3f} dB (target: 0.0)
• Std Dev: {fading_data.std():.3f} dB (target: {EXPECTED_FADING_STD})
• Min: {fading_data.min():.2f} dB
• Max: {fading_data.max():.2f} dB
• Count: {len(fading_data)} samples

Model Accuracy:
• Std Dev Error: {abs(fading_data.std() - EXPECTED_FADING_STD)/EXPECTED_FADING_STD*100:.1f}%
• Mean Error: {abs(fading_data.mean()):.3f} dB

Expected vs Actual:
• Target: Normal(0, {EXPECTED_FADING_STD})
• Actual: Normal({fading_data.mean():.2f}, {fading_data.std():.2f})

Normality Tests:"""

    if shapiro_stat is not None:
        test_results += f"\n• Shapiro-Wilk: p = {shapiro_p:.6f}"
    test_results += f"\n• KS Test vs Target: p = {ks_p:.6f}"
    
    validation_status = "✅ EXCELLENT" if abs(fading_data.std() - EXPECTED_FADING_STD) < 0.5 else "⚠️ ACCEPTABLE" if abs(fading_data.std() - EXPECTED_FADING_STD) < 1.0 else "❌ POOR"
    test_results += f"\n\nValidation: {validation_status}"
    
    ax3.text(0.05, 0.95, test_results, transform=ax3.transAxes, fontsize=9,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='lightblue', alpha=0.8))

    # 4. Time evolution of fading
    ax4 = axes[1, 1]
    if 'Time' in gt.radio_data.columns:
        time_hours = gt.radio_data['Time'] / 3600
        ax4.scatter(time_hours, fading_data, alpha=0.5, s=10)
        ax4.set_title('Fading Evolution Over Time')
        ax4.set_xlabel('Time (hours)')
        ax4.set_ylabel('Fading (dB)')
        ax4.axhline(y=0, color='red', linestyle='--', alpha=0.7, label='Target Mean (0 dB)')
        ax4.axhline(y=EXPECTED_FADING_STD, color='orange', linestyle='--', alpha=0.7, label=f'+1σ ({EXPECTED_FADING_STD} dB)')
        ax4.axhline(y=-EXPECTED_FADING_STD, color='orange', linestyle='--', alpha=0.7, label=f'-1σ ({-EXPECTED_FADING_STD} dB)')
        ax4.legend()
        ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(PLOT_DIR, 'enhanced_fading_validation.png'), dpi=150)
    plt.close()
    print("  -> Generated 'enhanced_fading_validation.png'")

def plot_gateway_diversity_analysis(gt: GroundTruthData):
    """Comprehensive gateway diversity and performance analysis."""
    print("\n--- 📡 Gateway Diversity Analysis ---")
    
    if gt.radio_data is None or 'GatewayID' not in gt.radio_data.columns:
        print("⚠️ Skipping: No gateway data available.")
        return

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Gateway Diversity and Performance Analysis\n(8 Paper Gateways)', fontsize=16, fontweight='bold')

    # 1. Reception distribution per gateway
    ax1 = axes[0, 0]
    gw_counts = gt.radio_data['GatewayID'].value_counts().sort_index()
    total_receptions = gw_counts.sum()
    extraction_rates = (gw_counts / total_receptions) * 100
    
    bars = ax1.bar(extraction_rates.index, extraction_rates.values, 
                   color=plt.cm.viridis(np.linspace(0, 1, len(extraction_rates))), 
                   edgecolor='black', alpha=0.8)
    ax1.set_title('Gateway Reception Share')
    ax1.set_xlabel('Gateway ID')
    ax1.set_ylabel('Share of Receptions (%)')
    ax1.set_xticks(extraction_rates.index)
    ax1.grid(axis='y', alpha=0.3)

    # Add value labels
    for bar in bars:
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height, f'{height:.1f}%', 
                ha='center', va='bottom', fontweight='bold')

    # 2. SNR distribution per gateway
    ax2 = axes[0, 1]
    if 'SNR_dB' in gt.radio_data.columns:
        gateway_ids = sorted(gt.radio_data['GatewayID'].unique())
        snr_by_gw = [gt.radio_data[gt.radio_data['GatewayID'] == gw]['SNR_dB'].values 
                     for gw in gateway_ids]
        
        bp = ax2.boxplot(snr_by_gw, labels=[f'GW{gw}' for gw in gateway_ids], patch_artist=True)
        
        # Color boxes by performance
        colors = plt.cm.RdYlGn(np.linspace(0.3, 0.9, len(bp['boxes'])))
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
            
        ax2.set_title('SNR Distribution per Gateway')
        ax2.set_ylabel('SNR (dB)')
        ax2.grid(True, alpha=0.3)
        ax2.axhline(y=-6, color='red', linestyle='--', alpha=0.7, label='Typical Threshold')
        ax2.legend()

    # 3. Gateway performance summary with paper mapping
    ax3 = axes[1, 0]
    ax3.axis('off')
    
    # Paper gateway names (from simulation)
    paper_gw_names = {
        0: "GW2 (High SNR)",
        1: "GW5 (High SNR)", 
        2: "GW6 (Medium SNR)",
        3: "GW8 (Medium SNR)",
        4: "GW3 (Low SNR)",
        5: "GW4 (Low SNR)",
        6: "GW_Edge (Urban Edge)",
        7: "GW_Distant (Distant)"
    }
    
    performance_text = "Gateway Performance (Paper Config):\n\n"
    if 'SNR_dB' in gt.radio_data.columns:
        for gw in sorted(gt.radio_data['GatewayID'].unique()):
            gw_data = gt.radio_data[gt.radio_data['GatewayID'] == gw]
            avg_snr = gw_data['SNR_dB'].mean()
            count = len(gw_data)
            share = (count / len(gt.radio_data)) * 100
            paper_name = paper_gw_names.get(gw, f"GW{gw}")
            
            performance_text += f"{gw}: {count:4d} pkts ({share:5.1f}%), SNR: {avg_snr:6.1f}dB\n"
            performance_text += f"    {paper_name}\n\n"
    
    ax3.text(0.05, 0.95, performance_text, transform=ax3.transAxes, fontsize=9,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow', alpha=0.8))

    # 4. Diversity benefit visualization
    ax4 = axes[1, 1]
    if 'Time' in gt.radio_data.columns:
        # Calculate unique gateways per time window
        time_windows = pd.cut(gt.radio_data['Time'], bins=20)
        diversity_per_window = gt.radio_data.groupby(time_windows)['GatewayID'].nunique()
        
        window_centers = [(interval.left + interval.right) / 2 / 3600 for interval in diversity_per_window.index]
        
        ax4.plot(window_centers, diversity_per_window.values, 'o-', linewidth=2, markersize=6, color='blue')
        ax4.set_title('Gateway Diversity Over Time')
        ax4.set_xlabel('Time (hours)')
        ax4.set_ylabel('Active Gateways per Window')
        ax4.set_ylim(0, EXPECTED_GATEWAYS + 1)
        ax4.grid(True, alpha=0.3)
        ax4.axhline(y=EXPECTED_GATEWAYS, color='green', linestyle='--', 
                   label=f'Max ({EXPECTED_GATEWAYS} Paper GWs)')
        ax4.legend()

    plt.tight_layout()
    plt.savefig(os.path.join(PLOT_DIR, 'gateway_diversity_analysis.png'), dpi=150)
    plt.close()
    print("  -> Generated 'gateway_diversity_analysis.png'")

def generate_research_summary(gt: GroundTruthData):
    """Generate comprehensive research summary report."""
    print("\n--- 📋 Generating Research Summary ---")
    
    summary_path = os.path.join(DEBUG_DIR, 'research_summary.txt')
    
    with open(summary_path, 'w') as f:
        f.write("="*80 + "\n")
        f.write("LORAWAN ADROPT SIMULATION - RESEARCH SUMMARY (Current Version)\n")
        f.write("="*80 + "\n\n")
        
        # Simulation overview
        f.write("SIMULATION OVERVIEW:\n")
        f.write("-" * 20 + "\n")
        f.write(f"Data Source: {gt.data_source_name}\n")
        f.write(f"Measurements: {len(gt.radio_data) if gt.radio_data is not None else 0}\n")
        f.write(f"Duration: {gt.simulation_duration:.2f} days (target: {EXPECTED_SIMULATION_DAYS})\n")
        f.write(f"Gateways: {gt.radio_data['GatewayID'].nunique() if gt.radio_data is not None and 'GatewayID' in gt.radio_data.columns else 'Unknown'} (expected: {EXPECTED_GATEWAYS})\n")
        f.write(f"Devices: {gt.radio_data['DeviceAddr'].nunique() if gt.radio_data is not None and 'DeviceAddr' in gt.radio_data.columns else 'Unknown'}\n")
        f.write(f"Expected Packets: ~{EXPECTED_TOTAL_PACKETS} ({EXPECTED_PACKET_INTERVAL}s intervals)\n\n")
        
        # Performance results
        f.write("PERFORMANCE RESULTS:\n")
        f.write("-" * 20 + "\n")
        if gt.packets_sent_per_device:
            for device_id in gt.packets_sent_per_device:
                sent = gt.packets_sent_per_device[device_id]
                received = gt.packets_received_per_device.get(device_id, 0)
                pdr = (received / sent * 100) if sent > 0 else 0
                per = 100 - pdr
                
                # Estimate DER based on paper's FEC capability
                if per <= 30.0:  # Paper: "PER < 0.3" for FEC recovery
                    estimated_der = max(0.01, per * 0.1)  # Conservative estimate
                else:
                    estimated_der = per  # FEC cannot recover
                
                f.write(f"Device {device_id}:\n")
                f.write(f"  Packets Sent: {sent} (expected: ~{EXPECTED_TOTAL_PACKETS})\n")
                f.write(f"  Packets Received: {received}\n")
                f.write(f"  PDR (Packet Delivery Rate): {pdr:.2f}%\n")
                f.write(f"  PER (Packet Error Rate): {per:.2f}%\n") 
                f.write(f"  Estimated DER (Data Error Rate with FEC): {estimated_der:.2f}% (target: <{PAPER_TARGET_DER}%)\n")
                f.write(f"  Performance Gap: {estimated_der - PAPER_TARGET_DER:.2f}% above DER target\n\n")
        
        # Quality assessment
        f.write("QUALITY ASSESSMENT:\n")
        f.write("-" * 20 + "\n")
        f.write(f"Overall Confidence: {gt.quality.overall_confidence}\n")
        f.write(f"Data Completeness: {gt.quality.data_completeness}\n")
        f.write(f"Duration Accuracy: {gt.quality.duration_accuracy}\n")
        f.write(f"Gateway Diversity: {gt.quality.gateway_diversity}\n")
        f.write(f"Channel Model: {gt.quality.channel_model_accuracy}\n")
        f.write(f"ADRopt Function: {gt.quality.adropt_functioning}\n\n")
        
        # Data export analysis
        if gt.radio_data is not None:
            actual_csv_rows = len(gt.radio_data)
            expected_measurements = EXPECTED_TOTAL_PACKETS * EXPECTED_GATEWAYS * 0.9
            csv_capture_rate = (actual_csv_rows / expected_measurements * 100) if expected_measurements > 0 else 0
            
            f.write("DATA EXPORT ANALYSIS:\n")
            f.write("-" * 20 + "\n")
            f.write(f"Expected Measurements: ~{expected_measurements:.0f}\n")
            f.write(f"Actual CSV Rows: {actual_csv_rows}\n")
            f.write(f"CSV Capture Rate: {csv_capture_rate:.1f}%\n")
            
            if csv_capture_rate < 1.0:
                f.write("⚠️ CSV export appears severely incomplete\n")
            elif csv_capture_rate < 10.0:
                f.write("⚠️ CSV export appears limited\n")
            else:
                f.write("✅ CSV export appears adequate\n")
            f.write("\n")
        
        # Warnings and recommendations
        if gt.quality.warnings:
            f.write("WARNINGS:\n")
            f.write("-" * 20 + "\n")
            for warning in gt.quality.warnings:
                f.write(f"  {warning}\n")
            f.write("\n")
        
        # Channel model validation
        if gt.radio_data is not None and 'Fading_dB' in gt.radio_data.columns:
            fading_data = gt.radio_data['Fading_dB'].dropna()
            f.write("CHANNEL MODEL VALIDATION:\n")
            f.write("-" * 20 + "\n")
            f.write(f"Fading Std Dev: {fading_data.std():.3f} dB (target: {EXPECTED_FADING_STD} dB)\n")
            f.write(f"Fading Mean: {fading_data.mean():.3f} dB (target: 0.0 dB)\n")
            f.write(f"Model Accuracy: {abs(fading_data.std() - EXPECTED_FADING_STD)/EXPECTED_FADING_STD*100:.1f}% error\n\n")
        
        # ADRopt analysis
        if gt.radio_data is not None:
            if 'SpreadingFactor' in gt.radio_data.columns:
                sf_range = gt.radio_data['SpreadingFactor'].max() - gt.radio_data['SpreadingFactor'].min()
                f.write("ADROPT PERFORMANCE:\n")
                f.write("-" * 20 + "\n")
                f.write(f"SF Range: {gt.radio_data['SpreadingFactor'].min()}-{gt.radio_data['SpreadingFactor'].max()} (variation: {sf_range})\n")
                
            if 'TxPower_dBm' in gt.radio_data.columns:
                power_range = gt.radio_data['TxPower_dBm'].max() - gt.radio_data['TxPower_dBm'].min()
                f.write(f"Power Range: {gt.radio_data['TxPower_dBm'].min():.1f}-{gt.radio_data['TxPower_dBm'].max():.1f} dBm (variation: {power_range:.1f} dB)\n")
                
                # Energy efficiency estimate
                if 'SpreadingFactor' in gt.radio_data.columns:
                    initial_energy = 14 + 10 * np.log10(2**12)  # Max power + SF12
                    final_energy = gt.radio_data['TxPower_dBm'].iloc[-10:].mean() + 10 * np.log10(2**gt.radio_data['SpreadingFactor'].iloc[-10:].mean())
                    energy_saving = ((initial_energy - final_energy) / initial_energy) * 100
                    f.write(f"Estimated Energy Savings: {energy_saving:.1f}%\n")
        
        # Configuration summary
        f.write("\nCONFIGURATION SUMMARY:\n")
        f.write("-" * 20 + "\n")
        f.write(f"Shell Script Parameters:\n")
        f.write(f"  PERIODS_TO_SIMULATE: 504 (= {EXPECTED_SIMULATION_DAYS} days)\n")
        f.write(f"  N_DEVICES: 1 (single test device)\n")
        f.write(f"  Packet Interval: {EXPECTED_PACKET_INTERVAL}s\n")
        f.write(f"  Expected Total Packets: ~{EXPECTED_TOTAL_PACKETS}\n")
        f.write(f"Simulation C++ Parameters:\n")
        f.write(f"  Gateways: {EXPECTED_GATEWAYS} (paper's exact configuration)\n")
        f.write(f"  Payload: 15 bytes\n")
        f.write(f"  ADRopt: Enabled\n")
        f.write(f"  Fading Model: Rayleigh ({EXPECTED_FADING_STD}dB std dev)\n")
    
    print(f"  -> Generated research summary: {summary_path}")

def validate_data_consistency(gt: GroundTruthData):
    """Validate consistency between simulation parameters and actual CSV data."""
    print("\n--- 🔍 Data Consistency Validation ---")
    
    if gt.radio_data is None:
        print("❌ No radio data loaded - cannot validate")
        return
    
    actual_csv_rows = len(gt.radio_data)
    print(f"📊 Actual CSV rows loaded: {actual_csv_rows}")
    
    # Calculate expected data based on current simulation parameters
    expected_packets = EXPECTED_TOTAL_PACKETS
    expected_measurements = expected_packets * EXPECTED_GATEWAYS * 0.9  # Assume 90% reception rate
    
    print(f"📈 Expected for {EXPECTED_SIMULATION_DAYS} days ({expected_packets} packets × {EXPECTED_GATEWAYS} GWs): ~{expected_measurements:.0f}")
    
    # Check against log claims
    total_sent = sum(gt.packets_sent_per_device.values()) if gt.packets_sent_per_device else 0
    if total_sent > 0:
        print(f"📋 Log/Estimated - Sent: {total_sent}")
        expected_from_packets = total_sent * EXPECTED_GATEWAYS * 0.9
        print(f"📋 Expected measurements from packets: ~{expected_from_packets:.0f}")
        
        ratio = actual_csv_rows / expected_from_packets if expected_from_packets > 0 else 0
        print(f"📊 CSV Data Capture Rate: {ratio*100:.1f}%")
        
        if ratio < 0.001:  # Less than 0.1%
            print("❌ CRITICAL: CSV export appears to be severely incomplete!")
            print("   Possible causes:")
            print("   - CSV export timing/interval issues")
            print("   - File overwriting instead of appending")
            print("   - Export happening only at simulation end")
            print("   - Simulation not completing properly")
        elif ratio < 0.01:  # Less than 1%
            print("❌ SEVERE: CSV export appears severely limited")
        elif ratio < 0.1:  # Less than 10%
            print("⚠️ WARNING: CSV export appears incomplete")
        else:
            print("✅ CSV data appears reasonably complete")
    else:
        print("⚠️ No packet count data available for comparison")
    
    # Check time span in CSV vs expected duration
    if 'Time' in gt.radio_data.columns and len(gt.radio_data) > 1:
        csv_time_span = (gt.radio_data['Time'].max() - gt.radio_data['Time'].min()) / (24 * 3600)
        print(f"📅 CSV time span: {csv_time_span:.2f} days")
        print(f"📅 Expected duration: {EXPECTED_SIMULATION_DAYS} days")
        
        if abs(csv_time_span - EXPECTED_SIMULATION_DAYS) > 1.0:
            print("⚠️ Time span mismatch detected!")
        else:
            print("✅ Time span appears consistent")

def main():
    """Enhanced main function with comprehensive analysis."""
    print("🚀 ENHANCED LORAWAN SIMULATION ANALYZER V8 (CORRECTED)")
    print(f"   Expected: {EXPECTED_SIMULATION_DAYS} days, {EXPECTED_TOTAL_PACKETS} packets, {EXPECTED_GATEWAYS} gateways")
    
    # Create directories
    for directory in [PLOT_DIR, DEBUG_DIR]:
        if not os.path.exists(directory):
            os.makedirs(directory)

    # Establish ground truth with validation
    ground_truth = establish_ground_truth()
    if not ground_truth.validated:
        print("\n❌ Cannot proceed with analysis due to data validation failures.")
        return

    # Add data consistency validation
    validate_data_consistency(ground_truth)

    # Generate all analyses
    print(f"\n🔬 RUNNING COMPREHENSIVE ANALYSIS (Confidence: {ground_truth.quality.overall_confidence})")
    print("-" * 70)
    
    try:
        plot_enhanced_performance_analysis(ground_truth)
        plot_adropt_evolution(ground_truth)
        plot_enhanced_fading_validation(ground_truth)
        plot_gateway_diversity_analysis(ground_truth)
        generate_research_summary(ground_truth)
        
        # Generate original plots for compatibility
        if ground_truth.radio_data is not None and 'Fading_dB' in ground_truth.radio_data.columns:
            plot_fading_distribution(ground_truth)
        
    except Exception as e:
        print(f"❌ Error during analysis: {e}")
        import traceback
        traceback.print_exc()

    print("\n" + "="*70)
    print("✅ ENHANCED ANALYSIS COMPLETE (CORRECTED VERSION)")
    print(f"📁 Plots saved in: {PLOT_DIR}/")
    print(f"📋 Debug info in: {DEBUG_DIR}/")
    print(f"🎯 Overall Confidence: {ground_truth.quality.overall_confidence}")
    print(f"🔧 Aligned with: {EXPECTED_SIMULATION_DAYS} days, {EXPECTED_GATEWAYS} GWs, {EXPECTED_PACKET_INTERVAL}s intervals")
    print("="*70)

def plot_fading_distribution(gt: GroundTruthData):
    """Original fading plot for compatibility."""
    if gt.radio_data is None or 'Fading_dB' not in gt.radio_data.columns:
        return
        
    fading_data = gt.radio_data['Fading_dB'].dropna()
    fading_std = fading_data.std()

    plt.figure(figsize=(12, 7))
    sns.histplot(fading_data, bins=40, kde=True, color='mediumpurple')
    
    title_text = f'Fading Distribution (Calculated Std Dev: {fading_std:.2f} dB, Target: {EXPECTED_FADING_STD} dB)'
    plt.title(title_text, fontsize=16, fontweight='bold')
    plt.xlabel('Fading Value (dB)', fontsize=12)
    plt.ylabel('Frequency', fontsize=12)
    
    annotation_text = f"Expected Std Dev from paper's model: ~{EXPECTED_FADING_STD} dB\nActual: {fading_std:.2f} dB (Error: {abs(fading_std-EXPECTED_FADING_STD)/EXPECTED_FADING_STD*100:.1f}%)"
    plt.text(0.95, 0.95, annotation_text, transform=plt.gca().transAxes,
             fontsize=12, verticalalignment='top', horizontalalignment='right',
             bbox=dict(boxstyle='round,pad=0.5', fc='ivory', alpha=0.8))
             
    plt.axvline(fading_data.mean(), color='black', linestyle='--', label=f'Mean: {fading_data.mean():.2f} dB')
    plt.axvline(0, color='red', linestyle='--', alpha=0.7, label='Target Mean: 0 dB')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.tight_layout()
    plt.savefig(os.path.join(PLOT_DIR, 'fading_distribution.png'), dpi=150)
    plt.close()

if __name__ == "__main__":
    main()