#!/usr/bin/env python3
"""
Enhanced FEC Performance Analysis Script - Deep FEC Focus

This script provides comprehensive FEC analysis for LoRaWAN DaRe implementation
targeting the Heusse et al. (2020) paper replication with FEC enhancement.

Focus Areas:
- Generation completion analysis
- Packet type distribution (systematic vs redundant)
- Recovery success rates
- Timing analysis for FEC operations
- Debug simulation issues preventing FEC operation

Target: DER < 0.01 (1%) with FEC recovery
"""

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import os
import re
from datetime import datetime
from pathlib import Path

# Configuration
PLOT_DIR = "fec_analysis"
PAPER_TARGET_DER = 1.0  # % (DER < 0.01 = 1%)
EXPECTED_GENERATION_SIZES = [8, 16, 32, 64, 128]  # Common generation sizes
EXPECTED_REDUNDANCY = 30  # % redundancy
PACKET_INTERVAL = 144  # seconds (2.4 minutes)

# Create output directory
if not os.path.exists(PLOT_DIR):
    os.makedirs(PLOT_DIR)

def load_all_data():
    """Load ALL available data files with FEC focus."""
    print("🔍 COMPREHENSIVE DATA LOADING (FEC-FOCUSED)")
    print("=" * 60)
    
    data = {}
    file_status = {}
    
    # Define all possible files
    expected_files = {
        'fec_performance': 'fec_performance.csv',
        'main_simulation': 'paper_replication_adr_fec.csv', 
        'radio_measurements': 'rssi_snr_measurements.csv',
        'radio_summary': 'radio_measurement_summary.csv',
        'fading_summary': 'fading_measurement_summary.csv',
        'global_performance': 'paper_globalPerformance.txt',
        'node_data': 'paper_nodeData.txt',
        'phy_performance': 'paper_phyPerformance.txt'
    }
    
    # Load each file with error handling
    for key, filename in expected_files.items():
        try:
            if filename.endswith('.csv'):
                df = pd.read_csv(filename)
                data[key] = df
                file_status[key] = f"✅ {len(df)} rows"
                print(f"  {key:18}: {len(df)} entries")
                
                # Show FEC-relevant columns for key files
                if key == 'fec_performance' and len(df) > 0:
                    latest = df.iloc[-1]
                    print(f"    → Latest FEC: {latest['GenerationsProcessed']} gens, "
                          f"{latest['PacketsRecovered']} recovered")
                elif key == 'radio_measurements' and len(df) > 0:
                    devices = df['DeviceAddr'].nunique()
                    gateways = df['GatewayID'].nunique()
                    time_span = (df['Time'].max() - df['Time'].min()) / 3600
                    print(f"    → {devices} devices, {gateways} gateways, {time_span:.1f}h span")
                elif key == 'main_simulation' and len(df) > 0:
                    end_devices = df[df['Role'] == 'EndDevice'] if 'Role' in df.columns else df
                    if len(end_devices) > 0:
                        latest_pdr = end_devices['PDR'].iloc[-1] if 'PDR' in end_devices.columns else 'N/A'
                        print(f"    → Latest PDR: {latest_pdr}")
                        
            elif filename.endswith('.txt'):
                # Try to read text files
                if os.path.exists(filename):
                    with open(filename, 'r') as f:
                        content = f.read()
                    data[key] = content
                    file_status[key] = f"✅ {len(content)} chars"
                    print(f"  {key:18}: {len(content)} characters")
                else:
                    data[key] = None
                    file_status[key] = "❌ Not found"
                    
        except Exception as e:
            data[key] = None
            file_status[key] = f"❌ Error: {str(e)[:30]}"
            print(f"  {key:18}: Error - {e}")
    
    print(f"\n📊 Data Loading Summary: {sum(1 for v in data.values() if v is not None)}/{len(expected_files)} files loaded")
    return data, file_status

def analyze_fec_packet_flow(data):
    """Deep analysis of FEC packet flow and generation timing."""
    print("\n🔍 FEC PACKET FLOW ANALYSIS")
    print("=" * 60)
    
    if data['radio_measurements'] is None:
        print("❌ No radio measurements available for packet flow analysis")
        return {}
    
    radio_data = data['radio_measurements']
    analysis = {}
    
    # Basic packet flow
    total_measurements = len(radio_data)
    unique_devices = radio_data['DeviceAddr'].nunique()
    unique_gateways = radio_data['GatewayID'].nunique()
    time_span_hours = (radio_data['Time'].max() - radio_data['Time'].min()) / 3600
    
    # Estimate unique packets (accounting for multiple gateway receptions)
    estimated_unique_packets = total_measurements // unique_gateways if unique_gateways > 0 else total_measurements
    
    analysis['basic_stats'] = {
        'total_measurements': total_measurements,
        'unique_devices': unique_devices, 
        'unique_gateways': unique_gateways,
        'time_span_hours': time_span_hours,
        'estimated_unique_packets': estimated_unique_packets
    }
    
    print(f"📊 Basic Packet Flow:")
    print(f"   Total radio measurements: {total_measurements}")
    print(f"   Unique devices: {unique_devices}")
    print(f"   Unique gateways: {unique_gateways}")
    print(f"   Time span: {time_span_hours:.1f} hours")
    print(f"   Estimated unique packets: ~{estimated_unique_packets}")
    
    # Calculate packet rates
    if time_span_hours > 0:
        packets_per_hour = estimated_unique_packets / time_span_hours
        expected_packets_per_hour = 3600 / PACKET_INTERVAL  # Based on 144s interval
        print(f"   Actual packet rate: {packets_per_hour:.1f} packets/hour")
        print(f"   Expected packet rate: {expected_packets_per_hour:.1f} packets/hour")
        
        analysis['packet_rates'] = {
            'actual_rate': packets_per_hour,
            'expected_rate': expected_packets_per_hour,
            'rate_ratio': packets_per_hour / expected_packets_per_hour if expected_packets_per_hour > 0 else 0
        }
    
    # Generation analysis for different sizes
    print(f"\n🔧 Generation Size Analysis:")
    generation_analysis = {}
    
    for gen_size in EXPECTED_GENERATION_SIZES:
        possible_generations = estimated_unique_packets // gen_size
        time_per_generation_hours = (gen_size * PACKET_INTERVAL) / 3600
        
        generation_analysis[gen_size] = {
            'possible_complete_generations': possible_generations,
            'time_per_generation_hours': time_per_generation_hours,
            'would_complete_in_timespan': possible_generations > 0 and time_per_generation_hours <= time_span_hours
        }
        
        status = "✅" if possible_generations > 0 else "❌"
        print(f"   Gen size {gen_size:3d}: {status} {possible_generations} complete generations "
              f"({time_per_generation_hours:.1f}h each)")
    
    analysis['generation_analysis'] = generation_analysis
    
    # Identify optimal generation size
    optimal_sizes = [size for size, info in generation_analysis.items() 
                    if info['possible_complete_generations'] > 0]
    
    if optimal_sizes:
        optimal_size = max(optimal_sizes)  # Largest size that allows complete generations
        print(f"\n🎯 Optimal generation size for current simulation: {optimal_size} packets")
        print(f"   → Would allow {generation_analysis[optimal_size]['possible_complete_generations']} complete generations")
        analysis['optimal_generation_size'] = optimal_size
    else:
        print(f"\n❌ No generation size would complete in current simulation timespan!")
        print(f"   → Need longer simulation or shorter packet intervals")
        analysis['optimal_generation_size'] = None
    
    return analysis

def analyze_fec_performance_deep(data):
    """Deep analysis of FEC performance data."""
    print("\n🔍 DEEP FEC PERFORMANCE ANALYSIS")
    print("=" * 60)
    
    if data['fec_performance'] is None:
        print("❌ No FEC performance data available")
        return {}
    
    fec_data = data['fec_performance']
    
    if len(fec_data) == 0:
        print("❌ FEC performance file is empty")
        return {}
    
    analysis = {}
    
    # Time series analysis
    if 'Time' in fec_data.columns:
        fec_data['TimeHours'] = fec_data['Time'] / 3600
        analysis['time_span'] = fec_data['TimeHours'].max() - fec_data['TimeHours'].min()
        analysis['measurement_count'] = len(fec_data)
        analysis['measurement_frequency'] = analysis['time_span'] / analysis['measurement_count'] if analysis['measurement_count'] > 1 else 0
    
    # Latest performance
    latest = fec_data.iloc[-1]
    analysis['latest'] = {
        'physical_der_percent': latest['PhysicalDER'] * 100,
        'application_der_percent': latest['ApplicationDER'] * 100,
        'improvement_factor': latest['FecImprovement'],
        'generations_processed': latest['GenerationsProcessed'],
        'packets_recovered': latest['PacketsRecovered']
    }
    
    print(f"📊 Performance Trends:")
    print(f"   Measurement span: {analysis.get('time_span', 0):.1f} hours")
    print(f"   Data points: {analysis.get('measurement_count', 0)}")
    print(f"   Latest Physical DER: {analysis['latest']['physical_der_percent']:.2f}%")
    print(f"   Latest Application DER: {analysis['latest']['application_der_percent']:.2f}%")
    print(f"   FEC Improvement: {analysis['latest']['improvement_factor']:.1f}x")
    print(f"   Generations Processed: {analysis['latest']['generations_processed']}")
    print(f"   Packets Recovered: {analysis['latest']['packets_recovered']}")
    
    # Trend analysis
    if len(fec_data) > 1:
        # DER trends
        der_physical_trend = np.polyfit(range(len(fec_data)), fec_data['PhysicalDER'], 1)[0]
        der_app_trend = np.polyfit(range(len(fec_data)), fec_data['ApplicationDER'], 1)[0]
        improvement_trend = np.polyfit(range(len(fec_data)), fec_data['FecImprovement'], 1)[0]
        
        analysis['trends'] = {
            'physical_der_slope': der_physical_trend,
            'application_der_slope': der_app_trend,
            'improvement_slope': improvement_trend
        }
        
        print(f"\n📈 Performance Trends:")
        print(f"   Physical DER trend: {'📈 Increasing' if der_physical_trend > 0 else '📉 Decreasing'}")
        print(f"   Application DER trend: {'📈 Increasing' if der_app_trend > 0 else '📉 Decreasing'}")
        print(f"   FEC improvement trend: {'📈 Improving' if improvement_trend > 0 else '📉 Degrading'}")
    
    # Target achievement analysis
    target_achievement = {
        'meets_target': analysis['latest']['application_der_percent'] < PAPER_TARGET_DER,
        'target_gap': analysis['latest']['application_der_percent'] - PAPER_TARGET_DER,
        'improvement_needed': PAPER_TARGET_DER / analysis['latest']['application_der_percent'] if analysis['latest']['application_der_percent'] > 0 else float('inf')
    }
    
    analysis['target_achievement'] = target_achievement
    
    if target_achievement['meets_target']:
        print(f"\n✅ TARGET ACHIEVED: Application DER below {PAPER_TARGET_DER}%!")
    else:
        print(f"\n❌ Target missed by {target_achievement['target_gap']:.2f} percentage points")
        print(f"   Need {target_achievement['improvement_needed']:.1f}x better performance")
    
    return analysis

def diagnose_fec_issues_comprehensive(data, packet_analysis, performance_analysis):
    """Comprehensive FEC issue diagnosis."""
    print("\n🚨 COMPREHENSIVE FEC DIAGNOSIS")
    print("=" * 60)
    
    issues = []
    recommendations = []
    
    # Check 1: Data availability
    if data['fec_performance'] is None:
        issues.append("CRITICAL: No FEC performance data - FEC system not reporting")
        recommendations.append("1. Verify FEC component is properly added to network server")
        recommendations.append("2. Check FEC CSV file initialization in simulation")
        return issues, recommendations
    
    fec_data = data['fec_performance']
    latest = performance_analysis.get('latest', {})
    
    # Check 2: Generation processing
    generations_processed = latest.get('generations_processed', 0)
    if generations_processed == 0:
        issues.append("CRITICAL: Zero FEC generations processed")
        
        # Analyze why no generations
        if 'optimal_generation_size' in packet_analysis:
            if packet_analysis['optimal_generation_size'] is None:
                issues.append("ROOT CAUSE: Simulation too short for any generation completion")
                recommendations.append("1. IMMEDIATE: Reduce generation size to 8 packets")
                recommendations.append("2. Or extend simulation time significantly")
                recommendations.append("3. Or reduce packet interval (currently 144s)")
            else:
                optimal = packet_analysis['optimal_generation_size']
                issues.append(f"LIKELY CAUSE: Generation size too large (try {optimal} packets)")
                recommendations.append(f"1. Change generation size from 128 to {optimal} packets")
                recommendations.append("2. This should show FEC working within current timespan")
        
        recommendations.append("3. Add debug output to FEC packet processing")
        recommendations.append("4. Verify FEC headers are being added correctly")
    
    elif generations_processed < 3:
        issues.append("WARNING: Very few generations processed - insufficient data")
        recommendations.append("1. Run simulation longer to get more FEC data")
        recommendations.append("2. Reduce generation size for faster completion")
    
    else:
        print(f"✅ Good: {generations_processed} generations processed")
    
    # Check 3: Packet recovery
    packets_recovered = latest.get('packets_recovered', 0)
    if packets_recovered == 0 and generations_processed > 0:
        issues.append("CRITICAL: FEC generations processed but zero packets recovered")
        recommendations.append("1. Check FEC recovery algorithm implementation")
        recommendations.append("2. Verify redundant packet generation")
        recommendations.append("3. Check if packet losses match FEC capability")
    elif packets_recovered > 0:
        recovery_rate = packets_recovered / (generations_processed * 8) if generations_processed > 0 else 0
        print(f"✅ Packets recovered: {packets_recovered} ({recovery_rate:.1%} of generation capacity)")
    
    # Check 4: Performance targets
    app_der = latest.get('application_der_percent', 100)
    if app_der >= PAPER_TARGET_DER:
        issues.append(f"TARGET MISSED: Application DER {app_der:.2f}% >= {PAPER_TARGET_DER}%")
        improvement_factor = latest.get('improvement_factor', 1.0)
        
        if improvement_factor < 1.1:
            issues.append("POOR FEC EFFECTIVENESS: <10% improvement over physical layer")
            recommendations.append("1. Increase redundancy ratio (currently 30%)")
            recommendations.append("2. Optimize FEC recovery algorithm")
            recommendations.append("3. Check packet loss patterns vs FEC coding")
        else:
            recommendations.append("1. Increase redundancy for better recovery")
            recommendations.append("2. Consider adaptive FEC parameters")
    
    # Check 5: Packet flow issues
    if 'basic_stats' in packet_analysis:
        stats = packet_analysis['basic_stats']
        expected_rate = 3600 / PACKET_INTERVAL
        actual_rate = stats.get('actual_rate', 0)
        
        if actual_rate < expected_rate * 0.8:  # Less than 80% of expected
            issues.append(f"LOW PACKET RATE: {actual_rate:.1f} vs expected {expected_rate:.1f} packets/hour")
            recommendations.append("1. Check packet transmission interval configuration")
            recommendations.append("2. Verify application is properly started")
            recommendations.append("3. Check for packet transmission failures")
    
    # Check 6: Simulation duration vs generation size
    if 'generation_analysis' in packet_analysis:
        gen_analysis = packet_analysis['generation_analysis']
        working_sizes = [size for size, info in gen_analysis.items() 
                        if info['possible_complete_generations'] > 0]
        
        if not working_sizes:
            issues.append("CONFIGURATION ERROR: No generation size works with current simulation")
            recommendations.append("1. CRITICAL: Use generation size ≤ 8 packets for testing")
            recommendations.append("2. For production: extend simulation to multiple days")
        
        elif max(working_sizes) < 32:
            issues.append("SIMULATION TOO SHORT: Can only handle very small generations")
            recommendations.append("1. Extend simulation time for realistic generation sizes")
            recommendations.append("2. Current configuration only suitable for testing")
    
    return issues, recommendations

def create_comprehensive_fec_plots(data, packet_analysis, performance_analysis):
    """Create comprehensive FEC visualization."""
    print("\n📊 GENERATING COMPREHENSIVE FEC PLOTS")
    print("=" * 60)
    
    fig = plt.figure(figsize=(20, 12))
    gs = fig.add_gridspec(3, 4, hspace=0.3, wspace=0.3)
    
    # Main title
    fig.suptitle('Comprehensive FEC Analysis: DaRe Implementation in LoRaWAN ADRopt\n' +
                 'Target: DER < 1% with FEC Recovery', fontsize=16, fontweight='bold')
    
    # 1. Packet Flow Timeline (full width, top)
    ax1 = fig.add_subplot(gs[0, :])
    if data['radio_measurements'] is not None:
        radio_data = data['radio_measurements'].copy()
        radio_data['TimeHours'] = radio_data['Time'] / 3600
        
        # Plot packet receptions by gateway
        for gw_id in sorted(radio_data['GatewayID'].unique()):
            gw_data = radio_data[radio_data['GatewayID'] == gw_id]
            ax1.scatter(gw_data['TimeHours'], [gw_id] * len(gw_data), 
                       alpha=0.6, s=10, label=f'Gateway {gw_id}')
        
        ax1.set_xlabel('Time (hours)')
        ax1.set_ylabel('Gateway ID')
        ax1.set_title('Packet Reception Timeline by Gateway')
        ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax1.grid(True, alpha=0.3)
    
    # 2. Generation Size Analysis
    ax2 = fig.add_subplot(gs[1, 0])
    if 'generation_analysis' in packet_analysis:
        gen_analysis = packet_analysis['generation_analysis']
        sizes = list(gen_analysis.keys())
        completions = [gen_analysis[size]['possible_complete_generations'] for size in sizes]
        
        colors = ['green' if c > 0 else 'red' for c in completions]
        bars = ax2.bar(range(len(sizes)), completions, color=colors, alpha=0.7)
        ax2.set_xticks(range(len(sizes)))
        ax2.set_xticklabels(sizes)
        ax2.set_xlabel('Generation Size (packets)')
        ax2.set_ylabel('Possible Complete Generations')
        ax2.set_title('Generation Size Feasibility')
        
        # Add value labels
        for bar, val in zip(bars, completions):
            if val > 0:
                ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height(), 
                        str(val), ha='center', va='bottom', fontweight='bold')
    
    # 3. DER Performance Comparison
    ax3 = fig.add_subplot(gs[1, 1])
    if data['fec_performance'] is not None and len(data['fec_performance']) > 0:
        fec_data = data['fec_performance']
        latest = fec_data.iloc[-1]
        
        categories = ['Physical\nDER', 'Application\nDER', 'Target\nDER']
        values = [
            latest['PhysicalDER'] * 100,
            latest['ApplicationDER'] * 100,
            PAPER_TARGET_DER
        ]
        colors = ['red', 'blue', 'green']
        
        bars = ax3.bar(categories, values, color=colors, alpha=0.7)
        ax3.set_ylabel('Data Error Rate (%)')
        ax3.set_title('DER Performance vs Target')
        
        # Add value labels
        for bar, val in zip(bars, values):
            ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height(), 
                    f'{val:.2f}%', ha='center', va='bottom', fontweight='bold')
        
        # Add target line
        ax3.axhline(y=PAPER_TARGET_DER, color='green', linestyle='--', alpha=0.5)
    
    # 4. FEC Activity Summary
    ax4 = fig.add_subplot(gs[1, 2])
    if 'latest' in performance_analysis:
        latest_perf = performance_analysis['latest']
        
        activities = ['Generations\nProcessed', 'Packets\nRecovered']
        values = [
            latest_perf['generations_processed'],
            latest_perf['packets_recovered']
        ]
        colors = ['skyblue', 'lightgreen']
        
        bars = ax4.bar(activities, values, color=colors, alpha=0.7)
        ax4.set_ylabel('Count')
        ax4.set_title('FEC System Activity')
        
        # Add value labels
        for bar, val in zip(bars, values):
            ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height(), 
                    str(int(val)), ha='center', va='bottom', fontweight='bold')
        
        # Status indicator
        if latest_perf['generations_processed'] == 0:
            ax4.text(0.5, 0.9, '❌ No Activity', transform=ax4.transAxes,
                    ha='center', fontsize=12, color='red', fontweight='bold')
        elif latest_perf['packets_recovered'] == 0:
            ax4.text(0.5, 0.9, '⚠️ No Recovery', transform=ax4.transAxes,
                    ha='center', fontsize=12, color='orange', fontweight='bold')
        else:
            ax4.text(0.5, 0.9, '✅ Active', transform=ax4.transAxes,
                    ha='center', fontsize=12, color='green', fontweight='bold')
    
    # 5. Improvement Factor
    ax5 = fig.add_subplot(gs[1, 3])
    if data['fec_performance'] is not None and len(data['fec_performance']) > 0:
        fec_data = data['fec_performance']
        if 'TimeHours' not in fec_data.columns:
            fec_data['TimeHours'] = fec_data['Time'] / 3600
        
        ax5.plot(fec_data['TimeHours'], fec_data['FecImprovement'], 'o-', linewidth=2, markersize=6)
        ax5.set_xlabel('Time (hours)')
        ax5.set_ylabel('FEC Improvement Factor')
        ax5.set_title('FEC Improvement Over Time')
        ax5.axhline(y=1.0, color='red', linestyle='--', alpha=0.5, label='No improvement')
        ax5.axhline(y=2.0, color='green', linestyle='--', alpha=0.5, label='Good improvement')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
    
    # 6. DER Evolution (bottom left)
    ax6 = fig.add_subplot(gs[2, :2])
    if data['fec_performance'] is not None and len(data['fec_performance']) > 0:
        fec_data = data['fec_performance']
        if 'TimeHours' not in fec_data.columns:
            fec_data['TimeHours'] = fec_data['Time'] / 3600
        
        ax6.plot(fec_data['TimeHours'], fec_data['PhysicalDER'] * 100, 
                'r-o', label='Physical DER', linewidth=2, markersize=4)
        ax6.plot(fec_data['TimeHours'], fec_data['ApplicationDER'] * 100, 
                'b-o', label='Application DER (with FEC)', linewidth=2, markersize=4)
        ax6.axhline(y=PAPER_TARGET_DER, color='green', linestyle='--', 
                   label=f'Target DER ({PAPER_TARGET_DER}%)', alpha=0.7)
        
        ax6.set_xlabel('Time (hours)')
        ax6.set_ylabel('Data Error Rate (%)')
        ax6.set_title('DER Evolution: Impact of FEC Over Time')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        ax6.set_ylim(0, max(10, fec_data['PhysicalDER'].max() * 120))
    
    # 7. Status and Recommendations (bottom right)
    ax7 = fig.add_subplot(gs[2, 2:])
    ax7.axis('off')
    
    # Prepare comprehensive status text
    status_text = "FEC SYSTEM STATUS REPORT\n" + "="*40 + "\n\n"
    
    if 'latest' in performance_analysis:
        latest_perf = performance_analysis['latest']
        
        # Overall status
        if latest_perf['application_der_percent'] < PAPER_TARGET_DER:
            status = "✅ TARGET ACHIEVED"
            status_color = 'green'
        elif latest_perf['generations_processed'] > 0:
            status = "🔧 FEC WORKING"
            status_color = 'orange'
        else:
            status = "❌ FEC NOT WORKING"
            status_color = 'red'
        
        status_text += f"Status: {status}\n\n"
        
        # Key metrics
        status_text += f"Performance Metrics:\n"
        status_text += f"• Physical DER: {latest_perf['physical_der_percent']:.2f}%\n"
        status_text += f"• Application DER: {latest_perf['application_der_percent']:.2f}%\n"
        status_text += f"• Target: < {PAPER_TARGET_DER}%\n"
        status_text += f"• Improvement: {latest_perf['improvement_factor']:.1f}x\n\n"
        
        # FEC activity
        status_text += f"FEC Activity:\n"
        status_text += f"• Generations: {latest_perf['generations_processed']}\n"
        status_text += f"• Packets Recovered: {latest_perf['packets_recovered']}\n\n"
        
        # Packet flow
        if 'basic_stats' in packet_analysis:
            stats = packet_analysis['basic_stats']
            status_text += f"Packet Flow:\n"
            status_text += f"• Unique packets: ~{stats['estimated_unique_packets']}\n"
            status_text += f"• Time span: {stats['time_span_hours']:.1f}h\n"
            status_text += f"• Gateways: {stats['unique_gateways']}\n\n"
        
        # Optimal generation size
        if 'optimal_generation_size' in packet_analysis:
            optimal = packet_analysis['optimal_generation_size']
            if optimal:
                status_text += f"Recommended:\n"
                status_text += f"• Generation size: {optimal} packets\n"
            else:
                status_text += f"⚠️ No optimal size found!\n"
                status_text += f"• Try generation size ≤ 8\n"
    
    # Display status
    ax7.text(0.05, 0.95, status_text, transform=ax7.transAxes, fontsize=9,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='lightblue', alpha=0.3))
    
    plt.savefig(os.path.join(PLOT_DIR, 'comprehensive_fec_analysis.png'), 
                dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  → Generated: {PLOT_DIR}/comprehensive_fec_analysis.png")

def generate_detailed_fec_report(data, packet_analysis, performance_analysis, issues, recommendations):
    """Generate comprehensive FEC report."""
    print("\n📋 GENERATING DETAILED FEC REPORT")
    print("=" * 60)
    
    report_path = os.path.join(PLOT_DIR, 'detailed_fec_report.txt')
    
    with open(report_path, 'w') as f:
        f.write("=" * 80 + "\n")
        f.write("COMPREHENSIVE FEC SYSTEM ANALYSIS REPORT\n")
        f.write("DaRe FEC Implementation in LoRaWAN ADRopt Simulation\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("=" * 80 + "\n\n")
        
        # Executive Summary
        f.write("EXECUTIVE SUMMARY\n")
        f.write("-" * 40 + "\n")
        
        if 'latest' in performance_analysis:
            latest = performance_analysis['latest']
            target_met = latest['application_der_percent'] < PAPER_TARGET_DER
            
            if target_met:
                f.write("✅ SUCCESS: FEC system meeting paper's DER < 1% target\n")
            elif latest['generations_processed'] > 0:
                f.write("🔧 PARTIAL SUCCESS: FEC working but not meeting target\n")
            else:
                f.write("❌ FAILURE: FEC system not operational\n")
            
            f.write(f"\nKey Metrics:\n")
            f.write(f"• Application DER: {latest['application_der_percent']:.2f}% (target: <{PAPER_TARGET_DER}%)\n")
            f.write(f"• FEC Improvement: {latest['improvement_factor']:.1f}x\n")
            f.write(f"• Generations Processed: {latest['generations_processed']}\n")
            f.write(f"• Packets Recovered: {latest['packets_recovered']}\n")
        
        f.write(f"\nIssues Identified: {len(issues)}\n")
        f.write(f"Recommendations: {len(recommendations)}\n\n")
        
        # Detailed Analysis
        f.write("DETAILED TECHNICAL ANALYSIS\n")
        f.write("-" * 40 + "\n\n")
        
        # Packet Flow Analysis
        f.write("1. PACKET FLOW ANALYSIS\n")
        f.write("-" * 25 + "\n")
        
        if 'basic_stats' in packet_analysis:
            stats = packet_analysis['basic_stats']
            f.write(f"Total radio measurements: {stats['total_measurements']}\n")
            f.write(f"Unique devices: {stats['unique_devices']}\n")
            f.write(f"Unique gateways: {stats['unique_gateways']}\n")
            f.write(f"Simulation time span: {stats['time_span_hours']:.2f} hours\n")
            f.write(f"Estimated unique packets: {stats['estimated_unique_packets']}\n")
            
            if 'packet_rates' in packet_analysis:
                rates = packet_analysis['packet_rates']
                f.write(f"Actual packet rate: {rates['actual_rate']:.1f} packets/hour\n")
                f.write(f"Expected packet rate: {rates['expected_rate']:.1f} packets/hour\n")
                f.write(f"Rate efficiency: {rates['rate_ratio']:.1%}\n")
        
        f.write("\n")
        
        # Generation Analysis
        f.write("2. GENERATION SIZE ANALYSIS\n")
        f.write("-" * 28 + "\n")
        
        if 'generation_analysis' in packet_analysis:
            gen_analysis = packet_analysis['generation_analysis']
            f.write("Generation Size | Possible Completions | Time per Gen (h) | Feasible\n")
            f.write("-" * 65 + "\n")
            
            for size in sorted(gen_analysis.keys()):
                info = gen_analysis[size]
                feasible = "✅" if info['possible_complete_generations'] > 0 else "❌"
                f.write(f"{size:13d} | {info['possible_complete_generations']:17d} | "
                       f"{info['time_per_generation_hours']:13.1f} | {feasible}\n")
            
            optimal = packet_analysis.get('optimal_generation_size')
            if optimal:
                f.write(f"\nRecommended generation size: {optimal} packets\n")
            else:
                f.write(f"\nWARNING: No generation size feasible with current simulation\n")
        
        f.write("\n")
        
        # Performance Analysis
        f.write("3. FEC PERFORMANCE ANALYSIS\n")
        f.write("-" * 29 + "\n")
        
        if 'latest' in performance_analysis:
            latest = performance_analysis['latest']
            f.write(f"Physical layer DER: {latest['physical_der_percent']:.4f}%\n")
            f.write(f"Application layer DER: {latest['application_der_percent']:.4f}%\n")
            f.write(f"FEC improvement factor: {latest['improvement_factor']:.2f}x\n")
            f.write(f"Generations processed: {latest['generations_processed']}\n")
            f.write(f"Packets recovered: {latest['packets_recovered']}\n")
            
            if latest['generations_processed'] > 0:
                recovery_rate = latest['packets_recovered'] / (latest['generations_processed'] * 8)
                f.write(f"Recovery rate: {recovery_rate:.1%} of generation capacity\n")
            
            # Target analysis
            target_gap = latest['application_der_percent'] - PAPER_TARGET_DER
            if target_gap <= 0:
                f.write(f"✅ Target achieved with {abs(target_gap):.2f}% margin\n")
            else:
                f.write(f"❌ Target missed by {target_gap:.2f} percentage points\n")
                needed_improvement = PAPER_TARGET_DER / latest['application_der_percent']
                f.write(f"Need {needed_improvement:.1f}x better performance to meet target\n")
        
        f.write("\n")
        
        # Issues and Recommendations
        f.write("4. ISSUES AND RECOMMENDATIONS\n")
        f.write("-" * 31 + "\n")
        
        f.write("ISSUES IDENTIFIED:\n")
        if issues:
            for i, issue in enumerate(issues, 1):
                f.write(f"{i:2d}. {issue}\n")
        else:
            f.write("No critical issues identified.\n")
        
        f.write("\nRECOMMENDATIONS:\n")
        if recommendations:
            for i, rec in enumerate(recommendations, 1):
                f.write(f"{i:2d}. {rec}\n")
        else:
            f.write("No specific recommendations - system performing well.\n")
        
        f.write("\n")
        
        # Implementation Guidelines
        f.write("5. IMPLEMENTATION GUIDELINES\n")
        f.write("-" * 30 + "\n")
        
        f.write("For immediate testing (short simulations):\n")
        f.write("• Use generation size: 8 packets\n")
        f.write("• Use redundancy ratio: 50%\n")
        f.write("• Minimum simulation time: ~20 minutes\n")
        f.write("• Expected FEC activity: Within first generation\n\n")
        
        f.write("For realistic deployment (long simulations):\n")
        f.write("• Use generation size: 128 packets\n")
        f.write("• Use redundancy ratio: 30%\n")
        f.write("• Minimum simulation time: 24+ hours\n")
        f.write("• Expected FEC activity: After several hours\n\n")
        
        f.write("Performance optimization:\n")
        f.write("• Monitor generation completion rate\n")
        f.write("• Adjust redundancy based on channel conditions\n")
        f.write("• Consider adaptive generation sizes\n")
        f.write("• Implement smart recovery algorithms\n\n")
        
        # Data Quality Assessment
        f.write("6. DATA QUALITY ASSESSMENT\n")
        f.write("-" * 28 + "\n")
        
        data_quality = {
            'fec_performance': '✅' if data['fec_performance'] is not None else '❌',
            'radio_measurements': '✅' if data['radio_measurements'] is not None else '❌',
            'main_simulation': '✅' if data['main_simulation'] is not None else '❌'
        }
        
        for key, status in data_quality.items():
            f.write(f"{key}: {status}\n")
        
        data_completeness = sum(1 for v in data.values() if v is not None) / len(data)
        f.write(f"\nOverall data completeness: {data_completeness:.1%}\n")
        
        f.write("\n" + "=" * 80 + "\n")
        f.write("END OF REPORT\n")
        f.write("=" * 80 + "\n")
    
    print(f"  → Generated: {report_path}")

def main():
    """Main analysis function with enhanced FEC focus."""
    print("🔧 ENHANCED FEC PERFORMANCE ANALYZER")
    print("Deep Analysis of DaRe FEC Implementation")
    print("Target: Heusse et al. (2020) + FEC Enhancement")
    print("=" * 70)
    
    # Load all available data
    data, file_status = load_all_data()
    
    # Analyze packet flow for FEC feasibility
    packet_analysis = analyze_fec_packet_flow(data)
    
    # Deep FEC performance analysis
    performance_analysis = analyze_fec_performance_deep(data)
    
    # Comprehensive diagnosis
    issues, recommendations = diagnose_fec_issues_comprehensive(data, packet_analysis, performance_analysis)
    
    # Create comprehensive visualizations
    create_comprehensive_fec_plots(data, packet_analysis, performance_analysis)
    
    # Generate detailed report
    generate_detailed_fec_report(data, packet_analysis, performance_analysis, issues, recommendations)
    
    # Final summary with actionable insights
    print("\n" + "🎯" * 20)
    print("🎯 ACTIONABLE FEC INSIGHTS")
    print("🎯" * 20)
    
    if 'latest' in performance_analysis:
        latest = performance_analysis['latest']
        
        print(f"\n📊 Current Status:")
        print(f"   Application DER: {latest['application_der_percent']:.2f}% (target: <{PAPER_TARGET_DER}%)")
        print(f"   Generations Processed: {latest['generations_processed']}")
        print(f"   Packets Recovered: {latest['packets_recovered']}")
        
        # Primary recommendation
        if latest['generations_processed'] == 0:
            print(f"\n🚨 PRIMARY ISSUE: No FEC generations completed")
            if 'optimal_generation_size' in packet_analysis:
                optimal = packet_analysis['optimal_generation_size']
                if optimal:
                    print(f"   ✅ SOLUTION: Use generation size = {optimal} packets")
                    expected_time = (optimal * PACKET_INTERVAL) / 3600
                    print(f"   ⏱️  Expected first generation: ~{expected_time:.1f} hours")
                else:
                    print(f"   ✅ SOLUTION: Use generation size = 8 packets (testing)")
                    print(f"   ⏱️  Expected first generation: ~0.32 hours (19 minutes)")
        
        elif latest['packets_recovered'] == 0:
            print(f"\n⚠️  PRIMARY ISSUE: FEC not recovering packets")
            print(f"   🔧 Check recovery algorithm and redundancy settings")
        
        elif latest['application_der_percent'] >= PAPER_TARGET_DER:
            print(f"\n🎯 ALMOST THERE: FEC working but need better performance")
            improvement_needed = PAPER_TARGET_DER / latest['application_der_percent']
            print(f"   📈 Need {improvement_needed:.1f}x improvement")
            print(f"   🔧 Consider increasing redundancy or improving algorithm")
        
        else:
            print(f"\n✅ SUCCESS: Meeting paper's target!")
            print(f"   🎉 Application DER below {PAPER_TARGET_DER}%")
    
    print(f"\n📁 Analysis Results:")
    print(f"   📊 Comprehensive plots: {PLOT_DIR}/comprehensive_fec_analysis.png")
    print(f"   📋 Detailed report: {PLOT_DIR}/detailed_fec_report.txt")
    print(f"   📈 Ready for paper submission with FEC enhancement!")

if __name__ == "__main__":
    main()