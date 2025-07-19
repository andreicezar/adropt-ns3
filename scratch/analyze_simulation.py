#!/usr/bin/env python3
"""
Enhanced LoRaWAN Simulation Analyzer
Analyzes simulation data including RSSI/SNR measurements for paper replication
"""

import csv
import os
import re
import statistics
from collections import defaultdict, Counter

def read_csv_file(filename):
    """Read CSV file and extract raw data"""
    if not os.path.exists(filename):
        return None
    
    data = []
    try:
        with open(filename, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                data.append(dict(row))
    except Exception as e:
        print(f"Error reading {filename}: {e}")
        return None
    return data

def read_text_file(filename):
    """Read text file and return content"""
    if not os.path.exists(filename):
        return None
    
    try:
        with open(filename, 'r') as file:
            return file.read()
    except:
        return None

def analyze_rssi_snr_measurements(filename):
    """Analyze detailed RSSI/SNR measurements"""
    data = read_csv_file(filename)
    if not data:
        return {}
    
    analysis = {
        'total_measurements': len(data),
        'devices': {},
        'gateways': {},
        'time_span': 0,
        'rssi_stats': {},
        'snr_stats': {},
        'gateway_diversity': {}
    }
    
    timestamps = []
    all_rssi = []
    all_snr = []
    device_gateways = defaultdict(set)
    
    for row in data:
        try:
            time_val = float(row.get('Time', 0))
            device_addr = row.get('DeviceAddr', '')
            gateway_id = row.get('GatewayID', '')
            rssi = float(row.get('RSSI_dBm', -999))
            snr = float(row.get('SNR_dB', -999))
            sf = int(row.get('SpreadingFactor', 12))
            tx_power = float(row.get('TxPower_dBm', 14))
            position = row.get('GatewayPosition', '').strip('"')
            
            timestamps.append(time_val)
            if rssi != -999:
                all_rssi.append(rssi)
            if snr != -999:
                all_snr.append(snr)
            
            # Track per device
            if device_addr not in analysis['devices']:
                analysis['devices'][device_addr] = {
                    'measurements': 0,
                    'rssi_values': [],
                    'snr_values': [],
                    'sf_values': [],
                    'tx_power_values': [],
                    'gateways_used': set()
                }
            
            dev_data = analysis['devices'][device_addr]
            dev_data['measurements'] += 1
            if rssi != -999:
                dev_data['rssi_values'].append(rssi)
            if snr != -999:
                dev_data['snr_values'].append(snr)
            dev_data['sf_values'].append(sf)
            dev_data['tx_power_values'].append(tx_power)
            dev_data['gateways_used'].add(gateway_id)
            device_gateways[device_addr].add(gateway_id)
            
            # Track per gateway
            if gateway_id not in analysis['gateways']:
                analysis['gateways'][gateway_id] = {
                    'measurements': 0,
                    'rssi_values': [],
                    'snr_values': [],
                    'position': position,
                    'devices_served': set()
                }
            
            gw_data = analysis['gateways'][gateway_id]
            gw_data['measurements'] += 1
            if rssi != -999:
                gw_data['rssi_values'].append(rssi)
            if snr != -999:
                gw_data['snr_values'].append(snr)
            gw_data['devices_served'].add(device_addr)
            
        except (ValueError, KeyError) as e:
            continue
    
    # Calculate statistics
    if timestamps:
        analysis['time_span'] = max(timestamps) - min(timestamps)
        analysis['first_time'] = min(timestamps)
        analysis['last_time'] = max(timestamps)
    
    if all_rssi:
        analysis['rssi_stats'] = {
            'min': min(all_rssi),
            'max': max(all_rssi),
            'mean': statistics.mean(all_rssi),
            'median': statistics.median(all_rssi),
            'stdev': statistics.stdev(all_rssi) if len(all_rssi) > 1 else 0
        }
    
    if all_snr:
        analysis['snr_stats'] = {
            'min': min(all_snr),
            'max': max(all_snr),
            'mean': statistics.mean(all_snr),
            'median': statistics.median(all_snr),
            'stdev': statistics.stdev(all_snr) if len(all_snr) > 1 else 0
        }
    
    # Gateway diversity per device
    for device_addr, gateways in device_gateways.items():
        analysis['gateway_diversity'][device_addr] = len(gateways)
    
    return analysis

def analyze_radio_summary(filename):
    """Analyze radio measurement summary file"""
    data = read_csv_file(filename)
    if not data:
        return {}
    
    analysis = {
        'devices_count': len(data),
        'total_measurements': 0,
        'devices': {}
    }
    
    for row in data:
        try:
            device_addr = row.get('DeviceAddr', '')
            measurement_count = int(row.get('MeasurementCount', 0))
            avg_rssi = float(row.get('AvgRSSI_dBm', -999))
            min_rssi = float(row.get('MinRSSI_dBm', -999))
            max_rssi = float(row.get('MaxRSSI_dBm', -999))
            avg_snr = float(row.get('AvgSNR_dB', -999))
            min_snr = float(row.get('MinSNR_dB', -999))
            max_snr = float(row.get('MaxSNR_dB', -999))
            rssi_stdev = float(row.get('RSSIStdDev', 0))
            snr_stdev = float(row.get('SNRStdDev', 0))
            
            analysis['total_measurements'] += measurement_count
            
            analysis['devices'][device_addr] = {
                'measurement_count': measurement_count,
                'avg_rssi': avg_rssi,
                'rssi_range': [min_rssi, max_rssi],
                'avg_snr': avg_snr,
                'snr_range': [min_snr, max_snr],
                'rssi_stdev': rssi_stdev,
                'snr_stdev': snr_stdev
            }
            
        except (ValueError, KeyError):
            continue
    
    return analysis

def analyze_output_file(filename):
    """Extract data from simulation output file"""
    content = read_text_file(filename)
    if not content:
        return {}
    
    data = {}
    
    # Extract final results
    patterns = {
        'total_sent': r'Total packets transmitted: (\d+)',
        'total_received': r'Total packets received: (\d+)', 
        'pdr_percent': r'Packet Delivery Rate \(PDR\): ([\d.]+)%',
        'der_value': r'Data Error Rate \(DER\): ([\d.]+)',
        'der_percent': r'\(([\d.]+)% success\)',
        'devices_count': r'Created paper\'s single test device',
        'gateways_count': r'Deployed (\d+) gateways',
        'simulation_time': r'Duration: ([\d.]+) seconds',
        'expected_packets': r'Expected packets: (\d+)',
        'rssi_measurements': r'(\d+) RSSI/SNR measurements',
        'gateway_diversity': r'(\d+) gateways',
        'avg_snr': r'Average SNR: ([\d.-]+) dB'
    }
    
    for key, pattern in patterns.items():
        matches = re.findall(pattern, content)
        if matches:
            try:
                if key == 'devices_count':
                    data[key] = 1  # Single device from paper
                else:
                    data[key] = float(matches[-1]) if '.' in matches[-1] else int(matches[-1])
            except:
                data[key] = matches[-1]
    
    # Check for performance assessment
    if 'EXCELLENT' in content:
        data['performance_assessment'] = 'EXCELLENT'
    elif 'GOOD' in content:
        data['performance_assessment'] = 'GOOD'
    elif 'ACCEPTABLE' in content:
        data['performance_assessment'] = 'ACCEPTABLE'
    elif 'POOR' in content:
        data['performance_assessment'] = 'POOR'
    
    # Check for radio measurement validation
    if 'Radio measurements will be saved' in content:
        data['radio_measurements_enabled'] = True
    
    return data

def analyze_node_data_file(filename):
    """Extract data from nodeData file"""
    content = read_text_file(filename)
    if not content:
        return {}
    
    data = {}
    lines = content.split('\n')
    
    # Count total lines
    data['total_lines'] = len([l for l in lines if l.strip()])
    
    # Extract time stamps
    timestamps = []
    for line in lines:
        time_match = re.search(r'^([\d.]+)', line.strip())
        if time_match:
            timestamps.append(float(time_match.group(1)))
    
    if timestamps:
        data['first_timestamp'] = min(timestamps)
        data['last_timestamp'] = max(timestamps)
        data['timestamps_count'] = len(timestamps)
        data['time_span'] = max(timestamps) - min(timestamps)
    
    return data

def analyze_phy_performance_file(filename):
    """Extract data from phyPerformance file"""
    content = read_text_file(filename)
    if not content:
        return {}
    
    data = {}
    lines = content.split('\n')
    
    # Count entries
    data['total_lines'] = len([l for l in lines if l.strip()])
    
    # Extract gateway mentions
    gateway_mentions = re.findall(r'GW_(\d+)', content)
    if gateway_mentions:
        data['gateway_ids'] = sorted(list(set(gateway_mentions)))
        data['gateway_count'] = len(set(gateway_mentions))
    
    return data

def analyze_csv_data(csv_data):
    """Extract raw data from CSV"""
    if not csv_data:
        return {}
    
    data = {
        'total_rows': len(csv_data),
        'devices': {},
        'gateways': {},
        'timestamps': []
    }
    
    for row in csv_data:
        # Extract timestamp
        time_val = row.get('Time', '')
        if time_val:
            try:
                data['timestamps'].append(float(time_val))
            except:
                pass
        
        device_type = row.get('DeviceType', '')
        device_id = row.get('DeviceID', '')
        
        if device_type == 'EndDevice':
            packets_sent = row.get('PacketsSent', '0')
            packets_received = row.get('PacketsReceived', '0')
            pdr = row.get('PDR', '0')
            nb_trans = row.get('NbTrans', '0')
            sf_dist = row.get('SF_Distribution', '')
            tp_dist = row.get('TxPower_Distribution', '')
            avg_rssi = row.get('AvgRSSI', '0')
            avg_snr = row.get('AvgSNR', '0')
            
            data['devices'][device_id] = {
                'sent': int(packets_sent) if packets_sent.isdigit() else 0,
                'received': int(packets_received) if packets_received.isdigit() else 0,
                'pdr': float(pdr) if pdr.replace('.','').replace('-','').isdigit() else 0,
                'nb_trans': int(nb_trans) if nb_trans.isdigit() else 0,
                'sf_distribution': sf_dist.strip('"'),
                'tp_distribution': tp_dist.strip('"'),
                'avg_rssi': float(avg_rssi) if avg_rssi.replace('.','').replace('-','').isdigit() else 0,
                'avg_snr': float(avg_snr) if avg_snr.replace('.','').replace('-','').isdigit() else 0
            }
            
        elif device_type == 'Gateway':
            packets_received = row.get('PacketsReceived', '0')
            position = row.get('Position', '')
            avg_rssi = row.get('AvgRSSI', '0')
            avg_snr = row.get('AvgSNR', '0')
            
            data['gateways'][device_id] = {
                'received': int(packets_received) if packets_received.isdigit() else 0,
                'position': position.strip('"'),
                'avg_rssi': float(avg_rssi) if avg_rssi.replace('.','').replace('-','').isdigit() else 0,
                'avg_snr': float(avg_snr) if avg_snr.replace('.','').replace('-','').isdigit() else 0
            }
    
    # Time analysis
    if data['timestamps']:
        data['first_time'] = min(data['timestamps'])
        data['last_time'] = max(data['timestamps'])
        data['time_span'] = max(data['timestamps']) - min(data['timestamps'])
    
    return data

def print_enhanced_summary(csv_data, output_data, node_data, phy_data, global_data, rssi_data, radio_summary):
    """Print comprehensive summary with radio measurements"""
    
    print("=" * 80)
    print("ENHANCED LORAWAN SIMULATION ANALYSIS WITH RADIO MEASUREMENTS")
    print("=" * 80)
    
    # Basic simulation info
    if output_data:
        print(f"\n🎯 SIMULATION OVERVIEW:")
        print(f"   Duration: {output_data.get('simulation_time', 0):.1f} seconds")
        print(f"   Devices: {output_data.get('devices_count', 0)}")
        print(f"   Gateways: {output_data.get('gateways_count', 0)}")
        if 'performance_assessment' in output_data:
            print(f"   Performance: {output_data['performance_assessment']}")
        if output_data.get('radio_measurements_enabled'):
            print(f"   📡 Radio measurements: ENABLED")
    
    # Traffic Statistics
    print(f"\n📊 TRAFFIC STATISTICS:")
    if output_data:
        total_sent = output_data.get('total_sent', 0)
        total_received = output_data.get('total_received', 0)
        pdr = output_data.get('pdr_percent', 0)
        
        print(f"   Packets sent: {total_sent}")
        print(f"   Packets received: {total_received}")
        print(f"   PDR: {pdr:.2f}%")
        
        if total_sent > 0:
            actual_pdr = (total_received / total_sent) * 100
            print(f"   Calculated PDR: {actual_pdr:.2f}%")
    
    # Radio Measurements Analysis
    if rssi_data:
        print(f"\n📡 RADIO MEASUREMENT ANALYSIS:")
        print(f"   Total measurements: {rssi_data.get('total_measurements', 0)}")
        print(f"   Time span: {rssi_data.get('time_span', 0):.1f} seconds")
        
        # RSSI Statistics
        rssi_stats = rssi_data.get('rssi_stats', {})
        if rssi_stats:
            print(f"\n   📶 RSSI Statistics:")
            print(f"     Range: {rssi_stats['min']:.1f} to {rssi_stats['max']:.1f} dBm")
            print(f"     Mean: {rssi_stats['mean']:.1f} dBm")
            print(f"     Std Dev: {rssi_stats['stdev']:.1f} dB")
        
        # SNR Statistics  
        snr_stats = rssi_data.get('snr_stats', {})
        if snr_stats:
            print(f"\n   📡 SNR Statistics:")
            print(f"     Range: {snr_stats['min']:.1f} to {snr_stats['max']:.1f} dB")
            print(f"     Mean: {snr_stats['mean']:.1f} dB")
            print(f"     Std Dev: {snr_stats['stdev']:.1f} dB")
        
        # Gateway Diversity
        gateway_diversity = rssi_data.get('gateway_diversity', {})
        if gateway_diversity:
            print(f"\n   🌐 Gateway Diversity:")
            for device, gw_count in gateway_diversity.items():
                print(f"     Device {device}: {gw_count} gateways")
        
        # Per-Gateway Analysis
        gateways = rssi_data.get('gateways', {})
        if gateways:
            print(f"\n   📡 Gateway Performance:")
            for gw_id, gw_data in gateways.items():
                avg_rssi = statistics.mean(gw_data['rssi_values']) if gw_data['rssi_values'] else 0
                avg_snr = statistics.mean(gw_data['snr_values']) if gw_data['snr_values'] else 0
                print(f"     GW_{gw_id}: {gw_data['measurements']} meas, "
                      f"RSSI: {avg_rssi:.1f}dBm, SNR: {avg_snr:.1f}dB")
                if gw_data['position']:
                    print(f"       Position: {gw_data['position']}")
    
    # Radio Summary Analysis
    if radio_summary:
        print(f"\n📋 RADIO MEASUREMENT SUMMARY:")
        print(f"   Devices analyzed: {radio_summary.get('devices_count', 0)}")
        print(f"   Total measurements: {radio_summary.get('total_measurements', 0)}")
        
        devices = radio_summary.get('devices', {})
        for device_addr, dev_data in devices.items():
            print(f"\n   Device {device_addr}:")
            print(f"     Measurements: {dev_data['measurement_count']}")
            print(f"     Avg RSSI: {dev_data['avg_rssi']:.1f} dBm")
            print(f"     RSSI range: [{dev_data['rssi_range'][0]:.1f}, {dev_data['rssi_range'][1]:.1f}] dBm")
            print(f"     Avg SNR: {dev_data['avg_snr']:.1f} dB")
            print(f"     SNR range: [{dev_data['snr_range'][0]:.1f}, {dev_data['snr_range'][1]:.1f}] dB")
    
    # CSV Data Analysis
    if csv_data:
        print(f"\n📈 CSV DATA ANALYSIS:")
        print(f"   Total CSV rows: {csv_data.get('total_rows', 0)}")
        
        devices = csv_data.get('devices', {})
        if devices:
            print(f"\n   📱 End Devices ({len(devices)}):")
            for device_id, data in devices.items():
                print(f"     {device_id}: {data['sent']} sent, {data['received']} received")
                print(f"       PDR: {data['pdr']:.4f}, NbTrans: {data['nb_trans']}")
                if data.get('avg_rssi', 0) != 0:
                    print(f"       Radio: RSSI {data['avg_rssi']:.1f}dBm, SNR {data['avg_snr']:.1f}dB")
                if data['sf_distribution']:
                    print(f"       SF Distribution: {data['sf_distribution']}")
        
        gateways = csv_data.get('gateways', {})
        if gateways:
            print(f"\n   📡 Gateways ({len(gateways)}):")
            for gw_id, data in gateways.items():
                print(f"     {gw_id}: {data['received']} received")
                if data.get('avg_rssi', 0) != 0:
                    print(f"       Radio: RSSI {data['avg_rssi']:.1f}dBm, SNR {data['avg_snr']:.1f}dB")
                if data['position']:
                    print(f"       Position: {data['position']}")
    
    # Performance Files
    if node_data:
        print(f"\n📝 NODE DATA FILE:")
        print(f"   Lines: {node_data.get('total_lines', 0)}")
        print(f"   Time span: {node_data.get('time_span', 0):.1f} seconds")
    
    if phy_data:
        print(f"\n🔧 PHY PERFORMANCE FILE:")
        print(f"   Lines: {phy_data.get('total_lines', 0)}")
        print(f"   Gateways detected: {phy_data.get('gateway_count', 0)}")
    
    print("\n" + "=" * 80)
    
    # Paper Validation Summary
    if output_data and rssi_data:
        print("🎯 PAPER REPLICATION VALIDATION:")
        
        pdr = output_data.get('pdr_percent', 0)
        if pdr >= 99:
            print("   ✅ PDR meets paper's DER < 0.01 target (99%+ success)")
        elif pdr >= 95:
            print("   🟡 PDR close to paper target (95-99%)")
        elif pdr >= 85:
            print("   🟠 PDR within typical LoRaWAN range (85-95%)")
        else:
            print("   🔴 PDR below typical expectations (<85%)")
        
        gw_diversity = len(rssi_data.get('gateways', {}))
        if gw_diversity >= 8:
            print(f"   ✅ Full 8-gateway macrodiversity achieved")
        elif gw_diversity >= 6:
            print(f"   🟡 Good gateway diversity ({gw_diversity}/8)")
        else:
            print(f"   🔴 Limited gateway diversity ({gw_diversity}/8)")
        
        measurements = rssi_data.get('total_measurements', 0)
        if measurements >= 100:
            print(f"   ✅ Sufficient radio measurements ({measurements})")
        else:
            print(f"   ⚠️  Limited radio measurements ({measurements})")
    
    print("=" * 80)

def main():
    """Main function - analyze all files including radio measurements"""
    print("ENHANCED LORAWAN SIMULATION ANALYZER")
    print("=" * 40)
    
    # File patterns to check
    csv_patterns = [
        "paper_replication_adr_with_rssi.csv",
        "paper_replication_adr.csv", 
        "realistic_adr_test.csv",
        "quick_test_adr.csv"
    ]
    
    output_patterns = [
        "paper_replication_output_with_rssi.txt",
        "paper_replication_output.txt",
        "realistic_simulation_output.txt",
        "quick_test_output.txt"
    ]
    
    # Radio measurement files
    rssi_patterns = [
        "rssi_snr_measurements.csv"
    ]
    
    radio_summary_patterns = [
        "radio_measurement_summary.csv"
    ]
    
    node_patterns = [
        "paper_nodeData.txt",
        "quick_nodeData.txt"
    ]
    
    phy_patterns = [
        "paper_phyPerformance.txt",
        "quick_phyPerformance.txt"
    ]
    
    global_patterns = [
        "paper_globalPerformance.txt",
        "quick_globalPerformance.txt"
    ]
    
    # Find existing files
    def find_file(patterns):
        for pattern in patterns:
            if os.path.exists(pattern):
                return pattern
        return None
    
    csv_file = find_file(csv_patterns)
    output_file = find_file(output_patterns)
    rssi_file = find_file(rssi_patterns)
    radio_summary_file = find_file(radio_summary_patterns)
    node_file = find_file(node_patterns)
    phy_file = find_file(phy_patterns)
    global_file = find_file(global_patterns)
    
    print(f"\n📁 FILES FOUND:")
    print(f"   Main CSV: {csv_file if csv_file else 'None'}")
    print(f"   Output log: {output_file if output_file else 'None'}")
    print(f"   📡 RSSI/SNR: {rssi_file if rssi_file else 'None'}")
    print(f"   📊 Radio summary: {radio_summary_file if radio_summary_file else 'None'}")
    print(f"   Node data: {node_file if node_file else 'None'}")
    print(f"   PHY data: {phy_file if phy_file else 'None'}")
    
    # Analyze files
    csv_data = None
    if csv_file:
        raw_csv = read_csv_file(csv_file)
        csv_data = analyze_csv_data(raw_csv)
    
    output_data = analyze_output_file(output_file) if output_file else None
    rssi_data = analyze_rssi_snr_measurements(rssi_file) if rssi_file else None
    radio_summary = analyze_radio_summary(radio_summary_file) if radio_summary_file else None
    node_data = analyze_node_data_file(node_file) if node_file else None
    phy_data = analyze_phy_performance_file(phy_file) if phy_file else None
    global_data = None  # Placeholder for future enhancement
    
    # Print comprehensive summary
    print_enhanced_summary(csv_data, output_data, node_data, phy_data, global_data, rssi_data, radio_summary)

if __name__ == "__main__":
    main()