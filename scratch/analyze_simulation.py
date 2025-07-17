#!/usr/bin/env python3
"""
Raw File Data Analyzer
Extracts plain data from all simulation files without interpretation
"""

import csv
import os
import re
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
    except:
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

def extract_numbers_from_text(text, pattern):
    """Extract numbers from text using regex pattern"""
    matches = re.findall(pattern, text)
    return matches

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
        'devices_count': r'Test Device: (\d+)',
        'gateways_count': r'Gateways: (\d+)',
        'simulation_time': r'Duration: ([\d.]+) seconds',
        'expected_packets': r'Expected packets: (\d+)'
    }
    
    for key, pattern in patterns.items():
        matches = re.findall(pattern, content)
        if matches:
            try:
                data[key] = float(matches[-1]) if '.' in matches[-1] else int(matches[-1])
            except:
                data[key] = matches[-1]
    
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

def analyze_global_performance_file(filename):
    """Extract data from globalPerformance file"""
    content = read_text_file(filename)
    if not content:
        return {}
    
    data = {}
    lines = content.split('\n')
    data['total_lines'] = len([l for l in lines if l.strip()])
    
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
            
            data['devices'][device_id] = {
                'sent': int(packets_sent) if packets_sent.isdigit() else 0,
                'received': int(packets_received) if packets_received.isdigit() else 0,
                'pdr': float(pdr) if pdr.replace('.','').isdigit() else 0,
                'nb_trans': int(nb_trans) if nb_trans.isdigit() else 0,
                'sf_distribution': sf_dist.strip('"'),
                'tp_distribution': tp_dist.strip('"')
            }
            
        elif device_type == 'Gateway':
            packets_received = row.get('PacketsReceived', '0')
            position = row.get('Position', '')
            
            data['gateways'][device_id] = {
                'received': int(packets_received) if packets_received.isdigit() else 0,
                'position': position.strip('"')
            }
    
    # Time analysis
    if data['timestamps']:
        data['first_time'] = min(data['timestamps'])
        data['last_time'] = max(data['timestamps'])
        data['time_span'] = max(data['timestamps']) - min(data['timestamps'])
    
    return data

def print_raw_summary(csv_data, output_data, node_data, phy_data, global_data):
    """Print plain summary without interpretation"""
    
    print("=" * 60)
    print("RAW SIMULATION DATA SUMMARY")
    print("=" * 60)
    
    # CSV Data
    if csv_data:
        print(f"\nCSV FILE DATA:")
        print(f"Total rows: {csv_data.get('total_rows', 0)}")
        
        if csv_data.get('timestamps'):
            print(f"Time span: {csv_data.get('time_span', 0):.1f} seconds")
            print(f"First time: {csv_data.get('first_time', 0):.1f} seconds")
            print(f"Last time: {csv_data.get('last_time', 0):.1f} seconds")
        
        # Device data
        devices = csv_data.get('devices', {})
        if devices:
            print(f"\nEND DEVICES ({len(devices)}):")
            for device_id, data in devices.items():
                print(f"  {device_id}: {data['sent']} sent, {data['received']} received, PDR: {data['pdr']:.4f}")
                if data['sf_distribution']:
                    print(f"    SF: {data['sf_distribution']}")
                if data['tp_distribution']:
                    print(f"    TP: {data['tp_distribution']}")
        
        # Gateway data
        gateways = csv_data.get('gateways', {})
        if gateways:
            print(f"\nGATEWAYS ({len(gateways)}):")
            for gw_id, data in gateways.items():
                print(f"  {gw_id}: {data['received']} received")
                if data['position']:
                    print(f"    Position: {data['position']}")
    
    # Output file data
    if output_data:
        print(f"\nOUTPUT FILE DATA:")
        for key, value in output_data.items():
            print(f"  {key}: {value}")
    
    # Node data file
    if node_data:
        print(f"\nNODE DATA FILE:")
        for key, value in node_data.items():
            print(f"  {key}: {value}")
    
    # Phy performance file
    if phy_data:
        print(f"\nPHY PERFORMANCE FILE:")
        for key, value in phy_data.items():
            print(f"  {key}: {value}")
    
    # Global performance file  
    if global_data:
        print(f"\nGLOBAL PERFORMANCE FILE:")
        for key, value in global_data.items():
            print(f"  {key}: {value}")
    
    print("\n" + "=" * 60)

def main():
    """Main function - analyze all files"""
    print("RAW FILE DATA ANALYZER")
    print("=" * 25)
    
    # File patterns to check
    file_patterns = [
        "quick_test_adr.csv",
        "paper_replication_adr.csv", 
        "realistic_adr_test.csv"
    ]
    
    output_patterns = [
        "quick_test_output.txt",
        "paper_replication_output.txt",
        "realistic_simulation_output.txt"
    ]
    
    node_patterns = [
        "quick_nodeData.txt",
        "paper_nodeData.txt"
    ]
    
    phy_patterns = [
        "quick_phyPerformance.txt", 
        "paper_phyPerformance.txt"
    ]
    
    global_patterns = [
        "quick_globalPerformance.txt",
        "paper_globalPerformance.txt"
    ]
    
    # Find existing files
    csv_file = None
    output_file = None
    node_file = None
    phy_file = None
    global_file = None
    
    for pattern in file_patterns:
        if os.path.exists(pattern):
            csv_file = pattern
            break
    
    for pattern in output_patterns:
        if os.path.exists(pattern):
            output_file = pattern
            break
            
    for pattern in node_patterns:
        if os.path.exists(pattern):
            node_file = pattern
            break
            
    for pattern in phy_patterns:
        if os.path.exists(pattern):
            phy_file = pattern
            break
            
    for pattern in global_patterns:
        if os.path.exists(pattern):
            global_file = pattern
            break
    
    print(f"\nFILES FOUND:")
    print(f"CSV: {csv_file if csv_file else 'None'}")
    print(f"Output: {output_file if output_file else 'None'}")
    print(f"Node: {node_file if node_file else 'None'}")
    print(f"Phy: {phy_file if phy_file else 'None'}")
    print(f"Global: {global_file if global_file else 'None'}")
    
    # Analyze files
    csv_data = None
    if csv_file:
        raw_csv = read_csv_file(csv_file)
        csv_data = analyze_csv_data(raw_csv)
    
    output_data = analyze_output_file(output_file) if output_file else None
    node_data = analyze_node_data_file(node_file) if node_file else None
    phy_data = analyze_phy_performance_file(phy_file) if phy_file else None
    global_data = analyze_global_performance_file(global_file) if global_file else None
    
    # Print summary
    print_raw_summary(csv_data, output_data, node_data, phy_data, global_data)

if __name__ == "__main__":
    main()