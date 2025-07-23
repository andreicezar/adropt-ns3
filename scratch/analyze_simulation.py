#!/usr/bin/env python3
"""
Packet Counter Script - Updated for Actual File Structure

Counts per end node:
1. Total packets sent
2. Total packets received  
3. Total packets received with success
"""

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

def normalize_device_id(device_id):
    """Normalize device IDs to handle ED_ prefix and other variations."""
    device_str = str(device_id)
    if device_str.startswith('ED_'):
        return device_str[3:]  # Remove 'ED_' prefix
    return device_str

def merge_device_stats(node_stats):
    """Merge duplicate device entries (e.g., 1811941192 and ED_1811941192)."""
    merged_stats = {}
    
    for device_id, stats in node_stats.items():
        normalized_id = normalize_device_id(device_id)
        
        if normalized_id not in merged_stats:
            merged_stats[normalized_id] = {
                'sent': 0,
                'received': 0,
                'success': 0,
                'sources': []
            }
        
        # Use the highest values (most complete data)
        if stats['sent'] > merged_stats[normalized_id]['sent']:
            merged_stats[normalized_id]['sent'] = stats['sent']
            merged_stats[normalized_id]['sources'].append(f"{device_id}(sent)")
            
        if stats['received'] > merged_stats[normalized_id]['received']:
            merged_stats[normalized_id]['received'] = stats['received']
            merged_stats[normalized_id]['sources'].append(f"{device_id}(recv)")
            
        if stats['success'] > merged_stats[normalized_id]['success']:
            merged_stats[normalized_id]['success'] = stats['success']
            merged_stats[normalized_id]['sources'].append(f"{device_id}(succ)")
    
    # Clean up sources info
    for device_id, stats in merged_stats.items():
        stats['sources'] = list(set(stats['sources']))  # Remove duplicates
    
    return merged_stats

def load_and_analyze():
    """Load data and count packets per end node."""
    print("📊 PACKET COUNTER PER END NODE")
    print("=" * 50)
    
    # Results storage
    node_stats = {}
    
    # 1. Load main simulation data (for packets sent)
    try:
        main_data = pd.read_csv('paper_replication_adr_fec.csv')
        print(f"✅ Main simulation: {len(main_data)} entries")
        print(f"   Columns: {list(main_data.columns)}")
        
        # Filter for end devices and get latest stats
        if 'Role' in main_data.columns:
            print(main_data['Role'].value_counts())
            device_data = main_data[main_data['Role'] == 'LoRaWAN_Transmitter']  # Fixed role name
        else:
            device_data = main_data
            
        print(f"   End device entries: {len(device_data)}")
        
        # Get packets sent per device (use latest entry for each device)
        if 'DeviceID' in device_data.columns and 'PacketsSent' in device_data.columns:
            for device_id in device_data['DeviceID'].unique():
                device_rows = device_data[device_data['DeviceID'] == device_id]
                latest_row = device_rows.iloc[-1]  # Get latest entry
                
                packets_sent = int(latest_row['PacketsSent'])
                packets_received = int(latest_row['PacketsReceived']) if 'PacketsReceived' in latest_row else 0
                
                # Keep original device_id as key (we'll normalize later)
                device_key = str(device_id)
                node_stats[device_key] = {
                    'sent': packets_sent,
                    'received': packets_received,
                    'success': packets_received  # Will be updated from radio data
                }
                
                print(f"   → Device {device_key}: {packets_sent} sent, {packets_received} received")
        
        elif 'NodeID' in device_data.columns and 'PacketsSent' in device_data.columns:
            # Alternative: use NodeID
            for node_id in device_data['NodeID'].unique():
                device_rows = device_data[device_data['NodeID'] == node_id]
                latest_row = device_rows.iloc[-1]
                
                packets_sent = int(latest_row['PacketsSent'])
                packets_received = int(latest_row['PacketsReceived']) if 'PacketsReceived' in latest_row else 0
                
                # Keep original node_id as key
                node_key = str(node_id)
                node_stats[node_key] = {
                    'sent': packets_sent,
                    'received': packets_received,
                    'success': packets_received
                }
                
                print(f"   → Node {node_key}: {packets_sent} sent, {packets_received} received")
                
    except Exception as e:
        print(f"❌ Could not load paper_replication_adr_fec.csv: {e}")
    
    # 2. Load radio measurements (for detailed packet success/failure)
    try:
        radio_data = pd.read_csv('radio_measurements.csv')
        print(f"✅ Radio measurements: {len(radio_data)} entries")
        print(f"   Columns: {list(radio_data.columns)}")
        
        if 'DeviceAddr' in radio_data.columns:
            # Count total receptions and successful receptions per device
            for device_addr in radio_data['DeviceAddr'].unique():
                device_packets = radio_data[radio_data['DeviceAddr'] == device_addr]
                
                total_receptions = len(device_packets)
                
                # Count successful packets (if PacketSuccess column exists)
                if 'PacketSuccess' in radio_data.columns:
                    successful_receptions = len(device_packets[device_packets['PacketSuccess'] == 1])
                else:
                    successful_receptions = total_receptions  # Assume all are successful
                
                # Estimate unique packets (divide by gateway count)
                num_gateways = radio_data['GatewayID'].nunique() if 'GatewayID' in radio_data.columns else 1
                unique_receptions = total_receptions // num_gateways
                unique_successes = successful_receptions // num_gateways
                
                # Update or create device stats (keep original device_addr)
                device_key = str(device_addr)
                if device_key not in node_stats:
                    node_stats[device_key] = {'sent': 0, 'received': 0, 'success': 0}
                
                node_stats[device_key]['received'] = unique_receptions
                node_stats[device_key]['success'] = unique_successes
                
                print(f"   → Device {device_key}: {total_receptions} total receptions, {successful_receptions} successful")
                print(f"      Estimated unique: {unique_receptions} received, {unique_successes} successful")
                
    except Exception as e:
        print(f"❌ Could not load radio_measurements.csv: {e}")
    
    # 3. Try alternative radio measurements file (rssi_snr_measurements.csv)
    try:
        rssi_data = pd.read_csv('rssi_snr_measurements.csv')
        print(f"✅ RSSI/SNR measurements: {len(rssi_data)} entries")
        print(f"   Columns: {list(rssi_data.columns)}")
        
        if 'DeviceAddr' in rssi_data.columns:
            for device_addr in rssi_data['DeviceAddr'].unique():
                device_packets = rssi_data[rssi_data['DeviceAddr'] == device_addr]
                total_receptions = len(device_packets)
                
                # Estimate unique packets
                num_gateways = rssi_data['GatewayID'].nunique() if 'GatewayID' in rssi_data.columns else 1
                unique_receptions = total_receptions // num_gateways
                
                # Update stats if device not already processed (keep original device_addr)
                device_key = str(device_addr)
                if device_key not in node_stats:
                    node_stats[device_key] = {'sent': 0, 'received': unique_receptions, 'success': unique_receptions}
                elif node_stats[device_key]['received'] == 0:
                    node_stats[device_key]['received'] = unique_receptions
                    node_stats[device_key]['success'] = unique_receptions
                
                print(f"   → Device {device_key}: {total_receptions} RSSI measurements, ~{unique_receptions} unique packets")
                
    except Exception as e:
        print(f"❌ Could not load rssi_snr_measurements.csv: {e}")
    
    # 4. Cross-reference with console log data (from the provided text)
    console_data = {
        '1811941192': {  # Keep original key, will be merged later
            'sent': 4997,
            'received': 4592,
            'pdr': 91.90
        }
    }
    
    # Update with console data if available
    for device_id, console_stats in console_data.items():
        if device_id in node_stats:
            # Use console data if it seems more complete
            if console_stats['sent'] > node_stats[device_id]['sent']:
                print(f"   → Using console data for device {device_id}")
                node_stats[device_id]['sent'] = console_stats['sent']
                node_stats[device_id]['received'] = console_stats['received']
                node_stats[device_id]['success'] = console_stats['received']  # Assume received = success for now
        else:
            node_stats[device_id] = {
                'sent': console_stats['sent'],
                'received': console_stats['received'],
                'success': console_stats['received']
            }
            print(f"   → Added device {device_id} from console data")
    
    # 5. Merge duplicate device entries (e.g., 1811941192 and ED_1811941192)
    print(f"\n🔗 MERGING DUPLICATE DEVICE ENTRIES")
    print("=" * 50)
    
    print(f"   Before merging: {len(node_stats)} entries")
    for device_id in node_stats.keys():
        print(f"     → {device_id}")
    
    merged_stats = merge_device_stats(node_stats)
    
    print(f"   After merging: {len(merged_stats)} unique devices")
    for device_id, stats in merged_stats.items():
        print(f"     → {device_id}: {stats['sent']} sent, {stats['received']} received")
        print(f"       Sources: {', '.join(stats['sources'])}")
    
    # Convert back to simple format for compatibility
    final_stats = {}
    for device_id, stats in merged_stats.items():
        final_stats[device_id] = {
            'sent': stats['sent'],
            'received': stats['received'],
            'success': stats['success']
        }
    
    return final_stats

def print_results(node_stats):
    """Print packet counting results."""
    print("\n📊 PACKET COUNTING RESULTS")
    print("=" * 80)
    
    if not node_stats:
        print("❌ No device data found")
        return
    
    # Header
    print(f"{'Device ID':<15} {'Sent':<8} {'Received':<10} {'Success':<10} {'PDR (%)':<8} {'DER (%)':<8}")
    print("-" * 80)
    
    total_sent = total_received = total_success = 0
    
    # Sort device IDs (handle both string and numeric IDs)
    try:
        sorted_devices = sorted(node_stats.items(), key=lambda x: int(x[0]))
    except ValueError:
        # If conversion to int fails, sort as strings
        sorted_devices = sorted(node_stats.items())
    
    for device_id, stats in sorted_devices:
        sent = stats['sent']
        received = stats['received']
        success = stats['success']
        
        # Calculate PDR and DER
        pdr = (success / sent * 100) if sent > 0 else 0
        der = 100 - pdr
        
        print(f"{device_id:<15} {sent:<8} {received:<10} {success:<10} {pdr:<8.1f} {der:<8.1f}")
        
        total_sent += sent
        total_received += received
        total_success += success
    
    print("-" * 80)
    overall_pdr = (total_success/total_sent*100) if total_sent > 0 else 0
    overall_der = 100 - overall_pdr
    print(f"{'TOTAL':<15} {total_sent:<8} {total_received:<10} {total_success:<10} {overall_pdr:<8.1f} {overall_der:<8.1f}")
    
    # Summary
    print(f"\n📋 SUMMARY:")
    print(f"   Total devices analyzed: {len(node_stats)}")
    print(f"   Total packets sent: {total_sent:,}")
    print(f"   Total packets received: {total_received:,}")
    print(f"   Total successful receptions: {total_success:,}")
    print(f"   Overall PDR: {overall_pdr:.2f}%")
    print(f"   Overall DER: {overall_der:.2f}%")
    
    # Performance assessment
    if overall_der < 1.0:
        print(f"   ✅ EXCELLENT: DER < 1% (meeting paper target)")
    elif overall_der < 5.0:
        print(f"   🟡 GOOD: DER < 5% (acceptable LoRaWAN performance)")
    elif overall_der < 10.0:
        print(f"   🟠 FAIR: DER < 10% (typical urban performance)")
    else:
        print(f"   ❌ POOR: DER > 10% (needs improvement)")

def save_results(node_stats):
    """Save results to CSV file."""
    if not node_stats:
        return
        
    # Create DataFrame
    data = []
    
    # Sort device IDs properly
    try:
        sorted_devices = sorted(node_stats.items(), key=lambda x: int(x[0]))
    except ValueError:
        sorted_devices = sorted(node_stats.items())
    
    for device_id, stats in sorted_devices:
        pdr = (stats['success'] / stats['sent'] * 100) if stats['sent'] > 0 else 0
        der = 100 - pdr
        
        data.append({
            'DeviceID': device_id,
            'PacketsSent': stats['sent'],
            'PacketsReceived': stats['received'],
            'PacketsSuccess': stats['success'],
            'PDR_Percent': round(pdr, 2),
            'DER_Percent': round(der, 2)
        })
    
    df = pd.DataFrame(data)
    output_file = 'packet_count_per_node.csv'
    df.to_csv(output_file, index=False)
    print(f"\n💾 Results saved to: {output_file}")

def analyze_per_node_distributions(node_stats):
    """Analyze SF, TP, RSSI, SNR/SNIR distributions per end node."""
    print("\n📊 PER-NODE DISTRIBUTION ANALYSIS")
    print("=" * 70)
    
    distribution_stats = {}
    
    # 1. Load radio measurements for SF, TP, RSSI analysis
    try:
        radio_data = pd.read_csv('radio_measurements.csv')
        print(f"✅ Radio measurements: {len(radio_data)} entries")
        
        for device_addr in radio_data['DeviceAddr'].unique():
            device_data = radio_data[radio_data['DeviceAddr'] == device_addr]
            # Normalize device key for consistency
            device_key = normalize_device_id(device_addr)
            
            stats = {}
            
            # Spreading Factor Analysis
            if 'SpreadingFactor' in device_data.columns:
                sf_values = device_data['SpreadingFactor']
                sf_counts = sf_values.value_counts().sort_index()
                stats['sf_distribution'] = sf_counts.to_dict()
                stats['sf_mean'] = sf_values.mean()
                stats['sf_mode'] = sf_values.mode()[0] if not sf_values.mode().empty else 'N/A'
                print(f"\n📡 Device {device_key} - Spreading Factor:")
                for sf, count in sf_counts.items():
                    percentage = (count / len(device_data)) * 100
                    print(f"   SF{sf}: {count:4d} packets ({percentage:5.1f}%)")
                print(f"   Mean SF: {stats['sf_mean']:.1f}, Mode SF: {stats['sf_mode']}")
            
            # Transmission Power Analysis  
            if 'TxPower_dBm' in device_data.columns:
                tp_values = device_data['TxPower_dBm']
                tp_counts = tp_values.value_counts().sort_index()
                stats['tp_distribution'] = tp_counts.to_dict()
                stats['tp_mean'] = tp_values.mean()
                stats['tp_min'] = tp_values.min()
                stats['tp_max'] = tp_values.max()
                print(f"\n🔋 Device {device_key} - Transmission Power:")
                for tp, count in tp_counts.items():
                    percentage = (count / len(device_data)) * 100
                    print(f"   {tp:4.0f}dBm: {count:4d} packets ({percentage:5.1f}%)")
                print(f"   Range: {stats['tp_min']:.0f} to {stats['tp_max']:.0f} dBm, Mean: {stats['tp_mean']:.1f} dBm")
            
            # RSSI Analysis
            if 'RSSI_dBm' in device_data.columns:
                rssi_values = device_data['RSSI_dBm']
                stats['rssi_mean'] = rssi_values.mean()
                stats['rssi_std'] = rssi_values.std()
                stats['rssi_min'] = rssi_values.min()
                stats['rssi_max'] = rssi_values.max()
                stats['rssi_q25'] = rssi_values.quantile(0.25)
                stats['rssi_q75'] = rssi_values.quantile(0.75)
                print(f"\n📶 Device {device_key} - RSSI Distribution:")
                print(f"   Mean: {stats['rssi_mean']:6.1f} dBm, Std: {stats['rssi_std']:5.1f} dB")
                print(f"   Range: [{stats['rssi_min']:6.1f}, {stats['rssi_max']:6.1f}] dBm")
                print(f"   Q25-Q75: [{stats['rssi_q25']:6.1f}, {stats['rssi_q75']:6.1f}] dBm")
            
            # SNR Analysis
            if 'SNR_dB' in device_data.columns:
                snr_values = device_data['SNR_dB']
                stats['snr_mean'] = snr_values.mean()
                stats['snr_std'] = snr_values.std()
                stats['snr_min'] = snr_values.min()
                stats['snr_max'] = snr_values.max()
                stats['snr_q25'] = snr_values.quantile(0.25)
                stats['snr_q75'] = snr_values.quantile(0.75)
                print(f"\n📡 Device {device_key} - SNR Distribution:")
                print(f"   Mean: {stats['snr_mean']:6.1f} dB, Std: {stats['snr_std']:5.1f} dB")
                print(f"   Range: [{stats['snr_min']:6.1f}, {stats['snr_max']:6.1f}] dB")
                print(f"   Q25-Q75: [{stats['snr_q25']:6.1f}, {stats['snr_q75']:6.1f}] dB")
            
            # SNIR Analysis (if available)
            if 'SNIR_dB' in device_data.columns:
                snir_values = device_data['SNIR_dB']
                stats['snir_mean'] = snir_values.mean()
                stats['snir_std'] = snir_values.std()
                stats['snir_min'] = snir_values.min()
                stats['snir_max'] = snir_values.max()
                print(f"\n📊 Device {device_key} - SNIR Distribution:")
                print(f"   Mean: {stats['snir_mean']:6.1f} dB, Std: {stats['snir_std']:5.1f} dB")
                print(f"   Range: [{stats['snir_min']:6.1f}, {stats['snir_max']:6.1f}] dB")
            
            distribution_stats[device_key] = stats
            
    except Exception as e:
        print(f"❌ Could not load radio_measurements.csv: {e}")
        
    # 2. Try alternative RSSI/SNR file if main file failed
    if not distribution_stats:
        try:
            rssi_data = pd.read_csv('rssi_snr_measurements.csv')
            print(f"✅ Using RSSI/SNR measurements: {len(rssi_data)} entries")
            
            for device_addr in rssi_data['DeviceAddr'].unique():
                device_data = rssi_data[rssi_data['DeviceAddr'] == device_addr]
                # Normalize device key for consistency
                device_key = normalize_device_id(device_addr)
                
                stats = {}
                
                # SF Analysis
                if 'SpreadingFactor' in device_data.columns:
                    sf_values = device_data['SpreadingFactor']
                    sf_counts = sf_values.value_counts().sort_index()
                    stats['sf_distribution'] = sf_counts.to_dict()
                    stats['sf_mean'] = sf_values.mean()
                    stats['sf_mode'] = sf_values.mode()[0] if not sf_values.mode().empty else 'N/A'
                
                # TP Analysis
                if 'TxPower_dBm' in device_data.columns:
                    tp_values = device_data['TxPower_dBm']
                    tp_counts = tp_values.value_counts().sort_index()
                    stats['tp_distribution'] = tp_counts.to_dict()
                    stats['tp_mean'] = tp_values.mean()
                
                # RSSI Analysis
                if 'RSSI_dBm' in device_data.columns:
                    rssi_values = device_data['RSSI_dBm']
                    stats['rssi_mean'] = rssi_values.mean()
                    stats['rssi_std'] = rssi_values.std()
                    stats['rssi_min'] = rssi_values.min()
                    stats['rssi_max'] = rssi_values.max()
                
                # SNR Analysis
                if 'SNR_dB' in device_data.columns:
                    snr_values = device_data['SNR_dB']
                    stats['snr_mean'] = snr_values.mean()
                    stats['snr_std'] = snr_values.std()
                    stats['snr_min'] = snr_values.min()
                    stats['snr_max'] = snr_values.max()
                
                distribution_stats[device_key] = stats
                
        except Exception as e:
            print(f"❌ Could not load rssi_snr_measurements.csv: {e}")
    
    return distribution_stats

def create_distribution_plots(distribution_stats):
    """Create comprehensive distribution plots."""
    print("\n📊 GENERATING DISTRIBUTION PLOTS")
    print("=" * 50)
    
    if not distribution_stats:
        print("❌ No distribution data available for plotting")
        return
    
    # Create figure with subplots
    fig = plt.figure(figsize=(20, 15))
    gs = fig.add_gridspec(3, 4, hspace=0.4, wspace=0.3)
    
    fig.suptitle('Per-Node Distribution Analysis\nSF, TP, RSSI, SNR/SNIR Distributions', 
                 fontsize=16, fontweight='bold')
    
    # Load raw data for plotting
    try:
        radio_data = pd.read_csv('radio_measurements.csv')
    except:
        try:
            radio_data = pd.read_csv('rssi_snr_measurements.csv')
        except:
            print("❌ Cannot load radio data for plotting")
            return
    
    devices = radio_data['DeviceAddr'].unique()
    # Normalize device IDs for consistent plotting
    normalized_devices = [normalize_device_id(d) for d in devices]
    device_mapping = dict(zip(devices, normalized_devices))
    
    # 1. Spreading Factor Distribution (top left)
    ax1 = fig.add_subplot(gs[0, 0])
    if 'SpreadingFactor' in radio_data.columns:
        sf_by_device = []
        device_labels = []
        for device_addr in devices:
            device_data = radio_data[radio_data['DeviceAddr'] == device_addr]
            normalized_id = device_mapping[device_addr]
            if len(device_data) > 0:
                sf_by_device.append(device_data['SpreadingFactor'].values)
                device_labels.append(f'Dev {normalized_id}')
        
        if sf_by_device:
            ax1.boxplot(sf_by_device, labels=device_labels)
            ax1.set_ylabel('Spreading Factor')
            ax1.set_title('SF Distribution by Device')
            ax1.grid(True, alpha=0.3)
    else:
        ax1.text(0.5, 0.5, 'No SF data', ha='center', va='center', transform=ax1.transAxes)
        ax1.set_title('SF Distribution - No Data')
    
    # 2. Transmission Power Distribution (top right)
    ax2 = fig.add_subplot(gs[0, 1])
    if 'TxPower_dBm' in radio_data.columns:
        tp_by_device = []
        device_labels = []
        for device_addr in devices:
            device_data = radio_data[radio_data['DeviceAddr'] == device_addr]
            normalized_id = device_mapping[device_addr]
            if len(device_data) > 0:
                tp_by_device.append(device_data['TxPower_dBm'].values)
                device_labels.append(f'Dev {normalized_id}')
        
        if tp_by_device:
            ax2.boxplot(tp_by_device, labels=device_labels)
            ax2.set_ylabel('Transmission Power (dBm)')
            ax2.set_title('TX Power Distribution by Device')
            ax2.grid(True, alpha=0.3)
    else:
        ax2.text(0.5, 0.5, 'No TP data', ha='center', va='center', transform=ax2.transAxes)
        ax2.set_title('TX Power Distribution - No Data')
    
    # 3. RSSI Distribution (middle left)
    ax3 = fig.add_subplot(gs[1, 0])
    if 'RSSI_dBm' in radio_data.columns:
        for i, device_addr in enumerate(devices):
            device_data = radio_data[radio_data['DeviceAddr'] == device_addr]
            normalized_id = device_mapping[device_addr]
            if len(device_data) > 0:
                ax3.hist(device_data['RSSI_dBm'], bins=50, alpha=0.7, 
                        label=f'Device {normalized_id}', density=True)
        
        ax3.set_xlabel('RSSI (dBm)')
        ax3.set_ylabel('Density')
        ax3.set_title('RSSI Distribution by Device')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
    else:
        ax3.text(0.5, 0.5, 'No RSSI data', ha='center', va='center', transform=ax3.transAxes)
        ax3.set_title('RSSI Distribution - No Data')
    
    # 4. SNR Distribution (middle right)
    ax4 = fig.add_subplot(gs[1, 1])
    if 'SNR_dB' in radio_data.columns:
        for i, device_addr in enumerate(devices):
            device_data = radio_data[radio_data['DeviceAddr'] == device_addr]
            normalized_id = device_mapping[device_addr]
            if len(device_data) > 0:
                ax4.hist(device_data['SNR_dB'], bins=50, alpha=0.7, 
                        label=f'Device {normalized_id}', density=True)
        
        ax4.set_xlabel('SNR (dB)')
        ax4.set_ylabel('Density')
        ax4.set_title('SNR Distribution by Device')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
    else:
        ax4.text(0.5, 0.5, 'No SNR data', ha='center', va='center', transform=ax4.transAxes)
        ax4.set_title('SNR Distribution - No Data')
    
    # 5. SNIR Distribution (if available)
    ax5 = fig.add_subplot(gs[1, 2])
    if 'SNIR_dB' in radio_data.columns:
        for i, device_addr in enumerate(devices):
            device_data = radio_data[radio_data['DeviceAddr'] == device_addr]
            normalized_id = device_mapping[device_addr]
            if len(device_data) > 0:
                ax5.hist(device_data['SNIR_dB'], bins=50, alpha=0.7, 
                        label=f'Device {normalized_id}', density=True)
        
        ax5.set_xlabel('SNIR (dB)')
        ax5.set_ylabel('Density')
        ax5.set_title('SNIR Distribution by Device')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
    else:
        ax5.text(0.5, 0.5, 'No SNIR data', ha='center', va='center', transform=ax5.transAxes)
        ax5.set_title('SNIR Distribution - No Data')
    
    # 6. SF vs Time (bottom left)
    ax6 = fig.add_subplot(gs[2, 0])
    if 'SpreadingFactor' in radio_data.columns and 'Time' in radio_data.columns:
        for device_addr in devices:
            device_data = radio_data[radio_data['DeviceAddr'] == device_addr]
            normalized_id = device_mapping[device_addr]
            if len(device_data) > 0:
                time_hours = device_data['Time'] / 3600
                ax6.scatter(time_hours, device_data['SpreadingFactor'], 
                           alpha=0.6, s=10, label=f'Device {normalized_id}')
        
        ax6.set_xlabel('Time (hours)')
        ax6.set_ylabel('Spreading Factor')
        ax6.set_title('SF Evolution Over Time')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
    else:
        ax6.text(0.5, 0.5, 'No SF/Time data', ha='center', va='center', transform=ax6.transAxes)
        ax6.set_title('SF vs Time - No Data')
    
    # 7. TX Power vs Time (bottom right)
    ax7 = fig.add_subplot(gs[2, 1])
    if 'TxPower_dBm' in radio_data.columns and 'Time' in radio_data.columns:
        for device_addr in devices:
            device_data = radio_data[radio_data['DeviceAddr'] == device_addr]
            normalized_id = device_mapping[device_addr]
            if len(device_data) > 0:
                time_hours = device_data['Time'] / 3600
                ax7.scatter(time_hours, device_data['TxPower_dBm'], 
                           alpha=0.6, s=10, label=f'Device {normalized_id}')
        
        ax7.set_xlabel('Time (hours)')
        ax7.set_ylabel('TX Power (dBm)')
        ax7.set_title('TX Power Evolution Over Time')
        ax7.legend()
        ax7.grid(True, alpha=0.3)
    else:
        ax7.text(0.5, 0.5, 'No TP/Time data', ha='center', va='center', transform=ax7.transAxes)
        ax7.set_title('TX Power vs Time - No Data')
    
    # 8. Summary Statistics Table
    ax8 = fig.add_subplot(gs[0:1, 2:])
    ax8.axis('off')
    
    summary_text = "DISTRIBUTION SUMMARY STATISTICS\n" + "="*50 + "\n\n"
    
    for device_id, stats in distribution_stats.items():
        summary_text += f"Device {device_id}:\n"
        
        if 'sf_mean' in stats:
            summary_text += f"• SF: Mean={stats['sf_mean']:.1f}, Mode=SF{stats['sf_mode']}\n"
        
        if 'tp_mean' in stats:
            summary_text += f"• TX Power: {stats.get('tp_min', 0):.0f}-{stats.get('tp_max', 0):.0f}dBm, "
            summary_text += f"Mean={stats['tp_mean']:.1f}dBm\n"
        
        if 'rssi_mean' in stats:
            summary_text += f"• RSSI: {stats['rssi_mean']:.1f}±{stats['rssi_std']:.1f}dBm\n"
        
        if 'snr_mean' in stats:
            summary_text += f"• SNR: {stats['snr_mean']:.1f}±{stats['snr_std']:.1f}dB\n"
        
        if 'snir_mean' in stats:
            summary_text += f"• SNIR: {stats['snir_mean']:.1f}±{stats['snir_std']:.1f}dB\n"
        
        summary_text += "\n"
    
    ax8.text(0.05, 0.95, summary_text, transform=ax8.transAxes, fontsize=10,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round,pad=0.5', facecolor='lightblue', alpha=0.3))
    
    plt.tight_layout()
    plt.savefig('per_node_distributions.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  → Generated: per_node_distributions.png")

def load_fec_summary():
    """Load and display FEC performance summary."""
    try:
        fec_data = pd.read_csv('fec_performance.csv')
        print(f"\n🔧 FEC PERFORMANCE SUMMARY")
        print("=" * 50)
        
        if len(fec_data) > 0:
            latest = fec_data.iloc[-1]
            
            print(f"   Physical DER: {latest['PhysicalDER']*100:.2f}%")
            print(f"   Application DER (with FEC): {latest['ApplicationDER']*100:.2f}%")
            print(f"   FEC Improvement Factor: {latest['FecImprovement']:.1f}x")
            print(f"   Generations Processed: {latest['GenerationsProcessed']}")
            print(f"   Packets Recovered: {latest['PacketsRecovered']}")
            
            if latest['FecImprovement'] > 1.1:
                print(f"   ✅ FEC providing improvement")
            elif latest['GenerationsProcessed'] > 0:
                print(f"   🔧 FEC working but minimal improvement")
            else:
                print(f"   ❌ FEC not processing generations")
        
    except Exception as e:
        print(f"❌ Could not load FEC performance data: {e}")
    """Load and display FEC performance summary."""
    try:
        fec_data = pd.read_csv('fec_performance.csv')
        print(f"\n🔧 FEC PERFORMANCE SUMMARY")
        print("=" * 50)
        
        if len(fec_data) > 0:
            latest = fec_data.iloc[-1]
            
            print(f"   Physical DER: {latest['PhysicalDER']*100:.2f}%")
            print(f"   Application DER (with FEC): {latest['ApplicationDER']*100:.2f}%")
            print(f"   FEC Improvement Factor: {latest['FecImprovement']:.1f}x")
            print(f"   Generations Processed: {latest['GenerationsProcessed']}")
            print(f"   Packets Recovered: {latest['PacketsRecovered']}")
            
            if latest['FecImprovement'] > 1.1:
                print(f"   ✅ FEC providing improvement")
            elif latest['GenerationsProcessed'] > 0:
                print(f"   🔧 FEC working but minimal improvement")
            else:
                print(f"   ❌ FEC not processing generations")
        
    except Exception as e:
        print(f"❌ Could not load FEC performance data: {e}")

def main():
    """Main function."""
    # Load and analyze packet counts
    node_stats = load_and_analyze()
    
    # Print results
    print_results(node_stats)
    
    # Save results
    save_results(node_stats)
    
    # Analyze distributions per node
    distribution_stats = analyze_per_node_distributions(node_stats)
    
    # Create distribution plots
    create_distribution_plots(distribution_stats)
    
    # Show FEC summary
    load_fec_summary()

if __name__ == "__main__":
    main()