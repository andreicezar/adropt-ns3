// =====================================================
// FIXED statistics-collector.cc (COMPILATION ERRORS RESOLVED)
// =====================================================

#include "statistics-collector.h"
#include "ns3/log.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/simulator.h"
#include "ns3/lorawan-mac-header.h"
#include "ns3/lora-frame-header.h"
#include "ns3/address.h"
#include <iomanip>
#include <numeric>
#include <algorithm>
#include <cmath>

namespace ns3 {
namespace lorawan {

NS_LOG_COMPONENT_DEFINE("StatisticsCollectorComponent");
NS_OBJECT_ENSURE_REGISTERED(StatisticsCollectorComponent);

TypeId StatisticsCollectorComponent::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::lorawan::StatisticsCollectorComponent")
            .SetGroupName("lorawan")
            .SetParent<NetworkControllerComponent>()
            .AddConstructor<StatisticsCollectorComponent>()
            .AddTraceSource("NbTransChanged", 
                           "Trace fired when NbTrans parameter changes",
                           MakeTraceSourceAccessor(&StatisticsCollectorComponent::m_nbTransChangedTrace),
                           "ns3::TracedCallback::Uint32Uint8Uint8")
            .AddTraceSource("TransmissionEfficiency", 
                           "Trace fired when transmission efficiency is updated",
                           MakeTraceSourceAccessor(&StatisticsCollectorComponent::m_transmissionEfficiencyTrace),
                           "ns3::TracedCallback::Uint32Double")
            .AddTraceSource("ErrorRate", 
                           "Trace fired when error rate is calculated",
                           MakeTraceSourceAccessor(&StatisticsCollectorComponent::m_errorRateTrace),
                           "ns3::TracedCallback::Uint32Uint32Uint32Double")
            .AddTraceSource("RadioMeasurement",
                           "Trace fired when radio measurement is recorded",
                           MakeTraceSourceAccessor(&StatisticsCollectorComponent::m_radioMeasurementTrace),
                           "ns3::TracedCallback::Uint32Uint32DoubleDoubleDouble")
            .AddTraceSource("LinkQuality",
                           "Trace fired when link quality is updated",
                           MakeTraceSourceAccessor(&StatisticsCollectorComponent::m_linkQualityTrace),
                           "ns3::TracedCallback::Uint32DoubleDouble");
    return tid;
}

StatisticsCollectorComponent::StatisticsCollectorComponent()
    : m_csvExportEnabled(false),
      m_csvFilename("adr_statistics.csv"),
      m_csvIntervalSeconds(300),
      m_csvHeaderWritten(false),
      m_radioMeasurementCsvEnabled(false),
      m_radioMeasurementCsvFilename("radio_measurements.csv"),
      m_radioMeasurementCsvIntervalSeconds(60),
      m_radioMeasurementCsvHeaderWritten(false)
{
    NS_LOG_FUNCTION(this);
}

StatisticsCollectorComponent::~StatisticsCollectorComponent()
{
    NS_LOG_FUNCTION(this);
}

// *** FIXED: Radio measurement recording ***
void StatisticsCollectorComponent::RecordRadioMeasurement(uint32_t deviceAddr, uint32_t gatewayId,
                                                         double rssi, double snr, double snir,
                                                         uint8_t sf, double txPower, uint32_t frequency)
{
    NS_LOG_FUNCTION(this << deviceAddr << gatewayId << rssi << snr << snir);
    
    Time currentTime = Simulator::Now();
    
    // Create radio measurement
    RadioMeasurement measurement;
    measurement.rssi = rssi;
    measurement.snr = snr;
    measurement.snir = snir;
    measurement.gatewayId = gatewayId;
    measurement.timestamp = currentTime;
    measurement.spreadingFactor = sf;
    measurement.txPower = txPower;
    measurement.frequency = frequency;
    
    // Update device statistics
    auto& devStats = m_deviceStats[deviceAddr];
    devStats.rssiHistory.push_back(measurement);
    devStats.snrHistory.push_back(measurement);
    
    // Limit history size to prevent memory issues
    const uint32_t maxHistorySize = 1000;
    if (devStats.rssiHistory.size() > maxHistorySize) {
        devStats.rssiHistory.pop_front();
        devStats.snrHistory.pop_front();
    }
    
    // Update gateway statistics
    auto& gwStats = m_gatewayStats[gatewayId];
    gwStats.measurementHistory.push_back(measurement);
    gwStats.totalMeasurements++;
    
    if (gwStats.measurementHistory.size() > maxHistorySize) {
        gwStats.measurementHistory.pop_front();
    }
    
    // Update packet tracking stats
    auto& pktStats = m_packetTrackingStats[deviceAddr];
    pktStats.perGatewayMeasurements[gatewayId].push_back(measurement);
    
    // Update running averages and extremes
    UpdateRadioStatistics(deviceAddr);
    UpdateGatewayStatistics(gatewayId);
    
    // Fire trace
    m_radioMeasurementTrace(deviceAddr, gatewayId, rssi, snr, snir);
    
    NS_LOG_DEBUG("Recorded radio measurement for device " << deviceAddr 
                << " via gateway " << gatewayId 
                << " - RSSI: " << rssi << "dBm, SNR: " << snr << "dB");
}

void StatisticsCollectorComponent::RecordPacketReception(const PacketReceptionEvent& event)
{
    NS_LOG_FUNCTION(this);
    
    m_packetReceptionHistory.push_back(event);
    
    // Limit history size
    if (m_packetReceptionHistory.size() > m_maxHistorySize) {
        m_packetReceptionHistory.pop_front();
    }
    
    NS_LOG_DEBUG("Recorded packet reception event for device " << event.deviceAddr 
                << " with " << event.gatewayMeasurements.size() << " gateway measurements");
}

void StatisticsCollectorComponent::UpdateRadioStatistics(uint32_t deviceAddr)
{
    auto& devStats = m_deviceStats[deviceAddr];
    
    if (devStats.rssiHistory.empty()) {
        return;
    }
    
    // Calculate averages
    double rssiSum = 0.0, snrSum = 0.0;
    uint32_t count = 0;
    
    // Initialize extremes with first measurement
    double bestRssi = devStats.rssiHistory.front().rssi;
    double worstRssi = devStats.rssiHistory.front().rssi;
    double bestSnr = devStats.snrHistory.front().snr;
    double worstSnr = devStats.snrHistory.front().snr;
    
    for (const auto& measurement : devStats.rssiHistory) {
        rssiSum += measurement.rssi;
        snrSum += measurement.snr;
        count++;
        
        // Update extremes
        if (measurement.rssi > bestRssi) bestRssi = measurement.rssi;
        if (measurement.rssi < worstRssi) worstRssi = measurement.rssi;
        if (measurement.snr > bestSnr) bestSnr = measurement.snr;
        if (measurement.snr < worstSnr) worstSnr = measurement.snr;
    }
    
    if (count > 0) {
        devStats.averageRssi = rssiSum / count;
        devStats.averageSnr = snrSum / count;
        devStats.bestRssi = bestRssi;
        devStats.worstRssi = worstRssi;
        devStats.bestSnr = bestSnr;
        devStats.worstSnr = worstSnr;
        
        // Fire link quality trace
        m_linkQualityTrace(deviceAddr, devStats.averageRssi, devStats.averageSnr);
    }
}

void StatisticsCollectorComponent::UpdateGatewayStatistics(uint32_t gatewayId)
{
    auto& gwStats = m_gatewayStats[gatewayId];
    
    if (gwStats.measurementHistory.empty()) {
        return;
    }
    
    double rssiSum = 0.0, snrSum = 0.0;
    uint32_t count = 0;
    
    for (const auto& measurement : gwStats.measurementHistory) {
        rssiSum += measurement.rssi;
        snrSum += measurement.snr;
        count++;
    }
    
    if (count > 0) {
        gwStats.averageRssi = rssiSum / count;
        gwStats.averageSnr = snrSum / count;
    }
}

double StatisticsCollectorComponent::CalculateSnir(double rssi, double noiseFloor, double bandwidth) const
{
    // SNIR = Signal / (Noise + Interference)
    // Simplified: assume interference is minimal compared to thermal noise
    double noiseFloorDbm = noiseFloor + 10.0 * std::log10(bandwidth) + 6.0; // 6dB noise figure
    return rssi - noiseFloorDbm;
}

// *** FIXED: Radio measurement CSV export ***
void StatisticsCollectorComponent::EnableRadioMeasurementCsv(const std::string& filename, uint32_t intervalSeconds)
{
    NS_LOG_FUNCTION(this << filename << intervalSeconds);
    
    m_radioMeasurementCsvEnabled = true;
    m_radioMeasurementCsvFilename = filename;
    m_radioMeasurementCsvIntervalSeconds = intervalSeconds;
    m_radioMeasurementCsvHeaderWritten = false;
    
    // Initialize the CSV file
    std::ofstream csvFile(m_radioMeasurementCsvFilename, std::ios::trunc);
    if (csvFile.is_open()) {
        csvFile.close();
    }
    
    // Schedule the first write
    ScheduleNextRadioMeasurementWrite();
    
    NS_LOG_INFO("Radio measurement CSV export enabled: " << filename << " every " << intervalSeconds << " seconds");
}

void StatisticsCollectorComponent::DisableRadioMeasurementCsv()
{
    NS_LOG_FUNCTION(this);
    m_radioMeasurementCsvEnabled = false;
    NS_LOG_INFO("Radio measurement CSV export disabled");
}

void StatisticsCollectorComponent::WriteRadioMeasurementCsv()
{
    if (!m_radioMeasurementCsvEnabled) {
        return;
    }
    
    std::ofstream csvFile(m_radioMeasurementCsvFilename, std::ios::app);
    
    if (!csvFile.is_open()) {
        NS_LOG_ERROR("Could not open radio measurement CSV file: " << m_radioMeasurementCsvFilename);
        return;
    }
    
    // Write header once
    if (!m_radioMeasurementCsvHeaderWritten) {
        csvFile << "Time,DeviceAddr,GatewayID,RSSI_dBm,SNR_dB,SNIR_dB,SpreadingFactor,TxPower_dBm,Frequency_Hz,GatewayPosition,PacketSuccess" << std::endl;
        m_radioMeasurementCsvHeaderWritten = true;
    }
    
    // Export recent measurements from all devices
    for (const auto& devicePair : m_deviceStats) {
        uint32_t deviceAddr = devicePair.first;
        const auto& devStats = devicePair.second;
        
        // Export recent measurements (last interval)
        Time cutoffTime = Simulator::Now() - Seconds(m_radioMeasurementCsvIntervalSeconds);
        
        for (const auto& measurement : devStats.rssiHistory) {
            if (measurement.timestamp >= cutoffTime) {
                // Get gateway position
                std::string gwPosition = "Unknown";
                auto gwIt = m_gatewayStats.find(measurement.gatewayId);
                if (gwIt != m_gatewayStats.end()) {
                    gwPosition = gwIt->second.position;
                }
                
                csvFile << std::fixed << std::setprecision(1) << measurement.timestamp.GetSeconds() << ","
                        << deviceAddr << ","
                        << measurement.gatewayId << ","
                        << std::setprecision(2) << measurement.rssi << ","
                        << std::setprecision(2) << measurement.snr << ","
                        << std::setprecision(2) << measurement.snir << ","
                        << static_cast<uint32_t>(measurement.spreadingFactor) << ","
                        << std::setprecision(1) << measurement.txPower << ","
                        << measurement.frequency << ","
                        << "\"" << gwPosition << "\","
                        << "1" << std::endl; // Assuming successful if we have measurement
            }
        }
    }
    
    csvFile.close();
}

void StatisticsCollectorComponent::ScheduleNextRadioMeasurementWrite()
{
    if (m_radioMeasurementCsvEnabled) {
        Simulator::Schedule(Seconds(m_radioMeasurementCsvIntervalSeconds), 
                           &StatisticsCollectorComponent::WriteRadioMeasurementCsv, this);
        Simulator::Schedule(Seconds(m_radioMeasurementCsvIntervalSeconds), 
                           &StatisticsCollectorComponent::ScheduleNextRadioMeasurementWrite, this);
    }
}

// *** FIXED: Radio measurement getters ***
std::vector<StatisticsCollectorComponent::RadioMeasurement> 
StatisticsCollectorComponent::GetRecentMeasurements(uint32_t deviceAddr, Time timeWindow) const
{
    std::vector<RadioMeasurement> recentMeasurements;
    
    auto it = m_deviceStats.find(deviceAddr);
    if (it != m_deviceStats.end()) {
        Time cutoffTime = Simulator::Now() - timeWindow;
        
        for (const auto& measurement : it->second.rssiHistory) {
            if (measurement.timestamp >= cutoffTime) {
                recentMeasurements.push_back(measurement);
            }
        }
    }
    
    return recentMeasurements;
}

StatisticsCollectorComponent::RadioMeasurement 
StatisticsCollectorComponent::GetBestMeasurement(uint32_t deviceAddr) const
{
    RadioMeasurement best;
    
    auto it = m_deviceStats.find(deviceAddr);
    if (it != m_deviceStats.end()) {
        best.rssi = it->second.bestRssi;
        best.snr = it->second.bestSnr;
    }
    
    return best;
}

StatisticsCollectorComponent::RadioMeasurement 
StatisticsCollectorComponent::GetWorstMeasurement(uint32_t deviceAddr) const
{
    RadioMeasurement worst;
    
    auto it = m_deviceStats.find(deviceAddr);
    if (it != m_deviceStats.end()) {
        worst.rssi = it->second.worstRssi;
        worst.snr = it->second.worstSnr;
    }
    
    return worst;
}

std::map<uint32_t, double> 
StatisticsCollectorComponent::GetPerGatewayAverageRssi(uint32_t deviceAddr) const
{
    std::map<uint32_t, double> perGatewayRssi;
    
    auto it = m_packetTrackingStats.find(deviceAddr);
    if (it != m_packetTrackingStats.end()) {
        for (const auto& gwPair : it->second.perGatewayMeasurements) {
            uint32_t gatewayId = gwPair.first;
            const auto& measurements = gwPair.second;
            
            if (!measurements.empty()) {
                double sum = 0.0;
                for (const auto& measurement : measurements) {
                    sum += measurement.rssi;
                }
                perGatewayRssi[gatewayId] = sum / measurements.size();
            }
        }
    }
    
    return perGatewayRssi;
}

std::map<uint32_t, double> 
StatisticsCollectorComponent::GetPerGatewayAverageSnr(uint32_t deviceAddr) const
{
    std::map<uint32_t, double> perGatewaySnr;
    
    auto it = m_packetTrackingStats.find(deviceAddr);
    if (it != m_packetTrackingStats.end()) {
        for (const auto& gwPair : it->second.perGatewayMeasurements) {
            uint32_t gatewayId = gwPair.first;
            const auto& measurements = gwPair.second;
            
            if (!measurements.empty()) {
                double sum = 0.0;
                for (const auto& measurement : measurements) {
                    sum += measurement.snr;
                }
                perGatewaySnr[gatewayId] = sum / measurements.size();
            }
        }
    }
    
    return perGatewaySnr;
}

// *** FIXED: Enhanced OnReceivedPacket with proper radio measurement extraction ***
void StatisticsCollectorComponent::OnReceivedPacket(Ptr<const Packet> packet,
                                                   Ptr<EndDeviceStatus> status,
                                                   Ptr<NetworkStatus> networkStatus)
{
    NS_LOG_FUNCTION(this << packet << status->m_endDeviceAddress);
    
    uint32_t deviceAddr = status->m_endDeviceAddress.Get();
    Time currentTime = Simulator::Now();
    
    // Enhanced duplicate detection
    static std::map<uint32_t, std::vector<std::pair<uint32_t, Time>>> recentPackets;
    uint32_t packetId = packet->GetUid();
    
    auto& devicePackets = recentPackets[deviceAddr];
    for (auto it = devicePackets.begin(); it != devicePackets.end();) {
        if ((currentTime - it->second).GetSeconds() > 1.0) {
            it = devicePackets.erase(it);
        } else {
            if (it->first == packetId) {
                NS_LOG_DEBUG("Duplicate packet " << packetId << " detected");
                return;
            }
            ++it;
        }
    }
    devicePackets.push_back(std::make_pair(packetId, currentTime));
    
    // Process radio measurements from received packet list
    auto receivedPacketList = status->GetReceivedPacketList();
    if (!receivedPacketList.empty()) {
        const auto& latestPacket = receivedPacketList.back().second;
        
        // Extract radio measurements from each gateway
        for (const auto& gwPair : latestPacket.gwList) {
            uint32_t gwId = ExtractGatewayId(gwPair.first);
            double rxPower = gwPair.second.rxPower;
            
            // *** FIXED: Calculate SNR from RSSI instead of accessing non-existent field ***
            double snr = CalculateSnir(rxPower);  // This calculates SNR from received power
            
            // *** FIXED: Use correct method signature ***
            double snir = CalculateSnir(rxPower, -174.0, 125000.0);
            
            // Get current transmission parameters
            uint8_t sf = status->GetFirstReceiveWindowSpreadingFactor();
            double txPower = 14.0; // Default, should get from MAC
            
            Ptr<EndDeviceLorawanMac> mac = DynamicCast<EndDeviceLorawanMac>(status->GetMac());
            if (mac) {
                txPower = mac->GetTransmissionPowerDbm();
            }
            
            // Record the measurement
            RecordRadioMeasurement(deviceAddr, gwId, rxPower, snr, snir, sf, txPower, 868100000);
        }
    }
    
    // Continue with existing packet processing...
    auto& devStats = m_deviceStats[deviceAddr];
    auto& pktStats = m_packetTrackingStats[deviceAddr];
    
    devStats.totalPackets++;
    devStats.successfulTransmissions++;
    devStats.totalTransmissionAttempts += devStats.currentNbTrans;
    devStats.averageTransmissionsPerPacket = CalculateTransmissionEfficiency(devStats);
    devStats.lastUpdateTime = currentTime;
    
    pktStats.packetsReceivedByNetworkServer++;
    
    // Continue with existing logic...
    CalculateErrorRates(deviceAddr);
}

// *** EXISTING METHODS - Include all your existing methods here ***

void StatisticsCollectorComponent::RecordPacketTransmission(uint32_t deviceAddr)
{
    NS_LOG_FUNCTION(this << deviceAddr);
    
    auto& devStats = m_deviceStats[deviceAddr];
    auto& pktStats = m_packetTrackingStats[deviceAddr];
    
    pktStats.totalPacketsSent++;
    devStats.totalTransmissionAttempts += devStats.currentNbTrans;
    
    Time now = Simulator::Now();
    if (pktStats.firstPacketTime == Time::Max()) {
        pktStats.firstPacketTime = now;
    }
    pktStats.lastPacketTime = now;
    
    devStats.lastUpdateTime = now;
    
    NS_LOG_DEBUG("Device " << deviceAddr << " transmitted packet #" << pktStats.totalPacketsSent);
}

void StatisticsCollectorComponent::RecordAdrAdjustment(uint32_t deviceAddr, uint8_t newNbTrans)
{
    NS_LOG_FUNCTION(this << deviceAddr << static_cast<uint32_t>(newNbTrans));
    
    auto& devStats = m_deviceStats[deviceAddr];
    uint8_t oldNbTrans = devStats.currentNbTrans;
    
    if (newNbTrans != oldNbTrans) {
        devStats.previousNbTrans = oldNbTrans;
        devStats.currentNbTrans = newNbTrans;
        devStats.adrAdjustmentCount++;
        devStats.lastNbTransChange = Simulator::Now();
        
        // Fire trace
        m_nbTransChangedTrace(deviceAddr, oldNbTrans, newNbTrans);
        
        NS_LOG_INFO("Device " << deviceAddr << " NbTrans changed: " 
                   << static_cast<uint32_t>(oldNbTrans) << " -> " 
                   << static_cast<uint32_t>(newNbTrans));
    }
}

void StatisticsCollectorComponent::RecordGatewayReception(uint32_t gatewayId, const std::string& position)
{
    NS_LOG_FUNCTION(this << gatewayId << position);
    
    auto& gwStats = m_gatewayStats[gatewayId];
    gwStats.packetsReceived++;
    gwStats.lastReceptionTime = Simulator::Now();
    
    if (position != "Unknown") {
        gwStats.position = position;
    }
    
    NS_LOG_DEBUG("Gateway " << gatewayId << " received packet #" << gwStats.packetsReceived);
}

void StatisticsCollectorComponent::SetNodeIdMapping(uint32_t nodeId, uint32_t deviceAddr)
{
    NS_LOG_FUNCTION(this << nodeId << deviceAddr);
    m_nodeIdToDeviceAddr[nodeId] = deviceAddr;
    NS_LOG_DEBUG("Node ID mapping: " << nodeId << " -> " << deviceAddr);
}

void StatisticsCollectorComponent::EnableAutomaticCsvExport(const std::string& filename, uint32_t intervalSeconds)
{
    NS_LOG_FUNCTION(this << filename << intervalSeconds);
    
    m_csvExportEnabled = true;
    m_csvFilename = filename;
    m_csvIntervalSeconds = intervalSeconds;
    m_csvHeaderWritten = false;
    
    // Schedule the first write
    ScheduleNextCsvWrite();
    
    NS_LOG_INFO("Automatic CSV export enabled: " << filename << " every " << intervalSeconds << " seconds");
}

void StatisticsCollectorComponent::DisableAutomaticCsvExport()
{
    NS_LOG_FUNCTION(this);
    m_csvExportEnabled = false;
    NS_LOG_INFO("Automatic CSV export disabled");
}

uint8_t StatisticsCollectorComponent::GetCurrentNbTrans(uint32_t deviceAddr) const
{
    auto it = m_deviceStats.find(deviceAddr);
    return (it != m_deviceStats.end()) ? it->second.currentNbTrans : 1;
}

double StatisticsCollectorComponent::GetTransmissionEfficiency(uint32_t deviceAddr) const
{
    auto it = m_deviceStats.find(deviceAddr);
    return (it != m_deviceStats.end()) ? it->second.averageTransmissionsPerPacket : 1.0;
}

uint32_t StatisticsCollectorComponent::GetTotalTransmissionAttempts(uint32_t deviceAddr) const
{
    auto it = m_deviceStats.find(deviceAddr);
    return (it != m_deviceStats.end()) ? it->second.totalTransmissionAttempts : 0;
}

uint32_t StatisticsCollectorComponent::GetAdrAdjustmentCount(uint32_t deviceAddr) const
{
    auto it = m_deviceStats.find(deviceAddr);
    return (it != m_deviceStats.end()) ? it->second.adrAdjustmentCount : 0;
}

StatisticsCollectorComponent::PacketTrackingStats 
StatisticsCollectorComponent::GetPacketTrackingStats(uint32_t deviceAddr) const
{
    auto it = m_packetTrackingStats.find(deviceAddr);
    return (it != m_packetTrackingStats.end()) ? it->second : PacketTrackingStats();
}

StatisticsCollectorComponent::DeviceStats 
StatisticsCollectorComponent::GetDeviceStats(uint32_t deviceAddr) const
{
    auto it = m_deviceStats.find(deviceAddr);
    return (it != m_deviceStats.end()) ? it->second : DeviceStats();
}

std::vector<uint32_t> StatisticsCollectorComponent::GetTrackedDevices() const
{
    std::vector<uint32_t> devices;
    for (const auto& pair : m_deviceStats) {
        devices.push_back(pair.first);
    }
    return devices;
}

uint32_t StatisticsCollectorComponent::GetNetworkTotalPacketsSent() const
{
    uint32_t total = 0;
    for (const auto& pair : m_packetTrackingStats) {
        total += pair.second.totalPacketsSent;
    }
    return total;
}

uint32_t StatisticsCollectorComponent::GetNetworkTotalPacketsReceived() const
{
    uint32_t total = 0;
    for (const auto& pair : m_packetTrackingStats) {
        total += pair.second.packetsReceivedByNetworkServer;
    }
    return total;
}

double StatisticsCollectorComponent::GetNetworkPacketDeliveryRate() const
{
    uint32_t totalSent = GetNetworkTotalPacketsSent();
    uint32_t totalReceived = GetNetworkTotalPacketsReceived();
    
    if (totalSent == 0) return 0.0;
    return static_cast<double>(totalReceived) / totalSent;
}

uint32_t StatisticsCollectorComponent::ExtractGatewayId(const Address& gwAddr)
{
    // Simple and robust approach: use address bytes for hashing
    uint8_t buffer[Address::MAX_SIZE];
    uint32_t size = gwAddr.CopyTo(buffer);  // CopyTo only takes one argument
    
    // Create a simple hash from the address bytes
    uint32_t hash = 0;
    for (uint32_t i = 0; i < size; ++i) {
        hash = (hash * 31) + buffer[i];
    }
    
    // Return gateway ID in reasonable range (0-999)
    uint32_t gatewayId = hash % 1000;
    
    NS_LOG_DEBUG("Extracted gateway ID " << gatewayId << " from address (size: " << size << ")");
    return gatewayId;
}

void StatisticsCollectorComponent::CalculateErrorRates(uint32_t deviceAddr)
{
    auto& pktStats = m_packetTrackingStats[deviceAddr];
    
    if (pktStats.totalPacketsSent > 0) {
        double successRate = static_cast<double>(pktStats.packetsReceivedByNetworkServer) / pktStats.totalPacketsSent;
        pktStats.endToEndErrorRate = 1.0 - successRate;
        
        // Fire trace
        m_errorRateTrace(deviceAddr, pktStats.totalPacketsSent, 
                        pktStats.packetsReceivedByNetworkServer, pktStats.endToEndErrorRate);
    }
}

double StatisticsCollectorComponent::CalculateTransmissionEfficiency(const DeviceStats& deviceStats) const
{
    if (deviceStats.successfulTransmissions == 0) return 1.0;
    return static_cast<double>(deviceStats.totalTransmissionAttempts) / deviceStats.successfulTransmissions;
}

void StatisticsCollectorComponent::WriteCsvData()
{
    if (!m_csvExportEnabled) {
        return;
    }
    
    std::ofstream csvFile(m_csvFilename, std::ios::app);
    
    if (!csvFile.is_open()) {
        NS_LOG_ERROR("Could not open CSV file for statistics: " << m_csvFilename);
        return;
    }
    
    // Write header once
    if (!m_csvHeaderWritten) {
        csvFile << "Time,DeviceType,DeviceID,NodeID,Role,PacketsSent,PacketsReceived,PDR,NbTrans,Efficiency,AdrAdjustments,SF_Distribution,TxPower_Distribution,GatewayDiversity,Position,AvgRSSI,AvgSNR,BestRSSI,WorstRSSI,BestSNR,WorstSNR" << std::endl;
        m_csvHeaderWritten = true;
    }
    
    double currentTime = Simulator::Now().GetSeconds();
    
    // Write END DEVICE statistics with radio measurements
    std::vector<uint32_t> devices = GetTrackedDevices();
    
    for (uint32_t deviceAddr : devices) {
        auto pktStats = GetPacketTrackingStats(deviceAddr);
        auto devStats = GetDeviceStats(deviceAddr);
        
        double pdr = (pktStats.totalPacketsSent > 0) ? 
                     (1.0 - pktStats.endToEndErrorRate) : 0.0;
        
        uint32_t nodeId = FindNodeIdForDeviceAddr(deviceAddr);
        
        // Format distributions
        std::string sfDist = "";
        for (const auto& sfPair : pktStats.sfDistribution) {
            if (!sfDist.empty()) sfDist += ",";
            sfDist += "SF" + std::to_string(static_cast<uint32_t>(sfPair.first)) + ":" + std::to_string(sfPair.second);
        }
        if (sfDist.empty()) sfDist = "None";
        
        std::string txPowerDist = "";
        for (const auto& powerPair : pktStats.txPowerDistribution) {
            if (!txPowerDist.empty()) txPowerDist += ",";
            txPowerDist += std::to_string(powerPair.first) + ":" + std::to_string(powerPair.second);
        }
        if (txPowerDist.empty()) txPowerDist = "None";
        
        uint32_t gatewayDiversity = pktStats.perGatewayReceptions.size();
        
        csvFile << std::fixed << std::setprecision(1) << currentTime << ","
                << "EndDevice,"
                << "ED_" << deviceAddr << ","
                << nodeId << ","
                << "LoRaWAN_Transmitter,"
                << pktStats.totalPacketsSent << ","
                << pktStats.packetsReceivedByNetworkServer << ","
                << std::setprecision(4) << pdr << ","
                << static_cast<uint32_t>(devStats.currentNbTrans) << ","
                << std::setprecision(3) << devStats.averageTransmissionsPerPacket << ","
                << devStats.adrAdjustmentCount << ","
                << "\"" << sfDist << "\","
                << "\"" << txPowerDist << "\","
                << gatewayDiversity << ","
                << "\"Mobile_Device\","
                << std::setprecision(2) << devStats.averageRssi << ","
                << std::setprecision(2) << devStats.averageSnr << ","
                << std::setprecision(2) << devStats.bestRssi << ","
                << std::setprecision(2) << devStats.worstRssi << ","
                << std::setprecision(2) << devStats.bestSnr << ","
                << std::setprecision(2) << devStats.worstSnr << std::endl;
    }
    
    // Write GATEWAY statistics with radio measurements
    for (const auto& gwPair : m_gatewayStats) {
        uint32_t gatewayId = gwPair.first;
        const auto& gwStats = gwPair.second;
        
        std::string position = "Position_" + std::to_string(gatewayId);
        if (gwStats.position != "Unknown") {
            position = gwStats.position;
        }
        
        csvFile << std::fixed << std::setprecision(1) << currentTime << ","
                << "Gateway,"
                << "GW_" << gatewayId << ","
                << gatewayId << ","
                << "LoRaWAN_Receiver,"
                << "N/A,"
                << gwStats.packetsReceived << ","
                << "N/A,"
                << "N/A,"
                << "N/A,"
                << "N/A,"
                << "\"N/A\","
                << "\"N/A\","
                << "N/A,"
                << "\"" << position << "\","
                << std::setprecision(2) << gwStats.averageRssi << ","
                << std::setprecision(2) << gwStats.averageSnr << ","
                << "N/A,N/A,N/A,N/A" << std::endl;
    }
    
    csvFile.close();
}

void StatisticsCollectorComponent::ScheduleNextCsvWrite()
{
    if (m_csvExportEnabled) {
        Simulator::Schedule(Seconds(m_csvIntervalSeconds), 
                           &StatisticsCollectorComponent::WriteCsvData, this);
        Simulator::Schedule(Seconds(m_csvIntervalSeconds), 
                           &StatisticsCollectorComponent::ScheduleNextCsvWrite, this);
    }
}

uint32_t StatisticsCollectorComponent::FindNodeIdForDeviceAddr(uint32_t deviceAddr) const
{
    for (const auto& pair : m_nodeIdToDeviceAddr) {
        if (pair.second == deviceAddr) {
            return pair.first;
        }
    }
    return 0; // Default if not found
}

} // namespace lorawan
} // namespace ns3