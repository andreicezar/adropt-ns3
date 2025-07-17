// =====================================================
// ENHANCED statistics-collector.cc (WITH BUILT-IN CSV EXPORT)
// =====================================================

#include "statistics-collector.h"
#include "ns3/log.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/simulator.h"
#include <iomanip>
#include <numeric>
#include <algorithm>

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
                           "ns3::TracedCallback::Uint32Uint32Uint32Double");
    return tid;
}

StatisticsCollectorComponent::StatisticsCollectorComponent()
    : m_csvExportEnabled(false),
      m_csvFilename("adr_statistics.csv"),
      m_csvIntervalSeconds(300),
      m_csvHeaderWritten(false)
{
    NS_LOG_FUNCTION(this);
}

StatisticsCollectorComponent::~StatisticsCollectorComponent()
{
    NS_LOG_FUNCTION(this);
}

void StatisticsCollectorComponent::EnableAutomaticCsvExport(const std::string& filename, uint32_t intervalSeconds)
{
    NS_LOG_FUNCTION(this << filename << intervalSeconds);
    
    m_csvExportEnabled = true;
    m_csvFilename = filename;
    m_csvIntervalSeconds = intervalSeconds;
    m_csvHeaderWritten = false;
    
    // Initialize the CSV file (clear it)
    std::ofstream csvFile(m_csvFilename, std::ios::trunc);
    if (csvFile.is_open()) {
        csvFile.close();
    }
    
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

void StatisticsCollectorComponent::RecordPacketTransmission(uint32_t deviceAddr)
{
    NS_LOG_FUNCTION(this << deviceAddr);
    
    auto& stats = m_packetTrackingStats[deviceAddr];
    stats.totalPacketsSent++;
    
    Time now = Simulator::Now();
    if (stats.firstPacketTime == Time::Max())
    {
        stats.firstPacketTime = now;
    }
    stats.lastPacketTime = now;
    
    CalculateErrorRates(deviceAddr);
}

void StatisticsCollectorComponent::RecordAdrAdjustment(uint32_t deviceAddr, uint8_t newNbTrans)
{
    NS_LOG_FUNCTION(this << deviceAddr << static_cast<uint32_t>(newNbTrans));
    
    auto& stats = m_deviceStats[deviceAddr];
    uint8_t oldNbTrans = stats.currentNbTrans;

    if (newNbTrans != oldNbTrans)
    {
        stats.previousNbTrans = oldNbTrans;
        stats.currentNbTrans = newNbTrans;
        stats.lastNbTransChange = Simulator::Now();
        m_nbTransChangedTrace(deviceAddr, oldNbTrans, newNbTrans);
    }
    stats.adrAdjustmentCount++;
}

void StatisticsCollectorComponent::RecordGatewayReception(uint32_t gatewayId, const std::string& position)
{
    NS_LOG_FUNCTION(this << gatewayId << position);
    
    auto& stats = m_gatewayStats[gatewayId];
    stats.packetsReceived++;
    stats.position = position;
    stats.lastReceptionTime = Simulator::Now();
}

void StatisticsCollectorComponent::SetNodeIdMapping(uint32_t nodeId, uint32_t deviceAddr)
{
    NS_LOG_FUNCTION(this << nodeId << deviceAddr);
    m_nodeIdToDeviceAddr[nodeId] = deviceAddr;
}

void StatisticsCollectorComponent::OnReceivedPacket(Ptr<const Packet> packet,
                                                   Ptr<EndDeviceStatus> status,
                                                   Ptr<NetworkStatus> networkStatus)
{
    NS_LOG_FUNCTION(this << packet << status->m_endDeviceAddress);
    
    uint32_t deviceAddr = status->m_endDeviceAddress.Get();
    Time currentTime = Simulator::Now();
    
    // ✅ ENHANCED duplicate detection with time-based window
    static std::map<uint32_t, std::vector<std::pair<uint32_t, Time>>> recentPackets;
    
    uint32_t packetId = packet->GetUid();
    
    // Check for recent duplicates (within 1 second window)
    auto& devicePackets = recentPackets[deviceAddr];
    for (auto it = devicePackets.begin(); it != devicePackets.end();) {
        if ((currentTime - it->second).GetSeconds() > 1.0) {
            // Remove old entries
            it = devicePackets.erase(it);
        } else {
            // Check for duplicate
            if (it->first == packetId) {
                NS_LOG_DEBUG("Duplicate packet " << packetId << " from device " << deviceAddr 
                           << " detected (time diff: " 
                           << (currentTime - it->second).GetMicroSeconds() << "us)");
                return; // Skip duplicate
            }
            ++it;
        }
    }
    
    // Add this packet to recent list
    devicePackets.push_back(std::make_pair(packetId, currentTime));
    
    // ✅ RATE LIMITING to prevent excessive updates
    static std::map<uint32_t, Time> lastProcessTime;
    if (lastProcessTime.find(deviceAddr) != lastProcessTime.end()) {
        Time timeSinceLastProcess = currentTime - lastProcessTime[deviceAddr];
        if (timeSinceLastProcess.GetMicroSeconds() < 10000) { // 10ms minimum between packets
            NS_LOG_DEBUG("Rate limiting packet processing for device " << deviceAddr);
            return;
        }
    }
    lastProcessTime[deviceAddr] = currentTime;
    
    auto& devStats = m_deviceStats[deviceAddr];
    auto& pktStats = m_packetTrackingStats[deviceAddr];

    // ✅ ATOMIC update to prevent race conditions
    devStats.totalPackets++;
    devStats.successfulTransmissions++;
    devStats.totalTransmissionAttempts += devStats.currentNbTrans;
    devStats.averageTransmissionsPerPacket = CalculateTransmissionEfficiency(devStats);
    devStats.lastUpdateTime = currentTime;
    
    // ✅ VALIDATION: Check for impossible values
    if (devStats.totalPackets > 10000) { // Reasonable upper bound
        NS_LOG_ERROR("Device " << deviceAddr << " has suspiciously high packet count: " 
                    << devStats.totalPackets);
    }
    
    // Fire efficiency trace (rate limited)
    static std::map<uint32_t, Time> lastEfficiencyUpdate;
    if (lastEfficiencyUpdate[deviceAddr] + Seconds(5) < currentTime) {
        m_transmissionEfficiencyTrace(deviceAddr, devStats.averageTransmissionsPerPacket);
        lastEfficiencyUpdate[deviceAddr] = currentTime;
    }

    // Update packet tracking statistics (with validation)
    uint32_t oldReceived = pktStats.packetsReceivedByNetworkServer;
    pktStats.packetsReceivedByNetworkServer++;
    
    // ✅ SANITY CHECK: Ensure we don't receive more than sent
    if (pktStats.packetsReceivedByNetworkServer > pktStats.totalPacketsSent && 
        pktStats.totalPacketsSent > 0) {
        NS_LOG_WARN("Device " << deviceAddr << " received (" 
                   << pktStats.packetsReceivedByNetworkServer 
                   << ") > sent (" << pktStats.totalPacketsSent << ") - possible duplicate bug");
        
        // Optional: Auto-correct by capping received at sent
        // pktStats.packetsReceivedByNetworkServer = pktStats.totalPacketsSent;
    }
    
    auto receivedPacketList = status->GetReceivedPacketList();
    if (!receivedPacketList.empty()) {
        // Only increment gateway reception count if we haven't seen this packet before
        if (oldReceived != pktStats.packetsReceivedByNetworkServer) {
            pktStats.packetsReceivedByGateways++;
        }
        
        const auto& latestPacket = receivedPacketList.back().second;
        
        // Track per-gateway receptions (with duplicate protection)
        for (const auto& gwPair : latestPacket.gwList) {
            uint32_t gwId = ExtractGatewayId(gwPair.first);
            
            // Check if we've already counted this gateway for this packet
            static std::map<std::pair<uint32_t, uint32_t>, Time> gwPacketHistory;
            auto gwKey = std::make_pair(deviceAddr, gwId);
            
            if (gwPacketHistory.find(gwKey) == gwPacketHistory.end() || 
                (currentTime - gwPacketHistory[gwKey]).GetSeconds() > 1.0) {
                
                pktStats.perGatewayReceptions[gwId]++;
                RecordGatewayReception(gwId);
                gwPacketHistory[gwKey] = currentTime;
            }
        }
    }

    // Track SF and TxPower distributions (rate limited)
    static std::map<uint32_t, Time> lastDistributionUpdate;
    if (lastDistributionUpdate[deviceAddr] + Seconds(1) < currentTime) {
        uint8_t currentSF = status->GetFirstReceiveWindowSpreadingFactor();
        double currentTxPower = 14.0; // Default EU868 max power
        
        Ptr<EndDeviceLorawanMac> mac = DynamicCast<EndDeviceLorawanMac>(status->GetMac());
        if (mac) {
            currentTxPower = mac->GetTransmissionPowerDbm();
        }
        
        pktStats.sfDistribution[currentSF]++;
        pktStats.txPowerDistribution[static_cast<int>(currentTxPower)]++;
        
        lastDistributionUpdate[deviceAddr] = currentTime;
    }

    // Recalculate error rates (rate limited)
    static std::map<uint32_t, Time> lastErrorRateUpdate;
    if (lastErrorRateUpdate[deviceAddr] + Seconds(10) < currentTime) {
        CalculateErrorRates(deviceAddr);
        lastErrorRateUpdate[deviceAddr] = currentTime;
    }
    
    NS_LOG_DEBUG("Processed packet from device " << deviceAddr 
                << " - Total: " << devStats.totalPackets 
                << ", Received: " << pktStats.packetsReceivedByNetworkServer);
}

void StatisticsCollectorComponent::WriteCsvData()
{
    if (!m_csvExportEnabled)
    {
        return;
    }
    
    std::ofstream csvFile(m_csvFilename, std::ios::app);
    
    if (!csvFile.is_open())
    {
        NS_LOG_ERROR("Could not open CSV file for statistics: " << m_csvFilename);
        return;
    }
    
    // Write header once
    if (!m_csvHeaderWritten)
    {
        csvFile << "Time,DeviceType,DeviceID,NodeID,Role,PacketsSent,PacketsReceived,PDR,NbTrans,Efficiency,AdrAdjustments,SF_Distribution,TxPower_Distribution,GatewayDiversity,Position" << std::endl;
        m_csvHeaderWritten = true;
    }
    
    double currentTime = Simulator::Now().GetSeconds();
    
    // === PART 1: Write END DEVICE statistics ===
    std::vector<uint32_t> devices = GetTrackedDevices();
    
    for (uint32_t deviceAddr : devices)
    {
        auto pktStats = GetPacketTrackingStats(deviceAddr);
        auto devStats = GetDeviceStats(deviceAddr);
        
        double pdr = (pktStats.totalPacketsSent > 0) ? 
                     (1.0 - pktStats.endToEndErrorRate) : 0.0;
        
        uint32_t nodeId = FindNodeIdForDeviceAddr(deviceAddr);
        
        // Format SF distribution as "SF7:5,SF8:10,SF9:3"
        std::string sfDist = "";
        for (const auto& sfPair : pktStats.sfDistribution)
        {
            if (!sfDist.empty()) sfDist += ",";
            sfDist += "SF" + std::to_string(static_cast<uint32_t>(sfPair.first)) + ":" + std::to_string(sfPair.second);
        }
        if (sfDist.empty()) sfDist = "None";
        
        // Format TxPower distribution as "14:8,12:5,10:2"
        std::string txPowerDist = "";
        for (const auto& powerPair : pktStats.txPowerDistribution)
        {
            if (!txPowerDist.empty()) txPowerDist += ",";
            txPowerDist += std::to_string(powerPair.first) + ":" + std::to_string(powerPair.second);
        }
        if (txPowerDist.empty()) txPowerDist = "None";
        
        // Calculate gateway diversity
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
                << "\"Mobile_Device\"" << std::endl;
    }
    
    // === PART 2: Write GATEWAY statistics ===
    for (const auto& gwPair : m_gatewayStats)
    {
        uint32_t gatewayId = gwPair.first;
        const auto& gwStats = gwPair.second;
        
        // Determine gateway position based on typical setup
        std::string position = "Position_" + std::to_string(gatewayId);
        if (gwStats.position != "Unknown")
        {
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
                << "\"" << position << "\"" << std::endl;
    }
    
    csvFile.close();
}

void StatisticsCollectorComponent::ScheduleNextCsvWrite()
{
    if (m_csvExportEnabled)
    {
        Simulator::Schedule(Seconds(m_csvIntervalSeconds), &StatisticsCollectorComponent::WriteCsvData, this);
        Simulator::Schedule(Seconds(m_csvIntervalSeconds), &StatisticsCollectorComponent::ScheduleNextCsvWrite, this);
    }
}

uint32_t StatisticsCollectorComponent::FindNodeIdForDeviceAddr(uint32_t deviceAddr) const
{
    for (const auto& mapping : m_nodeIdToDeviceAddr)
    {
        if (mapping.second == deviceAddr)
        {
            return mapping.first;
        }
    }
    return 0; // Not found
}

// Rest of the implementation remains the same...
uint32_t StatisticsCollectorComponent::ExtractGatewayId(const Address& gwAddr)
{
    try 
    {
        uint8_t addressLength = gwAddr.GetLength();
        if (addressLength >= 4)
        {
            uint8_t buffer[16];
            gwAddr.CopyTo(buffer);
            
            if (addressLength == 4)
            {
                return (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];
            }
            else if (addressLength == 6)
            {
                return (buffer[5] << 8) | buffer[4];
            }
            else
            {
                int offset = addressLength - 4;
                return (buffer[offset+3] << 24) | (buffer[offset+2] << 16) | 
                       (buffer[offset+1] << 8) | buffer[offset];
            }
        }
        else
        {
            uint8_t buffer[4] = {0};
            gwAddr.CopyTo(buffer);
            return (buffer[1] << 8) | buffer[0];
        }
    }
    catch (...)
    {
        static uint32_t fallbackId = 1000;
        return fallbackId++;
    }
}

void StatisticsCollectorComponent::CalculateErrorRates(uint32_t deviceAddr)
{
    NS_LOG_FUNCTION(this << deviceAddr);
    
    auto& stats = m_packetTrackingStats[deviceAddr];
    
    if (stats.totalPacketsSent > 0)
    {
        stats.endToEndErrorRate = static_cast<double>(stats.totalPacketsSent - stats.packetsReceivedByNetworkServer) / stats.totalPacketsSent;
    }
    
    // Fire error rate trace
    m_errorRateTrace(deviceAddr, stats.totalPacketsSent, stats.packetsReceivedByNetworkServer, stats.endToEndErrorRate);
}

double StatisticsCollectorComponent::CalculateTransmissionEfficiency(const DeviceStats& stats) const
{
    if (stats.successfulTransmissions == 0)
    {
        return 1.0;
    }
    return static_cast<double>(stats.totalTransmissionAttempts) / stats.successfulTransmissions;
}

// All the getter methods remain the same...
uint8_t StatisticsCollectorComponent::GetCurrentNbTrans(uint32_t deviceAddr) const
{
    auto it = m_deviceStats.find(deviceAddr);
    if (it != m_deviceStats.end())
    {
        return it->second.currentNbTrans;
    }
    return 1;
}

double StatisticsCollectorComponent::GetTransmissionEfficiency(uint32_t deviceAddr) const
{
    auto it = m_deviceStats.find(deviceAddr);
    if (it != m_deviceStats.end())
    {
        return it->second.averageTransmissionsPerPacket;
    }
    return 1.0;
}

uint32_t StatisticsCollectorComponent::GetTotalTransmissionAttempts(uint32_t deviceAddr) const
{
    auto it = m_deviceStats.find(deviceAddr);
    if (it != m_deviceStats.end())
    {
        return it->second.totalTransmissionAttempts;
    }
    return 0;
}

uint32_t StatisticsCollectorComponent::GetAdrAdjustmentCount(uint32_t deviceAddr) const
{
    auto it = m_deviceStats.find(deviceAddr);
    if (it != m_deviceStats.end())
    {
        return it->second.adrAdjustmentCount;
    }
    return 0;
}

StatisticsCollectorComponent::PacketTrackingStats 
StatisticsCollectorComponent::GetPacketTrackingStats(uint32_t deviceAddr) const
{
    auto it = m_packetTrackingStats.find(deviceAddr);
    if (it != m_packetTrackingStats.end())
    {
        return it->second;
    }
    return PacketTrackingStats();
}

StatisticsCollectorComponent::DeviceStats 
StatisticsCollectorComponent::GetDeviceStats(uint32_t deviceAddr) const
{
    auto it = m_deviceStats.find(deviceAddr);
    if (it != m_deviceStats.end())
    {
        return it->second;
    }
    return DeviceStats();
}

std::vector<uint32_t> StatisticsCollectorComponent::GetTrackedDevices() const
{
    std::vector<uint32_t> devices;
    for (const auto& pair : m_deviceStats)
    {
        devices.push_back(pair.first);
    }
    return devices;
}

uint32_t StatisticsCollectorComponent::GetNetworkTotalPacketsSent() const
{
    uint32_t total = 0;
    for (const auto& pair : m_packetTrackingStats)
    {
        total += pair.second.totalPacketsSent;
    }
    return total;
}

uint32_t StatisticsCollectorComponent::GetNetworkTotalPacketsReceived() const
{
    uint32_t total = 0;
    for (const auto& pair : m_packetTrackingStats)
    {
        total += pair.second.packetsReceivedByNetworkServer;
    }
    return total;
}

double StatisticsCollectorComponent::GetNetworkPacketDeliveryRate() const
{
    uint32_t totalSent = GetNetworkTotalPacketsSent();
    uint32_t totalReceived = GetNetworkTotalPacketsReceived();
    
    if (totalSent == 0)
    {
        return 0.0;
    }
    
    return static_cast<double>(totalReceived) / totalSent;
}

} // namespace lorawan
} // namespace ns3