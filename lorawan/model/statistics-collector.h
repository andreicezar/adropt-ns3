// =====================================================
// ENHANCED statistics-collector.h (WITH BUILT-IN CSV EXPORT)
// =====================================================

#ifndef STATISTICS_COLLECTOR_H
#define STATISTICS_COLLECTOR_H

#include "ns3/network-controller-components.h"
#include "ns3/network-status.h"
#include "ns3/nstime.h"
#include "ns3/traced-callback.h"
#include <map>
#include <vector>
#include <list>
#include <string>
#include <fstream>

namespace ns3 {
namespace lorawan {

class StatisticsCollectorComponent : public NetworkControllerComponent
{
public:
    struct DeviceStats {
        Time lastUpdateTime;
        uint32_t totalPackets = 0;
        uint8_t currentNbTrans = 1;
        uint8_t previousNbTrans = 1;
        uint32_t totalTransmissionAttempts = 0;
        uint32_t successfulTransmissions = 0;
        uint32_t adrAdjustmentCount = 0;
        double averageTransmissionsPerPacket = 1.0;
        Time lastNbTransChange = Time(0);
    };

    struct PacketTrackingStats {
        uint32_t totalPacketsSent = 0;
        uint32_t packetsReceivedByGateways = 0;
        uint32_t packetsReceivedByNetworkServer = 0;
        std::map<uint8_t, uint32_t> sfDistribution;
        std::map<int, uint32_t> txPowerDistribution;
        std::map<uint32_t, uint32_t> perGatewayReceptions;
        double endToEndErrorRate = 0.0;
        Time firstPacketTime = Time::Max();
        Time lastPacketTime = Time(0);
    };

    struct GatewayStats {
        uint32_t packetsReceived = 0;
        std::string position = "Unknown";
        Time lastReceptionTime = Time(0);
    };

    static TypeId GetTypeId();
    StatisticsCollectorComponent();
    virtual ~StatisticsCollectorComponent();

    // Component hooks
    void OnReceivedPacket(Ptr<const Packet> packet, Ptr<EndDeviceStatus> status, Ptr<NetworkStatus> networkStatus) override;
    void OnFailedReply(Ptr<EndDeviceStatus> status, Ptr<NetworkStatus> networkStatus) override {}
    void BeforeSendingReply(Ptr<EndDeviceStatus> status, Ptr<NetworkStatus> networkStatus) override {}

    // Public API for data collection
    void RecordPacketTransmission(uint32_t deviceAddr);
    void RecordAdrAdjustment(uint32_t deviceAddr, uint8_t newNbTrans);
    void RecordGatewayReception(uint32_t gatewayId, const std::string& position = "Unknown");
    void SetNodeIdMapping(uint32_t nodeId, uint32_t deviceAddr);
    
    // Automatic CSV export control
    void EnableAutomaticCsvExport(const std::string& filename = "adr_statistics.csv", uint32_t intervalSeconds = 300);
    void DisableAutomaticCsvExport();
    
    // Getters for statistics
    uint8_t GetCurrentNbTrans(uint32_t deviceAddr) const;
    double GetTransmissionEfficiency(uint32_t deviceAddr) const;
    uint32_t GetTotalTransmissionAttempts(uint32_t deviceAddr) const;
    uint32_t GetAdrAdjustmentCount(uint32_t deviceAddr) const;
    PacketTrackingStats GetPacketTrackingStats(uint32_t deviceAddr) const;
    DeviceStats GetDeviceStats(uint32_t deviceAddr) const;
    
    // Get all device addresses being tracked
    std::vector<uint32_t> GetTrackedDevices() const;
    
    // Network-wide statistics
    uint32_t GetNetworkTotalPacketsSent() const;
    uint32_t GetNetworkTotalPacketsReceived() const;
    double GetNetworkPacketDeliveryRate() const;

private:
    uint32_t ExtractGatewayId(const Address& gwAddr);
    void CalculateErrorRates(uint32_t deviceAddr);
    double CalculateTransmissionEfficiency(const DeviceStats& deviceStats) const;
    
    // Internal CSV export methods
    void WriteCsvData();
    void ScheduleNextCsvWrite();
    uint32_t FindNodeIdForDeviceAddr(uint32_t deviceAddr) const;

    std::map<uint32_t, DeviceStats> m_deviceStats;
    std::map<uint32_t, PacketTrackingStats> m_packetTrackingStats;
    std::map<uint32_t, GatewayStats> m_gatewayStats;
    std::map<uint32_t, uint32_t> m_nodeIdToDeviceAddr;

    // CSV export control
    bool m_csvExportEnabled;
    std::string m_csvFilename;
    uint32_t m_csvIntervalSeconds;
    bool m_csvHeaderWritten;

    // Trace sources for real-time monitoring
    TracedCallback<uint32_t, uint8_t, uint8_t> m_nbTransChangedTrace;
    TracedCallback<uint32_t, double> m_transmissionEfficiencyTrace;
    TracedCallback<uint32_t, uint32_t, uint32_t, double> m_errorRateTrace;
};

} // namespace lorawan
} // namespace ns3

#endif // STATISTICS_COLLECTOR_H