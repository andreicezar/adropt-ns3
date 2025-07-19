// =====================================================
// ENHANCED statistics-collector.h (WITH RSSI/SNR SUPPORT)
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
    // *** NEW: Radio measurement structure ***
    struct RadioMeasurement {
        double rssi = -999.0;
        double snr = -999.0;
        double snir = -999.0;
        uint32_t gatewayId = 0;
        Time timestamp = Time(0);
        uint8_t spreadingFactor = 12;
        double txPower = 14.0;
        uint32_t frequency = 868100000;
    };

    // *** NEW: Packet reception event for advanced tracking ***
    struct PacketReceptionEvent {
        uint32_t deviceAddr;
        Time timestamp;
        std::vector<RadioMeasurement> gatewayMeasurements;
        bool successful = false;
        uint8_t spreadingFactor = 12;
        double txPower = 14.0;
    };

    // *** ENHANCED: Device stats with radio measurements ***
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
        
        // *** NEW: Radio measurement fields ***
        std::list<RadioMeasurement> rssiHistory;
        std::list<RadioMeasurement> snrHistory;
        double averageRssi = -999.0;
        double averageSnr = -999.0;
        double bestRssi = -999.0;
        double worstRssi = -999.0;
        double bestSnr = -999.0;
        double worstSnr = -999.0;
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
        
        // *** NEW: Per-gateway radio measurements ***
        std::map<uint32_t, std::vector<RadioMeasurement>> perGatewayMeasurements;
    };

    // *** ENHANCED: Gateway stats with radio measurements ***
    struct GatewayStats {
        uint32_t packetsReceived = 0;
        uint32_t totalMeasurements = 0;
        std::string position = "Unknown";
        Time lastReceptionTime = Time(0);
        
        // *** NEW: Radio measurement fields ***
        std::list<RadioMeasurement> measurementHistory;
        double averageRssi = -999.0;
        double averageSnr = -999.0;
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
    
    // *** NEW: Radio measurement recording ***
    void RecordRadioMeasurement(uint32_t deviceAddr, uint32_t gatewayId,
                               double rssi, double snr, double snir,
                               uint8_t sf, double txPower, uint32_t frequency);
    void RecordPacketReception(const PacketReceptionEvent& event);
    
    // Automatic CSV export control
    void EnableAutomaticCsvExport(const std::string& filename = "adr_statistics.csv", uint32_t intervalSeconds = 300);
    void DisableAutomaticCsvExport();
    
    // *** NEW: Radio measurement CSV export ***
    void EnableRadioMeasurementCsv(const std::string& filename = "radio_measurements.csv", uint32_t intervalSeconds = 60);
    void DisableRadioMeasurementCsv();
    
    // Getters for statistics
    uint8_t GetCurrentNbTrans(uint32_t deviceAddr) const;
    double GetTransmissionEfficiency(uint32_t deviceAddr) const;
    uint32_t GetTotalTransmissionAttempts(uint32_t deviceAddr) const;
    uint32_t GetAdrAdjustmentCount(uint32_t deviceAddr) const;
    PacketTrackingStats GetPacketTrackingStats(uint32_t deviceAddr) const;
    DeviceStats GetDeviceStats(uint32_t deviceAddr) const;
    
    // *** NEW: Radio measurement getters ***
    std::vector<RadioMeasurement> GetRecentMeasurements(uint32_t deviceAddr, Time timeWindow) const;
    RadioMeasurement GetBestMeasurement(uint32_t deviceAddr) const;
    RadioMeasurement GetWorstMeasurement(uint32_t deviceAddr) const;
    std::map<uint32_t, double> GetPerGatewayAverageRssi(uint32_t deviceAddr) const;
    std::map<uint32_t, double> GetPerGatewayAverageSnr(uint32_t deviceAddr) const;
    
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
    
    // *** NEW: Radio measurement processing ***
    void UpdateRadioStatistics(uint32_t deviceAddr);
    void UpdateGatewayStatistics(uint32_t gatewayId);
    double CalculateSnir(double rssi, double noiseFloor = -174.0, double bandwidth = 125000.0) const;
    
    // Internal CSV export methods
    void WriteCsvData();
    void ScheduleNextCsvWrite();
    uint32_t FindNodeIdForDeviceAddr(uint32_t deviceAddr) const;
    
    // *** NEW: Radio measurement CSV methods ***
    void WriteRadioMeasurementCsv();
    void ScheduleNextRadioMeasurementWrite();

    // Core data storage
    std::map<uint32_t, DeviceStats> m_deviceStats;
    std::map<uint32_t, PacketTrackingStats> m_packetTrackingStats;
    std::map<uint32_t, GatewayStats> m_gatewayStats;
    std::map<uint32_t, uint32_t> m_nodeIdToDeviceAddr;

    // *** NEW: Packet reception history for advanced analysis ***
    std::list<PacketReceptionEvent> m_packetReceptionHistory;
    uint32_t m_maxHistorySize = 1000;

    // CSV export control
    bool m_csvExportEnabled;
    std::string m_csvFilename;
    uint32_t m_csvIntervalSeconds;
    bool m_csvHeaderWritten;

    // *** NEW: Radio measurement CSV export control ***
    bool m_radioMeasurementCsvEnabled;
    std::string m_radioMeasurementCsvFilename;
    uint32_t m_radioMeasurementCsvIntervalSeconds;
    bool m_radioMeasurementCsvHeaderWritten;

    // Trace sources for real-time monitoring
    TracedCallback<uint32_t, uint8_t, uint8_t> m_nbTransChangedTrace;
    TracedCallback<uint32_t, double> m_transmissionEfficiencyTrace;
    TracedCallback<uint32_t, uint32_t, uint32_t, double> m_errorRateTrace;
    
    // *** NEW: Radio measurement trace sources ***
    TracedCallback<uint32_t, uint32_t, double, double, double> m_radioMeasurementTrace;
    TracedCallback<uint32_t, double, double> m_linkQualityTrace;
};

} // namespace lorawan
} // namespace ns3

#endif // STATISTICS_COLLECTOR_H