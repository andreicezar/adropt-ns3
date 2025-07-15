/*
 * Simple ADRopt Component - Starting from scratch
 * Only uses basic LoRaWAN module information available by default
 */

#ifndef ADROPT_COMPONENT_H
#define ADROPT_COMPONENT_H

#include "network-controller-components.h"
#include "network-status.h"

#include "ns3/log.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/nstime.h"
#include "ns3/double.h"
#include "ns3/address.h"
#include "ns3/traced-callback.h"

#include <vector>
#include <set>
#include <map>
#include <list>

namespace ns3
{
namespace lorawan
{

/**
 * @brief Simple ADRopt implementation using only basic LoRaWAN data
 * 
 * This implementation focuses on the core ADR functionality using only:
 * - Received packet information (RxPower, gateway list)
 * - Current device parameters (SF, TxPower)
 * - Standard LoRaWAN calculations
 */
class ADRoptComponent : public NetworkControllerComponent
{
public:
    /**
     * @brief Configuration option for ADR optimization
     */
    struct ConfigOption
    {
        uint8_t dataRate;
        double txPower;
        uint8_t nbTrans;
        double predictedPER;
        double toa;
    };

    /**
     * @brief Device statistics tracking with enhanced transmission monitoring
     */
    struct DeviceStats
    {
        std::list<EndDeviceStatus::ReceivedPacketInfo> packetHistory;
        Time lastUpdateTime;
        uint32_t totalPackets;
        
        // Enhanced transmission tracking
        uint8_t currentNbTrans;           // Current NbTrans setting
        uint8_t previousNbTrans;          // Previous NbTrans for comparison
        uint32_t totalTransmissionAttempts; // Total number of transmission attempts
        uint32_t successfulTransmissions;   // Successful transmissions (packets received)
        uint32_t adrAdjustmentCount;        // Number of times ADR was applied
        
        // Transmission efficiency metrics
        double averageTransmissionsPerPacket; // Average attempts per successful packet
        Time lastNbTransChange;               // When NbTrans was last changed
        
        DeviceStats() 
            : lastUpdateTime(Time(0))
            , totalPackets(0)
            , currentNbTrans(1)
            , previousNbTrans(1)
            , totalTransmissionAttempts(0)
            , successfulTransmissions(0)
            , adrAdjustmentCount(0)
            , averageTransmissionsPerPacket(1.0)
            , lastNbTransChange(Time(0))
        {}
    };

    static TypeId GetTypeId();

    ADRoptComponent();
    virtual ~ADRoptComponent();

    // NetworkControllerComponent interface
    void OnReceivedPacket(Ptr<const Packet> packet,
                          Ptr<EndDeviceStatus> status,
                          Ptr<NetworkStatus> networkStatus) override;

    void BeforeSendingReply(Ptr<EndDeviceStatus> status,
                           Ptr<NetworkStatus> networkStatus) override;

    void OnFailedReply(Ptr<EndDeviceStatus> status,
                      Ptr<NetworkStatus> networkStatus) override;

    // Transmission tracking and reporting methods
    
    /**
     * @brief Get current NbTrans for a device
     * @param deviceAddr Device address
     * @return Current NbTrans value
     */
    uint8_t GetCurrentNbTrans(uint32_t deviceAddr) const;
    
    /**
     * @brief Get transmission efficiency for a device
     * @param deviceAddr Device address
     * @return Average number of transmissions per successful packet
     */
    double GetTransmissionEfficiency(uint32_t deviceAddr) const;
    
    /**
     * @brief Get total transmission attempts for a device
     * @param deviceAddr Device address
     * @return Total number of transmission attempts
     */
    uint32_t GetTotalTransmissionAttempts(uint32_t deviceAddr) const;
    
    /**
     * @brief Get ADR adjustment count for a device
     * @param deviceAddr Device address
     * @return Number of times ADR parameters were adjusted
     */
    uint32_t GetAdrAdjustmentCount(uint32_t deviceAddr) const;
    
    /**
     * @brief Print transmission statistics for all devices
     */
    void PrintTransmissionStatistics() const;
    
    /**
     * @brief Print transmission statistics for a specific device
     * @param deviceAddr Device address
     */
    void PrintDeviceTransmissionStats(uint32_t deviceAddr) const;

    /**
     * @brief Comprehensive packet tracking for error rate analysis
     */
    struct PacketTrackingStats
    {
        // Transmission tracking
        uint32_t totalPacketsSent;           // Total packets sent by device
        uint32_t packetsReceivedByGateways;  // Packets received by at least one gateway
        uint32_t packetsReceivedByNetworkServer; // Packets processed by network server
        
        // ADDED: SF and TxPower distribution tracking
        std::map<uint8_t, uint32_t> sfDistribution;        // SF -> packet count
        std::map<int, uint32_t> txPowerDistribution;       // TxPower(dBm) -> packet count
        
        // Per-gateway reception tracking
        std::map<uint32_t, uint32_t> perGatewayReceptions; // gwId -> packet count
        
        // Error rate calculations
        double deviceToGatewayErrorRate;     // (sent - received_gw) / sent
        double gatewayToNetworkServerErrorRate; // (received_gw - received_ns) / received_gw
        double endToEndErrorRate;            // (sent - received_ns) / sent
        
        // Timing information
        Time firstPacketTime;
        Time lastPacketTime;
        
        PacketTrackingStats() 
            : totalPacketsSent(0)
            , packetsReceivedByGateways(0)
            , packetsReceivedByNetworkServer(0)
            , deviceToGatewayErrorRate(0.0)
            , gatewayToNetworkServerErrorRate(0.0)
            , endToEndErrorRate(0.0)
            , firstPacketTime(Time::Max())
            , lastPacketTime(Time(0))
        {}
    };

    // Add these methods to the public section:

    /**
     * @brief Record a packet transmission from a device
     * @param deviceAddr Device address
     */
    void RecordPacketTransmission(uint32_t deviceAddr);

    /**
     * @brief Record packet reception at gateways
     * @param deviceAddr Device address
     * @param receivingGateways List of gateway IDs that received the packet
     */
    void RecordGatewayReception(uint32_t deviceAddr, const std::vector<uint32_t>& receivingGateways);

    /**
     * @brief Record packet processing at network server
     * @param deviceAddr Device address
     */
    void RecordNetworkServerReception(uint32_t deviceAddr);

    /**
     * @brief Calculate error rates for a specific device
     * @param deviceAddr Device address
     */
    void CalculateErrorRates(uint32_t deviceAddr);

    /**
     * @brief Get packet tracking statistics for a device
     * @param deviceAddr Device address
     * @return PacketTrackingStats structure
     */
    PacketTrackingStats GetPacketTrackingStats(uint32_t deviceAddr) const;

    /**
     * @brief Print comprehensive packet tracking statistics
     */
    void PrintPacketTrackingStatistics() const;

    /**
     * @brief Print packet tracking statistics for a specific device
     * @param deviceAddr Device address
     */
    void PrintDevicePacketTrackingStats(uint32_t deviceAddr) const;

    // Add to private section:
    std::map<uint32_t, PacketTrackingStats> m_packetTrackingStats;

    // Add new trace sources to the TypeId (in the .cc file):
    TracedCallback<uint32_t, uint32_t, uint32_t, double> m_errorRateTrace; // deviceAddr, sent, received, errorRate

private:
    // ADDED: Gateway ID extraction helper
    uint32_t ExtractGatewayId(const Address& gwAddr);
    // Core ADR algorithm
    bool RunADRoptAlgorithm(uint8_t* newDataRate,
                           double* newTxPowerDbm,
                           uint8_t* newNbTrans,
                           Ptr<EndDeviceStatus> status);

    // Basic calculations using only available data
    double EstimateCurrentPER(Ptr<EndDeviceStatus> status);
    double PredictPER(uint8_t dataRate, double txPower, uint8_t nbTrans, Ptr<EndDeviceStatus> status);
    double CalculateToA(uint8_t dataRate, uint8_t nbTrans);
    
    // Helper functions for basic LoRaWAN parameters
    double RxPowerToSNR(double rxPowerDbm);
    double CalculateFER(uint8_t dataRate, double snr);
    double GetSNRThreshold(uint8_t dataRate);
    uint8_t SfToDr(uint8_t sf);
    uint8_t DrToSf(uint8_t dr);
    uint8_t GetTxPowerIndex(double txPowerDbm);
    
    // Gateway analysis using basic data
    std::set<Address> GetActiveGateways(Ptr<EndDeviceStatus> status);
    double GetMeanSNRForGateway(const Address& gwAddr, Ptr<EndDeviceStatus> status);

    /**
     * @brief Update transmission statistics when NbTrans changes
     * @param deviceAddr Device address
     * @param newNbTrans New NbTrans value
     * @param oldNbTrans Previous NbTrans value
     */
    void UpdateTransmissionStats(uint32_t deviceAddr, uint8_t newNbTrans, uint8_t oldNbTrans);
    
    /**
     * @brief Calculate transmission efficiency for a device
     * @param deviceStats Device statistics
     * @return Transmission efficiency ratio
     */
    double CalculateTransmissionEfficiency(const DeviceStats& deviceStats) const;

    // Configuration parameters
    double m_perTarget;           // Target PER threshold
    uint32_t m_historyRange;      // Number of packets to consider
    bool m_enablePowerControl;    // Enable TX power optimization
    uint8_t m_payloadSize;        // Payload size for ToA calculation

    // Device state tracking
    std::map<uint32_t, DeviceStats> m_deviceStats;
    std::map<uint32_t, uint8_t> m_deviceNbTrans;

    // Trace sources for monitoring
    TracedCallback<uint32_t, uint8_t, uint8_t> m_nbTransChangedTrace;     // deviceAddr, oldNbTrans, newNbTrans
    TracedCallback<uint32_t, double> m_transmissionEfficiencyTrace;        // deviceAddr, efficiency
    TracedCallback<uint32_t, uint8_t, double, uint8_t> m_adrAdjustmentTrace; // deviceAddr, DR, txPower, nbTrans

    // LoRaWAN constants
    static const double NOISE_FLOOR_DBM;
    static const double BANDWIDTH_HZ;
    static const uint8_t MIN_TX_POWER;
    static const uint8_t MAX_TX_POWER;
    static const uint8_t PREAMBLE_LENGTH;
};

} // namespace lorawan
} // namespace ns3

#endif /* ADROPT_COMPONENT_H */