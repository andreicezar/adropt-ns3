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
     * @brief Device statistics tracking
     */
    struct DeviceStats
    {
        std::list<EndDeviceStatus::ReceivedPacketInfo> packetHistory;
        Time lastUpdateTime;
        uint32_t totalPackets;
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

private:
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

    // Configuration parameters
    double m_perTarget;           // Target PER threshold
    uint32_t m_historyRange;      // Number of packets to consider
    bool m_enablePowerControl;    // Enable TX power optimization
    uint8_t m_payloadSize;        // Payload size for ToA calculation

    // Device state tracking
    std::map<uint32_t, DeviceStats> m_deviceStats;
    std::map<uint32_t, uint8_t> m_deviceNbTrans;

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