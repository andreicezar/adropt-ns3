// =======================================================
// CLEANED adropt-component.h (OPTIMIZATION ONLY)
// =======================================================

/*
 * ADRopt Component - Focused on optimization only
 * Statistics collection is handled by StatisticsCollectorComponent
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

class ADRoptComponent : public NetworkControllerComponent
{
public:
    
    struct ConfigOption
    {
        uint8_t dataRate;
        double txPower;
        uint8_t nbTrans;
        double predictedPER;
        double toa;
    };

    struct DeviceStats
    {
        std::list<EndDeviceStatus::ReceivedPacketInfo> packetHistory;
        uint32_t totalPackets;
        uint8_t currentNbTrans;
        uint32_t adrAdjustmentCount;
        Time lastNbTransChange;
        
        DeviceStats() 
            : totalPackets(0),
              currentNbTrans(1),
              adrAdjustmentCount(0)
        {}
    };

    // --- Method Declarations ---

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

    // Public API for basic info (needed by simulation)
    uint8_t GetCurrentNbTrans(uint32_t deviceAddr) const;
    uint32_t GetAdrAdjustmentCount(uint32_t deviceAddr) const;

private:
    bool RunADRoptAlgorithm(uint8_t* newDataRate,
                           double* newTxPowerDbm,
                           uint8_t* newNbTrans,
                           Ptr<EndDeviceStatus> status);
    double EstimateCurrentPER(Ptr<EndDeviceStatus> status);
    double PredictPER(uint8_t dataRate, double txPower, uint8_t nbTrans, Ptr<EndDeviceStatus> status);
    double CalculateToA(uint8_t dataRate, uint8_t nbTrans);
    double RxPowerToSNR(double rxPowerDbm);
    double CalculateFER(uint8_t dataRate, double snr);
    double GetSNRThreshold(uint8_t dataRate);
    uint8_t SfToDr(uint8_t sf);
    uint8_t DrToSf(uint8_t dr);
    uint8_t GetTxPowerIndex(double txPowerDbm);
    std::set<Address> GetActiveGateways(Ptr<EndDeviceStatus> status);
    double GetMeanSNRForGateway(const Address& gwAddr, Ptr<EndDeviceStatus> status);
    void UpdateTransmissionStats(uint32_t deviceAddr, uint8_t newNbTrans, uint8_t oldNbTrans);

    // Configuration parameters
    double m_perTarget;
    uint32_t m_historyRange;
    bool m_enablePowerControl;
    uint8_t m_payloadSize;

    // Device state tracking (for optimization only)
    std::map<uint32_t, DeviceStats> m_deviceStats;
    std::map<uint32_t, uint8_t> m_deviceNbTrans;

    // Trace sources for optimization events
    TracedCallback<uint32_t, uint8_t, double, uint8_t> m_adrAdjustmentTrace;

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
