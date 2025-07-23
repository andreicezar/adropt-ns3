// model/fec-component.h

#ifndef FEC_COMPONENT_H
#define FEC_COMPONENT_H

#include "network-controller-components.h"
#include "network-status.h"
#include "ns3/nstime.h"
#include "ns3/packet.h"
#include "lora-frame-header.h"
#include "lorawan-mac-header.h"

#include <map>
#include <vector>
#include <queue>
#include <set>

namespace ns3
{
namespace lorawan
{

class FecComponent : public NetworkControllerComponent
{
  public:
    // Add to fec-component.h public section:
    double GetApplicationDER(uint32_t deviceAddr) const;

    static TypeId GetTypeId();
    FecComponent();
    virtual ~FecComponent();

    // --- Public API for configuration from simulation script ---
    void SetFecEnabled(bool enabled);
    void SetGenerationSize(uint32_t size);

    // --- Overridden methods from parent class ---
    void OnReceivedPacket(Ptr<const Packet> packet,
                          Ptr<EndDeviceStatus> status,
                          Ptr<NetworkStatus> networkStatus) override;

    void BeforeSendingReply(Ptr<EndDeviceStatus> status, Ptr<NetworkStatus> networkStatus) override;

    void OnFailedReply(Ptr<EndDeviceStatus> status, Ptr<NetworkStatus> networkStatus) override;

  private:
    // FEC Configuration
    struct FecConfig {
        bool enabled = true;
        uint32_t generationSize = 128;
        Time generationTimeout = Seconds(600); // 10 minutes
    } m_fecConfig;

    // ... (rest of the private members remain the same)
    // FEC Generation State
    struct FecGeneration {
        std::map<uint8_t, Ptr<Packet>> systematicPackets;
        std::vector<std::pair<std::vector<uint8_t>, Ptr<Packet>>> redundantPackets;
        std::set<uint8_t> recoveredIndices;
        Time lastActivity;
        bool isComplete = false;
        
        FecGeneration() : lastActivity(Simulator::Now()) {}
    };
    
    // Per-device FEC state
    std::map<uint32_t, std::map<uint16_t, FecGeneration>> m_deviceFecGenerations;
    
    // FEC Statistics
    std::map<uint32_t, uint32_t> m_deviceOriginalPackets;
    std::map<uint32_t, uint32_t> m_deviceRecoveredPackets;
    std::map<uint32_t, uint32_t> m_deviceLostPackets;

    // FEC Methods
    bool AttemptFecRecovery(uint32_t deviceAddr, uint16_t generationId);
    std::vector<Ptr<Packet>> SolveFecSystem(FecGeneration& generation);
    void DeliverApplicationPackets(uint32_t deviceAddr, const std::vector<Ptr<Packet>>& packets);
    void CleanupOldGenerations();

    // Galois Field operations
    uint8_t GfMultiply(uint8_t a, uint8_t b);
    uint8_t GfDivide(uint8_t a, uint8_t b);
    void InitializeGfTables();
    std::vector<uint8_t> m_gfExp;
    std::vector<uint8_t> m_gfLog;
};

} // namespace lorawan
} // namespace ns3

#endif // FEC_COMPONENT_H