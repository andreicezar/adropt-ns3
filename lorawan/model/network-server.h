/*
 * Copyright (c) 2018 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Authors: Davide Magrin <magrinda@dei.unipd.it>
 *          Martina Capuzzo <capuzzom@dei.unipd.it>
 */

#ifndef NETWORK_SERVER_H
#define NETWORK_SERVER_H

#include "class-a-end-device-lorawan-mac.h"
#include "gateway-status.h"
#include "lora-device-address.h"
#include "network-controller.h"
#include "network-scheduler.h"
#include "network-status.h"

#include "ns3/application.h"
#include "ns3/log.h"
#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/point-to-point-net-device.h"

// *** ADD INCLUDES FOR FEC ***
#include <map>
#include <vector>
#include <queue>
#include <set>

namespace ns3
{
namespace lorawan
{

/**
 * @ingroup lorawan
 *
 * The NetworkServer is an application standing on top of a node equipped with
 * links that connect it with the gateways.
 *
 * This version of the NetworkServer application attempts to closely mimic an actual
 * network server, by providing as much functionality as possible.
 */
class NetworkServer : public Application
{
  public:
    /**
     *  Register this type.
     *  @return The object TypeId.
     */
    static TypeId GetTypeId();

    NetworkServer();           //!< Default constructor
    ~NetworkServer() override; //!< Destructor

    /**
     * Start the network server application.
     */
    void StartApplication() override;

    /**
     * Stop the network server application.
     */
    void StopApplication() override;

    /**
     * Inform the NetworkServer application that these nodes are connected to the network.
     *
     * This method will create a DeviceStatus object for each new node, and add
     * it to the list.
     *
     * @param nodes The end device NodeContainer.
     */
    void AddNodes(NodeContainer nodes);

    /**
     * Inform the NetworkServer application that this node is connected to the network.
     *
     * This method will create a DeviceStatus object for the new node (if it
     * doesn't already exist).
     *
     * @param node The end device Node.
     */
    void AddNode(Ptr<Node> node);

    /**
     * Add the gateway to the list of gateways connected to this network server.
     *
     * Each gateway is identified by its Address in the network connecting it to the network
     * server.
     *
     * @param gateway A pointer to the gateway Node.
     * @param netDevice A pointer to the network server's NetDevice connected to the gateway.
     */
    void AddGateway(Ptr<Node> gateway, Ptr<NetDevice> netDevice);

    /**
     * Add a NetworkControllerComponent to this NetworkServer application.
     *
     * @param component A pointer to the NetworkControllerComponent object.
     */
    void AddComponent(Ptr<NetworkControllerComponent> component);

    /**
     * Receive a packet from a gateway.
     *
     * This function is meant to be provided to NetDevice objects as a ReceiveCallback.
     *
     * @copydoc ns3::NetDevice::ReceiveCallback
     */
    bool Receive(Ptr<NetDevice> device,
                 Ptr<const Packet> packet,
                 uint16_t protocol,
                 const Address& sender);

    /**
     * Get the NetworkStatus object of this NetworkServer application.
     *
     * @return A pointer to the NetworkStatus object.
     */
    Ptr<NetworkStatus> GetNetworkStatus();

    // *** ADD FEC PUBLIC METHODS ***
    void EnableFec(bool enable) { m_fecConfig.enabled = enable; }
    void SetFecGenerationSize(uint32_t size) { m_fecConfig.generationSize = size; }
    double GetApplicationDER(uint32_t deviceAddr) const;

  protected:
    Ptr<NetworkStatus> m_status;         //!< Ptr to the NetworkStatus object.
    Ptr<NetworkController> m_controller; //!< Ptr to the NetworkController object.
    Ptr<NetworkScheduler> m_scheduler;   //!< Ptr to the NetworkScheduler object.

    TracedCallback<Ptr<const Packet>> m_receivedPacket; //!< The `ReceivedPacket` trace source.

  private:
    // *** ADD FEC DECODER FUNCTIONALITY ***
    
    // FEC Configuration
    struct FecConfig {
        bool enabled = true;
        uint32_t generationSize = 128;
        double targetPER = 0.30;
        Time generationTimeout = Seconds(600); // 10 minutes
    } m_fecConfig;

    // FEC Generation State
    struct FecGeneration {
        std::map<uint8_t, Ptr<Packet>> systematicPackets;  // index -> packet
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
    bool ProcessFecPacket(Ptr<const Packet> packet, const Address& gwAddress);
    uint32_t ExtractDeviceAddress(Ptr<const Packet> packet);
    bool AttemptFecRecovery(uint32_t deviceAddr, uint16_t generationId);
    std::vector<Ptr<Packet>> SolveFecSystem(FecGeneration& generation);
    void DeliverApplicationPackets(uint32_t deviceAddr, const std::vector<Ptr<Packet>>& packets);
    void CleanupOldGenerations();
    void ProcessFecPacketAsync(Ptr<const Packet> packet, const Address& gwAddress);
    // Galois Field operations
    uint8_t GfMultiply(uint8_t a, uint8_t b);
    uint8_t GfDivide(uint8_t a, uint8_t b);
    void InitializeGfTables();
    std::vector<uint8_t> m_gfExp;
    std::vector<uint8_t> m_gfLog;
};

} // namespace lorawan

} // namespace ns3
#endif /* NETWORK_SERVER_H */