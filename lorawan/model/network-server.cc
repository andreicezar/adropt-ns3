/*
 * Copyright (c) 2018 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Authors: Davide Magrin <magrinda@dei.unipd.it>
 *          Martina Capuzzo <capuzzom@dei.unipd.it>
 */

#include "network-server.h"

#include "class-a-end-device-lorawan-mac.h"
#include "lora-device-address.h"
#include "lora-frame-header.h"
#include "lorawan-mac-header.h"
#include "mac-command.h"
#include "network-status.h"

#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/packet.h"
#include "ns3/point-to-point-net-device.h"

namespace ns3
{
namespace lorawan
{

NS_LOG_COMPONENT_DEFINE("NetworkServer");

NS_OBJECT_ENSURE_REGISTERED(NetworkServer);

TypeId
NetworkServer::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NetworkServer")
            .SetParent<Application>()
            .AddConstructor<NetworkServer>()
            .AddTraceSource(
                "ReceivedPacket",
                "Trace source that is fired when a packet arrives at the network server",
                MakeTraceSourceAccessor(&NetworkServer::m_receivedPacket),
                "ns3::Packet::TracedCallback")
            .SetGroupName("lorawan");
    return tid;
}

NetworkServer::NetworkServer()
    : m_status(Create<NetworkStatus>()),
      m_controller(Create<NetworkController>(m_status)),
      m_scheduler(Create<NetworkScheduler>(m_status, m_controller))
{
    NS_LOG_FUNCTION_NOARGS();
}

NetworkServer::~NetworkServer()
{
    NS_LOG_FUNCTION_NOARGS();
}

void
NetworkServer::StartApplication()
{
    NS_LOG_FUNCTION_NOARGS();
}

void
NetworkServer::StopApplication()
{
    NS_LOG_FUNCTION_NOARGS();
}

void
NetworkServer::AddGateway(Ptr<Node> gateway, Ptr<NetDevice> netDevice)
{
    NS_LOG_FUNCTION(this << gateway);

    // Get the PointToPointNetDevice
    Ptr<PointToPointNetDevice> p2pNetDevice;
    for (uint32_t i = 0; i < gateway->GetNDevices(); i++)
    {
        p2pNetDevice = DynamicCast<PointToPointNetDevice>(gateway->GetDevice(i));
        if (p2pNetDevice)
        {
            // We found a p2pNetDevice on the gateway
            break;
        }
    }

    // Get the gateway's LoRa MAC layer (assumes gateway's MAC is configured as first device)
    Ptr<GatewayLorawanMac> gwMac =
        DynamicCast<GatewayLorawanMac>(DynamicCast<LoraNetDevice>(gateway->GetDevice(0))->GetMac());
    NS_ASSERT(gwMac);

    // Get the Address
    Address gatewayAddress = p2pNetDevice->GetAddress();

    // Create new gatewayStatus
    Ptr<GatewayStatus> gwStatus = Create<GatewayStatus>(gatewayAddress, netDevice, gwMac);

    m_status->AddGateway(gatewayAddress, gwStatus);
}

void
NetworkServer::AddNodes(NodeContainer nodes)
{
    NS_LOG_FUNCTION_NOARGS();

    // For each node in the container, call the function to add that single node
    NodeContainer::Iterator it;
    for (it = nodes.Begin(); it != nodes.End(); it++)
    {
        AddNode(*it);
    }
}

void
NetworkServer::AddNode(Ptr<Node> node)
{
    NS_LOG_FUNCTION(this << node);

    // Get the LoraNetDevice
    Ptr<LoraNetDevice> loraNetDevice;
    for (uint32_t i = 0; i < node->GetNDevices(); i++)
    {
        loraNetDevice = DynamicCast<LoraNetDevice>(node->GetDevice(i));
        if (loraNetDevice)
        {
            // We found a LoraNetDevice on the node
            break;
        }
    }

    // Get the MAC
    Ptr<ClassAEndDeviceLorawanMac> edLorawanMac =
        DynamicCast<ClassAEndDeviceLorawanMac>(loraNetDevice->GetMac());

    // Update the NetworkStatus about the existence of this node
    m_status->AddNode(edLorawanMac);
}

bool
NetworkServer::Receive(Ptr<NetDevice> device,
                       Ptr<const Packet> packet,
                       uint16_t protocol,
                       const Address& address)
{
    NS_LOG_FUNCTION(this << packet << protocol << address);

    // Create a copy of the packet
    Ptr<Packet> myPacket = packet->Copy();

    // Fire the trace source
    m_receivedPacket(packet);

    // Inform the scheduler of the newly arrived packet
    m_scheduler->OnReceivedPacket(packet);

    // Inform the status of the newly arrived packet
    m_status->OnReceivedPacket(packet, address);

    // Inform the controller of the newly arrived packet
    m_controller->OnNewPacket(packet);

    return true;
}

void
NetworkServer::AddComponent(Ptr<NetworkControllerComponent> component)
{
    NS_LOG_FUNCTION(this << component);

    m_controller->Install(component);
}

Ptr<NetworkStatus>
NetworkServer::GetNetworkStatus()
{
    return m_status;
}

Ptr<NetworkController>
NetworkServer::GetNetworkController()
{
    return m_controller;
}

} // namespace lorawan
} // namespace ns3
