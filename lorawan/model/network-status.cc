/*
 * Copyright (c) 2018 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Authors: Davide Magrin <magrinda@dei.unipd.it>
 *          Martina Capuzzo <capuzzom@dei.unipd.it>
 */

#include "network-status.h"

#include "end-device-status.h"
#include "gateway-status.h"
#include "lora-device-address.h"

#include "ns3/log.h"
#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/packet.h"
#include "ns3/pointer.h"

namespace ns3
{
namespace lorawan
{

NS_LOG_COMPONENT_DEFINE("NetworkStatus");

NS_OBJECT_ENSURE_REGISTERED(NetworkStatus);

TypeId
NetworkStatus::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NetworkStatus")
                            .SetParent<Object>()
                            .AddConstructor<NetworkStatus>()
                            .SetGroupName("lorawan");
    return tid;
}

NetworkStatus::NetworkStatus()
{
    NS_LOG_FUNCTION_NOARGS();
}

NetworkStatus::~NetworkStatus()
{
    NS_LOG_FUNCTION_NOARGS();
}

void
NetworkStatus::AddNode(Ptr<ClassAEndDeviceLorawanMac> edMac)
{
    NS_LOG_FUNCTION(this << edMac);

    // Check whether this device already exists in our list
    LoraDeviceAddress edAddress = edMac->GetDeviceAddress();
    if (m_endDeviceStatuses.find(edAddress) == m_endDeviceStatuses.end())
    {
        // The device doesn't exist. Create new EndDeviceStatus
        Ptr<EndDeviceStatus> edStatus =
            CreateObject<EndDeviceStatus>(edAddress, DynamicCast<ClassAEndDeviceLorawanMac>(edMac));

        // Add it to the map
        m_endDeviceStatuses.insert(
            std::pair<LoraDeviceAddress, Ptr<EndDeviceStatus>>(edAddress, edStatus));
        NS_LOG_DEBUG("Added to the list a device with address " << edAddress.Print());
    }
}

void
NetworkStatus::AddGateway(Address& address, Ptr<GatewayStatus> gwStatus)
{
    NS_LOG_FUNCTION(this);

    // Check whether this device already exists in the list
    if (m_gatewayStatuses.find(address) == m_gatewayStatuses.end())
    {
        // The device doesn't exist.

        // Add it to the map
        m_gatewayStatuses.insert(std::pair<Address, Ptr<GatewayStatus>>(address, gwStatus));
        NS_LOG_DEBUG("Added to the list a gateway with address " << address);
    }
}

void
NetworkStatus::OnReceivedPacket(Ptr<const Packet> packet, const Address& gwAddress)
{
    NS_LOG_FUNCTION(this << packet << gwAddress);

    // Create a copy of the packet
    Ptr<Packet> myPacket = packet->Copy();

    // Extract the headers
    LorawanMacHeader macHdr;
    myPacket->RemoveHeader(macHdr);
    LoraFrameHeader frameHdr;
    frameHdr.SetAsUplink();
    myPacket->RemoveHeader(frameHdr);

    // Update the correct EndDeviceStatus object
    LoraDeviceAddress edAddr = frameHdr.GetAddress();
    NS_LOG_DEBUG("Node address: " << edAddr);
    m_endDeviceStatuses.at(edAddr)->InsertReceivedPacket(packet, gwAddress);
}

bool
NetworkStatus::NeedsReply(LoraDeviceAddress deviceAddress)
{
    // Throws out of range if no device is found
    return m_endDeviceStatuses.at(deviceAddress)->NeedsReply();
}

Address
NetworkStatus::GetBestGatewayForDevice(LoraDeviceAddress deviceAddress, int window)
{
    // Get the endDeviceStatus we are interested in
    Ptr<EndDeviceStatus> edStatus = m_endDeviceStatuses.at(deviceAddress);
    uint32_t replyFrequency;
    if (window == 1)
    {
        replyFrequency = edStatus->GetFirstReceiveWindowFrequency();
    }
    else if (window == 2)
    {
        replyFrequency = edStatus->GetSecondReceiveWindowFrequency();
    }
    else
    {
        NS_ABORT_MSG("Invalid window value");
    }

    // Get the list of gateways that this device can reach
    // NOTE: At this point, we could also take into account the whole network to
    // identify the best gateway according to various metrics. For now, we just
    // ask the EndDeviceStatus to pick the best gateway for us via its method.
    std::map<double, Address> gwAddresses = edStatus->GetPowerGatewayMap();

    // By iterating on the map in reverse, we go from the 'best'
    // gateway, i.e. the one with the highest received power, to the
    // worst.
    Address bestGwAddress;
    for (auto it = gwAddresses.rbegin(); it != gwAddresses.rend(); it++)
    {
        bool isAvailable =
            m_gatewayStatuses.find(it->second)->second->IsAvailableForTransmission(replyFrequency);
        if (isAvailable)
        {
            bestGwAddress = it->second;
            break;
        }
    }

    return bestGwAddress;
}

void
NetworkStatus::SendThroughGateway(Ptr<Packet> packet, Address gwAddress)
{
    NS_LOG_FUNCTION(packet << gwAddress);

    m_gatewayStatuses.find(gwAddress)->second->GetNetDevice()->Send(packet, gwAddress, 0x0800);
}

Ptr<Packet>
NetworkStatus::GetReplyForDevice(LoraDeviceAddress edAddress, int windowNumber)
{
    // Get the reply packet
    Ptr<EndDeviceStatus> edStatus = m_endDeviceStatuses.find(edAddress)->second;
    Ptr<Packet> packet = edStatus->GetCompleteReplyPacket();

    // Apply the appropriate tag
    LoraTag tag;
    switch (windowNumber)
    {
    case 1:
        tag.SetDataRate(edStatus->GetMac()->GetFirstReceiveWindowDataRate());
        tag.SetFrequency(edStatus->GetFirstReceiveWindowFrequency());
        break;
    case 2:
        tag.SetDataRate(edStatus->GetMac()->GetSecondReceiveWindowDataRate());
        tag.SetFrequency(edStatus->GetSecondReceiveWindowFrequency());
        break;
    }

    packet->AddPacketTag(tag);
    return packet;
}

Ptr<EndDeviceStatus>
NetworkStatus::GetEndDeviceStatus(Ptr<const Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);

    // Get the address
    LorawanMacHeader mHdr;
    LoraFrameHeader fHdr;
    Ptr<Packet> myPacket = packet->Copy();
    myPacket->RemoveHeader(mHdr);
    myPacket->RemoveHeader(fHdr);
    auto it = m_endDeviceStatuses.find(fHdr.GetAddress());
    if (it != m_endDeviceStatuses.end())
    {
        return (*it).second;
    }
    else
    {
        NS_LOG_ERROR("EndDeviceStatus not found");
        return nullptr;
    }
}

Ptr<EndDeviceStatus>
NetworkStatus::GetEndDeviceStatus(LoraDeviceAddress address)
{
    NS_LOG_FUNCTION(this << address);

    auto it = m_endDeviceStatuses.find(address);
    if (it != m_endDeviceStatuses.end())
    {
        return (*it).second;
    }
    else
    {
        NS_LOG_ERROR("EndDeviceStatus not found");
        return nullptr;
    }
}

int
NetworkStatus::CountEndDevices()
{
    NS_LOG_FUNCTION(this);

    return m_endDeviceStatuses.size();
}
} // namespace lorawan
} // namespace ns3
