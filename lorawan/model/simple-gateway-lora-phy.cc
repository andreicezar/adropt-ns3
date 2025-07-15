/*
 * Copyright (c) 2017 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 */

#include "simple-gateway-lora-phy.h"

#include "lora-tag.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{
namespace lorawan
{

NS_LOG_COMPONENT_DEFINE("SimpleGatewayLoraPhy");

NS_OBJECT_ENSURE_REGISTERED(SimpleGatewayLoraPhy);

/***********************************************************************
 *                 Implementation of gateway methods                   *
 ***********************************************************************/

TypeId
SimpleGatewayLoraPhy::GetTypeId()
{
    static TypeId tid = TypeId("ns3::SimpleGatewayLoraPhy")
                            .SetParent<GatewayLoraPhy>()
                            .SetGroupName("lorawan")
                            .AddConstructor<SimpleGatewayLoraPhy>();

    return tid;
}

SimpleGatewayLoraPhy::SimpleGatewayLoraPhy()
{
    NS_LOG_FUNCTION_NOARGS();
}

SimpleGatewayLoraPhy::~SimpleGatewayLoraPhy()
{
    NS_LOG_FUNCTION_NOARGS();
}

void
SimpleGatewayLoraPhy::Send(Ptr<Packet> packet,
                           LoraTxParameters txParams,
                           uint32_t frequencyHz,
                           double txPowerDbm)
{
    NS_LOG_FUNCTION(this << packet << frequencyHz << txPowerDbm);

    // Get the time a packet with these parameters will take to be transmitted
    Time duration = GetOnAirTime(packet, txParams);

    NS_LOG_DEBUG("Duration of packet: " << duration << ", SF" << unsigned(txParams.sf));

    // Interrupt all receive operations
    std::list<Ptr<SimpleGatewayLoraPhy::ReceptionPath>>::iterator it;
    for (it = m_receptionPaths.begin(); it != m_receptionPaths.end(); ++it)
    {
        Ptr<SimpleGatewayLoraPhy::ReceptionPath> currentPath = *it;

        if (!currentPath->IsAvailable()) // Reception path is occupied
        {
            // Call the callback for reception interrupted by transmission
            // Fire the trace source
            if (m_device)
            {
                m_noReceptionBecauseTransmitting(currentPath->GetEvent()->GetPacket(),
                                                 m_device->GetNode()->GetId());
            }
            else
            {
                m_noReceptionBecauseTransmitting(currentPath->GetEvent()->GetPacket(), 0);
            }

            // Cancel the scheduled EndReceive call
            Simulator::Cancel(currentPath->GetEndReceive());

            // Free it
            // This also resets all parameters like packet and endReceive call
            currentPath->Free();
        }
    }

    // Send the packet in the channel
    m_channel->Send(this, packet, txPowerDbm, txParams, duration, frequencyHz);

    Simulator::Schedule(duration, &SimpleGatewayLoraPhy::TxFinished, this, packet);

    m_isTransmitting = true;

    // Fire the trace source
    if (m_device)
    {
        m_startSending(packet, m_device->GetNode()->GetId());
    }
    else
    {
        m_startSending(packet, 0);
    }
}

void
SimpleGatewayLoraPhy::StartReceive(Ptr<Packet> packet,
                                   double rxPowerDbm,
                                   uint8_t sf,
                                   Time duration,
                                   uint32_t frequencyHz)
{
    NS_LOG_FUNCTION(this << packet << rxPowerDbm << duration << frequencyHz);

    // Fire the trace source
    m_phyRxBeginTrace(packet);

    if (m_isTransmitting)
    {
        // If we get to this point, there are no demodulators we can use
        NS_LOG_INFO("Dropping packet reception of packet with sf = "
                    << unsigned(sf) << " because we are in TX mode");

        m_phyRxEndTrace(packet);

        // Fire the trace source
        if (m_device)
        {
            m_noReceptionBecauseTransmitting(packet, m_device->GetNode()->GetId());
        }
        else
        {
            m_noReceptionBecauseTransmitting(packet, 0);
        }

        return;
    }

    // Add the event to the LoraInterferenceHelper
    Ptr<LoraInterferenceHelper::Event> event;
    event = m_interference.Add(duration, rxPowerDbm, sf, packet, frequencyHz);

    // Cycle over the receive paths to check availability to receive the packet
    std::list<Ptr<SimpleGatewayLoraPhy::ReceptionPath>>::iterator it;

    for (it = m_receptionPaths.begin(); it != m_receptionPaths.end(); ++it)
    {
        Ptr<SimpleGatewayLoraPhy::ReceptionPath> currentPath = *it;

        // If the receive path is available and listening on the channel of
        // interest, we have a candidate
        if (currentPath->IsAvailable())
        {
            // See whether the reception power is above or below the sensitivity
            // for that spreading factor
            double sensitivity = SimpleGatewayLoraPhy::sensitivity[unsigned(sf) - 7];

            if (rxPowerDbm < sensitivity) // Packet arrived below sensitivity
            {
                NS_LOG_INFO("Dropping packet reception of packet with sf = "
                            << unsigned(sf) << " because under the sensitivity of " << sensitivity
                            << " dBm");

                if (m_device)
                {
                    m_underSensitivity(packet, m_device->GetNode()->GetId());
                }
                else
                {
                    m_underSensitivity(packet, 0);
                }

                // Since the packet is below sensitivity, it makes no sense to
                // search for another ReceivePath
                return;
            }
            else // We have sufficient sensitivity to start receiving
            {
                NS_LOG_INFO("Scheduling reception of a packet, occupying one demodulator");

                // Block this resource
                currentPath->LockOnEvent(event);
                m_occupiedReceptionPaths++;

                // Schedule the end of the reception of the packet
                EventId endReceiveEventId =
                    Simulator::Schedule(duration, &LoraPhy::EndReceive, this, packet, event);

                currentPath->SetEndReceive(endReceiveEventId);

                // Make sure we don't go on searching for other ReceivePaths
                return;
            }
        }
    }
    // If we get to this point, there are no demodulators we can use
    NS_LOG_INFO("Dropping packet reception of packet with sf = "
                << unsigned(sf) << " and frequency " << frequencyHz
                << "Hz because no suitable demodulator was found");

    // Fire the trace source
    if (m_device)
    {
        m_noMoreDemodulators(packet, m_device->GetNode()->GetId());
    }
    else
    {
        m_noMoreDemodulators(packet, 0);
    }
}

void
SimpleGatewayLoraPhy::EndReceive(Ptr<Packet> packet, Ptr<LoraInterferenceHelper::Event> event)
{
    NS_LOG_FUNCTION(this << packet << *event);
    m_phyRxEndTrace(packet);

    uint8_t packetDestroyed = m_interference.IsDestroyedByInterference(event);

    if (packetDestroyed != uint8_t(0))
    {
        // Packet was destroyed
        LoraTag tag;
        packet->RemovePacketTag(tag);
        tag.SetDestroyedBy(packetDestroyed);
        packet->AddPacketTag(tag);
        if (m_device)
        {
            m_interferedPacket(packet, m_device->GetNode()->GetId());
        }
    }
    else // Reception was correct
    {
        NS_LOG_INFO("Packet with SF " << unsigned(event->GetSpreadingFactor()) << " received correctly");

        // Fire original trace
        if (m_device)
        {
            m_successfullyReceivedPacket(packet, m_device->GetNode()->GetId());
        }

        // --- CORRECTED TRACE FIRING ---
        double rssi = event->GetRxPowerdBm();
        double noiseFigureDb = 6.0;
        double bandwidthHz = 125000.0;
        double thermalNoiseDbm = -174 + 10 * log10(bandwidthHz);
        double noisePowerDbm = thermalNoiseDbm + noiseFigureDb;
        double snir = rssi - noisePowerDbm;

        // Fire the new trace with all four arguments
        if (m_device)
        {
             m_phyRxOkRssiSnir(packet, rssi, snir, m_device->GetNode()->GetId());
        }
        // --- END CORRECTION ---

        // Forward packet to MAC layer
        if (!m_rxOkCallback.IsNull())
        {
            LoraTag tag;
            packet->RemovePacketTag(tag);
            tag.SetReceivePower(event->GetRxPowerdBm());
            tag.SetFrequency(event->GetFrequency());
            packet->AddPacketTag(tag);
            m_rxOkCallback(packet);
        }
    }

    // Free the demodulator
    for (auto it = m_receptionPaths.begin(); it != m_receptionPaths.end(); ++it)
    {
        if ((*it)->GetEvent() == event)
        {
            (*it)->Free();
            m_occupiedReceptionPaths--;
            return;
        }
    }
}

} // namespace lorawan
} // namespace ns3
