/*
 * Copyright (c) 2017 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 */

#include "periodic-sender-helper.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/periodic-sender.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/trace-source-accessor.h"

namespace ns3
{
namespace lorawan
{

NS_LOG_COMPONENT_DEFINE("PeriodicSenderHelper");

PeriodicSenderHelper::PeriodicSenderHelper()
{
    m_factory.SetTypeId("ns3::PeriodicSender");

    // m_factory.Set ("PacketSizeRandomVariable", StringValue
    //                  ("ns3::ParetoRandomVariable[Bound=10|Shape=2.5]"));

    m_initialDelay = CreateObject<UniformRandomVariable>();
    m_initialDelay->SetAttribute("Min", DoubleValue(0));

    m_intervalProb = CreateObject<UniformRandomVariable>();
    m_intervalProb->SetAttribute("Min", DoubleValue(0));
    m_intervalProb->SetAttribute("Max", DoubleValue(1));

    m_pktSize = 10;
    m_pktSizeRV = nullptr;
}

PeriodicSenderHelper::~PeriodicSenderHelper()
{
}

void
PeriodicSenderHelper::SetAttribute(std::string name, const AttributeValue& value)
{
    m_factory.Set(name, value);
}

ApplicationContainer
PeriodicSenderHelper::Install(Ptr<Node> node) const
{
    return ApplicationContainer(InstallPriv(node));
}

ApplicationContainer
PeriodicSenderHelper::Install(NodeContainer c) const
{
    ApplicationContainer apps;
    for (auto i = c.Begin(); i != c.End(); ++i)
    {
        apps.Add(InstallPriv(*i));
    }

    return apps;
}

Ptr<Application>
PeriodicSenderHelper::InstallPriv(Ptr<Node> node) const
{
    NS_LOG_FUNCTION(this << node);

    Ptr<PeriodicSender> app = m_factory.Create<PeriodicSender>();

    Time interval;
    if (m_period.IsZero())
    {
        double intervalProb = m_intervalProb->GetValue();
        NS_LOG_DEBUG("IntervalProb = " << intervalProb);

        // Based on TR 45.820
        if (intervalProb < 0.4)
        {
            interval = Days(1);
        }
        else if (0.4 <= intervalProb && intervalProb < 0.8)
        {
            interval = Hours(2);
        }
        else if (0.8 <= intervalProb && intervalProb < 0.95)
        {
            interval = Hours(1);
        }
        else
        {
            interval = Minutes(30);
        }
    }
    else
    {
        interval = m_period;
    }

    app->SetInterval(interval);
    NS_LOG_DEBUG("Created an application with interval = " << interval.As(Time::H));

    app->SetInitialDelay(Seconds(m_initialDelay->GetValue(0, interval.GetSeconds())));
    app->SetPacketSize(m_pktSize);
    if (m_pktSizeRV)
    {
        app->SetPacketSizeRandomVariable(m_pktSizeRV);
    }

    app->SetNode(node);
    node->AddApplication(app);

    return app;
}

void
PeriodicSenderHelper::SetPeriod(Time period)
{
    m_period = period;
}

void
PeriodicSenderHelper::SetPacketSizeRandomVariable(Ptr<RandomVariableStream> rv)
{
    m_pktSizeRV = rv;
}

void
PeriodicSenderHelper::SetPacketSize(uint8_t size)
{
    m_pktSize = size;
}

} // namespace lorawan
} // namespace ns3
