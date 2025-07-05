#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/core-module.h"
#include "ns3/forwarder-helper.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/hex-grid-position-allocator.h"
#include "ns3/log.h"
#include "ns3/lora-channel.h"
#include "ns3/lora-device-address-generator.h"
#include "ns3/lora-helper.h"
#include "ns3/lora-phy-helper.h"
#include "ns3/lorawan-mac-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/network-module.h"
#include "ns3/network-server-helper.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/periodic-sender.h"
#include "ns3/point-to-point-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rectangle.h"
#include "ns3/string.h"

// Include the ADRopt component header
#include "ns3/adropt-component.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("AdrOptSimulation");

void
OnDataRateChange(uint8_t oldDr, uint8_t newDr)
{
    NS_LOG_INFO("DR" << unsigned(oldDr) << " -> DR" << unsigned(newDr));
}

void
OnTxPowerChange(double oldTxPower, double newTxPower)
{
    NS_LOG_INFO(oldTxPower << " dBm -> " << newTxPower << " dBm");
}

int main(int argc, char* argv[])
{
    // --- Parameters, default values matching example ---
    bool verbose = false;
    bool adrEnabled = true;
    bool initializeSF = false;
    int nDevices = 1; // Start with 1 device for testing
    int nPeriodsOf20Minutes = 100;
    double mobileNodeProbability = 0.0;
    double sideLengthMeters = 3000;
    int gatewayDistanceMeters = 1500;
    double maxRandomLossDb = 5;
    double minSpeedMetersPerSecond = 2;
    double maxSpeedMetersPerSecond = 16;
    std::string adrType = "ns3::lorawan::ADRoptComponent"; // Use ADRopt

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Whether to print output or not", verbose);
    cmd.AddValue("AdrEnabled", "Whether to enable ADR", adrEnabled);
    cmd.AddValue("nDevices", "Number of devices to simulate", nDevices);
    cmd.AddValue("PeriodsToSimulate", "Number of periods (20m) to simulate", nPeriodsOf20Minutes);
    cmd.AddValue("MobileNodeProbability", "Probability of a node being mobile", mobileNodeProbability);
    cmd.AddValue("sideLength", "Side length of placement area (meters)", sideLengthMeters);
    cmd.AddValue("maxRandomLoss", "Max random loss (dB)", maxRandomLossDb);
    cmd.AddValue("gatewayDistance", "Distance (m) between gateways", gatewayDistanceMeters);
    cmd.AddValue("initializeSF", "Whether to initialize the SFs", initializeSF);
    cmd.AddValue("MinSpeed", "Min speed (m/s) for mobile devices", minSpeedMetersPerSecond);
    cmd.AddValue("MaxSpeed", "Max speed (m/s) for mobile devices", maxSpeedMetersPerSecond);
    cmd.Parse(argc, argv);

    // Calculate number of gateways - ensure at least 1
    int gatewayRings = std::max(1, 2 + (int)((std::sqrt(2) * sideLengthMeters) / gatewayDistanceMeters));
    int nGateways = std::max(1, 3 * gatewayRings * gatewayRings - 3 * gatewayRings + 1);

    std::cout << "Simulation parameters:" << std::endl;
    std::cout << "  Devices: " << nDevices << std::endl;
    std::cout << "  Gateways: " << nGateways << std::endl;
    std::cout << "  ADR: " << (adrEnabled ? "Enabled" : "Disabled") << std::endl;
    std::cout << "  ADR Type: " << adrType << std::endl;

    // --- Logging setup - be more selective to avoid excessive output ---
    if (verbose)
    {
        LogComponentEnable("AdrOptSimulation", LOG_LEVEL_ALL);
        LogComponentEnable("ADRoptComponent", LOG_LEVEL_ALL);
        LogComponentEnable("NetworkServer", LOG_LEVEL_INFO);
        LogComponentEnable("NetworkStatus", LOG_LEVEL_INFO);
    }
    else
    {
        LogComponentEnable("AdrOptSimulation", LOG_LEVEL_INFO);
        LogComponentEnable("ADRoptComponent", LOG_LEVEL_INFO);
    }
    
    LogComponentEnableAll(LOG_PREFIX_FUNC);
    LogComponentEnableAll(LOG_PREFIX_NODE);
    LogComponentEnableAll(LOG_PREFIX_TIME);

    // --- Always enable ADR bit in MAC ---
    Config::SetDefault("ns3::EndDeviceLorawanMac::ADR", BooleanValue(true));

    // --- Channel setup (loss, delay, random fading) ---
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.76);
    loss->SetReference(1, 7.7);
    
    if (maxRandomLossDb > 0)
    {
        Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
        x->SetAttribute("Min", DoubleValue(0.0));
        x->SetAttribute("Max", DoubleValue(maxRandomLossDb));
        Ptr<RandomPropagationLossModel> randomLoss = CreateObject<RandomPropagationLossModel>();
        randomLoss->SetAttribute("Variable", PointerValue(x));
        loss->SetNext(randomLoss);
    }
    
    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);

    // --- Mobility ---
    MobilityHelper mobilityEd, mobilityGw;
    mobilityEd.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                    "X", PointerValue(CreateObjectWithAttributes<UniformRandomVariable>(
                                        "Min", DoubleValue(-sideLengthMeters/2),
                                        "Max", DoubleValue(sideLengthMeters/2))),
                                    "Y", PointerValue(CreateObjectWithAttributes<UniformRandomVariable>(
                                        "Min", DoubleValue(-sideLengthMeters/2),
                                        "Max", DoubleValue(sideLengthMeters/2))));
    
    Ptr<HexGridPositionAllocator> hexAllocator =
        CreateObject<HexGridPositionAllocator>(gatewayDistanceMeters / 2);
    mobilityGw.SetPositionAllocator(hexAllocator);
    mobilityGw.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    // --- Create gateways and install mobility/devices ---
    NodeContainer gateways;
    gateways.Create(nGateways);
    mobilityGw.Install(gateways);

    LoraPhyHelper phyHelper;
    phyHelper.SetChannel(channel);
    LorawanMacHelper macHelper;
    LoraHelper helper;
    helper.EnablePacketTracking();

    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    helper.Install(phyHelper, macHelper, gateways);

    // --- Create end devices and install mobility/devices ---
    NodeContainer endDevices;
    endDevices.Create(nDevices);

    // Fixed devices
    mobilityEd.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    int fixedPositionNodes = int(double(nDevices) * (1 - mobileNodeProbability));
    
    for (int i = 0; i < fixedPositionNodes; ++i)
    {
        mobilityEd.Install(endDevices.Get(i));
    }
    
    // Mobile devices (if any)
    if (mobileNodeProbability > 0.0 && fixedPositionNodes < nDevices) 
    {
        mobilityEd.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
            "Bounds", RectangleValue(Rectangle(-sideLengthMeters/2, sideLengthMeters/2, 
                                             -sideLengthMeters/2, sideLengthMeters/2)),
            "Distance", DoubleValue(1000),
            "Speed", PointerValue(CreateObjectWithAttributes<UniformRandomVariable>(
                "Min", DoubleValue(minSpeedMetersPerSecond),
                "Max", DoubleValue(maxSpeedMetersPerSecond))));
        
        for (int i = fixedPositionNodes; i < nDevices; ++i)
        {
            mobilityEd.Install(endDevices.Get(i));
        }
    }

    // --- LoraNetDeviceAddress ---
    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr<LoraDeviceAddressGenerator> addrGen =
        CreateObject<LoraDeviceAddressGenerator>(nwkId, nwkAddr);

    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_A);
    macHelper.SetAddressGenerator(addrGen);
    macHelper.SetRegion(LorawanMacHelper::EU);
    helper.Install(phyHelper, macHelper, endDevices);

    // --- Application on end devices ---
    int appPeriodSeconds = 60; // 1 minute for faster testing
    PeriodicSenderHelper appHelper;
    appHelper.SetPeriod(Seconds(appPeriodSeconds));
    appHelper.SetPacketSize(23); // Standard LoRaWAN packet size
    ApplicationContainer appContainer = appHelper.Install(endDevices);

    // --- Optionally set spreading factors up
    if (initializeSF) {
        LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);
    }

    // --- PointToPoint links between gateways and server ---
    Ptr<Node> networkServer = CreateObject<Node>();
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));
    
    typedef std::list<std::pair<Ptr<PointToPointNetDevice>, Ptr<Node>>> P2PGwRegistration_t;
    P2PGwRegistration_t gwRegistration;
    
    for (auto gw = gateways.Begin(); gw != gateways.End(); ++gw) 
    {
        auto container = p2p.Install(networkServer, *gw);
        auto serverP2PNetDev = DynamicCast<PointToPointNetDevice>(container.Get(0));
        gwRegistration.push_back({serverP2PNetDev, *gw});
    }

    // --- Network server app ---
    NetworkServerHelper networkServerHelper;
    networkServerHelper.EnableAdr(adrEnabled);
    networkServerHelper.SetAdr(adrType);  // Use ADRopt
    networkServerHelper.SetGatewaysP2P(gwRegistration);
    networkServerHelper.SetEndDevices(endDevices);
    networkServerHelper.Install(networkServer);

    // --- Forwarder app on gateways ---
    ForwarderHelper forwarderHelper;
    forwarderHelper.Install(gateways);

    // --- Tracing DR/TP changes ---
    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/TxPower",
        MakeCallback(&OnTxPowerChange));
    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/DataRate",
        MakeCallback(&OnDataRateChange));

    // --- Periodic state/metrics output ---
    Time stateSamplePeriod = Seconds(appPeriodSeconds * 10); // Sample every 10 periods
    helper.EnablePeriodicDeviceStatusPrinting(endDevices, gateways, "nodeData.txt", stateSamplePeriod);
    helper.EnablePeriodicPhyPerformancePrinting(gateways, "phyPerformance.txt", stateSamplePeriod);
    helper.EnablePeriodicGlobalPerformancePrinting("globalPerformance.txt", stateSamplePeriod);

    // --- Run the simulation ---
    Time simulationTime = Seconds(appPeriodSeconds * nPeriodsOf20Minutes);
    std::cout << "Running simulation for " << simulationTime.GetSeconds() << " seconds..." << std::endl;
    
    Simulator::Stop(simulationTime);
    Simulator::Run();
    Simulator::Destroy();

    // --- Print a summary ---
    LoraPacketTracker& tracker = helper.GetPacketTracker();
    std::cout << "Simulation completed!" << std::endl;
    std::cout << "Packets sent in final period: " 
              << tracker.CountMacPacketsGlobally(Seconds(appPeriodSeconds * (nPeriodsOf20Minutes - 2)),
                                                 Seconds(appPeriodSeconds * (nPeriodsOf20Minutes - 1)))
              << std::endl;

    return 0;
}