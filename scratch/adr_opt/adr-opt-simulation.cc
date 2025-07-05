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
    // --- Parameters for 3 devices, 8 gateways in 3x3km scenario ---
    bool verbose = false;
    bool adrEnabled = true;
    bool initializeSF = false;
    int nDevices = 3; // 3 end devices
    int nPeriodsOf20Minutes = 100;
    double mobileNodeProbability = 0.0;
    double sideLengthMeters = 1500; // 3x3km total area (1.5km radius)
    int gatewayDistanceMeters = 1000; // Closer gateways for better coverage
    double maxRandomLossDb = 10; // More realistic channel variation
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

    // Calculate number of gateways - fixed to 8 for this scenario
    int nGateways = 8;

    std::cout << "3 Devices + 8 Gateways in 3x3km Scenario:" << std::endl;
    std::cout << "  Devices: " << nDevices << std::endl;
    std::cout << "  Gateways: " << nGateways << std::endl;
    std::cout << "  Area: " << (sideLengthMeters*2/1000.0) << "x" << (sideLengthMeters*2/1000.0) << " km" << std::endl;
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

    // --- Mobility: Spread devices across 3x3km area ---
    MobilityHelper mobilityEd, mobilityGw;
    
    // End devices: randomly distributed in 3x3km area
    mobilityEd.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                    "X", PointerValue(CreateObjectWithAttributes<UniformRandomVariable>(
                                        "Min", DoubleValue(-sideLengthMeters),
                                        "Max", DoubleValue(sideLengthMeters))),
                                    "Y", PointerValue(CreateObjectWithAttributes<UniformRandomVariable>(
                                        "Min", DoubleValue(-sideLengthMeters),
                                        "Max", DoubleValue(sideLengthMeters))));
    
    // Gateways: manually positioned for good coverage in 3x3km
    Ptr<ListPositionAllocator> gwPositionAlloc = CreateObject<ListPositionAllocator>();
    
    // 8 gateways in strategic positions for 3x3km coverage
    gwPositionAlloc->Add(Vector(-1000, -1000, 15)); // Southwest
    gwPositionAlloc->Add(Vector(    0, -1000, 15)); // South
    gwPositionAlloc->Add(Vector( 1000, -1000, 15)); // Southeast
    gwPositionAlloc->Add(Vector(-1000,     0, 15)); // West
    gwPositionAlloc->Add(Vector( 1000,     0, 15)); // East
    gwPositionAlloc->Add(Vector(-1000,  1000, 15)); // Northwest
    gwPositionAlloc->Add(Vector(    0,  1000, 15)); // North
    gwPositionAlloc->Add(Vector( 1000,  1000, 15)); // Northeast
    
    mobilityGw.SetPositionAllocator(gwPositionAlloc);
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

    // --- Application: Different packet intervals for each device ---
    PeriodicSenderHelper appHelper;
    
    // Device 0: Every 2 minutes (fast)
    appHelper.SetPeriod(Seconds(120));
    appHelper.SetPacketSize(23);
    ApplicationContainer appContainer1 = appHelper.Install(endDevices.Get(0));
    
    // Device 1: Every 5 minutes (medium)
    appHelper.SetPeriod(Seconds(300));
    ApplicationContainer appContainer2 = appHelper.Install(endDevices.Get(1));
    
    // Device 2: Every 10 minutes (slow)
    appHelper.SetPeriod(Seconds(600));
    ApplicationContainer appContainer3 = appHelper.Install(endDevices.Get(2));
    
    std::cout << "Application intervals:" << std::endl;
    std::cout << "  Device 0: 2 minutes" << std::endl;
    std::cout << "  Device 1: 5 minutes" << std::endl;
    std::cout << "  Device 2: 10 minutes" << std::endl;

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
    Time stateSamplePeriod = Seconds(600); // Sample every 10 minutes
    helper.EnablePeriodicDeviceStatusPrinting(endDevices, gateways, "nodeData.txt", stateSamplePeriod);
    helper.EnablePeriodicPhyPerformancePrinting(gateways, "phyPerformance.txt", stateSamplePeriod);
    helper.EnablePeriodicGlobalPerformancePrinting("globalPerformance.txt", stateSamplePeriod);

    // --- Run the simulation ---
    Time simulationTime = Seconds(7200); // 2 hours for multiple device interactions
    std::cout << "Running simulation for " << simulationTime.GetSeconds() << " seconds (2 hours)..." << std::endl;
    
    Simulator::Stop(simulationTime);
    Simulator::Run();
    Simulator::Destroy();

    // --- Print a summary ---
    LoraPacketTracker& tracker = helper.GetPacketTracker();
    std::cout << "Simulation completed!" << std::endl;
    std::cout << "Final period packets: " 
              << tracker.CountMacPacketsGlobally(Seconds(simulationTime.GetSeconds() - 1200),
                                                 simulationTime)
              << std::endl;

    return 0;
}