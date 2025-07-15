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
#include "ns3/network-server.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/lora-net-device.h"
#include "ns3/rssi-snir-tracker.h"
#include <iomanip>
#include <numeric>
#include <fstream>  // FIXED: Added missing include

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("AdrOptSimulation");

// Global variables for tracking
Ptr<ADRoptComponent> g_adrOptComponent;
std::vector<uint32_t> g_deviceAddresses;
std::string g_outputFile = "adr_transmission_stats.txt";
uint32_t g_totalPacketsSent = 0;
std::map<uint32_t, uint32_t> g_gatewayReceptions;
std::map<uint32_t, uint32_t> g_nodeIdToDeviceAddr;

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

// Callback for NbTrans changes
void
OnNbTransChanged(uint32_t deviceAddr, uint8_t oldNbTrans, uint8_t newNbTrans)
{
    std::cout << "Time " << Simulator::Now().GetSeconds() << "s: "
              << "Device " << deviceAddr 
              << " NbTrans changed: " << static_cast<uint32_t>(oldNbTrans) 
              << " -> " << static_cast<uint32_t>(newNbTrans) << std::endl;
}

// Callback for transmission efficiency updates
void
OnTransmissionEfficiencyChanged(uint32_t deviceAddr, double efficiency)
{
    std::cout << "Time " << Simulator::Now().GetSeconds() << "s: "
              << "Device " << deviceAddr 
              << " transmission efficiency: " << efficiency << std::endl;
}

// Callback for ADR adjustments
void
OnAdrAdjustment(uint32_t deviceAddr, uint8_t dataRate, double txPower, uint8_t nbTrans)
{
    std::cout << "Time " << Simulator::Now().GetSeconds() << "s: "
              << "Device " << deviceAddr << " ADR adjustment - "
              << "DR: " << static_cast<uint32_t>(dataRate)
              << ", TxPower: " << txPower << " dBm"
              << ", NbTrans: " << static_cast<uint32_t>(nbTrans) << std::endl;
}

// Add callback for error rate monitoring
void
OnErrorRateUpdate(uint32_t deviceAddr, uint32_t totalSent, uint32_t totalReceived, double errorRate)
{
    std::cout << "📊 Device " << deviceAddr << " Error Rate Update:" << std::endl;
    std::cout << "   Sent: " << totalSent << ", Received: " << totalReceived << std::endl;
    std::cout << "   Error Rate: " << (errorRate * 100) << "%" << std::endl;
    std::cout << "   PDR: " << ((1.0 - errorRate) * 100) << "%" << std::endl;
}

// Periodic statistics printing
void
PrintPeriodicStats()
{
    if (!g_adrOptComponent)
    {
        return;
    }
    
    std::cout << "\n=== Periodic ADR Statistics (Time: " 
              << Simulator::Now().GetSeconds() << "s) ===" << std::endl;
    
    for (uint32_t deviceAddr : g_deviceAddresses)
    {
        uint8_t currentNbTrans = g_adrOptComponent->GetCurrentNbTrans(deviceAddr);
        double efficiency = g_adrOptComponent->GetTransmissionEfficiency(deviceAddr);
        uint32_t totalAttempts = g_adrOptComponent->GetTotalTransmissionAttempts(deviceAddr);
        uint32_t adjustments = g_adrOptComponent->GetAdrAdjustmentCount(deviceAddr);
        
        std::cout << "Device " << deviceAddr << ":" << std::endl;
        std::cout << "  Current NbTrans: " << static_cast<uint32_t>(currentNbTrans) << std::endl;
        std::cout << "  Transmission Efficiency: " << efficiency << std::endl;
        std::cout << "  Total Transmission Attempts: " << totalAttempts << std::endl;
        std::cout << "  ADR Adjustments: " << adjustments << std::endl;
    }
    std::cout << "======================================\n" << std::endl;
    
    // Schedule next periodic print
    Simulator::Schedule(Seconds(600), &PrintPeriodicStats); // Every 10 minutes
}

// Write detailed statistics to file
void
WriteDetailedStatsToFile()
{
    if (!g_adrOptComponent)
    {
        return;
    }
    
    std::ofstream outFile(g_outputFile, std::ios::app);
    if (!outFile.is_open())
    {
        NS_LOG_ERROR("Could not open output file: " << g_outputFile);
        return;
    }
    
    outFile << "Time: " << Simulator::Now().GetSeconds() << "s" << std::endl;
    
    for (uint32_t deviceAddr : g_deviceAddresses)
    {
        uint8_t currentNbTrans = g_adrOptComponent->GetCurrentNbTrans(deviceAddr);
        double efficiency = g_adrOptComponent->GetTransmissionEfficiency(deviceAddr);
        uint32_t totalAttempts = g_adrOptComponent->GetTotalTransmissionAttempts(deviceAddr);
        uint32_t adjustments = g_adrOptComponent->GetAdrAdjustmentCount(deviceAddr);
        
        outFile << "Device," << deviceAddr 
                << ",NbTrans," << static_cast<uint32_t>(currentNbTrans)
                << ",Efficiency," << efficiency
                << ",TotalAttempts," << totalAttempts
                << ",Adjustments," << adjustments << std::endl;
    }
    outFile << "---" << std::endl;
    outFile.close();
    
    // Schedule next write
    Simulator::Schedule(Seconds(300), &WriteDetailedStatsToFile); // Every 5 minutes
}

// FIXED: Combined ExtractDeviceAddresses function
void
ExtractDeviceAddresses(NodeContainer endDevices)
{
    for (auto it = endDevices.Begin(); it != endDevices.End(); ++it)
    {
        uint32_t nodeId = (*it)->GetId();
        Ptr<LoraNetDevice> loraNetDevice = (*it)->GetDevice(0)->GetObject<LoraNetDevice>();
        if (loraNetDevice)
        {
            Ptr<LorawanMac> mac = loraNetDevice->GetMac();
            if (mac)
            {
                Ptr<EndDeviceLorawanMac> edMac = DynamicCast<EndDeviceLorawanMac>(mac);
                if (edMac)
                {
                    LoraDeviceAddress addr = edMac->GetDeviceAddress();
                    uint32_t deviceAddr = addr.Get();
                    
                    g_deviceAddresses.push_back(deviceAddr);
                    g_nodeIdToDeviceAddr[nodeId] = deviceAddr; // Build mapping
                    
                    std::cout << "Extracted device - NodeID: " << nodeId 
                              << ", DeviceAddr: " << deviceAddr << std::endl;
                }
            }
        }
    }
}

// FIXED: Unified PrintFinalStatistics function
void
PrintFinalStatistics()
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "         FINAL ADR STATISTICS" << std::endl;
    std::cout << "========================================" << std::endl;
    
    if (!g_adrOptComponent)
    {
        std::cout << "No ADRopt component available for final statistics" << std::endl;
        return;
    }
    
    // Print comprehensive statistics including packet tracking
    g_adrOptComponent->PrintTransmissionStatistics();
    g_adrOptComponent->PrintPacketTrackingStatistics();
    
    // Summary table
    std::cout << "\n--- SUMMARY TABLE ---" << std::endl;
    std::cout << std::setw(10) << "Device" 
              << std::setw(10) << "NbTrans" 
              << std::setw(12) << "Efficiency" 
              << std::setw(12) << "Attempts" 
              << std::setw(12) << "ADR_Count" << std::endl;
    std::cout << std::string(56, '-') << std::endl;
    
    double totalEfficiency = 0.0;
    uint32_t totalAttempts = 0;
    uint32_t totalAdjustments = 0;
    
    for (uint32_t deviceAddr : g_deviceAddresses)
    {
        uint8_t currentNbTrans = g_adrOptComponent->GetCurrentNbTrans(deviceAddr);
        double efficiency = g_adrOptComponent->GetTransmissionEfficiency(deviceAddr);
        uint32_t attempts = g_adrOptComponent->GetTotalTransmissionAttempts(deviceAddr);
        uint32_t adjustments = g_adrOptComponent->GetAdrAdjustmentCount(deviceAddr);
        
        std::cout << std::setw(10) << deviceAddr
                  << std::setw(10) << static_cast<uint32_t>(currentNbTrans)
                  << std::setw(12) << std::fixed << std::setprecision(2) << efficiency
                  << std::setw(12) << attempts
                  << std::setw(12) << adjustments << std::endl;
        
        totalEfficiency += efficiency;
        totalAttempts += attempts;
        totalAdjustments += adjustments;
    }
    
    std::cout << std::string(56, '-') << std::endl;
    std::cout << std::setw(10) << "AVERAGE"
              << std::setw(10) << "-"
              << std::setw(12) << std::fixed << std::setprecision(2) 
              << (g_deviceAddresses.empty() ? 0.0 : totalEfficiency / g_deviceAddresses.size())
              << std::setw(12) << totalAttempts
              << std::setw(12) << totalAdjustments << std::endl;
    
    // Write final summary to file
    std::ofstream finalFile("final_adr_summary.txt");
    if (finalFile.is_open())
    {
        finalFile << "Final ADR Statistics Summary" << std::endl;
        finalFile << "Simulation Duration: " << Simulator::Now().GetSeconds() << " seconds" << std::endl;
        finalFile << "Number of Devices: " << g_deviceAddresses.size() << std::endl;
        finalFile << std::endl;
        
        finalFile << "Device,NbTrans,Efficiency,TotalAttempts,ADRCount" << std::endl;
        for (uint32_t deviceAddr : g_deviceAddresses)
        {
            finalFile << deviceAddr << ","
                      << static_cast<uint32_t>(g_adrOptComponent->GetCurrentNbTrans(deviceAddr)) << ","
                      << g_adrOptComponent->GetTransmissionEfficiency(deviceAddr) << ","
                      << g_adrOptComponent->GetTotalTransmissionAttempts(deviceAddr) << ","
                      << g_adrOptComponent->GetAdrAdjustmentCount(deviceAddr) << std::endl;
        }
        
        finalFile << std::endl;
        finalFile << "Total Transmission Attempts: " << totalAttempts << std::endl;
        finalFile << "Total ADR Adjustments: " << totalAdjustments << std::endl;
        finalFile << "Average Transmission Efficiency: " 
                  << (g_deviceAddresses.empty() ? 0.0 : totalEfficiency / g_deviceAddresses.size()) << std::endl;
        
        finalFile.close();
        std::cout << "\nDetailed statistics written to: final_adr_summary.txt" << std::endl;
    }
    
    std::cout << "========================================\n" << std::endl;
}

// Add debug function to test packet transmission
void OnPacketTransmitted(Ptr<const Packet> packet, uint32_t nodeId)
{
    std::cout << "📤 Packet transmitted by device " << nodeId 
              << " at time " << Simulator::Now().GetSeconds() << "s" 
              << " (size: " << packet->GetSize() << " bytes)" << std::endl;
}

// Add debug function to test packet reception at gateways
void OnGatewayPacketReceived(Ptr<const Packet> packet, uint32_t nodeId)
{
    std::cout << "📨 Packet received at gateway " << nodeId 
              << " at time " << Simulator::Now().GetSeconds() << "s"
              << " (size: " << packet->GetSize() << " bytes)" << std::endl;
}

// Test trace connections
void TestTraceConnections(NodeContainer gateways, NodeContainer endDevices)
{
    std::cout << "\n=== TESTING TRACE CONNECTIONS ===" << std::endl;
    
    // Test end device transmissions
    for (uint32_t i = 0; i < endDevices.GetN(); ++i) {
        uint32_t nodeId = endDevices.Get(i)->GetId();
        try {
            std::string tracePath = "/NodeList/" + std::to_string(nodeId) + 
                                   "/DeviceList/0/$ns3::LoraNetDevice/Phy/StartSending";
            Config::ConnectWithoutContext(tracePath, MakeCallback(&OnPacketTransmitted));
            std::cout << "✓ Connected to end device " << nodeId << " transmission trace" << std::endl;
        } catch (...) {
            std::cout << "❌ Failed to connect to end device " << nodeId << std::endl;
        }
    }
    
    // Test gateway receptions
    for (uint32_t i = 0; i < gateways.GetN(); ++i) {
        uint32_t nodeId = gateways.Get(i)->GetId();
        try {
            std::string tracePath = "/NodeList/" + std::to_string(nodeId) + 
                                   "/DeviceList/0/$ns3::LoraNetDevice/Phy/ReceivedPacket";
            Config::ConnectWithoutContext(tracePath, MakeCallback(&OnGatewayPacketReceived));
            std::cout << "✓ Connected to gateway " << nodeId << " reception trace" << std::endl;
        } catch (...) {
            std::cout << "❌ Failed to connect to gateway " << nodeId << std::endl;
        }
    }
}

// Updated OnPacketSent callback
void OnPacketSent(Ptr<const Packet> packet, uint32_t nodeId)
{
    g_totalPacketsSent++;
    std::cout << "📤 Packet #" << g_totalPacketsSent 
              << " sent by device " << nodeId 
              << " at time " << Simulator::Now().GetSeconds() << "s" << std::endl;
    
    // Record transmission in ADRopt component
    if (g_adrOptComponent)
    {
        auto it = g_nodeIdToDeviceAddr.find(nodeId);
        if (it != g_nodeIdToDeviceAddr.end())
        {
            uint32_t deviceAddr = it->second;
            g_adrOptComponent->RecordPacketTransmission(deviceAddr);
            std::cout << "   📊 Recorded transmission for device " << deviceAddr << std::endl;
        }
        else
        {
            std::cout << "   ⚠️ No device address mapping for node " << nodeId << std::endl;
        }
    }
    
    // Milestone notifications
    if (g_totalPacketsSent % 100 == 0) {
        std::cout << "🎯 Milestone: " << g_totalPacketsSent << " packets sent" << std::endl;
    }
}

void PrintFinalPacketCount()
{
    std::cout << "\n🎯 FINAL PACKET COUNT VERIFICATION" << std::endl;
    std::cout << "=================================" << std::endl;
    std::cout << "Total packets sent: " << g_totalPacketsSent << std::endl;
    std::cout << "Expected packets: 1440" << std::endl;
    
    if (g_totalPacketsSent == 1440) {
        std::cout << "✅ SUCCESS: Exactly 1440 packets transmitted!" << std::endl;
    } else {
        std::cout << "⚠️  WARNING: Expected 1440, got " << g_totalPacketsSent << std::endl;
    }
    
    double efficiency = (double)g_totalPacketsSent / 1440.0 * 100.0;
    std::cout << "Transmission efficiency: " << efficiency << "%" << std::endl;
    
    // Add gateway reception summary
    std::cout << "\n📡 GATEWAY RECEPTION SUMMARY" << std::endl;
    std::cout << "===========================" << std::endl;
    uint32_t totalReceptions = 0;
    for (const auto& [gwId, count] : g_gatewayReceptions) {
        std::cout << "Gateway " << gwId << ": " << count << " receptions" << std::endl;
        totalReceptions += count;
    }
    std::cout << "Total gateway receptions: " << totalReceptions << std::endl;
    std::cout << "Active gateways: " << g_gatewayReceptions.size() << "/8" << std::endl;
    
    if (totalReceptions >= g_totalPacketsSent) {
        std::cout << "✅ Gateway diversity working (multiple gateways receiving)" << std::endl;
    } else {
        std::cout << "⚠️  Some packets may not be reaching gateways" << std::endl;
    }
}

void OnGatewayReception(Ptr<const Packet> packet, uint32_t gatewayId)
{
    g_gatewayReceptions[gatewayId]++;
    
    if (g_gatewayReceptions[gatewayId] % 50 == 1) { // First and every 50th
        std::cout << "📡 Gateway " << gatewayId 
                  << " received packet #" << g_gatewayReceptions[gatewayId] << std::endl;
    }
}

void VerifyADRoptActivity()
{
    if (!g_adrOptComponent)
    {
        std::cout << "❌ ADRopt component is NULL!" << std::endl;
        return;
    }
    
    std::cout << "\n🔍 ADRopt Activity Check (Time: " << Simulator::Now().GetSeconds() << "s)" << std::endl;
    
    for (uint32_t deviceAddr : g_deviceAddresses)
    {
        // Check if ADRopt has tracked any packets for this device
        auto stats = g_adrOptComponent->GetPacketTrackingStats(deviceAddr);
        
        std::cout << "Device " << deviceAddr << ":" << std::endl;
        std::cout << "  Sent: " << stats.totalPacketsSent << std::endl;
        std::cout << "  NS Received: " << stats.packetsReceivedByNetworkServer << std::endl;
        std::cout << "  SF Distribution size: " << stats.sfDistribution.size() << std::endl;
        std::cout << "  TxPower Distribution size: " << stats.txPowerDistribution.size() << std::endl;
        
        if (stats.packetsReceivedByNetworkServer == 0)
        {
            std::cout << "  ❌ WARNING: ADRopt not receiving packets for this device!" << std::endl;
        }
        else
        {
            std::cout << "  ✅ ADRopt is tracking packets for this device" << std::endl;
        }
    }
}

void MonitorADRActivity()
{
    if (!g_adrOptComponent) return;
    
    static uint32_t lastPacketCount = 0;
    
    for (uint32_t deviceAddr : g_deviceAddresses)
    {
        auto stats = g_adrOptComponent->GetPacketTrackingStats(deviceAddr);
        
        if (stats.packetsReceivedByNetworkServer > lastPacketCount)
        {
            std::cout << "📊 ADR Activity Update (Time: " << Simulator::Now().GetSeconds() << "s)" << std::endl;
            std::cout << "  Device " << deviceAddr << " packets: " << stats.packetsReceivedByNetworkServer << std::endl;
            
            // Show SF and TxPower distributions in real-time
            if (!stats.sfDistribution.empty())
            {
                std::cout << "  📡 SF Distribution:" << std::endl;
                for (const auto& sfPair : stats.sfDistribution)
                {
                    std::cout << "    SF" << static_cast<uint32_t>(sfPair.first) 
                              << ": " << sfPair.second << " packets" << std::endl;
                }
            }
            
            if (!stats.txPowerDistribution.empty())
            {
                std::cout << "  ⚡ TxPower Distribution:" << std::endl;
                for (const auto& powerPair : stats.txPowerDistribution)
                {
                    std::cout << "    " << powerPair.first 
                              << "dBm: " << powerPair.second << " packets" << std::endl;
                }
            }
            
            lastPacketCount = stats.packetsReceivedByNetworkServer;
        }
    }
    
    // Schedule next check
    Simulator::Schedule(Seconds(300), &MonitorADRActivity); // Every 5 minutes
}

int main(int argc, char* argv[])
{
    // --- Parameters for 1 devices, 8 gateways in 3x3km scenario ---
    bool verbose = false;
    bool adrEnabled = true;
    bool initializeSF = false;
    int nDevices = 1; // 1 end devices
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
    cmd.AddValue("outputFile", "Output file for transmission statistics", g_outputFile);
    cmd.Parse(argc, argv);

    // Calculate number of gateways - fixed to 8 for this scenario
    int nGateways = 8;

    std::cout << "1 Device + 8 Gateways in 3x3km Scenario:" << std::endl;
    std::cout << "  Devices: " << nDevices << std::endl;
    std::cout << "  Gateways: " << nGateways << std::endl;
    std::cout << "  Area: " << (sideLengthMeters*2/1000.0) << "x" << (sideLengthMeters*2/1000.0) << " km" << std::endl;
    std::cout << "  ADR: " << (adrEnabled ? "Enabled" : "Disabled") << std::endl;
    std::cout << "  ADR Type: " << adrType << std::endl;
    std::cout << "  Output File: " << g_outputFile << std::endl;

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
    loss->SetPathLossExponent(2.8);
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
    gwPositionAlloc->Add(Vector(-1500, -1500, 15)); // Southwest
    gwPositionAlloc->Add(Vector(    0, -1500, 15)); // South
    gwPositionAlloc->Add(Vector( 1500, -1500, 15)); // Southeast
    gwPositionAlloc->Add(Vector(-1500,     0, 15)); // West
    gwPositionAlloc->Add(Vector( 1500,     0, 15)); // East
    gwPositionAlloc->Add(Vector(-1500,  1500, 15)); // Northwest
    gwPositionAlloc->Add(Vector(    0,  1500, 15)); // North
    gwPositionAlloc->Add(Vector( 1500,  1500, 15)); // Northeast
    
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
    NetDeviceContainer gatewayDevices = helper.Install(phyHelper, macHelper, gateways);

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

    // --- Connect packet transmission counter ---
    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Phy/StartSending",
        MakeCallback(&OnPacketSent));
    
    std::cout << "📊 Packet transmission counter enabled" << std::endl;

    // Connect gateway reception tracking
    for (uint32_t i = 0; i < gateways.GetN(); ++i) {
        uint32_t nodeId = gateways.Get(i)->GetId();
        std::string tracePath = "/NodeList/" + std::to_string(nodeId) +
                                "/DeviceList/0/$ns3::LoraNetDevice/Phy/ReceivedPacket";

        // Create a lambda that captures the gateway's nodeId
        auto callback = [nodeId](Ptr<const Packet> packet, uint32_t traceNodeId) {
            OnGatewayReception(packet, nodeId);
        };

        // Create the Callback object directly
        Callback<void, Ptr<const Packet>, uint32_t> cb(callback);
        Config::ConnectWithoutContext(tracePath, cb);

        std::cout << "✓ Connected gateway reception tracking for gateway " << nodeId << std::endl;
    }

    // --- TEST TRACE CONNECTIONS ---
    TestTraceConnections(gateways, endDevices);

    // --- Extract device addresses for tracking ---
    ExtractDeviceAddresses(endDevices);

    // --- Application: Different packet intervals for each device ---
    PeriodicSenderHelper appHelper;
    
    // Device 0: Every 2 minutes (fast)
    appHelper.SetPeriod(Seconds(120));
    appHelper.SetPacketSize(23);
    ApplicationContainer appContainer1 = appHelper.Install(endDevices.Get(0));

    std::cout << "Application intervals:" << std::endl;
    std::cout << "  Device 0: 2 minutes" << std::endl;

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

    // --- Create ADRopt component first ---
    if (adrEnabled && adrType == "ns3::lorawan::ADRoptComponent")
    {
        g_adrOptComponent = CreateObject<ADRoptComponent>();
        std::cout << "ADRopt component created!" << std::endl;
    }

    // --- Network server app ---
    NetworkServerHelper networkServerHelper;
    networkServerHelper.EnableAdr(adrEnabled);
    networkServerHelper.SetAdr(adrType);  // Use ADRopt
    networkServerHelper.SetGatewaysP2P(gwRegistration);
    networkServerHelper.SetEndDevices(endDevices);
    networkServerHelper.Install(networkServer);

    // FIXED: Properly integrated trace connection block
    if (g_adrOptComponent)
    {
        Ptr<NetworkServer> ns = networkServer->GetApplication(0)->GetObject<NetworkServer>();
        if (ns)
        {
            // CRITICAL: Add our component as the ONLY ADR component
            ns->AddComponent(g_adrOptComponent);
            std::cout << "✅ ADRopt component added as PRIMARY ADR component!" << std::endl;
            
            // Connect trace sources for real-time monitoring
            g_adrOptComponent->TraceConnectWithoutContext("NbTransChanged", 
                MakeCallback(&OnNbTransChanged));
            g_adrOptComponent->TraceConnectWithoutContext("TransmissionEfficiency",
                MakeCallback(&OnTransmissionEfficiencyChanged));
            g_adrOptComponent->TraceConnectWithoutContext("AdrAdjustment",
                MakeCallback(&OnAdrAdjustment));
            g_adrOptComponent->TraceConnectWithoutContext("ErrorRate",
                MakeCallback(&OnErrorRateUpdate));
                
            std::cout << "✅ All ADRopt trace sources connected!" << std::endl;
            
            // ADDED: Verify component is working
            std::cout << "🔍 ADRopt component status: " << g_adrOptComponent->GetInstanceTypeId().GetName() << std::endl;
        }
        else
        {
            std::cout << "❌ CRITICAL ERROR: Could not get NetworkServer!" << std::endl;
        }
    }
    else
    {
        std::cout << "❌ CRITICAL ERROR: ADRopt component not created!" << std::endl;
    }
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

    // --- Initialize output file ---
    std::ofstream initFile(g_outputFile);
    if (initFile.is_open())
    {
        initFile << "ADR Transmission Statistics Log" << std::endl;
        initFile << "Format: Time,Device,Field,Value" << std::endl;
        initFile << "---" << std::endl;
        initFile.close();
    }

    // --- Schedule periodic statistics printing ---
    Simulator::Schedule(Seconds(600), &PrintPeriodicStats); // First print at 10 minutes
    Simulator::Schedule(Seconds(300), &WriteDetailedStatsToFile); // First write at 5 minutes
    Simulator::Schedule(Seconds(1800), &VerifyADRoptActivity);
    Simulator::Schedule(Seconds(300), &MonitorADRActivity); // Start monitoring at 5 minutes
    // --- Run the simulation ---
    Time simulationTime = Seconds(172800); // 48 hours for multiple device interactions
    std::cout << "Running simulation for " << simulationTime.GetSeconds() << " seconds (48 hours)..." << std::endl;

    Simulator::Stop(simulationTime);
    Simulator::Run();

    // --- Print final statistics before destroying simulator ---
    PrintFinalStatistics();  
    PrintFinalPacketCount(); 
    Simulator::Destroy();
    
    // --- Print a summary ---
    LoraPacketTracker& packetTracker = helper.GetPacketTracker();
    std::cout << "Simulation completed!" << std::endl;
    std::cout << "Final period packets: " 
            << packetTracker.CountMacPacketsGlobally(Seconds(simulationTime.GetSeconds() - 1200),
                                                    simulationTime)
            << std::endl;

    return 0;
}