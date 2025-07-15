#include "rssi-snir-tracker.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/lora-net-device.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/simple-gateway-lora-phy.h"
#include "ns3/lorawan-mac-header.h"
#include "ns3/lora-frame-header.h"
#include "ns3/lora-tag.h"
#include "ns3/config.h"
#include <iomanip>
#include <numeric>
#include <algorithm>

namespace ns3 {
namespace lorawan {

NS_LOG_COMPONENT_DEFINE("RssiSnirTracker");
NS_OBJECT_ENSURE_REGISTERED(RssiSnirTracker);

TypeId RssiSnirTracker::GetTypeId()
{
    static TypeId tid = TypeId("ns3::RssiSnirTracker")
        .SetParent<Object>()
        .AddConstructor<RssiSnirTracker>();
    return tid;
}

RssiSnirTracker::RssiSnirTracker() : m_isTracking(false) 
{
    NS_LOG_FUNCTION(this);
}

RssiSnirTracker::~RssiSnirTracker() 
{ 
    StopTracking(); 
}

void RssiSnirTracker::Initialize(NodeContainer gateways)
{
    NS_LOG_FUNCTION(this);
    std::cout << "🔧 Initializing RSSI/SNIR Tracker for " << gateways.GetN() << " gateways..." << std::endl;
    
    for (uint32_t i = 0; i < gateways.GetN(); ++i)
    {
        Ptr<Node> gwNode = gateways.Get(i);
        uint32_t nodeId = gwNode->GetId();
        
        try {
            // Method 1: Use Config path approach which is more flexible
            std::string tracePath = "/NodeList/" + std::to_string(nodeId) + 
                                   "/DeviceList/0/$ns3::LoraNetDevice/Phy/ReceivedPacket";
            
            // Use MakeCallback with bound parameters - this is the scalable NS-3 way
            Config::ConnectWithoutContext(tracePath, 
                MakeCallback(&RssiSnirTracker::OnReceivedPacketGateway, this, nodeId));
            
            std::cout << "✓ Connected to gateway " << nodeId << " via Config path" << std::endl;
            
        } catch (const std::exception& e) {
            std::cout << "⚠️  Config connection failed for gateway " << nodeId << ": " << e.what() << std::endl;
        }
        
        try {
            // Method 2: Direct PHY connection as backup
            Ptr<LoraNetDevice> loraDevice = gwNode->GetDevice(0)->GetObject<LoraNetDevice>();
            if (loraDevice) {
                Ptr<LoraPhy> phy = loraDevice->GetPhy();
                if (phy) {
                    // This uses MakeCallback with bound arguments - scalable for any number of gateways
                    phy->TraceConnectWithoutContext("ReceivedPacket", 
                        MakeCallback(&RssiSnirTracker::OnReceivedPacketGateway, this, nodeId));
                    std::cout << "✓ Also connected directly to PHY for gateway " << nodeId << std::endl;
                }
            }
        } catch (const std::exception& e) {
            std::cout << "⚠️  Direct PHY connection failed for gateway " << nodeId << ": " << e.what() << std::endl;
        }
    }
    
    std::cout << "🔧 Tracker initialization complete." << std::endl;
}

void RssiSnirTracker::OnReceivedPacketGateway(uint32_t gatewayNodeId, Ptr<const Packet> packet, uint32_t traceNodeId)
{
    NS_LOG_FUNCTION(this << gatewayNodeId << packet << traceNodeId);
    
    if (!m_isTracking) return;
    
    std::cout << "📨 Packet received at gateway " << gatewayNodeId 
              << " at time " << Simulator::Now().GetSeconds() << "s" << std::endl;
    
    try {
        // Extract device address from packet headers
        uint32_t deviceAddr = ExtractDeviceAddress(packet);
        
        // Extract RSSI from LoraTag
        LoraTag tag;
        double rssi = -999; // Default invalid value
        double snir = -999;
        
        if (packet->PeekPacketTag(tag)) {
            rssi = tag.GetReceivePower();
            
            // Calculate SNIR (simplified calculation)
            double noiseFigureDb = 6.0;
            double bandwidthHz = 125000.0;
            double thermalNoiseDbm = -174 + 10 * log10(bandwidthHz);
            double noisePowerDbm = thermalNoiseDbm + noiseFigureDb;
            snir = rssi - noisePowerDbm;
            
            std::cout << "  📊 Device " << deviceAddr << ": RSSI=" << rssi 
                      << " dBm, SNIR=" << snir << " dB" << std::endl;
        } else {
            std::cout << "  ⚠️  No LoraTag found - using default values" << std::endl;
            rssi = -100; // Default placeholder
            snir = 10;   // Default placeholder
        }
        
        // Store measurement
        RssiSnirMeasurement m;
        m.timestamp = Simulator::Now().GetSeconds();
        m.gatewayNodeId = gatewayNodeId;
        m.deviceAddr = deviceAddr;
        m.rssi = rssi;
        m.snir = snir;
        m_measurements.push_back(m);
        
        // Write to file immediately
        m_outputFile << std::fixed << std::setprecision(3)
                     << m.timestamp << ","
                     << m.gatewayNodeId << ","
                     << m.deviceAddr << ","
                     << std::setprecision(2) << m.rssi << ","
                     << m.snir << std::endl;
        m_outputFile.flush(); // Force immediate write
        
        std::cout << "✓ Measurement logged to file" << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "❌ Error processing packet: " << e.what() << std::endl;
    }
}

void RssiSnirTracker::StartTracking(const std::string& filename)
{
    NS_LOG_FUNCTION(this << filename);
    m_outputFileName = filename;
    m_outputFile.open(filename);
    
    if (!m_outputFile.is_open()) {
        std::cout << "❌ ERROR: Failed to open file: " << filename << std::endl;
        return;
    }
    
    std::cout << "✓ Successfully opened file: " << filename << std::endl;
    
    m_outputFile << "Timestamp,GatewayId,DeviceAddr,RSSI_dBm,SNIR_dB" << std::endl;
    m_outputFile.flush();
    
    m_isTracking = true;
    std::cout << "✓ RSSI/SNIR tracking started" << std::endl;
}

void RssiSnirTracker::StopTracking()
{
    NS_LOG_FUNCTION(this);
    if (m_outputFile.is_open()) {
        m_outputFile.close();
    }
    m_isTracking = false;
}

uint32_t RssiSnirTracker::ExtractDeviceAddress(Ptr<const Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    try {
        Ptr<Packet> packetCopy = packet->Copy();
        LorawanMacHeader macHdr;
        packetCopy->RemoveHeader(macHdr);
        LoraFrameHeader fHdr;
        fHdr.SetAsUplink();
        packetCopy->RemoveHeader(fHdr);
        return fHdr.GetAddress().Get();
    } catch (...) {
        std::cout << "⚠️  Could not extract device address, using default" << std::endl;
        return 999999; // Default fallback
    }
}

void RssiSnirTracker::PrintAnalysis()
{
    std::cout << "\n📊 RSSI/SNIR ANALYSIS" << std::endl;
    std::cout << "======================" << std::endl;
    std::cout << "Total Measurements Recorded: " << m_measurements.size() << std::endl;

    if (m_measurements.empty()) {
        std::cout << "❌ No measurements recorded!" << std::endl;
        std::cout << "Possible issues:" << std::endl;
        std::cout << "  - Trace connections failed" << std::endl;
        std::cout << "  - No packets were actually transmitted" << std::endl;
        std::cout << "  - Timing issue with tracking start" << std::endl;
        return;
    }

    std::map<uint32_t, std::vector<double>> rssiMap;
    std::map<uint32_t, std::vector<double>> snirMap;
    std::map<uint32_t, std::set<uint32_t>> deviceGateways;

    for (const auto& m : m_measurements) {
        rssiMap[m.deviceAddr].push_back(m.rssi);
        snirMap[m.deviceAddr].push_back(m.snir);
        deviceGateways[m.deviceAddr].insert(m.gatewayNodeId);
    }

    for (const auto& [deviceAddr, rssiValues] : rssiMap) {
        const auto& snirValues = snirMap[deviceAddr];
        const auto& gateways = deviceGateways[deviceAddr];

        double avgRssi = std::accumulate(rssiValues.begin(), rssiValues.end(), 0.0) / rssiValues.size();
        double avgSnir = std::accumulate(snirValues.begin(), snirValues.end(), 0.0) / snirValues.size();

        std::cout << "\n📱 Device " << deviceAddr << ":" << std::endl;
        std::cout << "   - Measurements: " << rssiValues.size() << std::endl;
        std::cout << "   - Gateway diversity: " << gateways.size() << " gateways" << std::endl;
        std::cout << "   - Average RSSI: " << std::fixed << std::setprecision(2) << avgRssi << " dBm" << std::endl;
        std::cout << "   - Average SNIR: " << avgSnir << " dB" << std::endl;
        
        // Show which gateways received from this device
        std::cout << "   - Receiving gateways: ";
        for (auto gwId : gateways) {
            std::cout << gwId << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "\n💾 Raw data saved to: " << m_outputFileName << std::endl;
}

} // namespace lorawan
} // namespace ns3