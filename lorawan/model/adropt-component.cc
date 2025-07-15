/*
 * Simple ADRopt Component Implementation - Starting from scratch
 * Only uses basic LoRaWAN module information available by default
 */

#include "adropt-component.h"
#include "ns3/lorawan-mac-header.h"
#include "ns3/lora-frame-header.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/end-device-lorawan-mac.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iomanip>

namespace ns3
{
namespace lorawan
{

NS_LOG_COMPONENT_DEFINE("ADRoptComponent");
NS_OBJECT_ENSURE_REGISTERED(ADRoptComponent);

// Constants for basic LoRaWAN calculations
const double ADRoptComponent::NOISE_FLOOR_DBM = -174.0;
const double ADRoptComponent::BANDWIDTH_HZ = 125000.0;
const uint8_t ADRoptComponent::MIN_TX_POWER = 2;
const uint8_t ADRoptComponent::MAX_TX_POWER = 14;
const uint8_t ADRoptComponent::PREAMBLE_LENGTH = 8;

TypeId
ADRoptComponent::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::lorawan::ADRoptComponent")
            .SetGroupName("lorawan")
            .SetParent<NetworkControllerComponent>()
            .AddConstructor<ADRoptComponent>()
            .AddAttribute("PERTarget",
                          "Target Packet Error Rate",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&ADRoptComponent::m_perTarget),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("HistoryRange",
                          "Number of packets to consider for history",
                          UintegerValue(20),
                          MakeUintegerAccessor(&ADRoptComponent::m_historyRange),
                          MakeUintegerChecker<uint32_t>(1, 100))
            .AddAttribute("EnablePowerControl",
                          "Enable transmission power control",
                          BooleanValue(true),
                          MakeBooleanAccessor(&ADRoptComponent::m_enablePowerControl),
                          MakeBooleanChecker())
            .AddAttribute("PayloadSize",
                          "Payload size in bytes for ToA calculation",
                          UintegerValue(20),
                          MakeUintegerAccessor(&ADRoptComponent::m_payloadSize),
                          MakeUintegerChecker<uint8_t>(1, 255))
            .AddTraceSource("NbTransChanged",
                           "Trace fired when NbTrans parameter changes",
                           MakeTraceSourceAccessor(&ADRoptComponent::m_nbTransChangedTrace),
                           "ns3::TracedCallback::Uint32Uint8Uint8")
            .AddTraceSource("TransmissionEfficiency",
                           "Trace fired when transmission efficiency is updated",
                           MakeTraceSourceAccessor(&ADRoptComponent::m_transmissionEfficiencyTrace),
                           "ns3::TracedCallback::Uint32Double")
            .AddTraceSource("AdrAdjustment",
                           "Trace fired when ADR parameters are adjusted",
                           MakeTraceSourceAccessor(&ADRoptComponent::m_adrAdjustmentTrace),
                           "ns3::TracedCallback::Uint32Uint8DoubleUint8")
            .AddTraceSource("ErrorRate",
                           "Trace fired when error rate is calculated",
                           MakeTraceSourceAccessor(&ADRoptComponent::m_errorRateTrace),
                           "ns3::TracedCallback::Uint32Uint32Uint32Double");
    return tid;
}

ADRoptComponent::ADRoptComponent()
    : m_perTarget(0.1),
      m_historyRange(20),
      m_enablePowerControl(true),
      m_payloadSize(20)
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("ADRopt Component initialized");
}

ADRoptComponent::~ADRoptComponent()
{
    NS_LOG_FUNCTION(this);
}

void
ADRoptComponent::OnReceivedPacket(Ptr<const Packet> packet,
                                  Ptr<EndDeviceStatus> status,
                                  Ptr<NetworkStatus> networkStatus)
{
    NS_LOG_FUNCTION(this << packet << status->m_endDeviceAddress);
    
    if (!status || !packet)
    {
        NS_LOG_ERROR("ADRopt: Null pointer received");
        return;
    }
    
    uint32_t deviceAddr = status->m_endDeviceAddress.Get();
    NS_LOG_INFO("ADRopt: Received packet from device " << deviceAddr);

    // ADDED: Prevent duplicate processing by checking if this exact packet was already processed
    static std::map<uint32_t, Time> lastPacketTime;
    Time currentTime = Simulator::Now();
    
    if (lastPacketTime.find(deviceAddr) != lastPacketTime.end() && 
        (currentTime - lastPacketTime[deviceAddr]).GetMicroSeconds() < 1000) // Within 1ms = duplicate
    {
        NS_LOG_DEBUG("ADRopt: Duplicate packet detected for device " << deviceAddr << ", skipping");
        return;
    }
    lastPacketTime[deviceAddr] = currentTime;

    // ADDED: Track SF and TxPower distributions for this packet
    auto& stats = m_packetTrackingStats[deviceAddr];
    
    // Get current SF and TxPower from device status
    uint8_t currentSF = status->GetFirstReceiveWindowSpreadingFactor();
    // REMOVED: uint8_t currentDR = SfToDr(currentSF); // Not used, causing compilation error
    
    double currentTxPower = 14.0; // Default EU868 max power
    Ptr<EndDeviceLorawanMac> endDeviceMac = DynamicCast<EndDeviceLorawanMac>(status->GetMac());
    if (endDeviceMac)
    {
        currentTxPower = endDeviceMac->GetTransmissionPowerDbm();
    }
    
    // ADDED: Record SF and TxPower usage for this packet
    stats.sfDistribution[currentSF]++;
    stats.txPowerDistribution[static_cast<int>(currentTxPower)]++;
    
    NS_LOG_DEBUG("ADRopt: Device " << deviceAddr << " packet using SF" << static_cast<uint32_t>(currentSF) 
                 << ", TxPower: " << currentTxPower << "dBm");

    // Record network server reception (this is the final destination)
    RecordNetworkServerReception(deviceAddr);
    
    // Extract gateway information from the received packet for diversity tracking
    std::vector<uint32_t> receivingGateways;
    auto receivedPacketList = status->GetReceivedPacketList();
    if (!receivedPacketList.empty())
    {
        const auto& latestPacket = receivedPacketList.back().second;
        for (const auto& gwPair : latestPacket.gwList)
        {
            // FIXED: Use the declared ExtractGatewayId method
            uint32_t gwNodeId = ExtractGatewayId(gwPair.first);
            receivingGateways.push_back(gwNodeId);
        }
        
        NS_LOG_DEBUG("Packet received by " << receivingGateways.size() << " gateways");
    }
    
    // Update per-gateway statistics for diversity analysis
    for (uint32_t gwId : receivingGateways)
    {
        stats.perGatewayReceptions[gwId]++;
    }
    
    // FIXED: Only count this as ONE successful reception regardless of gateway count
    // The packet reached the network server, so it was successfully received
    if (!receivingGateways.empty())
    {
        // This packet was received by at least one gateway and processed by network server
        // We already incremented packetsReceivedByNetworkServer in RecordNetworkServerReception
        // Now we need to ensure packetsReceivedByGateways is consistent
        
        // FIXED: Simply ensure consistency - both should be equal for successful NS receptions
        stats.packetsReceivedByGateways = stats.packetsReceivedByNetworkServer;
    }

    // Update device statistics for ADR processing
    auto& deviceStats = m_deviceStats[deviceAddr];
    deviceStats.totalPackets++;
    deviceStats.successfulTransmissions++;
    deviceStats.lastUpdateTime = Simulator::Now();
    
    // Estimate transmission attempts for this packet based on current NbTrans
    deviceStats.totalTransmissionAttempts += deviceStats.currentNbTrans;
    
    // Update transmission efficiency
    double oldEfficiency = deviceStats.averageTransmissionsPerPacket;
    deviceStats.averageTransmissionsPerPacket = CalculateTransmissionEfficiency(deviceStats);
    
    // Fire trace if efficiency changed significantly
    if (std::abs(oldEfficiency - deviceStats.averageTransmissionsPerPacket) > 0.01)
    {
        m_transmissionEfficiencyTrace(deviceAddr, deviceStats.averageTransmissionsPerPacket);
    }

    // Get the latest received packet info for ADR processing
    if (!receivedPacketList.empty())
    {
        const auto& latestPacket = receivedPacketList.back().second;
        deviceStats.packetHistory.push_back(latestPacket);

        // Keep history within range
        if (deviceStats.packetHistory.size() > m_historyRange)
        {
            deviceStats.packetHistory.pop_front();
        }
    }
    else
    {
        NS_LOG_DEBUG("ADRopt: No packet list available for device " << deviceAddr);
    }

    // Recalculate error rates with corrected logic
    CalculateErrorRates(deviceAddr);

    NS_LOG_DEBUG("ADRopt: Device " << deviceAddr 
                 << " - Packets: " << deviceStats.totalPackets
                 << ", NbTrans: " << static_cast<uint32_t>(deviceStats.currentNbTrans)
                 << ", Efficiency: " << deviceStats.averageTransmissionsPerPacket
                 << ", History: " << deviceStats.packetHistory.size()
                 << ", SF: " << static_cast<uint32_t>(currentSF)
                 << ", TxPower: " << currentTxPower << "dBm");
}

uint32_t
ADRoptComponent::ExtractGatewayId(const Address& gwAddr)
{
    try 
    {
        uint8_t addressLength = gwAddr.GetLength();
        if (addressLength >= 4)
        {
            uint8_t buffer[16];
            gwAddr.CopyTo(buffer);
            
            if (addressLength == 4)
            {
                return (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];
            }
            else if (addressLength == 6)
            {
                return (buffer[5] << 8) | buffer[4];
            }
            else
            {
                int offset = addressLength - 4;
                return (buffer[offset+3] << 24) | (buffer[offset+2] << 16) | 
                       (buffer[offset+1] << 8) | buffer[offset];
            }
        }
        else
        {
            uint8_t buffer[4] = {0};
            gwAddr.CopyTo(buffer);
            return (buffer[1] << 8) | buffer[0];
        }
    }
    catch (...)
    {
        static uint32_t fallbackId = 1000;
        return fallbackId++;
    }
}
void
ADRoptComponent::BeforeSendingReply(Ptr<EndDeviceStatus> status,
                                   Ptr<NetworkStatus> networkStatus)
{
    NS_LOG_FUNCTION(this << status->m_endDeviceAddress);
    
    if (!status)
    {
        NS_LOG_ERROR("ADRopt: Null status pointer in BeforeSendingReply");
        return;
    }
    
    uint32_t deviceAddr = status->m_endDeviceAddress.Get();
    NS_LOG_INFO("ADRopt: Processing ADR for device " << deviceAddr);

    // Check if we have a packet to analyze
    Ptr<const Packet> lastPacket = status->GetLastPacketReceivedFromDevice();
    if (!lastPacket)
    {
        NS_LOG_DEBUG("ADRopt: No packet available for device " << deviceAddr);
        return;
    }

    // Parse packet to check ADR bit - be careful with packet modification
    try 
    {
        Ptr<Packet> packet = lastPacket->Copy();
        LorawanMacHeader mHdr;
        LoraFrameHeader fHdr;
        
        if (packet->GetSize() < mHdr.GetSerializedSize())
        {
            NS_LOG_DEBUG("ADRopt: Packet too small for MAC header");
            return;
        }
        
        packet->RemoveHeader(mHdr);
        
        if (packet->GetSize() < fHdr.GetSerializedSize())
        {
            NS_LOG_DEBUG("ADRopt: Packet too small for frame header");
            return;
        }
        
        packet->RemoveHeader(fHdr);
        
        if (!fHdr.GetAdr())
        {
            NS_LOG_DEBUG("ADRopt: ADR bit not set for device " << deviceAddr);
            return;
        }
    }
    catch (const std::exception& e)
    {
        NS_LOG_ERROR("ADRopt: Exception parsing packet headers: " << e.what());
        return;
    }

    // Check if we have enough history
    auto& deviceStats = m_deviceStats[deviceAddr];
    if (deviceStats.packetHistory.size() < m_historyRange)
    {
        NS_LOG_DEBUG("ADRopt: Insufficient history for device " << deviceAddr 
                     << " (have " << deviceStats.packetHistory.size() 
                     << ", need " << m_historyRange << ")");
        return;
    }

    // Get current NbTrans for tracking
    uint8_t oldNbTrans = deviceStats.currentNbTrans;

    // Run ADR algorithm
    uint8_t newDataRate;
    double newTxPower;
    uint8_t newNbTrans;
    
    bool parametersChanged = RunADRoptAlgorithm(&newDataRate, &newTxPower, &newNbTrans, status);
    
    if (parametersChanged)
    {
        NS_LOG_INFO("ADRopt: New parameters for device " << deviceAddr 
                    << " - DR: " << static_cast<uint32_t>(newDataRate) 
                    << ", TxPower: " << newTxPower 
                    << ", NbTrans: " << static_cast<uint32_t>(newNbTrans)
                    << " (was: " << static_cast<uint32_t>(oldNbTrans) << ")");

        // Update transmission tracking if NbTrans changed
        if (newNbTrans != oldNbTrans)
        {
            UpdateTransmissionStats(deviceAddr, newNbTrans, oldNbTrans);
        }

        // Create LinkAdrReq command
        std::list<int> enabledChannels = {0, 1, 2}; // Standard channels
        
        status->m_reply.frameHeader.AddLinkAdrReq(newDataRate,
                                                  GetTxPowerIndex(newTxPower),
                                                  enabledChannels,
                                                  newNbTrans);
        status->m_reply.frameHeader.SetAsDownlink();
        status->m_reply.macHeader.SetMType(LorawanMacHeader::UNCONFIRMED_DATA_DOWN);
        status->m_reply.needsReply = true;

        // Update internal tracking
        m_deviceNbTrans[deviceAddr] = newNbTrans;
        
        // Update MAC parameters if possible
        Ptr<ClassAEndDeviceLorawanMac> mac = DynamicCast<ClassAEndDeviceLorawanMac>(status->GetMac());
        if (mac)
        {
            mac->SetDataRate(newDataRate);
            mac->SetTransmissionPowerDbm(newTxPower);
        }

        // Fire trace for ADR adjustment
        m_adrAdjustmentTrace(deviceAddr, newDataRate, newTxPower, newNbTrans);

        // Clear history to start fresh cycle
        deviceStats.packetHistory.clear();
        
        // Print current statistics for this device
        PrintDeviceTransmissionStats(deviceAddr);
    }
    else
    {
        NS_LOG_DEBUG("ADRopt: No parameter change needed for device " << deviceAddr
                     << " (current NbTrans: " << static_cast<uint32_t>(oldNbTrans) << ")");
    }
}

void
ADRoptComponent::OnFailedReply(Ptr<EndDeviceStatus> status,
                               Ptr<NetworkStatus> networkStatus)
{
    NS_LOG_FUNCTION(this << status->m_endDeviceAddress);
    NS_LOG_WARN("ADRopt: Failed reply for device " << status->m_endDeviceAddress.Get());
    // Could implement retry logic here
}

// Transmission tracking methods implementation

uint8_t
ADRoptComponent::GetCurrentNbTrans(uint32_t deviceAddr) const
{
    auto it = m_deviceStats.find(deviceAddr);
    if (it != m_deviceStats.end())
    {
        return it->second.currentNbTrans;
    }
    return 1; // Default value
}

double
ADRoptComponent::GetTransmissionEfficiency(uint32_t deviceAddr) const
{
    auto it = m_deviceStats.find(deviceAddr);
    if (it != m_deviceStats.end())
    {
        return it->second.averageTransmissionsPerPacket;
    }
    return 1.0; // Default efficiency
}

uint32_t
ADRoptComponent::GetTotalTransmissionAttempts(uint32_t deviceAddr) const
{
    auto it = m_deviceStats.find(deviceAddr);
    if (it != m_deviceStats.end())
    {
        return it->second.totalTransmissionAttempts;
    }
    return 0;
}

uint32_t
ADRoptComponent::GetAdrAdjustmentCount(uint32_t deviceAddr) const
{
    auto it = m_deviceStats.find(deviceAddr);
    if (it != m_deviceStats.end())
    {
        return it->second.adrAdjustmentCount;
    }
    return 0;
}

void
ADRoptComponent::PrintTransmissionStatistics() const
{
    NS_LOG_INFO("=== ADRopt Transmission Statistics ===");
    for (const auto& pair : m_deviceStats)
    {
        PrintDeviceTransmissionStats(pair.first);
    }
}

void
ADRoptComponent::PrintDeviceTransmissionStats(uint32_t deviceAddr) const
{
    auto it = m_deviceStats.find(deviceAddr);
    if (it == m_deviceStats.end())
    {
        NS_LOG_INFO("No statistics available for device " << deviceAddr);
        return;
    }
    
    const DeviceStats& stats = it->second;
    NS_LOG_INFO("Device " << deviceAddr << " Transmission Stats:");
    NS_LOG_INFO("  Current NbTrans: " << static_cast<uint32_t>(stats.currentNbTrans));
    NS_LOG_INFO("  Previous NbTrans: " << static_cast<uint32_t>(stats.previousNbTrans));
    NS_LOG_INFO("  Total Packets: " << stats.totalPackets);
    NS_LOG_INFO("  Total Transmission Attempts: " << stats.totalTransmissionAttempts);
    NS_LOG_INFO("  Successful Transmissions: " << stats.successfulTransmissions);
    NS_LOG_INFO("  ADR Adjustments: " << stats.adrAdjustmentCount);
    NS_LOG_INFO("  Transmission Efficiency: " << stats.averageTransmissionsPerPacket);
    NS_LOG_INFO("  Last NbTrans Change: " << stats.lastNbTransChange.As(Time::S));
}

void
ADRoptComponent::UpdateTransmissionStats(uint32_t deviceAddr, uint8_t newNbTrans, uint8_t oldNbTrans)
{
    auto& stats = m_deviceStats[deviceAddr];
    stats.previousNbTrans = oldNbTrans;
    stats.currentNbTrans = newNbTrans;
    stats.lastNbTransChange = Simulator::Now();
    stats.adrAdjustmentCount++;
    
    // Fire trace
    m_nbTransChangedTrace(deviceAddr, oldNbTrans, newNbTrans);
    
    NS_LOG_INFO("Device " << deviceAddr << " NbTrans updated: " 
                << static_cast<uint32_t>(oldNbTrans) << " -> " 
                << static_cast<uint32_t>(newNbTrans));
}

double
ADRoptComponent::CalculateTransmissionEfficiency(const DeviceStats& stats) const
{
    if (stats.successfulTransmissions == 0)
    {
        return 1.0; // No data available
    }
    
    return static_cast<double>(stats.totalTransmissionAttempts) / stats.successfulTransmissions;
}

// Core ADR algorithm and other existing methods...

bool
ADRoptComponent::RunADRoptAlgorithm(uint8_t* newDataRate,
                                   double* newTxPowerDbm,
                                   uint8_t* newNbTrans,
                                   Ptr<EndDeviceStatus> status)
{
    NS_LOG_FUNCTION(this);
    
    if (!status || !newDataRate || !newTxPowerDbm || !newNbTrans)
    {
        NS_LOG_ERROR("ADRopt: Null pointer in RunADRoptAlgorithm");
        return false;
    }
    
    // Check if MAC is available and cast to the right type
    Ptr<EndDeviceLorawanMac> endDeviceMac = DynamicCast<EndDeviceLorawanMac>(status->GetMac());
    if (!endDeviceMac)
    {
        NS_LOG_ERROR("ADRopt: No EndDeviceLorawanMac available for device");
        return false;
    }
    
    // Get current parameters
    uint8_t currentSF = status->GetFirstReceiveWindowSpreadingFactor();
    uint8_t currentDR = SfToDr(currentSF);
    double currentTxPower = endDeviceMac->GetTransmissionPowerDbm();
    uint32_t deviceAddr = status->m_endDeviceAddress.Get();
    
    // Get current NbTrans (default to 1 if not tracked)
    auto it = m_deviceNbTrans.find(deviceAddr);
    uint8_t currentNbTrans = (it != m_deviceNbTrans.end()) ? it->second : 1;
    
    NS_LOG_DEBUG("ADRopt: Current config - DR: " << static_cast<uint32_t>(currentDR) 
                 << ", TxPower: " << currentTxPower 
                 << ", NbTrans: " << static_cast<uint32_t>(currentNbTrans));

    // Estimate current PER
    double currentPER = EstimateCurrentPER(status);
    NS_LOG_DEBUG("ADRopt: Estimated current PER: " << currentPER);

    // Initialize best configuration with current values
    *newDataRate = currentDR;
    *newTxPowerDbm = currentTxPower;
    *newNbTrans = currentNbTrans;
    
    double bestToA = CalculateToA(currentDR, currentNbTrans);
    bool foundBetter = false;

    // Search for optimal configuration
    // Try different data rates (DR0-DR5, i.e., SF12-SF7)
    for (uint8_t dr = 0; dr <= 5; ++dr)
    {
        // Try different power levels
        double startPower = m_enablePowerControl ? MIN_TX_POWER : currentTxPower;
        double endPower = m_enablePowerControl ? MAX_TX_POWER : currentTxPower;
        
        for (double power = startPower; power <= endPower; power += 2.0)
        {
            // Try different NbTrans values
            for (uint8_t nbt = 1; nbt <= 3; ++nbt)
            {
                double predictedPER = PredictPER(dr, power, nbt, status);
                double toa = CalculateToA(dr, nbt);
                
                NS_LOG_DEBUG("ADRopt: Testing DR" << static_cast<uint32_t>(dr) 
                            << ", Power:" << power 
                            << ", NbTrans:" << static_cast<uint32_t>(nbt) 
                            << " -> PER:" << predictedPER 
                            << ", ToA:" << toa);

                // Check if this configuration meets PER target and improves ToA
                if (predictedPER <= m_perTarget && toa < bestToA)
                {
                    *newDataRate = dr;
                    *newTxPowerDbm = power;
                    *newNbTrans = nbt;
                    bestToA = toa;
                    foundBetter = true;
                    
                    NS_LOG_DEBUG("ADRopt: New best config found");
                }
            }
        }
    }

    return foundBetter;
}

double
ADRoptComponent::EstimateCurrentPER(Ptr<EndDeviceStatus> status)
{
    NS_LOG_FUNCTION(this);
    
    uint32_t deviceAddr = status->m_endDeviceAddress.Get();
    auto& deviceStats = m_deviceStats[deviceAddr];
    
    if (deviceStats.packetHistory.empty())
    {
        return 1.0; // Assume worst case if no history
    }

    // Simple PER estimation based on packet reception
    uint32_t receivedCount = 0;
    uint32_t totalCount = deviceStats.packetHistory.size();
    
    for (const auto& pktInfo : deviceStats.packetHistory)
    {
        if (!pktInfo.gwList.empty())
        {
            receivedCount++;
        }
    }
    
    double pdr = static_cast<double>(receivedCount) / totalCount;
    double per = 1.0 - pdr;
    
    NS_LOG_DEBUG("ADRopt: PDR: " << pdr << ", PER: " << per);
    return per;
}

double
ADRoptComponent::PredictPER(uint8_t dataRate, double txPower, uint8_t nbTrans, 
                           Ptr<EndDeviceStatus> status)
{
    NS_LOG_FUNCTION(this);
    
    if (!status)
    {
        NS_LOG_ERROR("ADRopt: Null status in PredictPER");
        return 1.0;
    }
    
    std::set<Address> gateways = GetActiveGateways(status);
    if (gateways.empty())
    {
        NS_LOG_DEBUG("ADRopt: No active gateways for PER prediction");
        return 1.0; // No gateways available
    }

    double combinedPER = 1.0;
    
    // Get current TX power from EndDeviceMac
    Ptr<EndDeviceLorawanMac> endDeviceMac = DynamicCast<EndDeviceLorawanMac>(status->GetMac());
    if (!endDeviceMac)
    {
        NS_LOG_ERROR("ADRopt: No EndDeviceLorawanMac available in PredictPER");
        return 1.0;
    }
    
    double currentTxPower = endDeviceMac->GetTransmissionPowerDbm();
    
    // For each gateway, calculate the individual PER
    for (const Address& gwAddr : gateways)
    {
        double meanSNR = GetMeanSNRForGateway(gwAddr, status);
        
        // Adjust SNR based on power difference from current
        double snrAdjustment = txPower - currentTxPower;
        double adjustedSNR = meanSNR + snrAdjustment;
        
        // Calculate FER for this gateway and data rate
        double fer = CalculateFER(dataRate, adjustedSNR);
        
        // Apply NbTrans (multiple transmission attempts)
        double perThisGW = std::pow(fer, nbTrans);
        combinedPER *= perThisGW;
    }
    
    return std::min(combinedPER, 1.0);
}

double
ADRoptComponent::CalculateToA(uint8_t dataRate, uint8_t nbTrans)
{
    NS_LOG_FUNCTION(this);
    
    uint8_t sf = DrToSf(dataRate);
    double symbolTime = std::pow(2.0, sf) / BANDWIDTH_HZ;
    
    // Simplified ToA calculation
    double preambleTime = (PREAMBLE_LENGTH + 4.25) * symbolTime;
    
    // Approximate payload symbols (simplified)
    double payloadSymbols = 8 + std::max(0.0, std::ceil((8.0 * m_payloadSize - 4.0 * sf + 28.0 + 16.0) / (4.0 * sf)) * 5.0);
    double payloadTime = payloadSymbols * symbolTime;
    
    double singleToA = preambleTime + payloadTime;
    return singleToA * nbTrans * 1000.0; // Return in milliseconds
}

double
ADRoptComponent::RxPowerToSNR(double rxPowerDbm)
{
    // Simple noise floor calculation
    double noiseFloorDbm = NOISE_FLOOR_DBM + 10.0 * std::log10(BANDWIDTH_HZ) + 6.0; // 6dB NF
    return rxPowerDbm - noiseFloorDbm;
}

double
ADRoptComponent::CalculateFER(uint8_t dataRate, double snr)
{
    double snrThreshold = GetSNRThreshold(dataRate);
    
    // Simple exponential model for FER
    if (snr >= snrThreshold)
    {
        double margin = snr - snrThreshold;
        return std::exp(-margin); // Exponential decay with positive margin
    }
    else
    {
        return 1.0; // High error rate when below threshold
    }
}

double
ADRoptComponent::GetSNRThreshold(uint8_t dataRate)
{
    // LoRaWAN SNR thresholds for different data rates
    uint8_t sf = DrToSf(dataRate);
    return -20.0 + (12 - sf) * 2.5; // Standard LoRa SNR thresholds
}

uint8_t
ADRoptComponent::SfToDr(uint8_t sf)
{
    if (sf >= 7 && sf <= 12) return 12 - sf;
    return 0; // Default to DR0
}

uint8_t
ADRoptComponent::DrToSf(uint8_t dr)
{
    if (dr <= 5) return 12 - dr;
    return 12; // Default to SF12
}

uint8_t
ADRoptComponent::GetTxPowerIndex(double txPowerDbm)
{
    // EU868 power index mapping (0=14dBm, 1=12dBm, ..., 6=2dBm)
    txPowerDbm = std::max(static_cast<double>(MIN_TX_POWER), 
                         std::min(static_cast<double>(MAX_TX_POWER), txPowerDbm));
    return static_cast<uint8_t>((MAX_TX_POWER - txPowerDbm) / 2);
}

std::set<Address>
ADRoptComponent::GetActiveGateways(Ptr<EndDeviceStatus> status)
{
    NS_LOG_FUNCTION(this);
    
    std::set<Address> gateways;
    uint32_t deviceAddr = status->m_endDeviceAddress.Get();
    auto& deviceStats = m_deviceStats[deviceAddr];
    
    for (const auto& pktInfo : deviceStats.packetHistory)
    {
        for (const auto& gwPair : pktInfo.gwList)
        {
            gateways.insert(gwPair.first);
        }
    }
    
    return gateways;
}

double
ADRoptComponent::GetMeanSNRForGateway(const Address& gwAddr, Ptr<EndDeviceStatus> status)
{
    NS_LOG_FUNCTION(this);
    
    uint32_t deviceAddr = status->m_endDeviceAddress.Get();
    auto& deviceStats = m_deviceStats[deviceAddr];
    
    std::vector<double> snrValues;
    
    for (const auto& pktInfo : deviceStats.packetHistory)
    {
        auto gwIt = pktInfo.gwList.find(gwAddr);
        if (gwIt != pktInfo.gwList.end())
        {
            double snr = RxPowerToSNR(gwIt->second.rxPower);
            snrValues.push_back(snr);
        }
    }
    
    if (snrValues.empty())
    {
        return -30.0; // Default low SNR
    }
    
    double sum = std::accumulate(snrValues.begin(), snrValues.end(), 0.0);
    return sum / snrValues.size();
}

void
ADRoptComponent::RecordPacketTransmission(uint32_t deviceAddr)
{
    NS_LOG_FUNCTION(this << deviceAddr);
    
    auto& stats = m_packetTrackingStats[deviceAddr];
    stats.totalPacketsSent++;
    
    Time now = Simulator::Now();
    if (stats.firstPacketTime == Time::Max())
    {
        stats.firstPacketTime = now;
    }
    stats.lastPacketTime = now;
    
    NS_LOG_DEBUG("Device " << deviceAddr << " transmission recorded. Total sent: " << stats.totalPacketsSent);
}

void
ADRoptComponent::RecordGatewayReception(uint32_t deviceAddr, const std::vector<uint32_t>& receivingGateways)
{
    // REMOVED: This method should not be called from OnReceivedPacket anymore
    // Gateway reception tracking is now handled directly in OnReceivedPacket
    NS_LOG_FUNCTION(this << deviceAddr << receivingGateways.size());
    
    auto& stats = m_packetTrackingStats[deviceAddr];
    
    if (!receivingGateways.empty())
    {
        stats.packetsReceivedByGateways++;
        
        // Track per-gateway receptions
        for (uint32_t gwId : receivingGateways)
        {
            stats.perGatewayReceptions[gwId]++;
        }
        
        NS_LOG_DEBUG("Device " << deviceAddr << " packet received by " << receivingGateways.size() 
                     << " gateways. Total received: " << stats.packetsReceivedByGateways);
    }
    
    // Recalculate error rates
    CalculateErrorRates(deviceAddr);
}

void
ADRoptComponent::RecordNetworkServerReception(uint32_t deviceAddr)
{
    NS_LOG_FUNCTION(this << deviceAddr);
    
    auto& stats = m_packetTrackingStats[deviceAddr];
    stats.packetsReceivedByNetworkServer++;
    
    NS_LOG_DEBUG("Device " << deviceAddr << " packet processed by network server. Total: " 
                 << stats.packetsReceivedByNetworkServer);
    
    // Recalculate error rates
    CalculateErrorRates(deviceAddr);
}

void
ADRoptComponent::CalculateErrorRates(uint32_t deviceAddr)
{
    NS_LOG_FUNCTION(this << deviceAddr);
    
    auto& stats = m_packetTrackingStats[deviceAddr];
    
    // Ensure consistency: packets received by gateways should equal network server reception
    // since OnReceivedPacket only fires when network server processes a packet
    if (stats.packetsReceivedByNetworkServer > 0)
    {
        stats.packetsReceivedByGateways = stats.packetsReceivedByNetworkServer;
    }
    
    // Device to Gateway Error Rate
    if (stats.totalPacketsSent > 0)
    {
        stats.deviceToGatewayErrorRate = 
            static_cast<double>(stats.totalPacketsSent - stats.packetsReceivedByGateways) / 
            stats.totalPacketsSent;
    }
    
    // Gateway to Network Server Error Rate (should be 0 in this simulation)
    stats.gatewayToNetworkServerErrorRate = 0.0; // Always 0 since we only count successful NS receptions
    
    // End-to-End Error Rate
    if (stats.totalPacketsSent > 0)
    {
        stats.endToEndErrorRate = 
            static_cast<double>(stats.totalPacketsSent - stats.packetsReceivedByNetworkServer) / 
            stats.totalPacketsSent;
    }
    
    // Fire trace
    m_errorRateTrace(deviceAddr, stats.totalPacketsSent, stats.packetsReceivedByNetworkServer, 
                     stats.endToEndErrorRate);
    
    NS_LOG_DEBUG("Device " << deviceAddr << " error rates - "
                 << "Device->GW: " << (stats.deviceToGatewayErrorRate * 100) << "%, "
                 << "GW->NS: " << (stats.gatewayToNetworkServerErrorRate * 100) << "%, "
                 << "End-to-End: " << (stats.endToEndErrorRate * 100) << "%");
}

ADRoptComponent::PacketTrackingStats
ADRoptComponent::GetPacketTrackingStats(uint32_t deviceAddr) const
{
    auto it = m_packetTrackingStats.find(deviceAddr);
    if (it != m_packetTrackingStats.end())
    {
        return it->second;
    }
    return PacketTrackingStats(); // Return default empty stats
}

void
ADRoptComponent::PrintPacketTrackingStatistics() const
{
    NS_LOG_INFO("=== Comprehensive Packet Tracking Statistics ===");
    
    uint32_t totalDevices = m_packetTrackingStats.size();
    uint32_t totalSent = 0;
    uint32_t totalReceivedGW = 0;
    uint32_t totalReceivedNS = 0;
    
    for (const auto& pair : m_packetTrackingStats)
    {
        totalSent += pair.second.totalPacketsSent;
        totalReceivedGW += pair.second.packetsReceivedByGateways;
        totalReceivedNS += pair.second.packetsReceivedByNetworkServer;
        PrintDevicePacketTrackingStats(pair.first);
    }
    
    // Network-wide statistics
    NS_LOG_INFO("=== Network-Wide Packet Statistics ===");
    NS_LOG_INFO("Total Devices: " << totalDevices);
    NS_LOG_INFO("Total Packets Sent: " << totalSent);
    NS_LOG_INFO("Total Received by Gateways: " << totalReceivedGW);
    NS_LOG_INFO("Total Processed by Network Server: " << totalReceivedNS);
    
    if (totalSent > 0)
    {
        double networkWideErrorRate = static_cast<double>(totalSent - totalReceivedNS) / totalSent;
        NS_LOG_INFO("Network-Wide Error Rate: " << (networkWideErrorRate * 100) << "%");
        NS_LOG_INFO("Network-Wide PDR: " << ((1.0 - networkWideErrorRate) * 100) << "%");
    }
}

void
ADRoptComponent::PrintDevicePacketTrackingStats(uint32_t deviceAddr) const
{
    auto it = m_packetTrackingStats.find(deviceAddr);
    if (it == m_packetTrackingStats.end())
    {
        NS_LOG_INFO("No packet tracking statistics available for device " << deviceAddr);
        return;
    }
    
    const PacketTrackingStats& stats = it->second;
    
    NS_LOG_INFO("=== Device " << deviceAddr << " Comprehensive Packet Tracking ===");
    NS_LOG_INFO("  Total Packets Sent: " << stats.totalPacketsSent);
    NS_LOG_INFO("  Received by Gateways: " << stats.packetsReceivedByGateways);
    NS_LOG_INFO("  Processed by Network Server: " << stats.packetsReceivedByNetworkServer);
    
    NS_LOG_INFO("  Device -> Gateway Error Rate: " << (stats.deviceToGatewayErrorRate * 100) << "%");
    NS_LOG_INFO("  Gateway -> Network Server Error Rate: " << (stats.gatewayToNetworkServerErrorRate * 100) << "%");
    NS_LOG_INFO("  End-to-End Error Rate: " << (stats.endToEndErrorRate * 100) << "%");
    NS_LOG_INFO("  Packet Delivery Rate: " << ((1.0 - stats.endToEndErrorRate) * 100) << "%");
    
    // ADDED: Spreading Factor Distribution
    if (!stats.sfDistribution.empty())
    {
        NS_LOG_INFO("  📡 Spreading Factor Distribution:");
        uint32_t totalSFPackets = 0;
        for (const auto& sfPair : stats.sfDistribution)
        {
            totalSFPackets += sfPair.second;
        }
        
        for (const auto& sfPair : stats.sfDistribution)
        {
            double percentage = (static_cast<double>(sfPair.second) / totalSFPackets) * 100;
            NS_LOG_INFO("    SF" << static_cast<uint32_t>(sfPair.first) << ": " 
                       << sfPair.second << " packets (" << std::fixed << std::setprecision(1) 
                       << percentage << "%)");
        }
    }
    
    // ADDED: TX Power Distribution
    if (!stats.txPowerDistribution.empty())
    {
        NS_LOG_INFO("  ⚡ TX Power Distribution:");
        uint32_t totalPowerPackets = 0;
        for (const auto& powerPair : stats.txPowerDistribution)
        {
            totalPowerPackets += powerPair.second;
        }
        
        for (const auto& powerPair : stats.txPowerDistribution)
        {
            double percentage = (static_cast<double>(powerPair.second) / totalPowerPackets) * 100;
            NS_LOG_INFO("    " << powerPair.first << "dBm: " 
                       << powerPair.second << " packets (" << std::fixed << std::setprecision(1) 
                       << percentage << "%)");
        }
    }
    
    // Per-gateway reception statistics
    if (!stats.perGatewayReceptions.empty())
    {
        NS_LOG_INFO("  🌐 Per-Gateway Receptions:");
        for (const auto& gwPair : stats.perGatewayReceptions)
        {
            if (stats.packetsReceivedByNetworkServer > 0)
            {
                double gatewayReceptionRate = static_cast<double>(gwPair.second) / stats.packetsReceivedByNetworkServer;
                NS_LOG_INFO("    Gateway " << gwPair.first << ": " << gwPair.second 
                           << " packets (" << std::fixed << std::setprecision(1) 
                           << (gatewayReceptionRate * 100) << "%)");
            }
        }
    }
    
    // Timing information
    if (stats.firstPacketTime != Time::Max() && stats.packetsReceivedByNetworkServer > 0)
    {
        double durationSeconds = (stats.lastPacketTime - stats.firstPacketTime).GetSeconds();
        NS_LOG_INFO("  ⏱️  Duration: " << durationSeconds << " seconds");
        if (durationSeconds > 0)
        {
            double packetsPerHour = (stats.packetsReceivedByNetworkServer * 3600.0) / durationSeconds;
            NS_LOG_INFO("  📊 Reception Rate: " << std::fixed << std::setprecision(1) 
                       << packetsPerHour << " packets/hour");
        }
    }
}

} // namespace lorawan
} // namespace ns3