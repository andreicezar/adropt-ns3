#ifndef RSSI_SNIR_TRACKER_H
#define RSSI_SNIR_TRACKER_H

#include "ns3/object.h"
#include "ns3/node-container.h"
#include "ns3/packet.h"
#include "ns3/lorawan-mac-header.h"
#include "ns3/callback.h"
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <set>

namespace ns3 {
namespace lorawan {

// Data structure to hold a single reception measurement
struct RssiSnirMeasurement
{
    double timestamp;
    uint32_t deviceAddr;
    uint32_t gatewayNodeId;
    double rssi;
    double snir;
};

class RssiSnirTracker : public Object
{
public:
    static TypeId GetTypeId();
    RssiSnirTracker();
    virtual ~RssiSnirTracker();

    void Initialize(NodeContainer gateways);
    void StartTracking(const std::string& filename);
    void StopTracking();
    void PrintAnalysis();

private:
    // Callback methods - one for each parameter combination we need
    void OnReceivedPacketGateway(uint32_t gatewayNodeId, Ptr<const Packet> packet, uint32_t traceNodeId);
    
    // Helper to extract device address from packet
    uint32_t ExtractDeviceAddress(Ptr<const Packet> packet);

    // Member variables for tracking
    bool m_isTracking;
    std::ofstream m_outputFile;
    std::string m_outputFileName;
    std::vector<RssiSnirMeasurement> m_measurements;
};

} // namespace lorawan
} // namespace ns3

#endif // RSSI_SNIR_TRACKER_H