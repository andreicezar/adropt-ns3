/*
 * Copyright (c) 2017 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 */

#ifndef LORAWAN_MAC_H
#define LORAWAN_MAC_H

#include "logical-lora-channel-helper.h"
#include "lora-phy.h"

#include "ns3/object.h"
#include "ns3/packet.h"

#include <array>

namespace ns3
{
namespace lorawan
{

class LoraPhy;

/**
 * @ingroup lorawan
 *
 * Class representing the LoRaWAN MAC layer.
 *
 * This class is meant to be extended differently based on whether the layer
 * belongs to an end device or a gateway, while holding some functionality that
 * is common to both.
 */
class LorawanMac : public Object
{
  public:
    /**
     *  Register this type.
     *  @return The object TypeId.
     */
    static TypeId GetTypeId();

    LorawanMac();           //!< Default constructor
    ~LorawanMac() override; //!< Destructor

    /**
     * Matrix structure to store possible data rate value to be used by a LoRaWAN end device for
     * listening during the RX1 receive window. It is a function of the uplink data rate and the
     * RX1DROffset [0:5].
     */
    typedef std::array<std::array<uint8_t, 6>, 8> ReplyDataRateMatrix;

    /**
     * Set the underlying PHY layer.
     *
     * @param phy The phy layer.
     */
    void SetPhy(Ptr<LoraPhy> phy);

    /**
     * Get the underlying PHY layer.
     *
     * @return The PHY layer that this MAC is connected to.
     */
    Ptr<LoraPhy> GetPhy();

    /**
     * Send a packet.
     *
     * @param packet The packet to send.
     */
    virtual void Send(Ptr<Packet> packet) = 0;

    /**
     * Receive a packet from the lower layer.
     *
     * @param packet The received packet.
     */
    virtual void Receive(Ptr<const Packet> packet) = 0;

    /**
     * Function called by lower layers to inform this layer that reception of a
     * packet we were locked on failed.
     *
     * @param packet The packet we failed to receive.
     */
    virtual void FailedReception(Ptr<const Packet> packet) = 0;

    /**
     * Perform actions after sending a packet.
     *
     * @param packet The packet that just finished transmission.
     */
    virtual void TxFinished(Ptr<const Packet> packet) = 0;

    /**
     * Set the device this MAC layer is installed on.
     *
     * @param device The NetDevice this MAC layer will refer to.
     */
    void SetDevice(Ptr<NetDevice> device);

    /**
     * Get the device this MAC layer is installed on.
     *
     * @return The NetDevice this MAC layer will refer to.
     */
    Ptr<NetDevice> GetDevice();

    /**
     * Get the logical lora channel helper associated with this MAC.
     *
     * @return A Ptr to the instance of LogicalLoraChannelHelper that this MAC is using.
     */
    Ptr<LogicalLoraChannelHelper> GetLogicalLoraChannelHelper();

    /**
     * Set the LogicalLoraChannelHelper this MAC instance will use.
     *
     * @param helper A Ptr to the instance of the helper to use.
     */
    void SetLogicalLoraChannelHelper(Ptr<LogicalLoraChannelHelper> helper);

    /**
     * Get the spreading factor corresponding to a data rate, based on this MAC's region.
     *
     * @param dataRate The data rate we need to convert to a Spreading Factor
     * value.
     * @return The spreading factor that corresponds to a data rate in this MAC's region, or 0
     * if the dataRate is not valid.
     */
    uint8_t GetSfFromDataRate(uint8_t dataRate);

    /**
     * Get the bandwidth corresponding to a data rate, based on this MAC's region.
     *
     * @param dataRate The data rate we need to convert to a bandwidth value.
     * @return The bandwidth that corresponds to the parameter data rate in this
     * MAC's region, or 0 if the dataRate is not valid.
     */
    double GetBandwidthFromDataRate(uint8_t dataRate);

    /**
     * Get the transmission power in dBm that corresponds, in this region, to the
     * encoded 8-bit txPower.
     *
     * @param txPower The 8-bit encoded txPower to convert.
     *
     * @return The corresponding transmission power in dBm ERP, or -1 if the encoded
     * power was not recognized as valid.
     */
    double GetDbmForTxPower(uint8_t txPower);

    /**
     * Set the vector to use to check up correspondence between spreading factor and data rate.
     *
     * @param sfForDataRate A vector that contains at position i the spreading factor that
     * should correspond to data rate i.
     */
    void SetSfForDataRate(std::vector<uint8_t> sfForDataRate);

    /**
     * Set the vector to use to check up correspondence between bandwidth and
     * data rate.
     *
     * @param bandwidthForDataRate A vector that contains at position i the
     * bandwidth that should correspond to data rate i in this MAC's region.
     */
    void SetBandwidthForDataRate(std::vector<double> bandwidthForDataRate);

    /**
     * Set the maximum App layer payload for a set data rate.
     *
     * @param maxAppPayloadForDataRate A vector that contains at position i the
     * maximum Application layer payload that should correspond to data rate i in this
     * MAC's region.
     */
    void SetMaxAppPayloadForDataRate(std::vector<uint32_t> maxAppPayloadForDataRate);

    /**
     * Set the vector to use to check up which transmission power in Dbm
     * corresponds to a certain TxPower value in this MAC's region.
     *
     * @param txDbmForTxPower A vector that contains at position i the
     * transmission power in dBm that should correspond to a TXPOWER value of i in
     * this MAC's region.
     */
    void SetTxDbmForTxPower(std::vector<double> txDbmForTxPower);

    /**
     * Set the matrix to use when deciding with which data rate to respond. Region
     * based.
     *
     * @param replyDataRateMatrix A matrix containing the reply DataRates, based
     * on the sending data rate and on the value of the RX1DROffset parameter.
     */
    void SetReplyDataRateMatrix(ReplyDataRateMatrix replyDataRateMatrix);

    /**
     * Set the number of PHY preamble symbols this MAC is set to use.
     *
     * @param nPreambleSymbols The number of preamble symbols to use (typically 8).
     */
    void SetNPreambleSymbols(int nPreambleSymbols);

    /**
     * Get the number of PHY preamble symbols this MAC is set to use.
     *
     * @return The number of preamble symbols to use (typically 8).
     */
    int GetNPreambleSymbols() const;

  protected:
    /**
     * The trace source that is fired when a packet cannot be sent because of duty
     * cycle limitations.
     */
    TracedCallback<Ptr<const Packet>> m_cannotSendBecauseDutyCycle;

    /**
     * Trace source that is fired when a packet reaches the MAC layer.
     */
    TracedCallback<Ptr<const Packet>> m_receivedPacket;

    /**
     * Trace source that is fired when a new APP layer packet arrives at the MAC
     * layer.
     */
    TracedCallback<Ptr<const Packet>> m_sentNewPacket;

    /**
     * The PHY instance that sits under this MAC layer.
     */
    Ptr<LoraPhy> m_phy;

    /**
     * The device this MAC layer is installed on.
     */
    Ptr<NetDevice> m_device;

    /**
     * The LogicalLoraChannelHelper instance that is assigned to this MAC.
     */
    Ptr<LogicalLoraChannelHelper> m_channelHelper;

    /**
     * A vector holding the spreading factor each data rate corresponds to.
     */
    std::vector<uint8_t> m_sfForDataRate;

    /**
     * A vector holding the bandwidth each data rate corresponds to.
     */
    std::vector<double> m_bandwidthForDataRate;

    /**
     * A vector holding the maximum app payload size that corresponds to a
     * certain data rate.
     */
    std::vector<uint32_t> m_maxAppPayloadForDataRate;

    /**
     * The number of symbols to use in the PHY preamble.
     */
    int m_nPreambleSymbols;

    /**
     * A vector holding the power that corresponds to a certain TxPower value.
     */
    std::vector<double> m_txDbmForTxPower;

    /**
     * The matrix that decides the data rate the gateway will use in a reply based on the end
     * device's sending data rate and on the value of the RX1DROffset parameter.
     */
    ReplyDataRateMatrix m_replyDataRateMatrix;
};

} // namespace lorawan

} // namespace ns3
#endif /* LORAWAN_MAC_H */
