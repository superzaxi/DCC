// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_MAC_H
#define SCENSIM_MAC_H

#include <array>
#include "randomnumbergen.h"
#include "scensim_stats.h"
#include "scensim_queues.h"
#include "scensim_qoscontrol.h"
#include "scensim_tracedefs.h"

#include <iostream>//20210613

namespace ScenSim {

using std::cout;//20210613
using std::endl;//20210613

class ObjectMobilityPosition;

template<class MacAddress>
class MacAddressResolver {
public:
    virtual ~MacAddressResolver() { }

    virtual void GetMacAddress(
        const NetworkAddress& aNetworkAddress,
        const NetworkAddress& networkAddressMask,
        bool& wasFound,
        MacAddress& resolvedMacAddress) = 0;

    // Used only to get last hop address (not used in most situations).

    virtual void GetNetworkAddressIfAvailable(
        const MacAddress& macAddress,
        const NetworkAddress& subnetNetworkAddress,
        bool& wasFound,
        NetworkAddress& resolvedNetworkAddress) = 0;

};//MacAddressResolver//

typedef unsigned long long int GenericMacAddress;

inline
NodeId CalcNodeId(const GenericMacAddress& genericMacAddress)
{
    // Lower Bits always Node ID.
    return (static_cast<NodeId>(genericMacAddress));
}

const unsigned int MaxMulticastGroupNumber = (256 * 256 * 256) - 1;


// IEEE 802.X Style 6 byte MAC address.

class SixByteMacAddress {
public:
    static const SixByteMacAddress invalidMacAddress;

    static const unsigned int numberMacAddressBytes = 6;

    static SixByteMacAddress GetBroadcastAddress();

    SixByteMacAddress() { std::fill(addressBytes.begin(), addressBytes.end(), 0); }

    SixByteMacAddress(const NodeId theNodeId, const unsigned char interfaceSelectorByte) {
        addressBytes[addressTypeCodeBytePos] = 0;
        addressBytes[interfaceSelectorBytePos] = interfaceSelectorByte;
        (*this).SetLowerBitsWithNodeId(theNodeId);
    }

    void SetFromString(
        const string& addressString,
        bool& success);

    explicit
    SixByteMacAddress(const GenericMacAddress& address);

    GenericMacAddress ConvertToGenericMacAddress() const;

    void Clear() {
        for(unsigned int i = 0; (i < numberMacAddressBytes); i++) {
            addressBytes[i] = 0;
        }
    }

    bool IsABroadcastAddress() const
        { return (addressBytes[addressTypeCodeBytePos] == broadcastIndicatorByteCode); }

    bool IsAMulticastAddress() const
        { return (addressBytes[addressTypeCodeBytePos] == multicastIndicatorByteCode); }

    bool IsABroadcastOrAMulticastAddress() const
        { return (IsABroadcastAddress() || IsAMulticastAddress()); }

    bool operator==(const SixByteMacAddress& right) const;


    bool operator!=(const SixByteMacAddress& right) const { return (!(*this == right)); }

    bool operator<(const SixByteMacAddress& right) const;

    // Extra-simulation hack to encode Node Id into lower bits of mac-address.

    void SetLowerBitsWithNodeId(const NodeId theNodeId);

    NodeId ExtractNodeId() const;

    void SetInterfaceSelectorByte(const unsigned char selectorByte) {
        addressBytes[interfaceSelectorBytePos] = selectorByte;
    }

    unsigned char GetInterfaceSelectorByte() const { return addressBytes[interfaceSelectorBytePos]; }

    void SetToAMulticastAddress(const unsigned int multicastGroupNumber);

    unsigned int GetMulticastGroupNumber() const;

    std::array<unsigned char, numberMacAddressBytes> addressBytes;

private:
    static const unsigned int addressTypeCodeBytePos = 0;
    static const unsigned int interfaceSelectorBytePos = 1;

    static const unsigned char unicastIndicatorByteCode = 0;
    static const unsigned char multicastIndicatorByteCode = 1;
    static const unsigned char broadcastIndicatorByteCode = 0xFF;

};//SixByteMacAddress//



inline
SixByteMacAddress::SixByteMacAddress(const GenericMacAddress& address)
{
    GenericMacAddress current = address;
    for(int i = (numberMacAddressBytes-1); (i >= 0); i--) {
        addressBytes[i] = static_cast<unsigned char>(current % 256);
        current = current / 256;
    }//for//
}

inline
GenericMacAddress SixByteMacAddress::ConvertToGenericMacAddress() const
{
    assert(sizeof(GenericMacAddress) >= numberMacAddressBytes);

    GenericMacAddress retValue = 0;
    for(unsigned int i = 0; (i < numberMacAddressBytes); i++) {
        retValue = (retValue * 256) + addressBytes[i];
    }//for//

    return retValue;
}


inline
bool SixByteMacAddress::operator==(const SixByteMacAddress& right) const
{
    for(unsigned int i = 0; (i < numberMacAddressBytes); i++) {
        if (addressBytes[i] != right.addressBytes[i]) {
            return false;
        }//if//
    }//for//
    return true;
}


inline
bool SixByteMacAddress::operator<(const SixByteMacAddress& right) const
{
    // Embedded node ID order.  Then interface index and finally broadcast byte.

    for(unsigned int i = (interfaceSelectorBytePos+1); (i < numberMacAddressBytes); i++) {
        if (addressBytes[i] < right.addressBytes[i]) {
            return true;
        }
        else if (addressBytes[i] > right.addressBytes[i]) {
            return false;
        }//if//
    }//for//

    if (addressBytes[interfaceSelectorBytePos] < right.addressBytes[interfaceSelectorBytePos]) {
        return true;
    }
    else if (addressBytes[interfaceSelectorBytePos] > right.addressBytes[interfaceSelectorBytePos]) {
        return false;
    }//if//

    return (addressBytes[addressTypeCodeBytePos] < right.addressBytes[addressTypeCodeBytePos]);

}//<//


inline
void SixByteMacAddress::SetFromString(
    const string& addressString,
    bool& success)
{
    const unsigned int MacAddressStringSize = numberMacAddressBytes * 3 -1;

    success = false;

    if (addressString.size() != MacAddressStringSize) {
        return;
    };//if//

    for(unsigned int i = 0; (i < numberMacAddressBytes); i++) {
        const char digit1 = addressString[i*3];
        const char digit2 = addressString[i*3+1];
        char separator = ':';
        if (i < (numberMacAddressBytes -1)) {
            separator = addressString[i*3+2];
        }//if//

        if ((!isxdigit(digit1)) || (!isxdigit(digit2)) || (separator != ':')) {
            return;
        }//if//

        (*this).addressBytes[i] = ConvertTwoHexCharactersToByte(digit1, digit2);
    }//for//

    success = true;

}//SetFromString//


// Extra-simulation hack to encode Node Id into lower bits of mac-address.

inline
void SixByteMacAddress::SetLowerBitsWithNodeId(const NodeId theNodeId)
{
    addressBytes[2] = static_cast<unsigned char>((theNodeId / (256*256*256)));
    addressBytes[3] = static_cast<unsigned char>(((theNodeId / (256*256)) % 256));
    addressBytes[4] = static_cast<unsigned char>(((theNodeId / 256) % 256));
    addressBytes[5] = static_cast<unsigned char>((theNodeId % 256));
}


inline
NodeId SixByteMacAddress::ExtractNodeId() const
{
    assert((2+sizeof(NodeId)) == numberMacAddressBytes);

    assert(!IsABroadcastOrAMulticastAddress());

    NodeId retValue = 0;
    for(unsigned int i = 0; (i < sizeof(NodeId)); i++) {
        retValue = (retValue * 256) + addressBytes[2+i];
    }//for//

    return retValue;
}

inline
void SixByteMacAddress::SetToAMulticastAddress(const unsigned int multicastGroupNumber)
{
    assert(multicastGroupNumber <= MaxMulticastGroupNumber);
    std::fill(addressBytes.begin(), addressBytes.end(), 0);
    addressBytes[addressTypeCodeBytePos] = multicastIndicatorByteCode;
    addressBytes[3] = static_cast<unsigned char>(multicastGroupNumber/(256*256));
    addressBytes[4] = static_cast<unsigned char>((multicastGroupNumber/256) % 256);
    addressBytes[5] = static_cast<unsigned char>(multicastGroupNumber % 256);

}//SetToAMulticastAddress//


inline
unsigned int SixByteMacAddress::GetMulticastGroupNumber() const
{
    assert(IsAMulticastAddress());
    return ((addressBytes[3] * (256*256)) + (addressBytes[4] * 256) + addressBytes[5]);

}//GetMulticastGroupNumber//



inline
SixByteMacAddress SixByteMacAddress::GetBroadcastAddress()
{
    const unsigned char fillVal = broadcastIndicatorByteCode;  //MS extension workaround.

    SixByteMacAddress ret;
    std::fill(ret.addressBytes.begin(), ret.addressBytes.end(), fillVal);
    return ret;
}


//--------------------------------------------------------------------------------------------------

class MacLayerInterfaceForEmulation {
public:
    virtual ~MacLayerInterfaceForEmulation() { }

    virtual void SetSimpleLinkMode(const NodeId& otherNodeId) = 0;
    virtual SimulationEngineInterface& GetSimulationEngineInterface() = 0;

    virtual void QueueOutgoingEthernetPacket(
        unique_ptr<Packet>& ethernetPacketPtr,
        const bool isLink = false) = 0;

    virtual void QueueOutgoingNonEthernetPacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority) = 0;

    class AbstractEthernetPacketDiverter {
    public:
        virtual void ProcessIncomingPacket(unique_ptr<Packet>& ethernetPacketPtr) = 0;
    };//AbstractPacketDiverter//

    virtual void RegisterPacketDiverter(
        const shared_ptr<AbstractEthernetPacketDiverter>& packetDiverterPtr,
        const bool isLink) = 0;

    virtual void SetPacketDiverterEthernetDestinationMacAddress(
        const SixByteMacAddress& macAddress) = 0;

    class AbstractEmulatorInterfaceForMacLayer {
    public:
        virtual void SendScanRequests(
            const NodeId& theNodeId,
            const unsigned int channelNum,
            const ObjectMobilityPosition& nodePosition,
            const SimTime& scanDurationTime,
            unsigned int& numberRequestsSent) = 0;

        virtual void MoveNodeToAnotherPartition(
            const NodeId& theNodeId,
            const unsigned int emulationPartitionIndex,
            const unsigned int channelNum,
            const SixByteMacAddress& newApAddress) = 0;

    };//AbstractEmulatorInterfaceForMacLayer//


    virtual void RegisterEmulatorInterface(
        const shared_ptr<AbstractEmulatorInterfaceForMacLayer>& interfacePtr) = 0;

    virtual void ProcessRemoteChannelScanResults(unique_ptr<Packet>& messagePtr) = 0;

    virtual void DisconnectInterface() = 0;

};//MacLayerInterfaceForEmulation//


//--------------------------------------------------------------------------------------------------

class MacAndPhyInfoInterface {
public:
    virtual ~MacAndPhyInfoInterface() { }

    virtual double GetRssiOfLastFrameDbm() const = 0;
    virtual double GetSinrOfLastFrameDb() const = 0;

    virtual SimTime GetTotalIdleChannelTime() const = 0;
    virtual SimTime GetTotalBusyChannelTime() const = 0;
    virtual SimTime GetTotalTransmissionTime() const = 0;

    virtual unsigned int GetNumberOfReceivedFrames() const = 0;
    virtual unsigned int GetNumberOfFramesWithErrors() const = 0;
    virtual unsigned int GetNumberOfSignalCaptures() const = 0;

};//MacAndPhyInfoInterface//


//--------------------------------------------------------------------------------------------------
// Mac Layer
//--------------------------------------------------------------------------------------------------


class MacLayer {
public:
    virtual ~MacLayer() { }
    // Network Layer Interface:
    virtual void NetworkLayerQueueChangeNotification() = 0;
    virtual void DisconnectFromOtherLayers() = 0;
    virtual GenericMacAddress GetGenericMacAddress() const
        { assert(false); abort(); return GenericMacAddress(); }

    // Mac QoS control.

    virtual shared_ptr<MacQualityOfServiceControlInterface> GetQualityOfServiceInterface() const
        { return (shared_ptr<MacQualityOfServiceControlInterface>(/*nullptr*/)); }

    // "Headless" (no upper layers) Emulation mode direct ethernet Mac layer interface.

    virtual shared_ptr<MacLayerInterfaceForEmulation> GetMacLayerInterfaceForEmulation()
    {
        return (shared_ptr<MacLayerInterfaceForEmulation>(/*nullptr*/));
    }

    // Information interface for Mac and Phy layer

    virtual shared_ptr<MacAndPhyInfoInterface> GetMacAndPhyInfoInterface()
        { assert(false); abort(); return (shared_ptr<MacAndPhyInfoInterface>(/*nullptr*/)); }

};//MacLayer//

class SimpleMacPacketHandler {
public:
    virtual ~SimpleMacPacketHandler() {}
    virtual void ReceivePacketFromMac(
        unique_ptr<Packet>& packetPtr,
        const GenericMacAddress& transmitterAddress) = 0;
};


//==================================================================================================

class NetworkLayer;
class AbstractNetwork;

class AbstractNetworkMac: public MacLayer {
public:
    static const string modelName;

    AbstractNetworkMac(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<AbstractNetwork>& abstractNetworkPtr,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const unsigned int macInterfaceIndex,
        const RandomNumberGeneratorSeed& nodeSeed);

    virtual void NetworkLayerQueueChangeNotification() override {
        if (!isBusy) {
            (*this).SendAPacket();
        }//if//
    }

    virtual void DisconnectFromOtherLayers() override {
        abstractNetworkPtr.reset();
        networkLayerPtr.reset();
    }

    virtual GenericMacAddress GetGenericMacAddress() const override {
        const SixByteMacAddress macAddress(theNodeId, static_cast<unsigned char>(interfaceIndex));
        return (macAddress.ConvertToGenericMacAddress());
    }

private:
    NodeId GetNodeId() { return theNodeId; }

    //-----------------------------------------------------

    class PacketArrivalEvent: public SimulationEvent {
    public:
        PacketArrivalEvent(
            AbstractNetworkMac* initDestinationMac,
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initLastHopAddress,
            const EtherTypeField& initEtherType)
            :
            destinationMac(initDestinationMac),
            packetPtr(move(initPacketPtr)),
            lastHopAddress(initLastHopAddress),
            etherType(initEtherType)
        {
        }

        virtual void ExecuteEvent() { destinationMac->ReceivePacket(packetPtr, lastHopAddress, etherType); }
    private:
        AbstractNetworkMac* destinationMac;
        unique_ptr<Packet> packetPtr;
        NetworkAddress lastHopAddress;
        EtherTypeField etherType;

    };//PacketArrivalEvent//

    //-----------------------------------------------------

    class PacketSendFinishedEvent: public SimulationEvent {
    public:
        PacketSendFinishedEvent(AbstractNetworkMac* initMacPtr) : macPtr(initMacPtr) { }

        virtual void ExecuteEvent() { macPtr->PacketHasBeenSentEvent(); }
    private:
        AbstractNetworkMac* macPtr;

    };//PacketSentEvent//


    //-----------------------------------------------------

    shared_ptr<AbstractNetwork> abstractNetworkPtr;

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    NodeId theNodeId;

    shared_ptr<NetworkLayer> networkLayerPtr;
    unsigned int interfaceIndex;
    InterfaceId theInterfaceId;

    shared_ptr<InterfaceOutputQueue> outputQueuePtr;

    bool isBusy;

    unsigned int numberPacketsSent;

    std::set<unsigned int> packetsToLoseSet;

    //-----------------------------------------------------

    void SendAPacket();

    void ReceivePacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& lastHopAddress,
        const EtherTypeField& etherType);

    void PacketHasBeenSentEvent() {
        isBusy = false;
        SendAPacket();
        //cout << "sendapacket" << endl;//呼び出しなし
    }

    SimTime CalcBandwidthLatency(const unsigned int packetBytes) const
        { return (SimTime((packetBytes * SECOND ) / bandwidthBytesPerSecond)); }

    RandomNumberGenerator aRandomNumberGenerator;

    bool packetDropByRateEnabled;

    double packetDropRate;

    SimTime minimumLatency;
    SimTime maximumLatency;
    double bandwidthBytesPerSecond;

    // Statistics:
    shared_ptr<CounterStatistic> packetsSentStatPtr;
    shared_ptr<CounterStatistic> packetsDroppedStatPtr;
    shared_ptr<CounterStatistic> packetsReceivedStatPtr;

    void OutputTraceAndStatsForFrameSend(const Packet& aPacket) const;
    void OutputTraceAndStatsForFrameDrop(const Packet& aPacket) const;
    void OutputTraceAndStatsForFrameReceive(const Packet& aPacket) const;

    void OutputTraceForFrame(const Packet& aPacket, const string& eventName) const;
};//AbstractNetworkMac//



inline
void AbstractNetworkMac::OutputTraceForFrame(const Packet& aPacket, const string& eventName) const
{

    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            AbstractMacPacketTraceRecord traceData;

            const PacketId& thePacketId = aPacket.GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();
            traceData.packetLengthBytes = static_cast<uint16_t>(aPacket.LengthBytes());

            assert(sizeof(traceData) == ABSTRACT_MAC_PACKET_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, eventName, traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << aPacket.GetPacketId();
            msgStream << " FrameBytes= " << aPacket.LengthBytes();
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, eventName, msgStream.str());
        }//if//
    }//if//

}//OutputTraceForFrame//

inline
void AbstractNetworkMac::OutputTraceAndStatsForFrameSend(const Packet& aPacket) const
{

    OutputTraceForFrame(aPacket, "Send");

    (*this).packetsSentStatPtr->IncrementCounter();

}//OutputTraceAndStatsForFrameSend//


inline
void AbstractNetworkMac::OutputTraceAndStatsForFrameDrop(const Packet& aPacket) const
{

    OutputTraceForFrame(aPacket, "Drop");

    (*this).packetsDroppedStatPtr->IncrementCounter();

}//OutputTraceAndStatsForFrameDrop//


inline
void AbstractNetworkMac::OutputTraceAndStatsForFrameReceive(const Packet& aPacket) const
{

    OutputTraceForFrame(aPacket, "Recv");

    (*this).packetsReceivedStatPtr->IncrementCounter();

}//OutputTraceAndStatsForFrameReceive//



}//namespace//


#endif
