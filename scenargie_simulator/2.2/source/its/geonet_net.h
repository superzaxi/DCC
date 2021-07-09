// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef GEONET_NET_H
#define GEONET_NET_H

#include "scensim_netsim.h"

#include "its_queues.h"
#include "geonet_mac.h"
#include "geonet_tracedefs.h"

namespace GeoNet {

using std::ostringstream;
using std::enable_shared_from_this;
using std::unique_ptr;
using std::move;

using ScenSim::SixByteMacAddress;
using ScenSim::NetworkAddress;
using ScenSim::CounterStatistic;
using ScenSim::ObjectMobilityModel;
using ScenSim::GenericMacAddress;
using ScenSim::TraceNetwork;
using ScenSim::TraceTransport;
using ScenSim::ENQUEUE_SUCCESS;
using ScenSim::ENQUEUE_FAILURE_BY_MAX_PACKETS;
using ScenSim::ENQUEUE_FAILURE_BY_MAX_BYTES;

class GeoNetworkingProtocol;

typedef uint8_t TransportType;
enum  {
    GEO_UNICAST = 0,
    TSB = 1,
    SHB = 2,
    GEO_BROADCAST = 3,
    GEO_ANYCAST = 4
};

enum NextHeaderType {
    NH_ANY = 0,
    NH_BTP_A = 1,
    NH_BTP_B = 2,
    NH_IPV6 = 3
};

//8bytes
struct GeoNetworkingAddressType {
    GeoNetworkingAddressType() {}//TBD

    unsigned char manuallyConfigured:1;
    unsigned char stationType:4;
    unsigned char stationSubType:1;
    unsigned char remainingCountryCode_notUsed:2;
    unsigned char countryCode;
    SixByteMacAddress macAddress;

};


//28bytes
struct LongPositionVectorType {
    LongPositionVectorType() {}//TBD

    GeoNetworkingAddressType geoNetAddress;
    unsigned int time_notused;
    int latitude_notused;
    int longitude_notused;
    short int speed_notused;
    unsigned short int heading_notused;
    short int altitude_notused;
    unsigned char timeAccuracy_notused:4;
    unsigned char positionAccuracy_notused:4;
    unsigned char speedAccuracy_notused:3;
    unsigned char headingAccuracy_notused:3;
    unsigned char altitudeAccuracy_notused:2;

};




//when BTP-B, soucePort is not used.
//4bytes
struct BtpHeaderType {
    BtpHeaderType(
        unsigned short int initDestinationPort,
        unsigned short int initSourcePort)
        :
        destinationPort(initDestinationPort),
        sourcePort(initSourcePort)
    {}
    unsigned short int destinationPort;
    unsigned short int sourcePort;


};



// Basic Tranport Protocol(BTP)
// Reference: ETSI TS 102 636-5-1
class BasicTransportProtocol: public enable_shared_from_this<BasicTransportProtocol> {
public:
    static const string modelName;


    BasicTransportProtocol(const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr);

    void DisconnectFromOtherLayers();

    void ConnectToNetworkLayer(const shared_ptr<GeoNetworkingProtocol>& initGeoNetworkingPtr);


    //-----------------------------------------------------
    // BTP Interface for Applications

    void SendPacket(
        unique_ptr<Packet>& packetPtr,
        const unsigned short int sourcePort,
        const NetworkAddress& destinationAddress,
        const unsigned short int destinationPort,
        const PacketPriority& priority,
        const TransportType& transportType,
        const SimTime& maxPacketLifetime,
        const SimTime& repetitionInterval);

    bool PortIsAvailable(const unsigned short int portNumber) const { return (portMap.find(portNumber) == portMap.end()); }

    //---------------------------------

    class PacketForAppFromTransportLayerHandler {
    public:
        virtual void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const unsigned short int sourcePort,
            const unsigned short int destinationPort,
            const TransportType& transportType,
            const LongPositionVectorType& sourceLocationVector,
            const PacketPriority& priority,
            const unsigned char hopLimit) = 0;

        virtual ~PacketForAppFromTransportLayerHandler() { }

    };//PacketForAppFromTransportLayerHandler//


    void OpenSpecificBtpPort(
        const NetworkAddress& address,
        const unsigned short int portNumber,
        const shared_ptr<PacketForAppFromTransportLayerHandler>& packetHandlerPtr);

    //---------------------------------

    void ReceivePacketFromNetworkLayer(
        unique_ptr<Packet>& packetPtr,
        const NextHeaderType& nextHeaderType,
        const TransportType& transportType,
        const LongPositionVectorType& sourcePositionVector,
        const PacketPriority& trafficClass,
        const unsigned char hopLimit);

private:
    shared_ptr<GeoNetworkingProtocol> geoNetworkingPtr;

    struct PortInfoType {
        PortInfoType() { }
        PortInfoType(
            const NetworkAddress& initPortAddress,
            const shared_ptr<PacketForAppFromTransportLayerHandler>& initPacketHandlerPtr)
            :  packetHandlerPtr(initPacketHandlerPtr), portAddress(initPortAddress) {}

        shared_ptr<PacketForAppFromTransportLayerHandler> packetHandlerPtr;
        NetworkAddress portAddress;
    };

    // Note: Design disallows using the same port numbers for different apps
    // via using different interface addresses.

    map<unsigned short int, PortInfoType> portMap;

    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;

    // Statistics
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;

    void OutputTraceAndStatsForSendPacketToNetworkLayer(const Packet& packet) const;
    void OutputTraceAndStatsForReceivePacketFromNetworkLayer(const Packet& packet) const;

    // Disable:
    BasicTransportProtocol(const BasicTransportProtocol&);
    void operator=(const BasicTransportProtocol&);

};//BasicTransportProtocol//

inline
BasicTransportProtocol::BasicTransportProtocol(const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(modelName + "_BytesReceived")),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(modelName + "_BytesSent"))
{

}

inline
void BasicTransportProtocol::DisconnectFromOtherLayers()
{
    (*this).geoNetworkingPtr.reset();
    (*this).portMap.clear();
}


inline
void BasicTransportProtocol::OpenSpecificBtpPort(
    const NetworkAddress& portAddress,
    const unsigned short int portNumber,
    const shared_ptr<PacketForAppFromTransportLayerHandler>& packetHandlerPtr)
{
    assert(portMap.find(portNumber) == portMap.end());

    portMap[portNumber] = PortInfoType(portAddress, packetHandlerPtr);

}//OpenSpecificBtpPort//





// GeoNetworking Protocol
// Reference: ETSI TS 102 636-4-1
//Note: Current implementation GetNetworkingProtocol per interface

class GeoNetworkingProtocol: public enable_shared_from_this<GeoNetworkingProtocol> {
public:
    static const string modelName;



    enum HeaderType {
        HT_ANY = 0,
        HT_BEACON = 1,
        HT_GEOUNICAST = 2,
        HT_GEOANYCAST = 3,
        HT_GEOBROADCAST = 4,
        HT_TSB = 5,
        HT_LS = 6
    };

    enum HeaderSubType {
        HST_SINGLE_HOP = 0,
        HST_MULTI_HOP = 1
    };

    //36bytes
    struct CommonHeaderType {
        unsigned char version:4;
        unsigned char nextHeader:4;
        unsigned char headerType:4;
        unsigned char headerSubType:4;
        unsigned char reserved;
        unsigned char flags;
        unsigned short int networkHeaderPayload_notused;
        unsigned char trafficClass;
        unsigned char hopLimit;
        LongPositionVectorType senderPositionVector;

        CommonHeaderType(){}

        CommonHeaderType(
            const NextHeaderType& initNextHeader,
            const HeaderType& initHeaderType,
            const HeaderSubType& initHeaderSubType,
            const PacketPriority& initTrafficClass,
            const unsigned char initHopLimit)
            :
            version(0),
            nextHeader(static_cast<unsigned char>(initNextHeader)),
            headerType(static_cast<unsigned char>(initHeaderType)),
            headerSubType(static_cast<unsigned char>(initHeaderSubType)),
            trafficClass(initTrafficClass),
            hopLimit(initHopLimit)
        {}

    };


    GeoNetworkingProtocol(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeId& initNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
        const shared_ptr<GeoNetMac>& initGeoNetMacPtr,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr);

    ~GeoNetworkingProtocol() { }

    void DisconnectFromOtherLayers()
    {
        (*this).btpPtr.reset();
        (*this).networkLayerPtr.reset();
    }

    void RegisterBasicTranportProtocol(
        const shared_ptr<BasicTransportProtocol>& initBtpPtr)
    { (*this).btpPtr = initBtpPtr; }

    void ReceivePacketFromUpperLayer(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& destinationAddress,
        const PacketPriority& trafficClass,
        const TransportType& transportType,
        const SimTime& maxPacketLifetime,
        const SimTime& repetitionInterval);

    void ReceivePacketFromMac(
        unique_ptr<Packet>& packetPtr,
        const GenericMacAddress& peerMacAddress);


private:
    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;

    shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr;

    NodeId theNodeId;

    shared_ptr<BasicTransportProtocol> btpPtr;
    shared_ptr<GeoNetMac> geoNetMacPtr;

    shared_ptr<NetworkLayer> networkLayerPtr;//for IP

    //---------------------------------

    class GeoNetPacketHandler : public SimpleMacPacketHandler {
    public:
        GeoNetPacketHandler(GeoNetworkingProtocol* initGeoNetPtr)
            : geoNetPtr(initGeoNetPtr) {}
        void ReceivePacketFromMac(
            unique_ptr<Packet>& packetPtr,
            const GenericMacAddress& transmitterAddress) {
            geoNetPtr->ReceivePacketFromMac(packetPtr, transmitterAddress);
        }
    private:
        GeoNetworkingProtocol* geoNetPtr;
    };//GeoNetPacketHandler//

    //---------------------------------
    class PeriodicallyInsertPacketIntoQueueEvent : public SimulationEvent {
    public:
        PeriodicallyInsertPacketIntoQueueEvent(
            GeoNetworkingProtocol* initGeoNetPtr,
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress,
            const PacketPriority& initTrafficClass,
            const SimTime& initRemainingPacketLifetime,
            const SimTime& initRepetitionInterval)
            :
            geoNetPtr(initGeoNetPtr),
            packetPtr(initPacketPtr),
            nextHopAddress(initNextHopAddress),
            trafficClass(initTrafficClass),
            remainingPacketLifetime(initRemainingPacketLifetime),
            repetitionInterval(initRepetitionInterval)
            {}

        void ExecuteEvent()
        {
            geoNetPtr->InsertPacketIntoTransmitQueueAndScheduleEventIfNecessary(
                packetPtr, nextHopAddress, trafficClass, remainingPacketLifetime, repetitionInterval);
        }

    private:
        GeoNetworkingProtocol* geoNetPtr;
        unique_ptr<Packet>& packetPtr;
        NetworkAddress nextHopAddress;
        PacketPriority trafficClass;
        SimTime remainingPacketLifetime;
        SimTime repetitionInterval;
    };

    //---------------------------------


    void InsertPacketIntoTransmitQueue(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority trafficClass);

    void InsertPacketIntoTransmitQueueAndScheduleEventIfNecessary(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority trafficClass,
        const SimTime& remainingPacketLifetime,
        const SimTime& repetitionInterval);

    void SendSHBPacket(
        unique_ptr<Packet>& packetPtr,
        const PacketPriority& trafficClass,
        const SimTime& maxPacketLifetime,
        const SimTime& repetitionInterval);

    void ProcessSHBPacket(
        const CommonHeaderType& commonHeader,
        unique_ptr<Packet>& packetPtr);

    // Statistics:

    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<CounterStatistic> packetsSentStatPtr;

    shared_ptr<CounterStatistic> bytesReceivedStatPtr;
    shared_ptr<CounterStatistic> packetsReceivedStatPtr;
    shared_ptr<CounterStatistic> packetMaxPacketsQueueDropsStatPtr;
    shared_ptr<CounterStatistic> packetMaxBytesQueueDropsStatPtr;

    void OutputTraceAndStatsForInsertPacketIntoQueue(const Packet& packet) const;
    void OutputTraceAndStatsForFullQueueDrop(
        const Packet& packet, const EnqueueResultType enqueueResult) const;
    void OutputTraceAndStatsForReceivePacketFromMac(const Packet& packet) const;


    // Disable:

    GeoNetworkingProtocol(const GeoNetworkingProtocol&);
    void operator=(const GeoNetworkingProtocol&);

};//GeoNetworkingProtocol//




inline
void BasicTransportProtocol::ConnectToNetworkLayer(const shared_ptr<GeoNetworkingProtocol>& initGeoNetworkingPtr)
{
    this->geoNetworkingPtr = initGeoNetworkingPtr;

    geoNetworkingPtr->RegisterBasicTranportProtocol(shared_from_this());

}//ConnectToNetworkLayer//




inline
void BasicTransportProtocol::SendPacket(
    unique_ptr<Packet>& packetPtr,
    const unsigned short int sourcePort,
    const NetworkAddress& destinationAddress,
    const unsigned short int destinationPort,
    const PacketPriority& priority,
    const TransportType& transportType,
    const SimTime& maxPacketLifetime,
    const SimTime& repetitionInterval)
{
    packetPtr->AddPlainStructHeader(
        BtpHeaderType(destinationPort, sourcePort));

    OutputTraceAndStatsForSendPacketToNetworkLayer(*packetPtr);

    geoNetworkingPtr->ReceivePacketFromUpperLayer(
        packetPtr,
        destinationAddress,
        priority,
        transportType,
        maxPacketLifetime,
        repetitionInterval);

}//SendPacket//


inline
void BasicTransportProtocol::ReceivePacketFromNetworkLayer(
    unique_ptr<Packet>& packetPtr,
    const NextHeaderType& nextHeaderType,
    const TransportType& transportType,
    const LongPositionVectorType& sourcePositionVector,
    const PacketPriority& trafficClass,
    const unsigned char hopLimit)
{

    assert((nextHeaderType == NH_BTP_A) || (nextHeaderType == NH_BTP_B));

    OutputTraceAndStatsForReceivePacketFromNetworkLayer(*packetPtr);

    BtpHeaderType btpHeader = packetPtr->GetAndReinterpretPayloadData<BtpHeaderType>();

    packetPtr->DeleteHeader(sizeof(BtpHeaderType));

    map<unsigned short int, PortInfoType>::iterator mapIter = portMap.find(btpHeader.destinationPort);

    if (mapIter != portMap.end()) {
        // Deliver packet to application.

        mapIter->second.packetHandlerPtr->ReceivePacket(
            packetPtr, btpHeader.sourcePort, btpHeader.destinationPort,
            transportType, sourcePositionVector, trafficClass, hopLimit);
    }
    else {
        packetPtr = nullptr;
    }//if//

}//ReceivePacketFromNetworkLayer//


inline
void BasicTransportProtocol::OutputTraceAndStatsForSendPacketToNetworkLayer(const ScenSim::Packet &packet) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceTransport)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            BtpSendTraceRecord traceData;
            const PacketId& thePacketId = packet.GetPacketId();

            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == BTP_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "BtpSend", traceData);
        }
        else {
            ostringstream outStream;
            const PacketId& thePacketId = packet.GetPacketId();
            outStream << "PktId= " << thePacketId;
            simulationEngineInterfacePtr->OutputTrace(modelName, "", "BtpSend", outStream.str());

        }//if//
    }//if//

    const unsigned int packetLengthBytes = packet.LengthBytes();
    bytesSentStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForSendPacketToNetworkLayer//


inline
void BasicTransportProtocol::OutputTraceAndStatsForReceivePacketFromNetworkLayer(const ScenSim::Packet &packet) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceTransport)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            BtpReceiveTraceRecord traceData;
            const PacketId& thePacketId = packet.GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == BTP_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "BtpRecv", traceData);

        }
        else {

            ostringstream outStream;
            const PacketId& thePacketId = packet.GetPacketId();
            outStream << "PktId= " << thePacketId;
            simulationEngineInterfacePtr->OutputTrace(modelName, "", "BtpRecv", outStream.str());

        }//if//
    }//if//

    const unsigned int packetLengthBytes = packet.LengthBytes();
    bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForReceivePacketFromNetworkLayer//



inline
GeoNetworkingProtocol::GeoNetworkingProtocol(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeId& initNodeId,
    const InterfaceId& theInterfaceId,
    const unsigned int interfaceIndex,
    const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
    const shared_ptr<GeoNetMac>& initGeoNetMacPtr,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    theNodeId(initNodeId),
    nodeMobilityModelPtr(initNodeMobilityModelPtr),
    geoNetMacPtr(initGeoNetMacPtr),
    networkLayerPtr(initNetworkLayerPtr),
    bytesSentStatPtr(
        initSimulationEngineInterfacePtr->CreateCounterStat(modelName + "_BytesSent")),
    packetsSentStatPtr(
        initSimulationEngineInterfacePtr->CreateCounterStat(modelName + "_PacketsSent")),
    bytesReceivedStatPtr(
        initSimulationEngineInterfacePtr->CreateCounterStat(modelName + "_BytesReceived")),
    packetsReceivedStatPtr(
        initSimulationEngineInterfacePtr->CreateCounterStat(modelName + "_PacketsReceived")),
    packetMaxPacketsQueueDropsStatPtr(
        initSimulationEngineInterfacePtr->CreateCounterStat(modelName + "_MaxPacketsQueueDrops")),
    packetMaxBytesQueueDropsStatPtr(
        initSimulationEngineInterfacePtr->CreateCounterStat(modelName + "_MaxBytesQueueDrops"))
{

    //set mac->geonet interface
    initGeoNetMacPtr->SetGeoNetPacketHandler(
        shared_ptr<GeoNetPacketHandler>(new GeoNetPacketHandler(this)));

}


inline
void GeoNetworkingProtocol::InsertPacketIntoTransmitQueue(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority trafficClass)
{

    OutputTraceAndStatsForInsertPacketIntoQueue(*packetPtr);

    EnqueueResultType enqueueResult;
    unique_ptr<Packet> packetToDropPtr;

    geoNetMacPtr->InsertPacketIntoAnOutputQueue(
        packetPtr,
        nextHopAddress,
        trafficClass,
        ETHERTYPE_GEONET,
        enqueueResult,
        packetToDropPtr);

    if (enqueueResult != ENQUEUE_SUCCESS) {

        OutputTraceAndStatsForFullQueueDrop(*packetToDropPtr, enqueueResult);

        packetToDropPtr = nullptr;
        packetPtr = nullptr;

    }//if//

    //macLayerPtr->NetworkLayerQueueChangeNotification();

}//InsertPacketIntoAnOutputQueue//


inline
void GeoNetworkingProtocol::InsertPacketIntoTransmitQueueAndScheduleEventIfNecessary(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority trafficClass,
    const SimTime& remainingPacketLifetime,
    const SimTime& repetitionInterval)
{

    (*this).InsertPacketIntoTransmitQueue(packetPtr, nextHopAddress, trafficClass);

    if (remainingPacketLifetime >= repetitionInterval) {

        //TBD: copy packet
        unique_ptr<Packet> copyOfPacketPtr(new Packet(*packetPtr));

        const SimTime nextEnqueueTime =
            simulationEngineInterfacePtr->CurrentTime() + repetitionInterval;

        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new PeriodicallyInsertPacketIntoQueueEvent(
                this, copyOfPacketPtr, nextHopAddress, trafficClass,
                (remainingPacketLifetime - repetitionInterval), repetitionInterval)),
            nextEnqueueTime);

    }//if//

}//InsertPacketIntoTransmitQueueIfNecessary//


inline
void GeoNetworkingProtocol::ReceivePacketFromUpperLayer(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& destinationAddress,
    const PacketPriority& trafficClass,
    const TransportType& transportType,
    const SimTime& maxPacketLifetime,
    const SimTime& repetitionInterval)
{

    switch (transportType) {
    case SHB:
        assert(destinationAddress == NetworkAddress::broadcastAddress);
        (*this).SendSHBPacket(packetPtr, trafficClass, maxPacketLifetime, repetitionInterval);
        break;
    case GEO_UNICAST:
    case TSB:
    case GEO_BROADCAST:
    case GEO_ANYCAST:
    default:
        assert(false && "Unknown Transport Type.");
        exit(1);
        break;
    }//switch//

}//ReceivePacketFromUpperLayer//


inline
void GeoNetworkingProtocol::ReceivePacketFromMac(
    unique_ptr<Packet>& packetPtr,
    const GenericMacAddress& peerMacAddress)
{

    assert(packetPtr->LengthBytes() > 0);

    OutputTraceAndStatsForReceivePacketFromMac(*packetPtr);

    const CommonHeaderType commonHeader =
        packetPtr->GetAndReinterpretPayloadData<CommonHeaderType>();

    packetPtr->DeleteHeader(sizeof(CommonHeaderType));

    assert(commonHeader.version == 0);

    switch (commonHeader.headerType) {
    case HT_TSB:
        if (commonHeader.headerSubType == HST_SINGLE_HOP) {
            ProcessSHBPacket(commonHeader, packetPtr);
        }
        else {
            assert(false && "Unknown Header Sub Type.");
            exit(1);
        }
        break;
    case HT_ANY:
    case HT_BEACON:
    case HT_GEOUNICAST:
    case HT_GEOBROADCAST:
    case HT_LS:
    default:
        assert(false && "Unknown Header Type.");
        exit(1);
        break;
    }//switch//

}//ReceivePacketFromMac//


inline
void GeoNetworkingProtocol::ProcessSHBPacket(
    const CommonHeaderType& commonHeader,
    unique_ptr<Packet>& packetPtr)
{

    assert((commonHeader.headerType == HT_TSB) &&
        (commonHeader.headerSubType == HST_SINGLE_HOP));

    (*this).btpPtr->ReceivePacketFromNetworkLayer(
        packetPtr,
        static_cast<NextHeaderType>(commonHeader.nextHeader),
        SHB,
        commonHeader.senderPositionVector,
        commonHeader.trafficClass,
        commonHeader.hopLimit);

}//ProcessSHBPacket//


inline
void GeoNetworkingProtocol::OutputTraceAndStatsForInsertPacketIntoQueue(
    const Packet& packet) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceNetwork)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            GeoNetPacketTraceRecord traceData;
            const PacketId& thePacketId = packet.GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == GEONET_PACKET_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "GeoNetSend", traceData);

        }
        else {

            ostringstream outStream;
            const PacketId& thePacketId = packet.GetPacketId();
            outStream << "PktId= " << thePacketId;
            simulationEngineInterfacePtr->OutputTrace(modelName, "", "GeoNetSend", outStream.str());

        }//if//
    }//if//

    packetsSentStatPtr->IncrementCounter();

    const size_t packetLengthBytes = packet.LengthBytes();
    bytesSentStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForInsertPacketIntoQueue//


inline
void GeoNetworkingProtocol::OutputTraceAndStatsForFullQueueDrop(
    const Packet& packet, const EnqueueResultType enqueueResult) const
{

    if (simulationEngineInterfacePtr->TraceIsOn(TraceNetwork)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            GeoNetFullQueueDropTraceRecord traceData;
            const PacketId& thePacketId = packet.GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();
            traceData.enqueueResult = ConvertToUShortInt(enqueueResult);

            assert(sizeof(traceData) == GEONET_FULL_QUEUE_DROP_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "FullQueueDrop", traceData);

        }
        else {

            ostringstream outStream;
            outStream << "PktId= " << packet.GetPacketId();
            outStream << " Result= " << ConvertToEnqueueResultString(enqueueResult);

            simulationEngineInterfacePtr->OutputTrace(modelName, "", "FullQueueDrop", outStream.str());

        }//if//

    }//if//

    if (enqueueResult == ENQUEUE_FAILURE_BY_MAX_PACKETS) {
        packetMaxPacketsQueueDropsStatPtr->IncrementCounter();
    }
    else if (enqueueResult == ENQUEUE_FAILURE_BY_MAX_BYTES) {
        packetMaxBytesQueueDropsStatPtr->IncrementCounter();
    }//if//

}//OutputTraceAndStatsForFullQueueDrop//


inline
void GeoNetworkingProtocol::OutputTraceAndStatsForReceivePacketFromMac(
    const Packet& packet) const
{

    if (simulationEngineInterfacePtr->TraceIsOn(TraceNetwork)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            GeoNetPacketTraceRecord traceData;
            const PacketId& thePacketId = packet.GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == GEONET_PACKET_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "GeoNetRecv", traceData);

        }
        else {

            ostringstream outStream;
            const PacketId& thePacketId = packet.GetPacketId();
            outStream << "PktId= " << thePacketId;
            simulationEngineInterfacePtr->OutputTrace(modelName, "", "GeoNetRecv", outStream.str());

        }//if//
    }//if//

    packetsReceivedStatPtr->IncrementCounter();

    const size_t packetLengthBytes = packet.LengthBytes();
    bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForReceivePacketFromMac//


}//namespace//

#endif
