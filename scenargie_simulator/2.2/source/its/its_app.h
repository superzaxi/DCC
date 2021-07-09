// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef ITS_APP_H
#define ITS_APP_H

#include "scensim_netsim.h"
#include "geonet_net.h"

namespace Its {

using std::ostringstream;
using std::cerr;
using std::endl;

using std::enable_shared_from_this;
using std::shared_ptr;
using std::unique_ptr;
using std::move;

using ScenSim::SimulationEngineInterface;
using ScenSim::SimulationEvent;
using ScenSim::Application;
using ScenSim::UdpProtocol;
using ScenSim::ParameterDatabaseReader;
using ScenSim::ApplicationId;
using ScenSim::NodeId;
using ScenSim::ANY_NODEID;
using ScenSim::SimTime;
using ScenSim::INFINITE_TIME;
using ScenSim::ZERO_TIME;
using ScenSim::Packet;
using ScenSim::TraceApplication;
using ScenSim::APPLICATION_SEND_TRACE_RECORD_BYTES;
using ScenSim::APPLICATION_RECEIVE_TRACE_RECORD_BYTES;
using ScenSim::ApplicationReceiveTraceRecord;
using ScenSim::ApplicationSendTraceRecord;
using ScenSim::InterfaceId;
using ScenSim::ConvertTimeToDoubleSecs;
using ScenSim::ConvertToString;
using ScenSim::ConvertTimeToStringSecs;
using ScenSim::NetworkAddress;
using ScenSim::PacketPriority;
using ScenSim::CounterStatistic;
using ScenSim::RealStatistic;
using ScenSim::PacketId;
using ScenSim::ConvertToUChar;

using GeoNet::BasicTransportProtocol;
using GeoNet::TransportType;
using GeoNet::LongPositionVectorType;

class ItsBroadcastApplication :
    public Application, public enable_shared_from_this<ItsBroadcastApplication> {
public:
    static const string modelName;


    enum TransportLayerType {
        UDP_MODE, BTP_MODE
    };


    ItsBroadcastApplication(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const NodeId& initNodeId,
        const InterfaceId& initInterfaceId,
        const TransportLayerType& tranportLayerType,
        const shared_ptr<BasicTransportProtocol>& initBasicTransportProtocolPtr = shared_ptr<BasicTransportProtocol>());


    void CompleteInitialization(const ParameterDatabaseReader& theParameterDatabaseReader);

    void DisconnectFromOtherLayers();

private:
    static const ApplicationId theApplicationId;
    static const unsigned short int destinationPortId;

    shared_ptr<BasicTransportProtocol> basicTransportProtocolPtr;

    bool initializationIsComplete;

    struct ItsBroadcastPayloadType {
        unsigned int sequenceNumber;
        SimTime sendTime;

        ItsBroadcastPayloadType(
            const unsigned int initSequenceNumber,
            const SimTime initSendTime)
            :
            sequenceNumber(initSequenceNumber),
            sendTime(initSendTime)
        {}
    };//ItsBroadcastPayloadType//


    //-------------------------------------------------------------------------

    class ItsBroadcastEvent: public SimulationEvent {
    public:
        explicit
        ItsBroadcastEvent(
            const shared_ptr<ItsBroadcastApplication>& initItsBroadcastApplicationPtr)
            :
            itsBroadcastApplicationPtr(initItsBroadcastApplicationPtr) {}

        virtual void ExecuteEvent()
        { itsBroadcastApplicationPtr->SendPacket(); }

    private:
        shared_ptr<ItsBroadcastApplication> itsBroadcastApplicationPtr;

    };//ItsBroadcastEvent//


    //interface from UDP
    class UdpPacketHandler: public UdpProtocol::PacketForAppFromTransportLayerHandler {
    public:
        UdpPacketHandler(
            const shared_ptr<ItsBroadcastApplication>& initItsBroadcastApplicationPtr)
            :
            itsBroadcastApplicationPtr(initItsBroadcastApplicationPtr)
        {}

        void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const NetworkAddress& sourceAddress,
            const unsigned short int sourcePort,
            const NetworkAddress& destinationAddress,
            const PacketPriority& priority)
        {
            itsBroadcastApplicationPtr->ReceivePacket(packetPtr);
        }//ReceivePacket//

    private:
        shared_ptr<ItsBroadcastApplication> itsBroadcastApplicationPtr;

    };//UdpPacketHandler//

    //interface from BTP
    class BtpPacketHandler: public BasicTransportProtocol::PacketForAppFromTransportLayerHandler {
    public:
        BtpPacketHandler(
            const shared_ptr<ItsBroadcastApplication>& initItsBroadcastApplicationPtr)
            :
            itsBroadcastApplicationPtr(initItsBroadcastApplicationPtr)
        {}

        void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const unsigned short int sourcePort,
            const unsigned short int destinationPort,
            const TransportType& transportType,
            const LongPositionVectorType& sourceLocationVector,
            const PacketPriority& priority,
            const unsigned char hopLimit)
        {
            itsBroadcastApplicationPtr->ReceivePacket(packetPtr);
        }//ReceivePacket//

    private:
        shared_ptr<ItsBroadcastApplication> itsBroadcastApplicationPtr;

    };//BtpPacketHandler//


    //-------------------------------------------------------------------------

    NodeId theNodeId;
    InterfaceId theInterfaceId;
    TransportLayerType transportLayerType;

    SimTime itsBroadcastStartTime;
    SimTime itsBroadcastEndTime;
    SimTime packetInterval;
    PacketPriority packetPriority;
    unsigned int packetPayloadSizeBytes;
    SimTime geoNetMaxPacketLifetime;
    SimTime geoNetRepetitionInterval;

    void SendPacket();

    void ReceivePacket(unique_ptr<Packet>& packetPtr);

    unsigned int currentPacketSequenceNumber;
    unsigned int numberPacketsReceived;

    shared_ptr<CounterStatistic> packetsSentStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;

    shared_ptr<CounterStatistic> packetsReceivedStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;
    shared_ptr<RealStatistic> endToEndDelayStatPtr;

    void OutputTraceAndStatsForSendPacket(
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const unsigned int packetLengthBytes);

    void OutputTraceForReceivePacket(
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const unsigned int packetLengthBytes,
        const SimTime& delay);

};//ItsBroadcastApplication//


inline
ItsBroadcastApplication::ItsBroadcastApplication(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const NodeId& initNodeId,
    const InterfaceId& initInterfaceId,
    const TransportLayerType& initTranportLayerType,
    const shared_ptr<BasicTransportProtocol>& initBasicTransportProtocolPtr)
    :
    Application(initSimEngineInterfacePtr, theApplicationId),
    theNodeId(initNodeId),
    theInterfaceId(initInterfaceId),
    transportLayerType(initTranportLayerType),
    basicTransportProtocolPtr(initBasicTransportProtocolPtr),
    initializationIsComplete(false),
    itsBroadcastStartTime(INFINITE_TIME),
    itsBroadcastEndTime(INFINITE_TIME),
    packetInterval(INFINITE_TIME),
    packetPriority(0),
    packetPayloadSizeBytes(0),
    geoNetMaxPacketLifetime(ZERO_TIME),
    geoNetRepetitionInterval(ZERO_TIME),
    currentPacketSequenceNumber(0),
    numberPacketsReceived(0),
    packetsSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(modelName + "_PacketsSent")),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(modelName + "_BytesSent")),
    packetsReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(modelName + "_PacketsReceived")),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(modelName  + "_BytesReceived")),
    endToEndDelayStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(modelName + "_EndToEndDelay"))
{
}


inline
void ItsBroadcastApplication::CompleteInitialization(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{


    SimTime jitter = ZERO_TIME;
    if (theParameterDatabaseReader.ParameterExists("its-broadcast-app-traffic-start-time-max-jitter", theNodeId, theInterfaceId)) {
        jitter = static_cast<SimTime>(
            theParameterDatabaseReader.ReadTime("its-broadcast-app-traffic-start-time-max-jitter", theNodeId, theInterfaceId) *
            (*aRandomNumberGeneratorPtr).GenerateRandomDouble());
    }//if//

    if (theParameterDatabaseReader.ParameterExists("its-broadcast-app-traffic-start-time", theNodeId, theInterfaceId)) {
        itsBroadcastStartTime =
            theParameterDatabaseReader.ReadTime("its-broadcast-app-traffic-start-time", theNodeId, theInterfaceId) + jitter;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("its-broadcast-app-traffic-end-time", theNodeId, theInterfaceId)) {
        itsBroadcastEndTime =
            theParameterDatabaseReader.ReadTime("its-broadcast-app-traffic-end-time", theNodeId, theInterfaceId) + jitter;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("its-broadcast-app-traffic-interval", theNodeId, theInterfaceId)) {
        packetInterval =
            theParameterDatabaseReader.ReadTime("its-broadcast-app-traffic-interval", theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("its-broadcast-app-packet-priority", theNodeId, theInterfaceId)) {
        packetPriority =
            ConvertToUChar(
                theParameterDatabaseReader.ReadNonNegativeInt(
                    "its-broadcast-app-packet-priority", theNodeId, theInterfaceId),
                    "Error: \"its-broadcast-app-packet priority\" parameter is out of range.");

        assert(sizeof(PacketPriority) == sizeof(unsigned char));

    }//if//

    if (theParameterDatabaseReader.ParameterExists("its-broadcast-app-packet-payload-size-bytes", theNodeId, theInterfaceId)) {
        packetPayloadSizeBytes =
            theParameterDatabaseReader.ReadNonNegativeInt("its-broadcast-app-packet-payload-size-bytes", theNodeId, theInterfaceId);

        if (packetPayloadSizeBytes < sizeof(ItsBroadcastPayloadType)) {
            cerr << "its-broadcast-app-packet-payload-size-bytes must be more than "
                 << "ItsBroadcastPayloadType: "
                 << sizeof(ItsBroadcastPayloadType) << endl;
            exit(1);
        }//if//
    }//if//

    if (theParameterDatabaseReader.ParameterExists("its-broadcast-app-geonet-max-packet-lifetime", theNodeId, theInterfaceId)) {

        geoNetMaxPacketLifetime =
            theParameterDatabaseReader.ReadTime("its-broadcast-app-geonet-max-packet-lifetime", theNodeId, theInterfaceId);

        geoNetRepetitionInterval =
            theParameterDatabaseReader.ReadTime("its-broadcast-app-geonet-repetition-interval", theNodeId, theInterfaceId);

    }//if//


    //open port
    switch (transportLayerType) {
    case UDP_MODE:
        assert(transportLayerPtr->udpPtr->PortIsAvailable(destinationPortId));
        transportLayerPtr->udpPtr->OpenSpecificUdpPort(
            NetworkAddress::anyAddress,
            destinationPortId,
            shared_ptr<UdpPacketHandler>(new UdpPacketHandler(shared_from_this())));
        break;
    case BTP_MODE:
        assert(basicTransportProtocolPtr->PortIsAvailable(destinationPortId));
        basicTransportProtocolPtr->OpenSpecificBtpPort(
            NetworkAddress::anyAddress,
            destinationPortId,
            shared_ptr<BtpPacketHandler>(new BtpPacketHandler(shared_from_this())));
        break;
    default:
        assert(false); abort(); break;
    }//switch//

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    SimTime nextTransmissionTime = itsBroadcastStartTime;

    if (currentTime > itsBroadcastStartTime) {
        const size_t numberPassedTransmissionTimes =
            size_t(std::ceil(double(currentTime - itsBroadcastStartTime) / packetInterval));

        nextTransmissionTime =
            itsBroadcastStartTime +
            numberPassedTransmissionTimes * packetInterval;
    }//if//

    if (nextTransmissionTime < itsBroadcastEndTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new ItsBroadcastEvent(shared_from_this())), nextTransmissionTime);
    }//if//

    initializationIsComplete = true;
}


inline
void ItsBroadcastApplication::DisconnectFromOtherLayers()
{
    (*this).basicTransportProtocolPtr.reset();
    (*this).Application::DisconnectFromOtherLayers();

}//DisconnectFromOtherLayers//


inline
void ItsBroadcastApplication::SendPacket()
{
    assert((currentPacketSequenceNumber + 1) < UINT_MAX);
    currentPacketSequenceNumber++;

    ItsBroadcastPayloadType itsBroadcastAppPayload(
        currentPacketSequenceNumber, simulationEngineInterfacePtr->CurrentTime());

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(*simulationEngineInterfacePtr, itsBroadcastAppPayload, packetPayloadSizeBytes);

    OutputTraceAndStatsForSendPacket(
        currentPacketSequenceNumber,
        packetPtr->GetPacketId(),
        packetPayloadSizeBytes);

    switch (transportLayerType) {
    case UDP_MODE:
        transportLayerPtr->udpPtr->SendPacket(
            packetPtr, 0, NetworkAddress::broadcastAddress, destinationPortId, packetPriority);
        break;
    case BTP_MODE:
        basicTransportProtocolPtr->SendPacket(
            packetPtr, 0, NetworkAddress::broadcastAddress, destinationPortId, packetPriority,
            GeoNet::SHB, geoNetMaxPacketLifetime, geoNetRepetitionInterval);
        break;
    default:
        assert(false); abort(); break;
    }//switch//

    const SimTime nextTransmissionTime =
        simulationEngineInterfacePtr->CurrentTime() + packetInterval;

    if (nextTransmissionTime < itsBroadcastEndTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new ItsBroadcastEvent(shared_from_this())), nextTransmissionTime);

    }//if//

}//SendPacket//


inline
void ItsBroadcastApplication::ReceivePacket(unique_ptr<Packet>& packetPtr)
{

    assert(initializationIsComplete);

    const ItsBroadcastPayloadType itsBroadcastPayload =
        packetPtr->GetAndReinterpretPayloadData<ItsBroadcastPayloadType>();

    const SimTime delay =
        simulationEngineInterfacePtr->CurrentTime() - itsBroadcastPayload.sendTime;

    (*this).numberPacketsReceived++;

    OutputTraceForReceivePacket(
        itsBroadcastPayload.sequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes(),
        delay);

    packetPtr = nullptr;

}//ReceivePacket//


inline
void ItsBroadcastApplication::OutputTraceAndStatsForSendPacket(
    const unsigned int sequenceNumber,
    const PacketId& thePacketId,
    const unsigned int packetLengthBytes)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationSendTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.destinationNodeId = ANY_NODEID;
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == APPLICATION_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, "", "ItsBcSend", traceData);

        }
        else {

            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << thePacketId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, "", "ItsBcSend", outStream.str());
        }//if//

    }//if//

    packetsSentStatPtr->UpdateCounter(sequenceNumber);
    bytesSentStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForSendPacket//


inline
void ItsBroadcastApplication::OutputTraceForReceivePacket(
    const unsigned int sequenceNumber,
    const PacketId& thePacketId,
    const unsigned int packetLengthBytes,
    const SimTime& delay)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationReceiveTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();
            traceData.delay = delay;
            traceData.receivedPackets = numberPacketsReceived;
            traceData.packetLengthBytes = static_cast<uint16_t>(packetLengthBytes);

            assert(sizeof(traceData) == APPLICATION_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, "", "ItsBcRecv", traceData);

        }
        else {
            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << thePacketId
                      << " Delay= " << ConvertTimeToStringSecs(delay)
                      << " Pdr= " << numberPacketsReceived << '/' << sequenceNumber
                      << " PacketBytes= " << packetLengthBytes;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, "", "ItsBcRecv", outStream.str());

        }//if//

    }//if//

    packetsReceivedStatPtr->IncrementCounter();
    bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);
    endToEndDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(delay));

}//OutputTraceForReceivePacket//


}//namespace//

#endif
