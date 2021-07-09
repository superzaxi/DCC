// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_APP_TRACE_BASED_H
#define SCENSIM_APP_TRACE_BASED_H

#include <memory>

#include "scensim_application.h"
#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "scensim_nodeid.h"
#include "scensim_parmio.h"
#include "pcapglue.h"

namespace ScenSim {

using std::enable_shared_from_this;
using std::shared_ptr;

class TraceBasedApplication: public Application, public enable_shared_from_this<TraceBasedApplication> {
public:
    static const string modelName;

    TraceBasedApplication(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationId& initApplicationId,
        const NodeId& initSourceNodeId,
        const NodeId& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId);

    virtual ~TraceBasedApplication();

    void CompleteInitialization(
        const ParameterDatabaseReader& theParameterDatabaseReader);

    void DisconnectFromOtherLayers();

private:

    enum TraceBasedEventKind {
        SEND_PACKET
    };//TraceBasedEventKind//

    struct Header {
        SimTime sentTime;
        unsigned int sequenceNumber;
    };//Header//

    class PacketHandler
        : public UdpProtocol::PacketForAppFromTransportLayerHandler {
    public:
        PacketHandler(
            const shared_ptr<TraceBasedApplication>& initAppPtr)
            : appPtr(initAppPtr) {}

        void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const NetworkAddress& sourceAddress,
            const unsigned short int sourcePortId,
            const NetworkAddress& destinationAddress,
            const PacketPriority& priority)
        {
            appPtr->ReceivePacket(packetPtr);

        }//ReceivePacket//

    private:
        shared_ptr<TraceBasedApplication> appPtr;

    };//PacketHandler//

    class TraceBasedEvent: public SimulationEvent {
    public:
        explicit
        TraceBasedEvent(
            const shared_ptr<TraceBasedApplication>& initAppPtr,
            const TraceBasedEventKind& initEventKind)
            :
            appPtr(initAppPtr),
            eventKind(initEventKind)
        {}

        virtual void ExecuteEvent()
        {
            switch (eventKind) {
            case SEND_PACKET:
                appPtr->SendPacket();
                break;
            default:
                assert(false); abort();
            }//switch//

        }//ExecuteEvent//

    private:
        shared_ptr<TraceBasedApplication> appPtr;
        TraceBasedEventKind eventKind;

    };//TraceBasedEvent//

    static SimTime GetSentTime(const Packet& aPacket);
    static unsigned int GetSequenceNumber(const Packet& aPacket);

    bool IsSender() const;
    SimTime GetNextPacketSendTime();
    void SendPacket();
    void ReceivePacket(unique_ptr<Packet>& packetPtr);

    void OutputTraceAndStatsForSendPacket(const Packet& aPacket);
    void OutputTraceAndStatsForReceivePacket(const Packet& aPacket);

    bool useVirtualPayload;
    NodeId sourceNodeId;
    NodeId destinationNodeId;
    unsigned short int destinationPortId;
    SimTime startTime;
    SimTime endTime;
    SimTime firstPacketTime;
    unsigned int trimmingHeaderSizeBytes;
    PacketPriority priority;
    SimTime maxStartTimeJitter;

    shared_ptr<PcapGlue> pcapGluePtr;
    shared_ptr<PacketHandler> udpPacketHandlerPtr;
    SimTime deltaTime;
    unsigned int nextPacketSizeBytes;
    unsigned int sequenceNumber;

    int packetsReceived;
    shared_ptr<CounterStatistic> packetsSentStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<CounterStatistic> packetsReceivedStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;
    shared_ptr<RealStatistic> endToEndDelayStatPtr;

};//TraceBasedApplication//

inline
TraceBasedApplication::TraceBasedApplication(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationId& initApplicationId,
    const NodeId& initSourceNodeId,
    const NodeId& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId)
    :
    Application(initSimulationEngineInterfacePtr, initApplicationId),
    useVirtualPayload(false),
    sourceNodeId(initSourceNodeId),
    destinationNodeId(initDestinationNodeId),
    destinationPortId(initDefaultApplicationPortId),
    startTime(ZERO_TIME),
    endTime(INFINITE_TIME),
    firstPacketTime(ZERO_TIME),
    trimmingHeaderSizeBytes(0),
    priority(0),
    maxStartTimeJitter(ZERO_TIME),
    pcapGluePtr(),
    udpPacketHandlerPtr(),
    deltaTime(ZERO_TIME),
    nextPacketSizeBytes(0),
    sequenceNumber(0),
    packetsSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_PacketsSent"))),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_BytesSent"))),
    packetsReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_PacketsReceived"))),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_BytesReceived"))),
    endToEndDelayStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_" + initApplicationId + "_EndToEndDelay")))
{
    startTime = theParameterDatabaseReader.ReadTime(
        "trace-based-app-start-time", sourceNodeId, theApplicationId);

    if (theParameterDatabaseReader.ParameterExists(
        "trace-based-app-end-time", sourceNodeId, theApplicationId)) {

        endTime = theParameterDatabaseReader.ReadTime(
            "trace-based-app-end-time", sourceNodeId, theApplicationId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        "trace-based-app-priority", sourceNodeId, theApplicationId)) {

        priority = static_cast<PacketPriority>(
            theParameterDatabaseReader.ReadNonNegativeInt(
                "trace-based-app-priority", sourceNodeId, theApplicationId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            "trace-based-app-start-time-max-jitter", sourceNodeId, theApplicationId)) {

        maxStartTimeJitter =
            theParameterDatabaseReader.ReadTime(
                "trace-based-app-start-time-max-jitter", sourceNodeId, theApplicationId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        "trace-based-app-auto-port-mode", sourceNodeId, theApplicationId)) {

        if (!theParameterDatabaseReader.ReadBool(
                "trace-based-app-auto-port-mode", sourceNodeId, theApplicationId)) {

            destinationPortId = static_cast<unsigned short int>(
                theParameterDatabaseReader.ReadNonNegativeInt(
                    "trace-based-app-destination-port", sourceNodeId, theApplicationId));
        }//if//
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        "trace-based-app-use-virtual-payload", sourceNodeId, theApplicationId)) {

        useVirtualPayload = theParameterDatabaseReader.ReadBool(
            "trace-based-app-use-virtual-payload", sourceNodeId, theApplicationId);
    }//if//

    const string inputFileType = MakeLowerCaseString(
        theParameterDatabaseReader.ReadString(
            "trace-based-app-input-file-type", sourceNodeId, theApplicationId));

    if (inputFileType == "pcap") {

        firstPacketTime = theParameterDatabaseReader.ReadTime(
            "trace-based-app-pcap-first-packet-time", sourceNodeId, theApplicationId);

        trimmingHeaderSizeBytes = theParameterDatabaseReader.ReadNonNegativeInt(
            "trace-based-app-pcap-trimming-header-size-bytes", sourceNodeId, theApplicationId);

        const string pcapFileName =
            theParameterDatabaseReader.ReadString(
                "trace-based-app-pcap-input-file", sourceNodeId, theApplicationId);

        pcapGluePtr.reset(new PcapGlue(pcapFileName));
    }
    else {
        cerr << "Error: trace-based-app-input-file-type(" << inputFileType
             << ") must be pcap. Other types are not supported." << endl;
        exit(1);
    }//if//

}//TraceBasedApplication//

inline
TraceBasedApplication::~TraceBasedApplication()
{
}//~TraceBasedApplication//

inline
void TraceBasedApplication::CompleteInitialization(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    if (IsSender()) {
        SimTime startTimeJitter = ZERO_TIME;

        if (maxStartTimeJitter != ZERO_TIME) {
            startTimeJitter += static_cast<SimTime>(
                aRandomNumberGeneratorPtr->GenerateRandomDouble() * maxStartTimeJitter);
        }//if//

        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new TraceBasedEvent(shared_from_this(), SEND_PACKET)),
            GetNextPacketSendTime() + startTimeJitter);
    }
    else {
        udpPacketHandlerPtr = shared_ptr<PacketHandler>(
            new PacketHandler(shared_from_this()));

        transportLayerPtr->udpPtr->OpenSpecificUdpPort(
            NetworkAddress::anyAddress, destinationPortId, udpPacketHandlerPtr);
    }//if//

}//CompleteInitialization//

inline
void TraceBasedApplication::DisconnectFromOtherLayers()
{
    udpPacketHandlerPtr.reset();
    (*this).Application::DisconnectFromOtherLayers();

}//DisconnectFromOtherLayers//

inline
SimTime TraceBasedApplication::GetSentTime(const Packet& aPacket)
{
    const Header& header = aPacket.GetAndReinterpretPayloadData<const Header>();
    return header.sentTime;

}//GetSentTime//

inline
unsigned int TraceBasedApplication::GetSequenceNumber(const Packet& aPacket)
{
    const Header& header = aPacket.GetAndReinterpretPayloadData<const Header>();
    return header.sequenceNumber;

}//GetSequenceNumber//

inline
bool TraceBasedApplication::IsSender() const
{
    const NodeId theNodeId =
        simulationEngineInterfacePtr->GetNodeId();

    return (theNodeId == sourceNodeId);

}//IsSender//

inline
SimTime TraceBasedApplication::GetNextPacketSendTime()
{
    bool success;
    SimTime absoluteSendTime;
    unsigned int packetLengthBytes;

    pcapGluePtr->ReadPacketInfo(success, absoluteSendTime, packetLengthBytes);

    if (!success) {
        nextPacketSizeBytes = 0;
        return INFINITE_TIME;
    }//if//

    if (deltaTime == ZERO_TIME) {
        deltaTime = absoluteSendTime;
    }//if//

    if (packetLengthBytes > trimmingHeaderSizeBytes) {
        nextPacketSizeBytes = packetLengthBytes - trimmingHeaderSizeBytes;
    }
    else {
        cerr << "Error: trace-based-app-trimming-header-size-bytes(" << trimmingHeaderSizeBytes
             << ") is greater than packet size(" << packetLengthBytes << ")" << endl;
        exit(1);
    }//if//

    const SimTime relativeSendTime = absoluteSendTime - deltaTime;
    const SimTime nextPacketSendTime = firstPacketTime + relativeSendTime;
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (endTime < nextPacketSendTime) {
        nextPacketSizeBytes = 0;
        return INFINITE_TIME;
    }
    else if ((startTime <= nextPacketSendTime) && (currentTime <= nextPacketSendTime)) {
        return nextPacketSendTime;
    }
    else {
        return GetNextPacketSendTime();
    }//if//

}//GetNextPacketSendTime//

inline
void TraceBasedApplication::SendPacket()
{
    ++sequenceNumber;

    NetworkAddress sourceAddress;
    bool foundSourceAddress;

    networkAddressLookupInterfacePtr->LookupNetworkAddress(
        sourceNodeId, sourceAddress, foundSourceAddress);

    NetworkAddress destinationAddress;
    bool foundDestAddress;

    if (destinationNodeId == ANY_NODEID) {
        destinationAddress = NetworkAddress::broadcastAddress;
        foundDestAddress = true;
    }
    else {
        networkAddressLookupInterfacePtr->LookupNetworkAddress(
            destinationNodeId, destinationAddress, foundDestAddress);
    }//if//

    if (foundSourceAddress && foundDestAddress) {
        Header header;

        header.sentTime = simulationEngineInterfacePtr->CurrentTime();
        header.sequenceNumber = sequenceNumber;

        if (nextPacketSizeBytes < sizeof(header)) {
            nextPacketSizeBytes = sizeof(header);
        }//if//

        unique_ptr<Packet> packetPtr =
            Packet::CreatePacket(
                *simulationEngineInterfacePtr,
                header,
                nextPacketSizeBytes,
                useVirtualPayload);

        OutputTraceAndStatsForSendPacket(*packetPtr);

        transportLayerPtr->udpPtr->SendPacket(
            packetPtr, sourceAddress, 0, destinationAddress, destinationPortId, priority);
    }//if//

    simulationEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(new TraceBasedEvent(shared_from_this(), SEND_PACKET)),
        GetNextPacketSendTime());

}//SendPacket//

inline
void TraceBasedApplication::ReceivePacket(
    unique_ptr<Packet>& packetPtr)
{
    OutputTraceAndStatsForReceivePacket(*packetPtr);

    packetPtr = nullptr;

}//ReceivePacket//

inline
void TraceBasedApplication::OutputTraceAndStatsForSendPacket(const Packet& aPacket)
{
    packetsSentStatPtr->IncrementCounter();
    bytesSentStatPtr->IncrementCounter(aPacket.LengthBytes());

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationSendTraceRecord traceData;

            traceData.packetSequenceNumber = GetSequenceNumber(aPacket);
            traceData.sourceNodeId = aPacket.GetPacketId().GetSourceNodeId();
            traceData.destinationNodeId = destinationNodeId;
            traceData.sourceNodeSequenceNumber =
                aPacket.GetPacketId().GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == APPLICATION_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theApplicationId, "TraceBasedSend", traceData);
        }
        else {
            ostringstream outStream;

            outStream << "Seq= " << GetSequenceNumber(aPacket);
            outStream << " PktId= " << aPacket.GetPacketId();

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "TraceBasedSend", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForSendPacket//

inline
void TraceBasedApplication::OutputTraceAndStatsForReceivePacket(const Packet& aPacket)
{
    const SimTime delayTime =
        simulationEngineInterfacePtr->CurrentTime() - GetSentTime(aPacket);

    packetsReceived += 1;

    packetsReceivedStatPtr->IncrementCounter();
    bytesReceivedStatPtr->IncrementCounter(aPacket.LengthBytes());
    endToEndDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(delayTime));

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationReceiveTraceRecord traceData;

            traceData.packetSequenceNumber = GetSequenceNumber(aPacket);
            traceData.sourceNodeId = aPacket.GetPacketId().GetSourceNodeId();
            traceData.sourceNodeSequenceNumber =
                aPacket.GetPacketId().GetSourceNodeSequenceNumber();
            traceData.delay = delayTime;
            traceData.receivedPackets = packetsReceived;
            traceData.packetLengthBytes = static_cast<uint16_t>(aPacket.LengthBytes());

            assert(sizeof(traceData) == APPLICATION_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theApplicationId, "TraceBasedRecv", traceData);
        }
        else {
            ostringstream outStream;

            outStream << "Seq= " << GetSequenceNumber(aPacket);
            outStream << " PktId= " << aPacket.GetPacketId();
            outStream << " Delay= " << ConvertTimeToStringSecs(delayTime);
            outStream << " Pdr= " << packetsReceived << '/' << GetSequenceNumber(aPacket);
            outStream << " PacketBytes= " << aPacket.LengthBytes();

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "TraceBasedRecv", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForReceivePacket//

}//namespace//

#endif
