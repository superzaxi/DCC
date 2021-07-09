// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_APP_CBR_H
#define SCENSIM_APP_CBR_H

#include "scensim_netsim.h"

namespace ScenSim {

using std::cerr;
using std::endl;


//--------------------------------------------------------------------------------------------------

class CbrApplication: public Application {
public:
    static const string modelName;

    typedef MacQualityOfServiceControlInterface::SchedulingSchemeChoice SchedulingSchemeChoice;
    typedef MacQualityOfServiceControlInterface::ReservationSchemeChoice ReservationSchemeChoice;

    CbrApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationId& initApplicationId,
        const NodeId& initSourceNodeId,
        const NodeId& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initReserveBandwidthModeIsOn);

    struct CbrPayloadType {
        unsigned int sequenceNumber;
        SimTime sendTime;

        CbrPayloadType(
            const unsigned int initSequenceNumber,
            const SimTime initSendTime)
            :
            sequenceNumber(initSequenceNumber),
            sendTime(initSendTime)
        {}
    };//CbrPayloadType//

protected:
    NodeId sourceNodeId;
    NodeId destinationNodeId;
    NetworkAddress destinationMulticastIpAddress;
    unsigned short int destinationPortId;
    int packetPayloadSizeBytes;
    SimTime packetInterval;
    SimTime cbrStartTime;
    SimTime cbrEndTime;
    PacketPriority cbrPriority;
    SimTime maxStartTimeJitter;

    bool reserveBandwidthModeIsOn;
    double qosMinBandwidth;
    double qosMaxBandwidth;

    FlowId macQosFlowId;

    SchedulingSchemeChoice schedulingScheme;
    ReservationSchemeChoice reservationScheme;

    bool useVirtualPayload;

    string GetParameterNamePrefix() const {
        if (reserveBandwidthModeIsOn) {
            return  "cbr-with-qos";
        }//if//
        return "cbr";
    }

};//CbrApplication//

inline
CbrApplication::CbrApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationId& initApplicationId,
    const NodeId& initSourceNodeId,
    const NodeId& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initReserveBandwidthModeIsOn)
    :
    Application(initSimulationEngineInterfacePtr, initApplicationId),
    sourceNodeId(initSourceNodeId),
    destinationNodeId(initDestinationNodeId),
    destinationPortId(initDefaultApplicationPortId),
    packetPayloadSizeBytes(0),
    packetInterval(ZERO_TIME),
    cbrStartTime(ZERO_TIME),
    cbrEndTime(ZERO_TIME),
    cbrPriority(0),
    maxStartTimeJitter(ZERO_TIME),
    reserveBandwidthModeIsOn(initReserveBandwidthModeIsOn),
    qosMinBandwidth(0.0),
    qosMaxBandwidth(0.0),
    schedulingScheme(MacQualityOfServiceControlInterface::DefaultSchedulingScheme),
    reservationScheme(MacQualityOfServiceControlInterface::OptimisticLinkRate),
    useVirtualPayload(false)
{
    const string parameterPrefix = (*this).GetParameterNamePrefix();

    packetPayloadSizeBytes =
        parameterDatabaseReader.ReadInt(
            parameterPrefix + "-payload-size-bytes", sourceNodeId, theApplicationId);

    if (packetPayloadSizeBytes < (int)sizeof(CbrPayloadType)) {
        cerr << "Error: CBR Application payload size ("
            << packetPayloadSizeBytes << ") should be "
            << sizeof(CbrPayloadType) << " bytes or larger." << endl;
        exit(1);
    }//if//

    bool trafficSpecifiedInterval = false;
    bool trafficSpecifiedBps = false;
    bool trafficSpecifiedPps = false;

    if (parameterDatabaseReader.ParameterExists(
            parameterPrefix + "-traffic-defined-by", sourceNodeId, theApplicationId)) {

        const string trafficDefinedBy =
            MakeLowerCaseString(
                parameterDatabaseReader.ReadString(
                    parameterPrefix + "-traffic-defined-by", sourceNodeId, theApplicationId));

        if (trafficDefinedBy == "interval") {
            trafficSpecifiedInterval = true;
        }
        else if (trafficDefinedBy == "packetspersecond") {
            trafficSpecifiedPps = true;
        }
        else if (trafficDefinedBy == "bitspersecond") {
            trafficSpecifiedBps = true;
        }
        else {
            cerr << "Error : invalid traffic specification " << trafficDefinedBy << endl;
            exit(1);
        }//if//
    }
    else {
        trafficSpecifiedInterval =
            parameterDatabaseReader.ParameterExists(
                parameterPrefix + "-interval", sourceNodeId, theApplicationId);

        trafficSpecifiedBps =
            parameterDatabaseReader.ParameterExists(
                parameterPrefix + "-traffic-bps", sourceNodeId, theApplicationId);

        trafficSpecifiedPps =
            parameterDatabaseReader.ParameterExists(
                parameterPrefix + "-traffic-pps", sourceNodeId, theApplicationId);
    }//if//

    if ((trafficSpecifiedInterval && trafficSpecifiedBps) ||
        (trafficSpecifiedInterval && trafficSpecifiedPps) ||
        (trafficSpecifiedBps && trafficSpecifiedPps)) {
        cerr << "Error : application traffic specification is conflicted." << endl;
        exit(1);
    }//if//

    if (trafficSpecifiedBps) {
        const unsigned long long int  trafficBps =
            parameterDatabaseReader.ReadBigInt(
                parameterPrefix + "-traffic-bps", sourceNodeId, theApplicationId);

        if (trafficBps == 0) {
            cerr << "Error : Set " << parameterPrefix << "-traffic-bps > 0." << endl;
            exit(1);
        }

        const double trafficBytePerSec = static_cast<double>(trafficBps) / 8.;

        packetInterval = ConvertDoubleSecsToTime(packetPayloadSizeBytes / trafficBytePerSec);
    }
    else if (trafficSpecifiedPps) {
        const double trafficPps =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-traffic-pps", sourceNodeId, theApplicationId);

        if (trafficPps == 0) {
            cerr << "Error : Set " << parameterPrefix << "-traffic-pps > 0." << endl;
            exit(1);
        }

        packetInterval = ConvertDoubleSecsToTime(1./trafficPps);
    }
    else {
        packetInterval =
            parameterDatabaseReader.ReadTime(
                parameterPrefix + "-interval", sourceNodeId, theApplicationId);
    }//if//

    cbrStartTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-start-time", sourceNodeId, theApplicationId);

    cbrEndTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-end-time", sourceNodeId, theApplicationId);

    cbrPriority = static_cast<PacketPriority>(
        parameterDatabaseReader.ReadNonNegativeInt(
            parameterPrefix + "-priority", sourceNodeId, theApplicationId));

    if (parameterDatabaseReader.ParameterExists(
            parameterPrefix + "-start-time-max-jitter", sourceNodeId, theApplicationId)) {

        maxStartTimeJitter =
            parameterDatabaseReader.ReadTime(
                parameterPrefix + "-start-time-max-jitter", sourceNodeId, theApplicationId);
    }//if//

    if (parameterDatabaseReader.ParameterExists(
            parameterPrefix + "-auto-port-mode", sourceNodeId, theApplicationId)) {

        if (!parameterDatabaseReader.ReadBool(
                parameterPrefix + "-auto-port-mode", sourceNodeId, theApplicationId)) {

            destinationPortId = static_cast<unsigned short int>(
                parameterDatabaseReader.ReadNonNegativeInt(
                    parameterPrefix + "-destination-port", sourceNodeId, theApplicationId));
        }//if//
    }//if//

    if (reserveBandwidthModeIsOn) {
        qosMinBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-baseline-bandwidth-bytes", sourceNodeId, theApplicationId);

        qosMaxBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-max-bandwidth-bytes", sourceNodeId, theApplicationId);

        if (parameterDatabaseReader.ParameterExists(
                parameterPrefix + "-schedule-scheme", sourceNodeId, theApplicationId)) {

            const string schedulingSchemeChoiceString =
                parameterDatabaseReader.ReadString(
                    parameterPrefix + "-schedule-scheme", sourceNodeId, theApplicationId);

            bool succeeded;

            ConvertStringToSchedulingSchemeChoice(
                MakeLowerCaseString(schedulingSchemeChoiceString),
                succeeded,
                schedulingScheme);

            if (!succeeded) {
                cerr << "Error in " << modelName << " Application: Scheduling Scheme not recognized in:" << endl;
                cerr << "      >" << schedulingSchemeChoiceString << endl;
            exit(1);
            }//if//
        }//if//
    }//if//

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-use-virtual-payload", sourceNodeId, theApplicationId)) {

        useVirtualPayload = parameterDatabaseReader.ReadBool(
            parameterPrefix + "-use-virtual-payload", sourceNodeId, theApplicationId);
    }//if//


    destinationMulticastIpAddress = NetworkAddress::invalidAddress;

    if (parameterDatabaseReader.ParameterExists(
        (parameterPrefix + "-destination-multicast-group-number"), sourceNodeId, theApplicationId)) {

        if (destinationNodeId != ANY_NODEID) {
            cerr << "Error in " << modelName << " Multicast group number but destination is to a single node." << endl;
            exit(1);
        }//if//

        const unsigned int multicastGroupNumber =
            parameterDatabaseReader.ReadNonNegativeInt(
                (parameterPrefix + "-destination-multicast-group-number"),
                sourceNodeId, theApplicationId);


        if (multicastGroupNumber > NetworkAddress::maxMulticastGroupNumber) {
            cerr << "Error in " << modelName << " Multicast group number is too large: "
                 << multicastGroupNumber << endl;
        }//if//

        destinationMulticastIpAddress = NetworkAddress::MakeAMulticastAddress(multicastGroupNumber);

    }//if//

};//CbrApplication//



class CbrSourceApplication:
     public CbrApplication, public enable_shared_from_this<CbrSourceApplication> {
public:

    CbrSourceApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationId& initApplicationId,
        const NodeId& initSourceNodeId,
        const NodeId& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initReserveBandwidthModeIsOn);

    void CompleteInitialization();

private:
    class CbrEvent: public SimulationEvent {
    public:
        explicit
        CbrEvent(const shared_ptr<CbrSourceApplication>& initCbrApplicationPtr)
            : cbrApplicationPtr(initCbrApplicationPtr) {}
        virtual void ExecuteEvent() { cbrApplicationPtr->SendPacket(); }

    private:
        shared_ptr<CbrSourceApplication> cbrApplicationPtr;

    };//CbrEvent//

    class FlowRequestReplyFielder: public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<CbrSourceApplication>& initAppPtr) : appPtr(initAppPtr) { }

        void RequestAccepted(const FlowId& theFlowId)
            { appPtr->macQosFlowId = theFlowId; }
        void RequestDenied() { appPtr->ReserveBandwidthRequestDeniedAction(); }
    private:
        shared_ptr<CbrSourceApplication> appPtr;

    };//FlowRequestReplyFielder//

    unsigned int currentPacketSequenceNumber;

    shared_ptr<CounterStatistic> packetsSentStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;

    void SendPacket();

    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();

    //For stats

    void OutputTraceAndStatsForSendPacket(
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const unsigned int packetLengthBytes);

};//CbrSourceApplication//

inline
CbrSourceApplication::CbrSourceApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationId& initApplicationId,
    const NodeId& initSourceNodeId,
    const NodeId& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initReserveBandwidthModeIsOn)
    :
    CbrApplication(
        parameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initApplicationId,
        initSourceNodeId,
        initDestinationNodeId,
        initDefaultApplicationPortId,
        initReserveBandwidthModeIsOn),
    currentPacketSequenceNumber(0),
    packetsSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_PacketsSent"))),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_BytesSent")))
{
}//CbrSourceApplication//

// Two part initialization forced by shared_from_this().

inline
void CbrSourceApplication::CompleteInitialization()
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (maxStartTimeJitter != ZERO_TIME) {
        cbrStartTime += static_cast<SimTime>(
            aRandomNumberGeneratorPtr->GenerateRandomDouble() * maxStartTimeJitter);
    }//if//

    const SimTime minimumSetupTime = 1 * MILLI_SECOND;

    if (cbrStartTime < (currentTime + minimumSetupTime)) {
        const size_t nextTransmissionTime =
            size_t(ceil(double((currentTime + minimumSetupTime) - cbrStartTime) / packetInterval));

        cbrStartTime += (nextTransmissionTime * packetInterval);
    }//if//

    if (cbrStartTime < cbrEndTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new CbrEvent(shared_from_this())),
            cbrStartTime);
    }//if//
}//CompleteInitialization//


inline
void CbrSourceApplication::OutputTraceAndStatsForSendPacket(
    const unsigned int sequenceNumber,
    const PacketId& thePacketId,
    const unsigned int packetLengthBytes)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        string eventName;

        if (destinationNodeId == ANY_NODEID) {
            eventName = "CbrBcSend";
        }
        else {
            eventName = "CbrSend";
        }//if//

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationSendTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.destinationNodeId = destinationNodeId;
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == APPLICATION_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theApplicationId, eventName, traceData);
        }
        else {
            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << thePacketId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, eventName, outStream.str());
        }//if//
    }//if//

    const unsigned int numberPacketsSent = sequenceNumber;

    packetsSentStatPtr->UpdateCounter(numberPacketsSent);
    bytesSentStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForSendPacket//




inline
void CbrSourceApplication::SendPacket()
{
    assert(currentPacketSequenceNumber < UINT_MAX);

    if ((currentPacketSequenceNumber == 0) && (reserveBandwidthModeIsOn)) {
        (*this).ReserveBandwidth();
    }//if//

    currentPacketSequenceNumber++;

    CbrPayloadType cbrAppPayload(
        currentPacketSequenceNumber, simulationEngineInterfacePtr->CurrentTime());

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(
            *simulationEngineInterfacePtr,
            cbrAppPayload,
            packetPayloadSizeBytes,
            useVirtualPayload);

    // Use my primary address for source address.

    const NetworkAddress sourceAddress =
        networkAddressLookupInterfacePtr->LookupNetworkAddress(sourceNodeId);

    OutputTraceAndStatsForSendPacket(
        currentPacketSequenceNumber,
        packetPtr->GetPacketId(),
        packetPayloadSizeBytes);

    if (destinationNodeId == ANY_NODEID) {
        if (destinationMulticastIpAddress == NetworkAddress::invalidAddress) {
            transportLayerPtr->udpPtr->SendPacket(
                packetPtr, 0, NetworkAddress::broadcastAddress, destinationPortId, cbrPriority);
        }
        else {
            transportLayerPtr->udpPtr->SendPacket(
                packetPtr, 0, destinationMulticastIpAddress, destinationPortId, cbrPriority);
        }//if//
    }
    else {
        NetworkAddress destAddress;
        bool foundDestAddress;
        networkAddressLookupInterfacePtr->LookupNetworkAddress(destinationNodeId, destAddress, foundDestAddress);
        if (foundDestAddress) {
            transportLayerPtr->udpPtr->SendPacket(
                packetPtr, sourceAddress, 0, destAddress, destinationPortId, cbrPriority);
        }
        else {
            //cannot find destination address (destination node may not be created yet)
            //Future: output trace and stat
            packetPtr = nullptr;
        }//if//
    }//if//

    const SimTime nextPacketTime =
        simulationEngineInterfacePtr->CurrentTime() + packetInterval;

    if (nextPacketTime < cbrEndTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new CbrEvent(shared_from_this())),
            nextPacketTime);
    }
    else {
        if (reserveBandwidthModeIsOn) {
            (*this).UnreserveBandwidth();
        }//if//
    }//if//

}//SendPacket//



inline
void CbrSourceApplication::ReserveBandwidth()
{
    const NetworkAddress sourceAddress =
        networkAddressLookupInterfacePtr->LookupNetworkAddress(sourceNodeId);

    NetworkAddress destinationAddress;
    bool foundDestAddress;
    networkAddressLookupInterfacePtr->LookupNetworkAddress(
        destinationNodeId, destinationAddress, foundDestAddress);

    if (!foundDestAddress) {
        //Destination node may not be created yet.
        return;
    }//if//

    const NetworkLayer& theNetworkLayer = *transportLayerPtr->GetNetworkLayerPtr();

    bool success;
    unsigned int interfaceIndex;
    NetworkAddress nextHopAddress;
    theNetworkLayer.GetNextHopAddressAndInterfaceIndexForDestination(
        destinationAddress, success, nextHopAddress, interfaceIndex);

    if ((success) && (theNetworkLayer.MacSupportsQualityOfService(interfaceIndex))) {

        MacQualityOfServiceControlInterface& qosControlInterface =
            *theNetworkLayer.GetMacQualityOfServiceInterface(interfaceIndex);

        shared_ptr<FlowRequestReplyFielder> replyPtr(new FlowRequestReplyFielder(shared_from_this()));

        qosControlInterface.RequestUplinkFlowReservation(
            reservationScheme, schedulingScheme, cbrPriority, qosMinBandwidth, qosMaxBandwidth,
            destinationAddress, destinationPortId, sourceAddress, ANY_PORT, IP_PROTOCOL_NUMBER_UDP,
            replyPtr);
    }//if//

}//ReserveBandwidth//



inline
void CbrSourceApplication::ReserveBandwidthRequestDeniedAction()
{
    cerr << "Warning in CBR with QoS application: Bandwidth request denied." << endl;
}


inline
void CbrSourceApplication::UnreserveBandwidth()
{
    NetworkAddress destinationAddress;
    bool foundDestAddress;
    networkAddressLookupInterfacePtr->LookupNetworkAddress(
        destinationNodeId, destinationAddress, foundDestAddress);

    if (!foundDestAddress) {
        //Destination node might disappear.
        return;
    }//if//

    const NetworkLayer& theNetworkLayer = *transportLayerPtr->GetNetworkLayerPtr();

    bool success;
    unsigned int interfaceIndex;
    NetworkAddress nextHopAddress;
    theNetworkLayer.GetNextHopAddressAndInterfaceIndexForDestination(
        destinationAddress, success, nextHopAddress, interfaceIndex);

    if ((success) && (theNetworkLayer.MacSupportsQualityOfService(interfaceIndex))) {

        MacQualityOfServiceControlInterface& qosControlInterface =
            *theNetworkLayer.GetMacQualityOfServiceInterface(interfaceIndex);

        qosControlInterface.DeleteFlow(macQosFlowId);

    }//if//

}//UnreserveBandwidth//



//--------------------------------------------------------------------------------------------------

class CbrSinkApplication:
    public CbrApplication, public enable_shared_from_this<CbrSinkApplication> {
public:
    CbrSinkApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationId& initApplicationId,
        const NodeId& initSourceNodeId,
        const NodeId& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initReserveBandwidthModeIsOn);

    void CompleteInitialization();

    void DisconnectFromOtherLayers();

private:

    class FlowReservationEvent: public SimulationEvent {
    public:
        explicit
        FlowReservationEvent(const shared_ptr<CbrSinkApplication>& initCbrApplicationPtr)
            : cbrApplicationPtr(initCbrApplicationPtr) {}
        virtual void ExecuteEvent() { cbrApplicationPtr->ReserveOrUnreserveBandwidth(); }

    private:
        shared_ptr<CbrSinkApplication> cbrApplicationPtr;

    };//FlowReservationEvent//

    class PacketHandler: public UdpProtocol::PacketForAppFromTransportLayerHandler {
    public:
        PacketHandler(const shared_ptr<CbrSinkApplication>& initCbrSinkPtr) : cbrSinkPtr(initCbrSinkPtr) { }

        void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const NetworkAddress& sourceAddress,
            const unsigned short int sourcePort,
            const NetworkAddress& destinationAddress,
            const PacketPriority& priority)
        {
            cbrSinkPtr->ReceivePacket(packetPtr);
        }


    private:
        shared_ptr<CbrSinkApplication> cbrSinkPtr;

    };//PacketHandler//

    //-------------------------------------------------------------------------

    shared_ptr<PacketHandler> packetHandlerPtr;

    // This should be simulator parameters. TBD
    static const SimTime duplicateDetectorWindowTime = 60 * SECOND;
    static const unsigned int duplicateDetectorMinWindowSize = 128;

    static unsigned int CalcDuplicateDetectorWindowSize(
        const SimTime& timeInterval,
        const SimTime& totalTrafficTime)
    {
        const SimTime windowTime = std::min(duplicateDetectorWindowTime, totalTrafficTime);

        return (
            std::max(
                duplicateDetectorMinWindowSize,
                static_cast<unsigned int>(DivideAndRoundUp(windowTime, timeInterval))));
    }

    WindowedDuplicateSequenceNumberDetector<unsigned int> duplicateDetector;

    //---------------------------------

    class FlowRequestReplyFielder: public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<CbrSinkApplication>& initAppPtr) : appPtr(initAppPtr) { }

        void RequestAccepted(const FlowId& theFlowId)
            { appPtr->macQosFlowId = theFlowId; }
        void RequestDenied() { appPtr->ReserveBandwidthRequestDeniedAction(); }
    private:
        shared_ptr<CbrSinkApplication> appPtr;

    };//FlowRequestReplyFielder//

    //---------------------------------

    unsigned int numberPacketsReceived;
    unsigned int numberDuplicatePacketsReceived;

    shared_ptr<CounterStatistic> packetsReceivedStatPtr;
    shared_ptr<CounterStatistic> duplicatePacketsReceivedStatPtr;
    shared_ptr<CounterStatistic> duplicatePacketOutOfWindowErrorStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;
    shared_ptr<RealStatistic> endToEndDelayStatPtr;

    //---------------------------------

    void ReceivePacket(unique_ptr<Packet>& packetPtr);

    void OutputTraceAndStatsForReceivePacket(
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const unsigned int packetLengthBytes,
        const SimTime& delay);


    void ReserveOrUnreserveBandwidth();
    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();

};//CbrSinkApplication//



inline
CbrSinkApplication::CbrSinkApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationId& initApplicationId,
    const NodeId& initSourceNodeId,
    const NodeId& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initReserveBandwidthModeIsOn)
    :
    CbrApplication(
        parameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initApplicationId,
        initSourceNodeId,
        initDestinationNodeId,
        initDefaultApplicationPortId,
        initReserveBandwidthModeIsOn),
    duplicateDetector(
        CalcDuplicateDetectorWindowSize(packetInterval, (cbrEndTime - cbrStartTime))),
    numberPacketsReceived(0),
    numberDuplicatePacketsReceived(0),
    packetsReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_"  + initApplicationId + "_PacketsReceived"))),
    duplicatePacketsReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_"  + initApplicationId + "_DuplicatePacketsReceived"))),
    duplicatePacketOutOfWindowErrorStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_"  + initApplicationId + "_DuplicatePacketOutOfWindowErrors"))),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_"  + initApplicationId + "_BytesReceived"))),
    endToEndDelayStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_"  + initApplicationId + "_EndToEndDelay")))
{
}//CbrSinkApplication//



// Two part initialization forced by shared_from_this().

inline
void CbrSinkApplication::CompleteInitialization()
{
    packetHandlerPtr = shared_ptr<PacketHandler>(new PacketHandler(shared_from_this()));

    assert(transportLayerPtr->udpPtr->PortIsAvailable(destinationPortId));

    transportLayerPtr->udpPtr->OpenSpecificUdpPort(
        NetworkAddress::anyAddress,
        destinationPortId,
        packetHandlerPtr);

    if (reserveBandwidthModeIsOn) {

        const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
        const SimTime minimumSetupTime = 1 * MILLI_SECOND;
        const SimTime reservationLeewayTime = 1 * MILLI_SECOND;

        SimTime reservationTime;
        if ((currentTime + minimumSetupTime + reservationLeewayTime) <= cbrStartTime) {
            reservationTime = (cbrStartTime - reservationLeewayTime);
        }
        else {
            reservationTime = (currentTime + minimumSetupTime);
        }//if//

        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new FlowReservationEvent(shared_from_this())),
            reservationTime);

        //Future: QoS Flow termination delay parameter.

        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new FlowReservationEvent(shared_from_this())),
            (cbrEndTime + reservationLeewayTime));

    }//if//
}//CompleteInitialization//



inline
void CbrSinkApplication::DisconnectFromOtherLayers()
{
    packetHandlerPtr.reset();
    (*this).Application::DisconnectFromOtherLayers();

}//DisconnectFromOtherLayers//



inline
void CbrSinkApplication::OutputTraceAndStatsForReceivePacket(
    const unsigned int sequenceNumber,
    const PacketId& thePacketId,
    const unsigned int packetLengthBytes,
    const SimTime& delay)
{
    if (duplicateDetector.IsInSequenceNumberWindow(sequenceNumber)) {
        if (!duplicateDetector.IsDuplicate(sequenceNumber)) {
            duplicateDetector.SetAsSeen(sequenceNumber);
            numberPacketsReceived++;
            packetsReceivedStatPtr->IncrementCounter();
            bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);
            endToEndDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(delay));
        }
        else {
            numberDuplicatePacketsReceived++;
            duplicatePacketsReceivedStatPtr->IncrementCounter();
        }//if//
    }
    else {
        duplicatePacketOutOfWindowErrorStatPtr->IncrementCounter();
    }//if//


    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        string eventName;

        if (destinationNodeId == ANY_NODEID) {
            eventName = "CbrBcRecv";
        } else {
            eventName = "CbrRecv";
        }

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
                modelName, theApplicationId, eventName, traceData);
        }
        else {
            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << thePacketId
                      << " Delay= " << ConvertTimeToStringSecs(delay)
                      << " Pdr= " << numberPacketsReceived << '/' << sequenceNumber
                      << " PacketBytes= " << packetLengthBytes;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, eventName, outStream.str());

        }//if//
    }//if//

}//OutputTraceAndStatsForReceivePacket//



inline
void CbrSinkApplication::ReceivePacket(unique_ptr<Packet>& packetPtr)
{
    CbrPayloadType cbrPayload = packetPtr->GetAndReinterpretPayloadData<CbrPayloadType>();

    SimTime delay = simulationEngineInterfacePtr->CurrentTime() - cbrPayload.sendTime;

    OutputTraceAndStatsForReceivePacket(
        cbrPayload.sequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes(),
        delay);

    packetPtr = nullptr;
}


inline
void CbrSinkApplication::ReserveBandwidth()
{
    NetworkAddress sourceAddress;
    bool foundSourceAddress;
    networkAddressLookupInterfacePtr->LookupNetworkAddress(
        sourceNodeId, sourceAddress, foundSourceAddress);

    if (!foundSourceAddress) {
        return;
    }//if//

    const NetworkAddress destinationAddress =
        networkAddressLookupInterfacePtr->LookupNetworkAddress(destinationNodeId);

    const NetworkLayer& theNetworkLayer = *transportLayerPtr->GetNetworkLayerPtr();

    bool success;
    unsigned int interfaceIndex;
    NetworkAddress nextHopAddress;
    theNetworkLayer.GetNextHopAddressAndInterfaceIndexForDestination(
        destinationAddress, success, nextHopAddress, interfaceIndex);

    if ((success) && (theNetworkLayer.MacSupportsQualityOfService(interfaceIndex))) {

        MacQualityOfServiceControlInterface& qosControlInterface =
            *theNetworkLayer.GetMacQualityOfServiceInterface(interfaceIndex);

        shared_ptr<FlowRequestReplyFielder> replyPtr(new FlowRequestReplyFielder(shared_from_this()));

        qosControlInterface.RequestDownlinkFlowReservation(
            reservationScheme, schedulingScheme, cbrPriority, qosMinBandwidth, qosMaxBandwidth,
            sourceAddress, ANY_PORT, destinationAddress, destinationPortId, IP_PROTOCOL_NUMBER_UDP,
            replyPtr);
    }//if//

}//ReserveBandwidth//



inline
void CbrSinkApplication::ReserveBandwidthRequestDeniedAction()
{
    cerr << "Warning in CBR with QoS application: Bandwidth request denied." << endl;
}



inline
void CbrSinkApplication::UnreserveBandwidth()
{
    const NetworkAddress destinationAddress =
        networkAddressLookupInterfacePtr->LookupNetworkAddress(destinationNodeId);

    const NetworkLayer& theNetworkLayer = *transportLayerPtr->GetNetworkLayerPtr();

    bool success;
    unsigned int interfaceIndex;
    NetworkAddress nextHopAddress;
    theNetworkLayer.GetNextHopAddressAndInterfaceIndexForDestination(
        destinationAddress, success, nextHopAddress, interfaceIndex);

    if ((success) && (theNetworkLayer.MacSupportsQualityOfService(interfaceIndex))) {

        MacQualityOfServiceControlInterface& qosControlInterface =
            *theNetworkLayer.GetMacQualityOfServiceInterface(interfaceIndex);

        qosControlInterface.DeleteFlow(macQosFlowId);

    }//if//

}//UnreserveBandwidth//



inline
void CbrSinkApplication::ReserveOrUnreserveBandwidth()
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (currentTime < cbrEndTime) {
        (*this).ReserveBandwidth();
    }
    else {
        (*this).UnreserveBandwidth();
    }//if//
}


}//namespace//

#endif
