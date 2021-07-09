// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_APP_FTP_H
#define SCENSIM_APP_FTP_H

#include "scensim_netsim.h"

namespace ScenSim {

using std::cerr;
using std::endl;


//--------------------------------------------------------------------------------------------------

class FtpLikeFlowApplication: public Application {
public:
    static const string modelName;

    typedef MacQualityOfServiceControlInterface::SchedulingSchemeChoice SchedulingSchemeChoice;
    typedef MacQualityOfServiceControlInterface::ReservationSchemeChoice ReservationSchemeChoice;

    FtpLikeFlowApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationId& initApplicationId,
        const NodeId& initSourceNodeId,
        const NodeId& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl);

protected:

    NodeId sourceNodeId;
    NodeId destinationNodeId;
    unsigned short int sourcePortId;
    unsigned short int destinationPortId;
    SimTime flowStartTime;
    SimTime flowEndTime;
    unsigned long long int flowTotalBytes;
    PacketPriority flowPriority;
    SimTime maxStartTimeJitter;

    SchedulingSchemeChoice schedulingScheme;
    ReservationSchemeChoice reservationScheme;

    bool reserveBandwidthModeIsOn;
    double qosMinBandwidth;
    double qosMaxBandwidth;

    // For reservations of reverse channel (ACKs)

    double qosMinReverseBandwidth;
    double qosMaxReverseBandwidth;

    FlowId macQosFlowId;

    bool useVirtualPayload;

    string GetParameterNamePrefix() const {
        if (reserveBandwidthModeIsOn) {
            return  "ftp-with-qos";
        }//if//
        return "ftp";
    }
};//FtpLikeFlowSourceApplication//

inline
FtpLikeFlowApplication::FtpLikeFlowApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationId& initApplicationId,
    const NodeId& initSourceNodeId,
    const NodeId& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl)
    :
    Application(initSimulationEngineInterfacePtr, initApplicationId),
    sourceNodeId(initSourceNodeId),
    sourcePortId(initDefaultApplicationPortId),
    destinationNodeId(initDestinationNodeId),
    destinationPortId(initDefaultApplicationPortId),
    flowStartTime(ZERO_TIME),
    flowEndTime(ZERO_TIME),
    flowTotalBytes(0),
    flowPriority(0),
    maxStartTimeJitter(ZERO_TIME),
    reserveBandwidthModeIsOn(initEnableQosControl),
    qosMinBandwidth(0.0),
    qosMaxBandwidth(0.0),
    qosMinReverseBandwidth(0.0),
    qosMaxReverseBandwidth(0.0),
    schedulingScheme(MacQualityOfServiceControlInterface::DefaultSchedulingScheme),
    reservationScheme(MacQualityOfServiceControlInterface::OptimisticLinkRate),
    useVirtualPayload(false)
{
    const string parameterPrefix = (*this).GetParameterNamePrefix();

    flowTotalBytes =
        parameterDatabaseReader.ReadBigInt(
            parameterPrefix + "-flow-size-bytes", sourceNodeId, theApplicationId);

    flowStartTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-flow-start-time", sourceNodeId, theApplicationId);

    flowEndTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-flow-end-time", sourceNodeId, theApplicationId);

    flowPriority = static_cast<PacketPriority>(
        parameterDatabaseReader.ReadInt(
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
            sourcePortId = destinationPortId;

        }//if//
    }//if//

    if (initEnableQosControl) {
        qosMinBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-baseline-bandwidth-bytes", sourceNodeId, theApplicationId);

        qosMaxBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-max-bandwidth-bytes", sourceNodeId, theApplicationId);

        qosMinReverseBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-baseline-reverse-bandwidth-bytes", sourceNodeId, theApplicationId);

        qosMaxReverseBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-max-reverse-bandwidth-bytes", sourceNodeId, theApplicationId);

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

}//FtpLikeFlowApplication//

class FtpLikeFlowSourceApplication:
    public FtpLikeFlowApplication, public enable_shared_from_this<FtpLikeFlowSourceApplication> {
public:
    FtpLikeFlowSourceApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationId& initApplicationId,
        const NodeId& initSourceNodeId,
        const NodeId& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl);

    void CompleteInitialization();

    void DisconnectFromOtherLayers();

private:
    static const unsigned int tcpFlowBlockSize = 2048;

    void StartFlow();
    void SendDataIfNecessary();

    //----------------------------------

    class StartFlowEvent: public SimulationEvent {
    public:
        explicit
        StartFlowEvent(const shared_ptr<FtpLikeFlowSourceApplication>& initFtpLikeFlowApplicationPtr)
            : ftpLikeFlowApplicationPtr(initFtpLikeFlowApplicationPtr) {}

        virtual void ExecuteEvent() { ftpLikeFlowApplicationPtr->StartFlow(); }

    private:
        shared_ptr<FtpLikeFlowSourceApplication> ftpLikeFlowApplicationPtr;

    };//StartFtpLikeFlowEvent//

    class TcpEventHandler: public TcpConnection::AppTcpEventHandler {
    public:
        TcpEventHandler(const shared_ptr<FtpLikeFlowSourceApplication> initFtpLikeSourceAppPtr)
            : ftpLikeSourceAppPtr(initFtpLikeSourceAppPtr) {}

        void DoTcpIsReadyForMoreDataAction() { ftpLikeSourceAppPtr->SendDataIfNecessary(); }
        void ReceiveDataBlock(
            const unsigned char dataBlock[],
            const unsigned int dataLength,
            const unsigned int actualDataLength,
            bool& stallIncomingDataFlow)
            { assert(false && "App does not receive data."); abort(); }

    private:
        shared_ptr<FtpLikeFlowSourceApplication> ftpLikeSourceAppPtr;

    };//SendCompletionHandler//

    //----------------------------------

    unsigned long long int bytesSentSoFar;
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<RealStatistic> transmissionDelayStatPtr;

    void OutputTraceAndStatsStartFlow(
        const unsigned long long int& flowBytes);

    void OutputTraceAndStatsForEndFlow(
        const SimTime& transmissionDelay,
        const unsigned long long int& flowReceivedBytes);

    shared_ptr<TcpConnection> tcpConnectionPtr;
    shared_ptr<TcpEventHandler> tcpEventHandlerPtr;

    class FlowRequestReplyFielder: public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<FtpLikeFlowSourceApplication>& initAppPtr) : appPtr(initAppPtr) { }

        void RequestAccepted(const FlowId& theFlowId)
            { appPtr->macQosFlowId = theFlowId; }
        void RequestDenied() { appPtr->ReserveBandwidthRequestDeniedAction(); }
    private:
        shared_ptr<FtpLikeFlowSourceApplication> appPtr;

    };//FlowRequestReplyFielder//

    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();

};//FtpLikeFlowSourceApplication//




inline
FtpLikeFlowSourceApplication::FtpLikeFlowSourceApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationId& initApplicationId,
    const NodeId& initSourceNodeId,
    const NodeId& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl)
    :
    FtpLikeFlowApplication(
        parameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initApplicationId,
        initSourceNodeId,
        initDestinationNodeId,
        initDefaultApplicationPortId,
        initEnableQosControl),
    bytesSentSoFar(0),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_BytesSent"))),
    transmissionDelayStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_" + initApplicationId + "_TransmissionDelay")))
{
}//FtpLikeFlowSourceApplication//

// Two part initialization forced by shared_from_this().

inline
void FtpLikeFlowSourceApplication::CompleteInitialization()
{
    const string parameterPrefix = (*this).GetParameterNamePrefix();

    if (maxStartTimeJitter != ZERO_TIME) {
        flowStartTime += static_cast<SimTime>(
            aRandomNumberGeneratorPtr->GenerateRandomDouble() * maxStartTimeJitter);
    }//if//

    tcpEventHandlerPtr.reset(new TcpEventHandler(shared_from_this()));

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (currentTime <= flowStartTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new StartFlowEvent(shared_from_this())),
            flowStartTime);
    }

}//CompleteInitialization//

inline
void FtpLikeFlowSourceApplication::DisconnectFromOtherLayers()
{
    if (tcpConnectionPtr != nullptr) {
        tcpConnectionPtr->ClearAppTcpEventHandler();
        (*this).tcpConnectionPtr.reset();
    }//if//
    tcpEventHandlerPtr.reset();

    (*this).Application::DisconnectFromOtherLayers();

}//DisconnectFromOtherLayers//


inline
void FtpLikeFlowSourceApplication::StartFlow()
{
    if (reserveBandwidthModeIsOn) {
        (*this).ReserveBandwidth();
    }

    OutputTraceAndStatsStartFlow(flowTotalBytes);

    const NetworkAddress sourceAddress =
        networkAddressLookupInterfacePtr->LookupNetworkAddress(sourceNodeId);

    NetworkAddress destinationAddress;
    bool foundDestAddress;
    networkAddressLookupInterfacePtr->LookupNetworkAddress(
        destinationNodeId, destinationAddress, foundDestAddress);

    if (foundDestAddress) {
        transportLayerPtr->tcpPtr->CreateOutgoingTcpConnection(
            sourceAddress, sourcePortId, destinationAddress, destinationPortId, flowPriority,
            tcpEventHandlerPtr, tcpConnectionPtr);

        if (useVirtualPayload) {
            tcpConnectionPtr->EnableVirtualPayload();
        }//if//

    }
    else {
        //cannot find destination address (destination node may not be created yet)
        //Future: output trace and stat
    }//if//

}//StartFlow//




inline
void FtpLikeFlowSourceApplication::SendDataIfNecessary()
{
    using std::min;

    while ((bytesSentSoFar < flowTotalBytes) &&
           (tcpConnectionPtr->GetCurrentNumberOfAvailableBufferBytes() >= tcpFlowBlockSize)) {

        unsigned int blockSizeBytes = tcpFlowBlockSize;

        if ((flowTotalBytes - bytesSentSoFar) < tcpFlowBlockSize) {
            blockSizeBytes = static_cast<unsigned int>(flowTotalBytes - bytesSentSoFar);
        }//if//

        shared_ptr<vector<unsigned char> > dataBlockPtr(new vector<unsigned char>());

        if (useVirtualPayload) {
            bytesSentSoFar += blockSizeBytes;
        }
        else {
            dataBlockPtr->resize(blockSizeBytes);

            for(unsigned int i = 0; (i < blockSizeBytes); i++) {

                if (bytesSentSoFar % 2 == 0) {
                    (*dataBlockPtr)[i] = static_cast<unsigned char>((bytesSentSoFar+1)/256);
                }
                else {
                    (*dataBlockPtr)[i] = static_cast<unsigned char>(bytesSentSoFar);
                }//if//

                bytesSentSoFar++;

            }//for//
        }//if//

        bytesSentStatPtr->IncrementCounter(blockSizeBytes);

        tcpConnectionPtr->SendDataBlock(dataBlockPtr, blockSizeBytes);

        if (bytesSentSoFar >= flowTotalBytes) {
            tcpConnectionPtr->Close();
            (*this).tcpConnectionPtr.reset();

            if (reserveBandwidthModeIsOn) {
               // TBD Delay Unreserve.
               (*this).UnreserveBandwidth();
            }//if//

            const SimTime transmissionDelay =
                simulationEngineInterfacePtr->CurrentTime() - flowStartTime;
            OutputTraceAndStatsForEndFlow(transmissionDelay, flowTotalBytes);

        }//if//
    }//while//

}//SendDataIfNecessary//



inline
void FtpLikeFlowSourceApplication::OutputTraceAndStatsStartFlow(
    const unsigned long long int& flowBytes)
{

    // Trace
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {
            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theApplicationId, "FtpStartFlow", flowBytes);
        }
        else {

            ostringstream outStream;

            outStream << "FileBytes= " << flowBytes;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "FtpStartFlow", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsStartFlow//


inline
void FtpLikeFlowSourceApplication::OutputTraceAndStatsForEndFlow(
    const SimTime& transmissionDelay,
    const unsigned long long int& flowDeliveredBytes)
{

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

        ApplicationEndFlowTraceRecord traceData;

        traceData.transmissionDelay = transmissionDelay;
        traceData.flowDeliveredBytes = flowDeliveredBytes;

        assert(sizeof(traceData) == APPLICATION_END_FLOW_TRACE_RECORD_BYTES);

        simulationEngineInterfacePtr->OutputTraceInBinary(
            modelName, theApplicationId, "FtpEndFlow", traceData);


        }
        else {
            ostringstream outStream;

            outStream << "DeliveredBytes= " << flowDeliveredBytes
                      << " Delay= " << ConvertTimeToStringSecs(transmissionDelay);

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "FtpEndFlow", outStream.str());
        }//if//
    }//if//

    transmissionDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(transmissionDelay));

}//OutputTraceAndStatsForFlowEnd//


inline
void FtpLikeFlowSourceApplication::ReserveBandwidth()
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

        qosControlInterface.RequestDualFlowReservation(
            reservationScheme,
            schedulingScheme,
            flowPriority,
            qosMinReverseBandwidth, qosMaxReverseBandwidth,
            qosMinBandwidth, qosMaxBandwidth,
            destinationAddress, destinationPortId, sourceAddress, sourcePortId, IP_PROTOCOL_NUMBER_TCP,
            replyPtr);
    }//if//

}//ReserveBandwidth//



inline
void FtpLikeFlowSourceApplication::ReserveBandwidthRequestDeniedAction()
{
    cerr << "Warning in FTP with QoS application: Bandwidth request denied." << endl;
}


inline
void FtpLikeFlowSourceApplication::UnreserveBandwidth()
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

class FtpLikeFlowSinkApplication:
    public FtpLikeFlowApplication, public enable_shared_from_this<FtpLikeFlowSinkApplication> {
public:
    FtpLikeFlowSinkApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationId& initApplicationId,
        const NodeId& initSourceNodeId,
        const NodeId& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl);

    void CompleteInitialization();

    void DisconnectFromOtherLayers();
private:
    class FlowReservationEvent: public SimulationEvent {
    public:
        explicit
        FlowReservationEvent(const shared_ptr<FtpLikeFlowSinkApplication>& initFtpApplicationPtr)
            : ftpApplicationPtr(initFtpApplicationPtr) {}
        virtual void ExecuteEvent() { ftpApplicationPtr->ReserveOrUnreserveBandwidth(); }

    private:
        shared_ptr<FtpLikeFlowSinkApplication> ftpApplicationPtr;

    };//FlowReservationEvent//

    class ConnectionHandler: public ConnectionFromTcpProtocolHandler {
    public:
        ConnectionHandler(const shared_ptr<FtpLikeFlowSinkApplication>& initSinkAppPtr)
            : sinkApplicationPtr(initSinkAppPtr) {}

        void HandleNewConnection(const shared_ptr<TcpConnection>& connectionPtr)
        {
            sinkApplicationPtr->AcceptTcpConnection(connectionPtr);
        }
    private:
        shared_ptr<FtpLikeFlowSinkApplication> sinkApplicationPtr;

    };//ConnectionHandler//

    //-------------------------------------------

    class TcpEventHandler: public TcpConnection::AppTcpEventHandler {
    public:
        TcpEventHandler(const shared_ptr<FtpLikeFlowSinkApplication>& initSinkAppPtr)
            : sinkApplicationPtr(initSinkAppPtr) {}

        void DoTcpIsReadyForMoreDataAction() { }

        void ReceiveDataBlock(
            const unsigned char dataBlock[],
            const unsigned int dataLength,
            const unsigned int actualDataLength,
            bool& stallIncomingDataFlow)
        {
            sinkApplicationPtr->ReceiveData(
                dataBlock, dataLength, actualDataLength);
            stallIncomingDataFlow = false;
        }

        void DoTcpRemoteHostClosedAction() {
            sinkApplicationPtr->DoTcpRemoteHostClosedAction();
        }

    private:
        shared_ptr<FtpLikeFlowSinkApplication> sinkApplicationPtr;

    };//TcpEventHandler//

    //-------------------------------------------------------------------------

    void DoTcpRemoteHostClosedAction() {
        tcpConnectionPtr->Close();
        (*this).tcpConnectionPtr.reset();
    }

    void AcceptTcpConnection(const shared_ptr<TcpConnection>& connectionPtr);

    void ReceiveData(
        const unsigned char dataBlock[],
        const unsigned int dataLength,
        const unsigned int actualDataLength);

    void OutputTraceAndStatsForReceiveData(
        const unsigned int length);

    shared_ptr<TcpConnection> tcpConnectionPtr;

    unsigned long long int numBytesReceived;

    shared_ptr<CounterStatistic> bytesReceivedStatPtr;

    class FlowRequestReplyFielder: public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<FtpLikeFlowSinkApplication>& initAppPtr) : appPtr(initAppPtr) { }

        void RequestAccepted(const FlowId& theFlowId)
            { appPtr->macQosFlowId = theFlowId; }
        void RequestDenied() { appPtr->ReserveBandwidthRequestDeniedAction(); }
    private:
        shared_ptr<FtpLikeFlowSinkApplication> appPtr;

    };//FlowRequestReplyFielder//

    void ReserveOrUnreserveBandwidth();
    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();

};//FtpLikeFlowSinkApplication//


inline
FtpLikeFlowSinkApplication::FtpLikeFlowSinkApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationId& initApplicationId,
    const NodeId& initSourceNodeId,
    const NodeId& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl)
    :
    FtpLikeFlowApplication(
        parameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initApplicationId,
        initSourceNodeId,
        initDestinationNodeId,
        initDefaultApplicationPortId,
        initEnableQosControl),
    numBytesReceived(0),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_BytesReceived")))
{
}

// Two part initialized caused by shared_from_this() limitation

inline
void FtpLikeFlowSinkApplication::CompleteInitialization()
{
    shared_ptr<ConnectionHandler> connectionHandlerPtr(new ConnectionHandler(shared_from_this()));

    assert(transportLayerPtr->tcpPtr->PortIsAvailable(destinationPortId));

    transportLayerPtr->tcpPtr->OpenSpecificTcpPort(
        NetworkAddress::anyAddress,
        destinationPortId,
        connectionHandlerPtr);

    if (reserveBandwidthModeIsOn) {

        const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
        const SimTime minimumSetupTime = 1 * MILLI_SECOND;
        const SimTime reservationLeewayTime = 1000 * MILLI_SECOND;

        SimTime reservationTime;
        if ((currentTime + minimumSetupTime + reservationLeewayTime) <= flowStartTime) {
            reservationTime = (flowStartTime - reservationLeewayTime);
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
            (flowEndTime + reservationLeewayTime));

    }//if//
}//CompleteInitialization//



inline
void FtpLikeFlowSinkApplication::DisconnectFromOtherLayers()
{
    transportLayerPtr->tcpPtr->DisconnectConnectionHandlerForPort(
        NetworkAddress::anyAddress, destinationPortId);

    if (tcpConnectionPtr != nullptr) {
        tcpConnectionPtr->ClearAppTcpEventHandler();
        (*this).tcpConnectionPtr.reset();
    }//if//

    (*this).Application::DisconnectFromOtherLayers();

}//DisconnectFromOtherLayers//




inline
void FtpLikeFlowSinkApplication::AcceptTcpConnection(const shared_ptr<TcpConnection>& connectionPtr)
{
    assert((tcpConnectionPtr == nullptr) && "Should only be one connection");

    tcpConnectionPtr = connectionPtr;

    shared_ptr<TcpEventHandler> tcpEventHandlerPtr(new TcpEventHandler(shared_from_this()));

    tcpConnectionPtr->SetAppTcpEventHandler(tcpEventHandlerPtr);
    tcpConnectionPtr->SetPacketPriority(flowPriority);

    if (useVirtualPayload) {
        tcpConnectionPtr->EnableVirtualPayload();
    }//if//
}


inline
void FtpLikeFlowSinkApplication::OutputTraceAndStatsForReceiveData(const unsigned int length)
{
    bytesReceivedStatPtr->IncrementCounter(length);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationReceiveDataTraceRecord traceData;

            traceData.sourceNodeId = sourceNodeId;
            traceData.dataLengthBytes = length;

            assert(sizeof(traceData) == APPLICATION_RECEIVE_DATA_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theApplicationId, "FtpRecv", traceData);
        }
        else {
            ostringstream outStream;

            outStream << "SrcN= " << sourceNodeId
                      << " RecvBytes= " << length;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "FtpRecv", outStream.str());

        }//if//
    }//if//
}


inline
void FtpLikeFlowSinkApplication::ReceiveData(
    const unsigned char dataBlock[],
    const unsigned int dataLength,
    const unsigned int actualDataLength)
{
    for(size_t i = 0; (i < actualDataLength); i++) {

        if (numBytesReceived % 2 == 0) {
            assert(dataBlock[i] == static_cast<unsigned char>(((numBytesReceived+1)/256)));
        }
        else {
            assert(dataBlock[i] == static_cast<unsigned char>(numBytesReceived));
        }//if//

        numBytesReceived++;

    }//for//

    OutputTraceAndStatsForReceiveData(dataLength);
}




inline
void FtpLikeFlowSinkApplication::ReserveBandwidth()
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

        qosControlInterface.RequestDualFlowReservation(
            reservationScheme,
            schedulingScheme,
            flowPriority,
            qosMinBandwidth, qosMaxBandwidth,
            qosMinReverseBandwidth, qosMaxReverseBandwidth,
            sourceAddress, sourcePortId, destinationAddress, destinationPortId, IP_PROTOCOL_NUMBER_TCP,
            replyPtr);
    }//if//

}//ReserveBandwidth//


inline
void FtpLikeFlowSinkApplication::ReserveBandwidthRequestDeniedAction()
{
    cerr << "Warning in FTP with QoS application: Bandwidth request denied." << endl;
}


inline
void FtpLikeFlowSinkApplication::UnreserveBandwidth()
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
void FtpLikeFlowSinkApplication::ReserveOrUnreserveBandwidth()
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (currentTime < flowEndTime) {
        (*this).ReserveBandwidth();
    }
    else {
        (*this).UnreserveBandwidth();
    }//if//
}



//--------------------------------------------------------------------------------------------------

//
// ----- File Transfer Protocol Model -----
//
// Refer to "IEEE802.16m Evaluation Methodology"
//

class MultipleFtpApplication: public Application {
public:
    static const string modelName;

    typedef MacQualityOfServiceControlInterface::SchedulingSchemeChoice SchedulingSchemeChoice;
    typedef MacQualityOfServiceControlInterface::ReservationSchemeChoice ReservationSchemeChoice;

    MultipleFtpApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationId& initApplicationId,
        const NodeId& initSourceNodeId,
        const NodeId& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl);

protected:
    NodeId sourceNodeId;
    NodeId destinationNodeId;
    unsigned short int sourcePortId;
    unsigned short int destinationPortId;
    unsigned long long int maximumFlowDataBytes;

    PacketPriority flowPriority;
    SimTime maxStartTimeJitter;

    SimTime flowStartTime;
    SimTime flowEndTime;
    unsigned long long int flowTotalBytes;

    //----------------------------------

    SchedulingSchemeChoice schedulingScheme;
    ReservationSchemeChoice reservationScheme;

    bool reserveBandwidthModeIsOn;
    double qosMinBandwidth;
    double qosMaxBandwidth;

    // For reservations of reverse channel (ACKs)

    double qosMinReverseBandwidth;
    double qosMaxReverseBandwidth;

    FlowId macQosFlowId;

    bool useVirtualPayload;

    string GetParameterNamePrefix() const {
        if (reserveBandwidthModeIsOn) {
            return  "multiftp-with-qos";
        }//if//
        return "multiftp";
    }

};//MultipleFtpApplication//


inline
MultipleFtpApplication::MultipleFtpApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationId& initApplicationId,
    const NodeId& initSourceNodeId,
    const NodeId& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl)
    :
    Application(initSimulationEngineInterfacePtr, initApplicationId),
    sourceNodeId(initSourceNodeId),
    sourcePortId(initDefaultApplicationPortId),
    destinationNodeId(initDestinationNodeId),
    destinationPortId(initDefaultApplicationPortId),
    maximumFlowDataBytes(0),
    flowStartTime(ZERO_TIME),
    flowEndTime(ZERO_TIME),
    flowTotalBytes(0),
    flowPriority(0),
    maxStartTimeJitter(ZERO_TIME),
    reserveBandwidthModeIsOn(initEnableQosControl),
    qosMinBandwidth(0.0),
    qosMaxBandwidth(0.0),
    qosMinReverseBandwidth(0.0),
    qosMaxReverseBandwidth(0.0),
    schedulingScheme(MacQualityOfServiceControlInterface::DefaultSchedulingScheme),
    reservationScheme(MacQualityOfServiceControlInterface::OptimisticLinkRate),
    useVirtualPayload(false)
{
    const string parameterPrefix = (*this).GetParameterNamePrefix();

    maximumFlowDataBytes =
        parameterDatabaseReader.ReadBigInt(
            parameterPrefix + "-max-flow-data-bytes", sourceNodeId, theApplicationId);

    flowStartTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-start-time", sourceNodeId, theApplicationId);

    flowEndTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-end-time", sourceNodeId, theApplicationId);

    flowPriority = static_cast<PacketPriority>(
        parameterDatabaseReader.ReadInt(
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
            sourcePortId = destinationPortId;

        }//if//
    }//if//

    if (initEnableQosControl) {
        qosMinBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-baseline-bandwidth-bytes", sourceNodeId, theApplicationId);

        qosMaxBandwidth =
            parameterDatabaseReader.ReadDouble(parameterPrefix + "-max-bandwidth-bytes", sourceNodeId, theApplicationId);

        qosMinReverseBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-baseline-reverse-bandwidth-bytes", sourceNodeId, theApplicationId);

        qosMaxReverseBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-max-reverse-bandwidth-bytes", sourceNodeId, theApplicationId);

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

}//MultipleFtpApplication//


class MultipleFtpSourceApplication:
    public MultipleFtpApplication, public enable_shared_from_this<MultipleFtpSourceApplication> {
public:
    MultipleFtpSourceApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationId& initApplicationId,
        const NodeId& initSourceNodeId,
        const NodeId& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl,
        const RandomNumberGeneratorSeed& initNodeSeed);

    void CompleteInitialization();

    void DisconnectFromOtherLayers();

private:

    static const int SEED_HASH = 64354578;

    static const double maximumTcpFlowBlockSizeRate;
    static const unsigned long long int minimumTcpFlowBlockSize = 576;
    static const unsigned long long int maximumTcpFlowBlockSize = 1500;


    unsigned long long int GetFlowTotalBytes();
    SimTime GetReadingTime();

    void StartFlow();
    void SendDataIfNecessary();

    //----------------------------------

    class StartFlowEvent: public SimulationEvent {
    public:
        explicit
        StartFlowEvent(const shared_ptr<MultipleFtpSourceApplication>& initMultipleFtpApplicationPtr)
            : multipleFtpApplicationPtr(initMultipleFtpApplicationPtr) {}

        virtual void ExecuteEvent() { multipleFtpApplicationPtr->StartFlow(); }

    private:
        shared_ptr<MultipleFtpSourceApplication> multipleFtpApplicationPtr;

    };//StartFlowEvent//


    class TcpEventHandler: public TcpConnection::AppTcpEventHandler {
    public:
        TcpEventHandler(const shared_ptr<MultipleFtpSourceApplication> initMultipleFtpSourceAppPtr)
            : multipleFtpSourceAppPtr(initMultipleFtpSourceAppPtr) {}

        void DoTcpIsReadyForMoreDataAction()
            { multipleFtpSourceAppPtr->SendDataIfNecessary(); }
        void ReceiveDataBlock(
            const unsigned char dataBlock[],
            const unsigned int dataLength,
            const unsigned int actualDataLength,
            bool& stallIncomingDataFlow)
            { assert(false && "App does not receive data."); abort(); }

    private:
        shared_ptr<MultipleFtpSourceApplication> multipleFtpSourceAppPtr;

    };//SendCompletionHandler//


    //----------------------------------

    SimTime currentFlowStartTime;
    unsigned long long int currentFlowTotalBytes;
    unsigned int tcpFlowBlockSizeBytes;
    unsigned long long int bytesSentSoFar;

    shared_ptr<TcpConnection> tcpConnectionPtr;
    shared_ptr<TcpEventHandler> tcpEventHandlerPtr;

    //----------------------------------

    class FlowRequestReplyFielder: public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<MultipleFtpSourceApplication>& initAppPtr) : appPtr(initAppPtr) { }

        void RequestAccepted(const FlowId& theFlowId)
            { appPtr->macQosFlowId = theFlowId; }
        void RequestDenied() { appPtr->ReserveBandwidthRequestDeniedAction(); }
    private:
        shared_ptr<MultipleFtpSourceApplication> appPtr;

    };//FlowRequestReplyFielder//

    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();

    //----------------------------------

    // For trace and stats
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<RealStatistic> transmissionDelayStatPtr;

    void OutputTraceAndStatsStartFlow(
        const unsigned long long int& flowBytes);

    void OutputTraceAndStatsForEndFlow(
        const SimTime& transmissionDelay,
        const unsigned long long int& flowReceivedBytes);

    // aRamdomNumberGenerator for Lognormal Distribution.
    shared_ptr<boost::variate_generator<boost::mt19937, boost::lognormal_distribution<> > > aRandomNumberGeneratorForLognormalPtr;

    // aRandomNumberGenerator for Exponential Distribution.
    double lambda;
    RandomNumberGenerator aRandomNumberGeneratorForExponential;

    // aRandomNumberGenerator for Uniform Distribution.
    RandomNumberGenerator aRandomNumberGenerator;

};//MultipleFtpSourceApplication//

inline
MultipleFtpSourceApplication::MultipleFtpSourceApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationId& initApplicationId,
    const NodeId& initSourceNodeId,
    const NodeId& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl,
    const RandomNumberGeneratorSeed& initNodeSeed)
    :
    MultipleFtpApplication(
        parameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initApplicationId,
        initSourceNodeId,
        initDestinationNodeId,
        initDefaultApplicationPortId,
        initEnableQosControl),
    currentFlowStartTime(ZERO_TIME),
    currentFlowTotalBytes(0),
    tcpFlowBlockSizeBytes(0),
    bytesSentSoFar(0),
    lambda(0.),
    aRandomNumberGeneratorForExponential(
        HashInputsToMakeSeed(initNodeSeed, initApplicationId, SEED_HASH)),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(initNodeSeed, initApplicationId, SEED_HASH)),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_BytesSent"))),
    transmissionDelayStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_" + initApplicationId + "_TransmissionDelay")))
{
    const string parameterPrefix = (*this).GetParameterNamePrefix();

    const SimTime meanReadingTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-mean-reading-time", sourceNodeId, theApplicationId);

    lambda = 1.0/ConvertTimeToDoubleSecs(meanReadingTime);

    const unsigned long long int meanFlowDataBytes =
        parameterDatabaseReader.ReadBigInt(
            parameterPrefix + "-mean-flow-data-bytes", sourceNodeId, theApplicationId);

    const unsigned long long int standardDeviationFlowDataBytes =
        parameterDatabaseReader.ReadBigInt(
            parameterPrefix + "-standard-deviation-flow-data-bytes", sourceNodeId, theApplicationId);

    aRandomNumberGeneratorForLognormalPtr.reset(
        new boost::variate_generator<boost::mt19937, boost::lognormal_distribution<> > (
            boost::mt19937(static_cast<unsigned long>(HashInputsToMakeSeed(initNodeSeed, theApplicationId, SEED_HASH))),
            boost::lognormal_distribution<>(
                static_cast<double>(meanFlowDataBytes),
                static_cast<double>(standardDeviationFlowDataBytes))));

}//MultipleFtpSourceApplication//

// Two part initialized caused by shared_from_this() limitation

inline
void MultipleFtpSourceApplication::CompleteInitialization()
{
    const string parameterPrefix = (*this).GetParameterNamePrefix();

    if (maxStartTimeJitter != ZERO_TIME) {
        flowStartTime += static_cast<SimTime>(
            aRandomNumberGeneratorPtr->GenerateRandomDouble() * maxStartTimeJitter);
    }//if//

    tcpEventHandlerPtr.reset(new TcpEventHandler(shared_from_this()));

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
    const SimTime minimumSetupTime = 1 * MILLI_SECOND;

    if ((currentTime + minimumSetupTime) <= flowStartTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new StartFlowEvent(shared_from_this())), flowStartTime);
    }
    else {
        //schedule next flow
        const SimTime nextFlowStartTime =
            simulationEngineInterfacePtr->CurrentTime() + minimumSetupTime + (*this).GetReadingTime();

        if (nextFlowStartTime < flowEndTime) {
            simulationEngineInterfacePtr->ScheduleEvent(
                unique_ptr<SimulationEvent>(new StartFlowEvent(shared_from_this())),
                nextFlowStartTime);
        }//if//

    }//if//
}

inline
void MultipleFtpSourceApplication::DisconnectFromOtherLayers()
{

    if (tcpConnectionPtr != nullptr) {
        tcpConnectionPtr->ClearAppTcpEventHandler();
        (*this).tcpConnectionPtr.reset();
    }//if//
    (*this).tcpEventHandlerPtr.reset();

    (*this).Application::DisconnectFromOtherLayers();

}//DisconnectFromOtherLayers//



inline
unsigned long long int MultipleFtpSourceApplication::GetFlowTotalBytes()
{
    unsigned long long int result;

    while(true) {

        const double randomNumber = (*aRandomNumberGeneratorForLognormalPtr)();
        result = static_cast<unsigned long long int>(randomNumber);

        if (result >= 0 && result <= maximumFlowDataBytes) {
            break;
        }

    }//while//

    return result;

}//GetFlowBlockSizeBytes//


inline
SimTime MultipleFtpSourceApplication::GetReadingTime()
{
    const double randomNumber = aRandomNumberGeneratorForExponential.GenerateRandomDouble();

    return ConvertDoubleSecsToTime(-log(1.0 - randomNumber) / lambda);

}//GetReadingTime//



inline
void MultipleFtpSourceApplication::StartFlow()
{
    currentFlowStartTime = simulationEngineInterfacePtr->CurrentTime();

    currentFlowTotalBytes = (*this).GetFlowTotalBytes();
    flowTotalBytes += currentFlowTotalBytes;

    const double randomNumber = aRandomNumberGenerator.GenerateRandomDouble();
    if (randomNumber <= maximumTcpFlowBlockSizeRate) {
        tcpFlowBlockSizeBytes = maximumTcpFlowBlockSize;
    }
    else {
        tcpFlowBlockSizeBytes = minimumTcpFlowBlockSize;
    }//if//

    OutputTraceAndStatsStartFlow(currentFlowTotalBytes);

    if (tcpConnectionPtr == nullptr) {

        if (reserveBandwidthModeIsOn) {
            (*this).ReserveBandwidth();
        }

        const NetworkAddress sourceAddress =
            networkAddressLookupInterfacePtr->LookupNetworkAddress(sourceNodeId);

        NetworkAddress destinationAddress;
        bool foundDestAddress;
        networkAddressLookupInterfacePtr->LookupNetworkAddress(
            destinationNodeId, destinationAddress, foundDestAddress);

        if (foundDestAddress) {
            transportLayerPtr->tcpPtr->CreateOutgoingTcpConnection(
                sourceAddress, sourcePortId, destinationAddress, destinationPortId, flowPriority,
                tcpEventHandlerPtr, tcpConnectionPtr);

            if (useVirtualPayload) {
                tcpConnectionPtr->EnableVirtualPayload();
            }//if//
        }
        else {
            //cannot find destination address (destination node may not be created yet)
            //Future: output trace and stat

            //schedule next flow
            const SimTime nextFlowStartTime =
                simulationEngineInterfacePtr->CurrentTime() + (*this).GetReadingTime();

            if (nextFlowStartTime < flowEndTime) {
                simulationEngineInterfacePtr->ScheduleEvent(
                    unique_ptr<SimulationEvent>(new StartFlowEvent(shared_from_this())),
                    nextFlowStartTime);
            }
            else {
                if (reserveBandwidthModeIsOn) {
                    // TBD Delay Unreserve.
                    (*this).UnreserveBandwidth();
                }//if//
            }//if//
        }//if//
    }
    else {
        SendDataIfNecessary();
    }

}//StartFlow//


inline
void MultipleFtpSourceApplication::SendDataIfNecessary()
{
    while ((bytesSentSoFar < flowTotalBytes) &&
           (tcpConnectionPtr->GetCurrentNumberOfAvailableBufferBytes() >= tcpFlowBlockSizeBytes)) {

        unsigned int blockSizeBytes = static_cast<unsigned int>(tcpFlowBlockSizeBytes);

        if ((flowTotalBytes - bytesSentSoFar) < tcpFlowBlockSizeBytes) {
            blockSizeBytes = static_cast<unsigned int>(flowTotalBytes - bytesSentSoFar);
        }//if//

        shared_ptr<vector<unsigned char> > dataBlockPtr(new vector<unsigned char>());

        bytesSentSoFar += blockSizeBytes;

        if (!useVirtualPayload) {
            dataBlockPtr->resize(blockSizeBytes);
            for(size_t i = 0; (i < blockSizeBytes); i++) {
                (*dataBlockPtr)[i] = static_cast<unsigned char>(bytesSentSoFar/tcpFlowBlockSizeBytes);
            }//for//
        }//if//

        bytesSentStatPtr->IncrementCounter(blockSizeBytes);

        tcpConnectionPtr->SendDataBlock(dataBlockPtr, blockSizeBytes);

    }//while//


    if (tcpConnectionPtr->GetNumberOfDeliveredBytes() >= flowTotalBytes) {

        const SimTime transmissionDelay =
            simulationEngineInterfacePtr->CurrentTime() - currentFlowStartTime;
        unsigned long long int currentFlowDeliveredBytes =
            currentFlowTotalBytes + (tcpConnectionPtr->GetNumberOfDeliveredBytes() - flowTotalBytes);

        OutputTraceAndStatsForEndFlow(transmissionDelay, currentFlowDeliveredBytes);

        const SimTime nextFlowStartTime =
            simulationEngineInterfacePtr->CurrentTime() + (*this).GetReadingTime();

        if (nextFlowStartTime < flowEndTime) {
            simulationEngineInterfacePtr->ScheduleEvent(
                unique_ptr<SimulationEvent>(new StartFlowEvent(shared_from_this())),
                nextFlowStartTime);
        }
        else {
            if (reserveBandwidthModeIsOn) {
                // TBD Delay Unreserve.
                (*this).UnreserveBandwidth();
            }//if//
        }//if//
    }//if//

}//SendDataIfNecessary//


inline
void MultipleFtpSourceApplication::OutputTraceAndStatsStartFlow(
    const unsigned long long int& flowBytes)
{

    // Trace
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {
            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theApplicationId, "MultiFtpStartFlow", flowBytes);
        }
        else {

            ostringstream outStream;

            outStream << "FileBytes= " << flowBytes;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "MultiFtpStartFlow", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsStartFlow//


inline
void MultipleFtpSourceApplication::OutputTraceAndStatsForEndFlow(
    const SimTime& transmissionDelay,
    const unsigned long long int& flowDeliveredBytes)
{

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

        ApplicationEndFlowTraceRecord traceData;

        traceData.transmissionDelay = transmissionDelay;
        traceData.flowDeliveredBytes = flowDeliveredBytes;

        assert(sizeof(traceData) == APPLICATION_END_FLOW_TRACE_RECORD_BYTES);

        simulationEngineInterfacePtr->OutputTraceInBinary(
            modelName, theApplicationId, "MultiFtpEndFlow", traceData);


        }
        else {
            ostringstream outStream;

            outStream << "DeliveredBytes= " << flowDeliveredBytes
                      << " Delay= " << ConvertTimeToStringSecs(transmissionDelay);

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "MultiFtpEndFlow", outStream.str());
        }//if//
    }//if//

    // Stats
    transmissionDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(transmissionDelay));

}//OutputTraceAndStatsForFlowEnd//



inline
void MultipleFtpSourceApplication::ReserveBandwidth()
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

        qosControlInterface.RequestDualFlowReservation(
            reservationScheme,
            schedulingScheme,
            flowPriority,
            qosMinReverseBandwidth, qosMaxReverseBandwidth,
            qosMinBandwidth, qosMaxBandwidth,
            destinationAddress, destinationPortId, sourceAddress, sourcePortId, IP_PROTOCOL_NUMBER_TCP,
            replyPtr);
    }//if//

}//ReserveBandwidth//


inline
void MultipleFtpSourceApplication::ReserveBandwidthRequestDeniedAction()
{
    cerr << "Warning in MultipleFTP with QoS application: Bandwidth request denied." << endl;
}


inline
void MultipleFtpSourceApplication::UnreserveBandwidth()
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

class MultipleFtpSinkApplication:
    public MultipleFtpApplication, public enable_shared_from_this<MultipleFtpSinkApplication> {
public:
    MultipleFtpSinkApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationId& initApplicationId,
        const NodeId& initSourceNodeId,
        const NodeId& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl);

    void CompleteInitialization();

    void DisconnectFromOtherLayers();

private:

    class FlowReservationEvent: public SimulationEvent {
    public:
        explicit
        FlowReservationEvent(const shared_ptr<MultipleFtpSinkApplication>& initMultipleFtpAppPtr)
            : multipleFtpAppPtr(initMultipleFtpAppPtr) {}
        virtual void ExecuteEvent() { multipleFtpAppPtr->ReserveOrUnreserveBandwidth(); }

    private:
        shared_ptr<MultipleFtpSinkApplication> multipleFtpAppPtr;

    };//FlowReservationEvent//

    class ConnectionHandler: public ConnectionFromTcpProtocolHandler {
    public:
        ConnectionHandler(const shared_ptr<MultipleFtpSinkApplication>& initSinkAppPtr)
            : sinkApplicationPtr(initSinkAppPtr) {}

        void HandleNewConnection(const shared_ptr<TcpConnection>& connectionPtr)
        {
            sinkApplicationPtr->AcceptTcpConnection(connectionPtr);
        }
    private:
        shared_ptr<MultipleFtpSinkApplication> sinkApplicationPtr;

    };//ConnectionHandler//

    //-------------------------------------------
    class TcpEventHandler: public TcpConnection::AppTcpEventHandler {
    public:
        TcpEventHandler(const shared_ptr<MultipleFtpSinkApplication>& initSinkAppPtr)
            : sinkApplicationPtr(initSinkAppPtr) {}

        void DoTcpIsReadyForMoreDataAction() { }

        void ReceiveDataBlock(
            const unsigned char dataBlock[],
            const unsigned int dataLength,
            const unsigned int actualDataLength,
            bool& stallIncomingDataFlow)
        {
            sinkApplicationPtr->ReceiveData(
                dataBlock, dataLength, actualDataLength);
            stallIncomingDataFlow = false;
        }

        void DoTcpRemoteHostClosedAction() {
            sinkApplicationPtr->DoTcpRemoteHostClosedAction();
        }

    private:
        shared_ptr<MultipleFtpSinkApplication> sinkApplicationPtr;

    };//TcpEventHandler//

    //-------------------------------------------

    void DoTcpRemoteHostClosedAction() {
        tcpConnectionPtr->Close();
        (*this).tcpConnectionPtr.reset();
    }

    void AcceptTcpConnection(const shared_ptr<TcpConnection>& connectionPtr);

    void ReceiveData(
        const unsigned char dataBlock[],
        const unsigned int dataLength,
        const unsigned int actualDataLength);

    void OutputTraceAndStatsForReceiveData(
        const unsigned int length);

    //----------------------------------

    class FlowRequestReplyFielder: public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<MultipleFtpSinkApplication>& initAppPtr)
            : appPtr(initAppPtr) { }

        void RequestAccepted(const FlowId& theFlowId)
            { appPtr->macQosFlowId = theFlowId; }
        void RequestDenied() { appPtr->ReserveBandwidthRequestDeniedAction(); }
    private:
        shared_ptr<MultipleFtpSinkApplication> appPtr;

    };//FlowRequestReplyFielder//

    void ReserveOrUnreserveBandwidth();
    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();

    //----------------------------------

    shared_ptr<TcpConnection> tcpConnectionPtr;

    shared_ptr<CounterStatistic> bytesReceivedStatPtr;

};//MultipleFtpSinkApplication//




inline
MultipleFtpSinkApplication::MultipleFtpSinkApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationId& initApplicationId,
    const NodeId& initSourceNodeId,
    const NodeId& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl)
    :
    MultipleFtpApplication(
        parameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initApplicationId,
        initSourceNodeId,
        initDestinationNodeId,
        initDefaultApplicationPortId,
        initEnableQosControl),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_BytesReceived")))
{
}//MultipleFtpSinkApplication//


// Not in constructor becaused shared_from_this limitation.

inline
void MultipleFtpSinkApplication::CompleteInitialization()
{
    shared_ptr<ConnectionHandler> connectionHandlerPtr(new ConnectionHandler(shared_from_this()));

    assert(transportLayerPtr->tcpPtr->PortIsAvailable(destinationPortId));

    transportLayerPtr->tcpPtr->OpenSpecificTcpPort(
        NetworkAddress::anyAddress,
        destinationPortId,
        connectionHandlerPtr);

    if (reserveBandwidthModeIsOn) {

        const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
        const SimTime minimumSetupTime = 1 * MILLI_SECOND;
        const SimTime reservationLeewayTime = 1000 * MILLI_SECOND;

        SimTime reservationTime;
        if ((currentTime + minimumSetupTime + reservationLeewayTime) <= flowStartTime) {
            reservationTime = (flowStartTime - reservationLeewayTime);
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
            (flowEndTime + reservationLeewayTime));

    }//if//
}

inline
void MultipleFtpSinkApplication::DisconnectFromOtherLayers()
{
    transportLayerPtr->tcpPtr->DisconnectConnectionHandlerForPort(
        NetworkAddress::anyAddress, destinationPortId);

    if (tcpConnectionPtr != nullptr) {
        tcpConnectionPtr->ClearAppTcpEventHandler();
        (*this).tcpConnectionPtr.reset();
    }//if//

    (*this).Application::DisconnectFromOtherLayers();

}//DisconnectFromOtherLayers//




inline
void MultipleFtpSinkApplication::AcceptTcpConnection(const shared_ptr<TcpConnection>& connectionPtr)
{
    assert((tcpConnectionPtr == nullptr) && "Should only be one connection");

    tcpConnectionPtr = connectionPtr;

    shared_ptr<TcpEventHandler> tcpEventHandlerPtr(new TcpEventHandler(shared_from_this()));

    tcpConnectionPtr->SetAppTcpEventHandler(tcpEventHandlerPtr);
    tcpConnectionPtr->SetPacketPriority(flowPriority);

    if (useVirtualPayload) {
        tcpConnectionPtr->EnableVirtualPayload();
    }//if//
}


inline
void MultipleFtpSinkApplication::OutputTraceAndStatsForReceiveData(const unsigned int length)
{
    bytesReceivedStatPtr->IncrementCounter(length);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationReceiveDataTraceRecord traceData;

            traceData.sourceNodeId = sourceNodeId;
            traceData.dataLengthBytes = length;

            assert(sizeof(traceData) == APPLICATION_RECEIVE_DATA_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theApplicationId, "MultiFtpRecv", traceData);
        }
        else {
            ostringstream outStream;

            outStream << "SrcN= " << sourceNodeId
                      << " RecvBytes= " << length;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "MultiFtpRecv", outStream.str());

        }//if//
    }//if//
}


inline
void MultipleFtpSinkApplication::ReceiveData(
    const unsigned char dataBlock[],
    const unsigned int dataLength,
    const unsigned int actualDataLength)
{
    OutputTraceAndStatsForReceiveData(dataLength);
}




inline
void MultipleFtpSinkApplication::ReserveBandwidth()
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

        qosControlInterface.RequestDualFlowReservation(
            reservationScheme,
            schedulingScheme,
            flowPriority,
            qosMinBandwidth, qosMaxBandwidth,
            qosMinReverseBandwidth, qosMaxReverseBandwidth,
            sourceAddress, sourcePortId, destinationAddress, destinationPortId, IP_PROTOCOL_NUMBER_TCP,
            replyPtr);
    }//if//

}//ReserveBandwidth//


inline
void MultipleFtpSinkApplication::ReserveBandwidthRequestDeniedAction()
{
    cerr << "Warning in MultipleFTP with QoS application: Bandwidth request denied." << endl;
}


inline
void MultipleFtpSinkApplication::UnreserveBandwidth()
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
void MultipleFtpSinkApplication::ReserveOrUnreserveBandwidth()
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (currentTime < flowEndTime) {
        (*this).ReserveBandwidth();
    }
    else {
        (*this).UnreserveBandwidth();
    }//if//
}//ReserveOrUnreserveBandwidth//



}//namespace//

#endif
