// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_APP_VOIP_H
#define SCENSIM_APP_VOIP_H

#include "scensim_netsim.h"

namespace ScenSim {

using std::cerr;
using std::endl;


//--------------------------------------------------------------------------------------------------

// ----- SpeechSourceModel(VoIP) -----
//
// This model is based on IEEE802.16m Evaluation Methodology Document.
// Vocoder: AMR, Source bit rate: 12.2 bps, Frame duration: 20ms, Information bits per frame: 244
// Voice payload : 33 bytes for active at 20ms interval, 7 bytes for inactive at 160ms interval

class VoipApplication: public Application {

public:
    static const string modelName;

    typedef MacQualityOfServiceControlInterface::SchedulingSchemeChoice SchedulingSchemeChoice;
    typedef MacQualityOfServiceControlInterface::ReservationSchemeChoice ReservationSchemeChoice;

    //Voice payload(33 for active, 7 for inactive) + RTP header(12bytes)
    //VoipPayloadType: 12byte
    struct VoipPayloadType {
        unsigned int sequenceNumber;
        SimTime sendTime;

        VoipPayloadType(
            const unsigned int initSequenceNumber,
            const SimTime initSendTime)
            :
            sequenceNumber(initSequenceNumber),
            sendTime(initSendTime)
        {}
    };//VoipPayloadType//

    VoipApplication(
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
    unsigned short int destinationPortId;

    //VoIP Traffic Model Parameters
    SimTime meanStateDuration; //1.25S
    double stateTransitionProbability; //0.016
    SimTime betaForPacketArrivalDelayJittter; //beta = 5.11MS

    SimTime voipStartTime;
    SimTime voipEndTime;
    PacketPriority voipPriority;
    SimTime maxStartTimeJitter;

    //---------------------------------

    bool reserveBandwidthModeIsOn;
    double qosMinBandwidth;
    double qosMaxBandwidth;
    FlowId macQosFlowId;

    SchedulingSchemeChoice schedulingScheme;
    ReservationSchemeChoice reservationScheme;

    bool useVirtualPayload;

    string GetParameterNamePrefix() const {
        if (reserveBandwidthModeIsOn) {
            return  "voip-with-qos";
        }//if//
        return "voip";
    }

};//VoipApplication//

inline
VoipApplication::VoipApplication(
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
    destinationNodeId(initDestinationNodeId),
    destinationPortId(initDefaultApplicationPortId),
    meanStateDuration(ZERO_TIME),
    stateTransitionProbability(0.),
    betaForPacketArrivalDelayJittter(ZERO_TIME),
    voipStartTime(ZERO_TIME),
    voipEndTime(ZERO_TIME),
    voipPriority(0),
    maxStartTimeJitter(ZERO_TIME),
    reserveBandwidthModeIsOn(initEnableQosControl),
    qosMinBandwidth(0.0),
    qosMaxBandwidth(0.0),
    schedulingScheme(MacQualityOfServiceControlInterface::DefaultSchedulingScheme),
    reservationScheme(MacQualityOfServiceControlInterface::OptimisticLinkRate),
    useVirtualPayload(false)
{
    const string parameterPrefix = (*this).GetParameterNamePrefix();

    meanStateDuration =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-mean-state-duration", sourceNodeId, theApplicationId);

    stateTransitionProbability =
        parameterDatabaseReader.ReadDouble(
            parameterPrefix + "-state-transition-probability", sourceNodeId, theApplicationId);

    betaForPacketArrivalDelayJittter =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-beta-for-packet-arrival-delay-jitter", sourceNodeId, theApplicationId);

    voipStartTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-start-time", sourceNodeId, theApplicationId);

    voipEndTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-end-time", sourceNodeId, theApplicationId);

    voipPriority = static_cast<PacketPriority>(
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

    if (initEnableQosControl) {
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

}//VoipApplication//


class VoipSourceApplication: public VoipApplication, public enable_shared_from_this<VoipSourceApplication> {

public:
    VoipSourceApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationId& initApplicationId,
        const NodeId& initSourceNodeId,
        const NodeId& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl,
        const RandomNumberGeneratorSeed& initNodeSeed);

    void CompleteInitialization();

    virtual ~VoipSourceApplication() { }


private:

    static const int SEED_HASH = 5015807;

    static const unsigned int AMR_VOICE_PAYLOAD_BYTES_FOR_ACTIVE = 33;
    static const unsigned int AMR_VOICE_PAYLOAD_BYTES_FOR_INACTIVE = 7;
    static const unsigned int RTP_HEADER_BYTES = 12;
    static const unsigned int AMR_SID_UPDATE_COUNT = 8;
    static const SimTime ENCODER_FRAME_DURATION = 20 * MILLI_SECOND;

    //---------------------------------------
    class VoipSendPacketEvent: public SimulationEvent {
    public:
        explicit
        VoipSendPacketEvent(
            const shared_ptr<VoipSourceApplication>& initVoipSourceApplicationPtr,
            unique_ptr<Packet>& initPacketPtr,
            const unsigned int initSequenceNumber)
            :
            voipSourceApplicationPtr(initVoipSourceApplicationPtr),
            packetPtr(move(initPacketPtr)),
            sequenceNumber(initSequenceNumber)
        {
        }
        virtual void ExecuteEvent() { voipSourceApplicationPtr->SendPacket(packetPtr, sequenceNumber); }

    private:
        shared_ptr<VoipSourceApplication> voipSourceApplicationPtr;
        unique_ptr<Packet> packetPtr;
        unsigned int sequenceNumber;

        VoipSendPacketEvent(const VoipSendPacketEvent&);
        void operator=(const VoipSendPacketEvent&);

    };//VoipSendPacketEvent//

    class VoipSamplingEvent: public SimulationEvent {
    public:
        explicit
        VoipSamplingEvent(const shared_ptr<VoipSourceApplication>& initVoipSourceApplicationPtr)
            : voipSourceApplicationPtr(initVoipSourceApplicationPtr) {}
        virtual void ExecuteEvent() { voipSourceApplicationPtr->Sampling(); }

    private:
        shared_ptr<VoipSourceApplication> voipSourceApplicationPtr;

    };//VoipSamplingEvent//
    //---------------------------------------


    void SendPacket(unique_ptr<Packet>& packetPtr, const unsigned int sequenceNumber);
    void Sampling();

    const SimTime GetStateDuration();

    class FlowRequestReplyFielder: public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<VoipSourceApplication>& initAppPtr) : appPtr(initAppPtr) { }

        void RequestAccepted(const FlowId& theFlowId)
            { appPtr->macQosFlowId = theFlowId; }
        void RequestDenied() { appPtr->ReserveBandwidthRequestDeniedAction(); }
    private:
        shared_ptr<VoipSourceApplication> appPtr;

    };//FlowRequestReplyFielder//

    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();

    //---------------------------------

    RandomNumberGenerator aRandomNumberGenerator;

    enum VoipStateType {
        ACTIVE_STATE,
        INACTIVE_STATE,
    };

    VoipStateType voipState;
    unsigned int currentPacketSequenceNumber;
    SimTime nextStateTransitionTime;

    //Statistics
    shared_ptr<CounterStatistic> packetsSentStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;

    void OutputTraceAndStatsForSendPacket(
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const unsigned int packetLengthBytes);


    friend class VoipSinkApplication;

    //Disable
    VoipSourceApplication();

};//VoipSourceApplication//



inline
VoipSourceApplication::VoipSourceApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationId& initApplicationId,
    const NodeId& initSourceNodeId,
    const NodeId& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl,
    const RandomNumberGeneratorSeed& initNodeSeed)
    :
    VoipApplication(
        parameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initApplicationId,
        initSourceNodeId,
        initDestinationNodeId,
        initDefaultApplicationPortId,
        initEnableQosControl),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(initNodeSeed, initApplicationId, SEED_HASH)),
    voipState(ACTIVE_STATE),
    currentPacketSequenceNumber(0),
    nextStateTransitionTime(ZERO_TIME),
    packetsSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_PacketsSent"))),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_BytesSent")))
{

    nextStateTransitionTime = voipStartTime + GetStateDuration();

}//VoipSourceApplication//

// Two part initialization forced by shared_from_this().

inline
void VoipSourceApplication::CompleteInitialization()
{
    if (maxStartTimeJitter != ZERO_TIME) {
        voipStartTime += static_cast<SimTime>(
            aRandomNumberGeneratorPtr->GenerateRandomDouble() * maxStartTimeJitter);
    }//if//

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
    const SimTime minimumSetupTime = 1 * MILLI_SECOND;

    if (voipStartTime < (currentTime + minimumSetupTime)) {
        voipStartTime = (currentTime + minimumSetupTime);
    }//if//

    if (voipStartTime < voipEndTime) {

        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new VoipSamplingEvent(shared_from_this())), voipStartTime);

    }//if//

}//CompleteInitialization//



inline
const SimTime VoipSourceApplication::GetStateDuration()
{
    //exponential distribution
    const double randomNumber = aRandomNumberGenerator.GenerateRandomDouble();

    SimTime stateDuration = SimTime(-(meanStateDuration * log(1.0 - randomNumber)));

    return stateDuration;

}//GetStateDuration//


inline
void VoipSourceApplication::Sampling()
{
    if ((currentPacketSequenceNumber == 0) && (reserveBandwidthModeIsOn)) {
        (*this).ReserveBandwidth();
    }//if//

    assert((currentPacketSequenceNumber + 1) < UINT_MAX);
    currentPacketSequenceNumber++;//always incremented even if INACTIVE_STATE
    unsigned int frameCount = currentPacketSequenceNumber % AMR_SID_UPDATE_COUNT;

    //schedule to send a pakcet
    if ((voipState == ACTIVE_STATE) || ((voipState == INACTIVE_STATE) && (frameCount == 0))) {

        VoipPayloadType voipAppPayload(
            currentPacketSequenceNumber, simulationEngineInterfacePtr->CurrentTime());

        unsigned int packetPayloadSizeBytes;
        switch (voipState) {
        case ACTIVE_STATE:
            packetPayloadSizeBytes = AMR_VOICE_PAYLOAD_BYTES_FOR_ACTIVE + RTP_HEADER_BYTES;
            break;
        case INACTIVE_STATE:
            packetPayloadSizeBytes = AMR_VOICE_PAYLOAD_BYTES_FOR_INACTIVE + RTP_HEADER_BYTES;
            break;
        default:
            packetPayloadSizeBytes = 0;
            assert(false); abort(); break;
        }//switch//

        unique_ptr<Packet> packetPtr =
            Packet::CreatePacket(
                *simulationEngineInterfacePtr,
                voipAppPayload,
                packetPayloadSizeBytes,
                useVirtualPayload);

        //ENCODER_FRAME_DURATION * 4 for DL in 16m evaluation methodogolgy
        //add jitter
        const SimTime sendTime =
            simulationEngineInterfacePtr->CurrentTime() +
            static_cast<SimTime>(betaForPacketArrivalDelayJittter * aRandomNumberGenerator.GenerateRandomDouble());

        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(
                new VoipSendPacketEvent(shared_from_this(), packetPtr, currentPacketSequenceNumber)),
            sendTime);

    }//if//

    SimTime nextSamplingTime = simulationEngineInterfacePtr->CurrentTime() + ENCODER_FRAME_DURATION;

    while (nextSamplingTime > nextStateTransitionTime) {

        const double randomNumber = aRandomNumberGenerator.GenerateRandomDouble();

        if (randomNumber < stateTransitionProbability) {
            //switch state
            switch (voipState) {
            case ACTIVE_STATE:
                voipState = INACTIVE_STATE;
                break;
            case INACTIVE_STATE:
                voipState = ACTIVE_STATE;
                break;
            default:
                assert(false); abort(); break;
            }//switch//
        }//if//

        nextStateTransitionTime += GetStateDuration();

    }//while//

    if (nextSamplingTime < voipEndTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new VoipSamplingEvent(shared_from_this())), nextSamplingTime);
    }
    else {
        if (reserveBandwidthModeIsOn) {
            (*this).UnreserveBandwidth();
        }//if//
    }//if//

}//Sampling//



inline
void VoipSourceApplication::SendPacket(unique_ptr<Packet>& packetPtr, const unsigned int sequenceNumber)
{

    OutputTraceAndStatsForSendPacket(
        sequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes());

    const NetworkAddress sourceAddress =
        networkAddressLookupInterfacePtr->LookupNetworkAddress(sourceNodeId);

    NetworkAddress destAddress;
    bool foundDestAddress;
    networkAddressLookupInterfacePtr->LookupNetworkAddress(destinationNodeId, destAddress, foundDestAddress);

    if (foundDestAddress) {
        transportLayerPtr->udpPtr->SendPacket(
            packetPtr, sourceAddress, 0, destAddress, destinationPortId, voipPriority);
    }
    else {
        //cannot find destination address (destination node may not be created yet)
        //Future: output trace and stat
        packetPtr = nullptr;
    }//if//

}//SendPacket//



inline
void VoipSourceApplication::OutputTraceAndStatsForSendPacket(
    const unsigned int sequenceNumber,
    const PacketId& thePacketId,
    const unsigned int packetLengthBytes)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationSendTraceRecord traceData;
            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.destinationNodeId = destinationNodeId;
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == APPLICATION_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theApplicationId, "VoipSend", traceData);
        }
        else {

            ostringstream outStream;
            outStream << "Seq= " << sequenceNumber << " PktId= " << thePacketId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "VoipSend", outStream.str());

        }//if//
    }//if//

    packetsSentStatPtr->IncrementCounter();
    bytesSentStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForSendPacket//



inline
void VoipSourceApplication::ReserveBandwidth()
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
            reservationScheme, schedulingScheme, voipPriority, qosMinBandwidth, qosMaxBandwidth,
            destinationAddress, destinationPortId, sourceAddress, ANY_PORT, IP_PROTOCOL_NUMBER_UDP,
            replyPtr);
    }//if//

}//ReserveBandwidth//


inline
void VoipSourceApplication::ReserveBandwidthRequestDeniedAction()
{
    cerr << "Warning in VoIP with QoS application: Bandwidth request denied." << endl;
}


inline
void VoipSourceApplication::UnreserveBandwidth()
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

class VoipSinkApplication: public VoipApplication, public enable_shared_from_this<VoipSinkApplication> {
public:

    VoipSinkApplication(
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
    //-------------------------------------------------------------------------

    class FlowReservationEvent: public SimulationEvent {
    public:
        explicit
        FlowReservationEvent(const shared_ptr<VoipSinkApplication>& initVoipSinkAppPtr)
            : voipSinkAppPtr(initVoipSinkAppPtr) { }
        virtual void ExecuteEvent() { voipSinkAppPtr->ReserveOrUnreserveBandwidth(); }
    private:
        shared_ptr<VoipSinkApplication> voipSinkAppPtr;

    };//FlowReservationEvent//


    class PacketHandler: public UdpProtocol::PacketForAppFromTransportLayerHandler {
    public:
        PacketHandler(const shared_ptr<VoipSinkApplication>& initVoipSinkPtr) : voipSinkPtr(initVoipSinkPtr) { }

        void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const NetworkAddress& sourceAddress,
            const unsigned short int sourcePort,
            const NetworkAddress& destinationAddress,
            const PacketPriority& priority)
        {
            voipSinkPtr->ReceivePacket(packetPtr);
        }


    private:
        shared_ptr<VoipSinkApplication> voipSinkPtr;

    };//PacketHandler//
    //-------------------------------------------------------------------------

    shared_ptr<PacketHandler> packetHandlerPtr;

    class FlowRequestReplyFielder: public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<VoipSinkApplication>& initAppPtr)
            :appPtr(initAppPtr) { }
        void RequestAccepted(const FlowId& theFlowId)
            { appPtr->macQosFlowId = theFlowId; }
        void RequestDenied() { appPtr->ReserveBandwidthRequestDeniedAction(); }
    private:
        shared_ptr<VoipSinkApplication> appPtr;

    };//FlowRequestReplyFielder//

    void ReserveOrUnreserveBandwidth();
    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();

    //---------------------------------

    unsigned int numberPacketsReceived;

    bool useJitterBuffer;

    unsigned int previousMaxSequenceNumber;
    unsigned int targetSequenceNumber;

    struct JitterBufferEntry {
        unique_ptr<Packet> packetPtr;
        SimTime bufferedTime;

        JitterBufferEntry() : bufferedTime(ZERO_TIME) { }
        JitterBufferEntry(
            unique_ptr<Packet>& initPacketPtr,
            const SimTime& initBufferedTime)
            :
            packetPtr(move(initPacketPtr)), bufferedTime(initBufferedTime)
        {
        }

        void operator=(JitterBufferEntry&& right) {
            packetPtr = move(right.packetPtr);
            bufferedTime = right.bufferedTime;
        }

        JitterBufferEntry(JitterBufferEntry&& right) { (*this) = move(right); }

    };//JitterBufferEntry//

    map<unsigned int, JitterBufferEntry> jitterBuffer;
    SimTime jitterBufferWindow;
    void ClearJitterBuffer();
    bool decoderIsStarted;

    //-------------------------------------------------
    class ClearBufferEvent: public SimulationEvent {
    public:
        explicit
        ClearBufferEvent(
            const shared_ptr<VoipSinkApplication>& initVoipSinkApplicationPtr)
            :
        voipSinkApplicationPtr(initVoipSinkApplicationPtr) {}
        virtual void ExecuteEvent() { voipSinkApplicationPtr->ClearJitterBuffer(); }
    private:
        shared_ptr<VoipSinkApplication> voipSinkApplicationPtr;

    };//ClearBufferEvent//
    //-------------------------------------------------

    void ReceivePacket(unique_ptr<Packet>& packetPtr);

    //Statistics
    shared_ptr<CounterStatistic> packetsReceivedStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;
    shared_ptr<RealStatistic> endToEndDelayStatPtr;
    shared_ptr<CounterStatistic> outOfOrderStatPtr;
    shared_ptr<CounterStatistic> successFramesStatPtr;


    void OutputTraceAndStatsForReceivePacket(
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const unsigned int packetLengthBytes,
        const SimTime& delay);

};//VoipSinkApplication//


inline
VoipSinkApplication::VoipSinkApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationId& initApplicationId,
    const NodeId& initSourceNodeId,
    const NodeId& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl)
    :
    VoipApplication(
        parameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initApplicationId,
        initSourceNodeId,
        initDestinationNodeId,
        initDefaultApplicationPortId,
        initEnableQosControl),
    numberPacketsReceived(0),
    previousMaxSequenceNumber(0),
    jitterBufferWindow(ZERO_TIME),
    decoderIsStarted(false),
    packetsReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_PacketsReceived"))),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_BytesReceived"))),
    endToEndDelayStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_" + initApplicationId + "_EndToEndDelay"))),
    outOfOrderStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_PacketsOutOfOrder"))),
    successFramesStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_FramesSuccess")))
{
    const string parameterPrefix = (*this).GetParameterNamePrefix();

    jitterBufferWindow =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-jitter-buffer-window", sourceNodeId, theApplicationId);


    if (jitterBufferWindow == ZERO_TIME) {
        useJitterBuffer = false;
    }
    else {
        useJitterBuffer = true;
    }//if//

}//VoipSinkApplication//

// Two part initialization forced by shared_from_this().

inline
void VoipSinkApplication::CompleteInitialization()
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
        if ((currentTime + minimumSetupTime + reservationLeewayTime) <= voipStartTime) {
            reservationTime = (voipStartTime - reservationLeewayTime);
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
            (voipEndTime + reservationLeewayTime));

    }//if//

}//CompleteInitialization//



inline
void VoipSinkApplication::DisconnectFromOtherLayers()
{
    packetHandlerPtr.reset();
    (*this).Application::DisconnectFromOtherLayers();

}//DisconnectFromOtherLayers//



inline
void VoipSinkApplication::OutputTraceAndStatsForReceivePacket(
    const unsigned int sequenceNumber,
    const PacketId& thePacketId,
    const unsigned int packetLengthBytes,
    const SimTime& delay)
{
    numberPacketsReceived++;

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
                modelName, theApplicationId, "VoipRecv", traceData);

        }
        else {
            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << thePacketId
                      << " Delay= " << ConvertTimeToStringSecs(delay)
                      << " Pdr= " << numberPacketsReceived << '/' << sequenceNumber
                      << " PacketBytes= " << packetLengthBytes;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "VoipRecv", outStream.str());

        }//if//

    }//if//

    packetsReceivedStatPtr->IncrementCounter();
    bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);
    endToEndDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(delay));

}//OutputTraceAndStatsForReceivePacket//


inline
void VoipSinkApplication::ClearJitterBuffer()
{
    if (!jitterBuffer.empty()) {

        map<unsigned int, JitterBufferEntry>::iterator iter
            = jitterBuffer.find(targetSequenceNumber);

        if (iter != jitterBuffer.end()) {
            successFramesStatPtr->IncrementCounter();

            JitterBufferEntry& bufferEntry = iter->second;
            bufferEntry.packetPtr = nullptr;
            jitterBuffer.erase(iter);

        }//if//
    }//if//

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    targetSequenceNumber++;

    const SimTime eventTime
        = currentTime + VoipSourceApplication::ENCODER_FRAME_DURATION;
    simulationEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(new ClearBufferEvent(shared_from_this())), eventTime);

}//ClearJitterBuffer//



inline
void VoipSinkApplication::ReceivePacket(unique_ptr<Packet>& packetPtr)
{
    VoipPayloadType voipPayload = packetPtr->GetAndReinterpretPayloadData<VoipPayloadType>();

    const unsigned int sequenceNumber = voipPayload.sequenceNumber;

    const SimTime delay = simulationEngineInterfacePtr->CurrentTime() - voipPayload.sendTime;

    OutputTraceAndStatsForReceivePacket(
        sequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes(),
        delay);

    if (sequenceNumber < previousMaxSequenceNumber) {
        outOfOrderStatPtr->IncrementCounter();
    }
    else {
        previousMaxSequenceNumber = sequenceNumber;
        if (!useJitterBuffer) {
            successFramesStatPtr->IncrementCounter();
        }//if//
    }//if

    if (useJitterBuffer) {
        //Buffering
        jitterBuffer[sequenceNumber] = move(JitterBufferEntry(packetPtr, simulationEngineInterfacePtr->CurrentTime()));

        if (!decoderIsStarted) {
            decoderIsStarted = true;
            targetSequenceNumber = sequenceNumber;

            //schedule ClearJitterBuffer;
            const SimTime eventTime = simulationEngineInterfacePtr->CurrentTime() + jitterBufferWindow;
            simulationEngineInterfacePtr->ScheduleEvent(
                unique_ptr<SimulationEvent>(new ClearBufferEvent(shared_from_this())),
                eventTime);
        }//if
    }//if//
    else {
        packetPtr = nullptr;
    }//if//

}//ReceivePacket//



inline
void VoipSinkApplication::ReserveBandwidth()
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
            reservationScheme, schedulingScheme, voipPriority, qosMinBandwidth, qosMaxBandwidth,
            sourceAddress, ANY_PORT, destinationAddress, destinationPortId, IP_PROTOCOL_NUMBER_UDP,
            replyPtr);

    }//if//

}//ReserveBandwidth//


inline
void VoipSinkApplication::ReserveBandwidthRequestDeniedAction()
{
    cerr << "Warning in VoIP with QoS application: Bandwidth request denied." << endl;
}


inline
void VoipSinkApplication::UnreserveBandwidth()
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
void VoipSinkApplication::ReserveOrUnreserveBandwidth()
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (currentTime < voipEndTime) {
        (*this).ReserveBandwidth();
    }
    else {
        (*this).UnreserveBandwidth();
    }//if//
}//ReserveOrUnreserveBandwidth//



}//namespace//

#endif
