// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef ALOHAPHY_H
#define ALOHAPHY_H

#include <sstream>
#include <queue>
#include <memory>

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "scensim_prop.h"
#include "scensim_support.h"

#include "aloha_tracedefs.h"


namespace Aloha {

using std::cerr;
using std::endl;
using std::ostringstream;
using std::shared_ptr;
using std::enable_shared_from_this;
using std::unique_ptr;
using std::move;
using std::queue;

using ScenSim::SimTime;
using ScenSim::SECOND;
using ScenSim::ZERO_TIME;
using ScenSim::ConvertTimeToStringSecs;
using ScenSim::ConvertDoubleSecsToTime;
using ScenSim::ParameterDatabaseReader;
using ScenSim::SimulationEngineInterface;
using ScenSim::SimulationEvent;
using ScenSim::MakeLowerCaseString;
using ScenSim::RandomNumberGenerator;
using ScenSim::RandomNumberGeneratorSeed;
using ScenSim::HashInputsToMakeSeed;
using ScenSim::NodeId;
using ScenSim::INVALID_NODEID;
using ScenSim::InterfaceId;
using ScenSim::Packet;
using ScenSim::PacketId;
using ScenSim::SimplePropagationModel;
using ScenSim::SimplePropagationModelForNode;
using ScenSim::IntegralPower;
using ScenSim::ConvertToNonDb;
using ScenSim::TracePhy;
using ScenSim::TracePhyInterference;
using ScenSim::CounterStatistic;
using ScenSim::RealStatistic;
using ScenSim::CalculateThermalNoisePowerWatts;
using ScenSim::ConvertToDb;


typedef long long int DatarateBitsPerSec;

class AlohaMac;


class AlohaPhy {
public:
    static const string modelName;

    struct PropFrame {
        DatarateBitsPerSec datarateBitsPerSecond;
        unique_ptr<ScenSim::Packet> macFramePtr;

        PropFrame() : datarateBitsPerSecond(0) { }
    };//PropFrame//

    typedef SimplePropagationModel<PropFrame> PropagationModelType;
    typedef SimplePropagationModelForNode<PropFrame> PropagationInterfaceType;

    typedef SimplePropagationModelForNode<PropFrame>::IncomingSignal IncomingSignal;



    AlohaPhy(
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
        const shared_ptr<PropagationInterfaceType>& initPropModelInterfacePtr,
        AlohaMac* macLayerPtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const RandomNumberGeneratorSeed& nodeSeed);

    void DisconnectFromOtherObjects();

    const SimTime GetDelayUntilAirborne() const { return delayUntilAirborne; }

    SimTime CalculateFrameTransmitDuration(
        const size_t frameLengthBytes,
        const DatarateBitsPerSec& datarateBitsPerSecond) const;

    void TransmitAFrame(
        unique_ptr<Packet>& packetPtr,
        const DatarateBitsPerSec& datarateBitsPerSecond,
        const double& transmisionPowerDbm,
        const SimTime& frameDuration);

private:

    static const int SEED_HASH = 79829569;

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    NodeId theNodeId;
    InterfaceId theInterfaceId;
    RandomNumberGenerator aRandomNumberGenerator;

    AlohaMac* macLayerPtr;

    shared_ptr<PropagationInterfaceType> propModelInterfacePtr;

    enum PhyTxStateType { IDLE_STATE, TX_STARTING_STATE, TRANSMITTING_STATE };
    PhyTxStateType phyTxState;

    enum PhyRxStateType { SCANNING_STATE, RECEIVING_STATE };
    PhyRxStateType phyRxState;

    queue<shared_ptr<PropFrame> > propagatedFramePtrs;
    shared_ptr<PropFrame> currentPropagatedFramePtr; // Outbound frame

    static const int DEFAULT_PHY_FRAME_DATA_PADDING_BITS = 0;
    int phyFrameDataPaddingBits;

    double outgoingTransmissionPowerDbm;
    SimTime outgoingFrameDuration;

    SimTime delayUntilAirborne;

    double signalRxPowerThresholdDbm;

    struct IncomingFrameInfoType {
        NodeId sourceNodeId;
        double receivedPowerDbm;

        IncomingFrameInfoType()
            : sourceNodeId(INVALID_NODEID), receivedPowerDbm(-DBL_MAX) { }

        void Clear()
        {
            sourceNodeId = INVALID_NODEID;
            receivedPowerDbm = -DBL_MAX;
        }

        ~IncomingFrameInfoType() { }

    };//IncomingFrameInfoType//

    IncomingFrameInfoType currentlyReceivingFrameInfo;
    bool currentPacketHasAnError;

    unsigned int numberOfCurrentInterferingSignals;



    bool IsCurrentlyReceivingThisSignal(const IncomingSignal& aSignal) const;
    bool IsReceivingInterferingSignals() const { return ((*this).numberOfCurrentInterferingSignals > 0); }
    bool IsReceivingASignal() const
    {
        return ((*this).currentlyReceivingFrameInfo.sourceNodeId != INVALID_NODEID);
    }

    void ProcessEndOfTheSignalCurrentlyBeingReceived(const IncomingSignal& aSignal);
    void ProcessEndOfANoise(const IncomingSignal& aSignal);
    void EndReceivingSignalFromChannel(const IncomingSignal& aSignal);

    void StartReceivingThisSignal(const IncomingSignal& aSignal);
    void ProcessNewSignal(const IncomingSignal& aSignal);
    void StartReceivingSignalFromChannel(const IncomingSignal& aSignal);

    void StartTransmission();
    void EndCurrentTransmission();
    void StartOrEndTransmission();

    //-------------------------------------------------------------------------

    class SignalArrivalHandler : public PropagationInterfaceType::SignalHandler {
    public:
        SignalArrivalHandler(AlohaPhy* initPhyPtr) : phyPtr(initPhyPtr) { }
        void ProcessSignal(const IncomingSignal& aSignal)
        {
            phyPtr->StartReceivingSignalFromChannel(aSignal);
        }
    private:
        AlohaPhy* phyPtr;
    };//SignalArrivalHandler//

    //-------------------------------------------------------------------------
    class SignalEndHandler : public PropagationInterfaceType::SignalHandler {
    public:
        SignalEndHandler(AlohaPhy* initPhyPtr) : phyPtr(initPhyPtr) { }
        void ProcessSignal(const IncomingSignal& aSignal)
        {
            phyPtr->EndReceivingSignalFromChannel(aSignal);
        }
    private:
        AlohaPhy* phyPtr;
    };//SignalArrivalHandler//

    //-------------------------------------------------------------------------

    class TransmissionTimerEvent : public SimulationEvent {
    public:
        TransmissionTimerEvent(AlohaPhy* initPhyPtr) : phyPtr(initPhyPtr) { }
        void ExecuteEvent() { phyPtr->StartOrEndTransmission(); }
    private:
        AlohaPhy* phyPtr;
    };//TransmissionTimerEvent//

    shared_ptr<TransmissionTimerEvent> transmissionTimerEventPtr;

    //-------------------------------------------------------------------------
    // Trace and Statistics

    int tracePrecisionDigitsForDbm;

    int numberOfReceivedFrames;

    int numberOfFramesWithErrors;

    shared_ptr<CounterStatistic> transmittedFramesStatPtr;
    shared_ptr<CounterStatistic> droppedFramesStatPtr;
    shared_ptr<RealStatistic> receivedPacketRssiDbmStatPtr;
    shared_ptr<CounterStatistic> receivedFramesStatPtr;
    shared_ptr<CounterStatistic> framesWithErrorsStatPtr;

    void OutputTraceForNoiseStart(const IncomingSignal& aSignal) const;
    void OutputTraceForNoiseEnd(const IncomingSignal& aSignal) const;

    void OutputTraceAndStatsForTxStart(
        const Packet& aPacket,
        const double& txPowerDbm,
        const DatarateBitsPerSec& theDatarateBitsPerSec,
        const SimTime& duration) const;

    void OutputTraceForTxEnd() const;
    void OutputTraceAndStatsForTxFailure() const;

    void OutputTraceAndStatsForRxStart(const Packet& aPacket) const;
    void OutputTraceAndStatsForRxEnd(const IncomingSignal& aSignal);



    // Disable:
    AlohaPhy(const AlohaPhy&);
    void operator=(const AlohaPhy&);

};//AlohaPhy//




//--------------------------------------------------------------------------------------------------

inline
AlohaPhy::AlohaPhy(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<PropagationInterfaceType>& initPropModelInterfacePtr,
    AlohaMac* initMacLayerPtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& initNodeId,
    const InterfaceId& initInterfaceId,
    const RandomNumberGeneratorSeed& nodeSeed)
    :
    simEngineInterfacePtr(initSimulationEngineInterfacePtr),
    theNodeId(initNodeId),
    theInterfaceId(initInterfaceId),
    aRandomNumberGenerator(HashInputsToMakeSeed(nodeSeed, initInterfaceId, SEED_HASH)),
    macLayerPtr(initMacLayerPtr),
    propModelInterfacePtr(initPropModelInterfacePtr),
    phyTxState(IDLE_STATE),
    phyRxState(SCANNING_STATE),
    phyFrameDataPaddingBits(DEFAULT_PHY_FRAME_DATA_PADDING_BITS),
    outgoingTransmissionPowerDbm(-DBL_MAX),
    outgoingFrameDuration(ZERO_TIME),
    delayUntilAirborne(ZERO_TIME),
    signalRxPowerThresholdDbm(-DBL_MAX),
    currentPacketHasAnError(true),
    numberOfCurrentInterferingSignals(0),
    tracePrecisionDigitsForDbm(8),
    numberOfReceivedFrames(0),
    numberOfFramesWithErrors(0),
    transmittedFramesStatPtr(
        simEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_FramesTransmitted"))),
    droppedFramesStatPtr(
        simEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_FramesDropped"))),
    receivedPacketRssiDbmStatPtr(
        simEngineInterfacePtr->CreateRealStatWithDbConversion(
            (modelName + '_' + theInterfaceId + "_ReceivedFrameRssiDbm"))),
    receivedFramesStatPtr(
        simEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_FramesReceived"))),
    framesWithErrorsStatPtr(
        simEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_FramesWithErrors")))
{
    propagatedFramePtrs.push(shared_ptr<PropFrame>(new PropFrame()));
    propagatedFramePtrs.push(shared_ptr<PropFrame>(new PropFrame()));

    propModelInterfacePtr->RegisterSignalHandler(
        unique_ptr<PropagationInterfaceType::SignalHandler>(new SignalArrivalHandler(this)));
    propModelInterfacePtr->RegisterSignalEndHandler(
        unique_ptr<PropagationInterfaceType::SignalHandler>(new SignalEndHandler(this)));

    if (theParameterDatabaseReader.ParameterExists(
            "aloha-phy-frame-data-padding-bits", theNodeId, theInterfaceId)) {

        phyFrameDataPaddingBits =
            theParameterDatabaseReader.ReadInt(
                "aloha-phy-frame-data-padding-bits", theNodeId, theInterfaceId);

        if (phyFrameDataPaddingBits < 0) {
            cerr << "Invalid value (aloha-phy-frame-data-padding-bits): "
                 << phyFrameDataPaddingBits << endl;
            exit(1);
        }//if//
    }//if//

    delayUntilAirborne =
        theParameterDatabaseReader.ReadTime(
            "aloha-phy-delay-until-airborne", theNodeId, theInterfaceId);

    if (delayUntilAirborne <= ZERO_TIME) {
        cerr << "Invalid value (aloha-phy-delay-until-airborne): "
             << delayUntilAirborne << endl;
        exit(1);
    }//if//

    signalRxPowerThresholdDbm =
        theParameterDatabaseReader.ReadDouble(
            "aloha-signal-rx-power-threshold-dbm",
            theNodeId,
            theInterfaceId);

    transmissionTimerEventPtr.reset(new TransmissionTimerEvent(this));

}//AlohaPhy//



inline
void AlohaPhy::DisconnectFromOtherObjects()
{
    propModelInterfacePtr->DisconnectThisInterface();
    propModelInterfacePtr.reset();
}//DisconnectFromOtherObjects//



inline
SimTime AlohaPhy::CalculateFrameTransmitDuration(
    const size_t frameLengthBytes,
    const DatarateBitsPerSec& datarateBitsPerSecond) const
{
    // A more realistic computation requires values defined by the propagation model.

    const size_t numberFrameBits = (frameLengthBytes * 8) + (*this).phyFrameDataPaddingBits;

    return ConvertDoubleSecsToTime((double) numberFrameBits / datarateBitsPerSecond);

}//CalculateFrameTransmitDuration//



//----------------------------------------------------------------------------------------
// Rx Process
//----------------------------------------------------------------------------------------

inline
bool AlohaPhy::IsCurrentlyReceivingThisSignal(const IncomingSignal& aSignal) const
{
    assert(phyRxState == RECEIVING_STATE);
    return (aSignal.GetSourceNodeId() == currentlyReceivingFrameInfo.sourceNodeId);

}//IsCurrentlyReceivingThisSignal//


inline
void AlohaPhy::ProcessEndOfANoise(const IncomingSignal& aSignal)
{
    assert(phyRxState == RECEIVING_STATE);
    assert(numberOfCurrentInterferingSignals > 0);

    (*this).numberOfCurrentInterferingSignals--;

    (*this).OutputTraceForNoiseEnd(aSignal);

    if (!(IsReceivingInterferingSignals() || IsReceivingASignal())) {
        phyRxState = SCANNING_STATE;
    }//if//

}//ProcessEndOfANoise//


inline
void AlohaPhy::EndReceivingSignalFromChannel(const IncomingSignal& aSignal)
{
    if (aSignal.GetReceivedPowerDbm() < (*this).signalRxPowerThresholdDbm) {
        // Skip This Signal. (Too Weak)
        return;
    }//if//

    switch (phyRxState) {
    case SCANNING_STATE: {
        assert(false && "Invalid Rx State."); abort();
        break;
    }
    case RECEIVING_STATE: {

        if (IsCurrentlyReceivingThisSignal(aSignal)) {
            (*this).ProcessEndOfTheSignalCurrentlyBeingReceived(aSignal);
        }
        else {
            (*this).ProcessEndOfANoise(aSignal);
        }//if//

        break;
    }
    default:
        assert(false && "Unhandled Case"); abort();
        break;
    }//switch//

}//EndReceivingSignalFromChannel//



inline
void AlohaPhy::StartReceivingThisSignal(const IncomingSignal& aSignal)
{
    assert(phyRxState == SCANNING_STATE);

    phyRxState = RECEIVING_STATE;

    (*this).currentlyReceivingFrameInfo.sourceNodeId = aSignal.GetSourceNodeId();
    (*this).currentlyReceivingFrameInfo.receivedPowerDbm = aSignal.GetReceivedPowerDbm();

    (*this).OutputTraceAndStatsForRxStart(*aSignal.GetFrame().macFramePtr);

    (*this).currentPacketHasAnError = false;

}//StartReceivingThisSignal//


inline
void AlohaPhy::ProcessNewSignal(const IncomingSignal& aSignal)
{
    switch(phyRxState) {
    case SCANNING_STATE: {

        if(aSignal.HasACompleteFrame() && (phyTxState == IDLE_STATE)) {
            (*this).StartReceivingThisSignal(aSignal);
        }
        else {
            phyRxState = RECEIVING_STATE;

            (*this).numberOfCurrentInterferingSignals++;

            (*this).OutputTraceForNoiseStart(aSignal);

            (*this).currentPacketHasAnError = true;
        }//if//

        break;
    }
    case RECEIVING_STATE: {

        (*this).numberOfCurrentInterferingSignals++;

        (*this).OutputTraceForNoiseStart(aSignal);

        (*this).currentPacketHasAnError = true;

        break;
    }
    default:
        assert(false && "Unhandled Case"); abort();
        break;
    }//switch//

}//ProcessNewSignal//


inline
void AlohaPhy::StartReceivingSignalFromChannel(const IncomingSignal& aSignal)
{
    if (aSignal.GetReceivedPowerDbm() >= (*this).signalRxPowerThresholdDbm) {
        (*this).ProcessNewSignal(aSignal);
    }
    else {
        // Skip This Signal. (Too Weak)
    }//if//

}//StartReceivingSignalFromChannel//



//----------------------------------------------------------------------------------------
// Tx Process
//----------------------------------------------------------------------------------------

inline
void AlohaPhy::StartTransmission()
{
    assert(phyTxState == TX_STARTING_STATE);

    phyTxState = TRANSMITTING_STATE;

    (*this).OutputTraceAndStatsForTxStart(
        (*currentPropagatedFramePtr->macFramePtr),
        (*this).outgoingTransmissionPowerDbm,
        (*this).currentPropagatedFramePtr->datarateBitsPerSecond,
        (*this).outgoingFrameDuration);

    propModelInterfacePtr->TransmitSignal(
        (*this).outgoingTransmissionPowerDbm,
        (*this).outgoingFrameDuration,
        (*this).currentPropagatedFramePtr);

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
    simEngineInterfacePtr->ScheduleEvent(
        transmissionTimerEventPtr,
        currentTime + outgoingFrameDuration);

}//StartTransmission//


inline
void AlohaPhy::StartOrEndTransmission()
{
    if (phyTxState == TX_STARTING_STATE) {
        (*this).StartTransmission();
    }
    else if (phyTxState == TRANSMITTING_STATE) {
        (*this).EndCurrentTransmission();
    }
    else {
        assert(false && "Unknown state");
        abort();
    }//if//

}//StartOrEndTransmission//



//----------------------------------------------------------------------------------------
// Trace and Statistics
//----------------------------------------------------------------------------------------

inline
void AlohaPhy::OutputTraceForNoiseStart(const IncomingSignal& aSignal) const
{
    if (simEngineInterfacePtr->TraceIsOn(TracePhyInterference)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            AlohaPhySignalInterferenceTraceRecord traceData;
            const PacketId& thePacketId = aSignal.GetFrame().macFramePtr->GetPacketId();

            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == ALOHA_PHY_SIGNAL_INTERFERENCE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(
                modelName, theInterfaceId, "Noise_Start", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream << "PktId= " << aSignal.GetFrame().macFramePtr->GetPacketId();

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Noise_Start", msgStream.str());
        }//if//
    }//if//

}//OutputTraceForNoiseStart//


inline
void AlohaPhy::OutputTraceForNoiseEnd(const IncomingSignal& aSignal) const
{
    if (simEngineInterfacePtr->TraceIsOn(TracePhyInterference)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            AlohaPhySignalInterferenceTraceRecord traceData;
            const PacketId& thePacketId = aSignal.GetFrame().macFramePtr->GetPacketId();

            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == ALOHA_PHY_SIGNAL_INTERFERENCE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(
                modelName, theInterfaceId, "Noise_End", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream << "PktId= " << aSignal.GetFrame().macFramePtr->GetPacketId();

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Noise_End", msgStream.str());
        }//if//
    }//if//

}//OutputTraceForNoiseEnd//


inline
void AlohaPhy::OutputTraceAndStatsForTxFailure() const
{
    if (simEngineInterfacePtr->TraceIsOn(TracePhy)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx_Failed");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx_Failed", "");

        }//if//
    }//if//

    (*this).droppedFramesStatPtr->IncrementCounter();

}//OutputTraceAndStatsForTxFailure//


inline
void AlohaPhy::OutputTraceAndStatsForTxStart(
    const Packet& aPacket,
    const double& txPowerDbm,
    const DatarateBitsPerSec& theDatarateBitsPerSec,
    const SimTime& duration) const
{
    if (simEngineInterfacePtr->TraceIsOn(TracePhy)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            AlohaPhyTxStartTraceRecord traceData;
            const PacketId& thePacketId = aPacket.GetPacketId();

            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            traceData.txPower = txPowerDbm;
            traceData.dataRate = theDatarateBitsPerSec;
            traceData.duration = duration;

            assert(sizeof(traceData) == ALOHA_PHY_TX_START_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(
                modelName, theInterfaceId, "Tx_Start", traceData);
        }
        else {
            ostringstream msgStream;
            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "PktId= " << aPacket.GetPacketId()
                      << " TxPow= " << txPowerDbm
                      << " Rate= " << theDatarateBitsPerSec
                      << " Dur= " << ConvertTimeToStringSecs(duration);

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx_Start", msgStream.str());
        }//if//
    }//if//

    (*this).transmittedFramesStatPtr->IncrementCounter();

}//OutputTraceAndStatsForTxStart//


inline
void AlohaPhy::OutputTraceForTxEnd() const
{
    if (simEngineInterfacePtr->TraceIsOn(TracePhy)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx_End");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx_End", "");
        }//if//
    }//if//

}//OutputTraceForTxEnd//


inline
void AlohaPhy::OutputTraceAndStatsForRxStart(const Packet& aPacket) const
{
    if (simEngineInterfacePtr->TraceIsOn(TracePhy)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            AlohaPhyRxStartTraceRecord traceData;
            const PacketId& thePacketId = aPacket.GetPacketId();

            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();
            traceData.rxPower = (*this).currentlyReceivingFrameInfo.receivedPowerDbm;

            assert(sizeof(traceData) == ALOHA_PHY_RX_START_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(
                modelName, theInterfaceId, "Rx_Start", traceData);
        }
        else {

            ostringstream msgStream;
            msgStream.precision((*this).tracePrecisionDigitsForDbm);

            msgStream << "PktId= " << aPacket.GetPacketId()
                      << " RxPow= " << (*this).currentlyReceivingFrameInfo.receivedPowerDbm;

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Rx_Start", msgStream.str());

        }//if//
    }//if//

}//OutputTraceAndStatsForRxStart//


inline
void AlohaPhy::OutputTraceAndStatsForRxEnd(const IncomingSignal& aSignal)
{
    if (simEngineInterfacePtr->TraceIsOn(TracePhy)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            AlohaPhyRxEndTraceRecord traceData;
            const PacketId& thePacketId = aSignal.GetFrame().macFramePtr->GetPacketId();

            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();
            traceData.error = (*this).currentPacketHasAnError;

            assert(sizeof(traceData) == ALOHA_PHY_RX_END_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(
                modelName, theInterfaceId, "Rx_End", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream << "PktId= " << aSignal.GetFrame().macFramePtr->GetPacketId();

            if (currentPacketHasAnError) {
                msgStream << " Err= Yes";
            }
            else {
                msgStream << " Err= No";
            }//if//

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Rx_End", msgStream.str());

        }//if//
    }//if//

    (*this).receivedPacketRssiDbmStatPtr->RecordStatValue(ConvertToNonDb(aSignal.GetReceivedPowerDbm()));

    if (!(*this).currentPacketHasAnError) {

        (*this).numberOfReceivedFrames++;
        (*this).receivedFramesStatPtr->UpdateCounter(numberOfReceivedFrames);
    }
    else {

        (*this).numberOfFramesWithErrors++;
        (*this).framesWithErrorsStatPtr->UpdateCounter((*this).numberOfFramesWithErrors);

    }//if//

}//OutputTraceAndStatsForRxEnd//



}//namespace//

#endif
