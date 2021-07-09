// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11AD_MAC_STA_H
#define DOT11AD_MAC_STA_H

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "dot11ad_common.h"
#include "dot11ad_headers.h"

#include <queue>
#include <map>
#include <string>
#include <iomanip>

namespace Dot11ad {

using std::shared_ptr;
using std::deque;
using std::map;
using std::cerr;
using std::endl;
using std::cout;
using std::hex;
using std::string;

using ScenSim::SimulationEngineInterface;
using ScenSim::SimulationEvent;
using ScenSim::EventRescheduleTicket;
using ScenSim::SimTime;
using ScenSim::MILLI_SECOND;
using ScenSim::INFINITE_TIME;
using ScenSim::ZERO_TIME;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeId;
using ScenSim::InterfaceOrInstanceId;
using ScenSim::RandomNumberGenerator;
using ScenSim::RandomNumberGeneratorSeed;
using ScenSim::PacketPriority;
using ScenSim::EtherTypeField;
using ScenSim::HashInputsToMakeSeed;
using ScenSim::ConvertTimeToDoubleSecs;
using ScenSim::ConvertToDb;
using ScenSim::ConvertToNonDb;

class Dot11Mac;


//--------------------------------------------------------------------------------------------------


class AbstractChannelScanningController {
public:
    virtual void ClearCurrentChannelAndAccessPoint() = 0;

    virtual void SetCurrentChannelAndAccessPoint(
        const vector<unsigned int>& bondedChannelList,
        const MacAddress newAccessPoint) = 0;

    virtual void ReceiveBeaconInfo(
        const unsigned int channelId,
        const BeaconFrame& aBeaconFrame,
        const double& frameRssiDbm) = 0;

    virtual void ReceiveDirectionalBeaconInfo(
        const vector<unsigned int>& bondedChannelList,
        const DirectionalBeaconFrameType& aBeaconFrame,
        const double& frameRssiDbm) = 0;

    virtual void ReceiveProbeResponseInfo(
        const unsigned int channelId,
        const ProbeResponseFrame& probeResponseFrame,
        const double& frameRssiDbm) = 0;

    virtual void CalculateLinkStatus() = 0;

    virtual SimTime GetNextLinkCheckTime() const = 0;

    virtual SimTime GetNextScanStartTime() const = 0;

    virtual void StartScanSequence() = 0;

    virtual void GetChannelAndDurationToScan(
        bool& scanSequenceIsDone,
        unsigned int& channelId,
        SimTime& duration) = 0;

    virtual bool NoInRangeAccessPoints() const = 0;
    virtual bool ShouldSwitchAccessPoints() const = 0;

    virtual void GetAccessPointToSwitchTo(
        vector<unsigned int>& bondedChannelList,
        MacAddress& newAccessPoint) = 0;

};//AbstractChannelScanningController//




//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

class Dot11StaManagementController {
public:
    Dot11StaManagementController(
        const shared_ptr<Dot11Mac>& initMacLayerPtr,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId,
        const RandomNumberGeneratorSeed& interfaceSeed);

    void SetChannelScanningController(
        const shared_ptr<AbstractChannelScanningController>& scanningControllerPtr);

    void ProcessManagementFrame(const Packet& managementFrame);

    void GetCurrentAccessPointAddress(
        bool& hasAnAccessPoint,
        MacAddress& currentAccessPointAddress) const;

    void SwitchToAccessPoint(const MacAddress& accessPointAddress);

    void ReceiveOutgoingFrameDeliveryResults(
        const Packet& frame,
        const bool wasAcked);

    unsigned int GetTxAntennaSectorId(const MacAddress& destinationAddress) const;

    // For 11ad AP bypass.

    void GetNextHopMacAddress(
        const MacAddress& destinationMacAddress,
        bool& success,
        MacAddress& nextHopMacAddress) const;

    bool AllowDirectStaToStaTransmission(const MacAddress& destinationMacAddress) const;
    bool AllowDirectStaToStaReceive() const { return true; }

private:
    static const SimTime scanningStartDelay = 100 * MILLI_SECOND;  // "Legacy" Delete

    enum StaManagementStateType {
        NotAssociated,
        ChannelScanning,
        WaitingForAuthentication,
        WaitingForAssociationResponse,
        WaitingForReassociationResponse,
        Associated,
        StartingUpBackgroundChannelScanning,
        BackgroundChannelScanning,
        EndingBackgroundChannelScanning,
    };

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    NodeId theNodeId;
    InterfaceOrInstanceId theInterfaceId;

    string ssidString;

    shared_ptr<Dot11Mac> macLayerPtr;

    shared_ptr<AbstractChannelScanningController> scanningControllerPtr;

    unsigned int powerSavingListenIntervalBeacons;

    static const unsigned int seedHash = 1638757217;
    RandomNumberGenerator aRandomNumberGenerator;

    AssociationId theAssociationId;

    //static const int SEED_HASH = 58579017;
    //RandomNumberGenerator aRandomNumberGenerator;

    StaManagementStateType theStaManagementState;
    MacAddress currentApAddress;
    bool accessPointIsForced;
    MacAddress forcedApAddress;
    MacAddress lastApAddress;
    unsigned int lastChannelId;
    MacAddress lastAccessPointAddress;

    SimTime authenticationTimeoutInterval;
    SimTime associateFailureTimeoutInterval;

    bool isAuthenticated;

    int tracePrecisionDigitsForDbm;

    //------------------------------------------------------
    class InitializationEvent : public SimulationEvent {
    public:
        InitializationEvent(Dot11StaManagementController* initStaControllerPtr) :
            staControllerPtr(initStaControllerPtr) { }
        void ExecuteEvent() { staControllerPtr->StartChannelScanning(); }
    private:
        Dot11StaManagementController* staControllerPtr;
    };//InitializationEvent//

    EventRescheduleTicket initializationEventTicket;

    //------------------------------------------------------
    class ChannelScanTimeoutEvent : public SimulationEvent {
    public:
        ChannelScanTimeoutEvent(Dot11StaManagementController* initStaControllerPtr) :
            staControllerPtr(initStaControllerPtr) { }
        void ExecuteEvent() { staControllerPtr->ChannelScanTimedOut(); }
    private:
        Dot11StaManagementController* staControllerPtr;
    };//ChannelScanTimeoutEvent//

    shared_ptr<ChannelScanTimeoutEvent> scanTimeoutEventPtr;
    EventRescheduleTicket scanTimeoutEventTicket;

    //------------------------------------------------------

    class BackgroundScanStartEvent : public SimulationEvent {
    public:
        BackgroundScanStartEvent(Dot11StaManagementController* initStaControllerPtr) :
            staControllerPtr(initStaControllerPtr) { }
        void ExecuteEvent() { staControllerPtr->InitiateBackgroundChannelScanning(); }
    private:
        Dot11StaManagementController* staControllerPtr;
    };//ChannelScanTimeoutEvent//

    shared_ptr<ChannelScanTimeoutEvent> startBackgroundScanTimeoutEventPtr;
    EventRescheduleTicket startBackgroundScanTimeoutEventTicket;
    SimTime backgroundScanningEventTime;


    //------------------------------------------------------



    class AuthenticationTimeoutEvent : public SimulationEvent {
    public:
        AuthenticationTimeoutEvent(Dot11StaManagementController* initStaControllerPtr) :
            staControllerPtr(initStaControllerPtr) { }
        void ExecuteEvent() { staControllerPtr->AuthenticationTimeout(); }
    private:
        Dot11StaManagementController* staControllerPtr;
    };//AuthenticationTimeoutEvent//

    shared_ptr<AuthenticationTimeoutEvent> authenticationTimeoutEventPtr;
    EventRescheduleTicket authenticationTimeoutEventTicket;

    //------------------------------------------------------
    class AssociateFailureEvent : public SimulationEvent {
    public:
        AssociateFailureEvent(Dot11StaManagementController* initStaControllerPtr) :
            staControllerPtr(initStaControllerPtr) { }
        void ExecuteEvent() { staControllerPtr->ProcessAssociateFailure(); }
    private:
        Dot11StaManagementController* staControllerPtr;
    };//AssociateFailureEvent//

    shared_ptr<AssociateFailureEvent> associateFailureEventPtr;
    EventRescheduleTicket associateFailureEventTicket;


    //------------------------------------------------------
    class CheckLinkStatusEvent : public SimulationEvent {
    public:
        CheckLinkStatusEvent(Dot11StaManagementController* initStaControllerPtr) :
            staControllerPtr(initStaControllerPtr) { }
        void ExecuteEvent() { staControllerPtr->CheckLinkStatus(); }
    private:
        Dot11StaManagementController* staControllerPtr;
    };//CheckLinkStatusEvent//

    shared_ptr<CheckLinkStatusEvent> linkStatusCheckEventPtr;
    EventRescheduleTicket linkStatusCheckEventTicket;


    //------------------------------------------------------

    // 802.11ad:
    // DMG = Directional Multi-Gigabit.

    // A-BFT info received from from AP Beacons.

    unsigned int associationBeamformingTrainingAkaAbftNumSlots;
    SimTime associationBeamformingTrainingAkaAbftSlotTime;

    //SimTime sectorSweepFeedbackTimeoutDuration;

    //Future SimTime announcementTransmissionIntervalAkaAtiDuration;

    //---------------------------------

    struct DataTransferIntervalListElementType {
        bool isAContentionBasedAccessPeriodAkaCbap;
        bool mustDoReceiveSectorSweep;

        AssociationId sourceAssociationId;
        NodeId sourceNodeId;
        AssociationId destinationAssociationId;
        NodeId destinationNodeId;

        SimTime relativeStartTime;
        SimTime duration;

        DataTransferIntervalListElementType(
            const bool initIsAContentionBasedAccessPeriodAkaCbap,
            const bool initMustDoReceiveSectorSweep,
            const SimTime& initRelativeStartTime,
            const SimTime& initDuration,
            const AssociationId& initSourceAssociationId,
            const AssociationId& initDestinationAssociationId,
            const NodeId& initSourceNodeId,
            const NodeId& initDestinationNodeId)
        :
            isAContentionBasedAccessPeriodAkaCbap(initIsAContentionBasedAccessPeriodAkaCbap),
            mustDoReceiveSectorSweep(initMustDoReceiveSectorSweep),
            relativeStartTime(initRelativeStartTime),
            duration(initDuration),
            sourceAssociationId(initSourceAssociationId),
            destinationAssociationId(initDestinationAssociationId),
            sourceNodeId(initSourceNodeId),
            destinationNodeId(initDestinationNodeId)
        {}
    };

    //---------------------------------

    // Note: Time intervals can overlap (concurrent transmissions / spatial reuse).

    vector<DataTransferIntervalListElementType> dmgDataTransferIntervalList;
    SimTime dataTransferIntervalDuration;

    //---------------------------------

    //SimTime beaconSuperframeInterval;

    unsigned int beaconSuperframeCount;

    SimTime associationBeamformingTrainingSlotStartTime;

    SimTime dataTransferIntervalStartTime;
    SimTime dataTransferIntervalEndTime;
    SimTime allocatedAccessPeriodEndTime;

    bool beamformingEstablished;

    bool shouldSendASectorSweep;
    unsigned int currentSectorSweepFrameCount;
    unsigned int numberSectorSweepFramesToSend;

    // Send frames with same antenna sector (the receiver sweeps its sector).
    bool shouldTransmitReceiveSectorSweep;

    unsigned int currentReceiveSweepSectorId;

    unsigned int currentDataTransferIntervalListIndex;

    bool receivedADirectionalBeacon;

    class DirectionalMgSuperframeScheduleEvent : public SimulationEvent {
    public:
        DirectionalMgSuperframeScheduleEvent(Dot11StaManagementController* initStaControllerPtr) :
            staControllerPtr(initStaControllerPtr) { }
        void ExecuteEvent() { staControllerPtr->ProcessDirectionalMgSuperframeScheduleEvent(); }
    private:
        Dot11StaManagementController* staControllerPtr;
    };

    shared_ptr<DirectionalMgSuperframeScheduleEvent> directionalMgSuperframeScheduleEventPtr;
    EventRescheduleTicket directionalMgSuperframeScheduleEventTicket;

    enum DmgSuperframeScheduleEventTypes {
        BeaconTransmissionIntervalPeriod,
        AssociationBeamformingTrainingPeriod,
        AssociationBeamformingTrainingSlot,
        //Future TransmitSectorSweepFeedbackAckEvent,
        SectorSweepFeedbackFrameTimeout,
        //Future AnnouncementTransmissionIntervalPeriod,
        ReceiveSectorSweepStartReceivingNextSector,
        DataTransferIntervalPeriod,
        AllocatedAccessPeriod,
        AllocatedAccessPeriodAfterGap,
        //Future DataTransferScheduledPeriod,
    };

    DmgSuperframeScheduleEventTypes nextDirectionalMgScheduleEvent;

    //------------------------------------------------------

    // For trace:
    unsigned int lastBestDownlinkTxSectorId;

    BeamformingSectorSelectorFactory sectorSelectorFactory;

    // For STA crosslinks (STA<-->STA) (breaks usual AP-STA paradigm)

    struct SectorSweepInfoType {

        shared_ptr<BeamformingSectorSelector> beamformingSectorSelectorPtr;

        bool receivedASectorSweepFrame;

        unsigned int forcedTxBeamformingSectorId;
        unsigned int forcedRxBeamformingSectorId;

        unsigned int currentBestTxBeamformingSectorId;
        unsigned int currentBestRxBeamformingSectorId;

        // Only for trace output:

        unsigned int lastBestMeasuredUplinkTxSectorId;
        unsigned int lastBestReportedDownlinkTxSectorId;

        SectorSweepInfoType() :
            receivedASectorSweepFrame(false),
            forcedTxBeamformingSectorId(InvalidSectorId),
            forcedRxBeamformingSectorId(InvalidSectorId),
            currentBestTxBeamformingSectorId(QuasiOmniSectorId),
            currentBestRxBeamformingSectorId(QuasiOmniSectorId),
            lastBestMeasuredUplinkTxSectorId(QuasiOmniSectorId),
            lastBestReportedDownlinkTxSectorId(QuasiOmniSectorId) { }

    };//StaSectorSweepInfoType//

    map<MacAddress, shared_ptr<SectorSweepInfoType> > perLinkSectorSweepInfo;


    //------------------------------------------------------

    void StartChannelScanning();
    void ChannelScanTimedOut();
    void StartAuthentication(const MacAddress& accessPointAddress);
    void AuthenticationTimeout();

    void ProcessBeaconFrame(const Packet& managementFrame);

    void AssociateWithAccessPoint(
        const unsigned int channelId,
        const MacAddress& accessPointAddress);

    void ProcessAssociateFailure();
    void CheckLinkStatus();
    void UpdateBackgroundScanEventStartTime();
    void InitiateBackgroundChannelScanning();
    void StartBackgroundChannelScanning();
    void FinishChannelScanning();
    void PerformBackgroundChannelScanning();
    void FinishBackgroundChannelScanning();
    void Disassociate();

    void SwitchToAnotherAccessPoint(
        const vector<unsigned int>& newBondedChannelList,
        const MacAddress& newAccessPoint);



    void ReadInAdParameters(const ParameterDatabaseReader& theParameterDatabaseReader);

    void StartBeaconTransmissionIntervalPeriod();

    void ProcessDirectionalMgSuperframeScheduleEvent();

    void StartAssociationBeamformingTrainingPeriod();

    unsigned int GetAllocatedAccessPeriodIndex(const unsigned int startIndex = 0) const;

    void StartDataTransferIntervalPeriod();
    void StartAllocatedAccessPeriod();
    void ContinueAllocatedAccessPeriod();
    void StartSendingInAllocatedAccessPeriod();
    void ScheduleEventForNextAllocatedAccessPeriodOrSuperframe();

    void ProcessDirectionalBeaconFrame(const Packet& aBeaconFrame);
    void ProcessDirectionalBeaconFrameForAp(const Packet& aBeaconFrame);
    void ProcessSectorSweepFeedbackFrame(const SectorSweepFeedbackFrameType& feedbackFrame);
    void ProcessSectorSweepFrame(const SectorSweepFrameType& sweepFrame);

    void ProcessExtendedScheduleElement(const Packet& aBeaconFrame, const unsigned int elementOffset);

    void TransmitSectorSweepFrame();
    void TransmitSectorSweepFeedbackAckFrame();
    void ProcessSectorSweepFeedbackTimeout();

    void ContinueReceivingReceiveSectorSweep(const bool isFirstSector);

    // Trace n Stats:

    void OutputTraceForNewDownlinkBeamformingSector();
    void OutputTraceForNewUplinkBeamformingSector();
    void OutputTraceForDmgFrameReceiveOnSta(const unsigned short int transmissionSectorId);

};//Dot11StaManagementController//


//--------------------------------------------------------------------------------------------------


inline
void Dot11StaManagementController::SetChannelScanningController(
    const shared_ptr<AbstractChannelScanningController>& newScanningControllerPtr)
{
    (*this).scanningControllerPtr = newScanningControllerPtr;

}//SetChannelScanningController//



inline
void Dot11StaManagementController::SwitchToAccessPoint(const MacAddress& accessPointAddress)
{
    assert(theStaManagementState == NotAssociated);

    if (!initializationEventTicket.IsNull()) {
        simEngineInterfacePtr->CancelEvent(initializationEventTicket);
    }//if//

    (*this).StartAuthentication(accessPointAddress);
}


inline
void Dot11StaManagementController::GetCurrentAccessPointAddress(
    bool& hasAnAccessPoint,
    MacAddress& currentAccessPointAddress) const
{
    if (theStaManagementState != Associated) {
        hasAnAccessPoint = false;
        return;
    }

    hasAnAccessPoint = true;
    currentAccessPointAddress = (*this).currentApAddress;

}//GetCurrentAccessPointAddress//


inline
void Dot11StaManagementController::AuthenticationTimeout()
{
    assert(theStaManagementState == WaitingForAuthentication);
    assert(!isAuthenticated);

    authenticationTimeoutEventTicket.Clear();

    theStaManagementState = NotAssociated;

    beamformingEstablished = false;
    (*this).StartBeaconTransmissionIntervalPeriod();

    if (!accessPointIsForced) {
        currentApAddress = MacAddress::invalidMacAddress;
    }//if//

    (*this).StartChannelScanning();

}//AuthenticationTimeout//


inline
void Dot11StaManagementController::ProcessAssociateFailure()
{
    assert((theStaManagementState == WaitingForReassociationResponse) ||
           (theStaManagementState == WaitingForAssociationResponse));

    associateFailureEventTicket.Clear();

    theStaManagementState = NotAssociated;

    beamformingEstablished = false;
    (*this).StartBeaconTransmissionIntervalPeriod();

    isAuthenticated = false;

    if (!accessPointIsForced) {
        currentApAddress = MacAddress::invalidMacAddress;
    }//if//

    (*this).StartChannelScanning();

}//ProcessAssociateFailure//



inline
void Dot11StaManagementController::UpdateBackgroundScanEventStartTime()
{
    assert(theStaManagementState != BackgroundChannelScanning);

    if (scanningControllerPtr->GetNextScanStartTime() != backgroundScanningEventTime) {
        if (scanningControllerPtr->GetNextScanStartTime() != INFINITE_TIME) {
            if (startBackgroundScanTimeoutEventTicket.IsNull()) {
                simEngineInterfacePtr->ScheduleEvent(
                    startBackgroundScanTimeoutEventPtr,
                    scanningControllerPtr->GetNextScanStartTime(),
                    (*this).startBackgroundScanTimeoutEventTicket);
            }
            else {
                simEngineInterfacePtr->RescheduleEvent(
                    startBackgroundScanTimeoutEventTicket,
                    scanningControllerPtr->GetNextScanStartTime());
            }//if//
        }
        else {
            assert(backgroundScanningEventTime != INFINITE_TIME);
            simEngineInterfacePtr->CancelEvent(startBackgroundScanTimeoutEventTicket);
        }//if//

        backgroundScanningEventTime = scanningControllerPtr->GetNextScanStartTime();
    }//if//

}//UpdateBackgroundScanEventStartTime//




inline
void Dot11StaManagementController::CheckLinkStatus()
{
    assert(theStaManagementState == Associated);

    linkStatusCheckEventTicket.Clear();

    scanningControllerPtr->CalculateLinkStatus();

    if (scanningControllerPtr->NoInRangeAccessPoints()) {
        (*this).Disassociate();
        return;
    }//if//

    if (scanningControllerPtr->ShouldSwitchAccessPoints()) {
        vector<unsigned int> newBondedChannelList;
        MacAddress newAccessPointAddress;
        scanningControllerPtr->GetAccessPointToSwitchTo(newBondedChannelList, newAccessPointAddress);
        (*this).SwitchToAnotherAccessPoint(newBondedChannelList, newAccessPointAddress);
        return;
    }//if//

    (*this).UpdateBackgroundScanEventStartTime();

    simEngineInterfacePtr->ScheduleEvent(
        linkStatusCheckEventPtr,
        scanningControllerPtr->GetNextLinkCheckTime(),
        linkStatusCheckEventTicket);

}//CheckLinkStatus//



inline
void Dot11StaManagementController::ReceiveOutgoingFrameDeliveryResults(
    const Packet& aFrame,
    const bool wasAcked)
{
    const CommonFrameHeader& header = aFrame.GetAndReinterpretPayloadData<CommonFrameHeader>();

    if (header.theFrameControlField.frameTypeAndSubtype == NULL_FRAME_TYPE_CODE) {

        if (theStaManagementState == StartingUpBackgroundChannelScanning) {

            // Perform Background Scanning even if power update did not get acked!
            // Justification: Can't even send a small packet to AP with retries, better
            // start looking for new AP immediately (could add "try again logic").

            (*this).PerformBackgroundChannelScanning();
        }
        else if (theStaManagementState == EndingBackgroundChannelScanning) {
            if (wasAcked) {
                (*this).theStaManagementState = Associated;

                assert(linkStatusCheckEventTicket.IsNull());

                //11ad simEngineInterfacePtr->ScheduleEvent(
                //11ad     linkStatusCheckEventPtr,
                //11ad     scanningControllerPtr->GetNextLinkCheckTime(),
                //11ad     (*this).linkStatusCheckEventTicket);
            }
            else {
                // Could not tell the AP, that I am awake, just try some more scanning, then try again.

                (*this).PerformBackgroundChannelScanning();
            }//if//
        }
        else {
            assert(false); abort();
        }//if//
    }//if//

}//ReceiveOutgoingFrameDeliveryResults//




inline
void Dot11StaManagementController::ProcessExtendedScheduleElement(
    const Packet& aBeaconFrame, const unsigned int elementOffset)
{
    using std::max;

    const ExtendedScheduleElementHeaderType& elementHeader =
        aBeaconFrame.GetAndReinterpretPayloadData<ExtendedScheduleElementHeaderType>(elementOffset);

    assert(elementHeader.elementId == ExtendedScheduleElementHeaderType::elementIdValue);

    (*this).dataTransferIntervalDuration = ZERO_TIME;
    dmgDataTransferIntervalList.clear();

    unsigned int currentOffset = elementOffset + sizeof(elementHeader);

    while (currentOffset < (elementOffset + elementHeader.lengthBytes)) {
        const ExtendedScheduleElementAllocationType& allocation =
            aBeaconFrame.GetAndReinterpretPayloadData<ExtendedScheduleElementAllocationType>(currentOffset);

        dmgDataTransferIntervalList.push_back(
            DataTransferIntervalListElementType(
                (allocation.allocationControl.isContentionBasedAllocationAkaCbap == 1),
                (allocation.beamformingControl.doInitiatorRxssBeamformingTraining == 1),
                (allocation.allocationStartNs * NANO_SECOND),
                (allocation.allocationDurationNs * NANO_SECOND),
                allocation.sourceAssociationId,
                allocation.destinationAssociationId,
                allocation.sourceNodeId,
                allocation.destinationNodeId));

        (*this).dataTransferIntervalDuration =
            max(
                dataTransferIntervalDuration,
                (dmgDataTransferIntervalList.back().relativeStartTime +
                 dmgDataTransferIntervalList.back().duration));

        currentOffset += ExtendedScheduleElementAllocationType::actualSizeBytes;
    }//while//

    // Check order (increasing).

    for(unsigned int i = 1; (i < dmgDataTransferIntervalList.size()); i++) {
        assert(dmgDataTransferIntervalList[i-1].relativeStartTime <=
               dmgDataTransferIntervalList[i].relativeStartTime);
    }//for//

}//ProcessExtendedScheduleElement//


inline
unsigned int Dot11StaManagementController::GetAllocatedAccessPeriodIndex(
    const unsigned int startIndex) const
{
    for(unsigned int i = startIndex; (i < dmgDataTransferIntervalList.size()); i++) {
        const DataTransferIntervalListElementType& intervalInfo = dmgDataTransferIntervalList[i];

        if ((intervalInfo.isAContentionBasedAccessPeriodAkaCbap) ||
            (intervalInfo.sourceAssociationId == theAssociationId) ||
            (intervalInfo.destinationAssociationId == theAssociationId) ||
            (intervalInfo.destinationAssociationId == BroadcastAssociationId)) {

            return (i);

        }//for//
    }//for//

    // No interval assigned.

    return (static_cast<unsigned int>(dmgDataTransferIntervalList.size()));

}//GetAllocatedAccessPeriodIndex//


inline
void Dot11StaManagementController::ProcessDirectionalMgSuperframeScheduleEvent()
{
    directionalMgSuperframeScheduleEventTicket.Clear();

    switch (nextDirectionalMgScheduleEvent) {

    case BeaconTransmissionIntervalPeriod:
        (*this).StartBeaconTransmissionIntervalPeriod();
        break;

    case AssociationBeamformingTrainingPeriod:
        (*this).StartAssociationBeamformingTrainingPeriod();
        break;

    case AssociationBeamformingTrainingSlot:
        (*this).TransmitSectorSweepFrame();
        break;

    case SectorSweepFeedbackFrameTimeout:
        (*this).ProcessSectorSweepFeedbackTimeout();
        break;

    case DataTransferIntervalPeriod:
        (*this).StartDataTransferIntervalPeriod();
        break;

    case AllocatedAccessPeriod:
        (*this).StartAllocatedAccessPeriod();
        break;

    case AllocatedAccessPeriodAfterGap:
        (*this).StartSendingInAllocatedAccessPeriod();
        break;

    case ReceiveSectorSweepStartReceivingNextSector:
        (*this).ContinueReceivingReceiveSectorSweep(false);
        break;

    default:
        assert(false); abort();
        break;

    }//switch//

}//ProcessDirectionalMgSuperframeScheduleEvent//



inline
unsigned int Dot11StaManagementController::GetTxAntennaSectorId(const MacAddress& destinationAddress) const
{
    typedef map<MacAddress, shared_ptr<SectorSweepInfoType> >::const_iterator IterType;

    IterType iter = perLinkSectorSweepInfo.find(destinationAddress);

    if (iter != perLinkSectorSweepInfo.end()) {
        const SectorSweepInfoType& info = *iter->second;
        if (info.forcedTxBeamformingSectorId == InvalidSectorId) {

            return (info.currentBestTxBeamformingSectorId);
        }
        else {
            return (info.forcedTxBeamformingSectorId);
        }//if//
    }//if//

    return (QuasiOmniSectorId);

}//GetAntennaSectorId//



inline
void Dot11StaManagementController::GetNextHopMacAddress(
    const MacAddress& destinationMacAddress,
    bool& success,
    MacAddress& nextHopMacAddress) const
{
    success = true;
    if (destinationMacAddress == currentApAddress) {
        nextHopMacAddress = destinationMacAddress;
        return;
    }//if//

    // If there is a dedicated service period then go direct.

    for (unsigned int i = 0; (i < dmgDataTransferIntervalList.size()); i++) {
        const DataTransferIntervalListElementType& intervalInfo = dmgDataTransferIntervalList[i];

        if ((intervalInfo.sourceAssociationId == theAssociationId) &&
            (intervalInfo.destinationNodeId == destinationMacAddress.ExtractNodeId())) {

            nextHopMacAddress = destinationMacAddress;
            return;
        }//if//
    }//if//

    GetCurrentAccessPointAddress(success, nextHopMacAddress);

}//GetNextHopMacAddress//



inline
bool Dot11StaManagementController::AllowDirectStaToStaTransmission(
    const MacAddress& destinationMacAddress) const
{
    if (currentDataTransferIntervalListIndex < dmgDataTransferIntervalList.size()) {

        const DataTransferIntervalListElementType& intervalInfo =
            dmgDataTransferIntervalList.at(currentDataTransferIntervalListIndex);

        return ((intervalInfo.sourceAssociationId == theAssociationId) ||
                (intervalInfo.destinationNodeId == destinationMacAddress.ExtractNodeId()));
    }//if//

    return false;
}



class ChannelScanningController : public AbstractChannelScanningController {
public:
    ChannelScanningController(
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId,
        const unsigned int numberOfChannels,
        const RandomNumberGeneratorSeed& interfaceSeed);


    virtual void ClearCurrentChannelAndAccessPoint() override
    {
        (*this).SetCurrentChannelAndAccessPoint(
            vector<unsigned int>(), MacAddress::invalidMacAddress);
    }

    virtual void SetCurrentChannelAndAccessPoint(
        const vector<unsigned int>& newBondedChannelList,
        const MacAddress newAccessPoint) override;

    virtual void CalculateLinkStatus() override;

    virtual SimTime GetNextLinkCheckTime() const override { return (nextLinkCheckTime); }
    virtual SimTime GetNextScanStartTime() const override { return (nextScanStartTime); }

    virtual void StartScanSequence() override;

    virtual void GetChannelAndDurationToScan(
        bool& scanSequenceIsDone,
        unsigned int& channelId,
        SimTime& duration) override;

    virtual void ReceiveBeaconInfo(
        const unsigned int channelId,
        const BeaconFrame& aBeaconFrame,
        const double& frameRssiDbm) override;

    virtual void ReceiveDirectionalBeaconInfo(
        const vector<unsigned int>& bondedChannelList,
        const DirectionalBeaconFrameType& aBeaconFrame,
        const double& frameRssiDbm) override;

    virtual void ReceiveProbeResponseInfo(
        const unsigned int channelId,
        const ProbeResponseFrame& probeResponseFrame,
        const double& frameRssiDbm) override
    {
        (*this).ReceiveBeaconInfo(channelId, probeResponseFrame, frameRssiDbm);
    }


    virtual bool NoInRangeAccessPoints() const override
    {
        return
            ((newAccessPointMacAddress == MacAddress::invalidMacAddress) &&
             ((shouldDisassociateFromCurrentAp) ||
              (currentAccessPointMacAddress == MacAddress::invalidMacAddress)));
    }

    virtual bool ShouldSwitchAccessPoints() const override
    {
        return
            ((newAccessPointMacAddress != MacAddress::invalidMacAddress) &&
             (newAccessPointMacAddress != currentAccessPointMacAddress));
    }

    virtual void GetAccessPointToSwitchTo(
        vector<unsigned int>& newBondedChannelList,
        MacAddress& newAccessPoint) override;

private:

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    double associationThresholdRssiDbm;
    double disassociationThresholdRssiDbm;
    double movingAverageCoefficient;
    double rssiImprovementThresholdDbm;

    SimTime nextScanStartTime;
    SimTime nextLinkCheckTime;

    unsigned int numberOfChannels;
    SimTime scanTimeoutInterval;
    SimTime linkStatusCheckInterval;

    unsigned int nextScanChannelId;
    vector<unsigned int> currentAccessPointBondedChannelList;
    MacAddress currentAccessPointMacAddress;

    vector<unsigned int> newBondedChannelList;
    MacAddress newAccessPointMacAddress;

    bool shouldDisassociateFromCurrentAp;
    SimTime maxDmgBeaconTransmissionIntervalAkaBtiDuration;

    static const int SEED_HASH = 5857901;
    RandomNumberGenerator aRandomNumberGenerator;

    struct ReceivedBeaconInformationEntry {
        static const unsigned int invalidPartitionIndex = UINT_MAX;

        vector<unsigned int> bondedChannelList;

        MacAddress accessPointAddress;
        string ssid;
        double lastRssiDbm;
        double averageRssiDbm;
        SimTime lastReceivedTime;
        bool isAverageRssiDbmUpdated;

        ReceivedBeaconInformationEntry(
            const vector<unsigned int> initBondedChannelList,
            const MacAddress& initAccessPointAddress,
            const SsidField& ssidField,
            const double& initRssiDbm,
            const SimTime& initReceivedTime)
            :
            bondedChannelList(initBondedChannelList),
            accessPointAddress(initAccessPointAddress),
            ssid(ssidField.ssid, ssidField.length),
            lastRssiDbm(initRssiDbm),
            averageRssiDbm(initRssiDbm),
            lastReceivedTime(initReceivedTime),
            isAverageRssiDbmUpdated(false)
        {}
    };//ReceivedBeaconInformationEntry//


    map<MacAddress, shared_ptr<ReceivedBeaconInformationEntry> > receivedBeaconInformation;


    //-----------------------------------------------------

    bool IsAssociated() const {
        return (currentAccessPointMacAddress != MacAddress::invalidMacAddress);
    }

    double GetCurrentAccessPointRssiDbm() const;


};//ChannelScanningController//




inline
ChannelScanningController::ChannelScanningController(
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId,
    const unsigned int initNumberOfChannels,
    const RandomNumberGeneratorSeed& interfaceSeed)
    :
    simEngineInterfacePtr(simulationEngineInterfacePtr),
    movingAverageCoefficient(0.5),
    currentAccessPointMacAddress(MacAddress::invalidMacAddress),
    scanTimeoutInterval(500 * MILLI_SECOND),
    numberOfChannels(initNumberOfChannels),
    nextLinkCheckTime(INFINITE_TIME),
    nextScanChannelId(0),
    newAccessPointMacAddress(MacAddress::invalidMacAddress),
    shouldDisassociateFromCurrentAp(false),
    rssiImprovementThresholdDbm(1.0),
    aRandomNumberGenerator(HashInputsToMakeSeed(interfaceSeed, SEED_HASH))
{
    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "channel-scan-interval"), theNodeId, theInterfaceId)) {
        scanTimeoutInterval = theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "channel-scan-interval"), theNodeId, theInterfaceId);

        if (scanTimeoutInterval < ZERO_TIME) {
            cerr << "Invalid scan timeout interval: "
                 << ConvertTimeToDoubleSecs(scanTimeoutInterval) << endl;
            exit(1);
        }
    }//if//

    SimTime startTimeMaxJitter = scanTimeoutInterval;
    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "channel-scan-start-time-max-jitter"), theNodeId, theInterfaceId)) {
        startTimeMaxJitter = theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "channel-scan-start-time-max-jitter"), theNodeId, theInterfaceId);
    }//if//

    const SimTime scanningStartJitter =
        static_cast<SimTime>(startTimeMaxJitter * aRandomNumberGenerator.GenerateRandomDouble());

    nextScanStartTime = simEngineInterfacePtr->CurrentTime() + scanningStartJitter;

    linkStatusCheckInterval = (scanTimeoutInterval * numberOfChannels);

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "link-status-check-interval"), theNodeId, theInterfaceId)) {
        linkStatusCheckInterval =
            theParameterDatabaseReader.ReadTime(
                (parameterNamePrefix + "link-status-check-interval"), theNodeId, theInterfaceId);
    }//if//


    associationThresholdRssiDbm = theParameterDatabaseReader.ReadDouble(
        (parameterNamePrefix + "preamble-detection-power-threshold-dbm"), theNodeId, theInterfaceId);

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "association-threshold-rssi-dbm"), theNodeId, theInterfaceId)) {
        associationThresholdRssiDbm =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "association-threshold-rssi-dbm"), theNodeId, theInterfaceId);
    }//if//

    disassociationThresholdRssiDbm = associationThresholdRssiDbm - 3.0;
    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "disassociation-threshold-rssi-dbm"), theNodeId, theInterfaceId)) {
        disassociationThresholdRssiDbm =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "disassociation-threshold-rssi-dbm"), theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "beacon-rssi-moving-average-coefficient"), theNodeId, theInterfaceId)) {
        movingAverageCoefficient =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "beacon-rssi-moving-average-coefficient"), theNodeId, theInterfaceId);
        if ((movingAverageCoefficient < 0.0) || (movingAverageCoefficient > 1.0)) {
            cerr << "Invalid range: dot11-beacon-rssi-moving-average-coefficient= "
                 << movingAverageCoefficient << endl;
            exit(1);
        }
    }//if//

    maxDmgBeaconTransmissionIntervalAkaBtiDuration =
        theParameterDatabaseReader.ReadTime(
            "dot11ad-beacon-transmission-interval-aka-bti-duration", theNodeId, theInterfaceId)*2;

}//ChannelScanningController()//


inline
void ChannelScanningController::SetCurrentChannelAndAccessPoint(
    const vector<unsigned int>& apBondedChannelList,
    const MacAddress newAccessPointAddress)
{
    (*this).currentAccessPointBondedChannelList = apBondedChannelList;
    (*this).currentAccessPointMacAddress = newAccessPointAddress;
    (*this).newAccessPointMacAddress = MacAddress::invalidMacAddress;
    (*this).shouldDisassociateFromCurrentAp = false;

    (*this).nextLinkCheckTime = simEngineInterfacePtr->CurrentTime() + linkStatusCheckInterval;

}//SetCurrentChannelAndAccessPoint//




inline
void ChannelScanningController::CalculateLinkStatus()
{
    assert(IsAssociated());

    bool neverReceivedABeacon = false;

    typedef map<MacAddress, shared_ptr<ReceivedBeaconInformationEntry> >::const_iterator BeaconIterType;
    BeaconIterType beaconIter = receivedBeaconInformation.find(currentAccessPointMacAddress);

    if (beaconIter == receivedBeaconInformation.end()) {
        neverReceivedABeacon = true;
    }
    else {
        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
        const SimTime lastReceivedTime = beaconIter->second->lastReceivedTime;

        if ((lastReceivedTime + linkStatusCheckInterval) < currentTime) {
            neverReceivedABeacon = true;
        }
    }//if//

    if (neverReceivedABeacon) {
        (*this).shouldDisassociateFromCurrentAp = true;
    }//if//

    (*this).nextLinkCheckTime = simEngineInterfacePtr->CurrentTime() + linkStatusCheckInterval;

}//CalculateLinkStatus//



inline
void ChannelScanningController::StartScanSequence()
{
    (*this).nextScanStartTime = INFINITE_TIME;
    (*this).nextScanChannelId = 0;

    if ((IsAssociated()) && (nextScanChannelId == currentAccessPointBondedChannelList.front())) {
        (*this).nextScanChannelId++;
    }//if//

}//StartScanSequence//


inline
void ChannelScanningController::ReceiveBeaconInfo(
    const unsigned int channelId,
    const BeaconFrame& aBeaconFrame,
    const double& frameRssiDbm)
{
    assert(false && "Is not used in 11ad");

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    map<MacAddress, shared_ptr<ReceivedBeaconInformationEntry> >::const_iterator beaconIter =
        receivedBeaconInformation.find(aBeaconFrame.managementHeader.transmitterAddress);

    vector<unsigned int> beaconsBondedChannelList;
    for(unsigned int i = 0; (i < aBeaconFrame.htOperationFrameElement.GetNumberBondedChannels()); i++) {
        beaconsBondedChannelList.push_back(aBeaconFrame.htOperationFrameElement.bondedChannelList[i]);
    }//for//

    if (beaconIter == receivedBeaconInformation.end()) {
        // make a new entry
        shared_ptr<ReceivedBeaconInformationEntry> beaconEntryPtr(
            new ReceivedBeaconInformationEntry(
                beaconsBondedChannelList,
                aBeaconFrame.managementHeader.transmitterAddress,
                aBeaconFrame.ssidElement,
                frameRssiDbm,
                currentTime));

        receivedBeaconInformation.insert(
            make_pair(aBeaconFrame.managementHeader.transmitterAddress, beaconEntryPtr));

        // Don't make decisions based on single datapoint.
    }
    else {
        //update existing entry
        const shared_ptr<ReceivedBeaconInformationEntry> beaconEntryPtr = beaconIter->second;
        beaconEntryPtr->bondedChannelList = beaconsBondedChannelList;
        beaconEntryPtr->accessPointAddress = aBeaconFrame.managementHeader.transmitterAddress;
        beaconEntryPtr->ssid = string(aBeaconFrame.ssidElement.ssid, aBeaconFrame.ssidElement.length);
        beaconEntryPtr->lastRssiDbm = frameRssiDbm;
        beaconEntryPtr->lastReceivedTime = currentTime;
        beaconEntryPtr->averageRssiDbm =
            ConvertToDb(
                (movingAverageCoefficient * ConvertToNonDb(frameRssiDbm)) +
                ((1 - movingAverageCoefficient) * ConvertToNonDb(beaconEntryPtr->averageRssiDbm)));

        if (aBeaconFrame.managementHeader.transmitterAddress == currentAccessPointMacAddress) {
            (*this).shouldDisassociateFromCurrentAp =
                (beaconEntryPtr->averageRssiDbm < disassociationThresholdRssiDbm);
        }//if//
    }//if//

}//ReceiveBeaconInfo//

inline
void ChannelScanningController::ReceiveDirectionalBeaconInfo(
    const vector<unsigned int>& bondedChannelList,
    const DirectionalBeaconFrameType& aBeaconFrame,
    const double& frameRssiDbm)
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    map<MacAddress, shared_ptr<ReceivedBeaconInformationEntry> >::const_iterator beaconIter =
        receivedBeaconInformation.find(aBeaconFrame.bssid);

    if (beaconIter == receivedBeaconInformation.end()) {
        string notUsedSsidString = "";

        // make a new entry
        shared_ptr<ReceivedBeaconInformationEntry> beaconEntryPtr(
            new ReceivedBeaconInformationEntry(
                bondedChannelList,
                aBeaconFrame.bssid,
                notUsedSsidString,
                frameRssiDbm,
                currentTime));

        receivedBeaconInformation.insert(
            make_pair(aBeaconFrame.bssid, beaconEntryPtr));

        // Don't make decisions based on single datapoint.
    }
    else {
        //update existing entry
        const shared_ptr<ReceivedBeaconInformationEntry> beaconEntryPtr = beaconIter->second;
        beaconEntryPtr->bondedChannelList = bondedChannelList;
        beaconEntryPtr->accessPointAddress = aBeaconFrame.bssid;

        const bool isInANewBeaconTransmissionInterval =
            ((beaconEntryPtr->lastReceivedTime + maxDmgBeaconTransmissionIntervalAkaBtiDuration) < currentTime);

        const double lastMaxRssiDbm = beaconEntryPtr->lastRssiDbm;

        if (isInANewBeaconTransmissionInterval) {

            if (!beaconEntryPtr->isAverageRssiDbmUpdated) {
                beaconEntryPtr->isAverageRssiDbmUpdated = true;
                beaconEntryPtr->averageRssiDbm = frameRssiDbm;
            }
            else {
                beaconEntryPtr->averageRssiDbm =
                    ConvertToDb(
                        (movingAverageCoefficient * ConvertToNonDb(lastMaxRssiDbm)) +
                        ((1 - movingAverageCoefficient) * ConvertToNonDb(beaconEntryPtr->averageRssiDbm)));
            }//if//

            beaconEntryPtr->lastRssiDbm = frameRssiDbm;
        }
        else {

            beaconEntryPtr->lastRssiDbm = std::max(beaconEntryPtr->lastRssiDbm, frameRssiDbm);

        }//if//

        beaconEntryPtr->lastReceivedTime = currentTime;


        if ((isInANewBeaconTransmissionInterval) &&
            (aBeaconFrame.bssid == currentAccessPointMacAddress)) {

            (*this).shouldDisassociateFromCurrentAp =
                ((beaconEntryPtr->lastRssiDbm < disassociationThresholdRssiDbm) &&
                 (lastMaxRssiDbm < disassociationThresholdRssiDbm));
        }//if//
    }//if//

}//ReceiveDirectionalBeaconInfo//


inline
double ChannelScanningController::GetCurrentAccessPointRssiDbm() const {
    typedef map<MacAddress, shared_ptr<ReceivedBeaconInformationEntry> >::const_iterator IterType;

    IterType iter = receivedBeaconInformation.find(currentAccessPointMacAddress);
    if (iter == receivedBeaconInformation.end()) {
        return (-DBL_MAX);
    }//if//

    const ReceivedBeaconInformationEntry& beaconInfo = *(iter->second);

    if (beaconInfo.isAverageRssiDbmUpdated) {
        return beaconInfo.averageRssiDbm;
    }//if//

    return (DBL_MAX);

}//GetCurrentAccessPointRssiDbm//



inline
void ChannelScanningController::GetChannelAndDurationToScan(
    bool& scanSequenceIsDone,
    unsigned int& channelId,
    SimTime& duration)
{
    scanSequenceIsDone = (nextScanChannelId >= numberOfChannels);

    if (!scanSequenceIsDone) {
        channelId = nextScanChannelId;
        duration = scanTimeoutInterval;

        (*this).nextScanChannelId++;
         if ((IsAssociated()) && (nextScanChannelId == currentAccessPointBondedChannelList[0])) {
             (*this).nextScanChannelId++;
        }//if//
    }
    else {
        // Scanning is done, choose access point (stay with current or go to new one).

        (*this).newAccessPointMacAddress = MacAddress::invalidMacAddress;
        (*this).newBondedChannelList.clear();

        channelId = 0;
        duration = ZERO_TIME;

        // for now, simply attempt to associate with the AP whose RSSI is highest

        typedef map<MacAddress, shared_ptr<ReceivedBeaconInformationEntry> >::const_iterator IterType;

        double maxRssiDbm = -DBL_MAX;
        MacAddress bestAccessPointMacAddress = MacAddress::invalidMacAddress;
        vector<unsigned int> bestApChannelList;

        for(IterType apIter = receivedBeaconInformation.begin(); apIter != receivedBeaconInformation.end(); ++ apIter) {

            const ReceivedBeaconInformationEntry& beaconInfo = *(apIter->second);

            if (beaconInfo.lastRssiDbm > maxRssiDbm) {
                maxRssiDbm = beaconInfo.lastRssiDbm;
                bestAccessPointMacAddress = beaconInfo.accessPointAddress;
                bestApChannelList = beaconInfo.bondedChannelList;
            }//if//
        }//for//

        if (currentAccessPointMacAddress == MacAddress::invalidMacAddress) {

            // Not connected, pick AP with best RSSI.

            if (maxRssiDbm >= associationThresholdRssiDbm) {
                (*this).newAccessPointMacAddress = bestAccessPointMacAddress;
                (*this).newBondedChannelList = bestApChannelList;
            }//if//
        }
        else {
            if (maxRssiDbm < associationThresholdRssiDbm) {
                (*this).shouldDisassociateFromCurrentAp = true;
            }
            else {
                const double rssiImprovementDbm = (maxRssiDbm - GetCurrentAccessPointRssiDbm());

                if (rssiImprovementDbm >= rssiImprovementThresholdDbm) {
                    (*this).newAccessPointMacAddress = bestAccessPointMacAddress;
                    (*this).newBondedChannelList = bestApChannelList;
                }//if//
            }//if//
        }//if//
    }//if//

}//GetChannelAndDurationToScan//



inline
void ChannelScanningController::GetAccessPointToSwitchTo(
    vector<unsigned int>& bondedChannelList,
    MacAddress& accessPointMacAddress)
{
    assert(newAccessPointMacAddress != MacAddress::invalidMacAddress);
    bondedChannelList = (*this).newBondedChannelList;
    accessPointMacAddress = (*this).newAccessPointMacAddress;
}

}//namespace//

#endif
