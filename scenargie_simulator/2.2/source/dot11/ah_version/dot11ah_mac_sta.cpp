// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "dot11ah_mac.h"

namespace Dot11ah {

using ScenSim::EPSILON_TIME;



Dot11StaManagementController::Dot11StaManagementController(
    const shared_ptr<Dot11Mac>& initMacLayerPtr,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& initNodeId,
    const InterfaceOrInstanceId& initInterfaceId,
    const RandomNumberGeneratorSeed& interfaceSeed)
    :
    macLayerPtr(initMacLayerPtr),
    simEngineInterfacePtr(initSimulationEngineInterfacePtr),
    isARestrictedAccessWindowSta(true),
    theStaManagementState(NotAssociated),
    currentApAddress(MacAddress::invalidMacAddress),
    lastApAddress(MacAddress::invalidMacAddress),
    authenticationTimeoutInterval(1 * SECOND),
    isAuthenticated(false),
    associateFailureTimeoutInterval(1 * SECOND),
    theNodeId(initNodeId),
    theInterfaceId(initInterfaceId),
    backgroundScanningEventTime(INFINITE_TIME),
    powerSavingListenIntervalBeacons(0)
{
    scanTimeoutEventPtr.reset(new ChannelScanTimeoutEvent(this));
    authenticationTimeoutEventPtr.reset(new AuthenticationTimeoutEvent(this));
    associateFailureEventPtr.reset(new AssociateFailureEvent(this));
    linkStatusCheckEventPtr.reset(new CheckLinkStatusEvent(this));

    startRestrictedAccessWindowEventPtr.reset(new StartRestrictedAccessWindowEvent(this));
    startSlotEventPtr.reset(new StartSlotEvent(this));
    endRestrictedAccessWindowEventPtr.reset(new EndRestrictedAccessWindowEvent(this));

    (*this).scanningControllerPtr.reset(
        new ChannelScanningController(
            simEngineInterfacePtr,
            theParameterDatabaseReader,
            theNodeId,
            theInterfaceId,
            macLayerPtr->GetNumberOfChannels(),
            interfaceSeed));


    if (theParameterDatabaseReader.ParameterExists(
            "dot11ah-is-a-restricted-access-window-sta", theNodeId, theInterfaceId)) {

        isARestrictedAccessWindowSta =
            theParameterDatabaseReader.ReadBool(
            "dot11ah-is-a-restricted-access-window-sta", theNodeId, theInterfaceId);
    }//if//

    if (!isARestrictedAccessWindowSta) {
        cerr << "802.11ah Model currently does not support non-RAW STA's" << endl;
        exit(1);
    }//if//

    simEngineInterfacePtr->ScheduleEvent(
        shared_ptr<InitializationEvent>(new InitializationEvent(this)),
        scanningControllerPtr->GetNextScanStartTime(),
        initializationEventTicket);

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "authentication-timeout-interval"), theNodeId, theInterfaceId)) {

        authenticationTimeoutInterval =
            theParameterDatabaseReader.ReadTime(
                (parameterNamePrefix + "authentication-timeout-interval"), theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "associate-failure-timeout-interval"), theNodeId, theInterfaceId)) {

        associateFailureTimeoutInterval =
            theParameterDatabaseReader.ReadTime(
                (parameterNamePrefix + "associate-failure-timeout-interval"), theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "mobile-sta-ssid"), theNodeId, theInterfaceId)) {

        ssidString = theParameterDatabaseReader.ReadString(
            (parameterNamePrefix + "mobile-sta-ssid"), theNodeId, theInterfaceId);

        if (ssidString.length() > SSID_LENGTH) {
            cerr << "Error: SSID length must be " << SSID_LENGTH << " or under: " << ssidString << endl;
            exit(1);
        }//if//
    }//if//


    powerSavingListenIntervalBeacons = 0;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "-sta-power-saving-listen-interval-beacons"), theNodeId, theInterfaceId)) {

        powerSavingListenIntervalBeacons =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "-sta-power-saving-listen-interval-beacons"),
                theNodeId, theInterfaceId);

    }//if//

}//Dot11StaManagementController//



void Dot11StaManagementController::ProcessRestrictedAccessWindowParmInfoElement(
    const Packet& aBeaconFrame,
    unsigned int& offsetIndex)
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    const InfoElementHeaderType& infoHeader =
        aBeaconFrame.GetAndReinterpretPayloadData<InfoElementHeaderType>(offsetIndex);

    offsetIndex += sizeof(InfoElementHeaderType);

    (*this).restrictedAccessWindowInfo.clear();

    const unsigned int endOffsetIndex = offsetIndex + infoHeader.length;
    SimTime totalWindowDuration = ZERO_TIME;

    while (offsetIndex < endOffsetIndex) {

        const RestrictedAccessWindowAssignmentFieldType& assignmentField =
            aBeaconFrame.GetAndReinterpretPayloadData<RestrictedAccessWindowAssignmentFieldType>(
                offsetIndex);

        const SimTime rawDuration =
            (assignmentField.rawSlotDefinition.GetSlotDuration() *
            assignmentField.rawSlotDefinition.numberOfSlots);

        (*this).restrictedAccessWindowInfo.push_back(
            RestrictedAccessWindowInfoType(
                (currentTime + totalWindowDuration),
                rawDuration,
                assignmentField.rawSlotDefinition.GetSlotDuration(),
                assignmentField.rawSlotDefinition.numberOfSlots,
                IsAMemberOfRawGroupSet(assignmentField.rawGroup, theAssociationId)));

        totalWindowDuration += rawDuration;

        offsetIndex += sizeof(RestrictedAccessWindowAssignmentFieldType);

    }//while//

    if (!restrictedAccessWindowInfo.empty()) {
        simEngineInterfacePtr->ScheduleEvent(
            startRestrictedAccessWindowEventPtr,
            restrictedAccessWindowInfo.front().startTime);
    }//if//

}//ProcessRestrictedAccessWindowParmInfoElement//



void Dot11StaManagementController::ProcessBeaconFrame(const Packet& managementFrame)
{
    const BeaconFrame& aBeaconFrame =
            managementFrame.GetAndReinterpretPayloadData<BeaconFrame>();

    assert(aBeaconFrame.managementHeader.header.theFrameControlField.frameTypeAndSubtype ==
           BEACON_FRAME_TYPE_CODE);

    bool isChangingAp = false;

    if ((ssidString == SsidWildcardString) || (aBeaconFrame.ssidElement.IsEqualTo(ssidString))) {

        // Get the Rssi of the last received frame, which is the current frame

        const double lastFrameRssiDbm = macLayerPtr->GetRssiOfLastFrameDbm();
        const unsigned int channelId = macLayerPtr->GetCurrentChannelId();

        scanningControllerPtr->ReceiveBeaconInfo(channelId, aBeaconFrame, lastFrameRssiDbm);

        if (theStaManagementState == Associated) {

            // Check if last beacon triggered a decision to leave current AP.

            if (scanningControllerPtr->NoInRangeAccessPoints()) {
                isChangingAp = true;

                (*this).Disassociate();
            }
            else if (scanningControllerPtr->ShouldSwitchAccessPoints()) {
                isChangingAp = true;

                vector<unsigned int> newBondedChannelList;
                MacAddress newAccessPointAddress;
                scanningControllerPtr->GetAccessPointToSwitchTo(
                    newBondedChannelList, newAccessPointAddress);
                (*this).SwitchToAnotherAccessPoint(newBondedChannelList, newAccessPointAddress);
            }//if//
        }//if//
    }//if//

    if ((!isChangingAp) && (aBeaconFrame.managementHeader.transmitterAddress == currentApAddress)) {

        (*this).slotRandomizationOffset =
            aBeaconFrame.managementHeader.fcsRandomBitsHigh * 256 +
            aBeaconFrame.managementHeader.fcsRandomBitsLow;

        unsigned int offsetIndex = sizeof(BeaconFrame);

        while (offsetIndex < managementFrame.LengthBytes()) {

            const InfoElementHeaderType& infoHeader =
                managementFrame.GetAndReinterpretPayloadData<InfoElementHeaderType>(offsetIndex);

            if (infoHeader.elementId == RestrictedAccessWindowParmInfoElementId) {
                (*this).ProcessRestrictedAccessWindowParmInfoElement(managementFrame, offsetIndex);
            }
            else if (infoHeader.elementId == TrafficIndicatorMapInfoElementId) {
                assert(false); abort();
            }
            else {
                assert(false); abort();
            }//if//

            offsetIndex += sizeof(InfoElementHeaderType);

        }//while//
    }//if//

}//ProcessBeaconFrame//



void Dot11StaManagementController::ProcessAhResourceAllocationFrame(const Packet& aFrame)
{
    const AhResourceAllocationFrameHeaderType& header =
        aFrame.GetAndReinterpretPayloadData<AhResourceAllocationFrameHeaderType>();

    assert(header.frameControl.frameTypeAndSubtype == AH_RESOURCE_ALLOCATION_FRAME_TYPE_CODE);

    if ((header.bssId == currentApAddress) &&
        (IsAMemberOfRawGroupSet(header.rawGroup, theAssociationId))) {

        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

        assignedSlotInfos.clear();

        unsigned int offsetIndex = sizeof(AhResourceAllocationFrameHeaderType);

        while (offsetIndex < aFrame.LengthBytes()) {

            const SlotAssignmentFieldType& assignmentField =
                aFrame.GetAndReinterpretPayloadData<SlotAssignmentFieldType>(offsetIndex);

            if (assignmentField.theAssociationId == theAssociationId) {

                const SimTime slotWakeupTime =
                    currentTime + (assignmentField.slotStartOffset * TimeUnitDuration);

                assert(assignedSlotInfos.empty() || (
                       assignedSlotInfos.back().slotStartTime < slotWakeupTime) &&
                       "Slot wakeup times out of order");

                assignedSlotInfos.push_back(
                    AssignedSlotInfoType(
                        slotWakeupTime,
                        (assignmentField.uplinkIndicator == true)));
            }//if//

            offsetIndex += sizeof(SlotAssignmentFieldType);

        }//while//

        if (!assignedSlotInfos.empty()) {

            simEngineInterfacePtr->ScheduleEvent(
                startSlotEventPtr,
                assignedSlotInfos.front().slotStartTime);
        }//if//
    }//if//

}//ProcessAhResourceAllocationFrame//



void Dot11StaManagementController::ProcessManagementFrame(const Packet& managementFrame)
{
    const ManagementFrameHeader& header =
        managementFrame.GetAndReinterpretPayloadData<ManagementFrameHeader>();

    switch(header.header.theFrameControlField.frameTypeAndSubtype) {
    case ASSOCIATION_REQUEST_FRAME_TYPE_CODE:
    case REASSOCIATION_REQUEST_FRAME_TYPE_CODE:

        // ignore unexpected frame types
        break;

    case AUTHENTICATION_FRAME_TYPE_CODE: {
        // ignore unexpected frames
        if (theStaManagementState != WaitingForAuthentication) {
            return;
        }

        if (!authenticationTimeoutEventTicket.IsNull()) {
            simEngineInterfacePtr->CancelEvent(authenticationTimeoutEventTicket);
        }

        isAuthenticated = true;
        (*this).AssociateWithAccessPoint(
            macLayerPtr->GetCurrentChannelId(),
            currentApAddress);

        break;
    }
    case ASSOCIATION_RESPONSE_FRAME_TYPE_CODE: {
        // ignore unexpected frames
        if ((theStaManagementState != WaitingForAssociationResponse) ||
            (header.transmitterAddress != currentApAddress)) {

            return;
        }

        if (!associateFailureEventTicket.IsNull()) {
            simEngineInterfacePtr->CancelEvent(associateFailureEventTicket);
        }

        const AssociationResponseFrame& responseFrame =
            managementFrame.GetAndReinterpretPayloadData<AssociationResponseFrame>();

        (*this).theStaManagementState = Associated;
        (*this).theAssociationId = responseFrame.theAssociationId;

        if (isARestrictedAccessWindowSta) {
            macLayerPtr->SwitchToReceiveOnlyMode();
        }//if//

        macLayerPtr->SendLinkIsUpNotificationToNetworkLayer();

        scanningControllerPtr->SetCurrentChannelAndAccessPoint(
            macLayerPtr->GetCurrentBondedChannelList(),
            currentApAddress);

        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
        simEngineInterfacePtr->ScheduleEvent(
            linkStatusCheckEventPtr,
            scanningControllerPtr->GetNextLinkCheckTime(),
            linkStatusCheckEventTicket);

        break;
    }
    case REASSOCIATION_RESPONSE_FRAME_TYPE_CODE: {
        // ignore unexpected frames
        if ((theStaManagementState != WaitingForReassociationResponse) ||
            (header.transmitterAddress != currentApAddress)) {

            return;
        }

        if (!associateFailureEventTicket.IsNull()) {
            simEngineInterfacePtr->CancelEvent(associateFailureEventTicket);
        }

        theStaManagementState = Associated;

        macLayerPtr->SendLinkIsUpNotificationToNetworkLayer();

        scanningControllerPtr->SetCurrentChannelAndAccessPoint(
            macLayerPtr->GetCurrentBondedChannelList(),
            currentApAddress);

        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
        simEngineInterfacePtr->ScheduleEvent(
            linkStatusCheckEventPtr,
            scanningControllerPtr->GetNextLinkCheckTime(),
            linkStatusCheckEventTicket);

        break;
    }
    case DISASSOCIATION_FRAME_TYPE_CODE: {
        // ignore unexpected frames
        if ((theStaManagementState != Associated) ||
            (header.transmitterAddress != currentApAddress)) { return; }

        if (!linkStatusCheckEventTicket.IsNull()) {
            simEngineInterfacePtr->CancelEvent(linkStatusCheckEventTicket);
        }

        theStaManagementState = NotAssociated;
        isAuthenticated = false;

        macLayerPtr->SendLinkIsDownNotificationToNetworkLayer();

        (*this).lastApAddress = currentApAddress;
        (*this).currentApAddress = MacAddress::invalidMacAddress;
        scanningControllerPtr->ClearCurrentChannelAndAccessPoint();

        shared_ptr<InitializationEvent> initializationEventPtr(new InitializationEvent(this));
        simEngineInterfacePtr->ScheduleEvent(
            initializationEventPtr,
            simEngineInterfacePtr->CurrentTime());

        break;
    }
    case BEACON_FRAME_TYPE_CODE: {

        (*this).ProcessBeaconFrame(managementFrame);

        break;
    }
    case AH_RESOURCE_ALLOCATION_FRAME_TYPE_CODE: {

        (*this).ProcessAhResourceAllocationFrame(managementFrame);

        break;
    }
    default:
        assert(false); abort();
        break;
    }//switch//

}//ProcessManagementFrame//



void Dot11StaManagementController::StartChannelScanning()
{
    initializationEventTicket.Clear();

    assert(theStaManagementState == NotAssociated);

    theStaManagementState = ChannelScanning;

    scanningControllerPtr->StartScanSequence();

    bool scanSequenceIsDone;
    unsigned int channelId;
    SimTime duration;
    scanningControllerPtr->GetChannelAndDurationToScan(scanSequenceIsDone, channelId, duration);
    assert(!scanSequenceIsDone);

    macLayerPtr->SwitchToChannel(channelId);

    // wait to receive beacons
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    simEngineInterfacePtr->ScheduleEvent(
        scanTimeoutEventPtr,
        (currentTime + duration),
        scanTimeoutEventTicket);

}//StartChannelScanning//



void Dot11StaManagementController::StartBackgroundChannelScanning()
{
    assert(theStaManagementState == Associated);

    theStaManagementState = BackgroundChannelScanning;

    scanningControllerPtr->StartScanSequence();

    bool scanSequenceIsDone;
    unsigned int channelId;
    SimTime duration;
    scanningControllerPtr->GetChannelAndDurationToScan(scanSequenceIsDone, channelId, duration);
    assert(!scanSequenceIsDone);

    macLayerPtr->SwitchToChannel(channelId);

    // wait to receive beacons
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
    simEngineInterfacePtr->ScheduleEvent(
        scanTimeoutEventPtr,
        (currentTime + duration),
        scanTimeoutEventTicket);

}//StartChannelScanning//


inline
void Dot11StaManagementController::SwitchToAnotherAccessPoint(
    const vector<unsigned int>& newBondedChannelList,
    const MacAddress& newApAddress)
{
    (*this).lastChannelId = macLayerPtr->GetCurrentChannelId();
    (*this).lastApAddress = currentApAddress;
    (*this).theStaManagementState = NotAssociated;
    macLayerPtr->RequeueBufferedPackets();
    macLayerPtr->SwitchToChannels(newBondedChannelList);
    // Assuming AP can handle aggregation if STA can.

    if (macLayerPtr->MpduFrameAggregationIsEnabled()) {
        macLayerPtr->SetMpduFrameAggregationIsEnabledFor(newApAddress);
    }//if//

    (*this).isAuthenticated = false;
    scanningControllerPtr->ClearCurrentChannelAndAccessPoint();
    (*this).StartAuthentication(newApAddress);

}//SwitchToAnotherAccessPoint//



void Dot11StaManagementController::FinishChannelScanning()
{
    // finished scanning

    if (theStaManagementState == ChannelScanning) {
        (*this).theStaManagementState = NotAssociated;
        if (scanningControllerPtr->NoInRangeAccessPoints()) {
            // Just keep scanning.

            (*this).StartChannelScanning();
        }
        else if (scanningControllerPtr->ShouldSwitchAccessPoints()) {
            vector<unsigned int> newBondedChannelList;
            MacAddress newApAddress;
            scanningControllerPtr->GetAccessPointToSwitchTo(newBondedChannelList, newApAddress);
            (*this).SwitchToAnotherAccessPoint(newBondedChannelList, newApAddress);
        }
        else {
            assert(false); abort();
        }//if//
    }
    else if (theStaManagementState == BackgroundChannelScanning) {

        (*this).theStaManagementState = Associated;
        if (scanningControllerPtr->ShouldSwitchAccessPoints()) {
            vector<unsigned int> newBondedChannelList;
            MacAddress newApAddress;
            scanningControllerPtr->GetAccessPointToSwitchTo(newBondedChannelList, newApAddress);
            (*this).SwitchToAnotherAccessPoint(newBondedChannelList, newApAddress);
        }
        else {
            (*this).FinishBackgroundChannelScanning();
        }//if//
    }
    else {
        assert(false); abort();
    }//if//

}//FinishChannelScanning//



void Dot11StaManagementController::ChannelScanTimedOut()
{
    scanTimeoutEventTicket.Clear();

    assert((theStaManagementState == BackgroundChannelScanning) ||
           (theStaManagementState == ChannelScanning));

    bool scanSequenceIsDone;
    unsigned int channelId;
    SimTime duration;
    scanningControllerPtr->GetChannelAndDurationToScan(scanSequenceIsDone, channelId, duration);

    if (!scanSequenceIsDone) {

        macLayerPtr->SwitchToChannel(channelId);

        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
        simEngineInterfacePtr->ScheduleEvent(
            scanTimeoutEventPtr,
            (currentTime + duration),
            scanTimeoutEventTicket);
   }
   else {
       (*this).FinishChannelScanning();
    }//if//

}//ChannelScanTimedOut//



void Dot11StaManagementController::InitiateBackgroundChannelScanning()
{
    assert(theStaManagementState == Associated);
    (*this).theStaManagementState = StartingUpBackgroundChannelScanning;
    (*this).startBackgroundScanTimeoutEventTicket.Clear();
    (*this).backgroundScanningEventTime = INFINITE_TIME;
    macLayerPtr->SendPowerSaveNullFrame(currentApAddress, true);

    if (!linkStatusCheckEventTicket.IsNull()) {
        simEngineInterfacePtr->CancelEvent(linkStatusCheckEventTicket);
    }//if//

}//StartBackgroundChannelScanning//


void Dot11StaManagementController::FinishBackgroundChannelScanning()
{
    assert(theStaManagementState == BackgroundChannelScanning);
    (*this).theStaManagementState = EndingBackgroundChannelScanning;
    macLayerPtr->SendPowerSaveNullFrame(currentApAddress, false);

}//FinishBackgroundChannelScanning//



void Dot11StaManagementController::StartAuthentication(const MacAddress& accessPointAddress)
{
    assert(theStaManagementState == NotAssociated);
    assert(!isAuthenticated);

    currentApAddress = accessPointAddress;

    theStaManagementState = WaitingForAuthentication;

    if (isARestrictedAccessWindowSta) {
        macLayerPtr->SwitchToNormalAccessMode();
    }//if//

    macLayerPtr->SendAuthentication(accessPointAddress);

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
    simEngineInterfacePtr->ScheduleEvent(
        authenticationTimeoutEventPtr,
        currentTime + authenticationTimeoutInterval,
        authenticationTimeoutEventTicket);

}//StartAuthentication//



void Dot11StaManagementController::AssociateWithAccessPoint(
    const unsigned int channelId,
    const MacAddress& accessPointAddress)
{
    assert(isAuthenticated);

    if (isARestrictedAccessWindowSta) {
        macLayerPtr->SwitchToNormalAccessMode();
    }//if//

    if (theStaManagementState == Associated) {
        //handover
        lastApAddress = currentApAddress;
        currentApAddress = accessPointAddress;
        macLayerPtr->SendReassociationRequest(accessPointAddress, lastApAddress);

        theStaManagementState = WaitingForReassociationResponse;

    }
    else if (theStaManagementState == WaitingForAuthentication) {
        //new association
        currentApAddress = accessPointAddress;
        macLayerPtr->SendAssociationRequest(accessPointAddress);

        theStaManagementState = WaitingForAssociationResponse;
    }
    else {
        assert(false); abort();
    }//if//

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
    simEngineInterfacePtr->ScheduleEvent(
        associateFailureEventPtr,
        (currentTime + associateFailureTimeoutInterval),
        associateFailureEventTicket);

}//AssociateWithAccessPoint//



void Dot11StaManagementController::Disassociate()
{
    assert(theStaManagementState == Associated);

    if (!linkStatusCheckEventTicket.IsNull()) {
        simEngineInterfacePtr->CancelEvent(linkStatusCheckEventTicket);
    }
    theStaManagementState = NotAssociated;
    isAuthenticated = false;
    macLayerPtr->RequeueBufferedPackets();
    macLayerPtr->SendDisassociation(currentApAddress);
    macLayerPtr->SendLinkIsDownNotificationToNetworkLayer();

    (*this).lastApAddress = currentApAddress;
    (*this).currentApAddress = MacAddress::invalidMacAddress;
    scanningControllerPtr->ClearCurrentChannelAndAccessPoint();

    //restart channel scanning  ("Legacy" i.e. comparing results, then change to else).

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    simEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(new InitializationEvent(this)),
        (currentTime + scanningStartDelay));

}//Disassociate//



void Dot11StaManagementController::ProcessStartRestrictedAccessWindowEvent()
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    const RestrictedAccessWindowInfoType& window = restrictedAccessWindowInfo.front();

    assert(window.startTime == currentTime);

    // Start listening if power saving.... TBD

    if (window.allowedToSendInWindow) {
        const unsigned int slotIndex =
            (theAssociationId + slotRandomizationOffset) % window.numberOfSlots;

        const SimTime slotStartTime = currentTime + (slotIndex * window.slotDuration);

        simEngineInterfacePtr->ScheduleEvent(startSlotEventPtr, slotStartTime);
    }
    else {
        macLayerPtr->SwitchToSleepMode();

        const SimTime windowEndTime = window.startTime + window.duration;

        restrictedAccessWindowInfo.pop_front();

        if ((!restrictedAccessWindowInfo.empty()) &&
            (restrictedAccessWindowInfo.front().startTime == windowEndTime)) {

            simEngineInterfacePtr->ScheduleEvent(
                startRestrictedAccessWindowEventPtr,
                restrictedAccessWindowInfo.front().startTime);
        }
        else {
            simEngineInterfacePtr->ScheduleEvent(endRestrictedAccessWindowEventPtr, windowEndTime);
        }//if//
    }//if//

}//ProcessStartRestrictedAccessWindowEvent//


void Dot11StaManagementController::ProcessEndRestrictedAccessWindowEvent()
{
    // Note (TBD): assuming no "free for all" non-RAW areas.
    assert(isARestrictedAccessWindowSta);
    macLayerPtr->SwitchToReceiveOnlyMode();

    if (!restrictedAccessWindowInfo.empty()) {
        // Restricted Access Window not for me.

        simEngineInterfacePtr->ScheduleEvent(
            startRestrictedAccessWindowEventPtr,
            restrictedAccessWindowInfo.front().startTime);
    }//if//

}//ProcessEndRestrictedAccessWindowEvent//



void Dot11StaManagementController::ProcessStartSlotEvent()
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    const RestrictedAccessWindowInfoType& window = restrictedAccessWindowInfo.front();

    macLayerPtr->StartRestrictedAccessWindowPeriod(currentTime + window.slotDuration - EPSILON_TIME);

    const SimTime windowEndTime = window.startTime + window.duration;
    restrictedAccessWindowInfo.pop_front();

    if ((!restrictedAccessWindowInfo.empty()) &&
        (restrictedAccessWindowInfo.front().startTime == windowEndTime)) {

        simEngineInterfacePtr->ScheduleEvent(
            startRestrictedAccessWindowEventPtr,
            restrictedAccessWindowInfo.front().startTime);
    }
    else {
        simEngineInterfacePtr->ScheduleEvent(endRestrictedAccessWindowEventPtr, windowEndTime);
    }//if//

}//ProcessStartSlotEvent//


//Unused void Dot11StaManagementController::ProcessEndSlotEvent()
//Unused {
//Unused
//Unused }//ProcessStartSlotEvent//




}//namespace//





