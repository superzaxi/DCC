// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "dot11_mac.h"

namespace Dot11 {

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

    (*this).scanningControllerPtr.reset(
        new ChannelScanningController(
            simEngineInterfacePtr,
            theParameterDatabaseReader,
            theNodeId,
            theInterfaceId,
            macLayerPtr->GetBaseChannelNumber(),
            macLayerPtr->GetNumberOfChannels(),
            interfaceSeed));

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


void Dot11StaManagementController::ProcessBeaconFrame(const Packet& managementFrame)
{
    const BeaconFrame& aBeaconFrame =
            managementFrame.GetAndReinterpretPayloadData<BeaconFrame>();

    assert(aBeaconFrame.managementHeader.header.theFrameControlField.frameTypeAndSubtype ==
           BEACON_FRAME_TYPE_CODE);

    if ((ssidString == SsidWildcardString) || (aBeaconFrame.ssidElement.IsEqualTo(ssidString))) {

        // Get the Rssi of the last received frame, which is the current frame

        const double lastFrameRssiDbm = macLayerPtr->GetRssiOfLastFrameDbm();
        const unsigned int channelId = macLayerPtr->GetCurrentChannelId();

        scanningControllerPtr->ReceiveBeaconInfo(channelId, aBeaconFrame, lastFrameRssiDbm);

        if (theStaManagementState == Associated) {
            // Check if last beacon triggered a decision to leave current AP.

            if (scanningControllerPtr->NoInRangeAccessPoints()) {
                (*this).Disassociate();
            }
            else if (scanningControllerPtr->ShouldSwitchAccessPoints()) {
                vector<unsigned int> newBondedChannelList;
                MacAddress newAccessPointAddress;

                scanningControllerPtr->GetAccessPointToSwitchTo(
                    newBondedChannelList, newAccessPointAddress);

                (*this).SwitchToAnotherAccessPoint(
                    newBondedChannelList, newAccessPointAddress);
            }//if//
        }//if//
    }//if//

}//ProcessBeaconFrame//



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
        (*this).AssociateWithAccessPoint(macLayerPtr->GetCurrentChannelId(), currentApAddress);

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
    const vector<unsigned int>& apBondedChannelList,
    const MacAddress& newApAddress)
{
    (*this).lastChannelId = macLayerPtr->GetCurrentChannelId();
    (*this).lastApAddress = currentApAddress;
    (*this).theStaManagementState = NotAssociated;
    macLayerPtr->ResetOutgoingLinksTo(lastApAddress);

    unsigned int numberOfChannels = static_cast<unsigned int>(apBondedChannelList.size());
    vector<unsigned int> newBondedChannelList = apBondedChannelList;

    if (apBondedChannelList.size() > macLayerPtr->GetMaxBandwidthNumChannels()) {
        newBondedChannelList.resize(macLayerPtr->GetMaxBandwidthNumChannels());
        numberOfChannels = static_cast<unsigned int>(newBondedChannelList.size());
    }//if//

    macLayerPtr->SwitchToChannels(newBondedChannelList);

    // Assuming AP can handle high throughput (HT) and aggregation if STA can.

    macLayerPtr->GetAdaptiveRateControllerPtr()->AddNewStation(
        newApAddress,
        numberOfChannels,
        macLayerPtr->IsAHighThroughputStation());

    if (macLayerPtr->IsAHighThroughputStation()) {

        if (macLayerPtr->MpduFrameAggregationIsEnabled()) {
            macLayerPtr->SetMpduFrameAggregationIsEnabledFor(newApAddress);
        }//if//

        //Removed Feature: if (macLayerPtr->MsduFrameAggregationIsEnabled()) {
        //Removed Feature:     macLayerPtr->SetMsduFrameAggregationIsEnabledFor(newApAddress);
        //Removed Feature: }//if//
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
    macLayerPtr->SendDisassociation(currentApAddress);
    macLayerPtr->SendLinkIsDownNotificationToNetworkLayer();
    macLayerPtr->ResetOutgoingLinksTo(currentApAddress);

    (*this).lastApAddress = currentApAddress;
    (*this).currentApAddress = MacAddress::invalidMacAddress;
    scanningControllerPtr->ClearCurrentChannelAndAccessPoint();

    //restart channel scanning  ("Legacy" i.e. comparing results, then change to else).

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    simEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(new InitializationEvent(this)),
        (currentTime + scanningStartDelay));

}//Disassociate//


}//namespace//

