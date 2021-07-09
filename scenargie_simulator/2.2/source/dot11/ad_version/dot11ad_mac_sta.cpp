// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "dot11ad_mac.h"

namespace Dot11ad {

using ScenSim::ConvertAStringSequenceOfNumericPairsIntoAMap;
using ScenSim::DeleteTrailingSpaces;

void Dot11StaManagementController::ReadInAdParameters(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    typedef map<NodeId, unsigned int>::iterator IterType;

    map<NodeId, unsigned int> forcedTxBeamformingSectorIdMap;

    const string sectorListParmName = (adParamNamePrefix + "forced-beamforming-sector-list");

    if (theParameterDatabaseReader.ParameterExists(sectorListParmName, theNodeId, theInterfaceId)) {

        string forcedBeamformingSectorListString =
            theParameterDatabaseReader.ReadString(sectorListParmName, theNodeId, theInterfaceId);

        DeleteTrailingSpaces(forcedBeamformingSectorListString);

        bool success;
        ConvertAStringSequenceOfNumericPairsIntoAMap<NodeId, unsigned int>(
            forcedBeamformingSectorListString,
            success,
            forcedTxBeamformingSectorIdMap);

        if (!success) {
            cerr << "Error: In \"" << sectorListParmName << "\" parameter." << endl;
            exit(1);
        }//if//

        CheckSectorIdMap(
            theParameterDatabaseReader,
            macLayerPtr->GetNumberAntennaSectors(),
            forcedTxBeamformingSectorIdMap,
            sectorListParmName);

    }//if//

    map<NodeId, unsigned int> forcedRxBeamformingSectorIdMap;

    const string rxSectorListParmName = (adParamNamePrefix + "forced-rx-beamforming-sector-list");

    if (theParameterDatabaseReader.ParameterExists(rxSectorListParmName, theNodeId, theInterfaceId)) {

        string forcedBeamformingSectorListString =
            theParameterDatabaseReader.ReadString(rxSectorListParmName, theNodeId, theInterfaceId);

        DeleteTrailingSpaces(forcedBeamformingSectorListString);

        bool success;
        ConvertAStringSequenceOfNumericPairsIntoAMap<NodeId, unsigned int>(
            forcedBeamformingSectorListString,
            success,
            forcedRxBeamformingSectorIdMap);

        if (!success) {
            cerr << "Error: In \"" << rxSectorListParmName << "\" parameter." << endl;
            exit(1);
        }//if//

        CheckSectorIdMap(
            theParameterDatabaseReader,
            macLayerPtr->GetNumberAntennaSectors(),
            forcedRxBeamformingSectorIdMap,
            rxSectorListParmName);

    }//if//

    map<NodeId, double> forcedLinkTxPowerDbmMap;

    const string forcedLinkTxPowerDbmListParmName = (adParamNamePrefix + "forced-link-tx-power-dbm-list");

    if (theParameterDatabaseReader.ParameterExists(forcedLinkTxPowerDbmListParmName, theNodeId, theInterfaceId)) {

        string forcedLinkTxPowerDbmListString =
            theParameterDatabaseReader.ReadString(forcedLinkTxPowerDbmListParmName, theNodeId, theInterfaceId);

        DeleteTrailingSpaces(forcedLinkTxPowerDbmListString);

        bool success;
        ConvertAStringSequenceOfNumericPairsIntoAMap<NodeId, double>(
            forcedLinkTxPowerDbmListString,
            success,
            forcedLinkTxPowerDbmMap);

        if (!success) {
            cerr << "Error: In \"" << forcedLinkTxPowerDbmListParmName << "\" parameter." << endl;
            exit(1);
        }//if//
    }//if//

    for(IterType iter = forcedTxBeamformingSectorIdMap.begin();
         (iter != forcedTxBeamformingSectorIdMap.end()); ++iter) {

        const NodeId& aNodeId = iter->first;
        const MacAddress aMacAddress = macLayerPtr->MakeMacAddressFromNodeId(aNodeId);
        const unsigned int sectorId = iter->second;

        shared_ptr<SectorSweepInfoType> newSectorSweepInfoPtr(new SectorSweepInfoType());
        newSectorSweepInfoPtr->beamformingSectorSelectorPtr = sectorSelectorFactory.CreateSectorSelector();

        // Forced Tx and Rx sectors agree by default.

        newSectorSweepInfoPtr->forcedTxBeamformingSectorId = sectorId;
        newSectorSweepInfoPtr->forcedRxBeamformingSectorId = sectorId;

        map<NodeId, double>::const_iterator powerIter = forcedLinkTxPowerDbmMap.find(aNodeId);

        if (powerIter != forcedLinkTxPowerDbmMap.end()) {
            macLayerPtr->GetAdaptiveTxPowerControllerPtr()->SetForcedLinkTxPower(
                aMacAddress, powerIter->second);
        }//if//

        perLinkSectorSweepInfo.insert(make_pair(aMacAddress, newSectorSweepInfoPtr));
    }//for//


    for(IterType iter = forcedRxBeamformingSectorIdMap.begin();
         (iter != forcedRxBeamformingSectorIdMap.end()); ++iter) {

        const MacAddress aMacAddress = macLayerPtr->MakeMacAddressFromNodeId(iter->first);
        const unsigned int sectorId = iter->second;

        if (perLinkSectorSweepInfo.find(aMacAddress) != perLinkSectorSweepInfo.end()) {
            perLinkSectorSweepInfo.find(aMacAddress)->second->forcedRxBeamformingSectorId = sectorId;
        }
        else {
            cerr << "Error: In \"" << rxSectorListParmName << "\" parameter:" << endl;
            cerr << "Forced RX beamforming for a node must have corresponding forced TX beamforming defined." << endl;
            exit(1);
        }//if//
    }//for//

    if (theParameterDatabaseReader.ParameterExists(
        (adParamNamePrefix + "forced-ap-pcp-nodeid"), theNodeId, theInterfaceId)) {

        const NodeId readNodeId = static_cast<NodeId>(
            theParameterDatabaseReader.ReadInt(
            (adParamNamePrefix + "forced-ap-pcp-nodeid"), theNodeId, theInterfaceId));

        if (!theParameterDatabaseReader.CommNodeIdExists(readNodeId)) {
            cerr << "Error in parameter: " << (adParamNamePrefix + "forced-ap-pcp-nodeid")
                 << ": node ID was not found." << endl;
            exit(1);
        }//if//

        (*this).forcedApAddress = macLayerPtr->MakeMacAddressFromNodeId(readNodeId);
        (*this).accessPointIsForced = true;

    }//if//

}//ReadInAdParameters//


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
    powerSavingListenIntervalBeacons(0),
    beamformingEstablished(false),
    receivedADirectionalBeacon(false),
    shouldSendASectorSweep(false),
    dataTransferIntervalDuration(ZERO_TIME),
    dataTransferIntervalStartTime(INFINITE_TIME),
    dataTransferIntervalEndTime(INFINITE_TIME),
    allocatedAccessPeriodEndTime(ZERO_TIME),
    beaconSuperframeCount(0),
    lastBestDownlinkTxSectorId(QuasiOmniSectorId),
    shouldTransmitReceiveSectorSweep(false),
    currentSectorSweepFrameCount(0),
    currentReceiveSweepSectorId(QuasiOmniSectorId),
    associationBeamformingTrainingAkaAbftNumSlots(0),
    associationBeamformingTrainingAkaAbftSlotTime(ZERO_TIME),
    sectorSelectorFactory(theParameterDatabaseReader, initNodeId, initInterfaceId),
    accessPointIsForced(false),
    theAssociationId(InvalidAssociationId),
    aRandomNumberGenerator(HashInputsToMakeSeed(interfaceSeed, seedHash)),
    tracePrecisionDigitsForDbm(8)
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
            macLayerPtr->GetNumberOfChannels(),
            interfaceSeed));

    directionalMgSuperframeScheduleEventPtr.reset(new DirectionalMgSuperframeScheduleEvent(this));

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


    (*this).ReadInAdParameters(theParameterDatabaseReader);

    macLayerPtr->SwitchToQuasiOmniAntennaMode();

}//Dot11StaManagementController//


void Dot11StaManagementController::ProcessBeaconFrame(const Packet& managementFrame)
{
    assert(false && "Is not used in 11ad");

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
    const CommonFrameHeader& commonHeader =
        managementFrame.GetAndReinterpretPayloadData<CommonFrameHeader>();

    switch (commonHeader.theFrameControlField.frameTypeAndSubtype) {
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
        const ManagementFrameHeader& header =
            managementFrame.GetAndReinterpretPayloadData<ManagementFrameHeader>();

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

        if (!accessPointIsForced) {
            const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
            simEngineInterfacePtr->ScheduleEvent(
                linkStatusCheckEventPtr,
                scanningControllerPtr->GetNextLinkCheckTime(),
                linkStatusCheckEventTicket);
        }//if//

        break;
    }
    case REASSOCIATION_RESPONSE_FRAME_TYPE_CODE: {

        const ManagementFrameHeader& header =
            managementFrame.GetAndReinterpretPayloadData<ManagementFrameHeader>();

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

        if (!accessPointIsForced) {
            const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
            simEngineInterfacePtr->ScheduleEvent(
                linkStatusCheckEventPtr,
                scanningControllerPtr->GetNextLinkCheckTime(),
                linkStatusCheckEventTicket);
        }//if//

        break;
    }
    case DISASSOCIATION_FRAME_TYPE_CODE: {
        const ManagementFrameHeader& header =
            managementFrame.GetAndReinterpretPayloadData<ManagementFrameHeader>();

        // ignore unexpected frames
        if ((theStaManagementState != Associated) ||
            (header.transmitterAddress != currentApAddress)) { return; }

        if (!linkStatusCheckEventTicket.IsNull()) {
            simEngineInterfacePtr->CancelEvent(linkStatusCheckEventTicket);
        }

        theStaManagementState = NotAssociated;

        beamformingEstablished = false;
        (*this).StartBeaconTransmissionIntervalPeriod();

        isAuthenticated = false;

        macLayerPtr->SendLinkIsDownNotificationToNetworkLayer();

        (*this).lastApAddress = currentApAddress;
        if (!accessPointIsForced) {
            (*this).currentApAddress = MacAddress::invalidMacAddress;
        }//if//
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

    case DMG_BEACON_FRAME_TYPE_CODE: {

        (*this).ProcessDirectionalBeaconFrame(managementFrame);

        break;
    }

    case SECTOR_SWEEP_FEEDBACK_FRAME_TYPE_CODE: {

        const SectorSweepFeedbackFrameType& feedbackFrame =
            managementFrame.GetAndReinterpretPayloadData<SectorSweepFeedbackFrameType>();

        (*this).ProcessSectorSweepFeedbackFrame(feedbackFrame);
        break;
    }

    case SECTOR_SWEEP_FRAME_TYPE_CODE: {

        const SectorSweepFrameType& sweepFrame =
            managementFrame.GetAndReinterpretPayloadData<SectorSweepFrameType>();

        (*this).ProcessSectorSweepFrame(sweepFrame);
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

    if (accessPointIsForced) {
        if (currentApAddress != MacAddress::invalidMacAddress) {
            return;
        }//if//
    }//if//

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

    //11ad// if (macLayerPtr->IsAHighThroughputStation()) {

    if (macLayerPtr->MpduFrameAggregationIsEnabled()) {
        macLayerPtr->SetMpduFrameAggregationIsEnabledFor(newApAddress);
    }//if//

    //Removed Feature: if (macLayerPtr->MsduFrameAggregationIsEnabled()) {
    //Removed Feature:     macLayerPtr->SetMsduFrameAggregationIsEnabledFor(newApAddress);
    //Removed Feature: }//if//
    //11ad// }//if//

    beamformingEstablished = false;
    (*this).StartBeaconTransmissionIntervalPeriod();

    (*this).isAuthenticated = false;
    scanningControllerPtr->ClearCurrentChannelAndAccessPoint();

    // Cheating assocition
    currentApAddress = newApAddress;

    if (!initializationEventTicket.IsNull()) {
        simEngineInterfacePtr->CancelEvent(initializationEventTicket);
    }//if//
    if (!scanTimeoutEventTicket.IsNull()) {
        simEngineInterfacePtr->CancelEvent(scanTimeoutEventTicket);
    }//if//

    //11ad// (*this).StartAuthentication(newApAddress);

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

    assert((!accessPointIsForced) || (currentApAddress == accessPointAddress));
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
        assert((!accessPointIsForced) || (currentApAddress == accessPointAddress));
        currentApAddress = accessPointAddress;
        macLayerPtr->SendReassociationRequest(accessPointAddress, lastApAddress);

        theStaManagementState = WaitingForReassociationResponse;

    }
    else if (theStaManagementState == WaitingForAuthentication) {
        //new association
        assert((!accessPointIsForced) || (currentApAddress == accessPointAddress));
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
    if (!accessPointIsForced) {
        (*this).currentApAddress = MacAddress::invalidMacAddress;
    }//if//
    scanningControllerPtr->ClearCurrentChannelAndAccessPoint();

    //restart channel scanning  ("Legacy" i.e. comparing results, then change to else).

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    simEngineInterfacePtr->ScheduleEvent(
        shared_ptr<SimulationEvent>(new InitializationEvent(this)),
        (currentTime + scanningStartDelay));

}//Disassociate//


//--------------------------------------------------------------------------------------------------


void Dot11StaManagementController::StartBeaconTransmissionIntervalPeriod()
{

    if (!directionalMgSuperframeScheduleEventTicket.IsNull()) {
        simEngineInterfacePtr->CancelEvent(directionalMgSuperframeScheduleEventTicket);
    }//if//

    (*this).receivedADirectionalBeacon = false;
    (*this).shouldSendASectorSweep = false;
    macLayerPtr->SwitchToQuasiOmniAntennaMode();
    macLayerPtr->StartReceivingFrames();

}//StartBeaconTransmissionIntervalPeriod//



void Dot11StaManagementController::ProcessDirectionalBeaconFrame(const Packet& aBeaconFrame)
{
    typedef map<MacAddress, shared_ptr<SectorSweepInfoType> >::iterator IterType;

    // Get the Rssi of the last received frame, which is the current frame
    const double lastFrameRssiDbm = macLayerPtr->GetRssiOfLastFrameDbm();

    const DirectionalBeaconFrameType& beaconFrameHeader =
        aBeaconFrame.GetAndReinterpretPayloadData<DirectionalBeaconFrameType>();

    assert(beaconFrameHeader.theFrameControlField.frameTypeAndSubtype == DMG_BEACON_FRAME_TYPE_CODE);

    // Cheating: Assume if AP's node ID matches on this channel, then right AP interface.
    // Theoretically can have two interfaces on same channel.

    vector<unsigned int> bondedChannelList;

    for(size_t i = 0; i < beaconFrameHeader.GetNumberBondedChannels(); i++) {
        bondedChannelList.push_back(beaconFrameHeader.bondedChannelList[i]);
    }//for//


    if (accessPointIsForced) {
        if ((beaconFrameHeader.bssid == forcedApAddress) &&
            (beaconFrameHeader.bssid != currentApAddress) &&
            (beaconFrameHeader.bssid.ExtractNodeId() == forcedApAddress.ExtractNodeId())) {

            (*this).SwitchToAnotherAccessPoint(
                bondedChannelList,
                beaconFrameHeader.bssid);
        }//if//
    }
    else {
        scanningControllerPtr->ReceiveDirectionalBeaconInfo(
            bondedChannelList,
            beaconFrameHeader,
            lastFrameRssiDbm);

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
                (*this).SwitchToAnotherAccessPoint(newBondedChannelList, newAccessPointAddress);
            }//if//
        }//if//
    }//if//



    if (beaconFrameHeader.bssid == currentApAddress) {

        (*this).ProcessDirectionalBeaconFrameForAp(aBeaconFrame);
    }//if//

}//ProcessDirectionalBeaconFrame//


void Dot11StaManagementController::ProcessDirectionalBeaconFrameForAp(const Packet& aBeaconFrame)
{
    typedef map<MacAddress, shared_ptr<SectorSweepInfoType> >::iterator IterType;

    const DirectionalBeaconFrameType& beaconFrameHeader =
        aBeaconFrame.GetAndReinterpretPayloadData<DirectionalBeaconFrameType>();

    assert(beaconFrameHeader.theFrameControlField.frameTypeAndSubtype == DMG_BEACON_FRAME_TYPE_CODE);

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    IterType iter = perLinkSectorSweepInfo.find(beaconFrameHeader.bssid);
    if (iter == perLinkSectorSweepInfo.end()) {
        shared_ptr<SectorSweepInfoType> newSectorSweepInfoPtr(new SectorSweepInfoType());
        newSectorSweepInfoPtr->beamformingSectorSelectorPtr = sectorSelectorFactory.CreateSectorSelector();

        perLinkSectorSweepInfo.insert(make_pair(beaconFrameHeader.bssid, newSectorSweepInfoPtr));

        assert(perLinkSectorSweepInfo[beaconFrameHeader.bssid]->beamformingSectorSelectorPtr != nullptr);

        iter = perLinkSectorSweepInfo.find(beaconFrameHeader.bssid);
    }//if//

    SectorSweepInfoType& sectorSweepInfo = *iter->second;

    if (!receivedADirectionalBeacon) {

        (*this).receivedADirectionalBeacon = true;

        const SimTime shortInterframeSpaceDuration =
            macLayerPtr->GetShortBeamformingInterframeSpaceDuration();

        const SimTime mediumInterframeSpaceDuration =
            macLayerPtr->GetMediumBeamformingInterframeSpaceDuration();

        // Add time tick so that transmissions in A-BFT are slightly later than start of
        // A-BFT at receiver (even if propagation time is 0).

        const SimTime nextAssociationBeamformingTrainingPeriodStartTime =
            macLayerPtr->GetStartTimeOfLastFrame() +
            ConvertDurationFieldValueToTime(beaconFrameHeader.duration) +
            EPSILON_TIME;

        (*this).shouldTransmitReceiveSectorSweep =
            (beaconFrameHeader.beaconIntervalControlField.isResponderTxss == 0);

        (*this).associationBeamformingTrainingAkaAbftNumSlots =
            beaconFrameHeader.beaconIntervalControlField.numSectorSweepSlots;

        (*this).associationBeamformingTrainingAkaAbftSlotTime =
            macLayerPtr->CalcAssociationBeamformingTrainingSlotTime(
                beaconFrameHeader.beaconIntervalControlField.numSectorSweepFrames);

        const SimTime associationBeamformingTrainingAkaAbftDuration =
            associationBeamformingTrainingAkaAbftNumSlots *
            associationBeamformingTrainingAkaAbftSlotTime;

        (*this).dataTransferIntervalStartTime =
            nextAssociationBeamformingTrainingPeriodStartTime +
            associationBeamformingTrainingAkaAbftDuration;

        (*this).ProcessExtendedScheduleElement(aBeaconFrame, sizeof(beaconFrameHeader));

        (*this).dataTransferIntervalEndTime =
            (dataTransferIntervalStartTime + dataTransferIntervalDuration);

        (*this).nextDirectionalMgScheduleEvent = AssociationBeamformingTrainingPeriod;

        simEngineInterfacePtr->ScheduleEvent(
            directionalMgSuperframeScheduleEventPtr,
            nextAssociationBeamformingTrainingPeriodStartTime,
            directionalMgSuperframeScheduleEventTicket);


        (*this).lastBestDownlinkTxSectorId =
            sectorSweepInfo.beamformingSectorSelectorPtr->GetBestTransmissionSectorIdToThisNode();

        sectorSweepInfo.beamformingSectorSelectorPtr->StartNewBeaconInterval();

        if (shouldTransmitReceiveSectorSweep) {

            (*this).numberSectorSweepFramesToSend =
                beaconFrameHeader.beaconIntervalControlField.numSectorSweepFrames;
        }
        else {
            if (beaconFrameHeader.beaconIntervalControlField.numSectorSweepFrames <
                macLayerPtr->GetNumberAntennaSectors()) {

                cerr << "Error: Sector sweep by STA exceeds max number sweep frames in parameters:" << endl;
                cerr <<      "\"dot11ad-abft-max-num-*-frames\"." << endl;
                exit(1);

            }//if//

            (*this).numberSectorSweepFramesToSend = macLayerPtr->GetNumberAntennaSectors();

        }//if//
    }//if//

    sectorSweepInfo.beamformingSectorSelectorPtr->ReceiveSectorMetrics(
        currentTime,
        beaconFrameHeader.sectorSweepField.sectorId,
        currentReceiveSweepSectorId,
        macLayerPtr->GetRssiOfLastFrameDbm(),
        macLayerPtr->GetSinrOfLastFrameDb());

    OutputTraceForDmgFrameReceiveOnSta(beaconFrameHeader.sectorSweepField.sectorId);

    // Future: Sector sweeps should not be every time, because many STA's on a AP/PCP
    // would overload the sector sweep period.

    //if (!beamformingEstablished) {
    shouldSendASectorSweep = true;
    //}//if//

}//ProcessDirectionalBeaconFrameForAp//



void Dot11StaManagementController::TransmitSectorSweepFrame()
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    const SectorSweepInfoType& sectorSweepInfo =
        *perLinkSectorSweepInfo[currentApAddress];

    const BeamformingSectorSelector& beamformingSectorSelectorForAp =
        *sectorSweepInfo.beamformingSectorSelectorPtr;

    if (currentSectorSweepFrameCount == 0) {

        (*this).associationBeamformingTrainingSlotStartTime = currentTime;

        if (beamformingSectorSelectorForAp.GetBestTransmissionSectorIdToThisNode() !=
            lastBestDownlinkTxSectorId) {

            OutputTraceForNewDownlinkBeamformingSector();
        }//if//
    }//if//

    if (currentSectorSweepFrameCount < numberSectorSweepFramesToSend) {

        SectorSweepFrameType sectorSweepFrame;

        sectorSweepFrame.header.theFrameControlField.frameTypeAndSubtype = SECTOR_SWEEP_FRAME_TYPE_CODE;
        sectorSweepFrame.header.theFrameControlField.isRetry = 0;
        sectorSweepFrame.header.duration = 0;
        sectorSweepFrame.header.receiverAddress = currentApAddress;
        sectorSweepFrame.transmitterAddress = macLayerPtr->GetMacAddress();
        sectorSweepFrame.sectorSweepField.direction = 1;

        if (!shouldTransmitReceiveSectorSweep) {
            sectorSweepFrame.sectorSweepField.sectorId = currentSectorSweepFrameCount;
        }
        else {
            sectorSweepFrame.sectorSweepField.sectorId =
                sectorSweepInfo.currentBestTxBeamformingSectorId;
        }//if//

        sectorSweepFrame.sectorSweepField.cdown =
            (numberSectorSweepFramesToSend - 1) - currentSectorSweepFrameCount;

        sectorSweepFrame.sectorSweepFeedbackField.bestSectorId =
            beamformingSectorSelectorForAp.GetBestTransmissionSectorIdToThisNode();

        unique_ptr<Packet> framePtr = Packet::CreatePacket(*simEngineInterfacePtr, sectorSweepFrame);

        SimTime delayUntilAirborne = ZERO_TIME;

        // Modeling hack:
        // Initiator Rxss modeling limitation to ensure first sector beamforming (gain calculation)

        if ((currentSectorSweepFrameCount == 0) &&
            (shouldTransmitReceiveSectorSweep)) {
            delayUntilAirborne = 2*NANO_SECOND;
        }//if//

        SimTime endTransmissionTime;

        const double transmitPowerDbm =
            macLayerPtr->GetAdaptiveTxPowerControllerPtr()->GetCurrentTransmitPowerDbm(
                MacAddress::GetBroadcastAddress());

        macLayerPtr->TransmitDirectionalSectorSweepFrame(
            framePtr, sectorSweepFrame.sectorSweepField.sectorId, transmitPowerDbm, delayUntilAirborne,
            endTransmissionTime);

        currentSectorSweepFrameCount++;

        const SimTime shortInterframeSpaceDuration =
           macLayerPtr->GetShortBeamformingInterframeSpaceDuration();

        const SimTime nextSectorSweepStartTime =
            endTransmissionTime + shortInterframeSpaceDuration;

        if (nextSectorSweepStartTime >=
            (associationBeamformingTrainingSlotStartTime + associationBeamformingTrainingAkaAbftSlotTime)) {

            cerr << "Error in 802.11ad STA Mac: sector sweep is too long for sweep slot." << endl;
            cerr << "Increase the \"dot11ad-association-beamforming-training-aka-abft-slot-time\" "
                 << "parameter." << endl;
            exit(1);
        }//if//


        (*this).nextDirectionalMgScheduleEvent = AssociationBeamformingTrainingSlot;

        simEngineInterfacePtr->ScheduleEvent(
            directionalMgSuperframeScheduleEventPtr,
            nextSectorSweepStartTime,
            directionalMgSuperframeScheduleEventTicket);
    }
    else {
        // Sector Sweep is done. Wait for feedback.

        macLayerPtr->SwitchToQuasiOmniAntennaMode();
        macLayerPtr->StartReceivingFrames();

        const SimTime timeoutTime =
            (associationBeamformingTrainingSlotStartTime + associationBeamformingTrainingAkaAbftSlotTime);

        if (timeoutTime < currentTime) {
            cerr << "Error in 802.11ad STA Mac: sector sweep is too long for sweep slot." << endl;
            cerr << "Increase the \"dot11ad-association-beamforming-training-aka-abft-slot-time\" "
                 << "parameter." << endl;
            exit(1);
        }//if//

        (*this).nextDirectionalMgScheduleEvent = SectorSweepFeedbackFrameTimeout;

        simEngineInterfacePtr->ScheduleEvent(
            directionalMgSuperframeScheduleEventPtr,
            timeoutTime,
            directionalMgSuperframeScheduleEventTicket);

    }//if//

}//TransmitSectorSweepFrame//




void Dot11StaManagementController::StartAssociationBeamformingTrainingPeriod()
{
    macLayerPtr->StopReceivingFrames();

    if (!shouldSendASectorSweep) {
        (*this).nextDirectionalMgScheduleEvent = DataTransferIntervalPeriod;

        simEngineInterfacePtr->ScheduleEvent(
            directionalMgSuperframeScheduleEventPtr,
            dataTransferIntervalStartTime,
            directionalMgSuperframeScheduleEventTicket);
    }
    else {
        (*this).currentSectorSweepFrameCount = 0;

        const unsigned int slotToSendSectorSweep =
            aRandomNumberGenerator.GenerateRandomInt(
                0, (associationBeamformingTrainingAkaAbftNumSlots - 1));

            (*this).nextDirectionalMgScheduleEvent = AssociationBeamformingTrainingSlot;

            const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

            // Delay start of sector sweep slightly so that receiver can set up to receive
            // of the sweep (probably only needed for zero propagation delay mode).

            const SimTime slotStartTime =
                currentTime + (slotToSendSectorSweep * associationBeamformingTrainingAkaAbftSlotTime);

            simEngineInterfacePtr->ScheduleEvent(
                directionalMgSuperframeScheduleEventPtr,
                slotStartTime,
                directionalMgSuperframeScheduleEventTicket);

    }//if//

}//StartAssociationBeamformingTrainingPeriod//




void Dot11StaManagementController::ContinueReceivingReceiveSectorSweep(const bool isFirstSector)
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    if (isFirstSector) {
        currentReceiveSweepSectorId = 0;
        macLayerPtr->StartReceivingFrames();

        if (beamformingEstablished) {
            SectorSweepInfoType& sectorSweepInfo =
                *perLinkSectorSweepInfo[currentApAddress];

            sectorSweepInfo.currentBestRxBeamformingSectorId = QuasiOmniSectorId;
        }//if//

    }
    else {
        currentReceiveSweepSectorId++;
    }//if//

    if (currentReceiveSweepSectorId < macLayerPtr->GetNumberAntennaSectors()) {

        macLayerPtr->SwitchToDirectionalAntennaMode(currentReceiveSweepSectorId);

        const SimTime nextReceiveSectorSweepSectorStartTime =
            (currentTime + macLayerPtr->GetReceiveSweepPerSectorDuration());

        (*this).nextDirectionalMgScheduleEvent = ReceiveSectorSweepStartReceivingNextSector;

        simEngineInterfacePtr->ScheduleEvent(
            directionalMgSuperframeScheduleEventPtr,
            nextReceiveSectorSweepSectorStartTime,
            directionalMgSuperframeScheduleEventTicket);
    }
    else {
        currentReceiveSweepSectorId = QuasiOmniSectorId;
        macLayerPtr->SwitchToQuasiOmniAntennaMode();

        if (beamformingEstablished) {
            SectorSweepInfoType& sectorSweepInfo =
                *perLinkSectorSweepInfo[currentApAddress];

            sectorSweepInfo.currentBestRxBeamformingSectorId =
                sectorSweepInfo.beamformingSectorSelectorPtr->GetBestReceiveSectorId();
        }//if//

        // Continue allocated access period.

        (*this).ContinueAllocatedAccessPeriod();

    }//if//

}//StartReceivingReceiveSectorSweep//



void Dot11StaManagementController::ProcessSectorSweepFeedbackFrame(
    const SectorSweepFeedbackFrameType& feedbackFrame)
{
    if (!receivedADirectionalBeacon) {
        return;
    }//if//

    if (feedbackFrame.transmitterAddress == currentApAddress) {

        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

        SectorSweepInfoType& sectorSweepInfo =
            *perLinkSectorSweepInfo[currentApAddress];

        BeamformingSectorSelector& beamformingSectorSelectorForAp =
            *sectorSweepInfo.beamformingSectorSelectorPtr;

        const unsigned int originalTxSectorId =
            beamformingSectorSelectorForAp.GetBestTransmissionSectorId();

        beamformingSectorSelectorForAp.ReceiveTxSectorFeedback(
            currentTime,
            feedbackFrame.sectorSweepFeedbackField.bestSectorId);

        sectorSweepInfo.currentBestTxBeamformingSectorId =
            beamformingSectorSelectorForAp.GetBestTransmissionSectorId();

        if (beamformingSectorSelectorForAp.GetBestTransmissionSectorId() != originalTxSectorId) {

            (*this).OutputTraceForNewUplinkBeamformingSector();

        }//if//

        (*this).beamformingEstablished = true;

        if ((!isAuthenticated) && (theStaManagementState != WaitingForAuthentication)) {
            (*this).StartAuthentication(currentApAddress);
        }//if//

        //Was cheating (*this).theStaManagementState = Associated;

        //Future// (*this).nextDirectionalMgScheduleEvent = TransmitSectorSweepFeedbackAckEvent;
        //Future//
        //Future// simEngineInterfacePtr->ScheduleEvent(
        //Future//     directionalMgSuperframeScheduleEventPtr,
        //Future//     (currentTime + macLayerPtr->GetShortBeamformingInterframeSpaceDuration()));

        macLayerPtr->StopReceivingFrames();

        (*this).nextDirectionalMgScheduleEvent = DataTransferIntervalPeriod;

        simEngineInterfacePtr->RescheduleEvent(
            directionalMgSuperframeScheduleEventTicket,
            dataTransferIntervalStartTime);

    }//if//

}//ProcessSectorSweepFeedbackFrame//


void Dot11StaManagementController::ProcessSectorSweepFeedbackTimeout()
{
    SectorSweepInfoType& sectorSweepInfo =
        *perLinkSectorSweepInfo[currentApAddress];

    sectorSweepInfo.currentBestTxBeamformingSectorId = QuasiOmniSectorId;

    macLayerPtr->StopReceivingFrames();
    (*this).nextDirectionalMgScheduleEvent = DataTransferIntervalPeriod;

    simEngineInterfacePtr->ScheduleEvent(
        directionalMgSuperframeScheduleEventPtr,
        dataTransferIntervalStartTime,
        directionalMgSuperframeScheduleEventTicket);

}//ProcessSectorSweepFeedbackTimeout//



void Dot11StaManagementController::ProcessSectorSweepFrame(const SectorSweepFrameType& sectorSweepFrame)
{
    if (!receivedADirectionalBeacon) {
        return;
    }//if//

    if (sectorSweepFrame.transmitterAddress == currentApAddress) {

        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

        BeamformingSectorSelector& beamformingSectorSelectorForAp =
            *perLinkSectorSweepInfo[currentApAddress]->beamformingSectorSelectorPtr;

        assert((currentReceiveSweepSectorId == QuasiOmniSectorId) ||
               (currentReceiveSweepSectorId < macLayerPtr->GetNumberAntennaSectors()));

        beamformingSectorSelectorForAp.ReceiveSectorMetrics(
            currentTime,
            sectorSweepFrame.sectorSweepField.sectorId,
            currentReceiveSweepSectorId,
            macLayerPtr->GetRssiOfLastFrameDbm(),
            macLayerPtr->GetSinrOfLastFrameDb());

        OutputTraceForDmgFrameReceiveOnSta(sectorSweepFrame.sectorSweepField.sectorId);

        beamformingSectorSelectorForAp.ReceiveTxSectorFeedback(
            currentTime,
            sectorSweepFrame.sectorSweepFeedbackField.bestSectorId);

    }//if//

}//ProcessSectorSweepFeedbackFrame//



void Dot11StaManagementController::TransmitSectorSweepFeedbackAckFrame()
{
    assert(false && "DISABLED Future");

    SectorSweepAckFrameType sectorSweepAckFrame;

    sectorSweepAckFrame.header.theFrameControlField.frameTypeAndSubtype =
        SECTOR_SWEEP_ACK_FRAME_TYPE_CODE;
    sectorSweepAckFrame.header.theFrameControlField.isRetry = 0;
    sectorSweepAckFrame.header.duration = 0;
    sectorSweepAckFrame.header.receiverAddress = currentApAddress;
    sectorSweepAckFrame.transmitterAddress = macLayerPtr->GetMacAddress();

    unique_ptr<Packet> framePtr =
        Packet::CreatePacket(*simEngineInterfacePtr, sectorSweepAckFrame);

    //SimTime endTransmissionTime;

    //macLayerPtr->TransmitDirectionalSectorSweepFrame(
    //     framePtr, azimuthDegForSectors.at(currentSectorSweepFrameCount),
    //    endTransmissionTime);

}//TransmitSectorSweepFeedbackAck//



void Dot11StaManagementController::StartDataTransferIntervalPeriod()
{
    (*this).currentDataTransferIntervalListIndex = GetAllocatedAccessPeriodIndex();

    if (currentDataTransferIntervalListIndex >= dmgDataTransferIntervalList.size()) {
        // No allocations for node.
        macLayerPtr->StartReceivingFrames();

        return;
    }//if//

    const DataTransferIntervalListElementType& intervalInfo =
        dmgDataTransferIntervalList[currentDataTransferIntervalListIndex];

    if (intervalInfo.relativeStartTime == ZERO_TIME) {

        (*this).StartAllocatedAccessPeriod();
    }
    else {
        (*this).nextDirectionalMgScheduleEvent = AllocatedAccessPeriod;

        simEngineInterfacePtr->ScheduleEvent(
            directionalMgSuperframeScheduleEventPtr,
            (dataTransferIntervalStartTime + intervalInfo.relativeStartTime),
            directionalMgSuperframeScheduleEventTicket);
    }//if//

}//StartDataTransferIntervalPeriod//



void Dot11StaManagementController::ScheduleEventForNextAllocatedAccessPeriodOrSuperframe()
{
    (*this).currentDataTransferIntervalListIndex =
        GetAllocatedAccessPeriodIndex(currentDataTransferIntervalListIndex+1);

    if (currentDataTransferIntervalListIndex < dmgDataTransferIntervalList.size()) {
        const DataTransferIntervalListElementType& intervalInfo =
            dmgDataTransferIntervalList[currentDataTransferIntervalListIndex];

        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

        if ((dataTransferIntervalStartTime + intervalInfo.relativeStartTime) < currentTime) {
            cerr << "Error: DTI access period scheduling conflict at node " << theNodeId
                 << " at relative start time " << intervalInfo.relativeStartTime << "ns" << endl;
            exit(1);
        }//if//

        (*this).nextDirectionalMgScheduleEvent = AllocatedAccessPeriod;

        simEngineInterfacePtr->ScheduleEvent(
            directionalMgSuperframeScheduleEventPtr,
            (dataTransferIntervalStartTime + intervalInfo.relativeStartTime),
            directionalMgSuperframeScheduleEventTicket);
    }
    else {
        // Set event for next superframe.

        (*this).nextDirectionalMgScheduleEvent = BeaconTransmissionIntervalPeriod;

        simEngineInterfacePtr->ScheduleEvent(
            directionalMgSuperframeScheduleEventPtr,
            dataTransferIntervalEndTime,
            directionalMgSuperframeScheduleEventTicket);
    }//if//

}//ScheduleEventForNextAllocatedAccessPeriodOrSuperframe//



void Dot11StaManagementController::ContinueAllocatedAccessPeriod()
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    const DataTransferIntervalListElementType& intervalInfo =
        dmgDataTransferIntervalList.at(currentDataTransferIntervalListIndex);

    if (beamformingEstablished) {
        const SectorSweepInfoType& sectorSweepInfo =
            *perLinkSectorSweepInfo[currentApAddress];

        const BeamformingSectorSelector& beamformingSectorSelectorForAp =
            *sectorSweepInfo.beamformingSectorSelectorPtr;

        // Setup receive pattern.

        if ((intervalInfo.isAContentionBasedAccessPeriodAkaCbap) ||
            (intervalInfo.sourceAssociationId == AccessPointAssociationId)) {

            macLayerPtr->SwitchToDirectionalAntennaMode(
                sectorSweepInfo.currentBestRxBeamformingSectorId);

            macLayerPtr->StartReceivingFrames();
        }
        else if ((intervalInfo.sourceAssociationId != InvalidAssociationId) &&
                 (intervalInfo.sourceAssociationId != theAssociationId) &&
                 (intervalInfo.destinationAssociationId != InvalidAssociationId) &&
                 (intervalInfo.destinationAssociationId == theAssociationId)) {

           const SectorSweepInfoType& sweepInfo =
               *perLinkSectorSweepInfo[macLayerPtr->MakeMacAddressFromNodeId(intervalInfo.sourceNodeId)];

           assert((sweepInfo.forcedRxBeamformingSectorId != InvalidSectorId) &&
                  "Limitation: STA to STA links must have defined forced sector IDs");
           macLayerPtr->SwitchToDirectionalAntennaMode(sweepInfo.forcedRxBeamformingSectorId);
           macLayerPtr->StartReceivingFrames();
        }
        else {
           macLayerPtr->StopReceivingFrames();
        }//if//
    }
    else {
        macLayerPtr->StopReceivingFrames();
    }//if//

    if ((intervalInfo.isAContentionBasedAccessPeriodAkaCbap) ||
        ((intervalInfo.sourceAssociationId != InvalidAssociationId) &&
         (intervalInfo.sourceAssociationId == theAssociationId))) {

        const SimTime mediumInterframeSpaceDuration =
            macLayerPtr->GetMediumBeamformingInterframeSpaceDuration();

        (*this).nextDirectionalMgScheduleEvent = AllocatedAccessPeriodAfterGap;

        simEngineInterfacePtr->ScheduleEvent(
            directionalMgSuperframeScheduleEventPtr,
            (currentTime + mediumInterframeSpaceDuration),
            directionalMgSuperframeScheduleEventTicket);
    }
    else {
        (*this).ScheduleEventForNextAllocatedAccessPeriodOrSuperframe();
    }//if//

}//ContinueAllocatedAccessPeriod//




void Dot11StaManagementController::StartAllocatedAccessPeriod()
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    const DataTransferIntervalListElementType& intervalInfo =
        dmgDataTransferIntervalList.at(currentDataTransferIntervalListIndex);

    (*this).allocatedAccessPeriodEndTime = currentTime + intervalInfo.duration;

    if ((intervalInfo.mustDoReceiveSectorSweep) &&
        (!intervalInfo.isAContentionBasedAccessPeriodAkaCbap) &&
        (intervalInfo.destinationAssociationId != InvalidAssociationId) &&
        (intervalInfo.destinationAssociationId == theAssociationId)) {

        ContinueReceivingReceiveSectorSweep(true);
        return;
    }//if//

    (*this).ContinueAllocatedAccessPeriod();

}//StartAllocatedAccessPeriod//



void Dot11StaManagementController::StartSendingInAllocatedAccessPeriod()
{
    const DataTransferIntervalListElementType& intervalInfo =
        dmgDataTransferIntervalList.at(currentDataTransferIntervalListIndex);

    if (beamformingEstablished) {
        if (intervalInfo.isAContentionBasedAccessPeriodAkaCbap) {
            macLayerPtr->StartContentionPeriod(allocatedAccessPeriodEndTime - EPSILON_TIME);
        }
        else {
            macLayerPtr->StartNonContentionPeriod(allocatedAccessPeriodEndTime - EPSILON_TIME);
        }//if//
    }//if//

    (*this).ScheduleEventForNextAllocatedAccessPeriodOrSuperframe();

}//StartSendingInAllocatedAccessPeriod//



void Dot11StaManagementController::OutputTraceForNewDownlinkBeamformingSector()
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        const BeamformingSectorSelector& beamformingSectorSelectorForAp =
            *perLinkSectorSweepInfo[currentApAddress]->beamformingSectorSelectorPtr;

        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacDownlinkBeamformingTraceRecord traceData;

            traceData.sourceNodeId = 0;
            traceData.sectorId = beamformingSectorSelectorForAp.GetBestTransmissionSectorIdToThisNode();
            traceData.transmissionSectorMetric = beamformingSectorSelectorForAp.GetBestTransmissionSectorMetric();
            traceData.sourceIsStaOrNot = false;

            assert(sizeof(traceData) == MAC_DOWNLINK_BEAMFORMING_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(Dot11Mac::modelName, theInterfaceId, "NewDlBeamSector", traceData);
        }
        else {

            ostringstream msgStream;
            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "SectorId= "
                      << beamformingSectorSelectorForAp.GetBestTransmissionSectorIdToThisNode();
            msgStream << " LinkMetric= "
                      << beamformingSectorSelectorForAp.GetBestTransmissionSectorMetric();

            simEngineInterfacePtr->OutputTrace(
                Dot11Mac::modelName, theInterfaceId, "NewDlBeamSector", msgStream.str());
        }//if//
    }//if//

}//OutputTraceForNewDownlinkBeamformingSector//



void Dot11StaManagementController::OutputTraceForNewUplinkBeamformingSector()
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        const BeamformingSectorSelector& beamformingSectorSelectorForAp =
            *perLinkSectorSweepInfo[currentApAddress]->beamformingSectorSelectorPtr;

        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacUplinkBeamformingTraceRecord traceData;

            traceData.sourceNodeId = 0;
            traceData.sectorId = beamformingSectorSelectorForAp.GetBestTransmissionSectorId();
            traceData.sourceIsStaOrNot = false;

            assert(sizeof(traceData) == MAC_UPLINK_BEAMFORMING_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(Dot11Mac::modelName, theInterfaceId, "NewUlBeamSector", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream << "SectorId= " << beamformingSectorSelectorForAp.GetBestTransmissionSectorId();
            simEngineInterfacePtr->OutputTrace(
                Dot11Mac::modelName, theInterfaceId, "NewUlBeamSector", msgStream.str());
        }//if//
    }//if//

}//OutputTraceForNewUplinkBeamformingSector//



void Dot11StaManagementController::OutputTraceForDmgFrameReceiveOnSta(const unsigned short int transmissionSectorId)
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (! simEngineInterfacePtr->BinaryOutputIsOn()) {

            ostringstream msgStream;

            msgStream << "TxSectorId= " << transmissionSectorId
                << " RxSectorId= " << currentReceiveSweepSectorId
                << " RssiDb= " << macLayerPtr->GetRssiOfLastFrameDbm()
                << " SinrDb= " << macLayerPtr->GetSinrOfLastFrameDb();
            simEngineInterfacePtr->OutputTrace(
                Dot11Mac::modelName, theInterfaceId, "ReceiveDmgFrameOnSta", msgStream.str());
        }//if//
    }//if//

}//OutputTraceForDmgFrameReceive//


}//namespace//





