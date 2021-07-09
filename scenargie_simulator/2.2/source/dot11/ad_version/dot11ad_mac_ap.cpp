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
#include "dot11ad_ratecontrol.h"

namespace Dot11ad {

using ScenSim::ConvertToUChar;
using ScenSim::InvalidNodeId;
using ScenSim::ConvertAStringSequenceOfNumericPairsIntoAMap;


Dot11ApManagementController::Dot11ApManagementController(
    const shared_ptr<Dot11Mac>& initMacLayerPtr,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& initNodeId,
    const InterfaceOrInstanceId& initInterfaceId,
    const RandomNumberGeneratorSeed& interfaceSeed)
    :
    macLayerPtr(initMacLayerPtr),
    simEngineInterfacePtr(initSimulationEngineInterfacePtr),
    currentBeaconSuperframeStartTime(INFINITE_TIME),
    dataTransferIntervalStartTime(INFINITE_TIME),
    dataTransferIntervalEndTime(INFINITE_TIME),
    allocatedAccessPeriodEndTime(ZERO_TIME),
    currentDmgBeaconCount(0),
    currentDataTransferIntervalListIndex(0),
    currentSectorSweepFrameCount(0),
    numberSectorSweepFramesToSend(0),
    theNodeId(initNodeId),
    theInterfaceId(initInterfaceId),
    ssid(""),
    authProcessingDelay(ZERO_TIME),
    currentAssociationId(0),
    thereWasASectorSweepCollision(false),
    currentReceiveSweepSectorId(QuasiOmniSectorId),
    sectorSelectorFactory(theParameterDatabaseReader, initNodeId, initInterfaceId),
    aRandomNumberGenerator(HashInputsToMakeSeed(interfaceSeed, SEED_HASH))
{
    //read ssid
    if (theParameterDatabaseReader.ParameterExists("dot11-access-point-ssid", theNodeId, theInterfaceId)) {
        ssid = theParameterDatabaseReader.ReadString("dot11-access-point-ssid", theNodeId, theInterfaceId);

        if (ssid.length() > SSID_LENGTH) {
            cerr << "Error: SSID length must be " << SSID_LENGTH << " or under: " << ssid << endl;
            exit(1);
        }//if//
    }//if//

    //read authentication processing delay
    if (theParameterDatabaseReader.ParameterExists("dot11-access-point-auth-processing-delay", theNodeId, theInterfaceId)) {

        authProcessingDelay =
            theParameterDatabaseReader.ReadTime("dot11-access-point-auth-processing-delay", theNodeId, theInterfaceId);

    }//if//


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

    (*this).abftReceiveSectorSweepSlotTime =
         macLayerPtr->CalcAssociationBeamformingTrainingSlotTime(
             macLayerPtr->GetNumberAntennaSectors());

    //Future if (theParameterDatabaseReader.ParameterExists(
    //Future     (adParamNamePrefix + "use-sinr-for-beamforming-link-metric"), theNodeId, theInterfaceId)) {
    //Future
    //Future     assert(false && "Disabled");
    //Future
    //Future     //useSinrForBeamformingLinkMetric =
    //Future     //    theParameterDatabaseReader.ReadBool(
    //Future     //    (adParamNamePrefix + "use-sinr-for-beamforming-link-metric"), theNodeId, theInterfaceId);
    //Future
    //Future }//if//


}//Dot11ApManagementController//


void Dot11ApManagementController::CompleteInitialization(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    (*this).schedulerPtr.reset(
        new StaticScheduleScheduler(
            theParameterDatabaseReader,
            theNodeId,
            theInterfaceId,
            macLayerPtr));

    const SimTime beaconStartJitter = static_cast<SimTime>(
        (schedulerPtr->GetBeaconSuperframeIntervalDuration() *
         aRandomNumberGenerator.GenerateRandomDouble()));

    (*this).nextDirectionalMgScheduleEvent = BeaconTransmissionIntervalPeriod;

    directionalMgSuperframeScheduleEventPtr.reset(new DirectionalMgSuperframeScheduleEvent(this));

    simEngineInterfacePtr->ScheduleEvent(
        directionalMgSuperframeScheduleEventPtr,
        (currentTime + beaconStartJitter),
        directionalMgSuperframeScheduleEventTicket);

}//CompleteInitialization//



void Dot11ApManagementController::ProcessAssociationRequestFrame(const Packet& aFrame)
{
    const AssociationRequestFrame& associationFrame =
        aFrame.GetAndReinterpretPayloadData<AssociationRequestFrame>();

    assert(associationFrame.managementHeader.header.theFrameControlField.frameTypeAndSubtype ==
           ASSOCIATION_REQUEST_FRAME_TYPE_CODE);

    const MacAddress& transmitterAddress = associationFrame.managementHeader.transmitterAddress;

    if (!IsAnAssociatedStaAddress(transmitterAddress)) {
        (*this).AddNewAssociatedStaRecord(
            transmitterAddress,
            associationFrame.theHtCapabilitiesFrameElement.numChannelsBandwidth,
            associationFrame.theHtCapabilitiesFrameElement.highThroughputModeIsEnabled);
    }//if//

    //11ad// if (associationFrame.theHtCapabilitiesFrameElement.highThroughputModeIsEnabled) {
   //11ad//      if (associationFrame.theHtCapabilitiesFrameElement.aggregateMpdusAreEnabled != 0) {
    macLayerPtr->SetMpduFrameAggregationIsEnabledFor(transmitterAddress);
    //11ad//     }//if//
    //Removed Feature: if (associationFrame.theHtCapabilitiesFrameElement.aggregateMsdusAreEnabled != 0) {
    //Removed Feature:     macLayerPtr->SetMsduFrameAggregationIsEnabledFor(transmitterAddress);
        //Removed Feature: }//if//
    //11ad// }
    //11ad// else {
    //Removed Feature: if ((associationFrame.theHtCapabilitiesFrameElement.aggregateMpdusAreEnabled != 0) ||
    //Removed Feature:    (associationFrame.theHtCapabilitiesFrameElement.aggregateMsdusAreEnabled != 0)) {
    //11ad//     if (associationFrame.theHtCapabilitiesFrameElement.aggregateMpdusAreEnabled != 0) {

    //11ad//         cerr << "Error A-MPDU aggregation is enabled but \"High Throughput\" (HT) is not." << endl;
    //11ad//         exit(1);

    //11ad//     }//if//
    //11ad// }//if//

    macLayerPtr->ResetOutgoingLinksTo(transmitterAddress);
    macLayerPtr->SendNewLinkToANodeNotificationToNetworkLayer(transmitterAddress);

    macLayerPtr->SendAssociationResponse(
        transmitterAddress,
        associatedStaInformation[transmitterAddress]->theAssociationId);

}//ProcessAssociationRequestFrame//



void Dot11ApManagementController::ProcessReassociationRequestFrame(const Packet& aFrame)
{
    const ReassociationRequestFrame& reassociationFrame =
        aFrame.GetAndReinterpretPayloadData<ReassociationRequestFrame>();

    assert(reassociationFrame.managementHeader.header.theFrameControlField.frameTypeAndSubtype ==
           REASSOCIATION_REQUEST_FRAME_TYPE_CODE);

    const MacAddress& transmitterAddress =
        reassociationFrame.managementHeader.transmitterAddress;


    if (!IsAnAssociatedStaAddress(transmitterAddress)) {
        (*this).AddNewAssociatedStaRecord(
            transmitterAddress,
            reassociationFrame.theHtCapabilitiesFrameElement.numChannelsBandwidth,
            reassociationFrame.theHtCapabilitiesFrameElement.highThroughputModeIsEnabled);
    }//if//

    //11ad// if (reassociationFrame.theHtCapabilitiesFrameElement.highThroughputModeIsEnabled) {
    //11ad//     if (reassociationFrame.theHtCapabilitiesFrameElement.aggregateMpdusAreEnabled != 0) {
    macLayerPtr->SetMpduFrameAggregationIsEnabledFor(transmitterAddress);
    //11ad//     }//if//
    //Removed Feature: if (reassociationFrame.theHtCapabilitiesFrameElement.aggregateMsdusAreEnabled != 0) {
    //Removed Feature:        macLayerPtr->SetMsduFrameAggregationIsEnabledFor(transmitterAddress);
    //Removed Feature: }//if//
    //11ad// }//if//

    macLayerPtr->ResetOutgoingLinksTo(transmitterAddress);

    SendReassociationNotification(transmitterAddress, reassociationFrame.currentApAddress);

    macLayerPtr->SendNewLinkToANodeNotificationToNetworkLayer(transmitterAddress);
    macLayerPtr->SendReassociationResponse(
        transmitterAddress,
        associatedStaInformation[transmitterAddress]->theAssociationId);

}//ProcessReassociationRequestFrame//


bool Dot11ApManagementController::IsBeamformingSectorSelectorInitialized(const MacAddress& macAddress) const
{
    typedef map<MacAddress, StaSectorSweepInfoType>::const_iterator IterType;
    IterType staIterator = sectorSweepInfoForStation.find(macAddress);

    return (staIterator != sectorSweepInfoForStation.end());

}//IsBeamformingSectorSelectorInitialized//


void Dot11ApManagementController::InitBeamformingSectorSelectorFor(const MacAddress& macAddress)
 {
    typedef map<MacAddress, unique_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;
    IterType staIterator = associatedStaInformation.find(macAddress);

    assert(staIterator != associatedStaInformation.end());

    StaSectorSweepInfoType& sectorSweepInfo = sectorSweepInfoForStation[macAddress];

    sectorSweepInfo.beamformingSectorSelectorPtr = sectorSelectorFactory.CreateSectorSelector();

    const NodeId otherNodeId =
        CalcNodeId(macAddress.ConvertToGenericMacAddress());

    if (forcedTxBeamformingSectorIdMap.find(otherNodeId) != forcedTxBeamformingSectorIdMap.end()) {

       sectorSweepInfo.forcedTxBeamformingSectorId = forcedTxBeamformingSectorIdMap[otherNodeId];

       // Tx and Rx agree by default.

       sectorSweepInfo.forcedRxBeamformingSectorId = sectorSweepInfo.forcedTxBeamformingSectorId;

    }//if//

    if (forcedRxBeamformingSectorIdMap.find(otherNodeId) != forcedRxBeamformingSectorIdMap.end()) {

       sectorSweepInfo.forcedRxBeamformingSectorId = forcedRxBeamformingSectorIdMap[otherNodeId];

    }//if//

    if (forcedLinkTxPowerDbmMap.find(otherNodeId) != forcedLinkTxPowerDbmMap.end()) {
        macLayerPtr->GetAdaptiveTxPowerControllerPtr()->SetForcedLinkTxPower(
            macAddress, forcedLinkTxPowerDbmMap[otherNodeId]);
    }//if//

}//InitBeamformingSectorSelectorFor//



void Dot11ApManagementController::AddNewAssociatedStaRecord(
    const MacAddress& staAddress,
    const unsigned int stationBandwidthNumChannels,
    const bool highThroughputModeIsEnabled)
{
    unique_ptr<AssociatedStaInformationEntry>
        newStaInfoEntryPtr(new AssociatedStaInformationEntry());

    if (currentAssociationId < MaxAssociationId) {
        (*this).currentAssociationId++;
        newStaInfoEntryPtr->theAssociationId = currentAssociationId;
    }
    else {
        bool wasFound = false;
        for (AssociationId i = 1; (i <= MaxAssociationId); i++) {
            if (!associationIdIsBeingUsed[i]) {
                wasFound = true;
                newStaInfoEntryPtr->theAssociationId = i;
                break;
            }//if//
        }//for//
        assert((wasFound) && "Too Many STAs trying to associate with AP");
    }//if//

    (*this).associationIdIsBeingUsed.set(newStaInfoEntryPtr->theAssociationId);

    associatedStaInformation.insert(
        make_pair(staAddress, move(newStaInfoEntryPtr)));

    macLayerPtr->GetAdaptiveRateControllerPtr()->AddNewStation(
        staAddress,
        stationBandwidthNumChannels,
        highThroughputModeIsEnabled);

}//AddNewAssociatedStaRecord//


void Dot11ApManagementController::ProcessSectorSweepFrame(const SectorSweepFrameType& sectorSweepFrame)
{
    typedef map<MacAddress, unique_ptr<AssociatedStaInformationEntry> >::iterator IterType;

    if (thereWasASectorSweepCollision) {
        return;
    }//if//

    (*this).thereWasASectorSweepCollision =
        ((lastSectorSweepFrameSourceAddress != MacAddress::invalidMacAddress) &&
          (lastSectorSweepFrameSourceAddress != sectorSweepFrame.transmitterAddress));

    if (thereWasASectorSweepCollision) {
        StaSectorSweepInfoType& sweepInfo =
            sectorSweepInfoForStation[lastSectorSweepFrameSourceAddress];

        sweepInfo.beamformingSectorSelectorPtr->ReceiveSectorSweepCollisionNotification();
        sweepInfo.receivedASectorSweepFrame = false;
        lastSectorSweepFrameSourceAddress = MacAddress::invalidMacAddress;
        return;
    }//if//


    if (sectorSweepFrame.header.receiverAddress == macLayerPtr->GetMacAddress()) {
        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
        const MacAddress& transmitterAddress = sectorSweepFrame.transmitterAddress;

        if (!IsAnAssociatedStaAddress(transmitterAddress)) {
            (*this).AddNewAssociatedStaRecord(transmitterAddress, 1/*stationBandwidthNumChannels*/, false/*highThroughputModeIsEnabled*/);
        }//if//

        if (!IsBeamformingSectorSelectorInitialized(transmitterAddress)) {
            (*this).InitBeamformingSectorSelectorFor(transmitterAddress);
        }//if//

        StaSectorSweepInfoType& sectorSweepInfo =
            sectorSweepInfoForStation[transmitterAddress];

        if (schedulerPtr->GetShouldDoReceiveSectorSweepInAbft()) {

            lastSectorSweepFrameSourceAddress = sectorSweepFrame.transmitterAddress;

        }
        else {
            if (!sectorSweepInfo.receivedASectorSweepFrame) {
                sectorSweepInfo.receivedASectorSweepFrame = true;
                // Cheating

                sectorSweepInfo.numberStaBeamformingSectors =
                    (sectorSweepFrame.sectorSweepField.sectorId + sectorSweepFrame.sectorSweepField.cdown + 1);

                // For trace output only:
                sectorSweepInfo.lastBestReportedDownlinkTxSectorId =
                    sectorSweepInfo.beamformingSectorSelectorPtr->GetBestTransmissionSectorId();
                sectorSweepInfo.lastBestMeasuredUplinkTxSectorId =
                    sectorSweepInfo.beamformingSectorSelectorPtr->GetBestTransmissionSectorIdToThisNode();

                sectorSweepInfo.beamformingSectorSelectorPtr->StartNewBeaconInterval();

                lastSectorSweepFrameSourceAddress = sectorSweepFrame.transmitterAddress;

                const SimTime frameDuration = macLayerPtr->GetDurationOfLastFrame();

                const SimTime mediumBeamformingIfsDuration =
                    macLayerPtr->GetMediumBeamformingInterframeSpaceDuration();

                const SimTime sendFeedbackTime =
                    currentTime +
                    (sectorSweepFrame.sectorSweepField.cdown *
                     (frameDuration + macLayerPtr->GetShortBeamformingInterframeSpaceDuration())) +
                    mediumBeamformingIfsDuration;

                (*this).nextDirectionalMgScheduleEvent =
                    TransmitAssociationBeamformingTrainingFeedbackFrameEvent;

                simEngineInterfacePtr->RescheduleEvent(
                    directionalMgSuperframeScheduleEventTicket,
                    sendFeedbackTime);
            }//if//
        }//if//

        sectorSweepInfo.beamformingSectorSelectorPtr->ReceiveSectorMetrics(
            currentTime,
            sectorSweepFrame.sectorSweepField.sectorId,
            currentReceiveSweepSectorId,
            macLayerPtr->GetRssiOfLastFrameDbm(),
            macLayerPtr->GetSinrOfLastFrameDb());

        OutputTraceForDmgFrameReceiveOnAp(sectorSweepFrame);

        sectorSweepInfo.beamformingSectorSelectorPtr->ReceiveTxSectorFeedback(
            currentTime,
            sectorSweepFrame.sectorSweepFeedbackField.bestSectorId);

        if (sectorSweepInfo.forcedTxBeamformingSectorId == InvalidSectorId) {
            if (sectorSweepInfo.beamformingSectorSelectorPtr->GetBestTransmissionSectorId() !=
                sectorSweepFrame.sectorSweepFeedbackField.bestSectorId) {

                (*this).OutputTraceForNewDownlinkBeamformingSector(
                    sectorSweepFrame.transmitterAddress,
                    sectorSweepFrame.sectorSweepFeedbackField.bestSectorId);

            }//if//
        }//if//
    }//if//

}//ProcessSectorSweepFrame//


void Dot11ApManagementController::ProcessManagementFrame(const Packet& managementFrame)
{
    const CommonFrameHeader& commonHeader =
        managementFrame.GetAndReinterpretPayloadData<CommonFrameHeader>();

    switch(commonHeader.theFrameControlField.frameTypeAndSubtype) {
    case ASSOCIATION_REQUEST_FRAME_TYPE_CODE: {

        (*this).ProcessAssociationRequestFrame(managementFrame);

        break;
    }
    case REASSOCIATION_REQUEST_FRAME_TYPE_CODE: {

        (*this).ProcessReassociationRequestFrame(managementFrame);

        break;
    }
    case DISASSOCIATION_FRAME_TYPE_CODE: {
        const ManagementFrameHeader& header =
            managementFrame.GetAndReinterpretPayloadData<ManagementFrameHeader>();

        if (IsAnAssociatedStaAddress(header.transmitterAddress)) {
            (*this).associationIdIsBeingUsed.reset(
                associatedStaInformation[header.transmitterAddress]->theAssociationId);
            (*this).associatedStaInformation.erase(header.transmitterAddress);
        }
        break;
    }
    case AUTHENTICATION_FRAME_TYPE_CODE: {

        const ManagementFrameHeader& header =
            managementFrame.GetAndReinterpretPayloadData<ManagementFrameHeader>();
        (*this).ProcessAuthenticationFrame(header.transmitterAddress);

        break;
    }
    case SECTOR_SWEEP_FRAME_TYPE_CODE: {

        const SectorSweepFrameType& sweepFrame =
            managementFrame.GetAndReinterpretPayloadData<SectorSweepFrameType>();

        (*this).ProcessSectorSweepFrame(sweepFrame);
        break;
    }
    case ASSOCIATION_RESPONSE_FRAME_TYPE_CODE:
    case REASSOCIATION_RESPONSE_FRAME_TYPE_CODE:
    case BEACON_FRAME_TYPE_CODE:
    case DMG_BEACON_FRAME_TYPE_CODE:
    case SECTOR_SWEEP_FEEDBACK_FRAME_TYPE_CODE:

        // Ignore management frames from other access points.
        break;

    default:
        // Should not receive other management frame types
        assert(false); abort();
        break;
    }//switch//

}//ProcessManagementFrame//




void Dot11ApManagementController::ReceiveFramePowerManagementBit(
    const MacAddress& sourceAddress,
    const bool framePowerManagementBitIsOn)
{
    typedef map<MacAddress, unique_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

    IterType staIterator = associatedStaInformation.find(sourceAddress);
    if (staIterator == associatedStaInformation.end()) {
        // Unknown client station, ignore.
        return;
    }//if//

    AssociatedStaInformationEntry& staInfo = *(*staIterator).second;

    if (staInfo.isInPowersaveMode == framePowerManagementBitIsOn) {
        // No change, do nothing.
        return;
    }//if//

    staInfo.isInPowersaveMode = framePowerManagementBitIsOn;

    if (!staInfo.isInPowersaveMode) {
        while (!staInfo.powerSavePacketBuffer.empty()) {
            PowerSavePacketBufferElem& bufferElem = staInfo.powerSavePacketBuffer.back();

            if (bufferElem.destinationNetworkAddress == NetworkAddress::invalidAddress) {
                macLayerPtr->RequeueManagementFrame(
                    bufferElem.packetPtr,
                    bufferElem.retryTxCount);
            }
            else {
                macLayerPtr->RequeueBufferedPacket(
                    bufferElem.packetPtr,
                    bufferElem.destinationNetworkAddress,
                    bufferElem.priority,
                    bufferElem.etherType,
                    bufferElem.timestamp,
                    bufferElem.retryTxCount);
            }//if//

            staInfo.powerSavePacketBuffer.pop_back();
        }//while//
    }//if//

}//ReceiveFramePowerManagementBit//


void Dot11ApManagementController::SendAuthentication(const MacAddress& transmitterAddress)
{
    macLayerPtr->SendAuthentication(transmitterAddress);
}


void Dot11ApManagementController::SendReassociationNotification(
    const MacAddress& staAddress,
    const MacAddress& apAddress)
{
    // TBD: notify previous AP to disassociate

}//SendReassociationNotification//


void Dot11ApManagementController::SendBeaconFrame()
{
    assert(false && "Not called in 802.11ad.");

    typedef map<MacAddress, unique_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

    TrafficIndicationBitMap aTrafficIndicationBitMap;

    for(IterType iter = associatedStaInformation.begin(); iter != associatedStaInformation.end(); ++iter) {
        const AssociatedStaInformationEntry& staInfo = *iter->second;
        if (!staInfo.powerSavePacketBuffer.empty()) {
            aTrafficIndicationBitMap.AddBit(staInfo.theAssociationId);
        }//if//
    }//for//

    BeaconFrame beaconHeader(ssid);
    beaconHeader.managementHeader.header.theFrameControlField.frameTypeAndSubtype = BEACON_FRAME_TYPE_CODE;
    beaconHeader.managementHeader.header.theFrameControlField.isRetry = 0;
    beaconHeader.managementHeader.header.duration = 0;
    beaconHeader.managementHeader.header.receiverAddress = MacAddress::GetBroadcastAddress();
    beaconHeader.managementHeader.transmitterAddress = macLayerPtr->GetMacAddress();

    beaconHeader.htOperationFrameElement.highThroughputModeIsEnabled = 0;
    if (macLayerPtr->IsAHighThroughputStation()) {
        beaconHeader.htOperationFrameElement.highThroughputModeIsEnabled = 1;
    }//if

    const vector<unsigned int>& channelList = macLayerPtr->GetCurrentBondedChannelList();
    for(unsigned int i = 0; (i < channelList.size()); i++) {
        beaconHeader.htOperationFrameElement.bondedChannelList[i] = ConvertToUChar(channelList[i]);
    }//for//


    if (!aTrafficIndicationBitMap.IsEmpty()) {
        unique_ptr<Packet> framePtr =
            Packet::CreatePacketWithExtraHeaderSpace(
                *simEngineInterfacePtr,
                aTrafficIndicationBitMap.GetBitMapByteVector(),
                (sizeof(TrafficIndicationMapElementHeader) + sizeof(BeaconFrame)));

        TrafficIndicationMapElementHeader timHeader;
        timHeader.bitMapByteOffset =  aTrafficIndicationBitMap.GetStartByteOffset();
        framePtr->AddPlainStructHeader(timHeader);

        framePtr->AddPlainStructHeader(beaconHeader);
        macLayerPtr->SendManagementFrame(framePtr);
    }
    else {
        unique_ptr<Packet> framePtr = Packet::CreatePacket(*simEngineInterfacePtr, beaconHeader);
        macLayerPtr->SendManagementFrame(framePtr);
    }//if//

}//SendBeaconFrame//


void Dot11ApManagementController::SendDirectionalBeaconFrame(
    const SimTime& delayUntilAirborne,
    SimTime& transmissionEndTime)
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    // Just add extra space (does not need to be exact).

    const unsigned int extraAllocatedBytes =
        (static_cast<unsigned int>(sizeof(DirectionalBeaconFrameType)) +
         CalcExtendedScheduleElementsSizeBytes(
             schedulerPtr->GetNumberDataTransferIntervalPeriods()));

    unique_ptr<Packet> framePtr = Packet::CreatePacket(*simEngineInterfacePtr, extraAllocatedBytes);

    DirectionalBeaconFrameType aBeaconFrame;

    aBeaconFrame.theFrameControlField.frameTypeAndSubtype = DMG_BEACON_FRAME_TYPE_CODE;
    aBeaconFrame.theFrameControlField.isRetry = 0;
    aBeaconFrame.duration =
        ConvertTimeToDurationFieldValue(
            (currentBeaconSuperframeStartTime +
             schedulerPtr->GetDmgBeaconTransmissionIntervalAkaBtiDuration()) -
            (currentTime + delayUntilAirborne));

    aBeaconFrame.bssid = macLayerPtr->GetMacAddress();

    aBeaconFrame.sectorSweepField.direction = 0;
    aBeaconFrame.sectorSweepField.sectorId = currentDmgBeaconCount;
    aBeaconFrame.sectorSweepField.cdown =
        (macLayerPtr->GetNumberAntennaSectors() - 1) - currentDmgBeaconCount;

    aBeaconFrame.beaconIntervalControlField.isResponderTxss =
        (!schedulerPtr->GetShouldDoReceiveSectorSweepInAbft());

    aBeaconFrame.beaconIntervalControlField.numSectorSweepSlots =
        schedulerPtr->GetNumSectorSweepSlots();

    aBeaconFrame.beaconIntervalControlField.numSectorSweepFrames =
        static_cast<unsigned char>(schedulerPtr->GetNumSectorSweepFrames());

    const vector<unsigned int>& bondedChannelList =
        macLayerPtr->GetCurrentBondedChannelList();

    for(size_t i = 0; i < bondedChannelList.size(); i++) {
        aBeaconFrame.bondedChannelList[i] = static_cast<unsigned char>(bondedChannelList[i]);
    }//for//

    AddExtendedScheduleElementsToFrame(*framePtr);

    framePtr->AddPlainStructHeader(aBeaconFrame);

    macLayerPtr->TransmitDirectionalSectorSweepFrame(
        framePtr,
        aBeaconFrame.sectorSweepField.sectorId,
        macLayerPtr->GetAdaptiveTxPowerControllerPtr()->GetDmgBeaconTransmitPowerDbm(),
        delayUntilAirborne,
        transmissionEndTime);

}//SendDirectionalBeaconFrame//



void Dot11ApManagementController::TransmitReceiveSectorSweepFrame()
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    if (currentSectorSweepFrameCount < numberSectorSweepFramesToSend) {

        AssociatedStaInformationEntry& staInfo =
            *associatedStaInformation[sectorSweepDestinationMacAddress];

        BeamformingSectorSelector& beamformingSectorSelectorForSta =
            *sectorSweepInfoForStation[sectorSweepDestinationMacAddress].beamformingSectorSelectorPtr;

        SectorSweepFrameType sectorSweepFrame;

        sectorSweepFrame.header.theFrameControlField.frameTypeAndSubtype = SECTOR_SWEEP_FRAME_TYPE_CODE;
        sectorSweepFrame.header.theFrameControlField.isRetry = 0;
        sectorSweepFrame.header.duration = 0;
        sectorSweepFrame.header.receiverAddress = sectorSweepDestinationMacAddress;
        sectorSweepFrame.transmitterAddress = macLayerPtr->GetMacAddress();
        sectorSweepFrame.sectorSweepField.direction = 0;

        sectorSweepFrame.sectorSweepField.sectorId =
            beamformingSectorSelectorForSta.GetBestTransmissionSectorId();

        sectorSweepFrame.sectorSweepField.cdown =
            (numberSectorSweepFramesToSend - 1) - currentSectorSweepFrameCount;

        sectorSweepFrame.sectorSweepFeedbackField.bestSectorId =
            beamformingSectorSelectorForSta.GetBestTransmissionSectorIdToThisNode();

        unique_ptr<Packet> framePtr = Packet::CreatePacket(*simEngineInterfacePtr, sectorSweepFrame);

        SimTime delayUntilAirborne = ZERO_TIME;

        // Modeling hack:
        // Responder Rxss modeling limitation to ensure first sector beamforming (gain calculation)

        if (currentSectorSweepFrameCount == 0) {
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

        const SimTime nextSectorSweepFrameTransmitTime =
            endTransmissionTime + shortInterframeSpaceDuration;

        if (nextSectorSweepFrameTransmitTime >= allocatedAccessPeriodEndTime) {
            cerr << "Error in 802.11ad STA Mac: sector sweep is too long for period." << endl;
            exit(1);
        }//if//

        (*this).nextDirectionalMgScheduleEvent = SendNextSectorSweepFrame;

        simEngineInterfacePtr->ScheduleEvent(
            directionalMgSuperframeScheduleEventPtr,
            nextSectorSweepFrameTransmitTime);
    }
    else {
        // Sweep is done "restart" allocated access period.

        (*this).ContinueAllocatedAccessPeriod();

    }//if//

}//TransmitSectorSweepFrame//



void Dot11ApManagementController::ContinueReceivingReceiveSectorSweep(const bool isFirstSector)
{
    assert(schedulerPtr->GetShouldDoReceiveSectorSweepInAbft());

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    if (isFirstSector) {
        (*this).currentReceiveSweepSectorId = 0;
        (*this).currentStartReceiveSlotTime = currentTime;
        (*this).nextReceiveSectorSweepSlot++;
        (*this).thereWasASectorSweepCollision = false;
        (*this).lastSectorSweepFrameSourceAddress = MacAddress::invalidMacAddress;
    }
    else {
        (*this).currentReceiveSweepSectorId++;
    }//if//

    macLayerPtr->SwitchToDirectionalAntennaMode(currentReceiveSweepSectorId);

    if (currentReceiveSweepSectorId < (macLayerPtr->GetNumberAntennaSectors() - 1)) {

        const SimTime nextReceiveSectorSweepSectorStartTime =
            (currentTime + macLayerPtr->GetReceiveSweepPerSectorDuration());

        (*this).nextDirectionalMgScheduleEvent = StartReceivingNextSectorOfReceiveSectorSweep;

        simEngineInterfacePtr->ScheduleEvent(
            directionalMgSuperframeScheduleEventPtr,
            nextReceiveSectorSweepSectorStartTime,
            directionalMgSuperframeScheduleEventTicket);
    }
    else {

        if ((!thereWasASectorSweepCollision) &&
            (lastSectorSweepFrameSourceAddress != MacAddress::invalidMacAddress)) {

            StaSectorSweepInfoType& sweepInfo =
                sectorSweepInfoForStation[lastSectorSweepFrameSourceAddress];

            if (sweepInfo.beamformingSectorSelectorPtr != nullptr) {
                sweepInfo.currentBestRxBeamformingSectorId =
                    sweepInfo.beamformingSectorSelectorPtr->GetBestReceiveSectorId();
            }//if//
        }//if//

        if (nextReceiveSectorSweepSlot < schedulerPtr->GetNumSectorSweepSlots()) {
            const SimTime nextReceiveSectorSweepTime =
                currentStartReceiveSlotTime + abftReceiveSectorSweepSlotTime;

            (*this).nextDirectionalMgScheduleEvent = StartReceiveSectorSweep;

            simEngineInterfacePtr->ScheduleEvent(
                directionalMgSuperframeScheduleEventPtr,
                nextReceiveSectorSweepTime,
                directionalMgSuperframeScheduleEventTicket);
        }
        else {
            (*this).nextDirectionalMgScheduleEvent = DataTransferIntervalPeriod;

            simEngineInterfacePtr->ScheduleEvent(
                directionalMgSuperframeScheduleEventPtr,
                dataTransferIntervalStartTime,
                directionalMgSuperframeScheduleEventTicket);

        }//if//
    }//if//

}//ContinueReceivingReceiveSectorSweep//


void Dot11ApManagementController::StartAssociationBeamformingTrainingPeriod()
{
    (*this).lastSectorSweepFrameSourceAddress = MacAddress::invalidMacAddress;
    (*this).thereWasASectorSweepCollision = false;

    (*this).currentReceiveSweepSectorId = QuasiOmniSectorId;
    macLayerPtr->SwitchToQuasiOmniAntennaMode();
    if (macLayerPtr->IsNotReceivingFrames()) {
        macLayerPtr->StartReceivingFrames();
    }//if//

    if (!schedulerPtr->GetShouldDoReceiveSectorSweepInAbft()) {

        (*this).nextDirectionalMgScheduleEvent = DataTransferIntervalPeriod;

        simEngineInterfacePtr->ScheduleEvent(
            directionalMgSuperframeScheduleEventPtr,
            dataTransferIntervalStartTime,
            directionalMgSuperframeScheduleEventTicket);
    }
    else {

        typedef map<MacAddress, StaSectorSweepInfoType>::iterator IterType;

        for(IterType staIterator = sectorSweepInfoForStation.begin();
            (staIterator != sectorSweepInfoForStation.end()); staIterator++) {
            (*staIterator).second.currentBestRxBeamformingSectorId = QuasiOmniSectorId;
        }//for//

        (*this).nextReceiveSectorSweepSlot = 0;
        (*this).ContinueReceivingReceiveSectorSweep(true);

    }//if//

}//StartAssociationBeamformingTrainingPeriod//

void Dot11ApManagementController::ProcessAssociationBeamformingTrainingFeedback()
{
    assert(!schedulerPtr->GetShouldDoReceiveSectorSweepInAbft());

    if (!thereWasASectorSweepCollision) {
        (*this).TransmitAssociationBeamformingTrainingFeedbackFrame();
    }//if//

    const unsigned int nextAbftSectorSweepReceiveSlot =
        (*this).CalculateCurrentAbftSectorSweepSlot() + 1;

    SimTime nextEventTime;

    if (nextAbftSectorSweepReceiveSlot < schedulerPtr->GetNumSectorSweepSlots()) {

        const SimTime nextAbftSectorSweepReceiveSlotTime =
            (*this).CalculateCurrentAbftStartTime() +
            nextAbftSectorSweepReceiveSlot * schedulerPtr->GetAssociationBeamformingTrainingAkaAbftSlotTime();

        (*this).nextDirectionalMgScheduleEvent = AssociationBeamformingTrainingPeriod;

        nextEventTime = nextAbftSectorSweepReceiveSlotTime;

    }
    else {
        (*this).nextDirectionalMgScheduleEvent = DataTransferIntervalPeriod;

        nextEventTime = dataTransferIntervalStartTime;
    }//if//

    simEngineInterfacePtr->ScheduleEvent(
        directionalMgSuperframeScheduleEventPtr,
        nextEventTime,
        directionalMgSuperframeScheduleEventTicket);

}//ProcessAssociationBeamformingTrainingFeedback//

void Dot11ApManagementController::TransmitAssociationBeamformingTrainingFeedbackFrame()
{
    assert(!thereWasASectorSweepCollision);

    StaSectorSweepInfoType& sectorSweepInfo =
        sectorSweepInfoForStation[lastSectorSweepFrameSourceAddress];

    SectorSweepFeedbackFrameType feedbackFrame;

    feedbackFrame.header.theFrameControlField.frameTypeAndSubtype = SECTOR_SWEEP_FEEDBACK_FRAME_TYPE_CODE;
    feedbackFrame.header.theFrameControlField.isRetry = 0;
    feedbackFrame.header.duration = 0;
    feedbackFrame.header.receiverAddress = lastSectorSweepFrameSourceAddress;
    feedbackFrame.transmitterAddress = macLayerPtr->GetMacAddress();
    feedbackFrame.sectorSweepFeedbackField.bestSectorId =
        sectorSweepInfo.beamformingSectorSelectorPtr->GetBestTransmissionSectorIdToThisNode();

    unique_ptr<Packet> framePtr = Packet::CreatePacket(*simEngineInterfacePtr, feedbackFrame);

    unsigned int beamformingSectorId = sectorSweepInfo.forcedRxBeamformingSectorId;
    if (sectorSweepInfo.forcedRxBeamformingSectorId == InvalidSectorId) {
        beamformingSectorId = sectorSweepInfo.beamformingSectorSelectorPtr->GetBestTransmissionSectorId();
    }//if//

    SimTime transmissionEndTime;

    const double transmitPowerDbm =
        macLayerPtr->GetAdaptiveTxPowerControllerPtr()->GetCurrentTransmitPowerDbm(
            MacAddress::GetBroadcastAddress());

    macLayerPtr->TransmitDirectionalSectorSweepFrame(
        framePtr,
        beamformingSectorId,
        transmitPowerDbm,
        ZERO_TIME,
        transmissionEndTime);

    if (sectorSweepInfo.beamformingSectorSelectorPtr->GetBestTransmissionSectorIdToThisNode() !=
        sectorSweepInfo.lastBestMeasuredUplinkTxSectorId) {

        (*this).OutputTraceForNewUplinkBeamformingSector(
            lastSectorSweepFrameSourceAddress,
            sectorSweepInfo.beamformingSectorSelectorPtr->GetBestTransmissionSectorIdToThisNode());

    }//if//

    sectorSweepInfo.receivedASectorSweepFrame = false;

}//TransmitAssociationBeamformingTrainingFeedbackFrame//



void Dot11ApManagementController::StartBeaconTransmissionIntervalPeriod()
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    SimTime nextEventTime;

    SimTime delayUntilAirborne = ZERO_TIME;

    if (currentDmgBeaconCount == 0) {
        (*this).currentBeaconSuperframeStartTime = currentTime;
        schedulerPtr->CalcScheduleForNextBeaconPeriod();

        delayUntilAirborne =
            macLayerPtr->GetMediumBeamformingInterframeSpaceDuration();

        (*this).dataTransferIntervalStartTime =
            currentTime +
            schedulerPtr->GetDmgBeaconTransmissionIntervalAkaBtiDuration() +
            schedulerPtr->GetAssociationBeamformingTrainingAkaAbftDuration();

        (*this).dataTransferIntervalEndTime =
            (dataTransferIntervalStartTime + schedulerPtr->GetDataTransferIntervalAkaDtiDuration());

        macLayerPtr->StopReceivingFrames();
    }//if//


    if (currentDmgBeaconCount < macLayerPtr->GetNumberAntennaSectors()) {

        SimTime endTransmissionTime;
        (*this).SendDirectionalBeaconFrame(delayUntilAirborne, endTransmissionTime);
        (*this).currentDmgBeaconCount++;

        const SimTime endOfBtiInterval =
            currentBeaconSuperframeStartTime +
            schedulerPtr->GetDmgBeaconTransmissionIntervalAkaBtiDuration();

        if (endTransmissionTime > endOfBtiInterval) {
            cerr << "Error 802.11ad: BTI interval (fixed duration) is too short for beacons." << endl;
            exit(1);
        }//if//

        nextEventTime =
            endTransmissionTime + macLayerPtr->GetShortBeamformingInterframeSpaceDuration();
    }
    else {
        (*this).currentDmgBeaconCount = 0;

        macLayerPtr->StartReceivingFrames();
        (*this).currentDataTransferIntervalListIndex = 0;

        (*this).nextDirectionalMgScheduleEvent = AssociationBeamformingTrainingPeriod;

        nextEventTime =
            currentBeaconSuperframeStartTime +
            schedulerPtr->GetDmgBeaconTransmissionIntervalAkaBtiDuration();

    }//if//

    simEngineInterfacePtr->ScheduleEvent(
        directionalMgSuperframeScheduleEventPtr,
        nextEventTime,
        directionalMgSuperframeScheduleEventTicket);

}//StartBeaconTransmissionIntervalPeriod//

SimTime Dot11ApManagementController::CalculateCurrentAbftStartTime() const
{
    return (currentBeaconSuperframeStartTime +
            schedulerPtr->GetDmgBeaconTransmissionIntervalAkaBtiDuration());
}//CalculateCurrentAbftStartTime//

unsigned int Dot11ApManagementController::CalculateCurrentAbftSectorSweepSlot() const
{
    const SimTime currentAbftStartTime =
        (*this).CalculateCurrentAbftStartTime();

    const SimTime currentTime =
        simEngineInterfacePtr->CurrentTime();

    assert(currentTime > currentAbftStartTime);

    const SimTime elapsedTime = (currentTime - currentAbftStartTime);

    const unsigned int currentAbftSectorSweepSlot =
        static_cast<unsigned int>(elapsedTime / schedulerPtr->GetAssociationBeamformingTrainingAkaAbftSlotTime());

    return currentAbftSectorSweepSlot;

}///CalculateCurrentAbftStartTime




void Dot11ApManagementController::StartDataTransferIntervalPeriod()
{
    bool wasFound;
    GetNextAllocatedAccessPeriodIndex(0, wasFound, (*this).currentDataTransferIntervalListIndex);

    assert(wasFound);

    const SimTime startTime =
        schedulerPtr->GetDtiPeriodRelativeStartTime(currentDataTransferIntervalListIndex);

    if (startTime == ZERO_TIME) {

        (*this).StartAllocatedAccessPeriod();
    }
    else {
        (*this).nextDirectionalMgScheduleEvent = AllocatedAccessPeriod;

        simEngineInterfacePtr->ScheduleEvent(
            directionalMgSuperframeScheduleEventPtr,
            (dataTransferIntervalStartTime + startTime),
            directionalMgSuperframeScheduleEventTicket);
    }//if//

}//StartDataTransferIntervalPeriod//



void Dot11ApManagementController::ScheduleEventForNextAllocatedAccessPeriodOrSuperframe()
{
    (*this).currentDataTransferIntervalListIndex++;

    bool wasFound;
    GetNextAllocatedAccessPeriodIndex(
        currentDataTransferIntervalListIndex,
        wasFound, (*this).currentDataTransferIntervalListIndex);

    if (wasFound) {
        const SimTime startTime =
            schedulerPtr->GetDtiPeriodRelativeStartTime(currentDataTransferIntervalListIndex);

        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

        if ((dataTransferIntervalStartTime + startTime) < currentTime) {
            cerr << "Error: DTI access period scheduling conflict at node " << theNodeId
                 << " at relative start time " << startTime << "ns" << endl;
            exit(1);
        }//if//

        (*this).nextDirectionalMgScheduleEvent = AllocatedAccessPeriod;

        simEngineInterfacePtr->ScheduleEvent(
            directionalMgSuperframeScheduleEventPtr,
            (dataTransferIntervalStartTime + startTime),
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



void Dot11ApManagementController::StartAllocatedAccessPeriod()
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

   (*this).allocatedAccessPeriodEndTime =
       (currentTime + schedulerPtr->GetDtiPeriodDuration(currentDataTransferIntervalListIndex));

    const bool isAContentionPeriod =
        schedulerPtr->DtiPeriodIsContentionBased(currentDataTransferIntervalListIndex);

    const NodeId sourceNodeId =
        schedulerPtr->GetDtiPeriodSourceNodeId(currentDataTransferIntervalListIndex);

    const NodeId destinationNodeId =
        schedulerPtr->GetDtiPeriodDestinationNodeId(currentDataTransferIntervalListIndex);

    if ((!isAContentionPeriod) && (sourceNodeId == theNodeId) &&
        (schedulerPtr->MustDoRxssBeamformingTrainingInDtiPeriod(currentDataTransferIntervalListIndex))) {

        bool success;
        LookupAssociatedNodeMacAddress(
            destinationNodeId, success,
            (*this).sectorSweepDestinationMacAddress);

        if (success) {
            (*this).currentSectorSweepFrameCount = 0;
            (*this).numberSectorSweepFramesToSend =
                GetNumberAntennaSectorsFor(sectorSweepDestinationMacAddress);

            if (numberSectorSweepFramesToSend != 0) {
                macLayerPtr->StopReceivingFrames();

                (*this).TransmitReceiveSectorSweepFrame();
                return;
            }//if//
        }//if//
    }//if//

    (*this).ContinueAllocatedAccessPeriod();

}//StartAllocatedAccessPeriod//



void Dot11ApManagementController::ContinueAllocatedAccessPeriod()
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    const bool isAContentionPeriod =
        schedulerPtr->DtiPeriodIsContentionBased(currentDataTransferIntervalListIndex);

    const NodeId sourceNodeId =
        schedulerPtr->GetDtiPeriodSourceNodeId(currentDataTransferIntervalListIndex);

    const NodeId destinationNodeId =
        schedulerPtr->GetDtiPeriodDestinationNodeId(currentDataTransferIntervalListIndex);

    // Setup receive pattern:

    if (isAContentionPeriod) {
        macLayerPtr->SwitchToQuasiOmniAntennaMode();
    }
    else {
        if (sourceNodeId != theNodeId) {

            assert((destinationNodeId == theNodeId) || (destinationNodeId == ScenSim::AnyNodeId));

            macLayerPtr->SwitchToDirectionalAntennaMode(GetAntennaSectorIdForNode(sourceNodeId));
        }
        else {
            assert(sourceNodeId == theNodeId);

            bool success;
            MacAddress destinationMacAddress;

            LookupAssociatedNodeMacAddress(
                destinationNodeId, success, destinationMacAddress);

            if (success) {
                StaSectorSweepInfoType& sweepInfo =
                    sectorSweepInfoForStation[destinationMacAddress];

                macLayerPtr->SwitchToDirectionalAntennaMode(
                    sweepInfo.currentBestRxBeamformingSectorId);

            }//if//

        }//if//
    }//if//

    macLayerPtr->StartReceivingFrames();

    if ((isAContentionPeriod) || (sourceNodeId == theNodeId)) {

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



void Dot11ApManagementController::StartSendingInAllocatedAccessPeriod()
{
    if (schedulerPtr->DtiPeriodIsContentionBased(currentDataTransferIntervalListIndex)) {
        macLayerPtr->StartContentionPeriod(allocatedAccessPeriodEndTime - EPSILON_TIME);
    }
    else {
        macLayerPtr->StartNonContentionPeriod(allocatedAccessPeriodEndTime - EPSILON_TIME);
    }//if//

    (*this).ScheduleEventForNextAllocatedAccessPeriodOrSuperframe();

}//StartSendingInAllocatedAccessPeriod//


void Dot11ApManagementController::OutputTraceForNewDownlinkBeamformingSector(
    const MacAddress& staAddress, const unsigned int sectorId)
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacDownlinkBeamformingTraceRecord traceData;

            traceData.sourceNodeId = staAddress.ExtractNodeId();
            traceData.sectorId = sectorId;
            traceData.transmissionSectorMetric = 0.0;
            traceData.sourceIsStaOrNot = true;

            assert(sizeof(traceData) == MAC_DOWNLINK_BEAMFORMING_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(Dot11Mac::modelName, theInterfaceId, "NewDlBeamSector", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream << "STANodeId= " << staAddress.ExtractNodeId() << " SectorId= " << sectorId;
            simEngineInterfacePtr->OutputTrace(
                Dot11Mac::modelName, theInterfaceId, "NewDlBeamSector", msgStream.str());
        }//if//
    }//if//

}//OutputTraceForNewUplinkBeamformingSector//



void Dot11ApManagementController::OutputTraceForNewUplinkBeamformingSector(
    const MacAddress& staAddress, const unsigned int sectorId)
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacUplinkBeamformingTraceRecord traceData;

            traceData.sourceNodeId = staAddress.ExtractNodeId();
            traceData.sectorId = sectorId;
            traceData.sourceIsStaOrNot = true;

            assert(sizeof(traceData) == MAC_UPLINK_BEAMFORMING_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(Dot11Mac::modelName, theInterfaceId, "NewUlBeamSector", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream << "STANodeId= " << staAddress.ExtractNodeId() << " SectorId= " << sectorId;
            simEngineInterfacePtr->OutputTrace(
                Dot11Mac::modelName, theInterfaceId, "NewUlBeamSector", msgStream.str());
        }//if//
    }//if//

}//OutputTraceForNewUplinkBeamformingSector//



void Dot11ApManagementController::OutputTraceForDmgFrameReceiveOnAp(const SectorSweepFrameType& sectorSweepFrame)
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (! simEngineInterfacePtr->BinaryOutputIsOn()) {
            ostringstream msgStream;

            msgStream << "STANodeId=" << sectorSweepFrame.transmitterAddress.ExtractNodeId()
                << " TxSectorId= " << sectorSweepFrame.sectorSweepField.sectorId
                << " RxSectorId= " << currentReceiveSweepSectorId
                << " RssiDb= " << macLayerPtr->GetRssiOfLastFrameDbm()
                << " SinrDb= " << macLayerPtr->GetSinrOfLastFrameDb();
            simEngineInterfacePtr->OutputTrace(
                Dot11Mac::modelName, theInterfaceId, "ReceiveDmgFrameOnAp", msgStream.str());
        }//if//
    }//if//

}//OutputTraceForDmgFrameReceive//



}//namespace//

