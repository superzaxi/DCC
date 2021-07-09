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
            (associationFrame.ahCapabilityInfo.usesRestrictedAccessWindows == 1));
    }//if//

    if (associationFrame.theHtCapabilitiesFrameElement.aggregateMpdusAreEnabled != 0) {
        macLayerPtr->SetMpduFrameAggregationIsEnabledFor(transmitterAddress);
    }//if//

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
            (reassociationFrame.ahCapabilityInfo.usesRestrictedAccessWindows == 1));
    }//if//

    SendReassociationNotification(transmitterAddress, reassociationFrame.currentApAddress);

    if (reassociationFrame.theHtCapabilitiesFrameElement.aggregateMpdusAreEnabled != 0) {
        macLayerPtr->SetMpduFrameAggregationIsEnabledFor(transmitterAddress);
    }//if//

    macLayerPtr->SendNewLinkToANodeNotificationToNetworkLayer(transmitterAddress);
    macLayerPtr->SendReassociationResponse(
        transmitterAddress,
        associatedStaInformation[transmitterAddress]->theAssociationId);

}//ProcessReassociationRequestFrame//



void Dot11ApManagementController::ProcessManagementFrame(const Packet& managementFrame)
{
    const ManagementFrameHeader& header =
        managementFrame.GetAndReinterpretPayloadData<ManagementFrameHeader>();

    switch(header.header.theFrameControlField.frameTypeAndSubtype) {
    case ASSOCIATION_REQUEST_FRAME_TYPE_CODE: {

        (*this).ProcessAssociationRequestFrame(managementFrame);

        break;
    }
    case REASSOCIATION_REQUEST_FRAME_TYPE_CODE: {

        (*this).ProcessReassociationRequestFrame(managementFrame);

        break;
    }
    case DISASSOCIATION_FRAME_TYPE_CODE: {
        if (IsAnAssociatedStaAddress(header.transmitterAddress)) {
            (*this).associationIdToStaInfoMap.erase(
                associatedStaInformation[header.transmitterAddress]->theAssociationId);
            (*this).associatedStaInformation.erase(header.transmitterAddress);
        }
        break;
    }
    case AUTHENTICATION_FRAME_TYPE_CODE: {

        (*this).ProcessAuthenticationFrame(header.transmitterAddress);

        break;
    }
    case ASSOCIATION_RESPONSE_FRAME_TYPE_CODE:
    case REASSOCIATION_RESPONSE_FRAME_TYPE_CODE:
    case BEACON_FRAME_TYPE_CODE:
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
    typedef map<MacAddress, shared_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

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
                macLayerPtr->RequeueManagementFrame(bufferElem.packetPtr);
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
    typedef map<MacAddress, shared_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

    macLayerPtr->SwitchToNormalAccessMode();

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

    // Sending Random Bits instead of FCS:
    beaconHeader.managementHeader.fcsRandomBitsHigh = static_cast<unsigned char>(
        (*this).aRandomNumberGenerator.GenerateRandomInt(0, UCHAR_MAX));
    beaconHeader.managementHeader.fcsRandomBitsLow = static_cast<unsigned char>(
        (*this).aRandomNumberGenerator.GenerateRandomInt(0, UCHAR_MAX));

    const vector<unsigned int>& channelList = macLayerPtr->GetCurrentBondedChannelList();
    for(unsigned int i = 0; (i < channelList.size()); i++) {
        beaconHeader.htOperationFrameElement.bondedChannelList[i] = ConvertToUChar(channelList[i]);
    }//for//

    unsigned int extraSizeBytes = sizeof(BeaconFrame);

    unsigned int numberWindows = 0;

    if (!cannedRawWindows.empty()) {

        const unsigned int beaconNumber =
            cannedRawWindows[currentCannedRawWindowIndex].containingBeaconNumber;

        numberWindows = 1;

        for(unsigned int i = currentCannedRawWindowIndex + 1;
             ((i < cannedRawWindows.size()) &&
              (cannedRawWindows[i].containingBeaconNumber == beaconNumber)); i++) {

            numberWindows++;

        }//for//

        if (numberWindows > 0) {
            extraSizeBytes +=
                (sizeof(InfoElementHeaderType) +
                 (numberWindows * sizeof(RestrictedAccessWindowAssignmentFieldType)));
        }//if//
    }//if//

    unique_ptr<Packet> framePtr;

    if (!aTrafficIndicationBitMap.IsEmpty()) {

        extraSizeBytes += sizeof(TrafficIndicationMapElementHeader);

        framePtr =
            Packet::CreatePacketWithExtraHeaderSpace(
                *simEngineInterfacePtr,
                aTrafficIndicationBitMap.GetBitMapByteVector(),
                extraSizeBytes);

        TrafficIndicationMapElementHeader timHeader;
        timHeader.bitMapByteOffset = aTrafficIndicationBitMap.GetStartByteOffset();
        framePtr->AddPlainStructHeader(timHeader);
    }//if//

    if (numberWindows > 0) {
        if (framePtr == nullptr) {
            framePtr =
                Packet::CreatePacketWithExtraHeaderSpace(*simEngineInterfacePtr, extraSizeBytes);
        }//if//

        for(unsigned int i = 0; (i < numberWindows); i++) {

            const unsigned int windowIndex =
                (currentCannedRawWindowIndex + numberWindows - 1) - i;

            const RestrictedAccessWindowInfoType& window = cannedRawWindows[windowIndex];

            RestrictedAccessWindowAssignmentFieldType field;

            field.rawSlotDefinition.SetSlotDuration(window.slotDuration);

            const SimTime numberOfSlots =
                ((window.relativeEndTime - window.relativeStartTime) / window.slotDuration);

            assert(numberOfSlots > 0);
            assert(numberOfSlots <= RawSlotDefinitionFieldType::maxNumberOfSlots);

            field.rawSlotDefinition.numberOfSlots = static_cast<unsigned char>(numberOfSlots);

            field.rawGroup.SetRawAssociationIdRange(
                window.associationIdRangeStart,
                window.associationIdRangeEnd);

            framePtr->AddPlainStructHeader(field);

        }//for//
        InfoElementHeaderType rpsHeader;
        rpsHeader.elementId = RestrictedAccessWindowParmInfoElementId;
        rpsHeader.length =
            ConvertToUChar(numberWindows * sizeof(RestrictedAccessWindowAssignmentFieldType));

        framePtr->AddPlainStructHeader(rpsHeader);
    }
    else {
        framePtr = Packet::CreatePacketWithExtraHeaderSpace(*simEngineInterfacePtr, extraSizeBytes);
    }//if//

    framePtr->AddPlainStructHeader(beaconHeader);
    macLayerPtr->SendManagementFrame(framePtr);

}//SendBeaconFrame//



void Dot11ApManagementController::ProcessBeaconFrameTransmissionNotification()
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    (*this).lastBeaconTransmitEndTime = currentTime;

    if (currentCannedRawWindowIndex >= cannedRawWindows.size()) {
        cerr << "Error: Too few RAW definitions specified in Canned RAW config file." << endl;
        exit(1);
    }//if//

    const unsigned int beaconNumber =
        cannedRawWindows[currentCannedRawWindowIndex].containingBeaconNumber;

    const RestrictedAccessWindowInfoType& rawInfo = cannedRawWindows[currentCannedRawWindowIndex];

    if (rawInfo.relativeStartTime == ZERO_TIME) {
       // Start sending Now.

       (*this).StartUplinkRestrictedAccessWindow();
    }
    else {
        simEngineInterfacePtr->ScheduleEvent(
            restrictedAccessWindowStartEventPtr,
            (currentTime + rawInfo.relativeStartTime));

        macLayerPtr->StartRestrictedAccessWindowPeriod(currentTime + rawInfo.relativeStartTime);

    }//if//

}//ProcessBeaconFrameTransmissionNotification//




void Dot11ApManagementController::StartUplinkRestrictedAccessWindow()
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    const RestrictedAccessWindowInfoType& rawInfo = cannedRawWindows[currentCannedRawWindowIndex];

    simEngineInterfacePtr->ScheduleEvent(
        restrictedAccessWindowEndEventPtr,
        (currentTime + (rawInfo.relativeEndTime - rawInfo.relativeStartTime)));

    macLayerPtr->SwitchToReceiveOnlyMode();

}//StartDownlinkRestrictedAccessWindow//


void Dot11ApManagementController::EndUplinkRestrictedAccessWindow()
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    const RestrictedAccessWindowInfoType& rawInfo = cannedRawWindows[currentCannedRawWindowIndex];

    if (((currentCannedRawWindowIndex + 1) >= cannedRawWindows.size()) ||
        (cannedRawWindows[currentCannedRawWindowIndex + 1].containingBeaconNumber !=
         rawInfo.containingBeaconNumber)) {

        // Last RAW window of Beacon period.

        if (rawInfo.numberRepetitions != RestrictedAccessWindowInfoType::InfiniteRepetitions) {
            (*this).currentBeaconRepetitionCount++;
        }//if//

        if (currentBeaconRepetitionCount >= rawInfo.numberRepetitions) {

            // Goto next Beacon's RAW definitions. (Repetition ended).

            (*this).currentBeaconRepetitionCount = 0;

            currentCannedRawWindowIndex++;
        }
        else {
            // Reuse RAW definitions for next beacon period (Due to beacon definition repetition).

            while ((currentCannedRawWindowIndex > 0) &&
                   (cannedRawWindows[currentCannedRawWindowIndex - 1].containingBeaconNumber ==
                    rawInfo.containingBeaconNumber)) {

                currentCannedRawWindowIndex--;

            }//while//
        }//if//

        // Schedule next beacon event.

        simEngineInterfacePtr->ScheduleEvent(
            beaconIntervalTimeoutEventPtr,
            (lastBeaconTransmitEndTime + rawInfo.beaconIntervalDuration));

        macLayerPtr->SwitchToNormalAccessMode();
    }
    else {
        currentCannedRawWindowIndex++;
        const RestrictedAccessWindowInfoType& nextRawInfo = cannedRawWindows[currentCannedRawWindowIndex];

        if (nextRawInfo.relativeStartTime == rawInfo.relativeEndTime) {

            // Start next RAW now (no-gap).

            (*this).StartUplinkRestrictedAccessWindow();
        }
        else {
            // There is gap between RAW windows.

            simEngineInterfacePtr->ScheduleEvent(
                restrictedAccessWindowStartEventPtr,
                (lastBeaconTransmitEndTime + nextRawInfo.relativeStartTime));

            macLayerPtr->SwitchToNormalAccessMode();

        }//if//
    }//if//

}//StartDownlinkRestrictedAccessWindow//


}//namespace//


