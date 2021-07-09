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
#include "dot11_ratecontrol.h"

namespace Dot11 {

using ScenSim::ConvertToUChar;

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

    if (associationFrame.theHtCapabilitiesFrameElement.highThroughputModeIsEnabled) {
        if (associationFrame.theHtCapabilitiesFrameElement.aggregateMpdusAreEnabled != 0) {
            macLayerPtr->SetMpduFrameAggregationIsEnabledFor(transmitterAddress);
        }//if//
        //Removed Feature: if (associationFrame.theHtCapabilitiesFrameElement.aggregateMsdusAreEnabled != 0) {
        //Removed Feature:     macLayerPtr->SetMsduFrameAggregationIsEnabledFor(transmitterAddress);
        //Removed Feature: }//if//
    }
    else {
        //Removed Feature: if ((associationFrame.theHtCapabilitiesFrameElement.aggregateMpdusAreEnabled != 0) ||
        //Removed Feature:    (associationFrame.theHtCapabilitiesFrameElement.aggregateMsdusAreEnabled != 0)) {
        if (associationFrame.theHtCapabilitiesFrameElement.aggregateMpdusAreEnabled != 0) {

            cerr << "Error A-MPDU aggregation is enabled but \"High Throughput\" (HT) is not." << endl;
            exit(1);

        }//if//
    }//if//

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

    if (reassociationFrame.theHtCapabilitiesFrameElement.highThroughputModeIsEnabled) {
        if (reassociationFrame.theHtCapabilitiesFrameElement.aggregateMpdusAreEnabled != 0) {
            macLayerPtr->SetMpduFrameAggregationIsEnabledFor(transmitterAddress);
        }//if//
        //Removed Feature: if (reassociationFrame.theHtCapabilitiesFrameElement.aggregateMsdusAreEnabled != 0) {
        //Removed Feature:        macLayerPtr->SetMsduFrameAggregationIsEnabledFor(transmitterAddress);
        //Removed Feature: }//if//
    }//if//

    macLayerPtr->ResetOutgoingLinksTo(transmitterAddress);

    SendReassociationNotification(transmitterAddress, reassociationFrame.currentApAddress);

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
            (*this).associationIdIsBeingUsed.reset(
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
    typedef map<MacAddress, shared_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

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

void Dot11ApManagementController::AddNewAssociatedStaRecord(
    const MacAddress& staAddress,
    const unsigned int stationBandwidthNumChannels,
    const bool highThroughputModeIsEnabled)
{
    shared_ptr<AssociatedStaInformationEntry>
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
        make_pair(staAddress, newStaInfoEntryPtr));

    macLayerPtr->GetAdaptiveRateControllerPtr()->AddNewStation(
        staAddress,
        stationBandwidthNumChannels,
        highThroughputModeIsEnabled);

}//AddNewAssociatedStaRecord//


}//namespace//

