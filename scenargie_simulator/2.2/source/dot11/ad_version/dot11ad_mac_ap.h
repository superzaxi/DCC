// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11AD_MAC_AP_H
#define DOT11AD_MAC_AP_H

#include "scensim_engine.h"
#include "scensim_netsim.h"

#include "dot11ad_common.h"
#include "dot11ad_headers.h"

#include "dot11ad_sectorselector.h"
#include "dot11ad_scheduler.h"


#include <queue>
#include <map>
#include <string>
#include <iomanip>

namespace Dot11ad {

using std::shared_ptr;
using std::unique_ptr;
using std::deque;
using std::map;
using std::cout;
using std::cerr;
using std::endl;
using std::hex;
using std::string;
using std::move;

using ScenSim::SimulationEngineInterface;
using ScenSim::SimulationEvent;
using ScenSim::EventRescheduleTicket;
using ScenSim::SimTime;
using ScenSim::MILLI_SECOND;
using ScenSim::EPSILON_TIME;
using ScenSim::ZERO_TIME;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeId;
using ScenSim::InterfaceOrInstanceId;
using ScenSim::RandomNumberGenerator;
using ScenSim::RandomNumberGeneratorSeed;
using ScenSim::PacketPriority;
using ScenSim::EtherTypeField;
using ScenSim::NetworkAddress;
using ScenSim::HashInputsToMakeSeed;


using ScenSim::ConvertTimeToDoubleSecs;

class Dot11Mac;

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

class Dot11ApManagementController {
public:

    static shared_ptr<Dot11ApManagementController> Create(
        const shared_ptr<Dot11Mac>& macLayerPtr,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId,
        const RandomNumberGeneratorSeed& interfaceSeed)
    {
        shared_ptr<Dot11ApManagementController> controllerPtr(
            new Dot11ApManagementController(
                macLayerPtr,
                simulationEngineInterfacePtr,
                theParameterDatabaseReader,
                theNodeId,
                theInterfaceId,
                interfaceSeed));

        controllerPtr->CompleteInitialization(theParameterDatabaseReader);
        return (controllerPtr);
    }

    void ProcessManagementFrame(const Packet& managementFrame);

    void ReceiveFramePowerManagementBit(
        const MacAddress& sourceAddress,
        const bool framePowerManagementBitIsOn);

    bool IsAnAssociatedStaAddress(const MacAddress& theMacAddress) const;

    void LookupAssociatedNodeMacAddress(
        const NodeId& theNodeId,
        bool& wasFound,
        MacAddress& macAddress) const;

    void GetAssociatedStaAddressList(vector<MacAddress>& associatedStaAddressList) const;

    bool StationIsAsleep(const MacAddress& staAddress) const;

    void BufferPacketForSleepingStation(
        const MacAddress& staAddress,
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& destinationNetworkAddress,
        const PacketPriority& priority,
        const EtherTypeField etherType,
        const SimTime& queueInsertTime,
        const unsigned int retryTxCount);

    void BufferManagementFrameForSleepingStation(
        const MacAddress& staAddress,
        unique_ptr<Packet>& framePtr,
        const unsigned int retryTxCount,
        const SimTime& timestamp);

    void GetPowerSaveBufferedPacket(
        const MacAddress& staAddress,
        bool& wasRetrieved,
        unique_ptr<Packet>& packetToSendPtr,
        unsigned int& retryTxCount,
        PacketPriority& priority,
        EtherTypeField& etherType);

    // 802.11ad

    unsigned int GetTxAntennaSectorId(const MacAddress& destinationAddress) const;
    unsigned int GetAntennaSectorIdForNode(const NodeId& theNodeId) const;

private:

    Dot11ApManagementController(
        const shared_ptr<Dot11Mac>& initMacLayerPtr,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& initNodeId,
        const InterfaceOrInstanceId& initInterfaceId,
        const RandomNumberGeneratorSeed& interfaceSeed);

    void CompleteInitialization(const ParameterDatabaseReader& theParameterDatabaseReader);


    //------------------------------------------------------

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    NodeId theNodeId;
    InterfaceOrInstanceId theInterfaceId;
    string ssid;

    shared_ptr<Dot11Mac> macLayerPtr;

    AssociationId currentAssociationId;

    static const int SEED_HASH = 31843124;
    RandomNumberGenerator aRandomNumberGenerator;

    struct PowerSavePacketBufferElem {
        unique_ptr<Packet> packetPtr;
        NetworkAddress destinationNetworkAddress;
        PacketPriority priority;
        EtherTypeField etherType;
        unsigned int retryTxCount;
        SimTime timestamp;

        PowerSavePacketBufferElem() : packetPtr(nullptr) { }

        PowerSavePacketBufferElem(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initDestinationNetworkAddress,
            const PacketPriority& initPriority,
            const EtherTypeField& initEtherType,
            const unsigned int initRetryTxCount,
            const SimTime& initTimestamp)
        :
            packetPtr(move(initPacketPtr)),
            destinationNetworkAddress(initDestinationNetworkAddress),
            priority(initPriority),
            etherType(initEtherType),
            retryTxCount(initRetryTxCount),
            timestamp(initTimestamp)
        {
        }

        void operator=(PowerSavePacketBufferElem&& right)
        {
            packetPtr = move(right.packetPtr);
            destinationNetworkAddress = right.destinationNetworkAddress;
            priority = right.priority;
            etherType = right.etherType;
            retryTxCount = right.retryTxCount;
            timestamp = right.timestamp;
        }

        PowerSavePacketBufferElem(PowerSavePacketBufferElem&& right) { (*this) = move(right); }

    };//PowerSavePacketBufferElem//


    BeamformingSectorSelectorFactory sectorSelectorFactory;

    struct StaSectorSweepInfoType {

        unsigned int numberStaBeamformingSectors;

        unique_ptr<BeamformingSectorSelector> beamformingSectorSelectorPtr;

        bool receivedASectorSweepFrame;

        unsigned int forcedTxBeamformingSectorId;
        unsigned int forcedRxBeamformingSectorId;

        unsigned int currentBestRxBeamformingSectorId;

        // Only for trace output:

        unsigned int lastBestMeasuredUplinkTxSectorId;
        unsigned int lastBestReportedDownlinkTxSectorId;

        StaSectorSweepInfoType() :
            numberStaBeamformingSectors(0),
            receivedASectorSweepFrame(false),
            forcedTxBeamformingSectorId(InvalidSectorId),
            forcedRxBeamformingSectorId(InvalidSectorId),
            currentBestRxBeamformingSectorId(QuasiOmniSectorId),
            lastBestMeasuredUplinkTxSectorId(QuasiOmniSectorId),
            lastBestReportedDownlinkTxSectorId(QuasiOmniSectorId) { }

    };//StaSectorSweepInfoType//

    struct AssociatedStaInformationEntry {
        AssociationId theAssociationId;
        bool isInPowersaveMode;
        deque<PowerSavePacketBufferElem> powerSavePacketBuffer;

        AssociatedStaInformationEntry() :
            isInPowersaveMode(false), theAssociationId(InvalidAssociationId) { }
    };

    map<MacAddress, unique_ptr<AssociatedStaInformationEntry> > associatedStaInformation;

    // Need to reset with timer
    map<MacAddress, StaSectorSweepInfoType> sectorSweepInfoForStation;

    std::bitset<MaxAssociationId+1> associationIdIsBeingUsed;

    unique_ptr<DmgScheduler> schedulerPtr;

    //------------------------------------------------------

    //11ad// SimTime beaconInterval;

    //11ad// void BeaconIntervalTimeout();

    //11ad// class BeaconIntervalTimeoutEvent : public SimulationEvent {
    //11ad// public:
    //11ad//     BeaconIntervalTimeoutEvent(Dot11ApManagementController* initApControllerPtr) :
    //11ad//         apControllerPtr(initApControllerPtr) { }
    //11ad//     void ExecuteEvent() { apControllerPtr->BeaconIntervalTimeout(); }
    //11ad// private:
    //11ad//     Dot11ApManagementController* apControllerPtr;
    //11ad// };

    //11ad// shared_ptr <BeaconIntervalTimeoutEvent> beaconIntervalTimeoutEventPtr;
    //11ad// EventRescheduleTicket beaconIntervalRescheduleTicket;


    //------------------------------------------------------

    // 802.11ad:

    map<NodeId, unsigned int> forcedTxBeamformingSectorIdMap;
    map<NodeId, unsigned int> forcedRxBeamformingSectorIdMap;
    map<NodeId, double> forcedLinkTxPowerDbmMap;

    // For Responder Receive Sector Sweeps (Responder-RXSS)
    // (Responder (STA) transmits and AP sector sweeps receive).

    SimTime abftReceiveSectorSweepSlotTime;

    MacAddress lastSectorSweepFrameSourceAddress;
    bool thereWasASectorSweepCollision;

    unsigned int nextReceiveSectorSweepSlot;
    unsigned int currentReceiveSweepSectorId;

    //-----------------------------------------------------

    SimTime currentBeaconSuperframeStartTime;
    SimTime dataTransferIntervalStartTime;
    SimTime dataTransferIntervalEndTime;
    SimTime allocatedAccessPeriodEndTime;

    unsigned int currentDmgBeaconCount;
    unsigned int currentDataTransferIntervalListIndex;

    unsigned int currentSectorSweepFrameCount;
    unsigned int numberSectorSweepFramesToSend;
    MacAddress sectorSweepDestinationMacAddress;

    void ProcessDirectionalMgSuperframeScheduleEvent();

    class DirectionalMgSuperframeScheduleEvent : public SimulationEvent {
    public:
        DirectionalMgSuperframeScheduleEvent(Dot11ApManagementController* initApControllerPtr) :
            apControllerPtr(initApControllerPtr) { }
        void ExecuteEvent() { apControllerPtr->ProcessDirectionalMgSuperframeScheduleEvent(); }
    private:
        Dot11ApManagementController* apControllerPtr;
    };

    shared_ptr<DirectionalMgSuperframeScheduleEvent> directionalMgSuperframeScheduleEventPtr;
    EventRescheduleTicket directionalMgSuperframeScheduleEventTicket;

    enum DmgSuperframeScheduleEventTypes {
        BeaconTransmissionIntervalPeriod,
        AssociationBeamformingTrainingPeriod,
        TransmitAssociationBeamformingTrainingFeedbackFrameEvent,
        StartReceiveSectorSweep,
        StartReceivingNextSectorOfReceiveSectorSweep,
        SendNextSectorSweepFrame,
        DataTransferIntervalPeriod,
        AllocatedAccessPeriod,
        AllocatedAccessPeriodAfterGap,
    };

    DmgSuperframeScheduleEventTypes nextDirectionalMgScheduleEvent;

    SimTime currentStartReceiveSlotTime;

    //------------------------------------------------------

    SimTime authProcessingDelay;
    void ProcessAuthenticationFrame(const MacAddress& transmitterAddress);

    void SendAuthentication(const MacAddress& transmitterAddress);

    class SendAuthenticationEvent : public SimulationEvent {
    public:
        SendAuthenticationEvent(
            Dot11ApManagementController* initApControllerPtr,
            const MacAddress& initReceiverAddress)
            :
            apControllerPtr(initApControllerPtr),
            receiverAddress(initReceiverAddress)
        {}

        void ExecuteEvent() { apControllerPtr->SendAuthentication(receiverAddress); }
    private:
        Dot11ApManagementController* apControllerPtr;
        MacAddress receiverAddress;
    };

    //-----------------------------------------------------

    void ReadInAdParameters(const ParameterDatabaseReader& theParameterDatabaseReader);

    void SendBeaconFrame();
    void ProcessAssociationRequestFrame(const Packet& aFrame);
    void ProcessReassociationRequestFrame(const Packet& aFrame);
    void AddNewAssociatedStaRecord(
        const MacAddress& staAddress,
        const unsigned int stationBandwidthNumChannels,
        const bool highThroughputModeIsEnabled);

    void SendReassociationNotification(
        const MacAddress& staAddress,
        const MacAddress& apAddress);

    void SendDirectionalBeaconFrame(
        const SimTime& delayUntilAirborne,
        SimTime& transmissionEndTime);

    bool IsBeamformingSectorSelectorInitialized(const MacAddress& macAddress) const;
    void InitBeamformingSectorSelectorFor(const MacAddress& macAddress);

    void StartBeaconTransmissionIntervalPeriod();
    void StartAssociationBeamformingTrainingPeriod();
    void ProcessAssociationBeamformingTrainingFeedback();
    void ProcessSectorSweepFrame(const SectorSweepFrameType& sectorSweepFrame);
    void TransmitAssociationBeamformingTrainingFeedbackFrame();

    void TransmitReceiveSectorSweepFrame();

    void ContinueReceivingReceiveSectorSweep(const bool isFirstSector);

    void GetNextAllocatedAccessPeriodIndex(
        const unsigned int startIndex,
        bool& wasFound,
        unsigned int& periodIndex) const;

    void StartDataTransferIntervalPeriod();
    void StartAllocatedAccessPeriod();
    void ContinueAllocatedAccessPeriod();
    void StartSendingInAllocatedAccessPeriod();
    void ScheduleEventForNextAllocatedAccessPeriodOrSuperframe();

    void AddExtendedScheduleElementsToFrame(Packet& aFrame) const;

    unsigned int GetNumberAntennaSectorsFor(const MacAddress& destinationAddress) const;
    unsigned int GetNumAntennaSectorsAtNode(const NodeId& theNodeId) const;

    SimTime CalculateCurrentAbftStartTime() const;

    unsigned int CalculateCurrentAbftSectorSweepSlot() const;

    void OutputTraceForNewDownlinkBeamformingSector(
        const MacAddress& staAddress,
        const unsigned int sectorId);

    void OutputTraceForNewUplinkBeamformingSector(
        const MacAddress& staAddress,
        const unsigned int sectorId);

    void OutputTraceForDmgFrameReceiveOnAp(const SectorSweepFrameType& sectorSweepFrame);

};//Dot11ApManagementController//



inline
unsigned int CalcExtendedScheduleElementsSizeBytes(
    const unsigned int numberDmgDataTransferIntervalPeriods)
{
    return (static_cast<unsigned int>(
        (sizeof(ExtendedScheduleElementHeaderType) +
         (numberDmgDataTransferIntervalPeriods * ExtendedScheduleElementAllocationType::actualSizeBytes))));
}


inline
void Dot11ApManagementController::AddExtendedScheduleElementsToFrame(Packet& aFrame) const
{
    // Building Frame back to front:

    const int numberDtiPeriods =
        static_cast<int>(schedulerPtr->GetNumberDataTransferIntervalPeriods());

    for(int i = (numberDtiPeriods - 1); (i >= 0); i--) {
        ExtendedScheduleElementAllocationType allocation;

        allocation.allocationStartNs =
            static_cast<uint32_t>(schedulerPtr->GetDtiPeriodRelativeStartTime(i) / NANO_SECOND);

        allocation.allocationDurationNs = static_cast<uint32_t>(
            schedulerPtr->GetDtiPeriodDuration(i)/ NANO_SECOND);

        allocation.sourceAssociationId = InvalidAssociationId;
        allocation.sourceNodeId = 0;
        allocation.destinationAssociationId = InvalidAssociationId;
        allocation.destinationNodeId = 0;
        allocation.allocationControl.isContentionBasedAllocationAkaCbap = 1;
        allocation.beamformingControl.doInitiatorRxssBeamformingTraining = 0;

        if (!schedulerPtr->DtiPeriodIsContentionBased(i)) {
            allocation.allocationControl.isContentionBasedAllocationAkaCbap = 0;

            const NodeId sourceNodeId = schedulerPtr->GetDtiPeriodSourceNodeId(i);

            if (sourceNodeId == theNodeId) {
                allocation.sourceAssociationId = AccessPointAssociationId;
                assert(theNodeId <= UCHAR_MAX);
                allocation.sourceNodeId = static_cast<unsigned char>(theNodeId);
            }
            else {
                bool success;
                MacAddress macAddress;
                LookupAssociatedNodeMacAddress(sourceNodeId, success, macAddress);

                if (success) {
                    allocation.sourceAssociationId =
                        associatedStaInformation.find(macAddress)->second->theAssociationId;
                    assert(sourceNodeId <= UCHAR_MAX);
                    allocation.sourceNodeId = static_cast<unsigned char>(sourceNodeId);
                }//if//

            }//if//

            const NodeId destinationNodeId = schedulerPtr->GetDtiPeriodDestinationNodeId(i);

            if (destinationNodeId == theNodeId) {
                allocation.destinationAssociationId = AccessPointAssociationId;
                assert(theNodeId <= UCHAR_MAX);
                allocation.destinationNodeId = static_cast<unsigned char>(theNodeId);
            }
            else {
                bool success;
                MacAddress macAddress;
                LookupAssociatedNodeMacAddress(destinationNodeId, success, macAddress);

                if (success) {
                    allocation.destinationAssociationId =
                        associatedStaInformation.find(macAddress)->second->theAssociationId;
                    assert(destinationNodeId <= UCHAR_MAX);
                    allocation.destinationNodeId = static_cast<unsigned char>(destinationNodeId);
                }//if//
            }//if//

            if (schedulerPtr->MustDoRxssBeamformingTrainingInDtiPeriod(i)) {
                allocation.beamformingControl.doInitiatorRxssBeamformingTraining = 1;
            }//if//

        }//if//

        aFrame.AddPlainStructHeaderWithTrailingAlignmentBytes(
            allocation, (sizeof(allocation) - ExtendedScheduleElementAllocationType::actualSizeBytes));

    }//for//

    ExtendedScheduleElementHeaderType elementHeader;

    const size_t lengthBytes = CalcExtendedScheduleElementsSizeBytes(numberDtiPeriods);

    assert((lengthBytes <= 255) && "Period Schedule is too long for current implementation (add code)");

    elementHeader.lengthBytes = static_cast<uint8_t>(lengthBytes);

    aFrame.AddPlainStructHeader(elementHeader);

}//AddExtendedScheduleElementsToFrame//




inline
void Dot11ApManagementController::ProcessAuthenticationFrame(const MacAddress& transmitterAddress)
{
    shared_ptr<SendAuthenticationEvent> sendAuthenticationEventPtr(new SendAuthenticationEvent(this, transmitterAddress));
    simEngineInterfacePtr->ScheduleEvent(
        sendAuthenticationEventPtr,
        simEngineInterfacePtr->CurrentTime() + authProcessingDelay);

}//ProcessAuthenticationFrame//


inline
bool Dot11ApManagementController::IsAnAssociatedStaAddress(const MacAddress& theMacAddress) const
{
    typedef map<MacAddress, unique_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

    IterType staIterator = associatedStaInformation.find(theMacAddress);

    if (staIterator == associatedStaInformation.end()) {
        return false;
    } else {
        return true;
    }//if//

}//IsAnAssociatedStaAddress//



inline
void Dot11ApManagementController::LookupAssociatedNodeMacAddress(
    const NodeId& staNodeId,
    bool& wasFound,
    MacAddress& macAddress) const
{
    // This code assumes that the AP will not associate with two interfaces on the same node.
    // To provide this functionality would require ARP/Neighbor Discovery.

    wasFound = false;
    MacAddress searchAddress(staNodeId, 0);

    typedef map<MacAddress, unique_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

    IterType staIterator = associatedStaInformation.lower_bound(searchAddress);

    if (staIterator == associatedStaInformation.end()) {
        return;
    }//if//

    if (staIterator->first.ExtractNodeId() == staNodeId) {
        wasFound = true;
        macAddress = staIterator->first;
    }//if//

}//LookupAssociatedNodeMacAddress//



inline
bool Dot11ApManagementController::StationIsAsleep(const MacAddress& staAddress) const
{
    typedef map<MacAddress, unique_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;
    IterType staIterator = associatedStaInformation.find(staAddress);
    if (staIterator == associatedStaInformation.end()) {
        return false;
    }//if//

    const AssociatedStaInformationEntry& staInfo = *(staIterator->second);
    return (staInfo.isInPowersaveMode);

}//StationIsAsleep//


inline
void Dot11ApManagementController::BufferPacketForSleepingStation(
    const MacAddress& staAddress,
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& destinationNetworkAddress,
    const PacketPriority& priority,
    const EtherTypeField etherType,
    const SimTime& queueInsertTime,
    const unsigned int retryTxCount)
{
    AssociatedStaInformationEntry& staInfo = *associatedStaInformation[staAddress];

    assert(staInfo.isInPowersaveMode);

    assert(retryTxCount == 0);  // TBD: Retry complicated by sleep functions.

    staInfo.powerSavePacketBuffer.push_back(move(
        PowerSavePacketBufferElem(
            packetPtr,
            destinationNetworkAddress,
            priority,
            etherType,
            0,
            queueInsertTime)));

}//BufferPacketForSleepingStation//



inline
void Dot11ApManagementController::BufferManagementFrameForSleepingStation(
    const MacAddress& staAddress,
    unique_ptr<Packet>& framePtr,
    const unsigned int retryTxCount,
    const SimTime& timestamp)
{
    AssociatedStaInformationEntry& staInfo = *associatedStaInformation[staAddress];

    assert(staInfo.isInPowersaveMode);
    staInfo.powerSavePacketBuffer.push_back(move(
        PowerSavePacketBufferElem(
            framePtr,
            NetworkAddress::invalidAddress,
            ScenSim::InvalidPacketPriority,
            ScenSim::ETHERTYPE_IS_NOT_SPECIFIED,
            retryTxCount,
            timestamp)));

}//BufferPacketForSleepingStation//


inline
void Dot11ApManagementController::GetAssociatedStaAddressList(vector<MacAddress>& associatedStaAddressList) const
{
    associatedStaAddressList.clear();

    typedef map<MacAddress, unique_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

    for(IterType iter = associatedStaInformation.begin(); iter != associatedStaInformation.end(); ++iter) {
        const MacAddress& associatedStaAddress = iter->first;
        associatedStaAddressList.push_back(associatedStaAddress);
    }//for//

}//GetAssociatedStaAddressList//



inline
void Dot11ApManagementController::GetPowerSaveBufferedPacket(
    const MacAddress& staAddress,
    bool& wasRetrieved,
    unique_ptr<Packet>& packetPtr,
    unsigned int& retryTxCount,
    PacketPriority& priority,
    EtherTypeField& etherType)
{
    AssociatedStaInformationEntry& staInfo = *associatedStaInformation[staAddress];
    PowerSavePacketBufferElem& elem = staInfo.powerSavePacketBuffer.front();

    packetPtr = move(elem.packetPtr);
    priority = elem.priority;
    etherType = elem.etherType;
    retryTxCount = elem.retryTxCount;
    staInfo.powerSavePacketBuffer.pop_front();

}//GetPowerSaveBufferedPacket//


inline
unsigned int Dot11ApManagementController::GetTxAntennaSectorId(const MacAddress& destinationAddress) const
{
    typedef map<MacAddress, StaSectorSweepInfoType>::const_iterator IterType;

    IterType iter = sectorSweepInfoForStation.find(destinationAddress);
    if (iter != sectorSweepInfoForStation.end()) {

        const StaSectorSweepInfoType& info = (iter->second);

        if (info.forcedTxBeamformingSectorId == InvalidSectorId) {

            assert(info.beamformingSectorSelectorPtr != nullptr);

            return (info.beamformingSectorSelectorPtr->GetBestTransmissionSectorId());
        }
        else {
            return (info.forcedTxBeamformingSectorId);
        }//if//
    }//if//

    return (QuasiOmniSectorId);

}//GetAntennaSectorId//


inline
unsigned int Dot11ApManagementController::GetAntennaSectorIdForNode(const NodeId& staNodeId) const
{
    MacAddress macAddress;
    bool success;
    LookupAssociatedNodeMacAddress(staNodeId, success, macAddress);

    if (!success) {
        return (QuasiOmniSectorId);
    }//if//

    return (GetTxAntennaSectorId(macAddress));

}//GetAntennaSectorIdForNode//


inline
unsigned int Dot11ApManagementController::GetNumberAntennaSectorsFor(
    const MacAddress& destinationAddress) const
{
    typedef map<MacAddress, StaSectorSweepInfoType>::const_iterator IterType;

    IterType iter = sectorSweepInfoForStation.find(destinationAddress);
    if (iter != sectorSweepInfoForStation.end()) {

        const StaSectorSweepInfoType& info = (iter->second);

        return (info.numberStaBeamformingSectors);
    }//if//

    return 0;

}//GetNumberAntennaSectorsFor//



inline
unsigned int Dot11ApManagementController::GetNumAntennaSectorsAtNode(const NodeId& staNodeId) const
{
    MacAddress macAddress;
    bool success;
    LookupAssociatedNodeMacAddress(staNodeId, success, macAddress);

    if (!success) {
        return 0;
    }//if//

    return (GetNumberAntennaSectorsFor(macAddress));

}//GetAntennaSectorIdForNode//


inline
void Dot11ApManagementController::GetNextAllocatedAccessPeriodIndex(
    const unsigned int startIndex,
    bool& wasFound,
    unsigned int& dtiPeriodIndex) const
{
    wasFound = false;
    dtiPeriodIndex = UINT_MAX;

    for(unsigned int i = startIndex; (i < schedulerPtr->GetNumberDataTransferIntervalPeriods()); i++) {

        const NodeId destinationNodeId = schedulerPtr->GetDtiPeriodDestinationNodeId(i);

        if ((schedulerPtr->DtiPeriodIsContentionBased(i)) ||
            (schedulerPtr->GetDtiPeriodSourceNodeId(i) == theNodeId) ||
            (destinationNodeId == theNodeId) || (destinationNodeId == BroadcastAssociationId)) {

            wasFound = true;
            dtiPeriodIndex = i;
            return;

        }//for//
    }//for//

}//GetAllocatedAccessPeriodIndex//


inline
void Dot11ApManagementController::ProcessDirectionalMgSuperframeScheduleEvent()
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    directionalMgSuperframeScheduleEventTicket.Clear();

    switch (nextDirectionalMgScheduleEvent) {

    case BeaconTransmissionIntervalPeriod:
        (*this).StartBeaconTransmissionIntervalPeriod();
        break;

    case AssociationBeamformingTrainingPeriod:
        (*this).StartAssociationBeamformingTrainingPeriod();
        break;

    case TransmitAssociationBeamformingTrainingFeedbackFrameEvent:
        (*this).ProcessAssociationBeamformingTrainingFeedback();
        break;

    case StartReceiveSectorSweep:
        (*this).ContinueReceivingReceiveSectorSweep(true);
        break;

    case StartReceivingNextSectorOfReceiveSectorSweep:
        (*this).ContinueReceivingReceiveSectorSweep(false);
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

    case SendNextSectorSweepFrame:
        (*this).TransmitReceiveSectorSweepFrame();
        break;

    default:
        assert(false); abort(); break;
    }//switch//

}//ProcessDirectionalMgSuperframeScheduleEvent//


}//namespace//

#endif
