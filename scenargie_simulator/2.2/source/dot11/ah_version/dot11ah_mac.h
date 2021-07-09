// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

//--------------------------------------------------------------------------------------------------
// "NotUsed" data items in header structs are placeholders for standard
// 802.11 fields that are not currently used in the model.  The purpose of
// not including the unused standard field names is to make plain the
// features that are and are NOT implemented.  The "Not Used" fields should always be
// zeroed so that packets do not include random garbage.  Likewise, only
// frame types and codes that are used in model logic will be defined.
//
// This code ignores machine endian issues because it is a model, i.e. fields are the
// correct sizes but the actual bits will not be in "network order" on small endian machines.
//

#ifndef DOT11AH_MAC_H
#define DOT11AH_MAC_H

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "scensim_prop.h"
#include "scensim_bercurves.h"
#include "scensim_support.h"
#include "scensim_netif.h"

#include "dot11ah_common.h"
#include "dot11ah_headers.h"
#include "dot11ah_tracedefs.h"
#include "dot11ah_phy.h"
#include "dot11ah_ratecontrol.h"
#include "dot11ah_txpowercontrol.h"
#include "dot11ah_mac_ap.h"
#include "dot11ah_mac_sta.h"
#include "dot11ah_incoming_buffer.h"

#include <queue>
#include <map>
#include <string>
#include <vector>
#include <iomanip>

namespace Dot11ah {

using std::queue;
using std::deque;
using std::map;
using std::unique_ptr;
using std::cout;
using std::hex;
using std::string;

using ScenSim::MILLI_SECOND;
using ScenSim::MICRO_SECOND;
using ScenSim::NetworkAddress;
using ScenSim::InterfaceOrInstanceId;
using ScenSim::NetworkLayer;
using ScenSim::OutputQueueWithPrioritySubqueuesOlderVer;
using ScenSim::PacketPriority;
using ScenSim::MacLayer;
using ScenSim::MacAddressResolver;
using ScenSim::NetworkInterfaceManager;
using ScenSim::GenericMacAddress;
using ScenSim::DeleteTrailingSpaces;
using ScenSim::ConvertStringToLowerCase;
using ScenSim::ConvertTimeToDoubleSecs;
using ScenSim::EtherTypeField;
using ScenSim::ETHERTYPE_IS_NOT_SPECIFIED;
using ScenSim::EnqueueResultType;
using ScenSim::ENQUEUE_SUCCESS;
using ScenSim::TraceMac;
using ScenSim::MacAndPhyInfoInterface;
using ScenSim::MakeLowerCaseString;
using ScenSim::ConvertAStringSequenceOfANumericTypeIntoAVector;
using ScenSim::RoundToUint;
using ScenSim::IncrementTwelveBitSequenceNumber;
using ScenSim::CalcTwelveBitSequenceNumberDifference;
using ScenSim::RoundUpToNearestIntDivisibleBy4;
using ScenSim::ConvertToUShortInt;
using ScenSim::ConvertToUChar;


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

inline
void AddMpduDelimiterAndPaddingToFrame(Packet& aFrame)
{
    aFrame.AddPlainStructHeader(MpduDelimiterFrame());
    MpduDelimiterFrame& mpduDelimiter =
        aFrame.GetAndReinterpretPayloadData<MpduDelimiterFrame>();

    mpduDelimiter.lengthBytes = aFrame.LengthBytes();

    aFrame.AddTrailingPadding(
        RoundUpToNearestIntDivisibleBy4(aFrame.LengthBytes()) - aFrame.LengthBytes());

}//AddMpduDelimiterAndPaddingToFrame//


inline
void RemoveMpduDelimiterAndPaddingFromFrame(Packet& aFrame)
{
    const MpduDelimiterFrame& mpduDelimiter =
        aFrame.GetAndReinterpretPayloadData<MpduDelimiterFrame>();

    aFrame.RemoveTrailingPadding(aFrame.LengthBytes() - mpduDelimiter.lengthBytes);

    aFrame.DeleteHeader(sizeof(MpduDelimiterFrame));

}//RemoveMpduDelimiterAndPaddingFromFrame//


inline
void RemoveMpduAggregationPaddingFromFrame(Packet& aFrame)
{
    const MpduDelimiterFrame& mpduDelimiter =
        aFrame.GetAndReinterpretPayloadData<MpduDelimiterFrame>();

    aFrame.RemoveTrailingPadding(aFrame.LengthBytes() - mpduDelimiter.lengthBytes);

}//RemoveMpduAggregationPaddingFromFrame//


class Dot11Mac;


// Only allows for one non-infrastructure interface to the same channel at a time.


class SimpleMacAddressResolver : public MacAddressResolver<MacAddress> {
public:
    SimpleMacAddressResolver(Dot11Mac* initMacPtr) : macPtr(initMacPtr) { }

    void GetMacAddress(
        const NetworkAddress& aNetworkAddress,
        const NetworkAddress& networkAddressMask,
        bool& wasFound,
        MacAddress& resolvedMacAddress);

    // Used only to get last hop address.

    void GetNetworkAddressIfAvailable(
        const MacAddress& macAddress,
        const NetworkAddress& subnetNetworkAddress,
        bool& wasFound,
        NetworkAddress& resolvedNetworkAddress);
private:
    Dot11Mac* macPtr;

};//SimpleMacAddressResolver//


//-------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


class Dot11MacAccessPointScheduler {
public:
    virtual ~Dot11MacAccessPointScheduler() { }

    // virtual void MonitorQueueingOfOutgoingPacket(const size_t accessCategoryIndex) = 0;

    // virtual void MonitorTransmissionOfFrame(
    //    const size_t accessCategory,
    //    const MacAddress& destinationAddress,
    //    const unsigned int numberFrameBytes,
    //    const DatarateBitsPerSec& theDatarateBitsPerSec) = 0;

    // virtual void MonitorIncomingFrame(
    //    const size_t accessCategory,
    //    const MacAddress& sourceAddress,
    //    const unsigned int numberFrameBytes,
    //    const DatarateBitsPerSec& theDatarateBitsPerSec) = 0;

    virtual SimTime CalculateTxopDuration(const size_t accessCategory) = 0;

};//Dot11MacAccessPointScheduler//


// Schedulers are normally in the 802.11 APs and TXOP durations are sent to STAs via
// beacon frames.  As a stopgap, fixed TXOPs can be set for individual STAs.

class FixedTxopScheduler: public Dot11MacAccessPointScheduler {
public:

    FixedTxopScheduler(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId theNodeId,
        const InterfaceId& theInterfaceId,
        const size_t numberAccessCategories);


    // Example constructor interface for dynamic scheduler:
    //
    // FixedTxopAccessPointScheduler(
    //     const ParameterDatabaseReader& theParameterDatabaseReader,
    //     const NodeId theNodeId,
    //     const InterfaceId& theInterfaceId,
    //     const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
    //     const shared_ptr<OutputQueueType> outputQueuePtr,
    //     const Dot11Mac* const macPtr,
    //     const SimTime& fixedTxopDuration);

    // virtual
    SimTime CalculateTxopDuration(const size_t accessCategoryIndex)
        { return txopDurations.at(accessCategoryIndex); }

private:

    vector<SimTime> txopDurations;

};//FixedTxopScheduler//


inline
FixedTxopScheduler::FixedTxopScheduler(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId theNodeId,
    const InterfaceId& theInterfaceId,
    const size_t numberAccessCategories)
{
    txopDurations.resize(numberAccessCategories, ZERO_TIME);

    for(size_t i = 0; (i < txopDurations.size()); i++) {
        ostringstream nameStream;
        nameStream << parameterNamePrefix << "edca-category-" << i << "-downlink-txop-duration";
        const string parmName = nameStream.str();

        if (theParameterDatabaseReader.ParameterExists(parmName, theNodeId, theInterfaceId)) {
            txopDurations[i] =
                theParameterDatabaseReader.ReadTime(parmName, theNodeId, theInterfaceId);
        }//if//
    }//for//

}//FixedTxopAccessPointScheduler//


//-------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

enum Dot11MacOperationMode {
    AdhocMode,
    ApMode,
    InfrastructureMode
};

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


class Dot11Mac : public MacLayer,
    public enable_shared_from_this<Dot11Mac> {
public:
    static const string modelName;

    static const unsigned char AdhocModeAddressSelectorByte = UCHAR_MAX;

    static const SimTime maxTransmitOpportunityAkaTxopDuration = 8160 * MICRO_SECOND;

    static const unsigned int transmitOpportunityNumberPacketNoLimitValue = 0;

    typedef Dot11Phy::PropFrame PropFrame;

    static shared_ptr<Dot11Mac> Create(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<PropFrame> >& propModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const RandomNumberGeneratorSeed& nodeSeed)
    {
        shared_ptr<Dot11Mac> macPtr(
            new Dot11Mac(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                propModelInterfacePtr,
                berCurveDatabasePtr,
                theNodeId,
                theInterfaceId,
                interfaceIndex,
                networkLayerPtr,
                nodeSeed));

        macPtr->CompleteInitialization(theParameterDatabaseReader, nodeSeed);
        return (macPtr);

    }//Create//


    ~Dot11Mac();

    void SetCustomAdaptiveRateController(
        const shared_ptr<AdaptiveRateController>& rateControllerPtr)
    {
        this->theAdaptiveRateControllerPtr = rateControllerPtr;
    }


    shared_ptr<MacAndPhyInfoInterface> GetMacAndPhyInfoInterface() const
        { return physicalLayer.GetDot11InfoInterface(); }

    // Network Layer Interface:

    virtual void NetworkLayerQueueChangeNotification() override;

    virtual void DisconnectFromOtherLayers() override {
        networkLayerPtr.reset();
        apControllerPtr.reset();
        staControllerPtr.reset();
        theMacAddressResolverPtr.reset();
    }

    virtual GenericMacAddress GetGenericMacAddress() const override {
        return (myMacAddress.ConvertToGenericMacAddress());
    }

    MacAddress GetMacAddress() const { return (myMacAddress); }

    Dot11MacOperationMode GetOperationMode() const;

    bool GetIpMulticastAddressToMacAddressMappingIsEnabled() const
        { return (ipMulticastAddressToMacAddressMappingIsEnabled); }

    void SendManagementFrame(unique_ptr<Packet>& framePtr);

    unsigned int GetNumberOfChannels() const { return (physicalLayer.GetChannelCount()); }
    unsigned int GetCurrentChannelId() const { return (physicalLayer.GetCurrentChannelNumber()); }

    const vector<unsigned int>& GetCurrentBondedChannelList()
    {
        return (physicalLayer.GetCurrentBondedChannelList());
    }

    double GetRssiOfLastFrameDbm() const { return (physicalLayer.GetRssiOfLastFrameDbm()); }

    void SendAssociationRequest(const MacAddress& apAddress);

    void SendReassociationRequest(
        const MacAddress& apAddress,
        const MacAddress& currentApAddress);

    void SendAssociationResponse(
        const MacAddress& staAddress,
        const AssociationId& theAssociationId);

    void SendReassociationResponse(
        const MacAddress& staAddress,
        const AssociationId& theAssociationId);

    void SendDisassociation(const MacAddress& receiverAddress);

    void StopReceivingFrames() { physicalLayer.StopReceivingFrames(); }
    void StartReceivingFrames() { physicalLayer.StartReceivingFrames(); }
    bool IsNotReceivingFrames() const { return (physicalLayer.IsNotReceivingFrames()); }

    void SendAuthentication(const MacAddress& receiverAddress);

    void SendPowerSaveNullFrame(
        const MacAddress& receiverAddress,
        const bool goingToPowerManagementMode);

    void SwitchToChannel(const unsigned int channelNumber);
    void SwitchToChannels(const vector<unsigned int>& channels);

    // For 802.11AC primary secondary channel scheme.

    void RequeueBufferedPackets();

    void RequeueBufferedPacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        const EtherTypeField etherType,
        const SimTime& timestamp,
        const unsigned int retryTxCount);

    void RequeueManagementFrame(unique_ptr<Packet>& framePtr);

    // For other Mac Layer subsystems:

    void SendLinkIsUpNotificationToNetworkLayer()
        { networkLayerPtr->ProcessLinkIsUpNotification(interfaceIndex); }

    void SendLinkIsDownNotificationToNetworkLayer()
        { networkLayerPtr->ProcessLinkIsDownNotification(interfaceIndex); }

    void SendNewLinkToANodeNotificationToNetworkLayer(const MacAddress& macAddress) {
        networkLayerPtr->ProcessNewLinkToANodeNotification(
            interfaceIndex, macAddress.ConvertToGenericMacAddress());
    }

    void LookupMacAddressForNeighbor(const NodeId targetNodeId, bool& wasFound, MacAddress& macAddress);

    bool MpduFrameAggregationIsEnabled() const { return (maxAggregateMpduSizeBytes > 0); }

    void SetMpduFrameAggregationIsEnabledFor(const MacAddress& destinationAddress);

    void SwitchToNormalAccessMode();
    void SwitchToReceiveOnlyMode();
    void SwitchToSleepMode();

    void SetRestrictedAccessWindowEndTimeAndRestartMac(const SimTime& anRawEndTime);
    void SaveNonRawBackoffInfo();
    void RestoreNonRawBackoffInfo();
    void SetAllContentionWindowsToMinAndRegenerateBackoff();
    void StartRestrictedAccessWindowPeriod(const SimTime& anRawEndTime);

private:

    Dot11Mac(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<PropFrame> >& propModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const RandomNumberGeneratorSeed& nodeSeed);

    void CompleteInitialization(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const RandomNumberGeneratorSeed& nodeSeed);

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    NodeId theNodeId;
    InterfaceId theInterfaceId;

    Dot11MacOperationMode operationMode;

    // Note only one "Management Controller" exists at one time
    shared_ptr<Dot11ApManagementController> apControllerPtr;
    shared_ptr<Dot11StaManagementController> staControllerPtr;

    shared_ptr<NetworkLayer> networkLayerPtr;
    unsigned int interfaceIndex;
    MacAddress myMacAddress;

    bool ipMulticastAddressToMacAddressMappingIsEnabled;
    std::set<MacAddress> myMulticastMacAddresses;

    shared_ptr<OutputQueueWithPrioritySubqueuesOlderVer> networkOutputQueuePtr;

    bool refreshNextHopOnDequeueModeIsOn;

    deque<unique_ptr<Packet> > managementFrameQueue;

    IncomingFrameBuffer theIncomingFrameBuffer;

    shared_ptr<AdaptiveRateController> theAdaptiveRateControllerPtr;
    shared_ptr<AdaptiveTxPowerController> theAdaptiveTxPowerControllerPtr;

    Dot11Phy physicalLayer;

    unique_ptr<MacAddressResolver<MacAddress> > theMacAddressResolverPtr;

    unique_ptr<Dot11MacAccessPointScheduler> macSchedulerPtr;

    //-----------------------------------------------------

    // Acronyms: AIFS,DIFS,EIFS,SIFS: {Arbitration, Distributed, Extended, Short} InterFrame Space

    // EDCA = "Enhanced Distributed Channel Access" for QoS (added in 802.11e)
    // DCF = "Distributed Coordination Function" (802.11 classic access protocol).

    // Mac Non-EDCA specific parameters:

    unsigned int rtsThresholdSizeBytes;

    unsigned int shortFrameRetryLimit;
    unsigned int longFrameRetryLimit;

    // "Short Interframe Space" = SIFS

    SimTime aShortInterframeSpaceTime;
    SimTime aSlotTime;

    // Cannot detect an incoming signal while switching to transmission mode.

    SimTime aRxTxTurnaroundTime;

    // Extra interframe delay is added if last frame was corrupted (and thus virtual carrier sense
    // is disabled).

    SimTime additionalDelayForExtendedInterframeSpaceMode;

    SimTime clearToSendTimeoutDuration;
    SimTime ackTimeoutDuration;

    int contentionWindowSlotsMin;
    int contentionWindowSlotsMax;

    enum AckDatarateSelectionType {
        SameAsData,
        Lowest
    };

    // To make this (counter-intuitive) behavior explicit.

    bool allowFrameAggregationWithTxopZero;

    bool protectAggregateFramesWithSingleAckedFrame;

    unsigned int maxAggregateMpduSizeBytes;

    AckDatarateSelectionType ackDatarateSelection;

    RandomNumberGenerator aRandomNumberGenerator;

    enum MacStateType {
        IdleState,
        BusyMediumState,
        CtsOrAckTransmissionState,
        WaitingForNavExpirationState,
        WaitingForIfsAndBackoffState,
        WaitingForCtsState,
        WaitingForAckState,
        ChangingChannelsState,
        TransientState,
    };

    MacStateType macState;

    enum class SentFrameType {
        ShortFrame,
        RequestToSendFrame,
        LongFrame,
        AggregateFrame,
        BlockAckRequestFrame,
        PowerSavePollResponse,
    };

    SentFrameType lastSentFrameWasAn;

    MacAddress lastSentDataFrameDestinationMacAddress;

    unsigned int powerSavePollResponsePacketTxCount;

    SimTime mediumReservedUntilTimeAkaNAV;

    bool lastFrameReceivedWasCorrupt;

    // Used by block ack processing.

    MacAddress currentIncomingAggregateFramesSourceMacAddress;
    PacketPriority currentIncomingAggregateFramesTrafficId;
    unsigned int numSubframesReceivedFromCurrentAggregateFrame;

    //-----------------------------------------------------

    struct EdcaAccessCategoryInfo {
        // Parameters
        vector<PacketPriority> listOfPriorities;
        unsigned int minContentionWindowSlots;
        unsigned int maxContentionWindowSlots;
        unsigned int arbitrationInterframeSpaceDurationSlots;
        SimTime transmitOpportunityDurationAkaTxop;
        SimTime frameLifetime;//dot11EDCATableMSDULifetime//

        // State variables
        unsigned int currentContentionWindowSlots;
        unsigned int currentNumOfBackoffSlots;

        // Used in 802.11ah Restricted Access Window (RAW)

        unsigned int ahSavedNonRawContentionWindowSlots;
        unsigned int ahSavedNonRawNumOfBackoffSlots;

        // Calculated backoff duration without dynamically added Extended Interframe space.
        SimTime currentNonExtendedBackoffDuration;

        bool tryingToJumpOnMediumWithoutABackoff;
        bool hasPacketToSend;

        unsigned int currentShortFrameRetryCount;
        unsigned int currentLongFrameRetryCount;
        unsigned int currentAggregateFrameRetryCount;

        SimTime ifsAndBackoffStartTime;

        // currentPacketPtr (non-aggregate) sent before aggregate frame.

        unique_ptr<Packet> currentPacketPtr;
        unique_ptr<vector<unique_ptr<Packet> > > currentAggregateFramePtr;
        bool currentAggregateFrameIsAMpduAggregate;

        bool currentPacketIsAManagementFrame;
        NetworkAddress currentPacketsNextHopNetworkAddress;
        MacAddress currentPacketsDestinationMacAddress;
        unsigned short int currentPacketSequenceNumber;
        PacketPriority currentPacketPriorityAkaTrafficId;
        EtherTypeField currentPacketsEtherType;
        SimTime currentPacketsTimestamp;

        SimTime timeLeftInCurrentTransmitOpportunity;

        EdcaAccessCategoryInfo()
            :
            currentNumOfBackoffSlots(0),
            ahSavedNonRawContentionWindowSlots(0),
            ahSavedNonRawNumOfBackoffSlots(0),
            currentNonExtendedBackoffDuration(INFINITE_TIME),
            currentShortFrameRetryCount(0),
            currentLongFrameRetryCount(0),
            currentAggregateFrameRetryCount(0),
            currentPacketsTimestamp(ZERO_TIME),
            hasPacketToSend(false),
            currentPacketSequenceNumber(0),
            currentPacketPriorityAkaTrafficId(0),
            currentPacketsEtherType(ETHERTYPE_IS_NOT_SPECIFIED),
            tryingToJumpOnMediumWithoutABackoff(false),
            frameLifetime(INFINITE_TIME),
            transmitOpportunityDurationAkaTxop(ZERO_TIME),
            timeLeftInCurrentTransmitOpportunity(ZERO_TIME),
            currentPacketIsAManagementFrame(false),
            ifsAndBackoffStartTime(INFINITE_TIME),
            currentAggregateFrameIsAMpduAggregate(true)
        {}

        void operator=(EdcaAccessCategoryInfo&& right)
        {
            assert(this != &right);

            listOfPriorities = right.listOfPriorities;
            minContentionWindowSlots = right.minContentionWindowSlots;
            maxContentionWindowSlots = right.maxContentionWindowSlots;
            arbitrationInterframeSpaceDurationSlots = right.arbitrationInterframeSpaceDurationSlots;
            transmitOpportunityDurationAkaTxop = right.transmitOpportunityDurationAkaTxop;
            frameLifetime = right.frameLifetime;
            currentNumOfBackoffSlots = right.currentNumOfBackoffSlots;
            ahSavedNonRawNumOfBackoffSlots = right.ahSavedNonRawNumOfBackoffSlots;
            currentContentionWindowSlots = right.currentContentionWindowSlots;
            ahSavedNonRawContentionWindowSlots = right.ahSavedNonRawContentionWindowSlots;
            currentNonExtendedBackoffDuration = right.currentNonExtendedBackoffDuration;
            tryingToJumpOnMediumWithoutABackoff = right.tryingToJumpOnMediumWithoutABackoff;
            hasPacketToSend = right.hasPacketToSend;
            currentShortFrameRetryCount = right.currentShortFrameRetryCount;
            currentLongFrameRetryCount = right.currentLongFrameRetryCount;
            currentAggregateFrameRetryCount = right.currentAggregateFrameRetryCount;
            ifsAndBackoffStartTime = right.ifsAndBackoffStartTime;
            currentPacketPtr = move(right.currentPacketPtr);
            currentAggregateFramePtr = move(right.currentAggregateFramePtr);
            currentAggregateFrameIsAMpduAggregate = right.currentAggregateFrameIsAMpduAggregate;
            currentPacketIsAManagementFrame = right.currentPacketIsAManagementFrame;
            currentPacketsNextHopNetworkAddress = right.currentPacketsNextHopNetworkAddress;
            currentPacketsDestinationMacAddress = right.currentPacketsDestinationMacAddress;
            currentPacketSequenceNumber = right.currentPacketSequenceNumber;
            currentPacketPriorityAkaTrafficId = right.currentPacketPriorityAkaTrafficId;
            currentPacketsEtherType = right.currentPacketsEtherType;
            currentPacketsTimestamp = right.currentPacketsTimestamp;
        }//=//

        EdcaAccessCategoryInfo(EdcaAccessCategoryInfo&& right) { (*this) = move(right); }

    };//EdcaAccessCatagoryInfo//


    //-----------------------------------------------------

    PacketPriority maxPacketPriority;

    vector<EdcaAccessCategoryInfo> accessCategories;

    unsigned int numberAccessCategories;

    unsigned int accessCategoryIndexForLastSentFrame;

    unsigned int accessCategoryIndexForManagementFrames;

    SimTime currentTransmitOpportunityAkaTxopStartTime;
    SimTime currentTransmitOpportunityAkaTxopEndTime;

    unsigned int currentTransmitOpportunityAckedFrameCount;

    vector<unsigned int> switchingToThisChannelList;

    bool disabledToJumpOnMediumWithoutBackoff;

    //-------------------------------------------------------------------------

    class WakeupTimerEvent : public SimulationEvent {
    public:
        WakeupTimerEvent(Dot11Mac* initMacPtr) : macPtr(initMacPtr) { }
        void ExecuteEvent() { macPtr->ProcessWakeupTimerEvent(); }
    private:
        Dot11Mac* macPtr;
    };


    shared_ptr<WakeupTimerEvent> wakeupTimerEventPtr;
    EventRescheduleTicket wakeupTimerEventTicket;
    SimTime currentWakeupTimerExpirationTime;

    //-------------------------------------------------------------------------

    SimTime mediumBecameIdleTime;

    struct AddressAndTrafficIdMapKey {
        MacAddress transmitterAddress;
        PacketPriority trafficId;

        AddressAndTrafficIdMapKey(
            const MacAddress& initTransmitterAddress, const PacketPriority& initTrafficId)
            :
            transmitterAddress(initTransmitterAddress), trafficId(initTrafficId) {}

        bool operator<(const AddressAndTrafficIdMapKey& right) const {
            return ((transmitterAddress < right.transmitterAddress) ||
                    ((transmitterAddress == right.transmitterAddress) && (trafficId < right.trafficId)));
        }
    };

    struct OutgoingLinkInfo {
        unsigned short int outgoingSequenceNumber;
        unsigned short int windowStartSequenceNumber;
        // Frames at and before this sequence number will not be resent.
        unsigned short int lastDroppedFrameSequenceNumber;
        bool blockAckSessionIsEnabled;
        bool blockAckRequestNeedsToBeSent;

        OutgoingLinkInfo() :
            outgoingSequenceNumber(0),
            lastDroppedFrameSequenceNumber(0),
            windowStartSequenceNumber(0),
            blockAckSessionIsEnabled(false),
            blockAckRequestNeedsToBeSent(false) {}
    };

    map<AddressAndTrafficIdMapKey, OutgoingLinkInfo> outgoingLinkInfoMap;

    struct NeighborCapabilities {
        bool mpduFrameAggregationIsEnabled;

        NeighborCapabilities() :
            mpduFrameAggregationIsEnabled(false)
        { }
    };

    map<MacAddress, NeighborCapabilities> neighborCapabilitiesMap;

    //-------------------------------------------------------------------------
    // 802.11ah Support
    //-------------------------------------------------------------------------

    bool ahUseOptimizedNdpControlFrames;

    bool lastTransmittedFrameWasABeacon;

    // Regular non-RAW access is modelled with the RAW end time being infinite.
    // RAW end time of zero means currently can't access medium.

    SimTime ahRestrictedAccessWindowEndTime;
    bool nonRawBackoffInformationHasBeenSaved;

    // Statistics:

    //dropped by exceeded retries
    shared_ptr<CounterStatistic> droppedPacketsStatPtr;

    //data
    shared_ptr<CounterStatistic> unicastDataFramesSentStatPtr;
    shared_ptr<CounterStatistic> broadcastDataFramesSentStatPtr;
    shared_ptr<CounterStatistic> unicastDataFramesResentStatPtr;
    shared_ptr<CounterStatistic> dataFramesReceivedStatPtr;
    shared_ptr<CounterStatistic> dataDuplicatedFramesReceivedStatPtr;
    shared_ptr<CounterStatistic> dataAggregateFramesSentStatPtr;
    shared_ptr<CounterStatistic> dataAggregateFramesResentStatPtr;
    shared_ptr<CounterStatistic> dataAggregatedSubframesReceivedStatPtr;

    //control
    shared_ptr<CounterStatistic> ackFramesSentStatPtr;
    shared_ptr<CounterStatistic> ackFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> blockAckRequestFramesSentStatPtr;
    shared_ptr<CounterStatistic> blockAckRequestFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> blockAckFramesSentStatPtr;
    shared_ptr<CounterStatistic> blockAckFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> rtsFramesSentStatPtr;
    shared_ptr<CounterStatistic> rtsFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> ctsFramesSentStatPtr;
    shared_ptr<CounterStatistic> ctsFramesReceivedStatPtr;

    //management
    shared_ptr<CounterStatistic> beaconFramesSentStatPtr;
    shared_ptr<CounterStatistic> beaconFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> associationRequestFramesSentStatPtr;
    shared_ptr<CounterStatistic> associationRequestFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> associationResponseFramesSentStatPtr;
    shared_ptr<CounterStatistic> associationResponseFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> reassociationRequestFramesSentStatPtr;
    shared_ptr<CounterStatistic> reassociationRequestFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> reassociationResponseFramesSentStatPtr;
    shared_ptr<CounterStatistic> reassociationResponseFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> disassociationFramesSentStatPtr;
    shared_ptr<CounterStatistic> disassociationFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> authenticationFramesSentStatPtr;
    shared_ptr<CounterStatistic> authenticationFramesReceivedStatPtr;

    // Parallelism Stuff:

    unsigned int lookaheadIndex;

    //-------------------------------------------------------------------------


    SimTime CalculateAdditionalDelayForExtendedInterframeSpace() const;

    void AddPrioritiesToAnAccessCategory(
        const string& prioritiesInAString,
        const unsigned int accessCategoryIndex);


    void InitializeAccessCategories(const ParameterDatabaseReader& theParameterDatabaseReader);

    void ReadBondedChannelList(
        const string& bondedChannelListString,
        const string& parameterNameForErrorOutput,
        vector<unsigned int>& bondedChannelList);

    void ReadInMulticastGroupNumberList(const ParameterDatabaseReader& theParameterDatabaseReader);

    void SetAccessCategoriesAsEdca(const ParameterDatabaseReader& theParameterDatabaseReader);
    void SetAccessCategoriesAsDcf(const ParameterDatabaseReader& theParameterDatabaseReader);

    bool WakeupTimerIsActive() const { return !wakeupTimerEventTicket.IsNull(); }

    void ScheduleWakeupTimer(const SimTime& wakeupTime);
    void CancelWakeupTimer();

    bool FrameIsForThisNode(const Packet& aFrame, const unsigned int headerOffset = 0) const;

    bool AggregatedSubframeIsForThisNode(const Packet& frame) const;

    // Note: returns accessCategories.size() when all backoffs are INFINITE_TIME.

    unsigned int FindIndexOfAccessCategoryWithShortestBackoff() const;
    unsigned int FindIndexOfAccessCategoryWithEarliestBackoffExpiration() const;

    bool AccessCategoryIsActive(const unsigned int accessCategoryIndex) const;

    bool ThereAreQueuedPacketsForAccessCategory(const unsigned int accessCategoryIndex) const;

    bool NetworkLayerHasPacketForAccessCategory(const unsigned int accessCategoryIndex) const;

    void RetrievePacketFromNetworkLayerForAccessCategory(
        const unsigned int accessCategoryIndex,
        bool& wasRetrieved);

    SimTime CurrentBackoffDuration() const;
    SimTime CurrentBackoffExpirationTime() const;

    void RecalcRandomBackoff(EdcaAccessCategoryInfo& accessCategoryInfo);

    void StartBackoffIfNecessary();
    void StartBackoffForCategory(const unsigned int accessCategoryIndex);

    void StartPacketSendProcessForAnAccessCategory(
        const unsigned int accessCategoryIndex,
        const bool forceBackoff = false);

    void PauseBackoffForAnAccessCategory(const unsigned int accessCategoryIndex, const SimTime& elapsedTime);

    void PerformInternalCollisionBackoff(const unsigned int accessCategoryIndex);

    void DoubleTheContentionWindowAndPickANewBackoff(const unsigned int accessCategoryIndex);

    SimTime CalculateFrameDuration(
        const unsigned int frameWithMacHeaderSizeBytes,
        const TransmissionParameters& txParameters) const;

    SimTime CalculateFrameDurationWithoutPhyHeader(
        const unsigned int frameWithMacHeaderSizeBytes,
        const TransmissionParameters& txParameters) const;

    SimTime CalculateControlFrameDuration(const unsigned int frameSizeBytes);

    SimTime CalculateNonExtendedBackoffDuration(const EdcaAccessCategoryInfo& accessCategory) const;
    SimTime CalculateNonExtendedDurationForJumpingOnMediumWithNoBackoff(const EdcaAccessCategoryInfo& accessCategory) const;

    int NumberBackoffSlotsUsedDuringElapsedTime(
        const EdcaAccessCategoryInfo& accessCategory,
        const SimTime& elapsedTime) const;

    void GoIntoWaitForExpirationOfVirtualCarrierSenseAkaNavState();

    SimTime CalculateNavDurationForRtsFrame(
        const MacAddress& destinationMacAddress,
        const unsigned int packetSizeBytes,
        const TransmissionParameters& txParameters) const;

    SimTime CalculateNavDurationForCtsFrame(
        const DurationField& durationFromRtsFrame,
        const MacAddress& destinationAddress) const;

    SimTime CalculateNavDurationForAckedDataFrame(
        const TransmissionParameters& txParameters) const;

    SimTime CalculateNavDurationForAckedAggregateDataFrame(
        const TransmissionParameters& txParameters) const;

    SimTime CalculateNextFrameSequenceDuration(const unsigned int accessCategoryIndex) const;

    void AddExtraNavDurationToPacketForNextFrameIfInATxop(
        const unsigned int accessCategoryIndex,
        const SimTime& frameTransmissionEndTime,
        Packet& thePacket) const;

    void CalculateAndSetTransmitOpportunityDurationAkaTxop(const size_t accessCategoryIndex);

    void TransmitAFrame(
        const unsigned int accessCategoryIndex,
        const bool doNotRequestToSendAkaRts,
        const SimTime& delayUntilTransmitting,
        bool& packetHasBeenSentToPhy);

    void TransmitAnAggregateFrame(
        const unsigned int accessCategoryIndex,
        const TransmissionParameters& txParameters,
        const double& transmitPowerDbm,
        const SimTime& delayUntilTransmitting);

    void TransmitABlockAckRequest(
        const MacAddress& destinationAddress,
        const PacketPriority& trafficId,
        const SimTime& delayUntilTransmitting);

    void DropCurrentPacketAndGoToNextPacket(const unsigned int accessCategoryIndex);
    void DropCurrentAggregateFrameAndGoToNextPacket(const unsigned int accessCategoryIndex);

    void NeverReceivedClearToSendOrAcknowledgementAction();

    void ProcessRequestToSendFrame(
        const RequestToSendFrame& rtsFrame,
        const TransmissionParameters& receivedFrameTxParameters);

    void ProcessClearToSendFrame(const ClearToSendFrame& ctsFrame);

    void ProcessDataFrame(
        const Packet& dataFrame,
        const TransmissionParameters& receivedFrameTxParameters);

    void ProcessManagementFrame(
        const Packet& managementFrame,
        const TransmissionParameters& receivedFrameTxParameters);

    void ProcessNullFrame(
        const Packet& nullFrame,
        const TransmissionParameters& receivedFrameTxParameters);

    void ProcessAcknowledgementFrame(const AcknowledgementAkaAckFrame& ackFrame);

    void ProcessBlockAckRequestFrame(
        const BlockAcknowledgementRequestFrame& blockAckRequestFrame,
        const TransmissionParameters& receivedFrameTxParameters);

    void ProcessBlockAckFrame(const BlockAcknowledgementFrame& blockAckFrame);

    unique_ptr<Packet> CreateABlockAckRequestFrame(
        const MacAddress& destinationAddress,
        const PacketPriority& trafficId,
        const unsigned short int startFrameSequenceNumber) const;

    void ProcessPowerSavePollFrame(
        const PowerSavePollFrame& psPollFrame,
        const TransmissionParameters& pollFrameTxParameters);

    void SendAckFrame(
        const MacAddress& destinationAddress,
        const TransmissionParameters& receivedFrameTxParameters);

    void ProcessWakeupTimerEvent();
    void ProcessVirtualCarrierSenseAkaNavExpiration();
    void ProcessInterframeSpaceAndBackoffTimeout();
    void ProcessClearToSendOrAcknowledgementTimeout();

    void GetNewSequenceNumber(
        const MacAddress& destinationAddress,
        const PacketPriority& priority,
        const bool isNonBlockAckedFrame,
        unsigned short int& newSequenceNumber);

    unsigned int CalcNumberFramesLeftInSequenceNumWindow(
        const MacAddress& destinationAddress,
        const PacketPriority& priority) const;

    void SetSequenceNumberWindowStart(
        const MacAddress& destinationAddress,
        const PacketPriority& priority,
        const unsigned short int sequenceNum);

    bool BlockAckSessionIsEnabled(
        const MacAddress& destinationAddress,
        const PacketPriority& priority) const;

    void CheckPriorityToAccessCategoryMapping() const;

    void QueueOutgoingEthernetPacket(unique_ptr<Packet>& ethernetPacketPtr);
    void QueueOutgoingPacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority);

    // Phy interface routines

    void BusyChannelAtPhysicalLayerNotification();
    void ClearChannelAtPhysicalLayerNotification();
    void TransmissionIsCompleteNotification();
    void DoSuccessfulTransmissionPostProcessing(const bool wasJustTransmitting);

    void ReceiveFrameFromPhy(
        const Packet& aFrame,
        const TransmissionParameters& receivedFrameTxParameters);

    void ReceiveAggregatedSubframeFromPhy(
        unique_ptr<Packet>& subframePtr,
        const TransmissionParameters& receivedFrameTxParameters,
        const unsigned int aggregateFrameSubframeIndex,
        const unsigned int numberSubframes);

    void NotifyThatPhyReceivedCorruptedFrame();

    void NotifyThatPhyReceivedCorruptedAggregatedSubframe(
        const TransmissionParameters& receivedFrameTxParameters,
        const unsigned int aggregateFrameSubframeIndex,
        const unsigned int numberSubframes);

    class InterfaceForPhy : public Dot11MacInterfaceForPhy {
    public:
        InterfaceForPhy(Dot11Mac* initMacLayerPtr) : macLayerPtr(initMacLayerPtr) { }

        virtual void BusyChannelAtPhysicalLayerNotification() override
            { macLayerPtr->BusyChannelAtPhysicalLayerNotification(); }

        virtual void ClearChannelAtPhysicalLayerNotification() override
            { macLayerPtr->ClearChannelAtPhysicalLayerNotification(); }

        virtual void TransmissionIsCompleteNotification() override
            { macLayerPtr->TransmissionIsCompleteNotification(); }

        virtual void DoSuccessfulTransmissionPostProcessing(const bool wasJustTransmitting) override
            { macLayerPtr->DoSuccessfulTransmissionPostProcessing(wasJustTransmitting); }

        virtual void ReceiveFrameFromPhy(
            const Packet& aFrame,
            const TransmissionParameters& receivedFrameTxParameters) override
        {
            macLayerPtr->ReceiveFrameFromPhy(aFrame, receivedFrameTxParameters);
        }

        virtual void ReceiveAggregatedSubframeFromPhy(
            unique_ptr<Packet>& subframePtr,
            const TransmissionParameters& receivedFrameTxParameters,
            const unsigned int aggregateFrameSubframeIndex,
            const unsigned int numberSubframes) override
        {
            macLayerPtr->ReceiveAggregatedSubframeFromPhy(
                subframePtr, receivedFrameTxParameters, aggregateFrameSubframeIndex, numberSubframes);
        }


        virtual void NotifyThatPhyReceivedCorruptedFrame() override
            { macLayerPtr->NotifyThatPhyReceivedCorruptedFrame(); }


        virtual void NotifyThatPhyReceivedCorruptedAggregatedSubframe(
            const TransmissionParameters& receivedFrameTxParameters,
            const unsigned int aggregateFrameSubframeIndex,
            const unsigned int numberSubframes) override
        {
            macLayerPtr->NotifyThatPhyReceivedCorruptedAggregatedSubframe(
                receivedFrameTxParameters, aggregateFrameSubframeIndex, numberSubframes);
        }

        bool AggregatedSubframeIsForThisNode(const Packet& frame) const override
            { return (macLayerPtr->AggregatedSubframeIsForThisNode(frame)); }

    private:
        Dot11Mac* macLayerPtr;

    };//InterfaceForPhy//

    bool FrameAggregationIsEnabledFor(const MacAddress& destinationAddress) const;
    bool MpduFrameAggregationIsEnabledFor(const MacAddress& destinationAddress) const;

    bool NeedToSendABlockAckRequest(
        const MacAddress& destinationAddress,
        const PacketPriority& trafficId) const;

    void BuildAggregateFrameFromCurrentFrame(const unsigned int accessCategoryIndex);

    void SendPacketToNetworkLayer(unique_ptr<Packet>& dataFramePtr);

    void SendBlockAcknowledgementFrame(
        const MacAddress& destinationAddress,
        const PacketPriority& trafficId,
        const TransmissionParameters& receivedFrameTxParameters);


    // 802.11ah:

    // Note:  Regular non-RAW access is modelled as an infinite duration RAW period.

    bool AhScheduleSaysCanTransmitNow() const;
    void DoTransmissionTooLongForRawPeriodAction(const unsigned int accessCategoryIndex);

    //For trace output

    void OutputTraceForClearChannel() const;
    void OutputTraceForBusyChannel() const;
    void OutputTraceForNavStart(const SimTime& expirationTime) const;
    void OutputTraceForNavExpiration() const;

    void OutputTraceForIfsAndBackoffStart(const SimTime& expirationTime) const;
    void OutputTraceForInterruptedIfsAndBackoff(
        const unsigned int accessCategoryIndex,
        const SimTime nonExtendedDurationLeft) const;
    void OutputTraceForIfsAndBackoffExpiration() const;

    void OutputTraceAndStatsForFrameReceive(const Packet& aFrame) const;

    void OutputTraceAndStatsForAggregateSubframeReceive(
        const Packet& aFrame,
        const QosDataFrameHeader& dataFrameHeader);

    void OutputTraceForPacketDequeue(const unsigned int accessCategoryIndex,  const SimTime delayUntilAirBorne = 0) const;

    void OutputTraceAndStatsForRtsFrameTransmission(const unsigned int accessCategoryIndex) const;
    void OutputTraceForCtsFrameTransmission() const;
    void OutputTraceAndStatsForAggregatedFrameTransmission(const unsigned int accessCategoryIndex) const;
    void OutputTraceAndStatsForBroadcastDataFrameTransmission(const unsigned int accessCategoryIndex) const;
    void OutputTraceAndStatsForUnicastDataFrameTransmission(const unsigned int accessCategoryIndex) const;
    void OutputTraceAndStatsForManagementFrameTransmission(const unsigned int accessCategoryIndex) const;
    void OutputTraceForAckFrameTransmission() const;
    void OutputTraceAndStatsForBlockAckRequestFrameTransmission() const;
    void OutputTraceAndStatsForBlockAckFrameTransmission() const;

    void OutputTraceForCtsOrAckTimeout() const;

    void OutputTraceAndStatsForPacketRetriesExceeded(const unsigned int accessCategoryIndex) const;
    bool redundantTraceInformationModeIsOn;

};//Dot11Mac//


const unsigned int DefaultRtsThresholdSizeBytes = UINT_MAX;
const unsigned int DefaultShortFrameRetryLimit = 7;
const unsigned int DefaultLongFrameRetryLimit = 4;
const unsigned int DefaultContentionWindowSlotsMin = 15;
const unsigned int DefaultContentionWindowSlotsMax = 1023;
const PacketPriority DefaultMaxPacketPriority = 3;
const unsigned int MaxNumberAccessCategories = 4;

inline
SimTime Dot11Mac::CalculateAdditionalDelayForExtendedInterframeSpace() const
{
    TransmissionParameters txParameters;
    txParameters.channelBandwidthMhz = physicalLayer.GetBaseChannelBandwidthMhz();

    txParameters.modulationAndCodingScheme =
        theAdaptiveRateControllerPtr->GetLowestModulationAndCoding();

    SimTime longestPossibleAckDuration;

    if (!ahUseOptimizedNdpControlFrames) {
        longestPossibleAckDuration =
            CalculateFrameDuration(sizeof(AcknowledgementAkaAckFrame), txParameters);
    }
    else {
        // NDP Control frames puts (replaces) MAC frame data in the PHY Header "SIG" field.

        longestPossibleAckDuration = CalculateFrameDuration(0, txParameters);

    }//if//

    return (aShortInterframeSpaceTime + longestPossibleAckDuration);

}//CalculateAdditionalDelayForExtendedInterframeSpace//



inline
void Dot11Mac::AddPrioritiesToAnAccessCategory(
    const string& prioritiesInAString,
    const unsigned int accessCategoryIndex)
{
    bool success;
    vector<PacketPriority> priorityVector;

    ConvertAStringSequenceOfANumericTypeIntoAVector<PacketPriority>(
        prioritiesInAString,
        success,
        priorityVector);

    if (!success) {
        cerr << "Error: Configuration: Bad Priority sequence: " << prioritiesInAString << endl;
        exit(1);
    }//if//

    EdcaAccessCategoryInfo& accessCategory = accessCategories.at(accessCategoryIndex);

    accessCategory.listOfPriorities.clear();

    for(unsigned int i = 0; (i < priorityVector.size()); i++) {
        accessCategory.listOfPriorities.push_back(priorityVector[i]);
    }//for//

}//AddPrioritiesToAnAccessCategory//

inline
void Dot11Mac::CheckPriorityToAccessCategoryMapping() const
{
    for(PacketPriority priority = 0; (priority <= maxPacketPriority); priority++) {
        bool found = false;
        for(unsigned int cat = 0; (cat < numberAccessCategories); cat++) {
            for(unsigned int i = 0; (i < accessCategories[cat].listOfPriorities.size()); i++) {
                if (priority == accessCategories[cat].listOfPriorities[i]) {
                    if (found) {
                        cerr << "Error: Duplicate access category to priority mapping for "
                             << "priority = " << priority << endl;
                        exit(1);
                    }//if//
                    found = true;
                }//if//
                if (accessCategories[cat].listOfPriorities[i] > maxPacketPriority) {
                    cerr << "Error: Unexpected priority mapping for "
                         << "priority = " << int(accessCategories[cat].listOfPriorities[i])
                         << ", max priority = " << static_cast<int>(maxPacketPriority) << endl;
                    exit(1);
                }//if//
            }//for//
        }//for//
        if (!found) {
            cerr << "Error: No access category to priority mapping for priority = "
                 << priority << endl;
            exit(1);
        }//if//
    }//for//

}//CheckPriorityToAccessCategoryMapping//




inline
void Dot11Mac::InitializeAccessCategories(const ParameterDatabaseReader& theParameterDatabaseReader)
{
    string qosTypeString = "edca";

    if (theParameterDatabaseReader.ParameterExists(
            "dot11-qos-type", theNodeId, theInterfaceId)) {

        qosTypeString =  MakeLowerCaseString(
            theParameterDatabaseReader.ReadString("dot11-qos-type", theNodeId, theInterfaceId));

    }//if//

    if (qosTypeString == "edca") {
        (*this).SetAccessCategoriesAsEdca(theParameterDatabaseReader);
    }
    else if (qosTypeString == "dcf") {
        (*this).SetAccessCategoriesAsDcf(theParameterDatabaseReader);
    }
    else {
        cerr << "Error: Unknown dot11-qos-type: " << qosTypeString << endl;
        exit(1);
    }//if//

    CheckPriorityToAccessCategoryMapping();

    for(unsigned int i = 0; (i < accessCategories.size()); i++) {
        accessCategories[i].currentContentionWindowSlots = accessCategories[i].minContentionWindowSlots;
    }//for//

    accessCategoryIndexForManagementFrames = static_cast<unsigned int>(accessCategories.size() - 1);

}//InitializeAccessCategories//


inline
void Dot11Mac::SetAccessCategoriesAsEdca(const ParameterDatabaseReader& theParameterDatabaseReader)
{
    // Set defaults for WAVE.  Maybe overidden.

    accessCategories.clear();
    accessCategories.resize(4);

    accessCategories.at(0).minContentionWindowSlots = contentionWindowSlotsMin;
    accessCategories.at(0).maxContentionWindowSlots = contentionWindowSlotsMax;
    accessCategories.at(0).arbitrationInterframeSpaceDurationSlots = 9;

    accessCategories.at(1).minContentionWindowSlots = (contentionWindowSlotsMin+1)/2 - 1;
    accessCategories.at(1).maxContentionWindowSlots = contentionWindowSlotsMax;
    accessCategories.at(1).arbitrationInterframeSpaceDurationSlots = 6;

    accessCategories.at(2).minContentionWindowSlots = (contentionWindowSlotsMin+1)/4 - 1;
    accessCategories.at(2).maxContentionWindowSlots = (contentionWindowSlotsMin+1)/2 - 1;
    accessCategories.at(2).arbitrationInterframeSpaceDurationSlots = 3;

    accessCategories.at(3).minContentionWindowSlots = (contentionWindowSlotsMin+1)/4 - 1;
    accessCategories.at(3).maxContentionWindowSlots = (contentionWindowSlotsMin+1)/2 - 1;
    accessCategories.at(3).arbitrationInterframeSpaceDurationSlots = 2;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "num-edca-access-categories"), theNodeId, theInterfaceId)) {

        numberAccessCategories =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "num-edca-access-categories"),
                theNodeId,
                theInterfaceId);

        if (numberAccessCategories > MaxNumberAccessCategories) {
            cerr << "Error: the number of 802.11 access categories is greater than: "
                 << MaxNumberAccessCategories << endl;
            exit(1);
        }//if//

        accessCategories.resize(numberAccessCategories);

    }//if//

    assert(maxPacketPriority == 3);

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "max-packet-priority"), theNodeId, theInterfaceId)) {

        unsigned int readMaxPacketPriority =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "max-packet-priority"),
                theNodeId,
                theInterfaceId);

        if (readMaxPacketPriority > 7) {
            cerr << "Error in parameter \"" << (parameterNamePrefix + "max-packet-priority")
                 << "\",  maximum 802.11 priority is 7." << endl;
            exit(1);
        }//if//

        maxPacketPriority = static_cast<PacketPriority>(readMaxPacketPriority);

    }//if//


    // Default simple priority mapping: Distribute priorities across categories,
    // excess priorities in the front (low priority access categories).

    const unsigned int minPriPerCategory = (maxPacketPriority+1) / numberAccessCategories;

    assert(minPriPerCategory >= 1);

    const unsigned int excessPriorities = (maxPacketPriority+1) % numberAccessCategories;

    PacketPriority priority = 0;
    for(unsigned int i = 0; (i < numberAccessCategories); i++) {
        unsigned int numberPriorities = minPriPerCategory;
        if (i < excessPriorities) {
            numberPriorities++;
        }//if//

        for(unsigned int j = 0; (j < numberPriorities); j++) {
            accessCategories[i].listOfPriorities.push_back(priority);
            priority++;
        }//for//
    }//for//


    // Override defaults if parameters exist.

    for(unsigned int i = 0; (i < accessCategories.size()); i++) {
        ostringstream prefixStream;
        prefixStream << parameterNamePrefix << "edca-category-" << i << '-';
        string prefix = prefixStream.str();

        if (theParameterDatabaseReader.ParameterExists((prefix+"num-aifs-slots"), theNodeId, theInterfaceId)) {
            accessCategories[i].arbitrationInterframeSpaceDurationSlots =
                theParameterDatabaseReader.ReadNonNegativeInt(
                    (prefix+"num-aifs-slots"), theNodeId, theInterfaceId);
        }//if//

        if (theParameterDatabaseReader.ParameterExists((prefix+"contention-window-min-slots"), theNodeId, theInterfaceId)) {
            accessCategories[i].minContentionWindowSlots =
                theParameterDatabaseReader.ReadNonNegativeInt(
                    (prefix+"contention-window-min-slots"), theNodeId, theInterfaceId);
        }//if//

        if (theParameterDatabaseReader.ParameterExists((prefix+"contention-window-max-slots"), theNodeId, theInterfaceId)) {
            accessCategories[i].maxContentionWindowSlots =
                theParameterDatabaseReader.ReadNonNegativeInt(
                    (prefix+"contention-window-max-slots"), theNodeId, theInterfaceId);
        }//if//


        if (theParameterDatabaseReader.ParameterExists((prefix+"priority-list"), theNodeId, theInterfaceId)) {
            string prioritiesInAString =
                theParameterDatabaseReader.ReadString((prefix+"priority-list"), theNodeId, theInterfaceId);
            DeleteTrailingSpaces(prioritiesInAString);
            (*this).AddPrioritiesToAnAccessCategory(prioritiesInAString, i);
        }//if//

        if (theParameterDatabaseReader.ParameterExists((prefix+"frame-lifetime"), theNodeId, theInterfaceId)) {
            accessCategories[i].frameLifetime =
                theParameterDatabaseReader.ReadTime((prefix+"frame-lifetime"), theNodeId, theInterfaceId);
        }//if//

    }//for//

}//SetAccessCategoriesAsEdca//


inline
void Dot11Mac::SetAccessCategoriesAsDcf(const ParameterDatabaseReader& theParameterDatabaseReader)
{
    //Set defaults for DCF. Maybe overidden.

    accessCategories.clear();
    accessCategories.resize(1);

    EdcaAccessCategoryInfo& accessCategoryForDcf = accessCategories.at(0);
    accessCategoryForDcf.minContentionWindowSlots = contentionWindowSlotsMin;
    accessCategoryForDcf.maxContentionWindowSlots = contentionWindowSlotsMax;
    accessCategoryForDcf.arbitrationInterframeSpaceDurationSlots = 2;

    numberAccessCategories = 1;
    maxPacketPriority = 0;

    accessCategoryForDcf.listOfPriorities.push_back(0);

    // Override defaults if parameters exist.

    if (theParameterDatabaseReader.ParameterExists("dot11-dcf-num-difs-slots", theNodeId, theInterfaceId)) {
        accessCategoryForDcf.arbitrationInterframeSpaceDurationSlots =
            theParameterDatabaseReader.ReadNonNegativeInt(
                "dot11-dcf-num-difs-slots", theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("dot11-dcf-contention-window-min-slots", theNodeId, theInterfaceId)) {
        accessCategoryForDcf.minContentionWindowSlots  =
            theParameterDatabaseReader.ReadNonNegativeInt(
                "dot11-dcf-contention-window-min-slots", theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("dot11-dcf-contention-window-max-slots", theNodeId, theInterfaceId)) {
        accessCategoryForDcf.maxContentionWindowSlots =
            theParameterDatabaseReader.ReadNonNegativeInt(
                "dot11-dcf-contention-window-max-slots", theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("dot11-dcf-frame-lifetime", theNodeId, theInterfaceId)) {
        accessCategoryForDcf.frameLifetime =
            theParameterDatabaseReader.ReadTime("dot11-dcf-frame-lifetime", theNodeId, theInterfaceId);
    }//if//

}//SetAccessCategoriesAsDcf//


inline
void Dot11Mac::ReadBondedChannelList(
    const string& bondedChannelListString,
    const string& parameterNameForErrorOutput,
    vector<unsigned int>& bondedChannelList)
{
    bool success;
    ConvertAStringSequenceOfANumericTypeIntoAVector<unsigned int>(
        bondedChannelListString,
        success,
        bondedChannelList);

    if ((!success) || (bondedChannelList.empty())) {
        cerr << "Error in configuration parameter: " << parameterNameForErrorOutput << ":" << endl;
        cerr << "     " << bondedChannelListString << endl;
        exit(1);
    }//if//

    for(unsigned int i = 0; (i < bondedChannelList.size()); i++) {
        if (bondedChannelList[i] >= GetNumberOfChannels()) {
            cerr << "Error: invalid channel number: " << bondedChannelList[i] << endl;
            exit(1);
        }//if//
    }//for//

}//InitBondedChannel//



inline
void Dot11Mac::ReadInMulticastGroupNumberList(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    (*this).ipMulticastAddressToMacAddressMappingIsEnabled = false;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "map-ip-multicast-addresses"), theNodeId, theInterfaceId)) {

        (*this).ipMulticastAddressToMacAddressMappingIsEnabled =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "map-ip-multicast-addresses"), theNodeId, theInterfaceId);
    }//if//

    const string parameterName = parameterNamePrefix + "multicast-group-number-list";

    if (theParameterDatabaseReader.ParameterExists(parameterName, theNodeId, theInterfaceId)) {

        if (!ipMulticastAddressToMacAddressMappingIsEnabled) {
            cerr << "Error: MAC multicast is used but "
                 << (parameterNamePrefix + "map-ip-multicast-addresses") << " not enabled." << endl;
            exit(1);
        }//if//

        string multicastGroupNumberListString =
            theParameterDatabaseReader.ReadString(parameterName, theNodeId, theInterfaceId);

        DeleteTrailingSpaces(multicastGroupNumberListString);

        istringstream listStream(multicastGroupNumberListString);

        while (!listStream.eof()) {
            unsigned int multicastGroupNumber;

            listStream >> multicastGroupNumber;

            if (listStream.fail()) {
                cerr << "Error reading " << parameterName << " parameter:" << endl;
                cerr << "   " << multicastGroupNumberListString << endl;
                exit(1);
            }//if//

            if (multicastGroupNumber > NetworkAddress::maxMulticastGroupNumber) {
                cerr << "Error in " << parameterName << ": group number too large:" << endl;
                cerr << "     " << multicastGroupNumberListString << endl;
                exit(1);
            }//if//

            MacAddress multicastMacAddress;
            multicastMacAddress.SetToAMulticastAddress(multicastGroupNumber);

            (*this).myMulticastMacAddresses.insert(multicastMacAddress);

        }//while//
    }//if//

}//ReadInMulticastGroupNumberList//


#pragma warning(disable:4355)


inline
Dot11Mac::Dot11Mac(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const shared_ptr<SimplePropagationModelForNode<PropFrame> >& propModelInterfacePtr,
    const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
    const NodeId& initNodeId,
    const InterfaceId& initInterfaceId,
    const unsigned int initInterfaceIndex,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const RandomNumberGeneratorSeed& nodeSeed)
    :
    simEngineInterfacePtr(simulationEngineInterfacePtr),
    theNodeId(initNodeId),
    theInterfaceId(initInterfaceId),
    operationMode(AdhocMode),
    networkLayerPtr(initNetworkLayerPtr),
    refreshNextHopOnDequeueModeIsOn(false),
    interfaceIndex(initInterfaceIndex),
    rtsThresholdSizeBytes(DefaultRtsThresholdSizeBytes),
    shortFrameRetryLimit(DefaultShortFrameRetryLimit),
    longFrameRetryLimit(DefaultLongFrameRetryLimit),
    contentionWindowSlotsMin(DefaultContentionWindowSlotsMin),
    contentionWindowSlotsMax(DefaultContentionWindowSlotsMax),
    aShortInterframeSpaceTime(INFINITE_TIME),
    aSlotTime(INFINITE_TIME),
    aRxTxTurnaroundTime(INFINITE_TIME),
    additionalDelayForExtendedInterframeSpaceMode(INFINITE_TIME),
    maxPacketPriority(DefaultMaxPacketPriority),
    numberAccessCategories(MaxNumberAccessCategories),
    ackDatarateSelection(SameAsData),
    ahUseOptimizedNdpControlFrames(false),
    macState(IdleState),
    mediumReservedUntilTimeAkaNAV(ZERO_TIME),
    mediumBecameIdleTime(ZERO_TIME),
    currentWakeupTimerExpirationTime(INFINITE_TIME),
    lastFrameReceivedWasCorrupt(false),
    lastTransmittedFrameWasABeacon(false),
    numSubframesReceivedFromCurrentAggregateFrame(0),
    accessCategoryIndexForLastSentFrame(0),
    currentTransmitOpportunityAkaTxopStartTime(ZERO_TIME),
    currentTransmitOpportunityAkaTxopEndTime(ZERO_TIME),
    currentTransmitOpportunityAckedFrameCount(0),
    redundantTraceInformationModeIsOn(false),
    disabledToJumpOnMediumWithoutBackoff(false),
    protectAggregateFramesWithSingleAckedFrame(true),
    allowFrameAggregationWithTxopZero(false),
    maxAggregateMpduSizeBytes(0),
    ipMulticastAddressToMacAddressMappingIsEnabled(false),
    nonRawBackoffInformationHasBeenSaved(false),
    ahRestrictedAccessWindowEndTime(INFINITE_TIME),
    physicalLayer(
        theParameterDatabaseReader,
        initNodeId,
        initInterfaceId,
        simulationEngineInterfacePtr,
        propModelInterfacePtr,
        berCurveDatabasePtr,
        shared_ptr<InterfaceForPhy>(new InterfaceForPhy(this)),
        nodeSeed),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(nodeSeed, initInterfaceIndex)),
    droppedPacketsStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_FramesDropped"))),
    unicastDataFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_Data_UnicastFramesSent"))),
    unicastDataFramesResentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_Data_UnicastFramesResent"))),
    broadcastDataFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_Data_BroadcastFramesSent"))),
    dataFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_Data_FramesReceived"))),
    dataAggregateFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_Data_AggregateFramesSent"))),
    dataAggregateFramesResentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_Data_AggregateFramesResent"))),
    dataAggregatedSubframesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_Data_AggregatedSubframesReceived"))),
    dataDuplicatedFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_Data_DuplicatedFramesReceived"))),
    ackFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_ACK_FramesSent"))),
    ackFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_ACK_FramesReceived"))),
    blockAckRequestFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_BAR_FramesSent"))),
    blockAckRequestFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_BAR_FramesReceived"))),
    blockAckFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_BlockACK_FramesSent"))),
    blockAckFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_BlockACK_FramesReceived"))),
    rtsFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_RTS_FramesSent"))),
    rtsFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_RTS_FramesReceived"))),
    ctsFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_CTS_FramesSent"))),
    ctsFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_CTS_FramesReceived"))),
    beaconFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_Beacon_FramesSent"))),
    beaconFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_Beacon_FramesReceived"))),
    associationRequestFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_AssociationRequest_FramesSent"))),
    associationRequestFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_AssociationRequest_FramesReceived"))),
    associationResponseFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_AssociationResponse_FramesSent"))),
    associationResponseFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_AssociationResponse_FramesReceived"))),
    reassociationRequestFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_ReassociationRequest_FramesSent"))),
    reassociationRequestFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_ReassociationRequest_FramesReceived"))),
    reassociationResponseFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_ReassociationResponse_FramesSent"))),
    reassociationResponseFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_ReassociationResponse_FramesReceived"))),
    disassociationFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_Disassociation_FramesSent"))),
    disassociationFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_Disassociation_FramesReceived"))),
    authenticationFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_Authentication_FramesSent"))),
    authenticationFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_Authentication_FramesReceived")))
{
    shared_ptr<NetworkInterfaceManager> networkInterfaceManagerPtr =
        networkLayerPtr->GetNetworkInterfaceManagerPtr(interfaceIndex);

    if (networkInterfaceManagerPtr != nullptr) {
        theMacAddressResolverPtr.reset(
            networkInterfaceManagerPtr->CreateMacAddressResolver());
    }//if//

    if (theMacAddressResolverPtr.get() == nullptr) {
        theMacAddressResolverPtr.reset(new SimpleMacAddressResolver(this));
    }//if//

    aShortInterframeSpaceTime = physicalLayer.GetShortInterframeSpaceDuration();
    aSlotTime = physicalLayer.GetSlotDuration();
    aRxTxTurnaroundTime = physicalLayer.GetRxTxTurnaroundTime();
    SimTime aPhyRxStartDelay = physicalLayer.GetPhyRxStartDelay();
    clearToSendTimeoutDuration = aShortInterframeSpaceTime + aSlotTime + aPhyRxStartDelay;
    ackTimeoutDuration = clearToSendTimeoutDuration;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "rts-threshold-size-bytes"), theNodeId, theInterfaceId)) {

        rtsThresholdSizeBytes =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "rts-threshold-size-bytes"), theNodeId, theInterfaceId);

    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "short-frame-retry-limit"), theNodeId, theInterfaceId)) {

        shortFrameRetryLimit =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "short-frame-retry-limit"), theNodeId, theInterfaceId);

        if (shortFrameRetryLimit < 1) {
            cerr << "Error: "<< parameterNamePrefix
                << "short-frame-retry-limit must be greater than or equal to 1: "
                << shortFrameRetryLimit << endl;
            exit(1);
        }//if//

    }//if//


    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "long-frame-retry-limit"), theNodeId, theInterfaceId)) {

        longFrameRetryLimit =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "long-frame-retry-limit"), theNodeId, theInterfaceId);

        if (longFrameRetryLimit < 1) {
            cerr << "Error: "<< parameterNamePrefix
                << "long-frame-retry-limit must be greater than or equal to 1: "
                << longFrameRetryLimit << endl;
            exit(1);
        }//if//

    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "contention-window-min-slots"), theNodeId, theInterfaceId)) {

        contentionWindowSlotsMin =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "contention-window-min-slots"), theNodeId, theInterfaceId);

    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "contention-window-max-slots"), theNodeId, theInterfaceId)) {

        contentionWindowSlotsMax =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "contention-window-max-slots"), theNodeId, theInterfaceId);

    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "disabled-to-jump-on-medium-without-backoff"), theNodeId, theInterfaceId)) {

        disabledToJumpOnMediumWithoutBackoff =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "disabled-to-jump-on-medium-without-backoff"), theNodeId, theInterfaceId);

    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "max-aggregate-mpdu-size-bytes"), theNodeId, theInterfaceId)) {

        maxAggregateMpduSizeBytes =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "max-aggregate-mpdu-size-bytes"), theNodeId, theInterfaceId);

    }//if//

    protectAggregateFramesWithSingleAckedFrame = true;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "protect-aggregate-frames-with-single-acked-frame"), theNodeId, theInterfaceId)) {

        protectAggregateFramesWithSingleAckedFrame =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "protect-aggregate-frames-with-single-acked-frame"),
                theNodeId, theInterfaceId);
    }//if//


    allowFrameAggregationWithTxopZero = false;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "allow-frame-aggregation-with-txop-zero"), theNodeId, theInterfaceId)) {

        allowFrameAggregationWithTxopZero =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "allow-frame-aggregation-with-txop-zero"), theNodeId, theInterfaceId);

    }//if//


    //rate controller
    theAdaptiveRateControllerPtr =
        CreateAdaptiveRateController(
            simulationEngineInterfacePtr,
            theParameterDatabaseReader,
            theNodeId,
            theInterfaceId,
            physicalLayer.GetBaseChannelBandwidthMhz());

    //power controller
    theAdaptiveTxPowerControllerPtr =
        CreateAdaptiveTxPowerController(
            simulationEngineInterfacePtr,
            theParameterDatabaseReader,
            theNodeId,
            theInterfaceId);

    additionalDelayForExtendedInterframeSpaceMode = CalculateAdditionalDelayForExtendedInterframeSpace();

    (*this).InitializeAccessCategories(theParameterDatabaseReader);

    wakeupTimerEventPtr = shared_ptr<WakeupTimerEvent>(new WakeupTimerEvent(this));

    networkOutputQueuePtr.reset(
        new OutputQueueWithPrioritySubqueuesOlderVer(
            theParameterDatabaseReader,
            theInterfaceId,
            simulationEngineInterfacePtr,
            maxPacketPriority));

    if (networkLayerPtr != nullptr) {
        assert(networkLayerPtr->LookupInterfaceIndex(theInterfaceId) == interfaceIndex);
        networkLayerPtr->SetInterfaceOutputQueue(interfaceIndex, networkOutputQueuePtr);
    }
    else {
        // Only happens with emulation "headless nodes".
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "bonded-channel-number-list"), theNodeId, theInterfaceId)) {

        const string bondedChannelListString =
            theParameterDatabaseReader.ReadString(
                parameterNamePrefix + "bonded-channel-number-list", theNodeId, theInterfaceId);

        vector<unsigned int> bondedChannelList;

        (*this).ReadBondedChannelList(
            bondedChannelListString,
            (parameterNamePrefix + "bonded-channel-number-list"),
            bondedChannelList);

        if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "initial-channel-number"), theNodeId, theInterfaceId)) {

            cerr << "Error in configuration file: both "
                 << (parameterNamePrefix + "bonded-channel-number-list") << " and "
                 << (parameterNamePrefix + "initial-channel-number") << " have been specified." << endl;

            exit(1);
        }//if//

        (*this).SwitchToChannels(bondedChannelList);
    }
    else {
        unsigned int initialChannel = 0;

        if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "initial-channel-number"), theNodeId, theInterfaceId)) {

            initialChannel =
                theParameterDatabaseReader.ReadNonNegativeInt(
                    (parameterNamePrefix + "initial-channel-number"), theNodeId, theInterfaceId);

            if (initialChannel >= GetNumberOfChannels()) {
                cerr << "Error: invalid channel number: " << initialChannel << endl;
                exit(1);
            }//if//
        }//if//
        assert(physicalLayer.GetCurrentBondedChannelList().empty());
        (*this).SwitchToChannel(initialChannel);

    }//if//

    // Parallelism Stuff

    (*this).lookaheadIndex = (*this).simEngineInterfacePtr->AllocateLookaheadTimeIndex();
    simEngineInterfacePtr->SetALookaheadTimeForThisNode(
        physicalLayer.GetRxTxTurnaroundTime(), lookaheadIndex);

    if (theParameterDatabaseReader.ParameterExists(parameterNamePrefix + "redundant-trace-information-mode")) {
        redundantTraceInformationModeIsOn =
            theParameterDatabaseReader.ReadBool(parameterNamePrefix + "redundant-trace-information-mode");
    }//if//


    if (theParameterDatabaseReader.ParameterExists("channel-instance-id", theNodeId, theInterfaceId)) {
        InterfaceOrInstanceId propModelInstanceId =
            theParameterDatabaseReader.ReadString("channel-instance-id", theNodeId, theInterfaceId);

        ConvertStringToLowerCase(propModelInstanceId);

        if (propModelInstanceId != propModelInterfacePtr->GetInstanceId()) {
            cerr << "Error: Declared propagation model instance ID, does not match actual prop model ID." << endl;
            cerr << "       Node = " << theNodeId << ", Interface = " << theInterfaceId << endl;
            cerr << "       Declared ID = " << propModelInstanceId << ", Actual ID = "
                 << propModelInterfacePtr->GetInstanceId() << endl;
            exit(1);
        }//if//

    }//if//


    // Only not equal to theNodeId in emulation "split node" mode.

    const NodeId primaryNodeId = theParameterDatabaseReader.GetPossibleNodeIdRemap(theNodeId);

    // Mac address related multiple interface restrictions:
    //   - Only one Adhoc interface on a single propagation environment.
    //   - Can't have two interfaces associate with same AP.

    myMacAddress.SetLowerBitsWithNodeId(primaryNodeId);

    if (operationMode == AdhocMode) {
        myMacAddress.SetInterfaceSelectorByte(AdhocModeAddressSelectorByte);
    }
    else {
        myMacAddress.SetInterfaceSelectorByte(static_cast<unsigned char>(interfaceIndex));
    }//if//


    (*this).ReadInMulticastGroupNumberList(theParameterDatabaseReader);

    // Allow STAs to use fixed TXOP durations set in the parameter file
    // Normally, STA TXOPs are controlled by the AP sent via Beacon frames.

    macSchedulerPtr.reset(
        new FixedTxopScheduler(
            theParameterDatabaseReader,
            theNodeId,
            theInterfaceId,
            accessCategories.size()));


    // 802.11ah:

    if (theParameterDatabaseReader.ParameterExists("dot11ah-use-optimized-ndp-control-frames")) {

        ahUseOptimizedNdpControlFrames =
            theParameterDatabaseReader.ReadBool("dot11ah-use-optimized-ndp-control-frames");

    }//if//

}//Dot11Mac()//


#pragma warning(default:4355)




inline
void Dot11Mac::CompleteInitialization(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const RandomNumberGeneratorSeed& nodeSeed)
{
    //default

    string nodeTypeString = "ad-hoc";

    if (theParameterDatabaseReader.ParameterExists(parameterNamePrefix + "node-type", theNodeId, theInterfaceId)) {

        nodeTypeString = theParameterDatabaseReader.ReadString(parameterNamePrefix + "node-type", theNodeId, theInterfaceId);
        ConvertStringToLowerCase(nodeTypeString);
    }//if//

    if (nodeTypeString == "access-point") {
        operationMode = ApMode;
        apControllerPtr.reset(
            new Dot11ApManagementController(
                shared_from_this(),
                simEngineInterfacePtr,
                theParameterDatabaseReader,
                theNodeId,
                theInterfaceId,
                HashInputsToMakeSeed(nodeSeed, interfaceIndex)));
    }
    else if (nodeTypeString == "mobile-sta") {

        operationMode = InfrastructureMode;

        staControllerPtr.reset(
            new Dot11StaManagementController(
                shared_from_this(),
                simEngineInterfacePtr,
                theParameterDatabaseReader,
                theNodeId,
                theInterfaceId,
                HashInputsToMakeSeed(nodeSeed, interfaceIndex)));
    }
    else if (nodeTypeString == "ad-hoc") {

        operationMode = AdhocMode;

        // No controller needed
    }
    else {
        cerr << parameterNamePrefix << "node-type \"" << nodeTypeString << "\" is not defined." << endl;
        exit(1);
    }//if//

    assert((apControllerPtr == nullptr) || (staControllerPtr == nullptr));

}//CompleteInitialization//



inline
bool Dot11Mac::FrameIsForThisNode(const Packet& aFrame, const unsigned int headerOffset) const
{
    const CommonFrameHeader& header =
        aFrame.GetAndReinterpretPayloadData<CommonFrameHeader>(headerOffset);

    bool isForMe = false;

    if ((header.receiverAddress == myMacAddress) ||
        (header.receiverAddress.IsABroadcastAddress()) ||
        ((header.receiverAddress.IsAMulticastAddress() &&
         (myMulticastMacAddresses.count(header.receiverAddress) == 1)))) {

        // For me or broadcast

        switch (GetOperationMode()) {
        case AdhocMode:
            isForMe = true;
            break;

        case InfrastructureMode:
        {
            if (header.theFrameControlField.frameTypeAndSubtype == QOS_DATA_FRAME_TYPE_CODE) {

                // Data frame

                const QosDataFrameHeader& dataFrameHeader =
                    aFrame.GetAndReinterpretPayloadData<QosDataFrameHeader>(headerOffset);

                bool wasFound = false;
                MacAddress apMacAddress;
                staControllerPtr->GetCurrentAccessPointAddress(wasFound, apMacAddress);

                // Receive only from associated ap

                isForMe = ((wasFound) && (dataFrameHeader.transmitterAddress == apMacAddress));
            }
            else {
                //Control and managment frames

                isForMe = true;
            }//if//
            break;
        }

        case ApMode:
        {
            if (header.theFrameControlField.frameTypeAndSubtype == QOS_DATA_FRAME_TYPE_CODE) {

                // Data frame

                const QosDataFrameHeader& dataFrameHeader =
                    aFrame.GetAndReinterpretPayloadData<QosDataFrameHeader>();

                // Receive only from associated sta
                isForMe = apControllerPtr->IsAnAssociatedStaAddress(dataFrameHeader.transmitterAddress);

            }
            else {
                // Control and managment frames

                isForMe = true;
            }//if//
            break;
        }
        default:
            assert(false); abort(); break;
        }//switch//
    }
    else {
        // For others
        // Need to receive frames for others if mac supports link level multi-hop

    }//if//

    if ((!isForMe) && (header.receiverAddress == myMacAddress)) {
        // Delete this if it starts (correctly) spewing too many messages .
        cerr << "Warning: 802.11 Frame addressed to node is unreceivable (not associated): frame dropped" << endl;
    }//if//

    return (isForMe);

}//FrameIsForThisNode//


inline
bool Dot11Mac::AggregatedSubframeIsForThisNode(const Packet& frame) const
{
    const CommonFrameHeader& header =
        frame.GetAndReinterpretPayloadData<CommonFrameHeader>(sizeof(MpduDelimiterFrame));

    assert(!header.receiverAddress.IsABroadcastOrAMulticastAddress());

    return (header.receiverAddress == myMacAddress);
}



inline
void Dot11Mac::ScheduleWakeupTimer(const SimTime& wakeupTime)
{
    (*this).currentWakeupTimerExpirationTime = wakeupTime;

    if (wakeupTimerEventTicket.IsNull()) {
        simEngineInterfacePtr->ScheduleEvent(
            wakeupTimerEventPtr, wakeupTime, wakeupTimerEventTicket);
    }
    else {
        simEngineInterfacePtr->RescheduleEvent(wakeupTimerEventTicket, wakeupTime);

    }//if//

}//ScheduleWakeupTimer//


inline
void Dot11Mac::CancelWakeupTimer()
{
    assert(!wakeupTimerEventTicket.IsNull());
    (*this).currentWakeupTimerExpirationTime = INFINITE_TIME;
    simEngineInterfacePtr->CancelEvent(wakeupTimerEventTicket);
}



inline
SimTime Dot11Mac::CalculateNonExtendedBackoffDuration(
    const EdcaAccessCategoryInfo& accessCategory) const
{
    const int totalSlots =
        (accessCategory.arbitrationInterframeSpaceDurationSlots + accessCategory.currentNumOfBackoffSlots);

    return (aShortInterframeSpaceTime + (aSlotTime * totalSlots) - aRxTxTurnaroundTime);

}//CalculateNonExtendedBackoffDuration//


inline
SimTime Dot11Mac::CalculateNonExtendedDurationForJumpingOnMediumWithNoBackoff(
    const EdcaAccessCategoryInfo& accessCategory) const
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    SimTime ifsDuration = aShortInterframeSpaceTime +
        (aSlotTime * accessCategory.arbitrationInterframeSpaceDurationSlots) - aRxTxTurnaroundTime;

    if ((mediumBecameIdleTime + ifsDuration) >= currentTime) {

        // Wait for the interframe space.

        return (mediumBecameIdleTime + ifsDuration - currentTime);
    } else {
        // Wait until next slot start.

        return (aSlotTime - ((currentTime - mediumBecameIdleTime - ifsDuration) % aSlotTime));
    }//if//

}//CalculateNonExtendedDurationForJumpingOnMediumWithNoBackoff//


inline
bool Dot11Mac::AccessCategoryIsActive(const unsigned int accessCategoryIndex) const
{
    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    return ((accessCategoryInfo.currentNonExtendedBackoffDuration != INFINITE_TIME) ||
            (accessCategoryInfo.hasPacketToSend));
}

inline
unsigned int Dot11Mac::FindIndexOfAccessCategoryWithEarliestBackoffExpiration() const
{
    unsigned int shortestIndex = static_cast<unsigned int>(accessCategories.size());
    SimTime earliestExpirationTime = INFINITE_TIME;

    // Higher Access Indices have higher priority

    for(int i = (numberAccessCategories - 1); (i >= 0); i--) {
        const EdcaAccessCategoryInfo& accessCategoryInfo =
            accessCategories[i];

        if ((accessCategoryInfo.ifsAndBackoffStartTime != INFINITE_TIME) &&
            (accessCategoryInfo.currentNonExtendedBackoffDuration != INFINITE_TIME)) {

            const SimTime expirationTime =
                (accessCategories[i].ifsAndBackoffStartTime +
                 accessCategories[i].currentNonExtendedBackoffDuration);

            if (expirationTime < earliestExpirationTime) {
                earliestExpirationTime = expirationTime;
                shortestIndex = i;
            }//if//
        }//if//
    }//for//

    return shortestIndex;

}//FindIndexOfAccessCategoryWithEarliestBackoffExpiration//

inline
unsigned int Dot11Mac::FindIndexOfAccessCategoryWithShortestBackoff() const
{
    unsigned int shortestIndex = static_cast<unsigned int>(accessCategories.size());
    SimTime shortestBackoff = INFINITE_TIME;

    // Higher Access Indices have higher priority

    for(int i = (numberAccessCategories - 1); (i >= 0); i--) {
        if (shortestBackoff > accessCategories[i].currentNonExtendedBackoffDuration) {
            shortestBackoff = accessCategories[i].currentNonExtendedBackoffDuration;
            shortestIndex = i;
        }//if//
    }//for//

    return shortestIndex;

}//FindIndexOfAccessCategoryWithShortestBackoff//


inline
SimTime Dot11Mac::CurrentBackoffDuration() const
{
    unsigned int accessCategoryIndex = FindIndexOfAccessCategoryWithShortestBackoff();

    if (accessCategoryIndex == accessCategories.size()) {
        return INFINITE_TIME;
    }//if//

    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    if (lastFrameReceivedWasCorrupt) {
        return (additionalDelayForExtendedInterframeSpaceMode +
                accessCategoryInfo.currentNonExtendedBackoffDuration);
    }
    else {
        return accessCategoryInfo.currentNonExtendedBackoffDuration;
    }//if//

}//CurrentBackoffDuration//



inline
SimTime Dot11Mac::CurrentBackoffExpirationTime() const
{
    assert(macState == WaitingForIfsAndBackoffState);

    unsigned int accessCategoryIndex = FindIndexOfAccessCategoryWithEarliestBackoffExpiration();

    if (accessCategoryIndex == accessCategories.size()) {
        return INFINITE_TIME;
    }//if//

    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    if (lastFrameReceivedWasCorrupt) {
        return (additionalDelayForExtendedInterframeSpaceMode +
                accessCategoryInfo.ifsAndBackoffStartTime +
                accessCategoryInfo.currentNonExtendedBackoffDuration);
    }
    else {
        return (accessCategoryInfo.ifsAndBackoffStartTime +
                accessCategoryInfo.currentNonExtendedBackoffDuration);
    }//if//

}//CurrentBackoffExpirationTime//


//-------------------------------------------------------------------------------------------------


inline
void Dot11Mac::RecalcRandomBackoff(EdcaAccessCategoryInfo& accessCategoryInfo)
{
    accessCategoryInfo.currentNumOfBackoffSlots =
        aRandomNumberGenerator.GenerateRandomInt(0, accessCategoryInfo.currentContentionWindowSlots);

    accessCategoryInfo.currentNonExtendedBackoffDuration =
        CalculateNonExtendedBackoffDuration(accessCategoryInfo);
}


inline
bool Dot11Mac::AhScheduleSaysCanTransmitNow() const
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
    return (currentTime < ahRestrictedAccessWindowEndTime);
}


inline
void Dot11Mac::StartBackoffIfNecessary()
{
    assert(macState != WaitingForIfsAndBackoffState);
    assert(macState != BusyMediumState);

    if (!AhScheduleSaysCanTransmitNow()) {
        // Not in a RAW period, can't access medium.
        macState = IdleState;
        return;
    }//if//

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
    const SimTime backoffDuration = CurrentBackoffDuration();

    if (backoffDuration == INFINITE_TIME) {
        macState = IdleState;
        return;
    }//if//

    if ((currentTime + backoffDuration) >= ahRestrictedAccessWindowEndTime) {
        // Backoff would exceed RAW period, don't start backoff.
        macState = IdleState;
        return;
    }//if//

    macState = WaitingForIfsAndBackoffState;

    for(unsigned int i = 0; (i < accessCategories.size()); i++) {
        EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[i];
        if (accessCategoryInfo.currentNonExtendedBackoffDuration != INFINITE_TIME) {
            accessCategoryInfo.ifsAndBackoffStartTime = currentTime;
        }
        else {
            accessCategoryInfo.ifsAndBackoffStartTime = INFINITE_TIME;
        }//if//
    }//for//

    (*this).ScheduleWakeupTimer(currentTime + backoffDuration);

    OutputTraceForIfsAndBackoffStart(backoffDuration);

}//StartBackoffIfNecessary//


inline
void Dot11Mac::StartBackoffForCategory(const unsigned int accessCategoryIndex)
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    assert((macState == IdleState) || (macState == WaitingForIfsAndBackoffState));
    (*this).macState = WaitingForIfsAndBackoffState;

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];
    accessCategoryInfo.ifsAndBackoffStartTime = currentTime;

    const SimTime wakeupTime = CurrentBackoffExpirationTime();

    assert(wakeupTime < ahRestrictedAccessWindowEndTime);

    if (wakeupTime != currentWakeupTimerExpirationTime) {
        (*this).ScheduleWakeupTimer(wakeupTime);

        OutputTraceForIfsAndBackoffStart(wakeupTime - currentTime);
    }//if//

    assert(macState == WaitingForIfsAndBackoffState);

}//StartBackoffForCategory//




//-------------------------------------------------------------------------------------------------
//
// The following routines are for when a new packet appears at the Network Layer and a
// access category goes from idle to active.
//


inline
void Dot11Mac::StartPacketSendProcessForAnAccessCategory(
    const unsigned int accessCategoryIndex,
    const bool forceBackoff)
{
    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff = false;

    if ((accessCategoryInfo.currentPacketPtr == nullptr) &&
        (accessCategoryInfo.currentAggregateFramePtr == nullptr)) {

        accessCategoryInfo.hasPacketToSend =
            ThereAreQueuedPacketsForAccessCategory(accessCategoryIndex);

    }//if//

    if ((!forceBackoff) && (!accessCategoryInfo.hasPacketToSend)) {
        accessCategoryInfo.currentNonExtendedBackoffDuration = INFINITE_TIME;

        return;

    }//if//

    assert(accessCategoryInfo.currentNonExtendedBackoffDuration == INFINITE_TIME);

    assert(accessCategoryInfo.currentNonExtendedBackoffDuration == INFINITE_TIME);

    switch (macState) {
    case BusyMediumState:
    case CtsOrAckTransmissionState:
    case WaitingForNavExpirationState:
    case WaitingForCtsState:
    case WaitingForAckState:
    case ChangingChannelsState:
    {
        (*this).RecalcRandomBackoff(accessCategoryInfo);

        break;
    }
    case IdleState:
    case WaitingForIfsAndBackoffState:
    {
        if (ahRestrictedAccessWindowEndTime == INFINITE_TIME) {
            // No Restricted Access Window (RAW) mode.

            if (disabledToJumpOnMediumWithoutBackoff) {

                // Note: Add complete IFS even if channel is completely idle.

                (*this).RecalcRandomBackoff(accessCategoryInfo);

                accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff = false;
            }
            else {
                assert(!forceBackoff);

                accessCategoryInfo.currentNumOfBackoffSlots = 0;
                accessCategoryInfo.currentNonExtendedBackoffDuration =
                    CalculateNonExtendedDurationForJumpingOnMediumWithNoBackoff(accessCategoryInfo);
                accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff = true;

            }//if//

            (*this).StartBackoffForCategory(accessCategoryIndex);
        }
        else {
            // Currently in RAW don't allow "jump on medium without backoff".

            accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff = false;
            (*this).RecalcRandomBackoff(accessCategoryInfo);

            if ((AhScheduleSaysCanTransmitNow()) &&
                ((simEngineInterfacePtr->CurrentTime() + CurrentBackoffDuration()) <
                 ahRestrictedAccessWindowEndTime)) {

                (*this).StartBackoffForCategory(accessCategoryIndex);
            }//if//
        }//if//

        break;
    }
    default:
        assert(false); abort(); break;
    }//switch//

}//StartPacketSendProcessForAnAccessCategory//


inline
bool Dot11Mac::NetworkLayerHasPacketForAccessCategory(const unsigned int accessCategoryIndex) const
{
    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    for(unsigned int i = 0; (i < accessCategoryInfo.listOfPriorities.size()); i++) {
        if (networkOutputQueuePtr->HasPacketWithPriority(accessCategoryInfo.listOfPriorities[i])) {
            return true;
        }//if//
    }//for//

    return false;

}//NetworkLayerHasPacketForAccessCategory//



inline
void Dot11Mac::NetworkLayerQueueChangeNotification()
{
    // High Priority to Low priority category order:

    for(int i = (numberAccessCategories-1); (i >= 0); i--) {
        if (!AccessCategoryIsActive(i)) {

            (*this).StartPacketSendProcessForAnAccessCategory(i);

        }//if//
    }//for//

}//NetworkLayerQueueChangeNotification//




inline
bool Dot11Mac::ThereAreQueuedPacketsForAccessCategory(const unsigned int accessCategoryIndex) const
{
    return (((accessCategoryIndex == accessCategoryIndexForManagementFrames) &&
             (!managementFrameQueue.empty())) ||
            (NetworkLayerHasPacketForAccessCategory(accessCategoryIndex)));
}



//--------------------------------------------------------------------------------------------------
//
// The following routines have to do with effect of on/off transitions of carrier sense
// and virtual carrier sense (NAV) on the backoff of the EDCA access categories.
//


inline
void Dot11Mac::ProcessVirtualCarrierSenseAkaNavExpiration()
{
    assert(macState == WaitingForNavExpirationState);

    OutputTraceForNavExpiration();

    macState = TransientState;
    mediumBecameIdleTime = simEngineInterfacePtr->CurrentTime();
    (*this).StartBackoffIfNecessary();
}



inline
int Dot11Mac::NumberBackoffSlotsUsedDuringElapsedTime(
    const EdcaAccessCategoryInfo& accessCategory,
    const SimTime& elapsedTime) const
{
    SimTime interframeSpaceDuration =
        aShortInterframeSpaceTime + (aSlotTime * accessCategory.arbitrationInterframeSpaceDurationSlots);

    if (lastFrameReceivedWasCorrupt) {
        interframeSpaceDuration += additionalDelayForExtendedInterframeSpaceMode;
    }//if//


    if (elapsedTime <= interframeSpaceDuration) {
        return 0;
    }//if//

    return int((elapsedTime + aRxTxTurnaroundTime - interframeSpaceDuration) / aSlotTime);

}//NumberBackoffSlotsUsedDuringElapsedTime//


inline
void Dot11Mac::PauseBackoffForAnAccessCategory(
    const unsigned int accessCategoryIndex,
    const SimTime& elapsedTime)
{
    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    if (accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff) {
        // Interrupted in Interframe space (IFS), must start backoff process.

        accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff = false;
        accessCategoryInfo.currentNonExtendedBackoffDuration = INFINITE_TIME;

        (*this).StartPacketSendProcessForAnAccessCategory(accessCategoryIndex, true);
    }
    else {
        if (accessCategoryInfo.currentNonExtendedBackoffDuration != INFINITE_TIME) {

            accessCategoryInfo.currentNumOfBackoffSlots -=
                NumberBackoffSlotsUsedDuringElapsedTime(accessCategoryInfo, elapsedTime);

            assert(accessCategoryInfo.currentNumOfBackoffSlots >= 0);

            accessCategoryInfo.currentNonExtendedBackoffDuration =
                CalculateNonExtendedBackoffDuration(accessCategoryInfo);

            OutputTraceForInterruptedIfsAndBackoff(
                accessCategoryIndex,
                accessCategoryInfo.currentNonExtendedBackoffDuration);
        }//if//

    }//if//

}//PauseBackoffForAnAccessCategory//



inline
void Dot11Mac::BusyChannelAtPhysicalLayerNotification()
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    OutputTraceForBusyChannel();

    switch (macState) {
    case WaitingForNavExpirationState:
        (*this).CancelWakeupTimer();
        macState = BusyMediumState;

        break;

    case WaitingForIfsAndBackoffState: {

        macState = BusyMediumState;

        for(unsigned int i = 0; (i < accessCategories.size()); i++) {
            SimTime elapsedTime = (currentTime - accessCategories[i].ifsAndBackoffStartTime);

            (*this).PauseBackoffForAnAccessCategory(i, elapsedTime);
        }//for//

        (*this).CancelWakeupTimer();

        break;
    }
    case IdleState:
        macState = BusyMediumState;
        break;

    case WaitingForCtsState:
    case WaitingForAckState:

        // Ignore (incoming frame could be for this station).
        break;

    case BusyMediumState:
    case CtsOrAckTransmissionState:

        // Ignore: Still busy
        //assert(false && "PHY sending duplicate notifications, should not happen."); abort();
        break;

    default:
        assert(false); abort(); break;
    }//switch//

}//BusyChannelAtPhysicalLayerNotification//


inline
void Dot11Mac::GoIntoWaitForExpirationOfVirtualCarrierSenseAkaNavState()
{
    assert((macState == BusyMediumState) || (macState == TransientState));
    assert(wakeupTimerEventTicket.IsNull());

    macState = WaitingForNavExpirationState;

    (*this).ScheduleWakeupTimer(mediumReservedUntilTimeAkaNAV);

    OutputTraceForNavStart(mediumReservedUntilTimeAkaNAV);
}


inline
void Dot11Mac::ClearChannelAtPhysicalLayerNotification()
{
    OutputTraceForClearChannel();

    if ((macState == WaitingForCtsState) || (macState == WaitingForAckState)) {
        if (WakeupTimerIsActive()) {
            // Wait until the CTS/ACK timeout completes before taking action.
            return;
        }
        else {
            // Get out of the ACK/CTS timeout state.
            macState = TransientState;

            (*this).NeverReceivedClearToSendOrAcknowledgementAction();
        }//if//

    }//if//

    if (macState == WaitingForNavExpirationState) {
        return;
    }//if//

    assert((macState == BusyMediumState) || (macState == TransientState) ||
           ((macState == IdleState) && (!AhScheduleSaysCanTransmitNow())));

    if (simEngineInterfacePtr->CurrentTime() < mediumReservedUntilTimeAkaNAV) {
        (*this).GoIntoWaitForExpirationOfVirtualCarrierSenseAkaNavState();
    }
    else {
        mediumBecameIdleTime = simEngineInterfacePtr->CurrentTime();
        (*this).macState = TransientState;
        (*this).StartBackoffIfNecessary();

    }//if//

    assert((macState != BusyMediumState) && (macState != TransientState));

}//ClearChannelAtPhysicalLayerNotification//





inline
void Dot11Mac::DoSuccessfulTransmissionPostProcessing(const bool wasJustTransmitting)
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    if (currentTime >= ahRestrictedAccessWindowEndTime) {

        assert(macState == BusyMediumState);
        (*this).StartPacketSendProcessForAnAccessCategory(accessCategoryIndexForLastSentFrame, true);

        return;
    }//if//

    if (currentTransmitOpportunityAkaTxopEndTime == ZERO_TIME) {

        // TXOP is not being used (1 frame per medium access).
        // or number packet limit is exhausted.

        (*this).StartPacketSendProcessForAnAccessCategory(accessCategoryIndexForLastSentFrame, true);
        return;
    }//if//

    const SimTime nextFrameSequenceDuration =
        CalculateNextFrameSequenceDuration(accessCategoryIndexForLastSentFrame);

    if (nextFrameSequenceDuration == ZERO_TIME) {

        // Nothing To Send

        currentTransmitOpportunityAkaTxopEndTime = ZERO_TIME;
        (*this).StartPacketSendProcessForAnAccessCategory(accessCategoryIndexForLastSentFrame, true);

        return;
    }//if//

    if ((currentTime + nextFrameSequenceDuration) > currentTransmitOpportunityAkaTxopEndTime) {

        // Not enough time left in the TXOP for next packet.

        currentTransmitOpportunityAkaTxopEndTime = ZERO_TIME;
        (*this).StartPacketSendProcessForAnAccessCategory(accessCategoryIndexForLastSentFrame, true);
    }
    else {
        // Send another packet in the TXOP (Transmit opportunity).

        bool packetHasBeenSentToPhy;

        // Modelling hack having to do with mobility and consecutive frames.  Mobility may
        // shorten the propagation delay causing the end of the last frame to arrive later
        // than the start of the next frame (as mobility is not considered intra-frame).

        SimTime transmitDelay = physicalLayer.GetDelayBetweenConsecutiveFrames();

        if (!wasJustTransmitting) {
            transmitDelay = aShortInterframeSpaceTime;
        }//if//

        (*this).TransmitAFrame(
            accessCategoryIndexForLastSentFrame,
            true,
            transmitDelay,
            packetHasBeenSentToPhy);

        assert(packetHasBeenSentToPhy);
    }//if//

}//DoSuccessfulTransmissionPostProcessing//



// Could infer this functionality from Carrier/No Carrier notifications
// but added for clarity.

inline
void Dot11Mac::TransmissionIsCompleteNotification()
{
    if (macState == WaitingForCtsState) {

        (*this).ScheduleWakeupTimer(
            simEngineInterfacePtr->CurrentTime() + clearToSendTimeoutDuration);
    }
    else if (macState == WaitingForAckState) {

        (*this).ScheduleWakeupTimer(simEngineInterfacePtr->CurrentTime() + ackTimeoutDuration);
    }
    else if (macState == CtsOrAckTransmissionState) {
        macState = BusyMediumState;
    }
    else if (macState == ChangingChannelsState) {

        // Assume the medium is busy, but if it is idle the PHY will notify the MAC at channel
        // switch.

        macState = BusyMediumState;

        // Assume last transmission failed (unless it was a broadcast).

        (*this).StartPacketSendProcessForAnAccessCategory(accessCategoryIndexForLastSentFrame);
        (*this).SwitchToChannels(switchingToThisChannelList);
    }
    else if (macState == IdleState) {
        // Do nothing.
        assert(!AhScheduleSaysCanTransmitNow());
    }
    else {
        assert(macState == BusyMediumState);

        if (lastTransmittedFrameWasABeacon) {
            apControllerPtr->ProcessBeaconFrameTransmissionNotification();
        }//if//

        (*this).DoSuccessfulTransmissionPostProcessing(true);

    }//if//

}//TransmissionIsCompleteNotification//



//--------------------------------------------------------------------------------------------------

inline
void Dot11Mac::SetMpduFrameAggregationIsEnabledFor(const MacAddress& destinationAddress)
{
    neighborCapabilitiesMap[destinationAddress].mpduFrameAggregationIsEnabled = true;
}


inline
bool Dot11Mac::FrameAggregationIsEnabledFor(const MacAddress& destinationAddress) const
{
    typedef map<MacAddress, NeighborCapabilities>::const_iterator IterType;

    if (neighborCapabilitiesMap.empty()) {
        return false;
    }//if//

    IterType iter = neighborCapabilitiesMap.find(destinationAddress);

    if (iter == neighborCapabilitiesMap.end()) {
        return false;
    }//if//

    const NeighborCapabilities& capabilities = iter->second;

    return (capabilities.mpduFrameAggregationIsEnabled);

}//FrameAggregationIsEnabledForLink//



inline
bool Dot11Mac::MpduFrameAggregationIsEnabledFor(const MacAddress& destinationAddress) const
{
    typedef map<MacAddress, NeighborCapabilities>::const_iterator IterType;

    if (neighborCapabilitiesMap.empty()) {
        return false;
    }//if//

    IterType iter = neighborCapabilitiesMap.find(destinationAddress);

    if (iter == neighborCapabilitiesMap.end()) {
        return false;
    }//if//

    const NeighborCapabilities& capabilities = iter->second;

    return (capabilities.mpduFrameAggregationIsEnabled);

}//MpduFrameAggregationIsEnabledForLink//



inline
bool Dot11Mac::NeedToSendABlockAckRequest(
    const MacAddress& destinationAddress,
    const PacketPriority& trafficId) const
{
    typedef map<AddressAndTrafficIdMapKey, OutgoingLinkInfo>::const_iterator IterType;

    IterType iter =
        outgoingLinkInfoMap.find(AddressAndTrafficIdMapKey(destinationAddress, trafficId));

    if (iter == outgoingLinkInfoMap.end()) {
        return false;
    }//if//

    return ((*iter).second.blockAckRequestNeedsToBeSent);

}//NeedToSendBlockAckRequest//



inline
void Dot11Mac::BuildAggregateFrameFromCurrentFrame(const unsigned int accessCategoryIndex)
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    assert((allowFrameAggregationWithTxopZero) ||
           (currentTransmitOpportunityAkaTxopEndTime != ZERO_TIME));

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    const bool useMpduAggregation =
        MpduFrameAggregationIsEnabledFor(accessCategoryInfo.currentPacketsDestinationMacAddress);

    accessCategoryInfo.currentAggregateFrameIsAMpduAggregate = useMpduAggregation;

    assert(accessCategoryInfo.currentAggregateFramePtr == nullptr);
    accessCategoryInfo.currentAggregateFramePtr.reset(new vector<unique_ptr<Packet> >());

    accessCategoryInfo.currentAggregateFramePtr->push_back(move(accessCategoryInfo.currentPacketPtr));

    accessCategoryInfo.currentShortFrameRetryCount = 0;
    accessCategoryInfo.currentLongFrameRetryCount = 0;
    accessCategoryInfo.currentAggregateFrameRetryCount = 0;

    vector<unique_ptr<Packet> >& frameList = *accessCategoryInfo.currentAggregateFramePtr;

    const QosDataFrameHeader copyOfOriginalFrameHeader =
        frameList[0]->GetAndReinterpretPayloadData<QosDataFrameHeader>();

    AddMpduDelimiterAndPaddingToFrame(*frameList[0]);

    // Assuming No RTS/CTS for aggregated frames ("Greenfield" not supported).

    TransmissionParameters txParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForDataFrameToStation(
        accessCategoryInfo.currentPacketsDestinationMacAddress,
        txParameters);

    unsigned int totalBytes = frameList[0]->LengthBytes();

    SimTime endTime =
        currentTime +
        CalculateFrameDuration(frameList[0]->LengthBytes(), txParameters);

    endTime += aShortInterframeSpaceTime;
    endTime += CalculateFrameDuration(sizeof(BlockAcknowledgementFrame), txParameters);

    const PacketPriority& priority = accessCategoryInfo.currentPacketPriorityAkaTrafficId;

    const unsigned int maxNumSubframes =
        CalcNumberFramesLeftInSequenceNumWindow(
            accessCategoryInfo.currentPacketsDestinationMacAddress,
            accessCategoryInfo.currentPacketPriorityAkaTrafficId) + 1;

    while ((accessCategoryInfo.currentAggregateFramePtr->size() < maxNumSubframes) &&
           (((allowFrameAggregationWithTxopZero) &&
             (currentTransmitOpportunityAkaTxopEndTime == ZERO_TIME)) ||
            (endTime < currentTransmitOpportunityAkaTxopEndTime)) &&
           (networkOutputQueuePtr->HasPacketWithPriority(priority)) &&
           (networkOutputQueuePtr->NextHopAddressForTopPacket(priority) ==
            accessCategoryInfo.currentPacketsNextHopNetworkAddress)) {

        unsigned int packetLengthBytes;

        packetLengthBytes =
            (networkOutputQueuePtr->TopPacket(priority).LengthBytes() +
            sizeof(QosDataFrameHeader) +
            sizeof(MpduDelimiterFrame));

        if ((totalBytes + packetLengthBytes) > maxAggregateMpduSizeBytes) {
            break;
        }//if//

        // Only first frame of aggregate has Phy header.

        endTime += CalculateFrameDurationWithoutPhyHeader(packetLengthBytes, txParameters);

        if ((!allowFrameAggregationWithTxopZero) &&
            (endTime >= currentTransmitOpportunityAkaTxopEndTime)) {

            break;
        }//if//

        NetworkAddress nextHopAddress;
        EtherTypeField etherType;
        SimTime timestamp;
        unsigned int retryTxCount;

        unique_ptr<Packet> framePtr;

        networkOutputQueuePtr->DequeuePacketWithPriority(
            priority,
            framePtr,
            nextHopAddress,
            etherType,
            timestamp,
            retryTxCount);

        assert(nextHopAddress == accessCategoryInfo.currentPacketsNextHopNetworkAddress);
        assert(retryTxCount == 0);
        assert(etherType == accessCategoryInfo.currentPacketsEtherType);

        unsigned short int newSequenceNumber;
        (*this).GetNewSequenceNumber(
            accessCategoryInfo.currentPacketsDestinationMacAddress,
            priority, false, newSequenceNumber);

        QosDataFrameHeader dataFrameHeader = copyOfOriginalFrameHeader;
        dataFrameHeader.theSequenceControlField.sequenceNumber = newSequenceNumber;

        framePtr->AddPlainStructHeader(dataFrameHeader);
        AddMpduDelimiterAndPaddingToFrame(*framePtr);

        totalBytes += framePtr->LengthBytes();

        accessCategoryInfo.currentAggregateFramePtr->push_back(move(framePtr));

    }//while//

    if (accessCategoryInfo.currentAggregateFramePtr->size() > 1) {
        // No padding needed on last frame.
        RemoveMpduAggregationPaddingFromFrame(
            *accessCategoryInfo.currentAggregateFramePtr->back());
    }
    else {
        // Aggregation Failure (Back to single frame).

        accessCategoryInfo.currentPacketPtr =
            move(accessCategoryInfo.currentAggregateFramePtr->front());

        accessCategoryInfo.currentAggregateFramePtr.reset();

        RemoveMpduDelimiterAndPaddingFromFrame(*accessCategoryInfo.currentPacketPtr);
    }//if//

}//BuildAggregateFrameFromCurrentFrame//



inline
void Dot11Mac::RetrievePacketFromNetworkLayerForAccessCategory(
    const unsigned int accessCategoryIndex,
    bool& wasRetrieved)
{
    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    assert(accessCategoryInfo.currentPacketPtr == nullptr);

    wasRetrieved = false;

    for(unsigned int i = 0; (i < accessCategoryInfo.listOfPriorities.size()); i++) {
        PacketPriority priority = accessCategoryInfo.listOfPriorities[i];

        while ((!wasRetrieved) && (networkOutputQueuePtr->HasPacketWithPriority(priority))) {

            NetworkAddress nextHopAddress;
            EtherTypeField etherType;
            SimTime timestamp;
            unsigned int retryTxCount;

            networkOutputQueuePtr->DequeuePacketWithPriority(
                priority,
                accessCategoryInfo.currentPacketPtr,
                nextHopAddress,
                etherType,
                timestamp,
                retryTxCount);

            assert((nextHopAddress.IsTheBroadcastAddress()) ||
                   !(nextHopAddress.IsABroadcastAddress(networkLayerPtr->GetSubnetMask(interfaceIndex))) &&
                   "Make sure no mask dependent broadcast address from network layer");

            const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
            if (accessCategoryInfo.frameLifetime < (currentTime - timestamp)) {
                //lifetime expired
                networkLayerPtr->ReceiveUndeliveredPacketFromMac(
                    interfaceIndex,
                    accessCategoryInfo.currentPacketPtr,
                    NetworkAddress());

                // Loop again to get another packet.
                continue;
            }//if//

            accessCategoryInfo.currentPacketPriorityAkaTrafficId = priority;
            accessCategoryInfo.currentPacketsNextHopNetworkAddress = nextHopAddress;
            accessCategoryInfo.currentPacketsEtherType = etherType;
            accessCategoryInfo.currentPacketsTimestamp = timestamp;

            const unsigned int adjustedFrameLengthBytes =
                accessCategoryInfo.currentPacketPtr->LengthBytes() +
                sizeof(QosDataFrameHeader);

            if (adjustedFrameLengthBytes < rtsThresholdSizeBytes) {
                accessCategoryInfo.currentShortFrameRetryCount = retryTxCount;
                accessCategoryInfo.currentLongFrameRetryCount = 0;
            }
            else {
                accessCategoryInfo.currentShortFrameRetryCount = 0;
                accessCategoryInfo.currentLongFrameRetryCount = retryTxCount;
            }//if//

            if (redundantTraceInformationModeIsOn) {
                OutputTraceForPacketDequeue(accessCategoryIndex, aRxTxTurnaroundTime);
            } else {
                OutputTraceForPacketDequeue(accessCategoryIndex);
            }//if//

            if ((refreshNextHopOnDequeueModeIsOn) && (!nextHopAddress.IsTheBroadcastAddress())) {
                // Update the next hop to latest.

                bool nextHopWasFound;
                unsigned int nextHopInterfaceIndex;

                networkLayerPtr->GetNextHopAddressAndInterfaceIndexForNetworkPacket(
                    *accessCategoryInfo.currentPacketPtr,
                    nextHopWasFound,
                    accessCategoryInfo.currentPacketsNextHopNetworkAddress,
                    nextHopInterfaceIndex);

               if ((!nextHopWasFound) || (nextHopInterfaceIndex != interfaceIndex)) {
                    // Next hop is no longer valid (dynamic routing).

                    networkLayerPtr->ReceiveUndeliveredPacketFromMac(
                        interfaceIndex,
                        accessCategoryInfo.currentPacketPtr,
                        NetworkAddress());

                    // Loop to get another packet.
                    continue;
               }//if//
            }//if//

            wasRetrieved = false;

            theMacAddressResolverPtr->GetMacAddress(
                nextHopAddress,
                networkLayerPtr->GetSubnetMask(interfaceIndex),
                wasRetrieved,
                accessCategoryInfo.currentPacketsDestinationMacAddress);

            if (!wasRetrieved) {
                // There is not a mac address entry.

                networkLayerPtr->ReceiveUndeliveredPacketFromMac(
                    interfaceIndex,
                    accessCategoryInfo.currentPacketPtr,
                    NetworkAddress());

                // Loop to get another packet.
                continue;

            }//if//

            (*this).GetNewSequenceNumber(
                accessCategoryInfo.currentPacketsDestinationMacAddress,
                accessCategoryInfo.currentPacketPriorityAkaTrafficId,
                true,
                /*out*/accessCategoryInfo.currentPacketSequenceNumber);

            QosDataFrameHeader dataFrameHeader;

            dataFrameHeader.header.theFrameControlField.frameTypeAndSubtype = QOS_DATA_FRAME_TYPE_CODE;
            dataFrameHeader.header.theFrameControlField.isRetry = 0;
            dataFrameHeader.header.duration = 0;
            dataFrameHeader.header.receiverAddress = accessCategoryInfo.currentPacketsDestinationMacAddress;
            dataFrameHeader.theSequenceControlField.sequenceNumber = accessCategoryInfo.currentPacketSequenceNumber;
            dataFrameHeader.transmitterAddress = myMacAddress;
            dataFrameHeader.qosControlField.trafficId = accessCategoryInfo.currentPacketPriorityAkaTrafficId;
            dataFrameHeader.linkLayerHeader.etherType = HostToNet16(accessCategoryInfo.currentPacketsEtherType);

            accessCategoryInfo.currentPacketPtr->AddPlainStructHeader(dataFrameHeader);

            // Check for Power Saving destination.

            if ((GetOperationMode() == ApMode) &&
                (apControllerPtr->StationIsAsleep(
                    accessCategoryInfo.currentPacketsDestinationMacAddress))) {

                apControllerPtr->BufferPacketForSleepingStation(
                    accessCategoryInfo.currentPacketsDestinationMacAddress,
                    accessCategoryInfo.currentPacketPtr,
                    accessCategoryInfo.currentPacketsNextHopNetworkAddress,
                    priority,
                    etherType,
                    timestamp);

                wasRetrieved = false;
                // Loop to get another packet.
                continue;
            }//if//

            assert(wasRetrieved);

            if (((FrameAggregationIsEnabledFor(
                    accessCategoryInfo.currentPacketsDestinationMacAddress)) &&
                 (!NeedToSendABlockAckRequest(
                    accessCategoryInfo.currentPacketsDestinationMacAddress,
                    accessCategoryInfo.currentPacketPriorityAkaTrafficId))) &&
                ((allowFrameAggregationWithTxopZero) ||
                 (currentTransmitOpportunityAkaTxopEndTime != ZERO_TIME)) &&
                ((!protectAggregateFramesWithSingleAckedFrame) ||
                 (currentTransmitOpportunityAckedFrameCount > 0) ||
                 (dataFrameHeader.header.receiverAddress.IsABroadcastOrAMulticastAddress()))) {

                if (!BlockAckSessionIsEnabled(
                    accessCategoryInfo.currentPacketsDestinationMacAddress,
                    accessCategoryInfo.currentPacketPriorityAkaTrafficId)) {

                    OutgoingLinkInfo& linkInfo =
                        outgoingLinkInfoMap[
                            AddressAndTrafficIdMapKey(
                                accessCategoryInfo.currentPacketsDestinationMacAddress,
                                accessCategoryInfo.currentPacketPriorityAkaTrafficId)];

                    // Using Block Ack Request as ADDBA (Add Block Ack Session Request)

                    linkInfo.blockAckRequestNeedsToBeSent = true;
                    linkInfo.blockAckSessionIsEnabled = true;

                    // This makes the initial Block Ack Request to start the session
                    // Block Ack window on the next data/management frame.

                    linkInfo.lastDroppedFrameSequenceNumber = linkInfo.outgoingSequenceNumber;

                    return;
                }//if//

                (*this).BuildAggregateFrameFromCurrentFrame(accessCategoryIndex);

            }//if//

            return;

        }//while//
    }//for//

    assert(!wasRetrieved);

}//RetrievePacketFromNetworkLayerForAccessCategory//



inline
void Dot11Mac::RequeueBufferedPacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority priority,
    const EtherTypeField etherType,
    const SimTime& timestamp,
    const unsigned int txRetryCount)
{
    packetPtr->DeleteHeader(sizeof(QosDataFrameHeader));
    networkOutputQueuePtr->RequeueAtFront(
        packetPtr, nextHopAddress, priority, etherType, timestamp, txRetryCount);

}//RequeueBufferedPacket//



inline
void Dot11Mac::RequeueManagementFrame(unique_ptr<Packet>& framePtr)
{
    (*this).managementFrameQueue.push_front(move(framePtr));

}//RequeueManagementFrame//



inline
void Dot11Mac::RequeueBufferedPackets()
{
    for(unsigned int i = 0; (i < accessCategories.size()); i++) {
        EdcaAccessCategoryInfo& info = accessCategories[i];

        if (info.currentAggregateFramePtr != nullptr) {
            for (int j = static_cast<int>(info.currentAggregateFramePtr->size() - 1); (j > 0); j--) {
                (*this).RequeueBufferedPacket(
                    info.currentAggregateFramePtr->at(j),
                    info.currentPacketsNextHopNetworkAddress,
                    info.currentPacketPriorityAkaTrafficId,
                    info.currentPacketsEtherType,
                    info.currentPacketsTimestamp,
                    0);
            }//for//

            info.currentAggregateFramePtr.reset();

        }//if//

        if (info.currentPacketPtr != nullptr) {
            // Requeue packet and set TX count to 0.
            if (!info.currentPacketIsAManagementFrame) {
                (*this).RequeueBufferedPacket(
                    info.currentPacketPtr,
                    info.currentPacketsNextHopNetworkAddress,
                    info.currentPacketPriorityAkaTrafficId,
                    info.currentPacketsEtherType,
                    info.currentPacketsTimestamp,
                    0);
            }
            else {
                // Just Delete management frames.

                info.currentPacketPtr.reset();
            }//if//
        }//if//
    }//for//
}//RequeueBufferedDataPackets//



//--------------------------------------------------------------------------------------------------

inline
void Dot11Mac::DoubleTheContentionWindowAndPickANewBackoff(const unsigned int accessCategoryIndex)
{
    using std::min;

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    // Double contention window up to the maximum size.
    // (Actual sequence is CW(n) = (2^n - 1) ) {1,3,7,15,31,63,...}

    accessCategoryInfo.currentContentionWindowSlots =
        min(((accessCategoryInfo.currentContentionWindowSlots * 2) + 1) ,
            accessCategoryInfo.maxContentionWindowSlots);

    (*this).RecalcRandomBackoff(accessCategoryInfo);

}//DoubleTheContentionWindowAndPickANewBackoff//


//--------------------------------------------------------------------------------------------------

inline
void Dot11Mac::PerformInternalCollisionBackoff(const unsigned int accessCategoryIndex)
{
    (*this).DoubleTheContentionWindowAndPickANewBackoff(accessCategoryIndex);

}//PerformInternalCollisionBackoff//



//-------------------------------------------------------------------------------------------------

inline
DurationField ConvertTimeToDurationFieldValue(const SimTime& duration)
{
    using std::min;

    // Round up to nearest us and limit to maximum value.
    return (
        min(MaxDurationFieldValue, DurationField((duration + (MICRO_SECOND-1)) / MICRO_SECOND)));
}

inline
SimTime ConvertDurationFieldValueToTime(const DurationField& durationField)
{
    return (durationField * MICRO_SECOND);
}


inline
SimTime Dot11Mac::CalculateFrameDuration(
    const unsigned int frameWithMacHeaderSizeBytes,
    const TransmissionParameters& txParameters) const
{
    return (
        physicalLayer.CalculateFrameTransmitDuration(frameWithMacHeaderSizeBytes, txParameters));
}


inline
SimTime Dot11Mac::CalculateFrameDurationWithoutPhyHeader(
    const unsigned int frameWithMacHeaderSizeBytes,
    const TransmissionParameters& txParameters) const
{
    return (
        physicalLayer.CalculateFrameDataDuration(frameWithMacHeaderSizeBytes, txParameters));
}//CalculateFrameDurationWithoutPhyHeader//


inline
SimTime Dot11Mac::CalculateNavDurationForRtsFrame(
    const MacAddress& destinationMacAddress,
    const unsigned int packetWithMacHeaderSizeBytes,
    const TransmissionParameters& dataFrameTxParameters) const
{
    // Assume same adaptive modulation subsystems.  Alternative would be conservative
    // maximums.

    TransmissionParameters ctsTxParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
        destinationMacAddress,
        ctsTxParameters);

    const SimTime ctsFrameDuration =
        CalculateFrameDuration(sizeof(ClearToSendFrame), ctsTxParameters);

    const SimTime dataFrameDuration =
        CalculateFrameDuration(packetWithMacHeaderSizeBytes, dataFrameTxParameters);

    TransmissionParameters ackTxParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForAckFrame(
        destinationMacAddress,
        dataFrameTxParameters,
        ackTxParameters);

    SimTime ackFrameDuration;

    if (!ahUseOptimizedNdpControlFrames) {

        ackFrameDuration =
            CalculateFrameDuration(sizeof(AcknowledgementAkaAckFrame), ackTxParameters);
    }
    else {
        // NDP Control frames puts (replaces) MAC frame data in the PHY Header "SIG" field.

        ackFrameDuration = CalculateFrameDuration(0, ackTxParameters);

    }//if//


    return
        (aShortInterframeSpaceTime +
         ctsFrameDuration +
         aShortInterframeSpaceTime +
         dataFrameDuration +
         aShortInterframeSpaceTime +
         ackFrameDuration);

}//CalculateNavDurationForRtsFrame//


inline
SimTime Dot11Mac::CalculateNavDurationForCtsFrame(
    const DurationField& durationFromRtsFrame,
    const MacAddress& destinationMacAddress) const
{
    SimTime timeTypedDurationFromRtsFrame = SimTime(MICRO_SECOND * durationFromRtsFrame);

    const unsigned int clearToSendFrameSizeBytes = sizeof(ClearToSendFrame);

    TransmissionParameters txParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
        destinationMacAddress,
        txParameters);

    const SimTime clearToSendFrameDuration =
        CalculateFrameDuration(clearToSendFrameSizeBytes, txParameters);

    return (timeTypedDurationFromRtsFrame - (aShortInterframeSpaceTime + clearToSendFrameDuration));

}//CalculateNavDurationForCtsFrame//


inline
SimTime Dot11Mac::CalculateNavDurationForAckedDataFrame(
    const TransmissionParameters& txParameters) const
{
    // NDP Control frames puts (replaces) MAC frame data into the PHY Header "SIG" field.

    unsigned int ackFrameSizeBytes = 0;
    if (!ahUseOptimizedNdpControlFrames) {
        ackFrameSizeBytes = sizeof(AcknowledgementAkaAckFrame);
    }//if//

    const SimTime ackFrameDuration = CalculateFrameDuration(ackFrameSizeBytes, txParameters);

    // NAV Duration is the ACK (and a SIFs)

    return (aShortInterframeSpaceTime + ackFrameDuration);

}//CalculateNavDurationForAckedDataFrame//


inline
SimTime Dot11Mac::CalculateNavDurationForAckedAggregateDataFrame(
    const TransmissionParameters& txParameters) const
{
    const unsigned int blockAckFrameSizeBytes = sizeof(BlockAcknowledgementFrame);

    const SimTime blockAckFrameDuration = CalculateFrameDuration(blockAckFrameSizeBytes, txParameters);

    // NAV Duration is the ACK (and a SIFs)

    return (aShortInterframeSpaceTime + blockAckFrameDuration);

}//CalculateNavDurationForAckedAggregateDataFrame//


inline
SimTime Dot11Mac::CalculateNextFrameSequenceDuration(const unsigned int accessCategoryIndex) const
{
    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

    unsigned int frameSizeBytes = 0;
    MacAddress destinationMacAddress;
    TransmissionParameters txParameters;

    if (accessCategoryInfo.currentPacketPtr != nullptr) {
        destinationMacAddress = accessCategoryInfo.currentPacketsDestinationMacAddress;

        theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
            destinationMacAddress, txParameters);

        frameSizeBytes = accessCategoryInfo.currentPacketPtr->LengthBytes();

    }
    else if (accessCategoryInfo.currentAggregateFramePtr != nullptr) {

        // This situation should only occur when retrying an aggregate frame.
        // Warning:  This calculation is only approximate (low) for this case.
        // (Jay): Check (NAV) durations for block acked sequences.

        destinationMacAddress = accessCategoryInfo.currentPacketsDestinationMacAddress;

        theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
            destinationMacAddress, txParameters);

        for(unsigned int i = 0; (i < accessCategoryInfo.currentAggregateFramePtr->size()); i++) {
            frameSizeBytes += (*accessCategoryInfo.currentAggregateFramePtr)[i]->LengthBytes();
        }//for//
    }
    else if ((accessCategoryIndex == accessCategoryIndexForManagementFrames) && (!managementFrameQueue.empty())) {

        const Packet& aPacket = *managementFrameQueue.front();
        frameSizeBytes = aPacket.LengthBytes();

        const CommonFrameHeader& frameHeader =
            aPacket.GetAndReinterpretPayloadData<CommonFrameHeader>();

        destinationMacAddress = frameHeader.receiverAddress;

        theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
            destinationMacAddress, txParameters);
    }
    else {
        for(unsigned int i = 0; (i < accessCategoryInfo.listOfPriorities.size()); i++) {
            PacketPriority priority = accessCategoryInfo.listOfPriorities[i];

            if (networkOutputQueuePtr->HasPacketWithPriority(priority)) {
                frameSizeBytes =
                    (networkOutputQueuePtr->TopPacket(priority).LengthBytes() +
                     sizeof(QosDataFrameHeader));

                bool macAddressWasResolved;

                theMacAddressResolverPtr->GetMacAddress(
                    networkOutputQueuePtr->NextHopAddressForTopPacket(priority),
                    networkLayerPtr->GetSubnetMask(interfaceIndex),
                    macAddressWasResolved,
                    destinationMacAddress);

                if (!macAddressWasResolved) {
                    // Next packet is not deliverable so give up.
                    frameSizeBytes = 0;
                }//if//

                theAdaptiveRateControllerPtr->GetDataRateInfoForDataFrameToStation(
                    destinationMacAddress, txParameters);

                break;
            }//if//
        }//for//
    }//if//

    if (frameSizeBytes == 0) {
        return (ZERO_TIME);
    }//if//

    SimTime extraTimeBeforeFrame = ZERO_TIME;
    if (!accessCategoryInfo.currentPacketsDestinationMacAddress.IsABroadcastOrAMulticastAddress()) {
        extraTimeBeforeFrame = aShortInterframeSpaceTime;
    }//if//

    const SimTime frameDuration = CalculateFrameDuration(frameSizeBytes, txParameters);

    if (destinationMacAddress.IsABroadcastOrAMulticastAddress()) {
        return (extraTimeBeforeFrame + frameDuration);
    }
    else {
        return
            (extraTimeBeforeFrame +
             frameDuration +
             CalculateNavDurationForAckedDataFrame(txParameters));
    }//if//

}//CalculateNextFrameSequenceDuration//



inline
void Dot11Mac::AddExtraNavDurationToPacketForNextFrameIfInATxop(
    const unsigned int accessCategoryIndex,
    const SimTime& frameTransmissionEndTime,
    Packet& aFrame) const
{
    assert(currentTransmitOpportunityAkaTxopEndTime != ZERO_TIME);

    SimTime nextFrameSequenceDuration = CalculateNextFrameSequenceDuration(accessCategoryIndex);
    if (nextFrameSequenceDuration == ZERO_TIME) {
        // Nothing To Send next.
        return;
    }//if//

    CommonFrameHeader& header = aFrame.GetAndReinterpretPayloadData<CommonFrameHeader>();

    const SimTime currentFrameSequenceEndTime =
        frameTransmissionEndTime + ConvertDurationFieldValueToTime(header.duration);

    if ((currentFrameSequenceEndTime + nextFrameSequenceDuration) > currentTransmitOpportunityAkaTxopEndTime) {
        // Not enough time left for next frame sequence.
        return;
    }//if//

    header.duration += ConvertTimeToDurationFieldValue(nextFrameSequenceDuration);

}//AddExtraNavDurationToPacketForNextFrameIfInATxop//



//-------------------------------------------------------------------------------------------------


inline
void Dot11Mac::CalculateAndSetTransmitOpportunityDurationAkaTxop(const size_t accessCategoryIndex)
{
    EdcaAccessCategoryInfo& accessCategoryInfo = (*this).accessCategories.at(accessCategoryIndex);

    (*this).currentTransmitOpportunityAckedFrameCount = 0;

    accessCategoryInfo.transmitOpportunityDurationAkaTxop = ZERO_TIME;

    if (macSchedulerPtr.get() != nullptr) {
        accessCategoryInfo.transmitOpportunityDurationAkaTxop =
            macSchedulerPtr->CalculateTxopDuration(accessCategoryIndex);

    }//if//

    if (accessCategoryInfo.transmitOpportunityDurationAkaTxop == ZERO_TIME) {
        (*this).currentTransmitOpportunityAkaTxopEndTime = ZERO_TIME;
    }
    else {
        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

        (*this).currentTransmitOpportunityAkaTxopStartTime = currentTime;
        (*this).currentTransmitOpportunityAkaTxopEndTime =
            currentTime + accessCategoryInfo.transmitOpportunityDurationAkaTxop;

        if (currentTransmitOpportunityAkaTxopEndTime > ahRestrictedAccessWindowEndTime) {
            (*this).currentTransmitOpportunityAkaTxopEndTime = ahRestrictedAccessWindowEndTime;
        }//if//

    }//if//

}//CalculateAndSetTransmitOpportunityDurationAkaTxop//

//-------------------------------------------------------------------------------------------------


inline
void Dot11Mac::TransmitAnAggregateFrame(
    const unsigned int accessCategoryIndex,
    const TransmissionParameters& txParameters,
    const double& transmitPowerDbm,
    const SimTime& delayUntilTransmitting)
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);
    assert(accessCategoryInfo.currentAggregateFramePtr != nullptr);

    const DurationField navDurationField =
        ConvertTimeToDurationFieldValue(CalculateNavDurationForAckedAggregateDataFrame(txParameters));


    if (accessCategoryInfo.currentAggregateFrameIsAMpduAggregate) {
        for(unsigned int i = 0; (i < accessCategoryInfo.currentAggregateFramePtr->size()); i++) {
            Packet& aPacket = (*(*accessCategoryInfo.currentAggregateFramePtr)[i]);

            CommonFrameHeader& header =
                aPacket.GetAndReinterpretPayloadData<CommonFrameHeader>(
                    sizeof(MpduDelimiterFrame));

            header.duration = navDurationField;

        }//for//
    }
    else {
        CommonFrameHeader& header =
            accessCategoryInfo.currentAggregateFramePtr->front()->
                GetAndReinterpretPayloadData<CommonFrameHeader>();

        header.duration = navDurationField;

    }//if//

    (*this).accessCategoryIndexForLastSentFrame = accessCategoryIndex;
    (*this).macState = WaitingForAckState;
    (*this).lastSentFrameWasAn = SentFrameType::AggregateFrame;

    OutputTraceAndStatsForAggregatedFrameTransmission(accessCategoryIndex);

    physicalLayer.TransmitAggregateFrame(
        accessCategoryInfo.currentAggregateFramePtr,
        accessCategoryInfo.currentAggregateFrameIsAMpduAggregate,
        txParameters,
        transmitPowerDbm,
        delayUntilTransmitting);

}//TransmitAnAggregateFrame//



inline
void Dot11Mac::TransmitABlockAckRequest(
    const MacAddress& destinationAddress,
    const PacketPriority& trafficId,
    const SimTime& delayUntilTransmitting)
{
    const OutgoingLinkInfo& linkInfo =
        outgoingLinkInfoMap[AddressAndTrafficIdMapKey(destinationAddress, trafficId)];

    unique_ptr<Packet> frameToSendPtr =
        CreateABlockAckRequestFrame(
            destinationAddress,
            trafficId,
            (linkInfo.lastDroppedFrameSequenceNumber+1));

    const double transmitPowerDbm =
        theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(destinationAddress);

    TransmissionParameters txParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
        destinationAddress, txParameters);

    CommonFrameHeader& header =
        frameToSendPtr->GetAndReinterpretPayloadData<CommonFrameHeader>();

    header.duration =
        ConvertTimeToDurationFieldValue(
            CalculateNavDurationForAckedAggregateDataFrame(txParameters));

    (*this).macState = WaitingForAckState;
    (*this).lastSentFrameWasAn = SentFrameType::BlockAckRequestFrame;

    physicalLayer.TransmitFrame(
        frameToSendPtr,
        txParameters,
        transmitPowerDbm,
        delayUntilTransmitting);

    OutputTraceAndStatsForBlockAckRequestFrameTransmission();

}//TransmitABlockAckRequest//


inline
void Dot11Mac::TransmitAFrame(
    const unsigned int accessCategoryIndex,
    const bool doNotRequestToSendAkaRts,
    const SimTime& delayUntilTransmitting,
    bool& packetHasBeenSentToPhy)
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    lastTransmittedFrameWasABeacon = false;

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

    if ((accessCategoryInfo.currentPacketPtr == nullptr) &&
        (accessCategoryInfo.currentAggregateFramePtr == nullptr)) {

        if (accessCategoryIndex == accessCategoryIndexForManagementFrames) {

            while (!managementFrameQueue.empty()) {

                const ManagementFrameHeader& frameHeader =
                    managementFrameQueue.front()->GetAndReinterpretPayloadData<ManagementFrameHeader>();

                if ((GetOperationMode() != ApMode) ||
                    (!apControllerPtr->StationIsAsleep(frameHeader.header.receiverAddress))) {

                    accessCategoryInfo.currentPacketPtr = move(managementFrameQueue.front());
                    managementFrameQueue.pop_front();
                    accessCategoryInfo.currentPacketIsAManagementFrame = true;
                    accessCategoryInfo.currentPacketPriorityAkaTrafficId = maxPacketPriority;
                    accessCategoryInfo.currentPacketsDestinationMacAddress =
                        frameHeader.header.receiverAddress;
                    accessCategoryInfo.currentShortFrameRetryCount = 0;
                    accessCategoryInfo.currentLongFrameRetryCount = 0;
                    accessCategoryInfo.currentPacketSequenceNumber =
                        frameHeader.theSequenceControlField.sequenceNumber;

                    // 802.11ah
                    (*this).lastTransmittedFrameWasABeacon =
                        (frameHeader.header.theFrameControlField.frameTypeAndSubtype == BEACON_FRAME_TYPE_CODE);

                    // || (frameHeader.theFrameControlField.frameTypeAndSubtype ==
                    //     AH_SHORT_BEACON_FRAME_TYPE_CODE));

                    break;

                }//if//

                unique_ptr<Packet> aManagementFramePtr = move(managementFrameQueue.front());

                apControllerPtr->BufferManagementFrameForSleepingStation(
                    frameHeader.header.receiverAddress, aManagementFramePtr, currentTime);

                managementFrameQueue.pop_front();

                assert(accessCategoryInfo.currentPacketPtr == nullptr);

            }//while//

        }//if//

        if (accessCategoryInfo.currentPacketPtr == nullptr) {

            accessCategoryInfo.currentPacketIsAManagementFrame = false;

            bool wasRetrieved;

            (*this).RetrievePacketFromNetworkLayerForAccessCategory(accessCategoryIndex, wasRetrieved);

            if (!wasRetrieved) {
                packetHasBeenSentToPhy = false;
                accessCategoryInfo.hasPacketToSend = false;
                assert(accessCategoryInfo.currentPacketPtr == nullptr);
                return;
            }//if//
        }//if//
    }//if//


    TransmissionParameters txParameters;

    if (accessCategoryInfo.currentPacketIsAManagementFrame) {

        theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
            accessCategoryInfo.currentPacketsDestinationMacAddress, txParameters);

        assert(txParameters.channelBandwidthMhz == physicalLayer.GetBaseChannelBandwidthMhz());
    }
    else {
        theAdaptiveRateControllerPtr->GetDataRateInfoForDataFrameToStation(
            accessCategoryInfo.currentPacketsDestinationMacAddress, txParameters);
    }//if//

    const double transmitPowerDbm =
        theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(
            accessCategoryInfo.currentPacketsDestinationMacAddress);

    if (accessCategoryInfo.currentPacketPtr == nullptr) {
        if ((protectAggregateFramesWithSingleAckedFrame) &&
            (accessCategoryInfo.currentAggregateFrameIsAMpduAggregate) &&
            (currentTransmitOpportunityAckedFrameCount == 0)) {

            // Only retry first frame of MPDU aggregate frame for first acked TX of a TXOP.

            accessCategoryInfo.currentPacketPtr =
                move(accessCategoryInfo.currentAggregateFramePtr->front());

            accessCategoryInfo.currentAggregateFramePtr->erase(
                accessCategoryInfo.currentAggregateFramePtr->begin());

            if (accessCategoryInfo.currentAggregateFramePtr->empty()) {
                accessCategoryInfo.currentAggregateFramePtr.reset();
            }//if//

            if (accessCategoryInfo.currentAggregateFrameIsAMpduAggregate) {
                RemoveMpduDelimiterAndPaddingFromFrame(*accessCategoryInfo.currentPacketPtr);
            }
            else {
                const QosDataFrameHeader copyOfHeader =
                    accessCategoryInfo.currentPacketPtr->GetAndReinterpretPayloadData<QosDataFrameHeader>();

                accessCategoryInfo.currentPacketPtr->DeleteHeader(sizeof(QosDataFrameHeader));
                accessCategoryInfo.currentPacketPtr->AddPlainStructHeader(copyOfHeader);

                // Move header to next frame and get new sequence number.

                if (accessCategoryInfo.currentAggregateFramePtr != nullptr) {
                    accessCategoryInfo.currentAggregateFramePtr->front()->AddPlainStructHeader(copyOfHeader);

                    QosDataFrameHeader& dataFrameHeader =
                        accessCategoryInfo.currentPacketPtr->GetAndReinterpretPayloadData<QosDataFrameHeader>();

                    unsigned short int sequenceNumber;

                    (*this).GetNewSequenceNumber(
                        dataFrameHeader.header.receiverAddress,
                        dataFrameHeader.qosControlField.trafficId,
                        true,
                        /*out*/sequenceNumber);

                    dataFrameHeader.theSequenceControlField.sequenceNumber = sequenceNumber;
                }//if//
            }//if//

            QosDataFrameHeader& header =
                accessCategoryInfo.currentPacketPtr->GetAndReinterpretPayloadData<QosDataFrameHeader>();

            assert(header.header.theFrameControlField.frameTypeAndSubtype == QOS_DATA_FRAME_TYPE_CODE);

            accessCategoryInfo.currentPacketIsAManagementFrame = false;
            accessCategoryInfo.currentPacketSequenceNumber =
                header.theSequenceControlField.sequenceNumber;

            if (accessCategoryInfo.currentPacketPtr->LengthBytes() < rtsThresholdSizeBytes) {
                accessCategoryInfo.currentShortFrameRetryCount =
                    accessCategoryInfo.currentAggregateFrameRetryCount;
                accessCategoryInfo.currentLongFrameRetryCount = 0;
            }
            else {
                accessCategoryInfo.currentShortFrameRetryCount = 0;
                accessCategoryInfo.currentLongFrameRetryCount =
                    accessCategoryInfo.currentAggregateFrameRetryCount;
            }//if//
        }
        else {

            (*this).lastSentDataFrameDestinationMacAddress =
                accessCategoryInfo.currentPacketsDestinationMacAddress;

            (*this).TransmitAnAggregateFrame(
                accessCategoryIndex,
                txParameters,
                transmitPowerDbm,
                delayUntilTransmitting);

            packetHasBeenSentToPhy = true;
            return;

        }//if//
    }//if//

    if (NeedToSendABlockAckRequest(
        accessCategoryInfo.currentPacketsDestinationMacAddress,
        accessCategoryInfo.currentPacketPriorityAkaTrafficId)) {

        (*this).accessCategoryIndexForLastSentFrame = accessCategoryIndex;
        (*this).lastSentDataFrameDestinationMacAddress =
            accessCategoryInfo.currentPacketsDestinationMacAddress;

        (*this).TransmitABlockAckRequest(
            accessCategoryInfo.currentPacketsDestinationMacAddress,
            accessCategoryInfo.currentPacketPriorityAkaTrafficId,
            delayUntilTransmitting);

        packetHasBeenSentToPhy = true;

        return;
    }//if//

    const unsigned int frameSizeBytes = accessCategoryInfo.currentPacketPtr->LengthBytes();

    if (frameSizeBytes < rtsThresholdSizeBytes) {
        (*this).lastSentFrameWasAn = SentFrameType::ShortFrame;
    }
    else {
        (*this).lastSentFrameWasAn = SentFrameType::LongFrame;
    }//if//

    bool frameNeedsToBeAcked = true;

    if (accessCategoryInfo.currentPacketsDestinationMacAddress.IsABroadcastOrAMulticastAddress()) {

        frameNeedsToBeAcked = false;

        if (accessCategoryInfo.currentPacketIsAManagementFrame) {
            OutputTraceAndStatsForManagementFrameTransmission(accessCategoryIndex);
        }
        else {
            OutputTraceAndStatsForBroadcastDataFrameTransmission(accessCategoryIndex);

        }//if//

        // A transmission that does not require an immediate frame as a response is defined as a successful transmission.
        // Next Packet => Reset Contention window.

        accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

        // A transmission that does not require an immediate frame as a response is defined as a successful transmission.
        // Next Packet => Reset Contention window.

        accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

        macState = BusyMediumState;
    }
    else if ((doNotRequestToSendAkaRts) || (frameSizeBytes < rtsThresholdSizeBytes)) {

        bool isARetry;
        if (frameSizeBytes < rtsThresholdSizeBytes) {
            isARetry = (accessCategoryInfo.currentShortFrameRetryCount > 0);
        }
        else {
            isARetry = (accessCategoryInfo.currentLongFrameRetryCount > 0);
        }//if//

        const SimTime navDuration = CalculateNavDurationForAckedDataFrame(txParameters);

        CommonFrameHeader& frameHeader =
            accessCategoryInfo.currentPacketPtr->GetAndReinterpretPayloadData<CommonFrameHeader>();

        frameHeader.theFrameControlField.isRetry = isARetry;
        frameHeader.duration = ConvertTimeToDurationFieldValue(navDuration);

        if (accessCategoryInfo.currentPacketIsAManagementFrame) {
            OutputTraceAndStatsForManagementFrameTransmission(accessCategoryIndex);
        }
        else {
            OutputTraceAndStatsForUnicastDataFrameTransmission(accessCategoryIndex);
        }//if//

        (*this).lastSentDataFrameDestinationMacAddress = frameHeader.receiverAddress;
        (*this).macState = WaitingForAckState;
    }
    else {
        // RTS Special Case.

        (*this).lastSentFrameWasAn = SentFrameType::RequestToSendFrame;

        RequestToSendFrame rtsFrame;

        rtsFrame.header.theFrameControlField.frameTypeAndSubtype = RTS_FRAME_TYPE_CODE;
        rtsFrame.header.theFrameControlField.isRetry = (accessCategoryInfo.currentShortFrameRetryCount > 0);

        rtsFrame.header.duration =
            ConvertTimeToDurationFieldValue(
                CalculateNavDurationForRtsFrame(
                    rtsFrame.header.receiverAddress,
                    accessCategoryInfo.currentPacketPtr->LengthBytes(),
                    txParameters));

        rtsFrame.header.receiverAddress = accessCategoryInfo.currentPacketsDestinationMacAddress;
        rtsFrame.transmitterAddress = myMacAddress;

        unique_ptr<Packet> rtsPacketToSendPtr =
            Packet::CreatePacket(*simEngineInterfacePtr, rtsFrame);

        (*this).accessCategoryIndexForLastSentFrame = accessCategoryIndex;

        TransmissionParameters rtsTxParameters;

        theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
            accessCategoryInfo.currentPacketsDestinationMacAddress, rtsTxParameters);

        if (currentTransmitOpportunityAkaTxopEndTime != ZERO_TIME) {
            SimTime frameTransmissionEndTime =
                (currentTime + delayUntilTransmitting +
                 CalculateFrameDuration(rtsPacketToSendPtr->LengthBytes(), rtsTxParameters));

            AddExtraNavDurationToPacketForNextFrameIfInATxop(
                accessCategoryIndex, frameTransmissionEndTime, *rtsPacketToSendPtr);
        }//if//

        physicalLayer.TransmitFrame(
            rtsPacketToSendPtr,
            rtsTxParameters,
            transmitPowerDbm,
            delayUntilTransmitting);

        macState = WaitingForCtsState;

        OutputTraceAndStatsForRtsFrameTransmission(accessCategoryIndex);

        packetHasBeenSentToPhy = true;

        return;

    }//if//


    // Going to move frame down to Phy, but will retrieve it later if necessary (retry).

    unique_ptr<Packet> packetToSendPtr = move(accessCategoryInfo.currentPacketPtr);

    if ((currentTransmitOpportunityAkaTxopEndTime != ZERO_TIME) ||
        (ahRestrictedAccessWindowEndTime != INFINITE_TIME)) {

        // Also adding in propagation delay parameter (constant) to insure that
        // frames are fully propagated by the end of the contention period.

        SimTime frameTransmissionSequenceEndTime =
            (currentTime + delayUntilTransmitting +
             CalculateFrameDuration(packetToSendPtr->LengthBytes(), txParameters) +
             physicalLayer.GetAirPropagationTimeDuration());

        if (frameNeedsToBeAcked) {
            frameTransmissionSequenceEndTime +=
                (CalculateNavDurationForAckedDataFrame(txParameters) +
                physicalLayer.GetAirPropagationTimeDuration());
        }//if//

        if (frameTransmissionSequenceEndTime > ahRestrictedAccessWindowEndTime) {

            // Enforce 802.11ad Contention Period (don't transmit).

            if (lastSentFrameWasAn != SentFrameType::RequestToSendFrame) {

                // Not Sending => Move it back to buffer.

                accessCategoryInfo.currentPacketPtr = move(packetToSendPtr);
            }//if//

            (*this).DoTransmissionTooLongForRawPeriodAction(accessCategoryIndex);
            return;
        }//if//

        if (currentTransmitOpportunityAkaTxopEndTime != ZERO_TIME) {

            AddExtraNavDurationToPacketForNextFrameIfInATxop(
                    accessCategoryIndex, frameTransmissionSequenceEndTime, *packetToSendPtr);
        }//if//
    }//if//

    (*this).accessCategoryIndexForLastSentFrame = accessCategoryIndex;

    physicalLayer.TransmitFrame(
        packetToSendPtr,
        txParameters,
        transmitPowerDbm,
        delayUntilTransmitting);

    packetHasBeenSentToPhy = true;

}//TransmitAFrame//





inline
void Dot11Mac::ProcessInterframeSpaceAndBackoffTimeout()
{
    // Try to send a frame

    OutputTraceForIfsAndBackoffExpiration();

    assert(macState == WaitingForIfsAndBackoffState);

    SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    // Go through Access Categories in reverse order (high priority to low priority).

    bool packetHasBeenSentToThePhy = false;
    for(int i = (numberAccessCategories - 1); (i >= 0); i--) {

        const unsigned int accessCategoryIndex = static_cast<unsigned int>(i);

        EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

        if (accessCategoryInfo.currentNonExtendedBackoffDuration != INFINITE_TIME) {
            SimTime elapsedTime = currentTime - accessCategoryInfo.ifsAndBackoffStartTime;

            accessCategoryInfo.currentNumOfBackoffSlots -=
               NumberBackoffSlotsUsedDuringElapsedTime(accessCategoryInfo, elapsedTime);

            assert(accessCategoryInfo.currentNumOfBackoffSlots >= 0);

            if ((accessCategoryInfo.currentNumOfBackoffSlots == 0) &&
                (!packetHasBeenSentToThePhy)) {

                accessCategoryInfo.currentNonExtendedBackoffDuration = INFINITE_TIME;
                accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff = false;

                //If a packet should be sent exists, change hasPacketToSend.
                if (accessCategoryInfo.currentPacketPtr == nullptr) {
                    accessCategoryInfo.hasPacketToSend =
                        ThereAreQueuedPacketsForAccessCategory(accessCategoryIndex);
                }//if//

                if (accessCategoryInfo.hasPacketToSend) {
                   (*this).CalculateAndSetTransmitOpportunityDurationAkaTxop(accessCategoryIndex);

                   (*this).TransmitAFrame(
                       accessCategoryIndex, false, aRxTxTurnaroundTime, packetHasBeenSentToThePhy);
                }
                else {
                    // Post-transmission forced backoff only, no packets to send.
                }//if//
            }
            else {
                if (accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff) {
                    // Failed to jump on medium because another Access Category got there first.
                    // Must start normal backoff process.

                    accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff = false;
                    accessCategoryInfo.currentNonExtendedBackoffDuration = INFINITE_TIME;

                    (*this).StartPacketSendProcessForAnAccessCategory(accessCategoryIndex, true);
                }
                else if (accessCategoryInfo.currentNumOfBackoffSlots == 0) {
                    assert(packetHasBeenSentToThePhy);
                    // Another higher priority Access Category is transmitting.

                    (*this).PerformInternalCollisionBackoff(accessCategoryIndex);
                }
                else {

                    accessCategoryInfo.currentNonExtendedBackoffDuration =
                        CalculateNonExtendedBackoffDuration(accessCategoryInfo);
                }//if//

            }//if//
        }//if//
    }//for//

    if (!packetHasBeenSentToThePhy) {

        // Timeout was for post-transmission forced backoff (no actual packets to send).
        macState = TransientState;
        (*this).StartBackoffIfNecessary();

    }//if//

}//ProcessInterframeSpaceAndBackoffTimeout//


inline
void Dot11Mac::DropCurrentPacketAndGoToNextPacket(const unsigned int accessCategoryIndex)
{
    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    const MacAddress& destinationMacAddress =
        accessCategoryInfo.currentPacketsDestinationMacAddress;

    OutputTraceAndStatsForPacketRetriesExceeded(accessCategoryIndex);

    OutgoingLinkInfo& linkInfo =
        outgoingLinkInfoMap[
            AddressAndTrafficIdMapKey(
                destinationMacAddress,
                accessCategoryInfo.currentPacketPriorityAkaTrafficId)];

    linkInfo.lastDroppedFrameSequenceNumber = accessCategoryInfo.currentPacketSequenceNumber;

    if (linkInfo.blockAckSessionIsEnabled) {

        linkInfo.blockAckRequestNeedsToBeSent = true;

    }//if//

    if (!accessCategoryInfo.currentPacketIsAManagementFrame) {

        networkLayerPtr->ReceiveUndeliveredPacketFromMac(
            interfaceIndex,
            accessCategoryInfo.currentPacketPtr,
            accessCategoryInfo.currentPacketsNextHopNetworkAddress);
    }
    else {
        accessCategoryInfo.currentPacketPtr.reset();
    }//if//

    assert(accessCategoryInfo.currentPacketPtr == nullptr);

    accessCategoryInfo.hasPacketToSend = ThereAreQueuedPacketsForAccessCategory(accessCategoryIndex);

    accessCategoryInfo.currentShortFrameRetryCount = 0;
    accessCategoryInfo.currentLongFrameRetryCount = 0;

    // Next Packet => Reset Contention window.
    // Always do backoff procedure regardless of whether there is a packet to send.

    accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

    (*this).RecalcRandomBackoff(accessCategoryInfo);

}//DropCurrentPacketAndGoToNextPacket//



inline
void Dot11Mac::DropCurrentAggregateFrameAndGoToNextPacket(const unsigned int accessCategoryIndex)
{
    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    assert(accessCategoryInfo.currentAggregateFramePtr != nullptr);

    accessCategoryInfo.currentAggregateFrameRetryCount = 0;

    // Assuming sequence numbers are in order.

    const Packet& lastSubframe = *accessCategoryInfo.currentAggregateFramePtr->back();

    const QosDataFrameHeader& header =
        lastSubframe.GetAndReinterpretPayloadData<QosDataFrameHeader>(
            sizeof(MpduDelimiterFrame));

    const unsigned short int droppedFrameSequenceNumber = header.theSequenceControlField.sequenceNumber;

    vector<unique_ptr<Packet> >& subframes = *accessCategoryInfo.currentAggregateFramePtr;

    // For loop is useless for non-dummy unique_ptr.
    for(unsigned int i = 0; (i <subframes.size()); i++) {
        subframes[i].reset();
    }//for//

    accessCategoryInfo.currentAggregateFramePtr.reset();

    // Send Block Ack to shift sequence number
    // window at destination for dropped frame.

    OutgoingLinkInfo& linkInfo =
        outgoingLinkInfoMap[
            AddressAndTrafficIdMapKey(
                accessCategoryInfo.currentPacketsDestinationMacAddress,
                accessCategoryInfo.currentPacketPriorityAkaTrafficId)];

    assert(linkInfo.blockAckSessionIsEnabled);
    linkInfo.blockAckRequestNeedsToBeSent = true;
    linkInfo.lastDroppedFrameSequenceNumber = droppedFrameSequenceNumber;

    accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;
    (*this).RecalcRandomBackoff(accessCategoryInfo);

    //Future Idea // Obsolete G++ hack (combine statements after upgrade).
    //Future Idea
    //Future Idea unique_ptr<Packet> framePtr =
    //Future Idea     CreateABlockAckRequestFrame(
    //Future Idea         accessCategoryInfo.currentPacketsDestinationMacAddress,
    //Future Idea         accessCategoryInfo.currentPacketPriorityAkaTrafficId,
    //Future Idea         (droppedFrameSequenceNumber + 1));
    //Future Idea
    //Future Idea (*this).SendManagementFrame(framePtr);

}//DropCurrentAggregateFrameAndGoToNextPacket//




inline
void Dot11Mac::NeverReceivedClearToSendOrAcknowledgementAction()
{
    // Make sure that no longer in waiting for ACK or CTS state.

    assert((macState != WaitingForCtsState) && (macState != WaitingForAckState));

    if (WakeupTimerIsActive()) {
        (*this).CancelWakeupTimer();
    }//if//

    EdcaAccessCategoryInfo& accessCategoryInfo = (*this).accessCategories[accessCategoryIndexForLastSentFrame];

    if (lastSentFrameWasAn != SentFrameType::PowerSavePollResponse) {
        (*this).DoubleTheContentionWindowAndPickANewBackoff(accessCategoryIndexForLastSentFrame);
    }//if//

    OutputTraceForCtsOrAckTimeout();
    //CTS or ACK failed.
    theAdaptiveRateControllerPtr->NotifyAckFailed(accessCategoryInfo.currentPacketsDestinationMacAddress);

    switch (lastSentFrameWasAn) {

    case SentFrameType::RequestToSendFrame:
        accessCategoryInfo.currentShortFrameRetryCount++;
        if (accessCategoryInfo.currentShortFrameRetryCount >= shortFrameRetryLimit) {
            (*this).DropCurrentPacketAndGoToNextPacket(accessCategoryIndexForLastSentFrame);
        }//if//

        break;

    case SentFrameType::ShortFrame:
        physicalLayer.TakeOwnershipOfLastTransmittedFrame(accessCategoryInfo.currentPacketPtr);

        accessCategoryInfo.currentShortFrameRetryCount++;
        if (accessCategoryInfo.currentShortFrameRetryCount >= shortFrameRetryLimit) {
            (*this).DropCurrentPacketAndGoToNextPacket(accessCategoryIndexForLastSentFrame);
        }//if//
        break;

    case SentFrameType::LongFrame:
        physicalLayer.TakeOwnershipOfLastTransmittedFrame(accessCategoryInfo.currentPacketPtr);

        accessCategoryInfo.currentLongFrameRetryCount++;

        if (accessCategoryInfo.currentLongFrameRetryCount >= longFrameRetryLimit) {
            (*this).DropCurrentPacketAndGoToNextPacket(accessCategoryIndexForLastSentFrame);
        }//if//
        break;

    case SentFrameType::AggregateFrame:
        physicalLayer.TakeOwnershipOfLastTransmittedAggregateFrame(
            accessCategoryInfo.currentAggregateFramePtr);

        accessCategoryInfo.currentAggregateFrameRetryCount++;

        if (accessCategoryInfo.currentAggregateFrameRetryCount >= shortFrameRetryLimit) {
            assert(accessCategoryInfo.currentPacketPtr == nullptr);
            assert(!accessCategoryInfo.currentPacketIsAManagementFrame);

            (*this).DropCurrentAggregateFrameAndGoToNextPacket(accessCategoryIndexForLastSentFrame);

        }//if//
        break;

    case SentFrameType::BlockAckRequestFrame:

        accessCategoryInfo.currentShortFrameRetryCount++;
        if (accessCategoryInfo.currentShortFrameRetryCount >= shortFrameRetryLimit) {
            // Block Ack Request is being used like an RTS, if it fails to be sent
            // then the all the current dequeued DATA packets are thrown away
            // (Stops infinite Block Ack Requests).

            if (accessCategoryInfo.currentPacketPtr != nullptr) {
                (*this).DropCurrentPacketAndGoToNextPacket(accessCategoryIndexForLastSentFrame);
            }
            else if (accessCategoryInfo.currentAggregateFramePtr != nullptr) {
                (*this).DropCurrentAggregateFrameAndGoToNextPacket(accessCategoryIndexForLastSentFrame);
            }//if//
        }//if//

        break;

    case SentFrameType::PowerSavePollResponse:

        // Review this (Jay):
        assert(false && "Not implemented or unnecessary else"); abort();
        break;

    default:
        assert(false); abort();
        break;
    }//switch//

}//NeverReceivedClearToSendOrAcknowledgementAction//



inline
void Dot11Mac::ProcessClearToSendOrAcknowledgementTimeout()
{
    if (!physicalLayer.IsReceivingAFrame()) {
        // No possible ACK or CTS packet coming in.

        // Get out of the waiting for ACK/CTS state.
        (*this).macState = BusyMediumState;

        (*this).NeverReceivedClearToSendOrAcknowledgementAction();

        if (physicalLayer.ChannelIsClear()) {
            if (simEngineInterfacePtr->CurrentTime() < mediumReservedUntilTimeAkaNAV) {

                (*this).GoIntoWaitForExpirationOfVirtualCarrierSenseAkaNavState();
            }
            else {
                mediumBecameIdleTime = simEngineInterfacePtr->CurrentTime();
                macState = TransientState;
                (*this).StartBackoffIfNecessary();

            }//if//
        }//if//
    }//if//

}//ProcessClearToSendOrAcknowledgementTimeout//


//--------------------------------------------------------------------------------------------------



inline
void Dot11Mac::ProcessRequestToSendFrame(
    const RequestToSendFrame& rtsFrame,
    const TransmissionParameters& receivedFrameTxParameters)
{
    if ((macState == WaitingForCtsState) || (macState == WaitingForAckState)) {
        macState = BusyMediumState;
        (*this).NeverReceivedClearToSendOrAcknowledgementAction();
    }//if//

    if (simEngineInterfacePtr->CurrentTime() < mediumReservedUntilTimeAkaNAV) {
        // Virtual Carrier Sense is active, ignore RTS.
        return;
    }//if//

    ClearToSendFrame ctsFrame;

    ctsFrame.header.theFrameControlField.frameTypeAndSubtype = CTS_FRAME_TYPE_CODE;
    ctsFrame.header.receiverAddress = rtsFrame.transmitterAddress;
    ctsFrame.header.duration =
        ConvertTimeToDurationFieldValue(
            CalculateNavDurationForCtsFrame(
                rtsFrame.header.duration, ctsFrame.header.receiverAddress));

    unique_ptr<Packet> packetPtr = Packet::CreatePacket(*simEngineInterfacePtr, ctsFrame);

    const double transmitPowerDbm =
        theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(ctsFrame.header.receiverAddress);

    // Clear to send response, ensure not in a TXOP.

    (*this).currentTransmitOpportunityAkaTxopEndTime = ZERO_TIME;

    (*this).macState = CtsOrAckTransmissionState;

    OutputTraceForCtsFrameTransmission();

    physicalLayer.TransmitFrame(
        packetPtr,
        receivedFrameTxParameters,
        transmitPowerDbm,
        aShortInterframeSpaceTime);

}//ProcessRequestToSendFrame//



inline
void Dot11Mac::ProcessClearToSendFrame(const ClearToSendFrame& ctsFrame)
{
    if (macState != WaitingForCtsState) {
        // The request to send (RTS) has been canceled for some reason, ignore the CTS.
        return;
    }//if//

    // Cancel CTS timeout if it hasn't expired yet (fast CTS).

    if (!wakeupTimerEventTicket.IsNull()) {
        (*this).CancelWakeupTimer();
    }//if//

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndexForLastSentFrame];

    // Reset short retry count on successful RTS/CTS.

    accessCategoryInfo.currentShortFrameRetryCount = 0;

    bool packetWasSentToPhy;

    (*this).TransmitAFrame(
        accessCategoryIndexForLastSentFrame, true, aShortInterframeSpaceTime, packetWasSentToPhy);

    assert(packetWasSentToPhy);

}//ProcessClearToSendFrame//



inline
void Dot11Mac::ProcessPowerSavePollFrame(
    const PowerSavePollFrame& psPollFrame,
    const TransmissionParameters& pollFrameTxParameters)
{
    assert(GetOperationMode() == ApMode);

    unique_ptr<Packet> powerSavePollResponsePacketPtr;
    bool wasRetrieved;
    unsigned int retryTxCount;
    PacketPriority priority;
    EtherTypeField etherType;

    apControllerPtr->GetPowerSaveBufferedPacket(
        psPollFrame.transmitterAddress,
        wasRetrieved,
        powerSavePollResponsePacketPtr,
        retryTxCount,
        priority,
        etherType);

    if (!wasRetrieved) {
        (*this).SendAckFrame(psPollFrame.transmitterAddress, pollFrameTxParameters);
        return;
    }//if//

    (*this).lastSentFrameWasAn = SentFrameType::PowerSavePollResponse;
    (*this).lastSentDataFrameDestinationMacAddress = psPollFrame.transmitterAddress;
    (*this).powerSavePollResponsePacketTxCount = (retryTxCount + 1);

    const unsigned int frameSizeBytes =
        sizeof(QosDataFrameHeader) + powerSavePollResponsePacketPtr->LengthBytes();

    TransmissionParameters txParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForDataFrameToStation(
        psPollFrame.transmitterAddress,
        txParameters);

    QosDataFrameHeader dataFrameHeader;

    dataFrameHeader.header.theFrameControlField.frameTypeAndSubtype = QOS_DATA_FRAME_TYPE_CODE;
    dataFrameHeader.header.theFrameControlField.isRetry = (powerSavePollResponsePacketTxCount > 1);
    dataFrameHeader.header.duration =
        ConvertTimeToDurationFieldValue(CalculateNavDurationForAckedDataFrame(txParameters));

    dataFrameHeader.header.receiverAddress = psPollFrame.transmitterAddress;
    (*this).lastSentDataFrameDestinationMacAddress = dataFrameHeader.header.receiverAddress;

    unsigned short int sequenceNumber;

    (*this).GetNewSequenceNumber(
        psPollFrame.transmitterAddress,
        priority,
        true,
        /*out*/sequenceNumber);

    dataFrameHeader.theSequenceControlField.sequenceNumber = sequenceNumber;
    dataFrameHeader.transmitterAddress = myMacAddress;
    dataFrameHeader.qosControlField.trafficId = priority;
    dataFrameHeader.linkLayerHeader.etherType = HostToNet16(etherType);

    powerSavePollResponsePacketPtr->AddPlainStructHeader(dataFrameHeader);

    const double transmitPowerDbm =
        theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(psPollFrame.transmitterAddress);

    (*this).macState = WaitingForAckState;

    physicalLayer.TransmitFrame(
        powerSavePollResponsePacketPtr,
        txParameters,
        transmitPowerDbm,
        aShortInterframeSpaceTime);

}//ProcessPowerSavePollFrame//


//--------------------------------------------------------------------------------------------------

inline
bool Dot11Mac::BlockAckSessionIsEnabled(
    const MacAddress& destinationAddress,
    const PacketPriority& trafficId) const
{
    typedef map<AddressAndTrafficIdMapKey, OutgoingLinkInfo>::const_iterator IterType;

    AddressAndTrafficIdMapKey aKey(destinationAddress, trafficId);

    const IterType iter = outgoingLinkInfoMap.find(aKey);

    if (iter == outgoingLinkInfoMap.end()) {
        return false;
    }
    else {
        return (iter->second.blockAckSessionIsEnabled);
    }//if//

}//BlockAckSessionIsEnabled//


inline
void Dot11Mac::GetNewSequenceNumber(
    const MacAddress& destinationAddress,
    const PacketPriority& trafficId,
    const bool isNonBlockAckedFrame,
    unsigned short int& newSequenceNumber)
{
    typedef map<AddressAndTrafficIdMapKey, OutgoingLinkInfo>::iterator IterType;

    AddressAndTrafficIdMapKey aKey(destinationAddress, trafficId);

    const IterType iter = outgoingLinkInfoMap.find(aKey);

    if (iter == outgoingLinkInfoMap.end()) {
        outgoingLinkInfoMap[aKey].outgoingSequenceNumber = 1;
        outgoingLinkInfoMap[aKey].windowStartSequenceNumber = 1;

        newSequenceNumber = 1;
    }
    else {
        OutgoingLinkInfo& linkInfo = iter->second;
        unsigned short int& lastOutgoingSequenceNumber = linkInfo.outgoingSequenceNumber;
        IncrementTwelveBitSequenceNumber(lastOutgoingSequenceNumber);
        newSequenceNumber = lastOutgoingSequenceNumber;

        if (isNonBlockAckedFrame) {
            linkInfo.windowStartSequenceNumber = newSequenceNumber;
        }//if//

        assert(
            CalcTwelveBitSequenceNumberDifference(
                newSequenceNumber, linkInfo.windowStartSequenceNumber) < BlockAckBitMapNumBits);
    }//if//

}//GetNewSequenceNumber//


inline
unsigned int Dot11Mac::CalcNumberFramesLeftInSequenceNumWindow(
    const MacAddress& destinationAddress,
    const PacketPriority& trafficId) const
{
    const OutgoingLinkInfo& linkInfo =
        outgoingLinkInfoMap.find(AddressAndTrafficIdMapKey(destinationAddress, trafficId))->second;

    const int diff =
        CalcTwelveBitSequenceNumberDifference(
            linkInfo.outgoingSequenceNumber, linkInfo.windowStartSequenceNumber);

    assert((diff >= 0) && (diff <= BlockAckBitMapNumBits));

    return (BlockAckBitMapNumBits - diff - 1);

}//CalcNumberFramesLeftInSequenceNumWindow//



inline
void Dot11Mac::SetSequenceNumberWindowStart(
    const MacAddress& destinationAddress,
    const PacketPriority& trafficId,
    const unsigned short int sequenceNum)
{
    typedef map<AddressAndTrafficIdMapKey, OutgoingLinkInfo>::iterator IterType;

    const IterType iter =
        outgoingLinkInfoMap.find(AddressAndTrafficIdMapKey(destinationAddress, trafficId));

    assert(iter != outgoingLinkInfoMap.end());
    OutgoingLinkInfo& linkInfo = iter->second;

    linkInfo.windowStartSequenceNumber = sequenceNum;
}



//--------------------------------------------------------------------------------------------------


inline
void Dot11Mac::ProcessAcknowledgementFrame(const AcknowledgementAkaAckFrame& ackFrame)
{
    if (macState != WaitingForAckState) {
        // Received an ACK, but was not expecting it ==> ignore.
        return;
    }//if//

    // Cancel ACK timeout if it hasn't expired yet (fast ACK response).

    if (!wakeupTimerEventTicket.IsNull()) {
        (*this).CancelWakeupTimer();
    }//if//

    (*this).macState = BusyMediumState;

    if (lastSentFrameWasAn != SentFrameType::PowerSavePollResponse) {

        EdcaAccessCategoryInfo& accessCategoryInfo =
            accessCategories[accessCategoryIndexForLastSentFrame];

        // Success => Reset Contention window.

        accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

        // Ownership of packet was given to PHY.

        assert(accessCategoryInfo.currentPacketPtr == nullptr);

        assert(lastSentDataFrameDestinationMacAddress ==
               accessCategoryInfo.currentPacketsDestinationMacAddress);

    }//if//

    theAdaptiveRateControllerPtr->NotifyAckReceived(lastSentDataFrameDestinationMacAddress);

    (*this).currentTransmitOpportunityAckedFrameCount++;
    (*this).DoSuccessfulTransmissionPostProcessing(false);

}//ProcessAcknowledgementFrame//



inline
void Dot11Mac::ProcessBlockAckFrame(const BlockAcknowledgementFrame& blockAckFrame)
{
    if (macState != WaitingForAckState) {
        // Received an ACK, but was not expecting it ==> ignore.
        return;
    }//if//

    // Cancel ACK timeout if it hasn't expired yet (fast ACK response).

    if (!wakeupTimerEventTicket.IsNull()) {
        (*this).CancelWakeupTimer();
    }//if//

    assert(lastSentFrameWasAn != SentFrameType::PowerSavePollResponse);

    OutgoingLinkInfo& linkInfo =
        outgoingLinkInfoMap[
            AddressAndTrafficIdMapKey(
                blockAckFrame.transmitterAddress,
                blockAckFrame.blockAckControl.trafficId)];

    linkInfo.blockAckRequestNeedsToBeSent = false;

    (*this).currentTransmitOpportunityAckedFrameCount++;

    linkInfo.windowStartSequenceNumber = blockAckFrame.startingSequenceControl;

    for(unsigned int i = 0; (i < blockAckFrame.blockAckBitmap.size()); i++) {
        if (blockAckFrame.blockAckBitmap[i]) {
            IncrementTwelveBitSequenceNumber(linkInfo.windowStartSequenceNumber);
        }
        else {
            break;
        }//if//
    }//for//

    EdcaAccessCategoryInfo& accessCategoryInfo =
        accessCategories[accessCategoryIndexForLastSentFrame];

    if (lastSentFrameWasAn == SentFrameType::AggregateFrame) {
        assert(lastSentFrameWasAn == SentFrameType::AggregateFrame);

        assert(lastSentDataFrameDestinationMacAddress ==
           accessCategoryInfo.currentPacketsDestinationMacAddress);

        assert(accessCategoryInfo.currentPacketPtr == nullptr);
        assert(accessCategoryInfo.currentAggregateFramePtr == nullptr);

        physicalLayer.TakeOwnershipOfLastTransmittedAggregateFrame(
            accessCategoryInfo.currentAggregateFramePtr);

        (*this).macState = BusyMediumState;

        vector<unique_ptr<Packet> >& sentSubframes = *accessCategoryInfo.currentAggregateFramePtr;

        unsigned int numberAckedSubframes = 0;
        for(unsigned int i = 0; (i < sentSubframes.size()); i++) {
            const Packet& aSubframe = *sentSubframes[i];
            const QosDataFrameHeader& header =
                aSubframe.GetAndReinterpretPayloadData<QosDataFrameHeader>(
                    sizeof(MpduDelimiterFrame));

            if (blockAckFrame.IsAcked(header.theSequenceControlField.sequenceNumber)) {
                sentSubframes[i].reset();
                numberAckedSubframes++;
                theAdaptiveRateControllerPtr->NotifyAckReceived(lastSentDataFrameDestinationMacAddress);
            }
            else {
                theAdaptiveRateControllerPtr->NotifyAckFailed(lastSentDataFrameDestinationMacAddress);
            }//if//
        }//for//

        accessCategoryInfo.currentAggregateFrameRetryCount++;

        if (numberAckedSubframes == sentSubframes.size()) {
            accessCategoryInfo.currentAggregateFramePtr.reset();
            accessCategoryInfo.currentAggregateFrameRetryCount = 0;
        }
        else if (accessCategoryInfo.currentAggregateFrameRetryCount >= shortFrameRetryLimit) {

            unsigned short int droppedFrameSequenceNumber = 0;
            bool sequenceNumberWasSet = false;

            for(unsigned int i = 0; (i < sentSubframes.size()); i++) {
                if (sentSubframes[i] != nullptr) {
                    const Packet& aSubframe = *sentSubframes[i];
                    const QosDataFrameHeader& header =
                        aSubframe.GetAndReinterpretPayloadData<QosDataFrameHeader>(
                            sizeof(MpduDelimiterFrame));

                    droppedFrameSequenceNumber = header.theSequenceControlField.sequenceNumber;
                    sequenceNumberWasSet = true;
                }//if//
            }//for//

            accessCategoryInfo.currentAggregateFramePtr.reset();
            accessCategoryInfo.currentAggregateFrameRetryCount = 0;

            // Send Block Ack Request as a "management frame" to shift sequence number
            // window at destination for dropped frame.  In case that fails,
            // "mark" the link as needing an sequence number window update.

            linkInfo.blockAckRequestNeedsToBeSent = true;
            assert(sequenceNumberWasSet);
            linkInfo.lastDroppedFrameSequenceNumber = droppedFrameSequenceNumber;

            //Future Idea: Force a BAR by queueing a management frame that must be deleted if has
            //Future Idea: already been sent.  For quick delivery of buffered out-of-order frames at
            //Future Idea: the receiver.
            //Future Idea
            //Future Idea // Obsolete G++ Hack.
            //Future Idea unique_ptr<Packet> framePtr =
            //Future Idea    CreateABlockAckRequestFrame(
            //Future Idea        accessCategoryInfo.currentPacketsDestinationMacAddress,
            //Future Idea        accessCategoryInfo.currentPacketPriorityAkaTrafficId,
            //Future Idea        (droppedFrameSequenceNumber + 1));
            //Future Idea
            //Future Idea (*this).SendManagementFrame(framePtr);
        }
        else {
            unsigned int i = 0;
            while(i < sentSubframes.size()) {
                if (sentSubframes[i] == nullptr) {
                    sentSubframes.erase(sentSubframes.begin() + i);
                }
                else {
                    i++;
                }//if//
            }//while//
        }//if//

        assert((!protectAggregateFramesWithSingleAckedFrame) ||
            (accessCategoryInfo.currentContentionWindowSlots ==
               accessCategoryInfo.minContentionWindowSlots));

        accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

        (*this).DoSuccessfulTransmissionPostProcessing(false);
    }
    else {
        assert(lastSentFrameWasAn == SentFrameType::BlockAckRequestFrame);

        // This case is responding to block ack request (sequence # window sync)
        // (there are no buffered subframes in the aggregate).

        accessCategoryInfo.currentShortFrameRetryCount = 0;

        // Success => Reset Contention window.

        accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

        assert(lastSentDataFrameDestinationMacAddress == accessCategoryInfo.currentPacketsDestinationMacAddress);

        theAdaptiveRateControllerPtr->NotifyAckReceived(lastSentDataFrameDestinationMacAddress);

        (*this).macState = BusyMediumState;
        (*this).DoSuccessfulTransmissionPostProcessing(false);
    }//if//

}//ProcessBlockAckFrame//



inline
unique_ptr<Packet> Dot11Mac::CreateABlockAckRequestFrame(
    const MacAddress& destinationAddress,
    const PacketPriority& trafficId,
    const unsigned short int startFrameSequenceNumber) const
{
    BlockAcknowledgementRequestFrame blockAckRequest;

    blockAckRequest.header.theFrameControlField.frameTypeAndSubtype = BLOCK_ACK_REQUEST_FRAME_TYPE_CODE;
    blockAckRequest.header.receiverAddress = destinationAddress;
    blockAckRequest.header.duration = 0;
    blockAckRequest.header.theFrameControlField.isRetry = 0;
    blockAckRequest.blockAckRequestControl.trafficId = trafficId;
    blockAckRequest.transmitterAddress = GetMacAddress();
    blockAckRequest.startingSequenceControl = startFrameSequenceNumber;

    return (Packet::CreatePacket(*simEngineInterfacePtr, blockAckRequest));

}//CreateABlockAckRequestFrame//



inline
void Dot11Mac::NotifyThatPhyReceivedCorruptedFrame()
{
    (*this).lastFrameReceivedWasCorrupt = true;

    //globalMacStatistics.totalNumberReceivedCorruptedFrame++;

    if ((macState == WaitingForCtsState) || (macState == WaitingForAckState)) {
        macState = BusyMediumState;
        // Received a corrupt packet that wasn't the desired ACK or CTS for this node, must retry.
        (*this).NeverReceivedClearToSendOrAcknowledgementAction();

    }//if//

    assert(macState == BusyMediumState);
}


inline
void Dot11Mac::ProcessWakeupTimerEvent()
{
    (*this).wakeupTimerEventTicket.Clear();
    (*this).currentWakeupTimerExpirationTime = INFINITE_TIME;

    switch (macState) {
    case WaitingForNavExpirationState:
        (*this).ProcessVirtualCarrierSenseAkaNavExpiration();
        break;

    case WaitingForIfsAndBackoffState:
        (*this).ProcessInterframeSpaceAndBackoffTimeout();
        break;

    case WaitingForCtsState:
    case WaitingForAckState:
        (*this).ProcessClearToSendOrAcknowledgementTimeout();
        break;

    case IdleState:
        assert(false); abort(); break;
    default:
        assert(false); abort(); break;
    }//switch//

}//ProcessWakeupTimerEvent//


inline
void Dot11Mac::SendManagementFrame(unique_ptr<Packet>& framePtr)
{
    (*this).managementFrameQueue.push_back(move(framePtr));
    if (!AccessCategoryIsActive(accessCategoryIndexForManagementFrames)) {
        (*this).StartPacketSendProcessForAnAccessCategory(accessCategoryIndexForManagementFrames);
    }//if//
}



inline
void Dot11Mac::SendAssociationRequest(const MacAddress& apAddress)
{
    AssociationRequestFrame anAssociationRequestFrame;
    anAssociationRequestFrame.managementHeader.header.theFrameControlField.frameTypeAndSubtype = ASSOCIATION_REQUEST_FRAME_TYPE_CODE;
    anAssociationRequestFrame.managementHeader.header.theFrameControlField.isRetry = 0;
    anAssociationRequestFrame.managementHeader.header.duration = 0;
    anAssociationRequestFrame.managementHeader.header.receiverAddress = apAddress;
    anAssociationRequestFrame.managementHeader.transmitterAddress = myMacAddress;
    anAssociationRequestFrame.theHtCapabilitiesFrameElement.aggregateMpdusAreEnabled =
        (maxAggregateMpduSizeBytes > 0);

    assert(staControllerPtr != nullptr);

    anAssociationRequestFrame.ahCapabilityInfo.usesRestrictedAccessWindows =
        staControllerPtr->GetIsARestrictedAccessWindowSta();

    unsigned short int newSequenceNumber;
    (*this).GetNewSequenceNumber(apAddress, maxPacketPriority, true, newSequenceNumber);
    anAssociationRequestFrame.managementHeader.theSequenceControlField.sequenceNumber = newSequenceNumber;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<AssociationRequestFrame>(
        *simEngineInterfacePtr,
        anAssociationRequestFrame);

    (*this).SendManagementFrame(packetPtr);
}


inline
void Dot11Mac::SendReassociationRequest(
    const MacAddress& apAddress,
    const MacAddress& currentApAddress)
{
    ReassociationRequestFrame reassociationRequestFrame;
    reassociationRequestFrame.managementHeader.header.theFrameControlField.frameTypeAndSubtype = REASSOCIATION_REQUEST_FRAME_TYPE_CODE;
    reassociationRequestFrame.managementHeader.header.theFrameControlField.isRetry = 0;
    reassociationRequestFrame.managementHeader.header.duration = 0;
    reassociationRequestFrame.managementHeader.header.receiverAddress = apAddress;
    reassociationRequestFrame.managementHeader.transmitterAddress = myMacAddress;
    reassociationRequestFrame.currentApAddress = currentApAddress;
    reassociationRequestFrame.theHtCapabilitiesFrameElement.aggregateMpdusAreEnabled =
        (maxAggregateMpduSizeBytes > 0);

    reassociationRequestFrame.ahCapabilityInfo.usesRestrictedAccessWindows =
        staControllerPtr->GetIsARestrictedAccessWindowSta();

    unsigned short int newSequenceNumber;
    (*this).GetNewSequenceNumber(apAddress, maxPacketPriority, true, newSequenceNumber);
    reassociationRequestFrame.managementHeader.theSequenceControlField.sequenceNumber = newSequenceNumber;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<ReassociationRequestFrame>(
        *simEngineInterfacePtr,
        reassociationRequestFrame);

    (*this).SendManagementFrame(packetPtr);
}



inline
void Dot11Mac::SendDisassociation(const MacAddress& receiverAddress)
{
    DisassociationFrame aDisassociationFrame;

    aDisassociationFrame.managementHeader.header.theFrameControlField.frameTypeAndSubtype = DISASSOCIATION_FRAME_TYPE_CODE;
    aDisassociationFrame.managementHeader.header.theFrameControlField.isRetry = 0;
    aDisassociationFrame.managementHeader.header.duration = 0;
    aDisassociationFrame.managementHeader.header.receiverAddress = receiverAddress;
    aDisassociationFrame.managementHeader.transmitterAddress = myMacAddress;

    unsigned short int newSequenceNumber;
    (*this).GetNewSequenceNumber(receiverAddress, maxPacketPriority, true, newSequenceNumber);
    aDisassociationFrame.managementHeader.theSequenceControlField.sequenceNumber = newSequenceNumber;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<DisassociationFrame>(
        *simEngineInterfacePtr,
        aDisassociationFrame);

    (*this).SendManagementFrame(packetPtr);

}//SendDisassociation//



inline
void Dot11Mac::SendAssociationResponse(
    const MacAddress& staAddress,
    const AssociationId& theAssociationId)
{
    AssociationResponseFrame anAssociationResponseFrame;
    anAssociationResponseFrame.managementHeader.header.theFrameControlField.frameTypeAndSubtype = ASSOCIATION_RESPONSE_FRAME_TYPE_CODE;
    anAssociationResponseFrame.managementHeader.header.theFrameControlField.isRetry = 0;
    anAssociationResponseFrame.managementHeader.header.duration = 0;
    anAssociationResponseFrame.managementHeader.header.receiverAddress = staAddress;
    anAssociationResponseFrame.managementHeader.transmitterAddress = myMacAddress;
    anAssociationResponseFrame.theAssociationId = theAssociationId;
    anAssociationResponseFrame.theHtCapabilitiesFrameElement.aggregateMpdusAreEnabled =
        (maxAggregateMpduSizeBytes > 0);

    unsigned short int newSequenceNumber;
    (*this).GetNewSequenceNumber(staAddress, maxPacketPriority, true, newSequenceNumber);
    anAssociationResponseFrame.managementHeader.theSequenceControlField.sequenceNumber = newSequenceNumber;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<AssociationResponseFrame>(
        *simEngineInterfacePtr,
        anAssociationResponseFrame);

    (*this).SendManagementFrame(packetPtr);

}//SendAssociationResponse//



inline
void Dot11Mac::SendReassociationResponse(
    const MacAddress& staAddress,
    const AssociationId& theAssociationId)
{
    ReassociationResponseFrame aReassociationResponseFrame;
    aReassociationResponseFrame.managementHeader.header.theFrameControlField.frameTypeAndSubtype = REASSOCIATION_RESPONSE_FRAME_TYPE_CODE;
    aReassociationResponseFrame.managementHeader.header.theFrameControlField.isRetry = 0;
    aReassociationResponseFrame.managementHeader.header.duration = 0;
    aReassociationResponseFrame.managementHeader.header.receiverAddress = staAddress;
    aReassociationResponseFrame.managementHeader.transmitterAddress = myMacAddress;
    aReassociationResponseFrame.theHtCapabilitiesFrameElement.aggregateMpdusAreEnabled =
        (maxAggregateMpduSizeBytes > 0);

    aReassociationResponseFrame.theAssociationId = theAssociationId;

    unsigned short int newSequenceNumber;
    (*this).GetNewSequenceNumber(staAddress, maxPacketPriority, true, newSequenceNumber);
    aReassociationResponseFrame.managementHeader.theSequenceControlField.sequenceNumber = newSequenceNumber;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<ReassociationResponseFrame>(
        *simEngineInterfacePtr,
        aReassociationResponseFrame);

    (*this).SendManagementFrame(packetPtr);
}




inline
void Dot11Mac::SendAuthentication(const MacAddress& receiverAddress)
{
    AuthenticationFrame anAuthenticationFrame;

    anAuthenticationFrame.managementHeader.header.theFrameControlField.frameTypeAndSubtype = AUTHENTICATION_FRAME_TYPE_CODE;
    anAuthenticationFrame.managementHeader.header.theFrameControlField.isRetry = 0;
    anAuthenticationFrame.managementHeader.header.duration = 0;
    anAuthenticationFrame.managementHeader.header.receiverAddress = receiverAddress;
    anAuthenticationFrame.managementHeader.transmitterAddress = myMacAddress;

    unsigned short int newSequenceNumber;
    (*this).GetNewSequenceNumber(receiverAddress, maxPacketPriority, true, newSequenceNumber);
    anAuthenticationFrame.managementHeader.theSequenceControlField.sequenceNumber = newSequenceNumber;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<AuthenticationFrame>(
        *simEngineInterfacePtr,
        anAuthenticationFrame);

    (*this).SendManagementFrame(packetPtr);

}//SendAuthentication//



inline
void Dot11Mac::SendPowerSaveNullFrame(
    const MacAddress& receiverAddress,
    const bool goingToPowerManagementMode)
{
    assert(managementFrameQueue.empty());

    QosDataFrameHeader nullFrame;
    nullFrame.header.theFrameControlField.frameTypeAndSubtype = NULL_FRAME_TYPE_CODE;
    nullFrame.header.theFrameControlField.isRetry = 0;
    nullFrame.header.theFrameControlField.powerManagement = 0;
    if (goingToPowerManagementMode) {
        nullFrame.header.theFrameControlField.powerManagement = 1;
    }//if//
    nullFrame.header.duration = 0;
    nullFrame.header.receiverAddress = receiverAddress;

    nullFrame.theSequenceControlField.sequenceNumber = 0;
    nullFrame.transmitterAddress = myMacAddress;
    nullFrame.qosControlField.trafficId = 0;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<QosDataFrameHeader>(*simEngineInterfacePtr, nullFrame);

    (*this).SendManagementFrame(packetPtr);

}//SendPowerSaveNullFrame//


inline
void Dot11Mac::SwitchToChannels(const vector<unsigned int>& newBondedChannelList)
{
    if (physicalLayer.IsTransmittingAFrame()) {

        (*this).macState = ChangingChannelsState;
        (*this).switchingToThisChannelList = newBondedChannelList;
    }
    else {
        physicalLayer.SwitchToChannels(newBondedChannelList);
    }//if//

}//SwitchToChannels//

inline
void Dot11Mac::SwitchToChannel(const unsigned int channelNumber)
{
    vector<unsigned int> bondedChannelList(1);
    bondedChannelList[0] = channelNumber;
    (*this).SwitchToChannels(bondedChannelList);
}



inline
Dot11MacOperationMode Dot11Mac::GetOperationMode() const {
    return (operationMode);
}


inline
void Dot11Mac::OutputTraceAndStatsForFrameReceive(const Packet& aFrame) const
{
    const CommonFrameHeader& header = aFrame.GetAndReinterpretPayloadData<CommonFrameHeader>();

    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            MacFrameReceiveTraceRecord traceData;

            const PacketId& thePacketId = aFrame.GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            traceData.frameType = header.theFrameControlField.frameTypeAndSubtype;

            traceData.packetLengthBytes = static_cast<uint16_t>(aFrame.LengthBytes());

            assert(sizeof(traceData) == MAC_FRAME_RECEIVE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "RxFrame", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << aFrame.GetPacketId();

            msgStream << " FrameBytes= " << aFrame.LengthBytes();

            msgStream << " FrameType= ";

            msgStream << ConvertToDot11FrameTypeName(header.theFrameControlField.frameTypeAndSubtype);

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "RxFrame", msgStream.str());

        }//if//
    }//if//

    switch (header.theFrameControlField.frameTypeAndSubtype) {
    case RTS_FRAME_TYPE_CODE: {
        rtsFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case CTS_FRAME_TYPE_CODE: {
        ctsFramesReceivedStatPtr->IncrementCounter();
        break;
    }

    case QOS_DATA_FRAME_TYPE_CODE: {
        dataFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case ACK_FRAME_TYPE_CODE: {
        ackFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case BEACON_FRAME_TYPE_CODE: {
        beaconFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case ASSOCIATION_REQUEST_FRAME_TYPE_CODE: {
        associationRequestFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case ASSOCIATION_RESPONSE_FRAME_TYPE_CODE: {
        associationResponseFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case REASSOCIATION_REQUEST_FRAME_TYPE_CODE: {
        reassociationRequestFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case REASSOCIATION_RESPONSE_FRAME_TYPE_CODE: {
        reassociationResponseFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case DISASSOCIATION_FRAME_TYPE_CODE: {
        disassociationFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case AUTHENTICATION_FRAME_TYPE_CODE: {
        authenticationFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case BLOCK_ACK_FRAME_TYPE_CODE: {
        blockAckFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case BLOCK_ACK_REQUEST_FRAME_TYPE_CODE: {
        blockAckRequestFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    default:
        assert(false); abort();
        break;
    }//switch//

}//OutputTraceAndStatsForFrameReceive//



inline
void Dot11Mac::OutputTraceAndStatsForAggregateSubframeReceive(
    const Packet& aFrame,
    const QosDataFrameHeader& dataFrameHeader)
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            MacFrameReceiveTraceRecord traceData;

            const PacketId& thePacketId = aFrame.GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            traceData.frameType = dataFrameHeader.header.theFrameControlField.frameTypeAndSubtype;

            traceData.packetLengthBytes = static_cast<uint16_t>(aFrame.LengthBytes());

            assert(sizeof(traceData) == MAC_FRAME_RECEIVE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "RxFrame", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << aFrame.GetPacketId();

            msgStream << " FrameBytes= " << aFrame.LengthBytes();

            msgStream << " FrameType= ";

            msgStream << ConvertToDot11FrameTypeName(
                dataFrameHeader.header.theFrameControlField.frameTypeAndSubtype);

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "RxFrame", msgStream.str());

        }//if//
    }//if//

    dataAggregatedSubframesReceivedStatPtr->IncrementCounter();

}//OutputTraceAndStatsForAggregateSubframeReceive//



inline
void Dot11Mac::OutputTraceForClearChannel() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "ClearCh");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "ClearCh", "");
        }//if//
    }//if//
}


inline
void Dot11Mac::OutputTraceForBusyChannel() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "BusyCh");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "BusyCh", "");
        }//if//
    }//if//
}


inline
void Dot11Mac::OutputTraceForNavStart(const SimTime& expirationTime) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            SimTime currentTime = simEngineInterfacePtr->CurrentTime();

            SimTime duration = expirationTime - currentTime;

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "NAV-Start", duration);
        }
        else {
            ostringstream msgStream;

            SimTime currentTime = simEngineInterfacePtr->CurrentTime();

            msgStream << "Dur= " << ConvertTimeToStringSecs(expirationTime - currentTime);

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "NAV-Start", msgStream.str());
        }//if//
    }//if//
}

inline
void Dot11Mac::OutputTraceForNavExpiration() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "NAV-End");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "NAV-End", "");
        }//if//
    }//if//
}

inline
void Dot11Mac::OutputTraceForIfsAndBackoffStart(const SimTime& backoffDuration) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacIfsAndBackoffStartTraceRecord traceData;

            traceData.accessCategory = static_cast<uint32_t>(FindIndexOfAccessCategoryWithShortestBackoff());
            traceData.duration = backoffDuration;
            traceData.frameCorrupt = lastFrameReceivedWasCorrupt;

            assert(sizeof(traceData) == MAC_IFS_AND_BACKOFF_START_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "IFSAndBackoffStart", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream << "AC= " << FindIndexOfAccessCategoryWithShortestBackoff();
            msgStream << " Dur= " << ConvertTimeToStringSecs(backoffDuration);

            if (lastFrameReceivedWasCorrupt) {
                msgStream << " Ext= Yes";
            }
            else {
                msgStream << " Ext= No";
            }//if//

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "IFSAndBackoffStart", msgStream.str());

        }//if//
    }//if//

}//OutputTraceForIfsAndBackoffStart//


inline
void Dot11Mac::OutputTraceForInterruptedIfsAndBackoff(
    const unsigned int accessCategoryIndex,
    const SimTime nonExtendedDurationLeft) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacIfsAndBackoffPauseTraceRecord traceData;

            traceData.accessCategory = static_cast<uint32_t>(FindIndexOfAccessCategoryWithShortestBackoff());
            traceData.leftDuration = nonExtendedDurationLeft;

            assert(sizeof(traceData) == MAC_IFS_AND_BACKOFF_PAUSE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "IFSAndBackoffPause", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream << "AC= " << FindIndexOfAccessCategoryWithShortestBackoff();
            msgStream << " Left= " << ConvertTimeToStringSecs(nonExtendedDurationLeft);

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "IFSAndBackoffPause", msgStream.str());

        }//if//
    }//if//
}


inline
void Dot11Mac::OutputTraceForIfsAndBackoffExpiration() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "IFSAndBackoffEnd");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "IFSAndBackoffEnd", "");
        }//if//
    }//if//
}

inline
void Dot11Mac::OutputTraceForPacketDequeue(const unsigned int accessCategoryIndex, const SimTime delayUntilAirBorne) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacPacketDequeueTraceRecord traceData;

            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

            const PacketId& thePacketId = accessCategoryInfo.currentPacketPtr->GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            traceData.accessCategory = static_cast<uint32_t>(accessCategoryIndex);

            assert(!redundantTraceInformationModeIsOn);// no implementation for redundant trace

            assert(sizeof(traceData) == MAC_PACKET_DEQUEUE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Dequeue", traceData);
        }
        else {
            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

            ostringstream msgStream;

            msgStream << "PktId= " << accessCategoryInfo.currentPacketPtr->GetPacketId();
            msgStream << " AC= " << accessCategoryIndex;
            if (redundantTraceInformationModeIsOn) {
                msgStream << " Dur= " <<  ConvertTimeToStringSecs(delayUntilAirBorne);
            }
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Dequeue", msgStream.str());
        }//if//
    }//if//

}//OutputTraceForPacketDequeue//

inline
void Dot11Mac::OutputTraceAndStatsForRtsFrameTransmission(const unsigned int accessCategoryIndex) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacTxRtsTraceRecord traceData;

            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

            traceData.accessCategory = static_cast<uint32_t>(accessCategoryIndex);
            traceData.retry = accessCategoryInfo.currentShortFrameRetryCount;

            assert(sizeof(traceData) == MAC_TX_RTS_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-RTS", traceData);
        }
        else {
            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

            ostringstream msgStream;

            msgStream << "AC= " << accessCategoryIndex;
            msgStream << " S_Retry= " << accessCategoryInfo.currentShortFrameRetryCount;

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-RTS", msgStream.str());
        }//if//
    }//if//

    rtsFramesSentStatPtr->IncrementCounter();

}//OutputTraceAndStatsForRtsFrameTransmission//


inline
void Dot11Mac::OutputTraceForCtsFrameTransmission() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-CTS");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-CTS", "");
        }//if//
    }//if//

    ctsFramesSentStatPtr->IncrementCounter();

}//OutputTraceForCtsFrameTransmission//


inline
void Dot11Mac::OutputTraceAndStatsForAggregatedFrameTransmission(const unsigned int accessCategoryIndex) const
{
    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacTxUnicastDataTraceRecord traceData;

            const PacketId& thePacketId = accessCategoryInfo.currentAggregateFramePtr->front()->GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            traceData.accessCategory = static_cast<uint32_t>(accessCategoryIndex);
            traceData.shortFrameRetry = accessCategoryInfo.currentAggregateFrameRetryCount;
            traceData.longFrameRetry = accessCategoryInfo.currentLongFrameRetryCount;

            assert(sizeof(traceData) == MAC_TX_UNICAST_DATA_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-DATA-A", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << accessCategoryInfo.currentAggregateFramePtr->front()->GetPacketId();
            msgStream << " AC= " << accessCategoryIndex;
            msgStream << " S-Retry= " << accessCategoryInfo.currentAggregateFrameRetryCount;
            msgStream << " L-Retry= " << accessCategoryInfo.currentLongFrameRetryCount;

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-DATA-A", msgStream.str());
        }//if//
    }//if//

    dataAggregateFramesSentStatPtr->IncrementCounter();

    if (accessCategoryInfo.currentAggregateFrameRetryCount > 0) {
        dataAggregateFramesResentStatPtr->IncrementCounter();
    }//if//

}//OutputTraceAndStatsForAggregatedFrameTransmission//


inline
void Dot11Mac::OutputTraceAndStatsForBroadcastDataFrameTransmission(const unsigned int accessCategoryIndex) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacTxBroadcastDataTraceRecord traceData;
            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

            const PacketId& thePacketId = accessCategoryInfo.currentPacketPtr->GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            traceData.accessCategory = static_cast<uint32_t>(accessCategoryIndex);

            assert(sizeof(traceData) == MAC_TX_BROADCAST_DATA_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-DATA-B", traceData);
        }
        else {
            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);
            ostringstream msgStream;

            msgStream << "PktId= " << accessCategoryInfo.currentPacketPtr->GetPacketId();
            msgStream << " AC= " << accessCategoryIndex;

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-DATA-B", msgStream.str());
        }//if//
    }//if//

    broadcastDataFramesSentStatPtr->IncrementCounter();

}//OutputTraceAndStatsForBroadcastDataFrameTransmission//



inline
void Dot11Mac::OutputTraceAndStatsForUnicastDataFrameTransmission(const unsigned int accessCategoryIndex) const
{
    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacTxUnicastDataTraceRecord traceData;

            const PacketId& thePacketId = accessCategoryInfo.currentPacketPtr->GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            traceData.accessCategory = static_cast<uint32_t>(accessCategoryIndex);
            traceData.shortFrameRetry = accessCategoryInfo.currentShortFrameRetryCount;
            traceData.longFrameRetry = accessCategoryInfo.currentLongFrameRetryCount;

            assert(sizeof(traceData) == MAC_TX_UNICAST_DATA_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-DATA-U", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << accessCategoryInfo.currentPacketPtr->GetPacketId();
            msgStream << " AC= " << accessCategoryIndex;
            msgStream << " S-Retry= " << accessCategoryInfo.currentShortFrameRetryCount;
            msgStream << " L-Retry= " << accessCategoryInfo.currentLongFrameRetryCount;

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-DATA-U", msgStream.str());
        }//if//
    }//if//

    unicastDataFramesSentStatPtr->IncrementCounter();

    if ((accessCategoryInfo.currentShortFrameRetryCount > 0) || (accessCategoryInfo.currentLongFrameRetryCount > 0)) {
        unicastDataFramesResentStatPtr->IncrementCounter();
    }//if//

}//OutputTraceAndStatsForUnicastDataFrameTransmission//


inline
void Dot11Mac::OutputTraceAndStatsForManagementFrameTransmission(const unsigned int accessCategoryIndex) const
{
    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);
    const CommonFrameHeader& header =
                accessCategoryInfo.currentPacketPtr->GetAndReinterpretPayloadData<CommonFrameHeader>();
    const unsigned char frameType = header.theFrameControlField.frameTypeAndSubtype;

    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            Dot11MacTxManagementTraceRecord traceData;

            const PacketId& thePacketId = accessCategoryInfo.currentPacketPtr->GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            traceData.frameType = frameType;

            assert(sizeof(traceData) == DOT11_MAC_TX_MANAGEMENT_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-Management", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << accessCategoryInfo.currentPacketPtr->GetPacketId();

            msgStream << " FrameType= ";

            msgStream << ConvertToDot11FrameTypeName(frameType);

            simEngineInterfacePtr->OutputTrace(
                modelName, theInterfaceId, "Tx-Management", msgStream.str());

        }//if//
    }//if//

    //stat
    switch (frameType) {
    case BEACON_FRAME_TYPE_CODE: {
        beaconFramesSentStatPtr->IncrementCounter();
        break;
    }
    case ASSOCIATION_REQUEST_FRAME_TYPE_CODE: {
        associationRequestFramesSentStatPtr->IncrementCounter();
        break;
    }
    case ASSOCIATION_RESPONSE_FRAME_TYPE_CODE: {
        associationResponseFramesSentStatPtr->IncrementCounter();
        break;
    }
    case REASSOCIATION_REQUEST_FRAME_TYPE_CODE: {
        reassociationRequestFramesSentStatPtr->IncrementCounter();
        break;
    }
    case REASSOCIATION_RESPONSE_FRAME_TYPE_CODE: {
        reassociationResponseFramesSentStatPtr->IncrementCounter();
        break;
    }
    case DISASSOCIATION_FRAME_TYPE_CODE: {
        disassociationFramesSentStatPtr->IncrementCounter();
        break;
    }
    case AUTHENTICATION_FRAME_TYPE_CODE: {
        authenticationFramesSentStatPtr->IncrementCounter();
        break;
    }
    case BLOCK_ACK_REQUEST_FRAME_TYPE_CODE:
        // Management-esque frame.
        break;

    case QOS_DATA_FRAME_TYPE_CODE:
    case ACK_FRAME_TYPE_CODE:
    case RTS_FRAME_TYPE_CODE:
    case CTS_FRAME_TYPE_CODE:
    default:
        assert(false); abort();
        break;
    }//switch//

}//OutputTraceAndStatsForManagementFrameTransmission//


inline
void Dot11Mac::OutputTraceForAckFrameTransmission() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-ACK");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-ACK", "");
        }//if//
    }//if//

    ackFramesSentStatPtr->IncrementCounter();

}//OutputTraceForAckFrameTransmission//


inline
void Dot11Mac::OutputTraceAndStatsForBlockAckRequestFrameTransmission() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-BlockACK-Request");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-BlockACK-Request", "");
        }//if//
    }//if//

    blockAckRequestFramesSentStatPtr->IncrementCounter();

}//OutputTraceAndStatsForBlockAckRequestFrameTransmission//


inline
void Dot11Mac::OutputTraceAndStatsForBlockAckFrameTransmission() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-BlockACK");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-BlockACK", "");
        }//if//
    }//if//

     blockAckFramesSentStatPtr->IncrementCounter();

}//OutputTraceAndStatsForBlockAckFrameTransmission//



inline
void Dot11Mac::OutputTraceForCtsOrAckTimeout() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {

        const bool lastTransmissionWasAShortFrame =
            ((lastSentFrameWasAn == SentFrameType::ShortFrame) || (lastSentFrameWasAn == SentFrameType::RequestToSendFrame) ||
             (lastSentFrameWasAn == SentFrameType::BlockAckRequestFrame));

        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacCtsOrAckTimeoutTraceRecord traceData;

            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndexForLastSentFrame);

            traceData.accessCategory = static_cast<uint32_t>(accessCategoryIndexForLastSentFrame);
            traceData.windowSlot = accessCategoryInfo.currentContentionWindowSlots;
            traceData.shortFrameRetry = accessCategoryInfo.currentShortFrameRetryCount;
            traceData.longFrameRetry = accessCategoryInfo.currentLongFrameRetryCount;
            traceData.shortFrameOrNot = lastTransmissionWasAShortFrame;

            assert(sizeof(traceData) == MAC_CTSORACK_TIMEOUT_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Timeout", traceData);
        }
        else {
            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndexForLastSentFrame);
            ostringstream msgStream;

            msgStream << "AC= " << accessCategoryIndexForLastSentFrame;
            msgStream << " Win= " << accessCategoryInfo.currentContentionWindowSlots;
            msgStream << " S-Retry= " << accessCategoryInfo.currentShortFrameRetryCount;
            msgStream << " L-Retry= " << accessCategoryInfo.currentLongFrameRetryCount;

            if (lastTransmissionWasAShortFrame) {
                msgStream << " Len= short";
            }
            else {
                msgStream << " Len= long";
            }//if//

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Timeout", msgStream.str());
        }
    }//if//

}//OutputTraceForCtsOrAckTimeout//



inline
void Dot11Mac::OutputTraceAndStatsForPacketRetriesExceeded(const unsigned int accessCategoryIndex) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacPacketRetryExceededTraceRecord traceData;

            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

            const PacketId& thePacketId = accessCategoryInfo.currentPacketPtr->GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == MAC_PACKET_RETRY_EXCEEDED_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Drop", traceData);
        }
        else {
            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);
            ostringstream msgStream;

            msgStream << "PktId= " << accessCategoryInfo.currentPacketPtr->GetPacketId();
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Drop", msgStream.str());
        }//if//
    }//if//

    droppedPacketsStatPtr->IncrementCounter();

}//OutputTraceAndStatsForPacketRetriesExceeded//

inline
Dot11Mac::~Dot11Mac(){ }


inline
void Dot11Mac::SendAckFrame(
    const MacAddress& destinationAddress,
    const TransmissionParameters& receivedFrameTxParameters)
{
    AcknowledgementAkaAckFrame ackFrame;

    ackFrame.header.theFrameControlField.frameTypeAndSubtype = ACK_FRAME_TYPE_CODE;
    ackFrame.header.receiverAddress = destinationAddress;
    ackFrame.header.duration = 0;

    unique_ptr<Packet> ackPacketPtr = Packet::CreatePacket(*simEngineInterfacePtr, ackFrame);

    TransmissionParameters ackTxParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForAckFrame(
        destinationAddress, receivedFrameTxParameters, ackTxParameters);

    const double transmitPowerDbm = theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(destinationAddress);

    (*this).macState = CtsOrAckTransmissionState;

    OutputTraceForAckFrameTransmission();

    physicalLayer.TransmitFrame(
        ackPacketPtr,
        ackTxParameters,
        transmitPowerDbm,
        aShortInterframeSpaceTime,
        ahUseOptimizedNdpControlFrames);

}//SendAckFrame//



inline
void Dot11Mac::SendPacketToNetworkLayer(unique_ptr<Packet>& dataPacketPtr)
{
    const QosDataFrameHeader& header =
        dataPacketPtr->GetAndReinterpretPayloadData<QosDataFrameHeader>();

    bool wasFound;
    NetworkAddress lastHopAddress;
    theMacAddressResolverPtr->GetNetworkAddressIfAvailable(
        header.transmitterAddress,
        networkLayerPtr->GetSubnetAddress(interfaceIndex),
        wasFound,
        lastHopAddress);

    assert(wasFound || (lastHopAddress == NetworkAddress::invalidAddress));

    const EtherTypeField etherType = NetToHost16(header.linkLayerHeader.etherType);
    dataPacketPtr->DeleteHeader(sizeof(QosDataFrameHeader));
    networkLayerPtr->ReceivePacketFromMac(interfaceIndex, dataPacketPtr, lastHopAddress, etherType);

}//SendPacketToNetworkLayer//



inline
void Dot11Mac::ProcessDataFrame(
    const Packet& dataFrame,
    const TransmissionParameters& receivedFrameTxParameters)
{
    if ((macState == WaitingForCtsState) || (macState == WaitingForAckState)) {
        macState = BusyMediumState;
        (*this).NeverReceivedClearToSendOrAcknowledgementAction();
    }//if//

    const QosDataFrameHeader& dataFrameHeader = dataFrame.GetAndReinterpretPayloadData<QosDataFrameHeader>();

    bool frameIsInOrder = true;
    vector<unique_ptr<Packet> > bufferedPacketsToSendUp;

    if (!dataFrameHeader.header.receiverAddress.IsABroadcastOrAMulticastAddress()) {

        bool haveAlreadySeenThisFrame = false;

        theIncomingFrameBuffer.ProcessIncomingFrame(
            dataFrame,
            dataFrameHeader.transmitterAddress,
            dataFrameHeader.qosControlField.trafficId,
            dataFrameHeader.theSequenceControlField.sequenceNumber,
            frameIsInOrder,
            haveAlreadySeenThisFrame,
            bufferedPacketsToSendUp);

        (*this).SendAckFrame(dataFrameHeader.transmitterAddress, receivedFrameTxParameters);

        if (haveAlreadySeenThisFrame) {
            //duplicated
            dataDuplicatedFramesReceivedStatPtr->IncrementCounter();
        }//if//
    }//if//

    if (frameIsInOrder) {
        unique_ptr<Packet> packetPtr(new Packet(dataFrame));
        (*this).SendPacketToNetworkLayer(packetPtr);
    }//if//

    if (!bufferedPacketsToSendUp.empty()) {
        for(unsigned int i = 0; (i < bufferedPacketsToSendUp.size()); i++) {
            (*this).SendPacketToNetworkLayer(bufferedPacketsToSendUp[i]);
        }//for//
    }//if//

}//ProcessDataFrame//


inline
void Dot11Mac::ProcessBlockAckRequestFrame(
    const BlockAcknowledgementRequestFrame& blockAckRequestFrame,
    const TransmissionParameters& receivedFrameTxParameters)
{
    vector<unique_ptr<Packet> > bufferedPacketsToSendUp;
    theIncomingFrameBuffer.ProcessBlockAckRequestFrame(blockAckRequestFrame, bufferedPacketsToSendUp);

    (*this).SendBlockAcknowledgementFrame(
        blockAckRequestFrame.transmitterAddress,
        blockAckRequestFrame.blockAckRequestControl.trafficId,
        receivedFrameTxParameters);

    if (!bufferedPacketsToSendUp.empty()) {
        for(unsigned int i = 0; (i < bufferedPacketsToSendUp.size()); i++) {
            (*this).SendPacketToNetworkLayer(bufferedPacketsToSendUp[i]);
        }//for//
    }//if//

}//ProcessBlockAckRequestFrame//



inline
void Dot11Mac::ProcessManagementFrame(
    const Packet& managementFrame,
    const TransmissionParameters& receivedFrameTxParameters)
{
    if ((macState == WaitingForCtsState) || (macState == WaitingForAckState)) {
        macState = BusyMediumState;
        (*this).NeverReceivedClearToSendOrAcknowledgementAction();
    }//if//

    const ManagementFrameHeader& managementFrameHeader =
        managementFrame.GetAndReinterpretPayloadData<ManagementFrameHeader>();

    if (!managementFrameHeader.header.receiverAddress.IsABroadcastOrAMulticastAddress()) {

        bool haveAlreadySeenThisFrame = false;
        vector<unique_ptr<Packet> > bufferedPacketsToSendUp;

        theIncomingFrameBuffer.ProcessIncomingNonDataFrame(
            managementFrameHeader.transmitterAddress,
            maxPacketPriority,
            managementFrameHeader.theSequenceControlField.sequenceNumber,
            haveAlreadySeenThisFrame,
            bufferedPacketsToSendUp);

        (*this).SendAckFrame(managementFrameHeader.transmitterAddress, receivedFrameTxParameters);

        if (haveAlreadySeenThisFrame) {
            // duplicate => drop
            dataDuplicatedFramesReceivedStatPtr->IncrementCounter();
            return;
        }//if//

        if (!bufferedPacketsToSendUp.empty()) {
            for(unsigned int i = 0; (i < bufferedPacketsToSendUp.size()); i++) {
                (*this).SendPacketToNetworkLayer(bufferedPacketsToSendUp[i]);
            }//for//
        }//if//
    }//if//

    switch (operationMode) {
    case ApMode:
        apControllerPtr->ProcessManagementFrame(managementFrame);
        break;
    case InfrastructureMode:
        staControllerPtr->ProcessManagementFrame(managementFrame);
        break;
    case AdhocMode:
        // Ignore
        break;
    default:
        assert(false); abort();
    }//switch//

}//ProcessManagementFrame//


inline
void Dot11Mac::ProcessNullFrame(
    const Packet& nullFrame,
    const TransmissionParameters& receivedFrameTxParameters)
{
    if ((macState == WaitingForCtsState) || (macState == WaitingForAckState)) {
        macState = BusyMediumState;
        (*this).NeverReceivedClearToSendOrAcknowledgementAction();
    }//if//

    const QosNullFrameHeader& nullFrameHeader =
        nullFrame.GetAndReinterpretPayloadData<QosNullFrameHeader>();

    bool haveAlreadySeenThisFrame;
    vector<unique_ptr<Packet> > bufferedPacketsToSendUp;

    theIncomingFrameBuffer.ProcessIncomingNonDataFrame(
        nullFrameHeader.transmitterAddress,
        nullFrameHeader.qosControlField.trafficId,
        nullFrameHeader.theSequenceControlField.sequenceNumber,
        haveAlreadySeenThisFrame,
        bufferedPacketsToSendUp);

    (*this).SendAckFrame(nullFrameHeader.transmitterAddress, receivedFrameTxParameters);

    if ((!haveAlreadySeenThisFrame) && (GetOperationMode() == ApMode)) {

        apControllerPtr->ReceiveFramePowerManagementBit(
            nullFrameHeader.transmitterAddress,
            nullFrameHeader.header.theFrameControlField.powerManagement);

    }//if//

    if (!bufferedPacketsToSendUp.empty()) {
        // This may never actually happen.
        for(unsigned int i = 0; (i < bufferedPacketsToSendUp.size()); i++) {
            (*this).SendPacketToNetworkLayer(bufferedPacketsToSendUp[i]);
        }//for//
    }//if//

}//ProcessNullFrame//



inline
void Dot11Mac::ReceiveFrameFromPhy(
    const Packet& aFrame,
    const TransmissionParameters& receivedFrameTxParameters)
{
    using std::max;

    (*this).lastFrameReceivedWasCorrupt = false;

    const CommonFrameHeader& header = aFrame.GetAndReinterpretPayloadData<CommonFrameHeader>();

    if (FrameIsForThisNode(aFrame)) {

        OutputTraceAndStatsForFrameReceive(aFrame);

        switch (header.theFrameControlField.frameTypeAndSubtype) {
        case RTS_FRAME_TYPE_CODE: {
            const RequestToSendFrame& rtsFrame = aFrame.GetAndReinterpretPayloadData<RequestToSendFrame>();

            (*this).ProcessRequestToSendFrame(rtsFrame, receivedFrameTxParameters);

            break;
        }
        case CTS_FRAME_TYPE_CODE: {
            const ClearToSendFrame& ctsFrame = aFrame.GetAndReinterpretPayloadData<ClearToSendFrame>();

            (*this).ProcessClearToSendFrame(ctsFrame);

            break;
        }
        case NULL_FRAME_TYPE_CODE:
            (*this).ProcessNullFrame(aFrame, receivedFrameTxParameters);

            break;
        case QOS_DATA_FRAME_TYPE_CODE: {

            (*this).ProcessDataFrame(aFrame, receivedFrameTxParameters);

            break;
        }
        case ACK_FRAME_TYPE_CODE: {
            const AcknowledgementAkaAckFrame& ackFrame =
                aFrame.GetAndReinterpretPayloadData<AcknowledgementAkaAckFrame>();

            (*this).ProcessAcknowledgementFrame(ackFrame);

            break;
        }
        case BLOCK_ACK_REQUEST_FRAME_TYPE_CODE: {

            const BlockAcknowledgementRequestFrame& blockAckRequestFrame =
                aFrame.GetAndReinterpretPayloadData<BlockAcknowledgementRequestFrame>();

            (*this).ProcessBlockAckRequestFrame(blockAckRequestFrame, receivedFrameTxParameters);

            break;
        }
        case BLOCK_ACK_FRAME_TYPE_CODE: {

            const BlockAcknowledgementFrame& blockAckFrame =
                aFrame.GetAndReinterpretPayloadData<BlockAcknowledgementFrame>();

            (*this).ProcessBlockAckFrame(blockAckFrame);

            break;
        }
        case POWER_SAVE_POLL_FRAME_TYPE_CODE: {
            const PowerSavePollFrame& psPollFrame =
                aFrame.GetAndReinterpretPayloadData<PowerSavePollFrame>();

            (*this).ProcessPowerSavePollFrame(psPollFrame, receivedFrameTxParameters);

            break;
        }
        default:
            if (IsAManagementFrameTypeCode(header.theFrameControlField.frameTypeAndSubtype)) {
                (*this).ProcessManagementFrame(aFrame, receivedFrameTxParameters);
            } else {
                assert(false); abort();
            }
            break;

        }//switch//

    }
    else {
        // Not for this node.

        // Set Virtual(Software) Carrier Sense aka NAV.

        SimTime endOfDurationTime;

        if (header.theFrameControlField.frameTypeAndSubtype != POWER_SAVE_POLL_FRAME_TYPE_CODE) {
            endOfDurationTime =
                simEngineInterfacePtr->CurrentTime() + (header.duration * MICRO_SECOND);
        }
        else {
            // Duration field is not a duration but an Assocation ID.
            endOfDurationTime =
                simEngineInterfacePtr->CurrentTime() +
                CalculateNavDurationForAckedDataFrame(receivedFrameTxParameters);
        }//if//

        mediumReservedUntilTimeAkaNAV = max(mediumReservedUntilTimeAkaNAV, endOfDurationTime);

        bool wasInWaitingForCtsOrAckState =
            ((macState == WaitingForCtsState) || (macState == WaitingForAckState));

        macState = BusyMediumState;

        if (wasInWaitingForCtsOrAckState) {
            // Received a packet but wasn't the desired ACK or CTS for this node, must retry.
            (*this).NeverReceivedClearToSendOrAcknowledgementAction();

        }//if//
    }//if//

}//ReceiveFrameFromPhy//


inline
void Dot11Mac::SendBlockAcknowledgementFrame(
    const MacAddress& destinationAddress,
    const PacketPriority& trafficId,
    const TransmissionParameters& receivedFrameTxParameters)
{
    BlockAcknowledgementFrame blockAckFrame;
    blockAckFrame.header.theFrameControlField.frameTypeAndSubtype = BLOCK_ACK_FRAME_TYPE_CODE;
    blockAckFrame.header.receiverAddress = destinationAddress;
    blockAckFrame.header.duration = 0;

    blockAckFrame.transmitterAddress = myMacAddress;
    blockAckFrame.blockAckControl.trafficId = trafficId;

    bool success;

    unsigned short startingSequenceControlNum;

    theIncomingFrameBuffer.GetBlockAckInfo(
        destinationAddress,
        trafficId,
        success,
        startingSequenceControlNum,
        blockAckFrame.blockAckBitmap);

    assert(success);

    blockAckFrame.startingSequenceControl = startingSequenceControlNum;

    unique_ptr<Packet> ackPacketPtr = Packet::CreatePacket(*simEngineInterfacePtr, blockAckFrame);

    TransmissionParameters ackTxParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForAckFrame(
        destinationAddress, receivedFrameTxParameters, ackTxParameters);

    const double transmitPowerDbm =
        theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(destinationAddress);

    physicalLayer.TransmitFrame(
        ackPacketPtr,
        ackTxParameters,
        transmitPowerDbm,
        aShortInterframeSpaceTime);

    macState = CtsOrAckTransmissionState;

    OutputTraceAndStatsForBlockAckFrameTransmission();

}//SendBlockAcknowledgementFrame//


inline
void Dot11Mac::ReceiveAggregatedSubframeFromPhy(
    unique_ptr<Packet>& subframePtr,
    const TransmissionParameters& receivedFrameTxParameters,
    const unsigned int aggregateFrameSubframeIndex,
    const unsigned int numberSubframes)
{
    if ((macState == WaitingForCtsState) || (macState == WaitingForAckState)) {
        macState = BusyMediumState;
        (*this).NeverReceivedClearToSendOrAcknowledgementAction();
    }//if//

    if (!FrameIsForThisNode(*subframePtr, sizeof(MpduDelimiterFrame))) {

        // Phy filters (to avoid copying (extra-simulation))) only by mac address,
        // frame could still be rejected.

        subframePtr = nullptr;
        return;
    }//if//

    (*this).lastFrameReceivedWasCorrupt = false;

    if (aggregateFrameSubframeIndex == 0) {
        numSubframesReceivedFromCurrentAggregateFrame = 1;
    }
    else {
        numSubframesReceivedFromCurrentAggregateFrame++;
    }//if//

    RemoveMpduDelimiterAndPaddingFromFrame(*subframePtr);

    const QosDataFrameHeader header =
        subframePtr->GetAndReinterpretPayloadData<QosDataFrameHeader>();

    const MacAddress transmitterAddress = header.transmitterAddress;
    const PacketPriority trafficId = header.qosControlField.trafficId;

    (*this).currentIncomingAggregateFramesSourceMacAddress = transmitterAddress;
    (*this).currentIncomingAggregateFramesTrafficId = trafficId;

    bool frameIsInOrder;
    bool haveAlreadySeenThisFrame;
    vector<unique_ptr<Packet> > bufferedPacketsToSendUp;

    OutputTraceAndStatsForAggregateSubframeReceive(*subframePtr, header);

    theIncomingFrameBuffer.ProcessIncomingSubframe(
        subframePtr,
        transmitterAddress,
        trafficId,
        header.theSequenceControlField.sequenceNumber,
        frameIsInOrder,
        haveAlreadySeenThisFrame,
        bufferedPacketsToSendUp);

    if (aggregateFrameSubframeIndex >= (numberSubframes - 1)) {
        (*this).SendBlockAcknowledgementFrame(
            transmitterAddress, trafficId, receivedFrameTxParameters);
    }//if//

    if (frameIsInOrder) {
        (*this).SendPacketToNetworkLayer(subframePtr);
    }
    else if (haveAlreadySeenThisFrame) {
        subframePtr.reset();
    }//if//

    if (!bufferedPacketsToSendUp.empty()) {
        for(unsigned int i = 0; (i < bufferedPacketsToSendUp.size()); i++) {
            (*this).SendPacketToNetworkLayer(bufferedPacketsToSendUp[i]);
        }//for//
    }//if//

}//ReceiveAggregatedSubframeFromPhy//



inline
void Dot11Mac::NotifyThatPhyReceivedCorruptedAggregatedSubframe(
    const TransmissionParameters& receivedFrameTxParameters,
    const unsigned int aggregateFrameSubframeIndex,
    const unsigned int numberSubframes)
{
    if (aggregateFrameSubframeIndex == 0) {
        if ((macState == WaitingForCtsState) || (macState == WaitingForAckState)) {
            macState = BusyMediumState;
            (*this).NeverReceivedClearToSendOrAcknowledgementAction();
        }//if//

        (*this).numSubframesReceivedFromCurrentAggregateFrame = 0;
        (*this).currentIncomingAggregateFramesSourceMacAddress = MacAddress::invalidMacAddress;
    }//if//

    if ((aggregateFrameSubframeIndex >= (numberSubframes - 1)) &&
        (numSubframesReceivedFromCurrentAggregateFrame > 0)) {

        // Only send if at least one frame was good.

        (*this).SendBlockAcknowledgementFrame(
            currentIncomingAggregateFramesSourceMacAddress,
            currentIncomingAggregateFramesTrafficId,
            receivedFrameTxParameters);

        (*this).lastFrameReceivedWasCorrupt = false;
    }
    else {
        // Only set to true when 0 frames were received.

        (*this).lastFrameReceivedWasCorrupt = true;
    }//if//

}//NotifyThatPhyReceivedCorruptedAggregatedSubframe//




inline
void Dot11Mac::LookupMacAddressForNeighbor(
    const NodeId targetNodeId, bool& wasFound, MacAddress& macAddress)
{
    switch (GetOperationMode()) {
    case InfrastructureMode:
        staControllerPtr->GetCurrentAccessPointAddress(wasFound, macAddress);
        break;
    case ApMode:
        apControllerPtr->LookupAssociatedNodeMacAddress(targetNodeId, wasFound, macAddress);
        break;
    case AdhocMode:
        macAddress.Clear();
        macAddress.SetLowerBitsWithNodeId(targetNodeId);
        macAddress.SetInterfaceSelectorByte(AdhocModeAddressSelectorByte);
        wasFound = true;
        break;
    default:
        assert(false); abort(); break;
    }//switch//

}//LookupMacAddressForNeighbor//


inline
void Dot11Mac::QueueOutgoingPacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopIpAddress,
    const PacketPriority priority)
{
    EnqueueResultType enqueueResult;
    unique_ptr<Packet> packetToDrop;

    networkOutputQueuePtr->Insert(
        packetPtr,
        nextHopIpAddress,
        priority,
        enqueueResult,
        packetToDrop);

    if (enqueueResult != ENQUEUE_SUCCESS) {
        packetToDrop = nullptr;
    }//if//

    (*this).NetworkLayerQueueChangeNotification();

}//QueueOutgoingPacket//


//--------------------------------------------------------------------------------------------------


inline
void Dot11Mac::SaveNonRawBackoffInfo()
{
    assert(!nonRawBackoffInformationHasBeenSaved && "Don't Save twice (overwrites)");

    for(unsigned int i = 0; (i < accessCategories.size()); i++) {
        EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[i];
        accessCategoryInfo.ahSavedNonRawContentionWindowSlots =
            accessCategoryInfo.currentContentionWindowSlots;
        accessCategoryInfo.ahSavedNonRawNumOfBackoffSlots =
            accessCategoryInfo.currentNumOfBackoffSlots;
    }//for//

    (*this).nonRawBackoffInformationHasBeenSaved = true;

}//SaveNonRawBackoffInfo//


inline
void Dot11Mac::RestoreNonRawBackoffInfo()
{
    assert(nonRawBackoffInformationHasBeenSaved);

    for(unsigned int i = 0; (i < accessCategories.size()); i++) {
        EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[i];
        accessCategoryInfo.currentContentionWindowSlots =
            accessCategoryInfo.ahSavedNonRawContentionWindowSlots;

        // Only restore if backoff is active (Frame could have been sent in RAW).
        if (accessCategoryInfo.currentNonExtendedBackoffDuration != INFINITE_TIME) {
            accessCategoryInfo.currentNumOfBackoffSlots = accessCategoryInfo.ahSavedNonRawNumOfBackoffSlots;
            accessCategoryInfo.currentNonExtendedBackoffDuration =
                CalculateNonExtendedBackoffDuration(accessCategoryInfo);
        }//if//
    }//for//

    (*this).nonRawBackoffInformationHasBeenSaved = false;

}//RestoreNonRawBackoffInfo//



inline
void Dot11Mac::SetAllContentionWindowsToMinAndRegenerateBackoff()
{
    assert(nonRawBackoffInformationHasBeenSaved);

    for(unsigned int i = 0; (i < accessCategories.size()); i++) {
        EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[i];
        accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

        // Only recalc if backoff currently set.

        if (accessCategoryInfo.currentNonExtendedBackoffDuration != INFINITE_TIME) {
            (*this).RecalcRandomBackoff(accessCategoryInfo);
        }//if//
    }//for//

}//SetAllContentionWindowsToMinAndRegenerateBackoff//



inline
void Dot11Mac::SetRestrictedAccessWindowEndTimeAndRestartMac(const SimTime& anRawEndTime)
{
    assert((ahRestrictedAccessWindowEndTime == INFINITE_TIME) ||
           (ahRestrictedAccessWindowEndTime <= anRawEndTime));

    (*this).ahRestrictedAccessWindowEndTime = anRawEndTime;

    if (physicalLayer.IsNotReceivingFrames()) {
        physicalLayer.StartReceivingFrames();

        if ((macState == IdleState) && (!physicalLayer.ChannelIsClear())) {
            macState = BusyMediumState;
        }//if//
    }//if//

    if (macState == IdleState) {
        (*this).macState = TransientState;
        (*this).StartBackoffIfNecessary();
    }//if//

}//SetRestrictedAccessWindowEndTimeAndRestartMac//



inline
void Dot11Mac::StartRestrictedAccessWindowPeriod(const SimTime& anRawEndTime)
{
    if (!nonRawBackoffInformationHasBeenSaved) {
        (*this).SaveNonRawBackoffInfo();
    }//if//

    (*this).SetAllContentionWindowsToMinAndRegenerateBackoff();

    (*this).SetRestrictedAccessWindowEndTimeAndRestartMac(anRawEndTime);

}//StartRestrictedAccessWindowPeriod//


inline
void Dot11Mac::SwitchToNormalAccessMode()
{
    if (nonRawBackoffInformationHasBeenSaved) {
        (*this).RestoreNonRawBackoffInfo();
    }//if//

    (*this).SetRestrictedAccessWindowEndTimeAndRestartMac(INFINITE_TIME);
}


inline
void Dot11Mac::SwitchToReceiveOnlyMode()
{
    (*this).ahRestrictedAccessWindowEndTime = ZERO_TIME;

    if (physicalLayer.IsNotReceivingFrames()) {
        physicalLayer.StartReceivingFrames();
    }//if//
}


inline
void Dot11Mac::SwitchToSleepMode()
{
    (*this).ahRestrictedAccessWindowEndTime = ZERO_TIME;
    if (!physicalLayer.IsNotReceivingFrames()) {
        physicalLayer.StopReceivingFrames();
    }//if//
}





inline
void Dot11Mac::DoTransmissionTooLongForRawPeriodAction(const unsigned int accessCategoryIndex)
{
    // End the current contention period.
    (*this).ahRestrictedAccessWindowEndTime = ZERO_TIME;
    (*this).macState = IdleState;

    // Regenerate new random backoff using current contention window for next RAW contention period.

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    (*this).RecalcRandomBackoff(accessCategoryInfo);

    accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff = false;

}//DoTransmissionTooLongForRawPeriodAction//



//--------------------------------------------------------------------------------------------------

inline
void SimpleMacAddressResolver::GetMacAddress(
    const NetworkAddress& aNetworkAddress,
    const NetworkAddress& networkAddressMask,
    bool& wasFound,
    MacAddress& resolvedMacAddress)
{
    //assume lower bits of NetworkAddress is node ID.

    wasFound = true;

    assert(MacAddress::numberMacAddressBytes == 6);

    if (aNetworkAddress.IsTheBroadcastAddress()) {
        resolvedMacAddress = MacAddress::GetBroadcastAddress();
    }
    else if (aNetworkAddress.IsAMulticastAddress()) {
        if (macPtr->GetIpMulticastAddressToMacAddressMappingIsEnabled()) {
            resolvedMacAddress.SetToAMulticastAddress(aNetworkAddress.GetMulticastGroupNumber());
        }
        else {
            resolvedMacAddress = MacAddress::GetBroadcastAddress();
        }//if//
    }
    else {
        const NodeId theNodeId =
            aNetworkAddress.MakeAddressWithZeroedSubnetBits(networkAddressMask).GetRawAddressLow32Bits();

        switch (macPtr->GetOperationMode()) {
        case AdhocMode:
            resolvedMacAddress = MacAddress(theNodeId, Dot11Mac::AdhocModeAddressSelectorByte);
            break;

        case InfrastructureMode:
        case ApMode:
            macPtr->LookupMacAddressForNeighbor(theNodeId, wasFound, resolvedMacAddress);
            break;

        default:
            assert(false && "Unhandled Case"); abort();

        }//switch//

    }//if//

}//GetMacAddress//


inline
void SimpleMacAddressResolver::GetNetworkAddressIfAvailable(
    const MacAddress& macAddress,
    const NetworkAddress& subnetNetworkAddress,
    bool& wasFound,
    NetworkAddress& resolvedNetworkAddress)
{
    wasFound = true;
    resolvedNetworkAddress =
        NetworkAddress(subnetNetworkAddress, NetworkAddress(macAddress.ExtractNodeId()));

}//GetNetworkAddress//


}//namespace//

#endif
