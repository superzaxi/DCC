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
//--------------------------------------------------------------------------------------------------
//
// Note on Access Categories / MAC level priorities / Traffic ID:
// QoS using "Traffic ID"'s and priorities finer grained than the 4 Access Catagories
// (AC_BK, AC_BE, AC_VI, AC_VO) never gained traction in 802.11 industry and
// thus in this model currently (AC Index == MAC priority == Traffic ID).
// (Previous versions of this model had more (unused) flexibility).
// AC Index is used in most places but the "Traffic ID" moniker is used in
// 802.11 frame formats and (MAC) "priority" moniker is used with respect to the
// output queues (generic queue datastructure).  Also, IP/Ethernet priorities (0-7)
// can be mapped to the MAC priorities (0-3), if desired.
//


#ifndef DOT11_MAC_H
#define DOT11_MAC_H

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "scensim_prop.h"
#include "scensim_bercurves.h"
#include "scensim_support.h"
#include "scensim_netif.h"

#include "dot11_common.h"
#include "dot11_headers.h"
#include "dot11_phy.h"
#include "dot11_ratecontrol.h"
#include "dot11_txpowercontrol.h"
#include "dot11_mac_ap.h"
#include "dot11_mac_sta.h"
#include "dot11_incoming_buffer.h"
#include "dot11_tracedefs.h"

#include <queue>
#include <map>
#include <string>
#include <vector>
#include <iomanip>

namespace Dot11 {

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
using ScenSim::AbstractOutputQueueWithPrioritySubqueues;
using ScenSim::OutputQueueWithPrioritySubqueues;
using ScenSim::PacketPriority;
using ScenSim::ConvertToPacketPriority;
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
using ScenSim::DecrementTwelveBitSequenceNumber;
using ScenSim::AddTwelveBitSequenceNumbers;
using ScenSim::CalcTwelveBitSequenceNumberDifference;
using ScenSim::RoundUpToNearestIntDivisibleBy4;
using ScenSim::ConvertToUShortInt;
using ScenSim::ConvertToUChar;
using ScenSim::TwelveBitSequenceNumberIsLessThan;
using ScenSim::TwelveBitSequenceNumberPlus1;
using ScenSim::InvalidTwelveBitSequenceNumber;

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

class BasicOutputQueuePriorityMapper: public ScenSim::OutputQueuePriorityMapper {
public:
    BasicOutputQueuePriorityMapper(const vector<PacketPriority>& initPriorityMapping);

    virtual PacketPriority MaxMappedPriority() const override
        { return (static_cast<PacketPriority>(priorityMapping.size() - 1)); }

    virtual PacketPriority MapIpToMacPriority(const PacketPriority& ipPriority) const override
        { return (priorityMapping.at(ipPriority)); }
private:
    vector<PacketPriority> priorityMapping;

};//BasicOutputQueuePriorityMapper//

inline
BasicOutputQueuePriorityMapper::BasicOutputQueuePriorityMapper(
    const vector<PacketPriority>& initPriorityMapping)
    :
    priorityMapping(initPriorityMapping)
{}


class MapToZeroOutputQueuePriorityMapper: public ScenSim::OutputQueuePriorityMapper {
public:
    virtual PacketPriority MaxMappedPriority() const override
        { return (ScenSim::MaxAvailablePacketPriority); }

    virtual PacketPriority MapIpToMacPriority(const PacketPriority& ipPriority) const override
        { return 0; }

};//MapToZeroOutputQueuePriorityMapper//


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

    virtual SimTime CalculateTxopDuration(const unsigned int accessCategoryIndex) const = 0;

    // FIFO Flow: Next flow (destination+AC) to schedule is defined by the front packet
    //              in the queue.
    // NON-FIFO Flow: Scheduler can pick next STA to send to.

    virtual bool NonFifoFlowSchedulingModeIsOn() const = 0;

    // For peeking at next packets to schedule.

    virtual NetworkAddress CalcNextDestinationToPossiblySchedule(
        const unsigned int accessCategoryIndex) const = 0;

    // Called when actually scheduling.

    virtual void CalcNextDestinationToSchedule(
        const unsigned int accessCategoryIndex,
        NetworkAddress& newStaAddress) = 0;

    virtual bool AllowNextNonFifoFrameToBeAggregated(
        const unsigned int accessCategoryIndex,
        const NetworkAddress& nextHopAddress,
        const unsigned int currentAggregateFrameSizeBytes) const = 0;

    //Future Feature: virtual void MonitorQueueingOfOutgoingPacket(const size_t accessCategoryIndex) = 0;
    //Future Feature:
    //Future Feature: virtual void MonitorTransmissionOfFrame(
    //Future Feature:    const size_t accessCategory,
    //Future Feature:    const MacAddress& destinationAddress,
    //Future Feature:    const unsigned int numberFrameBytes,
    //Future Feature:    const DatarateBitsPerSec& theDatarateBitsPerSec) = 0;
    //Future Feature:
    //Future Feature: virtual void MonitorIncomingFrame(
    //Future Feature:    const size_t accessCategory,
    //Future Feature:    const MacAddress& sourceAddress,
    //Future Feature:    const unsigned int numberFrameBytes,
    //Future Feature:    const DatarateBitsPerSec& theDatarateBitsPerSec) = 0;

};//Dot11MacAccessPointScheduler//


// Schedulers are normally in the 802.11 APs and TXOP durations are sent to STAs via
// beacon frames.  As a stopgap, fixed TXOPs can be set for individual STAs.

class FifoFixedTxopScheduler: public Dot11MacAccessPointScheduler {
public:

    FifoFixedTxopScheduler(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId theNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int numberAccessCategories,
        const shared_ptr<AbstractOutputQueueWithPrioritySubqueues>& networkOutputQueuePtr);

    virtual SimTime CalculateTxopDuration(const unsigned int accessCategoryIndex) const override
        { return accessCategoryInfos.at(accessCategoryIndex).txopDuration; }


    virtual bool NonFifoFlowSchedulingModeIsOn() const override { return false; }

    virtual NetworkAddress CalcNextDestinationToPossiblySchedule(
        const unsigned int accessCategoryIndex) const override
        { assert(false); abort(); return (NetworkAddress()); }

    virtual void CalcNextDestinationToSchedule(
        const unsigned int accessCategoryIndex,
        NetworkAddress& newStaAddress) override { assert(false); abort(); }

    virtual bool AllowNextNonFifoFrameToBeAggregated(
        const unsigned int accessCategoryIndex,
        const NetworkAddress& nextHopAddress,
        const unsigned int currentAggregateFrameSizeBytes) const override;

private:
    struct AccessCategoryInfo {
        SimTime txopDuration;
        unsigned int maxNonFifoMpduAggregateSize;

        AccessCategoryInfo() : txopDuration(ZERO_TIME), maxNonFifoMpduAggregateSize(0) {}
    };

    vector<AccessCategoryInfo> accessCategoryInfos;
    shared_ptr<ScenSim::AbstractOutputQueueWithPrioritySubqueues> networkOutputQueuePtr;

};//FifoFixedTxopScheduler//


inline
FifoFixedTxopScheduler::FifoFixedTxopScheduler(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId theNodeId,
    const InterfaceId& theInterfaceId,
    const unsigned int numberAccessCategories,
    const shared_ptr<AbstractOutputQueueWithPrioritySubqueues>& initNetworkOutputQueuePtr)
    :
    networkOutputQueuePtr(initNetworkOutputQueuePtr)
{
    accessCategoryInfos.resize(numberAccessCategories);

    for(size_t i = 0; (i < accessCategoryInfos.size()); i++) {
        AccessCategoryInfo& accessCategoryInfo = accessCategoryInfos[i];

        ostringstream nameStream;
        nameStream << parameterNamePrefix << "edca-category-" << i;
        const string parmNamePrefix = nameStream.str();

        if (theParameterDatabaseReader.ParameterExists(
            (parmNamePrefix + "-downlink-txop-duration"), theNodeId, theInterfaceId)) {
            accessCategoryInfo.txopDuration =
                theParameterDatabaseReader.ReadTime(
                    (parmNamePrefix + "-downlink-txop-duration"), theNodeId, theInterfaceId);
        }//if//

        if (theParameterDatabaseReader.ParameterExists(
            (parmNamePrefix + "-max-non-fifo-aggregate-size-bytes"), theNodeId, theInterfaceId)) {
            accessCategoryInfo.maxNonFifoMpduAggregateSize =
                theParameterDatabaseReader.ReadNonNegativeInt(
                    (parmNamePrefix + "-max-non-fifo-aggregate-size-bytes"), theNodeId, theInterfaceId);

            if (accessCategoryInfo.maxNonFifoMpduAggregateSize > 0) {
                networkOutputQueuePtr->EnableNextHopSpecificDequeues();
            }//if//
        }//if//
    }//for//

}//FixedTxopAccessPointScheduler//


inline
bool FifoFixedTxopScheduler::AllowNextNonFifoFrameToBeAggregated(
    const unsigned int accessCategoryIndex,
    const NetworkAddress& nextHopAddress,
    const unsigned int currentAggregateFrameSizeBytes) const
{
    const AccessCategoryInfo& accessCategoryInfo =
        accessCategoryInfos.at(accessCategoryIndex);

    if (accessCategoryInfo.maxNonFifoMpduAggregateSize == 0) {
        return false;
    }//if//

    if (!networkOutputQueuePtr->HasPacketWithPriorityAndNextHop(
        ConvertToPacketPriority(accessCategoryIndex), nextHopAddress)) {

        return false;
    }//if//

    const Packet& nextFrame =
        networkOutputQueuePtr->GetNextPacketWithPriorityAndNextHop(
            ConvertToPacketPriority(accessCategoryIndex), nextHopAddress);

    // Assuming MPDU aggregation

    const unsigned int newTotalSizeBytes =
        currentAggregateFrameSizeBytes +
        sizeof(QosDataFrameHeader) + sizeof(MpduDelimiterFrame) +
        nextFrame.LengthBytes();

   return (newTotalSizeBytes <= accessCategoryInfo.maxNonFifoMpduAggregateSize);

}//AllowNextNonFifoFrameToBeAggregated//



//--------------------------------------------------------------------------------------------------


class RoundRobinFixedTxopScheduler: public Dot11MacAccessPointScheduler {
public:

    RoundRobinFixedTxopScheduler(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId theNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int numberAccessCategories,
        const shared_ptr<AbstractOutputQueueWithPrioritySubqueues>& networkOutputQueuePtr);

    virtual SimTime CalculateTxopDuration(const unsigned int accessCategoryIndex) const override
        { return accessCategoryInfos.at(accessCategoryIndex).txopDuration; }

    virtual bool NonFifoFlowSchedulingModeIsOn() const override { return true; }

    virtual NetworkAddress CalcNextDestinationToPossiblySchedule(
        const unsigned int accessCategoryIndex) const override;

    virtual void CalcNextDestinationToSchedule(
        const unsigned int accessCategoryIndex,
        NetworkAddress& newStaAddress) override;

    virtual bool AllowNextNonFifoFrameToBeAggregated(
        const unsigned int accessCategoryIndex,
        const NetworkAddress& nextHopAddress,
        const unsigned int currentAggregateFrameSizeBytes) const override;

private:
    struct AccessCategoryInfo {
        SimTime txopDuration;
        unsigned int maxNonFifoMpduAggregateSize;
        NetworkAddress lastStaNetworkAddress;

        AccessCategoryInfo() :
            txopDuration(ZERO_TIME),
            maxNonFifoMpduAggregateSize(0),
            lastStaNetworkAddress(NetworkAddress::invalidAddress) {}
    };

    vector<AccessCategoryInfo> accessCategoryInfos;
    shared_ptr<ScenSim::AbstractOutputQueueWithPrioritySubqueues> networkOutputQueuePtr;

};//RoundRobinFixedTxopScheduler//


inline
RoundRobinFixedTxopScheduler::RoundRobinFixedTxopScheduler(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId theNodeId,
    const InterfaceId& theInterfaceId,
    const unsigned int numberAccessCategories,
    const shared_ptr<AbstractOutputQueueWithPrioritySubqueues>& initNetworkOutputQueuePtr)
    :
    networkOutputQueuePtr(initNetworkOutputQueuePtr)
{
    networkOutputQueuePtr->EnableNextHopSpecificDequeues();

    accessCategoryInfos.resize(numberAccessCategories);

    for(size_t i = 0; (i < accessCategoryInfos.size()); i++) {
        AccessCategoryInfo& accessCategoryInfo = accessCategoryInfos[i];

        ostringstream nameStream;
        nameStream << parameterNamePrefix << "edca-category-" << i;
        const string parmNamePrefix = nameStream.str();

        if (theParameterDatabaseReader.ParameterExists(
            (parmNamePrefix + "-downlink-txop-duration"), theNodeId, theInterfaceId)) {
            accessCategoryInfo.txopDuration =
                theParameterDatabaseReader.ReadTime(
                    (parmNamePrefix + "-downlink-txop-duration"), theNodeId, theInterfaceId);
        }//if//

        if (theParameterDatabaseReader.ParameterExists(
            (parmNamePrefix + "-max-non-fifo-aggregate-size-bytes"), theNodeId, theInterfaceId)) {
            accessCategoryInfo.maxNonFifoMpduAggregateSize =
                theParameterDatabaseReader.ReadNonNegativeInt(
                    (parmNamePrefix + "-max-non-fifo-aggregate-size-bytes"), theNodeId, theInterfaceId);

            if (accessCategoryInfo.maxNonFifoMpduAggregateSize > 0) {
                networkOutputQueuePtr->EnableNextHopSpecificDequeues();
            }//if//
        }//if//
    }//for//

}//FixedTxopAccessPointScheduler//



inline
NetworkAddress RoundRobinFixedTxopScheduler::CalcNextDestinationToPossiblySchedule(
    const unsigned int accessCategoryIndex) const
{
    const AccessCategoryInfo& info = accessCategoryInfos.at(accessCategoryIndex);

    if (info.lastStaNetworkAddress == NetworkAddress::invalidAddress) {
        return (
            networkOutputQueuePtr->NextHopAddressForTopPacket(
                ConvertToPacketPriority(accessCategoryIndex)));
    }
    else {
        return (
            networkOutputQueuePtr->GetNetworkAddressOfNextActiveStationAfter(
                ConvertToPacketPriority(accessCategoryIndex),
                info.lastStaNetworkAddress));
    }//if//

}//CalcNextDestinationToPossiblySchedule//



inline
void RoundRobinFixedTxopScheduler::CalcNextDestinationToSchedule(
    const unsigned int accessCategoryIndex,
    NetworkAddress& newStaAddress)
{
    AccessCategoryInfo& info = accessCategoryInfos.at(accessCategoryIndex);

    if (info.lastStaNetworkAddress == NetworkAddress::invalidAddress) {

        newStaAddress =
            networkOutputQueuePtr->NextHopAddressForTopPacket(
                ConvertToPacketPriority(accessCategoryIndex));
    }
    else {
        newStaAddress =
            networkOutputQueuePtr->GetNetworkAddressOfNextActiveStationAfter(
                ConvertToPacketPriority(accessCategoryIndex),
                info.lastStaNetworkAddress);
    }//if//

    info.lastStaNetworkAddress = newStaAddress;

}//CalcNextDestinationToSchedule//



inline
bool RoundRobinFixedTxopScheduler::AllowNextNonFifoFrameToBeAggregated(
    const unsigned int accessCategoryIndex,
    const NetworkAddress& nextHopAddress,
    const unsigned int currentAggregateFrameSizeBytes) const
{
    const AccessCategoryInfo& accessCategoryInfo =
        accessCategoryInfos.at(accessCategoryIndex);

    if (accessCategoryInfo.maxNonFifoMpduAggregateSize == 0) {
        return false;
    }//if//

    if (!networkOutputQueuePtr->HasPacketWithPriorityAndNextHop(
        ConvertToPacketPriority(accessCategoryIndex), nextHopAddress)) {

        return false;
    }//if//

    const Packet& nextFrame =
        networkOutputQueuePtr->GetNextPacketWithPriorityAndNextHop(
            ConvertToPacketPriority(accessCategoryIndex), nextHopAddress);

    // Assuming MPDU aggregation

    const unsigned int newTotalSizeBytes =
        currentAggregateFrameSizeBytes +
        sizeof(QosDataFrameHeader) + sizeof(MpduDelimiterFrame) +
        nextFrame.LengthBytes();

   return (newTotalSizeBytes <= accessCategoryInfo.maxNonFifoMpduAggregateSize);

}//AllowNextNonFifoFrameToBeAggregated//



//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

class Dot11MacDurationCalculationInterface {
public:
    Dot11MacDurationCalculationInterface(const Dot11Mac* initMacPtr)
        :
        macPtr(initMacPtr)
    {}

    SimTime CalcContentionWindowMinBackoffDuration(const unsigned short retryCount) const;

    SimTime CalcMaxTxDurationForUnicastDataWithRts(
        const unsigned int packetWithMacHeaderSizeBytes,
        const TransmissionParameters& dataFrameTxParameters,
        const TransmissionParameters& managementFrameTxParameters) const;

    SimTime CalcMaxTxDurationForUnicastDataWithoutRts(
        const unsigned int packetWithMacHeaderSizeBytes,
        const TransmissionParameters& dataFrameTxParameters,
        const TransmissionParameters& managementFrameTxParameters) const;

    DatarateBitsPerSec CalcDatarateBitsPerSecond(
        const TransmissionParameters& txParameters) const;

    unsigned int GetMaxNumMimoSpatialStreams() const;

private:
    const Dot11Mac* macPtr;

};//Dot11MacDurationCalculationInterface//


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
        const shared_ptr<MimoChannelModelInterface>& mimoChannelModelInterfacePtr,
        const shared_ptr<FrequencySelectiveFadingModelInterface>& frequencySelectiveFadingModelInterfacePtr,
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
                mimoChannelModelInterfacePtr,
                frequencySelectiveFadingModelInterfacePtr,
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
        const shared_ptr<Dot11::AdaptiveRateController>& rateControllerPtr)
    {
        this->theAdaptiveRateControllerPtr = rateControllerPtr;
    }

    shared_ptr<Dot11::AdaptiveRateController> GetAdaptiveRateControllerPtr() const
    {
        return (theAdaptiveRateControllerPtr);
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

    Dot11MacOperationMode GetOperationMode() const { return (operationMode); }

    bool GetIpMulticastAddressToMacAddressMappingIsEnabled() const
        { return (ipMulticastAddressToMacAddressMappingIsEnabled); }

    void SendManagementFrame(unique_ptr<Packet>& framePtr);

    bool IsAHighThroughputStation() const { return (physicalLayer.GetIsAHighThroughputStation()); }
    unsigned int GetBaseChannelNumber() const { return (physicalLayer.GetBaseChannelNumber()); }
    unsigned int GetNumberOfChannels() const { return (physicalLayer.GetChannelCount()); }
    unsigned int GetCurrentChannelId() const { return (physicalLayer.GetCurrentChannelNumber()); }
    unsigned int GetMaxBandwidthNumChannels() const
        { return (physicalLayer.GetMaxBandwidthNumChannels()); }

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

    void ResetOutgoingLinksTo(const MacAddress& macAddress);

    void RequeueBufferedPacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        const EtherTypeField etherType,
        const SimTime& queueInsertionTime,
        const unsigned int txRetryCount);

    void RequeueManagementFrame(
        unique_ptr<Packet>& framePtr,
        const unsigned int retryTxCount);

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

    SimTime CalcContentionWindowMinBackoffDuration(const unsigned short retryCount) const;

    DatarateBitsPerSec CalcDatarateBitsPerSecond(const TransmissionParameters& txParameters) const;

    SimTime CalcMaxTxDurationForUnicastDataWithRts(
        const unsigned int packetWithMacHeaderSizeBytes,
        const TransmissionParameters& dataFrameTxParameters,
        const TransmissionParameters& managementFrameTxParameters) const;

    SimTime CalcMaxTxDurationForUnicastDataWithoutRts(
        const unsigned int packetWithMacHeaderSizeBytes,
        const TransmissionParameters& dataFrameTxParameters,
        const TransmissionParameters& managementFrameTxParameters) const;

    unsigned int GetMaxNumMimoSpatialStreams() const;

    double getCBR(){
        //cout << "macCBR: " << physicalLayer.channelBusyRatio_phy << endl;
        return physicalLayer.channelBusyRatio_phy;
    };

private:

    Dot11Mac(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<PropFrame> >& propModelInterfacePtr,
        const shared_ptr<MimoChannelModelInterface>& mimoChannelModelInterfacePtr,
        const shared_ptr<FrequencySelectiveFadingModelInterface>& frequencySelectiveFadingModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const RandomNumberGeneratorSeed& nodeSeed);

    void CompleteInitialization(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const RandomNumberGeneratorSeed& nodeSeed);

    //---------------------------------------------------------------
    
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

    shared_ptr<AbstractOutputQueueWithPrioritySubqueues> networkOutputQueuePtr;
    SimTime lastPacketLifetimeQueuePurgeTime;

    bool refreshNextHopOnDequeueModeIsOn;

    struct ManagementFrameQueueElem {
        unique_ptr<Packet> framePtr;
        unsigned int retryTxCount;

        ManagementFrameQueueElem(
            unique_ptr<Packet>&& initFramePtr,
            const unsigned int initRetryTxCount)
        :
            framePtr(move(initFramePtr)), retryTxCount(initRetryTxCount)
        {}

        void operator=(ManagementFrameQueueElem&& right)
        {
            assert(this != &right);
            framePtr = move(right.framePtr);
            retryTxCount = right.retryTxCount;
        }//=//

        ManagementFrameQueueElem(ManagementFrameQueueElem&& right) { (*this) = move(right); }
    };

    deque<ManagementFrameQueueElem> managementFrameQueue;

    IncomingFrameBuffer theIncomingFrameBuffer;

    shared_ptr<AdaptiveRateController> theAdaptiveRateControllerPtr;
    shared_ptr<AdaptiveTxPowerController> theAdaptiveTxPowerControllerPtr;

    Dot11Phy physicalLayer;

    unique_ptr<MacAddressResolver<MacAddress> > theMacAddressResolverPtr;

    unique_ptr<Dot11MacAccessPointScheduler> macSchedulerPtr;

    //-----------------------------------------------------

    //dcc
    /*std::map <SimTime, long int> packetSizeforDCC;
    SimTime DCCduration_before = 488000;
    SimTime currentTime_before = 0;
    double delta = 0; 
    int status = 0;
    int adaptiveCount = 0;
    double averageChannelBusyRatio = 0;
    double CBR_L_0_Hop_Previous = 0;
    SimTime get_duration();
    SimTime calcReactiveSendRate();
    double calcAdaptiveSendRate();*/

    //-----------------------------------------------------

    // Acronyms: AIFS,DIFS,EIFS,SIFS: {Arbitration, Distributed, Extended, Short} InterFrame Space

    // EDCA = "Enhanced Distributed Channel Access" for QoS (added in 802.11e)
    // DCF = "Distributed Coordination Function" (802.11 classic access protocol).

    // Mac Non-EDCA specific parameters:

    unsigned int rtsThresholdSizeBytes;

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
    unsigned int maxNumAggregateSubframes;

    // Legacy results parameter:

    bool isInStrictlyFifoRetriesMode;
    bool disallowAddingNewPacketsToRetries;

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

    unsigned int powerSavePollResponsePacketTxCount;

    SimTime mediumReservedUntilTimeAkaNAV;

    bool lastFrameReceivedWasCorrupt;

    // Used by block ack processing.

    MacAddress currentIncomingAggregateFramesSourceMacAddress;
    PacketPriority currentIncomingAggregateFramesTrafficId;
    unsigned int numSubframesReceivedFromCurrentAggregateFrame;

    // Note the AC is (now) also the priority/"Traffic ID".
    PacketPriority currentPacketsAccessCategoryIndex;

    unique_ptr<Packet> currentPacketPtr;
    unique_ptr<vector<unique_ptr<Packet> > > currentAggregateFramePtr;

    bool currentPacketIsForTxopCollisionDetect;

    bool currentPacketIsAManagementFrame;
    NetworkAddress currentPacketsNextHopNetworkAddress;
    MacAddress currentPacketsDestinationMacAddress;
    unsigned short int currentPacketsSequenceNumber;
    EtherTypeField currentPacketsEtherType;
    SimTime currentPacketsQueueInsertionTime;

    unsigned int requestToSendRetryTxLimit;
    unsigned int blockAckRequestRetryTxLimit;

    unsigned int shortFrameRetryTxLimit;
    unsigned int longFrameRetryTxLimit;

    // Retry limit can be dynamically overridden by rate control algorithms (Minstrel).

    unsigned int currentFrameRetryTxLimit;

    unsigned int currentPacketsRetryTxCount;

    struct AggregateSubframeRetryInfo {
        SimTime queueInsertionTime;
        unsigned int retryTxCount;
        AggregateSubframeRetryInfo() : retryTxCount(0), queueInsertionTime(INFINITE_TIME) {}
        AggregateSubframeRetryInfo(
            const SimTime& initQueueInsertionTime,
            const unsigned int initRetryTxCount)
            :
            queueInsertionTime(initQueueInsertionTime),
            retryTxCount(initRetryTxCount)
        {}
    };

    vector<AggregateSubframeRetryInfo> currentAggregateFrameRetryInfo;

    //-----------------------------------------------------

    struct EdcaAccessCategoryInfo {
        // Parameters

        unsigned int minContentionWindowSlots;
        unsigned int maxContentionWindowSlots;
        unsigned int arbitrationInterframeSpaceDurationSlots;
        SimTime transmitOpportunityDurationAkaTxop;
        SimTime frameLifetime;//dot11EDCATableMSDULifetime//

        // State variables
        unsigned int currentContentionWindowSlots;
        unsigned int currentNumOfBackoffSlots;

        // Calculated backoff duration without dynamically added Extended Interframe space.
        SimTime currentNonExtendedBackoffDuration;

        bool tryingToJumpOnMediumWithoutABackoff;
        bool hasPacketToSend;

        SimTime ifsAndBackoffStartTime;

        // currentPacketPtr (non-aggregate) sent before aggregate frame.

        // Only set if pure FIFO retries option is enabled.

        bool shouldRetransmitLastDataPacket;

        EdcaAccessCategoryInfo()
            :
            currentNumOfBackoffSlots(0),
            currentNonExtendedBackoffDuration(INFINITE_TIME),
            hasPacketToSend(false),
            tryingToJumpOnMediumWithoutABackoff(false),
            frameLifetime(INFINITE_TIME),
            transmitOpportunityDurationAkaTxop(ZERO_TIME),
            ifsAndBackoffStartTime(INFINITE_TIME),
            shouldRetransmitLastDataPacket(false)
        {}

        void operator=(EdcaAccessCategoryInfo&& right)
        {
            assert(this != &right);

            minContentionWindowSlots = right.minContentionWindowSlots;
            maxContentionWindowSlots = right.maxContentionWindowSlots;
            arbitrationInterframeSpaceDurationSlots = right.arbitrationInterframeSpaceDurationSlots;
            transmitOpportunityDurationAkaTxop = right.transmitOpportunityDurationAkaTxop;
            frameLifetime = right.frameLifetime;
            currentContentionWindowSlots = right.currentContentionWindowSlots;
            currentNumOfBackoffSlots = right.currentNumOfBackoffSlots;
            currentNonExtendedBackoffDuration = right.currentNonExtendedBackoffDuration;
            tryingToJumpOnMediumWithoutABackoff = right.tryingToJumpOnMediumWithoutABackoff;
            hasPacketToSend = right.hasPacketToSend;
            ifsAndBackoffStartTime = right.ifsAndBackoffStartTime;
            shouldRetransmitLastDataPacket = right.shouldRetransmitLastDataPacket;
        }//=//

        EdcaAccessCategoryInfo(EdcaAccessCategoryInfo&& right) { (*this) = move(right); }

    };//EdcaAccessCatagoryInfo//


    //-----------------------------------------------------

    vector<EdcaAccessCategoryInfo> accessCategories;

    unsigned int numberAccessCategories;

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


    // Note: "Traffic IDs" are currently just the MAC priority = Access Category

    struct AddressAndTrafficIdMapKey {
        MacAddress macAddress;
        PacketPriority trafficId;

        AddressAndTrafficIdMapKey(
            const MacAddress& initMacAddress, const PacketPriority& initTrafficId)
            :
            macAddress(initMacAddress), trafficId(initTrafficId) {}

        bool operator<(const AddressAndTrafficIdMapKey& right) const {
            return ((macAddress < right.macAddress) ||
                    ((macAddress == right.macAddress) && (trafficId < right.trafficId)));
        }
    };

    struct OutgoingLinkInfo {
        unsigned short int outgoingSequenceNumber;
        // Frames at and before this sequence number will not be resent.
        unsigned short int lowestLiveFrameSequenceNumber;
        unsigned short int requestToSendOrBarTxCount;

        bool blockAckSessionIsEnabled;
        bool blockAckRequestNeedsToBeSent;

        OutgoingLinkInfo() :
            outgoingSequenceNumber(InvalidTwelveBitSequenceNumber),
            lowestLiveFrameSequenceNumber(InvalidTwelveBitSequenceNumber),
            blockAckSessionIsEnabled(false),
            blockAckRequestNeedsToBeSent(false),
            requestToSendOrBarTxCount(0) {}
    };

    map<AddressAndTrafficIdMapKey, OutgoingLinkInfo> outgoingLinkInfoMap;


    struct NeighborCapabilities {
        bool mpduFrameAggregationIsEnabled;

        NeighborCapabilities() : mpduFrameAggregationIsEnabled(false) {}
    };

    bool allowMpduAggregationForMulticastAddresses;

    map<MacAddress, NeighborCapabilities> neighborCapabilitiesMap;

    // Statistics:

    //dropped by exceeded retries
    shared_ptr<CounterStatistic> droppedPacketsStatPtr;

    //sent/received mac frame bytes for L2 throughput
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;

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

    void InitializeAccessCategories(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        shared_ptr<ScenSim::OutputQueuePriorityMapper>& outputQueuePriorityMapperPtr);


    void MakePriorityToAccessCategoryMapping(
        const vector<vector<PacketPriority> > prioritiesForAccessCategories,
        vector<PacketPriority>& priorityMapping);


    void ReadBondedChannelList(
        const string& bondedChannelListString,
        const string& parameterNameForErrorOutput,
        vector<unsigned int>& bondedChannelList);

    void ReadInMulticastGroupNumberList(const ParameterDatabaseReader& theParameterDatabaseReader);

    void SetAccessCategoriesAsEdca(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        shared_ptr<ScenSim::OutputQueuePriorityMapper>& outputQueuePriorityMapperPtr);

    void SetAccessCategoriesAsDcf(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        shared_ptr<ScenSim::OutputQueuePriorityMapper>& outputQueuePriorityMapperPtr);

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
        const MacAddress& destinationMacAddress,
        const TransmissionParameters& txParameters) const;

    SimTime CalculateNavDurationForAckedAggregateDataFrame(
        const MacAddress& destinationMacAddress,
        const TransmissionParameters& txParameters) const;

    void RemoveLifetimeExpiredPacketsFromQueue(const unsigned int accessCategoryIndex);

    SimTime CalculateNextFrameSequenceDuration(const unsigned int accessCategoryIndex) const;

    void AddExtraNavDurationToPacketForNextFrameIfInATxop(
        const unsigned int accessCategoryIndex,
        const SimTime& frameTransmissionEndTime,
        Packet& thePacket) const;

    void CalculateAndSetTransmitOpportunityDurationAkaTxop(const unsigned int accessCategoryIndex);

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

    void DropCurrentPacketAndGoToNextPacket();

    void NeverReceivedClearToSendOrAcknowledgementAction();

    void NoResponseToAggregateFrameAction();

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

    //void CheckPriorityToAccessCategoryMapping() const;

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

    //----------------------------------

    bool FrameAggregationIsEnabledFor(const MacAddress& destinationAddress) const;
    bool MpduFrameAggregationIsEnabledFor(const MacAddress& destinationAddress) const;

    bool CanAggregateCurrentFrame() const;
    void BuildAggregateFrameFromCurrentFrame();

    void SendPacketToNetworkLayer(unique_ptr<Packet>& dataFramePtr);

    void SendBlockAcknowledgementFrame(
        const MacAddress& destinationAddress,
        const PacketPriority& trafficId,
        const TransmissionParameters& receivedFrameTxParameters);

    void RequeuePacketForRetry(
        const unsigned int accessCategoryIndex,
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        const EtherTypeField etherType,
        const SimTime& queueInsertionTime,
        const unsigned int txRetryCount);

    unsigned int CalcNormalRetryTxLimit(const unsigned int packetLengthBytes) const;

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

    void OutputTraceAndStatsForRtsFrameTransmission(
        const unsigned int rtsRetryTxCount,
        const unsigned int frameSizeBytes) const;
    void OutputTraceForCtsFrameTransmission(const unsigned int frameSizeBytes) const;
    void OutputTraceAndStatsForAggregatedFrameTransmission() const;
    void OutputTraceAndStatsForBroadcastDataFrameTransmission() const;
    void OutputTraceAndStatsForUnicastDataFrameTransmission() const;
    void OutputTraceAndStatsForManagementFrameTransmission() const;
    void OutputTraceForAckFrameTransmission(const unsigned int frameSizeBytes) const;
    void OutputTraceAndStatsForBlockAckRequestFrameTransmission(const unsigned int frameSizeBytes) const;
    void OutputTraceAndStatsForBlockAckFrameTransmission(const unsigned int frameSizeBytes) const;

    void OutputTraceForCtsOrAckTimeout() const;

    void OutputTraceAndStatsForPacketRetriesExceeded() const;
    bool redundantTraceInformationModeIsOn;

};//Dot11Mac//



const unsigned int DefaultRtsThresholdSizeBytes = UINT_MAX;
const unsigned int DefaultContentionWindowSlotsMin = 15;
const unsigned int DefaultContentionWindowSlotsMax = 1023;
const unsigned int MaxNumberAccessCategories = 4;

inline
SimTime Dot11Mac::CalculateAdditionalDelayForExtendedInterframeSpace() const
{
    TransmissionParameters txParameters;
    txParameters.bandwidthNumberChannels = 1;

    txParameters.modulationAndCodingScheme =
        theAdaptiveRateControllerPtr->GetLowestModulationAndCoding();

    SimTime longestPossibleAckDuration =
        CalculateFrameDuration(sizeof(AcknowledgementAkaAckFrame), txParameters);

    return (aShortInterframeSpaceTime + longestPossibleAckDuration);

}//CalculateAdditionalDelayForExtendedInterframeSpace//



inline
void Dot11Mac::MakePriorityToAccessCategoryMapping(
    const vector<vector<PacketPriority> > prioritiesForAccessCategories,
    vector<PacketPriority>& priorityMapping)
{
    priorityMapping.clear();

    bool thereIsAMapping = false;
    PacketPriority maxPacketPriority = 0;
    for(unsigned int i = 0; (i < prioritiesForAccessCategories.size()); i++) {
        for(unsigned j = 0; (j < prioritiesForAccessCategories[i].size()); j++) {
            thereIsAMapping = true;
            if (maxPacketPriority < prioritiesForAccessCategories[i][j]) {
                maxPacketPriority = prioritiesForAccessCategories[i][j];
            }//if//
        }//for//
    }//for//

    if (!thereIsAMapping) {
        return;
    }//if//

    if (maxPacketPriority > 7) {
        cerr << "Error in parameter \"" << (parameterNamePrefix + "priority-list")
             << "\",  maximum 802.11 priority is 7." << endl;
        exit(1);
    }//if//

    const PacketPriority badValue = (maxPacketPriority + 1);
    priorityMapping.resize((maxPacketPriority + 1), badValue);

    for(unsigned int i = 0; (i < prioritiesForAccessCategories.size()); i++) {
        for(unsigned j = 0; (j < prioritiesForAccessCategories[i].size()); j++) {
            if (priorityMapping.at(prioritiesForAccessCategories[i][j]) != badValue) {
                cerr << "Error: Duplicate priority to access category mapping for "
                     << "priority = " << prioritiesForAccessCategories[i][j] << endl;
                exit(1);
            }//if//

            priorityMapping.at(prioritiesForAccessCategories[i][j]) = ConvertToPacketPriority(i);
        }//for//
    }//for//

    for(unsigned int i = 0; (i < priorityMapping.size()); i++) {
        if (priorityMapping[i] == badValue) {
            cerr << "Error: No priority to access category mapping for "
                 << "priority = " << i << endl;
            exit(1);
        }//if//
    }//for//

}//MakePriorityToAccessCategoryMapping//



inline
void Dot11Mac::InitializeAccessCategories(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    shared_ptr<ScenSim::OutputQueuePriorityMapper>& outputQueuePriorityMapperPtr)
{
    string qosTypeString = "edca";

    if (theParameterDatabaseReader.ParameterExists(
            "dot11-qos-type", theNodeId, theInterfaceId)) {

        qosTypeString =  MakeLowerCaseString(
            theParameterDatabaseReader.ReadString("dot11-qos-type", theNodeId, theInterfaceId));

    }//if//

    if (qosTypeString == "edca") {
        (*this).SetAccessCategoriesAsEdca(
            theParameterDatabaseReader, outputQueuePriorityMapperPtr);
    }
    else if (qosTypeString == "dcf") {
        (*this).SetAccessCategoriesAsDcf(
            theParameterDatabaseReader, outputQueuePriorityMapperPtr);
    }
    else {
        cerr << "Error: Unknown dot11-qos-type: " << qosTypeString << endl;
        exit(1);
    }//if//

    for(unsigned int i = 0; (i < accessCategories.size()); i++) {
        accessCategories[i].currentContentionWindowSlots = accessCategories[i].minContentionWindowSlots;
    }//for//

    accessCategoryIndexForManagementFrames = static_cast<unsigned int>(accessCategories.size() - 1);

}//InitializeAccessCategories//


inline
void Dot11Mac::SetAccessCategoriesAsEdca(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    shared_ptr<ScenSim::OutputQueuePriorityMapper>& outputQueuePriorityMapperPtr)
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

    vector<vector<PacketPriority> > prioritiesForAccessCategories(accessCategories.size());

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

            bool success;

            ConvertAStringSequenceOfANumericTypeIntoAVector<PacketPriority>(
                prioritiesInAString,
                success,
                prioritiesForAccessCategories[i]);

            if (!success) {
                cerr << "Error: Configuration: Bad Priority sequence: " << prioritiesInAString << endl;
                exit(1);
            }//if//
        }//if//

        if (theParameterDatabaseReader.ParameterExists((prefix+"frame-lifetime"), theNodeId, theInterfaceId)) {
            accessCategories[i].frameLifetime =
                theParameterDatabaseReader.ReadTime((prefix+"frame-lifetime"), theNodeId, theInterfaceId);
        }//if//
    }//for//

    vector<PacketPriority> priorityMapping;

    MakePriorityToAccessCategoryMapping(prioritiesForAccessCategories, priorityMapping);

    if (!priorityMapping.empty()) {
        outputQueuePriorityMapperPtr = make_shared<BasicOutputQueuePriorityMapper>(priorityMapping);
    }//if//

}//SetAccessCategoriesAsEdca//


inline
void Dot11Mac::SetAccessCategoriesAsDcf(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    shared_ptr<ScenSim::OutputQueuePriorityMapper>& outputQueuePriorityMapperPtr)
{
    //Set defaults for DCF. Maybe overidden.

    accessCategories.clear();
    accessCategories.resize(1);

    EdcaAccessCategoryInfo& accessCategoryForDcf = accessCategories.at(0);
    accessCategoryForDcf.minContentionWindowSlots = contentionWindowSlotsMin;
    accessCategoryForDcf.maxContentionWindowSlots = contentionWindowSlotsMax;
    accessCategoryForDcf.arbitrationInterframeSpaceDurationSlots = 2;

    numberAccessCategories = 1;

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

    outputQueuePriorityMapperPtr = make_shared<MapToZeroOutputQueuePriorityMapper>();

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
            cerr << "(Please set the required number of channels for bonded channel: "
                 << bondedChannelListString << ")" << endl;
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
    const shared_ptr<MimoChannelModelInterface>& mimoChannelModelInterfacePtr,
    const shared_ptr<FrequencySelectiveFadingModelInterface>& frequencySelectiveFadingModelInterfacePtr,
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
    contentionWindowSlotsMin(DefaultContentionWindowSlotsMin),
    contentionWindowSlotsMax(DefaultContentionWindowSlotsMax),
    aShortInterframeSpaceTime(INFINITE_TIME),
    aSlotTime(INFINITE_TIME),
    aRxTxTurnaroundTime(INFINITE_TIME),
    additionalDelayForExtendedInterframeSpaceMode(INFINITE_TIME),
    numberAccessCategories(MaxNumberAccessCategories),
    ackDatarateSelection(SameAsData),
    macState(IdleState),
    mediumReservedUntilTimeAkaNAV(ZERO_TIME),
    mediumBecameIdleTime(ZERO_TIME),
    currentWakeupTimerExpirationTime(INFINITE_TIME),
    lastFrameReceivedWasCorrupt(false),
    numSubframesReceivedFromCurrentAggregateFrame(0),
    currentPacketIsForTxopCollisionDetect(false),
    currentPacketIsAManagementFrame(false),
    currentPacketsAccessCategoryIndex(0),
    currentTransmitOpportunityAkaTxopStartTime(ZERO_TIME),
    currentTransmitOpportunityAkaTxopEndTime(ZERO_TIME),
    currentTransmitOpportunityAckedFrameCount(0),
    redundantTraceInformationModeIsOn(false),
    disabledToJumpOnMediumWithoutBackoff(false),
    protectAggregateFramesWithSingleAckedFrame(true),
    allowFrameAggregationWithTxopZero(false),
    maxAggregateMpduSizeBytes(0),
    maxNumAggregateSubframes(BlockAckBitMapNumBits),
    allowMpduAggregationForMulticastAddresses(false),
    ipMulticastAddressToMacAddressMappingIsEnabled(false),
    isInStrictlyFifoRetriesMode(true),
    lastPacketLifetimeQueuePurgeTime(ZERO_TIME),
    currentFrameRetryTxLimit(0),
    physicalLayer(
        theParameterDatabaseReader,
        initNodeId,
        initInterfaceId,
        simulationEngineInterfacePtr,
        propModelInterfacePtr,
        mimoChannelModelInterfacePtr,
        frequencySelectiveFadingModelInterfacePtr,
        berCurveDatabasePtr,
        shared_ptr<InterfaceForPhy>(new InterfaceForPhy(this)),
        nodeSeed),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(nodeSeed, initInterfaceIndex)),
    droppedPacketsStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_FramesDropped"))),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_BytesSent"))),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_BytesReceived"))),
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
    //default

    string nodeTypeString = "ad-hoc";

    if (theParameterDatabaseReader.ParameterExists(parameterNamePrefix + "node-type", theNodeId, theInterfaceId)) {

        nodeTypeString = theParameterDatabaseReader.ReadString(parameterNamePrefix + "node-type", theNodeId, theInterfaceId);
        ConvertStringToLowerCase(nodeTypeString);
    }//if//

    if (nodeTypeString == "access-point") {
        operationMode = ApMode;
    }
    else if (nodeTypeString == "mobile-sta") {

        operationMode = InfrastructureMode;
    }
    else if (nodeTypeString == "ad-hoc") {

        operationMode = AdhocMode;

        // No controller needed
    }
    else {
        cerr << parameterNamePrefix << "node-type \"" << nodeTypeString << "\" is not defined." << endl;
        exit(1);
    }//if//

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

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "max-num-aggregate-subframes"), theNodeId, theInterfaceId)) {

        maxNumAggregateSubframes =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "max-num-aggregate-subframes"), theNodeId, theInterfaceId);

        if ((maxNumAggregateSubframes < 1) ||
            (maxNumAggregateSubframes > BlockAckBitMapNumBits)) {
            cerr << "Error: "<< parameterNamePrefix
                 << "max-num-aggregate-subframes: " << maxNumAggregateSubframes
                 << " must be between 1 and " << BlockAckBitMapNumBits << "." << endl;
            exit(1);
        }//if//

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

    shortFrameRetryTxLimit = DefaultShortFrameRetryLimit;

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "short-frame-retry-limit"), initNodeId, initInterfaceId)) {

        shortFrameRetryTxLimit =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "short-frame-retry-limit"), initNodeId, initInterfaceId);

        if (shortFrameRetryTxLimit < 1) {
            cerr << "Error: "<< parameterNamePrefix
                 << "short-frame-retry-limit must be greater than or equal to 1: "
                 << shortFrameRetryTxLimit << endl;
            exit(1);
        }//if//
    }//if//

    requestToSendRetryTxLimit = shortFrameRetryTxLimit;
    blockAckRequestRetryTxLimit = shortFrameRetryTxLimit;

    longFrameRetryTxLimit = DefaultLongFrameRetryLimit;

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "long-frame-retry-limit"), initNodeId, initInterfaceId)) {

        longFrameRetryTxLimit =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "long-frame-retry-limit"), initNodeId, initInterfaceId);

        if (longFrameRetryTxLimit < 1) {
            cerr << "Error: "<< parameterNamePrefix
                 << "long-frame-retry-limit must be greater than or equal to 1: "
                 << longFrameRetryTxLimit << endl;
            exit(1);
        }//if//
    }//if//


    // Legacy results comparison parameter.

    disallowAddingNewPacketsToRetries = false;

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "disallow-adding-new-packets-to-retries"), initNodeId, initInterfaceId)) {

        disallowAddingNewPacketsToRetries =
            longFrameRetryTxLimit =
                theParameterDatabaseReader.ReadBool(
                    (parameterNamePrefix + "disallow-adding-new-packets-to-retries"), initNodeId, initInterfaceId);
    }//if//


    //rate controller
    theAdaptiveRateControllerPtr =
        CreateAdaptiveRateController(
            simulationEngineInterfacePtr,
            theParameterDatabaseReader,
            theNodeId,
            theInterfaceId,
            interfaceIndex,
            physicalLayer.GetBaseChannelBandwidthMhz(),
            physicalLayer.GetMaxChannelBandwidthMhz(),
            physicalLayer.GetIsAHighThroughputStation(),
            shared_ptr<Dot11MacDurationCalculationInterface>(
                new Dot11MacDurationCalculationInterface(this)),
            nodeSeed);

    //power controller
    theAdaptiveTxPowerControllerPtr =
        CreateAdaptiveTxPowerController(
            simulationEngineInterfacePtr,
            theParameterDatabaseReader,
            theNodeId,
            theInterfaceId);

    additionalDelayForExtendedInterframeSpaceMode = CalculateAdditionalDelayForExtendedInterframeSpace();


    shared_ptr<ScenSim::OutputQueuePriorityMapper> outputQueuePriorityMapperPtr;

    (*this).InitializeAccessCategories(theParameterDatabaseReader, outputQueuePriorityMapperPtr);

    wakeupTimerEventPtr = shared_ptr<WakeupTimerEvent>(new WakeupTimerEvent(this));

    networkOutputQueuePtr.reset(
        new ScenSim::OutputQueueWithPrioritySubqueues(
            theParameterDatabaseReader,
            theInterfaceId,
            simulationEngineInterfacePtr,
            ConvertToPacketPriority((numberAccessCategories - 1)),
            outputQueuePriorityMapperPtr));

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
        unsigned int initialChannel = physicalLayer.GetBaseChannelNumber();

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



    if (operationMode != ApMode) {

        // Allow STAs to use fixed TXOP durations set in the parameter file
        // Normally, STA TXOPs are controlled by the AP sent via Beacon frames.

        macSchedulerPtr.reset(
            new FifoFixedTxopScheduler(
                theParameterDatabaseReader,
                theNodeId,
                theInterfaceId,
                static_cast<unsigned int>(accessCategories.size()),
                networkOutputQueuePtr));
    }
    else {
        string schedulingAlgorithmName = "fifo";

        if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "ap-scheduling-algorithm"), theNodeId, theInterfaceId)) {

             schedulingAlgorithmName =
                theParameterDatabaseReader.ReadString(
                    (parameterNamePrefix + "ap-scheduling-algorithm"), theNodeId, theInterfaceId);
            ConvertStringToLowerCase(schedulingAlgorithmName);
        }//if//


        if (schedulingAlgorithmName == "fifo") {

            macSchedulerPtr.reset(
                new FifoFixedTxopScheduler(
                    theParameterDatabaseReader,
                    theNodeId,
                    theInterfaceId,
                    static_cast<unsigned int>(accessCategories.size()),
                    networkOutputQueuePtr));
        }
        else if (schedulingAlgorithmName == "roundrobin") {
            macSchedulerPtr.reset(
                new RoundRobinFixedTxopScheduler(
                    theParameterDatabaseReader,
                    theNodeId,
                    theInterfaceId,
                    static_cast<unsigned int>(accessCategories.size()),
                    networkOutputQueuePtr));
        }
        else {
            cerr << "Error in parameter: " << (parameterNamePrefix + "ap-scheduling-algorithm")
                 << ": Unknown scheduling algorithm." << endl;
            exit(1);

        }//if//
    }//if//

}//Dot11Mac()//


#pragma warning(default:4355)


inline
void Dot11Mac::CompleteInitialization(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const RandomNumberGeneratorSeed& nodeSeed)
{
    switch (operationMode) {
    case ApMode:
        apControllerPtr.reset(
            new Dot11ApManagementController(
                shared_from_this(),
                simEngineInterfacePtr,
                theParameterDatabaseReader,
                theNodeId,
                theInterfaceId,
                HashInputsToMakeSeed(nodeSeed, interfaceIndex)));
        break;

    case InfrastructureMode:
        staControllerPtr.reset(
            new Dot11StaManagementController(
                shared_from_this(),
                simEngineInterfacePtr,
                theParameterDatabaseReader,
                theNodeId,
                theInterfaceId,
                HashInputsToMakeSeed(nodeSeed, interfaceIndex)));
        break;

    case AdhocMode:
        // No controller needed
        break;

    default:
        assert(false); abort();
        break;
    }//switch//

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
                    aFrame.GetAndReinterpretPayloadData<QosDataFrameHeader>(headerOffset);

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

    //dcc
    //cout << "currentTime: " << simEngineInterfacePtr->CurrentTime() << endl;

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

    // Higher Access Category Indices have higher priority

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

    // Higher Access Category Indices have higher priority

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

//dcc
/*inline
SimTime Dot11Mac::get_duration(){
    SimTime calcDuration = 0, Toff = 0;
    double delta;
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
    const long long int dataRate = 6000000;
    const long long int calcInterval = 100000000;
    long long int total_packet_size = 0;
    //reactive
    //Toff = calcReactiveSendRate();
    //reactive

    //adaptive
    delta = calcAdaptiveSendRate();
    //cout << "delta: " << delta << endl;
    //adaptive

    auto begin = packetSizeforDCC.begin(), end = packetSizeforDCC.end();
    for (auto iter = begin; iter != end; iter++){
        if(iter->first + calcInterval < currentTime){
            //cout << "erase: " << iter->first << endl;
            packetSizeforDCC.erase(iter->first);
        }else{
            total_packet_size += iter->second;
            cout << "packetsize_sum: " << total_packet_size << endl;
        }
    }*/

    //reactive
    //calcDuration = Toff + total_packet_size * 8/*byte to bit*/ * (1000000000 / 1000000) / (dataRate / 1000000) - calcInterval;//to prevent overflow
    //cout << "else: " << total_packet_size * 8/*byte to bit*/ * 1000000000 / dataRate << endl;
    //cout << "calcDuration: " << calcDuration << ", Toff: " << Toff << ", total_packet_size: " << total_packet_size << ", dataRate: " << dataRate << ", calcInterval: " << calcInterval << endl;
    //reactive

    //adaptive
    //calcDuration = total_packet_size * 8/*byte to bit*/ * (1000000000 / 1000000) / (dataRate / 1000000) / delta - calcInterval;
    //adaptive

    /*return calcDuration;
}
//dcc

//dcc reactive
inline
SimTime Dot11Mac::calcReactiveSendRate(){
    double channelBusyRatio;
    SimTime calcToff;
    channelBusyRatio = physicalLayer.channelBusyRatio_phy;
    //cout << "CBR: " << channelBusyRatio << endl;
    if(status == 0){
        if(channelBusyRatio >= 0.3){
            status = 1;
        }
    }else if(status == 1){
        if(channelBusyRatio >= 0.4){
            status = 2;
        }else if(channelBusyRatio < 0.3){
            status = 0;
        }
    }else if(status == 2){
        if(channelBusyRatio >= 0.5){
            status = 3;
        }else if(channelBusyRatio < 0.4){
            status = 1;
        }
    }else if(status == 3){
        if(channelBusyRatio > 0.6){
            status = 4;
        }else if(channelBusyRatio < 0.5){
            status = 2;
        }
    }else if(status == 4){
        if(channelBusyRatio <= 0.6){
            status = 3;
        }
    }
    //Ton_max = 1ms
    if(status == 0){
        calcToff = 100000000;
    }else if(status == 1){
        calcToff = 200000000;
    }else if(status == 2){
        calcToff = 400000000;
    }else if(status == 3){
        calcToff = 500000000;
    }else if(status == 4){
        calcToff = 1000000000;
    }
    //cout << "calcToff: " << calcToff << endl;
    return calcToff;
}

//dcc adaptive
inline
double Dot11Mac::calcAdaptiveSendRate(){
    //dcc
    double channelBusyRatio, offset;
    double calcDelta = 0;

    const double A = 0.016;//LIMERIC
    const double B = 0.0012;//LIMERIC
    const double target = 0.68;//LIMERIC
    const double delta_max = 0.03;
    const double delta_min = 0.0006;
    const double gplus = 0.0005;
    const double gminus = -0.00025;
    const int nodeNum = 9; 
    channelBusyRatio = physicalLayer.channelBusyRatio_phy;

    //LIMERIC
    if(adaptiveCount == 0){
        averageChannelBusyRatio = channelBusyRatio;
        CBR_L_0_Hop_Previous = channelBusyRatio;
    }else if(adaptiveCount >= 1){
        averageChannelBusyRatio = 0.5 * averageChannelBusyRatio + 0.5 * (channelBusyRatio + CBR_L_0_Hop_Previous) / 2;
        CBR_L_0_Hop_Previous = channelBusyRatio;
    }else{
        //cout << "error" << endl;
    }
    if(target - averageChannelBusyRatio > 0){
        offset = std::min(B * (target - averageChannelBusyRatio), gplus);
    }else{
        offset = std::max(B * (target - averageChannelBusyRatio), gminus);
    }
    delta = (1 - A) * delta + offset;
    if(delta > delta_max){
        delta = delta_max;
    }
    if(delta < delta_min){
        delta = delta_min;
    }
    calcDelta = delta;
    adaptiveCount++;*/
    //busyTime = dot11MacPtr->getBusyTime(theNodeId);
    //totalBusyTime = dot11MacPtr->getTotalBusyTime(nodeNum);
    //packetInterval = (1 - A) * busyTime_before + B * (r_g - totalBusyTime_before) 

    //int gateopen_basetime, gateopen_basecount, nextpackettime;
    //double calcDelta = 0;
    //double averageChannelBusyRatio = 0;
    //int i, status, openflag /*CBR_L_0_Hop,*//* CBR_L_0_Hop_Previous*/;
    //static map<int, long long int> currentTime_before, gateopen_basetime, gateopen_basecount, nextpackettime;
    //static int totalBusyTime, totalBusyChannelTime_before;
    //shared_ptr<NetworkLayer> networkLayerPtr = transportLayerPtr->GetNetworkLayerPtr();
    //shared_ptr<MacLayer> macLayerPtr = networkLayerPtr->GetMacLayerPtr(0);
    //shared_ptr<Dot11::Dot11Mac> dot11MacPtr = std::dynamic_pointer_cast<Dot11::Dot11Mac>(macLayerPtr); 
    //0:Relaxed, 1:Active1, 2:Active2, 3:Active3, 4:Restrictive
    //const double A = 0.016;//LIMERIC
    //const double B = 0.0012;//LIMERIC
    //const double target = 0.68;//LIMERIC
    //const double delta_max = 0.03;
    //const double delta_min = 0.0006;
    //const double gplus = 0.0005;
    //const double gminus = -0.00025;
    //const int nodeNum = 9; 
    //channelBusyRatio = getCBR();
    //20210526
    //cout << "time: " << currentTime << endl;
    //cout << "interval: " << packetInterval << endl;
    
    //reactive
    /*if(i[theNodeId] == 0){
        status = 0;
        gateopen_basecount[theNodeId] = i[theNodeId];
        gateopen_basetime[theNodeId] = currentTime;
        //cout << "first, nodeid: " << theNodeId << ", currenttime: " << currentTime << endl;
        //cout << "basetime: " << gateopen_basetime[theNodeId] << endl;//wrong
        openflag[theNodeId] = 1;
    }*/
    //cout << "status_before: " << status << endl;
    
    //cout << "nextpackettime: " << nextpackettime[theNodeId] << endl;
    /*if(theNodeId == 65){
        cout << "openflag: " << openflag[theNodeId] << endl;
        cout << "i: " << i[theNodeId] << endl;
        cout << "basecount: " << gateopen_basecount[theNodeId] << endl;
        cout << "basetime: " << gateopen_basetime[theNodeId] << endl;
        cout << "nextpackettime: " << nextpackettime[theNodeId] << endl;
        cout << "currenttime: " << currentTime << endl;
    }*/
    /*if(openflag[theNodeId] == 1 && currentTime - gateopen_basetime[theNodeId] >= 1000000){
        openflag[theNodeId] = 0;
        cout << "gate close at node: " << theNodeId << endl;
    }*///gate close
    /*if(nextpackettime[theNodeId] <= currentTime - gateopen_basetime[theNodeId]){
        gateopen_basecount[theNodeId] = i[theNodeId];
        gateopen_basetime[theNodeId] = currentTime;
        openflag[theNodeId] = 1;
        cout << "gate open at node: " << theNodeId << endl;
    }*///gate open
    /*if(openflag[theNodeId] == 1){
        networkOutputQueuePtr->DequeuePacketWithPriority(
            priority,
            (*this).currentPacketPtr,
            (*this).currentPacketsNextHopNetworkAddress,
            (*this).currentPacketsEtherType,
            (*this).currentPacketsQueueInsertionTime,
            isANewPacket,
            (*this).currentPacketsRetryTxCount,
            (*this).currentPacketsSequenceNumber);
    }*/
    /*if(currentTime >= gateopen_basetime[theNodeId] + nextpackettime[theNodeId]){
            networkOutputQueuePtr->DequeuePacketWithPriority(
                priority,
                (*this).currentPacketPtr,
                (*this).currentPacketsNextHopNetworkAddress,
                (*this).currentPacketsEtherType,
                (*this).currentPacketsQueueInsertionTime,
                isANewPacket,
                (*this).currentPacketsRetryTxCount,
                (*this).currentPacketsSequenceNumber);
        }*///gate open
    /*if(openflag[theNodeId] == 1 && i[theNodeId] == gateopen_basecount[theNodeId] + 1){
        openflag[theNodeId] = 0;
        cout << "gate close at node: " << theNodeId << endl; 
    }//gate close
    if(currentTime >= gateopen_basetime[theNodeId] + nextpackettime[theNodeId]){
        gateopen_basecount[theNodeId] = i[theNodeId];
        gateopen_basetime[theNodeId] = currentTime;
        openflag[theNodeId] = 1;
        cout << "gate open at node: " << theNodeId << endl;
    }*///gate open

    /*cout << "id: " << theNodeId << ", CBR: " << channelBusyRatio << ", interval: " << packetInterval << endl;
    cout << "status: " << status << endl;
    cout << "interval: " << packetInterval << endl;*/
    //cout << "time: " << (double)currentTime / (double)1000000000 << endl;
    //shared_ptr<NetworkLayer> networkLayerPtr = transportLayerPtr->GetNetworkLayerPtr();
    /*shared_ptr<NetworkLayer> networkLayerPtr = transportLayerPtr->GetNetworkLayerPtr();
    cout << "A" << endl;
    const unsigned int interfaceIndex = networkLayerPtr->LookupInterfaceIndex(0);
    cout << "B" << endl;
    shared_ptr<MacLayer> macLayerPtr = networkLayerPtr->GetMacLayerPtr(interfaceIndex);
    cout << "C" << endl;
    //cout << "model: " << macLayerPtr->GetCBR() << endl;
    shared_ptr<MacAndPhyInfoInterface> macAndPhyInfoInterfacePtr =
        macLayerPtr->GetMacAndPhyInfoInterface();
    cout << "D" << endl;
    int busyTime;
    busyTime = macAndPhyInfoInterfacePtr->GetTotalBusyChannelTime();
    cout << "busytime: " << busyTime << endl;*/
    //macLayerPtr->ModelName;
    //nextpackettimeをいじれば間隔を調整できる？
    //Dot11Phy dp;
    //dp.DccReactive();
    //Dot11::Dot11Phy::DccReactive();
    //Dot11::Dot11Phy* dp;
    /*SimTime busyTime;
    SimTime idleTime;
    busyTime = dp->GetTotalBusyChannelTime();
    idleTime = dp->GetTotalIdleChannelTime();*/
    //cout << "busytime2: " << dp->GetTotalBusyChannelTime() << endl;
    //cout << "idletime2: " << dp->GetTotalIdleChannelTime() << endl;
    //cout << "CBR: " << /*dp->*/channelBusyRatio << endl;
    //cout << "CBR: " << getCBR() << endl;
    //i[theNodeId]++;
    //20210520
    //reactive

    /*return calcDelta;
}*/
//dcc

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
void Dot11Mac::StartBackoffIfNecessary()
{
    assert(macState != WaitingForIfsAndBackoffState);

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    SimTime DCCduration;
    
    SimTime backoffDuration = CurrentBackoffDuration();

    if (backoffDuration == INFINITE_TIME) {
        macState = IdleState;
        return;
    }//if//

    //dcc
    //cout << "duration_before: " << backoffDuration << endl;
    //cout << "currentTime: " << currentTime << ", timebefore: " << currentTime_before << endl;
    //if(currentTime >= currentTime_before + 100000000){
        //cout << "duration: " << get_duration() << endl;
        /*if(get_duration() > backoffDuration){
            backoffDuration = get_duration();
        }*/
        //backoffDuration = get_duration(); //dcc
    //}
    //currentTime_before = currentTime;
    
    /*if(currentTime >= currentTime_before + 100000000){
        DCCduration = get_duration();
    }*/
    //DCCduration = get_duration();
    macState = WaitingForIfsAndBackoffState;
    //cout << "macState: " << macState << endl;

    for(unsigned int i = 0; (i < accessCategories.size()); i++) {
        EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[i];
        if (accessCategoryInfo.currentNonExtendedBackoffDuration != INFINITE_TIME) {
            accessCategoryInfo.ifsAndBackoffStartTime = currentTime;
        }
        else {
            accessCategoryInfo.ifsAndBackoffStartTime = INFINITE_TIME;
        }//if//
    }//for//

    //cout << "currentTime: " << currentTime << ", DCCduration: " << DCCduration << ", backoffduration: " << backoffDuration << endl;
    
    /*if(DCCduration > 0){
        DCCduration_before = DCCduration;
        (*this).ScheduleWakeupTimer(currentTime + DCCduration);
        OutputTraceForIfsAndBackoffStart(DCCduration);
    }else{
         (*this).ScheduleWakeupTimer(currentTime + DCCduration_before);
        OutputTraceForIfsAndBackoffStart(DCCduration_before);
    }*/
    /*else{
        (*this).ScheduleWakeupTimer(currentTime + backoffDuration);

        OutputTraceForIfsAndBackoffStart(backoffDuration);
    }*/
    //currentTime_before = currentTime;

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

    accessCategoryInfo.hasPacketToSend =
        ThereAreQueuedPacketsForAccessCategory(accessCategoryIndex);

    if ((!forceBackoff) && (!accessCategoryInfo.hasPacketToSend)) {
        accessCategoryInfo.currentNonExtendedBackoffDuration = INFINITE_TIME;

        return;

    }//if//

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
        }

        (*this).StartBackoffForCategory(accessCategoryIndex);

        break;
    }
    default:
        assert(false); abort(); break;
    }//switch//

}//StartPacketSendProcessForAnAccessCategory//


inline
void Dot11Mac::NetworkLayerQueueChangeNotification()
{
    // High Priority to Low priority category order:

    //cout << "NetworkLayerQueueChangeNotification" << endl;
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
            (networkOutputQueuePtr->HasPacketWithPriority(ConvertToPacketPriority(accessCategoryIndex))));
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


            const unsigned int numSlotsUsed =
                NumberBackoffSlotsUsedDuringElapsedTime(accessCategoryInfo, elapsedTime);

            assert(numSlotsUsed <= accessCategoryInfo.currentNumOfBackoffSlots);

            accessCategoryInfo.currentNumOfBackoffSlots -= numSlotsUsed;

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

    assert((macState == BusyMediumState) || (macState == TransientState));

    if (simEngineInterfacePtr->CurrentTime() < mediumReservedUntilTimeAkaNAV) {
        (*this).GoIntoWaitForExpirationOfVirtualCarrierSenseAkaNavState();
    }
    else {
        mediumBecameIdleTime = simEngineInterfacePtr->CurrentTime();
        (*this).StartBackoffIfNecessary();

    }//if//

    assert((macState != BusyMediumState) && (macState != TransientState));

}//ClearChannelAtPhysicalLayerNotification//





inline
void Dot11Mac::DoSuccessfulTransmissionPostProcessing(const bool wasJustTransmitting)
{
    if (currentTransmitOpportunityAkaTxopEndTime == ZERO_TIME) {

        // TXOP is not being used (1 frame per medium access).
        // or number packet limit is exhausted.

        (*this).StartPacketSendProcessForAnAccessCategory(currentPacketsAccessCategoryIndex, true);
        return;
    }//if//

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    (*this).RemoveLifetimeExpiredPacketsFromQueue(currentPacketsAccessCategoryIndex);

    const SimTime nextFrameSequenceDuration =
        CalculateNextFrameSequenceDuration(currentPacketsAccessCategoryIndex);

    if (nextFrameSequenceDuration == ZERO_TIME) {

        // Nothing To Send

        currentTransmitOpportunityAkaTxopEndTime = ZERO_TIME;
        (*this).StartPacketSendProcessForAnAccessCategory(currentPacketsAccessCategoryIndex, true);

        return;
    }//if//

    if ((currentTime + nextFrameSequenceDuration) > currentTransmitOpportunityAkaTxopEndTime) {

        // Not enough time left in the TXOP for next packet.

        currentTransmitOpportunityAkaTxopEndTime = ZERO_TIME;
        (*this).StartPacketSendProcessForAnAccessCategory(currentPacketsAccessCategoryIndex, true);
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
            currentPacketsAccessCategoryIndex,
            true,
            transmitDelay,
            packetHasBeenSentToPhy);
        //cout << "DoSuccessfulTransmissionPostProcessing" << endl;//no


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
        // Clear old backoff value (restarts a backoff).

        EdcaAccessCategoryInfo& accessCategoryInfo =
            accessCategories.at(currentPacketsAccessCategoryIndex);

        accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff = false;
        accessCategoryInfo.currentNonExtendedBackoffDuration = INFINITE_TIME;

        (*this).StartPacketSendProcessForAnAccessCategory(currentPacketsAccessCategoryIndex);
        (*this).SwitchToChannels(switchingToThisChannelList);
    }
    else {
        assert(macState == BusyMediumState);

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

    if ((destinationAddress.IsAMulticastAddress()) && (MpduFrameAggregationIsEnabled())) {

        return (allowMpduAggregationForMulticastAddresses);

    }//if//

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

    if (destinationAddress.IsAMulticastAddress()) {

        return (allowMpduAggregationForMulticastAddresses);

    }//if//

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
void Dot11Mac::BuildAggregateFrameFromCurrentFrame()
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    const unsigned int accessCategoryIndex = currentPacketsAccessCategoryIndex;

    assert((allowFrameAggregationWithTxopZero) ||
           (currentTransmitOpportunityAkaTxopEndTime != ZERO_TIME));

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

    const bool useMpduAggregation =
        MpduFrameAggregationIsEnabledFor(currentPacketsDestinationMacAddress);

    const bool firstFrameIsARetry = (currentPacketsRetryTxCount > 0);

    assert(currentAggregateFramePtr == nullptr);
    currentAggregateFramePtr.reset(new vector<unique_ptr<Packet> >());
    (*this).currentAggregateFrameRetryInfo.clear();

    currentAggregateFramePtr->push_back(move(currentPacketPtr));

    (*this).currentAggregateFrameRetryInfo.push_back(
        AggregateSubframeRetryInfo(
            currentPacketsQueueInsertionTime,
            currentPacketsRetryTxCount));

    vector<unique_ptr<Packet> >& frameList = *currentAggregateFramePtr;

    const QosDataFrameHeader copyOfOriginalFrameHeader =
        frameList[0]->GetAndReinterpretPayloadData<QosDataFrameHeader>();

    AddMpduDelimiterAndPaddingToFrame(*frameList[0]);

    // Assuming No RTS/CTS for aggregated frames ("Greenfield" not supported).

    TransmissionParameters txParameters;
    bool notUsedUseRtsEvenIfDisabled;
    bool notUsedOverridesStandardRetryLimits;
    unsigned int notUsedOverridenRetryTxLimit;

    // Minstrel forcing RTS CTS is ignored.

    theAdaptiveRateControllerPtr->GetDataRateInfoForDataFrameToStation(
        currentPacketsDestinationMacAddress,
        1, // Assume Not a retry (Minstrel)
        accessCategoryIndex,
        txParameters,
        notUsedUseRtsEvenIfDisabled,
        notUsedOverridesStandardRetryLimits,
        notUsedOverridenRetryTxLimit);

    unsigned int totalBytes = frameList[0]->LengthBytes();

    SimTime endTime =
        currentTime +
        CalculateFrameDuration(frameList[0]->LengthBytes(), txParameters);

    endTime += aShortInterframeSpaceTime;
    endTime += CalculateFrameDuration(sizeof(BlockAcknowledgementFrame), txParameters);

    const unsigned int maxNumNonRetrySubframes =
        std::min(
            CalcNumberFramesLeftInSequenceNumWindow(
                currentPacketsDestinationMacAddress,
                currentPacketsAccessCategoryIndex),
            maxNumAggregateSubframes - 1/*first frame*/);

    unsigned int numNonRetrySubframes = 0;

    const NetworkAddress& nextHopAddress = currentPacketsNextHopNetworkAddress;

    // Aggregation loop. Stop aggregating when (any of these):
    // * Reached max # of subframes or would violate TXOP duration (unless special
    //   TXOP = 0 mode).
    // * Ran out frames for the desired priority.
    // * Next frame (FIFO) is not for the destination and the scheduler disallows the non-FIFO
    //   frame.

    while ((((allowFrameAggregationWithTxopZero) &&
             (currentTransmitOpportunityAkaTxopEndTime == ZERO_TIME)) ||
            (endTime < currentTransmitOpportunityAkaTxopEndTime)) &&
           (networkOutputQueuePtr->HasPacketWithPriority(currentPacketsAccessCategoryIndex)) &&
           ((networkOutputQueuePtr->NextHopAddressForTopPacket(currentPacketsAccessCategoryIndex) ==
             nextHopAddress) ||
            ((macSchedulerPtr != nullptr) &&
             (macSchedulerPtr->AllowNextNonFifoFrameToBeAggregated(
                 currentPacketsAccessCategoryIndex, nextHopAddress, totalBytes))))) {


        if (((numNonRetrySubframes == maxNumNonRetrySubframes) ||
            ((disallowAddingNewPacketsToRetries) && (firstFrameIsARetry))) &&
            (!networkOutputQueuePtr->NextPacketIsARetry(currentPacketsAccessCategoryIndex, nextHopAddress))) {
            // No more non-retries can be sent or legacy behavior condition.
            break;
        }//if//

        const Packet& nextFrame =
            networkOutputQueuePtr->GetNextPacketWithPriorityAndNextHop(
                currentPacketsAccessCategoryIndex, nextHopAddress);

        const unsigned int packetLengthBytes =
            (nextFrame.LengthBytes() +
             sizeof(QosDataFrameHeader) + sizeof(MpduDelimiterFrame));

        if ((totalBytes + packetLengthBytes) > maxAggregateMpduSizeBytes) {
            break;
        }//if//

        // Only first frame of aggregate has Phy header.

        endTime += CalculateFrameDurationWithoutPhyHeader(packetLengthBytes, txParameters);

        if ((!allowFrameAggregationWithTxopZero) &&
            (endTime >= currentTransmitOpportunityAkaTxopEndTime)) {

            break;
        }//if//

        EtherTypeField etherType;
        SimTime queueInsertionTime;
        bool isANewPacket;
        unsigned int retryTxCount;
        unsigned short int sequenceNumber;
        unique_ptr<Packet> framePtr;

        networkOutputQueuePtr->DequeuePacketWithPriorityAndNextHop(
            currentPacketsAccessCategoryIndex,
            nextHopAddress,
            framePtr,
            etherType,
            queueInsertionTime,
            isANewPacket,
            retryTxCount,
            sequenceNumber);

        assert(etherType == currentPacketsEtherType);

        if (isANewPacket) {
            assert(retryTxCount == 0);
            (*this).GetNewSequenceNumber(
                currentPacketsDestinationMacAddress,
                currentPacketsAccessCategoryIndex,
                false,
                sequenceNumber);
        }//if//

        QosDataFrameHeader dataFrameHeader = copyOfOriginalFrameHeader;
        dataFrameHeader.theSequenceControlField.sequenceNumber = sequenceNumber;

        framePtr->AddPlainStructHeader(dataFrameHeader);
        AddMpduDelimiterAndPaddingToFrame(*framePtr);

        totalBytes += framePtr->LengthBytes();

        currentAggregateFramePtr->push_back(move(framePtr));

        (*this).currentAggregateFrameRetryInfo.push_back(
            AggregateSubframeRetryInfo(queueInsertionTime, retryTxCount));

        if (retryTxCount == 0) {
            numNonRetrySubframes++;
        }//if//
    }//while//

    assert(currentAggregateFramePtr->size() <= BlockAckBitMapNumBits);

    if (currentAggregateFramePtr->size() > 1) {
        // No padding needed on last frame.
        RemoveMpduAggregationPaddingFromFrame(*currentAggregateFramePtr->back());
    }
    else {
        // Aggregation Failure (Back to single frame).

        currentPacketPtr = move(currentAggregateFramePtr->front());

        currentAggregateFramePtr.reset();

        RemoveMpduDelimiterAndPaddingFromFrame(*currentPacketPtr);
    }//if//

}//BuildAggregateFrameFromCurrentFrame//



inline
void Dot11Mac::RetrievePacketFromNetworkLayerForAccessCategory(
    const unsigned int accessCategoryIndex,
    bool& wasRetrieved)
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    const PacketPriority priority = ConvertToPacketPriority(accessCategoryIndex);

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    assert(currentPacketPtr == nullptr);

    wasRetrieved = false;

    while ((!wasRetrieved) &&
           (networkOutputQueuePtr->HasPacketWithPriority(priority))) {

        bool isANewPacket;

        if (!macSchedulerPtr->NonFifoFlowSchedulingModeIsOn()) {
            
            //if(openflag[theNodeId] == 1){
            
            networkOutputQueuePtr->DequeuePacketWithPriority(
                priority,
                (*this).currentPacketPtr,
                (*this).currentPacketsNextHopNetworkAddress,
                (*this).currentPacketsEtherType,
                (*this).currentPacketsQueueInsertionTime,
                isANewPacket,
                (*this).currentPacketsRetryTxCount,
                (*this).currentPacketsSequenceNumber);
            //}
        }
        else {
            if ((currentPacketIsForTxopCollisionDetect) &&
                networkOutputQueuePtr->HasPacketWithPriorityAndNextHop(
                    priority, currentPacketsNextHopNetworkAddress)) {

                // Keep same next hop because first packet was for TXOP collision detect.

                currentPacketIsForTxopCollisionDetect = false;
            }
            else {
                macSchedulerPtr->CalcNextDestinationToSchedule(
                    accessCategoryIndex,
                    (*this).currentPacketsNextHopNetworkAddress);

                currentPacketIsForTxopCollisionDetect =
                    (currentTransmitOpportunityAckedFrameCount == 0);

            }//if//

            networkOutputQueuePtr->DequeuePacketWithPriorityAndNextHop(
                priority,
                currentPacketsNextHopNetworkAddress,
                (*this).currentPacketPtr,
                (*this).currentPacketsEtherType,
                (*this).currentPacketsQueueInsertionTime,
                isANewPacket,
                (*this).currentPacketsRetryTxCount,
                (*this).currentPacketsSequenceNumber);
        }//if//

        
        (*this).currentPacketsAccessCategoryIndex = priority;

        assert((currentPacketsNextHopNetworkAddress.IsTheBroadcastAddress()) ||
               !(currentPacketsNextHopNetworkAddress.IsABroadcastAddress(
                   networkLayerPtr->GetSubnetMask(interfaceIndex))) &&
               "Make sure no mask dependent broadcast address from network layer");

        /*if((currentTime - currentPacketsQueueInsertionTime) >= accessCategoryInfo.frameLifetime){
            cout << "currenttime: " << currentTime << ", insertiontime: " << currentPacketsQueueInsertionTime << ", lifetime: " << accessCategoryInfo.frameLifetime << endl;
        }*/
        assert((currentTime - currentPacketsQueueInsertionTime) <= accessCategoryInfo.frameLifetime);

        if (redundantTraceInformationModeIsOn) {
            OutputTraceForPacketDequeue(accessCategoryIndex, aRxTxTurnaroundTime);
        } else {
            OutputTraceForPacketDequeue(accessCategoryIndex);
        }//if//

        if ((refreshNextHopOnDequeueModeIsOn) &&
            (!currentPacketsNextHopNetworkAddress.IsTheBroadcastAddress())) {
            // Update the next hop to latest.

            bool nextHopWasFound;
            unsigned int nextHopInterfaceIndex;

            networkLayerPtr->GetNextHopAddressAndInterfaceIndexForNetworkPacket(
                *currentPacketPtr,
                nextHopWasFound,
                (*this).currentPacketsNextHopNetworkAddress,
                nextHopInterfaceIndex);

            if ((!nextHopWasFound) || (nextHopInterfaceIndex != interfaceIndex)) {
                // Next hop is no longer valid (dynamic routing).

                networkLayerPtr->ReceiveUndeliveredPacketFromMac(
                    interfaceIndex, currentPacketPtr, NetworkAddress());

                // Loop to get another packet.
                continue;

            }//if//
        }//if//

        wasRetrieved = false;

        theMacAddressResolverPtr->GetMacAddress(
            currentPacketsNextHopNetworkAddress,
            networkLayerPtr->GetSubnetMask(interfaceIndex),
            wasRetrieved,
            (*this).currentPacketsDestinationMacAddress);

        if (!wasRetrieved) {
            // There is not a mac address entry.

            networkLayerPtr->ReceiveUndeliveredPacketFromMac(
                interfaceIndex, currentPacketPtr, NetworkAddress());

            // Loop to get another packet.
            continue;

        }//if//

        if (isANewPacket) {
            assert(currentPacketsRetryTxCount == 0);
            (*this).currentPacketsRetryTxCount = 0;

            (*this).GetNewSequenceNumber(
                currentPacketsDestinationMacAddress,
                currentPacketsAccessCategoryIndex,
                true,
                (*this).currentPacketsSequenceNumber);
        }//if//

        QosDataFrameHeader dataFrameHeader;

        dataFrameHeader.header.theFrameControlField.frameTypeAndSubtype = QOS_DATA_FRAME_TYPE_CODE;
        dataFrameHeader.header.theFrameControlField.isRetry = 0;
        dataFrameHeader.header.duration = 0;
        dataFrameHeader.header.receiverAddress = currentPacketsDestinationMacAddress;
        dataFrameHeader.theSequenceControlField.sequenceNumber = currentPacketsSequenceNumber;
        dataFrameHeader.transmitterAddress = myMacAddress;
        dataFrameHeader.qosControlField.trafficId = currentPacketsAccessCategoryIndex;
        dataFrameHeader.linkLayerHeader.etherType = HostToNet16(currentPacketsEtherType);

        currentPacketPtr->AddPlainStructHeader(dataFrameHeader);

        // Check for Power Saving destination.

        if ((GetOperationMode() == ApMode) &&
            (apControllerPtr->StationIsAsleep(currentPacketsDestinationMacAddress))) {

            apControllerPtr->BufferPacketForSleepingStation(
                currentPacketsDestinationMacAddress,
                currentPacketPtr,
                currentPacketsNextHopNetworkAddress,
                currentPacketsAccessCategoryIndex,
                currentPacketsEtherType,
                currentPacketsQueueInsertionTime,
                currentPacketsRetryTxCount);

            wasRetrieved = false;
            // Loop to get another packet.
            continue;
        }//if//

        assert(wasRetrieved);

    }//while//

}//RetrievePacketFromNetworkLayerForAccessCategory//



inline
void Dot11Mac::RequeueBufferedPacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority priority,
    const EtherTypeField etherType,
    const SimTime& queueInsertionTime,
    const unsigned int txRetryCount)
{
    const QosDataFrameHeader& header =
        packetPtr->GetAndReinterpretPayloadData<QosDataFrameHeader>();

    const unsigned short int sequenceNumber = header.theSequenceControlField.sequenceNumber;

    packetPtr->DeleteHeader(sizeof(QosDataFrameHeader));
    networkOutputQueuePtr->RequeueAtFront(
        packetPtr, nextHopAddress, priority, etherType,
        queueInsertionTime, txRetryCount, sequenceNumber);

}//RequeueBufferedPacket//



inline
void Dot11Mac::RequeueManagementFrame(
    unique_ptr<Packet>& framePtr,
    const unsigned int retryTxCount)
{
    (*this).managementFrameQueue.push_front(
        ManagementFrameQueueElem(move(framePtr), retryTxCount));

}//RequeueManagementFrame//

inline
void Dot11Mac::RequeuePacketForRetry(
    const unsigned int accessCategoryIndex,
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority priority,
    const EtherTypeField etherType,
    const SimTime& queueInsertionTime,
    const unsigned int txRetryCount)
{
    const QosDataFrameHeader& header =
        packetPtr->GetAndReinterpretPayloadData<QosDataFrameHeader>();

    const unsigned short int sequenceNumber = header.theSequenceControlField.sequenceNumber;

    packetPtr->DeleteHeader(sizeof(QosDataFrameHeader));
    networkOutputQueuePtr->RequeueAtFront(
        packetPtr, nextHopAddress, priority, etherType,
        queueInsertionTime, txRetryCount, sequenceNumber);

    if (isInStrictlyFifoRetriesMode) {
        accessCategories[accessCategoryIndex].shouldRetransmitLastDataPacket = true;
    }//if//

}//RequeueBufferedPacketForRetry//


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
}



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

    theAdaptiveRateControllerPtr->GetDataRateInfoForAckFrameToStation(
        destinationMacAddress,
        dataFrameTxParameters,
        ackTxParameters);

    const SimTime ackFrameDuration =
        CalculateFrameDuration(sizeof(AcknowledgementAkaAckFrame), ackTxParameters);

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
    const MacAddress& destinationMacAddress,
    const TransmissionParameters& txParameters) const
{
    const unsigned int ackFrameSizeBytes = sizeof(AcknowledgementAkaAckFrame);

    TransmissionParameters ackTxParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForAckFrameFromStation(
        destinationMacAddress, txParameters, ackTxParameters);

    const SimTime ackFrameDuration = CalculateFrameDuration(ackFrameSizeBytes, ackTxParameters);

    // NAV Duration is the ACK (and a SIFs)

    return (aShortInterframeSpaceTime + ackFrameDuration);

}//CalculateNavDurationForAckedDataFrame//


inline
SimTime Dot11Mac::CalculateNavDurationForAckedAggregateDataFrame(
    const MacAddress& destinationMacAddress,
    const TransmissionParameters& txParameters) const
{
    const unsigned int blockAckFrameSizeBytes = sizeof(BlockAcknowledgementFrame);

    TransmissionParameters ackTxParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForAckFrameFromStation(
        destinationMacAddress, txParameters, ackTxParameters);

    const SimTime blockAckFrameDuration = CalculateFrameDuration(blockAckFrameSizeBytes, ackTxParameters);

    // NAV Duration is the ACK (and a SIFs)

    return (aShortInterframeSpaceTime + blockAckFrameDuration);

}//CalculateNavDurationForAckedAggregateDataFrame//



inline
void Dot11Mac::RemoveLifetimeExpiredPacketsFromQueue(const unsigned int accessCategoryIndex)
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    if (lastPacketLifetimeQueuePurgeTime == currentTime) {
        return;
    }//if//

    (*this).lastPacketLifetimeQueuePurgeTime = currentTime;

    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

    if (currentTime < accessCategoryInfo.frameLifetime) {
        return;
    }//if//

    vector<AbstractOutputQueueWithPrioritySubqueues::ExpiredPacketInfoType> packetList;

    networkOutputQueuePtr->DequeueLifetimeExpiredPackets(
        ConvertToPacketPriority(accessCategoryIndex),
        (currentTime - accessCategoryInfo.frameLifetime),
        packetList);

    for(unsigned int i = 0; (i < packetList.size()); i++) {
        networkLayerPtr->ReceiveUndeliveredPacketFromMac(
            interfaceIndex,
            packetList[i].packetPtr,
            packetList[i].nextHopAddress);
    }//for//

}//RemoveLifetimeExpiredPacketsFromQueue//



// This function is called to determine whether a frame can be sent in the current TXOP
// after the first packet transmission.  May also be called when calculating NAV
// For a RTS (include current frame and next frame).

inline
SimTime Dot11Mac::CalculateNextFrameSequenceDuration(const unsigned int accessCategoryIndex) const
{
    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);
    const PacketPriority priority = ConvertToPacketPriority(accessCategoryIndex);

    unsigned int frameSizeBytes = 0;
    MacAddress destinationMacAddress;
    TransmissionParameters txParameters;

    if ((accessCategoryIndex == accessCategoryIndexForManagementFrames) &&
        (!managementFrameQueue.empty())) {

        const Packet& aPacket = *managementFrameQueue.front().framePtr;
        frameSizeBytes = aPacket.LengthBytes();

        const CommonFrameHeader& frameHeader =
            aPacket.GetAndReinterpretPayloadData<CommonFrameHeader>();

        destinationMacAddress = frameHeader.receiverAddress;

        theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
            destinationMacAddress, txParameters);
    }
    else if (networkOutputQueuePtr->HasPacketWithPriority(priority)) {

        NetworkAddress destinationNetworkAddress;

        if (!macSchedulerPtr->NonFifoFlowSchedulingModeIsOn()) {

            frameSizeBytes = networkOutputQueuePtr->TopPacket(priority).LengthBytes();
            destinationNetworkAddress = networkOutputQueuePtr->NextHopAddressForTopPacket(priority);
        }
        else {
            destinationNetworkAddress =
                macSchedulerPtr->CalcNextDestinationToPossiblySchedule(accessCategoryIndex);

            frameSizeBytes =
                networkOutputQueuePtr->GetNextPacketWithPriorityAndNextHop(
                    priority, destinationNetworkAddress).LengthBytes();
        }//if//

        frameSizeBytes += sizeof(QosDataFrameHeader);

        bool macAddressWasResolved;

        theMacAddressResolverPtr->GetMacAddress(
            destinationNetworkAddress,
            networkLayerPtr->GetSubnetMask(interfaceIndex),
            macAddressWasResolved,
            destinationMacAddress);

        if (!macAddressWasResolved) {
            // Next packet is not deliverable so give up.  This
            // forces the TXOP to end.

            return (ZERO_TIME);
        }//if//

        theAdaptiveRateControllerPtr->GetDataRateInfoForDataFrameToStation(
            destinationMacAddress,
            1, // Assume Tx count will be 1 (first transmiission) after successful transmission.
            accessCategoryIndex,
            txParameters);
    }
    else {
        return (ZERO_TIME);
    }//if//

    SimTime extraTimeBeforeFrame = ZERO_TIME;
    if (!currentPacketsDestinationMacAddress.IsABroadcastOrAMulticastAddress()) {
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
             CalculateNavDurationForAckedDataFrame(destinationMacAddress, txParameters));
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
void Dot11Mac::CalculateAndSetTransmitOpportunityDurationAkaTxop(const unsigned int accessCategoryIndex)
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
    assert(currentAggregateFramePtr != nullptr);

    const DurationField navDurationField =
        ConvertTimeToDurationFieldValue(CalculateNavDurationForAckedAggregateDataFrame(currentPacketsDestinationMacAddress, txParameters));


    for(unsigned int i = 0; (i < currentAggregateFramePtr->size()); i++) {
        Packet& aPacket = (*(*currentAggregateFramePtr)[i]);

        CommonFrameHeader& header =
            aPacket.GetAndReinterpretPayloadData<CommonFrameHeader>(
                sizeof(MpduDelimiterFrame));

        header.duration = navDurationField;

    }//for//

    (*this).currentPacketsAccessCategoryIndex = ConvertToPacketPriority(accessCategoryIndex);
    (*this).macState = WaitingForAckState;
    (*this).lastSentFrameWasAn = SentFrameType::AggregateFrame;

    OutputTraceAndStatsForAggregatedFrameTransmission();

    physicalLayer.TransmitAggregateFrame(
        currentAggregateFramePtr,
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
            linkInfo.lowestLiveFrameSequenceNumber);

    const double transmitPowerDbm =
        theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(destinationAddress);

    TransmissionParameters txParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
        destinationAddress, txParameters);

    CommonFrameHeader& header =
        frameToSendPtr->GetAndReinterpretPayloadData<CommonFrameHeader>();

    header.duration =
        ConvertTimeToDurationFieldValue(
            CalculateNavDurationForAckedAggregateDataFrame(destinationAddress, txParameters));

    (*this).macState = WaitingForAckState;
    (*this).lastSentFrameWasAn = SentFrameType::BlockAckRequestFrame;

    OutputTraceAndStatsForBlockAckRequestFrameTransmission(frameToSendPtr->LengthBytes());

    physicalLayer.TransmitFrame(
        frameToSendPtr,
        txParameters,
        transmitPowerDbm,
        delayUntilTransmitting);

}//TransmitABlockAckRequest//


inline
bool Dot11Mac::CanAggregateCurrentFrame() const
{
    assert(currentPacketPtr != nullptr);
    assert(currentAggregateFramePtr == nullptr);

    if (currentPacketIsAManagementFrame) {
        return false;
    }//if//


    if (!FrameAggregationIsEnabledFor(currentPacketsDestinationMacAddress)) {
        return false;
    }//if//

    if ((!allowFrameAggregationWithTxopZero) &&
        (currentTransmitOpportunityAkaTxopEndTime == ZERO_TIME)) {

        return false;
    }//if//

    if ((protectAggregateFramesWithSingleAckedFrame) &&
        (currentTransmitOpportunityAckedFrameCount == 0)) {

        return false;
    }//if//

    return true;

}//CanAggregateCurrentFrame//


inline
unsigned int Dot11Mac::CalcNormalRetryTxLimit(const unsigned int packetLengthBytes) const
{
    if (packetLengthBytes <= rtsThresholdSizeBytes) {
        return (shortFrameRetryTxLimit);
    }
    else {
        return (longFrameRetryTxLimit);
    }//if//
}



inline
void Dot11Mac::TransmitAFrame(
    const unsigned int accessCategoryIndex,
    const bool doNotRequestToSendAkaRts,
    const SimTime& delayUntilTransmitting,
    bool& packetHasBeenSentToPhy)
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    (*this).RemoveLifetimeExpiredPacketsFromQueue(accessCategoryIndex);

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

    assert (currentAggregateFramePtr == nullptr);

    if (currentPacketPtr == nullptr) {

        if ((accessCategoryIndex == accessCategoryIndexForManagementFrames) &&
            (!accessCategoryInfo.shouldRetransmitLastDataPacket)) {

            while (!managementFrameQueue.empty()) {

                ManagementFrameHeader& frameHeader =
                    managementFrameQueue.front().framePtr->
                        GetAndReinterpretPayloadData<ManagementFrameHeader>();

                if ((GetOperationMode() != ApMode) ||
                    (!apControllerPtr->StationIsAsleep(frameHeader.header.receiverAddress))) {

                    (*this).currentPacketPtr = move(managementFrameQueue.front().framePtr);
                    (*this).currentPacketsRetryTxCount = managementFrameQueue.front().retryTxCount;
                    (*this).managementFrameQueue.pop_front();
                    (*this).currentPacketIsAManagementFrame = true;
                    (*this).currentPacketIsForTxopCollisionDetect = false;
                    (*this).currentPacketsAccessCategoryIndex =
                        ConvertToPacketPriority(accessCategoryIndexForManagementFrames);

                    (*this).currentPacketsDestinationMacAddress = frameHeader.header.receiverAddress;

                    (*this).GetNewSequenceNumber(
                        currentPacketsDestinationMacAddress,
                        ConvertToPacketPriority(accessCategoryIndexForManagementFrames),
                        true,
                        (*this).currentPacketsSequenceNumber);

                    frameHeader.theSequenceControlField.sequenceNumber = currentPacketsSequenceNumber;
                    break;

                }//if//

                unique_ptr<Packet> aManagementFramePtr = move(managementFrameQueue.front().framePtr);

                apControllerPtr->BufferManagementFrameForSleepingStation(
                    frameHeader.header.receiverAddress, aManagementFramePtr,
                    managementFrameQueue.front().retryTxCount,
                    currentTime);

                managementFrameQueue.pop_front();

                assert(currentPacketPtr == nullptr);

            }//while//

        }//if//

        // Note: this flag only for stopping management frames from preempting data frame
        // retries (only strict FIFO retries option is enabled).

        accessCategoryInfo.shouldRetransmitLastDataPacket = false;

        if (currentPacketPtr == nullptr) {

            currentPacketIsAManagementFrame = false;

            bool wasRetrieved;

            (*this).RetrievePacketFromNetworkLayerForAccessCategory(accessCategoryIndex, wasRetrieved);
            //cout << "TransmitAFrame" << endl;//yes

            if (!wasRetrieved) {
                packetHasBeenSentToPhy = false;
                accessCategoryInfo.hasPacketToSend = false;
                assert(currentPacketPtr == nullptr);
                return;
            }//if//
        }//if//
    }//if//

    if ((operationMode == AdhocMode) &&
        (!theAdaptiveRateControllerPtr->StationIsKnown(currentPacketsDestinationMacAddress))) {

        // In AdHoc mode, assume everyone is like this station.

        theAdaptiveRateControllerPtr->AddNewStation(
            currentPacketsDestinationMacAddress,
            physicalLayer.GetChannelCount(),
            physicalLayer.GetIsAHighThroughputStation());

    }//if//

    if (!currentPacketIsAManagementFrame) {
        if (currentPacketsRetryTxCount == 0) {

            theAdaptiveRateControllerPtr->NotifyStartingAFrameTransmissionSequence(
                currentPacketsDestinationMacAddress, accessCategoryIndex);

        }//if//
    }//if//

    TransmissionParameters txParameters;

    // Bizarrely, Minstrel rate adaption algorithm may try to force the use of RTS/CTS
    // even if RTS is not supposed to be used.  It also molests retry limits.

    bool minstrelUseRtsEvenIfDisabled = false;
    bool overridesStandardRetryTxLimits = false;

    if (currentPacketIsAManagementFrame) {
        if ((currentPacketPtr != nullptr) && (IsABeaconFrame(*currentPacketPtr))) {

            theAdaptiveRateControllerPtr->GetDataRateInfoForBeaconFrame(txParameters);

            assert(txParameters.bandwidthNumberChannels == 1);
        }
        else {
            theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
                currentPacketsDestinationMacAddress, txParameters);
        }//if//
    }
    else {
        unsigned int dynamicRetryTxLimit = 0;

        theAdaptiveRateControllerPtr->GetDataRateInfoForDataFrameToStation(
            currentPacketsDestinationMacAddress,
            (currentPacketsRetryTxCount + 1),
            accessCategoryIndex,
            txParameters,
            minstrelUseRtsEvenIfDisabled,
            overridesStandardRetryTxLimits,
            dynamicRetryTxLimit);

        if (overridesStandardRetryTxLimits) {
            // Minstrel can override retry limit.
            (*this).currentFrameRetryTxLimit = dynamicRetryTxLimit;
        }//if//
    }//if//


    if (!overridesStandardRetryTxLimits) {
        (*this).currentFrameRetryTxLimit =
            CalcNormalRetryTxLimit(currentPacketPtr->LengthBytes());
    }//if//

    const double transmitPowerDbm =
        theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(currentPacketsDestinationMacAddress);

    assert(currentPacketPtr != nullptr);
    assert(currentAggregateFramePtr == nullptr);

    if ((!currentPacketIsAManagementFrame) &&
        (FrameAggregationIsEnabledFor(currentPacketsDestinationMacAddress))) {

        if (!currentPacketsDestinationMacAddress.IsAMulticastAddress()) {

            OutgoingLinkInfo& linkInfo =
                outgoingLinkInfoMap[
                    AddressAndTrafficIdMapKey(
                        currentPacketsDestinationMacAddress,
                        currentPacketsAccessCategoryIndex)];

            linkInfo.lowestLiveFrameSequenceNumber = currentPacketsSequenceNumber;

            if (!linkInfo.blockAckSessionIsEnabled) {

                // Using Block Ack Request as ADDBA (Add Block Ack Session Request)
                // This makes the initial Block Ack Request to start the session
                // and setup the Block Ack sequence number windows.

                linkInfo.blockAckSessionIsEnabled = true;
                linkInfo.blockAckRequestNeedsToBeSent = true;

            }//if//

            if (linkInfo.blockAckRequestNeedsToBeSent) {
                (*this).currentPacketsAccessCategoryIndex = ConvertToPacketPriority(accessCategoryIndex);

                (*this).RequeuePacketForRetry(
                    accessCategoryIndex,
                    currentPacketPtr,
                    currentPacketsNextHopNetworkAddress,
                    currentPacketsAccessCategoryIndex,
                    currentPacketsEtherType,
                    currentPacketsQueueInsertionTime,
                    currentPacketsRetryTxCount);

                accessCategoryInfo.shouldRetransmitLastDataPacket = false;

                (*this).TransmitABlockAckRequest(
                    currentPacketsDestinationMacAddress,
                    currentPacketsAccessCategoryIndex,
                    delayUntilTransmitting);

                packetHasBeenSentToPhy = true;
                return;
            }//if//
        }//if//

        assert(currentAggregateFramePtr == nullptr);

        if (CanAggregateCurrentFrame()) {

            (*this).BuildAggregateFrameFromCurrentFrame();

        }//if//
    }//if//


    if (currentPacketPtr == nullptr) {
        assert(currentAggregateFramePtr != nullptr);

        (*this).TransmitAnAggregateFrame(
            accessCategoryIndex,
            txParameters,
            transmitPowerDbm,
            delayUntilTransmitting);

        packetHasBeenSentToPhy = true;
        return;

    }//if//

    const unsigned int frameSizeBytes = currentPacketPtr->LengthBytes();

    if (frameSizeBytes <= rtsThresholdSizeBytes) {
        (*this).lastSentFrameWasAn = SentFrameType::ShortFrame;
    }
    else {
        (*this).lastSentFrameWasAn = SentFrameType::LongFrame;
    }//if//

    if (currentPacketsDestinationMacAddress.IsABroadcastOrAMulticastAddress()) {

        if (currentPacketIsAManagementFrame) {
            OutputTraceAndStatsForManagementFrameTransmission();
        }
        else {
            OutputTraceAndStatsForBroadcastDataFrameTransmission();

        }//if//

        // A transmission that does not require an immediate frame as a response is defined as a successful transmission.
        // Next Packet => Reset Contention window.

        accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

        macState = BusyMediumState;
    }
    else {
        OutgoingLinkInfo& linkInfo =
            outgoingLinkInfoMap[
                AddressAndTrafficIdMapKey(
                    currentPacketsDestinationMacAddress, currentPacketsAccessCategoryIndex)];

        linkInfo.lowestLiveFrameSequenceNumber = currentPacketsSequenceNumber;

        if ((doNotRequestToSendAkaRts) ||
             ((frameSizeBytes <= rtsThresholdSizeBytes) && (!minstrelUseRtsEvenIfDisabled))) {

            const SimTime navDuration =
                CalculateNavDurationForAckedDataFrame(currentPacketsDestinationMacAddress, txParameters);

            CommonFrameHeader& frameHeader =
                currentPacketPtr->GetAndReinterpretPayloadData<CommonFrameHeader>();

            frameHeader.theFrameControlField.isRetry = (currentPacketsRetryTxCount > 0);
            frameHeader.duration = ConvertTimeToDurationFieldValue(navDuration);

            if (currentPacketIsAManagementFrame) {
                OutputTraceAndStatsForManagementFrameTransmission();
            }
            else {
                OutputTraceAndStatsForUnicastDataFrameTransmission();
            }//if//

            (*this).currentPacketsDestinationMacAddress = frameHeader.receiverAddress;
            (*this).macState = WaitingForAckState;
        }
        else {
            // RTS Special Case.

            (*this).lastSentFrameWasAn = SentFrameType::RequestToSendFrame;

            RequestToSendFrame rtsFrame;

            rtsFrame.header.theFrameControlField.frameTypeAndSubtype = RTS_FRAME_TYPE_CODE;
            rtsFrame.header.theFrameControlField.isRetry = (linkInfo.requestToSendOrBarTxCount > 0);
            rtsFrame.header.receiverAddress = currentPacketsDestinationMacAddress;
            rtsFrame.transmitterAddress = myMacAddress;

            rtsFrame.header.duration =
                ConvertTimeToDurationFieldValue(
                    CalculateNavDurationForRtsFrame(
                        rtsFrame.header.receiverAddress,
                        currentPacketPtr->LengthBytes(),
                        txParameters));

            unique_ptr<Packet> rtsPacketToSendPtr =
                Packet::CreatePacket(*simEngineInterfacePtr, rtsFrame);

            (*this).currentPacketsAccessCategoryIndex = ConvertToPacketPriority(accessCategoryIndex);

            TransmissionParameters rtsTxParameters;

            theAdaptiveRateControllerPtr->GetDataRateInfoForRtsOrCtsFrameToStation(
                currentPacketsDestinationMacAddress, rtsTxParameters);

            if (currentTransmitOpportunityAkaTxopEndTime != ZERO_TIME) {
                SimTime frameTransmissionEndTime =
                    (currentTime + delayUntilTransmitting +
                     CalculateFrameDuration(rtsPacketToSendPtr->LengthBytes(), rtsTxParameters));

                // Extend NAV beyond current packet sequence.

                AddExtraNavDurationToPacketForNextFrameIfInATxop(
                    accessCategoryIndex, frameTransmissionEndTime, *rtsPacketToSendPtr);
            }//if//

            OutputTraceAndStatsForRtsFrameTransmission(
                linkInfo.requestToSendOrBarTxCount,
                rtsPacketToSendPtr->LengthBytes());

            physicalLayer.TransmitFrame(
                rtsPacketToSendPtr,
                rtsTxParameters,
                transmitPowerDbm,
                delayUntilTransmitting);

            macState = WaitingForCtsState;

            packetHasBeenSentToPhy = true;

            return;

        }//if//
    }//if//

    // Going to move frame down to Phy, but will retrieve it later if necessary (retry).

    unique_ptr<Packet> packetToSendPtr = move(currentPacketPtr);

    if (currentTransmitOpportunityAkaTxopEndTime != ZERO_TIME) {
        SimTime frameTransmissionEndTime =
            (currentTime + delayUntilTransmitting +
             CalculateFrameDuration(packetToSendPtr->LengthBytes(), txParameters));

        AddExtraNavDurationToPacketForNextFrameIfInATxop(
            accessCategoryIndex, frameTransmissionEndTime, *packetToSendPtr);
    }//if//

    (*this).currentPacketsAccessCategoryIndex = ConvertToPacketPriority(accessCategoryIndex);

    //dcc
    assert(packetToSendPtr != nullptr);
    //packetSizeforDCC[currentTime] = packetToSendPtr->LengthBytes();
    //cout << "packetsize: " << packetToSendPtr->LengthBytes() << endl;

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

    //cout << "before dequeue time: " << currentTime << endl;

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
                if (currentPacketPtr == nullptr) {
                    accessCategoryInfo.hasPacketToSend =
                        ThereAreQueuedPacketsForAccessCategory(accessCategoryIndex);
                }//if//

                if (accessCategoryInfo.hasPacketToSend) {
                    (*this).CalculateAndSetTransmitOpportunityDurationAkaTxop(accessCategoryIndex);

                    (*this).TransmitAFrame(
                        accessCategoryIndex, false, aRxTxTurnaroundTime, packetHasBeenSentToThePhy);
                    //cout << "ProcessInterframeSpaceAndBackoffTimeout" << endl;//yes
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
void Dot11Mac::DropCurrentPacketAndGoToNextPacket()
{
    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[currentPacketsAccessCategoryIndex];

    OutputTraceAndStatsForPacketRetriesExceeded();

    OutgoingLinkInfo& linkInfo =
        outgoingLinkInfoMap[
            AddressAndTrafficIdMapKey(
                currentPacketsDestinationMacAddress,
                currentPacketsAccessCategoryIndex)];

    linkInfo.lowestLiveFrameSequenceNumber =
        TwelveBitSequenceNumberPlus1(currentPacketsSequenceNumber);

    if (linkInfo.blockAckSessionIsEnabled) {

        linkInfo.blockAckRequestNeedsToBeSent = true;

    }//if//

    if (!currentPacketIsAManagementFrame) {

        networkLayerPtr->ReceiveUndeliveredPacketFromMac(
            interfaceIndex,
            currentPacketPtr,
            currentPacketsNextHopNetworkAddress);
    }
    else {
        currentPacketPtr.reset();
    }//if//

    assert(currentPacketPtr == nullptr);

    accessCategoryInfo.hasPacketToSend =
        ThereAreQueuedPacketsForAccessCategory(currentPacketsAccessCategoryIndex);

    // Next Packet => Reset Contention window.
    // Always do backoff procedure regardless of whether there is a packet to send.

    accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

    if (!currentPacketIsAManagementFrame) {
        theAdaptiveRateControllerPtr->NotifyFinishingAFrameTransmissionSequence(
            currentPacketsDestinationMacAddress,
            1,
            0, // # Successful mpdus.
            currentPacketsRetryTxCount,
            currentPacketsAccessCategoryIndex);
    }//if//

    (*this).RecalcRandomBackoff(accessCategoryInfo);

}//DropCurrentPacketAndGoToNextPacket//



inline
void Dot11Mac::NoResponseToAggregateFrameAction()
{
    assert(lastSentFrameWasAn == SentFrameType::AggregateFrame);

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    OutgoingLinkInfo& linkInfo =
        outgoingLinkInfoMap[
            AddressAndTrafficIdMapKey(
                currentPacketsDestinationMacAddress,
                currentPacketsAccessCategoryIndex)];

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[currentPacketsAccessCategoryIndex];

    assert(currentPacketPtr == nullptr);
    assert(currentAggregateFramePtr == nullptr);

    physicalLayer.TakeOwnershipOfLastTransmittedAggregateFrame(currentAggregateFramePtr);

    (*this).macState = BusyMediumState;

    vector<unique_ptr<Packet> >& sentSubframes = *currentAggregateFramePtr;

    const unsigned int numberTransmitedSubframes = static_cast<unsigned int>(sentSubframes.size());

    theAdaptiveRateControllerPtr->NotifyFinishingAFrameTransmissionSequence(
        currentPacketsDestinationMacAddress,
        numberTransmitedSubframes,
        0, // # Successful MPDUs
        (currentPacketsRetryTxCount + 1),
        currentPacketsAccessCategoryIndex);

    // Determine if frames should be requeued for retry.

    for(unsigned int i = 0; (i < sentSubframes.size()); i++) {
        const AggregateSubframeRetryInfo& retryInfo = currentAggregateFrameRetryInfo[i];

        const unsigned int packetLengthBytes =
            (sentSubframes[i]->LengthBytes() - sizeof(MpduDelimiterFrame));

        if ((retryInfo.retryTxCount >= currentFrameRetryTxLimit) &&
            ((currentTime - retryInfo.queueInsertionTime) >= accessCategoryInfo.frameLifetime)) {

            const Packet& aSubframe = *sentSubframes[i];
            const QosDataFrameHeader& header =
                aSubframe.GetAndReinterpretPayloadData<QosDataFrameHeader>(
                    sizeof(MpduDelimiterFrame));

            // Send Block Ack Request as a "management frame" to shift sequence number
            // window at destination for dropped frame.  In case that fails,
            // "mark" the link as needing an sequence number window update.

            linkInfo.blockAckRequestNeedsToBeSent = true;
            linkInfo.lowestLiveFrameSequenceNumber =
                TwelveBitSequenceNumberPlus1(header.theSequenceControlField.sequenceNumber);

            RemoveMpduDelimiterAndPaddingFromFrame(*sentSubframes[i]);
            sentSubframes[i]->DeleteHeader(sizeof(QosDataFrameHeader));

            networkLayerPtr->ReceiveUndeliveredPacketFromMac(
                interfaceIndex,
                sentSubframes[i],
                currentPacketsNextHopNetworkAddress);

        }//if//
    }//for//

    // Requeue frames in reverse order.

    for(int i = static_cast<int>(sentSubframes.size()) - 1; (i >= 0); i--) {
        const unsigned int subframeIndex = static_cast<unsigned int>(i);

        if (sentSubframes[subframeIndex] != nullptr) {

            const AggregateSubframeRetryInfo& retryInfo =
                currentAggregateFrameRetryInfo[subframeIndex];

            const Packet& aSubframe = *sentSubframes[i];
            const QosDataFrameHeader& header =
                aSubframe.GetAndReinterpretPayloadData<QosDataFrameHeader>(
                    sizeof(MpduDelimiterFrame));

            const unsigned short int sequenceNumber = header.theSequenceControlField.sequenceNumber;

            RemoveMpduDelimiterAndPaddingFromFrame(*sentSubframes[subframeIndex]);
            sentSubframes[subframeIndex]->DeleteHeader(sizeof(QosDataFrameHeader));

            networkOutputQueuePtr->RequeueAtFront(
                sentSubframes[subframeIndex],
                currentPacketsNextHopNetworkAddress,
                currentPacketsAccessCategoryIndex,
                currentPacketsEtherType,
                retryInfo.queueInsertionTime,
                (retryInfo.retryTxCount + 1),
                sequenceNumber);

            if (isInStrictlyFifoRetriesMode) {
                accessCategoryInfo.shouldRetransmitLastDataPacket = true;
            }//if//
        }//if//
    }//for//

    currentAggregateFramePtr.reset();

}//NoResponseToAggregateFrameAction//




inline
void Dot11Mac::NeverReceivedClearToSendOrAcknowledgementAction()
{
    // Make sure that no longer in waiting for ACK or CTS state.

    assert((macState != WaitingForCtsState) && (macState != WaitingForAckState));

    if (WakeupTimerIsActive()) {
        (*this).CancelWakeupTimer();
    }//if//

    EdcaAccessCategoryInfo& accessCategoryInfo = (*this).accessCategories[currentPacketsAccessCategoryIndex];

    if (lastSentFrameWasAn != SentFrameType::PowerSavePollResponse) {
        (*this).DoubleTheContentionWindowAndPickANewBackoff(currentPacketsAccessCategoryIndex);
    }//if//

    OutputTraceForCtsOrAckTimeout();

    switch (lastSentFrameWasAn) {

    case SentFrameType::RequestToSendFrame: {

        OutgoingLinkInfo& linkInfo =
            outgoingLinkInfoMap[
                AddressAndTrafficIdMapKey(
                    currentPacketsDestinationMacAddress,
                    currentPacketsAccessCategoryIndex)];

        linkInfo.requestToSendOrBarTxCount++;

        if (linkInfo.requestToSendOrBarTxCount < requestToSendRetryTxLimit) {
            if (!currentPacketIsAManagementFrame) {

                (*this).RequeuePacketForRetry(
                    currentPacketsAccessCategoryIndex,
                    currentPacketPtr,
                    currentPacketsNextHopNetworkAddress,
                    currentPacketsAccessCategoryIndex,
                    currentPacketsEtherType,
                    currentPacketsQueueInsertionTime,
                    currentPacketsRetryTxCount);
            }
            else {
                (*this).RequeueManagementFrame(currentPacketPtr, currentPacketsRetryTxCount);
            }//if//
        }
        else {
            linkInfo.requestToSendOrBarTxCount = 0;
            (*this).DropCurrentPacketAndGoToNextPacket();
        }//if//

        break;

    }
    case SentFrameType::ShortFrame:
    case SentFrameType::LongFrame:
        // Long Frames not used when RTS disabled (most of the time)

        physicalLayer.TakeOwnershipOfLastTransmittedFrame(currentPacketPtr);

        (*this).currentPacketsRetryTxCount++;

        if (currentPacketsRetryTxCount < currentFrameRetryTxLimit) {
            if (!currentPacketIsAManagementFrame) {

                (*this).RequeuePacketForRetry(
                    currentPacketsAccessCategoryIndex,
                    currentPacketPtr,
                    currentPacketsNextHopNetworkAddress,
                    currentPacketsAccessCategoryIndex,
                    currentPacketsEtherType,
                    currentPacketsQueueInsertionTime,
                    currentPacketsRetryTxCount);
            }
            else {
                (*this).RequeueManagementFrame(currentPacketPtr,  currentPacketsRetryTxCount);
            }//if//

        }
        else {
            (*this).DropCurrentPacketAndGoToNextPacket();
        }//if//

        break;

    case SentFrameType::AggregateFrame:

        (*this).NoResponseToAggregateFrameAction();

        break;

    case SentFrameType::BlockAckRequestFrame: {

        OutgoingLinkInfo& linkInfo =
            outgoingLinkInfoMap[
                AddressAndTrafficIdMapKey(
                    currentPacketsDestinationMacAddress,
                    currentPacketsAccessCategoryIndex)];

        linkInfo.requestToSendOrBarTxCount++;

        if (linkInfo.requestToSendOrBarTxCount >= blockAckRequestRetryTxLimit) {

            linkInfo.requestToSendOrBarTxCount = 0;

            // Block Ack Request is being used like an RTS, if it fails to be sent
            // then one DATA packet is dequeued. and dropped. (Stops infinite
            // Block Ack Requests).

            bool wasRetrieved;

            (*this).RetrievePacketFromNetworkLayerForAccessCategory(
                currentPacketsAccessCategoryIndex, wasRetrieved);
            //cout << "NeverReceivedClearToSendOrAcknowledgementAction" << endl;//no

            assert(wasRetrieved);

            (*this).DropCurrentPacketAndGoToNextPacket();

        }//if//

        break;

    }
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
                (*this).macState = TransientState;
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

    OutputTraceForCtsFrameTransmission(packetPtr->LengthBytes());

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

    // Reset RTS Retry Count / short retry count on successful RTS/CTS.

    OutgoingLinkInfo& linkInfo =
        outgoingLinkInfoMap[
            AddressAndTrafficIdMapKey(
                currentPacketsDestinationMacAddress,
                currentPacketsAccessCategoryIndex)];

    linkInfo.requestToSendOrBarTxCount = 0;

    bool packetWasSentToPhy;

    (*this).TransmitAFrame(
        currentPacketsAccessCategoryIndex, true, aShortInterframeSpaceTime, packetWasSentToPhy);
    //cout << "ProcessClearToSendFrame" << endl;//no

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
    (*this).currentPacketsDestinationMacAddress = psPollFrame.transmitterAddress;
    (*this).powerSavePollResponsePacketTxCount = (retryTxCount + 1);

    const unsigned int frameSizeBytes =
        sizeof(QosDataFrameHeader) + powerSavePollResponsePacketPtr->LengthBytes();

    TransmissionParameters txParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForDataFrameToStation(
        psPollFrame.transmitterAddress,
        1, // Assume not a retry
        0, // Assume AC does not matter (Minstrel).
        txParameters);

    QosDataFrameHeader dataFrameHeader;

    dataFrameHeader.header.theFrameControlField.frameTypeAndSubtype = QOS_DATA_FRAME_TYPE_CODE;
    dataFrameHeader.header.theFrameControlField.isRetry = (powerSavePollResponsePacketTxCount > 1);
    dataFrameHeader.header.duration =
        ConvertTimeToDurationFieldValue(CalculateNavDurationForAckedDataFrame(
            psPollFrame.transmitterAddress, txParameters));

    dataFrameHeader.header.receiverAddress = psPollFrame.transmitterAddress;
    (*this).currentPacketsDestinationMacAddress = dataFrameHeader.header.receiverAddress;

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
        outgoingLinkInfoMap[aKey].lowestLiveFrameSequenceNumber = 1;

        newSequenceNumber = 1;
    }
    else {
        OutgoingLinkInfo& linkInfo = iter->second;
        IncrementTwelveBitSequenceNumber(linkInfo.outgoingSequenceNumber);
        newSequenceNumber = linkInfo.outgoingSequenceNumber;

        if (isNonBlockAckedFrame) {
            linkInfo.lowestLiveFrameSequenceNumber = newSequenceNumber;
        }//if//

        assert(
            CalcTwelveBitSequenceNumberDifference(
                newSequenceNumber, linkInfo.lowestLiveFrameSequenceNumber) < BlockAckBitMapNumBits);
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
            linkInfo.outgoingSequenceNumber, linkInfo.lowestLiveFrameSequenceNumber);

    assert((diff >= 0) && (diff < BlockAckBitMapNumBits));

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

    linkInfo.lowestLiveFrameSequenceNumber = sequenceNum;
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

    EdcaAccessCategoryInfo& accessCategoryInfo =
        accessCategories[currentPacketsAccessCategoryIndex];

    if (lastSentFrameWasAn != SentFrameType::PowerSavePollResponse) {

        // Success => Reset Contention window.

        accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

        // Ownership of packet was given to PHY.

        assert(currentPacketPtr == nullptr);

        OutgoingLinkInfo& linkInfo =
            outgoingLinkInfoMap[
                AddressAndTrafficIdMapKey(
                    currentPacketsDestinationMacAddress,
                    ConvertToPacketPriority(currentPacketsAccessCategoryIndex))];

        assert(currentPacketsSequenceNumber == linkInfo.lowestLiveFrameSequenceNumber);

    }//if//

    if (!currentPacketIsAManagementFrame) {
        theAdaptiveRateControllerPtr->NotifyFinishingAFrameTransmissionSequence(
            currentPacketsDestinationMacAddress,
            1,
            1, // One successful transmission.
            (currentPacketsRetryTxCount + 1),
            currentPacketsAccessCategoryIndex);
    }//if//

    (*this).currentTransmitOpportunityAckedFrameCount++;
    (*this).DoSuccessfulTransmissionPostProcessing(false);

}//ProcessAcknowledgementFrame//



inline
void Dot11Mac::ProcessBlockAckFrame(const BlockAcknowledgementFrame& blockAckFrame)
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

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

    linkInfo.requestToSendOrBarTxCount = 0;
    linkInfo.blockAckRequestNeedsToBeSent = false;

    (*this).currentTransmitOpportunityAckedFrameCount++;

    assert(blockAckFrame.blockAckBitmap[0] == 0);
    assert(blockAckFrame.transmitterAddress == currentPacketsDestinationMacAddress);

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[currentPacketsAccessCategoryIndex];

    if (lastSentFrameWasAn == SentFrameType::AggregateFrame) {

        assert(currentPacketPtr == nullptr);
        assert(currentAggregateFramePtr == nullptr);

        physicalLayer.TakeOwnershipOfLastTransmittedAggregateFrame(currentAggregateFramePtr);

        (*this).macState = BusyMediumState;

        vector<unique_ptr<Packet> >& sentSubframes = *currentAggregateFramePtr;

        // Count and remove delivered frames

        unsigned int numberAckedSubframes = 0;

        // Determine if frames should be requeued for retry.

        bool advancingLowestLiveSequenceNumber = true;

        for(unsigned int i = 0; (i < sentSubframes.size()); i++) {

            const Packet& aSubframe = *sentSubframes[i];
            const QosDataFrameHeader& header =
                aSubframe.GetAndReinterpretPayloadData<QosDataFrameHeader>(
                    sizeof(MpduDelimiterFrame));

            if (blockAckFrame.IsAcked(header.theSequenceControlField.sequenceNumber)) {
                numberAckedSubframes++;

                if (advancingLowestLiveSequenceNumber) {
                    linkInfo.lowestLiveFrameSequenceNumber =
                        TwelveBitSequenceNumberPlus1(header.theSequenceControlField.sequenceNumber);
                }//if//

                sentSubframes[i].reset();
            }
            else {
                const unsigned int packetLength =
                    (aSubframe.LengthBytes() - sizeof(MpduDelimiterFrame));

                const AggregateSubframeRetryInfo& retryInfo = currentAggregateFrameRetryInfo[i];


                if ((retryInfo.retryTxCount >= currentFrameRetryTxLimit) &&
                    ((currentTime - retryInfo.queueInsertionTime) >= accessCategoryInfo.frameLifetime)) {

                    // Send Block Ack Request as a "management frame" to shift sequence number
                    // window at destination for dropped frame.  In case that fails,
                    // "mark" the link as needing an sequence number window update.

                    linkInfo.blockAckRequestNeedsToBeSent = true;

                    if (advancingLowestLiveSequenceNumber) {
                        linkInfo.lowestLiveFrameSequenceNumber =
                            TwelveBitSequenceNumberPlus1(header.theSequenceControlField.sequenceNumber);
                    }//if//

                    RemoveMpduDelimiterAndPaddingFromFrame(*sentSubframes[i]);
                    sentSubframes[i]->DeleteHeader(sizeof(QosDataFrameHeader));

                    networkLayerPtr->ReceiveUndeliveredPacketFromMac(
                        interfaceIndex,
                        sentSubframes[i],
                        currentPacketsNextHopNetworkAddress);
                }
                else {
                    if (advancingLowestLiveSequenceNumber) {
                        // Because aggregate subframes are sent in sequence # order.
                        // The first frame to retry becomes the lowest live sequence number.

                        linkInfo.lowestLiveFrameSequenceNumber =
                            header.theSequenceControlField.sequenceNumber;
                        advancingLowestLiveSequenceNumber = false;
                    }//if//
                }//if//
            }//if//
        }//for//

        theAdaptiveRateControllerPtr->NotifyFinishingAFrameTransmissionSequence(
            currentPacketsDestinationMacAddress,
            static_cast<unsigned int>(sentSubframes.size()),
            numberAckedSubframes,
            (currentPacketsRetryTxCount + 1),
            currentPacketsAccessCategoryIndex);

        // Requeue frames to retry in reverse order.

        for(int i = static_cast<int>(sentSubframes.size()) - 1; (i >= 0); i--) {
            const unsigned int subframeIndex = static_cast<unsigned int>(i);

            if (sentSubframes[subframeIndex] != nullptr) {

                const AggregateSubframeRetryInfo& retryInfo =
                    currentAggregateFrameRetryInfo[subframeIndex];

                const Packet& aSubframe = *sentSubframes[i];
                const QosDataFrameHeader& header =
                    aSubframe.GetAndReinterpretPayloadData<QosDataFrameHeader>(
                        sizeof(MpduDelimiterFrame));
                const unsigned short sequenceNumber = header.theSequenceControlField.sequenceNumber;

                RemoveMpduDelimiterAndPaddingFromFrame(*sentSubframes[subframeIndex]);
                sentSubframes[subframeIndex]->DeleteHeader(sizeof(QosDataFrameHeader));

                networkOutputQueuePtr->RequeueAtFront(
                    sentSubframes[subframeIndex],
                    currentPacketsNextHopNetworkAddress,
                    currentPacketsAccessCategoryIndex,
                    currentPacketsEtherType,
                    retryInfo.queueInsertionTime,
                    (retryInfo.retryTxCount + 1),
                    sequenceNumber);

                if (isInStrictlyFifoRetriesMode) {
                    // To enforce frame order with respect to management frames.

                    accessCategoryInfo.shouldRetransmitLastDataPacket = true;
                }//if//
            }//if//
        }//for//

        currentAggregateFramePtr.reset();

        assert((!protectAggregateFramesWithSingleAckedFrame) ||
            (accessCategoryInfo.currentContentionWindowSlots ==
               accessCategoryInfo.minContentionWindowSlots));

        accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;


        (*this).DoSuccessfulTransmissionPostProcessing(false);
    }
    else {
        assert(lastSentFrameWasAn == SentFrameType::BlockAckRequestFrame);

        //
        // This case is responding to block ack request (delayed BA).
        // There are no buffered subframes in the aggregate, must delete from Queue.
        //

        vector<unsigned short int> ackedSequenceNumbers;
        for(unsigned short int i = 0; (i < blockAckFrame.blockAckBitmap.size()); i++) {
            if (blockAckFrame.blockAckBitmap[i]) {
                ackedSequenceNumbers.push_back(
                    AddTwelveBitSequenceNumbers(blockAckFrame.startingSequenceControl, i));
            }//if//
        }//for//

        networkOutputQueuePtr->DeletePacketsBySequenceNumber(
            blockAckFrame.blockAckControl.trafficId,
            currentPacketsNextHopNetworkAddress,
            ackedSequenceNumbers);

        // Success => Reset Contention window.

        accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

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
    //cout << "ProcessWakeupTimerEvent: " << (double)simEngineInterfacePtr->CurrentTime() / (double)1000000000 << endl;
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
    (*this).managementFrameQueue.push_back(ManagementFrameQueueElem(move(framePtr), 0));
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

    anAssociationRequestFrame.theHtCapabilitiesFrameElement.highThroughputModeIsEnabled =
        physicalLayer.GetIsAHighThroughputStation();

    anAssociationRequestFrame.theHtCapabilitiesFrameElement.numChannelsBandwidth =
        static_cast<unsigned char>(physicalLayer.GetCurrentBandwidthNumChannels());

    anAssociationRequestFrame.managementHeader.theSequenceControlField.sequenceNumber = 0;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<AssociationRequestFrame>(
        *simEngineInterfacePtr,
        anAssociationRequestFrame);

    (*this).SendManagementFrame(packetPtr);

}//SendAssociationRequest//



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

    reassociationRequestFrame.managementHeader.theSequenceControlField.sequenceNumber = 0;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<ReassociationRequestFrame>(
        *simEngineInterfacePtr,
        reassociationRequestFrame);

    (*this).SendManagementFrame(packetPtr);

}//SendReassociationRequest//



inline
void Dot11Mac::SendDisassociation(const MacAddress& receiverAddress)
{
    DisassociationFrame aDisassociationFrame;

    aDisassociationFrame.managementHeader.header.theFrameControlField.frameTypeAndSubtype = DISASSOCIATION_FRAME_TYPE_CODE;
    aDisassociationFrame.managementHeader.header.theFrameControlField.isRetry = 0;
    aDisassociationFrame.managementHeader.header.duration = 0;
    aDisassociationFrame.managementHeader.header.receiverAddress = receiverAddress;
    aDisassociationFrame.managementHeader.transmitterAddress = myMacAddress;

    aDisassociationFrame.managementHeader.theSequenceControlField.sequenceNumber = 0;

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

    anAssociationResponseFrame.managementHeader.theSequenceControlField.sequenceNumber = 0;

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


    aReassociationResponseFrame.managementHeader.theSequenceControlField.sequenceNumber = 0;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<ReassociationResponseFrame>(
        *simEngineInterfacePtr,
        aReassociationResponseFrame);

    (*this).SendManagementFrame(packetPtr);

}//SendReassociationResponse//



inline
void Dot11Mac::SendAuthentication(const MacAddress& receiverAddress)
{
    AuthenticationFrame anAuthenticationFrame;

    anAuthenticationFrame.managementHeader.header.theFrameControlField.frameTypeAndSubtype = AUTHENTICATION_FRAME_TYPE_CODE;
    anAuthenticationFrame.managementHeader.header.theFrameControlField.isRetry = 0;
    anAuthenticationFrame.managementHeader.header.duration = 0;
    anAuthenticationFrame.managementHeader.header.receiverAddress = receiverAddress;
    anAuthenticationFrame.managementHeader.transmitterAddress = myMacAddress;

    anAuthenticationFrame.managementHeader.theSequenceControlField.sequenceNumber = 0;

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
void Dot11Mac::ResetOutgoingLinksTo(const MacAddress& macAddress)
{
    typedef map<AddressAndTrafficIdMapKey, OutgoingLinkInfo>::iterator IterType;

    for(IterType iter = outgoingLinkInfoMap.begin(); (iter != outgoingLinkInfoMap.end()); ++iter) {
        if (iter->first.macAddress == macAddress) {
            iter->second.blockAckSessionIsEnabled = false;
        }//if//
    }//for//

}//ResetOutgoingLinksTo//



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

    const unsigned int frameSizeBytes = aFrame.LengthBytes();
    bytesReceivedStatPtr->IncrementCounter(frameSizeBytes);

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

    const unsigned int frameSizeBytes = aFrame.LengthBytes();
    bytesReceivedStatPtr->IncrementCounter(frameSizeBytes);

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
void Dot11Mac::OutputTraceForPacketDequeue(
    const unsigned int accessCategoryIndex,
    const SimTime delayUntilAirBorne) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacPacketDequeueTraceRecord traceData;

            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

            const PacketId& thePacketId = currentPacketPtr->GetPacketId();
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

            msgStream << "PktId= " << currentPacketPtr->GetPacketId();
            msgStream << " AC= " << accessCategoryIndex;
            if (redundantTraceInformationModeIsOn) {
                msgStream << " Dur= " <<  ConvertTimeToStringSecs(delayUntilAirBorne);
            }
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Dequeue", msgStream.str());
        }//if//
    }//if//

}//OutputTraceForPacketDequeue//



inline
void Dot11Mac::OutputTraceAndStatsForRtsFrameTransmission(
    const unsigned int rtsRetryTxCount,
    const unsigned int frameSizeBytes) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacTxRtsTraceRecord traceData;

            traceData.accessCategory = static_cast<uint32_t>(currentPacketsAccessCategoryIndex);
            traceData.retry = rtsRetryTxCount;
            traceData.destNodeId = currentPacketsDestinationMacAddress.ExtractNodeId();

            assert(sizeof(traceData) == MAC_TX_RTS_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-RTS", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream << "AC= " << static_cast<unsigned int>(currentPacketsAccessCategoryIndex);
            msgStream << " S_Retry= " << rtsRetryTxCount;
            msgStream << " DestNode= " << currentPacketsDestinationMacAddress.ExtractNodeId();

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-RTS", msgStream.str());
        }//if//
    }//if//

    bytesSentStatPtr->IncrementCounter(frameSizeBytes);
    rtsFramesSentStatPtr->IncrementCounter();

}//OutputTraceAndStatsForRtsFrameTransmission//



inline
void Dot11Mac::OutputTraceForCtsFrameTransmission(const unsigned int frameSizeBytes) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-CTS");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-CTS", "");
        }//if//
    }//if//

    bytesSentStatPtr->IncrementCounter(frameSizeBytes);
    ctsFramesSentStatPtr->IncrementCounter();

}//OutputTraceForCtsFrameTransmission//


inline
void Dot11Mac::OutputTraceAndStatsForAggregatedFrameTransmission() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {

        assert(!currentAggregateFrameRetryInfo.empty());

        // Note: needs updated for A-MPDU RTS rules (no standard).

        const unsigned int shortFrameRetryTxCount = currentAggregateFrameRetryInfo.front().retryTxCount;
        const unsigned int longFrameRetryTxCount = 0;

        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacTxUnicastDataTraceRecord traceData;

            const PacketId& thePacketId = currentAggregateFramePtr->front()->GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            traceData.accessCategory = static_cast<uint32_t>(currentPacketsAccessCategoryIndex);
            traceData.shortFrameRetry = shortFrameRetryTxCount;
            traceData.longFrameRetry = longFrameRetryTxCount;
            traceData.destNodeId = currentPacketsDestinationMacAddress.ExtractNodeId();
            traceData.numSubframes = static_cast<uint32_t>(currentAggregateFramePtr->size());

            assert(sizeof(traceData) == MAC_TX_UNICAST_DATA_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-DATA-A", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << currentAggregateFramePtr->front()->GetPacketId();
            msgStream << " AC= " << static_cast<unsigned int>(currentPacketsAccessCategoryIndex);
            msgStream << " S-Retry= " << shortFrameRetryTxCount;
            msgStream << " L-Retry= " << longFrameRetryTxCount;
            msgStream << " DestNode= " << currentPacketsDestinationMacAddress.ExtractNodeId();
            msgStream << " Subframes= " << currentAggregateFramePtr->size();

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-DATA-A", msgStream.str());
        }//if//
    }//if//

    unsigned int aggregateFrameSizeBytes = 0;
    for(unsigned int i = 0; (i < currentAggregateFramePtr->size()); i++) {
        const Packet& aFrame = (*(*currentAggregateFramePtr)[i]);
        aggregateFrameSizeBytes += aFrame.LengthBytes();
    }//for//
    bytesSentStatPtr->IncrementCounter(aggregateFrameSizeBytes);

    dataAggregateFramesSentStatPtr->IncrementCounter();

    if (currentPacketsRetryTxCount > 0) {
        dataAggregateFramesResentStatPtr->IncrementCounter();
    }//if//

}//OutputTraceAndStatsForAggregatedFrameTransmission//



inline
void Dot11Mac::OutputTraceAndStatsForBroadcastDataFrameTransmission() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacTxBroadcastDataTraceRecord traceData;
            const PacketId& thePacketId = currentPacketPtr->GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            traceData.accessCategory = static_cast<uint32_t>(currentPacketsAccessCategoryIndex);

            assert(sizeof(traceData) == MAC_TX_BROADCAST_DATA_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-DATA-B", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream << "PktId= " << currentPacketPtr->GetPacketId();
            msgStream << " AC= " << static_cast<unsigned int>(currentPacketsAccessCategoryIndex);

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-DATA-B", msgStream.str());
        }//if//
    }//if//

    const unsigned int frameSizeBytes = currentPacketPtr->LengthBytes();
    bytesSentStatPtr->IncrementCounter(frameSizeBytes);

    broadcastDataFramesSentStatPtr->IncrementCounter();

}//OutputTraceAndStatsForBroadcastDataFrameTransmission//



inline
void Dot11Mac::OutputTraceAndStatsForUnicastDataFrameTransmission() const
{
    const unsigned int frameSizeBytes = currentPacketPtr->LengthBytes();

    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {

        unsigned int shortFrameRetryTxCount;
        unsigned int longFrameRetryTxCount;

        if (frameSizeBytes <= rtsThresholdSizeBytes) {
            shortFrameRetryTxCount = currentPacketsRetryTxCount;
            longFrameRetryTxCount = 0;

        }
        else {
            shortFrameRetryTxCount = 0;
            longFrameRetryTxCount = currentPacketsRetryTxCount;
        }//if//


        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacTxUnicastDataTraceRecord traceData;

            const PacketId& thePacketId = currentPacketPtr->GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            traceData.accessCategory = static_cast<uint32_t>(currentPacketsAccessCategoryIndex);
            traceData.shortFrameRetry = shortFrameRetryTxCount;
            traceData.longFrameRetry = longFrameRetryTxCount;
            traceData.destNodeId = currentPacketsDestinationMacAddress.ExtractNodeId();
            traceData.numSubframes = 0;

            assert(sizeof(traceData) == MAC_TX_UNICAST_DATA_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-DATA-U", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << currentPacketPtr->GetPacketId();
            msgStream << " AC= " << static_cast<unsigned int>(currentPacketsAccessCategoryIndex);
            msgStream << " S-Retry= " << shortFrameRetryTxCount;
            msgStream << " L-Retry= " << longFrameRetryTxCount;
            msgStream << " DestNode= " << currentPacketsDestinationMacAddress.ExtractNodeId();

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-DATA-U", msgStream.str());
        }//if//
    }//if//

    bytesSentStatPtr->IncrementCounter(frameSizeBytes);

    unicastDataFramesSentStatPtr->IncrementCounter();

    if (currentPacketsRetryTxCount > 0) {
        unicastDataFramesResentStatPtr->IncrementCounter();
    }//if//

}//OutputTraceAndStatsForUnicastDataFrameTransmission//



inline
void Dot11Mac::OutputTraceAndStatsForManagementFrameTransmission() const
{
    const CommonFrameHeader& header =
        currentPacketPtr->GetAndReinterpretPayloadData<CommonFrameHeader>();
    const unsigned char frameType = header.theFrameControlField.frameTypeAndSubtype;

    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {

        NodeId destNodeId = 0;
        if (!currentPacketsDestinationMacAddress.IsABroadcastOrAMulticastAddress()) {
            destNodeId = currentPacketsDestinationMacAddress.ExtractNodeId();
        }//if//

        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            Dot11MacTxManagementTraceRecord traceData;

            const PacketId& thePacketId = currentPacketPtr->GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            traceData.frameType = frameType;
            traceData.destNodeId = destNodeId;

            assert(sizeof(traceData) == DOT11_MAC_TX_MANAGEMENT_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-Management", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << currentPacketPtr->GetPacketId();
            msgStream << " FrameType= ";
            msgStream << ConvertToDot11FrameTypeName(frameType);

            msgStream << " DestNode= " << destNodeId;

            simEngineInterfacePtr->OutputTrace(
                modelName, theInterfaceId, "Tx-Management", msgStream.str());

        }//if//
    }//if//

    //stat
    const unsigned int frameSizeBytes = currentPacketPtr->LengthBytes();
    bytesSentStatPtr->IncrementCounter(frameSizeBytes);

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
void Dot11Mac::OutputTraceForAckFrameTransmission(const unsigned int frameSizeBytes) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-ACK");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-ACK", "");
        }//if//
    }//if//

    bytesSentStatPtr->IncrementCounter(frameSizeBytes);
    ackFramesSentStatPtr->IncrementCounter();

}//OutputTraceForAckFrameTransmission//



inline
void Dot11Mac::OutputTraceAndStatsForBlockAckRequestFrameTransmission(const unsigned int frameSizeBytes) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-BlockACK-Request");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-BlockACK-Request", "");
        }//if//
    }//if//

    bytesSentStatPtr->IncrementCounter(frameSizeBytes);
    blockAckRequestFramesSentStatPtr->IncrementCounter();

}//OutputTraceAndStatsForBlockAckRequestFrameTransmission//



inline
void Dot11Mac::OutputTraceAndStatsForBlockAckFrameTransmission(const unsigned int frameSizeBytes) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Tx-BlockACK");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "Tx-BlockACK", "");
        }//if//
    }//if//

    bytesSentStatPtr->IncrementCounter(frameSizeBytes);
    blockAckFramesSentStatPtr->IncrementCounter();

}//OutputTraceAndStatsForBlockAckFrameTransmission//



inline
void Dot11Mac::OutputTraceForCtsOrAckTimeout() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        const bool lastTransmissionWasAShortFrame =
            ((lastSentFrameWasAn == SentFrameType::ShortFrame) || (lastSentFrameWasAn == SentFrameType::RequestToSendFrame) ||
             (lastSentFrameWasAn == SentFrameType::BlockAckRequestFrame));


        unsigned int shortFrameRetryTxCount;
        unsigned int longFrameRetryTxCount = 0;

        if (lastSentFrameWasAn == SentFrameType::ShortFrame) {
            shortFrameRetryTxCount = currentPacketsRetryTxCount;
        }
        else {
            const OutgoingLinkInfo& linkInfo =
                 outgoingLinkInfoMap.find(
                    AddressAndTrafficIdMapKey(
                        currentPacketsDestinationMacAddress,
                        currentPacketsAccessCategoryIndex))->second;

            shortFrameRetryTxCount = linkInfo.requestToSendOrBarTxCount;

            if (lastSentFrameWasAn == SentFrameType::LongFrame) {
                longFrameRetryTxCount = currentPacketsRetryTxCount;
            }//if//
        }//if//


        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacCtsOrAckTimeoutTraceRecord traceData;

            const EdcaAccessCategoryInfo& accessCategoryInfo =
                accessCategories.at(currentPacketsAccessCategoryIndex);

            traceData.accessCategory = static_cast<uint32_t>(currentPacketsAccessCategoryIndex);
            traceData.windowSlot = accessCategoryInfo.currentContentionWindowSlots;
            traceData.shortFrameRetry = shortFrameRetryTxCount;
            traceData.longFrameRetry = longFrameRetryTxCount;
            traceData.shortFrameOrNot = lastTransmissionWasAShortFrame;

            assert(sizeof(traceData) == MAC_CTSORACK_TIMEOUT_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Timeout", traceData);
        }
        else {
            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(currentPacketsAccessCategoryIndex);
            ostringstream msgStream;

            msgStream << "AC= " << static_cast<unsigned int>(currentPacketsAccessCategoryIndex);
            msgStream << " Win= " << accessCategoryInfo.currentContentionWindowSlots;
            msgStream << " S-Retry= " << shortFrameRetryTxCount;
            msgStream << " L-Retry= " << longFrameRetryTxCount;

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
void Dot11Mac::OutputTraceAndStatsForPacketRetriesExceeded() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacPacketRetryExceededTraceRecord traceData;

            const PacketId& thePacketId = currentPacketPtr->GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == MAC_PACKET_RETRY_EXCEEDED_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "Drop", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream << "PktId= " << currentPacketPtr->GetPacketId();
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

    theAdaptiveRateControllerPtr->GetDataRateInfoForAckFrameToStation(
        destinationAddress, receivedFrameTxParameters, ackTxParameters);

    const double transmitPowerDbm = theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(destinationAddress);

    (*this).macState = CtsOrAckTransmissionState;

    OutputTraceForAckFrameTransmission(ackPacketPtr->LengthBytes());

    physicalLayer.TransmitFrame(
        ackPacketPtr,
        ackTxParameters,
        transmitPowerDbm,
        aShortInterframeSpaceTime);

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

    vector<unique_ptr<Packet> > bufferedPacketsToSendUp;

    bool haveAlreadySeenThisFrame = false;

    if (!dataFrameHeader.header.receiverAddress.IsABroadcastOrAMulticastAddress()) {

        theIncomingFrameBuffer.ProcessIncomingFrame(
            dataFrameHeader.transmitterAddress,
            dataFrameHeader.qosControlField.trafficId,
            dataFrameHeader.theSequenceControlField.sequenceNumber,
            haveAlreadySeenThisFrame,
            bufferedPacketsToSendUp);

        (*this).SendAckFrame(dataFrameHeader.transmitterAddress, receivedFrameTxParameters);

        if (haveAlreadySeenThisFrame) {
            //duplicated
            dataDuplicatedFramesReceivedStatPtr->IncrementCounter();
        }//if//
    }//if//

    if (!haveAlreadySeenThisFrame) {
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
    if ((macState == WaitingForCtsState) || (macState == WaitingForAckState)) {
        macState = BusyMediumState;
        (*this).NeverReceivedClearToSendOrAcknowledgementAction();
    }//if//

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

        theIncomingFrameBuffer.ProcessIncomingFrame(
            managementFrameHeader.transmitterAddress,
            ConvertToPacketPriority(accessCategoryIndexForManagementFrames),
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

    theIncomingFrameBuffer.ProcessIncomingFrame(
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
                CalculateNavDurationForAckedDataFrame(
                    header.receiverAddress, receivedFrameTxParameters);
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

    assert(blockAckFrame.blockAckBitmap[0] == 0);

    unique_ptr<Packet> ackPacketPtr = Packet::CreatePacket(*simEngineInterfacePtr, blockAckFrame);

    TransmissionParameters ackTxParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForAckFrameToStation(
        destinationAddress, receivedFrameTxParameters, ackTxParameters);

    const double transmitPowerDbm =
        theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(destinationAddress);

    OutputTraceAndStatsForBlockAckFrameTransmission(ackPacketPtr->LengthBytes());

    physicalLayer.TransmitFrame(
        ackPacketPtr,
        ackTxParameters,
        transmitPowerDbm,
        aShortInterframeSpaceTime);

    macState = CtsOrAckTransmissionState;

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

inline
SimTime Dot11Mac::CalcContentionWindowMinBackoffDuration(const unsigned short retryCount) const
{
    using std::min;

    const unsigned int contentionWindowSlots =
        min(((DefaultContentionWindowSlotsMin << retryCount) + 1), DefaultContentionWindowSlotsMax);

    return contentionWindowSlots*aSlotTime;

}//CalcContentionWindowMinBackoffDuration//


inline
DatarateBitsPerSec Dot11Mac::CalcDatarateBitsPerSecond(const TransmissionParameters& txParameters) const
{
    return physicalLayer.CalculateDatarateBitsPerSecond(txParameters);

}//CalcDatarateBitsPerSecond//

 inline
unsigned int Dot11Mac::GetMaxNumMimoSpatialStreams() const
{
    return physicalLayer.GetMaxNumMimoSpatialStreams();

}//GetMaxNumMimoSpatialStreams//

inline
SimTime Dot11Mac::CalcMaxTxDurationForUnicastDataWithRts(
    const unsigned int packetWithMacHeaderSizeBytes,
    const TransmissionParameters& dataFrameTxParameters,
    const TransmissionParameters& managementFrameTxParameters) const
{
    const SimTime rtsFrameDuration =
        CalculateFrameDuration(sizeof(RequestToSendFrame), managementFrameTxParameters);

    const SimTime ctsFrameDuration =
        CalculateFrameDuration(sizeof(ClearToSendFrame), managementFrameTxParameters);

    return
        (aShortInterframeSpaceTime +
         rtsFrameDuration +
         aShortInterframeSpaceTime +
         ctsFrameDuration +
         (*this).CalcMaxTxDurationForUnicastDataWithoutRts(
             packetWithMacHeaderSizeBytes,
             dataFrameTxParameters,
             managementFrameTxParameters));

}//CalcMaxTxDurationForUnicastDataWithRts//


inline
SimTime Dot11Mac::CalcMaxTxDurationForUnicastDataWithoutRts(
    const unsigned int packetWithMacHeaderSizeBytes,
    const TransmissionParameters& dataFrameTxParameters,
    const TransmissionParameters& managementFrameTxParameters) const
{
    const SimTime dataFrameDuration =
        CalculateFrameDuration(packetWithMacHeaderSizeBytes, dataFrameTxParameters);

    const SimTime ackFrameDuration =
        CalculateFrameDuration(sizeof(AcknowledgementAkaAckFrame), managementFrameTxParameters);

    return
        (aShortInterframeSpaceTime +
         dataFrameDuration +
         aShortInterframeSpaceTime +
         ackFrameDuration);

}//CalcMaxTxDurationForUnicastDataWithoutRts//


inline
SimTime Dot11MacDurationCalculationInterface::CalcContentionWindowMinBackoffDuration(const unsigned short retryCount) const
{
    return macPtr->CalcContentionWindowMinBackoffDuration(retryCount);
}//CalcContentionWindowMinBackoffDuration//

inline
SimTime Dot11MacDurationCalculationInterface::CalcMaxTxDurationForUnicastDataWithRts(
    const unsigned int packetWithMacHeaderSizeBytes,
    const TransmissionParameters& dataFrameTxParameters,
    const TransmissionParameters& managementFrameTxParameters) const
{
    return macPtr->CalcMaxTxDurationForUnicastDataWithRts(
        packetWithMacHeaderSizeBytes,
        dataFrameTxParameters,
        managementFrameTxParameters);

}//CalculateMaxFrameTransmissionDurationForDataFrameProtectedByRts//

inline
SimTime Dot11MacDurationCalculationInterface::CalcMaxTxDurationForUnicastDataWithoutRts(
    const unsigned int packetWithMacHeaderSizeBytes,
    const TransmissionParameters& dataFrameTxParameters,
    const TransmissionParameters& managementFrameTxParameters) const
{
    return macPtr->CalcMaxTxDurationForUnicastDataWithoutRts(
        packetWithMacHeaderSizeBytes,
        dataFrameTxParameters,
        managementFrameTxParameters);

}//CalculateMaxFrameTransmissionDurationForSingleDataFrame//

inline
DatarateBitsPerSec Dot11MacDurationCalculationInterface::CalcDatarateBitsPerSecond(
    const TransmissionParameters& txParameters) const
{
    return macPtr->CalcDatarateBitsPerSecond(txParameters);

}//CalcDatarateBitsPerSecond//

inline
unsigned int Dot11MacDurationCalculationInterface::GetMaxNumMimoSpatialStreams() const
{
    return macPtr->GetMaxNumMimoSpatialStreams();

}//GetMaxNumMimoSpatialStreams//


}//namespace//

#endif
