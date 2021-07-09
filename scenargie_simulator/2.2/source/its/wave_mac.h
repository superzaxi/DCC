// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef WAVE_MAC_H
#define WAVE_MAC_H

// WAVE = "Wireless Access in Vehicular Environment"
// CCH = "Control Channel"
// SCH = "Service Channel"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "scensim_netsim.h"

#include "dot11_mac.h"
#include "dot11_phy.h"
#include "wave_tracedefs.h"
#include "its_queues.h"

namespace Wave {

using std::shared_ptr;
using std::unique_ptr;
using std::move;
using std::cerr;
using std::endl;
using std::make_pair;
using std::list;
using std::ostringstream;

using ScenSim::SimTime;
using ScenSim::ParameterDatabaseReader;
using ScenSim::SimulationEngineInterface;
using ScenSim::NodeId;
using ScenSim::InterfaceId;
using ScenSim::NetworkLayer;
using ScenSim::RandomNumberGeneratorSeed;
using ScenSim::SimpleMacPacketHandler;
using ScenSim::NetworkAddress;
using ScenSim::PacketPriority;
using ScenSim::GenericMacAddress;
using ScenSim::Packet;
using ScenSim::ObjectMobilityModel;
using ScenSim::CounterStatistic;
using ScenSim::MacLayer;
using ScenSim::EventRescheduleTicket;
using ScenSim::ConvertStringToLowerCase;
using ScenSim::ConvertStringToInt;
using ScenSim::ConvertToString;
using ScenSim::ConvertStringToLowerCase;
using ScenSim::ZERO_TIME;
using ScenSim::INFINITE_TIME;
using ScenSim::NANO_SECOND;
using ScenSim::SECOND;
using ScenSim::MILLI_SECOND;
using ScenSim::ApplicationId;
using ScenSim::RandomNumberGenerator;
using ScenSim::RealStatistic;
using ScenSim::TraceApplication;
using ScenSim::ApplicationSendTraceRecord;
using ScenSim::ANY_NODEID;
using ScenSim::APPLICATION_SEND_TRACE_RECORD_BYTES;
using ScenSim::ApplicationReceiveTraceRecord;
using ScenSim::APPLICATION_RECEIVE_TRACE_RECORD_BYTES;
using ScenSim::ConvertTimeToDoubleSecs;
using ScenSim::ExtrinsicPacketInfoId;
using ScenSim::SimulationEvent;
using ScenSim::TraceMac;
using ScenSim::ObjectMobilityPosition;
using ScenSim::TraceNetwork;
using ScenSim::HashInputsToMakeSeed;
using ScenSim::PacketId;
using ScenSim::ConvertTimeToStringSecs;
using ScenSim::InterfaceOutputQueue;
using ScenSim::ExtrinsicPacketInformation;
using ScenSim::Application;
using ScenSim::EtherTypeField;
using ScenSim::ETHERTYPE_WSMP;
using ScenSim::EnqueueResultType;

using Dot11::ItsOutputQueueWithPrioritySubqueues;
using Dot11::DatarateBitsPerSec;

using Dot11::Dot11Mac;
using Dot11::Dot11Phy;
using std::set;
using std::pair;
using std::map;

typedef uint8_t ChannelNumberIndexType;
enum {
    CHANNEL_NUMBER_172 = 0,
    CHANNEL_NUMBER_174 = 1,
    CHANNEL_NUMBER_176 = 2,
    CHANNEL_NUMBER_178 = 3, //CCH
    CHANNEL_NUMBER_180 = 4,
    CHANNEL_NUMBER_182 = 5,
    CHANNEL_NUMBER_184 = 6,

    NUMBER_CHANNELS = 7,



    CHANNEL_NUMBER_UNKNOWN = 0, // use only for initializing.

    MIN_CHANNEL_NUMBER = 172,
    MAX_CHANNEL_NUMBER = 184,
    CHANNEL_INTERVAL = 2,
};//ChannelNumberIndexType//

typedef uint8_t ChannelCategoryType;
enum {
    CHANNEL_CATEGORY_CCH = 0,
    CHANNEL_CATEGORY_SCH = 1,

    // Add extra category below this line if necessary.


    NUMBER_CHANNEL_CATEGORIES,
};//ChannelCategoryType//

static const char* CHANNEL_CATEGORY_NAMES[] = {"cch", "sch", /*ext*/};


enum ChannelAccessType {
    CHANNEL_ACCESS_CONTINUE,
    CHANNEL_ACCESS_ALTERNATING,
};//ChannelAccessType//

struct WaveMacPhyInput {
    string phyDeviceName;
    shared_ptr<Dot11Phy> phyPtr;
    vector<ChannelNumberIndexType> channelNumberIds[NUMBER_CHANNEL_CATEGORIES];
};//WaveMacPhyInput

class WaveMac : public MacLayer {
public:
    static const string modelName;

    WaveMac(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int initInterfaceIndex,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const vector<WaveMacPhyInput>& initPhyInputs,
        const RandomNumberGeneratorSeed& nodeSeed);

    void SetEdcaParameter(
        const ChannelNumberIndexType& channelNumberId,
        const size_t accessCategoryIndex,
        const bool admissionControlIsEnabled,
        const int arbitrationInterframeSpaceNumber,
        const int contentionWindowMin,
        const int contentionWindowMax,
        const SimTime& txopDuration);

    void SetWsmpPacketHandler(
        const shared_ptr<SimpleMacPacketHandler>& initWsmpPacketHandlerPtr);

    virtual void NetworkLayerQueueChangeNotification();
    virtual void DisconnectFromOtherLayers();


    // Note: InsertPacektIntoCchOrSchQueueWhichSupportsChannelIdOf()
    //       inserts a packet to CCH/SCH queue which supports the
    //       specified channel number even if current channel number
    //       is not corresponeded to specified channel number.
    //
    //       Change channel number with following functions if necessary.
    //       - SetContinuousAccess()
    //       - SetAlternatingAccess()
    //       - SetImmediateSchAccess()
    //       - SetExtendedSchAccess()

    void InsertPacektIntoCchOrSchQueueWhichSupportsChannelIdOf(
        const ChannelNumberIndexType& channelNumberId,
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        const EtherTypeField& etherType,
        const DatarateBitsPerSec& theDatarateBitsPerSec,
        const double txPowerDbm,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDropPtr);

    void SetContinuousAccess(
        const ChannelNumberIndexType& channelNumberId);

    typedef pair<ChannelCategoryType, ChannelNumberIndexType> ChannelCategoryAndNumberIdType;

    void SetAlternatingAccess(
        const vector<ChannelCategoryAndNumberIdType>& channelCategoryAndNumberIds);

    void SetImmediateSchAccess(
        const ChannelNumberIndexType& schChannelNumberId);

    void SetExtendedSchAccess(
        const ChannelNumberIndexType& schChannelNumberId,
        const size_t numberExtendedAccessSlots);

private:

    class PhyEntity {
    public:
        PhyEntity(
            const WaveMac& waveMac,
            const WaveMacPhyInput& initPhyInput,
            const vector<shared_ptr<Dot11Mac> > initMacPtrsPerCategory);

        void SetContinuousAccess(
            const ChannelNumberIndexType& channelNumberId);
        void SetAlternatingAccess();
        void SetAlternatingAccess(
            const vector<ChannelCategoryType>& channelCategories);
        void SetAlternatingAccess(
            const vector<ChannelCategoryAndNumberIdType>& channelCategoryAndNumberIds);
        void SetImmediateSchAccess(
            const ChannelNumberIndexType& schChannelNumberId);
        void SetExtendedSchAccess(
            const ChannelNumberIndexType& schChannelNumberId,
            const size_t initNumberExtendedAccessSlots);

    private:
        const WaveMac& waveMac;
        const string phyDeviceName;

        shared_ptr<Dot11Phy> phyPtr;
        shared_ptr<Dot11Mac> macPtrs[NUMBER_CHANNEL_CATEGORIES];
        set<ChannelNumberIndexType> availableChannelNumberIds[NUMBER_CHANNEL_CATEGORIES];

        size_t numberExtendedSchAccessSlots;
        ChannelCategoryType currentCategory;
        ChannelNumberIndexType channelNumberPerCategory[NUMBER_CHANNEL_CATEGORIES];
        vector<ChannelCategoryType> alternatingChannelCategories;

        ChannelNumberIndexType nextChannelNumberIdInWaitingTransmissionSuspensionState;

        enum EventType {
            EVENT_TX_RX_START,
            EVENT_TX_RX_END,
            EVENT_SUSPEND_TRANSMISSION,
            EVENT_CHANNEL_SWITCH,
        };

        class TimerEvent : public SimulationEvent {
        public:
            TimerEvent(PhyEntity* initPhyEntityPtr)
                :
                phyEntityPtr(initPhyEntityPtr),
                event(EVENT_TX_RX_START)
            {}
            virtual void ExecuteEvent();
            void SetNextEventType(const EventType& initEvent) { event = initEvent; }
        private:
            PhyEntity* phyEntityPtr;
            EventType event;
        };
        shared_ptr<TimerEvent> timerEventPtr;
        EventRescheduleTicket timerEventTicket;

        bool CurrentChannelOrNextSwitchingChannelIs(const ChannelNumberIndexType& channelNumberId) const;
        SimTime CalculateCurrentTxRxOperationEndTime() const;
        ChannelCategoryType CalculateNonExtendedChannelCategoryAt(const SimTime time) const;

        void SwitchToCurrentIntervalChannel();
        void SwitchToChannelIfNecessary(const ChannelNumberIndexType& channelNumberId);

        void ResumeTxRxOperationAndScheduleEndEventIfNecessary();
        void SuspendTransmissionAndRescheduleChannelSwitchingEvent(
            const ChannelNumberIndexType& channelNumberId);
        void StartChannelSwitchngTime();
        void RescheduleTimerEvent(const EventType& event, const SimTime time);
        ChannelCategoryType GetChannelCategory(const ChannelNumberIndexType& channelNumberId) const;
        bool CurrentAlternatingChannelAccessSupports(const ChannelCategoryType& channelCategory) const;

        void OutputTraceAndStatsForNewChannelInterval(
            const ChannelNumberIndexType& prevChannelNumberId,
            const ChannelNumberIndexType& nextChannelNumberId) const;

        void OutputTraceAndStatsForChannelChange(
            const ChannelNumberIndexType& channelNumberId) const;
    };

    struct ChannelEntity {
        shared_ptr<PhyEntity> phyEntityPtr;
        shared_ptr<Dot11Mac> macPtr;
        shared_ptr<ItsOutputQueueWithPrioritySubqueues> outputQueuePtr;
    };

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    vector<ChannelEntity> channelEntities;

    SimTime channelIntervals[NUMBER_CHANNEL_CATEGORIES];
    SimTime syncTolerance;
    SimTime maxChannelSwitchingTime;

    void CreateDot11MacAndOutputQueueForChannelCategory(
        const ChannelCategoryType& channelCategory,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceId& phyDeviceName,
        const unsigned int initInterfaceIndex,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const shared_ptr<Dot11Phy> initPhyPtr,
        const RandomNumberGeneratorSeed& nodeSeed,
        shared_ptr<Dot11Mac>& macPtr,
        shared_ptr<ItsOutputQueueWithPrioritySubqueues>& outputQueuePtr);
};//WaveMac//

//------------------------------------------------------------------------

static inline
ChannelNumberIndexType ConvertToChannelNumberIndex(const int channelNumber)
{
    if (!((MIN_CHANNEL_NUMBER <= channelNumber && channelNumber <= MAX_CHANNEL_NUMBER) &&
          (channelNumber % CHANNEL_INTERVAL == 0))) {
        cerr << "Not supported channel number of " << channelNumber << endl;
        exit(1);
    }

    return ChannelNumberIndexType((channelNumber - MIN_CHANNEL_NUMBER) / CHANNEL_INTERVAL);
}//ConvertToChannelNumberIndex//

static inline
int ConvertToChannelNumber(const ChannelNumberIndexType& channelNumberId)
{
    return int(channelNumberId)*CHANNEL_INTERVAL + MIN_CHANNEL_NUMBER;
}//ConvertToChannelNumber//

static inline
string ConvertChannelNumberIdToString(const ChannelNumberIndexType& channelNumberId)
{
    return ConvertToString(ConvertToChannelNumber(channelNumberId));
}//ConvertChannelNumberIdToString//

static inline
void GetChannelCategoryAndNumber(
    const string& channelString,
    ChannelCategoryType& channelCategory,
    ChannelNumberIndexType& channelNumberId)
{
    assert(!channelString.empty());

    string lowerString = channelString;
    ConvertStringToLowerCase(lowerString);

    size_t numberStartPos = 0;
    for(; numberStartPos < lowerString.size(); numberStartPos++) {
        if (isdigit(lowerString[numberStartPos])) {
            break;
        }//if//
    }//for//

    int channelNumber;
    bool success;
    ConvertStringToInt(lowerString.substr(numberStartPos), channelNumber, success);

    if (!success) {
        cerr << "Error: invalid channel number " + channelString << endl;
        exit(1);
    }//if//

    channelNumberId = ConvertToChannelNumberIndex(channelNumber);

    if (lowerString.find("cch") != string::npos) {

        channelCategory = CHANNEL_CATEGORY_CCH;

    } else if (lowerString.find("sch") != string::npos) {

        channelCategory = CHANNEL_CATEGORY_SCH;

    } else {

        if (channelNumberId == CHANNEL_NUMBER_178) {
            channelCategory = CHANNEL_CATEGORY_CCH;
        } else {
            channelCategory = CHANNEL_CATEGORY_SCH;
        }
    }//if//
}//GetChannelCategoryAndNumber//

//-------------------------------------------------------------------------------

inline
WaveMac::WaveMac(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId,
    const unsigned int initInterfaceIndex,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const vector<WaveMacPhyInput>& initPhyInputs,
    const RandomNumberGeneratorSeed& nodeSeed)
    :
    simEngineInterfacePtr(initSimulationEngineInterfacePtr),
    channelEntities(NUMBER_CHANNELS),
    syncTolerance(
        theParameterDatabaseReader.ReadTime(
            "its-wave-sync-tolerance", theNodeId, theInterfaceId)),
    maxChannelSwitchingTime(
        theParameterDatabaseReader.ReadTime(
            "its-wave-max-channel-switching-time", theNodeId, theInterfaceId))
{
    for(ChannelCategoryType i = 0; i < NUMBER_CHANNEL_CATEGORIES; i++) {
        channelIntervals[i] =
            theParameterDatabaseReader.ReadTime(
                string("its-wave-") + CHANNEL_CATEGORY_NAMES[i] + "-interval", theNodeId, theInterfaceId);
    }//for//

    for(size_t i = 0; i < initPhyInputs.size(); i++) {
        const WaveMacPhyInput& phyInput = initPhyInputs[i];

        vector<shared_ptr<Dot11Mac> > macPtrs(NUMBER_CHANNEL_CATEGORIES);
        vector<shared_ptr<ItsOutputQueueWithPrioritySubqueues> > outputQueuePtrs(NUMBER_CHANNEL_CATEGORIES);

        for(ChannelCategoryType category = 0; category < NUMBER_CHANNEL_CATEGORIES; category++) {
            if (phyInput.channelNumberIds[category].empty()) {
                continue;
            }//if//

            (*this).CreateDot11MacAndOutputQueueForChannelCategory(
                category,
                theParameterDatabaseReader,
                theNodeId,
                phyInput.phyDeviceName,
                initInterfaceIndex,
                initNetworkLayerPtr,
                phyInput.phyPtr,
                nodeSeed,
                macPtrs[category],
                outputQueuePtrs[category]);
        }//for//

        shared_ptr<PhyEntity> phyEntityPtr(new PhyEntity(*this, phyInput, macPtrs));

        for(ChannelCategoryType category = 0; category < NUMBER_CHANNEL_CATEGORIES; category++) {
            const vector<ChannelNumberIndexType>& channelNumberIds = phyInput.channelNumberIds[category];

            for(size_t k = 0; k < channelNumberIds.size(); k++) {
                const ChannelNumberIndexType& channelNumberId = channelNumberIds[k];
                ChannelEntity& channelEntity = channelEntities.at(channelNumberId);

                assert(channelEntity.phyEntityPtr == nullptr &&
                       channelEntity.macPtr == nullptr &&
                       channelEntity.outputQueuePtr == nullptr &&
                       "Already assigned a phy device");

                channelEntity.phyEntityPtr = phyEntityPtr;
                channelEntity.macPtr = macPtrs[category];
                channelEntity.outputQueuePtr = outputQueuePtrs[category];
            }//for//
        }//for//
    }//for//

    ChannelNumberIndexType ipChannelNumberId = CHANNEL_NUMBER_172;

    if (theParameterDatabaseReader.ParameterExists(
            "its-wave-ip-packet-channel-number", theNodeId, theInterfaceId)) {

        ipChannelNumberId =
            ConvertToChannelNumberIndex(
                theParameterDatabaseReader.ReadInt(
                    "its-wave-ip-packet-channel-number", theNodeId, theInterfaceId));
    }//if //

    initNetworkLayerPtr->SetInterfaceMacLayer(
        initInterfaceIndex, channelEntities.at(ipChannelNumberId).macPtr);
    initNetworkLayerPtr->SetInterfaceOutputQueue(
        initInterfaceIndex, channelEntities.at(ipChannelNumberId).outputQueuePtr);
}//WaveMac//

inline
void WaveMac::CreateDot11MacAndOutputQueueForChannelCategory(
    const ChannelCategoryType& channelCategory,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& phyDeviceName,
    const unsigned int initInterfaceIndex,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const shared_ptr<Dot11Phy> initPhyPtr,
    const RandomNumberGeneratorSeed& nodeSeed,
    shared_ptr<Dot11Mac>& macPtr,
    shared_ptr<ItsOutputQueueWithPrioritySubqueues>& outputQueuePtr)
{
    string lowerCaseCategoryName = CHANNEL_CATEGORY_NAMES[channelCategory];
    ConvertStringToLowerCase(lowerCaseCategoryName);

    InterfaceId macInstance = phyDeviceName + "_" + lowerCaseCategoryName;

    if (theParameterDatabaseReader.ParameterExists(
            string("its-wave-phy-") + CHANNEL_CATEGORY_NAMES[channelCategory] + "-mac-name",
            theNodeId, phyDeviceName)) {

        macInstance = theParameterDatabaseReader.ReadString(
            string("its-wave-phy-") + CHANNEL_CATEGORY_NAMES[channelCategory] + "-mac-name",
            theNodeId, phyDeviceName);
    }//if//

    macPtr =
        Dot11Mac::Create(
            theParameterDatabaseReader,
            simEngineInterfacePtr,
            theNodeId,
            macInstance,
            initInterfaceIndex,
            initNetworkLayerPtr,
            initPhyPtr,
            nodeSeed);

    outputQueuePtr.reset(
        new ItsOutputQueueWithPrioritySubqueues(
            theParameterDatabaseReader,
            macInstance,
            simEngineInterfacePtr,
            macPtr->GetMaxPacketPriority()));

    macPtr->SetNetworkOutputQueue(outputQueuePtr);
}//CreateDot11MacAndOutputQueueForChannelCategory//

inline
void WaveMac::SetEdcaParameter(
    const ChannelNumberIndexType& channelNumberId,
    const size_t accessCategoryIndex,
    const bool admissionControlIsEnabled,
    const int arbitrationInterframeSpaceNumber,
    const int contentionWindowMin,
    const int contentionWindowMax,
    const SimTime& txopDuration)
{
    if (channelEntities.at(channelNumberId).macPtr == nullptr) {
        cerr << "Error: no mac for channel "
             << ConvertChannelNumberIdToString(channelNumberId) << endl;
        exit(1);
    }//if//

    channelEntities.at(channelNumberId).macPtr->SetEdcaParameter(
            accessCategoryIndex,
            admissionControlIsEnabled,
            arbitrationInterframeSpaceNumber,
            contentionWindowMin,
            contentionWindowMax,
            txopDuration);
}//SetEdcaParameter//

inline
void WaveMac::InsertPacektIntoCchOrSchQueueWhichSupportsChannelIdOf(
    const ChannelNumberIndexType& channelNumberId,
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority priority,
    const EtherTypeField& etherType,
    const DatarateBitsPerSec& theDatarateBitsPerSec,
    const double txPowerDbm,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDropPtr)
{
    if (channelEntities.at(channelNumberId).phyEntityPtr == nullptr) {
        cerr << "Error: no phy device for channel "
             << ConvertChannelNumberIdToString(channelNumberId) << endl;
        exit(1);
    }//if//

    channelEntities.at(channelNumberId).outputQueuePtr->
        InsertWithEtherTypeAndDatarateAndTxPower(
            packetPtr,
            nextHopAddress,
            priority,
            etherType,
            theDatarateBitsPerSec,
            txPowerDbm,
            enqueueResult,
            packetToDropPtr);

    channelEntities.at(channelNumberId).macPtr->NetworkLayerQueueChangeNotification();
}//InsertPacektIntoCchOrSchQueueWhichSupportsChannelIdOf//

inline
void WaveMac::NetworkLayerQueueChangeNotification()
{
    for(size_t i = 0; i < channelEntities.size(); i++) {
        if (channelEntities[i].macPtr != nullptr) {
            channelEntities[i].macPtr->NetworkLayerQueueChangeNotification();
        }//if//
    }//for//
}//NetworkLayerQueueChangeNotification//

inline
void WaveMac::SetWsmpPacketHandler(
    const shared_ptr<SimpleMacPacketHandler>& initWsmpPacketHandlerPtr)
{
    for(size_t i = 0; i < channelEntities.size(); i++) {
        if (channelEntities[i].macPtr != nullptr) {
            channelEntities[i].macPtr->SetMacPacketHandler(ETHERTYPE_WSMP, initWsmpPacketHandlerPtr);
        }//if//
    }//for//
}//SetWsmpPacketHandler//

inline
void WaveMac::DisconnectFromOtherLayers()
{
    channelEntities.clear();
}//DisconnectFromOtherLayers//

inline
void WaveMac::SetContinuousAccess(
    const ChannelNumberIndexType& channelNumberId)
{
    if (channelEntities.at(channelNumberId).phyEntityPtr == nullptr) {
        cerr << "Error: no phy device for channel "
             << ConvertChannelNumberIdToString(channelNumberId) << endl;
        exit(1);
    }//if//

    channelEntities.at(channelNumberId).phyEntityPtr->SetContinuousAccess(channelNumberId);
}//SetContinuousAccess//

inline
void WaveMac::SetAlternatingAccess(
    const vector<ChannelCategoryAndNumberIdType>& channelCategoryAndNumberIds)
{
    for(size_t i = 0; i < channelCategoryAndNumberIds.size(); i++) {
        const ChannelCategoryAndNumberIdType& categoryAndNumberId = channelCategoryAndNumberIds[i];
        const ChannelNumberIndexType channelNumberId = categoryAndNumberId.second;

        if (channelEntities.at(channelNumberId).phyEntityPtr == nullptr) {
            cerr << "Error: no phy device for channel "
                 << ConvertChannelNumberIdToString(channelNumberId) << endl;
            exit(1);
        }//if//
    }//for//

    const ChannelCategoryAndNumberIdType& aCategoryAndNumberId = channelCategoryAndNumberIds.front();
    const ChannelNumberIndexType aChannelNumberId = aCategoryAndNumberId.second;

    channelEntities.at(aChannelNumberId).phyEntityPtr->SetAlternatingAccess(channelCategoryAndNumberIds);
}//SetAlternatingAccess//

inline
void WaveMac::SetImmediateSchAccess(
    const ChannelNumberIndexType& schChannelNumberId)
{
    if (channelEntities.at(schChannelNumberId).phyEntityPtr == nullptr) {
        cerr << "Error: no phy device for channel "
             << ConvertChannelNumberIdToString(schChannelNumberId) << endl;
        exit(1);
    }//if//

    channelEntities.at(schChannelNumberId).phyEntityPtr->SetImmediateSchAccess(schChannelNumberId);
}//SetImmediateSchAccess//

inline
void WaveMac::SetExtendedSchAccess(
    const ChannelNumberIndexType& schChannelNumberId,
    const size_t numberExtendedAccessSlots)
{
    if (channelEntities.at(schChannelNumberId).phyEntityPtr == nullptr) {
        cerr << "Error: no phy device for channel "
             << ConvertChannelNumberIdToString(schChannelNumberId) << endl;
        exit(1);
    }//if//

    channelEntities.at(schChannelNumberId).phyEntityPtr->SetExtendedSchAccess(
        schChannelNumberId,
        numberExtendedAccessSlots);
}//SetExtendedSchAccess//

#pragma warning(disable:4355)

inline
WaveMac::PhyEntity::PhyEntity(
    const WaveMac& initWaveMac,
    const WaveMacPhyInput& initPhyInput,
    const vector<shared_ptr<Dot11Mac> > initMacPtrsPerCategory)
    :
    waveMac(initWaveMac),
    phyDeviceName(initPhyInput.phyDeviceName),
    phyPtr(initPhyInput.phyPtr),
    numberExtendedSchAccessSlots(0),
    nextChannelNumberIdInWaitingTransmissionSuspensionState(CHANNEL_NUMBER_UNKNOWN),
    timerEventPtr(new TimerEvent(this))
{
    set<ChannelNumberIndexType> assignedChannelNumberIds;

    assert(initMacPtrsPerCategory.size() == NUMBER_CHANNEL_CATEGORIES);

    vector<ChannelCategoryAndNumberIdType> availableCategoryAndNumberIds;

    for(ChannelCategoryType i = 0; i < NUMBER_CHANNEL_CATEGORIES; i++) {
        const vector<ChannelNumberIndexType>& channelNumberIds = initPhyInput.channelNumberIds[i];

        if (channelNumberIds.empty()) {
            continue;
        }//if//

        macPtrs[i] = initMacPtrsPerCategory.at(i);
        macPtrs[i]->SuspendTransmissionFunction();
        availableChannelNumberIds[i].insert(channelNumberIds.begin(), channelNumberIds.end());

        const size_t numberLastAssignedChannels = assignedChannelNumberIds.size();

        assignedChannelNumberIds.insert(channelNumberIds.begin(), channelNumberIds.end());

        if (assignedChannelNumberIds.size() != numberLastAssignedChannels + channelNumberIds.size()) {
            cerr << "Error: CCH and SCH channel number is conflicted." << endl;
            exit(1);
        }//if//

        channelNumberPerCategory[i] = channelNumberIds.front();
        availableCategoryAndNumberIds.push_back(make_pair(i, channelNumberIds.front()));
        alternatingChannelCategories.push_back(i);
    }//for//

    assert(!assignedChannelNumberIds.empty());
    assert(!availableCategoryAndNumberIds.empty());


    const SimTime currentTime = waveMac.simEngineInterfacePtr->CurrentTime();

    currentCategory = (*this).CalculateNonExtendedChannelCategoryAt(currentTime);

    const shared_ptr<Dot11Mac> macPtr = macPtrs[currentCategory];

    phyPtr->SetMacInterfaceForPhy(macPtr->CreateInterfaceForPhy());
    phyPtr->SwitchToChannelNumber(channelNumberPerCategory[currentCategory]);


    if (availableCategoryAndNumberIds.size() == 1) {
        const ChannelCategoryAndNumberIdType& aCategoryAndNumberId = availableCategoryAndNumberIds.front();

        (*this).SetContinuousAccess(aCategoryAndNumberId.second);

    } else {

        (*this).SetAlternatingAccess(availableCategoryAndNumberIds);
    }//if//

    (*this).ResumeTxRxOperationAndScheduleEndEventIfNecessary();
}//PhyEntity//

#pragma warning(default:4355)

inline
void WaveMac::PhyEntity::SetContinuousAccess(
    const ChannelNumberIndexType& channelNumberId)
{
    numberExtendedSchAccessSlots = 0;

    alternatingChannelCategories.clear();
    alternatingChannelCategories.push_back(
        (*this).GetChannelCategory(channelNumberId));

    (*this).SwitchToChannelIfNecessary(channelNumberId);
}//SetContinuousAccess//

inline
void WaveMac::PhyEntity::SetAlternatingAccess()
{
    vector<ChannelCategoryType> channelCategories;

    for(ChannelCategoryType channelCategory = CHANNEL_CATEGORY_CCH;
        channelCategory < NUMBER_CHANNEL_CATEGORIES;
        channelCategory++) {
        channelCategories.push_back(channelCategory);
    }//for//

    (*this).SetAlternatingAccess(channelCategories);
}//SetAlternatingAccess//
inline
void WaveMac::PhyEntity::SetAlternatingAccess(
    const vector<ChannelCategoryType>& channelCategories)
{
    assert(channelCategories.size() >= 2);

    vector<ChannelCategoryAndNumberIdType> categoryAndNumberIds;

    for(size_t i = 0; i < channelCategories.size(); i++) {
        const ChannelCategoryType& channelCategory = channelCategories[i];

        assert(!availableChannelNumberIds[channelCategory].empty());

        categoryAndNumberIds.push_back(
            make_pair(channelCategory,
                      channelNumberPerCategory[channelCategory]));
    }//for//

    (*this).SetAlternatingAccess(categoryAndNumberIds);
}//SetAlternatingAccess//
inline
void WaveMac::PhyEntity::SetAlternatingAccess(
    const vector<ChannelCategoryAndNumberIdType>& channelCategoryAndNumberIds)
{
    set<ChannelCategoryType> specifiedChannelCategories;

    for(size_t i = 0; i < channelCategoryAndNumberIds.size(); i++) {
        const ChannelCategoryAndNumberIdType& categoryAndNumberId = channelCategoryAndNumberIds[i];
        const ChannelCategoryType channelCategory = categoryAndNumberId.first;
        const ChannelNumberIndexType channelNumberId = categoryAndNumberId.second;

        assert(availableChannelNumberIds[channelCategory].find(channelNumberId) !=
               availableChannelNumberIds[channelCategory].end());

        assert(specifiedChannelCategories.find(channelCategory) == specifiedChannelCategories.end() &&
               "Duplicated channel accesses are specified for a channel category in alternaing access.");

        specifiedChannelCategories.insert(channelCategory);

        channelNumberPerCategory[channelCategory] = channelNumberId;
    }//for//

    alternatingChannelCategories.assign(specifiedChannelCategories.begin(), specifiedChannelCategories.end());

    // The change is applicable at the begining of next interval.
    (*this).ResumeTxRxOperationAndScheduleEndEventIfNecessary();
}//SetAlternatingAccess//

inline
void WaveMac::PhyEntity::SetImmediateSchAccess(
    const ChannelNumberIndexType& schChannelNumberId)
{
    // Immediate SCH access allosws immediate communcation access to the SCH without waiting for the next CCH.
    assert(alternatingChannelCategories.size() > 1 && "Immediate SCH access is valid on alternating access.");

    assert((*this).CurrentAlternatingChannelAccessSupports(CHANNEL_CATEGORY_SCH));

    assert(availableChannelNumberIds[CHANNEL_CATEGORY_SCH].find(schChannelNumberId) !=
           availableChannelNumberIds[CHANNEL_CATEGORY_SCH].end() &&
           "Specified channel number is not SCH or the phy doesn't support this channel.");

    channelNumberPerCategory[CHANNEL_CATEGORY_SCH] = schChannelNumberId;

    (*this).SwitchToChannelIfNecessary(schChannelNumberId);
}//SetImmediateSchAccess//

inline
void WaveMac::PhyEntity::SetExtendedSchAccess(
    const ChannelNumberIndexType& schChannelNumberId,
    const size_t initNumberExtendedAccessSlots)
{
    assert(availableChannelNumberIds[CHANNEL_CATEGORY_SCH].find(schChannelNumberId) !=
           availableChannelNumberIds[CHANNEL_CATEGORY_SCH].end() &&
           "Specified channel number is not SCH or the phy doesn't support this channel.");

    numberExtendedSchAccessSlots = initNumberExtendedAccessSlots;
    channelNumberPerCategory[CHANNEL_CATEGORY_SCH] = schChannelNumberId;

    const SimTime currentTime = waveMac.simEngineInterfacePtr->CurrentTime();

    if ((*this).CalculateNonExtendedChannelCategoryAt(currentTime) == CHANNEL_CATEGORY_SCH) {

        (*this).SwitchToChannelIfNecessary(schChannelNumberId);

    } else {

        (*this).ResumeTxRxOperationAndScheduleEndEventIfNecessary();
    }//if//
}//SetExtendedSchAccess//

inline
bool WaveMac::PhyEntity::CurrentChannelOrNextSwitchingChannelIs(
    const ChannelNumberIndexType& channelNumberId) const
{
    return (channelNumberPerCategory[currentCategory] == channelNumberId);
}//CurrentChannelOrNextSwitchingChannelIs//

inline
SimTime WaveMac::PhyEntity::CalculateCurrentTxRxOperationEndTime() const
{
    assert(alternatingChannelCategories.size() >= 1);

    const SimTime currentTime = waveMac.simEngineInterfacePtr->CurrentTime();
    SimTime syncInterval = ZERO_TIME;

    for(size_t i = 0; i < alternatingChannelCategories.size(); i++) {
        syncInterval += waveMac.channelIntervals[alternatingChannelCategories[i]];
    }//for//

    SimTime timeFromStart = (currentTime % syncInterval);
    ChannelCategoryType channelCategory = NUMBER_CHANNEL_CATEGORIES;
    SimTime operationEndTime = currentTime;

    for(size_t i = 0; i < alternatingChannelCategories.size(); i++) {

        channelCategory = alternatingChannelCategories[i];
        const SimTime channelInterval = waveMac.channelIntervals[channelCategory];

        if (timeFromStart < channelInterval) {
            operationEndTime += (channelInterval - timeFromStart);
            break;
        }//if//
        timeFromStart -= channelInterval;
    }//for//

    assert(channelCategory != NUMBER_CHANNEL_CATEGORIES);

    if (channelCategory == CHANNEL_CATEGORY_SCH) {

        for(size_t i = 0; i < numberExtendedSchAccessSlots; i++) {

            for(size_t j = 0; j < alternatingChannelCategories.size(); j++) {
                const ChannelCategoryType alternatingChannelCategory = alternatingChannelCategories[j];

                operationEndTime += waveMac.channelIntervals[alternatingChannelCategory];
            }//for//
        }//for//
    }//if//

    return operationEndTime;
}//CalculateCurrentTxRxOperationEndTime//

inline
ChannelCategoryType WaveMac::PhyEntity::CalculateNonExtendedChannelCategoryAt(const SimTime time) const
{
    assert(!alternatingChannelCategories.empty());

    if (alternatingChannelCategories.size() == 1) {
        if (numberExtendedSchAccessSlots == 0) {
            return alternatingChannelCategories.front();
        } else {
            return CHANNEL_CATEGORY_SCH;
        }//if//
    }//if//

    SimTime syncInterval = ZERO_TIME;

    for(size_t i = 0; i < alternatingChannelCategories.size(); i++) {
        syncInterval += waveMac.channelIntervals[alternatingChannelCategories[i]];
    }//for//

    SimTime timeFromStart = (time % syncInterval);

    for(size_t i = 0; i < alternatingChannelCategories.size(); i++) {
        const ChannelCategoryType channelCategory = alternatingChannelCategories[i];
        const SimTime channelInterval = waveMac.channelIntervals[channelCategory];

        if (timeFromStart < channelInterval) {
            return channelCategory;
        }//if//
        timeFromStart -= channelInterval;
    }//for//

    assert(false);
    return CHANNEL_CATEGORY_CCH;
}//CalculateNonExtendedChannelCategoryAt//

inline
void WaveMac::PhyEntity::SwitchToCurrentIntervalChannel()
{
    const SimTime currentTime = waveMac.simEngineInterfacePtr->CurrentTime();
    const ChannelCategoryType channdlCategory =
        (*this).CalculateNonExtendedChannelCategoryAt(currentTime);

    (*this).SwitchToChannelIfNecessary(channelNumberPerCategory[channdlCategory]);
}//SwitchToCurrentIntervalChannel//

inline
void WaveMac::PhyEntity::SwitchToChannelIfNecessary(
    const ChannelNumberIndexType& channelNumberId)
{
    if ((*this).CurrentChannelOrNextSwitchingChannelIs(channelNumberId)) {
        (*this).ResumeTxRxOperationAndScheduleEndEventIfNecessary();
        return;
    }//if//

    (*this).SuspendTransmissionAndRescheduleChannelSwitchingEvent(channelNumberId);
}//SwitchToChannelIfNecessary//

inline
void WaveMac::PhyEntity::TimerEvent::ExecuteEvent()
{
    phyEntityPtr->timerEventTicket.Clear();

    switch(event) {
    case EVENT_TX_RX_START:
        phyEntityPtr->ResumeTxRxOperationAndScheduleEndEventIfNecessary();
        break;

    case EVENT_TX_RX_END:
        phyEntityPtr->SwitchToCurrentIntervalChannel();
        break;

    case EVENT_SUSPEND_TRANSMISSION:
        phyEntityPtr->SuspendTransmissionAndRescheduleChannelSwitchingEvent(
            phyEntityPtr->nextChannelNumberIdInWaitingTransmissionSuspensionState);
        break;

    case EVENT_CHANNEL_SWITCH:
        phyEntityPtr->StartChannelSwitchngTime();
        break;

    default:
        assert(false);
        break;
    }//switch//
}//ExecuteEvent//

inline
void WaveMac::PhyEntity::ResumeTxRxOperationAndScheduleEndEventIfNecessary()
{
    const shared_ptr<Dot11Mac> macPtr = macPtrs[currentCategory];

    if (alternatingChannelCategories.size() == 1 &&
        numberExtendedSchAccessSlots == 0) {

        if (!timerEventTicket.IsNull()) {
            waveMac.simEngineInterfacePtr->CancelEvent(timerEventTicket);
        }//if//

        macPtr->ResumeTransmissionFunction(INFINITE_TIME);
        return;
    }//if//

    const SimTime txRxOperationEndTime = (*this).CalculateCurrentTxRxOperationEndTime();

    (*this).RescheduleTimerEvent(EVENT_TX_RX_END, txRxOperationEndTime);

    macPtr->ResumeTransmissionFunction(txRxOperationEndTime);

    if (currentCategory == CHANNEL_CATEGORY_SCH) {
        numberExtendedSchAccessSlots = 0;
    }//if//
}//ResumeTxRxOperationAndScheduleEndEventIfNecessary//

inline
void WaveMac::PhyEntity::SuspendTransmissionAndRescheduleChannelSwitchingEvent(
    const ChannelNumberIndexType& channelNumberId)
{
    const SimTime currentTime = waveMac.simEngineInterfacePtr->CurrentTime();
    const SimTime lastTransmissionEndTime = phyPtr->GetOutgoingTransmissionEndTime();

    if (currentTime < lastTransmissionEndTime) {

        nextChannelNumberIdInWaitingTransmissionSuspensionState = channelNumberId;

        (*this).RescheduleTimerEvent(
            EVENT_SUSPEND_TRANSMISSION,
            lastTransmissionEndTime + 1 * NANO_SECOND);

    } else {

        assert(!phyPtr->IsTransmittingAFrame());

        const ChannelNumberIndexType prevChannelNumberId = channelNumberPerCategory[currentCategory];
        const shared_ptr<Dot11Mac> macPtr = macPtrs[currentCategory];

        macPtr->SuspendTransmissionFunction();

        assert(prevChannelNumberId != channelNumberId &&
               "No need to swtich channel!");

        currentCategory = (*this).GetChannelCategory(channelNumberId);
        channelNumberPerCategory[currentCategory] = channelNumberId;

        (*this).RescheduleTimerEvent(
            EVENT_CHANNEL_SWITCH,
            currentTime + waveMac.syncTolerance/2);

        (*this).OutputTraceAndStatsForNewChannelInterval(prevChannelNumberId, channelNumberId);
    }//if//
}//SuspendTransmissionAndRescheduleChannelSwitchingEvent//

inline
void WaveMac::PhyEntity::StartChannelSwitchngTime()
{
    const shared_ptr<Dot11Mac> macPtr = macPtrs[currentCategory];

    (*this).OutputTraceAndStatsForChannelChange(channelNumberPerCategory[currentCategory]);

    // receive only
    phyPtr->SetMacInterfaceForPhy(macPtr->CreateInterfaceForPhy());
    phyPtr->SwitchToChannelNumber(channelNumberPerCategory[currentCategory]);

    const SimTime currentTime = waveMac.simEngineInterfacePtr->CurrentTime();

    (*this).RescheduleTimerEvent(
        EVENT_TX_RX_START,
        currentTime + waveMac.maxChannelSwitchingTime + waveMac.syncTolerance/2);
}//StartChannelSwitchngTime//

inline
void WaveMac::PhyEntity::RescheduleTimerEvent(
    const EventType& event, const SimTime time)
{
    timerEventPtr->SetNextEventType(event);

    if (timerEventTicket.IsNull()) {
        waveMac.simEngineInterfacePtr->ScheduleEvent(
            timerEventPtr, time, timerEventTicket);
    } else {
        waveMac.simEngineInterfacePtr->RescheduleEvent(
            timerEventTicket, time);
    }//if//
}//RescheduleTimerEvent//

inline
ChannelCategoryType WaveMac::PhyEntity::GetChannelCategory(const ChannelNumberIndexType& channelNumberId) const
{
    ChannelCategoryType channelCategory;

    for(channelCategory = CHANNEL_CATEGORY_CCH;
        channelCategory < NUMBER_CHANNEL_CATEGORIES;
        channelCategory++) {

        const set<ChannelNumberIndexType>& channelNumberIds = availableChannelNumberIds[channelCategory];

        if (channelNumberIds.find(channelNumberId) != channelNumberIds.end()) {
            break;
        }//if//
    }//for//

    if (channelCategory >= NUMBER_CHANNEL_CATEGORIES) {
        cerr << "Error: " << waveMac.simEngineInterfacePtr->GetNodeId() << ":"
             << phyDeviceName << " doesnt't support channel " << int(channelNumberId) << endl;
        exit(1);
    }//if

    return channelCategory;
}//GetChannelCategory//

inline
bool WaveMac::PhyEntity::CurrentAlternatingChannelAccessSupports(const ChannelCategoryType& channelCategory) const
{
    for(size_t i = 0; i < alternatingChannelCategories.size(); i++) {
        if (alternatingChannelCategories[i] == channelCategory) {
            return true;
        }//if//
    }//for//

    return false;
}//CurrentAlternatingChannelAccessSupports//

inline
void WaveMac::PhyEntity::OutputTraceAndStatsForNewChannelInterval(
    const ChannelNumberIndexType& prevChannelNumberId,
    const ChannelNumberIndexType& nextChannelNumberId) const
{
    if (waveMac.simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (waveMac.simEngineInterfacePtr->BinaryOutputIsOn()) {

            WaveNewChIntervalTraceRecord traceData;

            const ChannelCategoryType prevChannelCategory = (*this).GetChannelCategory(prevChannelNumberId);
            const ChannelCategoryType nextChannelCategory = (*this).GetChannelCategory(nextChannelNumberId);

            traceData.previousChannelCategory = prevChannelCategory;
            traceData.previousChannelNumber = ConvertToChannelNumber(prevChannelNumberId);
            traceData.nextChannelCategory = nextChannelCategory;
            traceData.nextChannelNumber = ConvertToChannelNumber(nextChannelNumberId);

            assert(sizeof(traceData) == WAVE_NEW_CH_INTERVAL_TRACE_RECORD_BYTES);

            waveMac.simEngineInterfacePtr->OutputTraceInBinary(modelName, phyDeviceName, "NewChInterval",  traceData);
        }
        else {

            const ChannelCategoryType prevChannelCategory = (*this).GetChannelCategory(prevChannelNumberId);
            const ChannelCategoryType nextChannelCategory = (*this).GetChannelCategory(nextChannelNumberId);

            ostringstream msgStream;

            msgStream << "Prev= "
                      << CHANNEL_CATEGORY_NAMES[prevChannelCategory]
                      << ConvertChannelNumberIdToString(prevChannelNumberId)
                      << " Next= "
                      << CHANNEL_CATEGORY_NAMES[nextChannelCategory]
                      << ConvertChannelNumberIdToString(nextChannelNumberId);

            waveMac.simEngineInterfacePtr->OutputTrace(modelName, phyDeviceName, "NewChInterval", msgStream.str());
        }//if//
    }//if//
}//OutputTraceAndStatsForNewChannelInterval//

inline
void WaveMac::PhyEntity::OutputTraceAndStatsForChannelChange(
    const ChannelNumberIndexType& channelNumberId) const
{
    if (waveMac.simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (waveMac.simEngineInterfacePtr->BinaryOutputIsOn()) {

            WavePhyChSwitchTraceRecord traceData;

            const ChannelCategoryType channelCategory = (*this).GetChannelCategory(channelNumberId);

            traceData.channelCategory = channelCategory;
            traceData.channelNumber = ConvertToChannelNumber(channelNumberId);

            assert(sizeof(traceData) == WAVE_PHY_CH_SWITCH_TRACE_RECORD_BYTES);

            waveMac.simEngineInterfacePtr->OutputTraceInBinary(modelName, phyDeviceName, "PhyChSwitch",  traceData);
        }
        else {

            const ChannelCategoryType channelCategory = (*this).GetChannelCategory(channelNumberId);
            ostringstream msgStream;

            msgStream << "Ch= "
                      << CHANNEL_CATEGORY_NAMES[channelCategory]
                      << ConvertChannelNumberIdToString(channelNumberId);

            waveMac.simEngineInterfacePtr->OutputTrace(modelName, phyDeviceName, "PhyChSwitch", msgStream.str());
        }//if//
    }//if//
}//OutputTraceAndStatsForChannelChange//

} //namespae Wave//

#endif
