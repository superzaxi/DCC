// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_APP_FLOODING_H
#define SCENSIM_APP_FLOODING_H

#include "scensim_netsim.h"
//#include "D:\rei_yamazaki\mos-its-group\DCC\scenargie_simulator\2.2\source\dot11/dot11_phy.h"//20210519
#include "dot11_mac.h"

#include <stdio.h>
#include <iostream>//20210519
#include <vector>//20210604
#include <map>//20210604

//dcc
/*static int status = 0;
static void checkCBR(double a){
    //cbr = a;
    if(a <= 30){
        status = 1;
    }else if(a <= 60){
        status = 2;
    }else{
        status = 3;
    }
}
static void check2(){
    cout << "check: " << status << endl;
}*/
//20210524

namespace ScenSim {

using std::cout;//20210519
using std::cerr;
using std::endl;
using std::map;

//using namespace Dot11;


//--------------------------------------------------------------------------------------------------


//dcc
struct CPMInfoType {

    double positionX, positionY;
    SimTime CPMTime;

    // Warning: Records are owned by "fifoQueue" and raw pointers are used in
    // "destinationSpecificInfos"

    CPMInfoType() 
    : 
    positionX(0.0),
    positionY(0.0),
    CPMTime(0)
        { }

    void operator=(CPMInfoType&& right) {
        positionX = right.positionX;
        positionY = right.positionY;
        CPMTime = right.CPMTime;
    }

    CPMInfoType(CPMInfoType&& right) { (*this) = move(right); }

};//CPMInfoType//

map<NodeId, CPMInfoType> CPMInfo;
long int sendCAMCount = 0, stopCAMCount = 0, sendCPMCount = 0, stopCPMCount = 0;

class FloodingApplication: public Application, public enable_shared_from_this<FloodingApplication> {
public:
    static const string modelName;
    static const int APPLICATION_ID_CHAR_MAX_LENGTH = 16;
    typedef char ApplicationIdCharType[APPLICATION_ID_CHAR_MAX_LENGTH];

    //dcc
    std::multimap <SimTime, long int> packetSizeforDCC;
    //vector<unique_ptr<Packet>> stockPacket;
    SimTime Toff = 0;
    //SimTime DCCduration_before = 488000;
    SimTime currentTime_before = 0;
    double delta = 0; 
    int status = 0;
    int adaptiveCount = 0;
    double averageChannelBusyRatio = 0;
    double CBR_L_0_Hop_Previous = 0;
    SimTime get_duration();
    SimTime calcReactiveSendRate();
    double calcAdaptiveSendRate();

    int distanceCount = 0;
    double maxDistance;
    
    struct PastCPMInfoType {

        double pastPositionX, pastPositionY;
        SimTime pastCPMTime;

        // Warning: Records are owned by "fifoQueue" and raw pointers are used in
        // "destinationSpecificInfos"

        PastCPMInfoType() 
        : 
        pastPositionX(0.0),
        pastPositionY(0.0),
        pastCPMTime(0)
            { }

        void operator=(PastCPMInfoType&& right) {
            pastPositionX = right.pastPositionX;
            pastPositionY = right.pastPositionY;
            pastCPMTime = right.pastCPMTime;
        }

        PastCPMInfoType(PastCPMInfoType&& right) { (*this) = move(right); }

    };//PastCPMInfoType//

    map<NodeId, PastCPMInfoType> pastCPMInfo;

    FloodingApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationId& initApplicationId,
        const NodeId& initNodeId,
        const unsigned short int initDefaultApplicationPortId);

    void CompleteInitialization();

    void DisconnectFromOtherLayers();

    void AddSenderSetting(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationId& initApplicationId);

    void AddReceiverSetting(
        const ApplicationId& theApplicationId);

    virtual ~FloodingApplication() {}

    struct FloodingPayloadIdType {
        ApplicationIdCharType applicationIdChar;
        NodeId sourceNodeId;
        unsigned int sequenceNumber;

        FloodingPayloadIdType(
            const ApplicationId& initApplicationId,
            const NodeId& initSourceNodeId,
            const unsigned int initSequenceNumber)
            :
            sourceNodeId(initSourceNodeId),
            sequenceNumber(initSequenceNumber)
        {
            if (initApplicationId.size() >= APPLICATION_ID_CHAR_MAX_LENGTH) {
                cerr << "Error: A length of application ID ("
                    << initApplicationId << ") should be less than "
                    << APPLICATION_ID_CHAR_MAX_LENGTH << ")." << endl;
                exit(1);
            }//if//

            std::fill_n(applicationIdChar, APPLICATION_ID_CHAR_MAX_LENGTH, 0);
            initApplicationId.copy(applicationIdChar, APPLICATION_ID_CHAR_MAX_LENGTH - 1, 0);

        }//FloodingPayloadIdType//

        bool operator<(const FloodingPayloadIdType& right) const
        {
            return ((strcmp(applicationIdChar, right.applicationIdChar) < 0) ||
                    ((strcmp(applicationIdChar, right.applicationIdChar) == 0) &&
                     (sourceNodeId < right.sourceNodeId)) ||
                    ((strcmp(applicationIdChar, right.applicationIdChar) == 0) &&
                     (sourceNodeId == right.sourceNodeId) &&
                     (sequenceNumber < right.sequenceNumber)));
        }
    };//FloodingPayloadIdType//

    struct FloodingPayloadType {
        FloodingPayloadIdType id;
        SimTime broadcastTime;
        double nodePositionX;
        double nodePositionY;
        PacketPriority floodingPriority;
        unsigned int maxHopCount;
        unsigned int hopCount;
        SimTime minWaitingPeriod;
        SimTime maxWaitingPeriod;
        unsigned int counterThreshold;
        double distanceThresholdInMeters;

        FloodingPayloadType(
            const ApplicationId initApplicationId,
            const NodeId initSourceNodeId,
            const unsigned int initSequenceNumber,
            const SimTime initBroadcastTime,
            const double initNodePositionX,
            const double initNodePositionY,
            const PacketPriority initFloodingPriority,
            const unsigned int initMaxHopCount,
            const unsigned int initHopCount,
            const SimTime initMinWaitingPeriod,
            const SimTime initMaxWaitingPeriod,
            const unsigned int initCounterThreshold,
            const double initDistanceThresholdInMeters)
            :
            id(initApplicationId, initSourceNodeId, initSequenceNumber),
            broadcastTime(initBroadcastTime),
            nodePositionX(initNodePositionX),
            nodePositionY(initNodePositionY),
            floodingPriority(initFloodingPriority),
            maxHopCount(initMaxHopCount),
            hopCount(initHopCount),
            minWaitingPeriod(initMinWaitingPeriod),
            maxWaitingPeriod(initMaxWaitingPeriod),
            counterThreshold(initCounterThreshold),
            distanceThresholdInMeters(initDistanceThresholdInMeters)
        {}
    };//FloodingPayloadType//

private:

    class FloodingStartEvent: public SimulationEvent {
    public:
        explicit
        FloodingStartEvent(
            const shared_ptr<FloodingApplication>& initFloodingApplicationPtr,
            const ApplicationId& initApplicationId)
            :
            floodingApplicationPtr(initFloodingApplicationPtr),
            theApplicationId(initApplicationId)
        {}

        virtual void ExecuteEvent()
        {
            floodingApplicationPtr->Broadcast(theApplicationId);
            //cout << "A" << endl;

        }//ExecuteEvent//

    private:
        shared_ptr<FloodingApplication> floodingApplicationPtr;
        ApplicationId theApplicationId;

    };//FloodingStartEvent//

    class FloodingRebroadcastEvent: public SimulationEvent {
    public:
        explicit
        FloodingRebroadcastEvent(
            const shared_ptr<FloodingApplication>& initFloodingApplicationPtr,
            const FloodingPayloadType& initFloodingPayload,
            const unsigned int initPacketPayloadSizeByte)
            :
            floodingApplicationPtr(initFloodingApplicationPtr),
            floodingPayload(
                string(initFloodingPayload.id.applicationIdChar),
                initFloodingPayload.id.sourceNodeId,
                initFloodingPayload.id.sequenceNumber,
                initFloodingPayload.broadcastTime,
                initFloodingPayload.nodePositionX,
                initFloodingPayload.nodePositionY,
                initFloodingPayload.floodingPriority,
                initFloodingPayload.maxHopCount,
                initFloodingPayload.hopCount,
                initFloodingPayload.minWaitingPeriod,
                initFloodingPayload.maxWaitingPeriod,
                initFloodingPayload.counterThreshold,
                initFloodingPayload.distanceThresholdInMeters),
            packetPayloadSizeBytes(initPacketPayloadSizeByte)
        {}

        virtual void ExecuteEvent()
        {
            floodingApplicationPtr->RebroadcastIfNecessary(
                floodingPayload,
                packetPayloadSizeBytes);

        }//ExecuteEvent//

    private:
        shared_ptr<FloodingApplication> floodingApplicationPtr;
        FloodingPayloadType floodingPayload;
        unsigned int packetPayloadSizeBytes;

    };//FloodingRebroadcastEvent//

    class PacketHandler: public UdpProtocol::PacketForAppFromTransportLayerHandler {
    public:
        PacketHandler(
            const shared_ptr<FloodingApplication>& initFloodingApplicationPtr)
            :
            floodingApplicationPtr(initFloodingApplicationPtr)
        {}

        void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const NetworkAddress& sourceAddress,
            const unsigned short int sourcePort,
            const NetworkAddress& destinationAddress,
            const PacketPriority& priority)
        {
            floodingApplicationPtr->ReceivePacket(packetPtr);

        }//ReceivePacket//

    private:
        shared_ptr<FloodingApplication> floodingApplicationPtr;

    };//PacketHandler//

    struct FloodingSenderSettingType {
        int packetPayloadSizeBytes;
        SimTime packetInterval;
        SimTime floodingEndTime;
        PacketPriority floodingPriority;
        unsigned int maxHopCount;
        SimTime minWaitingPeriod;
        SimTime maxWaitingPeriod;
        unsigned int counterThreshold;
        double distanceThresholdInMeters;
        unsigned int sequenceNumber;

        FloodingSenderSettingType(
            const int initPacketPayloadSizeBytes,
            const SimTime initPacketInterval,
            const SimTime initFloodingEndTime,
            const PacketPriority initFloodingPriority,
            const unsigned int initMaxHopCount,
            const SimTime initMinWaitingPeriod,
            const SimTime initMaxWaitingPeriod,
            const unsigned int initCounterThreshold,
            const double initDistanceThresholdInMeters)
            :
            packetPayloadSizeBytes(initPacketPayloadSizeBytes),
            packetInterval(initPacketInterval),
            floodingEndTime(initFloodingEndTime),
            floodingPriority(initFloodingPriority),
            maxHopCount(initMaxHopCount),
            minWaitingPeriod(initMinWaitingPeriod),
            maxWaitingPeriod(initMaxWaitingPeriod),
            counterThreshold(initCounterThreshold),
            distanceThresholdInMeters(initDistanceThresholdInMeters),
            sequenceNumber(0)
        {}
    };//FloodingSenderSettingType//

    struct FloodingSenderStatType {
        shared_ptr<CounterStatistic> packetsBroadcastStatPtr;
        shared_ptr<CounterStatistic> bytesBroadcastStatPtr;

        FloodingSenderStatType(
            const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
            const string& initModelName,
            const ApplicationId initApplicationId)
            :
            packetsBroadcastStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName + "_" + initApplicationId + "_PacketsBroadcast"))),
            bytesBroadcastStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName + "_" + initApplicationId + "_BytesBroadcast")))
        {}
    };//FloodingSenderStatType//

    struct FloodingReceiverStatType {
        shared_ptr<CounterStatistic> packetsRebroadcastStatPtr;
        shared_ptr<CounterStatistic> bytesRebroadcastStatPtr;
        shared_ptr<CounterStatistic> packetsReceivedStatPtr;
        shared_ptr<CounterStatistic> bytesReceivedStatPtr;
        shared_ptr<CounterStatistic> packetsDiscardedStatPtr;
        shared_ptr<CounterStatistic> bytesDiscardedStatPtr;
        shared_ptr<RealStatistic> endToEndDelayStatPtr;
        shared_ptr<RealStatistic> hopCountStatPtr;
        unsigned int countOfOriginalPacketReceived;

        FloodingReceiverStatType(
            const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
            const string& initModelName,
            const ApplicationId initApplicationId)
            :
            packetsRebroadcastStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName + "_" + initApplicationId + "_PacketsRebroadcast"))),
            bytesRebroadcastStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName + "_" + initApplicationId + "_BytesRebroadcast"))),
            packetsReceivedStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName + "_" + initApplicationId + "_PacketsReceived"))),
            bytesReceivedStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName + "_" + initApplicationId + "_BytesReceived"))),
            packetsDiscardedStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName +"_" +  initApplicationId + "_PacketsDiscarded"))),
            bytesDiscardedStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName + "_" + initApplicationId + "_BytesDiscarded"))),
            endToEndDelayStatPtr(
                initSimulationEngineInterfacePtr->CreateRealStat(
                    (initModelName + "_" + initApplicationId + "_EndToEndDelay"))),
            hopCountStatPtr(
                initSimulationEngineInterfacePtr->CreateRealStat(
                    (initModelName + "_" + initApplicationId + "_HopCount"))),
            countOfOriginalPacketReceived(0)
        {}
    };//FloodingReceiverStatType//

    void Broadcast(const ApplicationId& originatorApplicationId);

    void Rebroadcast(
        FloodingPayloadType& floodingPayload,
        const unsigned int packetPayloadSizeBytes);

    void RebroadcastIfNecessary(
        FloodingPayloadType& floodingPayload,
        const unsigned int packetPayloadSizeBytes);

    void ScheduleRebroadcastEvent(
        const FloodingPayloadType& floodingPayload,
        const unsigned int packetPayloadSizeBytes);

    void ReceivePacket(unique_ptr<Packet>& packetPtr);

    void IncrementCountOfPacketReceived(const FloodingPayloadIdType& id);
    void UpdateMinDistanceBetweenNodesInMeters(const FloodingPayloadType& floodingPayload);

    double CalculateDistanceBetweenPointsInMeters(
        const double& x1, const double& y1,
        const double& x2, const double& y2) const;

    bool IsSenderMyself(const FloodingPayloadIdType& id) const;
    bool IsPacketFirstlyReceived(const FloodingPayloadIdType& id) const;
    bool IsPacketReceived(const FloodingPayloadType& floodingPayload) const;
    bool IsMaxHopCountReached(const FloodingPayloadType& floodingPayload) const;
    bool IsCounterThresholdReached(const FloodingPayloadType& floodingPayload) const;
    bool IsLessThanDistanceThresholdInMeters(const FloodingPayloadType& floodingPayload) const;

    void OutputTraceAndStatsForBroadcast(
        const ApplicationId& applicatoinId,
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const unsigned int packetLengthBytes);

    void OutputTraceAndStatsForRebroadcast(
        const ApplicationId& applicatoinId,
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const unsigned int packetLengthBytes);

    void OutputTraceAndStatsForReceivePacket(
        const ApplicationId& applicatoinId,
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const unsigned int packetLengthBytes,
        const unsigned int hopCount,
        const SimTime& delay);

    void OutputTraceAndStatsForDiscardPacket(
        const ApplicationId& applicatoinId,
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const unsigned int packetLengthBytes,
        const SimTime& delay);

    NodeId theNodeId;
    unsigned short int destinationPortId;
    shared_ptr<PacketHandler> packetHandlerPtr;
    map<ApplicationId, FloodingSenderSettingType> floodingSenderSettings;
    map<ApplicationId, FloodingSenderStatType> floodingSenderStats;
    map<ApplicationId, FloodingReceiverStatType> floodingReceiverStats;
    map<FloodingPayloadIdType, unsigned int> countOfPacketReceived;
    map<FloodingPayloadIdType, double> minDistanceBetweenNodesInMeters;
    bool useVirtualPayload;

};//FloodingApplication//


inline
FloodingApplication::FloodingApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationId& initApplicationId,
    const NodeId& initNodeId,
    const unsigned short int initDefaultApplicationPortId)
    :
    Application(initSimulationEngineInterfacePtr, modelName),
    theNodeId(initNodeId),
    destinationPortId(initDefaultApplicationPortId),
    packetHandlerPtr(),
    floodingSenderSettings(),
    floodingSenderStats(),
    floodingReceiverStats(),
    countOfPacketReceived(),
    minDistanceBetweenNodesInMeters(),
    useVirtualPayload(false)
{
    if (parameterDatabaseReader.ParameterExists(
            "flooding-auto-port-mode", theNodeId, initApplicationId)) {

        if (!parameterDatabaseReader.ReadBool(
                "flooding-auto-port-mode", theNodeId, initApplicationId)) {

            destinationPortId = static_cast<unsigned short int>(
                parameterDatabaseReader.ReadNonNegativeInt(
                    "flooding-destination-port", theNodeId, initApplicationId));
        }//if//
    }//if//

    if (parameterDatabaseReader.ParameterExists(
        "flooding-use-virtual-payload", theNodeId, initApplicationId)) {

        useVirtualPayload = parameterDatabaseReader.ReadBool(
            "flooding-use-virtual-payload", theNodeId, initApplicationId);
    }//if//

}//FloodingApplication//


inline
void FloodingApplication::CompleteInitialization()
{
    packetHandlerPtr = shared_ptr<PacketHandler>(new PacketHandler(shared_from_this()));

    assert(transportLayerPtr->udpPtr->PortIsAvailable(destinationPortId));

    transportLayerPtr->udpPtr->OpenSpecificUdpPort(
        NetworkAddress::anyAddress,
        destinationPortId,
        packetHandlerPtr);

}//CompleteInitialization//


inline
void FloodingApplication::DisconnectFromOtherLayers()
{
    packetHandlerPtr.reset();
    (*this).Application::DisconnectFromOtherLayers();

}//DisconnectFromOtherLayers//


inline
void FloodingApplication::AddSenderSetting(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationId& initApplicationId)
{
    theApplicationId = initApplicationId;

    const int packetPayloadSizeBytes =
        theParameterDatabaseReader.ReadInt("flooding-payload-size-bytes", theNodeId, theApplicationId);

    const SimTime packetInterval =
        theParameterDatabaseReader.ReadTime("flooding-interval", theNodeId, theApplicationId);

    const SimTime floodingStartTime =
        theParameterDatabaseReader.ReadTime("flooding-start-time", theNodeId, theApplicationId);

    const SimTime floodingEndTime =
        theParameterDatabaseReader.ReadTime("flooding-end-time", theNodeId, theApplicationId);

    const PacketPriority floodingPriority = static_cast<PacketPriority>(
        theParameterDatabaseReader.ReadNonNegativeInt("flooding-priority", theNodeId, theApplicationId));

    const unsigned int maxHopCount =
        theParameterDatabaseReader.ReadNonNegativeInt("flooding-max-hop-count", theNodeId, theApplicationId);

    const SimTime minWaitingPeriod =
        theParameterDatabaseReader.ReadTime("flooding-min-waiting-period", theNodeId, theApplicationId);

    const SimTime maxWaitingPeriod =
        theParameterDatabaseReader.ReadTime("flooding-max-waiting-period", theNodeId, theApplicationId);

    const unsigned int counterThreshold =
        theParameterDatabaseReader.ReadNonNegativeInt("flooding-counter-threshold", theNodeId, theApplicationId);

    const double distanceThresholdInMeters =
        theParameterDatabaseReader.ReadDouble("flooding-distance-threshold-in-meters", theNodeId, theApplicationId);

    if (packetPayloadSizeBytes < (int)sizeof(FloodingPayloadType)) {
        cerr << "Error: Packet payload size ("
            << packetPayloadSizeBytes << ") should be "
            << sizeof(FloodingPayloadType) << " bytes or larger." << endl;
        exit(1);
    }//if//

    if (packetInterval <= ZERO_TIME) {
        cerr << "Error: Broadcast interval ("
            << minWaitingPeriod << ") should be larger than "
            << ZERO_TIME << "." << endl;
        exit(1);
    }//if//

    if (floodingStartTime < ZERO_TIME) {
        cerr << "Error: Start time ("
            << floodingStartTime << ") should be "
            << ZERO_TIME << " or larger." << endl;
        exit(1);
    }//if//

    if (floodingStartTime >= floodingEndTime) {
        cerr << "Error: End time ("
            << floodingEndTime << ") should be larger than start time ("
            << floodingStartTime << ")." << endl;
        exit(1);
    }//if//

    if (minWaitingPeriod < ZERO_TIME) {
        cerr << "Error: Min waiting period ("
            << minWaitingPeriod << ") should be "
            << ZERO_TIME << " or larger." << endl;
        exit(1);
    }//if//

    if (minWaitingPeriod > maxWaitingPeriod) {
        cerr << "Error: Max waiting period ("
            << maxWaitingPeriod << ") should be min waiting period ("
            << minWaitingPeriod << ") or larger." << endl;
        exit(1);
    }//if//

    if (distanceThresholdInMeters < 0) {
        cerr << "Error: Distance threshold ("
            << distanceThresholdInMeters << ") should be 0 meters or larger." << endl;
        exit(1);
    }//if//
    
    FloodingSenderSettingType floodingSenderSetting(
        packetPayloadSizeBytes,
        packetInterval,
        floodingEndTime,
        floodingPriority,
        maxHopCount,
        minWaitingPeriod,
        maxWaitingPeriod,
        counterThreshold,
        distanceThresholdInMeters);

    floodingSenderSettings.insert(make_pair(theApplicationId, floodingSenderSetting));

    FloodingSenderStatType floodingSenderStat(
        simulationEngineInterfacePtr,
        modelName,
        theApplicationId);

    floodingSenderStats.insert(make_pair(theApplicationId, floodingSenderStat));

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    SimTime startTime = floodingStartTime;

    if (theParameterDatabaseReader.ParameterExists(
            "flooding-start-time-max-jitter", theNodeId, theApplicationId)) {

        const SimTime maxStartTimeJitter =
            theParameterDatabaseReader.ReadTime(
                "flooding-start-time-max-jitter", theNodeId, theApplicationId);

        startTime += static_cast<SimTime>(
            aRandomNumberGeneratorPtr->GenerateRandomDouble() * maxStartTimeJitter);
    }//if//

    if (currentTime > startTime) {
        const size_t nextTransmissionTime =
            size_t(ceil(double(currentTime - floodingStartTime) / packetInterval));
        startTime += nextTransmissionTime * packetInterval;
    }//if//

    if (startTime < floodingEndTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new FloodingStartEvent(shared_from_this(), theApplicationId)),
            startTime);
    }//if//

}//AddSenderSetting//


inline
void FloodingApplication::AddReceiverSetting(
    const ApplicationId& senderApplicationId)
{
    FloodingReceiverStatType floodingReceiverStat(
        simulationEngineInterfacePtr,
        modelName,
        senderApplicationId);

    floodingReceiverStats.insert(
        make_pair(senderApplicationId, floodingReceiverStat));

}//AddReceiverSetting//

//dcc
inline
SimTime FloodingApplication::get_duration(){
    SimTime calcDuration = 0;
    double calcDelta;
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
    const long long int dataRate = 6000000;
    const long long int calcInterval = 100000000;
    long long int total_packet_size = 0;
    //reactive
    //Toff = calcReactiveSendRate();
    //reactive

    //adaptive
    calcDelta = calcAdaptiveSendRate();
    //cout << "delta: " << delta << endl;
    //adaptive

    auto begin = packetSizeforDCC.begin(), end = packetSizeforDCC.end();
    for (auto iter = begin; iter != end; iter++){
        if(iter->first + calcInterval < currentTime){
            //cout << "erase: " << iter->first << endl;
            packetSizeforDCC.erase(iter->first);
        }else{
            total_packet_size += iter->second;
            //cout << "packetsize_sum: " << total_packet_size << endl;
        }
    }

    //reactive
    //calcDuration = Toff + total_packet_size * 8/*byte to bit*/ * (1000000000 / 1000000) / (dataRate / 1000000) - calcInterval;//to prevent overflow
    //cout << "else: " << total_packet_size * 8/*byte to bit*/ * 1000000000 / dataRate << endl;
    //cout << "calcDuration: " << calcDuration << ", Toff: " << Toff << ", total_packet_size: " << total_packet_size << ", dataRate: " << dataRate << ", calcInterval: " << calcInterval << endl;
    //reactive

    //adaptive
    calcDuration = total_packet_size * 8/*byte to bit*/ * (1000000000 / 1000000) / (dataRate / 1000000) / calcDelta;
    //adaptive

    return calcDuration;
}
//dcc

//dcc reactive
inline
SimTime FloodingApplication::calcReactiveSendRate(){
    double channelBusyRatio;
    SimTime calcToff;
    shared_ptr<NetworkLayer> networkLayerPtr = transportLayerPtr->GetNetworkLayerPtr();
    shared_ptr<MacLayer> macLayerPtr = networkLayerPtr->GetMacLayerPtr(0);
    shared_ptr<Dot11::Dot11Mac> dot11MacPtr = std::dynamic_pointer_cast<Dot11::Dot11Mac>(macLayerPtr); 
    channelBusyRatio = dot11MacPtr->getCBR();
    cout << "CBR: " << channelBusyRatio << " at: " << theNodeId << endl;
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
double FloodingApplication::calcAdaptiveSendRate(){
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
    //const int nodeNum = 9; 
    shared_ptr<NetworkLayer> networkLayerPtr = transportLayerPtr->GetNetworkLayerPtr();
    shared_ptr<MacLayer> macLayerPtr = networkLayerPtr->GetMacLayerPtr(0);
    shared_ptr<Dot11::Dot11Mac> dot11MacPtr = std::dynamic_pointer_cast<Dot11::Dot11Mac>(macLayerPtr); 
    channelBusyRatio = dot11MacPtr->getCBR();

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
    adaptiveCount++;
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

    return calcDelta;
}
//dcc

inline
void FloodingApplication::Broadcast(const ApplicationId& originatorApplicationId)
{
    typedef map<ApplicationId, FloodingSenderSettingType>::iterator IterType;
    IterType iter = floodingSenderSettings.find(originatorApplicationId);

    assert(iter != floodingSenderSettings.end());
    assert((*iter).second.sequenceNumber < UINT_MAX);
    (*iter).second.sequenceNumber++;

    ObjectMobilityPosition nodePosition;
    nodeMobilityModelPtr->GetPositionForTime(
        simulationEngineInterfacePtr->CurrentTime(), nodePosition);

    const int packetPayloadSizeBytes = (*iter).second.packetPayloadSizeBytes;
    /*const*/ SimTime packetInterval = (*iter).second.packetInterval;//ns
    const SimTime floodingEndTime = (*iter).second.floodingEndTime;
    const unsigned int sequenceNumber = (*iter).second.sequenceNumber;
    /*const*/ PacketPriority floodingPriority = (*iter).second.floodingPriority;
    //a:DENM+ b:DENM c:CAM d:other
    const unsigned int maxHopCount = (*iter).second.maxHopCount;
    const SimTime minWaitingPeriod = (*iter).second.minWaitingPeriod;
    const SimTime maxWaitingPeriod = (*iter).second.maxWaitingPeriod;
    const unsigned int counterThreshold = (*iter).second.counterThreshold;
    const double distanceThresholdInMeters = (*iter).second.distanceThresholdInMeters;
    const double nodePositionX = nodePosition.X_PositionMeters();
    const double nodePositionY = nodePosition.Y_PositionMeters();
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
    const SimTime nextPacketTime = currentTime + packetInterval;
    
    int objectNum = 0; 
    floodingPriority = 2;//cam

    FloodingPayloadType floodingPayload(
        originatorApplicationId,
        theNodeId,
        sequenceNumber,
        currentTime,
        nodePositionX,
        nodePositionY,
        floodingPriority,
        maxHopCount,
        0,
        minWaitingPeriod,
        maxWaitingPeriod,
        counterThreshold,
        distanceThresholdInMeters);

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(
            *simulationEngineInterfacePtr,
            floodingPayload,
            packetPayloadSizeBytes,
            useVirtualPayload);
    
    //cout << "node: " << theNodeId << endl;
    //cout << "packetsize(cam): "  << packetPayloadSizeBytes << endl;

    //dcc
    //Toff = get_duration();//adaptive
    Toff = calcReactiveSendRate();//reactive
    //cout << "Toff: " << Toff << endl;
    if(currentTime_before + Toff <= currentTime){
        OutputTraceAndStatsForBroadcast(
            originatorApplicationId,
            sequenceNumber,
            packetPtr->GetPacketId(),
            packetPtr->LengthBytes());

        transportLayerPtr->udpPtr->SendPacket(
            packetPtr, 0, NetworkAddress::broadcastAddress, destinationPortId, floodingPriority);

        sendCAMCount++;
        
        packetSizeforDCC.insert(make_pair(currentTime, packetPayloadSizeBytes));
        //currentTime_before = currentTime;
    }else{
        stopCAMCount++;
    }
    //dcc

        /*else{
            stockPacket.push_back(std::move(packetPtr));
        }*/
    //}
    //adaptive

    //no dcc
    /*OutputTraceAndStatsForBroadcast(
        originatorApplicationId,
        sequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes());

    transportLayerPtr->udpPtr->SendPacket(
        packetPtr, 0, NetworkAddress::broadcastAddress, destinationPortId, floodingPriority);

    sendCAMCount++;*/

    //dcc
    //cout << "node: " << theNodeId << endl;
    auto begin = CPMInfo.begin(), end = CPMInfo.end();
    auto pastBegin = pastCPMInfo.begin(), pastEnd = pastCPMInfo.end();
    
    /*for (auto pastIter = pastBegin; pastIter != pastEnd; pastIter++){
        if((pastIter->second.pastPositionX - CPMInfo[pastIter->first].positionX) * (pastIter->second.pastPositionX - CPMInfo[pastIter->first].positionX) + (pastIter->second.pastPositionY - CPMInfo[pastIter->first].positionY) * (pastIter->second.pastPositionY - CPMInfo[pastIter->first].positionY) > 150 * 150){
            pastCPMInfo.erase(pastIter->first);
            //cout << "remove: " << pastIter->first << endl;
        }
    }*/
    
    for (auto iter = begin; iter != end; iter++){
        if((iter->second.positionX - nodePositionX) * (iter->second.positionX - nodePositionX) + (iter->second.positionY - nodePositionY) * (iter->second.positionY - nodePositionY) <= 150 * 150/*senser range*//*should be variable*/){
            if(pastCPMInfo.count(iter->first) == 0 && iter->first != theNodeId){
                pastCPMInfo[iter->first].pastPositionX = iter->second.positionX;
                pastCPMInfo[iter->first].pastPositionY = iter->second.positionY;
                pastCPMInfo[iter->first].pastCPMTime = iter->second.CPMTime;
                objectNum++;
                if(objectNum == 36) break;
                //cout << "new object CPM: " << iter->first << endl;
            }else if(pastCPMInfo.count(iter->first) > 0){
                if(iter->second.CPMTime >= pastCPMInfo[iter->first].pastCPMTime + 1000000000 || (iter->second.positionX - pastCPMInfo[iter->first].pastPositionX) * (iter->second.positionX - pastCPMInfo[iter->first].pastPositionX) + (iter->second.positionY - pastCPMInfo[iter->first].pastPositionY) * (iter->second.positionY - pastCPMInfo[iter->first].pastPositionY) >= 16){
                    pastCPMInfo[iter->first].pastPositionX = iter->second.positionX;
                    pastCPMInfo[iter->first].pastPositionY = iter->second.positionY;
                    pastCPMInfo[iter->first].pastCPMTime = iter->second.CPMTime;
                    objectNum++;
                    if(objectNum == 36) break;
                    //cout << "update object CPM: " << iter->first << endl;
                }else{
                    //cout << "not update: " << iter->first << endl;
                }
            }else{
                //cout << "error" << endl;
            }
            //cout << "sensing: " << iter->first << endl;
            //objectNum++;
        }
    }

    CPMInfo[theNodeId].positionX = nodePositionX;
    CPMInfo[theNodeId].positionY = nodePositionY;
    CPMInfo[theNodeId].CPMTime = currentTime;
    
    floodingPriority = 1;//cpm

    if(objectNum != 0){
        const int cpm_packetPayloadSizeBytes = 121 + 35 * 1/*sensor num*/ + 35 * objectNum;
        packetPtr =
            Packet::CreatePacket(
                *simulationEngineInterfacePtr,
                floodingPayload,
                cpm_packetPayloadSizeBytes,
                useVirtualPayload);

        if(currentTime_before + Toff <= currentTime){
            OutputTraceAndStatsForBroadcast(
                originatorApplicationId,
                sequenceNumber,
                packetPtr->GetPacketId(),
                packetPtr->LengthBytes());

            transportLayerPtr->udpPtr->SendPacket(
                packetPtr, 0, NetworkAddress::broadcastAddress, destinationPortId, floodingPriority);

            sendCPMCount++;
            
            packetSizeforDCC.insert(make_pair(currentTime, cpm_packetPayloadSizeBytes));
            currentTime_before = currentTime;
        }else{
            stopCPMCount++;
        }
        
        //no dcc
        /*OutputTraceAndStatsForBroadcast(
            originatorApplicationId,
            sequenceNumber,
            packetPtr->GetPacketId(),
            packetPtr->LengthBytes());

        transportLayerPtr->udpPtr->SendPacket(
            packetPtr, 0, NetworkAddress::broadcastAddress, destinationPortId, floodingPriority);

        sendCPMCount++;*/
    }else{
        if(currentTime_before + Toff <= currentTime){
            currentTime_before = currentTime;
        }
    }
    //cout << "application: " << theNodeId << endl;
    

    if(currentTime >= 109700000000){
        cout << "sendCAMCount: " << sendCAMCount << endl;
        cout << "stopCAMCount: " << stopCAMCount << endl;
        cout << "sendCPMCount: " << sendCPMCount << endl;
        cout << "stopCPMCount: " << stopCPMCount << endl;
    }

    if (nextPacketTime < floodingEndTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new FloodingStartEvent(shared_from_this(), originatorApplicationId)),
            nextPacketTime);
    }//if//

}//Broadcast//


inline
void FloodingApplication::Rebroadcast(
    FloodingPayloadType& floodingPayload,
    const unsigned int packetPayloadSizeBytes)
{
    //cout << "rebroadcast packet from " << floodingPayload.id.sourceNodeId << " at node " << theNodeId << endl;
    
    const PacketPriority floodingPriority = floodingPayload.floodingPriority;

    ObjectMobilityPosition nodePosition;
    nodeMobilityModelPtr->GetPositionForTime(
        simulationEngineInterfacePtr->CurrentTime(), nodePosition);

    floodingPayload.nodePositionX = nodePosition.X_PositionMeters();
    floodingPayload.nodePositionY = nodePosition.Y_PositionMeters();

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(
            *simulationEngineInterfacePtr,
            floodingPayload,
            packetPayloadSizeBytes,
            useVirtualPayload);

    OutputTraceAndStatsForRebroadcast(
        string(floodingPayload.id.applicationIdChar),
        floodingPayload.id.sequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes());

    //cout << "rebroadcast" << endl;
    transportLayerPtr->udpPtr->SendPacket(
        packetPtr, 0, NetworkAddress::broadcastAddress, destinationPortId, floodingPriority);

}//Rebroadcast//


inline
void FloodingApplication::RebroadcastIfNecessary(
    FloodingPayloadType& floodingPayload,
    const unsigned int packetPayloadSizeBytes)
{
    if (IsCounterThresholdReached(floodingPayload)) {
        return;
    }//if//

    if (IsLessThanDistanceThresholdInMeters(floodingPayload)) {
        return;
    }//if//

    Rebroadcast(floodingPayload, packetPayloadSizeBytes);

}//RebroadcastIfNecessary//


inline
void FloodingApplication::ScheduleRebroadcastEvent(
    const FloodingPayloadType& floodingPayload,
    const unsigned int packetPayloadSizeBytes)
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
    const SimTime minWaitingPeriod = floodingPayload.minWaitingPeriod;
    const SimTime maxWaitingPeriod = floodingPayload.maxWaitingPeriod;

    SimTime nextPacketTime = currentTime + minWaitingPeriod;

    nextPacketTime += static_cast<SimTime>(
        aRandomNumberGeneratorPtr->GenerateRandomDouble() * (maxWaitingPeriod - minWaitingPeriod));

    simulationEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(
            new FloodingRebroadcastEvent(
                shared_from_this(),
                floodingPayload,
                packetPayloadSizeBytes)),
        nextPacketTime);

}//ScheduleRebroadcastEvent//


inline
void FloodingApplication::ReceivePacket(unique_ptr<Packet>& packetPtr)
{
    FloodingPayloadType floodingPayload =
        packetPtr->GetAndReinterpretPayloadData<FloodingPayloadType>();

    /*const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
    ObjectMobilityPosition nodePosition;
    nodeMobilityModelPtr->GetPositionForTime(
        simulationEngineInterfacePtr->CurrentTime(), nodePosition);
    double distancetmp = (floodingPayload.nodePositionX - nodePosition.X_PositionMeters()) * (floodingPayload.nodePositionX - nodePosition.X_PositionMeters()) + (floodingPayload.nodePositionY - nodePosition.Y_PositionMeters()) * (floodingPayload.nodePositionY - nodePosition.Y_PositionMeters());
    //cout << "receive at: " << theNodeId << ", xpos: " << nodePosition.X_PositionMeters() << ", ypos: " << nodePosition.Y_PositionMeters() << ", from: " << floodingPayload.id.sourceNodeId << ", xpos: " << floodingPayload.nodePositionX << ", ypos: " << floodingPayload.nodePositionY << endl;
    if(distanceCount == 0){
        maxDistance = distancetmp;
        distanceCount = 1;
    }else{
        if(distancetmp > maxDistance){
            maxDistance = distancetmp;
        }
    }
    if(currentTime >= 109700000000){
        cout << "maxdistance: " << maxDistance << endl;
    }*/

    assert(floodingPayload.hopCount < floodingPayload.maxHopCount);
    floodingPayload.hopCount++;

    IncrementCountOfPacketReceived(floodingPayload.id);
    UpdateMinDistanceBetweenNodesInMeters(floodingPayload);

    const SimTime delay =
        simulationEngineInterfacePtr->CurrentTime() - floodingPayload.broadcastTime;

    if (IsPacketReceived(floodingPayload)) {

        OutputTraceAndStatsForReceivePacket(
            string(floodingPayload.id.applicationIdChar),
            floodingPayload.id.sequenceNumber,
            packetPtr->GetPacketId(),
            packetPtr->LengthBytes(),
            floodingPayload.hopCount,
            delay);

        if (!IsMaxHopCountReached(floodingPayload)) {
            ScheduleRebroadcastEvent(floodingPayload, packetPtr->LengthBytes());
        }//if//
    }
    else {

        OutputTraceAndStatsForDiscardPacket(
            string(floodingPayload.id.applicationIdChar),
            floodingPayload.id.sequenceNumber,
            packetPtr->GetPacketId(),
            packetPtr->LengthBytes(),
            delay);
    }//if//

    packetPtr = nullptr;

}//ReceivePacket//


inline
void FloodingApplication::IncrementCountOfPacketReceived(
    const FloodingPayloadIdType& id)
{
    typedef map<FloodingPayloadIdType, unsigned int>::iterator IterType;
    IterType iter = countOfPacketReceived.find(id);

    if (iter == countOfPacketReceived.end()) {
        countOfPacketReceived.insert(make_pair(id, 1));
    }
    else {
        unsigned int count = (*iter).second;
        count++;
        (*iter).second = count;
    }//if//

}//IncrementCountOfPacketReceived//


inline
void FloodingApplication::UpdateMinDistanceBetweenNodesInMeters(
    const FloodingPayloadType& floodingPayload)
{
    ObjectMobilityPosition nodePosition;
    nodeMobilityModelPtr->GetPositionForTime(
        simulationEngineInterfacePtr->CurrentTime(), nodePosition);

    const double distanceInMeters =
        CalculateDistanceBetweenPointsInMeters(
            nodePosition.X_PositionMeters(),
            nodePosition.Y_PositionMeters(),
            floodingPayload.nodePositionX,
            floodingPayload.nodePositionY);

    typedef map<FloodingPayloadIdType, double>::iterator IterType;
    IterType iter = minDistanceBetweenNodesInMeters.find(floodingPayload.id);

    if (iter == minDistanceBetweenNodesInMeters.end()) {
        minDistanceBetweenNodesInMeters.insert(make_pair(floodingPayload.id, distanceInMeters));
    }
    else {
        double minDistanceInMeters = (*iter).second;

        if (distanceInMeters < minDistanceInMeters) {
            minDistanceInMeters = distanceInMeters;
            (*iter).second = minDistanceInMeters;
        }//if//
    }//if//

}//UpdateMinDistanceBetweenNodesInMeters//


inline
double FloodingApplication::CalculateDistanceBetweenPointsInMeters(
    const double& x1, const double& y1,
    const double& x2, const double& y2) const
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

}//CalculateDistanceBetweenPointsInMeters//


inline
bool FloodingApplication::IsSenderMyself(
    const FloodingPayloadIdType& id) const
{
    if (id.sourceNodeId == theNodeId) {
        return true;
    }
    else {
        return false;
    }//if//

}//IsSenderMyself//


inline
bool FloodingApplication::IsPacketFirstlyReceived(
    const FloodingPayloadIdType& id) const
{
    typedef map<FloodingPayloadIdType, unsigned int>::const_iterator IterType;
    IterType iter = countOfPacketReceived.find(id);

    assert(iter != countOfPacketReceived.end());

    const unsigned int count = (*iter).second;

    if (count == 1) {
        return true;
    }
    else {
        return false;
    }//if//

}//IsPacketFirstlyReceived//


inline
bool FloodingApplication::IsPacketReceived(
    const FloodingPayloadType& floodingPayload) const
{
    if (!IsSenderMyself(floodingPayload.id) &&
        IsPacketFirstlyReceived(floodingPayload.id)) {

        return true;
    }
    else {
        return false;
    }//if//

}//IsPacketReceived//


inline
bool FloodingApplication::IsMaxHopCountReached(
    const FloodingPayloadType& floodingPayload) const
{
    if (floodingPayload.hopCount == floodingPayload.maxHopCount) {
        return true;
    }
    else {
        return false;
    }//if//

}//IsMaxHopCountReached//


inline
bool FloodingApplication::IsCounterThresholdReached(
    const FloodingPayloadType& floodingPayload) const
{
    typedef map<FloodingPayloadIdType, unsigned int>::const_iterator IterType;
    IterType iter = countOfPacketReceived.find(floodingPayload.id);

    assert(iter != countOfPacketReceived.end());

    const unsigned int count = (*iter).second;

    if (count >= floodingPayload.counterThreshold) {
        return true;
    }
    else {
        return false;
    }//if//

}//IsCounterThresholdReached//


inline
bool FloodingApplication::IsLessThanDistanceThresholdInMeters(
    const FloodingPayloadType& floodingPayload) const
{
    typedef map<FloodingPayloadIdType, double>::const_iterator IterType;
    IterType iter = minDistanceBetweenNodesInMeters.find(floodingPayload.id);

    assert(iter != minDistanceBetweenNodesInMeters.end());

    const double minDistanceInMeters = (*iter).second;

    if (minDistanceInMeters < floodingPayload.distanceThresholdInMeters) {
        return true;
    }
    else {
        return false;
    }//if//

}//IsLessThanDistanceThresholdInMeters//


inline
void FloodingApplication::OutputTraceAndStatsForBroadcast(
    const ApplicationId& originatorApplicationId,
    const unsigned int sequenceNumber,
    const PacketId& thePacketId,
    const unsigned int packetLengthBytes)
{
    typedef map<ApplicationId, FloodingSenderStatType>::iterator IterType;
    IterType iter = floodingSenderStats.find(originatorApplicationId);

    assert(iter != floodingSenderStats.end());

    (*iter).second.packetsBroadcastStatPtr->IncrementCounter();
    (*iter).second.bytesBroadcastStatPtr->IncrementCounter(packetLengthBytes);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationSendTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.destinationNodeId = ANY_NODEID;
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == APPLICATION_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, originatorApplicationId, "FloodingBroadcast", traceData);
        }
        else {

            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << thePacketId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, originatorApplicationId, "FloodingBroadcast", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForBroadcast//


inline
void FloodingApplication::OutputTraceAndStatsForRebroadcast(
    const ApplicationId& receiveApplicationId,
    const unsigned int sequenceNumber,
    const PacketId& thePacketId,
    const unsigned int packetLengthBytes)
{
    typedef map<ApplicationId, FloodingReceiverStatType>::iterator IterType;
    IterType iter = floodingReceiverStats.find(receiveApplicationId);

    assert(iter != floodingReceiverStats.end());

    (*iter).second.packetsRebroadcastStatPtr->IncrementCounter();
    (*iter).second.bytesRebroadcastStatPtr->IncrementCounter(packetLengthBytes);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationSendTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.destinationNodeId = ANY_NODEID;
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == APPLICATION_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, receiveApplicationId, "FloodingRebroadcast", traceData);
        }
        else {

            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << thePacketId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, receiveApplicationId, "FloodingRebroadcast", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForRebroadcast//


inline
void FloodingApplication::OutputTraceAndStatsForReceivePacket(
    const ApplicationId& receiveApplicationId,
    const unsigned int sequenceNumber,
    const PacketId& thePacketId,
    const unsigned int packetLengthBytes,
    const unsigned int hopCount,
    const SimTime& delay)
{
    typedef map<ApplicationId, FloodingReceiverStatType>::iterator IterType;
    IterType iter = floodingReceiverStats.find(receiveApplicationId);

    assert(iter != floodingReceiverStats.end());

    (*iter).second.packetsReceivedStatPtr->IncrementCounter();
    (*iter).second.bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);
    (*iter).second.endToEndDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(delay));
    (*iter).second.hopCountStatPtr->RecordStatValue(static_cast<double>(hopCount));
    (*iter).second.countOfOriginalPacketReceived++;

    const unsigned int countOfOriginalPacketReceived = (*iter).second.countOfOriginalPacketReceived;

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationReceiveTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();
            traceData.delay = delay;
            traceData.receivedPackets = countOfOriginalPacketReceived;
            traceData.packetLengthBytes = static_cast<uint16_t>(packetLengthBytes);

            assert(sizeof(traceData) == APPLICATION_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, receiveApplicationId, "FloodingReceive", traceData);
        }
        else {

            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << thePacketId
                      << " Delay= " << ConvertTimeToStringSecs(delay)
                      << " Pdr= " << countOfOriginalPacketReceived << '/' << sequenceNumber
                      << " PacketBytes= " << packetLengthBytes;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, receiveApplicationId, "FloodingReceive", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForReceivePacket//


inline
void FloodingApplication::OutputTraceAndStatsForDiscardPacket(
    const ApplicationId& receiveApplicationId,
    const unsigned int sequenceNumber,
    const PacketId& thePacketId,
    const unsigned int packetLengthBytes,
    const SimTime& delay)
{
    typedef map<ApplicationId, FloodingReceiverStatType>::iterator IterType;
    IterType iter = floodingReceiverStats.find(receiveApplicationId);

    assert(iter != floodingReceiverStats.end());

    (*iter).second.packetsDiscardedStatPtr->IncrementCounter();
    (*iter).second.bytesDiscardedStatPtr->IncrementCounter(packetLengthBytes);

    const unsigned int countOfOriginalPacketReceived = (*iter).second.countOfOriginalPacketReceived;

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            FloodingApplicationDiscardTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();
            traceData.packetLengthBytes = static_cast<uint16_t>(packetLengthBytes);

            assert(sizeof(traceData) == FLOODING_APPLICATION_DISCARD_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, receiveApplicationId, "FloodingDiscard", traceData);
        }
        else {

            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << thePacketId
                      << " PacketBytes= " << packetLengthBytes;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, receiveApplicationId, "FloodingDiscard", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForDiscardPacket//






}//namespace//

#endif
