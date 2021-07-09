// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef T109_APP_H
#define T109_APP_H

#include "scensim_parmio.h"
#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "scensim_gis.h"

#include "t109_mac.h"

namespace T109 {

using std::pair;
using std::vector;

typedef int16_t PacketType;
enum {
    PACKET_TYPE_RSU_TO_CAR,
    PACKET_TYPE_CAR_TO_CAR,
};

struct ApplicationHeaderType {
    int16_t packetType;
    int16_t forwardCount; //-1 is ack
    int32_t sequenceNumber;

    NodeId sourceNodeId;
    NodeId lastNodeId;

    SimTime creationTime;

    ApplicationHeaderType(
        const PacketType& initPacketType,
        const NodeId& initSourceNodeId,
        const int initSequenceNumber,
        const SimTime& initCreationTime)
        :
        packetType(initPacketType),
        forwardCount(0),
        sequenceNumber(initSequenceNumber),
        sourceNodeId(initSourceNodeId),
        lastNodeId(initSourceNodeId),
        creationTime(initCreationTime)
    {}
};

class T109App : public Application {
public:
    typedef T109Phy::PropFrame PropFrame;

    T109App(
        const ParameterDatabaseReader& initParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<PropFrame> >& initPropModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& initBerCurveDatabasePtr,
        const NodeId& initNodeId,
        const InterfaceId& initInterfaceId,
        const size_t initInterfaceIndex,
        const RandomNumberGeneratorSeed& initNodeSeed,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr);

    void ReceivePacketFromMac(
        unique_ptr<Packet>& packetPtr,
        const double receiveRssiDbm,
        const SixByteMacAddress& lastHopAddress);

    shared_ptr<T109Mac> GetMacPtr() const { return macPtr; }

private:
    static const ApplicationId theApplicationId;
    static const uint16_t portNumber = 1000;
    static const uint16_t packetPriority = 0;
    static const int SEED_HASH = 2732192;
    static const string modelName;

    const NodeId theNodeId;
    const InterfaceId theInterfaceId;
    RandomNumberGenerator aRandomNumberGenerator;

    shared_ptr<ObjectMobilityModel> mobilityModelPtr;
    shared_ptr<T109Mac> macPtr;

    int currentPacketPayloadSizeByte;
    SimTime transmissionInterval;
    SimTime startTime;
    SimTime endTime;

    SimTime currentStepStartTimeOffset;
    SimTime currentStepEndTimeOffset;
    int startTimePacketSize;
    int endTimePacketSize;

    int32_t currentPacketSequenceNumber;
    int32_t numberPacketsReceived;

    vector<pair<SimTime, size_t> > packetSizeToTimes;

    shared_ptr<CounterStatistic> packetsSentStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<CounterStatistic> packetsReceivedStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;

    void PeriodicallySendPacket();

    void UpdateCurrentTimePacketSize();

    void OutputTraceAndStatsForSendPacket(
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const unsigned int packetLengthBytes);

    void OutputTraceForReceivePacket(
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const unsigned int packetLengthBytes,
        const SimTime& delay);

    class PacketSendEvent : public SimulationEvent {
    public:
        PacketSendEvent(T109App* initAppPtr) : appPtr(initAppPtr) {}

        virtual void ExecuteEvent() { appPtr->PeriodicallySendPacket(); }
    private:
        T109App* appPtr;
    };
};

}//namespace

#endif
