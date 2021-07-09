// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef WAVE_APP_H
#define WAVE_APP_H

// The standard specified a message set, and its data elements specifically
// for use by applications intended to utilize the 5.9GHz Dedicated Short
// Range Communcations for Wireless Access in Vehicular Environments(DSRC/WAVE)

// DSRC = "Dedicated Short Range Communications" = (DSRC/WAVE)

#include "wave_net.h"

namespace Wave {

typedef uint8_t DsrcMessageIdType;
enum {

    // Currently supported messages
    // 0. A La Carte
    // 1. Basic Safety Message

    DSRC_MESSAGE_A_LA_CARTE = 0,
    DSRC_MESSAGE_BASIC_SAFETY,


    // not yet
    DSRC_MESSAGE_COMMON_SAFETY_REQUEST,
    DSRC_MESSAGE_EMERGENCY_VEHICLE_ALERT,
    DSRC_MESSAGE_INTERSECTION_COLLISION_AVOIDANCE,
    DSRC_MESSAGE_MAP_DATA,
    DSRC_MESSAGE_NMEA_CORRECTIONS,
    DSRC_MESSAGE_PROBE_DATA_MANAGEMENT,
    DSRC_MESSAGE_PROBE_VEHICLE_DATA,
    DSRC_MESSAGE_ROADSIDE_ALERT,
    DSRC_MESSAGE_RTCM_CORRECTIONS,
    DSRC_MESSAGE_SIGNAL_PHASE_AND_TIMING,
    DSRC_MESSAGE_SIGNAL_REQUEST,
    DSRC_MESSAGE_SIGNAL_STATUS,
    DSRC_MESSAGE_TRAVELER_INFORMATION,
};//DsrcMessageIdType//

struct DsrcAccelerationSetType {
    uint16_t accelerationX;
    uint16_t accelerationY;
    uint8_t accelerationV;
    uint16_t yawRate;

    DsrcAccelerationSetType()
        :
        accelerationX(0),
        accelerationY(0),
        accelerationV(0),
        yawRate(0)
    {}

    DsrcAccelerationSetType(const char* payload)
        :
        accelerationX(*reinterpret_cast<const uint16_t* >(&payload[0])),
        accelerationY(*reinterpret_cast<const uint16_t* >(&payload[2])),
        accelerationV(*reinterpret_cast<const uint8_t* >(&payload[4])),
        yawRate(*reinterpret_cast<const uint16_t* >(&payload[5]))
    {}

    void Write(char* payload) const {
        *reinterpret_cast<uint16_t* >(&payload[0]) = uint16_t(accelerationX);
        *reinterpret_cast<uint16_t* >(&payload[2]) = uint16_t(accelerationY);
        *reinterpret_cast<uint8_t* >(&payload[4]) = uint8_t(accelerationV);
        *reinterpret_cast<uint16_t* >(&payload[5]) = uint16_t(yawRate);
    }
};//DsrcAccelerationSetType//

struct DsrcPositionAccurancyType {
    uint8_t semiMajorAccurancyMeters;
    uint8_t semiMinorAccurancyMeters;
    double orientationDegrees;

    DsrcPositionAccurancyType()
    :
        semiMajorAccurancyMeters(0),
        semiMinorAccurancyMeters(0),
        orientationDegrees(0)
    {}

    DsrcPositionAccurancyType(const char* payload)
        :
        semiMajorAccurancyMeters(*reinterpret_cast<const uint8_t* >(&payload[0]) / 2),
        semiMinorAccurancyMeters(*reinterpret_cast<const uint8_t* >(&payload[1]) / 2),
        orientationDegrees(*reinterpret_cast<const uint16_t* >(&payload[2]) * (360./65535))
    {}

    void Write(char* payload) const {
        *reinterpret_cast<uint8_t* >(&payload[0]) = uint8_t(semiMajorAccurancyMeters * 2);
        *reinterpret_cast<uint8_t* >(&payload[1]) = uint8_t(semiMinorAccurancyMeters * 2);
        *reinterpret_cast<uint16_t* >(&payload[2]) = uint16_t(orientationDegrees / (360./65535));
    }
};//DsrcPositionAccurancyType//

struct DsrcTransmissionAndSpeedType {
    uint16_t transmissionState;
    double speedMeters;

    struct FieldIoType {
        uint16_t state:2;
        uint16_t speed:14;
    };

    DsrcTransmissionAndSpeedType()
        :
        transmissionState(0),
        speedMeters(0)
    {}

    DsrcTransmissionAndSpeedType(const char* payload) {
        const FieldIoType& fieldIo = *reinterpret_cast<const FieldIoType* >(payload);
        transmissionState = fieldIo.state;
        speedMeters = fieldIo.speed * 0.02;
    }

    void Write(char* payload) const {
        FieldIoType& fieldIo = *reinterpret_cast<FieldIoType* >(payload);
        fieldIo.state = transmissionState;
        fieldIo.speed = uint16_t(speedMeters / 0.02);
    }
};//DsrcTransmissionAndSpeedType//

struct DsrcBrakeSystemStatusType {
    uint16_t wheelBrakes:4;
    uint16_t wheelBrakesUnavailable:1;
    uint16_t spareBit:1;
    uint16_t traction:2;
    uint16_t abs:2;
    uint16_t scs:2;
    uint16_t braksBoost:2;
    uint16_t auxBrakes:2;

    DsrcBrakeSystemStatusType()
        :
        wheelBrakes(0),
        wheelBrakesUnavailable(0),
        spareBit(0),
        traction(0),
        abs(0),
        scs(0),
        braksBoost(0),
        auxBrakes(0)
    {}
};//DsrcBrakeSystemStatusType//


struct DsrcVehicleSizeType {
    double widthMeters;
    double lengthMeters;

    DsrcVehicleSizeType()
        :
        widthMeters(0),
        lengthMeters(0)
    {}

    DsrcVehicleSizeType(const char* payload)
        :
        widthMeters(*reinterpret_cast<const uint8_t* >(&payload[0])), // simplified for simulation
        lengthMeters(*reinterpret_cast<const uint8_t* >(&payload[1]))
    {}

    void Write(char* payload) const {
        *reinterpret_cast<uint8_t* >(&payload[0]) = uint8_t(widthMeters);
        *reinterpret_cast<uint8_t* >(&payload[1]) = uint8_t(lengthMeters);
    }
};//DsrcVehicleSizeType//

struct DsrcALaCarteMessageType {

    DsrcMessageIdType messageId;
    char tailCrc[2];

    DsrcALaCarteMessageType() : messageId(DSRC_MESSAGE_A_LA_CARTE) {}
};//DsrcALaCarteMessageType//

struct DsrcBasicSafetyMessagePart1Type {

    DsrcMessageIdType messageId;
    char blob[38];

    uint8_t GetMessageCount() const { return *reinterpret_cast<const uint8_t* >(&blob[0]); }
    string GetTemporaryId() const { return string()+ blob[1]+blob[2]+blob[3]+blob[4]; }
    SimTime GetSecondMark() const { return SimTime(*reinterpret_cast<const uint16_t* >(&blob[5])) * MILLI_SECOND; }
    float GetXMeters() const { return *reinterpret_cast<const float* >(&blob[7]); }
    float GetYMeters() const { return *reinterpret_cast<const float* >(&blob[11]); }
    uint16_t GetElevationMeters() const { return (*reinterpret_cast<const uint16_t* >(&blob[15])) / 10; }
    DsrcPositionAccurancyType GetAccurancy() const { return DsrcPositionAccurancyType(&blob[17]); }
    DsrcTransmissionAndSpeedType GetSpeed() const { return DsrcTransmissionAndSpeedType(&blob[21]); }
    double GetHeading() const { return (*reinterpret_cast<const uint16_t* >(&blob[23]))*0.0125; }
    int GetAngleDegreesPerSec() const { return int(*reinterpret_cast<const int8_t* >(&blob[25]))*3; }
    DsrcAccelerationSetType GetAccelSet() const { return DsrcAccelerationSetType(&blob[26]); }
    DsrcBrakeSystemStatusType GetBrakes() const { return *reinterpret_cast<const DsrcBrakeSystemStatusType* >(&blob[33]); }
    DsrcVehicleSizeType GetSize() const { return DsrcVehicleSizeType(&blob[35]); }

    void SetMessageCount(const uint8_t messageCount) { *reinterpret_cast<uint8_t* >(&blob[0]) = messageCount; }
    void SetTemporaryId(const string& id) { assert(id.size() == 4); for(size_t i = 0; i < 4; i++) {blob[i+1] = id[i];} }
    void SetSecondMark(const SimTime& secondMark) { *reinterpret_cast<uint16_t* >(&blob[5]) = uint16_t(secondMark * MILLI_SECOND); }
    void SetXMeters(const float xMeters) { *reinterpret_cast<float* >(&blob[7]) = xMeters; }
    void SetYMeters(const float yMeters) { *reinterpret_cast<float* >(&blob[11]) = yMeters; }
    void SetElevationMeters(const uint16_t elevMeters) { *reinterpret_cast<uint16_t* >(&blob[15]) = uint16_t(elevMeters * 10); }
    void SetAccurancy(const DsrcPositionAccurancyType& arccurancy) { arccurancy.Write(&blob[17]); }
    void SetSpeed(const DsrcTransmissionAndSpeedType& speed) { speed.Write(&blob[21]); }
    void SetHeading(const double heading) { *reinterpret_cast<uint16_t* >(&blob[23]) = uint16_t(heading/0.0125); }
    void SetAngle(const int angleDegreesPerSec) { *reinterpret_cast<int8_t* >(&blob[25]) = int8_t(angleDegreesPerSec/3); }
    void SetAccelSet(const DsrcAccelerationSetType& accelSet) { accelSet.Write(&blob[26]); }
    void SetBrakes(const DsrcBrakeSystemStatusType& brakes) { *reinterpret_cast<DsrcBrakeSystemStatusType* >(&blob[33]) = brakes; }
    void SetSize(const DsrcVehicleSizeType& size) { size.Write(&blob[35]); }

    DsrcBasicSafetyMessagePart1Type();
};//DsrcBasicSafetyMessagePart1Type//

class DsrcPacketExtrinsicInformation : public ExtrinsicPacketInformation {
public:
    static const ExtrinsicPacketInfoId id;

    DsrcPacketExtrinsicInformation() : sequenceNumber(0), transmissionTime(ZERO_TIME) {}
    DsrcPacketExtrinsicInformation(
        const uint32_t initSequenceNumber,
        const SimTime& initTransmissionTime)
        :
        sequenceNumber(initSequenceNumber),
        transmissionTime(initTransmissionTime)
    {}

    virtual shared_ptr<ExtrinsicPacketInformation> Clone() {
        return shared_ptr<ExtrinsicPacketInformation>(
            new DsrcPacketExtrinsicInformation(*this));
    }

    uint32_t sequenceNumber;
    SimTime transmissionTime;
};//DsrcPacketExtrinsicInformation//

class DsrcMessageApplication : public Application {
public:
    DsrcMessageApplication(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const shared_ptr<WsmpLayer>& initWsmpLayerPtr,
        const NodeId& initNodeId,
        const RandomNumberGeneratorSeed& initNodeSeed,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr);

    void SendALaCarteMessage(
        unique_ptr<Packet>& packetPtr,
        const ChannelNumberIndexType& channelNumberId,
        const string& providerServiceId,
        const PacketPriority& priority);

    void SendBasicSafetyMessage(
        const DsrcBasicSafetyMessagePart1Type& basicSafetyMessagePart1Type);

    void SendBasicSafetyMessage(
        const DsrcBasicSafetyMessagePart1Type& basicSafetyMessagePart1Type,
        const unsigned char* part2Payload,
        const size_t part2PayloadSize);


private:
    class PeriodicBasicSafetyMessageTransmissionEvent : public SimulationEvent {
    public:
        PeriodicBasicSafetyMessageTransmissionEvent(
            DsrcMessageApplication* initDsrcMessageApp) : dsrcMessageApp(initDsrcMessageApp) {}
        virtual void ExecuteEvent() { dsrcMessageApp->PeriodicallyTransmitBasicSafetyMessage(); }
    private:
        DsrcMessageApplication* dsrcMessageApp;
    };

    class PacketHandler: public WsmpLayer::WsmApplicationHandler {
    public:
        PacketHandler(DsrcMessageApplication* initDsrcMessageApp) : dsrcMessageApp(initDsrcMessageApp) {}

        virtual void ReceiveWsm(unique_ptr<Packet>& packetPtr) { dsrcMessageApp->ReceivePacketFromLowerLayer(packetPtr); }
    private:
        DsrcMessageApplication* dsrcMessageApp;
    };

    static const string basicSafetyAppModelName;
    static const ApplicationId theApplicationId;
    static const long long SEED_HASH = 1758612;

    shared_ptr<WsmpLayer> wsmpLayerPtr;
    shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr;
    RandomNumberGenerator aRandomNumberGenerator;

    struct BasicSafetyMessageInfo {
        SimTime startTime;
        SimTime endTime;
        SimTime transmissionInterval;
        PacketPriority priority;
        string providerServiceId;
        size_t extendedPayloadSizeBytes;
        ChannelNumberIndexType channelNumberId;

        unsigned int currentSequenceNumber;
        unsigned int numberPacketsReceived;

        shared_ptr<CounterStatistic> packetsSentStatPtr;
        shared_ptr<CounterStatistic> bytesSentStatPtr;
        shared_ptr<CounterStatistic> packetsReceivedStatPtr;
        shared_ptr<CounterStatistic> bytesReceivedStatPtr;
        shared_ptr<RealStatistic> endToEndDelayStatPtr;

        BasicSafetyMessageInfo()
            :
            startTime(ZERO_TIME),
            endTime(ZERO_TIME),
            transmissionInterval(INFINITE_TIME),
            priority(0),
            providerServiceId("0"),
            extendedPayloadSizeBytes(0),
            channelNumberId(CHANNEL_NUMBER_178),
            currentSequenceNumber(0),
            numberPacketsReceived(0)
        {}
    };

    BasicSafetyMessageInfo basicSafetyMessageInfo;


    void ReceivePacketFromLowerLayer(unique_ptr<Packet>& packetPtr);
    void ReceiveALaCarteMessage(unique_ptr<Packet>& packetPtr);
    void ReceiveBasicSafetyMessage(unique_ptr<Packet>& packetPtr);

    void PeriodicallyTransmitBasicSafetyMessage();

    void OutputTraceAndStatsForSendBasicSafetyMessage(
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const size_t packetLengthBytes);

    void OutputTraceAndStatsForReceiveBasicSafetyMessage(
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const size_t packetLengthBytes,
        const SimTime& delay);
};//DsrcMessageApplication//

//--------------------------------------------------------------------------

inline
DsrcBasicSafetyMessagePart1Type::DsrcBasicSafetyMessagePart1Type()
    :
    messageId(DSRC_MESSAGE_BASIC_SAFETY)
{
    (*this).SetMessageCount(0);
    (*this).SetTemporaryId("0000");
    (*this).SetSecondMark(ZERO_TIME);
    (*this).SetXMeters(0);
    (*this).SetYMeters(0);
    (*this).SetElevationMeters(0);
    (*this).SetAccurancy(DsrcPositionAccurancyType());
    (*this).SetSpeed(DsrcTransmissionAndSpeedType());
    (*this).SetHeading(0);
    (*this).SetAngle(0);
    (*this).SetAccelSet(DsrcAccelerationSetType());
    (*this).SetBrakes(DsrcBrakeSystemStatusType());
    (*this).SetSize(DsrcVehicleSizeType());
}//DsrcBasicSafetyMessagePart1Type//

inline
DsrcMessageApplication::DsrcMessageApplication(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<WsmpLayer>& initWsmpLayerPtr,
    const NodeId& initNodeId,
    const RandomNumberGeneratorSeed& initNodeSeed,
    const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr)
    :
    Application(initSimEngineInterfacePtr, theApplicationId),
    wsmpLayerPtr(initWsmpLayerPtr),
    nodeMobilityModelPtr(initNodeMobilityModelPtr),
    aRandomNumberGenerator(HashInputsToMakeSeed(initNodeSeed, SEED_HASH))
{
    const SimTime jitter = static_cast<SimTime>(
        theParameterDatabaseReader.ReadTime("its-bsm-app-traffic-start-time-max-jitter", initNodeId) *
        aRandomNumberGenerator.GenerateRandomDouble());

    basicSafetyMessageInfo.startTime =
        theParameterDatabaseReader.ReadTime("its-bsm-app-traffic-start-time", initNodeId) + jitter;

    basicSafetyMessageInfo.endTime =
        theParameterDatabaseReader.ReadTime("its-bsm-app-traffic-end-time", initNodeId) + jitter;

    basicSafetyMessageInfo.transmissionInterval =
        theParameterDatabaseReader.ReadTime("its-bsm-app-traffic-interval", initNodeId);

    basicSafetyMessageInfo.priority =
        ConvertToUChar(
            theParameterDatabaseReader.ReadNonNegativeInt("its-bsm-app-packet-priority", initNodeId),
            "Error in parameter: \"its-bsm-app-packet-priority\"");

    if (theParameterDatabaseReader.ParameterExists("its-bsm-app-service-provider-id", initNodeId)) {
        basicSafetyMessageInfo.providerServiceId =
            ConvertToProviderServiceIdString(theParameterDatabaseReader.ReadNonNegativeInt("its-bsm-app-service-provider-id", initNodeId));
    }//if//

    basicSafetyMessageInfo.extendedPayloadSizeBytes = sizeof(DsrcBasicSafetyMessagePart1Type);

    if (theParameterDatabaseReader.ParameterExists("its-bsm-app-packet-payload-size-bytes", initNodeId)) {
        basicSafetyMessageInfo.extendedPayloadSizeBytes =
            theParameterDatabaseReader.ReadNonNegativeInt("its-bsm-app-packet-payload-size-bytes", initNodeId);

        if (basicSafetyMessageInfo.extendedPayloadSizeBytes < sizeof(DsrcBasicSafetyMessagePart1Type)) {
            cerr << "its-bsm-app-packet-payload-size-bytes must be more than "
                 << "BasicSafetyMessagePayloadPart1:"
                 << sizeof(DsrcBasicSafetyMessagePart1Type) << endl;
            exit(1);
        }//if//
    }//if//

    if (theParameterDatabaseReader.ParameterExists("its-bsm-channel-number", initNodeId)) {
        basicSafetyMessageInfo.channelNumberId =
            ConvertToChannelNumberIndex(
                theParameterDatabaseReader.ReadInt("its-bsm-channel-number", initNodeId));
    }//if//

    basicSafetyMessageInfo.packetsSentStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            (basicSafetyAppModelName + "_PacketsSent"));

    basicSafetyMessageInfo.bytesSentStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            (basicSafetyAppModelName + "_BytesSent"));

    basicSafetyMessageInfo.packetsReceivedStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            (basicSafetyAppModelName + "_PacketsReceived"));

    basicSafetyMessageInfo.bytesReceivedStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            (basicSafetyAppModelName  + "_BytesReceived"));

    basicSafetyMessageInfo.endToEndDelayStatPtr =
        simulationEngineInterfacePtr->CreateRealStat(
            (basicSafetyAppModelName + "_EndToEndDelay"));

    wsmpLayerPtr->SetWsmApplicationHandler(
        basicSafetyMessageInfo.providerServiceId,
        shared_ptr<WsmpLayer::WsmApplicationHandler>(
            new PacketHandler(this)));

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    SimTime nextTransmissionTime = basicSafetyMessageInfo.startTime;

    if (currentTime > basicSafetyMessageInfo.startTime) {
        const size_t numberPassedTransmissionTimes =
            size_t(std::ceil(double(currentTime - basicSafetyMessageInfo.startTime) /
                             basicSafetyMessageInfo.transmissionInterval));

        nextTransmissionTime =
            basicSafetyMessageInfo.startTime +
            numberPassedTransmissionTimes*basicSafetyMessageInfo.transmissionInterval;
    }//if//

    if (nextTransmissionTime < basicSafetyMessageInfo.endTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(
                new PeriodicBasicSafetyMessageTransmissionEvent(this)),
            nextTransmissionTime);
    }//if//
}//DsrcMessageApplication//

inline
void DsrcMessageApplication::SendALaCarteMessage(
    unique_ptr<Packet>& packetPtr,
    const ChannelNumberIndexType& channelNumberId,
    const string& providerServiceId,
    const PacketPriority& priority)
{
    packetPtr->AddPlainStructHeader(DsrcALaCarteMessageType());
    wsmpLayerPtr->SendWsm(
        packetPtr,
        NetworkAddress::broadcastAddress,
        channelNumberId,
        providerServiceId,
        priority);
}//SendALaCarteMessage//

inline
void DsrcMessageApplication::SendBasicSafetyMessage(
    const DsrcBasicSafetyMessagePart1Type& basicSafetyMessagePart1)
{
    (*this).SendBasicSafetyMessage(basicSafetyMessagePart1, nullptr, 0);
}//SendBasicSafetyMessage//

inline
void DsrcMessageApplication::SendBasicSafetyMessage(
    const DsrcBasicSafetyMessagePart1Type& basicSafetyMessagePart1,
    const unsigned char* part2Payload,
    const size_t part2PayloadSize)
{
    const unsigned char* part1Payload =
        reinterpret_cast<const unsigned char* >(&basicSafetyMessagePart1);

    const size_t part1PayloadSize =
        sizeof(DsrcBasicSafetyMessagePart1Type);


    vector<unsigned char> payload(part1PayloadSize + part2PayloadSize);

    for(size_t i = 0; i < part1PayloadSize; i++) {
        payload[i] = part1Payload[i];
    }//for
    for(size_t i = 0; i < part2PayloadSize; i++) {
        payload[part1PayloadSize + i] = part2Payload[i];
    }//for


    unique_ptr<Packet> packetPtr = Packet::CreatePacket(*simulationEngineInterfacePtr, payload);

    packetPtr->AddExtrinsicPacketInformation(
        DsrcPacketExtrinsicInformation::id,
        shared_ptr<DsrcPacketExtrinsicInformation>(
            new DsrcPacketExtrinsicInformation(
                basicSafetyMessageInfo.currentSequenceNumber,
                simulationEngineInterfacePtr->CurrentTime())));

    (*this).OutputTraceAndStatsForSendBasicSafetyMessage(
        basicSafetyMessageInfo.currentSequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes());

    wsmpLayerPtr->SendWsm(
        packetPtr,
        NetworkAddress::broadcastAddress,
        basicSafetyMessageInfo.channelNumberId,
        basicSafetyMessageInfo.providerServiceId,
        basicSafetyMessageInfo.priority);

    basicSafetyMessageInfo.currentSequenceNumber++;
}//SendBasicSafetyMessage//

inline
void DsrcMessageApplication::PeriodicallyTransmitBasicSafetyMessage()
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
    ObjectMobilityPosition position;
    nodeMobilityModelPtr->GetPositionForTime(currentTime, position);

    DsrcBasicSafetyMessagePart1Type basicSafetyMessagePart1;

    basicSafetyMessagePart1.SetMessageCount(uint8_t(basicSafetyMessageInfo.currentSequenceNumber));
    basicSafetyMessagePart1.SetXMeters(float(position.X_PositionMeters()));
    basicSafetyMessagePart1.SetYMeters(float(position.Y_PositionMeters()));
    basicSafetyMessagePart1.SetElevationMeters(uint16_t(position.HeightFromGroundMeters()));

    assert(basicSafetyMessageInfo.extendedPayloadSizeBytes >=
           sizeof(DsrcBasicSafetyMessagePart1Type));

    size_t part2PayloadSize = std::max<size_t>(0, basicSafetyMessageInfo.extendedPayloadSizeBytes - sizeof(DsrcBasicSafetyMessagePart1Type));
    unsigned char* part2Payload = new unsigned char[part2PayloadSize];

    (*this).SendBasicSafetyMessage(
        basicSafetyMessagePart1,
        part2Payload,
        basicSafetyMessageInfo.extendedPayloadSizeBytes - sizeof(DsrcBasicSafetyMessagePart1Type));


    delete [] part2Payload;


    if (currentTime + basicSafetyMessageInfo.transmissionInterval < basicSafetyMessageInfo.endTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(
                new PeriodicBasicSafetyMessageTransmissionEvent(this)),
            (currentTime + basicSafetyMessageInfo.transmissionInterval));
    }//if//
}//PeriodicallyTransmitBasicSafetyMessage//

inline
void DsrcMessageApplication::ReceivePacketFromLowerLayer(unique_ptr<Packet>& packetPtr)
{
    const DsrcMessageIdType messageId = packetPtr->GetAndReinterpretPayloadData<DsrcMessageIdType>();

    switch (messageId) {
    case DSRC_MESSAGE_A_LA_CARTE:
        (*this).ReceiveALaCarteMessage(packetPtr);
        break;

    case DSRC_MESSAGE_BASIC_SAFETY:
        (*this).ReceiveBasicSafetyMessage(packetPtr);
        break;

    default:
        assert("Received not supported DSRC message type");
        break;
    }//switch//

    packetPtr = nullptr;

}//ReceivePacketFromLowerLayer//

inline
void DsrcMessageApplication::ReceiveALaCarteMessage(unique_ptr<Packet>& packetPtr)
{
    packetPtr->DeleteHeader(sizeof(DsrcALaCarteMessageType));

    // nothing to do
}//ReceiveALaCarteMessage//

inline
void DsrcMessageApplication::ReceiveBasicSafetyMessage(unique_ptr<Packet>& packetPtr)
{
    const DsrcBasicSafetyMessagePart1Type part1 =
        packetPtr->GetAndReinterpretPayloadData<DsrcBasicSafetyMessagePart1Type>();

    if (packetPtr->LengthBytes() > sizeof(part1)) {
        const unsigned char* part2 =
            packetPtr->GetRawPayloadData(sizeof(part1), packetPtr->LengthBytes() - sizeof(part1));

    }//if//

    const DsrcPacketExtrinsicInformation& extInfo =
        packetPtr->GetExtrinsicPacketInformation<DsrcPacketExtrinsicInformation>(
            DsrcPacketExtrinsicInformation::id);

    const SimTime delay =
        simulationEngineInterfacePtr->CurrentTime() - extInfo.transmissionTime;

    basicSafetyMessageInfo.numberPacketsReceived++;

    (*this).OutputTraceAndStatsForReceiveBasicSafetyMessage(
        extInfo.sequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes(),
        delay);
}//ReceiveBasicSafetyMessage//

inline
void DsrcMessageApplication::OutputTraceAndStatsForSendBasicSafetyMessage(
    const unsigned int sequenceNumber,
    const PacketId& thePacketId,
    const size_t packetLengthBytes)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationSendTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.destinationNodeId = ANY_NODEID;
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == APPLICATION_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                basicSafetyAppModelName,
                "",
                "BsmSend",
                traceData);

        } else {

            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << thePacketId;

            simulationEngineInterfacePtr->OutputTrace(
                basicSafetyAppModelName,
                "",
                "BsmSend",
                outStream.str());
        }//if//
    }//if//

    basicSafetyMessageInfo.packetsSentStatPtr->IncrementCounter();
    basicSafetyMessageInfo.bytesSentStatPtr->IncrementCounter(packetLengthBytes);
}//OutputTraceAndStatsForSendBasicSafetyMessage//

inline
void DsrcMessageApplication::OutputTraceAndStatsForReceiveBasicSafetyMessage(
    const unsigned int sequenceNumber,
    const PacketId& thePacketId,
    const size_t packetLengthBytes,
    const SimTime& delay)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationReceiveTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();
            traceData.delay = delay;
            traceData.receivedPackets = basicSafetyMessageInfo.numberPacketsReceived;
            traceData.packetLengthBytes = static_cast<uint16_t>(packetLengthBytes);

            assert(sizeof(traceData) == APPLICATION_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                basicSafetyAppModelName,
                "",
                "BsmRecv",
                traceData);

        } else {
            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << thePacketId
                      << " Delay= " << ConvertTimeToStringSecs(delay)
                      << " Pdr= " << basicSafetyMessageInfo.numberPacketsReceived << '/' << sequenceNumber
                      << " PacketBytes= " << packetLengthBytes;

            simulationEngineInterfacePtr->OutputTrace(
                basicSafetyAppModelName,
                "",
                "BsmRecv",
                outStream.str());
        }//if//
    }//if//

    basicSafetyMessageInfo.packetsReceivedStatPtr->IncrementCounter();
    basicSafetyMessageInfo.bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);
    basicSafetyMessageInfo.endToEndDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(delay));
}//OutputTraceAndStatsForReceiveBasicSafetyMessage//

} //namespace Wave//

#endif
