// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_APP_IPERF_UDP_H
#define SCENSIM_APP_IPERF_UDP_H

#include "scensim_netsim.h"
#include <algorithm>

namespace ScenSim {

using std::max;

class IperfUdpApplication
    : public Application, public enable_shared_from_this<IperfUdpApplication> {
public:
    static const string modelName;
    static const string modelNameClientOnly;
    static const string modelNameServerOnly;

    IperfUdpApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationId& initApplicationId,
        const NodeId& initNodeIdWithParameter,
        const NodeId& initSourceNodeId,
        const NodeId& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl);

    void CompleteInitialization();

    void DisconnectFromOtherLayers();

    virtual ~IperfUdpApplication() {}

private:
    static const uint32_t HEADER_VERSION1 = 0x80000000;

    enum IperfUdpEventKind {
        START_CLIENT,
        SEND_PACKET_FROM_CLIENT,
        START_SERVER,
        FLOW_RESERVATION
    };//IperfUdpEventKind//

    struct UdpDatagram {
        int32_t id;
        uint32_t tv_sec;
        uint32_t tv_usec;
    };//UdpDatagram//

    struct ClientHeader {
        int32_t flags;
        int32_t numThreads;
        int32_t mPort;
        int32_t bufferlen;
        int32_t mWinBand;
        int32_t mAmount;
    };//ClientHeader//

    struct ServerHeader {
        int32_t flags;
        int32_t total_len1;
        int32_t total_len2;
        int32_t stop_sec;
        int32_t stop_usec;
        int32_t error_cnt;
        int32_t outorder_cnt;
        int32_t datagrams;
        int32_t jitter1;
        int32_t jitter2;
    };//ServerHeader//

    struct TransferInfo {
        int id;
        int cntError;
        int cntOutofOrder;
        int cntDatagrams;
        int TotalLen;
        double jitter;
        double transit;
        SimTime startTime;
        SimTime endTime;
    };//TransferInfo//

    template<typename T> static T& GetHeader(
        vector<unsigned char>& buffer, int offset);
    template<typename T> static T& GetHeader(const Packet& aPacket, int offset);
    template<typename T> static const T& GetHeader(const Packet& aPacket, int offset);
    static bool IsFinPacket(const Packet& aPacket);
    static int GetId(const Packet& aPacket);
    static SimTime GetSentTime(const Packet& aPacket);

    class PacketHandler
        : public UdpProtocol::PacketForAppFromTransportLayerHandler {
    public:
        PacketHandler(
            const shared_ptr<IperfUdpApplication>& initIperfUdpApplicationPtr)
            : iperfUdpApplicationPtr(initIperfUdpApplicationPtr) {}

        void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const NetworkAddress& sourceAddress,
            const unsigned short int sourcePortId,
            const NetworkAddress& destinationAddress,
            const PacketPriority& priority)
        {
            if (iperfUdpApplicationPtr->IsServer()) {
                iperfUdpApplicationPtr->ReceivePacketOnServer(
                    packetPtr, sourceAddress, sourcePortId);
            }
            else {
                iperfUdpApplicationPtr->ReceivePacketOnClient(packetPtr);
            }//if//

        }//ReceivePacket//

    private:
        shared_ptr<IperfUdpApplication> iperfUdpApplicationPtr;

    };//PacketHandler//

    class IperfUdpEvent: public SimulationEvent {
    public:
        explicit
        IperfUdpEvent(
            const shared_ptr<IperfUdpApplication>& initIperfUdpApplicationPtr,
            const IperfUdpEventKind& initIperfUdpEventKind)
            :
            iperfUdpApplicationPtr(initIperfUdpApplicationPtr),
            iperfUdpEventKind(initIperfUdpEventKind)
        {}

        virtual void ExecuteEvent()
        {
            switch (iperfUdpEventKind) {
            case START_CLIENT:
                iperfUdpApplicationPtr->StartClient();
                break;
            case SEND_PACKET_FROM_CLIENT:
                iperfUdpApplicationPtr->SendPacketFromClient();
                break;
            case START_SERVER:
                iperfUdpApplicationPtr->StartServer();
                break;
            case FLOW_RESERVATION:
                iperfUdpApplicationPtr->ReserveBandwidth();
                break;
            }//switch//

        }//ExecuteEvent//

    private:
        shared_ptr<IperfUdpApplication> iperfUdpApplicationPtr;
        IperfUdpEventKind iperfUdpEventKind;

    };//IperfUdpEvent//

    class FlowRequestReplyFielder
        : public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<IperfUdpApplication>& initAppPtr)
            : appPtr(initAppPtr) {}
        void RequestAccepted(const FlowId& theFlowId)
            { appPtr->macQosFlowId = theFlowId; }
        void RequestDenied()
            { appPtr->ReserveBandwidthRequestDeniedAction(); }
    private:
        shared_ptr<IperfUdpApplication> appPtr;

    };//FlowRequestReplyFielder//

    void FillPattern(int length);
    SimTime GetDifferenceTime(const SimTime& pastTime) const;
    string GetModelName() const;
    bool IsServer() const;
    bool IsServerApp() const;
    bool IsClientApp() const;
    void InitUdpDatagram();
    void UpdateUdpDatagram(bool fin = false);
    void InitClientHeader();
    void InitServerHeader();
    void InitTransferInfo();
    void UpdateTransferInfo(const Packet& aPacket);
    void FinishTransferInfo();

    void StartClient();
    void EndClient();
    void SendPacketFromClient();
    void SendFinPacketFromClient();
    void ReceivePacketOnClient(unique_ptr<Packet>& packetPtr);

    void StartServer();
    void EndServer();
    void SendAckPacketFromServer(
        const NetworkAddress& sourceAddress,
        const unsigned short int portId);
    void ReceivePacketOnServer(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const unsigned short int portId);

    void OutputTraceAndStatsForStart();
    void OutputTraceAndStatsForEnd(const Packet& aPacket);
    void OutputTraceAndStatsForSendPacket(const Packet& aPacket);
    void OutputTraceAndStatsForReceivePacket(const Packet& aPacket);

    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();

    string GetParameterNamePrefix() const
    {
        string parameterNamePrefix = "iperf-udp";

        if (IsServerApp()) {
            parameterNamePrefix += "-server";
        }
        else if (IsClientApp()) {
            parameterNamePrefix += "-client";
        }//if//

        if (reserveBandwidthModeIsOn) {
            parameterNamePrefix += "-with-qos";
        }//if//

        return parameterNamePrefix;

    }//GetParameterNamePrefix//

    bool useVirtualPayload;
    bool autoAddressModeEnabled;
    NetworkAddress specifiedDestinationAddress;
    NodeId sourceNodeId;
    NodeId destinationNodeId;
    unsigned short int sourcePortId;
    unsigned short int destinationPortId;
    PacketPriority priority;
    SimTime startTime;

    bool reserveBandwidthModeIsOn;
    double qosMinBandwidth;
    double qosMaxBandwidth;
    FlowId macQosFlowId;
    MacQualityOfServiceControlInterface::SchedulingSchemeChoice schedulingScheme;
    MacQualityOfServiceControlInterface::ReservationSchemeChoice reservationScheme;

    bool waitingAck;
    bool timeModeEnabled;
    SimTime totalTime;
    unsigned long long int totalSendingBytes;
    unsigned long long int totalSentBytes;
    int bufferLength;
    unsigned long long int udpRateInBps;
    bool useSystemTimeInPacket;
    SimTime endTime;
    SimTime intervalTime;
    bool isPortOpened;
    vector<unsigned char> buffer;
    shared_ptr<PacketHandler> udpPacketHandlerPtr;
    TransferInfo transferInfo;

    int packetsReceived;
    shared_ptr<CounterStatistic> packetsSentStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<CounterStatistic> packetsReceivedStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;
    shared_ptr<RealStatistic> endToEndDelayStatPtr;

};//IperfUdpApplication//

inline
IperfUdpApplication::IperfUdpApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationId& initApplicationId,
    const NodeId& initNodeIdWithParameter,
    const NodeId& initSourceNodeId,
    const NodeId& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl)
    :
    Application(initSimulationEngineInterfacePtr, initApplicationId),
    useVirtualPayload(false),
    autoAddressModeEnabled(true),
    specifiedDestinationAddress(NetworkAddress::invalidAddress),
    sourceNodeId(initSourceNodeId),
    destinationNodeId(initDestinationNodeId),
    sourcePortId(initDefaultApplicationPortId),
    destinationPortId(initDefaultApplicationPortId),
    priority(0),
    startTime(ZERO_TIME),
    reserveBandwidthModeIsOn(initEnableQosControl),
    qosMinBandwidth(0.0),
    qosMaxBandwidth(0.0),
    macQosFlowId(),
    schedulingScheme(MacQualityOfServiceControlInterface::DefaultSchedulingScheme),
    reservationScheme(MacQualityOfServiceControlInterface::OptimisticLinkRate),
    waitingAck(false),
    timeModeEnabled(true),
    totalTime(10 * SECOND),
    totalSendingBytes(1024 * 1024 * 10 / 8),
    totalSentBytes(0),
    bufferLength(1470),
    udpRateInBps(1024 * 1024),
    useSystemTimeInPacket(false),
    endTime(ZERO_TIME),
    intervalTime(8 * bufferLength * SECOND / udpRateInBps),
    isPortOpened(false),
    buffer(),
    udpPacketHandlerPtr(),
    transferInfo(),
    packetsReceived(0),
    packetsSentStatPtr(),
    bytesSentStatPtr(),
    packetsReceivedStatPtr(),
    bytesReceivedStatPtr(),
    endToEndDelayStatPtr()
{
    const string parameterPrefix = GetParameterNamePrefix();

    priority = static_cast<PacketPriority>(
        parameterDatabaseReader.ReadNonNegativeInt(
            parameterPrefix + "-priority", initNodeIdWithParameter, theApplicationId));

    startTime = parameterDatabaseReader.ReadTime(
        parameterPrefix + "-start-time", initNodeIdWithParameter, theApplicationId);

    endTime = startTime + totalTime;

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-auto-port-mode", initNodeIdWithParameter, theApplicationId)) {

        if (!parameterDatabaseReader.ReadBool(
                parameterPrefix + "-auto-port-mode", initNodeIdWithParameter, theApplicationId)) {

            destinationPortId = static_cast<unsigned short int>(
                parameterDatabaseReader.ReadNonNegativeInt(
                    parameterPrefix + "-destination-port", initNodeIdWithParameter, theApplicationId));

            sourcePortId = destinationPortId;
        }//if//
    }
    else {
        if (IsClientApp() || IsServerApp()) {
            destinationPortId = static_cast<unsigned short int>(
                parameterDatabaseReader.ReadNonNegativeInt(
                    parameterPrefix + "-destination-port", initNodeIdWithParameter, theApplicationId));

            sourcePortId = destinationPortId;
        }//if//
    }//if//

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-auto-address-mode", initNodeIdWithParameter, theApplicationId)) {

        autoAddressModeEnabled = parameterDatabaseReader.ReadBool(
            parameterPrefix + "-auto-address-mode", initNodeIdWithParameter, theApplicationId);
    }
    else {
        if (IsClientApp()) {
            autoAddressModeEnabled = false;
        }//if//
    }//if//

    if (!autoAddressModeEnabled) {
        const string addressString = parameterDatabaseReader.ReadString(
            parameterPrefix + "-destination-address", initNodeIdWithParameter, theApplicationId);

        bool success;
        specifiedDestinationAddress.SetAddressFromString(addressString, success);
        if (!success) {
            cerr << parameterPrefix + "-destination-address(" << addressString
                 << ") is invalid." << endl;
            exit(1);
        }//if//
    }//if//

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-time-mode", initNodeIdWithParameter, theApplicationId)) {

        timeModeEnabled = parameterDatabaseReader.ReadBool(
            parameterPrefix + "-time-mode", initNodeIdWithParameter, theApplicationId);
    }//if//

    if (timeModeEnabled) {
        if (parameterDatabaseReader.ParameterExists(
            parameterPrefix + "-total-time", initNodeIdWithParameter, theApplicationId)) {

            totalTime = parameterDatabaseReader.ReadTime(
                parameterPrefix + "-total-time", initNodeIdWithParameter, theApplicationId);

            endTime = startTime + totalTime;
        }//if//
    }
    else {
        if (parameterDatabaseReader.ParameterExists(
            parameterPrefix + "-total-size-bytes", initNodeIdWithParameter, theApplicationId)) {

            totalSendingBytes = parameterDatabaseReader.ReadNonNegativeBigInt(
                parameterPrefix + "-total-size-bytes", initNodeIdWithParameter, theApplicationId);
        }//if//
    }//if//

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-payload-size-bytes", initNodeIdWithParameter, theApplicationId)) {

        bufferLength = parameterDatabaseReader.ReadInt(
            parameterPrefix + "-payload-size-bytes", initNodeIdWithParameter, theApplicationId);

        const int leastLength = static_cast<int>(
            sizeof(UdpDatagram) + max(sizeof(ClientHeader), sizeof(ServerHeader)));
        if (bufferLength < leastLength) {
            cerr << parameterPrefix + "-payload-size-bytes(" << bufferLength
                 << ") is too small." << endl
                 << parameterPrefix + "-payload-size-bytes should be greater or equal to "
                 << leastLength << "." << endl;
            exit(1);
        }//if//

        intervalTime = 8 * bufferLength * SECOND / udpRateInBps;
    }//if//

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-rate-bps", initNodeIdWithParameter, theApplicationId)) {

        udpRateInBps =
            parameterDatabaseReader.ReadNonNegativeBigInt(
                parameterPrefix + "-rate-bps", initNodeIdWithParameter, theApplicationId);

        intervalTime = 8 * bufferLength * SECOND / udpRateInBps;
    }//if//

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-use-system-time", initNodeIdWithParameter, theApplicationId)) {

        useSystemTimeInPacket = parameterDatabaseReader.ReadBool(
            parameterPrefix + "-use-system-time", initNodeIdWithParameter, theApplicationId);
    }//if//

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-use-virtual-payload", initNodeIdWithParameter, theApplicationId)) {

        useVirtualPayload = parameterDatabaseReader.ReadBool(
            parameterPrefix + "-use-virtual-payload", initNodeIdWithParameter, theApplicationId);
    }//if//

    if (initEnableQosControl) {
        qosMinBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-baseline-bandwidth-bytes", initNodeIdWithParameter, theApplicationId);

        qosMaxBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-max-bandwidth-bytes", initNodeIdWithParameter, theApplicationId);

        if (parameterDatabaseReader.ParameterExists(
                parameterPrefix + "-schedule-scheme", initNodeIdWithParameter, theApplicationId)) {

            const string schedulingSchemeChoiceString =
                parameterDatabaseReader.ReadString(
                    parameterPrefix + "-schedule-scheme", initNodeIdWithParameter, theApplicationId);

            bool succeeded;

            ConvertStringToSchedulingSchemeChoice(
                MakeLowerCaseString(schedulingSchemeChoiceString),
                succeeded,
                schedulingScheme);

            if (!succeeded) {
                cerr << "Error in " << GetModelName() << " Application: Scheduling Scheme not recognized in:" << endl;
                cerr << "      >" << schedulingSchemeChoiceString << endl;
                exit(1);
            }//if//
        }//if//
    }//if//

    FillPattern(bufferLength);

    if (!IsServerApp()) {
        packetsSentStatPtr =
            simulationEngineInterfacePtr->CreateCounterStat(
                GetModelName() + "_" + initApplicationId + "_PacketsSent");
        bytesSentStatPtr =
            simulationEngineInterfacePtr->CreateCounterStat(
                GetModelName() + "_" + initApplicationId + "_BytesSent");
    }//if//

    if (!IsClientApp()) {
        packetsReceivedStatPtr =
            simulationEngineInterfacePtr->CreateCounterStat(
                GetModelName() + "_" + initApplicationId + "_PacketsReceived");
        bytesReceivedStatPtr =
            simulationEngineInterfacePtr->CreateCounterStat(
                GetModelName() + "_" + initApplicationId + "_BytesReceived");
        endToEndDelayStatPtr =
            simulationEngineInterfacePtr->CreateRealStat(
                GetModelName() + "_" + initApplicationId + "_EndToEndDelay");
    }//if//

}//IperfUdpApplication//

// Two part initialization forced by shared_from_this().

inline
void IperfUdpApplication::CompleteInitialization()
{
    udpPacketHandlerPtr = shared_ptr<PacketHandler>(
        new PacketHandler(shared_from_this()));

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
    SimTime adjustedStartTime = std::max(currentTime, startTime);

    if (reserveBandwidthModeIsOn) {
        const SimTime minimumSetupTime = 1 * MILLI_SECOND;
        const SimTime reservationLeewayTime = 1 * MILLI_SECOND;

        SimTime reservationTime;
        if ((currentTime + minimumSetupTime + reservationLeewayTime) <= startTime) {
            reservationTime = (startTime - reservationLeewayTime);
        }
        else {
            reservationTime = (currentTime + minimumSetupTime);
        }//if//

        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new IperfUdpEvent(shared_from_this(), FLOW_RESERVATION)),
            reservationTime);

        adjustedStartTime = std::max(currentTime + minimumSetupTime, startTime);

        //Future: QoS Flow termination delay parameter.

        endTime += reservationLeewayTime;

    }//if//

    if (IsServer()) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new IperfUdpEvent(shared_from_this(), START_SERVER)),
            adjustedStartTime);
    }
    else {
        if (timeModeEnabled) {
            if (adjustedStartTime < endTime) {
                simulationEngineInterfacePtr->ScheduleEvent(
                    unique_ptr<SimulationEvent>(new IperfUdpEvent(shared_from_this(), START_CLIENT)),
                    adjustedStartTime);
            }//if//
        }
        else {
            simulationEngineInterfacePtr->ScheduleEvent(
                unique_ptr<SimulationEvent>(new IperfUdpEvent(shared_from_this(), START_CLIENT)),
                adjustedStartTime);
        }//if//
    }//if//

}//CompleteInitialization//

inline
void IperfUdpApplication::DisconnectFromOtherLayers()
{
    udpPacketHandlerPtr.reset();
    (*this).Application::DisconnectFromOtherLayers();

}//DisconnectFromOtherLayers//

template<typename T> inline
T& IperfUdpApplication::GetHeader(vector<unsigned char>& buffer, int offset)
{
    assert(buffer.size() >= offset + sizeof(T));
    return *reinterpret_cast<T*>(&buffer[offset]);

}//GetHeader//

inline
bool IperfUdpApplication::IsFinPacket(const Packet& aPacket)
{
    const UdpDatagram& udpDatagram = aPacket.GetAndReinterpretPayloadData<const UdpDatagram>();
    return (static_cast<int>(NetToHost32(udpDatagram.id)) < 0);

}//IsFinPacket//

inline
int IperfUdpApplication::GetId(const Packet& aPacket)
{
    const UdpDatagram& udpDatagram = aPacket.GetAndReinterpretPayloadData<const UdpDatagram>();
    return NetToHost32(udpDatagram.id);

}//GetId//

inline
SimTime IperfUdpApplication::GetSentTime(const Packet& aPacket)
{

    const UdpDatagram& udpDatagram = aPacket.GetAndReinterpretPayloadData<const UdpDatagram>();

    const SimTime sentTime =
        NetToHost32(udpDatagram.tv_sec) * SECOND +
            NetToHost32(udpDatagram.tv_usec) * MICRO_SECOND;

    return sentTime;

}//GetSentTime//

inline
void IperfUdpApplication::FillPattern(int length)
{
    if (useVirtualPayload) {
        if (IsServer()) {
            buffer.resize(sizeof(UdpDatagram) + sizeof(ServerHeader));
        }
        else {
            buffer.resize(sizeof(UdpDatagram) + sizeof(ClientHeader));
        }//if//
    }
    else {
        buffer.resize(length);

        while (length-- > 0) {
            buffer.at(length) = (length % 10) + '0';
        }//while//
    }//if//

}//FillPattern//

inline
SimTime IperfUdpApplication::GetDifferenceTime(const SimTime& pastTime) const
{
    SimTime currentTime;

    if (useSystemTimeInPacket) {
        uint32_t tvSec;
        uint32_t tvUsec;
        bool success;
        GetTimeOfDay(tvSec, tvUsec, success);
        if (success) {
            currentTime = tvSec * SECOND + tvUsec * MICRO_SECOND;
        }
        else {
            assert(false && "GetTimeOfDay() failed");
            return ZERO_TIME;
        }//if//
    }
    else {
        currentTime = simulationEngineInterfacePtr->CurrentTime();
    }//if//

    assert(currentTime >= pastTime);
    const SimTime diffTime = currentTime - pastTime;
    return diffTime;

}//GetDifferenceTime//

inline
string IperfUdpApplication::GetModelName() const
{
    if (IsClientApp()) {
        return modelNameClientOnly;
    }
    else if (IsServerApp()) {
        return modelNameServerOnly;
    }
    else {
        return modelName;
    }//if//

}//GetModelName//

inline
bool IperfUdpApplication::IsServer() const
{
    const NodeId theNodeId =
        simulationEngineInterfacePtr->GetNodeId();

    return (theNodeId == destinationNodeId);

}//IsServer//

inline
bool IperfUdpApplication::IsServerApp() const
{
    return (sourceNodeId == INVALID_NODEID);

}//IsServerApp//

inline
bool IperfUdpApplication::IsClientApp() const
{
    return (destinationNodeId == INVALID_NODEID);

}//IsClientApp//

inline
void IperfUdpApplication::InitUdpDatagram()
{
    UdpDatagram& udpDatagram =
        GetHeader<UdpDatagram>(buffer, 0);

    udpDatagram.id = HostToNet32(static_cast<uint32_t>(-1));
    udpDatagram.tv_sec = 0;
    udpDatagram.tv_usec = 0;

}//InitUdpDatagram//

inline
void IperfUdpApplication::UpdateUdpDatagram(bool fin)
{
    UdpDatagram& udpDatagram =
        GetHeader<UdpDatagram>(buffer, 0);

    const int32_t id = NetToHost32(udpDatagram.id) + 1;

    if (fin) {
        udpDatagram.id = HostToNet32(-id);
    }
    else {
        udpDatagram.id = HostToNet32(id);
    }//if//

    if (useSystemTimeInPacket) {
        uint32_t tvSec;
        uint32_t tvUsec;
        bool success;
        GetTimeOfDay(tvSec, tvUsec, success);
        if (success) {
            udpDatagram.tv_sec = HostToNet32(tvSec);
            udpDatagram.tv_usec = HostToNet32(tvUsec);
        }
        else {
            assert(false && "GetTimeOfDay() failed");
            udpDatagram.tv_sec = 0;
            udpDatagram.tv_usec = 0;
        }//if//
    }
    else {
        const SimTime currentTime =
            simulationEngineInterfacePtr->CurrentTime();
        udpDatagram.tv_sec = HostToNet32(static_cast<uint32_t>(currentTime / SECOND));
        udpDatagram.tv_usec = HostToNet32((currentTime % SECOND) / MICRO_SECOND);
    }//if//

}//UpdateUdpDatagram//

void IperfUdpApplication::InitClientHeader()
{
    ClientHeader& clientHeader =
        GetHeader<ClientHeader>(buffer, sizeof(UdpDatagram));

    clientHeader.flags = 0;
    clientHeader.numThreads = 0;
    clientHeader.mPort = 0;
    clientHeader.bufferlen = 0;
    clientHeader.mWinBand = 0;
    clientHeader.mAmount = 0;

}//InitClientHeader//

inline
void IperfUdpApplication::InitServerHeader()
{
    ServerHeader& serverHeader =
        GetHeader<ServerHeader>(buffer, sizeof(UdpDatagram));

    const SimTime stopTime =
        transferInfo.endTime - transferInfo.startTime;
    const SimTime jitterTime =
        ConvertDoubleSecsToTime(transferInfo.jitter);

    serverHeader.flags = HostToNet32(HEADER_VERSION1);
    serverHeader.total_len1 = HostToNet32(0);
    serverHeader.total_len2 = HostToNet32(transferInfo.TotalLen & 0xFFFFFFFF);
    serverHeader.stop_sec = HostToNet32(static_cast<uint32_t>(stopTime / SECOND));
    serverHeader.stop_usec = HostToNet32((stopTime % SECOND) / MICRO_SECOND);
    serverHeader.error_cnt = HostToNet32(transferInfo.cntError);
    serverHeader.outorder_cnt = HostToNet32(transferInfo.cntOutofOrder);
    serverHeader.datagrams = HostToNet32(transferInfo.cntDatagrams);
    serverHeader.jitter1 = HostToNet32(static_cast<uint32_t>(jitterTime / SECOND));
    serverHeader.jitter2 = HostToNet32((jitterTime % SECOND) / MICRO_SECOND);

}//InitServerHeader//

inline
void IperfUdpApplication::InitTransferInfo()
{
    transferInfo.id = -1;
    transferInfo.cntError = 0;
    transferInfo.cntOutofOrder = 0;
    transferInfo.cntDatagrams = 0;
    transferInfo.TotalLen = 0;
    transferInfo.jitter = 0.0;
    transferInfo.transit = 0.0;
    transferInfo.startTime = ZERO_TIME;
    transferInfo.endTime = ZERO_TIME;

}//InitTransferInfo//

inline
void IperfUdpApplication::UpdateTransferInfo(const Packet& aPacket)
{
    if (transferInfo.startTime == ZERO_TIME) {
        SimTime currentTime;

        if (useSystemTimeInPacket) {
            uint32_t tvSec;
            uint32_t tvUsec;
            bool success;
            GetTimeOfDay(tvSec, tvUsec, success);
            if (success) {
                currentTime = tvSec * SECOND + tvUsec * MICRO_SECOND;
            }
            else {
                assert(false && "GetTimeOfDay() failed");
                currentTime = ZERO_TIME;
            }//if//
        }
        else {
            currentTime = simulationEngineInterfacePtr->CurrentTime();
        }//if//

        transferInfo.startTime = currentTime;
    }//if//

    const SimTime sentTime = GetSentTime(aPacket);
    const SimTime diffTime = GetDifferenceTime(sentTime);
    const double transitTime = ConvertTimeToDoubleSecs(diffTime);

    if (transferInfo.transit != 0.0) {
        double deltaTransitTime = transitTime - transferInfo.transit;
        if (transferInfo.transit < 0.0 ) {
            deltaTransitTime = -deltaTransitTime;
        }//if//
        transferInfo.jitter += (deltaTransitTime - transferInfo.jitter) / 16.0;
    }//if//
    transferInfo.transit = transitTime;

    const int id = GetId(aPacket);

    if (id == transferInfo.id + 1) {
        transferInfo.id = id;
    }
    else if (id < transferInfo.id + 1) {
        transferInfo.cntOutofOrder++;
    }
    else {
        transferInfo.cntError += id - transferInfo.id - 1;
        transferInfo.id = id;
    }//if//

    transferInfo.cntDatagrams += 1;
    transferInfo.TotalLen += aPacket.LengthBytes();

}//UpdateTransferInfo//

inline
void IperfUdpApplication::FinishTransferInfo()
{
    SimTime currentTime;

    if (useSystemTimeInPacket) {
        uint32_t tvSec;
        uint32_t tvUsec;
        bool success;
        GetTimeOfDay(tvSec, tvUsec, success);
        if (success) {
            currentTime = tvSec * SECOND + tvUsec * MICRO_SECOND;
        }
        else {
            assert(false && "GetTimeOfDay() failed");
            currentTime = ZERO_TIME;
        }//if//
    }
    else {
        currentTime = simulationEngineInterfacePtr->CurrentTime();
    }//if//

    transferInfo.endTime = currentTime;

    if (transferInfo.cntError >= transferInfo.cntOutofOrder) {
        transferInfo.cntError -= transferInfo.cntOutofOrder;
    }//if//

}//FinishTransferInfo//

inline
void IperfUdpApplication::StartClient()
{
    if (!isPortOpened) {
        transportLayerPtr->udpPtr->OpenSpecificUdpPort(
            NetworkAddress::anyAddress, sourcePortId, udpPacketHandlerPtr);
        isPortOpened = true;
    }//if//

    OutputTraceAndStatsForStart();

    InitUdpDatagram();
    InitClientHeader();
    SendPacketFromClient();

}//StartClient//

inline
void IperfUdpApplication::EndClient()
{
    UnreserveBandwidth();

}//EndClient//

inline
void IperfUdpApplication::SendPacketFromClient()
{
    UpdateUdpDatagram();

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(
            *simulationEngineInterfacePtr,
            buffer,
            static_cast<unsigned int>(bufferLength),
            useVirtualPayload);

    const NetworkAddress sourceAddress =
        networkAddressLookupInterfacePtr->LookupNetworkAddress(sourceNodeId);

    NetworkAddress destinationAddress;
    bool foundDestAddress;
    if (autoAddressModeEnabled) {
        networkAddressLookupInterfacePtr->LookupNetworkAddress(
            destinationNodeId, destinationAddress, foundDestAddress);
    }
    else {
        destinationAddress = specifiedDestinationAddress;
        foundDestAddress = true;
    }//if//

    if (foundDestAddress) {
        OutputTraceAndStatsForSendPacket(*packetPtr);
        totalSentBytes += packetPtr->LengthBytes();

        transportLayerPtr->udpPtr->SendPacket(
            packetPtr, sourceAddress, sourcePortId,
            destinationAddress, destinationPortId, priority);
    }
    else {
        //cannot find destination address (destination node may not be created yet)
        //Future: output trace and stat
        packetPtr = nullptr;
    }//if//

    const SimTime nextTime =
        simulationEngineInterfacePtr->CurrentTime() + intervalTime;

    if ((timeModeEnabled && endTime < nextTime) ||
        (!timeModeEnabled && totalSendingBytes <= totalSentBytes)) {
        SendFinPacketFromClient();
    }
    else {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new IperfUdpEvent(shared_from_this(), SEND_PACKET_FROM_CLIENT)),
            nextTime);
    }//if//

}//SendPacketFromClient//

inline
void IperfUdpApplication::SendFinPacketFromClient()
{
    UpdateUdpDatagram(true);

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(
            *simulationEngineInterfacePtr,
            buffer,
            static_cast<unsigned int>(bufferLength),
            useVirtualPayload);

    const NetworkAddress sourceAddress =
        networkAddressLookupInterfacePtr->LookupNetworkAddress(sourceNodeId);

    NetworkAddress destinationAddress;
    bool foundDestAddress;
    if (autoAddressModeEnabled) {
        networkAddressLookupInterfacePtr->LookupNetworkAddress(
            destinationNodeId, destinationAddress, foundDestAddress);
    }
    else {
        destinationAddress = specifiedDestinationAddress;
        foundDestAddress = true;
    }//if//

    if (foundDestAddress) {
        transportLayerPtr->udpPtr->SendPacket(
            packetPtr, sourceAddress, sourcePortId,
            destinationAddress, destinationPortId, priority);

        waitingAck = true;
    }
    else {
        packetPtr = nullptr;
    }//if//

}//SendFinPacketFromClient//

inline
void IperfUdpApplication::ReceivePacketOnClient(unique_ptr<Packet>& packetPtr)
{
    const ServerHeader& serverHeader = packetPtr->GetAndReinterpretPayloadData<const ServerHeader>(sizeof(UdpDatagram));
    const bool fromServer = (NetToHost32(serverHeader.flags) == HEADER_VERSION1);

    if (waitingAck && fromServer) {
        OutputTraceAndStatsForEnd(*packetPtr);
        EndClient();
    }//if//

    packetPtr = nullptr;

}//ReceivePacketOnClient//

inline
void IperfUdpApplication::StartServer()
{
    if (!isPortOpened) {
        transportLayerPtr->udpPtr->OpenSpecificUdpPort(
            NetworkAddress::anyAddress, destinationPortId, udpPacketHandlerPtr);
        isPortOpened = true;
    }//if//

    InitUdpDatagram();
    InitTransferInfo();

}//StartServer//

inline
void IperfUdpApplication::EndServer()
{
    UnreserveBandwidth();
    StartServer();

}//EndServer//

inline
void IperfUdpApplication::SendAckPacketFromServer(
    const NetworkAddress& sourceAddress,
    const unsigned short int portId)
{
    InitServerHeader();
    UpdateUdpDatagram();

    unique_ptr<Packet> ackPacketPtr =
        Packet::CreatePacket(
            *simulationEngineInterfacePtr,
            buffer,
            static_cast<unsigned int>(bufferLength),
            useVirtualPayload);

    NetworkAddress destinationAddress;
    bool foundDestAddress;
    if (autoAddressModeEnabled) {
        networkAddressLookupInterfacePtr->LookupNetworkAddress(
            destinationNodeId, destinationAddress, foundDestAddress);
    }
    else {
        destinationAddress = specifiedDestinationAddress;
        foundDestAddress = true;
    }//if//

    if (foundDestAddress) {
        transportLayerPtr->udpPtr->SendPacket(
            ackPacketPtr, destinationAddress, destinationPortId,
            sourceAddress, portId, priority);
    }
    else {
        ackPacketPtr = nullptr;
    }//if//

}//SendAckPacketFromServer//

inline
void IperfUdpApplication::ReceivePacketOnServer(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const unsigned short int portId)
{
    if (IsFinPacket(*packetPtr)) {
        FinishTransferInfo();
        SendAckPacketFromServer(sourceAddress, portId);
        EndServer();
    }
    else {
        UpdateTransferInfo(*packetPtr);
        OutputTraceAndStatsForReceivePacket(*packetPtr);
    }//if//

    packetPtr = nullptr;

}//ReceivePacketOnServer//

inline
void IperfUdpApplication::OutputTraceAndStatsForStart()
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            IperfUdpApplicationStartRecord traceData;

            traceData.timeModeEnabled = timeModeEnabled;
            if (timeModeEnabled) {
                traceData.totalTimeOrBytes = totalTime;
            }
            else {
                traceData.totalTimeOrBytes = totalSendingBytes;
            }//if//

            assert(sizeof(traceData) == IPERF_APPLICATION_UDP_START_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                GetModelName(), theApplicationId, "IperfUdpStart", traceData);
        }
        else {
            ostringstream outStream;

            if (timeModeEnabled) {
                outStream << "TotalTime= " << ConvertTimeToStringSecs(totalTime);
            }
            else {
                outStream << "TotalBytes= " << totalSendingBytes;
            }//if//

            simulationEngineInterfacePtr->OutputTrace(
                GetModelName(), theApplicationId, "IperfUdpStart", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForStart//

inline
void IperfUdpApplication::OutputTraceAndStatsForEnd(const Packet& aPacket)
{
    const ServerHeader& serverHeader = aPacket.GetAndReinterpretPayloadData<const ServerHeader>(sizeof(UdpDatagram));

    const double totalTimeInSecs =
        NetToHost32(serverHeader.stop_sec) +
            NetToHost32(serverHeader.stop_usec) / 1000000.0;
    const double jitterTimeInSecs =
        NetToHost32(serverHeader.jitter1) +
            NetToHost32(serverHeader.jitter2) / 1000000.0;
    const uint64_t totalBytes = NetToHost32(serverHeader.total_len2);
    const uint32_t totalPackets = NetToHost32(serverHeader.datagrams);
    const uint32_t errorPackets = NetToHost32(serverHeader.error_cnt);
    const uint32_t outOfOrderPackets = NetToHost32(serverHeader.outorder_cnt);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            IperfUdpApplicationEndRecord traceData;

            traceData.totalTime = ConvertDoubleSecsToTime(totalTimeInSecs);
            traceData.jitter = ConvertDoubleSecsToTime(jitterTimeInSecs);
            traceData.totalBytes = totalBytes;
            traceData.totalPackets = totalPackets;
            traceData.errorPackets = errorPackets;
            traceData.outOfOrderPackets = outOfOrderPackets;

            assert(sizeof(traceData) == IPERF_APPLICATION_UDP_END_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                GetModelName(), theApplicationId, "IperfUdpEnd", traceData);
        }
        else {
            ostringstream outStream;

            outStream << "TotalTime= " << ConvertTimeToStringSecs(ConvertDoubleSecsToTime(totalTimeInSecs));
            outStream << " TotalBytes= " << totalBytes;
            outStream << " Bps= " <<
                8 * totalBytes * SECOND / ConvertDoubleSecsToTime(totalTimeInSecs);
            outStream << " TotalPackets= " << totalPackets;
            outStream << " ErrorPackets= " << errorPackets;
            outStream << " OutOfOrderPackets= " << outOfOrderPackets;
            outStream << " Jitter= " << ConvertTimeToStringSecs(ConvertDoubleSecsToTime(jitterTimeInSecs));

            simulationEngineInterfacePtr->OutputTrace(
                GetModelName(), theApplicationId, "IperfUdpEnd", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForEnd//

inline
void IperfUdpApplication::OutputTraceAndStatsForSendPacket(const Packet& aPacket)
{
    packetsSentStatPtr->IncrementCounter();
    bytesSentStatPtr->IncrementCounter(aPacket.LengthBytes());

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationSendTraceRecord traceData;

            traceData.packetSequenceNumber = GetId(aPacket) + 1;
            traceData.sourceNodeId = aPacket.GetPacketId().GetSourceNodeId();
            traceData.destinationNodeId = destinationNodeId;
            traceData.sourceNodeSequenceNumber =
                aPacket.GetPacketId().GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == APPLICATION_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                GetModelName(), theApplicationId, "IperfUdpSend", traceData);
        }
        else {
            ostringstream outStream;

            outStream << "Seq= " << GetId(aPacket) + 1;
            outStream << " PktId= " << aPacket.GetPacketId();

            simulationEngineInterfacePtr->OutputTrace(
                GetModelName(), theApplicationId, "IperfUdpSend", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForSendPacket//

inline
void IperfUdpApplication::OutputTraceAndStatsForReceivePacket(const Packet& aPacket)
{
    const SimTime sentTime = GetSentTime(aPacket);
    const SimTime delayTime = GetDifferenceTime(sentTime);

    packetsReceived += 1;
    packetsReceivedStatPtr->IncrementCounter();
    bytesReceivedStatPtr->IncrementCounter(aPacket.LengthBytes());
    endToEndDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(delayTime));

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationReceiveTraceRecord traceData;

            traceData.packetSequenceNumber = GetId(aPacket) + 1;
            traceData.sourceNodeId = aPacket.GetPacketId().GetSourceNodeId();
            traceData.sourceNodeSequenceNumber =
                aPacket.GetPacketId().GetSourceNodeSequenceNumber();
            traceData.delay = delayTime;
            traceData.receivedPackets = packetsReceived;
            traceData.packetLengthBytes = static_cast<uint16_t>(aPacket.LengthBytes());

            assert(sizeof(traceData) == APPLICATION_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                GetModelName(), theApplicationId, "IperfUdpRecv", traceData);
        }
        else {
            ostringstream outStream;

            outStream << "Seq= " << GetId(aPacket) + 1;
            outStream << " PktId= " << aPacket.GetPacketId();
            outStream << " Delay= " << ConvertTimeToStringSecs(delayTime);
            outStream << " Pdr= " << packetsReceived << '/' << GetId(aPacket) + 1;
            outStream << " PacketBytes= " << aPacket.LengthBytes();

            simulationEngineInterfacePtr->OutputTrace(
                GetModelName(), theApplicationId, "IperfUdpRecv", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForReceivePacket//

inline
void IperfUdpApplication::ReserveBandwidth()
{
    if (!reserveBandwidthModeIsOn) {
        return;
    }//if//

    NetworkAddress sourceAddress;
    bool foundSourceAddress;

    networkAddressLookupInterfacePtr->LookupNetworkAddress(
        sourceNodeId, sourceAddress, foundSourceAddress);

    if (!foundSourceAddress) {
        return;
    }//if//

    NetworkAddress destinationAddress;
    bool foundDestAddress;

    networkAddressLookupInterfacePtr->LookupNetworkAddress(
        destinationNodeId, destinationAddress, foundDestAddress);

    if (!foundDestAddress) {
        return;
    }//if//

    const NetworkLayer& theNetworkLayer = *transportLayerPtr->GetNetworkLayerPtr();

    bool success;
    NetworkAddress nextHopAddress;
    unsigned int interfaceIndex;

    theNetworkLayer.GetNextHopAddressAndInterfaceIndexForDestination(
        destinationAddress, success, nextHopAddress, interfaceIndex);

    if (success && theNetworkLayer.MacSupportsQualityOfService(interfaceIndex)) {
        MacQualityOfServiceControlInterface& qosControlInterface =
            *theNetworkLayer.GetMacQualityOfServiceInterface(interfaceIndex);

        shared_ptr<FlowRequestReplyFielder> replyPtr(new FlowRequestReplyFielder(shared_from_this()));

        if (IsServer()) {
            qosControlInterface.RequestDownlinkFlowReservation(
                reservationScheme, schedulingScheme, priority, qosMinBandwidth, qosMaxBandwidth,
                sourceAddress, sourcePortId, destinationAddress, destinationPortId, IP_PROTOCOL_NUMBER_UDP,
                replyPtr);
        }
        else {
            qosControlInterface.RequestUplinkFlowReservation(
                reservationScheme, schedulingScheme, priority, qosMinBandwidth, qosMaxBandwidth,
                destinationAddress, destinationPortId, sourceAddress, sourcePortId, IP_PROTOCOL_NUMBER_UDP,
                replyPtr);
        }//if//
    }//if//

}//ReserveBandwidth//

inline
void IperfUdpApplication::ReserveBandwidthRequestDeniedAction()
{
    assert(reserveBandwidthModeIsOn);
    cerr << "Warning in IperfUdp with QoS application: Bandwidth request denied." << endl;

}//ReserveBandwidthRequestDeniedAction//

inline
void IperfUdpApplication::UnreserveBandwidth()
{
    if (!reserveBandwidthModeIsOn) {
        return;
    }//if//

    NetworkAddress destinationAddress;
    bool foundDestAddress;

    networkAddressLookupInterfacePtr->LookupNetworkAddress(
        destinationNodeId, destinationAddress, foundDestAddress);

    if (!foundDestAddress) {
        return;
    }//if//

    const NetworkLayer& theNetworkLayer = *transportLayerPtr->GetNetworkLayerPtr();

    bool success;
    NetworkAddress nextHopAddress;
    unsigned int interfaceIndex;

    theNetworkLayer.GetNextHopAddressAndInterfaceIndexForDestination(
        destinationAddress, success, nextHopAddress, interfaceIndex);

    if (success && theNetworkLayer.MacSupportsQualityOfService(interfaceIndex)) {
        MacQualityOfServiceControlInterface& qosControlInterface =
            *theNetworkLayer.GetMacQualityOfServiceInterface(interfaceIndex);

        qosControlInterface.DeleteFlow(macQosFlowId);
    }//if//

}//UnreserveBandwidth//

}//namespace//

#endif
