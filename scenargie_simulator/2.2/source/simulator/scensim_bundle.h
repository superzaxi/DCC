// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_BUNDLE_H
#define SCENSIM_BUNDLE_H

#include "scensim_netsim.h"
#include <queue>

namespace ScenSim {

using std::cerr;
using std::endl;
using std::queue;
using std::find;

typedef unsigned long long int BundleIdType;

class BundleMessageApplication;

//--------------------------------------------------------------------------------------------------
//bundle info to be sent as part of bundle
struct BundleHeader {
    uint64_t bundleId;
    uint32_t bundleSizeBytes;
    uint32_t targetNodeId;
    SimTime sendTime;
    SimTime expirationTime;
    uint32_t numOfCopies;
    uint32_t hopCount;

    BundleHeader() {}

    BundleHeader(
        const BundleIdType& initBundleId,
        const unsigned int initBundleSizeBytes,
        const NodeId& initTargetNodeId,
        const SimTime& initSendTime,
        const SimTime& initExpirationTime,
        const unsigned int initNumOfCopies,
        const unsigned int initHopCount)
        :
        bundleId(initBundleId),
        bundleSizeBytes(initBundleSizeBytes),
        targetNodeId(initTargetNodeId),
        sendTime(initSendTime),
        expirationTime(initExpirationTime),
        numOfCopies(initNumOfCopies),
        hopCount(initHopCount)
        { }

};

class BundleProtocol: public Application, public enable_shared_from_this<BundleProtocol> {
public:
    static const string modelName;

    BundleProtocol(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeId& initNodeId,
        const unsigned short int initDefaultDestinationPortId);

    void CompleteInitialization();

    void AddSenderSetting(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationId& theApplicationId,
        const NodeId& targetNodeId);

    ~BundleProtocol() { };

    void DisconnectFromOtherLayers() override;

    //API from traffic generator (BundleMessageApplication) to Bundle Protocol

    void ReceiveBundleSendRequest(
        const NodeId& targetNodeId,
        const SimTime& bundleLifeTime,
        const unsigned int bundleSizeBytes);

protected:
    NodeId theNodeId;

    shared_ptr<BundleMessageApplication> bundleMessageAppPtr;

    //message
    enum MessageType {
        HELLO = 0x00,
        REQUEST = 0x01,
        BUNDLE = 0x02,
        ENCOUNTER_PROB = 0x03,
        ACK = 0x04
    };

    struct PacketHeader {
        uint32_t senderNodeId;
        uint16_t type;
        uint16_t length;

        PacketHeader(
            const NodeId& initSenderNodeId,
            const MessageType initType,
            const size_t& initLength)
            :
            senderNodeId(initSenderNodeId),
            type(static_cast<uint16_t>(initType)),
            length(static_cast<uint16_t>(initLength))
            { }

    };

    //local bundle info
    struct BundleInfoType {
        BundleIdType bundleId;
        unsigned int remainingCopies;
        double cost;
        BundleHeader bundleHeader;

        BundleInfoType() {}

        BundleInfoType(
            const BundleIdType& initBundleId,
            const unsigned int initRemainingCopies,
            const BundleHeader& initBundleHeader)
            :
            bundleId(initBundleId),
            remainingCopies(initRemainingCopies),
            bundleHeader(initBundleHeader)
            { }

        bool operator==(const BundleIdType& rightBundleId) const {
            return ((*this).bundleId == rightBundleId);
        }

    };

    //properties
    unsigned int maximumCopies;
    unsigned long long int maxStorageSizeBytes;
    SimTime helloInterval;
    size_t maxControlPacketSizeBytes;

    //variables
    vector<BundleInfoType> storedBundles;
    unsigned long long int currentStrageUsageBytes;


    void GetBundleInfo(
        const BundleIdType& bundleId, bool& exists, BundleInfoType& bundleInfo);

    void DecrementStorageUsage(
        const unsigned int dataSizeBytes);

    void ScheduleControlPacketSendEvent(
        const MessageType& messageType,
        const NetworkAddress& destinationAddress,
        const vector<unsigned char>& payload);


    //routing algorithm dependend

    virtual void SubtractNumberOfCopies(
        const BundleIdType& targetBundleId) { }

    virtual void DoHelloPacketPostProcessing(
        const NodeId& sourceNodeId,
        const NetworkAddress& sourceNetworkAddress) { }

    virtual void SortRequestedBundlesIfNecessary(
        vector<BundleIdType>& requestedBundleIds) { }

    virtual void ProcessEncounterProbPacket(
        const Packet& packet,
        const int offsetBytes,
        const unsigned int messageLength,
        const NetworkAddress& sourceNetworkAddress) { assert(false); exit(1); }

    virtual void ManageStorageCapacity(const unsigned int newDataSizeBytes) { }

private:
    typedef unsigned int TcpConnectionIdType;

    unsigned short int destinationPortNumber;
    unsigned short int nextAvailableLocalPortNumber;
    PacketPriority controlPriority;
    PacketPriority dataPriority;
    bool useVirtualPayload;

    //properties
    bool deliveryAckIsEnabled;
    SimTime requestResendInterval;
    SimTime controlPacketMaxJitter;

    enum TransportModeType {
        TCP, UDP
    };

    TransportModeType transportMode;

    //variables
    set<BundleIdType> deliveredBundleIds;
    map<BundleIdType, SimTime> requestingBundleIds;


    bool TheBundleExists(const BundleIdType& bundleId) const;

    //as receiver
    //-------------------------------------------------------------------------

    //UDP
    class UdpPacketHandler: public UdpProtocol::PacketForAppFromTransportLayerHandler {
    public:
        UdpPacketHandler(
            const shared_ptr<BundleProtocol>& initAppPtr)
            :
            appPtr(initAppPtr) { }

        void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const NetworkAddress& sourceAddress,
            const unsigned short int sourcePort,
            const NetworkAddress& destinationAddress,
            const PacketPriority& priority)
        {
            appPtr->ReceiveUdpPacket(packetPtr, sourceAddress);
        }

    private:
        NoOverhead_weak_ptr<BundleProtocol> appPtr;

    };//PacketHandler//

    shared_ptr<UdpPacketHandler> udpPacketHandlerPtr;

    void ReceiveUdpPacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceNetworkAddress);

   void ProcessHelloPacket(
        const Packet& packet,
        const unsigned int messageLength,
        const NodeId& sourceNodeId,
        const NetworkAddress& sourceNetworkAddress);

    void ProcessAckInfo(
        const Packet& packet,
        const int offsetBytes,
        const unsigned int messageLength,
        const NetworkAddress& sourceNetworkAddress);

    void ProcessRequestPacket(
        const Packet& packet,
        const unsigned int messageLength,
        const NodeId& senderNodeId,
        const NetworkAddress& sourceNetworkAddress);

    void ProcessBundlePacket(
        const Packet& packet,
        const unsigned int messageLength,
        const NetworkAddress& sourceNetworkAddress);

    void ProcessBundleData(
        const NetworkAddress& sourceAddress,
        BundleHeader& bundleHeader);


    //as sender
    //-------------------------------------------------------------------------
    class SendHelloEvent: public SimulationEvent {
    public:
        explicit
        SendHelloEvent(
            const shared_ptr<BundleProtocol>& initAppPtr)
            :
            appPtr(initAppPtr)
        {}

        void ExecuteEvent()
        {
            appPtr->SendHello();

        }//ExecuteEvent//

    private:
        NoOverhead_weak_ptr<BundleProtocol> appPtr;

    };//SendHelloEvent//

    class ControlPacketSendEvent: public SimulationEvent {
    public:
        explicit
        ControlPacketSendEvent(
            const shared_ptr<BundleProtocol> initAppPtr,
            const MessageType& initMessageType,
            const NetworkAddress& initDestinationAddress,
            const vector<unsigned char>& initPayload)
            :
            appPtr(initAppPtr),
            messageType(initMessageType),
            destinationAddress(initDestinationAddress),
            payload(initPayload)
        {}

        void ExecuteEvent()
        {
            appPtr->SendControlPacket(
                messageType,
                destinationAddress,
                payload);

        }//ExecuteEvent//

    private:
        NoOverhead_weak_ptr<BundleProtocol> appPtr;
        MessageType messageType;
        NetworkAddress destinationAddress;
        vector<unsigned char> payload;

    };//ControlPacketSendEvent//


    shared_ptr<SendHelloEvent> sendHelloEventPtr;

    EventRescheduleTicket sendHelloEventTicket;
    SimTime helloStartTimeJitter;

    unsigned int currentSequenceNumber;

    BundleIdType GenerateNewBundleId();

    NodeId ExtractOriginatorNodeId(
        const BundleIdType& bundleId) const;

    unsigned int ExtractOriginatorSequenceNumber(
        const BundleIdType& bundleId) const;

    void IncrementStorageUsage(
        const unsigned int dataSizeBytes);

    void StoreBundle(
        const BundleInfoType& bundle,
        bool& success);

    void DiscardExpiredBundles();

    void SendHello();

    void SendControlPacket(
        const MessageType& messageType,
        const NetworkAddress& destinationAddress,
        const vector<unsigned char>& payload);

    void SendBundleIdInfo(
        const MessageType& messageType,
        const vector<BundleIdType>& bundleIdsForSend,
        const NetworkAddress& destinationAddress);

    void SendBundleWithUdp(
        const vector<BundleIdType>& bundleIds,
        const NetworkAddress& destinationNetworkAddress);


    //TCP
    class TcpConnectionHandler: public ConnectionFromTcpProtocolHandler {
    public:
        TcpConnectionHandler(
            const shared_ptr<BundleProtocol>& initAppPtr)
            :
            appPtr(initAppPtr)
        { }

        void HandleNewConnection(const shared_ptr<TcpConnection>& newTcpConnectionPtr)
        {
            appPtr->AcceptTcpConnection(newTcpConnectionPtr);
        }//HandleNewConnection//

    private:
        NoOverhead_weak_ptr<BundleProtocol> appPtr;

    };//TcpConnectionHandler//


    class TcpEventHandler: public TcpConnection::AppTcpEventHandler {
    public:
        TcpEventHandler(
            const shared_ptr<BundleProtocol>& initAppPtr,
            const TcpConnectionIdType& initConnectionId)
            :
            appPtr(initAppPtr),
            connectionId(initConnectionId)
        { }

        void DoTcpIsReadyForMoreDataAction()
        {
           appPtr->DoTcpIsReadyForMoreDataAction(connectionId);

        }//DoTcpIsReadyForMoreDataAction//

        void ReceiveDataBlock(
            const unsigned char dataBlock[],
            const unsigned int dataLength,
            const unsigned int actualDataLength,
            bool& stallIncomingDataFlow)
        {
            appPtr->ReceiveDataBlock(connectionId, dataBlock, dataLength, actualDataLength);
            stallIncomingDataFlow = false;

        }//ReceiveDataBlock//

        void DoTcpRemoteHostClosedAction()
        {
            appPtr->DoTcpRemoteHostClosedAction(connectionId);

        }//DoTcpRemoteHostClosedAction//

        void DoTcpLocalHostClosedAction()
        {
            appPtr->DoTcpLocalHostClosedAction(connectionId);

        }//DoTcpLocalHostClosedAction//

    private:
        NoOverhead_weak_ptr<BundleProtocol> appPtr;
        TcpConnectionIdType connectionId;

    };//TcpEventHandler//


    void AcceptTcpConnection(
        const shared_ptr<TcpConnection>& connectionPtr);
    shared_ptr<TcpConnectionHandler> tcpConnectionHandlerPtr;

    enum TcpStateType {
        CLOSE,
        WAITING_FOR_ESTABLISHING,
        ESTABLISHED,
        WAITING_FOR_CLOSING
    };//TcpStateType//

    struct TcpConnectionInfoType {
        shared_ptr<TcpEventHandler> tcpEventHandlerPtr;
        shared_ptr<TcpConnection> tcpConnectionPtr;
        TcpStateType tcpState;
        NetworkAddress destinationAddress;
        size_t sendingBundleIndex;
        vector<BundleIdType> sendingBundleIds;
        vector<unsigned int> sendingBundleSizeBytes;
        vector<shared_ptr<vector<unsigned char> > > sendingData;
        unsigned long long int tcpAccumulatedDeliveredBytes;
        unsigned long long int totalSendingDataBytes;
        unsigned long long int bundleSizeBytesToBeReceived;
        unsigned long long int receivedDataBytes;
        vector<unsigned char> receivedData;
        BundleHeader receivingBundleHeader;

        TcpConnectionInfoType()
            :
            tcpEventHandlerPtr(),
            tcpConnectionPtr(),
            tcpState(CLOSE),
            destinationAddress(),
            sendingBundleIndex(0),
            tcpAccumulatedDeliveredBytes(0),
            totalSendingDataBytes(0),
            receivedDataBytes(0),
            bundleSizeBytesToBeReceived(0),
            receivedData()
        {}
    };//TcpConnectionInfoType//

    TcpConnectionIdType nextTcpConnectionId;

    map<TcpConnectionIdType, TcpConnectionInfoType> tcpConnectionDatabase;

    void DeleteTcpConnectionInfo(const TcpConnectionIdType& connectionId);

    void RegisterBundleToTcpConnectionInfo(
        const vector<BundleIdType>& bundleIds,
        TcpConnectionInfoType& tcpConnectionInfo);

    void SendBundleWithTcp(
        const vector<BundleIdType>& bundleIds,
        const NetworkAddress& destinationNetworkAddress);


    void DoTcpIsReadyForMoreDataAction(const TcpConnectionIdType& connectionId);

    void SendDataBlock(const TcpConnectionIdType& connectionId);

    void ReceiveDataBlock(
        const TcpConnectionIdType& connectionId,
        const unsigned char dataBlock[],
        const unsigned int dataLength,
        const unsigned int actualDataLength);

    void DoTcpRemoteHostClosedAction(const TcpConnectionIdType& connectionId);

    void DoTcpLocalHostClosedAction(const TcpConnectionIdType& connectionId);

    TcpConnectionInfoType& GetRefToTcpConnectionInfo(const TcpConnectionIdType& connectionId);


    //trace and stats

    //at originator or target node
    shared_ptr<CounterStatistic> bundlesGeneratedStatPtr;
    shared_ptr<CounterStatistic> bundleGenerationFailedDueToLackOfStorageStatPtr;
    shared_ptr<CounterStatistic> bytesGeneratedStatPtr;
    shared_ptr<CounterStatistic> bundlesDeliveredStatPtr;
    shared_ptr<CounterStatistic> bytesDeliveredStatPtr;
    shared_ptr<RealStatistic> bundleEndToEndDelayStatPtr;

    //at any nodes
    shared_ptr<CounterStatistic> bundlesSentStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<CounterStatistic> bundlesReceivedStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;
    shared_ptr<CounterStatistic> duplicateBundleReceivedStatPtr;
    shared_ptr<CounterStatistic> bundlesDiscardedDueToLackOfStorageStatPtr;
    shared_ptr<RealStatistic> storageUsageBytesStatPtr;


    void OutputTraceAndStatForStorageUsage();

    void OutputTraceForControlPacketSend(
        const MessageType& messageType,
        const NetworkAddress& destinationAddress);

    void OutputTraceForControlPacketReceive(
        const MessageType& messageType,
        const NetworkAddress& sourceAddress);

    void OutputTraceAndStatsForGenerateBundle(
        const BundleIdType& bundleId,
        const NodeId& targetNodeId,
        const size_t& bundleSizeBytes);

    void OutputTraceAndStatsForDeliveryBundle(
        const BundleIdType& bundleId,
        const NodeId& targetNodeId,
        const size_t& bundleSizeBytes,
        const SimTime& delay);

    void OutputTraceAndStatsForSendBundle(
        const NetworkAddress& nextNodeAddress,
        const BundleIdType& bundleId,
        const NodeId& targetNodeId,
        const size_t& bundleSizeBytes);

    void OutputTraceAndStatsForReceiveBundle(
        const NetworkAddress& lastNodeAddress,
        const BundleIdType& bundleId,
        const NodeId& targetNodeId,
        const size_t& bundleSizeBytes);

};

//-------------------------------------------------------------

class BundleMessageApplication: public enable_shared_from_this<BundleMessageApplication> {
public:

    BundleMessageApplication(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<BundleProtocol>& initBundleProtocolPtr);

    void AddMessageGenerationSetting(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const ApplicationId& theApplicationId,
        const shared_ptr<RandomNumberGenerator>& aRandomNumberGeneratorPtr,
        const NodeId& targetNodeId);

    //API from Bundle Protocol to Bundle Message

    void ReceiveBundle(
        const BundleHeader& bundleHeader) const {
        //do something
    }


private:
    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    shared_ptr<BundleProtocol> bundleProtocolPtr;

    void CreateBundle(
        const NodeId targetNodeId,
        const SimTime& bundleLifeTime,
        const unsigned int bundleSizeBytes,
        const SimTime& bundleEndTime,
        const SimTime& bundleInterval);

    class CreateBundleEvent: public SimulationEvent {
    public:
        explicit
        CreateBundleEvent(
            const shared_ptr<BundleMessageApplication>& initAppPtr,
            const NodeId initTargetNodeId,
            const SimTime& initBundleLifeTime,
            const unsigned int& initBundleSizeBytes,
            const SimTime& initBundleEndTime,
            const SimTime& initBundleInterval)
            :
            appPtr(initAppPtr),
            targetNodeId(initTargetNodeId),
            bundleLifeTime(initBundleLifeTime),
            bundleSizeBytes(initBundleSizeBytes),
            bundleEndTime(initBundleEndTime),
            bundleInterval(initBundleInterval)
        {}

        void ExecuteEvent()
        {
            appPtr->CreateBundle(targetNodeId, bundleLifeTime, bundleSizeBytes, bundleEndTime, bundleInterval);

        }//ExecuteEvent//

    private:
        NoOverhead_weak_ptr<BundleMessageApplication> appPtr;
        NodeId targetNodeId;
        SimTime bundleLifeTime;
        unsigned int bundleSizeBytes;
        SimTime bundleEndTime;
        SimTime bundleInterval;

    };//CreateBundleEvent//

};


inline
BundleMessageApplication::BundleMessageApplication(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<BundleProtocol>& initBundleProtocolPtr)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    bundleProtocolPtr(initBundleProtocolPtr)
{


}//BundleMessageApplication//


inline
void BundleMessageApplication::AddMessageGenerationSetting(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const ApplicationId& theApplicationId,
    const shared_ptr<RandomNumberGenerator>& aRandomNumberGeneratorPtr,
    const NodeId& targetNodeId)
{

    SimTime bundleLifeTime = INFINITE_TIME;
    if (theParameterDatabaseReader.ParameterExists("bundle-message-lifetime", theNodeId, theApplicationId)) {
        bundleLifeTime =
            theParameterDatabaseReader.ReadTime("bundle-message-lifetime", theNodeId, theApplicationId);
    }//if//

    SimTime messageSendTimeJitter = ZERO_TIME;
    if (theParameterDatabaseReader.ParameterExists(
        "bundle-message-max-jitter", theNodeId, theApplicationId)) {

        const SimTime messageSendTimeMaxJitter =
            theParameterDatabaseReader.ReadTime(
                "bundle-message-max-jitter", theNodeId, theApplicationId);

        messageSendTimeJitter =
            static_cast<SimTime>(aRandomNumberGeneratorPtr->GenerateRandomDouble() * messageSendTimeMaxJitter);

    }//if//

    SimTime messageSendTime =
        theParameterDatabaseReader.ReadTime(
            "bundle-message-start-time", theNodeId, theApplicationId);

    messageSendTime += messageSendTimeJitter;

    const SimTime messageEndTime =
        theParameterDatabaseReader.ReadTime(
            "bundle-message-end-time", theNodeId, theApplicationId);

    const SimTime messageSendInterval =
        theParameterDatabaseReader.ReadTime(
            "bundle-message-send-interval", theNodeId, theApplicationId);

    const unsigned int messageSizeBytes =
        theParameterDatabaseReader.ReadNonNegativeInt(
            "bundle-message-size-bytes", theNodeId, theApplicationId);

    if (messageSizeBytes < sizeof(BundleHeader)) {

        cerr << "'Error: bundle-message-size-bytes = " << messageSizeBytes
            << " should be larger than " << sizeof(BundleHeader) << endl;
        exit(1);

    }//if//

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (currentTime > messageSendTime) {
        const size_t nextTransmissionTime = size_t(ceil(double(currentTime - messageSendTime) / messageSendInterval));
        messageSendTime += nextTransmissionTime * messageSendInterval;
    }//if//

    if (messageSendTime < messageEndTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(
                new CreateBundleEvent(
                    shared_from_this(), targetNodeId, bundleLifeTime, messageSizeBytes,
                    messageEndTime, messageSendInterval)), messageSendTime);
    }//if//

}//AddMessageCreationSetting//


inline
void BundleMessageApplication::CreateBundle(
    const NodeId targetNodeId,
    const SimTime& bundleLifeTime,
    const unsigned int bundleSizeBytes,
    const SimTime& bundleEndTime,
    const SimTime& bundleInterval)
{

    bundleProtocolPtr->ReceiveBundleSendRequest(
        targetNodeId, bundleLifeTime, bundleSizeBytes);

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    //schedule next event
    if (bundleInterval <= (INFINITE_TIME - currentTime)) {

        const SimTime nextEventTime = currentTime + bundleInterval;
        if (nextEventTime < bundleEndTime) {
            simulationEngineInterfacePtr->ScheduleEvent(
                unique_ptr<SimulationEvent>(
                    new CreateBundleEvent(
                        shared_from_this(), targetNodeId, bundleLifeTime, bundleSizeBytes,
                        bundleEndTime, bundleInterval)), nextEventTime);
        }
    }//if//

}//CreateBundle//


inline
BundleProtocol::BundleProtocol(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeId& initNodeId,
    const unsigned short int initDefaultDestinationPortId)
    :
    Application(initSimulationEngineInterfacePtr, modelName),
    theNodeId(initNodeId),
    helloInterval(INFINITE_TIME),
    helloStartTimeJitter(ZERO_TIME),
    maxControlPacketSizeBytes(1472),
    controlPriority(0),
    dataPriority(0),
    destinationPortNumber(initDefaultDestinationPortId),
    nextAvailableLocalPortNumber(0),
    maxStorageSizeBytes(ULLONG_MAX),
    currentStrageUsageBytes(0),
    maximumCopies(UINT_MAX),
    transportMode(TCP),
    deliveryAckIsEnabled(false),
    currentSequenceNumber(0),
    requestResendInterval(2 * SECOND),
    controlPacketMaxJitter(ZERO_TIME),
    useVirtualPayload(false),
    nextTcpConnectionId(0),
    bundlesGeneratedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BundlesGenerated"))),
    bundleGenerationFailedDueToLackOfStorageStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BundleGenerationFailedDueToLackOfStorage"))),
    bytesGeneratedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BytesGenerated"))),
    bundlesDeliveredStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BundlesDelivered"))),
    bytesDeliveredStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BytesDelivered"))),
    bundleEndToEndDelayStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_BundleEndToEndDelay"))),
    bundlesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BundlesSent"))),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BytesSent"))),
    bundlesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BundlesReceived"))),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BytesReceived"))),
    duplicateBundleReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_DuplicateBundleReceived"))),
    bundlesDiscardedDueToLackOfStorageStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BundlesDiscardedDueToLackOfStorage"))),
    storageUsageBytesStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_StorageUsageBytes")))
{

    nextAvailableLocalPortNumber = destinationPortNumber + 1;

    if (theParameterDatabaseReader.ParameterExists("bundle-max-storage-size-bytes", theNodeId)) {
        maxStorageSizeBytes =
            theParameterDatabaseReader.ReadBigInt("bundle-max-storage-size-bytes", theNodeId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("bundle-use-virtual-payload", theNodeId)) {
        useVirtualPayload =
            theParameterDatabaseReader.ReadBool("bundle-use-virtual-payload", theNodeId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("bundle-transport-mode", theNodeId)) {

        const string transportModeString =
            MakeLowerCaseString(theParameterDatabaseReader.ReadString("bundle-transport-mode", theNodeId));

        if (transportModeString == "tcp") {
            transportMode = TCP;
        }
        else if (transportModeString == "udp") {
            transportMode = UDP;
        }
        else {
            cerr << "Error: bundle-transport-mode = " << transportModeString << " is unknown." << endl;
            exit(1);
        }//if//

    }//if//

    if (theParameterDatabaseReader.ParameterExists("bundle-enable-delivery-ack", theNodeId)) {

        deliveryAckIsEnabled =
            theParameterDatabaseReader.ReadBool("bundle-enable-delivery-ack", theNodeId);

    }//if//

    helloInterval =
        theParameterDatabaseReader.ReadTime("bundle-hello-interval", theNodeId);

    if (theParameterDatabaseReader.ParameterExists(
            "bundle-hello-max-jitter", theNodeId)) {

        helloStartTimeJitter =
            theParameterDatabaseReader.ReadTime(
                "bundle-hello-max-jitter", theNodeId);

    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            "bundle-request-resend-interval", theNodeId)) {

        requestResendInterval =
            theParameterDatabaseReader.ReadTime(
                "bundle-request-resend-interval", theNodeId);

    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            "bundle-control-packet-max-jitter", theNodeId)) {

        controlPacketMaxJitter =
            theParameterDatabaseReader.ReadTime(
                "bundle-control-packet-max-jitter", theNodeId);

    }//if//

    if (theParameterDatabaseReader.ParameterExists("bundle-data-packet-priority", theNodeId, theApplicationId)) {
        dataPriority = static_cast<PacketPriority>(
            theParameterDatabaseReader.ReadNonNegativeInt("bundle-data-packet-priority", theNodeId, theApplicationId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists("bundle-control-packet-priority", theNodeId, theApplicationId)) {
        controlPriority = static_cast<PacketPriority>(
            theParameterDatabaseReader.ReadNonNegativeInt("bundle-control-packet-priority", theNodeId, theApplicationId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            "bundle-max-control-packet-size-bytes", theNodeId)) {

        maxControlPacketSizeBytes =
            theParameterDatabaseReader.ReadNonNegativeInt(
                "bundle-max-control-packet-size-bytes", theNodeId);

    }//if//

}//BundleProtocol//


inline
void BundleProtocol::DisconnectFromOtherLayers()
{
    bundleMessageAppPtr.reset();

    transportLayerPtr.reset();
    applicationLayerPtr.reset();

}//DisconnectFromOtherLayers//


inline
void BundleProtocol::CompleteInitialization()
{

    //set application (traffic generator)
    bundleMessageAppPtr =
        shared_ptr<BundleMessageApplication>(new BundleMessageApplication(simulationEngineInterfacePtr, shared_from_this()));


    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
    SimTime helloStartTime = currentTime + helloInterval;

    if (helloStartTimeJitter !=  ZERO_TIME) {
        helloStartTime += static_cast<SimTime>(
            aRandomNumberGeneratorPtr->GenerateRandomDouble() * helloStartTimeJitter);
    }//if//

    sendHelloEventPtr = shared_ptr<SendHelloEvent>(new SendHelloEvent(shared_from_this()));

    simulationEngineInterfacePtr->ScheduleEvent(sendHelloEventPtr, helloStartTime, sendHelloEventTicket);


    //as receiver
    udpPacketHandlerPtr = shared_ptr<UdpPacketHandler>(new UdpPacketHandler(shared_from_this()));

    assert(transportLayerPtr->udpPtr->PortIsAvailable(destinationPortNumber));

    transportLayerPtr->udpPtr->OpenSpecificUdpPort(
        NetworkAddress::anyAddress,
        destinationPortNumber,
        udpPacketHandlerPtr);

    tcpConnectionHandlerPtr =
        shared_ptr<TcpConnectionHandler>(new TcpConnectionHandler(shared_from_this()));

    assert(transportLayerPtr->tcpPtr->PortIsAvailable(destinationPortNumber));

    transportLayerPtr->tcpPtr->OpenSpecificTcpPort(
        NetworkAddress::anyAddress,
        destinationPortNumber,
        tcpConnectionHandlerPtr);

    OutputTraceAndStatForStorageUsage();

}//CompleteInitialization//

inline
BundleProtocol::TcpConnectionInfoType&
BundleProtocol::GetRefToTcpConnectionInfo(const TcpConnectionIdType& connectionId)
{
    typedef map<TcpConnectionIdType, TcpConnectionInfoType>::iterator IterType;
    IterType iter = tcpConnectionDatabase.find(connectionId);
    assert(iter != tcpConnectionDatabase.end());
    return (iter->second);
}



inline
void BundleProtocol::AddSenderSetting(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationId& appIdForBundleMessageApp,
    const NodeId& targetNodeId)
{

    bundleMessageAppPtr->AddMessageGenerationSetting(
        theParameterDatabaseReader,
        theNodeId,
        appIdForBundleMessageApp,
        aRandomNumberGeneratorPtr,
        targetNodeId);

}//AddSenderSetting//


inline
void BundleProtocol::GetBundleInfo(
    const BundleIdType& bundleId,
    bool& exists,
    BundleInfoType& bundleInfo)
{

    exists = false;

    vector<BundleInfoType>::iterator iter =
        find(storedBundles.begin(), storedBundles.end(), bundleId);

    if (iter != storedBundles.end()) {

        exists = true;
        bundleInfo = (*iter);

    }//if//

}//GetBundleInfo//


inline
bool BundleProtocol::TheBundleExists(const BundleIdType& bundleId) const
{

    vector<BundleInfoType>::const_iterator iter =
        find(storedBundles.begin(), storedBundles.end(), bundleId);

    return (iter != storedBundles.end());

}//TheBundleExists//


inline
BundleIdType BundleProtocol::GenerateNewBundleId()
{
    currentSequenceNumber++;

    assert(currentSequenceNumber < UINT_MAX);

    BundleIdType nodeIdPart = theNodeId;
    nodeIdPart = nodeIdPart << 32;
    return (nodeIdPart + currentSequenceNumber);

}//GenerateNewBundleId//


inline
NodeId BundleProtocol::ExtractOriginatorNodeId(
    const BundleIdType& bundleId) const
{
    return static_cast<NodeId>(bundleId / UINT_MAX);

}//ExtractOriginatorNodeId//


inline
unsigned int BundleProtocol::ExtractOriginatorSequenceNumber(
    const BundleIdType& bundleId) const
{

    return static_cast<unsigned int>(bundleId);

}//ExtractSequcenceNumber//


inline
void BundleProtocol::IncrementStorageUsage(
    const unsigned int dataSizeBytes)
{

    assert((*this).currentStrageUsageBytes <= (ULLONG_MAX - dataSizeBytes));

    (*this).currentStrageUsageBytes += dataSizeBytes;

    OutputTraceAndStatForStorageUsage();

}//IncrementStorageUsage//


inline
void BundleProtocol::DecrementStorageUsage(
    const unsigned int dataSizeBytes)
{

    assert((*this).currentStrageUsageBytes >= dataSizeBytes);

    (*this).currentStrageUsageBytes -= dataSizeBytes;

    OutputTraceAndStatForStorageUsage();

}//DecrementStorageUsage//


inline
void BundleProtocol::ScheduleControlPacketSendEvent(
    const MessageType& messageType,
    const NetworkAddress& destinationAddress,
    const vector<unsigned char>& payload)
{

    assert((messageType == ACK) ||
        (messageType == ENCOUNTER_PROB) ||
        (messageType == REQUEST));

    const SimTime eventTime =
        simulationEngineInterfacePtr->CurrentTime() +
        static_cast<SimTime>(aRandomNumberGeneratorPtr->GenerateRandomDouble() * controlPacketMaxJitter);

    simulationEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(
            new ControlPacketSendEvent(
                shared_from_this(), messageType, destinationAddress, payload)), eventTime);


}//ScheduleControlPacketSendEvent//


inline
void BundleProtocol::SendControlPacket(
    const MessageType& messageType,
    const NetworkAddress& destinationAddress,
    const vector<unsigned char>& payload)
{

    if (payload.size() > USHRT_MAX) {
        cerr << "Too big message: " << payload.size() << endl;
        exit(1);
    }//if//

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(*simulationEngineInterfacePtr, payload);

    OutputTraceForControlPacketSend(messageType, destinationAddress);

    transportLayerPtr->udpPtr->SendPacket(
        packetPtr, 0, destinationAddress, destinationPortNumber, controlPriority);

}//SendControlPacket//


inline
void BundleProtocol::StoreBundle(
    const BundleInfoType& bundle,
    bool& success)
{

    //discard expired bundles before storing bundle
    DiscardExpiredBundles();

    const unsigned int dataSizeBytes = static_cast<unsigned int>(bundle.bundleHeader.bundleSizeBytes);

    //if capacity is over
    if (((*this).currentStrageUsageBytes + dataSizeBytes) > (*this).maxStorageSizeBytes) {

        ManageStorageCapacity(dataSizeBytes);

    }//if//

    //check storage capacity
    if (((*this).currentStrageUsageBytes + dataSizeBytes) <= (*this).maxStorageSizeBytes) {

        storedBundles.push_back(bundle);

        //update strage usage
        IncrementStorageUsage(dataSizeBytes);

        success = true;

    }
    else {
        //over capacity, cannot store the bundle
        success = false;
    }//if//

}//StoreBundle//


inline
void BundleProtocol::ReceiveBundleSendRequest(
    const NodeId& targetNodeId,
    const SimTime& bundleLifeTime,
    const unsigned int bundleSizeBytes)
{

    const BundleIdType newBundleId = (*this).GenerateNewBundleId();

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    SimTime bundleExpirationTime;
    if (bundleLifeTime > (INFINITE_TIME - currentTime)) {
        bundleExpirationTime = INFINITE_TIME;
    }
    else {
        bundleExpirationTime = currentTime + bundleLifeTime;
    }//if//

    const BundleHeader bundleHeader(newBundleId, bundleSizeBytes, targetNodeId, currentTime, bundleExpirationTime, maximumCopies, 0);

    const BundleInfoType bundle(newBundleId, maximumCopies, bundleHeader);

    (*this).OutputTraceAndStatsForGenerateBundle(newBundleId, targetNodeId, bundleSizeBytes);

    bool success = false;
    StoreBundle(bundle, success);

    if (!success) {
        bundleGenerationFailedDueToLackOfStorageStatPtr->IncrementCounter();
    }//if//

}//ReceiveBundleSendRequest//


inline
void BundleProtocol::DiscardExpiredBundles()
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    unsigned int i = 0;
    while (i < storedBundles.size()) {
        const BundleInfoType& bundle = storedBundles[i];

        if (bundle.bundleHeader.expirationTime <= currentTime) {
            //update strage usage
            DecrementStorageUsage(bundle.bundleHeader.bundleSizeBytes);
            storedBundles.erase(storedBundles.begin() + i);
        }
        else {
            i++;
        }//if//
    }//while//

}//DiscardExpiredBundles//


inline
void BundleProtocol::SendHello()
{
    sendHelloEventTicket.Clear();

    //discard expired bundles before sending hello with bundle info
    DiscardExpiredBundles();

    vector<BundleIdType> transferableBundleIds;

    for(size_t i = 0; i < storedBundles.size(); i++) {

        const BundleInfoType& bundleInfo = storedBundles[i];

        //check remaining copies:
        //0: cannot transfer, 1: can transfer to final destination, 2: can transfer to anytime
        if (bundleInfo.remainingCopies < 1) continue;

        transferableBundleIds.push_back(bundleInfo.bundleId);

    }//for//


    if (!transferableBundleIds.empty()) {
        //send hello with transferable bundle IDs
        SendBundleIdInfo(HELLO, transferableBundleIds, NetworkAddress::broadcastAddress);
    }
    else {
        //send hello without bundle IDs (there is no transferable bundle)
        vector<unsigned char> payload;
        payload.resize(sizeof(PacketHeader));

        PacketHeader packetHeader(theNodeId, HELLO, 0);
        memcpy(&payload[0], &packetHeader, sizeof(PacketHeader));
        SendControlPacket(HELLO, NetworkAddress::broadcastAddress, payload);

    }//if//


    //schedule next hello
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
    simulationEngineInterfacePtr->ScheduleEvent(
        sendHelloEventPtr, currentTime + helloInterval, sendHelloEventTicket);

}//SendHello//


inline
void BundleProtocol::SendBundleIdInfo(
    const MessageType& messageType,
    const vector<BundleIdType>& bundleIdsForSend,
    const NetworkAddress& destinationAddress)
{

    assert(!bundleIdsForSend.empty());

    vector<unsigned char> payload;
    size_t payloadPosition = sizeof(PacketHeader);
    payload.resize(payloadPosition);
    size_t remainingPayloadSizeBytes = maxControlPacketSizeBytes - payloadPosition;

    if (maxControlPacketSizeBytes < (sizeof(PacketHeader) + sizeof(BundleIdType))) {
        cout << "Error: cannot pack bundle ID info in a packet" << endl;
        cout << "Set bundle-max-control-packet-size-bytes is equal to or greater than "
            << (sizeof(PacketHeader) + sizeof(BundleIdType)) << endl;
        exit(1);
    }//if//

    for(size_t i = 0; i < bundleIdsForSend.size(); i++) {

        if (remainingPayloadSizeBytes < sizeof(BundleIdType)) {

            assert(payload.size() > sizeof(PacketHeader));

            PacketHeader pakcetHeader(theNodeId, messageType, (payload.size() - sizeof(PacketHeader)));
            memcpy(&payload[0], &pakcetHeader, sizeof(PacketHeader));

            if (messageType == HELLO) {
                SendControlPacket(messageType, destinationAddress, payload);
            }
            else {
                ScheduleControlPacketSendEvent(messageType, destinationAddress, payload);
            }//if//

            //reset
            payloadPosition = sizeof(PacketHeader);
            payload.clear();
            payload.resize(payloadPosition);
            remainingPayloadSizeBytes = maxControlPacketSizeBytes - payloadPosition;

        }//if//

        assert(remainingPayloadSizeBytes >= sizeof(BundleIdType));

        //insert request bundle info
        remainingPayloadSizeBytes -= sizeof(BundleIdType);
        payload.resize(payloadPosition + sizeof(BundleIdType));
        memcpy(&payload[payloadPosition], &bundleIdsForSend[i], sizeof(BundleIdType));
        payloadPosition += sizeof(BundleIdType);

    }//for//

    if (payload.size() > sizeof(PacketHeader)) {

        //send last packet

        PacketHeader pakcetHeader(theNodeId, messageType, (payload.size() - sizeof(PacketHeader)));
        memcpy(&payload[0], &pakcetHeader, sizeof(PacketHeader));

        if (messageType == HELLO) {
            SendControlPacket(messageType, destinationAddress, payload);
        }
        else {
            ScheduleControlPacketSendEvent(messageType, destinationAddress, payload);
        }//if//

    }//if//

}//SendBundleIdInfo//


inline
void BundleProtocol::SendBundleWithUdp(
    const vector<BundleIdType>& bundleIds,
    const NetworkAddress& destinationNetworkAddress)
{

    assert(transportMode == UDP);

    for(size_t i = 0; i < bundleIds.size(); i++) {

        const BundleIdType bundleId = bundleIds[i];

        if (!TheBundleExists(bundleId)) {
            //already deleted
            continue;
        }//if//

        //subtract num of copies
        SubtractNumberOfCopies(bundleId);

        bool exists;
        BundleInfoType bundleInfo;
        GetBundleInfo(bundleId, exists, bundleInfo);
        assert(exists);

        if (bundleInfo.bundleHeader.bundleSizeBytes > USHRT_MAX) {
            cerr << "Too big bundle message for UDP packet: " << bundleInfo.bundleHeader.bundleSizeBytes << endl;
            exit(1);
        }//if//

        PacketHeader packetHeader(theNodeId, BUNDLE, bundleInfo.bundleHeader.bundleSizeBytes);

        unique_ptr<Packet> packetPtr =
            Packet::CreatePacket(*simulationEngineInterfacePtr, bundleInfo.bundleHeader,
            bundleInfo.bundleHeader.bundleSizeBytes, useVirtualPayload);

        OutputTraceAndStatsForSendBundle(
            destinationNetworkAddress, bundleId, bundleInfo.bundleHeader.targetNodeId, (*packetPtr).LengthBytes());

        packetPtr->AddPlainStructHeader(packetHeader);

        transportLayerPtr->udpPtr->SendPacket(
            packetPtr, 0, destinationNetworkAddress, destinationPortNumber, dataPriority);

    }//for//

}//SendBundleWithUdp//


inline
void BundleProtocol::DeleteTcpConnectionInfo(const TcpConnectionIdType& connectionId)
{
    tcpConnectionDatabase.erase(connectionId);

}//DeleteTcpConnectionInfo//


inline
void BundleProtocol::RegisterBundleToTcpConnectionInfo(
    const vector<BundleIdType>& bundleIds,
    TcpConnectionInfoType& tcpConnectionInfo)
{

    for(size_t i = 0; i < bundleIds.size(); i++) {

        const BundleIdType bundleId = bundleIds[i];

         if (!TheBundleExists(bundleId)) {
            //already deleted
            continue;
        }//if//

        //subtract num of copies
        SubtractNumberOfCopies(bundleId);

        bool exists;
        BundleInfoType bundleInfo;
        GetBundleInfo(bundleId, exists, bundleInfo);
        assert(exists);

        const unsigned int bundleSizeBytes = bundleInfo.bundleHeader.bundleSizeBytes;
        const unsigned int bundleHeaderSizeBytes = sizeof(bundleInfo.bundleHeader);

        assert(sizeof(bundleInfo.bundleHeader) == sizeof(BundleHeader));

        //copy info
        tcpConnectionInfo.sendingBundleIds.push_back(bundleId);
        tcpConnectionInfo.sendingBundleSizeBytes.push_back(bundleSizeBytes);

        shared_ptr<vector<unsigned char> > bundleDataPtr(new vector<unsigned char>(bundleHeaderSizeBytes));
        memcpy(&(*bundleDataPtr)[0], &(bundleInfo.bundleHeader), sizeof(BundleHeader));

        tcpConnectionInfo.sendingData.push_back(bundleDataPtr);
        tcpConnectionInfo.totalSendingDataBytes += bundleSizeBytes;

        OutputTraceAndStatsForSendBundle(
            tcpConnectionInfo.destinationAddress,
            bundleId, bundleInfo.bundleHeader.targetNodeId, bundleSizeBytes);

    }//for//

}//RegisterBundleToTcpConnectionInfo//


inline
void BundleProtocol::SendBundleWithTcp(
    const vector<BundleIdType>& bundleIds,
    const NetworkAddress& destinationNetworkAddress)
{
    assert(transportMode == TCP);
    assert(destinationNetworkAddress != NetworkAddress::invalidAddress);

    shared_ptr<TcpEventHandler> tcpHandler;

    typedef map<TcpConnectionIdType, TcpConnectionInfoType>::iterator IterType;

    bool wasFound = false;
    TcpConnectionIdType connectionId;

    for (IterType iter = tcpConnectionDatabase.begin(); iter != tcpConnectionDatabase.end(); ++iter) {

        if (iter->second.destinationAddress == destinationNetworkAddress) {

            connectionId = iter->first;
            wasFound = true;

            break;
        }//if//
    }//for//


    if (!wasFound) {

        //new connection

        TcpConnectionInfoType tcpConnectionInfo;

        tcpConnectionInfo.tcpEventHandlerPtr =
            shared_ptr<TcpEventHandler>(
                new TcpEventHandler(shared_from_this(), nextTcpConnectionId));

        const NetworkAddress sourceNetworkAddress =
            networkAddressLookupInterfacePtr->LookupNetworkAddress(theNodeId);

        transportLayerPtr->tcpPtr->CreateOutgoingTcpConnection(
            sourceNetworkAddress, nextAvailableLocalPortNumber, destinationNetworkAddress, destinationPortNumber, dataPriority,
            tcpConnectionInfo.tcpEventHandlerPtr, tcpConnectionInfo.tcpConnectionPtr);

        if (useVirtualPayload) {
            tcpConnectionInfo.tcpConnectionPtr->EnableVirtualPayload();
        }//if//

        nextAvailableLocalPortNumber++;

        tcpConnectionInfo.tcpState = WAITING_FOR_ESTABLISHING;
        tcpConnectionInfo.destinationAddress = destinationNetworkAddress;

        RegisterBundleToTcpConnectionInfo(bundleIds, tcpConnectionInfo);

        tcpConnectionDatabase[nextTcpConnectionId] = tcpConnectionInfo;
        nextTcpConnectionId++;

    }
    else {

        TcpConnectionInfoType& tcpConnectionInfo = (*this).GetRefToTcpConnectionInfo(connectionId);

        if (tcpConnectionInfo.totalSendingDataBytes != 0) {
            //now transferring other bundles
            return;
        }//if

        assert(tcpConnectionInfo.totalSendingDataBytes == 0);
        assert(tcpConnectionInfo.sendingBundleIds.empty());

        RegisterBundleToTcpConnectionInfo(bundleIds, tcpConnectionInfo);

        if (tcpConnectionInfo.tcpState == ESTABLISHED) {
            //send data
            SendDataBlock(connectionId);
        }//if//

    }//if//

}//SendBundleWithTcp//


inline
void BundleProtocol::ReceiveUdpPacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceNetworkAddress)
{

    int offsetBytes = 0;
    while (offsetBytes < (int)packetPtr->LengthBytes()) {

        const PacketHeader& packetHeader =
            packetPtr->GetAndReinterpretPayloadData<PacketHeader>(offsetBytes);

        switch (packetHeader.type) {
        case HELLO:

            OutputTraceForControlPacketReceive(HELLO, sourceNetworkAddress);

            ProcessHelloPacket((*packetPtr), packetHeader.length, packetHeader.senderNodeId, sourceNetworkAddress);

            break;
        case ACK:

            ProcessAckInfo((*packetPtr), offsetBytes, packetHeader.length, sourceNetworkAddress);

            break;
        case ENCOUNTER_PROB:

            OutputTraceForControlPacketReceive(ENCOUNTER_PROB, sourceNetworkAddress);

            ProcessEncounterProbPacket(
                (*packetPtr), offsetBytes, packetHeader.length, sourceNetworkAddress);

            break;
        case REQUEST:

            OutputTraceForControlPacketReceive(REQUEST, sourceNetworkAddress);

            ProcessRequestPacket((*packetPtr), packetHeader.length, packetHeader.senderNodeId, sourceNetworkAddress);

            break;
        case BUNDLE:

            ProcessBundlePacket((*packetPtr), packetHeader.length, sourceNetworkAddress);

            break;
        default:
            assert(false);
            exit(1);
        }//switch//

        offsetBytes += sizeof(PacketHeader) + packetHeader.length;

    }//while//

    packetPtr = nullptr;

}//ReceiveUdpPacket//


inline
void BundleProtocol::ProcessHelloPacket(
    const Packet& packet,
    const unsigned int messageLength,
    const NodeId& sourceNodeId,
    const NetworkAddress& sourceNetworkAddress)
{

    assert((messageLength % sizeof(BundleIdType)) == 0);
    const size_t numberBundleIds = messageLength / sizeof(BundleIdType);

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    vector<BundleIdType> newBundleIds;
    vector<BundleIdType> bundleIdsForAck;

    int offsetBytes = sizeof(PacketHeader);
    for(size_t i = 0; i < numberBundleIds; i++) {

        const BundleIdType bundleId =
            packet.GetAndReinterpretPayloadData<BundleIdType>(offsetBytes);

        if (requestingBundleIds.find(bundleId) != requestingBundleIds.end()) {

            //check requested bundles expiration
            if ((requestingBundleIds.at(bundleId) + requestResendInterval) < currentTime) {

                requestingBundleIds.erase(bundleId);

            }//if//

        }//if//

        if (deliveredBundleIds.find(bundleId) != deliveredBundleIds.end()) {
            //already acked bundle
            bundleIdsForAck.push_back(bundleId);
        }
        else if ((ExtractOriginatorNodeId(bundleId) != theNodeId) &&
            (!TheBundleExists(bundleId)) &&
            (requestingBundleIds.find(bundleId) == requestingBundleIds.end())) {

            //brand new bundle
            newBundleIds.push_back(bundleId);
            requestingBundleIds[bundleId] = currentTime;

        }//if//

        offsetBytes += sizeof(BundleIdType);

    }//for//


    if (!newBundleIds.empty()) {

        //send request
        SendBundleIdInfo(REQUEST, newBundleIds, sourceNetworkAddress);

    }//if//


    if ((deliveryAckIsEnabled) && (!bundleIdsForAck.empty())) {
        //send ack
        SendBundleIdInfo(ACK, bundleIdsForAck, NetworkAddress::broadcastAddress);

    }//if//

    DoHelloPacketPostProcessing(sourceNodeId, sourceNetworkAddress);

}//ProcessHelloPacket//


inline
void BundleProtocol::ProcessAckInfo(
    const Packet& packet,
    const int offsetBytes,
    const unsigned int messageLength,
    const NetworkAddress& sourceNetworkAddress)
{
    assert(deliveryAckIsEnabled);

    assert((messageLength % sizeof(BundleIdType)) == 0);
    const size_t numberBundleIds = messageLength / sizeof(BundleIdType);

    vector<BundleIdType> ackedBundleIds;

    int packetPositon = offsetBytes + sizeof(PacketHeader);

    for(size_t i = 0; i < numberBundleIds; i++) {

        const BundleIdType bundleId =
            packet.GetAndReinterpretPayloadData<BundleIdType>(packetPositon);

        ackedBundleIds.push_back(bundleId);

        packetPositon += sizeof(BundleIdType);

    }//for//


    //remove bundle and add delivered list
    for (size_t i = 0; i < ackedBundleIds.size(); i++) {

        const BundleIdType bundleId = ackedBundleIds[i];

        if (deliveredBundleIds.find(bundleId) != deliveredBundleIds.end()) {
            //already listed
            continue;
        }//if//

        //added acked bunlde in the list
        deliveredBundleIds.insert(bundleId);

        vector<BundleInfoType>::iterator iter =
            find(storedBundles.begin(), storedBundles.end(), bundleId);

        if (iter != storedBundles.end()) {

            //update strage usage
            DecrementStorageUsage((*iter).bundleHeader.bundleSizeBytes);

            //remove bundle from buffer, and add in deliveredBundleIds
            storedBundles.erase(iter);

        }//if//

    }//for//

}//ProcessAckInfo//


inline
void BundleProtocol::ProcessRequestPacket(
    const Packet& packet,
    const unsigned int messageLength,
    const NodeId& senderNodeId,
    const NetworkAddress& sourceNetworkAddress)
{

    assert((messageLength % sizeof(BundleIdType)) == 0);
    const size_t numberBundleIds = messageLength / sizeof(BundleIdType);

    int offsetBytes = sizeof(PacketHeader);
    vector<BundleIdType> requestedBundleIds;
    for(size_t i = 0; i < numberBundleIds; i++) {

        const BundleIdType bundleId =
            packet.GetAndReinterpretPayloadData<BundleIdType>(offsetBytes);

        bool exists;
        BundleInfoType bundleInfo;
        GetBundleInfo(bundleId, exists, bundleInfo);

        if (!exists) {
             //already deleted
            offsetBytes += sizeof(BundleIdType);
            continue;
        }//if//

        //check number of remaining copies
        if ((bundleInfo.remainingCopies >= 2) ||
            ((bundleInfo.remainingCopies == 1) && ((bundleInfo.bundleHeader.targetNodeId == senderNodeId) || (bundleInfo.bundleHeader.targetNodeId == ANY_NODEID)))) {

            requestedBundleIds.push_back(bundleId);

        }//if//

        offsetBytes += sizeof(BundleIdType);

    }//for//

    SortRequestedBundlesIfNecessary(requestedBundleIds);

    if (transportMode == TCP) {
        SendBundleWithTcp(requestedBundleIds, sourceNetworkAddress);
    }
    else if (transportMode == UDP) {
        SendBundleWithUdp(requestedBundleIds, sourceNetworkAddress);
    }
    else {
        assert(false);
        exit(1);
    }//if//

}//ProcessRequestPacket//


inline
void BundleProtocol::ProcessBundlePacket(
    const Packet& packet,
    const unsigned int messageLength,
    const NetworkAddress& sourceNetworkAddress)
{

    BundleHeader bundleHeader = packet.GetAndReinterpretPayloadData<BundleHeader>(sizeof(PacketHeader));

    ProcessBundleData(sourceNetworkAddress, bundleHeader);

}//ProcessRequestPacket//


inline
void BundleProtocol::ProcessBundleData(
    const NetworkAddress& sourceAddress,
    BundleHeader& bundleHeader)
{

    //update hop count
    bundleHeader.hopCount++;

    OutputTraceAndStatsForReceiveBundle(
        sourceAddress,
        bundleHeader.bundleId,
        bundleHeader.targetNodeId,
        bundleHeader.bundleSizeBytes);

    //already received
    if (TheBundleExists(bundleHeader.bundleId) || (deliveredBundleIds.find(bundleHeader.bundleId) != deliveredBundleIds.end())) {
        duplicateBundleReceivedStatPtr->IncrementCounter();
        return;
    }//if//

    const BundleInfoType bundleInfo(bundleHeader.bundleId, bundleHeader.numOfCopies, bundleHeader);

    if ((bundleHeader.targetNodeId == theNodeId) || (bundleHeader.targetNodeId == ANY_NODEID)) {

        //for me

        deliveredBundleIds.insert(bundleHeader.bundleId);

        const SimTime delay =
            simulationEngineInterfacePtr->CurrentTime() - bundleHeader.sendTime;

        OutputTraceAndStatsForDeliveryBundle(
            bundleHeader.bundleId,
            bundleHeader.targetNodeId,
            bundleHeader.bundleSizeBytes,
            delay);

        bundleMessageAppPtr->ReceiveBundle(
            bundleHeader);
    }
    else {

        //for someone else

        bool success = false;
        StoreBundle(bundleInfo, success);

        if (!success) {
            bundlesDiscardedDueToLackOfStorageStatPtr->IncrementCounter();
        }//if//

    }//if//

}//ProcessBundleData//


inline
void BundleProtocol::AcceptTcpConnection(
    const shared_ptr<TcpConnection>& newTcpConnectionPtr)
{
    TcpConnectionInfoType tcpConnectionInfo;

    tcpConnectionInfo.tcpEventHandlerPtr =
        shared_ptr<TcpEventHandler>(new TcpEventHandler(shared_from_this(), nextTcpConnectionId));

    tcpConnectionInfo.tcpConnectionPtr = newTcpConnectionPtr;
    tcpConnectionInfo.tcpConnectionPtr->SetAppTcpEventHandler(tcpConnectionInfo.tcpEventHandlerPtr);
    tcpConnectionInfo.tcpConnectionPtr->SetPacketPriority(dataPriority);

    if (useVirtualPayload) {
        tcpConnectionInfo.tcpConnectionPtr->EnableVirtualPayload();
    }//if//

    tcpConnectionInfo.tcpState = WAITING_FOR_ESTABLISHING;
    tcpConnectionInfo.destinationAddress =
        newTcpConnectionPtr->GetForeignAddress();

    tcpConnectionDatabase[nextTcpConnectionId] = tcpConnectionInfo;
    nextTcpConnectionId++;

}//AcceptTcpConnection//


inline
void BundleProtocol::DoTcpIsReadyForMoreDataAction(const TcpConnectionIdType& connectionId)
{
    TcpStateType& tcpState = (*this).GetRefToTcpConnectionInfo(connectionId).tcpState;

    assert(
        tcpState == WAITING_FOR_ESTABLISHING ||
        tcpState == ESTABLISHED ||
        tcpState == WAITING_FOR_CLOSING);

    if (tcpState == WAITING_FOR_ESTABLISHING) {
        tcpState = ESTABLISHED;
    }
    else if (tcpState == WAITING_FOR_CLOSING) {
        return;
    }//if//

    SendDataBlock(connectionId);

}//DoTcpIsReadyForMoreDataAction//


inline
void BundleProtocol::SendDataBlock(const TcpConnectionIdType& connectionId)
{

    TcpConnectionInfoType& tcpConnectionInfo = (*this).GetRefToTcpConnectionInfo(connectionId);

    shared_ptr<TcpConnection>& tcpConnectionPtr = tcpConnectionInfo.tcpConnectionPtr;
    const TcpStateType tcpState = tcpConnectionInfo.tcpState;
    unsigned long long int& tcpAccumulatedDeliveredBytes = tcpConnectionInfo.tcpAccumulatedDeliveredBytes;
    unsigned long long int& totalSendingDataBytes = tcpConnectionInfo.totalSendingDataBytes;
    vector<BundleIdType>& sendingBundleIds = tcpConnectionInfo.sendingBundleIds;
    size_t& sendingBundleIndex = tcpConnectionInfo.sendingBundleIndex;

    assert(tcpState == ESTABLISHED);

    if (totalSendingDataBytes == 0) {
        return;
    }//if//

    const unsigned int tcpSentDataBytes =
        static_cast<unsigned int>(tcpConnectionPtr->GetNumberOfSentBytes() - tcpAccumulatedDeliveredBytes);

    assert(tcpSentDataBytes >= 0);

    if (tcpSentDataBytes < totalSendingDataBytes) {

        const unsigned int restOfSendingData = static_cast<unsigned int>(totalSendingDataBytes - tcpSentDataBytes);

        const unsigned int numberOfAvailableBufferBytes =
            static_cast<unsigned int>(tcpConnectionPtr->GetCurrentNumberOfAvailableBufferBytes());

        unsigned int dataBlockSize = 0;

        if (restOfSendingData > numberOfAvailableBufferBytes) {
            dataBlockSize = numberOfAvailableBufferBytes;
        }
        else {
            dataBlockSize = restOfSendingData;
        }//if//

        if (dataBlockSize > 0) {

            unsigned int remainingSentBytes = tcpSentDataBytes;

            //calculate remainingSentBytes
            for(size_t i = 0; i < sendingBundleIndex; i++) {

                assert(tcpConnectionInfo.sendingBundleSizeBytes[i] <= remainingSentBytes);

                remainingSentBytes -= tcpConnectionInfo.sendingBundleSizeBytes[i];

            }//for//

            BundleIdType workingBundleId;
            shared_ptr<vector<unsigned char> > workingBundlePtr;
            unsigned int workingBundleSizeBytes;

            shared_ptr<vector<unsigned char> > dataBlockPtr =
                shared_ptr<vector<unsigned char> >(new vector<unsigned char>());
            const unsigned int bundleHeaderSize = sizeof(BundleHeader);

            unsigned int availableDataBlockSize = dataBlockSize;

            while ((sendingBundleIndex < sendingBundleIds.size()) && (availableDataBlockSize > 0)) {

                workingBundleId = sendingBundleIds[sendingBundleIndex];
                workingBundlePtr = tcpConnectionInfo.sendingData[sendingBundleIndex];
                workingBundleSizeBytes = tcpConnectionInfo.sendingBundleSizeBytes[sendingBundleIndex];

                assert((*workingBundlePtr).size() == sizeof(BundleHeader));

                bool virtualPayloadUnavailable = false;
                if ((workingBundleSizeBytes - remainingSentBytes) < availableDataBlockSize) {
                    //can send more than one bundle
                    virtualPayloadUnavailable = true;
                }//if//

                //header part
                if (remainingSentBytes < bundleHeaderSize) {

                    const unsigned int headerPartSizeToBeSent = std::min((bundleHeaderSize - remainingSentBytes), availableDataBlockSize);
                    vector<unsigned char>::const_iterator iter =
                        (*workingBundlePtr).begin() + remainingSentBytes;

                    (*dataBlockPtr).insert((*dataBlockPtr).end(), iter, iter + headerPartSizeToBeSent);

                    availableDataBlockSize -= headerPartSizeToBeSent;

                }//if//

                //non header part
                if (bundleHeaderSize < (remainingSentBytes + availableDataBlockSize)) {

                    unsigned int nonHeaderPartSizeToBeSent;
                    if ((workingBundleSizeBytes - std::max(bundleHeaderSize, remainingSentBytes)) <= availableDataBlockSize) {
                        //can send rest of one bundle data
                        nonHeaderPartSizeToBeSent = (workingBundleSizeBytes - std::max(bundleHeaderSize, remainingSentBytes));

                        //go to next bundle
                        sendingBundleIndex++;
                        remainingSentBytes = 0;

                    }
                    else {

                        nonHeaderPartSizeToBeSent = availableDataBlockSize;

                    }//if//

                    if (virtualPayloadUnavailable) {
                        //add dummy data
                        (*dataBlockPtr).insert((*dataBlockPtr).end(), nonHeaderPartSizeToBeSent, 0);
                    }//if//

                    availableDataBlockSize -= nonHeaderPartSizeToBeSent;

                }//if//

                assert(availableDataBlockSize >= 0);

            }//while//

            tcpConnectionPtr->SendDataBlock(dataBlockPtr, dataBlockSize);

        }//if//
    }
    else {
        assert(tcpSentDataBytes == totalSendingDataBytes);
    }//if//

    const unsigned long long int numberOfDeliveredBytes =
        tcpConnectionPtr->GetNumberOfDeliveredBytes() - tcpAccumulatedDeliveredBytes;

    assert(numberOfDeliveredBytes >= 0);

    if (numberOfDeliveredBytes == totalSendingDataBytes) {

        assert(sendingBundleIndex == tcpConnectionInfo.sendingBundleSizeBytes.size());

        tcpAccumulatedDeliveredBytes = tcpConnectionPtr->GetNumberOfDeliveredBytes();
        totalSendingDataBytes = 0;
        sendingBundleIds.clear();
        sendingBundleIndex = 0;
        tcpConnectionInfo.sendingData.clear();
        tcpConnectionInfo.sendingBundleSizeBytes.clear();

    }
    else {
        assert(numberOfDeliveredBytes < totalSendingDataBytes);
    }//if//

}//SendDataBlock//


inline
void BundleProtocol::ReceiveDataBlock(
    const TcpConnectionIdType& connectionId,
    const unsigned char dataBlock[],
    const unsigned int dataLength,
    const unsigned int actualDataLength)
{
    TcpConnectionInfoType& tcpConnectionInfo = (*this).GetRefToTcpConnectionInfo(connectionId);

    const TcpStateType& tcpState = tcpConnectionInfo.tcpState;
    unsigned long long int& bundleSizeBytesToBeReceived = tcpConnectionInfo.bundleSizeBytesToBeReceived;
    unsigned long long int& receivedDataBytes = tcpConnectionInfo.receivedDataBytes;
    vector<unsigned char>& receivedData = tcpConnectionInfo.receivedData;
    BundleHeader& receivingBundleHeader = tcpConnectionInfo.receivingBundleHeader;

    if (tcpState != ESTABLISHED) return;

    assert(tcpState == ESTABLISHED);
    assert(dataLength > 0);

    receivedDataBytes += dataLength;

    for (unsigned int i = 0; i < actualDataLength; i++) {
        back_inserter(receivedData) = dataBlock[i];
    }//for//

    if (dataLength != actualDataLength) {
        //insert dummy data
        receivedData.insert(receivedData.end(), (dataLength - actualDataLength), '0');

    }//if//

    while (((bundleSizeBytesToBeReceived == 0) && (receivedDataBytes >= sizeof(BundleHeader))) ||
        ((bundleSizeBytesToBeReceived != 0) && (receivedDataBytes >= bundleSizeBytesToBeReceived))) {

        //read header
        if ((bundleSizeBytesToBeReceived == 0) && (receivedDataBytes >= sizeof(BundleHeader))) {

            BundleHeader bundleHeader;
            memcpy(&bundleHeader, &receivedData[0], sizeof(BundleHeader));

            receivingBundleHeader = bundleHeader;

            bundleSizeBytesToBeReceived = bundleHeader.bundleSizeBytes;

        }//if//

        //read body
        if ((bundleSizeBytesToBeReceived != 0) && (receivedDataBytes >= bundleSizeBytesToBeReceived)) {

            vector<unsigned char>::const_iterator iter = receivedData.begin();

            vector<unsigned char> nextFragmentedBundle(iter + static_cast<unsigned int>(bundleSizeBytesToBeReceived), iter + static_cast<unsigned int>(receivedDataBytes));

            //received one bundle
            ProcessBundleData(
                tcpConnectionInfo.tcpConnectionPtr->GetForeignAddress(),
                receivingBundleHeader);

            receivedDataBytes -= bundleSizeBytesToBeReceived;
            bundleSizeBytesToBeReceived = 0;
            receivedData.assign(nextFragmentedBundle.begin(), nextFragmentedBundle.end());
        }
        else {
            if (receivedDataBytes >= sizeof(BundleHeader)) {
                assert(receivedDataBytes < bundleSizeBytesToBeReceived);
            }//if//
        }//if//

    }//while//

}//ReceiveDataBlock//


inline
void BundleProtocol::DoTcpRemoteHostClosedAction(const TcpConnectionIdType& connectionId)
{
    TcpConnectionInfoType& tcpConnectionInfo = (*this).GetRefToTcpConnectionInfo(connectionId);

    assert(
        tcpConnectionInfo.tcpState == WAITING_FOR_ESTABLISHING ||
        tcpConnectionInfo.tcpState == ESTABLISHED);

    tcpConnectionInfo.tcpState = WAITING_FOR_CLOSING;
    const shared_ptr<TcpConnection> closingTcpConnectionPtr = tcpConnectionInfo.tcpConnectionPtr;
    tcpConnectionInfo.tcpConnectionPtr.reset();
    closingTcpConnectionPtr->Close();

}//DoTcpRemoteHostClosedAction//


inline
void BundleProtocol::DoTcpLocalHostClosedAction(const TcpConnectionIdType& connectionId)
{
    TcpStateType& tcpState = (*this).GetRefToTcpConnectionInfo(connectionId).tcpState;

    assert(
        tcpState == WAITING_FOR_ESTABLISHING ||
        tcpState == ESTABLISHED ||
        tcpState == WAITING_FOR_CLOSING);

    tcpState = CLOSE;

    DeleteTcpConnectionInfo(connectionId);

}//DoTcpLocalHostClosedAction//


inline
void BundleProtocol::OutputTraceForControlPacketSend(
    const MessageType& messageType,
    const NetworkAddress& destinationAddress)
{

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        NodeId destinationNodeId;
        if (destinationAddress.IsTheBroadcastOrAMulticastAddress()) {
            destinationNodeId = ANY_NODEID;
        }
        else {

            bool foundNodeId;
            networkAddressLookupInterfacePtr->LookupNodeId(
                destinationAddress, destinationNodeId, foundNodeId);
            if (!foundNodeId) {
                destinationNodeId = INVALID_NODEID;
            }//if//

        }//if//

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            BundleControlSendTraceRecord traceData;

            traceData.messageType = static_cast<unsigned char>(messageType);
            traceData.destinationNodeId = destinationNodeId;

            assert(sizeof(traceData) == BUNDLE_CONTROL_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theApplicationId, "CtrlSend", traceData);

        }
        else {

            ostringstream outStream;

            outStream
                << "Type= " << ConvertBundleMessageTypeToString(static_cast<unsigned char>(messageType))
                << " DestNodeId= " << destinationNodeId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "CtrlSend", outStream.str());

        }//if//

    }//if//

}//OutputTraceForControlPacketSend//


inline
void BundleProtocol::OutputTraceForControlPacketReceive(
    const MessageType& messageType,
    const NetworkAddress& sourceAddress)
{

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        NodeId sourceNodeId;
        bool foundNodeId;
        networkAddressLookupInterfacePtr->LookupNodeId(
            sourceAddress, sourceNodeId, foundNodeId);

        if (!foundNodeId) {
            sourceNodeId = INVALID_NODEID;
        }//if//

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            BundleControlReceiveTraceRecord traceData;

            traceData.messageType = static_cast<unsigned char>(messageType);
            traceData.sourceNodeId = sourceNodeId;

            assert(sizeof(traceData) == BUNDLE_CONTROL_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theApplicationId, "CtrlRecv", traceData);

        }
        else {

            ostringstream outStream;

            outStream
                << "Type= " << ConvertBundleMessageTypeToString(static_cast<unsigned char>(messageType))
                << " SrcNodeId= " << sourceNodeId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "CtrlRecv", outStream.str());

        }//if//

    }//if//

}//OutputTraceForControlPacketReceive//


inline
void BundleProtocol::OutputTraceAndStatsForGenerateBundle(
    const BundleIdType& bundleId,
    const NodeId& targetNodeId,
    const size_t& bundleSizeBytes)
{
    bundlesGeneratedStatPtr->IncrementCounter();
    bytesGeneratedStatPtr->IncrementCounter(bundleSizeBytes);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            BundleGenerateTraceRecord traceData;

            traceData.bundleSizeBytes = static_cast<uint32_t>(bundleSizeBytes);
            traceData.originatorNodeId = ExtractOriginatorNodeId(bundleId);
            traceData.originatorSequenceNumber = ExtractOriginatorSequenceNumber(bundleId);
            traceData.targetNodeId = targetNodeId;

            assert(sizeof(traceData) == BUNDLE_GENERATE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theApplicationId, "Generate", traceData);
        }
        else {

            ostringstream outStream;

            outStream
                << "Size= " << bundleSizeBytes
                << " OrigNodeId= " << ExtractOriginatorNodeId(bundleId)
                << " OrigSeq= " << ExtractOriginatorSequenceNumber(bundleId)
                << " TargetNodeId= " << targetNodeId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "Generate", outStream.str());

        }//if//

    }//if//


}//OutputTraceAndStatsForGenerateBundle//


inline
void BundleProtocol::OutputTraceAndStatsForDeliveryBundle(
    const BundleIdType& bundleId,
    const NodeId& targetNodeId,
    const size_t& bundleSizeBytes,
    const SimTime& delay)
{
    bundlesDeliveredStatPtr->IncrementCounter();
    bytesDeliveredStatPtr->IncrementCounter(bundleSizeBytes);
    bundleEndToEndDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(delay));

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            BundleDeliveryTraceRecord traceData;

            traceData.bundleSizeBytes = static_cast<uint32_t>(bundleSizeBytes);
            traceData.originatorNodeId = ExtractOriginatorNodeId(bundleId);
            traceData.originatorSequenceNumber = ExtractOriginatorSequenceNumber(bundleId);
            traceData.targetNodeId = targetNodeId;
            traceData.delay = delay;

            assert(sizeof(traceData) == BUNDLE_DELIVERY_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theApplicationId, "Delivery", traceData);

        }
        else {

            ostringstream outStream;

            outStream
                << "Size= " << bundleSizeBytes
                << " OrigNodeId= " << ExtractOriginatorNodeId(bundleId)
                << " OrigSeq= " << ExtractOriginatorSequenceNumber(bundleId)
                << " TargetNodeId= " << targetNodeId
                << " Delay= " << ConvertTimeToStringSecs(delay);

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "Delivery", outStream.str());

        }//if//
    }//if//

}//OutputTraceAndStatsForDeliveryBundle//



inline
void BundleProtocol::OutputTraceAndStatsForSendBundle(
    const NetworkAddress& nextNodeAddress,
    const BundleIdType& bundleId,
    const NodeId& targetNodeId,
    const size_t& bundleSizeBytes)
{
    bundlesSentStatPtr->IncrementCounter();
    bytesSentStatPtr->IncrementCounter(bundleSizeBytes);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        NodeId nextNodeId;
        bool foundNodeId;
        networkAddressLookupInterfacePtr->LookupNodeId(
            nextNodeAddress, nextNodeId, foundNodeId);
        if (!foundNodeId) {
            nextNodeId = INVALID_NODEID;
        }//if//

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            BundleSendTraceRecord traceData;

            traceData.destinationNodeId = nextNodeId;
            traceData.bundleSizeBytes = static_cast<uint32_t>(bundleSizeBytes);
            traceData.originatorNodeId = ExtractOriginatorNodeId(bundleId);
            traceData.originatorSequenceNumber = ExtractOriginatorSequenceNumber(bundleId);
            traceData.targetNodeId = targetNodeId;

            assert(sizeof(traceData) == BUNDLE_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theApplicationId, "Send", traceData);

        }
        else {

            ostringstream outStream;

            outStream
                << "DestNodeId= " << nextNodeId
                << " Size= " << bundleSizeBytes
                << " OrigNodeId= " << ExtractOriginatorNodeId(bundleId)
                << " OrigSeq= " << ExtractOriginatorSequenceNumber(bundleId)
                << " TargetNodeId= " << targetNodeId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "Send", outStream.str());

        }//if//

    }//if//

}//OutputTraceAndStatsForSendBundle//



inline
void BundleProtocol::OutputTraceAndStatsForReceiveBundle(
    const NetworkAddress& lastNodeAddress,
    const BundleIdType& bundleId,
    const NodeId& targetNodeId,
    const size_t& bundleSizeBytes)
{
    bundlesReceivedStatPtr->IncrementCounter();
    bytesReceivedStatPtr->IncrementCounter(bundleSizeBytes);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        NodeId lastNodeId;
        bool foundNodeId;
        networkAddressLookupInterfacePtr->LookupNodeId(
            lastNodeAddress, lastNodeId, foundNodeId);
        if (!foundNodeId) {
            lastNodeId = INVALID_NODEID;
        }//if//

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            BundleReceiveTraceRecord traceData;

            traceData.sourceNodeId = lastNodeId;
            traceData.bundleSizeBytes = static_cast<uint32_t>(bundleSizeBytes);
            traceData.originatorNodeId = ExtractOriginatorNodeId(bundleId);
            traceData.originatorSequenceNumber = ExtractOriginatorSequenceNumber(bundleId);
            traceData.targetNodeId = targetNodeId;

            assert(sizeof(traceData) == BUNDLE_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theApplicationId, "Recv", traceData);

        }
        else {

            ostringstream outStream;

            outStream
                << "SrcNodeId= " << lastNodeId
                << " Size= " << bundleSizeBytes
                << " OrigNodeId= " << ExtractOriginatorNodeId(bundleId)
                << " OrigSeq= " << ExtractOriginatorSequenceNumber(bundleId)
                << " TargetNodeId= " << targetNodeId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "Recv", outStream.str());

        }//if//

    }//if//

}//OutputTraceAndStatsForReceiveBundle//


inline
void BundleProtocol::OutputTraceAndStatForStorageUsage()
{
    storageUsageBytesStatPtr->RecordStatValue(static_cast<double>(currentStrageUsageBytes));

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theApplicationId, "StorageUsage", currentStrageUsageBytes);

        }
        else {

            ostringstream outStream;

            outStream << "StorageUsage= " << currentStrageUsageBytes;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, theApplicationId, "StorageUsage", outStream.str());

        }//if//
    }//if//

}//OutputTraceAndStatForStorageUsage//

//--------------------------------------------------------
class EpidemicBundleProtocol: public BundleProtocol {
public:

    EpidemicBundleProtocol(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeId& initNodeId,
        const unsigned short int initDefaultDestinationPortId);

};


inline
EpidemicBundleProtocol::EpidemicBundleProtocol(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeId& initNodeId,
    const unsigned short int initDefaultDestinationPortId)
    :
    BundleProtocol(
        theParameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initNodeId,
        initDefaultDestinationPortId)
{

    //set limit of bundle copy
    maximumCopies = UINT_MAX;


}//EpidemicBundleProtocol//




//--------------------------------------------------------
class DirectDeliveryBundleProtocol: public BundleProtocol {
public:

    DirectDeliveryBundleProtocol(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeId& initNodeId,
        const unsigned short int initDefaultDestinationPortId);

};


inline
DirectDeliveryBundleProtocol::DirectDeliveryBundleProtocol(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeId& initNodeId,
    const unsigned short int initDefaultDestinationPortId)
    :
    BundleProtocol(
        theParameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initNodeId,
        initDefaultDestinationPortId)
{

    //set limit of bundle copy
    maximumCopies = 1;

}//DirectDeliveryBundleProtocol//


//--------------------------------------------------------
class SprayAndWaitBundleProtocol: public BundleProtocol {
public:

    SprayAndWaitBundleProtocol(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeId& initNodeId,
        const unsigned short int initDefaultDestinationPortId);

private:
    bool binaryMode;

    void SubtractNumberOfCopies(
        const BundleIdType& targetBundleId) override;

};


inline
SprayAndWaitBundleProtocol::SprayAndWaitBundleProtocol(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeId& initNodeId,
    const unsigned short int initDefaultDestinationPortId)
    :
    BundleProtocol(
        theParameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initNodeId,
        initDefaultDestinationPortId),
    binaryMode(false)
{

    if (theParameterDatabaseReader.ParameterExists(
            "bundle-spray-and-wait-binary-mode", initNodeId)) {

        binaryMode =
            theParameterDatabaseReader.ReadBool(
                "bundle-spray-and-wait-binary-mode", initNodeId);

    }//if//

    //set limit of bundle copy
    maximumCopies =
        theParameterDatabaseReader.ReadNonNegativeInt("bundle-maximum-number-of-copies", initNodeId);

    if (maximumCopies < 1) {
        cerr << "Error: bundle-maximum-number-of-copies must be greater than or equal to 1: " << maximumCopies << endl;
        exit(1);
    }//if//

}//SprayAndWaitBundleProtocol//


inline
void SprayAndWaitBundleProtocol::SubtractNumberOfCopies(
    const BundleIdType& targetBundleId)
{

    vector<BundleInfoType>::iterator iter =
        find(storedBundles.begin(), storedBundles.end(), targetBundleId);

    assert(iter != storedBundles.end());

    BundleInfoType& bundleInfo = (*iter);

    assert(bundleInfo.remainingCopies >= 1);

    if (binaryMode) {

        bundleInfo.bundleHeader.numOfCopies =
            static_cast<unsigned int>(std::ceil(double(bundleInfo.remainingCopies) / 2));
        bundleInfo.remainingCopies /= 2;

    }
    else {

        bundleInfo.bundleHeader.numOfCopies = 1;
        bundleInfo.remainingCopies--;

    }//if//

}//SubtractNumberOfCopies//


//--------------------------------------------------------
//Reference: "MaxProp: Routing for Vehicle-Based Disruption-Tolerant Networks" by J. Burgress, etc.
class MaxPropBundleProtocol: public BundleProtocol {
public:

    MaxPropBundleProtocol(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeId& initNodeId,
        const unsigned short int initDefaultDestinationPortId);

private:

    SimTime lastEncounterProbInfoSentTime;

    void DoHelloPacketPostProcessing(
        const NodeId& sourceNodeId,
        const NetworkAddress& sourceNetworkAddress) override;

    void ProcessEncounterProbPacket(
        const Packet& packet,
        const int offsetBytes,
        const unsigned int messageLength,
        const NetworkAddress& sourceNetworkAddress) override;

    void SortRequestedBundlesIfNecessary(
        vector<BundleIdType>& requestedBundleIds) override;

    void ManageStorageCapacity(const unsigned int newDataSizeBytes) override;


    struct EncounterProbInfoType {
        NodeId theNodeId;
        unsigned int numOfEntries;
        SimTime timestamp;

        EncounterProbInfoType(
            const NodeId& initNodeId,
            const unsigned int initNumOfEntries,
            const SimTime& initTimestamp)
            :
            theNodeId(initNodeId),
            numOfEntries(initNumOfEntries),
            timestamp(initTimestamp)
            { }

    };

    struct EncounterProbDataType {
        EncounterProbInfoType entryInfo;
        map<NodeId, float> encounterEntries;

        EncounterProbDataType(
            const NodeId& theNodeId,
            const unsigned int numOfEntries,
            const SimTime& timestamp,
            const map<NodeId, float>& initEncounterEntries)
            :
            entryInfo(EncounterProbInfoType(theNodeId, numOfEntries, timestamp)),
            encounterEntries(initEncounterEntries)
            { }

        EncounterProbDataType(
            const EncounterProbInfoType& initEntryInfo,
            const map<NodeId, float>& initEncounterEntries)
            :
            entryInfo(initEntryInfo),
            encounterEntries(initEncounterEntries)
            { }

    };

    struct RouteSearchNodeType {
        NodeId theNodeId;
        double pathCost;

        bool isClosed;
        NodeId previousNodeId;

        RouteSearchNodeType()
            :
            theNodeId(INVALID_NODEID),
            pathCost(DBL_MAX),
            isClosed(false),
            previousNodeId(INVALID_NODEID)
        {}

        RouteSearchNodeType(
            const NodeId& initNodeId)
            :
            theNodeId(initNodeId),
            pathCost(DBL_MAX),
            isClosed(false),
            previousNodeId(INVALID_NODEID)
        {}

        bool operator>(const RouteSearchNodeType& right) const {
            return ((*this).pathCost > right.pathCost);
        }
        bool operator==(const NodeId& right) const {
            return ((*this).theNodeId == right);
        }
    };


    struct EncounterEntryType {
        NodeId targetNodeId;
        float probability;
    };

    struct HopCountBasedComparator {
        bool operator()(const BundleInfoType& left, const BundleInfoType& right) const
        { return (left.bundleHeader.hopCount < right.bundleHeader.hopCount); }
    };

    struct CostBasedComparator {
        bool operator()(const BundleInfoType& left, const BundleInfoType& right) const
        { return ((left.cost < right.cost) ||
            ((left.cost == right.cost) && (left.bundleHeader.hopCount < right.bundleHeader.hopCount))); }
    };

    map<NodeId, EncounterProbDataType> encounterProbDatabse;

    double averageTransferredBytes;
    unsigned int averagingCount;
    unsigned long long int headStartThresholdSizeBytes;


    void UpdateAverageTransferredBytes(const vector<BundleIdType>& bundleIds);

    void UpdatePathCost();

    void UpdateStoredBundleOrder();

    void SortBundleIds(vector<BundleIdType>& bundleIds) const;

};


inline
MaxPropBundleProtocol::MaxPropBundleProtocol(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeId& initNodeId,
    const unsigned short int initDefaultDestinationPortId)
    :
    BundleProtocol(
        theParameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initNodeId,
        initDefaultDestinationPortId),
    lastEncounterProbInfoSentTime(ZERO_TIME),
    averageTransferredBytes(0.0),
    averagingCount(0),
    headStartThresholdSizeBytes(0)
{

    //set limit of bundle copy
    maximumCopies = UINT_MAX;

}//MaxPropBundleProtocol//


void MaxPropBundleProtocol::DoHelloPacketPostProcessing(
    const NodeId& sourceNodeId,
    const NetworkAddress& sourceNetworkAddress)
{

    //update self encounter probability
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    typedef map<NodeId, EncounterProbDataType>::iterator NodeIterTypeForUpdate;

    NodeIterTypeForUpdate measurementNodeIter = encounterProbDatabse.find(theNodeId);

    if (measurementNodeIter != encounterProbDatabse.end()) {

        EncounterProbInfoType& selfEntryInfo = (measurementNodeIter->second).entryInfo;
        map<NodeId, float>& selfEncounterEntries = (measurementNodeIter->second).encounterEntries;

        if (selfEncounterEntries.find(sourceNodeId) == selfEncounterEntries.end()) {
            selfEncounterEntries[sourceNodeId] = 1.0;
            selfEntryInfo.numOfEntries++;
        }
        else {
            selfEncounterEntries[sourceNodeId] += 1.0;
        }

        selfEntryInfo.timestamp = currentTime;

        //normalize
        for(map<NodeId, float>::iterator entryIter = selfEncounterEntries.begin();
            entryIter != selfEncounterEntries.end(); ++entryIter) {
            (entryIter->second) /= 2;
        }//for//


    }
    else {
        map<NodeId, float> newEntry;
        newEntry.insert(make_pair(sourceNodeId, (float)1.0));
        EncounterProbDataType encounterProbData(theNodeId, 1, currentTime, newEntry);

        encounterProbDatabse.insert(make_pair(theNodeId, encounterProbData));

    }//if//


    //broadcast encounter probability info

    if (currentTime < (lastEncounterProbInfoSentTime + helloInterval)) {
        return;
    }//if//

    lastEncounterProbInfoSentTime = currentTime;

    vector<unsigned char> payload;
    size_t payloadPosition = sizeof(PacketHeader);
    payload.resize(payloadPosition);

    size_t remainingPayloadSizeBytes = maxControlPacketSizeBytes - payloadPosition;

    typedef map<NodeId, EncounterProbDataType>::const_iterator NodeIterTypeForPack;

    for(NodeIterTypeForPack nodeIter = encounterProbDatabse.begin();
        nodeIter != encounterProbDatabse.end(); ++nodeIter) {

        const EncounterProbInfoType& entryInfo = (nodeIter->second).entryInfo;
        const map<NodeId, float>& encounterEntries = (nodeIter->second).encounterEntries;

        assert(entryInfo.numOfEntries == encounterEntries.size());

        const size_t nextPayloadSizeBytes =
            sizeof(EncounterProbInfoType) + (sizeof(EncounterEntryType) * encounterEntries.size());

        if (maxControlPacketSizeBytes < (sizeof(PacketHeader) + nextPayloadSizeBytes)) {
            cout << "Error: cannot pack encounter prop info in a packet" << endl;
            cout << "Set bundle-max-control-packet-size-bytes is equal to or greater than "
                << (sizeof(PacketHeader) + nextPayloadSizeBytes) << endl;
            exit(1);
        }//if//

        if (remainingPayloadSizeBytes < nextPayloadSizeBytes) {

            assert(payload.size() > sizeof(PacketHeader));

            PacketHeader encounterProbHeader(theNodeId, ENCOUNTER_PROB, (payload.size() - sizeof(PacketHeader)));
            memcpy(&payload[0], &encounterProbHeader, sizeof(PacketHeader));

            ScheduleControlPacketSendEvent(ENCOUNTER_PROB, NetworkAddress::broadcastAddress, payload);

            //reset
            payloadPosition = sizeof(PacketHeader);
            payload.clear();
            payload.resize(payloadPosition);
            remainingPayloadSizeBytes = maxControlPacketSizeBytes - payloadPosition;

        }//if//

        assert(remainingPayloadSizeBytes >= nextPayloadSizeBytes);

        //insert encounter prob info
        remainingPayloadSizeBytes -= sizeof(EncounterProbInfoType);
        payload.resize(payloadPosition + sizeof(EncounterProbInfoType));
        memcpy(&payload[payloadPosition], &entryInfo, sizeof(EncounterProbInfoType));
        payloadPosition += sizeof(EncounterProbInfoType);

        typedef map<NodeId, float>::const_iterator EntryIterType;
        for(EntryIterType entryIter = encounterEntries.begin(); entryIter != encounterEntries.end(); ++entryIter) {

            EncounterEntryType encounterEntry;
            encounterEntry.targetNodeId = (entryIter->first);
            encounterEntry.probability = (entryIter->second);

            remainingPayloadSizeBytes -= sizeof(EncounterEntryType);
            payload.resize(payloadPosition + sizeof(EncounterEntryType));
            memcpy(&payload[payloadPosition], &encounterEntry, sizeof(EncounterEntryType));
            payloadPosition += sizeof(EncounterEntryType);

        }//for//

    }//for//

    if (payload.size() > sizeof(PacketHeader)) {

        //send last packet

        PacketHeader encounterProbHeader(theNodeId, ENCOUNTER_PROB, (payload.size() - sizeof(PacketHeader)));
        memcpy(&payload[0], &encounterProbHeader, sizeof(PacketHeader));

        ScheduleControlPacketSendEvent(ENCOUNTER_PROB, NetworkAddress::broadcastAddress, payload);

    }//if//

}//DoHelloPacketPostProcessing//


inline
void MaxPropBundleProtocol::ProcessEncounterProbPacket(
    const Packet& packet,
    const int offsetBytes,
    const unsigned int messageLength,
    const NetworkAddress& sourceNetworkAddress)
{

    //update received encounter probability

    typedef map<NodeId, EncounterProbDataType>::iterator MesurementNodeIterType;

    int packetPositon = offsetBytes + sizeof(PacketHeader);

    while (packetPositon < (int)(offsetBytes + sizeof(PacketHeader) + messageLength)) {

        const EncounterProbInfoType receivedEntryInfo =
            packet.GetAndReinterpretPayloadData<EncounterProbInfoType>(packetPositon);

        packetPositon += sizeof(EncounterProbInfoType);

        bool updateEntry = false;

        MesurementNodeIterType measurementNodeIter = encounterProbDatabse.find(receivedEntryInfo.theNodeId);

        if (measurementNodeIter == encounterProbDatabse.end()) {
            //band new info
            updateEntry = true;
        }
        else {

            const EncounterProbInfoType& existingEntryInfo = (measurementNodeIter->second).entryInfo;

            if (existingEntryInfo.timestamp < receivedEntryInfo.timestamp) {
                //newer info
                updateEntry = true;

                //clear exisiting
                encounterProbDatabse.erase(measurementNodeIter);

            }//if//
        }//if//

        if (updateEntry) {

            map<NodeId, float> newEntries;

            for(size_t i = 0; i < receivedEntryInfo.numOfEntries; i++) {

                const EncounterEntryType aEntry =
                    packet.GetAndReinterpretPayloadData<EncounterEntryType>(packetPositon);

                newEntries.insert(make_pair(aEntry.targetNodeId, aEntry.probability));

                packetPositon += sizeof(EncounterEntryType);

            }//for//

            assert(encounterProbDatabse.find(receivedEntryInfo.theNodeId) == encounterProbDatabse.end());

            encounterProbDatabse.insert(make_pair(receivedEntryInfo.theNodeId, EncounterProbDataType(receivedEntryInfo, newEntries)));


        }
        else {

            //jump packet position to skip
            packetPositon += (sizeof(EncounterEntryType) * receivedEntryInfo.numOfEntries);

        }//if//

    }//while//


}//ProcessEncounterProbPacket//


inline
void MaxPropBundleProtocol::SortRequestedBundlesIfNecessary(
    vector<BundleIdType>& requestedBundleIds)
{

    //update bundle order
    if (requestedBundleIds.size() >= 2) {

        UpdateStoredBundleOrder();

        SortBundleIds(requestedBundleIds);

    }//if//

    //update averaging transferred bytes to calculate head start threshold
    UpdateAverageTransferredBytes(requestedBundleIds);

}//SortRequestedBundlesIfNecessary//


inline
void MaxPropBundleProtocol::ManageStorageCapacity(const unsigned int newDataSizeBytes)
{

    //update bundle order
    UpdateStoredBundleOrder();

    unsigned int shortageStorageSizeBytes =
        static_cast<unsigned int>((currentStrageUsageBytes + newDataSizeBytes) - maxStorageSizeBytes);

    while ((shortageStorageSizeBytes > 0) && (!storedBundles.empty())) {

        //update strage usage
        DecrementStorageUsage(storedBundles.back().bundleHeader.bundleSizeBytes);
        shortageStorageSizeBytes -= storedBundles.back().bundleHeader.bundleSizeBytes;

        //delete lowest priority bundle
        storedBundles.pop_back();

    }//while//

 }//ManageStorageCapacity//


inline
void MaxPropBundleProtocol::UpdateAverageTransferredBytes(const vector<BundleIdType>& bundleIds)
{

    averagingCount++;

    //sum bundle size to transfer
    unsigned int tranferredBytes = 0;

    for(size_t i = 0; i < bundleIds.size(); i++) {

        const BundleIdType bundleId = bundleIds[i];

        bool exists;
        BundleInfoType bundleInfo;
        GetBundleInfo(bundleId, exists, bundleInfo);

        if (exists) {

            tranferredBytes += (bundleInfo.bundleHeader).bundleSizeBytes;

        }//if//

    }//for//


    //use all trasfer history
    averageTransferredBytes =
        (averageTransferredBytes * (averagingCount - 1) + tranferredBytes) / averagingCount;

}//UpdateAverageTransferredBytes//


inline
void MaxPropBundleProtocol::SortBundleIds(vector<BundleIdType>& bundleIds) const
{

    set<BundleIdType> remainingBundleIds(bundleIds.begin(), bundleIds.end());

    bundleIds.clear();

    for(size_t i = 0; i < storedBundles.size(); i++) {

        const BundleIdType targetBundleId = storedBundles[i].bundleId;

        if (remainingBundleIds.find(targetBundleId) != remainingBundleIds.end()) {

            bundleIds.push_back(targetBundleId);

            remainingBundleIds.erase(targetBundleId);

        }//if//

        if (remainingBundleIds.empty()) {
            break;
        }//if//

    }//for//

}//SortBundleIds//


void MaxPropBundleProtocol::UpdatePathCost()
{

    if (encounterProbDatabse.find(theNodeId) == encounterProbDatabse.end()) {
        //did not receive any hello packet
        return;
    }//if//

    // initialize
    vector<RouteSearchNodeType> targetNodes;
    targetNodes.clear();
    targetNodes.reserve(encounterProbDatabse.size());

    for(map<NodeId, EncounterProbDataType>::const_iterator iter = encounterProbDatabse.begin(); iter != encounterProbDatabse.end(); ++iter) {
        targetNodes.push_back(RouteSearchNodeType(iter->first));
    }//for//

    typedef vector<RouteSearchNodeType>::iterator NodeIter;

    // Path cost update
    NodeIter sourceNodeIter = find(targetNodes.begin(), targetNodes.end(), theNodeId);
    assert(sourceNodeIter != targetNodes.end());

    (*sourceNodeIter).pathCost = 0.0;

    priority_queue<RouteSearchNodeType, vector<RouteSearchNodeType>, std::greater<RouteSearchNodeType> > openQueue;

    openQueue.push((*sourceNodeIter));

    while(!openQueue.empty() ) {

        RouteSearchNodeType aNode = openQueue.top();
        openQueue.pop();

        aNode.isClosed = true;

        const NodeId& startNodeId = aNode.theNodeId;

        map<NodeId, EncounterProbDataType>::const_iterator measurementNodeIter =
            encounterProbDatabse.find(startNodeId);

        assert(measurementNodeIter != encounterProbDatabse.end());

        const map<NodeId, float>& encounterEntries = (measurementNodeIter->second).encounterEntries;

        for (map<NodeId, float>::const_iterator iter = encounterEntries.begin(); iter != encounterEntries.end(); ++iter) {

            const NodeId targetNodeId = (iter->first);
            const NodeIter targetNodeIter = find(targetNodes.begin(), targetNodes.end(), targetNodeId);

            if (targetNodeIter == targetNodes.end()) continue;

            const double candidatePathCost = aNode.pathCost + (1.0 - (iter->second));

            if (candidatePathCost < (*targetNodeIter).pathCost) {

                (*targetNodeIter).pathCost = candidatePathCost;
                (*targetNodeIter).previousNodeId = aNode.theNodeId;

                if (!(*targetNodeIter).isClosed) {
                    openQueue.push((*targetNodeIter));
                }//if//
            }//for//

        }//for//
    }//while//


    // Update cost
    for(size_t i = 0; i < storedBundles.size(); i++) {

        const NodeId destinationNodeId = storedBundles[i].bundleHeader.targetNodeId;

        if (destinationNodeId == ANY_NODEID) {
            storedBundles[i].cost = 0.0;//highest priority
        }
        else {

            vector<RouteSearchNodeType>::const_iterator iter =
                find(targetNodes.begin(), targetNodes.end(), destinationNodeId);

            if (iter == targetNodes.end()) {
                //unreachable
                storedBundles[i].cost = DBL_MAX;//lowest priority
            }
            else {
                storedBundles[i].cost = (*iter).pathCost;
            }//if//

        }//if//

    }//for//

}//UpdatePathCost//


inline
void MaxPropBundleProtocol::UpdateStoredBundleOrder()
{

    if (storedBundles.size() < 2) {
        return;
    }//if//

    //averageTransferredBytes: x in the literature
    //currentStorageUsageBytes: b in the literature
    //headStartThresholdSizeBytes: p in the literature

    //calculate head start threshold
    if (currentStrageUsageBytes <= averageTransferredBytes) {

        headStartThresholdSizeBytes = 0;

    }
    else if (averageTransferredBytes < (currentStrageUsageBytes / 2)) {

        headStartThresholdSizeBytes = static_cast<unsigned long long int>(averageTransferredBytes);
    }
    else {

        headStartThresholdSizeBytes =
            static_cast<unsigned int>(std::min<double>(averageTransferredBytes, (currentStrageUsageBytes - averageTransferredBytes)));

    }//if//

    //update cost
    UpdatePathCost();

    if (headStartThresholdSizeBytes == 0) {

        //order by cost
        std::sort(storedBundles.begin(), storedBundles.end(), CostBasedComparator());

    }
    else {

        //order by hop count first
        std::sort(storedBundles.begin(), storedBundles.end(), HopCountBasedComparator());

        unsigned long long int hopCountSortedStorageSizeBytes = 0;
        vector<BundleInfoType>::iterator iter;
        unsigned int thresholdHopCount = 0;

        for(iter = storedBundles.begin(); iter != storedBundles.end(); ++iter) {

            hopCountSortedStorageSizeBytes += (*iter).bundleHeader.bundleSizeBytes;

            if (hopCountSortedStorageSizeBytes >= headStartThresholdSizeBytes) {
                thresholdHopCount = (*iter).bundleHeader.hopCount;
                break;
            }//if/

        }//for//

        assert(iter != storedBundles.end());

        while ((*iter).bundleHeader.hopCount == thresholdHopCount) {
            ++iter;
            if (iter == storedBundles.end()) {
                break;
            }//if//
        }//while

        if (iter != storedBundles.end()) {
            //order by cost
            std::sort(iter, storedBundles.end(), CostBasedComparator());
        }//if//

    }//if//

}//UpdateStoredBundleOrder//



//--------------------------------------------------------
// CreateBundleProtocol
//--------------------------------------------------------

shared_ptr<BundleProtocol> CreateBundleProtocol(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const NodeId& theNodeId,
    const unsigned short int defaultDestinationPortId)
{

    const string routingAlgorithmString =
        MakeLowerCaseString(theParameterDatabaseReader.ReadString("bundle-routing-algorithm", theNodeId));

    if (routingAlgorithmString == "epidemic") {

        return shared_ptr<BundleProtocol>(
            new EpidemicBundleProtocol(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                theNodeId,
                defaultDestinationPortId));

    }
    else if (routingAlgorithmString == "direct-delivery") {

        return shared_ptr<BundleProtocol>(
            new DirectDeliveryBundleProtocol(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                theNodeId,
                defaultDestinationPortId));

    }
    else if (routingAlgorithmString == "spray-and-wait") {

        return shared_ptr<BundleProtocol>(
            new SprayAndWaitBundleProtocol(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                theNodeId,
                defaultDestinationPortId));

    }
    else if (routingAlgorithmString == "maxprop") {

        return shared_ptr<BundleProtocol>(
            new MaxPropBundleProtocol(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                theNodeId,
                defaultDestinationPortId));

    }//if//

    cerr << "Error: bundle-routing-algorithm = " << routingAlgorithmString << " is unknown." << endl;
    exit(1);

    return shared_ptr<BundleProtocol>();

}//CreateBundleProtocol//

}//namespace//

#endif
