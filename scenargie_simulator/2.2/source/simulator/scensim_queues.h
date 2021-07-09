// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_QUEUES_H
#define SCENSIM_QUEUES_H

#include <queue>
#include "scensim_parmio.h"
#include "scensim_engine.h"
#include "scensim_netaddress.h"
#include "scensim_packet.h"

#include <iostream>//20210613

namespace ScenSim {

using std::cout;//20210613
using std::endl;//20210613

typedef unsigned short EtherTypeField;

const EtherTypeField ETHERTYPE_IS_NOT_SPECIFIED = 65535; //0xFFFF
const EtherTypeField ETHERTYPE_IP = 2048; //0x0800
const EtherTypeField ETHERTYPE_ARP = 2054; //0x0806
const EtherTypeField ETHERTYPE_VLAN = 33024; //0x8100
const EtherTypeField ETHERTYPE_IPV6 = 34525; //0x86DD
const EtherTypeField ETHERTYPE_WSMP = 35036; //0x88DC
const EtherTypeField ETHERTYPE_GEONET = 1799; //0x0707


enum EnqueueResultType {
    ENQUEUE_SUCCESS,
    ENQUEUE_FAILURE_BY_MAX_PACKETS,
    ENQUEUE_FAILURE_BY_MAX_BYTES,
    ENQUEUE_FAILURE_BY_OUT_OF_SCOPE
};


class OutputQueuePriorityMapper {
public:
    virtual PacketPriority MaxMappedPriority() const = 0;
    virtual PacketPriority MapIpToMacPriority(const PacketPriority& ipPriority) const = 0;
};


//--------------------------------------

class InterfaceOutputQueue {
public:
    virtual ~InterfaceOutputQueue() { }

    virtual bool InsertWithFullPacketInformationModeIsOn() const { return false; }

    virtual void Insert(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDropPtr,
        const EtherTypeField etherType = ETHERTYPE_IS_NOT_SPECIFIED){
            //cout << "Insert_virtual" << endl;//no
        }

    virtual void InsertWithFullPacketInformation(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const NetworkAddress& sourceAddress,
        const unsigned short int sourcePort,
        const NetworkAddress& destinationAddress,
        const unsigned short int destinationPort,
        const unsigned char protocolCode,
        const PacketPriority priority,
        const unsigned short int ipv6FlowLabel,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDropPtr) { 
            assert(false); abort(); 
            //cout << "Insertwith" << endl;//no
        }

    virtual PacketPriority MaxPossiblePacketPriority() const { return MAX_AVAILABLE_PACKET_PRIORITY; }

    virtual bool IsEmpty() const = 0;

    virtual void DequeuePacket(
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        PacketPriority& priority,
        EtherTypeField& etherType) = 0;

};//InterfaceOutputQueue//


//--------------------------------------------------------------------------------------------------

class FifoInterfaceOutputQueue : public InterfaceOutputQueue {
public:
    FifoInterfaceOutputQueue(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceId& theInterfaceId,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const string& parameterNamePrefix = "interface-output-queue-");

    bool IsEmpty() const { return theQueue.empty(); }

    void Insert(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop,
        const EtherTypeField etherType = ETHERTYPE_IS_NOT_SPECIFIED);

    void DequeuePacket(
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        PacketPriority& priority,
        EtherTypeField& etherType);

private:

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    struct OutputQueueRecord {
        OutputQueueRecord(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress,
            const PacketPriority& initTypeOfService,
            const EtherTypeField initEtherType)
            :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress),
            trafficClass(initTypeOfService),
            etherType(initEtherType)
        {
        }

        OutputQueueRecord(OutputQueueRecord&& right) :
            packetPtr(move(right.packetPtr)),
            nextHopAddress(right.nextHopAddress),
            trafficClass(right.trafficClass),
            etherType(right.etherType) {}

        unique_ptr<Packet> packetPtr;
        NetworkAddress nextHopAddress;
        PacketPriority trafficClass;
        EtherTypeField etherType;
    };

    std::queue<OutputQueueRecord> theQueue;

    unsigned int maxNumberPackets;
    unsigned int maxNumberBytes;
    size_t currentNumberBytes;

};//FifoInterfaceOutputQueue//


inline
FifoInterfaceOutputQueue::FifoInterfaceOutputQueue(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceId& theInterfaceId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const string& parameterNamePrefix)
    :
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    maxNumberPackets(0),
    maxNumberBytes(0),
    currentNumberBytes(0)
{
    const NodeId theNodeId = simEngineInterfacePtr->GetNodeId();

    if (theParameterDatabaseReader.ParameterExists((parameterNamePrefix + "max-packets"),
        theNodeId, theInterfaceId)) {

        maxNumberPackets =
            theParameterDatabaseReader.ReadNonNegativeInt((parameterNamePrefix + "max-packets"), theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists((parameterNamePrefix + "max-bytes"), theNodeId, theInterfaceId)) {
        maxNumberBytes =
            theParameterDatabaseReader.ReadNonNegativeInt((parameterNamePrefix + "max-bytes"), theNodeId, theInterfaceId);
    }//if//

}//FifoInterfaceOutputQueue()//


inline
void FifoInterfaceOutputQueue::Insert(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority priority,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDrop,
    const EtherTypeField etherType)
{
    //cout << "Insert1" << endl;//no

    assert((maxNumberPackets == 0) || (theQueue.size() <= maxNumberPackets));
    assert((maxNumberBytes == 0) || (currentNumberBytes <= maxNumberBytes));

    const size_t packetSizeBytes = packetPtr->LengthBytes();

    if ((maxNumberPackets != 0) && (theQueue.size() == maxNumberPackets)) {
        enqueueResult = ENQUEUE_FAILURE_BY_MAX_PACKETS;
        packetToDrop = move(packetPtr);
    }
    else if ((maxNumberBytes != 0) && ((currentNumberBytes + packetSizeBytes) > maxNumberBytes)) {
        enqueueResult = ENQUEUE_FAILURE_BY_MAX_BYTES;
        packetToDrop = move(packetPtr);
    }
    else {
        enqueueResult = ENQUEUE_SUCCESS;
        currentNumberBytes += packetSizeBytes;
        theQueue.push(move(OutputQueueRecord(packetPtr, nextHopAddress, priority, etherType)));
    }//if//

}//Insert//


inline
void FifoInterfaceOutputQueue::DequeuePacket(
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    PacketPriority& priority,
    EtherTypeField& etherType)
{
    //cout << "fifo" << endl;//no
    assert(!theQueue.empty());

    OutputQueueRecord& queueRecord = theQueue.front();

    packetPtr = move(queueRecord.packetPtr);
    nextHopAddress = queueRecord.nextHopAddress;
    priority = queueRecord.trafficClass;
    etherType = queueRecord.etherType;
    theQueue.pop();

    currentNumberBytes -= packetPtr->LengthBytes();

}//DequeuePacket//




//--------------------------------------------------------------------------------------------------

class AbstractOutputQueueWithPrioritySubqueues: public InterfaceOutputQueue {
public:
    virtual void Insert(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop,
        const EtherTypeField etherType = ETHERTYPE_IS_NOT_SPECIFIED) override = 0;

    virtual void RequeueAtFront(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        const EtherTypeField etherType,
        const SimTime& queueInsertionTime,
        const unsigned int retryTxCount,
        const unsigned short int sequenceNumber) = 0;

    virtual PacketPriority MaxPossiblePacketPriority() const override = 0;

    virtual bool IsEmpty() const override = 0;

    virtual unsigned int NumberPackets() const = 0;

    virtual unsigned long long int NumberPacketBytes() const = 0;

    virtual bool HasPacketWithPriority(const PacketPriority priority) const = 0;

    virtual unsigned int NumberPacketsWithPriority(const PacketPriority priority) const = 0;
    virtual unsigned long long int NumberPacketBytesForPriority(const PacketPriority priority) const = 0;

    virtual const Packet& TopPacket(const PacketPriority priority) const = 0;
    virtual const NetworkAddress& NextHopAddressForTopPacket(const PacketPriority priority) const = 0;
    //Jay virtual const SimTime& InsertionTimeForTopPacket(const PacketPriority priority) const = 0;


    virtual void DequeuePacket(
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        PacketPriority& priority,
        EtherTypeField& etherType) override = 0;


    virtual void DequeuePacketWithPriority(
        const PacketPriority& priority,
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        EtherTypeField& etherType,
        SimTime& queueInsertionTime,
        bool& isANewPacket,
        unsigned int& retryTxCount,
        unsigned short int& sequenceNumber) = 0;

    // Allows extracting frames to a destination in non-FIFO order.  For frame aggregation.

    virtual void EnableNextHopSpecificDequeues() = 0;

    virtual bool NextHopSpecificDequeueIsEnabled() const = 0;

    virtual bool HasPacketWithPriorityAndNextHop(
        const PacketPriority priority,
        const NetworkAddress& nextHopAddress) const = 0;

    // These methods still work in FIFO mode, but the next hop address must match
    // top of queue (assert).

    virtual const Packet& GetNextPacketWithPriorityAndNextHop(
        const PacketPriority priority,
        const NetworkAddress& nextHopAddress) const = 0;

    virtual bool NextPacketIsARetry(
        const PacketPriority priority,
        const NetworkAddress& nextHopAddress) const = 0;

    virtual void DequeuePacketWithPriorityAndNextHop(
        const PacketPriority& priority,
        const NetworkAddress& nextHopAddress,
        unique_ptr<Packet>& packetPtr,
        EtherTypeField& etherType,
        SimTime& queueInsertionTime,
        bool& isANewPacket,
        unsigned int& retryTxCount,
        unsigned short int& sequenceNumber) = 0;

    // Round robin support (Assuming order based on address).

    virtual NetworkAddress GetNetworkAddressOfNextActiveStationAfter(
        const PacketPriority& priority,
        const NetworkAddress& address) const = 0;


    // For Getting rid of expired packets.

    struct ExpiredPacketInfoType {
        unique_ptr<Packet> packetPtr;
        NetworkAddress nextHopAddress;

        ExpiredPacketInfoType(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress)
        :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress)
        {}

        void operator=(ExpiredPacketInfoType&& right)
        {
            packetPtr = move(right.packetPtr);
            nextHopAddress = right.nextHopAddress;
        }

        ExpiredPacketInfoType(ExpiredPacketInfoType&& right) { (*this) = move(right); }
    };

    virtual void DequeueLifetimeExpiredPackets(
        const PacketPriority& priority,
        const SimTime& packetsInsertedBeforeTime,
        vector<ExpiredPacketInfoType>& packetList) = 0;

    // For delayed Block ACKed packets.

    virtual void DeletePacketsBySequenceNumber(
        const PacketPriority& priority,
        const NetworkAddress& nextHopAddress,
        const vector<unsigned short int>& sequenceNumberList) = 0;

};//AbstractOutputQueueWithPrioritySubqueues//




//--------------------------------------------------------------------------------------------------

class OutputQueueWithPrioritySubqueues: public AbstractOutputQueueWithPrioritySubqueues {
public:

    OutputQueueWithPrioritySubqueues(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceId& theInterfaceId,
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
        const PacketPriority& maximumPriority,
        const shared_ptr<OutputQueuePriorityMapper>& priorityMapperPtr);

    virtual void Insert(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop,
        const EtherTypeField etherType = ETHERTYPE_IS_NOT_SPECIFIED) override;

    void RequeueAtFront(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        const EtherTypeField etherType,
        const SimTime& queueInsertionTime,
        const unsigned int retryTxCount,
        const unsigned short int sequenceNumber) override;

    virtual PacketPriority MaxPossiblePacketPriority() const override { return (maximumPriority); }

    virtual bool IsEmpty() const override { return (totalPackets == 0); }

    virtual unsigned int NumberPackets() const override { return (totalPackets); }

    virtual unsigned long long int NumberPacketBytes() const override { return (totalPacketBytes); }

    virtual bool HasPacketWithPriority(const PacketPriority priority) const override
    {
        assert(priority <= maximumPriority);
        //return (!outputSubqueues.at(priority).fifoQueue.empty());
        return (!outputSubqueues.at(priority).priorityQueue.empty());
    }

    virtual unsigned int NumberPacketsWithPriority(const PacketPriority priority) const override
    {
        assert(priority <= maximumPriority);
        //return (static_cast<unsigned int>(outputSubqueues.at(priority).fifoQueue.size()));
        return (static_cast<unsigned int>(outputSubqueues.at(priority).priorityQueue.size()));
    }

    virtual unsigned long long int NumberPacketBytesForPriority(const PacketPriority priority) const override
    {
        return (outputSubqueues.at(priority).currentNumberBytes);
    }

    const Packet& TopPacket(const PacketPriority priority) const override;
    const NetworkAddress& NextHopAddressForTopPacket(const PacketPriority priority) const override;

    virtual NetworkAddress GetNetworkAddressOfNextActiveStationAfter(
        const PacketPriority& priority,
        const NetworkAddress& address) const override;

    virtual void DequeuePacket(
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        PacketPriority& priority,
        EtherTypeField& etherType) override;

    virtual void DequeuePacketWithPriority(
        const PacketPriority& priority,
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        EtherTypeField& etherType,
        SimTime& queueInsertionTime,
        bool& isANewPacket,
        unsigned int& retryTxCount,
        unsigned short int& sequenceNumber) override;

    // Allows extracting frames to a destination in non-FIFO order.  For frame aggregation.

    virtual void EnableNextHopSpecificDequeues() override {
        assert(IsEmpty());
        (*this).nextHopSpecificQueuesAreEnabled = true;
    }

    virtual bool NextHopSpecificDequeueIsEnabled() const  override { return (nextHopSpecificQueuesAreEnabled); }

    virtual bool HasPacketWithPriorityAndNextHop(
        const PacketPriority priority,
        const NetworkAddress& nextHopAddress) const override;

    // These methods still work in FIFO mode, but the next hop address must match
    // top of queue (assert).

    virtual const Packet& GetNextPacketWithPriorityAndNextHop(
        const PacketPriority priority,
        const NetworkAddress& nextHopAddress) const override;

    virtual bool NextPacketIsARetry(
        const PacketPriority priority,
        const NetworkAddress& nextHopAddress) const override;

    virtual void DequeuePacketWithPriorityAndNextHop(
        const PacketPriority& priority,
        const NetworkAddress& nextHopAddress,
        unique_ptr<Packet>& packetPtr,
        EtherTypeField& etherType,
        SimTime& queueInsertionTime,
        bool& isANewPacket,
        unsigned int& retryTxCount,
        unsigned short int& sequenceNumber) override;

    virtual void DequeueLifetimeExpiredPackets(
        const PacketPriority& priority,
        const SimTime& packetsInsertedBeforeTime,
        vector<ExpiredPacketInfoType>& packetList) override;

    virtual void DeletePacketsBySequenceNumber(
        const PacketPriority& priority,
        const NetworkAddress& nextHopAddress,
        const vector<unsigned short int>& sequenceNumberList) override;

private:

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    shared_ptr<OutputQueuePriorityMapper> priorityMapperPtr;

    struct OutputQueueRecordType {
        OutputQueueRecordType(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress,
            const EtherTypeField initEtherType,
            const SimTime& initQueueInsertionTime)
            :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress),
            etherType(initEtherType),
            queueInsertionTime(initQueueInsertionTime),
            retryTxCount(0),
            sequenceNumber(0),
            isARequeuedPacket(false)
        {}

        OutputQueueRecordType(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress,
            const EtherTypeField initEtherType,
            const SimTime& initQueueInsertionTime,
            const unsigned int initRetryTxCount,
            const unsigned short int initSequenceNumber)
            :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress),
            etherType(initEtherType),
            queueInsertionTime(initQueueInsertionTime),
            retryTxCount(initRetryTxCount),
            sequenceNumber(initSequenceNumber),
            isARequeuedPacket(true)
        {}

        void operator=(OutputQueueRecordType&& right) {
            assert(this != &right);
            packetPtr = move(right.packetPtr);
            nextHopAddress = right.nextHopAddress;
            etherType = right.etherType;
            queueInsertionTime = right.queueInsertionTime;
            retryTxCount = right.retryTxCount;
            sequenceNumber = right.sequenceNumber;
            isARequeuedPacket = right.isARequeuedPacket;
        }

        OutputQueueRecordType(OutputQueueRecordType&& right)  { (*this) = move(right); }

        unique_ptr<Packet> packetPtr;
        SimTime queueInsertionTime;
        NetworkAddress nextHopAddress;
        unsigned int retryTxCount;
        EtherTypeField etherType;
        unsigned short int sequenceNumber;
        bool isARequeuedPacket;

    };//OutputQueueRecordType//


    struct DestinationSpecificInfoType {

        // Warning: Raw pointers used here for speed.  Records are owned by "fifoQueue".

        std::deque<OutputQueueRecordType*> aQueue;
        SimTime transitionedToEmptyTime;

        DestinationSpecificInfoType(): transitionedToEmptyTime(INFINITE_TIME) { }
    };


    struct OutputSubqueueInfoType {

        unsigned long long int currentNumberBytes;

        // Warning: Records are owned by "fifoQueue" and raw pointers are used in
        // "destinationSpecificInfos"

        //std::deque<unique_ptr<OutputQueueRecordType> > fifoQueue;
        std::deque<unique_ptr<OutputQueueRecordType> > priorityQueue;//dcc

        map<NetworkAddress, DestinationSpecificInfoType> destinationSpecificInfos;

        OutputSubqueueInfoType() : currentNumberBytes(0) { }

        void operator=(OutputSubqueueInfoType&& right) {
            currentNumberBytes = right.currentNumberBytes;
            //fifoQueue = move(right.fifoQueue);
            priorityQueue = move(right.priorityQueue);
            destinationSpecificInfos = move(right.destinationSpecificInfos);
        }

        OutputSubqueueInfoType(OutputSubqueueInfoType&& right) { (*this) = move(right); }

    };//OutputSubqueueInfoType//


    bool nextHopSpecificQueuesAreEnabled;

    PacketPriority maximumPriority;

    unsigned int totalPackets;
    unsigned long long int totalPacketBytes;

    unsigned int subqueueMaxPackets;
    unsigned long long int subqueueMaxBytes;

    vector<OutputSubqueueInfoType> outputSubqueues;

    // Disable:

    OutputQueueWithPrioritySubqueues(const OutputQueueWithPrioritySubqueues&);
    void operator=(const OutputQueueWithPrioritySubqueues&);

};//OutputQueueWithPrioritySubqueues//

//----------------------------------------------------------

inline
OutputQueueWithPrioritySubqueues::OutputQueueWithPrioritySubqueues(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceId& theInterfaceId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const PacketPriority& initMaximumPriority,
    const shared_ptr<OutputQueuePriorityMapper>& initPriorityMapperPtr)
    :
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    priorityMapperPtr(initPriorityMapperPtr),
    maximumPriority(initMaximumPriority),
    nextHopSpecificQueuesAreEnabled(false),
    totalPackets(0),
    totalPacketBytes(0),
    outputSubqueues(initMaximumPriority + 1),
    subqueueMaxPackets(0),
    subqueueMaxBytes(0)
{
    const NodeId theNodeId = simEngineInterfacePtr->GetNodeId();

    if (theParameterDatabaseReader.ParameterExists("interface-output-queue-max-packets-per-subq", theNodeId, theInterfaceId)){
        subqueueMaxPackets =
            theParameterDatabaseReader.ReadNonNegativeInt("interface-output-queue-max-packets-per-subq", theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("interface-output-queue-max-bytes-per-subq", theNodeId, theInterfaceId)){
        subqueueMaxBytes =
            theParameterDatabaseReader.ReadNonNegativeBigInt("interface-output-queue-max-bytes-per-subq", theNodeId, theInterfaceId);
    }//if//

    //cout << "OutputQueueWithPrioritySubqueues" << endl;//yes (before flooding) 

}//OutputQueueWithPrioritySubqueues()/


inline
const Packet& OutputQueueWithPrioritySubqueues::TopPacket(const PacketPriority priority) const
{
    assert(priority < outputSubqueues.size());
    //assert(!outputSubqueues[priority].fifoQueue.empty());
    assert(!outputSubqueues[priority].priorityQueue.empty());

    //cout << "TopPacket" << endl;//no

    //return (*outputSubqueues[priority].fifoQueue.front()->packetPtr);
    return (*outputSubqueues[priority].priorityQueue.front()->packetPtr);
}


inline
const NetworkAddress& OutputQueueWithPrioritySubqueues::NextHopAddressForTopPacket(
    const PacketPriority priority) const
{
    assert(priority < outputSubqueues.size());
    //assert(!outputSubqueues[priority].fifoQueue.empty());
    assert(!outputSubqueues[priority].priorityQueue.empty());

    //cout << "NextHopAddressForTopPacket" << endl;//no

    //return (outputSubqueues[priority].fifoQueue.front()->nextHopAddress);
    return (outputSubqueues[priority].priorityQueue.front()->nextHopAddress);
}

inline
NetworkAddress OutputQueueWithPrioritySubqueues::GetNetworkAddressOfNextActiveStationAfter(
    const PacketPriority& priority,
    const NetworkAddress& address) const
{
    assert(priority < outputSubqueues.size());
    const OutputSubqueueInfoType& outputSubqueue = outputSubqueues.at(priority);

    assert(nextHopSpecificQueuesAreEnabled);
    assert(!outputSubqueue.destinationSpecificInfos.empty());

    if (outputSubqueue.destinationSpecificInfos.size() == 1) {

        // Only one station. Next station is the station.

        assert(!outputSubqueue.destinationSpecificInfos.begin()->second.aQueue.empty());

        return(outputSubqueue.destinationSpecificInfos.begin()->first);
    }//if//

    typedef map<NetworkAddress, DestinationSpecificInfoType>::const_iterator IterType;

    // upper_bound() returns next address or the end().

    const IterType startIter = outputSubqueue.destinationSpecificInfos.upper_bound(address);

    IterType iter = startIter;

    // Search for non-empty queue in order (wrapped): forloop only stops infinite loop.

    while (true) {
        if (iter == outputSubqueue.destinationSpecificInfos.end()) {
            iter = outputSubqueue.destinationSpecificInfos.begin();
        }//if//

        if (!iter->second.aQueue.empty()) {
            return (iter->first);
        }//if//

        ++iter;

        assert((iter != startIter) && "No non-empty queues were found!");

    }//while//

    //cout << "GetNetworkAddressOfNextActiveStationAfter" << endl;//no

    assert(false); abort();  return (NetworkAddress());

}//GetNetworkAddressOfNextActiveStationAfter//


inline
bool OutputQueueWithPrioritySubqueues::HasPacketWithPriorityAndNextHop(
    const PacketPriority priority,
    const NetworkAddress& nextHopAddress) const
{
    assert(nextHopSpecificQueuesAreEnabled);
    assert(priority < outputSubqueues.size());
    const OutputSubqueueInfoType& outputSubqueue = outputSubqueues.at(priority);

    typedef map<NetworkAddress, DestinationSpecificInfoType>::const_iterator IterType;
    const IterType iter = outputSubqueue.destinationSpecificInfos.find(nextHopAddress);

    if (iter == outputSubqueue.destinationSpecificInfos.end()) {
        return false;
    }//if//

    //cout << "HasPacketWithPriorityAndNextHop" << endl;//no

    return (!iter->second.aQueue.empty());

}//HasPacketWithPriorityAndNextHop//



inline
const Packet& OutputQueueWithPrioritySubqueues::GetNextPacketWithPriorityAndNextHop(
    const PacketPriority priority,
    const NetworkAddress& nextHopAddress) const
{
    assert(priority < outputSubqueues.size());
    const OutputSubqueueInfoType& outputSubqueue = outputSubqueues.at(priority);

    if (!nextHopSpecificQueuesAreEnabled) {
        //assert((outputSubqueue.fifoQueue.front()->nextHopAddress == nextHopAddress) &&
        assert((outputSubqueue.priorityQueue.front()->nextHopAddress == nextHopAddress) &&
               "Access must be strictly FIFO without destination specific queues.");

        //return (*outputSubqueue.fifoQueue.front()->packetPtr);
        return (*outputSubqueue.priorityQueue.front()->packetPtr);
    }//if//

    typedef map<NetworkAddress, DestinationSpecificInfoType>::const_iterator IterType;

    const IterType iter = outputSubqueue.destinationSpecificInfos.find(nextHopAddress);
    assert(iter != outputSubqueue.destinationSpecificInfos.end());

    //cout << "GetNextPacketWithPriorityAndNextHop" << endl;//no

    return (*iter->second.aQueue.front()->packetPtr);

}//GetNextPacketWithPriorityAndNextHop//



inline
bool OutputQueueWithPrioritySubqueues::NextPacketIsARetry(
    const PacketPriority priority,
    const NetworkAddress& nextHopAddress) const
{
    assert(priority < outputSubqueues.size());

    const OutputSubqueueInfoType& outputSubqueue = outputSubqueues.at(priority);

    if (!nextHopSpecificQueuesAreEnabled) {
        //assert((outputSubqueue.fifoQueue.front()->nextHopAddress == nextHopAddress) &&
        assert((outputSubqueue.priorityQueue.front()->nextHopAddress == nextHopAddress) &&
               "Access must be strictly FIFO without destination specific queues.");

        /*if (outputSubqueue.fifoQueue.empty()) {
            return false;
        }//if//*/

        if (outputSubqueue.priorityQueue.empty()) {
            return false;
        }//if//

        //return (outputSubqueue.fifoQueue.front()->retryTxCount > 0);
        return (outputSubqueue.priorityQueue.front()->retryTxCount > 0);
    }//if//

    typedef map<NetworkAddress, DestinationSpecificInfoType>::const_iterator IterType;

    const IterType iter = outputSubqueue.destinationSpecificInfos.find(nextHopAddress);

    if ((iter == outputSubqueue.destinationSpecificInfos.end()) ||
        (iter->second.aQueue.empty())) {

        return false;
    }//if//

    //cout << "NextPacketWithPriorityAndNextHop" << endl;//no

    return (iter->second.aQueue.front()->retryTxCount > 0);

}//NextPacketWithPriorityAndNextHop//



inline
void OutputQueueWithPrioritySubqueues::Insert(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority ipPriority,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDrop,
    const EtherTypeField etherType)
{
    //cout << "Insert2" << endl;//yes
    //cout << "ippriority: " << (int)ipPriority << endl;
    /*for(int i = 0; i < outputSubqueues.size(); ++i){
        cout << "queue: " << i << endl; //size=4
    }*/

    PacketPriority priority = ipPriority;
    //cout << "Insert3" << endl;
    /*if (priorityMapperPtr != nullptr) {
        priority = priorityMapperPtr->MapIpToMacPriority(ipPriority);
    }//if//*/
    //cout << "Insert4" << endl;

    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);
    //cout << "Insert5" << endl;
    //cout << "priority: " << priority << endl;
    //cout << "priority_int: " << (int)priority << endl;
    //if ((subqueueMaxPackets != 0) && (queueInfo.fifoQueue.size() >= subqueueMaxPackets)) {
    if ((subqueueMaxPackets != 0) && (queueInfo.priorityQueue.size() >= subqueueMaxPackets)) {
        enqueueResult = ENQUEUE_FAILURE_BY_MAX_PACKETS;
        packetToDrop = move(packetPtr);
        cout << "packet overflow" << endl;
    }
    else if ((subqueueMaxBytes != 0) &&
             ((queueInfo.currentNumberBytes + packetPtr->LengthBytes()) > subqueueMaxBytes)) {

        enqueueResult = ENQUEUE_FAILURE_BY_MAX_BYTES;
        packetToDrop = move(packetPtr);
        cout << "byte overflow" << endl;
    }
    else {
        enqueueResult = ENQUEUE_SUCCESS;
        packetToDrop = nullptr;
        totalPacketBytes += packetPtr->LengthBytes();
        queueInfo.currentNumberBytes += packetPtr->LengthBytes();
        totalPackets++;
        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
        //queueInfo.fifoQueue.push_back(
        cout << "priority_insert_before: " << (int)priority << ", queuesize_insert_before: " << queueInfo.priorityQueue.size() << endl;
        queueInfo.priorityQueue.push_back(
            unique_ptr<OutputQueueRecordType>(
                new OutputQueueRecordType(packetPtr, nextHopAddress, etherType, currentTime)));
        cout << "priority_insert_after: " << (int)priority << ", queuesize_insert_after: " << queueInfo.priorityQueue.size() << endl;
        if (nextHopSpecificQueuesAreEnabled) {

            // Also add to destination specific queue.

            queueInfo.destinationSpecificInfos[nextHopAddress].aQueue.push_back(
                //queueInfo.fifoQueue.back().get());
                queueInfo.priorityQueue.back().get());
        }//if//
    }//if//
    //cout << "Insert6" << endl;
    //cout << queueInfo.fifoQueue.size() << endl;
    //DCC queue
    /*if(priority == 1){
        cout << "A" << endl;
    }else if(priority == 2){
        cout << "B" << endl;
    }else if(priority == 3){
        cout << "C" << endl;
    }else if(priority == 4){
        cout << "D" << endl;
    }*/
    //DCC queue

}//Insert//



inline
void OutputQueueWithPrioritySubqueues::RequeueAtFront(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority priority,
    const EtherTypeField etherType,
    const SimTime& queueInsertionTime,
    const unsigned int retryTxCount,
    const unsigned short int sequenceNumber)
{
    assert(priority < outputSubqueues.size());

    const unsigned int packetLengthBytes = packetPtr->LengthBytes();
    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    //Can overstuff

    //queueInfo.fifoQueue.push_front(
    queueInfo.priorityQueue.push_front(
        unique_ptr<OutputQueueRecordType>(
            new OutputQueueRecordType(
                packetPtr, nextHopAddress, etherType,
                queueInsertionTime, retryTxCount, sequenceNumber)));

    queueInfo.currentNumberBytes += packetLengthBytes;
    totalPackets++;
    totalPacketBytes += packetLengthBytes;

    if (nextHopSpecificQueuesAreEnabled) {
        // Also add to destination specific.
        queueInfo.destinationSpecificInfos[nextHopAddress].aQueue.push_front(
            //queueInfo.fifoQueue.front().get());
            queueInfo.priorityQueue.front().get());
    }//if//

    assert(packetPtr == nullptr);

    //cout << "InsertAtFront" << endl;//no

}//InsertAtFront//



inline
void OutputQueueWithPrioritySubqueues::DequeuePacketWithPriority(
    const PacketPriority& priority,
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    EtherTypeField& etherType,
    SimTime& queueInsertionTime,
    bool& isANewPacket,
    unsigned int& retryTxCount,
    unsigned short int& sequenceNumber)
{
    assert(priority < outputSubqueues.size());

    //cout << "priority_max_size: " << outputSubqueues.size() << endl;//size = 4

    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    //OutputQueueRecordType& queueRecord = *queueInfo.fifoQueue.front();
    OutputQueueRecordType& queueRecord = *queueInfo.priorityQueue.front();
    packetPtr = move(queueRecord.packetPtr);
    nextHopAddress = queueRecord.nextHopAddress;
    etherType = queueRecord.etherType;
    queueInsertionTime = queueRecord.queueInsertionTime;
    isANewPacket = !queueRecord.isARequeuedPacket;
    retryTxCount = queueRecord.retryTxCount;
    sequenceNumber = queueRecord.sequenceNumber;

    if (nextHopSpecificQueuesAreEnabled) {

        DestinationSpecificInfoType& destinationInfo =
            queueInfo.destinationSpecificInfos[nextHopAddress];
        std::deque<OutputQueueRecordType*>& destSpecificQueue = destinationInfo.aQueue;

        //assert(destSpecificQueue.front() == queueInfo.fifoQueue.front().get());
        assert(destSpecificQueue.front() == queueInfo.priorityQueue.front().get());
        destSpecificQueue.pop_front();

        if (destSpecificQueue.empty()) {
            destinationInfo.transitionedToEmptyTime = simEngineInterfacePtr->CurrentTime();
        }//if//
    }//if//

    //queueInfo.fifoQueue.pop_front();
    cout << "priority_dequeue_before: " << (int)priority << ", queuesize_dequeue_before: " << queueInfo.priorityQueue.size() << endl;
    queueInfo.priorityQueue.pop_front();
    cout << "priority_dequeue_after: " << (int)priority << ", queuesize_dequeue_after: " << queueInfo.priorityQueue.size() << endl;
    //cout << "DequeuePacketWithPriority" << endl;//yes

    queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
    totalPackets--;
    totalPacketBytes -= packetPtr->LengthBytes();

    // Cleanup:

    if (nextHopSpecificQueuesAreEnabled) {
        /*while((!queueInfo.fifoQueue.empty()) && (queueInfo.fifoQueue.front()->packetPtr == nullptr)) {
            queueInfo.fifoQueue.pop_front();
        }//while//*/
        while((!queueInfo.priorityQueue.empty()) && (queueInfo.priorityQueue.front()->packetPtr == nullptr)) {
            queueInfo.priorityQueue.pop_front();
        }//while//
    }//if//

}//DequeuePacketWithPriority//


inline
void OutputQueueWithPrioritySubqueues::DequeuePacketWithPriorityAndNextHop(
    const PacketPriority& priority,
    const NetworkAddress& nextHopAddress,
    unique_ptr<Packet>& packetPtr,
    EtherTypeField& etherType,
    SimTime& queueInsertionTime,
    bool& isANewPacket,
    unsigned int& retryTxCount,
    unsigned short int& sequenceNumber)
{
    if (!nextHopSpecificQueuesAreEnabled) {
        NetworkAddress actualNextHopAddress;

        (*this).DequeuePacketWithPriority(
            priority,
            packetPtr,
            actualNextHopAddress,
            etherType,
            queueInsertionTime,
            isANewPacket,
            retryTxCount,
            sequenceNumber);

        assert(actualNextHopAddress == nextHopAddress);
        return;

    }//if//

    assert(priority < outputSubqueues.size());

    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    DestinationSpecificInfoType& destinationInfo = queueInfo.destinationSpecificInfos[nextHopAddress];
    std::deque<OutputQueueRecordType*>& destinationSpecificQueue = destinationInfo.aQueue;

    assert(!destinationSpecificQueue.empty());

    OutputQueueRecordType& queueRecord = *destinationSpecificQueue.front();

    packetPtr = move(queueRecord.packetPtr);
    etherType = queueRecord.etherType;
    queueInsertionTime = queueRecord.queueInsertionTime;
    isANewPacket = !queueRecord.isARequeuedPacket;
    retryTxCount = queueRecord.retryTxCount;
    sequenceNumber = queueRecord.sequenceNumber;

    destinationSpecificQueue.pop_front();

    queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
    totalPackets--;
    totalPacketBytes -= packetPtr->LengthBytes();

    if (destinationSpecificQueue.empty()) {
        destinationInfo.transitionedToEmptyTime = simEngineInterfacePtr->CurrentTime();
    }//if//

    // Cleanup

    /*while((!queueInfo.fifoQueue.empty()) && (queueInfo.fifoQueue.front()->packetPtr == nullptr)) {
        queueInfo.fifoQueue.pop_front();
        //cout << "DequeuePacketWithPriorityAndNextHop" << endl;//no
    }//while//*/

    while((!queueInfo.priorityQueue.empty()) && (queueInfo.priorityQueue.front()->packetPtr == nullptr)) {
        queueInfo.priorityQueue.pop_front();
        //cout << "DequeuePacketWithPriorityAndNextHop" << endl;//no
    }//while//

    //cout << "DequeuePacketWithPriorityAndNextHop" << endl;//no

}//DequeuePacketWithPriorityAndNextHop//



inline
void OutputQueueWithPrioritySubqueues::DequeuePacket(
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    PacketPriority& priority,
    EtherTypeField& etherType)
{
    //cout << "prioritydequeue" << endl;//no
    size_t i = outputSubqueues.size() - 1;
    while(true) {
        OutputSubqueueInfoType& queueInfo = outputSubqueues.at(i);

        //if (!queueInfo.fifoQueue.empty()) {
        if (!queueInfo.priorityQueue.empty()) {

            priority = PacketPriority(i);
            SimTime notUsed1;
            bool isANewPacket;
            unsigned int notUsed2;
            unsigned short int notUsed3;

            (*this).DequeuePacketWithPriority(
                priority, packetPtr, nextHopAddress, etherType,
                notUsed1, isANewPacket, notUsed2, notUsed3);

            assert(isANewPacket);

            return;
        }//if//

        if (i == 0) {
            break;
        }//if//

        i--;

    }//while//

    assert(false && "Program Error: All Queues are Empty!"); abort();

}//DequeuePacket//



inline
void OutputQueueWithPrioritySubqueues::DequeueLifetimeExpiredPackets(
    const PacketPriority& priority,
    const SimTime& packetsInsertedBeforeTime,
    vector<ExpiredPacketInfoType>& packetList)
{
    packetList.clear();
    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    if (!nextHopSpecificQueuesAreEnabled) {

        while((!queueInfo./*fifoQueue*/priorityQueue.empty()) &&
            (queueInfo./*fifoQueue*/priorityQueue.front()->queueInsertionTime < packetsInsertedBeforeTime)) {

            OutputQueueRecordType& info = *queueInfo./*fifoQueue*/priorityQueue.front();

            queueInfo.currentNumberBytes -= info.packetPtr->LengthBytes();
            totalPackets--;
            totalPacketBytes -= info.packetPtr->LengthBytes();

            packetList.push_back(ExpiredPacketInfoType(info.packetPtr, info.nextHopAddress));
            queueInfo./*fifoQueue*/priorityQueue.pop_front();
            //cout << "DequeueLifetimeExpiredPackets1" << endl;//no

        }//while//
    }
    else {
        typedef map<NetworkAddress, DestinationSpecificInfoType>::iterator IterType;

        for(IterType iter = queueInfo.destinationSpecificInfos.begin();
            (iter != queueInfo.destinationSpecificInfos.end()); ++iter) {

            DestinationSpecificInfoType& destInfo = iter->second;

            while((destInfo.aQueue.empty()) &&
                  (destInfo.aQueue.front()->queueInsertionTime < packetsInsertedBeforeTime)) {

                OutputQueueRecordType& record = *destInfo.aQueue.front();

                queueInfo.currentNumberBytes -= record.packetPtr->LengthBytes();
                totalPackets--;
                totalPacketBytes -= record.packetPtr->LengthBytes();

                packetList.push_back(ExpiredPacketInfoType(record.packetPtr, record.nextHopAddress));
                queueInfo./*fifoQueue*/priorityQueue.pop_front();
                //cout << "DequeueLifetimeExpiredPackets2" << endl;//no

            }//for//
        }//for//

        // Cleanup

        while((!queueInfo./*fifoQueue*/priorityQueue.empty()) && (queueInfo./*fifoQueue*/priorityQueue.front()->packetPtr == nullptr)) {
            queueInfo./*fifoQueue*/priorityQueue.pop_front();
            //cout << "DequeueLifetimeExpiredPackets3" << endl;//no
        }//while//

    }//if//

    //cout << "DequeueLifetimeExpiredPackets" << endl;//no

}//DequeueLifetimeExpiredPackets//



inline
bool SequenceNumberListIsInOrder(const vector<unsigned short int>& sequenceNumberList)
{
    for(unsigned int i = 1; (i < sequenceNumberList.size()); i++) {
        if (!TwelveBitSequenceNumberIsLessThan(sequenceNumberList[i-1], sequenceNumberList[i])) {
            return false;
        }//if//
    }//for//
    return true;
}


inline
void OutputQueueWithPrioritySubqueues::DeletePacketsBySequenceNumber(
    const PacketPriority& priority,
    const NetworkAddress& nextHopAddress,
    const vector<unsigned short int>& sequenceNumberList)
{
    assert(SequenceNumberListIsInOrder(sequenceNumberList));

    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    DestinationSpecificInfoType& destinationInfo = queueInfo.destinationSpecificInfos[nextHopAddress];
    std::deque<OutputQueueRecordType*>& destinationSpecificQueue = destinationInfo.aQueue;

    unsigned int i = 0;
    unsigned int pos = 0;

    while ((i < sequenceNumberList.size()) && (pos < destinationSpecificQueue.size())) {
        OutputQueueRecordType& record = *destinationSpecificQueue[pos];
        if (!record.isARequeuedPacket) {
            break;
        }//if//

        if (record.packetPtr == nullptr) {
            pos++;
        }
        else if (record.sequenceNumber == sequenceNumberList[i]) {
            queueInfo.currentNumberBytes -= record.packetPtr->LengthBytes();
            totalPackets--;
            totalPacketBytes -= record.packetPtr->LengthBytes();
            record.packetPtr.reset();
            i++;
            pos++;
        }
        else if (TwelveBitSequenceNumberIsLessThan(sequenceNumberList[i], record.sequenceNumber)) {
            i++;
        }
        else {
            pos++;
        }//if//
    }//while//


    // Cleanup

    while((!queueInfo./*fifoQueue*/priorityQueue.empty()) && (queueInfo./*fifoQueue*/priorityQueue.front()->packetPtr == nullptr)) {
        queueInfo./*fifoQueue*/priorityQueue.pop_front();
        //cout << "DeletePacketsBySequenceNumber" << endl;//no
    }//while//

    //cout << "DeletePacketsBySequenceNumber" << endl;//no

}//DeletePacketsBySequenceNumber//



//--------------------------------------------------------------------------------------------------


class BasicOutputQueueWithPrioritySubqueues: public InterfaceOutputQueue {
public:
    BasicOutputQueueWithPrioritySubqueues(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceId& theInterfaceId,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const PacketPriority& initMaximumPriority);

    virtual void Insert(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop,
        const EtherTypeField etherType = ETHERTYPE_IS_NOT_SPECIFIED) override;

    virtual PacketPriority MaxPossiblePacketPriority() const override { return (maximumPriority); }

    virtual bool IsEmpty() const override { return (totalPackets == 0); }

    unsigned int NumberPackets() const { return (totalPackets); }

    unsigned long long int NumberPacketBytes() const { return (totalPacketBytes); }

    bool HasPacketWithPriority(const PacketPriority priority) const
    {
        assert(priority <= maximumPriority);
        return (!outputSubqueues.at(priority).aQueue.empty());
    }

    unsigned int NumberPacketsWithPriority(const PacketPriority priority) const
    {
        assert(priority <= maximumPriority);
        return (static_cast<unsigned int>(outputSubqueues.at(priority).aQueue.size()));
    }

    unsigned long long int NumberPacketBytesForPriority(const PacketPriority priority) const
    {
        return (outputSubqueues.at(priority).currentNumberBytes);
    }

    const Packet& TopPacket(const PacketPriority priority) const;

    const Packet& GetPacket(
        const PacketPriority priority,
        const unsigned int positionInSubqueue) const;

    const NetworkAddress NextHopForTopPacket(const PacketPriority priority) const;

    const NetworkAddress NextHopForPacket(
        const PacketPriority priority,
        const unsigned int positionInSubqueue) const;

    virtual void DequeuePacket(
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        PacketPriority& priority,
        EtherTypeField& etherType) override;

    virtual void DequeuePacketWithPriority(
        const PacketPriority& priority,
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        EtherTypeField& etherType,
        SimTime& timestamp,
        unsigned int& retryTxCount)
    {
        (*this).DequeuePacketWithPriorityAndPosition(
            priority, 0, packetPtr, nextHopAddress, etherType, timestamp, retryTxCount);
    }

    void DequeuePacketWithPriorityAndPosition(
        const PacketPriority& priority,
        const unsigned int positionInSubqueue,
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        EtherTypeField& etherType,
        SimTime& timestamp,
        unsigned int& retryTxCount);

private:

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    struct OutputQueueRecord {
        OutputQueueRecord(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress,
            const EtherTypeField initEtherType,
            const SimTime& initTimestamp,
            const unsigned int initRetryTxCount = 0)
            :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress),
            etherType(initEtherType),
            timestamp(initTimestamp),
            retryTxCount(initRetryTxCount)
        {
        }

        void operator=(OutputQueueRecord&& right) {
            assert(this != &right);
            packetPtr = move(right.packetPtr);
            nextHopAddress = right.nextHopAddress;
            etherType = right.etherType;
            timestamp = right.timestamp;
            retryTxCount = right.retryTxCount;
        }

        OutputQueueRecord(OutputQueueRecord&& right)  { (*this) = move(right); }

        unique_ptr<Packet> packetPtr;
        NetworkAddress nextHopAddress;
        EtherTypeField etherType;
        SimTime timestamp;
        unsigned int retryTxCount;

    };//OutputQueueRecord//

    struct OutputSubqueueInfo {
        unsigned long long int currentNumberBytes;
        std::deque<OutputQueueRecord> aQueue;

        OutputSubqueueInfo() : currentNumberBytes(0) { }

        void operator=(OutputSubqueueInfo&& right) {
            currentNumberBytes = right.currentNumberBytes;
            aQueue = move(right.aQueue);
        }

        OutputSubqueueInfo(OutputSubqueueInfo&& right) { (*this) = move(right); }
    };

    PacketPriority maximumPriority;

    unsigned int totalPackets;
    unsigned long long int totalPacketBytes;

    unsigned int subqueueMaxPackets;
    unsigned int subqueueMaxBytes;

    vector<OutputSubqueueInfo> outputSubqueues;

    // Disable:

    BasicOutputQueueWithPrioritySubqueues(const BasicOutputQueueWithPrioritySubqueues&);
    void operator=(const BasicOutputQueueWithPrioritySubqueues&);

};//BasicOutputQueueWithPrioritySubqueues//


//----------------------------------------------------------

inline
BasicOutputQueueWithPrioritySubqueues::BasicOutputQueueWithPrioritySubqueues(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceId& theInterfaceId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const PacketPriority& initMaximumPriority)
    :
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    maximumPriority(initMaximumPriority),
    totalPackets(0),
    totalPacketBytes(0),
    outputSubqueues(initMaximumPriority + 1),
    subqueueMaxPackets(0),
    subqueueMaxBytes(0)
{
    const NodeId theNodeId = simEngineInterfacePtr->GetNodeId();

    if (theParameterDatabaseReader.ParameterExists("interface-output-queue-max-packets-per-subq", theNodeId, theInterfaceId)){
        subqueueMaxPackets =
            theParameterDatabaseReader.ReadNonNegativeInt("interface-output-queue-max-packets-per-subq", theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("interface-output-queue-max-bytes-per-subq", theNodeId, theInterfaceId)){
        subqueueMaxBytes =
            theParameterDatabaseReader.ReadNonNegativeInt("interface-output-queue-max-bytes-per-subq", theNodeId, theInterfaceId);
    }//if//

}//BasicOutputQueueWithPrioritySubqueues()/


inline
const Packet& BasicOutputQueueWithPrioritySubqueues::TopPacket(const PacketPriority priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].aQueue.empty());

    return (*outputSubqueues[priority].aQueue.front().packetPtr);
}


inline
const Packet& BasicOutputQueueWithPrioritySubqueues::GetPacket(
    const PacketPriority priority,
    const unsigned int positionInSubqueue) const
{
    assert(priority <= maximumPriority);
    return (*outputSubqueues[priority].aQueue.at(positionInSubqueue).packetPtr);
}



inline
const NetworkAddress BasicOutputQueueWithPrioritySubqueues::NextHopForTopPacket(const PacketPriority priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].aQueue.empty());

    return (outputSubqueues[priority].aQueue.front().nextHopAddress);
}

inline
const NetworkAddress BasicOutputQueueWithPrioritySubqueues::NextHopForPacket(
    const PacketPriority priority,
    const unsigned int positionInSubqueue) const
{
    assert(priority <= maximumPriority);
    return (outputSubqueues[priority].aQueue.at(positionInSubqueue).nextHopAddress);
}


inline
void BasicOutputQueueWithPrioritySubqueues::Insert(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority priority,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDrop,
    const EtherTypeField etherType)
{
    //cout << "Insert2" << endl;//no
    
    assert(priority <= maximumPriority);

    OutputSubqueueInfo& queueInfo = outputSubqueues.at(priority);

    if ((subqueueMaxPackets != 0) && (queueInfo.aQueue.size() >= subqueueMaxPackets)) {
        enqueueResult = ENQUEUE_FAILURE_BY_MAX_PACKETS;
        packetToDrop = move(packetPtr);
    }
    else if ((subqueueMaxBytes != 0) && ((queueInfo.currentNumberBytes + packetPtr->LengthBytes()) > subqueueMaxBytes)) {
        enqueueResult = ENQUEUE_FAILURE_BY_MAX_BYTES;
        packetToDrop = move(packetPtr);
    }
    else {
        enqueueResult = ENQUEUE_SUCCESS;
        packetToDrop = nullptr;
        totalPacketBytes += packetPtr->LengthBytes();
        queueInfo.currentNumberBytes += packetPtr->LengthBytes();
        totalPackets++;
        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
        queueInfo.aQueue.push_back(
            move(OutputQueueRecord(packetPtr, nextHopAddress, etherType, currentTime)));
    }//if//

}//Insert//


inline
void BasicOutputQueueWithPrioritySubqueues::DequeuePacket(
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    PacketPriority& priority,
    EtherTypeField& etherType)
{
    //cout << "basic_priority" << endl;//no
    size_t i = outputSubqueues.size() - 1;
    while(true) {
        OutputSubqueueInfo& queueInfo = outputSubqueues.at(i);

        if (!queueInfo.aQueue.empty()) {
            OutputQueueRecord& queueRecord = queueInfo.aQueue.front();

            packetPtr = move(queueRecord.packetPtr);
            nextHopAddress = queueRecord.nextHopAddress;
            priority = PacketPriority(i);
            etherType = queueRecord.etherType;

            queueInfo.aQueue.pop_front();
            queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
            totalPackets--;
            totalPacketBytes -= packetPtr->LengthBytes();

            return;
        }//if//

        if (i == 0) {
            break;
        }//if//

        i--;

    }//while//

    assert(false && "Program Error: All Queues are Empty!"); abort();

}//DequeuePacket//



inline
void BasicOutputQueueWithPrioritySubqueues::DequeuePacketWithPriorityAndPosition(
    const PacketPriority& priority,
    const unsigned int positionInSubqueue,
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    EtherTypeField& etherType,
    SimTime& timestamp,
    unsigned int& retryTxCount)
{
    assert(priority <= maximumPriority);
    OutputSubqueueInfo& queueInfo = outputSubqueues.at(priority);

    OutputQueueRecord& queueRecord = queueInfo.aQueue.at(positionInSubqueue);
    packetPtr = move(queueRecord.packetPtr);
    nextHopAddress = queueRecord.nextHopAddress;
    etherType = queueRecord.etherType;
    timestamp = queueRecord.timestamp;
    retryTxCount = queueRecord.retryTxCount;

    if (positionInSubqueue == 0) {
        queueInfo.aQueue.pop_front();
    }
    else {
        queueInfo.aQueue.erase(queueInfo.aQueue.begin() + positionInSubqueue);
    }//if//

    queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
    totalPackets--;
    totalPacketBytes -= packetPtr->LengthBytes();

}//DequeuePacketWithPriorityAndPosition//



//--------------------------------------------------------------------------------------------------

class OutputQueueWithPrioritySubqueuesOlderVer: public InterfaceOutputQueue {
public:
    OutputQueueWithPrioritySubqueuesOlderVer(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceId& theInterfaceId,
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
        const PacketPriority& maximumPriority);

    virtual void Insert(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop,
        const EtherTypeField etherType = ETHERTYPE_IS_NOT_SPECIFIED) override;

    void RequeueAtFront(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        const EtherTypeField etherType,
        const SimTime& timestamp,
        const unsigned int retryTxCount);

    virtual PacketPriority MaxPossiblePacketPriority() const override { return (maximumPriority); }

    virtual bool IsEmpty() const override { return (totalPackets == 0); }

    unsigned int NumberPackets() const { return (totalPackets); }

    unsigned long long int NumberPacketBytes() const { return (totalPacketBytes); }

    bool HasPacketWithPriority(const PacketPriority priority) const
    {
        assert(priority <= maximumPriority);
        return (!outputSubqueues.at(priority).fifoQueue.empty());
    }

    const Packet& TopPacket(const PacketPriority priority) const;
    const NetworkAddress& NextHopAddressForTopPacket(const PacketPriority priority) const;

    virtual void DequeuePacket(
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        PacketPriority& priority,
        EtherTypeField& etherType) override;

    virtual void DequeuePacketWithPriority(
        const PacketPriority& priority,
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        EtherTypeField& etherType,
        SimTime& timestamp,
        unsigned int& retryTxCount);

    // Allows extracting frames to a destination in non-FIFO order.  For frame aggregation.

    void EnableNextHopSpecificDequeues() {
        assert(IsEmpty());
        (*this).nextHopSpecificQueuesAreEnabled = true;
    }

    bool NextHopSpecificDequeueIsEnabled() const { return (nextHopSpecificQueuesAreEnabled); }

    bool HasPacketWithPriorityAndNextHop(
        const PacketPriority priority,
        const NetworkAddress& nextHopAddress) const;

    // These methods still work in FIFO mode, but the next hop address must match
    // top of queue (assert).

    const Packet& NextPacketWithPriorityAndNextHop(
        const PacketPriority priority,
        const NetworkAddress& nextHopAddress) const;

    void DequeuePacketWithPriorityAndNextHop(
        const PacketPriority& priority,
        const NetworkAddress& nextHopAddress,
        unique_ptr<Packet>& packetPtr,
        EtherTypeField& etherType,
        SimTime& timestamp,
        unsigned int& retryTxCount);

private:

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    struct OutputQueueRecordType {
        OutputQueueRecordType(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress,
            const EtherTypeField initEtherType,
            const SimTime& initTimestamp,
            const unsigned int initRetryTxCount = 0)
            :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress),
            etherType(initEtherType),
            timestamp(initTimestamp),
            retryTxCount(initRetryTxCount)
        {
        }

        void operator=(OutputQueueRecordType&& right) {
            assert(this != &right);
            packetPtr = move(right.packetPtr);
            nextHopAddress = right.nextHopAddress;
            etherType = right.etherType;
            timestamp = right.timestamp;
            retryTxCount = right.retryTxCount;
        }

        OutputQueueRecordType(OutputQueueRecordType&& right)  { (*this) = move(right); }

        unique_ptr<Packet> packetPtr;
        NetworkAddress nextHopAddress;
        EtherTypeField etherType;
        SimTime timestamp;
        unsigned int retryTxCount;

    };//OutputQueueRecordType//


    struct OutputSubqueueInfoType {

        unsigned long long int currentNumberBytes;

        // Records are owned by fifoQueue using unique_ptr. Raw pointers instead of shared_ptrs
        // are used in "destinationSpecificQueue"'s for speed. Warning!

        std::deque<unique_ptr<OutputQueueRecordType> > fifoQueue;
        map<NetworkAddress, std::deque<OutputQueueRecordType*> > destinationSpecificQueues;

        OutputSubqueueInfoType() : currentNumberBytes(0) { }

        void operator=(OutputSubqueueInfoType&& right) {
            currentNumberBytes = right.currentNumberBytes;
            fifoQueue = move(right.fifoQueue);
            destinationSpecificQueues = move(right.destinationSpecificQueues);
        }

        OutputSubqueueInfoType(OutputSubqueueInfoType&& right) { (*this) = move(right); }
    };

    bool nextHopSpecificQueuesAreEnabled;

    PacketPriority maximumPriority;

    unsigned int totalPackets;
    unsigned long long int totalPacketBytes;

    unsigned int subqueueMaxPackets;
    unsigned int subqueueMaxBytes;

    vector<OutputSubqueueInfoType> outputSubqueues;

    // Disable:

    OutputQueueWithPrioritySubqueuesOlderVer(const OutputQueueWithPrioritySubqueuesOlderVer&);
    void operator=(const OutputQueueWithPrioritySubqueuesOlderVer&);

};//OutputQueueWithPrioritySubqueuesOlderVer//


//----------------------------------------------------------

inline
OutputQueueWithPrioritySubqueuesOlderVer::OutputQueueWithPrioritySubqueuesOlderVer(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceId& theInterfaceId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const PacketPriority& initMaximumPriority)
    :
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    maximumPriority(initMaximumPriority),
    nextHopSpecificQueuesAreEnabled(false),
    totalPackets(0),
    totalPacketBytes(0),
    outputSubqueues(initMaximumPriority + 1),
    subqueueMaxPackets(0),
    subqueueMaxBytes(0)
{
    const NodeId theNodeId = simEngineInterfacePtr->GetNodeId();

    if (theParameterDatabaseReader.ParameterExists("interface-output-queue-max-packets-per-subq", theNodeId, theInterfaceId)){
        subqueueMaxPackets =
            theParameterDatabaseReader.ReadNonNegativeInt("interface-output-queue-max-packets-per-subq", theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("interface-output-queue-max-bytes-per-subq", theNodeId, theInterfaceId)){
        subqueueMaxBytes =
            theParameterDatabaseReader.ReadNonNegativeInt("interface-output-queue-max-bytes-per-subq", theNodeId, theInterfaceId);
    }//if//

}//OutputQueueWithPrioritySubqueuesOlderVer()/


inline
const Packet& OutputQueueWithPrioritySubqueuesOlderVer::TopPacket(const PacketPriority priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].fifoQueue.empty());

    return (*outputSubqueues[priority].fifoQueue.front()->packetPtr);
}


inline
const NetworkAddress& OutputQueueWithPrioritySubqueuesOlderVer::NextHopAddressForTopPacket(
    const PacketPriority priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].fifoQueue.empty());

    return (outputSubqueues[priority].fifoQueue.front()->nextHopAddress);
}


inline
bool OutputQueueWithPrioritySubqueuesOlderVer::HasPacketWithPriorityAndNextHop(
    const PacketPriority priority,
    const NetworkAddress& nextHopAddress) const
{
    assert(nextHopSpecificQueuesAreEnabled);
    assert(priority <= maximumPriority);
    const OutputSubqueueInfoType& outputSubqueue = outputSubqueues.at(priority);

    typedef map<NetworkAddress, std::deque<OutputQueueRecordType*> >::const_iterator IterType;
    const IterType iter = outputSubqueue.destinationSpecificQueues.find(nextHopAddress);

    if (iter == outputSubqueue.destinationSpecificQueues.end()) {
        return false;
    }//if//

    return (!iter->second.empty());

}//HasPacketWithPriorityAndNextHop//


inline
const Packet& OutputQueueWithPrioritySubqueuesOlderVer::NextPacketWithPriorityAndNextHop(
    const PacketPriority priority,
    const NetworkAddress& nextHopAddress) const
{
    assert(priority <= maximumPriority);
    const OutputSubqueueInfoType& outputSubqueue = outputSubqueues.at(priority);

    if (!nextHopSpecificQueuesAreEnabled) {
        assert((outputSubqueue.fifoQueue.front()->nextHopAddress == nextHopAddress) &&
               "Access must be strictly FIFO without destination specific queues.");

        return (*outputSubqueue.fifoQueue.front()->packetPtr);
    }//if//

    typedef map<NetworkAddress, std::deque<OutputQueueRecordType*> >::const_iterator IterType;

    const IterType iter = outputSubqueue.destinationSpecificQueues.find(nextHopAddress);
    assert(iter != outputSubqueue.destinationSpecificQueues.end());

    return (*iter->second.front()->packetPtr);

}//NextPacketWithPriorityAndNextHop//



inline
void OutputQueueWithPrioritySubqueuesOlderVer::Insert(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority priority,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDrop,
    const EtherTypeField etherType)
{
    //cout << "Insert3" << endl;//no

    assert(priority <= maximumPriority);

    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    if ((subqueueMaxPackets != 0) && (queueInfo.fifoQueue.size() >= subqueueMaxPackets)) {
        enqueueResult = ENQUEUE_FAILURE_BY_MAX_PACKETS;
        packetToDrop = move(packetPtr);
    }
    else if ((subqueueMaxBytes != 0) &&
             ((queueInfo.currentNumberBytes + packetPtr->LengthBytes()) > subqueueMaxBytes)) {

        enqueueResult = ENQUEUE_FAILURE_BY_MAX_BYTES;
        packetToDrop = move(packetPtr);
    }
    else {
        enqueueResult = ENQUEUE_SUCCESS;
        packetToDrop = nullptr;
        totalPacketBytes += packetPtr->LengthBytes();
        queueInfo.currentNumberBytes += packetPtr->LengthBytes();
        totalPackets++;
        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
        queueInfo.fifoQueue.push_back(
            unique_ptr<OutputQueueRecordType>(
                new OutputQueueRecordType(packetPtr, nextHopAddress, etherType, currentTime)));

        if (nextHopSpecificQueuesAreEnabled) {

            // Also add to destination specific queue.

            queueInfo.destinationSpecificQueues[nextHopAddress].push_back(
                queueInfo.fifoQueue.back().get());
        }//if//
    }//if//

}//Insert//



inline
void OutputQueueWithPrioritySubqueuesOlderVer::RequeueAtFront(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority priority,
    const EtherTypeField etherType,
    const SimTime& timestamp,
    const unsigned int retryTxCount)
{
    assert(priority <= maximumPriority);

    const unsigned int packetLengthBytes = packetPtr->LengthBytes();
    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    //Can overstuff

    queueInfo.fifoQueue.push_front(
        unique_ptr<OutputQueueRecordType>(
            new OutputQueueRecordType(packetPtr, nextHopAddress, etherType, timestamp, retryTxCount)));

    queueInfo.currentNumberBytes += packetLengthBytes;
    totalPackets++;
    totalPacketBytes += packetLengthBytes;

    if (nextHopSpecificQueuesAreEnabled) {
        // Also add to destination specific.
        queueInfo.destinationSpecificQueues[nextHopAddress].push_front(
            queueInfo.fifoQueue.front().get());
    }//if//

    assert(packetPtr == nullptr);

}//InsertAtFront//



inline
void OutputQueueWithPrioritySubqueuesOlderVer::DequeuePacketWithPriority(
    const PacketPriority& priority,
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    EtherTypeField& etherType,
    SimTime& timestamp,
    unsigned int& retryTxCount)
{
    assert(priority <= maximumPriority);
    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    OutputQueueRecordType& queueRecord = *queueInfo.fifoQueue.front();
    packetPtr = move(queueRecord.packetPtr);
    nextHopAddress = queueRecord.nextHopAddress;
    etherType = queueRecord.etherType;
    timestamp = queueRecord.timestamp;
    retryTxCount = queueRecord.retryTxCount;

    if (nextHopSpecificQueuesAreEnabled) {
        std::deque<OutputQueueRecordType*>& destSpecificQueue =
            queueInfo.destinationSpecificQueues[nextHopAddress];
        assert(destSpecificQueue.front() == queueInfo.fifoQueue.front().get());
        destSpecificQueue.pop_front();
    }//if//

    queueInfo.fifoQueue.pop_front();
    //cout << "DequeuePacketWithPriority_old" << endl;//no

    queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
    totalPackets--;
    totalPacketBytes -= packetPtr->LengthBytes();

    // Cleanup:

    if (nextHopSpecificQueuesAreEnabled) {
        while((!queueInfo.fifoQueue.empty()) && (queueInfo.fifoQueue.front()->packetPtr == nullptr)) {
            queueInfo.fifoQueue.pop_front();
        }//while//
    }//if//

}//DequeuePacketWithPriority//


inline
void OutputQueueWithPrioritySubqueuesOlderVer::DequeuePacketWithPriorityAndNextHop(
    const PacketPriority& priority,
    const NetworkAddress& nextHopAddress,
    unique_ptr<Packet>& packetPtr,
    EtherTypeField& etherType,
    SimTime& timestamp,
    unsigned int& retryTxCount)
{
    if (!nextHopSpecificQueuesAreEnabled) {
        NetworkAddress actualNextHopAddress;

        (*this).DequeuePacketWithPriority(
            priority,
            packetPtr,
            actualNextHopAddress,
            etherType,
            timestamp,
            retryTxCount);

        assert(actualNextHopAddress == nextHopAddress);
        return;

    }//if//

    assert(priority <= maximumPriority);
    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    std::deque<OutputQueueRecordType*>& destinationSpecificQueue =
        queueInfo.destinationSpecificQueues[nextHopAddress];

    assert(!destinationSpecificQueue.empty());

    OutputQueueRecordType& queueRecord = *destinationSpecificQueue.front();

    packetPtr = move(queueRecord.packetPtr);
    etherType = queueRecord.etherType;
    timestamp = queueRecord.timestamp;
    retryTxCount = queueRecord.retryTxCount;

    destinationSpecificQueue.pop_front();

    queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
    totalPackets--;
    totalPacketBytes -= packetPtr->LengthBytes();

    // Cleanup

    while((!queueInfo.fifoQueue.empty()) && (queueInfo.fifoQueue.front()->packetPtr == nullptr)) {
        queueInfo.fifoQueue.pop_front();
        //cout << "DequeuePacketWithPriorityAndNextHop_old" << endl;//no
    }//while//

}//DequeuePacketWithPriorityAndNextHop//


inline
void OutputQueueWithPrioritySubqueuesOlderVer::DequeuePacket(
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    PacketPriority& priority,
    EtherTypeField& etherType)
{
    //cout << "older" << endl;//no
    size_t i = outputSubqueues.size() - 1;
    while(true) {
        OutputSubqueueInfoType& queueInfo = outputSubqueues.at(i);

        if (!queueInfo.fifoQueue.empty()) {

            priority = PacketPriority(i);
            SimTime notUsed1;
            unsigned int notUsed2;

            (*this).DequeuePacketWithPriority(
                priority, packetPtr, nextHopAddress, etherType, notUsed1, notUsed2);

            return;
        }//if//

        if (i == 0) {
            break;
        }//if//

        i--;

    }//while//

    assert(false && "Program Error: All Queues are Empty!"); abort();

}//DequeuePacket//

//--------------------------------------------------------------------------------------------------


}//namespace//

#endif
