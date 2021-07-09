// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef ITS_QUEUES_H
#define ITS_QUEUES_H

#include <queue>
#include "scensim_parmio.h"
#include "scensim_engine.h"
#include "scensim_netaddress.h"
#include "scensim_packet.h"
#include "scensim_queues.h"
#include "dot11_common.h"

namespace Dot11 {

using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::move;

using ScenSim::ParameterDatabaseReader;
using ScenSim::InterfaceOutputQueue;
using ScenSim::Packet;
using ScenSim::PacketPriority;
using ScenSim::NetworkAddress;
using ScenSim::InterfaceId;
using ScenSim::SimulationEngineInterface;
using ScenSim::NodeId;
using ScenSim::EtherTypeField;
using ScenSim::ETHERTYPE_IS_NOT_SPECIFIED;
using ScenSim::SimTime;
using ScenSim::EnqueueResultType;
using ScenSim::ENQUEUE_SUCCESS;
using ScenSim::ENQUEUE_FAILURE_BY_MAX_PACKETS;
using ScenSim::ENQUEUE_FAILURE_BY_MAX_BYTES;

//--------------------------------------------------------------------------------------------------

class ItsOutputQueueWithPrioritySubqueues: public InterfaceOutputQueue {
public:
    ItsOutputQueueWithPrioritySubqueues(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceId& theInterfaceId,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const PacketPriority& initMaximumPriority);

    void Insert(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop,
        const EtherTypeField etherType = ETHERTYPE_IS_NOT_SPECIFIED);

    void InsertWithEtherTypeAndDatarateAndTxPower(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        const EtherTypeField& etherType,
        const DatarateBitsPerSec& theDatarateBitsPerSec,
        const double txPowerDbm,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop);

    PacketPriority MaxPossiblePacketPriority() const { return (maximumPriority); }

    bool IsEmpty() const { return (totalPackets == 0); }

    bool HasPacketWithPriority(const PacketPriority priority) const
    {
        assert(priority <= maximumPriority);
        return (!outputSubqueues.at(priority).aQueue.empty());
    }
    bool TopPacketDatarateIsSpecified(const PacketPriority priority) const;

    const Packet& TopPacket(const PacketPriority priority) const;
    const NetworkAddress NextHopForTopPacket(const PacketPriority priority) const;
    DatarateBitsPerSec DatarateBitsPerSecForTopPacket(const PacketPriority priority) const;

    void DequeuePacket(
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        PacketPriority& priority,
        EtherTypeField& etherType);

    void DequeuePacketWithPriority(
        const PacketPriority& priority,
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        EtherTypeField& etherType,
        SimTime& timestamp,
        unsigned int& retryTxCount);

    void DequeuePacketWithEtherTypeAndDatarateAndTxPower(
        const PacketPriority& priority,
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        SimTime& timestamp,
        unsigned int& retryTxCount,
        EtherTypeField& etherType,
        bool& datarateAndTxPowerAreaSpecified,
        DatarateBitsPerSec& theDatarateBitsPerSec,
        double& txPowerDbm);


    // For reinsertion for power management.

    virtual void RequeueAtFront(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        const EtherTypeField etherType,
        const SimTime& timestamp,
        const unsigned int retryTxCount)
    {
        assert(false && "ITS/Wave should not need power save mode"); abort();
    }

    void RequeueAtFront(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        const EtherTypeField etherType,
        const SimTime& timestamp,
        const unsigned int retryTxCount,
        const bool datarateAndTxPowerAreaSpecified,
        const DatarateBitsPerSec theDatarateBitsPerSec,
        const double txPowerDbm);

private:

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    struct OutputQueueRecord {
        unique_ptr<Packet> packetPtr;
        NetworkAddress nextHopAddress;
        EtherTypeField etherType;
        SimTime timestamp;
        bool datarateAndTxPowerAreaSpecified;
        DatarateBitsPerSec theDatarateBitsPerSec;
        double txPowerDbm;
        unsigned int retryTxCount;

        OutputQueueRecord(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress,
            const EtherTypeField& initEtherType,
            const SimTime& initTimestamp,
            const bool initDatarateAndTxPowerAreaSpecified,
            const DatarateBitsPerSec& initDatarateBitsPerSec,
            const double& initTxPowerDbm,
            const unsigned int initRetryTxCount = 0)
            :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress),
            etherType(initEtherType),
            timestamp(initTimestamp),
            datarateAndTxPowerAreaSpecified(initDatarateAndTxPowerAreaSpecified),
            theDatarateBitsPerSec(initDatarateBitsPerSec),
            txPowerDbm(initTxPowerDbm),
            retryTxCount(initRetryTxCount)
        {
        }

        void operator=(OutputQueueRecord&& right)
        {
            packetPtr = move(right.packetPtr);
            nextHopAddress = right.nextHopAddress;
            etherType = right.etherType;
            timestamp = right.timestamp;
            datarateAndTxPowerAreaSpecified = right.datarateAndTxPowerAreaSpecified;
            theDatarateBitsPerSec = right.theDatarateBitsPerSec;
            txPowerDbm = right.txPowerDbm;
            retryTxCount = right.retryTxCount;
        }

        OutputQueueRecord(OutputQueueRecord&& right) { (*this) = move(right); }

    };

    struct OutputSubqueueInfo {
        DatarateBitsPerSec currentNumberBytes;
        std::deque<OutputQueueRecord> aQueue;

        OutputSubqueueInfo() : currentNumberBytes(0) { }

        void operator=(OutputSubqueueInfo&& right) {
            currentNumberBytes = right.currentNumberBytes;
            aQueue = move(aQueue);
        }

        OutputSubqueueInfo(OutputSubqueueInfo&& right) { (*this) = move(right); }
    };

    PacketPriority maximumPriority;

    unsigned int totalPackets;

    unsigned int subqueueMaxPackets;
    unsigned int subqueueMaxBytes;

    vector<OutputSubqueueInfo> outputSubqueues;


    void InsertWithEtherTypeAndDatarateAndTxPower(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority priority,
        const EtherTypeField& etherType,
        const bool datarateAndTxPowerAreaSpecified,
        const DatarateBitsPerSec& theDatarateBitsPerSec,
        const double txPowerDbm,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop);

    // not used
    unsigned int NumberPackets() const
        { assert(false && "This function is not currently used."); abort(); return 0; }
    unsigned long long int NumberPacketBytes() const
                { assert(false && "This function is not currently used."); abort(); return 0; }

    // Disable:

    ItsOutputQueueWithPrioritySubqueues(ItsOutputQueueWithPrioritySubqueues&);
    void operator=(ItsOutputQueueWithPrioritySubqueues&);

};//ItsOutputQueueWithPrioritySubqueues//


//----------------------------------------------------------

inline
ItsOutputQueueWithPrioritySubqueues::ItsOutputQueueWithPrioritySubqueues(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceId& theInterfaceId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const PacketPriority& initMaximumPriority)
    :
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    maximumPriority(initMaximumPriority),
    totalPackets(0),
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

}//ItsOutputQueueWithPrioritySubqueues()/


inline
bool ItsOutputQueueWithPrioritySubqueues::TopPacketDatarateIsSpecified(const PacketPriority priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].aQueue.empty());

    return (outputSubqueues[priority].aQueue.front().datarateAndTxPowerAreaSpecified);
}

inline
const Packet& ItsOutputQueueWithPrioritySubqueues::TopPacket(const PacketPriority priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].aQueue.empty());

    return (*outputSubqueues[priority].aQueue.front().packetPtr);
}


inline
const NetworkAddress ItsOutputQueueWithPrioritySubqueues::NextHopForTopPacket(const PacketPriority priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].aQueue.empty());

    return (outputSubqueues[priority].aQueue.front().nextHopAddress);
}

inline
DatarateBitsPerSec ItsOutputQueueWithPrioritySubqueues::DatarateBitsPerSecForTopPacket(const PacketPriority priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].aQueue.empty());

    return (outputSubqueues[priority].aQueue.front().theDatarateBitsPerSec);
}

inline
void ItsOutputQueueWithPrioritySubqueues::Insert(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority priority,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDrop,
    const EtherTypeField etherType)
{
    const bool datarateAndTxPowerAreaSpecified(false);
    const DatarateBitsPerSec notSpecifiedDatarateBitsPerSec(0);
    const double notSpecifiedTxPowerDbm(0);

    (*this).InsertWithEtherTypeAndDatarateAndTxPower(
        packetPtr,
        nextHopAddress,
        priority,
        ETHERTYPE_IS_NOT_SPECIFIED,
        datarateAndTxPowerAreaSpecified,
        notSpecifiedDatarateBitsPerSec,
        notSpecifiedTxPowerDbm,
        enqueueResult,
        packetToDrop);
}

inline
void ItsOutputQueueWithPrioritySubqueues::RequeueAtFront(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority priority,
    const EtherTypeField etherType,
    const SimTime& timestamp,
    const unsigned int retryTxCount,
    const bool datarateAndTxPowerAreaSpecified,
    const DatarateBitsPerSec theDatarateBitsPerSec,
    const double txPowerDbm)
{
    assert(priority <= maximumPriority);

    OutputSubqueueInfo& queueInfo = outputSubqueues.at(priority);
    queueInfo.aQueue.push_front(
        move(
            OutputQueueRecord(
                packetPtr,
                nextHopAddress,
                etherType,
                timestamp,
                datarateAndTxPowerAreaSpecified,
                theDatarateBitsPerSec,
                txPowerDbm,
                retryTxCount)));

    queueInfo.currentNumberBytes += packetPtr->LengthBytes();
    totalPackets++;

    packetPtr = nullptr;

}//InsertAtFront//

inline
void ItsOutputQueueWithPrioritySubqueues::InsertWithEtherTypeAndDatarateAndTxPower(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority priority,
    const EtherTypeField& etherType,
    const DatarateBitsPerSec& theDatarateBitsPerSec,
    const double txPowerDbm,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDrop)
{
    const bool datarateAndTxPowerAreaSpecified(true);

    (*this).InsertWithEtherTypeAndDatarateAndTxPower(
        packetPtr,
        nextHopAddress,
        priority,
        etherType,
        datarateAndTxPowerAreaSpecified,
        theDatarateBitsPerSec,
        txPowerDbm,
        enqueueResult,
        packetToDrop);
}

inline
void ItsOutputQueueWithPrioritySubqueues::InsertWithEtherTypeAndDatarateAndTxPower(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority priority,
    const EtherTypeField& etherType,
    const bool datarateAndTxPowerAreaSpecified,
    const DatarateBitsPerSec& theDatarateBitsPerSec,
    const double txPowerDbm,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDrop)
{
    assert(priority <= maximumPriority);

    OutputSubqueueInfo& queueInfo = outputSubqueues.at(priority);
    assert((subqueueMaxPackets == 0) || (queueInfo.aQueue.size() <= subqueueMaxPackets));
    assert((subqueueMaxBytes == 0) || (queueInfo.currentNumberBytes <= subqueueMaxBytes));

    if ((subqueueMaxPackets != 0) && (queueInfo.aQueue.size() == subqueueMaxPackets)) {
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
        queueInfo.currentNumberBytes += packetPtr->LengthBytes();
        totalPackets++;
        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
        queueInfo.aQueue.push_back(move(
            OutputQueueRecord(
                packetPtr,
                nextHopAddress,
                etherType,
                currentTime,
                datarateAndTxPowerAreaSpecified,
                theDatarateBitsPerSec,
                txPowerDbm)));
    }//if//
}//Insert//




inline
void ItsOutputQueueWithPrioritySubqueues::DequeuePacket(
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    PacketPriority& priority,
    EtherTypeField& etherType)
{
    etherType = ETHERTYPE_IS_NOT_SPECIFIED;

    size_t i = outputSubqueues.size() - 1;
    while(true) {
        OutputSubqueueInfo& queueInfo = outputSubqueues.at(i);

        if (!queueInfo.aQueue.empty()) {
            OutputQueueRecord& queueRecord = queueInfo.aQueue.front();

            queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
            packetPtr = move(queueRecord.packetPtr);
            nextHopAddress = queueRecord.nextHopAddress;
            priority = PacketPriority(i);
            queueInfo.aQueue.pop_front();
            totalPackets--;

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
void ItsOutputQueueWithPrioritySubqueues::DequeuePacketWithPriority(
    const PacketPriority& priority,
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    EtherTypeField& etherType,
    SimTime& timestamp,
    unsigned int& retryTxCount)
{
    bool datarateAndTxPowerAreaSpecified;
    DatarateBitsPerSec notUsedDatarateBitsPerSec;
    double notUsedTxPowerDbm;

    (*this).DequeuePacketWithEtherTypeAndDatarateAndTxPower(
        priority,
        packetPtr,
        nextHopAddress,
        timestamp,
        retryTxCount,
        etherType,
        datarateAndTxPowerAreaSpecified,
        notUsedDatarateBitsPerSec,
        notUsedTxPowerDbm);

}//DequeuePacketWithPriority//

inline
void ItsOutputQueueWithPrioritySubqueues::DequeuePacketWithEtherTypeAndDatarateAndTxPower(
    const PacketPriority& priority,
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    SimTime& timestamp,
    unsigned int& retryTxCount,
    EtherTypeField& etherType,
    bool& datarateAndTxPowerAreaSpecified,
    DatarateBitsPerSec& theDatarateBitsPerSec,
    double& txPowerDbm)
{
    assert(priority <= maximumPriority);
    OutputSubqueueInfo& queueInfo = outputSubqueues.at(priority);
    assert(!queueInfo.aQueue.empty());

    OutputQueueRecord& queueRecord = queueInfo.aQueue.front();
    packetPtr = move(queueRecord.packetPtr);
    nextHopAddress = queueRecord.nextHopAddress;
    retryTxCount = queueRecord.retryTxCount;
    timestamp = queueRecord.timestamp;
    etherType = queueRecord.etherType;
    datarateAndTxPowerAreaSpecified = queueRecord.datarateAndTxPowerAreaSpecified;
    theDatarateBitsPerSec = queueRecord.theDatarateBitsPerSec;
    txPowerDbm = queueRecord.txPowerDbm;

    queueInfo.aQueue.pop_front();
    queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
    totalPackets--;

}//DequeuePacketWithPriority//


}//namespace//

#endif
