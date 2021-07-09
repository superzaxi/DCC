// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scensim_mac.h"
#include "scensim_network.h"

#include <iostream>//20210613

namespace ScenSim {

using std::cout;//20210613
using std::endl;//20210613

const SixByteMacAddress SixByteMacAddress::invalidMacAddress; // Default Constructor (all 0).


AbstractNetworkMac::AbstractNetworkMac(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<AbstractNetwork>& initAbstractNetworkPtr,
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const unsigned int initInterfaceIndex,
    const RandomNumberGeneratorSeed& nodeSeed)
    :
    abstractNetworkPtr(initAbstractNetworkPtr),
    simEngineInterfacePtr(simulationEngineInterfacePtr),
    theNodeId(initNetworkLayerPtr->GetNodeId()),
    networkLayerPtr(initNetworkLayerPtr),
    interfaceIndex(initInterfaceIndex),
    theInterfaceId(initNetworkLayerPtr->GetInterfaceId(initInterfaceIndex)),
    isBusy(false),
    outputQueuePtr(
        new FifoInterfaceOutputQueue(
            theParameterDatabaseReader,
            initNetworkLayerPtr->GetInterfaceId(initInterfaceIndex),
            simulationEngineInterfacePtr)),
    numberPacketsSent(0),
    packetDropByRateEnabled(false),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(nodeSeed, initInterfaceIndex)),
    minimumLatency(ZERO_TIME),
    maximumLatency(ZERO_TIME),
    bandwidthBytesPerSecond(0),
    packetsSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_FramesSent"))),
    packetsDroppedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_FramesDropped"))),
    packetsReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_FramesReceived")))
{
    networkLayerPtr->SetInterfaceOutputQueue(interfaceIndex, outputQueuePtr);

    if ((theParameterDatabaseReader.ParameterExists(
        "abstract-network-mac-packets-to-lose-number-set",
        theNodeId,
        theInterfaceId)) &&
        (theParameterDatabaseReader.ParameterExists(
        "abstract-network-mac-packet-drop-rate",
        theNodeId,
        theInterfaceId))) {
            cerr << "Error: Cannot define abstract-network-mac-packets-to-lose-number-set" << endl;
            cerr << "                 and abstract-network-mac-packet-drop-rate simultaneously." << endl;
        exit(1);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        "abstract-network-mac-packets-to-lose-number-set",
        theNodeId,
        theInterfaceId)) {

        string packetSetString =
            theParameterDatabaseReader.ReadString(
                "abstract-network-mac-packets-to-lose-number-set",
                theNodeId,
                theInterfaceId);

        DeleteTrailingSpaces(packetSetString);
        istringstream packetListStream(packetSetString);

        while(!packetListStream.eof()) {
            unsigned int packetNumberToLose;

            packetListStream >> packetNumberToLose;

            if (packetListStream.fail()) {
                cerr << "Error in: abstract-network-mac-packets-to-lose-number-set parameter." << endl;
                cerr << "          " << packetSetString << endl;
                exit(1);
            }//if//

            (*this).packetsToLoseSet.insert(packetNumberToLose);

        }//if//
    }
    else if (theParameterDatabaseReader.ParameterExists(
                 "abstract-network-mac-packet-drop-rate", theNodeId, theInterfaceId)) {

        packetDropByRateEnabled = true;

        packetDropRate =
            theParameterDatabaseReader.ReadDouble(
                "abstract-network-mac-packet-drop-rate",
                theNodeId,
                theInterfaceId);

        if ((packetDropRate < 0) || (packetDropRate > 1)) {
            cerr << "Error: abstract-network-mac-packet-drop-rate parameter: ";
            cerr << packetDropRate << " should be between 0.0 and 1.0." << endl;
            exit(1);
        }//if//

    }//if//

    minimumLatency =
        theParameterDatabaseReader.ReadTime("abstract-network-min-latency", theNodeId, theInterfaceId);

    if (theParameterDatabaseReader.ParameterExists("abstract-network-max-latency", theNodeId, theInterfaceId)) {
        maximumLatency =
            theParameterDatabaseReader.ReadTime("abstract-network-max-latency", theNodeId, theInterfaceId);
    }
    else {
        maximumLatency = minimumLatency;
    }//if//

    if (minimumLatency > maximumLatency) {
        cerr << "Error: abstract-network-max-latency("
             << ConvertTimeToStringSecs(maximumLatency) << ") must be greater than or equal to abstract-network-min-latency("
             << ConvertTimeToStringSecs(minimumLatency) << ")." << endl;
        exit(1);
    }//if//

    const long long int bandwidthBitsPerSec =
        theParameterDatabaseReader.ReadBigInt("abstract-network-output-bandwidth-bits-per-sec", theNodeId, theInterfaceId);
    bandwidthBytesPerSecond = static_cast<double>(bandwidthBitsPerSec / 8);

}//AbstractNetworkMac()//


void AbstractNetworkMac::SendAPacket()
{
    typedef list<shared_ptr<AbstractNetworkMac> >::iterator IterType;

    assert(!isBusy);
    if (outputQueuePtr->IsEmpty()) {
        return;
    }//if//

    isBusy = true;

    unique_ptr<Packet> packetPtr;
    NetworkAddress nextHopAddress;
    PacketPriority notUsed;
    EtherTypeField etherType;

    outputQueuePtr->DequeuePacket(packetPtr, nextHopAddress, notUsed, etherType);
    //cout << "dequeue" << endl;//no

    assert(
        nextHopAddress.IsInSameSubnetAs(
            networkLayerPtr->GetNetworkAddress(interfaceIndex),
            networkLayerPtr->GetSubnetMask(interfaceIndex)));

    const SimTime packetOutTheDoorTime =
        simEngineInterfacePtr->CurrentTime() +
        (*this).CalcBandwidthLatency(packetPtr->LengthBytes());

    SimTime latency = minimumLatency;
    if (maximumLatency > minimumLatency) {
        const SimTime rangeOfLatency = maximumLatency - minimumLatency;
        latency += static_cast<SimTime>(aRandomNumberGenerator.GenerateRandomDouble() * rangeOfLatency);
    }//if//
    const SimTime arrivalTime = packetOutTheDoorTime + latency;

    (*this).numberPacketsSent++;

    const bool destinationAddressIsABroadcastAddress =
        (nextHopAddress.IsABroadcastOrAMulticastAddress(
            networkLayerPtr->GetSubnetMask(interfaceIndex)));

    if (!destinationAddressIsABroadcastAddress) {

        shared_ptr<AbstractNetworkMac> destinationMacPtr =
            abstractNetworkPtr->GetDestinationMacPtr(nextHopAddress);

        bool dropThePacket = false;

        if (packetDropByRateEnabled) {

            const double randomNumber = aRandomNumberGenerator.GenerateRandomDouble();

            dropThePacket = (randomNumber <= packetDropRate);
        }
        else if (!packetsToLoseSet.empty()) {

            dropThePacket = (packetsToLoseSet.find(numberPacketsSent) != packetsToLoseSet.end());

        }//if//


        if (!dropThePacket) {

            (*this).OutputTraceAndStatsForFrameSend(*packetPtr);

            simEngineInterfacePtr->ScheduleExternalEventAtNode(
                destinationMacPtr->GetNodeId(),
                unique_ptr<SimulationEvent>(
                    new PacketArrivalEvent(
                        destinationMacPtr.get(),
                        packetPtr,
                        networkLayerPtr->GetNetworkAddress(interfaceIndex),
                        etherType)),
                arrivalTime);
        }
        else {

            //dropped
            (*this).OutputTraceAndStatsForFrameDrop(*packetPtr);

            packetPtr = nullptr;
        }//if//
    }
    else {

        list<shared_ptr<AbstractNetworkMac> > destinationMacPtrs;

        abstractNetworkPtr->GetDestinationMacPtrsForBroadcast(
            networkLayerPtr->GetNetworkAddress(interfaceIndex),
            networkLayerPtr->GetSubnetMask(interfaceIndex),
            destinationMacPtrs);

        for(IterType iter = destinationMacPtrs.begin(); iter != destinationMacPtrs.end(); iter++) {

            shared_ptr<AbstractNetworkMac>& destinationMacPtr = (*iter);

            bool dropThePacket = false;

            if (packetDropByRateEnabled) {

                const double randomNumber = aRandomNumberGenerator.GenerateRandomDouble();
                dropThePacket = (randomNumber <= packetDropRate);
            }//if//

            if (!dropThePacket) {
                unique_ptr<Packet> copyOfPacketPtr = unique_ptr<Packet>(new Packet(*packetPtr));

                (*this).OutputTraceAndStatsForFrameSend(*copyOfPacketPtr);

                simEngineInterfacePtr->ScheduleExternalEventAtNode(
                    destinationMacPtr->GetNodeId(),
                    unique_ptr<SimulationEvent>(
                        new PacketArrivalEvent(
                            destinationMacPtr.get(),
                            copyOfPacketPtr,
                            networkLayerPtr->GetNetworkAddress(interfaceIndex),
                            etherType)),
                    arrivalTime);
            }
            else {
                //dropped

                // If packet drop is per destination with broadcast then special
                // trace and stats are needed.

                //(*this).OutputTraceAndStatsForFrameDrop(*packetPtr);
            }//if//

        }//for//

        // Delete original (copies were sent).

        packetPtr = nullptr;
    }//for//

    simEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(new PacketSendFinishedEvent(this)), packetOutTheDoorTime);

}//SendAPacket//


void AbstractNetworkMac::ReceivePacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& lastHopAddress,
    const EtherTypeField& etherType)
{

    (*this).OutputTraceAndStatsForFrameReceive(*packetPtr);

    networkLayerPtr->ReceivePacketFromMac(interfaceIndex, packetPtr, lastHopAddress, etherType);

}//ReceivePacket//



}//namespace//
