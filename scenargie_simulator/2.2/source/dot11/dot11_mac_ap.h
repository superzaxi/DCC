// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11_MAC_AP_H
#define DOT11_MAC_AP_H

#include "scensim_engine.h"
#include "scensim_netsim.h"

#include "dot11_common.h"
#include "dot11_headers.h"

#include <queue>
#include <map>
#include <string>
#include <iomanip>

namespace Dot11 {

using std::shared_ptr;
using std::unique_ptr;
using std::deque;
using std::map;
using std::cout;
using std::cerr;
using std::endl;
using std::hex;
using std::string;
using std::move;

using ScenSim::SimulationEngineInterface;
using ScenSim::SimulationEvent;
using ScenSim::EventRescheduleTicket;
using ScenSim::SimTime;
using ScenSim::MILLI_SECOND;
using ScenSim::ZERO_TIME;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeId;
using ScenSim::InterfaceOrInstanceId;
using ScenSim::RandomNumberGenerator;
using ScenSim::RandomNumberGeneratorSeed;
using ScenSim::PacketPriority;
using ScenSim::EtherTypeField;
using ScenSim::NetworkAddress;
using ScenSim::HashInputsToMakeSeed;


using ScenSim::ConvertTimeToDoubleSecs;

class Dot11Mac;

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

class Dot11ApManagementController {
public:
    Dot11ApManagementController(
        const shared_ptr<Dot11Mac>& initMacLayerPtr,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& initNodeId,
        const InterfaceOrInstanceId& initInterfaceId,
        const RandomNumberGeneratorSeed& interfaceSeed);

    void ProcessManagementFrame(const Packet& managementFrame);

    void ReceiveFramePowerManagementBit(
        const MacAddress& sourceAddress,
        const bool framePowerManagementBitIsOn);

    bool IsAnAssociatedStaAddress(const MacAddress& theMacAddress) const;

    void LookupAssociatedNodeMacAddress(
        const NodeId& targetNodeId,
        bool& wasFound,
        MacAddress& macAddress) const;

    void GetAssociatedStaAddressList(vector<MacAddress>& associatedStaAddressList) const;

    bool StationIsAsleep(const MacAddress& staAddress) const;

    void BufferPacketForSleepingStation(
        const MacAddress& staAddress,
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& destinationNetworkAddress,
        const PacketPriority& priority,
        const EtherTypeField etherType,
        const SimTime& queueInsertTime,
        const unsigned int retryTxCount);

    void BufferManagementFrameForSleepingStation(
        const MacAddress& staAddress,
        unique_ptr<Packet>& framePtr,
        const unsigned int retryTxCount,
        const SimTime& timestamp);

    void GetPowerSaveBufferedPacket(
        const MacAddress& staAddress,
        bool& wasRetrieved,
        unique_ptr<Packet>& packetToSendPtr,
        unsigned int& retryTxCount,
        PacketPriority& priority,
        EtherTypeField& etherType);

private:

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    NodeId theNodeId;
    InterfaceOrInstanceId theInterfaceId;
    string ssid;

    shared_ptr<Dot11Mac> macLayerPtr;

    AssociationId currentAssociationId;

    static const int SEED_HASH = 31843124;
    RandomNumberGenerator aRandomNumberGenerator;

    struct PowerSavePacketBufferElem {
        unique_ptr<Packet> packetPtr;
        NetworkAddress destinationNetworkAddress;
        PacketPriority priority;
        EtherTypeField etherType;
        unsigned int retryTxCount;
        SimTime timestamp;

        PowerSavePacketBufferElem() : packetPtr(nullptr) { }

        PowerSavePacketBufferElem(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initDestinationNetworkAddress,
            const PacketPriority& initPriority,
            const EtherTypeField& initEtherType,
            const unsigned int initRetryTxCount,
            const SimTime& initTimestamp)
        :
            packetPtr(move(initPacketPtr)),
            destinationNetworkAddress(initDestinationNetworkAddress),
            priority(initPriority),
            etherType(initEtherType),
            retryTxCount(initRetryTxCount),
            timestamp(initTimestamp)
        {
        }

        void operator=(PowerSavePacketBufferElem&& right)
        {
            packetPtr = move(right.packetPtr);
            destinationNetworkAddress = right.destinationNetworkAddress;
            priority = right.priority;
            etherType = right.etherType;
            retryTxCount = right.retryTxCount;
            timestamp = right.timestamp;
        }

        PowerSavePacketBufferElem(PowerSavePacketBufferElem&& right) { (*this) = move(right); }

    };//PowerSavePacketBufferElem//


    struct AssociatedStaInformationEntry {
        AssociationId theAssociationId;
        bool isInPowersaveMode;
        deque<PowerSavePacketBufferElem> powerSavePacketBuffer;

        AssociatedStaInformationEntry() : isInPowersaveMode(false) { }
    };

    map<MacAddress, shared_ptr<AssociatedStaInformationEntry> > associatedStaInformation;
    std::bitset<MaxAssociationId+1> associationIdIsBeingUsed;

    void SendReassociationNotification(
        const MacAddress& staAddress,
        const MacAddress& apAddress);

    //------------------------------------------------------

    SimTime beaconInterval;

    void BeaconIntervalTimeout();

    class BeaconIntervalTimeoutEvent : public SimulationEvent {
    public:
        BeaconIntervalTimeoutEvent(Dot11ApManagementController* initApControllerPtr) :
            apControllerPtr(initApControllerPtr) { }
        void ExecuteEvent() { apControllerPtr->BeaconIntervalTimeout(); }
    private:
        Dot11ApManagementController* apControllerPtr;
    };

    shared_ptr <BeaconIntervalTimeoutEvent> beaconIntervalTimeoutEventPtr;
    EventRescheduleTicket beaconIntervalRescheduleTicket;


    //------------------------------------------------------
    SimTime authProcessingDelay;
    void ProcessAuthenticationFrame(const MacAddress& transmitterAddress);

    void SendAuthentication(const MacAddress& transmitterAddress);

    class SendAuthenticationEvent : public SimulationEvent {
    public:
        SendAuthenticationEvent(
            Dot11ApManagementController* initApControllerPtr,
            const MacAddress& initReceiverAddress)
            :
            apControllerPtr(initApControllerPtr),
            receiverAddress(initReceiverAddress)
        {}

        void ExecuteEvent() { apControllerPtr->SendAuthentication(receiverAddress); }
    private:
        Dot11ApManagementController* apControllerPtr;
        MacAddress receiverAddress;
    };

    void SendBeaconFrame();
    void ProcessAssociationRequestFrame(const Packet& aFrame);
    void ProcessReassociationRequestFrame(const Packet& aFrame);
    void AddNewAssociatedStaRecord(
        const MacAddress& staAddress,
        const unsigned int stationBandwidthNumChannels,
        const bool highThroughputModeIsEnabled);

};//Dot11ApManagementController//


inline
Dot11ApManagementController::Dot11ApManagementController(
    const shared_ptr<Dot11Mac>& initMacLayerPtr,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& initNodeId,
    const InterfaceOrInstanceId& initInterfaceId,
    const RandomNumberGeneratorSeed& interfaceSeed)
    :
    macLayerPtr(initMacLayerPtr),
    simEngineInterfacePtr(initSimulationEngineInterfacePtr),
    beaconInterval(100 * MILLI_SECOND),
    theNodeId(initNodeId),
    theInterfaceId(initInterfaceId),
    ssid(""),
    authProcessingDelay(ZERO_TIME),
    currentAssociationId(0),
    aRandomNumberGenerator(HashInputsToMakeSeed(interfaceSeed, SEED_HASH))
{
    if (theParameterDatabaseReader.ParameterExists(
            "dot11-beacon-transmit-interval", theNodeId, theInterfaceId)) {
        beaconInterval = theParameterDatabaseReader.ReadTime(
            "dot11-beacon-transmit-interval", theNodeId, theInterfaceId);

        if (beaconInterval < ZERO_TIME) {
            cerr << "Invalid beacon interval: " << ConvertTimeToDoubleSecs(beaconInterval) << endl;
            exit(1);
        }
    }//if//

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
    const SimTime beaconStartJitter =
        static_cast<SimTime>(beaconInterval * aRandomNumberGenerator.GenerateRandomDouble());

    beaconIntervalTimeoutEventPtr.reset(new BeaconIntervalTimeoutEvent(this));

    simEngineInterfacePtr->ScheduleEvent(
        beaconIntervalTimeoutEventPtr,
        currentTime + beaconStartJitter,
        beaconIntervalRescheduleTicket);

    //read ssid
    if (theParameterDatabaseReader.ParameterExists("dot11-access-point-ssid", theNodeId, theInterfaceId)) {
        ssid = theParameterDatabaseReader.ReadString("dot11-access-point-ssid", theNodeId, theInterfaceId);

        if (ssid.length() > SSID_LENGTH) {
            cerr << "Error: SSID length must be " << SSID_LENGTH << " or under: " << ssid << endl;
            exit(1);
        }//if//
    }//if//

    //read authentication processing delay
    if (theParameterDatabaseReader.ParameterExists("dot11-access-point-auth-processing-delay", theNodeId, theInterfaceId)) {

        authProcessingDelay =
            theParameterDatabaseReader.ReadTime("dot11-access-point-auth-processing-delay", theNodeId, theInterfaceId);

    }//if//

}//Dot11ApManagementController//



inline
void Dot11ApManagementController::ProcessAuthenticationFrame(const MacAddress& transmitterAddress)
{
    shared_ptr<SendAuthenticationEvent> sendAuthenticationEventPtr(new SendAuthenticationEvent(this, transmitterAddress));
    simEngineInterfacePtr->ScheduleEvent(
        sendAuthenticationEventPtr,
        simEngineInterfacePtr->CurrentTime() + authProcessingDelay);

}//ProcessAuthenticationFrame//


inline
bool Dot11ApManagementController::IsAnAssociatedStaAddress(const MacAddress& theMacAddress) const
{
    typedef map<MacAddress, shared_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

    IterType staIterator = associatedStaInformation.find(theMacAddress);

    if (staIterator == associatedStaInformation.end()) {
        return false;
    } else {
        return true;
    }//if//

}//IsAnAssociatedStaAddress//



inline
void Dot11ApManagementController::LookupAssociatedNodeMacAddress(
    const NodeId& targetNodeId,
    bool& wasFound,
    MacAddress& macAddress) const
{
    // This code assumes that the AP will not associate with two interfaces on the same node.
    // To provide this functionality would require ARP/Neighbor Discovery.

    wasFound = false;
    MacAddress searchAddress(targetNodeId, 0);

    typedef map<MacAddress, shared_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

    IterType staIterator = associatedStaInformation.lower_bound(searchAddress);

    if (staIterator == associatedStaInformation.end()) {
        return;
    }//if//

    if (staIterator->first.ExtractNodeId() == targetNodeId) {
        wasFound = true;
        macAddress = staIterator->first;
    }//if//

}//LookupAssociatedNodeMacAddress//



inline
bool Dot11ApManagementController::StationIsAsleep(const MacAddress& staAddress) const
{
    typedef map<MacAddress, shared_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;
    IterType staIterator = associatedStaInformation.find(staAddress);
    if (staIterator == associatedStaInformation.end()) {
        return false;
    }//if//

    const AssociatedStaInformationEntry& staInfo = *(staIterator->second);
    return (staInfo.isInPowersaveMode);

}//StationIsAsleep//


inline
void Dot11ApManagementController::BufferPacketForSleepingStation(
    const MacAddress& staAddress,
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& destinationNetworkAddress,
    const PacketPriority& priority,
    const EtherTypeField etherType,
    const SimTime& queueInsertTime,
    const unsigned int retryTxCount)
{
    AssociatedStaInformationEntry& staInfo = *associatedStaInformation[staAddress];

    assert(staInfo.isInPowersaveMode);

    assert(retryTxCount == 0);  // TBD: Retry complicated by sleep functions.

    staInfo.powerSavePacketBuffer.push_back(move(
        PowerSavePacketBufferElem(
            packetPtr,
            destinationNetworkAddress,
            priority,
            etherType,
            0,
            queueInsertTime)));

}//BufferPacketForSleepingStation//



inline
void Dot11ApManagementController::BufferManagementFrameForSleepingStation(
    const MacAddress& staAddress,
    unique_ptr<Packet>& framePtr,
    const unsigned int retryTxCount,
    const SimTime& timestamp)
{
    AssociatedStaInformationEntry& staInfo = *associatedStaInformation[staAddress];

    assert(staInfo.isInPowersaveMode);
    staInfo.powerSavePacketBuffer.push_back(move(
        PowerSavePacketBufferElem(
            framePtr,
            NetworkAddress::invalidAddress,
            ScenSim::InvalidPacketPriority,
            ScenSim::ETHERTYPE_IS_NOT_SPECIFIED,
            retryTxCount,
            timestamp)));

}//BufferPacketForSleepingStation//


inline
void Dot11ApManagementController::GetAssociatedStaAddressList(vector<MacAddress>& associatedStaAddressList) const
{
    associatedStaAddressList.clear();

    typedef map<MacAddress, shared_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

    for(IterType iter = associatedStaInformation.begin(); iter != associatedStaInformation.end(); ++iter) {
        const MacAddress& associatedStaAddress = iter->first;
        associatedStaAddressList.push_back(associatedStaAddress);
    }//for//

}//GetAssociatedStaAddressList//



inline
void Dot11ApManagementController::GetPowerSaveBufferedPacket(
    const MacAddress& staAddress,
    bool& wasRetrieved,
    unique_ptr<Packet>& packetPtr,
    unsigned int& retryTxCount,
    PacketPriority& priority,
    EtherTypeField& etherType)
{
    AssociatedStaInformationEntry& staInfo = *associatedStaInformation[staAddress];
    PowerSavePacketBufferElem& elem = staInfo.powerSavePacketBuffer.front();

    packetPtr = move(elem.packetPtr);
    priority = elem.priority;
    etherType = elem.etherType;
    retryTxCount = elem.retryTxCount;
    staInfo.powerSavePacketBuffer.pop_front();

}//GetPowerSaveBufferedPacket//


inline
void Dot11ApManagementController::BeaconIntervalTimeout()
{
    beaconIntervalRescheduleTicket.Clear();

    (*this).SendBeaconFrame();

    simEngineInterfacePtr->ScheduleEvent(
        beaconIntervalTimeoutEventPtr,
        simEngineInterfacePtr->CurrentTime() + beaconInterval,
        beaconIntervalRescheduleTicket);

}//BeaconIntervalTimeout//

}//namespace//

#endif
