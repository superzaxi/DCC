// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11AH_MACAP_H
#define DOT11AH_MACAP_H

#include "scensim_engine.h"
#include "scensim_netsim.h"

#include "dot11ah_common.h"
#include "dot11ah_headers.h"

#include <queue>
#include <map>
#include <string>
#include <iomanip>

namespace Dot11ah {

using std::shared_ptr;
using std::deque;
using std::map;
using std::unique_ptr;
using std::cout;
using std::hex;
using std::string;
using std::make_pair;

using ScenSim::SimulationEngineInterface;
using ScenSim::SimulationEvent;
using ScenSim::EventRescheduleTicket;
using ScenSim::SimTime;
using ScenSim::MILLI_SECOND;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeId;
using ScenSim::InterfaceOrInstanceId;
using ScenSim::RandomNumberGenerator;
using ScenSim::RandomNumberGeneratorSeed;
using ScenSim::PacketPriority;
using ScenSim::EtherTypeField;
using ScenSim::NetworkAddress;
using ScenSim::DeleteTrailingSpaces;
using ScenSim::IsAConfigFileCommentLine;
using ScenSim::ConvertStringToTime;
using ScenSim::ConvertTimeToDoubleSecs;
using ScenSim::ConvertStringToNonNegativeInt;

class Dot11Mac;


//--------------------------------------------------------------------------------------------------

class AssociationIdTable {
public:
    AssociationIdTable(const string& filename);

    AssociationId GetAssociationIdFor(const MacAddress& macAddress) const;

private:
    map<NodeId, AssociationId> idMap;

};//AssociationIdTable//


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
        const SimTime& timestamp);

    void BufferManagementFrameForSleepingStation(
        const MacAddress& staAddress,
        unique_ptr<Packet>& framePtr,
        const SimTime& timestamp);

    void GetPowerSaveBufferedPacket(
        const MacAddress& staAddress,
        bool& wasRetrieved,
        unique_ptr<Packet>& packetToSendPtr,
        unsigned int& retryTxCount,
        PacketPriority& priority,
        EtherTypeField& etherType);

    // 802.11ah (For RAW timing).

    void ProcessBeaconFrameTransmissionNotification();

private:

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    NodeId theNodeId;
    InterfaceOrInstanceId theInterfaceId;
    string ssid;

    shared_ptr<Dot11Mac> macLayerPtr;

    AssociationId currentAssociationId;

    static const int seedHash = 31843124;
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
        bool usesRestrictedAccessWindowMode;
        bool isInPowersaveMode;
        deque<PowerSavePacketBufferElem> powerSavePacketBuffer;

        AssociatedStaInformationEntry(const bool initUsesRestrictedAccessWindowMode)
            :
            usesRestrictedAccessWindowMode(initUsesRestrictedAccessWindowMode),
            isInPowersaveMode(false)
        {}
    };//AssociatedStaInformationEntry//


    map<MacAddress, shared_ptr<AssociatedStaInformationEntry> > associatedStaInformation;
    map<AssociationId, shared_ptr<AssociatedStaInformationEntry> > associationIdToStaInfoMap;


    // 802.11ah

    SimTime lastBeaconTransmitEndTime;

    unique_ptr<AssociationIdTable> associationIdTablePtr;

    struct RestrictedAccessWindowInfoType {
        unsigned int containingBeaconNumber;
        static const unsigned int InfiniteRepetitions = UINT_MAX;
        unsigned int numberRepetitions;
        SimTime beaconIntervalDuration;
        SimTime relativeStartTime;
        SimTime relativeEndTime;
        SimTime slotDuration;
        AssociationId associationIdRangeStart;
        AssociationId associationIdRangeEnd;
        bool isUplinkRaw;

        vector<AssociationId> slotAssignments;

        RestrictedAccessWindowInfoType(
            const unsigned int initContainingBeaconNumber,
            const unsigned int initNumberRepetitions,
            const SimTime& initBeaconIntervalDuration,
            const SimTime& initRelativeStartTime,
            const SimTime& initRelativeEndTime,
            const SimTime& initSlotDuration,
            const AssociationId& initAssociationIdRangeStart,
            const AssociationId& initAssociationIdRangeEnd,
            const bool initIsUplinkRaw)
        :
            containingBeaconNumber(initContainingBeaconNumber),
            numberRepetitions(initNumberRepetitions),
            beaconIntervalDuration(initBeaconIntervalDuration),
            relativeStartTime(initRelativeStartTime),
            relativeEndTime(initRelativeEndTime),
            slotDuration(initSlotDuration),
            associationIdRangeStart(initAssociationIdRangeStart),
            associationIdRangeEnd(initAssociationIdRangeEnd),
            isUplinkRaw(initIsUplinkRaw)
        {}

    };//RestrictedAccessWindowInfoType//


    vector<RestrictedAccessWindowInfoType> cannedRawWindows;
    unsigned int currentCannedRawWindowIndex;
    unsigned int currentBeaconRepetitionCount;

    //------------------------------------------------------

    SimTime beaconInterval;

    class BeaconIntervalTimeoutEvent : public SimulationEvent {
    public:
        BeaconIntervalTimeoutEvent(Dot11ApManagementController* initApControllerPtr) :
            apControllerPtr(initApControllerPtr) { }
        void ExecuteEvent() { apControllerPtr->BeaconIntervalTimeout(); }
    private:
        Dot11ApManagementController* apControllerPtr;
    };

    shared_ptr<BeaconIntervalTimeoutEvent> beaconIntervalTimeoutEventPtr;

    //------------------------------------------------------

    class RestrictedAccessWindowStartEvent : public SimulationEvent {
    public:
        RestrictedAccessWindowStartEvent(Dot11ApManagementController* initApControllerPtr) :
            apControllerPtr(initApControllerPtr) { }
        void ExecuteEvent() { apControllerPtr->StartUplinkRestrictedAccessWindow(); }
    private:
        Dot11ApManagementController* apControllerPtr;
    };

    shared_ptr<RestrictedAccessWindowStartEvent> restrictedAccessWindowStartEventPtr;

    //------------------------------------------------------

    class RestrictedAccessWindowEndEvent : public SimulationEvent {
    public:
        RestrictedAccessWindowEndEvent(Dot11ApManagementController* initApControllerPtr) :
            apControllerPtr(initApControllerPtr) { }
        void ExecuteEvent() { apControllerPtr->EndUplinkRestrictedAccessWindow(); }
    private:
        Dot11ApManagementController* apControllerPtr;
    };

    shared_ptr<RestrictedAccessWindowEndEvent> restrictedAccessWindowEndEventPtr;


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

    void BeaconIntervalTimeout();
    void StartUplinkRestrictedAccessWindow();
    void EndUplinkRestrictedAccessWindow();

    void SendBeaconFrame();
    void ProcessAssociationRequestFrame(const Packet& aFrame);
    void ProcessReassociationRequestFrame(const Packet& aFrame);

    void AddNewAssociatedStaRecord(
        const MacAddress& staAddress,
        const bool usesRestrictedAccessMode);

    void SendReassociationNotification(
        const MacAddress& staAddress,
        const MacAddress& apAddress);

    void ReadInCannedRestrictedAccessWindows(const string& filename);
    void ReadInCannedRestrictedAccessWindowLine(
        const string& aLine,
        istringstream& aLineStream,
        unsigned int& lastBeaconNumber);

    void ReadInCannedRawAssignmentLine(
        const string& aLine,
        istringstream& aLineStream);

    void CheckLastRawBeaconFieldConsistency() const;

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
    beaconInterval(INFINITE_TIME),
    theNodeId(initNodeId),
    theInterfaceId(initInterfaceId),
    ssid(""),
    authProcessingDelay(ZERO_TIME),
    currentAssociationId(0),
    currentCannedRawWindowIndex(0),
    currentBeaconRepetitionCount(0),
    lastBeaconTransmitEndTime(ZERO_TIME),
    aRandomNumberGenerator(HashInputsToMakeSeed(interfaceSeed, seedHash))
{
    if (theParameterDatabaseReader.ParameterExists("dot11ah-association-id-table-file", theNodeId, theInterfaceId)) {

        const string filename =
            theParameterDatabaseReader.ReadString("dot11ah-association-id-table-file", theNodeId, theInterfaceId);

        associationIdTablePtr.reset(new AssociationIdTable(filename));

    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        "dot11ah-canned-restricted-access-windows-file", theNodeId, theInterfaceId)) {

        const string filename =
            theParameterDatabaseReader.ReadString(
                "dot11ah-canned-restricted-access-windows-file", theNodeId, theInterfaceId);

        (*this).ReadInCannedRestrictedAccessWindows(filename);

    }//if//

    if (!cannedRawWindows.empty()) {
        // Rely on unused parameter warning (instead of error or warning).
        //
        // if (theParameterDatabaseReader.ParameterExists(
        //    "dot11-beacon-transmit-interval", theNodeId, theInterfaceId)) {
        //
        //    cerr << "Warning in Dot11ah: Can't use a canned Beacon/RAW schedule with Beacon "
        //         << "interval parameter (ignored)." << endl;
        //}//if//
    }
    else {

        beaconInterval =
            theParameterDatabaseReader.ReadTime(
                "dot11-beacon-transmit-interval", theNodeId, theInterfaceId);

        if (beaconInterval < ZERO_TIME) {
            cerr << "Invalid beacon interval: " << ConvertTimeToDoubleSecs(beaconInterval) << endl;
            exit(1);
        }//if//
    }//if//

    beaconIntervalTimeoutEventPtr.reset(new BeaconIntervalTimeoutEvent(this));
    restrictedAccessWindowStartEventPtr.reset(new RestrictedAccessWindowStartEvent(this));
    restrictedAccessWindowEndEventPtr.reset(new RestrictedAccessWindowEndEvent(this));

    SimTime firstBeaconTime;

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();


    SimTime beaconStartJitter;
    if (cannedRawWindows.empty()) {
        beaconStartJitter =
            static_cast<SimTime>(beaconInterval * aRandomNumberGenerator.GenerateRandomDouble());
    }
    else {
        beaconStartJitter =
            static_cast<SimTime>(
                cannedRawWindows.front().beaconIntervalDuration * aRandomNumberGenerator.GenerateRandomDouble());
    }//if//

    firstBeaconTime = (currentTime + beaconStartJitter);

    simEngineInterfacePtr->ScheduleEvent(
        beaconIntervalTimeoutEventPtr,
        firstBeaconTime);

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
    const SimTime& timestamp)
{
    AssociatedStaInformationEntry& staInfo = *associatedStaInformation[staAddress];

    assert(staInfo.isInPowersaveMode);

    staInfo.powerSavePacketBuffer.push_back(move(
        PowerSavePacketBufferElem(
            packetPtr,
            destinationNetworkAddress,
            priority,
            etherType,
            0,
            timestamp)));

}//BufferPacketForSleepingStation//



inline
void Dot11ApManagementController::BufferManagementFrameForSleepingStation(
    const MacAddress& staAddress,
    unique_ptr<Packet>& framePtr,
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
            0,
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
    (*this).SendBeaconFrame();

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    if (cannedRawWindows.empty()) {
        simEngineInterfacePtr->ScheduleEvent(
            beaconIntervalTimeoutEventPtr,
            (currentTime + beaconInterval));
    }
    else {
        // Wait for transmission notification.
    }//if//

}//BeaconIntervalTimeout//


inline
void Dot11ApManagementController::AddNewAssociatedStaRecord(
    const MacAddress& staAddress,
    const bool usesRestrictedAccessMode)
{
    assert(associatedStaInformation.find(staAddress) == associatedStaInformation.end());

    shared_ptr<AssociatedStaInformationEntry>
        newStaInfoEntryPtr(new AssociatedStaInformationEntry(usesRestrictedAccessMode));

    if (associationIdTablePtr.get() != nullptr) {
        // Static table.
        newStaInfoEntryPtr->theAssociationId = associationIdTablePtr->GetAssociationIdFor(staAddress);
    }
    else {
        if (usesRestrictedAccessMode) {
            cerr << "An Association ID table file must be configured with RAW mode." << endl;
            exit(1);
        }//if//

        if (currentAssociationId < MaxAssociationId) {
            (*this).currentAssociationId++;
            newStaInfoEntryPtr->theAssociationId = currentAssociationId;
        }
        else {
            typedef map<AssociationId, shared_ptr<AssociatedStaInformationEntry> >::iterator IterType;
            bool wasFound = false;

            // Find a free association ID.

            AssociationId assId = 1;

            for (IterType iter = associationIdToStaInfoMap.begin();
                 iter != associationIdToStaInfoMap.end(); ++iter) {

                 if (iter->second->theAssociationId != assId) {
                     wasFound = true;
                     newStaInfoEntryPtr->theAssociationId = assId;
                     break;
                 }//if//

                 assId++;
            }//for//

            assert((wasFound) && "Too Many STAs trying to associate with AP");
            assert(associationIdToStaInfoMap.find(assId) == associationIdToStaInfoMap.end());
        }//if//
    }//if//

    (*this).associatedStaInformation.insert(make_pair(staAddress, newStaInfoEntryPtr));
    (*this).associationIdToStaInfoMap.insert(
        make_pair(newStaInfoEntryPtr->theAssociationId, newStaInfoEntryPtr));

}//AddNewAssociatedStaRecord//



inline
void Dot11ApManagementController::CheckLastRawBeaconFieldConsistency() const
{
    if (cannedRawWindows.size() == 1) {
        return;
    }//if//

    const RestrictedAccessWindowInfoType& nextToLast = cannedRawWindows[cannedRawWindows.size() - 2];
    const RestrictedAccessWindowInfoType& last = cannedRawWindows.back();

    if (last.containingBeaconNumber != nextToLast.containingBeaconNumber) {
        return;
    }//if//


    if ((last.beaconIntervalDuration != nextToLast.beaconIntervalDuration) ||
        (last.numberRepetitions != nextToLast.numberRepetitions)) {

        cerr << "Error: Canned Restricted Access Window (RAW) file format incorrect:" << endl;
        cerr << "Beacon Fields don't match for beacon #: " << last.containingBeaconNumber << "." << endl;

        exit(1);
    }//if//


    if ((last.containingBeaconNumber == nextToLast.containingBeaconNumber) &&
        (nextToLast.relativeEndTime > last.relativeStartTime)) {

        cerr << "Error in Canned Restricted Access Window (RAW) file: "
             << "Schedule is not in time order in beacon #:"
             << last.containingBeaconNumber << "." << endl;
        exit(1);
    }//if//

}//CheckLastRawBeaconFieldConsistency//



inline
void Dot11ApManagementController::ReadInCannedRestrictedAccessWindowLine(
    const string& aLine,
    istringstream& aLineStream,
    unsigned int& lastBeaconNumber)
{
    unsigned int containingBeaconNumber;
    unsigned int numberRepetitions;
    string numberRepetitionsString;
    string beaconIntervalDurationString;
    string relativeStartTimeString;
    string relativeEndTimeString;
    string slotDurationString;
    unsigned int associationIdRangeStart;
    unsigned int associationIdRangeEnd;
    string upOrDownIndicatorString;

    aLineStream >> containingBeaconNumber;
    aLineStream >> numberRepetitionsString;
    aLineStream >> beaconIntervalDurationString;
    aLineStream >> relativeStartTimeString;
    aLineStream >> relativeEndTimeString;
    aLineStream >> slotDurationString;
    aLineStream >> associationIdRangeStart;
    aLineStream >> associationIdRangeEnd;
    //Future aLineStream >> upOrDownIndicatorString;
    upOrDownIndicatorString = "up"; // Not Used

    if (aLineStream.fail()) {
        cerr << "Error reading Canned Restricted Access Window (RAW) file:" << endl;
        cerr << "Not enough fields or incorrect numeric field in line:" << endl;
        cerr << '"' << aLine << '"' << endl;
        exit(1);
    }//if//

    if (!aLineStream.eof()) {
        cerr << "Error: Canned Restricted Access Window (RAW) file format incorrect"
             << " (too many terms):" << endl;
        cerr << '"' << aLine << '"' << endl;
        exit(1);
    }//if//

    if (containingBeaconNumber < lastBeaconNumber) {
        cerr << "Error Canned Restricted Access Window (RAW) file:"
             << " Beacon numbers not increasing." << endl;
        cerr << '"' << aLine << '"' << endl;
        exit(1);
    }//if//

    lastBeaconNumber = containingBeaconNumber;

    SimTime beaconIntervalDuration = INFINITE_TIME;
    SimTime relativeStartTime = INFINITE_TIME;
    SimTime relativeEndTime = INFINITE_TIME;
    SimTime slotDuration = INFINITE_TIME;


    bool success = false;

    do {  // "break block"

        if (numberRepetitionsString == "*") {
            numberRepetitions = RestrictedAccessWindowInfoType::InfiniteRepetitions;
        }
        else {
            ConvertStringToNonNegativeInt(numberRepetitionsString, numberRepetitions, success);
            if (!success) { break; }
        }//if//

        ConvertStringToTime(beaconIntervalDurationString, beaconIntervalDuration, success);
        if (!success) { break; }

        ConvertStringToTime(relativeStartTimeString, relativeStartTime, success);
        if (!success) { break; }

        ConvertStringToTime(relativeEndTimeString, relativeEndTime, success);
        if (!success) { break; }

        ConvertStringToTime(slotDurationString, slotDuration, success);
        if (!success) { break; }

    } while (false && "Not a loop");


    if (!success) {
        cerr << "Error in Canned Restricted Access Window (RAW) file: "
             << "Bad format in a time field:" << endl;
        cerr << '"' << aLine << '"' << endl;
        exit(1);
    }//if//

    if (relativeStartTime >= relativeEndTime) {
        cerr << "Error in Canned Restricted Access Window (RAW) file: "
             << "Window start time greater than end time:" << endl;
        cerr << '"' << aLine << '"' << endl;
        exit(1);
    }//if//

    const SimTime rawDuration = (relativeEndTime - relativeStartTime);

    if (rawDuration < slotDuration) {
        cerr << "Error in Canned Restricted Access Window (RAW) file: "
             << "Window is shorter than a slot duration:" << endl;
        cerr << '"' << aLine << '"' << endl;
        exit(1);
    }//if//

    if (rawDuration > (slotDuration * RawSlotDefinitionFieldType::maxNumberOfSlots)) {
        cerr << "Error in Canned Restricted Access Window (RAW) file: "
             << "Window is too long:" << endl;
        cerr << '"' << aLine << '"' << endl;
        exit(1);
    }//if//

    if (relativeEndTime > beaconIntervalDuration) {
        cerr << "Error in Canned Restricted Access Window (RAW) file: "
             << "Window longer than beacon interval:" << endl;
        cerr << '"' << aLine << '"' << endl;
        exit(1);
    }//if//

    if ((associationIdRangeStart > associationIdRangeEnd) ||
        (associationIdRangeEnd > MaxAssociationId)) {

        cerr << "Error in Canned Restricted Access Window (RAW) file: "
             << "Bad Association ID range:" << endl;
        cerr << '"' << aLine << '"' << endl;
        exit(1);
    }//if//

    ConvertStringToLowerCase(upOrDownIndicatorString);

    if ((upOrDownIndicatorString != "up") && (upOrDownIndicatorString != "down")) {
       cerr << "Error in Canned Restricted Access Window (RAW) file: "
            << "Uplink/Downlink indicator should be \"Up\" or \"Down\":" << endl;
       cerr << '"' << aLine << '"' << endl;
       exit(1);
    }//if//

    (*this).cannedRawWindows.push_back(
        RestrictedAccessWindowInfoType(
            containingBeaconNumber,
            numberRepetitions,
            beaconIntervalDuration,
            relativeStartTime,
            relativeEndTime,
            slotDuration,
            static_cast<AssociationId>(associationIdRangeStart),
            static_cast<AssociationId>(associationIdRangeEnd),
            (upOrDownIndicatorString == "up")));

    CheckLastRawBeaconFieldConsistency();

}//ReadInCannedRestrictedAccessWindowLine//


inline
void Dot11ApManagementController::ReadInCannedRawAssignmentLine(
    const string& aLine,
    istringstream& aLineStream)
{
    unsigned int slotIndex;
    unsigned int theAssociationId;

    aLineStream >> slotIndex;
    aLineStream >> theAssociationId;

    if (aLineStream.fail()) {
        cerr << "Error reading Canned Restricted Access Window (RAW) file:" << endl;
        cerr << '"' << aLine << '"' << endl;
        exit(1);
    }//if//

    if (!aLineStream.eof()) {
        cerr << "Error: Canned Restricted Access Window (RAW) file format incorrect"
             << " (too many terms):" << endl;
        cerr << '"' << aLine << '"' << endl;
        exit(1);
    }//if//

    vector<AssociationId>& slotAssignments = (*this).cannedRawWindows.back().slotAssignments;

    if (slotIndex != slotAssignments.size()) {
        cerr << "Error reading Canned Restricted Access Window (RAW) file: Slot Assignment out of order:" << endl;
        cerr << '"' << aLine << '"' << endl;
        exit(1);
    }//if//

    if ((theAssociationId > MaxAssociationId)) {

        cerr << "Error in Canned Restricted Access Window (RAW) file: "
             << "Bad Association ID:" << endl;
        cerr << '"' << aLine << '"' << endl;
        exit(1);
    }//if//

    slotAssignments.push_back(static_cast<AssociationId>(theAssociationId));

}//ReadInCannedRawAssignmentLine/



inline
void Dot11ApManagementController::ReadInCannedRestrictedAccessWindows(const string& filename)
{
    (*this).currentCannedRawWindowIndex = 0;

    std::ifstream inFile(filename.c_str());

    if (!inFile.good()) {
        cerr << "Could not open Canned Restricted Access Window (RAW) file: " << filename << endl;
        exit(1);
    }//if//

    unsigned int lastBeaconNumber = 0;

    while(!inFile.eof()) {
        string aLine;
        getline(inFile, aLine);

        if (inFile.eof()) {
            break;
        }
        else if (inFile.fail()) {
            cerr << "Error reading Canned Restricted Access Window (RAW) file: " << filename << endl;
            exit(1);
        }//if//

        DeleteTrailingSpaces(aLine);
        ConvertStringToLowerCase(aLine);

        if (!IsAConfigFileCommentLine(aLine)) {
            istringstream aLineStream(aLine);

            string lineDesignator;
            aLineStream >> lineDesignator;

            if (lineDesignator == "raw") {

                (*this).ReadInCannedRestrictedAccessWindowLine(aLine, aLineStream, lastBeaconNumber);
            }
            else if (lineDesignator == "ass") {

                (*this).ReadInCannedRawAssignmentLine(aLine, aLineStream);
            }
            else {
                cerr << "Error reading Canned Restricted Access Window (RAW) file: Unknown format:" << endl;
                cerr << '"' << aLine << '"' << endl;
                exit(1);
            }//if//
        }//if//
    }//while//

}//ReadInCannedRestrictedAccessWindows//




inline
AssociationIdTable::AssociationIdTable(const string& filename)
{
    std::ifstream inFile(filename.c_str());

    if (!inFile.good()) {
        cerr << "Could not open Association ID Table file: " << filename << endl;
        exit(1);
    }//if//

    while(!inFile.eof()) {
        string aLine;
        getline(inFile, aLine);

        if (inFile.eof()) {
            break;
        }
        else if (inFile.fail()) {
            cerr << "Error reading Association ID Table file: " << filename << endl;
            exit(1);
        }//if//

        DeleteTrailingSpaces(aLine);

        if (!IsAConfigFileCommentLine(aLine)) {
            istringstream lineStream(aLine);

            int readNodeId;
            int readAssociationId;

            lineStream >> readNodeId;
            lineStream >> readAssociationId;

            if ((lineStream.fail()) || (readNodeId <= 0) || (readAssociationId < 0)) {

                cerr << "Error: Assocation ID Table file line format incorrect:" << endl;
                cerr << '"' << aLine << '"' << endl;
                exit(1);
            }//if//

            if (!lineStream.eof()) {
                cerr << "Error: Assocation ID Table file line format incorrect (too many terms):" << endl;
                cerr << '"' << aLine << '"' << endl;
                exit(1);
            }//if//

            const NodeId theNodeId = static_cast<NodeId>(readNodeId);
            const AssociationId assocationId = static_cast<AssociationId>(readAssociationId);

            if (idMap.find(theNodeId) != idMap.end()) {
                cerr << "Error: Assocation ID Table file: duplicate definition:" << endl;
                cerr << '"' << aLine << '"' << endl;
                exit(1);
            }//if//

            idMap[theNodeId] = assocationId;

        }//if//
    }//while//

}//AssociationIdTable//


inline
AssociationId AssociationIdTable::GetAssociationIdFor(const MacAddress& macAddress) const
{
    typedef map<NodeId, AssociationId>::const_iterator IterType;

    IterType iter = idMap.find(macAddress.ExtractNodeId());

    if (iter == idMap.end()) {
        cerr << "Error in Association ID map file: Node ID mapping not found: NodeID = "
             << macAddress.ExtractNodeId() << endl;
        exit(1);
    }//if//

    return (iter->second);

}//GetAssociationIdFor//



}//namespace//

#endif

