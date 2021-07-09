// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11AH_RATECONTROL_H
#define DOT11AH_RATECONTROL_H

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "dot11ah_common.h"
#include "dot11ah_phy.h"

#include <map>
#include <string>
#include <vector>


namespace Dot11ah {

using std::map;
using std::set;
using std::istringstream;
using std::cerr;
using std::endl;
using std::shared_ptr;
using std::enable_shared_from_this;

using ScenSim::EventRescheduleTicket;
using ScenSim::ConvertAStringSequenceOfANumericTypeIntoAVector;
using ScenSim::ConvertStringToLowerCase;
using ScenSim::ConvertAStringSequenceOfRealValuedPairsIntoAMap;
using ScenSim::ConvertAStringIntoAVectorOfStrings;

using ScenSim::CalcNodeId;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeId;
using ScenSim::InterfaceId;
using ScenSim::SimulationEngineInterface;
using ScenSim::SimTime;
using ScenSim::SimulationEvent;

enum AckDatarateOptionType {
    ackRateSameAsData,
    ackRateLowest,
};


inline
AckDatarateOptionType ReadAckDatarateOption(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId)
{
    if (theParameterDatabaseReader.ParameterExists(
        parameterNamePrefix + "ack-datarate-selection-type", theNodeId, theInterfaceId)) {

        const string ackDatarateSelectionString =
            MakeLowerCaseString(
                theParameterDatabaseReader.ReadString(
                    parameterNamePrefix + "ack-datarate-selection-type", theNodeId, theInterfaceId));

        if (ackDatarateSelectionString == "sameasdata") {
            return (ackRateSameAsData);
        }
        else if (ackDatarateSelectionString == "lowest") {
            return (ackRateLowest);
        }
        else {
            cerr << "Error: Unkown dot11-ack-datarate-selection-type = " << ackDatarateSelectionString << endl;
            exit(1);
        }//if//
    }//if//

    return (ackRateLowest);

}//ReadAckDatarateOption//



class AdaptiveRateController {
public:
    virtual ~AdaptiveRateController() { }

    virtual void SetHighThroughputModeForLink(const MacAddress& macAddress) = 0;

    virtual unsigned int GetMinChannelBandwidthMhz() const = 0;
    virtual unsigned int GetMaxChannelBandwidthMhz() const = 0;

    virtual ModulationAndCodingSchemesType GetLowestModulationAndCoding() const = 0;

    virtual void GetDataRateInfoForDataFrameToStation(
        const MacAddress& macAddress,
        TransmissionParameters& txParameters) const = 0;

    virtual void GetDataRateInfoForAckFrame(
        const MacAddress& macAddress,
        const TransmissionParameters& receivedFrameTxParameters,
        TransmissionParameters& ackTxParameters) const = 0;

    virtual void GetDataRateInfoForManagementFrameToStation(
        const MacAddress& macAddress,
        TransmissionParameters& txParameters) const = 0;

    virtual void GetDataRateInfoForBeaconFrame(
        TransmissionParameters& txParameters) const = 0;

    virtual void NotifyAckReceived(const MacAddress& macAddress) { }
    virtual void NotifyAckFailed(const MacAddress& macAddress) { }

    virtual void ReceiveIncomingFrameSinrValue(
        const MacAddress& sourceMacAddress,
        const double& measuredSinrValue) { }

};//AdaptiveRateController//



//--------------------------------------------------------------------------------------------------


class StaticRateController : public AdaptiveRateController {
public:
    StaticRateController(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int baseChannelBandwidthMhz);

    virtual ~StaticRateController() { }

    virtual void SetHighThroughputModeForLink(const MacAddress& macAddress) override
        { assert(false && "TBD"); }

    virtual ModulationAndCodingSchemesType GetLowestModulationAndCoding() const override {
        return (lowestModulationAndCoding);
    }

    virtual unsigned int GetMinChannelBandwidthMhz() const override
        { return (minChannelBandwidthMhz); }

    virtual unsigned int GetMaxChannelBandwidthMhz() const override
        { return (maxChannelBandwidthMhz); }

    virtual void GetDataRateInfoForDataFrameToStation(
        const MacAddress& macAddress,
        TransmissionParameters& txParameters) const override;

    virtual void GetDataRateInfoForAckFrame(
        const MacAddress& macAddress,
        const TransmissionParameters& receivedFrameTxParameters,
        TransmissionParameters& ackTxParameters) const override;

    virtual void GetDataRateInfoForManagementFrameToStation(
        const MacAddress& macAddress,
        TransmissionParameters& txParameters) const override;

    virtual void GetDataRateInfoForBeaconFrame(
        TransmissionParameters& txParameters) const override;

private:
    unsigned int minChannelBandwidthMhz;
    unsigned int maxChannelBandwidthMhz;

    bool highThroughputModeIsOn;

    AckDatarateOptionType ackDatarateOption;

    ModulationAndCodingSchemesType lowestModulationAndCoding;

    ModulationAndCodingSchemesType defaultModulationAndCoding;
    ModulationAndCodingSchemesType modAndCodingForManagementFrames;
    ModulationAndCodingSchemesType modAndCodingForBroadcast;

    bool perLinkDatarate;

    map<NodeId, ModulationAndCodingSchemesType> datarateTable;

    void ParseDatarateTable(
        const string& datarateTableString, bool& success);

};//StaticRateController//




inline
StaticRateController::StaticRateController(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId,
    const unsigned int baseChannelBandwidthMhz)
    :
    perLinkDatarate(false),
    minChannelBandwidthMhz(baseChannelBandwidthMhz),
    maxChannelBandwidthMhz(baseChannelBandwidthMhz)
{
    using std::min;

    ackDatarateOption = ReadAckDatarateOption(theParameterDatabaseReader, theNodeId, theInterfaceId);

    if (theParameterDatabaseReader.ParameterExists("dot11-max-channel-bandwidth-mhz", theNodeId, theInterfaceId)) {
        maxChannelBandwidthMhz =
           theParameterDatabaseReader.ReadNonNegativeInt("dot11-max-channel-bandwidth-mhz", theNodeId, theInterfaceId);
    }//if//


    highThroughputModeIsOn = false;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "enable-high-throughput-mode"),theNodeId, theInterfaceId)) {

        highThroughputModeIsOn =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "enable-high-throughput-mode"), theNodeId, theInterfaceId);
    }//if//


    //default
    defaultModulationAndCoding =
        ConvertNameToModulationAndCodingScheme(
            theParameterDatabaseReader.ReadString("dot11-modulation-and-coding", theNodeId, theInterfaceId),
            "dot11-modulation-and-coding");

    //management
    if (theParameterDatabaseReader.ParameterExists(
        "dot11-modulation-and-coding-for-management-frames", theNodeId, theInterfaceId)) {

        modAndCodingForManagementFrames =
            ConvertNameToModulationAndCodingScheme(
                theParameterDatabaseReader.ReadString(
                    "dot11-modulation-and-coding-for-management-frames", theNodeId, theInterfaceId),
                    "dot11-modulation-and-coding-for-management-frames");
    }
    else {
        //default datarate
        modAndCodingForManagementFrames = defaultModulationAndCoding;
    }//if//

    //broadcast
    if (theParameterDatabaseReader.ParameterExists(
        "dot11-modulation-and-coding-for-broadcast", theNodeId, theInterfaceId)) {

        modAndCodingForBroadcast =
            ConvertNameToModulationAndCodingScheme(
                theParameterDatabaseReader.ReadString(
                    "dot11-modulation-and-coding-for-broadcast", theNodeId, theInterfaceId),
                "dot11-modulation-and-coding-for-broadcast");
    }
    else {
        //default datarate
        modAndCodingForBroadcast = defaultModulationAndCoding;

    }//if//

    //lowest

    lowestModulationAndCoding =
        min(defaultModulationAndCoding,
            min(modAndCodingForManagementFrames, modAndCodingForBroadcast));

    //per link datarate
    if (theParameterDatabaseReader.ParameterExists("dot11-modulation-and-coding-table", theNodeId, theInterfaceId)) {

        const string datarateTableString =
            theParameterDatabaseReader.ReadString("dot11-modulation-and-coding-table", theNodeId, theInterfaceId);

        bool success = false;
        ParseDatarateTable(datarateTableString, success);

        if (!success) {
            cerr << "Invalid value: \"dot11-modulation-and-coding-table\": " << datarateTableString << endl;
            exit(1);
         }//if//

        perLinkDatarate = true;

    }//if//

}//StaticRateController//


inline
void StaticRateController::ParseDatarateTable(const string& datarateTableString, bool& success)
{
    using std::min;

    istringstream datarateTableStringStream(datarateTableString);

    while (!datarateTableStringStream.eof()) {

        string aString;
        datarateTableStringStream >> aString;

        const size_t endOfTargetNodesTerminatorPos = aString.find_first_of(':');

        if (endOfTargetNodesTerminatorPos == string::npos) {
            success = false;
            return;
        }//if//

        const string targetNodesString =
            aString.substr(0, endOfTargetNodesTerminatorPos);

        const string targetModAndCodingString =
            aString.substr(endOfTargetNodesTerminatorPos + 1, aString.length() - endOfTargetNodesTerminatorPos);


        ModulationAndCodingSchemesType targetModAndCoding =
            ConvertNameToModulationAndCodingScheme(
                targetModAndCodingString, "dot11-modulation-and-coding-table");

        size_t currentPosition = 0;
        set<NodeId> targetNodeIds;
        while (currentPosition < targetNodesString.length()) {

            size_t endOfSubfieldTerminatorPos = targetNodesString.find_first_of(",-", currentPosition);

            if (endOfSubfieldTerminatorPos == string::npos) {
                endOfSubfieldTerminatorPos = targetNodesString.length();
            }

            if ((endOfSubfieldTerminatorPos == targetNodesString.length())
                || (targetNodesString.at(endOfSubfieldTerminatorPos) == ',')) {

                const NodeId extractedNumber =
                    atoi(targetNodesString.substr(currentPosition, (endOfSubfieldTerminatorPos - currentPosition)).c_str());

                targetNodeIds.insert(extractedNumber);
            }
            else if (targetNodesString.at(endOfSubfieldTerminatorPos) == '-') {

                //This is a range.

                const size_t dashPosition = endOfSubfieldTerminatorPos;

                endOfSubfieldTerminatorPos = targetNodesString.find(',', dashPosition);

                if (endOfSubfieldTerminatorPos == string::npos) {
                    endOfSubfieldTerminatorPos = targetNodesString.length();
                }

                const NodeId lowerBoundNumber =
                    atoi(targetNodesString.substr(currentPosition, (dashPosition - currentPosition)).c_str());

                const NodeId upperBoundNumber =
                    atoi(targetNodesString.substr((dashPosition + 1), (endOfSubfieldTerminatorPos - dashPosition - 1)).c_str());

                if (lowerBoundNumber > upperBoundNumber) {
                    success = false;
                    return;
                }//if//

                for(NodeId theNodeId = lowerBoundNumber; (theNodeId <= upperBoundNumber); theNodeId++) {
                    targetNodeIds.insert(theNodeId);
                }//for//

            }//if//

            currentPosition = endOfSubfieldTerminatorPos + 1;

        }//while//


        //set datarate per node
        for(set<NodeId>::const_iterator iter = targetNodeIds.begin();
            iter != targetNodeIds.end(); ++iter) {

            const NodeId targetNodeId = *iter;

            (*this).datarateTable[targetNodeId] = targetModAndCoding;

            (*this).lowestModulationAndCoding = min(lowestModulationAndCoding, targetModAndCoding);

        }//for//

    }//while//

    success = true;

}//ParseDatarateTable//


inline
void StaticRateController::GetDataRateInfoForDataFrameToStation(
    const MacAddress& macAddress,
    TransmissionParameters& txParameters) const
{
    if (macAddress.IsABroadcastOrAMulticastAddress()) {

        //broadcast
        txParameters.modulationAndCodingScheme = modAndCodingForBroadcast;
    }
    else {
        //unicast
        if (perLinkDatarate) {

            const NodeId targetNodeId = CalcNodeId(macAddress.ConvertToGenericMacAddress());

            map<NodeId,ModulationAndCodingSchemesType>::const_iterator iter =
                datarateTable.find(targetNodeId);

            if (iter == datarateTable.end()) {
                txParameters.modulationAndCodingScheme = defaultModulationAndCoding;
            }
            else {
                txParameters.modulationAndCodingScheme = iter->second;
            }//if//

        }
        else {

            txParameters.modulationAndCodingScheme = defaultModulationAndCoding;

        }//if//
    }//if//

    txParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = highThroughputModeIsOn;

}//GetDataRateInformationForStation//



inline
void StaticRateController::GetDataRateInfoForAckFrame(
    const MacAddress& macAddress,
    const TransmissionParameters& receivedFrameTxParameters,
    TransmissionParameters& ackTxParameters) const
{
    ackTxParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    if (ackDatarateOption == ackRateSameAsData) {
        ackTxParameters.modulationAndCodingScheme =
            receivedFrameTxParameters.modulationAndCodingScheme;
    }//if//

    ackTxParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    ackTxParameters.numberSpatialStreams = 1;
    ackTxParameters.isHighThroughputFrame = highThroughputModeIsOn;

}//GetDataRateInfoForAckFrame//


inline
void StaticRateController::GetDataRateInfoForManagementFrameToStation(
    const MacAddress& macAddress,
    TransmissionParameters& txParameters) const
{
    txParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    txParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = highThroughputModeIsOn;

}//GetDataRateInfoForManagementFrameToStation//


inline
void StaticRateController::GetDataRateInfoForBeaconFrame(
    TransmissionParameters& txParameters) const
{
    txParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    txParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = false;
}




//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


// Abbr: Automatic Rate Fallback = ARF

class AdaptiveRateControllerWithArf : public AdaptiveRateController {
public:
    AdaptiveRateControllerWithArf(
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int baseChannelBandwidthMhz);

    virtual void SetHighThroughputModeForLink(const MacAddress& macAddress) override
        { assert(false && "TBD"); }

    virtual ModulationAndCodingSchemesType GetLowestModulationAndCoding() const override {
        return (lowestModulationAndCoding);
    }

    virtual unsigned int GetMinChannelBandwidthMhz() const override
        { return (minChannelBandwidthMhz); }

    virtual unsigned int GetMaxChannelBandwidthMhz() const override
        { return (maxChannelBandwidthMhz); }

    virtual void GetDataRateInfoForDataFrameToStation(
        const MacAddress& macAddress,
        TransmissionParameters& txParameters) const override;

    virtual void GetDataRateInfoForAckFrame(
        const MacAddress& macAddress,
        const TransmissionParameters& receivedFrameTxParameters,
        TransmissionParameters& ackTxParameters) const override;

    virtual void GetDataRateInfoForManagementFrameToStation(
        const MacAddress& macAddress,
        TransmissionParameters& txParameters) const override;

    virtual void GetDataRateInfoForBeaconFrame(
        TransmissionParameters& txParameters) const override;

    virtual void NotifyAckReceived(const MacAddress& macAddress) override;
    virtual void NotifyAckFailed(const MacAddress& macAddress) override;

    virtual ~AdaptiveRateControllerWithArf() { }

private:
    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    unsigned int minChannelBandwidthMhz;
    unsigned int maxChannelBandwidthMhz;

    bool highThroughputModeIsOn;

    ModulationAndCodingSchemesType lowestModulationAndCoding;
    ModulationAndCodingSchemesType modAndCodingForManagementFrames;
    ModulationAndCodingSchemesType modAndCodingForBroadcast;
    vector<ModulationAndCodingSchemesType> modulationAndCodingSchemes;

    AckDatarateOptionType ackDatarateOption;

    SimTime rateUpgradeTimerDuration;
    int ackInSuccessCountThreshold;
    int ackInFailureCountThreshold;
    int ackInFailureCountThresholdOfNewRateState;

    class AckCounterEntry : public enable_shared_from_this<AckCounterEntry> {
    public:
        AckCounterEntry(
            const AdaptiveRateControllerWithArf* initArfPtr,
            const unsigned int initModAndCodingIndex)
            :
            arfPtr(initArfPtr),
            consecutiveUnicastPacketsSentStat(0),
            consecutiveDroppedPacketsStat(0),
            currentModulationAndCodingIndex(initModAndCodingIndex),
            isStableState(true),
            highThroughputModeIsEnabled(false)
        {}

        void IncrementUnicastSentCount();
        void IncrementUnicastDroppedCount();

        bool CanIncreaseDatarate() const;
        bool ShouldDecreaseDatarate() const;

        void IncreaseDatarate();
        void DecreaseDatarate();

        void CancelRateUpgradeEvent();
        void ScheduleRateUpgradeEvent();
        void ExecuteRateUpgradeEvent();

        ModulationAndCodingSchemesType GetCurrentModulationAndCoding() const
            { return (arfPtr->modulationAndCodingSchemes.at(currentModulationAndCodingIndex)); }

    private:
        const AdaptiveRateControllerWithArf* arfPtr;

        int consecutiveUnicastPacketsSentStat;
        int consecutiveDroppedPacketsStat;
        unsigned int currentModulationAndCodingIndex;
        bool isStableState;
        EventRescheduleTicket rateUpgradeEventTicket;
        bool highThroughputModeIsEnabled;

    };//AckCounterEntry//

    mutable map<MacAddress, shared_ptr<AckCounterEntry> > macAddressAndDatarateMap;

    class RateUpgradeEvent : public SimulationEvent {
    public:
        RateUpgradeEvent(const shared_ptr<AckCounterEntry>& initAckCounterEnvtryPtr)
            : ackCounterEnvtryPtr(initAckCounterEnvtryPtr) {}

        void ExecuteEvent() {
            ackCounterEnvtryPtr->ExecuteRateUpgradeEvent();
        }
    private:
        const shared_ptr<AckCounterEntry> ackCounterEnvtryPtr;
    };//RateUpgradeEvent//

    AckCounterEntry& GetAckCounterEntry(const MacAddress& macAddress) const;

};//AdaptiveRateControllerWithArf//


inline
AdaptiveRateControllerWithArf::AdaptiveRateControllerWithArf(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId,
    const unsigned int baseChannelBandwidthMhz)
    :
    simEngineInterfacePtr(initSimulationEngineInterfacePtr),
    minChannelBandwidthMhz(baseChannelBandwidthMhz),
    maxChannelBandwidthMhz(baseChannelBandwidthMhz)
{
    ackDatarateOption = ReadAckDatarateOption(theParameterDatabaseReader, theNodeId, theInterfaceId);

    rateUpgradeTimerDuration =
        theParameterDatabaseReader.ReadTime("dot11-arf-timer-duration", theNodeId, theInterfaceId);

    ackInSuccessCountThreshold =
        theParameterDatabaseReader.ReadInt("dot11-arf-ack-in-success-count", theNodeId, theInterfaceId);

    ackInFailureCountThreshold =
        theParameterDatabaseReader.ReadInt("dot11-arf-ack-in-failure-count", theNodeId, theInterfaceId);

    ackInFailureCountThresholdOfNewRateState =
        theParameterDatabaseReader.ReadInt("dot11-arf-ack-in-failure-count-of-new-rate-state", theNodeId, theInterfaceId);

    lowestModulationAndCoding = minModulationAndCodingScheme;

    highThroughputModeIsOn = false;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "enable-high-throughput-mode"),theNodeId, theInterfaceId)) {

        highThroughputModeIsOn =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "enable-high-throughput-mode"), theNodeId, theInterfaceId);
    }//if//

    //unicast
    const string modAndCodingListString =
        theParameterDatabaseReader.ReadString("dot11-modulation-and-coding-list", theNodeId, theInterfaceId);

    vector<string> modAndCodingNameVector;
    bool isSuccess;
    ConvertAStringIntoAVectorOfStrings(modAndCodingListString, isSuccess, modAndCodingNameVector);

    if ((!isSuccess) || (modAndCodingNameVector.empty())) {
        cerr << "Error in \"dot11-modulation-and-coding-list\" parameter was invalid." << endl;
        exit(1);
    }//if//

    modulationAndCodingSchemes.resize(modAndCodingNameVector.size());

    for(unsigned int i = 0; (i < modAndCodingNameVector.size()); i++) {
        bool success;
        GetModulationAndCodingSchemeFromName(
            modAndCodingNameVector[i],
            success,
            modulationAndCodingSchemes[i]);

        if (!success) {
            cerr << "Error in \"dot11-modulation-and-coding-list\" value: "
                 << modAndCodingNameVector[i] << " was invalid." << endl;
            exit(1);
        }//if//

        if ((i != 0) && (modulationAndCodingSchemes[i-1] >= modulationAndCodingSchemes[i])) {
            cerr << "Error in \"dot11-modulation-and-coding-list\". Schemes must be in "
                 << "increasing speed order." << endl;
            exit(1);
        }//if//
    }//for//

    //management
    if (theParameterDatabaseReader.ParameterExists(
        "dot11-modulation-and-coding-for-management-frames", theNodeId, theInterfaceId)) {

        modAndCodingForManagementFrames =
            ConvertNameToModulationAndCodingScheme(
                theParameterDatabaseReader.ReadString(
                    "dot11-modulation-and-coding-for-management-frames", theNodeId, theInterfaceId),
                    "dot11-modulation-and-coding-for-management-frames");
    }
    else {
        modAndCodingForManagementFrames = lowestModulationAndCoding;
    }//if//

    //broadcast
    if (theParameterDatabaseReader.ParameterExists(
        "dot11-modulation-and-coding-for-broadcast", theNodeId, theInterfaceId)) {

        modAndCodingForBroadcast =
            ConvertNameToModulationAndCodingScheme(
                theParameterDatabaseReader.ReadString(
                    "dot11-modulation-and-coding-for-broadcast", theNodeId, theInterfaceId),
                "dot11-modulation-and-coding-for-broadcast");
    }
    else {
        modAndCodingForBroadcast = lowestModulationAndCoding;
    }//if//

}//AdaptiveRateControllerWithArf//



inline
void AdaptiveRateControllerWithArf::GetDataRateInfoForDataFrameToStation(
        const MacAddress& macAddress,
        TransmissionParameters& txParameters) const
{
    if (macAddress.IsABroadcastOrAMulticastAddress()) {
        txParameters.modulationAndCodingScheme = modAndCodingForBroadcast;
    }
    else {
        txParameters.modulationAndCodingScheme =
            GetAckCounterEntry(macAddress).GetCurrentModulationAndCoding();
    }//if//

    txParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = highThroughputModeIsOn;

}//GetDataRateInformationForStation//


inline
void AdaptiveRateControllerWithArf::GetDataRateInfoForAckFrame(
    const MacAddress& macAddress,
    const TransmissionParameters& receivedFrameTxParameters,
    TransmissionParameters& ackTxParameters) const
{
    ackTxParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    if (ackDatarateOption == ackRateSameAsData) {
        ackTxParameters.modulationAndCodingScheme =
            receivedFrameTxParameters.modulationAndCodingScheme;
    }//if//

    ackTxParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    ackTxParameters.numberSpatialStreams = 1;
    ackTxParameters.isHighThroughputFrame = highThroughputModeIsOn;

}//GetDataRateInfoForAckFrame//



inline
void AdaptiveRateControllerWithArf::GetDataRateInfoForManagementFrameToStation(
    const MacAddress& macAddress,
    TransmissionParameters& txParameters) const
{
    txParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    txParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = highThroughputModeIsOn;

}//GetDataRateInfoForManagementFrameToStation//


inline
void AdaptiveRateControllerWithArf::GetDataRateInfoForBeaconFrame(
    TransmissionParameters& txParameters) const
{
    txParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    txParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = false;
}



inline
AdaptiveRateControllerWithArf::AckCounterEntry&
AdaptiveRateControllerWithArf::GetAckCounterEntry(const MacAddress& macAddress) const
{
    typedef map<MacAddress, shared_ptr<AckCounterEntry> >::const_iterator IterType;

    IterType iter = macAddressAndDatarateMap.find(macAddress);

    if (iter != macAddressAndDatarateMap.end()) {
        return *(*iter).second;
    }//if//

    const unsigned int highestModulationIndex = static_cast<unsigned int>(modulationAndCodingSchemes.size() - 1);

    shared_ptr<AckCounterEntry> newEntryPtr(new AckCounterEntry(this, highestModulationIndex));

    macAddressAndDatarateMap[macAddress] = newEntryPtr;

    return *newEntryPtr;

}//GetAckCounterEntryPtr//


inline
void AdaptiveRateControllerWithArf::NotifyAckReceived(const MacAddress& macAddress)
{
    // Ignore ack frames for management frame before transmitting data

    if (macAddressAndDatarateMap.find(macAddress) == macAddressAndDatarateMap.end()) {
        return;
    }//if//

    AckCounterEntry& ackCounterEntry = (*this).GetAckCounterEntry(macAddress);

    ackCounterEntry.IncrementUnicastSentCount();

    if (ackCounterEntry.CanIncreaseDatarate()) {
        ackCounterEntry.IncreaseDatarate();
        ackCounterEntry.CancelRateUpgradeEvent();
    }//if//

}//NotifyAckReceived//


inline
void AdaptiveRateControllerWithArf::NotifyAckFailed(const MacAddress& macAddress)
{
    AckCounterEntry& ackCounterEntry = (*this).GetAckCounterEntry(macAddress);

    ackCounterEntry.IncrementUnicastDroppedCount();

    if (ackCounterEntry.ShouldDecreaseDatarate()) {
        ackCounterEntry.DecreaseDatarate();
        ackCounterEntry.ScheduleRateUpgradeEvent();
    }//if//

}//NotifyAckFailed//


inline
void AdaptiveRateControllerWithArf::AckCounterEntry::IncrementUnicastSentCount()
{
    consecutiveDroppedPacketsStat = 0;

    consecutiveUnicastPacketsSentStat = std::min(
        consecutiveUnicastPacketsSentStat + 1,
        arfPtr->ackInSuccessCountThreshold);

    isStableState = true;

}//IncrementUnicastSentCount//


inline
void AdaptiveRateControllerWithArf::AckCounterEntry::IncrementUnicastDroppedCount()
{
    consecutiveUnicastPacketsSentStat = 0;

    const int maxAckInFailureCountThreshold = std::max(
        arfPtr->ackInFailureCountThreshold,
        arfPtr->ackInFailureCountThresholdOfNewRateState);

    consecutiveDroppedPacketsStat = std::min(
        consecutiveDroppedPacketsStat + 1,
        maxAckInFailureCountThreshold);

}//IncrementUnicastDroppedCount//


inline
bool AdaptiveRateControllerWithArf::AckCounterEntry::CanIncreaseDatarate() const
{
    //The number of success is larger than threshold//
    return (consecutiveUnicastPacketsSentStat >= arfPtr->ackInSuccessCountThreshold);

}//CanIncreaseDatarate//


inline
bool AdaptiveRateControllerWithArf::AckCounterEntry::ShouldDecreaseDatarate() const
{
    //The number of failure is larger than threshold, or starting connection of new rate is failure//
    return
        ((currentModulationAndCodingIndex != 0) &&
         ((consecutiveDroppedPacketsStat >= arfPtr->ackInFailureCountThreshold) ||
         ((!isStableState) &&
          (consecutiveDroppedPacketsStat >= arfPtr->ackInFailureCountThresholdOfNewRateState))));

}//ShouldDecreaseDatarate//


inline
void AdaptiveRateControllerWithArf::AckCounterEntry::IncreaseDatarate()
{
    if (currentModulationAndCodingIndex != (arfPtr->modulationAndCodingSchemes.size() - 1)) {
        currentModulationAndCodingIndex++;
        isStableState = false;
    }//if//

    consecutiveDroppedPacketsStat = 0;
    consecutiveUnicastPacketsSentStat = 0;

}//IncreaseDatarate//


inline
void AdaptiveRateControllerWithArf::AckCounterEntry::DecreaseDatarate()
{
    if (currentModulationAndCodingIndex != 0) {
        currentModulationAndCodingIndex--;
    }//if//

    consecutiveDroppedPacketsStat = 0;
    consecutiveUnicastPacketsSentStat = 0;

}//DecreaseDatarate//


inline
void AdaptiveRateControllerWithArf::AckCounterEntry::CancelRateUpgradeEvent()
{
    if (!rateUpgradeEventTicket.IsNull()) {
        arfPtr->simEngineInterfacePtr->CancelEvent(rateUpgradeEventTicket);
    }//if//

}//CancelRateUpgradeEvent//


inline
void AdaptiveRateControllerWithArf::AckCounterEntry::ScheduleRateUpgradeEvent()
{

    const SimTime rateUpgradeTime =
        (*arfPtr).simEngineInterfacePtr->CurrentTime() + arfPtr->rateUpgradeTimerDuration;

    if (rateUpgradeEventTicket.IsNull()) {
        const shared_ptr<RateUpgradeEvent> rateUpgradeEventPtr(
            new RateUpgradeEvent((*this).shared_from_this()));

        (*arfPtr).simEngineInterfacePtr->ScheduleEvent(
            rateUpgradeEventPtr,
            rateUpgradeTime,
            rateUpgradeEventTicket);
    }
    else {
        (*arfPtr).simEngineInterfacePtr->RescheduleEvent(
            rateUpgradeEventTicket,
            rateUpgradeTime);
    }//if//

}//ScheduleRateUpgradeEvent//


inline
void AdaptiveRateControllerWithArf::AckCounterEntry::ExecuteRateUpgradeEvent()
{
    (*this).IncreaseDatarate();
    rateUpgradeEventTicket.Clear();

}//AdaptiveRateControllerWithArf//

//--------------------------------------------------------------------------------------------------

inline
shared_ptr<AdaptiveRateController> CreateAdaptiveRateController(
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId,
    const unsigned int baseChannelBandwidthMhz)
{
    if (!theParameterDatabaseReader.ParameterExists(
            "dot11-adaptive-rate-control-type", theNodeId, theInterfaceId)) {

        return (shared_ptr<AdaptiveRateController>(
            new StaticRateController(
                theParameterDatabaseReader,
                theNodeId,
                theInterfaceId,
                baseChannelBandwidthMhz)));
    }
    else {
        string adaptiveRateControlType =
            theParameterDatabaseReader.ReadString(
                "dot11-adaptive-rate-control-type", theNodeId, theInterfaceId);

        ConvertStringToLowerCase(adaptiveRateControlType);

        if (adaptiveRateControlType == "arf") {
            return (shared_ptr<AdaptiveRateController>(
                new AdaptiveRateControllerWithArf(
                    simulationEngineInterfacePtr,
                    theParameterDatabaseReader,
                    theNodeId,
                    theInterfaceId,
                    baseChannelBandwidthMhz)));
        }
        else if ((adaptiveRateControlType == "static") || (adaptiveRateControlType == "fixed"/*will be deleted*/)) {
            return (shared_ptr<AdaptiveRateController>(
                new StaticRateController(
                    theParameterDatabaseReader,
                    theNodeId,
                    theInterfaceId,
                    baseChannelBandwidthMhz)));
        }
        else {
            cerr << "Error: dot11-adaptive-rate-control-type parameter is not valid: "
                 << adaptiveRateControlType << endl;
            exit(1);
            return shared_ptr<AdaptiveRateController>();
        }//if//
    }//if//

}//CreateAdaptiveRateController//


}//namespace

#endif
