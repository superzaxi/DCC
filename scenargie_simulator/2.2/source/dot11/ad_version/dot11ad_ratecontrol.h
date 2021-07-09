// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11AD_RATECONTROL_H
#define DOT11AD_RATECONTROL_H

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "dot11ad_common.h"
#include "dot11ad_phy.h"

#include <map>
#include <string>
#include <vector>

namespace Dot11ad {

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
using ScenSim::ConvertAStringSequenceOfStringPairsIntoAMap;

using ScenSim::CalcNodeId;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeId;
using ScenSim::InterfaceId;
using ScenSim::SimulationEngineInterface;
using ScenSim::SimTime;
using ScenSim::SimulationEvent;

using ScenSim::MILLI_SECOND;

using ScenSim::TraceMac;

class Dot11MacDurationCalculationInterface;


//----------------------------------------------------------------------------------------------


class SimpleAckDatarateController {
public:
    SimpleAckDatarateController(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId);

    void GetDataRateInfoForAckFrame(
        const MacAddress& macAddress,
        const TransmissionParameters& receivedFrameTxParameters,
        TransmissionParameters& ackTxParameters) const;

private:
    enum AckDatarateOptionType {
        AckRateSameAsData,
        AckRateFromTable,
    };

    AckDatarateOptionType ackDatarateOption;

    // Seems crazy:
    bool matchNumSpatialStreamsModeIsOn;

    vector<ModulationAndCodingSchemesType> ackDatarateTable;

    void ReadInAckDatarateTable(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId);

};//SimpleAckDatarateController//



inline
void SimpleAckDatarateController::ReadInAckDatarateTable(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId)
{
    string ackDatarateSelectionTableString =
        MakeLowerCaseString(
            theParameterDatabaseReader.ReadString(
                parameterNamePrefix + "ack-datarate-selection-table", theNodeId, theInterfaceId));

    ConvertStringToUpperCase(ackDatarateSelectionTableString);
    DeleteTrailingSpaces(ackDatarateSelectionTableString);

    map<string, string> ackDatarateMap;
    bool success;

    ConvertAStringSequenceOfStringPairsIntoAMap(
        ackDatarateSelectionTableString,
        success,
        ackDatarateMap);

    if (!success) {
        cerr << "Error in parameter \"" << (parameterNamePrefix + "ack-datarate-selection-table")
             << "\": bad format." << endl;
        cerr << "     " << ackDatarateSelectionTableString << endl;
        exit(1);
    }//if//

    // Check for bad key values.

    typedef map<string, string>::const_iterator IterType;

    for(IterType iter = ackDatarateMap.begin(); (iter != ackDatarateMap.end()); ++iter) {
        ModulationAndCodingSchemesType notUsed;

        GetModulationAndCodingSchemeFromName(
            MakeUpperCaseString(iter->first), success, notUsed);

        if (!success) {
            cerr << "Error in parameter \"" << (parameterNamePrefix + "ack-datarate-selection-table")
                 << "\": unknown MCS = " << iter->first << endl;
            exit(1);
        }//if//
    }//for//

    ackDatarateTable.resize(maxModulationAndCodingScheme - minModulationAndCodingScheme + 1);

    ModulationAndCodingSchemesType mcs = minModulationAndCodingScheme;

    while (mcs <= maxModulationAndCodingScheme) {
        const string mcsName = GetModulationAndCodingName(mcs);
        if (ackDatarateMap.find(mcsName) == ackDatarateMap.end()) {
            cerr << "Error in parameter \"" << (parameterNamePrefix + "ack-datarate-selection-table")
                 << "\": no value for MCS = " << mcsName << endl;
            exit(1);
        }//if//

        const string mcsValueString = ackDatarateMap[mcsName];

        ModulationAndCodingSchemesType mcsValue = minModulationAndCodingScheme;

        GetModulationAndCodingSchemeFromName(
            MakeUpperCaseString(mcsValueString), success, mcsValue);

        if (!success) {
            cerr << "Error in parameter \"" << (parameterNamePrefix + "ack-datarate-selection-table")
                 << "\": unknown MCS = " << mcsValueString << endl;
            exit(1);
        }//if//

        (*this).ackDatarateTable.at(mcs) = mcsValue;

        IncrementModAndCodingScheme(mcs);
    }//while//

}//ReadInAckDatarateTable//



inline
SimpleAckDatarateController::SimpleAckDatarateController(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId)
    :
    matchNumSpatialStreamsModeIsOn(false)
{
    ackDatarateOption = AckRateSameAsData;

    if (theParameterDatabaseReader.ParameterExists(
        parameterNamePrefix + "ack-datarate-selection-type", theNodeId, theInterfaceId)) {

        string ackDatarateSelectionString =
            MakeLowerCaseString(
                theParameterDatabaseReader.ReadString(
                    parameterNamePrefix + "ack-datarate-selection-type", theNodeId, theInterfaceId));

        ConvertStringToLowerCase(ackDatarateSelectionString);

        if (ackDatarateSelectionString == "sameasdata") {
            ackDatarateOption = AckRateSameAsData;
        }
        else if (ackDatarateSelectionString == "table") {

            ackDatarateOption = AckRateFromTable;

            (*this).ReadInAckDatarateTable(theParameterDatabaseReader, theNodeId, theInterfaceId);
        }
        else {
            cerr << "Error: Unkown dot11-ack-datarate-selection-type = " << ackDatarateSelectionString << endl;
            exit(1);
        }//if//
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "ack-datarate-match-num-spatial-streams"), theNodeId, theInterfaceId)) {

        matchNumSpatialStreamsModeIsOn =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "ack-datarate-match-num-spatial-streams"), theNodeId, theInterfaceId);

    }//if//

}//SimpleAckDatarateController//



inline
void SimpleAckDatarateController::GetDataRateInfoForAckFrame(
    const MacAddress& macAddress,
    const TransmissionParameters& receivedFrameTxParameters,
    TransmissionParameters& ackTxParameters) const
{
    ackTxParameters.numberSpatialStreams = 1;
    if (matchNumSpatialStreamsModeIsOn) {
        ackTxParameters.numberSpatialStreams = receivedFrameTxParameters.numberSpatialStreams;
    }//if//

    ackTxParameters.isHighThroughputFrame = receivedFrameTxParameters.isHighThroughputFrame;
    ackTxParameters.bandwidthNumberChannels = receivedFrameTxParameters.bandwidthNumberChannels;

    switch (ackDatarateOption) {
    case AckRateSameAsData:
        ackTxParameters.modulationAndCodingScheme = receivedFrameTxParameters.modulationAndCodingScheme;
        break;

    case AckRateFromTable:
        ackTxParameters.modulationAndCodingScheme =
            ackDatarateTable.at(
                (receivedFrameTxParameters.modulationAndCodingScheme - minModulationAndCodingScheme));
        break;
    default:
        assert(false); abort();
        break;
    }//switch//

}//GetDataRateInfoForAckFrameToStation//

const unsigned int DefaultShortFrameRetryLimit = 7;
const unsigned int DefaultLongFrameRetryLimit = 4;


//-------------------------------------------------------------------------------------------------

class AdaptiveRateController {
public:
    static const string modelName;

    AdaptiveRateController(
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const InterfaceId& initInterfaceId,
        const shared_ptr<Dot11MacDurationCalculationInterface>& initMacDurationCalculationInterfacePtr)
        :
        simEngineInterfacePtr(initSimEngineInterfacePtr),
        theInterfaceId(initInterfaceId),
        macDurationCalculationInterfacePtr(initMacDurationCalculationInterfacePtr)
    {}

    virtual ~AdaptiveRateController() { }

    virtual bool GetHighThroughputModeIsOn() const = 0;

    virtual unsigned int GetBaseChannelBandwidthMhz() const = 0;

    virtual void SetMaxChannelBandwidthMhz(const unsigned int newMaxChannelBandwidthMhz) = 0;

    virtual unsigned int GetMaxChannelBandwidthMhz() const = 0;

    virtual ModulationAndCodingSchemesType GetLowestModulationAndCoding() const = 0;

    virtual bool StationIsKnown(const MacAddress& macAddress) const = 0;

    virtual unsigned int GetStationBandwidthNumChannels(const MacAddress& macAddress) const = 0;

    virtual void AddNewStation(
        const MacAddress& macAddress,
        const unsigned int stationBandwidthNumChannels,
        const bool isHighThroughputStation) = 0;

    // Bizarrely, Minstrel wants to sporatically force the use of RTS/CTS even if the
    // RTS/CTS is disabled.  Minstrel also arbitrarily changes retry limits.

    virtual void GetDataRateInfoForDataFrameToStation(
        const MacAddress& macAddress,
        const unsigned int retryTxCount,
        const unsigned int retrySequenceIndex, // For Minstrel's "Retry Chain/Sequences" (Is the AC).
        TransmissionParameters& txParameters,
        bool& useRtsEvenIfDisabled,
        bool& overridesStandardRetryTxLimits,
        unsigned int& overriddenRetryTxLimit) const = 0;

    // Ignore Minstrel stuff version (force RTS/CTS and retry count manipulation).

    virtual void GetDataRateInfoForDataFrameToStation(
        const MacAddress& macAddress,
        const unsigned int retryTxCount,
        const unsigned int retrySequenceIndex,
        TransmissionParameters& txParameters) const
    {
        bool notUsedUseRtsEvenIfDisabled;
        bool notUsedOverridesStandardRetryTxLimits;
        unsigned int notUsedOverriddenRetryTxLimit;

        (*this).GetDataRateInfoForDataFrameToStation(
            macAddress,
            retryTxCount,
            retrySequenceIndex,
            txParameters,
            notUsedUseRtsEvenIfDisabled,
            notUsedOverridesStandardRetryTxLimits,
            notUsedOverriddenRetryTxLimit);
    }


    virtual void GetDataRateInfoForAckFrameToStation(
        const MacAddress& macAddress,
        const TransmissionParameters& receivedFrameTxParameters,
        TransmissionParameters& ackTxParameters) const = 0;

    virtual void GetDataRateInfoForRtsOrCtsFrameToStation(
        const MacAddress& macAddress,
        TransmissionParameters& txParameters) const;

    // For calculating NAV duration.

    virtual void GetDataRateInfoForAckFrameFromStation(
        const MacAddress& macAddress,
        const TransmissionParameters& sentFrameTxParameters,
        TransmissionParameters& ackTxParameters) const = 0;

    virtual void GetDataRateInfoForManagementFrameToStation(
        const MacAddress& macAddress,
        TransmissionParameters& txParameters) const = 0;

    virtual void GetDataRateInfoForBeaconFrame(
        TransmissionParameters& txParameters) const = 0;

    virtual void NotifyAckReceived(const MacAddress& macAddress) { }
    virtual void NotifyAckFailed(const MacAddress& macAddress) { }

    //Not Used virtual void NotifyBlockAckNotReceivedForAggregate(
    //Not Used     const MacAddress& macAddress,
    //Not Used     const unsigned int totalNumberSubframes) { }

    //Not Used virtual void NotifyBlockAckReceived(
    //Not Used     const MacAddress& macAddress,
    //Not Used     const unsigned int numberAckedSubframes,
    //Not Used     const unsigned int totalNumberSubframes) { }

    virtual void ReceiveIncomingFrameSinrValue(
        const MacAddress& sourceMacAddress,
        const double& measuredSinrValue) { }

    virtual void NotifyStartingAFrameTransmissionSequence(
        const MacAddress& macAddress,
        const unsigned int retrySequenceIndex) {}

    virtual void NotifyFinishingAFrameTransmissionSequence(
        const MacAddress& macAddress,
        const unsigned int numberTransmittedMpdus,
        const unsigned int numberSuccessfulMpdus,
        const unsigned int finalDataFrameRetryTxCount,
        const unsigned int retrySequenceIndex) {}

    void OutputTraceForTxDatarateUpdate(const MacAddress& macAddress);

protected:

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    InterfaceId theInterfaceId;

    shared_ptr<Dot11MacDurationCalculationInterface> macDurationCalculationInterfacePtr;

};//AdaptiveRateController//


inline
void AdaptiveRateController::GetDataRateInfoForRtsOrCtsFrameToStation(
    const MacAddress& macAddress,
    TransmissionParameters& txParameters) const
{
    (*this).GetDataRateInfoForManagementFrameToStation(macAddress, txParameters);
    assert(txParameters.bandwidthNumberChannels == 1);
    assert(GetStationBandwidthNumChannels(macAddress) == 1);

    txParameters.bandwidthNumberChannels = 1;

}//GetDataRateInfoForRtsOrCtsFrameToStation//


//--------------------------------------------------------------------------------------------------


class StaticRateController : public AdaptiveRateController {
public:
    StaticRateController(
        const shared_ptr<SimulationEngineInterface>& initimulationEngineInterfacePtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int baseChannelBandwidthMhz,
        const unsigned int maxChannelBandwidthMhz,
        const bool initHighThroughModeIsOn,
        const shared_ptr<Dot11MacDurationCalculationInterface>& initMacDurationCalculationInterfacePtr);

    virtual ~StaticRateController() { }

    virtual bool GetHighThroughputModeIsOn() const override { return (highThroughputModeIsOn); }

    virtual ModulationAndCodingSchemesType GetLowestModulationAndCoding() const override {
        return (lowestModulationAndCoding);
    }

    virtual unsigned int GetBaseChannelBandwidthMhz() const override
        { return (baseChannelBandwidthMhz); }

    virtual void SetMaxChannelBandwidthMhz(const unsigned int newMaxChannelBandwidthMhz) override
        { (*this).maxChannelBandwidthMhz = newMaxChannelBandwidthMhz; }

    virtual unsigned int GetMaxChannelBandwidthMhz() const override
        { return (maxChannelBandwidthMhz); }

    // Don't have to Add a station.

    virtual bool StationIsKnown(const MacAddress& macAddress) const override { return true; }

    virtual unsigned int GetStationBandwidthNumChannels(const MacAddress& macAddress) const override;

    virtual void AddNewStation(
        const MacAddress& macAddress,
        const unsigned int stationBandwidthNumChannels,
        const bool isHighThroughputStation) override;

    virtual void GetDataRateInfoForDataFrameToStation(
        const MacAddress& macAddress,
        const unsigned int retryTxCount,
        const unsigned int retrySequenceIndex,
        TransmissionParameters& txParameters,
        bool& useRtsEvenIfDisabled,
        bool& overridesStandardRetryTxLimits,
        unsigned int& overriddenRetryTxLimit) const override;

    virtual void GetDataRateInfoForAckFrameToStation(
        const MacAddress& macAddress,
        const TransmissionParameters& receivedFrameTxParameters,
        TransmissionParameters& ackTxParameters) const override;

    virtual void GetDataRateInfoForAckFrameFromStation(
        const MacAddress& macAddress,
        const TransmissionParameters& receivedFrameTxParameters,
        TransmissionParameters& ackTxParameters) const override;

    virtual void GetDataRateInfoForManagementFrameToStation(
        const MacAddress& macAddress,
        TransmissionParameters& txParameters) const override;

    virtual void GetDataRateInfoForBeaconFrame(
        TransmissionParameters& txParameters) const override;

private:
    unsigned int baseChannelBandwidthMhz;
    unsigned int maxChannelBandwidthMhz;

    bool highThroughputModeIsOn;

    // For broadcast/unknown destinations, except Beacons.
    bool forceUseOfHighThroughputFrames;

    ModulationAndCodingSchemesType lowestModulationAndCoding;

    ModAndCodingSchemeWithNumSpatialStreams defaultDataModAndCodingAndNumStreams;
    ModAndCodingSchemeWithNumSpatialStreams modAndCodingForBroadcast;
    bool useFullBandwidthForBroadcast;

    ModulationAndCodingSchemesType modAndCodingForManagementFrames;

    struct StationInfo {
        bool isAHighThroughputStation;
        unsigned int stationBandwidthNumChannels;
        ModAndCodingSchemeWithNumSpatialStreams modAndCodingWithStreams;
        StationInfo() : isAHighThroughputStation(true), stationBandwidthNumChannels(1) { }
    };

    map<NodeId, StationInfo> stationInfoMap;

    SimpleAckDatarateController ackDatarateController;

    void ParseDatarateTable(
        const string& datarateTableString, bool& success);

};//StaticRateController//




inline
StaticRateController::StaticRateController(
    const shared_ptr<SimulationEngineInterface>& initimulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& initInterfaceId,
    const unsigned int initBaseChannelBandwidthMhz,
    const unsigned int initMaxChannelBandwidthMhz,
    const bool initHighThroughputModeIsOn,
    const shared_ptr<Dot11MacDurationCalculationInterface>& initMacDurationCalculationInterfacePtr)
    :
    AdaptiveRateController(
        initimulationEngineInterfacePtr,
        initInterfaceId,
        initMacDurationCalculationInterfacePtr),
    baseChannelBandwidthMhz(initBaseChannelBandwidthMhz),
    maxChannelBandwidthMhz(initMaxChannelBandwidthMhz),
    highThroughputModeIsOn(initHighThroughputModeIsOn),
    ackDatarateController(theParameterDatabaseReader, theNodeId, theInterfaceId)
{
    forceUseOfHighThroughputFrames = false;

    if (highThroughputModeIsOn) {

        if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "force-use-of-high-throughput-frames"), theNodeId, theInterfaceId)) {

            forceUseOfHighThroughputFrames =
                theParameterDatabaseReader.ReadBool(
                    (parameterNamePrefix + "force-use-of-high-throughput-frames"),
                    theNodeId, theInterfaceId);
        }//if//
    }//if//

    if ((maxChannelBandwidthMhz > baseChannelBandwidthMhz) && (!highThroughputModeIsOn)) {
        cerr << "Error: Wide bandwidth channels defined but "
             << (parameterNamePrefix + "enable-high-throughput-mode")
             << " parameter is not defined or false." << endl;
        exit(1);
    }//if//

    //Default Data

    defaultDataModAndCodingAndNumStreams =
        ConvertNameToModAndCodingSchemeWithNumStreams(
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
        modAndCodingForManagementFrames = defaultDataModAndCodingAndNumStreams.modAndCoding;
    }//if//

    // Broadcast

    if (theParameterDatabaseReader.ParameterExists(
            "dot11-modulation-and-coding-for-broadcast", theNodeId, theInterfaceId)) {

        modAndCodingForBroadcast =
            ConvertNameToModAndCodingSchemeWithNumStreams(
                theParameterDatabaseReader.ReadString(
                    "dot11-modulation-and-coding-for-broadcast", theNodeId, theInterfaceId),
                "dot11-modulation-and-coding-for-broadcast");
    }
    else {
        //default datarate
        modAndCodingForBroadcast = defaultDataModAndCodingAndNumStreams;

    }//if//

    // Broadcast

    useFullBandwidthForBroadcast = false;

    if (theParameterDatabaseReader.ParameterExists(
            "dot11-use-full-bandwidth-for-broadcast", theNodeId, theInterfaceId)) {

        useFullBandwidthForBroadcast =
            theParameterDatabaseReader.ReadBool(
                "dot11-use-full-bandwidth-for-broadcast", theNodeId, theInterfaceId);
    }//if//

    //lowest

    lowestModulationAndCoding =
        ScenSim::MinOf3(
            defaultDataModAndCodingAndNumStreams.modAndCoding,
            modAndCodingForManagementFrames,
            modAndCodingForBroadcast.modAndCoding);

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

    }//if//

}//StaticRateController//


inline
void StaticRateController::ParseDatarateTable(const string& datarateTableString, bool& success)
{
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

        const string targetModAndCodingWithSpatialStreamsName =
            aString.substr(endOfTargetNodesTerminatorPos + 1, aString.length() - endOfTargetNodesTerminatorPos);

        ModAndCodingSchemeWithNumSpatialStreams targetModAndCodingWithNumSpatialStreams =
            ConvertNameToModAndCodingSchemeWithNumStreams(
                targetModAndCodingWithSpatialStreamsName, "dot11-modulation-and-coding-table");

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
                    atoi(
                        targetNodesString.substr(
                            currentPosition,
                            (endOfSubfieldTerminatorPos - currentPosition)).c_str());

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
                    atoi(
                        targetNodesString.substr(
                            currentPosition,
                            (dashPosition - currentPosition)).c_str());

                const NodeId upperBoundNumber =
                    atoi(
                        targetNodesString.substr(
                            (dashPosition + 1),
                            (endOfSubfieldTerminatorPos - dashPosition - 1)).c_str());

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

            (*this).stationInfoMap[targetNodeId].modAndCodingWithStreams =
                targetModAndCodingWithNumSpatialStreams;

        }//for//

    }//while//

    success = true;

}//ParseDatarateTable//

inline
unsigned int StaticRateController::GetStationBandwidthNumChannels(
    const MacAddress& macAddress) const
{
    const NodeId targetNodeId = CalcNodeId(macAddress.ConvertToGenericMacAddress());

    typedef map<NodeId, StationInfo>::const_iterator IterType;
    const IterType iter = stationInfoMap.find(targetNodeId);

    if (iter == stationInfoMap.end()) {
        return 1;
    }

    const StationInfo& theStationInfo = iter->second;
    return (theStationInfo.stationBandwidthNumChannels);
}


inline
void StaticRateController::AddNewStation(
    const MacAddress& macAddress,
    const unsigned int stationBandwidthNumChannels,
    const bool isHighThroughputStation)
{
    const NodeId targetNodeId = CalcNodeId(macAddress.ConvertToGenericMacAddress());

    stationInfoMap[targetNodeId].stationBandwidthNumChannels = stationBandwidthNumChannels;
    stationInfoMap[targetNodeId].isAHighThroughputStation = isHighThroughputStation;

}//AddNewStation//



inline
void StaticRateController::GetDataRateInfoForDataFrameToStation(
    const MacAddress& macAddress,
    const unsigned int notUsedRetryTxCount,
    const unsigned int notUsedRetrySequenceIndex,
    TransmissionParameters& txParameters,
    bool& useRtsEvenIfDisabled,
    bool& overridesStandardRetryTxLimits,
    unsigned int& overriddenRetryTxLimit) const
{
    useRtsEvenIfDisabled = false;
    overridesStandardRetryTxLimits = false;

    txParameters.bandwidthNumberChannels = 1;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = false;

    if (macAddress.IsABroadcastOrAMulticastAddress()) {

        //broadcast
        txParameters.modulationAndCodingScheme = modAndCodingForBroadcast.modAndCoding;
        txParameters.isHighThroughputFrame = forceUseOfHighThroughputFrames;
        if (useFullBandwidthForBroadcast) {
            txParameters.bandwidthNumberChannels =
                (maxChannelBandwidthMhz / baseChannelBandwidthMhz);

            txParameters.numberSpatialStreams = modAndCodingForBroadcast.numberSpatialStreams;

            if (txParameters.bandwidthNumberChannels > 1) {
                txParameters.isHighThroughputFrame = true;
            }//if//
        }//if//
    }
    else {
        //unicast

         const NodeId targetNodeId = CalcNodeId(macAddress.ConvertToGenericMacAddress());

         map<NodeId, StationInfo>::const_iterator iter = stationInfoMap.find(targetNodeId);

         if (iter == stationInfoMap.end()) {
             txParameters.modulationAndCodingScheme =
                 defaultDataModAndCodingAndNumStreams.modAndCoding;
             txParameters.numberSpatialStreams =
                 defaultDataModAndCodingAndNumStreams.numberSpatialStreams;

             // Why was this set to max bandwidth? (Probably need parameter).

             txParameters.bandwidthNumberChannels =
                 (maxChannelBandwidthMhz / baseChannelBandwidthMhz);

             txParameters.isHighThroughputFrame = true;
             if (txParameters.bandwidthNumberChannels == 1) {
                 txParameters.isHighThroughputFrame = forceUseOfHighThroughputFrames;
             }//if//
         }
         else {
             const StationInfo& theStationInfo = iter->second;

             txParameters.bandwidthNumberChannels = theStationInfo.stationBandwidthNumChannels;
             txParameters.isHighThroughputFrame = theStationInfo.isAHighThroughputStation;

             if (theStationInfo.modAndCodingWithStreams.modAndCoding != McsInvalid) {

                 txParameters.modulationAndCodingScheme =
                     theStationInfo.modAndCodingWithStreams.modAndCoding;
                 txParameters.numberSpatialStreams =
                     theStationInfo.modAndCodingWithStreams.numberSpatialStreams;
             }
             else {
                 txParameters.modulationAndCodingScheme =
                     defaultDataModAndCodingAndNumStreams.modAndCoding;
                 txParameters.numberSpatialStreams =
                     defaultDataModAndCodingAndNumStreams.numberSpatialStreams;
             }//if//
         }//if//
    }//if//

}//GetDataRateInfoForDataFrameToStation//


inline
void StaticRateController::GetDataRateInfoForAckFrameToStation(
    const MacAddress& macAddress,
    const TransmissionParameters& receivedFrameTxParameters,
    TransmissionParameters& ackTxParameters) const
{
    ackDatarateController.GetDataRateInfoForAckFrame(
        macAddress, receivedFrameTxParameters, ackTxParameters);

}//GetDataRateInfoForAckFrameToStation//



inline
void StaticRateController::GetDataRateInfoForAckFrameFromStation(
    const MacAddress& macAddress,
    const TransmissionParameters& receivedFrameTxParameters,
    TransmissionParameters& ackTxParameters) const
{
    // For now mac address specific ack rates not used.

    ackDatarateController.GetDataRateInfoForAckFrame(
        macAddress, receivedFrameTxParameters, ackTxParameters);

}//GetDataRateInfoForAckFrameFromStation//


inline
void StaticRateController::GetDataRateInfoForBeaconFrame(
    TransmissionParameters& txParameters) const
{
    txParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    txParameters.bandwidthNumberChannels = 1;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = false;
}


inline
void StaticRateController::GetDataRateInfoForManagementFrameToStation(
    const MacAddress& macAddress,
    TransmissionParameters& txParameters) const
{
    txParameters.bandwidthNumberChannels = 1;
    txParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = false;

}//GetDataRateInfoForManagementFrameToStation//



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
        const unsigned int baseChannelBandwidthMhz,
        const unsigned int maxChannelBandwidthMhz,
        const bool initHighThroughputModeIsOn,
        const shared_ptr<Dot11MacDurationCalculationInterface>& initMacDurationCalculationInterfacePtr);

    virtual bool GetHighThroughputModeIsOn() const override { return (highThroughputModeIsOn); }

    virtual ModulationAndCodingSchemesType GetLowestModulationAndCoding() const override {
        return (lowestModulationAndCoding);
    }

    virtual unsigned int GetBaseChannelBandwidthMhz() const override
        { return (baseChannelBandwidthMhz); }

    virtual void SetMaxChannelBandwidthMhz(const unsigned int newMaxChannelBandwidthMhz) override
        { (*this).maxChannelBandwidthMhz = newMaxChannelBandwidthMhz; }

    virtual unsigned int GetMaxChannelBandwidthMhz() const override
        { return (maxChannelBandwidthMhz); }

    virtual bool StationIsKnown(const MacAddress& macAddress) const override
    {
        return (stationInfoMap.find(macAddress) != stationInfoMap.end());
    }

    virtual unsigned int GetStationBandwidthNumChannels(const MacAddress& macAddress) const override;

    virtual void AddNewStation(
        const MacAddress& macAddress,
        const unsigned int stationBandwidthNumChannels,
        const bool isHighThroughputStation) override;

    virtual void GetDataRateInfoForDataFrameToStation(
        const MacAddress& macAddress,
        const unsigned int retryTxCount,
        const unsigned int retrySequenceIndex,
        TransmissionParameters& txParameters,
        bool& useRtsEvenIfDisabled,
        bool& overridesStandardRetryTxLimits,
        unsigned int& overriddenRetryTxLimit) const override;

    virtual void GetDataRateInfoForAckFrameToStation(
        const MacAddress& macAddress,
        const TransmissionParameters& receivedFrameTxParameters,
        TransmissionParameters& ackTxParameters) const override;

    virtual void GetDataRateInfoForAckFrameFromStation(
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

    virtual void NotifyFinishingAFrameTransmissionSequence(
        const MacAddress& macAddress,
        const unsigned int numberTransmittedMpdus,
        const unsigned int numberSuccessfulMpdus,
        const unsigned int finalDataFrameRetryTxCount,
        const unsigned int retrySequenceIndex) override;

    virtual ~AdaptiveRateControllerWithArf() { }

private:

    unsigned int baseChannelBandwidthMhz;
    unsigned int maxChannelBandwidthMhz;

    bool highThroughputModeIsOn;
    // For broadcast/unknown destinations, except Beacons.
    bool forceUseOfHighThroughputFrames;


    ModulationAndCodingSchemesType lowestModulationAndCoding;
    ModulationAndCodingSchemesType modAndCodingForManagementFrames;
    ModulationAndCodingSchemesType modAndCodingForBroadcast;
    vector<ModulationAndCodingSchemesType> modulationAndCodingSchemes;

    SimTime rateUpgradeTimerDuration;
    int ackInSuccessCountThreshold;
    int ackInFailureCountThreshold;
    int ackInFailureCountThresholdOfNewRateState;

    SimpleAckDatarateController ackDatarateController;

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


    struct StationInfo {
        unsigned int stationBandwidthNumChannels;
        bool isAHighThroughputStation;
        shared_ptr<AckCounterEntry> ackCounterEntryPtr;

        StationInfo() : stationBandwidthNumChannels(1), isAHighThroughputStation(true) { }
    };

    map<MacAddress, StationInfo>  stationInfoMap;

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

};//AdaptiveRateControllerWithArf//



inline
AdaptiveRateControllerWithArf::AdaptiveRateControllerWithArf(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& initInterfaceId,
    const unsigned int initBaseChannelBandwidthMhz,
    const unsigned int initMaxChannelBandwidthMhz,
    const bool initHighThroughputModeIsOn,
    const shared_ptr<Dot11MacDurationCalculationInterface>& initMacDurationCalculationInterfacePtr)
    :
    AdaptiveRateController(
        initSimulationEngineInterfacePtr,
        initInterfaceId,
        initMacDurationCalculationInterfacePtr),
    baseChannelBandwidthMhz(initBaseChannelBandwidthMhz),
    maxChannelBandwidthMhz(initMaxChannelBandwidthMhz),
    highThroughputModeIsOn(initHighThroughputModeIsOn),
    ackDatarateController(theParameterDatabaseReader, theNodeId, theInterfaceId)
{
    rateUpgradeTimerDuration =
        theParameterDatabaseReader.ReadTime("dot11-arf-timer-duration", theNodeId, theInterfaceId);

    ackInSuccessCountThreshold =
        theParameterDatabaseReader.ReadInt("dot11-arf-ack-in-success-count", theNodeId, theInterfaceId);

    ackInFailureCountThreshold =
        theParameterDatabaseReader.ReadInt("dot11-arf-ack-in-failure-count", theNodeId, theInterfaceId);

    ackInFailureCountThresholdOfNewRateState =
        theParameterDatabaseReader.ReadInt(
            "dot11-arf-ack-in-failure-count-of-new-rate-state", theNodeId, theInterfaceId);

    lowestModulationAndCoding = minModulationAndCodingScheme;

    forceUseOfHighThroughputFrames = false;

    if (highThroughputModeIsOn) {
        if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "force-use-of-high-throughput-frames"), theNodeId, theInterfaceId)) {

            forceUseOfHighThroughputFrames =
                theParameterDatabaseReader.ReadBool(
                    (parameterNamePrefix + "force-use-of-high-throughput-frames"),
                    theNodeId, theInterfaceId);
        }//if//
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
            MakeUpperCaseString(modAndCodingNameVector[i]),
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
unsigned int AdaptiveRateControllerWithArf::GetStationBandwidthNumChannels(
    const MacAddress& macAddress) const
{
    typedef map<MacAddress, StationInfo>::const_iterator IterType;

    IterType iter = stationInfoMap.find(macAddress);
    assert(iter != stationInfoMap.end());

    const StationInfo& theStationInfo = iter->second;
    return (theStationInfo.stationBandwidthNumChannels);
}


inline
void AdaptiveRateControllerWithArf::AddNewStation(
   const MacAddress& macAddress,
   const unsigned int stationBandwidthNumChannels,
   const bool isHighThroughputStation)
{
    typedef map<MacAddress, StationInfo>::iterator IterType;

    IterType iter = stationInfoMap.find(macAddress);
    if (iter == stationInfoMap.end()) {
        const unsigned int highestModulationIndex =
            static_cast<unsigned int>(modulationAndCodingSchemes.size() - 1);

        StationInfo theStationInfo;
        theStationInfo.stationBandwidthNumChannels = stationBandwidthNumChannels;
        theStationInfo.isAHighThroughputStation = isHighThroughputStation;
        theStationInfo.ackCounterEntryPtr = make_shared<AckCounterEntry>(this, highestModulationIndex);

        (*this).stationInfoMap[macAddress] = theStationInfo;
    }
    else {
        iter->second.isAHighThroughputStation = isHighThroughputStation;
    }//if//

}//AddNewStation//



inline
void AdaptiveRateControllerWithArf::GetDataRateInfoForDataFrameToStation(
    const MacAddress& macAddress,
    const unsigned int notUsedRetryTxCount,
    const unsigned int notUsedRetrySequenceIndex,
    TransmissionParameters& txParameters,
    bool& useRtsEvenIfDisabled,
    bool& overridesStandardRetryTxLimits,
    unsigned int& overriddenRetryTxLimit) const
{
    useRtsEvenIfDisabled = false;
    overridesStandardRetryTxLimits = false;

    txParameters.bandwidthNumberChannels = 1;
    txParameters.numberSpatialStreams = 1;

    if (macAddress.IsABroadcastOrAMulticastAddress()) {
        txParameters.modulationAndCodingScheme = modAndCodingForBroadcast;
        txParameters.isHighThroughputFrame = forceUseOfHighThroughputFrames;
    }
    else {
        typedef map<MacAddress, StationInfo>::const_iterator IterType;

        IterType iter = stationInfoMap.find(macAddress);

        assert((iter != stationInfoMap.end()) && "Station Has not been added");

        const StationInfo& theStationInfo = iter->second;

        txParameters.bandwidthNumberChannels = theStationInfo.stationBandwidthNumChannels;
        txParameters.isHighThroughputFrame = theStationInfo.isAHighThroughputStation;
        txParameters.modulationAndCodingScheme =
            theStationInfo.ackCounterEntryPtr->GetCurrentModulationAndCoding();

    }//if//

}//GetDataRateInfoForDataFrameToStation//


inline
void AdaptiveRateControllerWithArf::GetDataRateInfoForAckFrameToStation(
    const MacAddress& macAddress,
    const TransmissionParameters& receivedFrameTxParameters,
    TransmissionParameters& ackTxParameters) const
{
    ackDatarateController.GetDataRateInfoForAckFrame(
        macAddress, receivedFrameTxParameters, ackTxParameters);

}//GetDataRateInfoForAckFrameToStation//


inline
void AdaptiveRateControllerWithArf::GetDataRateInfoForAckFrameFromStation(
    const MacAddress& macAddress,
    const TransmissionParameters& receivedFrameTxParameters,
    TransmissionParameters& ackTxParameters) const
{
    // For now mac address specific ack rates not used.

    ackDatarateController.GetDataRateInfoForAckFrame(
        macAddress, receivedFrameTxParameters, ackTxParameters);

}//GetDataRateInfoForAckFrameFromStation//



inline
void AdaptiveRateControllerWithArf::GetDataRateInfoForManagementFrameToStation(
    const MacAddress& macAddress,
    TransmissionParameters& txParameters) const
{
    txParameters.bandwidthNumberChannels = 1;
    txParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = false;

}//GetDataRateInfoForManagementFrameToStation//


inline
void AdaptiveRateControllerWithArf::GetDataRateInfoForBeaconFrame(
    TransmissionParameters& txParameters) const
{
    txParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    txParameters.bandwidthNumberChannels = 1;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = false;
}


inline
void AdaptiveRateControllerWithArf::NotifyAckReceived(const MacAddress& macAddress)
{
    // Ignore ack frames for management frame before transmitting data

    if (stationInfoMap.find(macAddress) == stationInfoMap.end()) {
        return;
    }//if//

    StationInfo& theStationInfo = (*this).stationInfoMap[macAddress];

    AckCounterEntry& ackCounterEntry = *theStationInfo.ackCounterEntryPtr;

    ackCounterEntry.IncrementUnicastSentCount();

    if (ackCounterEntry.CanIncreaseDatarate()) {
        ackCounterEntry.IncreaseDatarate();
        ackCounterEntry.CancelRateUpgradeEvent();

        (*this).OutputTraceForTxDatarateUpdate(macAddress);
    }//if//

}//NotifyAckReceived//


inline
void AdaptiveRateControllerWithArf::NotifyAckFailed(const MacAddress& macAddress)
{
    AckCounterEntry& ackCounterEntry = *(*this).stationInfoMap[macAddress].ackCounterEntryPtr;

    ackCounterEntry.IncrementUnicastDroppedCount();

    if (ackCounterEntry.ShouldDecreaseDatarate()) {
        ackCounterEntry.DecreaseDatarate();
        ackCounterEntry.ScheduleRateUpgradeEvent();

        (*this).OutputTraceForTxDatarateUpdate(macAddress);
    }//if//

}//NotifyAckFailed//


inline
void AdaptiveRateControllerWithArf::NotifyFinishingAFrameTransmissionSequence(
    const MacAddress& macAddress,
    const unsigned int numberTransmittedMpdus,
    const unsigned int numberSuccessfulMpdus,
    const unsigned int finalDataFrameRetryTxCount,
    const unsigned int retrySequenceIndex)
{
    // Ignore ack frames for management frame before transmitting data

    if (stationInfoMap.find(macAddress) == stationInfoMap.end()) {
        return;
    }//if//

    if (finalDataFrameRetryTxCount == 0) {
        // Special case: The frame was dropped before transmission by RTS failure

        // Maybe need to downgrade the transmission rate
        // with reference to the management frame(;lowest) transmission rate.
        return;
    }//if//

    AckCounterEntry& ackCounterEntry =
        *(*this).stationInfoMap[macAddress].ackCounterEntryPtr;

    assert((numberTransmittedMpdus*finalDataFrameRetryTxCount) >= numberSuccessfulMpdus);

    const unsigned int numberFailureMpdus =
        ((numberTransmittedMpdus*finalDataFrameRetryTxCount) - numberSuccessfulMpdus);


    // Assuming: success count weigt is event with failure count for A-MPDU.

    if (numberFailureMpdus > numberSuccessfulMpdus) {
        for(unsigned int i = 0; i < (numberFailureMpdus - numberSuccessfulMpdus); i++) {
            ackCounterEntry.IncrementUnicastDroppedCount();
        }//for//

        if (ackCounterEntry.ShouldDecreaseDatarate()) {
            ackCounterEntry.DecreaseDatarate();
            ackCounterEntry.ScheduleRateUpgradeEvent();

            (*this).OutputTraceForTxDatarateUpdate(macAddress);

        }//ifr//
    }
    else {

        for(unsigned int i = 0; i < (numberSuccessfulMpdus - numberFailureMpdus); i++) {
            ackCounterEntry.IncrementUnicastSentCount();
        }//for//

        if (ackCounterEntry.CanIncreaseDatarate()) {
            ackCounterEntry.IncreaseDatarate();
            ackCounterEntry.CancelRateUpgradeEvent();

            (*this).OutputTraceForTxDatarateUpdate(macAddress);
        }//if//
    }

}//NotifyFinishingAFrameTransmissionSequence//


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

class AdaptiveRateControllerWithMinstrelHT : public AdaptiveRateController {
public:
    AdaptiveRateControllerWithMinstrelHT(
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const unsigned int baseChannelBandwidthMhz,
        const unsigned int maxChannelBandwidthMhz,
        const bool highThroughputModeIsOn,
        const shared_ptr<Dot11MacDurationCalculationInterface>& macDurationCalculationInterfacePtr,
        const RandomNumberGeneratorSeed& initNodeSeed);

    virtual void NotifyStartingAFrameTransmissionSequence(
        const MacAddress& macAddress,
        const unsigned int retrySequenceIndex) override;

    virtual void NotifyFinishingAFrameTransmissionSequence(
        const MacAddress& macAddress,
        const unsigned int numberTransmittedMpdus,
        const unsigned int numberSuccessfulMpdus,
        const unsigned int finalDataFrameRetryTxCount,
        const unsigned int retrySequenceIndex) override;

    virtual bool GetHighThroughputModeIsOn() const override { return (highThroughputModeIsOn); }

    virtual ModulationAndCodingSchemesType GetLowestModulationAndCoding() const override {
        return (lowestModulationAndCoding);
    }

    virtual unsigned int GetBaseChannelBandwidthMhz() const override
        { return (baseChannelBandwidthMhz); }

    virtual void SetMaxChannelBandwidthMhz(const unsigned int newMaxChannelBandwidthMhz) override
        { (*this).maxChannelBandwidthMhz = newMaxChannelBandwidthMhz; }

    virtual unsigned int GetMaxChannelBandwidthMhz() const override
        { return (maxChannelBandwidthMhz); }

    virtual bool StationIsKnown(const MacAddress& macAddress) const override {
        return (stationInfoForAddr.find(macAddress) != stationInfoForAddr.end());
    }

    virtual unsigned int GetStationBandwidthNumChannels(
        const MacAddress& macAddress) const override;

    virtual void AddNewStation(
        const MacAddress& macAddress,
        const unsigned int stationBandwidthNumChannels,
        const bool isHighThroughputStation) override;

    virtual void GetDataRateInfoForDataFrameToStation(
        const MacAddress& macAddress,
        const unsigned int retryTxCount,
        const unsigned int retrySequenceIndex,  // For Minstrel's "Retry Chain/Sequences" (Is the AC).
        TransmissionParameters& txParameters,
        bool& useRtsEvenIfDisabled,
        bool& overridesStandardRetryTxLimits,
        unsigned int& overriddenRetryTxLimit) const override;

    virtual void GetDataRateInfoForAckFrameToStation(
        const MacAddress& macAddress,
        const TransmissionParameters& receivedFrameTxParameters,
        TransmissionParameters& ackTxParameters) const override;

    virtual void GetDataRateInfoForAckFrameFromStation(
        const MacAddress& macAddress,
        const TransmissionParameters& receivedFrameTxParameters,
        TransmissionParameters& ackTxParameters) const override;

    virtual void GetDataRateInfoForManagementFrameToStation(
        const MacAddress& macAddress,
        TransmissionParameters& txParameters) const override;

    virtual void GetDataRateInfoForBeaconFrame(
        TransmissionParameters& txParameters) const override;

    virtual ~AdaptiveRateControllerWithMinstrelHT() { }

private:
    typedef uint8_t McsEntryIndex;

    RandomNumberGenerator aRandomNumberGenerator;

    unsigned int baseChannelBandwidthMhz;
    unsigned int maxChannelBandwidthMhz;

    bool highThroughputModeIsOn;
    // For broadcast/unknown destinations, except Beacons.
    bool forceUseOfHighThroughputFrames;

    ModulationAndCodingSchemesType lowestModulationAndCoding;
    ModulationAndCodingSchemesType modAndCodingForManagementFrames;
    ModulationAndCodingSchemesType modAndCodingForBroadcast;

    unsigned int typicalTransmissionUnitLengthBytes;

    unsigned int maxRetryCount;
    unsigned int minRetryCount;
    unsigned int samplingRetryCount;

    SimTime samplingResultUpdateInterval;
    SimTime maxTransmissionDurationForAMultirateRetryStage;

    double exponentiallyWeight;
    double goodSuccessRate;
    double badSuccessRate;
    double worstSuccessRate;
    double maxFrameSuccessRateToEstimateThroughput;

    unsigned int samplingTryingThreshold;
    unsigned int maxLowRateSamplingCount;
    unsigned int enoughNumberOfTransmittedMpdusToSample;
    unsigned int samplingTransmissionInterval;
    unsigned int initialSamplingTransmissionCount;

    static const unsigned int primaryThroughputEntryIndex;
    static const unsigned int secondaryThroughputEntryIndex;

    unsigned int maxRetrySequenceIndex;

    struct ModulationAndCodingSchemeEntry {
        ModulationAndCodingSchemesType modulationAndCodingScheme;
        uint8_t numberOfStreams;
        uint8_t numberOfBondedChannels;

        //bool isShortGuardInterval; --> false: normal guard interval (in dot11ad_phy.h default)

        uint8_t frameRetryCount;
        uint8_t frameRetryCountWithRts;

        DatarateBitsPerSec datarate;

        ModulationAndCodingSchemeEntry(
            const ModulationAndCodingSchemesType& initMdulationAndCodingScheme,
            const uint8_t initNumberOfStreams,
            const uint8_t initNumberOfBondedChannels,
            const DatarateBitsPerSec& initDatarate)
            :
            modulationAndCodingScheme(initMdulationAndCodingScheme),
            numberOfStreams(initNumberOfStreams),
            numberOfBondedChannels(initNumberOfBondedChannels),
            datarate(initDatarate),
            frameRetryCount(2),
            frameRetryCountWithRts(2)
        {}
    };

    vector<ModulationAndCodingSchemesType> modulationAndCodingSchemes;
    vector<ModulationAndCodingSchemeEntry> modulationAndCodingSchemeEntries;

    class SampleTable {
    public:
        SampleTable()
            :
            numberOfSampleColumns(0),
            numberOfRateRows(0)
        {}

        SampleTable(
            const unsigned int initNumberOfRateRows,
            RandomNumberGenerator& initRandomNumberGenerator)
            :
            numberOfSampleColumns(10),
            numberOfRateRows(initNumberOfRateRows),
            currentSampleRow(0),
            currentSampleColumn(0),
            sampleValues(numberOfSampleColumns*numberOfRateRows)
        {
            assert(numberOfRateRows <= 0xFF);
            assert(numberOfRateRows > 0);

            const unsigned int tableSize = numberOfSampleColumns*numberOfRateRows;
            const McsEntryIndex initialValue = 0xff;

            for(unsigned int i = 0; i < tableSize; i++) {
                sampleValues[i] = initialValue;
            }//for//

            for(unsigned int column = 0; column < numberOfSampleColumns; column++) {

                array<McsEntryIndex, 8> randomMcsIndexList;

                for(size_t i = 0; i < randomMcsIndexList.size(); i++) {
                    randomMcsIndexList[i] =
                        static_cast<McsEntryIndex>(
                            initRandomNumberGenerator.GenerateRandomInt(0x00, 0xff));
                }//for//

                for(unsigned int row = 0; row < numberOfRateRows; row++) {
                    unsigned int mcsEntryIndex =
                        (row + randomMcsIndexList[row & (8 - 1)]) % numberOfRateRows;

                    while ((*this).At(mcsEntryIndex, column) != initialValue) {
                        mcsEntryIndex = (mcsEntryIndex+1) % numberOfRateRows;
                    }//while//

                    sampleValues[numberOfSampleColumns*mcsEntryIndex + column] =
                        static_cast<McsEntryIndex>(row);

                }//for//
            }//for//
        }//SampleTable//

        void AdvanceSamplingMcsEntryIndex();

        McsEntryIndex GetCurrentMcsEntryIndex() const {
            return (*this).At(currentSampleRow, currentSampleColumn);
        }//GetCurrentMcsEntryIndex//

    private:
        McsEntryIndex At(const unsigned int row, const unsigned int column) const {
            return sampleValues[numberOfSampleColumns*row + column];
        }

        unsigned int numberOfSampleColumns;
        unsigned int numberOfRateRows;

        unsigned int currentSampleRow;
        unsigned int currentSampleColumn;

        vector<McsEntryIndex> sampleValues;
    };//SampleTable//




    struct ThroughputAndMcsEntryIndex {

        ThroughputAndMcsEntryIndex()
            :
            datarate(0),
            mcsEntryIndex(0)
        {}

        ThroughputAndMcsEntryIndex(
            const DatarateBitsPerSec initDatarate,
            const McsEntryIndex& initMcsEntryIndex)
            :
            datarate(initDatarate),
            mcsEntryIndex(initMcsEntryIndex)
        {}

        DatarateBitsPerSec datarate;
        McsEntryIndex mcsEntryIndex;

        bool operator<(const ThroughputAndMcsEntryIndex& right) const {
            return ((datarate > right.datarate) ||
                    ((datarate == right.datarate) &&
                     (mcsEntryIndex > right.mcsEntryIndex)));
        }

    };//ThroughputAndMcsEntryIndex//

    struct ModulationAndCodingSchemeSamplingInfo {
        unsigned int totalNumberOfTransmittedMpdus;
        unsigned int totalTransmissionSucceededCount;

        double frameSuccessRate;
        bool isFirstTimeSuccessRateCalculation;

        DatarateBitsPerSec estimatedDatarate;

        unsigned int skippedSamplingCount;

        ModulationAndCodingSchemeSamplingInfo()
            :
            totalNumberOfTransmittedMpdus(0),
            totalTransmissionSucceededCount(0),
            frameSuccessRate(0.0),
            isFirstTimeSuccessRateCalculation(true),
            estimatedDatarate(0),
            skippedSamplingCount(0)
        {}

        double GetTransmissionSuccessRate() const {
            return double(totalTransmissionSucceededCount) / totalNumberOfTransmittedMpdus;
        }
    };//ModulationAndCodingSchemeSamplingInfo//


    struct RetryStagesInfo {
        array<unsigned int, 4> dataFrameRetryLimitForRetryStage;
        array<bool, 4> isFrameProtectedByRtsForRetryStage;
    };


    struct StationInfo {

        StationInfo()
            :
            stationBandwidthNumChannels(1),
            isAHighThroughputStation(true),
            lastSamplingResultUpdatedTime(ZERO_TIME),
            remainingInitialSamplingTransmissionCount(0),
            remainingSamplingCount(0),
            samplingWaitCount(0),
            lowRateSampledCount(0),
            totalNumberOfTransmittedMpdus(0),
            totalNumberOfTransmittedFrames(0),
            averageNumberOfAggregatedMpdus(0),
            maxProbabilityMcsEntryIndex(0)
        {
            const McsEntryIndex lowestMcsEntryIndex = 0;

            multirateRetryMcsEntryIndexList.fill(lowestMcsEntryIndex);
        }

        SimTime lastSamplingResultUpdatedTime;

        shared_ptr<SampleTable> sampleTablePtr;

        unsigned int stationBandwidthNumChannels;
        bool isAHighThroughputStation;
        unsigned int remainingInitialSamplingTransmissionCount;
        unsigned int remainingSamplingCount;
        unsigned int samplingWaitCount;
        unsigned int lowRateSampledCount;
        unsigned int totalNumberOfTransmittedMpdus;
        unsigned int totalNumberOfTransmittedFrames;
        unsigned int averageNumberOfAggregatedMpdus;

        McsEntryIndex maxProbabilityMcsEntryIndex;

        array<McsEntryIndex, 4> multirateRetryMcsEntryIndexList;
        vector<ModulationAndCodingSchemeSamplingInfo> modulationAndCodingSchemeSamplingInfoList;
        vector<ThroughputAndMcsEntryIndex> throughputAndMcsIndexList;
        vector<vector<ThroughputAndMcsEntryIndex> > throughputAndMcsIndexListPerStreamGroup;

        // Index by Access Category (to imitate previous model behavor).

        vector<RetryStagesInfo> retryStagesInfos;

    };//StationInfo//


    map<MacAddress, StationInfo> stationInfoForAddr;

    SimpleAckDatarateController ackDatarateController;

    void CreateStationInfoFor(const MacAddress& macAddress);

    void UpdateSamplingResults(
        const MacAddress& stationMacAddress,
        StationInfo& theStationInfo);

    void SetLessNumberOfStreamButHighThroughputMcsAsTheMaxProbabilityMcs(
        StationInfo& theStationInfo);

    void RecalculateFrameRetryCount(
        const TransmissionParameters& dataFrameTxParameters,
        ModulationAndCodingSchemeEntry& mcsEntry);

    void SetRatesAndSamplingTransmissionIfNecessary(
        const MacAddress& stationMacAddress,
        const unsigned int retrySequenceIndex);

    void SetDefaultFrameRetryLimit(
        const StationInfo& theStationInfo,
        RetryStagesInfo& theRetryStagesInfo);

    void DowngradeMcsForSuddenlyClosedStreamEntry(
        StationInfo& theStationInfo,
        unsigned int downgradeThroughputEntryIndex);


    static unsigned int CalcRetryStageIndex(
        const RetryStagesInfo& theRetryStagesInfo,
        const unsigned int retryTxCount);

    static unsigned int CalcRetryTxCountForStage(
        const RetryStagesInfo& theRetryStagesInfo,
        const unsigned int stageIndex,
        const unsigned int retryTxCount);

};//AdaptiveRateControllerWithMinstrelHT//


inline
void AdaptiveRateControllerWithMinstrelHT::SampleTable::AdvanceSamplingMcsEntryIndex()
{
    currentSampleRow++;

    if (currentSampleRow >= numberOfRateRows) {
        currentSampleRow = 0;
        currentSampleColumn++;

        if (currentSampleColumn >= numberOfSampleColumns) {
            currentSampleColumn = 0;
        }//if//
    }//if//

}//AdvanceSamplingMcsEntryIndex//


inline
void AdaptiveRateControllerWithMinstrelHT::CreateStationInfoFor(const MacAddress& macAddress)
{
    stationInfoForAddr.insert(make_pair(macAddress, StationInfo()));

    StationInfo& theStationInfo = stationInfoForAddr[macAddress];

    theStationInfo.retryStagesInfos.resize(maxRetrySequenceIndex + 1);

    const unsigned int numberOfRateRows =
        static_cast<unsigned int>(modulationAndCodingSchemeEntries.size());

    theStationInfo.sampleTablePtr.reset(
        new SampleTable(
            numberOfRateRows,
            aRandomNumberGenerator));

    theStationInfo.modulationAndCodingSchemeSamplingInfoList.resize(
        modulationAndCodingSchemeEntries.size());

    theStationInfo.remainingSamplingCount =
        static_cast<unsigned int>(theStationInfo.modulationAndCodingSchemeSamplingInfoList.size());

    theStationInfo.remainingInitialSamplingTransmissionCount =
        initialSamplingTransmissionCount;

    theStationInfo.throughputAndMcsIndexListPerStreamGroup.resize(
        modulationAndCodingSchemeEntries.size() /
        modulationAndCodingSchemes.size());

    (*this).UpdateSamplingResults(macAddress, theStationInfo);

}//CreateStationInfoFor//


inline
unsigned int AdaptiveRateControllerWithMinstrelHT::GetStationBandwidthNumChannels(
    const MacAddress& macAddress) const
{
    typedef map<MacAddress, StationInfo>::const_iterator IterType;

    const IterType iter = stationInfoForAddr.find(macAddress);

    assert(iter != stationInfoForAddr.end());

    const StationInfo& theStationInfo = (*iter).second;
    return (theStationInfo.stationBandwidthNumChannels);
}



inline
void AdaptiveRateControllerWithMinstrelHT::AddNewStation(
    const MacAddress& macAddress,
    const unsigned int stationBandwidthNumChannels,
    const bool isHighThroughputStation)
{
    // Reset sampling information for a station

    (*this).stationInfoForAddr.erase(macAddress);
    (*this).CreateStationInfoFor(macAddress);
    (*this).stationInfoForAddr[macAddress].stationBandwidthNumChannels = stationBandwidthNumChannels;
    (*this).stationInfoForAddr[macAddress].isAHighThroughputStation = isHighThroughputStation;

}//AddNewStation//






inline
void AdaptiveRateControllerWithMinstrelHT::NotifyStartingAFrameTransmissionSequence(
    const MacAddress& macAddress,
    const unsigned int retrySequenceIndex)
{
    (*this).SetRatesAndSamplingTransmissionIfNecessary(
        macAddress,
        retrySequenceIndex);

}//NotifyStartingAFrameTransmissionSequence//



inline
void AdaptiveRateControllerWithMinstrelHT::SetDefaultFrameRetryLimit(
    const StationInfo& theStationInfo,
    RetryStagesInfo& theRetryStagesInfo)
{
    const vector<ModulationAndCodingSchemeSamplingInfo>& modulationAndCodingSchemeSamplingInfoList =
        theStationInfo.modulationAndCodingSchemeSamplingInfoList;

    assert(theStationInfo.multirateRetryMcsEntryIndexList.size() ==
           theRetryStagesInfo.isFrameProtectedByRtsForRetryStage.size());

    theRetryStagesInfo.isFrameProtectedByRtsForRetryStage.fill(false);

    for(unsigned int i = 0; i < theStationInfo.multirateRetryMcsEntryIndexList.size(); i++) {
        const McsEntryIndex& mcsEntryIndex =
            theStationInfo.multirateRetryMcsEntryIndexList[i];

        const ModulationAndCodingSchemeSamplingInfo& mcsSamplingInfo =
            modulationAndCodingSchemeSamplingInfoList.at(mcsEntryIndex);

        const ModulationAndCodingSchemeEntry& mcsEntry =
            modulationAndCodingSchemeEntries[mcsEntryIndex];

        // Use RTS/CTS other than 1st stage of multirate retry

        if (i > 0) {
            theRetryStagesInfo.isFrameProtectedByRtsForRetryStage[i] = true;
        }//if//

        unsigned int retryCount;

        if (mcsSamplingInfo.frameSuccessRate < worstSuccessRate) {
            retryCount = 1;
        }
        else if (mcsSamplingInfo.frameSuccessRate < badSuccessRate) {
            retryCount = minRetryCount;
        }
        else {
            if (i > 0) {
                retryCount = mcsEntry.frameRetryCountWithRts;
            }
            else {
                retryCount = mcsEntry.frameRetryCount;
            }//if//

        }//if//

        theRetryStagesInfo.dataFrameRetryLimitForRetryStage[i] = retryCount;

    }//for//

}//SetFrameRetryLimit//


inline
void AdaptiveRateControllerWithMinstrelHT::SetRatesAndSamplingTransmissionIfNecessary(
    const MacAddress& stationMacAddress,
    const unsigned int retrySequenceIndex)
{
    StationInfo& theStationInfo = (*this).stationInfoForAddr[stationMacAddress];
    RetryStagesInfo& theRetryStagesInfo = theStationInfo.retryStagesInfos.at(retrySequenceIndex);

    const vector<ThroughputAndMcsEntryIndex>& throughputAndMcsIndexList =
        theStationInfo.throughputAndMcsIndexList;

    assert(throughputAndMcsIndexList.size() >= 2);

    const McsEntryIndex& bestThroughputMcsEntryIndex =
        throughputAndMcsIndexList[primaryThroughputEntryIndex].mcsEntryIndex;

    const McsEntryIndex& secondaryThroughputMcsEntryIndex =
        throughputAndMcsIndexList[secondaryThroughputEntryIndex].mcsEntryIndex;
    const McsEntryIndex& maxProbabilityMcsEntryIndex = theStationInfo.maxProbabilityMcsEntryIndex;

    const McsEntryIndex lowestMcsEntryIndex = 0;

    theStationInfo.multirateRetryMcsEntryIndexList[0] = bestThroughputMcsEntryIndex;
    theStationInfo.multirateRetryMcsEntryIndexList[1] = secondaryThroughputMcsEntryIndex;
    theStationInfo.multirateRetryMcsEntryIndexList[2] = maxProbabilityMcsEntryIndex;
    theStationInfo.multirateRetryMcsEntryIndexList[3] = lowestMcsEntryIndex;

    // Update frame retry limits after setting rates.

    (*this).SetDefaultFrameRetryLimit(theStationInfo, theRetryStagesInfo);

    // Check sampling necessity

    if ((theStationInfo.remainingInitialSamplingTransmissionCount == 0) &&
        (theStationInfo.samplingWaitCount > 0)) {

        theStationInfo.samplingWaitCount--;
        return;
    }//if//


    SampleTable& sampleTable = *theStationInfo.sampleTablePtr;

    sampleTable.AdvanceSamplingMcsEntryIndex();

    const McsEntryIndex samplingMcsEntryIndex =
        sampleTable.GetCurrentMcsEntryIndex();

    if ((samplingMcsEntryIndex == bestThroughputMcsEntryIndex) ||
        (samplingMcsEntryIndex == secondaryThroughputMcsEntryIndex) ||
        (samplingMcsEntryIndex == maxProbabilityMcsEntryIndex)) {

        return;
    }//if//

    ModulationAndCodingSchemeSamplingInfo& samplingMcsInfo =
        theStationInfo.modulationAndCodingSchemeSamplingInfoList[samplingMcsEntryIndex];

    // Not need to transmit a sampling frame for good success rate entry.

    if (samplingMcsInfo.frameSuccessRate > goodSuccessRate) {
        return;
    }//if//

    const ModulationAndCodingSchemeEntry& samplingMcsEntry =
        modulationAndCodingSchemeEntries[samplingMcsEntryIndex];

    const ModulationAndCodingSchemeEntry& bestThroughputMcsEntry =
        modulationAndCodingSchemeEntries[bestThroughputMcsEntryIndex];

    const ModulationAndCodingSchemeEntry& maxProbabilityMcsEntry =
        modulationAndCodingSchemeEntries[maxProbabilityMcsEntryIndex];

    if (samplingMcsEntry.datarate < bestThroughputMcsEntry.datarate) {

        if ((bestThroughputMcsEntry.numberOfStreams <= samplingMcsEntry.numberOfStreams) ||
            (samplingMcsEntry.datarate <= maxProbabilityMcsEntry.datarate)) {

            if (samplingMcsInfo.skippedSamplingCount < samplingTryingThreshold) {
                return;
            }//if//

            if (theStationInfo.lowRateSampledCount > maxLowRateSamplingCount) {
                return;
            }//if//

            theStationInfo.lowRateSampledCount++;
        }//if//
    }//if//


    // Set sampling transmission for the 1st stage of multirate retry in HT model.
    // Note: In contrast, non-HT Minstrel uses not always the 1st stage but also 2nd stage for a sampling transmission.

    theStationInfo.multirateRetryMcsEntryIndexList[0] = samplingMcsEntryIndex;
    theRetryStagesInfo.dataFrameRetryLimitForRetryStage[0] = samplingRetryCount;

    if (theStationInfo.remainingInitialSamplingTransmissionCount > 0) {
        theStationInfo.remainingInitialSamplingTransmissionCount--;
    }//if//

}//SetRatesAndSamplingTransmissionIfNecessary//



inline
double CalculateExponentiallyWeightedMovingAverage(
    const double lastValue,
    const double newValue,
    const double weight)
{
    return (newValue*(1.0 - weight)) + (lastValue*weight);

}//CalculateExponentiallyWeightedMovingAverage//

inline
void AdaptiveRateControllerWithMinstrelHT::UpdateSamplingResults(
    const MacAddress& stationMacAddress,
    StationInfo& theStationInfo)
{
    vector<ModulationAndCodingSchemeSamplingInfo>& modulationAndCodingSchemeSamplingInfoList =
        theStationInfo.modulationAndCodingSchemeSamplingInfoList;

    vector<ThroughputAndMcsEntryIndex>& throughputAndMcsIndexList =
        theStationInfo.throughputAndMcsIndexList;

    assert(modulationAndCodingSchemeEntries.size() == modulationAndCodingSchemeSamplingInfoList.size());


    throughputAndMcsIndexList.clear();

    theStationInfo.lowRateSampledCount = 0;
    theStationInfo.remainingSamplingCount = static_cast<unsigned int>(modulationAndCodingSchemeEntries.size());

    for(size_t i = 0; i < theStationInfo.throughputAndMcsIndexListPerStreamGroup.size(); i++) {
        theStationInfo.throughputAndMcsIndexListPerStreamGroup[i].clear();
    }//for//

    for(size_t i = 0; i < modulationAndCodingSchemeEntries.size(); i++) {
        const ModulationAndCodingSchemeEntry& mcsEntry = modulationAndCodingSchemeEntries[i];
        const McsEntryIndex mcsEntryIndex = static_cast<McsEntryIndex>(i);

        ModulationAndCodingSchemeSamplingInfo& mcsSamplingInfo = modulationAndCodingSchemeSamplingInfoList[i];


        // Update the transmission success rate by EWMA

        if (mcsSamplingInfo.totalNumberOfTransmittedMpdus > 0) {

            const double currentFrameSuccessRate =
                (static_cast<double>(mcsSamplingInfo.totalTransmissionSucceededCount) /
                 mcsSamplingInfo.totalNumberOfTransmittedMpdus);

            if (mcsSamplingInfo.isFirstTimeSuccessRateCalculation) {

                // Use raw success rate
                mcsSamplingInfo.frameSuccessRate = currentFrameSuccessRate;

            }
            else {

                mcsSamplingInfo.frameSuccessRate =
                    CalculateExponentiallyWeightedMovingAverage(
                        mcsSamplingInfo.frameSuccessRate,
                        currentFrameSuccessRate,
                        exponentiallyWeight);

                // The success rate should be decreased greater than EWMA rate
                // if "currentFrameSuccessRate" is 0 despite the enough transmission count.

                //TBD// if ((mcsSamplingInfo.totalTransmissionSucceededCount == 0) &&
                //TBD//     (mcsSamplingInfo.totalNumberOfTransmittedMpdus > mcsEntry.frameRetryCount)) {
                //TBD//     mcsSamplingInfo.frameSuccessRate = 0.0;
                //TBD// }//if//

            }//if//

            mcsSamplingInfo.isFirstTimeSuccessRateCalculation = false;
            mcsSamplingInfo.totalNumberOfTransmittedMpdus = 0;
            mcsSamplingInfo.totalTransmissionSucceededCount = 0;
            mcsSamplingInfo.skippedSamplingCount = 0;
        }
        else {
            assert(mcsSamplingInfo.totalTransmissionSucceededCount == 0);

            mcsSamplingInfo.skippedSamplingCount++;
        }//if//


        // Update datarate estimation with the transmission success rate.

        if (mcsSamplingInfo.frameSuccessRate < worstSuccessRate) {
            mcsSamplingInfo.estimatedDatarate = 0;
        }
        else {
            const double frameSuccessRateToEstimateThroughput =
                std::min(maxFrameSuccessRateToEstimateThroughput,
                         mcsSamplingInfo.frameSuccessRate);

            mcsSamplingInfo.estimatedDatarate =
                static_cast<DatarateBitsPerSec>(frameSuccessRateToEstimateThroughput*mcsEntry.datarate);
        }//if//


        // Update max throughput entry index for both entire entries and stream grouped entries.

        assert(mcsEntry.numberOfStreams > 0);

        const size_t streamGroupIndex = (i / modulationAndCodingSchemes.size());

        vector<ThroughputAndMcsEntryIndex>& throughputAndMcsIndexListForStreamGroup =
            theStationInfo.throughputAndMcsIndexListPerStreamGroup[streamGroupIndex];

        throughputAndMcsIndexListForStreamGroup.push_back(
            ThroughputAndMcsEntryIndex(
                mcsSamplingInfo.estimatedDatarate, mcsEntryIndex));

        throughputAndMcsIndexList.push_back(
            ThroughputAndMcsEntryIndex(
                mcsSamplingInfo.estimatedDatarate, mcsEntryIndex));
    }//for//


    // Sort MCS index list by estimated throughput.
    // The index will be used for MRR(Multirate Retry):
    // - 1st stage MCS of MRR: "throughputAndMcsIndexList[primaryThroughputEntryIndex]"
    // - 2nd stage MCS of MRR: "throughputAndMcsIndexList[secondaryThroughputEntryIndex]"

    std::sort(throughputAndMcsIndexList.begin(), throughputAndMcsIndexList.end());


    // Stream grouped throughput list will be used for rate downgrading in
    // an unexpected stream lost situation before updating sampling results.

    for(size_t i = 0; i < theStationInfo.throughputAndMcsIndexListPerStreamGroup.size(); i++) {
        vector<ThroughputAndMcsEntryIndex>& throughputAndMcsIndexListForStreamGroup =
            theStationInfo.throughputAndMcsIndexListPerStreamGroup[i];

        std::sort(
            throughputAndMcsIndexListForStreamGroup.begin(),
            throughputAndMcsIndexListForStreamGroup.end());

    }//for//


    (*this).SetLessNumberOfStreamButHighThroughputMcsAsTheMaxProbabilityMcs(theStationInfo);


    if (theStationInfo.totalNumberOfTransmittedFrames > 0) {
        theStationInfo.averageNumberOfAggregatedMpdus =
            static_cast<unsigned int>(
                CalculateExponentiallyWeightedMovingAverage(
                    theStationInfo.averageNumberOfAggregatedMpdus,
                    (static_cast<double>(theStationInfo.totalNumberOfTransmittedMpdus) /
                     theStationInfo.totalNumberOfTransmittedFrames),
                    exponentiallyWeight));

        theStationInfo.totalNumberOfTransmittedMpdus = 0;
        theStationInfo.totalNumberOfTransmittedFrames = 0;
    }//if//

    (*this).OutputTraceForTxDatarateUpdate(stationMacAddress);

}//UpdateSamplingResults//



inline
void AdaptiveRateControllerWithMinstrelHT::SetLessNumberOfStreamButHighThroughputMcsAsTheMaxProbabilityMcs(
    StationInfo& theStationInfo)
{
    const vector<ModulationAndCodingSchemeSamplingInfo>& modulationAndCodingSchemeSamplingInfoList =
        theStationInfo.modulationAndCodingSchemeSamplingInfoList;

    assert(!modulationAndCodingSchemes.empty());

    const size_t numberOfStreamGroups =
        (modulationAndCodingSchemeEntries.size() / modulationAndCodingSchemes.size());

    vector<McsEntryIndex> maxProbabilityMcsEntryIndexPerStreamGroup;
    McsEntryIndex maxProbabilityMcsEntryIndex = 0/*= lowest MCS entry index*/;

    for(size_t i = 0; i < numberOfStreamGroups; i++) {
        const size_t startMcsIndexForThisStreamGroup = (i*modulationAndCodingSchemes.size());

        McsEntryIndex maxProbabilityMcsEntryIndexForStreamGroup = 0/*= lowest MCS entry index*/;

        for(size_t j = 0; j < modulationAndCodingSchemes.size(); j++) {
            const McsEntryIndex mcsEntryIndex =
                static_cast<McsEntryIndex>(startMcsIndexForThisStreamGroup + j);

            const ModulationAndCodingSchemeEntry& mcsEntry =
                modulationAndCodingSchemeEntries.at(mcsEntryIndex);

            const ModulationAndCodingSchemeSamplingInfo& mcsSamplingInfo =
                modulationAndCodingSchemeSamplingInfoList.at(mcsEntryIndex);

            if (mcsSamplingInfo.frameSuccessRate >= goodSuccessRate) {

                if (mcsSamplingInfo.estimatedDatarate >=
                    modulationAndCodingSchemeSamplingInfoList[maxProbabilityMcsEntryIndex].estimatedDatarate) {

                    maxProbabilityMcsEntryIndex = mcsEntryIndex;

                }//if//

                if (mcsSamplingInfo.estimatedDatarate >=
                    modulationAndCodingSchemeSamplingInfoList[maxProbabilityMcsEntryIndexForStreamGroup].estimatedDatarate) {

                    maxProbabilityMcsEntryIndexForStreamGroup = mcsEntryIndex;

                }//if//
            }
            else {
                if (mcsSamplingInfo.frameSuccessRate >=
                    modulationAndCodingSchemeSamplingInfoList[maxProbabilityMcsEntryIndex].frameSuccessRate) {

                    maxProbabilityMcsEntryIndex = mcsEntryIndex;
                }//if//
                if (mcsSamplingInfo.frameSuccessRate >=
                    modulationAndCodingSchemeSamplingInfoList[maxProbabilityMcsEntryIndexForStreamGroup].frameSuccessRate) {

                    maxProbabilityMcsEntryIndexForStreamGroup = mcsEntryIndex;

                }//if//
            }//if//
        }//for//

        maxProbabilityMcsEntryIndexPerStreamGroup.push_back(maxProbabilityMcsEntryIndexForStreamGroup);

    }//for//

    const vector<ThroughputAndMcsEntryIndex>& throughputAndMcsIndexList =
        theStationInfo.throughputAndMcsIndexList;

    assert(!throughputAndMcsIndexList.empty());

    const McsEntryIndex& bestThroughputMcsEntryIndex =
        throughputAndMcsIndexList[primaryThroughputEntryIndex].mcsEntryIndex;

    const ModulationAndCodingSchemeEntry& bestThroughputMcsEntry =
        modulationAndCodingSchemeEntries[bestThroughputMcsEntryIndex];

    DatarateBitsPerSec maxDatarateByReducingNumberOfStreams = 0;
    DatarateBitsPerSec maxDatarateByReducingBondedChannelCount = 0;
    uint8_t minNumberOfStreams = bestThroughputMcsEntry.numberOfStreams;
    uint8_t minNumberOfBondedChannels = bestThroughputMcsEntry.numberOfBondedChannels;
    bool foundALessNumberOfStreamMcs = false;
    bool foundALessNumberOfBondedChannelMcs = false;

    // First, check number of streams.

    for(size_t i = 0; i < maxProbabilityMcsEntryIndexPerStreamGroup.size(); i++) {
        const McsEntryIndex& mcsEntryIndex = maxProbabilityMcsEntryIndexPerStreamGroup[i];
        const ModulationAndCodingSchemeEntry& mcsEntry = modulationAndCodingSchemeEntries[mcsEntryIndex];

        const ModulationAndCodingSchemeSamplingInfo& mcsSamplingInfo =
            modulationAndCodingSchemeSamplingInfoList[mcsEntryIndex];

        if ((mcsSamplingInfo.estimatedDatarate >= maxDatarateByReducingNumberOfStreams) &&
            (mcsEntry.numberOfStreams < minNumberOfStreams)) {

            maxDatarateByReducingNumberOfStreams = mcsSamplingInfo.estimatedDatarate;
            minNumberOfStreams = mcsEntry.numberOfStreams;
            theStationInfo.maxProbabilityMcsEntryIndex = mcsEntryIndex;

            foundALessNumberOfStreamMcs = true;
        }//if//

    }//for//

    if (!foundALessNumberOfStreamMcs) {

        // Second, check number of bounded channels.

        for(size_t i = 0; i < maxProbabilityMcsEntryIndexPerStreamGroup.size(); i++) {
            const McsEntryIndex& mcsEntryIndex = maxProbabilityMcsEntryIndexPerStreamGroup[i];
            const ModulationAndCodingSchemeEntry& mcsEntry = modulationAndCodingSchemeEntries[mcsEntryIndex];
            const ModulationAndCodingSchemeSamplingInfo& mcsSamplingInfo =
                modulationAndCodingSchemeSamplingInfoList[mcsEntryIndex];

            if ((mcsSamplingInfo.estimatedDatarate >= maxDatarateByReducingBondedChannelCount) &&
                (mcsEntry.numberOfBondedChannels < minNumberOfBondedChannels)) {

                maxDatarateByReducingBondedChannelCount = mcsSamplingInfo.estimatedDatarate;
                minNumberOfBondedChannels = mcsEntry.numberOfBondedChannels;
                theStationInfo.maxProbabilityMcsEntryIndex = mcsEntryIndex;

                foundALessNumberOfBondedChannelMcs = true;
            }//if//
        }//for//


        if (!foundALessNumberOfBondedChannelMcs) {

            // Last case
            theStationInfo.maxProbabilityMcsEntryIndex = maxProbabilityMcsEntryIndex;

        }//if//

    }//if//

}//SetLessStreamButHighThroughputMcsAsTheMaxProbabilityMcs//


inline
unsigned int AdaptiveRateControllerWithMinstrelHT::CalcRetryStageIndex(
    const RetryStagesInfo& theRetryStagesInfo,
    const unsigned int retryTxCount)
{
    unsigned int txCount = 0;
    for(unsigned int i = 0; (i < theRetryStagesInfo.dataFrameRetryLimitForRetryStage.size()); i++) {

        txCount += theRetryStagesInfo.dataFrameRetryLimitForRetryStage[i];

        if (retryTxCount <= txCount) {
            return i;
        }//if//
    }//for//

    assert(false && "Too many retries"); abort(); return 0;

}//CalcRetryStageIndex//


inline
unsigned int AdaptiveRateControllerWithMinstrelHT::CalcRetryTxCountForStage(
    const RetryStagesInfo& theRetryStagesInfo,
    const unsigned int stageIndex,
    const unsigned int retryTxCount)
{
    using std::min;

    assert(stageIndex < theRetryStagesInfo.dataFrameRetryLimitForRetryStage.size());

    unsigned int previousStagesTxCount = 0;
    for(unsigned int i = 0; (i < stageIndex); i++) {

        previousStagesTxCount += theRetryStagesInfo.dataFrameRetryLimitForRetryStage[i];

    }//for//

    if (retryTxCount <= previousStagesTxCount) {
        return 0;
    }//if//

    return (
        min(
            theRetryStagesInfo.dataFrameRetryLimitForRetryStage[stageIndex],
            (retryTxCount - previousStagesTxCount)));

}//CalcRetryTxCountForStage//



inline
void AdaptiveRateControllerWithMinstrelHT::GetDataRateInfoForDataFrameToStation(
    const MacAddress& macAddress,
    const unsigned int retryTxCount,
    const unsigned int retrySequenceIndex,
    TransmissionParameters& txParameters,
    bool& useRtsEvenIfDisabled,
    bool& overridesStandardRetryTxLimits,
    unsigned int& newRetryTxLimit) const
{
    useRtsEvenIfDisabled = false;
    overridesStandardRetryTxLimits = false;

    ModulationAndCodingSchemesType modulationAndCodingScheme;

    uint8_t bandwidthNumberChannels = 1;
    uint8_t numberSpatialStreams = 1;
    bool isHighThroughputFrame = false;

    if (macAddress.IsABroadcastOrAMulticastAddress()) {
        modulationAndCodingScheme = modAndCodingForBroadcast;
    }
    else {
        typedef map<MacAddress, StationInfo>::const_iterator IterType;

        IterType iter = stationInfoForAddr.find(macAddress);

        assert(iter != stationInfoForAddr.end());

        const StationInfo& theStationInfo = (*iter).second;
        const RetryStagesInfo& theRetryStagesInfo = theStationInfo.retryStagesInfos.at(retrySequenceIndex);

        const unsigned int currentRetryStageIndex = CalcRetryStageIndex(theRetryStagesInfo, retryTxCount);

        const McsEntryIndex mcsEntryIndex =
            theStationInfo.multirateRetryMcsEntryIndexList[currentRetryStageIndex];

        const ModulationAndCodingSchemeEntry& mcsEntry = modulationAndCodingSchemeEntries[mcsEntryIndex];

        modulationAndCodingScheme = mcsEntry.modulationAndCodingScheme;
        bandwidthNumberChannels = mcsEntry.numberOfBondedChannels;
        numberSpatialStreams = mcsEntry.numberOfStreams;
        isHighThroughputFrame = theStationInfo.isAHighThroughputStation;

        overridesStandardRetryTxLimits = true;
        newRetryTxLimit =
            std::accumulate(
                theRetryStagesInfo.dataFrameRetryLimitForRetryStage.begin(),
                theRetryStagesInfo.dataFrameRetryLimitForRetryStage.end(),
                0);
    }//if//

    txParameters.modulationAndCodingScheme = modulationAndCodingScheme;
    txParameters.bandwidthNumberChannels = bandwidthNumberChannels;
    txParameters.numberSpatialStreams = numberSpatialStreams;
    txParameters.isHighThroughputFrame = isHighThroughputFrame;



}//GetDataRateInfoForDataFrameToStation//



inline
void AdaptiveRateControllerWithMinstrelHT::GetDataRateInfoForAckFrameToStation(
    const MacAddress& macAddress,
    const TransmissionParameters& receivedFrameTxParameters,
    TransmissionParameters& ackTxParameters) const
{
    ackDatarateController.GetDataRateInfoForAckFrame(
        macAddress, receivedFrameTxParameters, ackTxParameters);

}//GetDataRateInfoForAckFrameToStation//



inline
void AdaptiveRateControllerWithMinstrelHT::GetDataRateInfoForAckFrameFromStation(
    const MacAddress& macAddress,
    const TransmissionParameters& receivedFrameTxParameters,
    TransmissionParameters& ackTxParameters) const
{
    // For now mac address specific ack rates not used.

    ackDatarateController.GetDataRateInfoForAckFrame(
        macAddress, receivedFrameTxParameters, ackTxParameters);

}//GetDataRateInfoForAckFrameFromStation//


inline
void AdaptiveRateControllerWithMinstrelHT::GetDataRateInfoForManagementFrameToStation(
    const MacAddress& macAddress,
    TransmissionParameters& txParameters) const
{
    txParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    txParameters.bandwidthNumberChannels = 1;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = false;

}//GetDataRateInfoForManagementFrameToStation//


inline
void AdaptiveRateControllerWithMinstrelHT::GetDataRateInfoForBeaconFrame(
    TransmissionParameters& txParameters) const
{
    txParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    txParameters.bandwidthNumberChannels = 1;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = false;

}//GetDataRateInfoForBeaconFrame//



inline
void AdaptiveRateControllerWithMinstrelHT::NotifyFinishingAFrameTransmissionSequence(
    const MacAddress& macAddress,
    const unsigned int numberTransmittedMpdus,
    const unsigned int numberSuccessfulMpdus,
    const unsigned int finalDataFrameRetryTxCount,
    const unsigned int retrySequenceIndex)
{
    // Ignore ack frames for management frame before transmitting data

    if (stationInfoForAddr.find(macAddress) == stationInfoForAddr.end()) {
        return;
    }//if//

    StationInfo& theStationInfo = stationInfoForAddr[macAddress];
    RetryStagesInfo& theRetryStagesInfo = theStationInfo.retryStagesInfos.at(retrySequenceIndex);

    vector<ModulationAndCodingSchemeSamplingInfo>& modulationAndCodingSchemeSamplingInfoList =
        theStationInfo.modulationAndCodingSchemeSamplingInfoList;

    const unsigned int lastRetryStageIndex = CalcRetryStageIndex(theRetryStagesInfo, finalDataFrameRetryTxCount);

    assert(lastRetryStageIndex < theStationInfo.multirateRetryMcsEntryIndexList.size());

    // Stat retry and succeeded transmission counts.

    for(unsigned int i = 0; i <= lastRetryStageIndex; i++) {
        const McsEntryIndex& mcsEntryIndex = theStationInfo.multirateRetryMcsEntryIndexList[i];

        // Wrong for A-MPDUs.

        modulationAndCodingSchemeSamplingInfoList[mcsEntryIndex].totalNumberOfTransmittedMpdus +=
            (CalcRetryTxCountForStage(theRetryStagesInfo, i, finalDataFrameRetryTxCount) *
             numberTransmittedMpdus);

    }//for//

    const McsEntryIndex& lastMcsEntryIndex =
        theStationInfo.multirateRetryMcsEntryIndexList[lastRetryStageIndex];

    ModulationAndCodingSchemeSamplingInfo& lastModulationAndCodingSchemeSamplingInfo =
        modulationAndCodingSchemeSamplingInfoList[lastMcsEntryIndex];

    lastModulationAndCodingSchemeSamplingInfo.totalTransmissionSucceededCount += numberSuccessfulMpdus;

    theStationInfo.totalNumberOfTransmittedFrames++;
    theStationInfo.totalNumberOfTransmittedMpdus += numberTransmittedMpdus;

    // Set next smpling transmission if the last transmission is a sampling transmission.

    if ((theStationInfo.remainingInitialSamplingTransmissionCount == 0) &&
        (theStationInfo.samplingWaitCount == 0) &&
        (theStationInfo.remainingSamplingCount > 0)) {

        theStationInfo.samplingWaitCount =
            samplingTransmissionInterval + theStationInfo.averageNumberOfAggregatedMpdus*2;

        theStationInfo.remainingSamplingCount--;
    }//if//


    // Downgrade rate for suddenly closed streams

    (*this).DowngradeMcsForSuddenlyClosedStreamEntry(theStationInfo, primaryThroughputEntryIndex);
    (*this).DowngradeMcsForSuddenlyClosedStreamEntry(theStationInfo, secondaryThroughputEntryIndex);

    // Update rate information for next time transmission.

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    if (currentTime >= (theStationInfo.lastSamplingResultUpdatedTime + samplingResultUpdateInterval)) {

        (*this).UpdateSamplingResults(macAddress, theStationInfo);

        theStationInfo.lastSamplingResultUpdatedTime = currentTime;
    }//if//

}//NotifyFinishingAFrameTransmissionSequence//




inline
void AdaptiveRateControllerWithMinstrelHT::DowngradeMcsForSuddenlyClosedStreamEntry(
    StationInfo& theStationInfo,
    unsigned int downgradeThroughputEntryIndex)
{
    vector<ThroughputAndMcsEntryIndex>& throughputAndMcsIndexList =
        theStationInfo.throughputAndMcsIndexList;

    const vector<ModulationAndCodingSchemeSamplingInfo>& modulationAndCodingSchemeSamplingInfoList =
        theStationInfo.modulationAndCodingSchemeSamplingInfoList;

    const McsEntryIndex& mcsEntryIndex =
        throughputAndMcsIndexList[downgradeThroughputEntryIndex].mcsEntryIndex;

    const ModulationAndCodingSchemeSamplingInfo& throughputMcsSamplingInfo =
        modulationAndCodingSchemeSamplingInfoList[mcsEntryIndex];

    if ((throughputMcsSamplingInfo.totalNumberOfTransmittedMpdus > enoughNumberOfTransmittedMpdusToSample) &&
        (throughputMcsSamplingInfo.GetTransmissionSuccessRate() < badSuccessRate)) {

        size_t streamGroupIndex = (mcsEntryIndex / modulationAndCodingSchemes.size());

        while (streamGroupIndex > 0) {

            // decrease stream group
            streamGroupIndex--;

            const vector<ThroughputAndMcsEntryIndex>& throughputAndMcsIndexListForStreamGroup =
                theStationInfo.throughputAndMcsIndexListPerStreamGroup[streamGroupIndex];

            const McsEntryIndex& downMcsEntryIndex =
                throughputAndMcsIndexListForStreamGroup[downgradeThroughputEntryIndex].mcsEntryIndex;

            if (modulationAndCodingSchemeEntries[downMcsEntryIndex].numberOfStreams <=
                modulationAndCodingSchemeEntries[downgradeThroughputEntryIndex].numberOfStreams) {

                throughputAndMcsIndexList[downgradeThroughputEntryIndex] =
                    throughputAndMcsIndexListForStreamGroup[downgradeThroughputEntryIndex];

            }//if//
        }//while//
    }//if//

}//DowngradeMcsForSuddenlyClosedStreamEntry//


//--------------------------------------------------------------------------------------------------

inline
shared_ptr<AdaptiveRateController> CreateAdaptiveRateController(
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId,
    const unsigned int interfaceIndex,
    const unsigned int baseChannelBandwidthMhz,
    const unsigned int maxChannelBandwidthMhz,
    const bool highThroughputModeIsOn,
    const shared_ptr<Dot11MacDurationCalculationInterface>& macDurationCalculationInterfacePtr,
    const RandomNumberGeneratorSeed& nodeSeed)
{
    if (!theParameterDatabaseReader.ParameterExists(
            "dot11-adaptive-rate-control-type", theNodeId, theInterfaceId)) {

        return (shared_ptr<AdaptiveRateController>(
            new StaticRateController(
                simulationEngineInterfacePtr,
                theParameterDatabaseReader,
                theNodeId,
                theInterfaceId,
                baseChannelBandwidthMhz,
                maxChannelBandwidthMhz,
                highThroughputModeIsOn,
                macDurationCalculationInterfacePtr)));
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
                    baseChannelBandwidthMhz,
                    maxChannelBandwidthMhz,
                    highThroughputModeIsOn,
                    macDurationCalculationInterfacePtr)));
        }
        else if ((adaptiveRateControlType == "static") ||
                 (adaptiveRateControlType == "fixed"/*will be deleted*/)) {

            return (shared_ptr<AdaptiveRateController>(
                new StaticRateController(
                    simulationEngineInterfacePtr,
                    theParameterDatabaseReader,
                    theNodeId,
                    theInterfaceId,
                    baseChannelBandwidthMhz,
                    maxChannelBandwidthMhz,
                    highThroughputModeIsOn,
                    macDurationCalculationInterfacePtr)));
        }
        else if (adaptiveRateControlType == "minstrel-ht") {

            return (shared_ptr<AdaptiveRateController>(
                new AdaptiveRateControllerWithMinstrelHT(
                    simulationEngineInterfacePtr,
                    theParameterDatabaseReader,
                    theNodeId,
                    theInterfaceId,
                    interfaceIndex,
                    baseChannelBandwidthMhz,
                    maxChannelBandwidthMhz,
                    highThroughputModeIsOn,
                    macDurationCalculationInterfacePtr,
                    nodeSeed)));
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
