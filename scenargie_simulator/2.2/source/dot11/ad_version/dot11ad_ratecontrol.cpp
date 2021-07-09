// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "dot11ad_ratecontrol.h"
#include "dot11ad_mac.h"

namespace Dot11ad {
const string AdaptiveRateController::modelName = "Dot11Mac";

const unsigned int AdaptiveRateControllerWithMinstrelHT::primaryThroughputEntryIndex = 0;
const unsigned int AdaptiveRateControllerWithMinstrelHT::secondaryThroughputEntryIndex = 1;

template <typename T>
static inline
uint8_t ConvertToUInt8(const T& aValue)
{
    assert(aValue >= 0);
    assert(aValue <= static_cast<T>(255));

    return static_cast<uint8_t>(aValue);
}//ConvertToUInt8//

template <typename T>
static inline
uint16_t ConvertToUInt16(const T& aValue)
{
    assert(aValue >= 0);
    assert(aValue <= static_cast<T>(65535));

    return static_cast<uint16_t>(aValue);
}//ConvertToUInt16//



void AdaptiveRateController::OutputTraceForTxDatarateUpdate(const MacAddress& destMacAddress)
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {

        TransmissionParameters txParameters;

        (*this).GetDataRateInfoForDataFrameToStation(
            destMacAddress,
            0,  // Use AC 0.
            0,  // Use First Tx rate.
            txParameters);

        const DatarateBitsPerSec theDatarateBitsPerSec =
            macDurationCalculationInterfacePtr->CalcDatarateBitsPerSecond(txParameters);

        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacTxDatarateUpdateTraceRecord traceData;

            traceData.txDatarateBps = theDatarateBitsPerSec;
            traceData.destNodeId = destMacAddress.ExtractNodeId();

            assert(sizeof(traceData) == MAC_TX_DATARATE_UPDATE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, theInterfaceId, "TxRateUpdate", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "TxRate= " << theDatarateBitsPerSec;

            msgStream << " Dest= " << destMacAddress.ExtractNodeId();

            simEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "TxRateUpdate", msgStream.str());

        }//if//

    }//if//

}//OutputTraceForTxDatarateUpdate//



AdaptiveRateControllerWithMinstrelHT::AdaptiveRateControllerWithMinstrelHT(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& initInterfaceId,
    const unsigned int initInterfaceIndex,
    const unsigned int initBaseChannelBandwidthMhz,
    const unsigned int initMaxChannelBandwidthMhz,
    const bool initHighThroughputModeIsOn,
    const shared_ptr<Dot11MacDurationCalculationInterface>& initMacDurationCalculationInterfacePtr,
    const RandomNumberGeneratorSeed& initNodeSeed)
    :
    AdaptiveRateController(
        initSimulationEngineInterfacePtr,
        initInterfaceId,
        initMacDurationCalculationInterfacePtr),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(initNodeSeed, initInterfaceIndex)),
    baseChannelBandwidthMhz(initBaseChannelBandwidthMhz),
    maxChannelBandwidthMhz(initMaxChannelBandwidthMhz),
    highThroughputModeIsOn(initHighThroughputModeIsOn),
    typicalTransmissionUnitLengthBytes(1200),
    maxRetryCount(7),
    minRetryCount(2),
    samplingRetryCount(1),
    exponentiallyWeight(0.75),
    goodSuccessRate(0.95),
    badSuccessRate(0.20),
    worstSuccessRate(0.10),
    maxFrameSuccessRateToEstimateThroughput(0.9),
    samplingResultUpdateInterval(100*MILLI_SECOND),
    maxTransmissionDurationForAMultirateRetryStage(6*MILLI_SECOND),
    samplingTryingThreshold(20),
    maxLowRateSamplingCount(2),
    enoughNumberOfTransmittedMpdusToSample(30),
    samplingTransmissionInterval(16),
    initialSamplingTransmissionCount(4),
    ackDatarateController(theParameterDatabaseReader, theNodeId, theInterfaceId),
    maxRetrySequenceIndex(MaxNumberAccessCategories - 1)
{
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

    //unicast
    const string modAndCodingListString =
        theParameterDatabaseReader.ReadString(parameterNamePrefix + "modulation-and-coding-list", theNodeId, theInterfaceId);

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

    const unsigned int maxNumberOfBondedChannels =
        maxChannelBandwidthMhz/baseChannelBandwidthMhz;

    unsigned int maxNumMimoSpatialStreams = MaxNumMimoSpatialStreams;

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-max-number-of-spatial-streams"), theNodeId, theInterfaceId)) {

        maxNumMimoSpatialStreams =
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-max-number-of-spatial-streams"), theNodeId, theInterfaceId);

        if (maxNumMimoSpatialStreams > MaxNumMimoSpatialStreams) {
            cerr << "Error in \"dot11-minstrel-ht-max-number-of-spatial-streams\". Max number of Spatial Stream is "<< MaxNumMimoSpatialStreams << "." << endl;
            exit(1);
        }//if//
    }//if//

    for(unsigned int numberOfBondedChannels = 1; numberOfBondedChannels <= maxNumberOfBondedChannels; numberOfBondedChannels*= 2) {

        for(unsigned int numberOfStreams = 1; numberOfStreams <= maxNumMimoSpatialStreams; numberOfStreams++) {

            for(unsigned int modulationAndCodingSchemeIndex = 0; modulationAndCodingSchemeIndex < modulationAndCodingSchemes.size(); modulationAndCodingSchemeIndex++) {

                const ModulationAndCodingSchemesType& modulationAndCodingScheme =
                    modulationAndCodingSchemes[modulationAndCodingSchemeIndex];


                TransmissionParameters txParameters;

                txParameters.bandwidthNumberChannels = numberOfBondedChannels;
                txParameters.modulationAndCodingScheme = modulationAndCodingScheme;
                txParameters.numberSpatialStreams = numberOfStreams;

                if (numberOfBondedChannels == 1) {
                    txParameters.isHighThroughputFrame = highThroughputModeIsOn;
                }
                else {
                    assert(numberOfBondedChannels >= 2);
                    txParameters.isHighThroughputFrame = true;
                }//if//

                modulationAndCodingSchemeEntries.push_back(
                    ModulationAndCodingSchemeEntry(
                        modulationAndCodingScheme,
                        ConvertToUInt8(numberOfStreams),
                        ConvertToUInt8(numberOfBondedChannels),
                        macDurationCalculationInterfacePtr->CalcDatarateBitsPerSecond(txParameters)));

                const ModulationAndCodingSchemeEntry& mcsEntry = modulationAndCodingSchemeEntries.back();

                (*this).RecalculateFrameRetryCount(
                    txParameters,
                    modulationAndCodingSchemeEntries.back());

            }//for//
        }//for//
    }//for//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-typical-transmission-unit-length-bytes"), theNodeId, theInterfaceId)) {
        typicalTransmissionUnitLengthBytes =
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-typical-transmission-unit-length-bytes"), theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-max-retry-count"), theNodeId, theInterfaceId)) {
        maxRetryCount = ConvertToUInt8(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-max-retry-count"), theNodeId, theInterfaceId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-min-retry-count"), theNodeId, theInterfaceId)) {

        minRetryCount = ConvertToUInt8(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-min-retry-count"), theNodeId, theInterfaceId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-sampling-retry-count"), theNodeId, theInterfaceId)) {

        samplingRetryCount = ConvertToUInt8(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-sampling-retry-count"), theNodeId, theInterfaceId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-moving-average-exponentially-weight"), theNodeId, theInterfaceId)) {

        exponentiallyWeight =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "minstrel-ht-moving-average-exponentially-weight"), theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-good-success-rate"), theNodeId, theInterfaceId)) {

        goodSuccessRate =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "minstrel-ht-good-success-rate"), theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-bad-success-rate"), theNodeId, theInterfaceId)) {

        badSuccessRate =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "minstrel-ht-bad-success-rate"), theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-worst-success-rate"), theNodeId, theInterfaceId)) {

        worstSuccessRate =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "minstrel-ht-worst-success-rate"), theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-max-success-rate-to-estimate-throughput"), theNodeId, theInterfaceId)) {

        maxFrameSuccessRateToEstimateThroughput =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "minstrel-ht-max-success-rate-to-estimate-throughput"), theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-sampling-result-update-interval"), theNodeId, theInterfaceId)) {

        samplingResultUpdateInterval =
            theParameterDatabaseReader.ReadTime(
                (parameterNamePrefix + "minstrel-ht-sampling-result-update-interval"), theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-max-transmission-duration-for-a-multirate-retry-stage"), theNodeId, theInterfaceId)) {

        maxTransmissionDurationForAMultirateRetryStage =
            theParameterDatabaseReader.ReadTime(
                (parameterNamePrefix + "minstrel-ht-max-transmission-duration-for-a-multirate-retry-stage"), theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-low-rate-sampling-trying-threshold"), theNodeId, theInterfaceId)) {

        samplingTryingThreshold = ConvertToUInt16(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-low-rate-sampling-trying-threshold"), theNodeId, theInterfaceId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-max-low-rate-sampling-count"), theNodeId, theInterfaceId)) {

        maxLowRateSamplingCount = ConvertToUInt16(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-max-low-rate-sampling-count"), theNodeId, theInterfaceId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-enough-number-of-transmitted-mpdus-to-sample"), theNodeId, theInterfaceId)) {

        enoughNumberOfTransmittedMpdusToSample = ConvertToUInt16(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-enough-number-of-transmitted-mpdus-to-sample"), theNodeId, theInterfaceId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-sampling-transmission-interval-count"), theNodeId, theInterfaceId)) {

        samplingTransmissionInterval = ConvertToUInt16(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-sampling-transmission-interval-count"), theNodeId, theInterfaceId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-initial-sampling-transmission-count"), theNodeId, theInterfaceId)) {

        initialSamplingTransmissionCount = ConvertToUInt16(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-initial-sampling-transmission-count"), theNodeId, theInterfaceId));
    }//if//

}//AdaptiveRateControllerWithMinstrelHT//



void AdaptiveRateControllerWithMinstrelHT::RecalculateFrameRetryCount(
    const TransmissionParameters& dataFrameTxParameters,
    ModulationAndCodingSchemeEntry& mcsEntry)
{
    TransmissionParameters managementFrameTxParameters;

    managementFrameTxParameters.bandwidthNumberChannels = 1;
    managementFrameTxParameters.modulationAndCodingScheme = lowestModulationAndCoding;
    managementFrameTxParameters.numberSpatialStreams =1;
    managementFrameTxParameters.isHighThroughputFrame = highThroughputModeIsOn;

    const SimTime dataFrameTransmissionDuration =
        macDurationCalculationInterfacePtr->CalcMaxTxDurationForUnicastDataWithoutRts(
            typicalTransmissionUnitLengthBytes,
            dataFrameTxParameters,
            managementFrameTxParameters);

    const SimTime dataFrameTransmissionDurationWithRts =
        macDurationCalculationInterfacePtr->CalcMaxTxDurationForUnicastDataWithRts(
            typicalTransmissionUnitLengthBytes,
            dataFrameTxParameters,
            managementFrameTxParameters);

    assert(dataFrameTransmissionDuration < dataFrameTransmissionDurationWithRts);

    SimTime totalDataFrameTransmissionDuration = ZERO_TIME;
    SimTime totalDataFrameTransmissionDurationWithRts = ZERO_TIME;

    mcsEntry.frameRetryCount = 0;
    mcsEntry.frameRetryCountWithRts = 0;

    while (mcsEntry.frameRetryCount < maxRetryCount) {

        const SimTime averageBackoffDuration =
            macDurationCalculationInterfacePtr->CalcContentionWindowMinBackoffDuration(mcsEntry.frameRetryCount) / 2;

        totalDataFrameTransmissionDuration +=
            (averageBackoffDuration + dataFrameTransmissionDuration);

        totalDataFrameTransmissionDurationWithRts +=
            (averageBackoffDuration + dataFrameTransmissionDurationWithRts);

        if ((mcsEntry.frameRetryCount >= minRetryCount) &&
            (totalDataFrameTransmissionDuration >= maxTransmissionDurationForAMultirateRetryStage)) {
            break;
        }//if//

        mcsEntry.frameRetryCount++;

        if (totalDataFrameTransmissionDurationWithRts < maxTransmissionDurationForAMultirateRetryStage) {
            mcsEntry.frameRetryCountWithRts++;
        }//if//

    }//while//

}//RecalculateFrameRetryCount//


}//namespace
