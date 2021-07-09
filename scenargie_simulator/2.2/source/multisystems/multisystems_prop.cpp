// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "sim.h"

void SimNode::SetupMovingObject(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<GisSubsystem>& gisSubsystemPtr)
{
    string stationTypeString;

    if (stationType == BS) {
        stationTypeString = "bs";
    }
    else if (stationType == UE) {
        stationTypeString = "ue";
    }
    else {
        stationTypeString = "";
    }//if//

    gisSubsystemPtr->EnableMovingObjectIfNecessary(
        theParameterDatabaseReader,
        theNodeId,
        simulationEngineInterfacePtr->CurrentTime(),
        nodeMobilityModelPtr,
        stationTypeString);

}//SetupMovingObject//

void SimNode::DisconnectPropInterfaces()
{
    for(size_t i = 0; i < interfaces.size(); i++) {
        vector<shared_ptr<AntennaType> >& antennaPtrs = interfaces[i].antennaPtrs;

        for(size_t j = 0; j < antennaPtrs.size(); j++) {
            AntennaType& antenna = (*antennaPtrs[j]);

            if (antenna.alohaPropagationModelInterfacePtr != nullptr) {
                antenna.alohaPropagationModelInterfacePtr->DisconnectThisInterface();
            }//if//
            if (antenna.dot11PropagationModelInterfacePtr != nullptr) {
                antenna.dot11PropagationModelInterfacePtr->DisconnectThisInterface();
            }//if//
            if (antenna.dot11adPropagationModelInterfacePtr != nullptr) {
                antenna.dot11adPropagationModelInterfacePtr->DisconnectThisInterface();
            }//if//
            if (antenna.dot11ahPropagationModelInterfacePtr != nullptr) {
                antenna.dot11ahPropagationModelInterfacePtr->DisconnectThisInterface();
            }//if//
            if (antenna.t109PropagationModelInterfacePtr != nullptr) {
                antenna.t109PropagationModelInterfacePtr->DisconnectThisInterface();
            }//if//
            if (antenna.dot15PropagationModelInterfacePtr != nullptr) {
                antenna.dot15PropagationModelInterfacePtr->DisconnectThisInterface();
            }//if//
            if (antenna.blePropagationModelInterfacePtr != nullptr) {
                antenna.blePropagationModelInterfacePtr->DisconnectThisInterface();
            }//if//
            if (antenna.lteDownlinkPropagationModelInterfacePtr != nullptr) {
                antenna.lteDownlinkPropagationModelInterfacePtr->DisconnectThisInterface();
            }//if//
            if (antenna.lteUplinkPropagationModelInterfacePtr != nullptr) {
                antenna.lteUplinkPropagationModelInterfacePtr->DisconnectThisInterface();
            }//if//
        }//for//
    }//for//

    interfaces.clear();

}//DisconnectPropInterfaces//


void SimNode::CalculatePathlossToLocation(
    const PropagationInformationType& informationType,
    const unsigned int antennaNumber,
    const double& positionXMeters,
    const double& positionYMeters,
    const double& positionZMeters,
    PropagationStatisticsType& propagationStatistics) const
{
    AntennaType& antenna = *(*this).GetAntennaPtr(antennaNumber);

    const SimTime currentTime =
        simulationEngineInterfacePtr->CurrentTime();

    if (stationType == BS) {
        antenna.lteDownlinkPropagationModelInterfacePtr->CalculatePathlossToLocation(
            informationType,
            positionXMeters,
            positionYMeters,
            positionZMeters,
            propagationStatistics);
    }
    else if (stationType == UE) {
        antenna.lteUplinkPropagationModelInterfacePtr->CalculatePathlossToLocation(
            informationType,
            positionXMeters,
            positionYMeters,
            positionZMeters,
            propagationStatistics);
    }
    else if (antenna.dot11PropagationModelInterfacePtr != nullptr) {
        antenna.dot11PropagationModelInterfacePtr->CalculatePathlossToLocation(
            informationType,
            positionXMeters,
            positionYMeters,
            positionZMeters,
            propagationStatistics);
    }
    else if (antenna.dot11adPropagationModelInterfacePtr != nullptr) {
        antenna.dot11adPropagationModelInterfacePtr->CalculatePathlossToLocation(
            informationType,
            positionXMeters,
            positionYMeters,
            positionZMeters,
            propagationStatistics);
    }
    else if (antenna.dot11ahPropagationModelInterfacePtr != nullptr) {
        antenna.dot11ahPropagationModelInterfacePtr->CalculatePathlossToLocation(
            informationType,
            positionXMeters,
            positionYMeters,
            positionZMeters,
            propagationStatistics);
    }
    else if (antenna.t109PropagationModelInterfacePtr != nullptr) {
        antenna.t109PropagationModelInterfacePtr->CalculatePathlossToLocation(
            informationType,
            positionXMeters,
            positionYMeters,
            positionZMeters,
            propagationStatistics);
    }
    else if (antenna.dot15PropagationModelInterfacePtr != nullptr) {
        antenna.dot15PropagationModelInterfacePtr->CalculatePathlossToLocation(
            informationType,
            positionXMeters,
            positionYMeters,
            positionZMeters,
            propagationStatistics);
    }
    else if (antenna.blePropagationModelInterfacePtr != nullptr) {
        antenna.blePropagationModelInterfacePtr->CalculatePathlossToLocation(
            informationType,
            positionXMeters,
            positionYMeters,
            positionZMeters,
            propagationStatistics);
    }
    else if (antenna.alohaPropagationModelInterfacePtr != nullptr) {
        antenna.alohaPropagationModelInterfacePtr->CalculatePathlossToLocation(
            informationType,
            positionXMeters,
            positionYMeters,
            positionZMeters,
            propagationStatistics);
    }//if//

}//CalculatePathlossToLocation//

template <typename PropagationModelInterfaceType>
static inline
unsigned int GetChannelNumberForPropagationCalculation(const PropagationModelInterfaceType& propagationInterface)
{
    const vector<unsigned int>& channelNumberSet = propagationInterface.GetCurrentChannelNumberSet();

    unsigned int channelNumber;

    if (channelNumberSet.empty()) {
        channelNumber = propagationInterface.GetCurrentChannelNumber();
    }
    else {
        channelNumber = channelNumberSet.front();
    }//if//

    return channelNumber;
}

void SimNode::CalculatePathlossToNode(
    const PropagationInformationType& informationType,
    const unsigned int antennaNumber,
    const ObjectMobilityPosition& rxAntennaPosition,
    const ObjectMobilityModel::MobilityObjectId& rxNodeId,
    const AntennaModel& rxAntennaModel,
    PropagationStatisticsType& propagationStatistics) const
{
    AntennaType& antenna = *(*this).GetAntennaPtr(antennaNumber);

    if (stationType == BS) {

        antenna.lteDownlinkPropagationModelInterfacePtr->GetPropagationModel()->CalculatePathlossToNode(
            informationType,
            antenna.lteDownlinkPropagationModelInterfacePtr->GetCurrentMobilityPosition(),
            theNodeId,
            *antenna.antennaModelPtr,
            rxAntennaPosition,
            rxNodeId,
            rxAntennaModel,
            GetChannelNumberForPropagationCalculation(*antenna.lteDownlinkPropagationModelInterfacePtr),
            propagationStatistics);
    }
    else if (stationType == UE) {

        antenna.lteUplinkPropagationModelInterfacePtr->GetPropagationModel()->CalculatePathlossToNode(
            informationType,
            antenna.lteUplinkPropagationModelInterfacePtr->GetCurrentMobilityPosition(),
            theNodeId,
            *antenna.antennaModelPtr,
            rxAntennaPosition,
            rxNodeId,
            rxAntennaModel,
            GetChannelNumberForPropagationCalculation(*antenna.lteUplinkPropagationModelInterfacePtr),
            propagationStatistics);
    }
    else if (antenna.dot11PropagationModelInterfacePtr != nullptr) {

        antenna.dot11PropagationModelInterfacePtr->GetPropagationModel()->CalculatePathlossToNode(
            informationType,
            antenna.dot11PropagationModelInterfacePtr->GetCurrentMobilityPosition(),
            theNodeId,
            *antenna.antennaModelPtr,
            rxAntennaPosition,
            rxNodeId,
            rxAntennaModel,
            GetChannelNumberForPropagationCalculation(*antenna.dot11PropagationModelInterfacePtr),
            propagationStatistics);
    }
    else if (antenna.dot11adPropagationModelInterfacePtr != nullptr) {

        antenna.dot11adPropagationModelInterfacePtr->GetPropagationModel()->CalculatePathlossToNode(
            informationType,
            antenna.dot11adPropagationModelInterfacePtr->GetCurrentMobilityPosition(),
            theNodeId,
            *antenna.antennaModelPtr,
            rxAntennaPosition,
            rxNodeId,
            rxAntennaModel,
            GetChannelNumberForPropagationCalculation(*antenna.dot11adPropagationModelInterfacePtr),
            propagationStatistics);
    }
    else if (antenna.dot11ahPropagationModelInterfacePtr != nullptr) {

        antenna.dot11ahPropagationModelInterfacePtr->GetPropagationModel()->CalculatePathlossToNode(
            informationType,
            antenna.dot11ahPropagationModelInterfacePtr->GetCurrentMobilityPosition(),
            theNodeId,
            *antenna.antennaModelPtr,
            rxAntennaPosition,
            rxNodeId,
            rxAntennaModel,
            GetChannelNumberForPropagationCalculation(*antenna.dot11ahPropagationModelInterfacePtr),
            propagationStatistics);
    }
    else if (antenna.t109PropagationModelInterfacePtr != nullptr) {

        antenna.t109PropagationModelInterfacePtr->GetPropagationModel()->CalculatePathlossToNode(
            informationType,
            antenna.t109PropagationModelInterfacePtr->GetCurrentMobilityPosition(),
            theNodeId,
            *antenna.antennaModelPtr,
            rxAntennaPosition,
            rxNodeId,
            rxAntennaModel,
            GetChannelNumberForPropagationCalculation(*antenna.t109PropagationModelInterfacePtr),
            propagationStatistics);
    }
    else if (antenna.dot15PropagationModelInterfacePtr != nullptr) {

        antenna.dot15PropagationModelInterfacePtr->GetPropagationModel()->CalculatePathlossToNode(
            informationType,
            antenna.dot15PropagationModelInterfacePtr->GetCurrentMobilityPosition(),
            theNodeId,
            *antenna.antennaModelPtr,
            rxAntennaPosition,
            rxNodeId,
            rxAntennaModel,
            GetChannelNumberForPropagationCalculation(*antenna.dot15PropagationModelInterfacePtr),
            propagationStatistics);
    }
    else if (antenna.blePropagationModelInterfacePtr != nullptr) {

        antenna.blePropagationModelInterfacePtr->GetPropagationModel()->CalculatePathlossToNode(
            informationType,
            antenna.blePropagationModelInterfacePtr->GetCurrentMobilityPosition(),
            theNodeId,
            *antenna.antennaModelPtr,
            rxAntennaPosition,
            rxNodeId,
            rxAntennaModel,
            GetChannelNumberForPropagationCalculation(*antenna.blePropagationModelInterfacePtr),
            propagationStatistics);
    }
    else if (antenna.alohaPropagationModelInterfacePtr != nullptr) {

        antenna.alohaPropagationModelInterfacePtr->GetPropagationModel()->CalculatePathlossToNode(
            informationType,
            antenna.alohaPropagationModelInterfacePtr->GetCurrentMobilityPosition(),
            theNodeId,
            *antenna.antennaModelPtr,
            rxAntennaPosition,
            rxNodeId,
            rxAntennaModel,
            GetChannelNumberForPropagationCalculation(*antenna.alohaPropagationModelInterfacePtr),
            propagationStatistics);
    }//if//

}//CalculatePathlossToNode//

//----------------------------------------------------------------------------------------

shared_ptr<Aloha::PropagationModel> ChannelModelSet::GetAlohaPropagationModel(const ChannelInstanceIdType& instanceId)
{
    if (alohaPropagationModelPtrs.find(instanceId) == alohaPropagationModelPtrs.end()) {
        alohaPropagationModelPtrs[instanceId].reset(
            new Aloha::PropagationModel(
                (*theParameterDatabaseReaderPtr),
                runSeed,
                theSimulationEnginePtr,
                theGisSubsystemPtr,
                instanceId));

        simulatorPtr->AddPropagationCalculationTraceIfNecessary(
            instanceId,
            alohaPropagationModelPtrs[instanceId]->GetPropagationCalculationModel());
    }//if//

    return alohaPropagationModelPtrs[instanceId];
}//GetAlohaPropagationModel//

shared_ptr<Dot11::PropagationModel> ChannelModelSet::GetDot11PropagationModel(const ChannelInstanceIdType& instanceId)
{
    if (dot11PropagationModelPtrs.find(instanceId) == dot11PropagationModelPtrs.end()) {
        dot11PropagationModelPtrs[instanceId].reset(
            new Dot11::PropagationModel(
                (*theParameterDatabaseReaderPtr),
                runSeed,
                theSimulationEnginePtr,
                theGisSubsystemPtr,
                instanceId));

        simulatorPtr->AddPropagationCalculationTraceIfNecessary(
            instanceId,
            dot11PropagationModelPtrs[instanceId]->GetPropagationCalculationModel());
    }//if//

    return dot11PropagationModelPtrs[instanceId];
}//GetDot11PropagationModel//

shared_ptr<Dot11ad::PropagationModel> ChannelModelSet::GetDot11adPropagationModel(const ChannelInstanceIdType& instanceId)
{
    if (dot11adPropagationModelPtrs.find(instanceId) == dot11adPropagationModelPtrs.end()) {
        dot11adPropagationModelPtrs[instanceId].reset(
            new Dot11ad::PropagationModel(
                (*theParameterDatabaseReaderPtr),
                runSeed,
                theSimulationEnginePtr,
                theGisSubsystemPtr,
                instanceId));

        simulatorPtr->AddPropagationCalculationTraceIfNecessary(
            instanceId,
            dot11adPropagationModelPtrs[instanceId]->GetPropagationCalculationModel());
    }//if//

    return dot11adPropagationModelPtrs[instanceId];
}//GetDot11adPropagationModel//

shared_ptr<Dot11ah::PropagationModel> ChannelModelSet::GetDot11ahPropagationModel(const ChannelInstanceIdType& instanceId)
{
    if (dot11ahPropagationModelPtrs.find(instanceId) == dot11ahPropagationModelPtrs.end()) {
        dot11ahPropagationModelPtrs[instanceId].reset(
            new Dot11ah::PropagationModel(
                (*theParameterDatabaseReaderPtr),
                runSeed,
                theSimulationEnginePtr,
                theGisSubsystemPtr,
                instanceId));

        simulatorPtr->AddPropagationCalculationTraceIfNecessary(
            instanceId,
            dot11ahPropagationModelPtrs[instanceId]->GetPropagationCalculationModel());
    }//if//

    return dot11ahPropagationModelPtrs[instanceId];
}//GetDot11PropagationModel//

shared_ptr<T109::PropagationModel> ChannelModelSet::GetT109PropagationModel(const ChannelInstanceIdType& instanceId)
{
    if (t109PropagationModelPtrs.find(instanceId) == t109PropagationModelPtrs.end()) {
        t109PropagationModelPtrs[instanceId].reset(
            new T109::PropagationModel(
                (*theParameterDatabaseReaderPtr),
                runSeed,
                theSimulationEnginePtr,
                theGisSubsystemPtr,
                instanceId));

        simulatorPtr->AddPropagationCalculationTraceIfNecessary(
            instanceId,
            t109PropagationModelPtrs[instanceId]->GetPropagationCalculationModel());
    }//if//

    return t109PropagationModelPtrs[instanceId];
}//GetT109PropagationModel//

shared_ptr<Dot15::PropagationModel> ChannelModelSet::GetDot15PropagationModel(const ChannelInstanceIdType& instanceId)
{
    if (dot15PropagationModelPtrs.find(instanceId) == dot15PropagationModelPtrs.end()) {
        dot15PropagationModelPtrs[instanceId].reset(
            new Dot15::PropagationModel(
                (*theParameterDatabaseReaderPtr),
                runSeed,
                theSimulationEnginePtr,
                theGisSubsystemPtr,
                instanceId));

        simulatorPtr->AddPropagationCalculationTraceIfNecessary(
            instanceId,
            dot15PropagationModelPtrs[instanceId]->GetPropagationCalculationModel());
    }//if//

    return dot15PropagationModelPtrs[instanceId];
}//GetDot15PropagationModel//

shared_ptr<Ble::PropagationModel> ChannelModelSet::GetBlePropagationModel(const ChannelInstanceIdType& instanceId)
{
    if (blePropagationModelPtrs.find(instanceId) == blePropagationModelPtrs.end()) {
        blePropagationModelPtrs[instanceId].reset(
            new Ble::PropagationModel(
            (*theParameterDatabaseReaderPtr),
                runSeed,
                theSimulationEnginePtr,
                theGisSubsystemPtr,
                instanceId));

        simulatorPtr->AddPropagationCalculationTraceIfNecessary(
            instanceId,
            blePropagationModelPtrs[instanceId]->GetPropagationCalculationModel());
    }//if//

    return blePropagationModelPtrs[instanceId];
}//GetBlePropagationModel//

shared_ptr<Lte::DownlinkPropagationModel> ChannelModelSet::GetLteDownlinkPropagationModel(const ChannelInstanceIdType& instanceId)
{
    if (lteDownlinkPropagationModelPtrs.find(instanceId) == lteDownlinkPropagationModelPtrs.end()) {
        lteDownlinkPropagationModelPtrs[instanceId].reset(
            new Lte::DownlinkPropagationModel(
                (*theParameterDatabaseReaderPtr),
                runSeed,
                theSimulationEnginePtr,
                theGisSubsystemPtr,
                instanceId));

        simulatorPtr->AddPropagationCalculationTraceIfNecessary(
            instanceId,
            lteDownlinkPropagationModelPtrs[instanceId]->GetPropagationCalculationModel());
    }//if//

    return lteDownlinkPropagationModelPtrs[instanceId];
}//GetLteDownlinkPropagationModel//

shared_ptr<Lte::UplinkPropagationModel> ChannelModelSet::GetLteUplinkPropagationModel(const ChannelInstanceIdType& instanceId)
{
    if (lteUplinkPropagationModelPtrs.find(instanceId) == lteUplinkPropagationModelPtrs.end()) {
        lteUplinkPropagationModelPtrs[instanceId].reset(
            new Lte::UplinkPropagationModel(
                (*theParameterDatabaseReaderPtr),
                runSeed,
                theSimulationEnginePtr,
                theGisSubsystemPtr,
                instanceId));

        simulatorPtr->AddPropagationCalculationTraceIfNecessary(
            instanceId,
            lteUplinkPropagationModelPtrs[instanceId]->GetPropagationCalculationModel());
    }//if//

    return lteUplinkPropagationModelPtrs[instanceId];
}//GetLteUplinkPropagationModel//

void ChannelModelSet::GetDot11ChannelModel(
    const ChannelInstanceIdType& instanceId,
    const unsigned int baseChannelNumber,
    const unsigned int numberChannels,
    const vector<double>& channelCarrierFrequenciesMhz,
    const vector<double>& channelBandwidthsMhz,
    shared_ptr<MimoChannelModel>& mimoChannelModelPtr,
    shared_ptr<FrequencySelectiveFadingModel>& frequencySelectiveFadingModelPtr)
{
    if (channelModelInfoMap.find(instanceId) == channelModelInfoMap.end()) {

        ChannelModelInfoType& channelModelInfo = channelModelInfoMap[instanceId];

        Dot11::ChooseAndCreateChannelModel(
            (*theParameterDatabaseReaderPtr),
            instanceId,
            baseChannelNumber,
            numberChannels,
            channelCarrierFrequenciesMhz,
            channelBandwidthsMhz,
            runSeed,
            *GetDot11PropagationModel(instanceId),
            channelModelInfo.mimoChannelModelPtr,
            channelModelInfo.frequencySelectiveFadingModelPtr);

    }//if//

    ChannelModelInfoType& channelModelInfo = channelModelInfoMap[instanceId];

    mimoChannelModelPtr = channelModelInfo.mimoChannelModelPtr;
    frequencySelectiveFadingModelPtr = channelModelInfo.frequencySelectiveFadingModelPtr;

    assert((mimoChannelModelPtr == nullptr) || (frequencySelectiveFadingModelPtr == nullptr) &&
           "Channel models are mutually exclusive");

}//GetDot11ChannelModel//

void ChannelModelSet::GetDot11adChannelModel(
    const ChannelInstanceIdType& instanceId,
    const unsigned int baseChannelNumber,
    const unsigned int numberChannels,
    const vector<double>& channelCarrierFrequenciesMhz,
    const vector<double>& channelBandwidthsMhz,
    shared_ptr<MimoChannelModel>& mimoChannelModelPtr,
    shared_ptr<FrequencySelectiveFadingModel>& frequencySelectiveFadingModelPtr)
{
    if (channelModelInfoMap.find(instanceId) == channelModelInfoMap.end()) {

        ChannelModelInfoType& channelModelInfo = channelModelInfoMap[instanceId];

        Dot11ad::ChooseAndCreateChannelModel(
            (*theParameterDatabaseReaderPtr),
            instanceId,
            baseChannelNumber,
            numberChannels,
            channelCarrierFrequenciesMhz,
            channelBandwidthsMhz,
            runSeed,
            channelModelInfo.mimoChannelModelPtr,
            channelModelInfo.frequencySelectiveFadingModelPtr);

    }//if//

    ChannelModelInfoType& channelModelInfo = channelModelInfoMap[instanceId];

    mimoChannelModelPtr = channelModelInfo.mimoChannelModelPtr;
    frequencySelectiveFadingModelPtr = channelModelInfo.frequencySelectiveFadingModelPtr;

    assert((mimoChannelModelPtr == nullptr) || (frequencySelectiveFadingModelPtr == nullptr) &&
           "Channel models are mutually exclusive");

}//GetDot11adChannelModel//

void ChannelModelSet::GetLteMimoOrFadingModelPtr(
    const ChannelInstanceIdType& downlinkInstanceId,
    const ChannelInstanceIdType& uplinkInstanceId,
    shared_ptr<ScenSim::MimoChannelModel>& downlinkMimoChannelModelPtr,
    shared_ptr<ScenSim::MimoChannelModel>& uplinkMimoChannelModelPtr,
    shared_ptr<ScenSim::FrequencySelectiveFadingModel>& downlinkFrequencySelectiveFadingModelPtr,
    shared_ptr<ScenSim::FrequencySelectiveFadingModel>& uplinkFrequencySelectiveFadingModelPtr)
{
    typedef map<ChannelInstanceIdType, LteChannelInfoType>::const_iterator IterType;

    IterType iter = lteChannelModelInfos.find(downlinkInstanceId);

    if (iter != lteChannelModelInfos.end()) {

        const LteChannelInfoType& channelInfo = iter->second;

        assert(uplinkInstanceId == channelInfo.uplinkChannelInstanceId);

        downlinkMimoChannelModelPtr = channelInfo.downlinkMimoChannelModelPtr;
        uplinkMimoChannelModelPtr = channelInfo.uplinkMimoChannelModelPtr;
        downlinkFrequencySelectiveFadingModelPtr =
            channelInfo.downlinkFrequencySelectiveFadingModelPtr;
        uplinkFrequencySelectiveFadingModelPtr =
            channelInfo.uplinkFrequencySelectiveFadingModelPtr;
        return;

    }//if//

    if (lteDownlinkPropagationModelPtrs.find(downlinkInstanceId) == lteDownlinkPropagationModelPtrs.end()) {
        cerr << "Error: LTE Downlink propagation model " << downlinkInstanceId
             << " must be initialized before fading model initialization." << endl;
        exit(1);
    }//if//

    if (lteUplinkPropagationModelPtrs.find(uplinkInstanceId) == lteUplinkPropagationModelPtrs.end()) {
        cerr << "Error: LTE Uplink propagation model " << uplinkInstanceId
             << " must be initialized before fading model initialization." << endl;
        exit(1);
    }//if//

    const Lte::DownlinkPropagationModel& downlinkPropModel = *lteDownlinkPropagationModelPtrs[downlinkInstanceId];
    const Lte::UplinkPropagationModel& uplinkPropModel = *lteUplinkPropagationModelPtrs[uplinkInstanceId];

    Lte::ChooseAndCreateChannelModel(
        *theParameterDatabaseReaderPtr,
        downlinkPropModel.GetInstanceId(),
        downlinkPropModel.GetBaseChannelNumber(),
        downlinkPropModel.GetChannelCount(),
        downlinkMimoChannelModelPtr,
        downlinkFrequencySelectiveFadingModelPtr);

    assert((downlinkMimoChannelModelPtr == nullptr) ||
           (downlinkFrequencySelectiveFadingModelPtr == nullptr));

    Lte::ChooseAndCreateChannelModel(
        *theParameterDatabaseReaderPtr,
        uplinkPropModel.GetInstanceId(),
        uplinkPropModel.GetBaseChannelNumber(),
        uplinkPropModel.GetChannelCount(),
        uplinkMimoChannelModelPtr,
        uplinkFrequencySelectiveFadingModelPtr);

    assert((uplinkMimoChannelModelPtr == nullptr) ||
           (uplinkFrequencySelectiveFadingModelPtr == nullptr));


    LteChannelInfoType& channelInfo = lteChannelModelInfos[downlinkInstanceId];

    channelInfo.uplinkChannelInstanceId = uplinkInstanceId;
    channelInfo.downlinkMimoChannelModelPtr = downlinkMimoChannelModelPtr;
    channelInfo.uplinkMimoChannelModelPtr = uplinkMimoChannelModelPtr;
    channelInfo.downlinkFrequencySelectiveFadingModelPtr = downlinkFrequencySelectiveFadingModelPtr;
    channelInfo.uplinkFrequencySelectiveFadingModelPtr = uplinkFrequencySelectiveFadingModelPtr;

}//GetLteMimoOrFadingModelPtr//


shared_ptr<Lte::LteGlobalParameters> ChannelModelSet::GetLteGlobals()
{
    if (lteGlobalsPtr != nullptr) {
        return lteGlobalsPtr;
    }//if//

    lteGlobalsPtr.reset(new Lte::LteGlobalParameters(*theParameterDatabaseReaderPtr));

    return lteGlobalsPtr;
}//GetLteGlobals//

shared_ptr<BitOrBlockErrorRateCurveDatabase> ChannelModelSet::GetDot11BitOrBlockErrorRateCurveDatabase()
{
    if (dot11BitOrBlockErrorRateCurveDatabasePtr != nullptr) {
        return dot11BitOrBlockErrorRateCurveDatabasePtr;
    }//if//

    const string berCurveFileName =
        theParameterDatabaseReaderPtr->ReadString("dot11-bit-error-rate-curve-file");

    dot11BitOrBlockErrorRateCurveDatabasePtr.reset(
        new BitOrBlockErrorRateCurveDatabase(berCurveFileName));

    return dot11BitOrBlockErrorRateCurveDatabasePtr;
}//GetDot11BitOrBlockErrorRateCurveDatabase//

shared_ptr<BitOrBlockErrorRateCurveDatabase> ChannelModelSet::GetDot11AdBitOrBlockErrorRateCurveDatabase()
{
    if (dot11AdBitOrBlockErrorRateCurveDatabasePtr != nullptr) {
        return dot11AdBitOrBlockErrorRateCurveDatabasePtr;
    }//if//

    const string berCurveFileName =
        theParameterDatabaseReaderPtr->ReadString("dot11ad-bit-error-rate-curve-file");

    dot11AdBitOrBlockErrorRateCurveDatabasePtr.reset(
        new BitOrBlockErrorRateCurveDatabase(berCurveFileName));

    return dot11AdBitOrBlockErrorRateCurveDatabasePtr;
}//GetDot11AdBitOrBlockErrorRateCurveDatabase//

shared_ptr<BitOrBlockErrorRateCurveDatabase> ChannelModelSet::GetDot11AhBitOrBlockErrorRateCurveDatabase()
{
    if (dot11AhBitOrBlockErrorRateCurveDatabasePtr != nullptr) {
        return dot11AhBitOrBlockErrorRateCurveDatabasePtr;
    }//if//

    const string berCurveFileName =
        theParameterDatabaseReaderPtr->ReadString("dot11ah-bit-error-rate-curve-file");

    dot11AhBitOrBlockErrorRateCurveDatabasePtr.reset(
        new BitOrBlockErrorRateCurveDatabase(berCurveFileName));

    return dot11AhBitOrBlockErrorRateCurveDatabasePtr;
}//GetDot11AhBitOrBlockErrorRateCurveDatabase//

shared_ptr<BitOrBlockErrorRateCurveDatabase> ChannelModelSet::GetT109BitOrBlockErrorRateCurveDatabase()
{
    return (*this).GetDot11BitOrBlockErrorRateCurveDatabase();

}//GetT109BitOrBlockErrorRateCurveDatabase//

shared_ptr<BitOrBlockErrorRateCurveDatabase> ChannelModelSet::GetDot15BitOrBlockErrorRateCurveDatabase()
{
    if (dot15BitOrBlockErrorRateCurveDatabasePtr != nullptr) {
        return dot15BitOrBlockErrorRateCurveDatabasePtr;
    }//if//

    const string berCurveFileName =
        theParameterDatabaseReaderPtr->ReadString("dot15-bit-error-rate-curve-file");

    dot15BitOrBlockErrorRateCurveDatabasePtr.reset(
        new BitOrBlockErrorRateCurveDatabase(berCurveFileName));

    return dot15BitOrBlockErrorRateCurveDatabasePtr;
}//GetDot15BitOrBlockErrorRateCurveDatabase//

shared_ptr<BitOrBlockErrorRateCurveDatabase> ChannelModelSet::GetBleBitOrBlockErrorRateCurveDatabase()
{
    if (bleBitOrBlockErrorRateCurveDatabasePtr != nullptr) {
        return bleBitOrBlockErrorRateCurveDatabasePtr;
    }//if//

    const string berCurveFileName =
        theParameterDatabaseReaderPtr->ReadString("ble-bit-error-rate-curve-file");

    bleBitOrBlockErrorRateCurveDatabasePtr.reset(
        new BitOrBlockErrorRateCurveDatabase(berCurveFileName));

    return bleBitOrBlockErrorRateCurveDatabasePtr;
}//GetBleBitOrBlockErrorRateCurveDatabase//

shared_ptr<BitOrBlockErrorRateCurveDatabase> ChannelModelSet::GetLteBitOrBlockErrorRateCurveDatabase()
{
    if (lteBitOrBlockErrorRateCurveDatabasePtr != nullptr) {
        return lteBitOrBlockErrorRateCurveDatabasePtr;
    }//if//

    const string blerCurveFileName =
        theParameterDatabaseReaderPtr->ReadString(Lte::parameterNamePrefix + "block-error-rate-curve-file");

    lteBitOrBlockErrorRateCurveDatabasePtr.reset(
        new BitOrBlockErrorRateCurveDatabase());

    lteBitOrBlockErrorRateCurveDatabasePtr->LoadBlockErrorRateCurveFile(blerCurveFileName);

    return lteBitOrBlockErrorRateCurveDatabasePtr;
}//GetLteBitOrBlockErrorRateCurveDatabase//
