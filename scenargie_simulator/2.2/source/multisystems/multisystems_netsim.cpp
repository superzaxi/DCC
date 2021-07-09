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

using ScenSim::ConvertAStringSequenceOfANumericTypeIntoAVector;
using ScenSim::InterPropagationModelInterferenceSignalInterface;

#pragma warning(disable:4355)

MultiSystemsSimulator::MultiSystemsSimulator(
    const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const RandomNumberGeneratorSeed& initRunSeed,
    const bool initRunSequentially,
    const bool initIsScenarioSettingOutputMode,
    const string& initInputConfigFileName,
    const string& initOutputConfigFileName)
    :
    MultiAgentSimulator(
        initParameterDatabaseReaderPtr,
        initSimulationEnginePtr,
        initRunSeed,
        initRunSequentially,
        initIsScenarioSettingOutputMode,
        initInputConfigFileName,
        initOutputConfigFileName),
    channelModelSetPtr(
        new ChannelModelSet(
            this,
            initParameterDatabaseReaderPtr,
            initSimulationEnginePtr,
            theGisSubsystemPtr,
            initRunSeed))
{

    (*this).CompleteSimulatorConstruction();

    (*this).SetupInterChannelModelInterference(*initParameterDatabaseReaderPtr);

}//MultiSystemsSimulator()//

#pragma warning(default:4355)

void MultiSystemsSimulator::CreateNewNode(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const shared_ptr<ObjectMobilityModel>& nodeMobilityModelPtr,
    const string& nodeTypeName)
{
    unsigned int partitionIndex = 0;

    if (theParameterDatabaseReader.ParameterExists("parallelization-partition-index", theNodeId)) {
        partitionIndex =
            (theParameterDatabaseReader.ReadNonNegativeInt("parallelization-partition-index", theNodeId) %
             theSimulationEnginePtr->GetNumberPartitionThreads());
    }//if//

    const shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr =
        theSimulationEnginePtr->GetSimulationEngineInterface(
            theParameterDatabaseReader, theNodeId, partitionIndex);

    const shared_ptr<AttachedAntennaMobilityModel> attachedAntennaMobilityModelPtr(
        new AttachedAntennaMobilityModel(nodeMobilityModelPtr));

    shared_ptr<SimNode> aSimNodePtr(
        new SimNode(
            theParameterDatabaseReader,
            theGlobalNetworkingObjectBag,
            simulationEngineInterfacePtr,
            theNodeId,
            runSeed,
            theGisSubsystemPtr,
            attachedAntennaMobilityModelPtr,
            channelModelSetPtr));

    (*this).AddCommunicationNode(aSimNodePtr);

    simNodePtrs[theNodeId] = aSimNodePtr;

}//CreateNewNode//

void MultiSystemsSimulator::CompleteSimulatorConstruction()
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    set<NodeId> setOfNodeIds;
    theParameterDatabaseReader.MakeSetOfAllCommNodeIds(setOfNodeIds);

    typedef set<NodeId>::const_iterator IterType;

    for(IterType iter = setOfNodeIds.begin();
        iter != setOfNodeIds.end(); iter++) {

        const NodeId theNodeId = (*iter);

        assert(theNodeId <= MAX_COMMUNICATION_NODEID);

        unsigned int partitionIndex = 0;

        if (theParameterDatabaseReader.ParameterExists("parallelization-partition-index", theNodeId)) {
            partitionIndex =
                (theParameterDatabaseReader.ReadNonNegativeInt("parallelization-partition-index", theNodeId) %
                 theSimulationEnginePtr->GetNumberPartitionThreads());
        }//if//

        shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr(
            theSimulationEnginePtr->GetSimulationEngineInterface(
                theParameterDatabaseReader, theNodeId, partitionIndex));

        shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr =
            CreateAntennaMobilityModel(
                theParameterDatabaseReader,
                theNodeId,
                ScenSim::nullInstanceId,
                mobilitySeed,
                mobilityFileCache,
                theGisSubsystemPtr);

        if ((*this).IsEqualToAgentId(theNodeId)) {

            (*this).CreateCommunicationNodeAtWakeupTimeFor(theNodeId);
        }
        else {

            const SimTime currentSimTime = simulationEngineInterfacePtr->CurrentTime();

            SimTime creationTime = ZERO_TIME;
            SimTime deletionTime = INFINITE_TIME;

            if (nodeMobilityModelPtr != nullptr) {
                creationTime = nodeMobilityModelPtr->GetCreationTime();
                deletionTime = nodeMobilityModelPtr->GetDeletionTime();
            }//if//

            if (creationTime <= currentSimTime) {

                (*this).CreateNewNode(theParameterDatabaseReader, theNodeId, nodeMobilityModelPtr);

            }
            else if (creationTime < deletionTime &&
                     creationTime < INFINITE_TIME) {

                simulationEngineInterfacePtr->ScheduleEvent(
                    unique_ptr<SimulationEvent>(new NodeEnterEvent(this, theNodeId, nodeMobilityModelPtr)), creationTime);
            }//if//

            if (creationTime < deletionTime &&
                deletionTime < INFINITE_TIME) {

                simulationEngineInterfacePtr->ScheduleEvent(
                    unique_ptr<SimulationEvent>(new NodeLeaveEvent(this, theNodeId)), deletionTime);
            }//if//
        }//if//
    }//for//

    (*this).CheckTheNecessityOfMultiAgentSupport();

    (*this).SetupStatOutputFile();
    (*this).ExecuteTimestepBasedEvent();
}//CompleteSimulatorConstruction//

void MultiSystemsSimulator::DeleteNode(const NodeId& theNodeId)
{
    NetworkSimulator::DeleteNode(theNodeId);

    simNodePtrs.erase(theNodeId);
}//DeleteNode//

unsigned int MultiSystemsSimulator::LookupInterfaceIndex(
    const NodeId& theNodeId,
    const InterfaceId& interfaceName) const
{
    typedef map<NodeId, shared_ptr<SimNode> > ::const_iterator IterType;

    IterType iter = simNodePtrs.find(theNodeId);

    if (iter == simNodePtrs.end() ||
        nodes.find(theNodeId) == nodes.end()) {
        cerr << "Error: Network Node Id: " << theNodeId << " Not Found." << endl;
        exit(1);
    }//if

    return (*iter).second->GetAntennaNumber(interfaceName);

}//LookupInterfaceIndex//

inline
void ConvertVectorIntoMatrix(
    const vector<double>& aVector,
    const unsigned int numRows,
    const unsigned int numColumns,
    bool& success,
    vector<vector<double> >& aMatrix)
{
    success = false;
    if (aVector.size() == (numRows * numColumns)) {
        aMatrix.resize(numRows, vector<double>(numColumns));
        for (unsigned int i = 0; (i < numRows); i++) {
            for (unsigned int j = 0; (j < numColumns); j++) {
                aMatrix[i][j] = aVector[i * numColumns + j];
            }//for//
        }//for//

        success = true;
    }//if//
}//ConvertVectorIntoMatrix//

void MultiSystemsSimulator::SetupInterChannelModelInterference(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{

    const string destIdParmName = "propagation-inter-model-interference-destination-instance-id";
    const string matrixParmName = "propagation-inter-model-channel-interference-matrix";

    set<InterfaceOrInstanceId> interferenceSourceInstanceIds;

    theParameterDatabaseReader.MakeSetOfAllInstanceIds(destIdParmName, interferenceSourceInstanceIds);

    for (set<InterfaceOrInstanceId>::const_iterator iter = interferenceSourceInstanceIds.begin();
        iter != interferenceSourceInstanceIds.end(); ++iter) {
        
        const InterfaceOrInstanceId sourceChannelId = (*iter);

        const string destinationChannelId =
            MakeLowerCaseString(
                theParameterDatabaseReader.ReadString(destIdParmName, sourceChannelId));

        const string channelMatrixString =
                theParameterDatabaseReader.ReadString(matrixParmName, sourceChannelId);

        vector<double> matrixVector;
        bool success;
        ConvertAStringSequenceOfANumericTypeIntoAVector(channelMatrixString, success, matrixVector);

        if (!success) {
            cerr << "Error in parameter \"" << matrixParmName << "\" : Is the wrong size." << endl;
            exit(1);
        }//if//

        if ((channelModelSetPtr->BlePropagationModelExists(sourceChannelId)) &&
            (channelModelSetPtr->Dot11PropagationModelExists(destinationChannelId))) {

            //BLE -> Dot11
            shared_ptr<Ble::PropagationModel> sourcePropModelPtr = channelModelSetPtr->GetBlePropagationModel(sourceChannelId);
            shared_ptr<Dot11::PropagationModel> destPropModelPtr = channelModelSetPtr->GetDot11PropagationModel(destinationChannelId);

            vector<vector<double> > channelInterferenceMatrix;

            bool success = false;
            ConvertVectorIntoMatrix(
                matrixVector,
                sourcePropModelPtr->GetChannelCount(),
                destPropModelPtr->GetChannelCount(),
                success,
                channelInterferenceMatrix);

            if (!success) {
                cerr << "Error in parameter \"" << matrixParmName << "\" : Is the wrong size." << endl;
                exit(1);
            }//if//

            unique_ptr<InterPropagationModelInterferenceSignalInterface> interfacePtr =
                destPropModelPtr->GetInterPropagationModelInterferenceSignalInterface();

            sourcePropModelPtr->SetupInterPropagationModelInterference(interfacePtr, channelInterferenceMatrix);

        }
        else if ((channelModelSetPtr->Dot11PropagationModelExists(sourceChannelId)) &&
            (channelModelSetPtr->BlePropagationModelExists(destinationChannelId))) {

            //Dot11 -> BLE
            shared_ptr<Dot11::PropagationModel> sourcePropModelPtr = channelModelSetPtr->GetDot11PropagationModel(sourceChannelId);
            shared_ptr<Ble::PropagationModel> destPropModelPtr = channelModelSetPtr->GetBlePropagationModel(destinationChannelId);

            vector<vector<double> > channelInterferenceMatrix;

            bool success = false;
            ConvertVectorIntoMatrix(
                matrixVector,
                sourcePropModelPtr->GetChannelCount(),
                destPropModelPtr->GetChannelCount(),
                success,
                channelInterferenceMatrix);

            if (!success) {
                cerr << "Error in parameter \"" << matrixParmName << "\" : Is the wrong size." << endl;
                exit(1);
            }//if//

            unique_ptr<InterPropagationModelInterferenceSignalInterface> interfacePtr =
                destPropModelPtr->GetInterPropagationModelInterferenceSignalInterface();

            sourcePropModelPtr->SetupInterPropagationModelInterference(interfacePtr, channelInterferenceMatrix);

        }
        else if ((channelModelSetPtr->LteDownlinkPropagationModelExists(sourceChannelId)) &&
            (channelModelSetPtr->Dot11PropagationModelExists(destinationChannelId))) {

            //LTE Downlink -> Dot11
            shared_ptr<Lte::DownlinkPropagationModel> sourcePropModelPtr = channelModelSetPtr->GetLteDownlinkPropagationModel(sourceChannelId);
            shared_ptr<Dot11::PropagationModel> destPropModelPtr = channelModelSetPtr->GetDot11PropagationModel(destinationChannelId);

            vector<vector<double> > channelInterferenceMatrix;

            bool success = false;
            ConvertVectorIntoMatrix(
                matrixVector,
                sourcePropModelPtr->GetChannelCount(),
                destPropModelPtr->GetChannelCount(),
                success,
                channelInterferenceMatrix);

            if (!success) {
                cerr << "Error in parameter \"" << matrixParmName << "\" : Is the wrong size." << endl;
                exit(1);
            }//if//

            unique_ptr<InterPropagationModelInterferenceSignalInterface> interfacePtr =
                destPropModelPtr->GetInterPropagationModelInterferenceSignalInterface();

            sourcePropModelPtr->SetupInterPropagationModelInterference(interfacePtr, channelInterferenceMatrix);

        }
        else if ((channelModelSetPtr->Dot11PropagationModelExists(sourceChannelId)) &&
            (channelModelSetPtr->LteDownlinkPropagationModelExists(destinationChannelId))) {

            //Dot11 -> LTE Downlink
            shared_ptr<Dot11::PropagationModel> sourcePropModelPtr = channelModelSetPtr->GetDot11PropagationModel(sourceChannelId);
            shared_ptr<Lte::DownlinkPropagationModel> destPropModelPtr = channelModelSetPtr->GetLteDownlinkPropagationModel(destinationChannelId);

            vector<vector<double> > channelInterferenceMatrix;

            bool success = false;
            ConvertVectorIntoMatrix(
                matrixVector,
                sourcePropModelPtr->GetChannelCount(),
                destPropModelPtr->GetChannelCount(),
                success,
                channelInterferenceMatrix);

            if (!success) {
                cerr << "Error in parameter \"" << matrixParmName << "\" : Is the wrong size." << endl;
                exit(1);
            }//if//

            unique_ptr<InterPropagationModelInterferenceSignalInterface> interfacePtr =
                destPropModelPtr->GetInterPropagationModelInterferenceSignalInterface();

            sourcePropModelPtr->SetupInterPropagationModelInterference(interfacePtr, channelInterferenceMatrix);

        }
        else if ((channelModelSetPtr->Dot11PropagationModelExists(sourceChannelId)) &&
            (channelModelSetPtr->Dot11PropagationModelExists(destinationChannelId))) {

            //Dot11 -> Dot11
            shared_ptr<Dot11::PropagationModel> sourcePropModelPtr = channelModelSetPtr->GetDot11PropagationModel(sourceChannelId);
            shared_ptr<Dot11::PropagationModel> destPropModelPtr = channelModelSetPtr->GetDot11PropagationModel(destinationChannelId);

            vector<vector<double> > channelInterferenceMatrix;

            bool success = false;
            ConvertVectorIntoMatrix(
                matrixVector,
                sourcePropModelPtr->GetChannelCount(),
                destPropModelPtr->GetChannelCount(),
                success,
                channelInterferenceMatrix);

            if (!success) {
                cerr << "Error in parameter \"" << matrixParmName << "\" : Is the wrong size." << endl;
                exit(1);
            }//if//

            unique_ptr<InterPropagationModelInterferenceSignalInterface> interfacePtr =
                destPropModelPtr->GetInterPropagationModelInterferenceSignalInterface();

            sourcePropModelPtr->SetupInterPropagationModelInterference(interfacePtr, channelInterferenceMatrix);

        }
        else {
            //unsupported combination or dest channel doesn't exsit
            cerr << "Error: Unsupported or unknown inter channel model interference from channel:" 
                << sourceChannelId << " to channel:" << destinationChannelId << endl;
            cerr << "Supported inter channel model interference combinations: LTE Downlink vs Dot11, BLE vs Dot11, or Dot11 vs Dot11." << endl;

            exit(1);
        }//if//

    }//for//

}//SetupInterChannelModelInterference//

