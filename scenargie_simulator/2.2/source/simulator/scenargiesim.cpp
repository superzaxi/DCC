// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include <iomanip>
#include "scensim_netsim.h"
#include "scenargiesim.h"


namespace ScenSim {

using std::setw;
using std::ofstream;


const PacketId PacketId::nullPacketId = PacketId();


shared_ptr<AntennaModel> CreateAntennaModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId,
    const AntennaPatternDatabase& anAntennaPatternDatabase)
{
    string antennaModelString = theParameterDatabaseReader.ReadString("antenna-model", theNodeId, theInterfaceId);
    ConvertStringToLowerCase(antennaModelString);

    if ((antennaModelString == "omni") || (antennaModelString == "omnidirectional")) {

        double antennaGainDbi;

        if (theParameterDatabaseReader.ParameterExists("max-antenna-gain-dbi", theNodeId, theInterfaceId)) {

            //will be removed in the future (only antenna-gain-dbi is available for omni)

            antennaGainDbi =
                theParameterDatabaseReader.ReadDouble("max-antenna-gain-dbi", theNodeId, theInterfaceId);

            if (theParameterDatabaseReader.ParameterExists("antenna-gain-dbi", theNodeId, theInterfaceId)) {
                cerr << "Error in configuration file: antenna-gain-dbi and max-antenna-gain-dbi both defined." << endl;
                exit(1);
            }//if//
        }
        else {
            antennaGainDbi =
                theParameterDatabaseReader.ReadDouble("antenna-gain-dbi", theNodeId, theInterfaceId);
        }//if//

        return (shared_ptr<AntennaModel>(new OmniAntennaModel(antennaGainDbi)));
    }
    else if (antennaModelString == "sectored") {
        const double antennaGainDbi =
            theParameterDatabaseReader.ReadDouble("max-antenna-gain-dbi", theNodeId, theInterfaceId);

        return (shared_ptr<AntennaModel>(new SectoredAntennaModel(antennaGainDbi)));
    }
    else if ((antennaModelString == "fupm") ||
             (antennaModelString == "hfpm") ||
             (antennaModelString == "fupm/hfpm")) {
        //use InSight .uan file
        return (shared_ptr<AntennaModel>(new OmniAntennaModel(0.0)));
    }
    else if (anAntennaPatternDatabase.IsDefined(antennaModelString)) {

        bool useQuasiOmniMode = false;

        if (theParameterDatabaseReader.ParameterExists(
            "antenna-model-use-quasi-omni-mode", theNodeId, theInterfaceId)) {

            useQuasiOmniMode =
                theParameterDatabaseReader.ReadBool(
                    "antenna-model-use-quasi-omni-mode", theNodeId, theInterfaceId);

        }//if//

        if (theParameterDatabaseReader.ParameterExists(
            "antenna-model-quasi-omni-mode-gain-dbi", theNodeId, theInterfaceId)) {

            const double quasiOmniAntennaGainDbi =
                theParameterDatabaseReader.ReadDouble(
                    "antenna-model-quasi-omni-mode-gain-dbi", theNodeId, theInterfaceId);

            return (shared_ptr<AntennaModel>(
                new CustomAntennaModel(
                    anAntennaPatternDatabase,
                    antennaModelString,
                    useQuasiOmniMode,
                    quasiOmniAntennaGainDbi)));
        }
        else {
            return (shared_ptr<AntennaModel>(
                new CustomAntennaModel(
                    anAntennaPatternDatabase,
                    antennaModelString,
                    useQuasiOmniMode)));
        }//if//
    }
    else {
        cerr << "Antenna Model: " << antennaModelString << " is invalid." << endl;
        exit(1);
    }//if//

}//CreateAntennaModel//



NetworkSimulator::~NetworkSimulator()
{
    assert(nodes.empty() &&
           "Error: All Nodes must be deleted sometime before the calling of NetworkSimulator"
           " destructor (base class) (can call DeleteAllNodes() in child's destructor)");

    if (!statsOutputFilename.empty()) {

        const SimTime currentTime = theSimulationEnginePtr->CurrentTime();

        OutputStatsToFile(statsOutputFilename, fileControlledStatViews, noDataOutputIsEnabled, ZERO_TIME, currentTime);
    }//if//
}


void NetworkSimulator::CalculatePathlossFromNodeToLocation(
    const NodeId& txNodeId,
    const PropagationInformationType& informationType,
    const unsigned int interfaceIndex,
    const double& positionXMeters,
    const double& positionYMeters,
    const double& positionZMeters,
    PropagationStatisticsType& propagationStatistics)
{
    nodes[txNodeId]->CalculatePathlossToLocation(
        informationType,
        interfaceIndex,
        positionXMeters,
        positionYMeters,
        positionZMeters,
        propagationStatistics);
}


void NetworkSimulator::CalculatePathlossFromNodeToNode(
    const NodeId& txNodeId,
    const NodeId& rxNodeId,
    const PropagationInformationType& informationType,
    const unsigned int txInterfaceIndex,
    const unsigned int rxInterfaceIndex,
    PropagationStatisticsType& propagationStatistics)
{
    const shared_ptr<NetworkNode> rxNodePtr = nodes[rxNodeId];

    nodes[txNodeId]->CalculatePathlossToNode(
        informationType,
        txInterfaceIndex,
        rxNodePtr->GetAntennaLocation(rxInterfaceIndex),
        rxNodeId,
        *rxNodePtr->GetAntennaModelPtr(rxInterfaceIndex),
        propagationStatistics);

}//CalculatePathlossFromNodeToNode//



void NetworkSimulator::OutputNodePositionsInXY(
    const SimTime lastOutputTime,
    std::ostream& nodePositionOutStream) const
{
    typedef map<NodeId, shared_ptr<NetworkNode> >::const_iterator IterType;

    for(IterType iter = nodes.begin(); (iter != nodes.end()); ++iter) {
        const NodeId& theNodeId = iter->first;
        const NetworkNode& node = *iter->second;

        const ObjectMobilityPosition nodePosition = node.GetCurrentLocation();

        // Don't output stationary nodes.

        if ((lastOutputTime == ZERO_TIME) || (nodePosition.LastMoveTime() > lastOutputTime)) {

            nodePositionOutStream.precision(10);

            nodePositionOutStream << ' ' << theNodeId << ' '
                                  << setw(11) << nodePosition.X_PositionMeters() << ' '
                                  << setw(11) << nodePosition.Y_PositionMeters() << ' '
                                  << nodePosition.HeightFromGroundMeters() << ' '
                                  << nodePosition.AttitudeAzimuthFromNorthClockwiseDegrees() << ' '
                                  << nodePosition.AttitudeElevationFromHorizonDegrees();

        }//if//
    }//for//
    nodePositionOutStream.flush();

}//OutputNodePositionsInXY//



void NetworkSimulator::OutputTraceForAllNodePositions(const SimTime& lastOutputTime) const
{

    typedef map<NodeId, shared_ptr<NetworkNode> >::const_iterator IterType;

    for(IterType iter = nodes.begin(); (iter != nodes.end()); ++iter) {

        const NodeId& theNodeId = iter->first;
        const NetworkNode& node = *iter->second;

        node.OutputTraceForNodePosition(lastOutputTime);

    }//for//

}//OutputTraceForAllNodePositions//



void NetworkSimulator::OutputAllNodeIds(std::ostream& outStream) const
{
    typedef map<NodeId, shared_ptr<NetworkNode> >::const_iterator IterType;

    for(IterType iter = nodes.begin(); (iter != nodes.end()); ++iter) {
        outStream << ' ' << iter->first;
    }//for//
}



void NetworkSimulator::OutputRecentlyAddedNodeIdsWithTypes(std::ostream& outStream)
{
    for(unsigned int i = 0; (i < recentlyAddedNodeList.size()); i++) {

        string nodeTypeName = nodes[recentlyAddedNodeList[i]]->GetNodeTypeName();
        if (nodeTypeName == "") {
            nodeTypeName = "no_node_type_name";
        }//if//

        outStream << ' ' << recentlyAddedNodeList[i] << ' ' << nodeTypeName;
    }//for//
    recentlyAddedNodeList.clear();

}//OutputRecentlyAddedNodeIdsWithTypes//



void NetworkSimulator::OutputRecentlyDeletedNodeIds(std::ostream& outStream)
{
    for(unsigned int i = 0; (i < recentlyDeletedNodeList.size()); i++) {
        outStream << ' ' << recentlyDeletedNodeList[i];
    }//for//
    recentlyDeletedNodeList.clear();
}

void NetworkSimulator::CompleteSimulatorConstruction()
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    set<NodeId> setOfNodeIds;
    theParameterDatabaseReader.MakeSetOfAllCommNodeIds(setOfNodeIds);

    typedef set<NodeId>::const_iterator IterType;

    for(IterType iter = setOfNodeIds.begin(); iter != setOfNodeIds.end(); ++iter) {

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
                nullInstanceId,
                mobilitySeed,
                mobilityFileCache,
                theGisSubsystemPtr);

        const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
        const SimTime creationTime = nodeMobilityModelPtr->GetCreationTime();
        const SimTime deletionTime = nodeMobilityModelPtr->GetDeletionTime();

        if (creationTime <= currentTime) {

            (*this).CreateNewNode(theParameterDatabaseReader, theNodeId, nodeMobilityModelPtr);

        }
        else if ((creationTime < deletionTime) && (creationTime < INFINITE_TIME)) {

            simulationEngineInterfacePtr->ScheduleEvent(
                unique_ptr<SimulationEvent>(new NodeEnterEvent(this, theNodeId, nodeMobilityModelPtr)),
                creationTime);

        }//if//

        if ((creationTime < deletionTime) && (deletionTime < INFINITE_TIME)) {

            simulationEngineInterfacePtr->ScheduleEvent(
                unique_ptr<SimulationEvent>(new NodeLeaveEvent(this, theNodeId)),
                deletionTime);

        }//if//
    }//for//

    (*this).CheckTheNecessityOfMultiAgentSupport();

    (*this).ExecuteTimestepBasedEvent();

}//CompleteSimulatorConstruction//

void NetworkSimulator::CheckTheNecessityOfMultiAgentSupport()
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    vector<string> multiagentParameters;

    multiagentParameters.push_back("multiagent-profile-type");
    multiagentParameters.push_back("multiagent-behavior-type");

    bool needMultiAgentModule = false;

    for(size_t i = 0; i < multiagentParameters.size(); i++) {
        set<NodeId> nodeIds;

        theParameterDatabaseReader.MakeSetOfAllNodeIdsWithParameter(multiagentParameters[i], nodeIds);

        if (!nodeIds.empty()) {
            needMultiAgentModule = true;
            break;
        }//if//
    }//for//


    if (needMultiAgentModule) {
        if (!(*this).SupportMultiAgent()) {
            cerr << "Error: Found a MultiAgent Module parameter specification."
                 << "Use MultiAgent build option for Simulator. (build option: MULTIAGENT_MODULE=on)" << endl;
            exit(1);
        }//if//
    }//if//

}//CheckTheNecessityOfMultiAgentSupport//

void NetworkSimulator::SetupStatOutputFile()
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    if (theParameterDatabaseReader.ParameterExists("statistics-output-file")) {
        statsOutputFilename = theParameterDatabaseReader.ReadString("statistics-output-file");

        SetupFileControlledStats(
            theParameterDatabaseReader,
            theSimulationEnginePtr->GetRuntimeStatisticsSystem(),
            fileControlledStatViews,
            statsOutputFilename,
            noDataOutputIsEnabled);

    }//if//

}//SetupStatOutputFile//



void NetworkSimulator::AddPropagationCalculationTraceIfNecessary(
    const InterfaceId& channelId,
    const shared_ptr<SimplePropagationLossCalculationModel>& propagationCalculationModelPtr)
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    if (!theParameterDatabaseReader.ParameterExists("proptrace-filename", channelId)) {
        return;
    }

    string propagationModel =
        MakeLowerCaseString(theParameterDatabaseReader.ReadString("propagation-model", channelId));

    if (propagationModel == "trace") {
        return;
    }

    // set propagation trace output

    if (theParameterDatabaseReader.ParameterExists("proptrace-filename", channelId)) {

        const string traceOutputFileName =
            theParameterDatabaseReader.ReadString("proptrace-filename", channelId);

        ofstream outStream(traceOutputFileName.c_str(), std::ios::out);

        if (!outStream.good()) {
            cerr << "Could Not open propagation trace file: \"" << traceOutputFileName << "\"" << endl;
            exit(1);
        }//if//

        outStream << "TimeStepPropagationTrace2 " << ConvertTimeToStringSecs(timeStepEventSynchronizationStep) << endl;

        propagationTraceOutputInfos.push_back(
            PropagationTraceOutputInfo(
                channelId,
                propagationCalculationModelPtr,
                traceOutputFileName));

    }//if//

}//AddPropagationCalculationTraceIfNecessary//




static inline
double CalculateXySquaredDistance(const ObjectMobilityPosition& p1, const ObjectMobilityPosition& p2)
{
    return SquaredXYDistanceBetweenVertices(
        Vertex(p1.X_PositionMeters(), p1.Y_PositionMeters(), p1.HeightFromGroundMeters()),
        Vertex(p2.X_PositionMeters(), p2.Y_PositionMeters(), p2.HeightFromGroundMeters()));

}//CalculateXySquaredDistance//


struct PropagationResult {
    NodeId nodeId1;
    NodeId nodeId2;
    double lossValueDb12;
    double lossValueDb21;

    PropagationResult()
        :
        nodeId1(INVALID_NODEID),
        nodeId2(INVALID_NODEID),
        lossValueDb12(0),
        lossValueDb21(0)
    {}

    PropagationResult(
        const NodeId& initNodeId1,
        const NodeId& initNodeId2,
        const double initLossValueDb12,
        const double initLossValueDb21)
        :
        nodeId1(initNodeId1),
        nodeId2(initNodeId2),
        lossValueDb12(initLossValueDb12),
        lossValueDb21(initLossValueDb21)
    {}
};//PropagationResult//


static inline
bool IsSingleInterfaceNode(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId)
{
    const string antennaParameterName = "antenna-model";

    set<InterfaceOrInstanceId> setOfInstances;
    theParameterDatabaseReader.MakeSetOfAllInterfaceIdsForANode(theNodeId, antennaParameterName, setOfInstances);

    if (setOfInstances.empty()) {

        return theParameterDatabaseReader.ParameterExists(antennaParameterName, theNodeId);
    }

    return (setOfInstances.size() == 1);
}

static inline
void CheckAllNodeIsSingleInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const map<NodeId, shared_ptr<NetworkNode> >& nodes)
{
    typedef map<NodeId, shared_ptr<NetworkNode> >::const_iterator IterType;

    for(IterType iter = nodes.begin(); iter != nodes.end(); iter++) {

        if (!IsSingleInterfaceNode(theParameterDatabaseReader, (*iter).first)) {
            cerr << "Error: For propagation trace output all communication nodes must be single interface." << endl;
            exit(1);
        }//if//
    }//for//
}

void NetworkSimulator::OutputPropagationTrace(const SimTime& time)
{
    if (propagationTraceOutputInfos.empty()) {
        return;
    }

    if (propagationTraceOutputInfos.size() > 1) {
        cerr << "Error: Found propagation trace output option for multiple propagation models" << endl
             << "       Propagation trace output is available for single propagation calculation." << endl;
        exit(1);
    }

    assert(propagationTraceOutputInfos.size() == 1);


    // Assume all node is single interface node.

    CheckAllNodeIsSingleInterface(*theParameterDatabaseReaderPtr, nodes);


    // Calculate propagation trace for all node(;interface) paris

    for(size_t i = 0; i < propagationTraceOutputInfos.size(); i++) {
        const PropagationTraceOutputInfo& propagationTraceOutputInfo = propagationTraceOutputInfos[i];

        ofstream outStream(propagationTraceOutputInfo.outputFileName.c_str(), std::ios::binary | std::ios::out | std::ios::app);

        assert(outStream.good());
        outStream.write(reinterpret_cast<const char *>(&time), sizeof(time));

        SimplePropagationLossCalculationModel& calculationModel = *propagationTraceOutputInfo.propagationCalculationModelPtr;

        if (calculationModel.SupportMultipointCalculation()) {
            calculationModel.CacheMultipointPropagationLossDb();
        }//if//

        typedef map<NodeId, shared_ptr<NetworkNode> >::const_iterator NodeIter;

        const InterfaceId& channelId = propagationTraceOutputInfo.channelId;

        vector<PropagationResult> propagationResults;

        for(NodeIter nodeIter1 = nodes.begin(); nodeIter1 != nodes.end(); nodeIter1++) {
            const NetworkNode& node1 = *(*nodeIter1).second;

            for(NodeIter nodeIter2 = nodeIter1; nodeIter2 != nodes.end(); nodeIter2++) {
                const NetworkNode& node2 = *(*nodeIter2).second;

                // Use first interface (interfaceIndexx = 0) for propagation trace calculation.
                const unsigned int propagationCalculationInterfaceIndex = 0;
                const ObjectMobilityPosition pos1 = node1.GetAntennaLocation(propagationCalculationInterfaceIndex);
                const ObjectMobilityPosition pos2 = node2.GetAntennaLocation(propagationCalculationInterfaceIndex);
                const double distance = CalculateXySquaredDistance(pos1, pos2);

                const NodeId nodeId1 = node1.GetNodeId();
                const NodeId nodeId2 = node2.GetNodeId();

                propagationResults.push_back(
                    PropagationResult(
                        node1.GetNodeId(),
                        node2.GetNodeId(),
                        calculationModel.CalculatePropagationLossDb(pos1, nodeId1, pos2, nodeId2, distance),
                        calculationModel.CalculatePropagationLossDb(pos2, nodeId2, pos1, nodeId1, distance)));
            }//for//
        }//for//


        // Output bidirection propagation calculation results.

        const uint32_t numberResults = static_cast<uint32_t>(propagationResults.size());

        outStream.write(reinterpret_cast<const char *>(&numberResults), sizeof(numberResults));

        for(size_t j = 0; j < propagationResults.size(); j++) {
            const PropagationResult& propagationResult = propagationResults[j];

            outStream.write(reinterpret_cast<const char *>(&propagationResult.nodeId1), sizeof(propagationResult.nodeId1));
            outStream.write(reinterpret_cast<const char *>(&propagationResult.nodeId2), sizeof(propagationResult.nodeId2));
            outStream.write(reinterpret_cast<const char *>(&propagationResult.lossValueDb12), sizeof(propagationResult.lossValueDb12));
            outStream.write(reinterpret_cast<const char *>(&propagationResult.lossValueDb21), sizeof(propagationResult.lossValueDb21));
        }//if//
    }//for//

}//OutputPropagationTrace//


}//namespace//

