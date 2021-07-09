// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_NETSIM_H
#define SCENSIM_NETSIM_H

#include "scensim_application.h"
#include "scensim_transport.h"
#include "scensim_network.h"
#include "scensim_sensing.h"
#include <iostream>//20210519

using std::cout;//20210519
using std::endl;//20210519

namespace ScenSim {


class NetworkSimulator;

class NetworkAddressLookupInterface {
public:
    virtual NetworkAddress LookupNetworkAddress(const NodeId& theNodeId) const = 0;
    virtual void LookupNetworkAddress(
        const NodeId& theNodeId, NetworkAddress& networkAddress, bool& success) const = 0;

    virtual NodeId LookupNodeId(const NetworkAddress& aNetworkAddress) const = 0;
    virtual void LookupNodeId(
        const NetworkAddress& aNetworkAddress, NodeId& theNodeId, bool& success) const = 0;

    // virtual NetworkAddress LookupNetworkAddress(const string& DNS_Name) const;

    virtual ~NetworkAddressLookupInterface() { }
};



class ExtrasimulationNetAddressLookup: public NetworkAddressLookupInterface {
public:
    ExtrasimulationNetAddressLookup(NetworkSimulator* initNetworkSimulatorPtr)
        : networkSimulatorPtr(initNetworkSimulatorPtr) {}

    NetworkAddress LookupNetworkAddress(const NodeId& theNodeId) const;
    void LookupNetworkAddress(
        const NodeId& theNodeId, NetworkAddress& networkAddress, bool& success) const;

    NodeId LookupNodeId(const NetworkAddress& aNetworkAddress) const;
    void LookupNodeId(
        const NetworkAddress& aNetworkAddress, NodeId& theNodeId, bool& success) const;


    ~ExtrasimulationNetAddressLookup() { }
private:
    NetworkSimulator* networkSimulatorPtr;
};



class NetworkNode {
public:

    NetworkNode(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
        const NodeId& theNodeId,
        const RandomNumberGeneratorSeed& runSeed,
        const bool dontBuildStackLayers = false);

    virtual ~NetworkNode();

    NodeId GetNodeId() const { return theNodeId; }
    string GetNodeTypeName() const { return nodeTypeName; }
    void SetNodeTypeName(const string& typeName) { nodeTypeName = typeName; }
    NetworkAddress GetPrimaryNetworkAddress() const { return (GetNetworkLayerRef().GetPrimaryNetworkAddress()); }
    RandomNumberGeneratorSeed GetNodeSeed() const { return nodeSeed; }

    virtual shared_ptr<NetworkLayer> GetNetworkLayerPtr() const { return networkLayerPtr; }
    virtual const NetworkLayer& GetNetworkLayerRef() const { return (*networkLayerPtr);  }

    virtual shared_ptr<TransportLayer> GetTransportLayerPtr() const { return transportLayerPtr; }

    virtual shared_ptr<ApplicationLayer> GetAppLayerPtr() const { return appLayerPtr; }

    virtual const ObjectMobilityPosition GetCurrentLocation() const
    {  assert(false); abort(); return ObjectMobilityPosition(0,0,0,0,0,false, 0.0, 0.0, 0.0, 0.0, 0.0); }

    virtual void CalculatePathlossToLocation(
        const PropagationInformationType& informationType,
        const unsigned int interfaceIndex,
        const double& positionXMeters,
        const double& positionYMeters,
        const double& positionZMeters,
        PropagationStatisticsType& propagationStatistics) const
    {
        assert(false); abort();
    }

    virtual void CalculatePathlossToNode(
        const PropagationInformationType& informationType,
        const unsigned int interfaceIndex,
        const ObjectMobilityPosition& rxAntennaPosition,
        const ObjectMobilityModel::MobilityObjectId& rxObjectId,
        const AntennaModel& rxAntennaModel,
        PropagationStatisticsType& propagationStatistics) const
    {
        assert(false); abort();
    }

    virtual bool HasAntenna(const InterfaceId& channelId) const { return false; }
    virtual shared_ptr<AntennaModel> GetAntennaModelPtr(const unsigned int interfaceIndex) const { return shared_ptr<AntennaModel>(); }

    virtual ObjectMobilityPosition GetAntennaLocation(const unsigned int interfaceIndex) const
    {  assert(false); abort(); return ObjectMobilityPosition(0,0,0,0,0,false, 0.0, 0.0, 0.0, 0.0, 0.0); }

    virtual void OutputTraceForNodePosition(const SimTime& lastOutputTime) const;

    void OutputTraceForAddNode() const;
    void OutputTraceForDeleteNode() const;

    virtual void TriggerApplication() {};

    virtual void CreateDynamicApplication(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const NodeId& sourceNodeId,
        const InterfaceOrInstanceId& instanceId);

protected:

    NodeId theNodeId;
    string nodeTypeName;

    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;


    shared_ptr<NetworkLayer> networkLayerPtr;
    shared_ptr<TransportLayer> transportLayerPtr;
    shared_ptr<ApplicationLayer> appLayerPtr;
    shared_ptr<NetworkAddressLookupInterface> networkAddressLookupInterfacePtr;

    RandomNumberGeneratorSeed nodeSeed;

    //Disabled:
    NetworkNode(NetworkNode&);
    void operator=(NetworkNode&);

};//NetworkNode//





inline
NetworkNode::NetworkNode(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
    const NodeId& initNodeId,
    const RandomNumberGeneratorSeed& runSeed,
    const bool dontBuildStackLayers)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    theNodeId(initNodeId),
    nodeSeed(HashInputsToMakeSeed(runSeed, initNodeId)),
    networkAddressLookupInterfacePtr(theGlobalNetworkingObjectBag.networkAddressLookupInterfacePtr)
{
    // Don't build stack layers capability is a special case for emulation, where parts
    // of the stack are not simulated (real data).

    if (!dontBuildStackLayers) {

        networkLayerPtr =
            shared_ptr<BasicNetworkLayer>(
                BasicNetworkLayer::CreateNetworkLayer(
                    theParameterDatabaseReader,
                    theGlobalNetworkingObjectBag,
                    initSimulationEngineInterfacePtr,
                    initNodeId,
                    nodeSeed));

        transportLayerPtr = shared_ptr<TransportLayer>(
            new TransportLayer(
                theParameterDatabaseReader,
                initSimulationEngineInterfacePtr,
                networkLayerPtr,
                initNodeId,
                nodeSeed));

        appLayerPtr = shared_ptr<ApplicationLayer>(
            new ApplicationLayer(
                networkAddressLookupInterfacePtr,
                initSimulationEngineInterfacePtr,
                transportLayerPtr,
                initNodeMobilityModelPtr,
                initNodeId,
                nodeSeed));

        networkLayerPtr->SetupDhcpServerAndClientIfNecessary(
            theParameterDatabaseReader,
            appLayerPtr);

        ApplicationMaker appMaker(
            simulationEngineInterfacePtr,
            appLayerPtr,
            theNodeId,
            nodeSeed);

        const bool specifiedBasciApplicationFile =
            theParameterDatabaseReader.ParameterExists("basic-applications-file", theNodeId);

        const bool specifiedConfigBasedApplicationFile =
            theParameterDatabaseReader.ParameterExists("config-based-application-file", theNodeId);

        if (specifiedBasciApplicationFile || specifiedConfigBasedApplicationFile) {
            cerr << "\"basic-applications-file\" and \"config-based-application-file\" are old appplication specification parameter." << endl
                 << "Specify applications in \".config\"" << endl
                 << "To convert old application specification of \".app\", use application converter. Usage: bin/update_old_config" << endl;
            exit(1);
        }

        appMaker.ReadApplicationLineFromConfig(
            theParameterDatabaseReader,
            theGlobalNetworkingObjectBag);

    }//if//

}//NetworkNode()

inline
NetworkNode::~NetworkNode()
{
    // Note: Emulation modes may have missing layers.

    if (appLayerPtr != nullptr) {
        appLayerPtr->DisconnectFromOtherLayers();
    }
    if (transportLayerPtr != nullptr) {
        transportLayerPtr->DisconnectProtocolsFromOtherLayers();
    }
    if (networkLayerPtr != nullptr) {
        networkLayerPtr->DisconnectFromOtherLayers();
    }

    // Delete the layers now before shutting down the simulator interface for the node.

    std::weak_ptr<NetworkLayer> checkNetworkLayerPtr = networkLayerPtr;
    std::weak_ptr<TransportLayer> checkTransportLayerPtr = transportLayerPtr;
    std::weak_ptr<ApplicationLayer> checkAppLayerPtr = appLayerPtr;

    networkLayerPtr.reset();
    transportLayerPtr.reset();
    appLayerPtr.reset();

    simulationEngineInterfacePtr->ShutdownThisInterface();

    assert(checkNetworkLayerPtr.expired() && "Memory Leak!");
    assert(checkTransportLayerPtr.expired() && "Memory Leak!");
    assert(checkAppLayerPtr.expired() && "Memory Leak!");

}//~NetworkNode()//


inline
void NetworkNode::CreateDynamicApplication(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const NodeId& sourceNodeId,
    const InterfaceOrInstanceId& instanceId)
{
    ApplicationMaker appMaker(
        simulationEngineInterfacePtr,
        appLayerPtr,
        theNodeId,
        nodeSeed);

    appMaker.ReadSpecificApplicationLineFromConfig(
        theParameterDatabaseReader,
        theGlobalNetworkingObjectBag,
        sourceNodeId,
        instanceId);

}//CreateDynamicApplication//


inline
void NetworkNode::OutputTraceForNodePosition(const SimTime& lastOutputTime) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceMobility)) {

        const ObjectMobilityPosition& nodePosition = GetCurrentLocation();

        if ((lastOutputTime == ZERO_TIME) || (nodePosition.LastMoveTime() > lastOutputTime)) {

            if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

                NodePositionTraceRecord traceData;

                traceData.xPositionMeters = nodePosition.X_PositionMeters();
                traceData.yPositionMeters = nodePosition.Y_PositionMeters();
                traceData.theHeightFromGroundMeters = nodePosition.HeightFromGroundMeters();
                traceData.attitudeAzimuthDegrees = nodePosition.AttitudeAzimuthFromNorthClockwiseDegrees();
                traceData.attitudeElevationDegrees = nodePosition.AttitudeElevationFromHorizonDegrees();

                assert(sizeof(traceData) == NODE_POSITION_TRACE_RECORD_BYTES);

                simulationEngineInterfacePtr->OutputTraceInBinary(
                    "Node", "", "NodePosition", traceData);

            }
            else {
                ostringstream msgStream;

                msgStream << "X= " << nodePosition.X_PositionMeters();
                msgStream << " Y= " << nodePosition.Y_PositionMeters();
                msgStream << " Z= " << nodePosition.HeightFromGroundMeters();
                msgStream << " Azm= " << nodePosition.AttitudeAzimuthFromNorthClockwiseDegrees();
                msgStream << " Elv= " << nodePosition.AttitudeElevationFromHorizonDegrees();

                simulationEngineInterfacePtr->OutputTrace("Node", "", "NodePosition", msgStream.str());

            }//if//

        }//if//

    }//if//

}//OutputTraceForNodePosition//


inline
void NetworkNode::OutputTraceForAddNode() const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceMobility)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {
            simulationEngineInterfacePtr->OutputTraceInBinary("Node", "", "AddNode");
        }
        else {
            simulationEngineInterfacePtr->OutputTrace("Node", "", "AddNode", "");
        }//if//
    }//if//
}//OutputTraceForAddNode//


inline
void NetworkNode::OutputTraceForDeleteNode() const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceMobility)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {
            simulationEngineInterfacePtr->OutputTraceInBinary("Node", "", "DeleteNode");
        }
        else {
            simulationEngineInterfacePtr->OutputTrace("Node", "", "DeleteNode", "");
        }//if//
    }//if//
}//OutputTraceForAddNode//



//=============================================================================

void OutputConfigBasedAppFile(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& fileName);


//=============================================================================
/*
//dcc
void DynamicDcc(){
    cout << "Hello" << endl;
};
//20210520
*/
//=============================================================================

class NetworkSimulator {
public:
    NetworkSimulator(
        const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
        const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
        const RandomNumberGeneratorSeed& runSeed,
        const bool initRunSequentially = true);

    virtual ~NetworkSimulator();
    void DeleteAllNodes() { nodes.clear(); }

    void GetListOfNodeIds(vector<NodeId>& nodeIds);

    virtual NetworkAddress LookupNetworkAddress(const NodeId& theNodeId) const;
    virtual void LookupNetworkAddress(
        const NodeId& theNodeId, NetworkAddress& networkAddress, bool& success) const;

    virtual NodeId LookupNodeId(const NetworkAddress& aNetworkAddress) const;
    virtual void LookupNodeId(
        const NetworkAddress& aNetworkAddress, NodeId& theNodeId, bool& success) const;

    virtual unsigned int LookupInterfaceIndex(const NodeId& theNodeId, const InterfaceId& interfaceName) const;

    virtual void CreateNewNode(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const string& nodeTypeName = "") { assert(false); abort(); }

    virtual void CreateNewNode(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const shared_ptr<ObjectMobilityModel>& nodeMobilityModelPtr,
        const string& nodeTypeName = "") { assert(false); abort(); }

    virtual void DeleteNode(const NodeId& theNodeId) { (*this).RemoveNode(theNodeId); }

    void InsertApplicationIntoANode(const NodeId& theNodeId, const shared_ptr<Application>& appPtr);

    shared_ptr<MacLayerInterfaceForEmulation> GetMacLayerInterfaceForEmulation(
        const NodeId& theNodeId) const;

    const GlobalNetworkingObjectBag& GetGlobalNetworkingObjectBag() const
        { return theGlobalNetworkingObjectBag; }

    virtual void CalculatePathlossFromNodeToLocation(
        const NodeId& theNodeId,
        const PropagationInformationType& informationType,
        const unsigned int interfaceIndex,
        const double& positionXMeters,
        const double& positionYMeters,
        const double& positionZMeters,
        PropagationStatisticsType& propagationStatistics);

    virtual void CalculatePathlossFromNodeToNode(
        const NodeId& txNodeId,
        const NodeId& rxNodeId,
        const PropagationInformationType& informationType,
        const unsigned int txInterfaceIndex,
        const unsigned int rxInterfaceIndex,
        PropagationStatisticsType& propagationStatistics);

    virtual void OutputNodePositionsInXY(
        const SimTime lastOutputTime,
        std::ostream& nodePositionOutStream) const;

    virtual void OutputTraceForAllNodePositions(
        const SimTime& lastOutputTime) const;

    void OutputAllNodeIds(std::ostream& outStream) const;
    void OutputRecentlyAddedNodeIdsWithTypes(std::ostream& outStream);
    void OutputRecentlyDeletedNodeIds(std::ostream& outStream);

    void RunSimulationUntil(const SimTime& simulateUpToTime);

    void AddPropagationCalculationTraceIfNecessary(
        const InterfaceId& channelId,
        const shared_ptr<SimplePropagationLossCalculationModel>& propagationCalculationModelPtr);

    virtual const ObjectMobilityPosition GetNodePosition(const NodeId& theNodeId)
    {   return (nodes[theNodeId]->GetCurrentLocation()); }

    virtual const ObjectMobilityPosition GetAntennaLocation(const NodeId& theNodeId, const unsigned int interfaceIndex);

    virtual void TriggerApplication(const NodeId& theNodeId)
    {
        typedef map<NodeId, shared_ptr<NetworkNode> >::iterator IterType;
        IterType iter = nodes.find(theNodeId);

        if (iter != nodes.end()) {
            iter->second->TriggerApplication();
        }//if//

    }//TriggerApplication//

    SimTime GetTimeStepEventSynchronizationStep() const { return timeStepEventSynchronizationStep; }

    RandomNumberGeneratorSeed GetMobilitySeed() const { return (mobilitySeed); }

    void AddNode(const shared_ptr<NetworkNode>& aNodePtr);
    void RemoveNode(const NodeId& theNodeId);

protected:
    virtual void CompleteSimulatorConstruction();
    virtual bool SupportMultiAgent() const { return false; }

    void SetupStatOutputFile();

    void CheckTheNecessityOfMultiAgentSupport();

    virtual void ExecuteTimestepBasedEvent();

    shared_ptr<SimulationEngine> theSimulationEnginePtr;

    GlobalNetworkingObjectBag theGlobalNetworkingObjectBag;

    shared_ptr<ParameterDatabaseReader> theParameterDatabaseReaderPtr;
    shared_ptr<GisSubsystem> theGisSubsystemPtr;
    shared_ptr<SensingSubsystem> theSensingSubsystemPtr;
    InorderFileCache mobilityFileCache;

    SimTime timeStepEventSynchronizationStep;

    map<NodeId, shared_ptr<NetworkNode> > nodes;

    class NodeEnterEvent : public SimulationEvent {
    public:
        NodeEnterEvent(
            NetworkSimulator* initNetworkSimulator,
            const NodeId& initNodeId,
            const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr)
            :
            networkSimulator(initNetworkSimulator),
            theNodeId(initNodeId),
            nodeMobilityModelPtr(initNodeMobilityModelPtr)
        {}
        virtual void ExecuteEvent() {
            networkSimulator->CreateNewNode(
                *networkSimulator->theParameterDatabaseReaderPtr, theNodeId, nodeMobilityModelPtr);
        }

    private:
        NetworkSimulator* networkSimulator;
        NodeId theNodeId;
        shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr;
    };

    class NodeLeaveEvent : public SimulationEvent {
    public:
        NodeLeaveEvent(
            NetworkSimulator* initNetworkSimulator,
            const NodeId& initNodeId)
            :
            networkSimulator(initNetworkSimulator),
            theNodeId(initNodeId)
        {}
        virtual void ExecuteEvent() { networkSimulator->DeleteNode(theNodeId); }

    private:
        NetworkSimulator* networkSimulator;
        NodeId theNodeId;
    };


    RandomNumberGeneratorSeed runSeed; //seed for communication system
    RandomNumberGeneratorSeed mobilitySeed;

private:

    vector<NodeId> recentlyAddedNodeList;
    vector<NodeId> recentlyDeletedNodeList;

    StatViewCollection fileControlledStatViews;
    string statsOutputFilename;
    bool noDataOutputIsEnabled;

    bool runSequentially;
    unsigned int nextSynchronizationTimeStep;

    struct PropagationTraceOutputInfo {
        InterfaceId channelId;
        shared_ptr<SimplePropagationLossCalculationModel> propagationCalculationModelPtr;
        string outputFileName;

        PropagationTraceOutputInfo(
            const InterfaceId& initChannelId,
            const shared_ptr<SimplePropagationLossCalculationModel>& initPropagationCalculationModelPtr,
            const string& initOutputFileName)
            :
            channelId(initChannelId),
            propagationCalculationModelPtr(initPropagationCalculationModelPtr),
            outputFileName(initOutputFileName)
        {}
    };
    vector<PropagationTraceOutputInfo> propagationTraceOutputInfos;

    void OutputPropagationTrace(const SimTime& time);

};//NetworkSimulator//


inline
NetworkSimulator::NetworkSimulator(
    const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const RandomNumberGeneratorSeed& initRunSeed,
    const bool initRunSequentially)
    :
    theSimulationEnginePtr(initSimulationEnginePtr),
    runSeed(initRunSeed),
    mobilitySeed(initRunSeed),
    theParameterDatabaseReaderPtr(initParameterDatabaseReaderPtr),
    theGisSubsystemPtr(new GisSubsystem(*theParameterDatabaseReaderPtr, initSimulationEnginePtr)),
    timeStepEventSynchronizationStep(INFINITE_TIME),
    runSequentially(initRunSequentially),
    nextSynchronizationTimeStep(0)
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    theGisSubsystemPtr->SynchronizeTopology(ZERO_TIME);

    theSensingSubsystemPtr.reset(
        new SensingSubsystem(
            shared_ptr<NetworkSimulatorInterfaceForSensingSubsystem>(
                new NetworkSimulatorInterfaceForSensingSubsystem(this)),
            theGisSubsystemPtr,
            runSeed)),

    theGlobalNetworkingObjectBag.networkAddressLookupInterfacePtr =
        shared_ptr<NetworkAddressLookupInterface>(new ExtrasimulationNetAddressLookup(this));

    theGlobalNetworkingObjectBag.abstractNetworkPtr =
        shared_ptr<AbstractNetwork>(new AbstractNetwork(theParameterDatabaseReader));

    theGlobalNetworkingObjectBag.bitOrBlockErrorRateCurveDatabasePtr.reset(
            new BitOrBlockErrorRateCurveDatabase());

    theGlobalNetworkingObjectBag.sensingSubsystemInterfacePtr =
        theSensingSubsystemPtr->CreateSubsystemInterfacePtr();

    string antennaFileName;
    if (theParameterDatabaseReader.ParameterExists("custom-antenna-file")) {
        antennaFileName = theParameterDatabaseReader.ReadString("custom-antenna-file");
    }//if//

    bool useLegacyAntennaPatternFileFormatMode = false;

    if (theParameterDatabaseReader.ParameterExists("antenna-patterns-are-in-legacy-format")) {
        useLegacyAntennaPatternFileFormatMode =
            theParameterDatabaseReader.ReadBool("antenna-patterns-are-in-legacy-format");
    }//if//

    string antennaPatternDebugDumpFileName;
    if (theParameterDatabaseReader.ParameterExists("antenna-pattern-debug-dump-file")) {
        antennaPatternDebugDumpFileName=
            theParameterDatabaseReader.ReadString("antenna-pattern-debug-dump-file");
    }//if//

    int two2dTo3dInterpolationAlgorithmNumber = 1;
    if (theParameterDatabaseReader.ParameterExists(
        "antenna-pattern-two-2d-to-3d-interpolation-algorithm-number")) {

        two2dTo3dInterpolationAlgorithmNumber =
            theParameterDatabaseReader.ReadInt(
                "antenna-pattern-two-2d-to-3d-interpolation-algorithm-number");
    }//if//

    theGlobalNetworkingObjectBag.antennaPatternDatabasePtr.reset(
        new AntennaPatternDatabase(
            antennaFileName,
            useLegacyAntennaPatternFileFormatMode,
            two2dTo3dInterpolationAlgorithmNumber,
            antennaPatternDebugDumpFileName));

    if (theParameterDatabaseReader.ParameterExists("time-step-event-synchronization-step")) {
        timeStepEventSynchronizationStep = theParameterDatabaseReader.ReadTime("time-step-event-synchronization-step");
    }//if//

    (*this).SetupStatOutputFile();

    if (theParameterDatabaseReader.ParameterExists("mobility-seed")) {
        mobilitySeed = theParameterDatabaseReader.ReadInt("mobility-seed");
    }//if//

}//NetworkSimulator//

inline
void NetworkSimulator::AddNode(const shared_ptr<NetworkNode>& aNodePtr)
{
    const NodeId theNodeId = aNodePtr->GetNodeId();

    assert(nodes.find(theNodeId) == nodes.end());
    nodes.insert(make_pair(theNodeId, aNodePtr));
    recentlyAddedNodeList.push_back(theNodeId);
    aNodePtr->OutputTraceForAddNode();
    theSensingSubsystemPtr->AddNode(theNodeId);
}//AddNode//

inline
void NetworkSimulator::RemoveNode(const NodeId& theNodeId)
{
    nodes[theNodeId]->OutputTraceForDeleteNode();
    nodes[theNodeId].reset();
    nodes.erase(theNodeId);
    recentlyDeletedNodeList.push_back(theNodeId);

    theGisSubsystemPtr->RemoveMovingObject(theNodeId);
    theSensingSubsystemPtr->RemoveNode(theNodeId);
}//RemoveNode//


inline
void NetworkSimulator::RunSimulationUntil(const SimTime& simulateUpToTime)
{
    if (timeStepEventSynchronizationStep == INFINITE_TIME) {

        // Output positions for the first time step.
        if (theSimulationEnginePtr->CurrentTime() == ZERO_TIME &&
            nextSynchronizationTimeStep == 0) {

            (*this).OutputTraceForAllNodePositions(ZERO_TIME);
            nextSynchronizationTimeStep++;
        }//if//

    } else {

        const unsigned int endSynchronizationTimeStep =
            static_cast<unsigned int>(std::floor(double(simulateUpToTime) / timeStepEventSynchronizationStep));

        const unsigned int startSynchronizationTimeStep = nextSynchronizationTimeStep;

        SimTime lastPositionOutputTime = ZERO_TIME;

        for(unsigned int i = startSynchronizationTimeStep; i <= endSynchronizationTimeStep; i++) {

            const SimTime halfwaySimulationTime = timeStepEventSynchronizationStep * i;

            if (runSequentially) {
                theSimulationEnginePtr->RunSimulationSequentially(halfwaySimulationTime);
            }
            else {
                theSimulationEnginePtr->RunSimulationInParallel(halfwaySimulationTime);
            }//if//

            (*this).ExecuteTimestepBasedEvent();
            //cout << "RunSimulationUntil" << endl;//yes

            (*this).OutputTraceForAllNodePositions(lastPositionOutputTime);

            lastPositionOutputTime = halfwaySimulationTime;
            nextSynchronizationTimeStep = i+1;
        }//for//
    }//if//

    if (runSequentially) {
        theSimulationEnginePtr->RunSimulationSequentially(simulateUpToTime);
    }
    else {
        theSimulationEnginePtr->RunSimulationInParallel(simulateUpToTime);
    }//if//

}//RunSimulationUntil//

inline
void NetworkSimulator::ExecuteTimestepBasedEvent()
{
    const SimTime currentTime = theSimulationEnginePtr->CurrentTime();

    theGisSubsystemPtr->SynchronizeTopology(currentTime);
    //cout << "ExecuteTimestepBasedEvent" << endl;//yes

    theSensingSubsystemPtr->ExecuteTimestepBasedEvent(currentTime);

    (*this).OutputPropagationTrace(currentTime);

}//ExecuteTimestepBasedEvent//

inline
void NetworkSimulator::GetListOfNodeIds(vector<NodeId>& nodeIds)
{
    typedef map<NodeId, shared_ptr<NetworkNode> >::const_iterator IterType;
    nodeIds.clear();

    for(IterType iter = nodes.begin(); (iter != nodes.end()); ++iter) {
        nodeIds.push_back(iter->first);
    }//for//

}//GetListOfNodeIds//


inline
NetworkAddress NetworkSimulator::LookupNetworkAddress(const NodeId& theNodeId) const
{
    typedef map<NodeId, shared_ptr<NetworkNode> >::const_iterator IterType;

    IterType iterPosition = nodes.find(theNodeId);

    if (iterPosition == nodes.end()) {
        cerr << "Error: Network Node Id: " << theNodeId << " Not Found." << endl;
        exit(1);
    }//if//

    return (iterPosition->second->GetPrimaryNetworkAddress());
}


inline
void NetworkSimulator::LookupNetworkAddress(
    const NodeId& theNodeId, NetworkAddress& networkAddress, bool& success) const
{

    typedef map<NodeId, shared_ptr<NetworkNode> >::const_iterator IterType;

    IterType iterPosition = nodes.find(theNodeId);

    if (iterPosition != nodes.end()) {
        success = true;
        networkAddress = (iterPosition->second->GetPrimaryNetworkAddress());
    }
    else {
        success = false;
    }//if//

}


inline
NodeId NetworkSimulator::LookupNodeId(const NetworkAddress& aNetworkAddress) const
{
    typedef map<NodeId, shared_ptr<NetworkNode> >::const_iterator IterType;

    for(IterType iter = nodes.begin(); (iter != nodes.end()); iter++) {
        if (iter->second->GetPrimaryNetworkAddress() == aNetworkAddress) {
            return (iter->first);
        }//if//
    }//for//

    cerr << "Error in NetworkSimulator::LookupNodeId: Network Address "
         << aNetworkAddress.ConvertToString() << " Not Found." << endl;
    exit(1);
}


inline
void NetworkSimulator::LookupNodeId(
    const NetworkAddress& aNetworkAddress, NodeId& theNodeId, bool& success) const
{
    success = false;

    typedef map<NodeId, shared_ptr<NetworkNode> >::const_iterator IterType;

    for(IterType iter = nodes.begin(); (iter != nodes.end()); iter++) {
        if (iter->second->GetPrimaryNetworkAddress() == aNetworkAddress) {
            success = true;
            theNodeId = (iter->first);
            return;
        }//if//
    }//for//
}


inline
unsigned int NetworkSimulator::LookupInterfaceIndex(const NodeId& theNodeId, const InterfaceId& interfaceName) const
{
    typedef map<NodeId, shared_ptr<NetworkNode> >::const_iterator IterType;

    IterType iter = nodes.find(theNodeId);

    if (iter == nodes.end()) {
        cerr << "Error: Network Node Id: " << theNodeId << " Not Found." << endl;
        exit(1);
    }//if

    return iter->second->GetNetworkLayerPtr()->LookupInterfaceIndex(interfaceName);
}

inline
const ObjectMobilityPosition NetworkSimulator::GetAntennaLocation(
    const NodeId& theNodeId,
    const unsigned int interfaceIndex)
{
    ObjectMobilityPosition antennaPosition =
        nodes[theNodeId]->GetAntennaLocation(interfaceIndex);

    if (!antennaPosition.TheHeightContainsGroundHeightMeters()) {
        const Vertex antennaVertex(
            antennaPosition.X_PositionMeters(),
            antennaPosition.Y_PositionMeters(),
            antennaPosition.HeightFromGroundMeters());

        const shared_ptr<const GroundLayer> groundLayerPtr =
            theGisSubsystemPtr->GetGroundLayerPtr();

        const double groundMeters =
            groundLayerPtr->GetElevationMetersAt(antennaVertex);

        antennaPosition.SetHeightFromGroundMeters(
            antennaPosition.HeightFromGroundMeters() + groundMeters);
    }

    return antennaPosition;
}

inline
void NetworkSimulator::InsertApplicationIntoANode(
    const NodeId& theNodeId,
    const shared_ptr<Application>& appPtr)
{
    nodes[theNodeId]->GetAppLayerPtr()->AddApp(appPtr);
}

inline
shared_ptr<MacLayerInterfaceForEmulation>
    NetworkSimulator::GetMacLayerInterfaceForEmulation(const NodeId& theNodeId) const
{
    typedef map<NodeId, shared_ptr<NetworkNode> >::const_iterator IterType;

    IterType iter = nodes.find(theNodeId);

    if (iter == nodes.end()) {
        cerr << "Error: Network Node Id: " << theNodeId << " Not Found." << endl;
        exit(1);
    }//if//

    const NetworkLayer& theNetworkLayer = *iter->second->GetNetworkLayerPtr();

    if (theNetworkLayer.NumberOfInterfaces() != 1) {
        cerr << "Emulation Error: The number of MAC layers for " << theNodeId << " is not equal to 1." << endl;
        exit(1);
    }//if//

    return (theNetworkLayer.GetMacLayerPtr(0)->GetMacLayerInterfaceForEmulation());

}//GetMacLayerInterfaceForEmulation//


inline
NetworkAddress ExtrasimulationNetAddressLookup::LookupNetworkAddress(
    const NodeId& theNodeId) const
{
    return networkSimulatorPtr->LookupNetworkAddress(theNodeId);
}

inline
void ExtrasimulationNetAddressLookup::LookupNetworkAddress(
    const NodeId& theNodeId, NetworkAddress& networkAddress, bool& success) const
{
    networkSimulatorPtr->LookupNetworkAddress(theNodeId, networkAddress, success);
}

inline
NodeId ExtrasimulationNetAddressLookup::LookupNodeId(
    const NetworkAddress& aNetworkAddress) const
{
    return networkSimulatorPtr->LookupNodeId(aNetworkAddress);
}

inline
void ExtrasimulationNetAddressLookup::LookupNodeId(
    const NetworkAddress& aNetworkAddress, NodeId& theNodeId, bool& success) const
{
    return networkSimulatorPtr->LookupNodeId(aNetworkAddress, theNodeId, success);
}

class SimpleAccessPointFinder : public AccessPointFinderInterface {
public:
    void LookupAccessPointFor(
        const NetworkAddress& destinationAddress,
        bool& foundTheAccessPoint,
        NetworkAddress& accessPointAddress);

    void SetAccessPointFor(
        const NetworkAddress& nodeAddress,
        const NetworkAddress& accessPointAddress)
    {
        nodeToAccessPointMap[nodeAddress] = accessPointAddress;
    }

    void ClearAccessPointFor(const NetworkAddress& nodeAddress);


private:
    std::map<NetworkAddress, NetworkAddress> nodeToAccessPointMap;

};//SimpleAccessPointFinder//

inline
void SimpleAccessPointFinder::LookupAccessPointFor(
    const NetworkAddress& destinationAddress,
    bool& foundTheAccessPoint,
    NetworkAddress& accessPointAddress)
{
    typedef std::map<NetworkAddress, NetworkAddress>::iterator IterType;

    foundTheAccessPoint = false;

    IterType iter = nodeToAccessPointMap.find(destinationAddress);
    if (iter != nodeToAccessPointMap.end()) {

        foundTheAccessPoint = true;
        accessPointAddress = iter->second;
    }//if/

}//LookupAccessPointFor//


inline
void SimpleAccessPointFinder::ClearAccessPointFor(const NetworkAddress& nodeAddress)
{
    typedef std::map<NetworkAddress, NetworkAddress>::iterator IterType;

    IterType iter = nodeToAccessPointMap.find(nodeAddress);
    if (iter != nodeToAccessPointMap.end()) {
        nodeToAccessPointMap.erase(iter);
    }//if/

}//ClearAccessPointFor//

/*
//scensim_app_flooding.h + dot11_phy.h
void DccReactive{
    cout << "Hello" << endl;  
}//20210519
*/

}//namespace//


#endif
