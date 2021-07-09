// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>

#include "scenargiesim.h"
#include "scensim_gui_interface.h"
#include "scensim_mobility.h"
#include "scensim_routing_support.h"
#include "dot11_mac.h"

using std::string;
using std::shared_ptr;
using std::make_shared;
using std::map;
using std::cerr;
using std::endl;
using std::ofstream;

using ScenSim::NodeId;
using ScenSim::InterfaceOrInstanceId;
using ScenSim::ParameterDatabaseReader;
using ScenSim::SimulationEngine;
using ScenSim::SimulationEngineInterface;
using ScenSim::SimTime;

using ScenSim::NetworkSimulator;
using ScenSim::NetworkNode;
using ScenSim::InterfaceId;
using ScenSim::SimplePropagationModel;
using ScenSim::SimplePropagationModelForNode;
using ScenSim::SimplePropagationLossCalculationModel;
using ScenSim::MimoChannelModel;
using ScenSim::MimoChannelModelInterface;
using ScenSim::FrequencySelectiveFadingModel;
using ScenSim::FrequencySelectiveFadingModelInterface;
using ScenSim::RandomNumberGeneratorSeed;
using ScenSim::AntennaModel;
using ScenSim::AttachedAntennaMobilityModel;
using ScenSim::ObjectMobilityModel;
using ScenSim::ObjectMobilityPosition;
using ScenSim::RandomNumberGeneratorSeed;
using ScenSim::GlobalNetworkingObjectBag;
using ScenSim::GisSubsystem;
using ScenSim::PropagationInformationType;
using ScenSim::PropagationStatisticsType;
using ScenSim::MacLayer;
using ScenSim::MakeLowerCaseString;
using ScenSim::ConvertTimeToStringSecs;
using ScenSim::SquaredXYDistanceBetweenVertices;
using ScenSim::Vertex;
using ScenSim::GrabArgvForLicensing;
using ScenSim::MainFunctionArgvProcessingBasicParallelVersion1;
using ScenSim::GuiInterfacingSubsystem;
using ScenSim::ConvertToUShortInt;


class SimNode;

typedef Dot11::Dot11Phy::PropFrame FrameType;
typedef SimplePropagationModel<FrameType> PropagationModelType;
typedef SimplePropagationModelForNode<FrameType> PropagationInterfaceType;

class Dot11Simulator : public NetworkSimulator {
public:
    Dot11Simulator(
        const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
        const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
        const RandomNumberGeneratorSeed& initRunSeed,
        const bool initRunSequentially);

    ~Dot11Simulator() { (*this).DeleteAllNodes(); }

    virtual void CreateNewNode(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const shared_ptr<ObjectMobilityModel>& nodeMobilityModelPtr,
        const string& nodeTypeName = "");

    shared_ptr<PropagationModelType> GetPropagationModel(
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId);

    void GetChannelModel(
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int baseChannelNumber,
        const unsigned int numberChannels,
        const vector<double>& channelCarrierFrequecies,
        const vector<double>& channelBandwidthMhz,
        shared_ptr<MimoChannelModel>& mimoChannelModelPtr,
        shared_ptr<FrequencySelectiveFadingModel>& frequencySelectiveFadingModelPtr);

private:
    struct PropAndChannelInfoType {
        shared_ptr<PropagationModelType> propagationModelPtr;
        shared_ptr<MimoChannelModel> mimoChannelModelPtr;
        shared_ptr<FrequencySelectiveFadingModel> frequencySelectiveFadingModelPtr;
    };

    map<InterfaceOrInstanceId, PropAndChannelInfoType> propAndChannelInfoMap;

};//Dot11Simulator//


inline
Dot11Simulator::Dot11Simulator(
    const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const RandomNumberGeneratorSeed& initRunSeed,
    const bool initRunSequentially)
    :
    NetworkSimulator(
        initParameterDatabaseReaderPtr,
        initSimulationEnginePtr,
        initRunSeed,
        initRunSequentially)
{
    const string berCurveFileName =
        initParameterDatabaseReaderPtr->ReadString("dot11-bit-error-rate-curve-file");

    (*this).theGlobalNetworkingObjectBag.bitOrBlockErrorRateCurveDatabasePtr->LoadBerCurveFile(
        berCurveFileName);

    // Call inherited implementation:

    (*this).CompleteSimulatorConstruction();

}//Dot11Simulator()//




class SimNode : public NetworkNode {
public:
    SimNode(
        const ParameterDatabaseReader& initParameterDatabaseReader,
        const GlobalNetworkingObjectBag& globalNetworkingObjectBag,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const NodeId& theNodeId,
        const RandomNumberGeneratorSeed& runSeed,
        const shared_ptr<ObjectMobilityModel>& nodeMobilityModelPtr);

    void CompleteInitializationAndConnectPropModels(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        Dot11Simulator& theDot11Simulator);

    ~SimNode() {}

    virtual const ObjectMobilityPosition GetCurrentLocation() const {
        const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
        ObjectMobilityPosition position;
        nodeMobilityModelPtr->GetPositionForTime(currentTime, position);
        return (position);
    }

    virtual void CalculatePathlossToLocation(
        const PropagationInformationType& informationType,
        const unsigned int interfaceNumber,
        const double& positionXMeters,
        const double& positionYMeters,
        const double& positionZMeters,
        PropagationStatisticsType& propagationStatistics) const;

    virtual void CalculatePathlossToNode(
        const PropagationInformationType& informationType,
        const unsigned int interfaceIndex,
        const ObjectMobilityPosition& rxAntennaPosition,
        const ObjectMobilityModel::MobilityObjectId& rxObjectId,
        const AntennaModel& rxAntennaModel,
        PropagationStatisticsType& propagationStatistics) const;

    virtual bool HasAntenna(const InterfaceId& channelId) const {

        typedef  map<InterfaceId, unsigned int>::const_iterator IterType;
        IterType iter = interfaceNumberToChannel.find(channelId);

        if (iter == interfaceNumberToChannel.end()) {
            return false;
        }

        return (interfaces.at((*iter).second).antennaModelPtr != nullptr);
    }//HasAntenna//

    virtual shared_ptr<AntennaModel> GetAntennaModelPtr(const unsigned int interfaceIndex) const {
        return interfaces.at(interfaceIndex).antennaModelPtr;
    }

    virtual ObjectMobilityPosition GetAntennaLocation(const unsigned int interfaceIndex) const {

        const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
        ObjectMobilityPosition position;
        interfaces.at(interfaceIndex).antennaMobilityModelPtr->GetPositionForTime(
            currentTime, position);
        return (position);

    }//GetAntennaLocation//

    void DisconnectPropInterfaces();

    void SetupMacAndPhyInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceNumber,
        Dot11Simulator& theDot11Simulator);

private:
    const shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr;

    struct InterfaceType {
        shared_ptr<MacLayer> macPtr;
        shared_ptr<AntennaModel> antennaModelPtr;
        shared_ptr<ObjectMobilityModel> antennaMobilityModelPtr;
        shared_ptr<PropagationInterfaceType> propagationInterfacePtr;
        shared_ptr<MimoChannelModelInterface> mimoChannelModelInterfacePtr;
        shared_ptr<FrequencySelectiveFadingModelInterface> frequencySelectiveFadingModelInterfacePtr;
    };

    map<InterfaceId, unsigned int> interfaceNumberToChannel;
    vector<InterfaceType> interfaces;
};



inline
SimNode::SimNode(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeId& initNodeId,
    const RandomNumberGeneratorSeed& initRunSeed,
    const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr)
    :
    NetworkNode(
        theParameterDatabaseReader,
        theGlobalNetworkingObjectBag,
        initSimulationEngineInterfacePtr,
        initNodeMobilityModelPtr,
        initNodeId,
        initRunSeed),
    nodeMobilityModelPtr(initNodeMobilityModelPtr)
{
}


inline
void SimNode::CompleteInitializationAndConnectPropModels(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const shared_ptr<GisSubsystem>& gisSubsystemPtr,
    Dot11Simulator& theDot11Simulator)
{
    const unsigned int numberInterfaces = networkLayerPtr->NumberOfInterfaces();

    for(unsigned int i = 0; i < numberInterfaces; i++) {

        const InterfaceId theInterfaceId = networkLayerPtr->GetInterfaceId(i);

        (*this).SetupMacAndPhyInterface(
            theParameterDatabaseReader,
            theGlobalNetworkingObjectBag,
            theInterfaceId,
            i,
            theDot11Simulator);

    }//for//

    // Add interface routing protocols from here. -------------------

    SetupRoutingProtocol(
        theParameterDatabaseReader,
        theNodeId,
        simulationEngineInterfacePtr,
        networkLayerPtr,
        nodeSeed);

    // Add (routing)/transport/application protocols from here. -----


    // --------------------------------------------------------------

    gisSubsystemPtr->EnableMovingObjectIfNecessary(
        theParameterDatabaseReader,
        theNodeId,
        simulationEngineInterfacePtr->CurrentTime(),
        nodeMobilityModelPtr);

}//CompleteInitializationAndConnectPropModels//




void SimNode::SetupMacAndPhyInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const InterfaceId& theInterfaceId,
    const unsigned int interfaceNumber,
    Dot11Simulator& theDot11Simulator)
{
    interfaceNumberToChannel[theInterfaceId] = interfaceNumber;
    while (interfaces.size() <= interfaceNumber) {
        interfaces.push_back(InterfaceType());
    }

    if (InterfaceIsWired(theParameterDatabaseReader, theNodeId, theInterfaceId) ||
        IsNoInterface(theParameterDatabaseReader, theNodeId, theInterfaceId)) {
        // no mac protocol
        // Abstract Network has virtual connections between their network layers.
        return;
    }//if//

    // check dot11 interface
    const string macProtocol = MakeLowerCaseString(
        theParameterDatabaseReader.ReadString("mac-protocol", theNodeId, theInterfaceId));

    if (macProtocol != "dot11") {
        cerr << "Error: bad mac-protocol format: " << macProtocol
             << " for node: " << theNodeId << endl
             << "only \"dot11\" is available." << endl;
        exit(1);
    }//if//

    InterfaceType& interface = interfaces.at(interfaceNumber);

    // Antenna Model
    interface.antennaModelPtr =
        CreateAntennaModel(
            theParameterDatabaseReader,
            theNodeId,
            theInterfaceId,
            *theGlobalNetworkingObjectBag.antennaPatternDatabasePtr);

    // Antenna Mobility
    if (AttachedAntennaMobilityModel::AntennaIsAttachedAntenna(
            theParameterDatabaseReader, theNodeId, theInterfaceId)) {

        interface.antennaMobilityModelPtr =
            make_shared<AttachedAntennaMobilityModel>(
                theParameterDatabaseReader,
                theNodeId,
                theInterfaceId,
                nodeMobilityModelPtr);
    }
    else {
        interface.antennaMobilityModelPtr = nodeMobilityModelPtr;
    }//if//

    const shared_ptr<PropagationModelType> propagationModelPtr =
        theDot11Simulator.GetPropagationModel(theNodeId, theInterfaceId);

    // Propagation Interface
    interface.propagationInterfacePtr =
        propagationModelPtr->GetNewPropagationModelInterface(
            simulationEngineInterfacePtr,
            interface.antennaModelPtr,
            interface.antennaMobilityModelPtr,
            theNodeId,
            theInterfaceId,
            interfaceNumber);

    // Create optional channel model interface.

    shared_ptr<MimoChannelModel> mimoChannelModelPtr;
    shared_ptr<FrequencySelectiveFadingModel> frequencySelectiveFadingModelPtr;

    theDot11Simulator.GetChannelModel(
        theNodeId,
        theInterfaceId,
        propagationModelPtr->GetBaseChannelNumber(),
        propagationModelPtr->GetChannelCount(),
        GetListOfCarrierFreqenciesMhz(*propagationModelPtr),
        GetListOfChannelBandwidthsMhz(*propagationModelPtr),
        mimoChannelModelPtr,
        frequencySelectiveFadingModelPtr);

    if (mimoChannelModelPtr != nullptr) {
        mimoChannelModelPtr->CreateNewMimoChannelModelInterface(
            theParameterDatabaseReader,
            theNodeId,
            theInterfaceId,
            interface.antennaMobilityModelPtr,
            interface.mimoChannelModelInterfacePtr);
    }//if//

    if (frequencySelectiveFadingModelPtr != nullptr) {
        assert(mimoChannelModelPtr == nullptr);

        frequencySelectiveFadingModelPtr->GetNewFadingModelInterface(
            theNodeId,
            interface.frequencySelectiveFadingModelInterfacePtr);
    }//if//

    // MAC protocol
    interface.macPtr =
        Dot11::Dot11Mac::Create(
            theParameterDatabaseReader,
            simulationEngineInterfacePtr,
            interface.propagationInterfacePtr,
            interface.mimoChannelModelInterfacePtr,
            interface.frequencySelectiveFadingModelInterfacePtr,
            theGlobalNetworkingObjectBag.bitOrBlockErrorRateCurveDatabasePtr,
            theNodeId,
            theInterfaceId,
            interfaceNumber,
            networkLayerPtr,
            nodeSeed);

    networkLayerPtr->SetInterfaceMacLayer(interfaceNumber, interface.macPtr);

    //cout << "A" << endl;//20210526

}//SetupMacAndPhyInterface//



void SimNode::DisconnectPropInterfaces()
{
    for(unsigned int i = 0; i < interfaces.size(); i++) {
        interfaces[i].propagationInterfacePtr->DisconnectThisInterface();
    }//for//

    interfaces.clear();

}//DisconnectPropInterfaces//


void SimNode::CalculatePathlossToLocation(
    const PropagationInformationType& informationType,
    const unsigned int interfaceNumber,
    const double& positionXMeters,
    const double& positionYMeters,
    const double& positionZMeters,
    PropagationStatisticsType& propagationStatistics) const
{
    interfaces.at(interfaceNumber).
        propagationInterfacePtr->CalculatePathlossToLocation(
            informationType,
            positionXMeters,
            positionYMeters,
            positionZMeters,
            propagationStatistics);
}//CalculatePathlossToLocation//



void SimNode::CalculatePathlossToNode(
    const PropagationInformationType& informationType,
    const unsigned int interfaceNumber,
    const ObjectMobilityPosition& rxAntennaPosition,
    const ObjectMobilityModel::MobilityObjectId& rxObjectId,
    const AntennaModel& rxAntennaModel,
    PropagationStatisticsType& propagationStatistics) const
{
    const shared_ptr<PropagationInterfaceType> propagationInterfacePtr =
        interfaces.at(interfaceNumber).propagationInterfacePtr;

    const shared_ptr<PropagationModelType> propagationModelPtr =
        propagationInterfacePtr->GetPropagationModel();

    const vector<unsigned int>& channelNumberSet = propagationInterfacePtr->GetCurrentChannelNumberSet();

    unsigned int channelNumber;

    if (channelNumberSet.empty()) {
        channelNumber = propagationInterfacePtr->GetCurrentChannelNumber();
    }
    else {
        channelNumber = channelNumberSet.front();
    }//if//

    propagationModelPtr->CalculatePathlossToNode(
            informationType,
            propagationInterfacePtr->GetCurrentMobilityPosition(),
            theNodeId,
            *(*this).GetAntennaModelPtr(interfaceNumber),
            rxAntennaPosition,
            rxObjectId,
            rxAntennaModel,
            channelNumber,
            propagationStatistics);

}//CalculatePathlossToNode//


shared_ptr<PropagationModelType> Dot11Simulator::GetPropagationModel(
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId)
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    assert(!InterfaceIsWired(theParameterDatabaseReader, theNodeId, theInterfaceId));

    const string propModelInstanceName = GetChannelInstanceId(theParameterDatabaseReader, theNodeId, theInterfaceId);

    if (propAndChannelInfoMap.find(propModelInstanceName) == propAndChannelInfoMap.end()) {

        shared_ptr<PropagationModelType> propagationModelPtr(
            new PropagationModelType(
                (*theParameterDatabaseReaderPtr),
                runSeed,
                theSimulationEnginePtr,
                theGisSubsystemPtr,
                propModelInstanceName));

        (*this).propAndChannelInfoMap[propModelInstanceName].propagationModelPtr = propagationModelPtr;

        (*this).AddPropagationCalculationTraceIfNecessary(
            propModelInstanceName,
            propagationModelPtr->GetPropagationCalculationModel());
    }//if//

    return (propAndChannelInfoMap[propModelInstanceName].propagationModelPtr);

}//GetPropagationModel//


void Dot11Simulator::GetChannelModel(
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId,
    const unsigned int baseChannelNumber,
    const unsigned int numberChannels,
    const vector<double>& channelCarrierFrequenciesMhz,
    const vector<double>& channelBandwidthsMhz,
    shared_ptr<MimoChannelModel>& mimoChannelModelPtr,
    shared_ptr<ScenSim::FrequencySelectiveFadingModel>& frequencySelectiveFadingModelPtr)
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    assert(!InterfaceIsWired(theParameterDatabaseReader, theNodeId, theInterfaceId));

    const string channelModelInstanceName = GetChannelInstanceId(theParameterDatabaseReader, theNodeId, theInterfaceId);

    assert((propAndChannelInfoMap.find(channelModelInstanceName) != propAndChannelInfoMap.end()) &&
           "Propagation model must be created before channel model.");

    PropAndChannelInfoType& propAndChannelInfo = (*this).propAndChannelInfoMap[channelModelInstanceName];

    if ((propAndChannelInfo.mimoChannelModelPtr != nullptr) ||
        (propAndChannelInfo.frequencySelectiveFadingModelPtr != nullptr)) {

        mimoChannelModelPtr = propAndChannelInfo.mimoChannelModelPtr;
        frequencySelectiveFadingModelPtr = propAndChannelInfo.frequencySelectiveFadingModelPtr;
    }
    else {
        Dot11::ChooseAndCreateChannelModel(
            (*theParameterDatabaseReaderPtr),
            channelModelInstanceName,
            baseChannelNumber,
            numberChannels,
            channelCarrierFrequenciesMhz,
            channelBandwidthsMhz,
            runSeed,
            *propAndChannelInfo.propagationModelPtr,
            mimoChannelModelPtr,
            frequencySelectiveFadingModelPtr);

        propAndChannelInfo.mimoChannelModelPtr = mimoChannelModelPtr;
        propAndChannelInfo.frequencySelectiveFadingModelPtr = frequencySelectiveFadingModelPtr;
    }//if//

    assert((mimoChannelModelPtr == nullptr) || (frequencySelectiveFadingModelPtr == nullptr) &&
           "Channel models are mutually exclusive");

}//GetChannelModel//



void Dot11Simulator::CreateNewNode(
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
    }

    const shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr =
        theSimulationEnginePtr->GetSimulationEngineInterface(
            theParameterDatabaseReader, theNodeId, partitionIndex);

    shared_ptr<SimNode> aSimNodePtr(
        new SimNode(
            theParameterDatabaseReader,
            theGlobalNetworkingObjectBag,
            simulationEngineInterfacePtr,
            theNodeId,
            runSeed,
            nodeMobilityModelPtr));

    // Initialization separated from constructor to avoid a modifible parameter on constructor
    // (this "simulator" object).

    aSimNodePtr->CompleteInitializationAndConnectPropModels(
            theParameterDatabaseReader,
            theGlobalNetworkingObjectBag,
            theGisSubsystemPtr,
            (*this));

    (*this).AddNode(aSimNodePtr);

}//CreateNewNode//


//
//-------------------- main --------------------
//

int main(int argc, char* argv[])
{
    GrabArgvForLicensing(argc, argv);

    string configFileName;
    bool isControlledByGui;
    unsigned int numberParallelThreads;
    bool runSequentially;
    bool seedIsSet;
    RandomNumberGeneratorSeed runSeed;

    //dcc
    //double channelBusyRatio;
    //Dot11::Dot11Mac dm;
    //20210526

    MainFunctionArgvProcessingBasicParallelVersion1(
        argc,
        argv,
        configFileName,
        isControlledByGui,
        numberParallelThreads,
        runSequentially,
        seedIsSet,
        runSeed);

    shared_ptr<ParameterDatabaseReader> theParameterDatabaseReaderPtr(
        new ParameterDatabaseReader(configFileName));
    ParameterDatabaseReader& theParameterDatabaseReader =
        *theParameterDatabaseReaderPtr;

    if (!seedIsSet) {
        if (!theParameterDatabaseReader.ParameterExists("seed")) {
            cerr << "Error: No seed parameter found." << endl;
            exit(1);
        }
        runSeed = theParameterDatabaseReader.ReadInt("seed");
    }

    shared_ptr<SimulationEngine> theSimulationEnginePtr(
        new SimulationEngine(
            theParameterDatabaseReader,
            runSequentially,
            numberParallelThreads));

    Dot11Simulator theNetworkSimulator(
        theParameterDatabaseReaderPtr,
        theSimulationEnginePtr,
        runSeed,
        runSequentially);

    if (isControlledByGui) {

        GuiInterfacingSubsystem guiInterface(
            theSimulationEnginePtr,
            ConvertToUShortInt(
                theParameterDatabaseReader.ReadNonNegativeInt("gui-portnumber-sim"),
                "Error in parameter gui-portnumber-sim: port number is too large."),
            ConvertToUShortInt(
                theParameterDatabaseReader.ReadNonNegativeInt("gui-portnumber-pausecommand"),
                "Error in parameter gui-portnumber-pausecommand: port number is too large."));

        while (true) {
            SimTime simulateUpToTime;

            guiInterface.GetAndExecuteGuiCommands(
                theParameterDatabaseReader,
                theNetworkSimulator,
                simulateUpToTime);

            if (theSimulationEnginePtr->SimulationIsDone()) {
                break;
            }//if//

            theNetworkSimulator.RunSimulationUntil(simulateUpToTime);

        }//while//

    } else {
        const SimTime endSimTime = theParameterDatabaseReader.ReadTime("simulation-time");

        theNetworkSimulator.RunSimulationUntil(endSimTime);

    }//if//

    return 0;

}//main//




