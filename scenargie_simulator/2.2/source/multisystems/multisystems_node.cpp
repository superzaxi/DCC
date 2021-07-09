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

SimNode::SimNode(
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const GlobalNetworkingObjectBag& initGlobalNetworkingObjectBag,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeId& initNodeId,
    const RandomNumberGeneratorSeed& initRunSeed,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
    const shared_ptr<AttachedAntennaMobilityModel>& initNodeMobilityModelPtr,
    const shared_ptr<ChannelModelSet>& initChannelModelSetPtr)
    :
    AgentCommunicationNode(
        initParameterDatabaseReader,
        initGlobalNetworkingObjectBag,
        initSimulationEngineInterfacePtr,
        initNodeMobilityModelPtr,
        initNodeId,
        initRunSeed),
    stationType(OTHER),
    platformMobilityModelPtr(initNodeMobilityModelPtr),
    bleHostPtr(nullptr)
{
    (*this).SetupStationType(
        initParameterDatabaseReader);

    // Initialize moving object before channel switching in mac.
    (*this).SetupMovingObject(
        initParameterDatabaseReader,
        initGisSubsystemPtr);

    (*this).SetupInterfaces(
        initParameterDatabaseReader,
        initGlobalNetworkingObjectBag,
        initGisSubsystemPtr,
        initChannelModelSetPtr);

    (*this).SetupNetworkProtocols(
        initParameterDatabaseReader);

}//SimNode()//

void SimNode::SetupStationType(const ParameterDatabaseReader& theParameterDatabaseReader)
{
    if (!theParameterDatabaseReader.ParameterExists((Lte::parameterNamePrefix + "station-type"), theNodeId)) {
        stationType = OTHER;
    }
    else {

        string stationTypeString =
            theParameterDatabaseReader.ReadString((Lte::parameterNamePrefix + "station-type"), theNodeId);

        ConvertStringToLowerCase(stationTypeString);

        if (stationTypeString == "bs") {
            stationType = BS;
        }
        else if ((stationTypeString == "ue") || (stationTypeString == "mobile")) {
            stationType = UE;
            stationTypeString = "mobile";
        }
        else {
            cerr << "Error: " << Lte::parameterNamePrefix << "STATION-TYPE parameter: "
                 << stationTypeString << " for node: " << theNodeId << " is invalid." << endl;
            exit(1);
        }//if//
    }//if//
}//SetupStationType//

void SimNode::Attach(const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr)
{
    platformMobilityModelPtr->SetPlatformMobility(initNodeMobilityModelPtr);
}//Attach//

void SimNode::Detach()
{
    (*this).DisconnectPropInterfaces();
}//Detach//
