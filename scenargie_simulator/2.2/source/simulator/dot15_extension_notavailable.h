// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT15_EXTENSION_NOTAVAILABLE_H
#define DOT15_EXTENSION_NOTAVAILABLE_H

#ifndef DOT15_EXTENSION_CHOOSER_H
#error "Include dot15_extension_chooser.h (indirect include only)"
#endif

#include "scensim_prop.h"
#include "scensim_gis.h"
#include "scensim_mac.h"
#include "scensim_application.h"
#include "scensim_bercurves.h"

namespace Dot15 {

using std::cerr;
using std::endl;
using std::string;
using std::shared_ptr;
using ScenSim::SimplePropagationModelForNode;
using ScenSim::SimulationEngineInterface;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeId;
using ScenSim::InterfaceId;
using ScenSim::InterfaceOrInstanceId;
using ScenSim::RandomNumberGeneratorSeed;
using ScenSim::ApplicationLayer;
using ScenSim::BitOrBlockErrorRateCurveDatabase;
using ScenSim::MacLayer;

class Dot15Phy {
public:
    struct PropFrame {};

    Dot15Phy(
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<PropFrame> >& propModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const RandomNumberGeneratorSeed& nodeSeed)
    {
        cerr << "Error: DOT15 module is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables DOT15 Module." << endl;
        exit(1);
   }
};//Dot11Phy//

class Dot15Mac : public MacLayer {
public:
    typedef Dot15Phy::PropFrame PropFrame;

    Dot15Mac(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<PropFrame> >& propModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
        const NodeId& initNodeId,
        const InterfaceId& initInterfaceId,
        const RandomNumberGeneratorSeed& nodeSeed)
    {
        cerr << "Error: DOT15 module is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables DOT15 Module." << endl;
        exit(1);
    }

    void NetworkLayerQueueChangeNotification() {}
    void DisconnectFromOtherLayers() {}

};//Dot15Mac//

class Dot15AbstractTransportProtocol {
public:
    static const string modelName;

    Dot15AbstractTransportProtocol(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr)
    {
        cerr << "Error: DOT15 module is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables DOT15 Module." << endl;
        exit(1);
    }

    void ConnectToMacLayer(const shared_ptr<Dot15Mac>& macLayerPtr){};

};//Dot15AbstractTransportProtocol//

class Dot15ApplicationMaker {
public:

    Dot15ApplicationMaker(
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<ApplicationLayer>& appLayerPtr,
        const shared_ptr<Dot15AbstractTransportProtocol> initAptPtr,
        const NodeId& theNodeId,
        const RandomNumberGeneratorSeed& nodeSeed)
    {
        cerr << "Error: DOT15 module is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables DOT15 Module." << endl;
        exit(1);
    }

    void ReadApplicationLineFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader){};

};//Dot15ApplicationMaker//



}//namespace//


#endif
