// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef BLE_EXTENSION_NOTAVAILABLE_H
#define BLE_EXTENSION_NOTAVAILABLE_H

#ifndef BLE_EXTENSION_CHOOSER_H
#error "Include ble_extension_chooser.h (indirect include only)"
#endif

#include <iostream>
#include <memory>

#include "scensim_prop.h"
#include "scensim_application.h"

namespace Ble {

using std::cerr;
using std::endl;
using std::shared_ptr;
using ScenSim::SimplePropagationModelForNode;
using ScenSim::SimulationEngineInterface;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeIdType;
using ScenSim::InterfaceIdType;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::ApplicationLayer;
using ScenSim::BitOrBlockErrorRateCurveDatabase;
using ScenSim::MacLayer;
using ScenSim::NetworkLayer;

class BlePhy {
public:
    struct PropFrame {};

};//BlePhy//

class BleHost : public MacLayer {
public:

    BleHost(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<BlePhy::PropFrame> >& propagationModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const NodeIdType& initNodeId,
        const InterfaceIdType& interfaceId,
        const RandomNumberGeneratorSeedType& nodeSeed)
    {
        cerr << "Error: BLE module is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables BLE Module." << endl;
        exit(1);
    }

    void NetworkLayerQueueChangeNotification() {}
    void DisconnectFromOtherLayers() {}

};//BleHost//

class BleApplicationMaker {
public:

    BleApplicationMaker(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<ApplicationLayer>& initAppLayerPtr,
        const shared_ptr<BleHost>& initBleHostPtr,
        const NodeIdType& initNodeId,
        const RandomNumberGeneratorSeedType& initNodeSeed)
    {
        cerr << "Error: BLE module is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables BLE Module." << endl;
        exit(1);
    }

    void ReadApplicationLineFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader) {}

};//BleApplicationMaker//


}//namespace//

#endif
