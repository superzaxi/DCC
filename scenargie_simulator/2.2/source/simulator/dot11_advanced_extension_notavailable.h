// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11_ADVANCED_EXTENSION_NOTAVAILABLE_H
#define DOT11_ADVANCED_EXTENSION_NOTAVAILABLE_H

#ifndef DOT11_ADVANCED_EXTENSION_CHOOSER_H
#error "Include dot11_advanced extension_chooser.h (indirect include only)"
#endif

#include "scensim_proploss.h"
#include "scensim_prop.h"
#include "scensim_mimo_channel.h"
#include "scensim_freqsel_channel.h"
#include "scensim_gis.h"
#include "scensim_mac.h"
#include "scensim_bercurves.h"

namespace Dot11ad {

using std::cerr;
using std::endl;
using std::string;
using std::shared_ptr;
using ScenSim::SimplePropagationModelForNode;
using ScenSim::SimulationEngineInterface;
using ScenSim::ParameterDatabaseReader;
using ScenSim::AntennaPatternDatabase;
using ScenSim::CustomAntennaModel;
using ScenSim::NodeId;
using ScenSim::InterfaceId;
using ScenSim::RandomNumberGeneratorSeed;
using ScenSim::NetworkLayer;
using ScenSim::BitOrBlockErrorRateCurveDatabase;
using ScenSim::MacLayer;
using ScenSim::MimoChannelModelInterface;
using ScenSim::FrequencySelectiveFadingModelInterface;
using ScenSim::InterfaceOrInstanceId;
using ScenSim::MimoChannelModel;
using ScenSim::FrequencySelectiveFadingModel;


class Dot11Phy {
public:
    struct PropFrame {};

};//Dot11Phy//

class Dot11Mac : public MacLayer {
public:
    static shared_ptr<Dot11Mac> Create(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const AntennaPatternDatabase& theAntennaPatternDatabase,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<Dot11Phy::PropFrame> >& propModelInterfacePtr,
        const shared_ptr<MimoChannelModelInterface>& mimoChannelModelInterfacePtr,
        const shared_ptr<FrequencySelectiveFadingModelInterface>& frequencySelectiveFadingModelInterfacePtr,
        const shared_ptr<CustomAntennaModel>& antennaModelPtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const RandomNumberGeneratorSeed& nodeSeed)
    {
        cerr << "Error: DOT11 Advanced module is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables DOT11 Advanced Module." << endl;
        exit(1);
    }

};//Dot11Mac//

inline
void ChooseAndCreateChannelModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceOrInstanceId& propagationModeInstanceId,
    const unsigned int baseChannelNumber,
    const unsigned int numberChannels,
    const vector<double>& channelCarrierFrequenciesMhz,
    const vector<double>& channelBandwidthsMhz,
    const RandomNumberGeneratorSeed& runSeed,
    shared_ptr<MimoChannelModel>& mimoChannelModelPtr,
    shared_ptr<FrequencySelectiveFadingModel>& frequencySelectiveFadingModelPtr)
{
}//ChooseAndCreateChannelModel//

}//namespace//


namespace Dot11ah {

using std::cerr;
using std::endl;
using std::string;
using std::shared_ptr;
using ScenSim::SimplePropagationModelForNode;
using ScenSim::SimulationEngineInterface;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeId;
using ScenSim::InterfaceId;
using ScenSim::RandomNumberGeneratorSeed;
using ScenSim::NetworkLayer;
using ScenSim::BitOrBlockErrorRateCurveDatabase;
using ScenSim::MacLayer;

class Dot11Phy {
public:
    struct PropFrame {};

};//Dot11Phy//

class Dot11Mac : public MacLayer {
public:
    static shared_ptr<Dot11Mac> Create(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<Dot11Phy::PropFrame> >& propModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const RandomNumberGeneratorSeed& nodeSeed)
    {
        cerr << "Error: DOT11 Advanced module is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables DOT11 Advanced Module." << endl;
        exit(1);
    }

};//Dot11Mac//

}//namespace//


#endif
