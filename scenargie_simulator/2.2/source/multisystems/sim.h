// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SIM_H
#define SIM_H

#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <set>
#include <vector>

#include "scenargiesim.h"
#include "scensim_network.h"
#include "scensim_application.h"

#include "aloha_mac.h"

#include "lte_extension_chooser.h"
#include "its_extension_chooser.h"
#include "dot11_extension_chooser.h"
#include "dot11_advanced_extension_chooser.h"
#include "dot15_extension_chooser.h"
#include "ble_extension_chooser.h"
#include "multiagent_extension_chooser.h"

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::set;
using std::map;
using std::pair;
using std::make_pair;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::move;
using std::istringstream;

using ScenSim::InterfaceOrInstanceId;
using ScenSim::ParameterDatabaseReader;
using ScenSim::SimulationEngine;
using ScenSim::SimulationEngineInterface;
using ScenSim::GlobalNetworkingObjectBag;
using ScenSim::GisSubsystem;
using ScenSim::RandomNumberGeneratorSeed;
using ScenSim::BitOrBlockErrorRateCurveDatabase;
using ScenSim::NodeId;
using ScenSim::NetworkAddress;
using ScenSim::ObjectMobilityModel;
using ScenSim::AttachedAntennaMobilityModel;
using ScenSim::ObjectMobilityPosition;
using ScenSim::PropagationInformationType;
using ScenSim::PropagationStatisticsType;
using ScenSim::MimoChannelModel;
using ScenSim::MimoChannelModelInterface;
using ScenSim::FrequencySelectiveFadingModel;
using ScenSim::FrequencySelectiveFadingModelInterface;
using ScenSim::AntennaModel;
using ScenSim::InterfaceId;
using ScenSim::SimulationEvent;
using ScenSim::SimTime;
using ScenSim::ZERO_TIME;
using ScenSim::MILLI_SECOND;
using ScenSim::SECOND;
using ScenSim::INFINITE_TIME;
using ScenSim::NetworkSimulator;
using ScenSim::NetworkNode;
using ScenSim::MacLayer;
using ScenSim::MakeLowerCaseString;
using ScenSim::ConvertStringToLowerCase;
using ScenSim::ConvertTimeToStringSecs;
using ScenSim::SquaredXYDistanceBetweenVertices;
using ScenSim::Vertex;
using ScenSim::MAX_COMMUNICATION_NODEID;
using ScenSim::INVALID_NODEID;
using ScenSim::ApplicationId;
using ScenSim::ConvertStringToInt;
using ScenSim::nullInstanceId;
using ScenSim::PI;
using ScenSim::Packet;
using ScenSim::PacketPriority;
using ScenSim::EtherTypeField;
using ScenSim::EnqueueResultType;

using MultiAgent::MultiAgentSimulator;
using MultiAgent::AgentCommunicationNode;

class MultiSystemsSimulator;
class SimNode;

namespace Aloha {
    typedef ScenSim::SimplePropagationModel<AlohaPhy::PropFrame> PropagationModel;
    typedef SimplePropagationModelForNode<AlohaPhy::PropFrame> PropagationModelInterface;
};

namespace Dot11 {
    typedef ScenSim::SimplePropagationModel<Dot11Phy::PropFrame> PropagationModel;
    typedef SimplePropagationModelForNode<Dot11Phy::PropFrame> PropagationModelInterface;
};

namespace Dot11ad {
    typedef ScenSim::SimplePropagationModel<Dot11Phy::PropFrame> PropagationModel;
    typedef SimplePropagationModelForNode<Dot11Phy::PropFrame> PropagationModelInterface;
};

namespace Dot11ah {
    typedef ScenSim::SimplePropagationModel<Dot11Phy::PropFrame> PropagationModel;
    typedef SimplePropagationModelForNode<Dot11Phy::PropFrame> PropagationModelInterface;
};

namespace T109 {
    typedef ScenSim::SimplePropagationModel<T109Phy::PropFrame> PropagationModel;
    typedef SimplePropagationModelForNode<T109Phy::PropFrame> PropagationModelInterface;
};

namespace Wave {
    class WsmpLayer;
};

namespace Dot15 {
    typedef ScenSim::SimplePropagationModel<Dot15Phy::PropFrame> PropagationModel;
    typedef SimplePropagationModelForNode<Dot15Phy::PropFrame> PropagationModelInterface;
};

namespace Ble {
    typedef ScenSim::SimplePropagationModel<BlePhy::PropFrame> PropagationModel;
    typedef SimplePropagationModelForNode<BlePhy::PropFrame> PropagationModelInterface;
};

typedef InterfaceOrInstanceId ChannelInstanceIdType;

//----------------------------------------------------------------------------------------

class ChannelModelSet {
public:
    ChannelModelSet(
        MultiSystemsSimulator* initSimulatorPtr,
        const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
        const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        const RandomNumberGeneratorSeed& initRunSeed)
        :
        simulatorPtr(initSimulatorPtr),
        theParameterDatabaseReaderPtr(initParameterDatabaseReaderPtr),
        theSimulationEnginePtr(initSimulationEnginePtr),
        theGisSubsystemPtr(initGisSubsystemPtr),
        runSeed(initRunSeed)
    {}

    shared_ptr<Aloha::PropagationModel> GetAlohaPropagationModel(const InterfaceId& theInterfaceId);
    shared_ptr<Dot11::PropagationModel> GetDot11PropagationModel(const InterfaceId& theInterfaceId);
    shared_ptr<Dot11ad::PropagationModel> GetDot11adPropagationModel(const InterfaceId& theInterfaceId);
    shared_ptr<Dot11ah::PropagationModel> GetDot11ahPropagationModel(const InterfaceId& theInterfaceId);
    shared_ptr<T109::PropagationModel> GetT109PropagationModel(const InterfaceId& theInterfaceId);
    shared_ptr<Dot15::PropagationModel> GetDot15PropagationModel(const InterfaceId& theInterfaceId);
    shared_ptr<Ble::PropagationModel> GetBlePropagationModel(const InterfaceId& interfaceId);
    shared_ptr<Lte::DownlinkPropagationModel > GetLteDownlinkPropagationModel(const InterfaceId& theInterfaceId);
    shared_ptr<Lte::UplinkPropagationModel > GetLteUplinkPropagationModel(const InterfaceId& theInterfaceId);

    void GetDot11ChannelModel(
        const InterfaceId& theInterfaceId,
        const unsigned int baseChannelNumber,
        const unsigned int numberChannels,
        const vector<double>& channelCarrierFrequenciesMhz,
        const vector<double>& channelBandwidthsMhz,
        shared_ptr<MimoChannelModel>& mimoChannelModelPtr,
        shared_ptr<FrequencySelectiveFadingModel>& frequencySelectiveFadingModelPtr);

    void GetDot11adChannelModel(
        const InterfaceId& theInterfaceId,
        const unsigned int baseChannelNumber,
        const unsigned int numberChannels,
        const vector<double>& channelCarrierFrequenciesMhz,
        const vector<double>& channelBandwidthsMhz,
        shared_ptr<MimoChannelModel>& mimoChannelModelPtr,
        shared_ptr<FrequencySelectiveFadingModel>& frequencySelectiveFadingModelPtr);

    shared_ptr<Lte::LteGlobalParameters> GetLteGlobals();

    void GetLteMimoOrFadingModelPtr(
        const ChannelInstanceIdType& downlinkInstanceId,
        const ChannelInstanceIdType& uplinkInstanceId,
        shared_ptr<ScenSim::MimoChannelModel>& downlinkMimoChannelModelPtr,
        shared_ptr<ScenSim::MimoChannelModel>& uplinkMimoChannelModelPtr,
        shared_ptr<ScenSim::FrequencySelectiveFadingModel>& downlinkFrequencySelectiveFadingModelPtr,
        shared_ptr<ScenSim::FrequencySelectiveFadingModel>& uplinkFrequencySelectiveFadingModelPtr);

    bool Dot11PropagationModelExists(const InterfaceOrInstanceId& instanceId) const {
        return (dot11PropagationModelPtrs.find(instanceId) != dot11PropagationModelPtrs.end());
    }
    bool BlePropagationModelExists(const InterfaceOrInstanceId& instanceId) const {
        return (blePropagationModelPtrs.find(instanceId) != blePropagationModelPtrs.end());
    }
    bool LteDownlinkPropagationModelExists(const InterfaceOrInstanceId& instanceId) const {
        return (lteDownlinkPropagationModelPtrs.find(instanceId) != lteDownlinkPropagationModelPtrs.end());
    }

    shared_ptr<BitOrBlockErrorRateCurveDatabase> GetDot11BitOrBlockErrorRateCurveDatabase();
    shared_ptr<BitOrBlockErrorRateCurveDatabase> GetDot11AdBitOrBlockErrorRateCurveDatabase();
    shared_ptr<BitOrBlockErrorRateCurveDatabase> GetDot11AhBitOrBlockErrorRateCurveDatabase();

    shared_ptr<BitOrBlockErrorRateCurveDatabase> GetT109BitOrBlockErrorRateCurveDatabase();
    shared_ptr<BitOrBlockErrorRateCurveDatabase> GetDot15BitOrBlockErrorRateCurveDatabase();
    shared_ptr<BitOrBlockErrorRateCurveDatabase> GetBleBitOrBlockErrorRateCurveDatabase();
    shared_ptr<BitOrBlockErrorRateCurveDatabase> GetLteBitOrBlockErrorRateCurveDatabase();

private:
    MultiSystemsSimulator* simulatorPtr;
    shared_ptr<ParameterDatabaseReader> theParameterDatabaseReaderPtr;
    shared_ptr<SimulationEngine> theSimulationEnginePtr;
    shared_ptr<GisSubsystem> theGisSubsystemPtr;
    RandomNumberGeneratorSeed runSeed;

    map<ChannelInstanceIdType, shared_ptr<Aloha::PropagationModel> > alohaPropagationModelPtrs;
    map<ChannelInstanceIdType, shared_ptr<Dot11::PropagationModel> > dot11PropagationModelPtrs;
    map<ChannelInstanceIdType, shared_ptr<Dot11ad::PropagationModel> > dot11adPropagationModelPtrs;
    map<ChannelInstanceIdType, shared_ptr<Dot11ah::PropagationModel> > dot11ahPropagationModelPtrs;
    map<ChannelInstanceIdType, shared_ptr<T109::PropagationModel> > t109PropagationModelPtrs;
    map<ChannelInstanceIdType, shared_ptr<Dot15::PropagationModel> > dot15PropagationModelPtrs;
    map<ChannelInstanceIdType, shared_ptr<Ble::PropagationModel> > blePropagationModelPtrs;
    map<ChannelInstanceIdType, shared_ptr<Lte::UplinkPropagationModel> > lteUplinkPropagationModelPtrs;
    map<ChannelInstanceIdType, shared_ptr<Lte::DownlinkPropagationModel> > lteDownlinkPropagationModelPtrs;

    struct ChannelModelInfoType {
        shared_ptr<MimoChannelModel> mimoChannelModelPtr;
        shared_ptr<FrequencySelectiveFadingModel> frequencySelectiveFadingModelPtr;
    };

    map<ChannelInstanceIdType, ChannelModelInfoType> channelModelInfoMap;


    struct LteChannelInfoType {
        ChannelInstanceIdType uplinkChannelInstanceId;
        shared_ptr<ScenSim::MimoChannelModel> downlinkMimoChannelModelPtr;
        shared_ptr<ScenSim::MimoChannelModel> uplinkMimoChannelModelPtr;
        shared_ptr<ScenSim::FrequencySelectiveFadingModel> downlinkFrequencySelectiveFadingModelPtr;
        shared_ptr<ScenSim::FrequencySelectiveFadingModel> uplinkFrequencySelectiveFadingModelPtr;
    };

    map<ChannelInstanceIdType, LteChannelInfoType> lteChannelModelInfos;

    shared_ptr<BitOrBlockErrorRateCurveDatabase> dot11BitOrBlockErrorRateCurveDatabasePtr;
    shared_ptr<BitOrBlockErrorRateCurveDatabase> dot11AdBitOrBlockErrorRateCurveDatabasePtr;
    shared_ptr<BitOrBlockErrorRateCurveDatabase> dot11AhBitOrBlockErrorRateCurveDatabasePtr;

    shared_ptr<BitOrBlockErrorRateCurveDatabase> dot15BitOrBlockErrorRateCurveDatabasePtr;
    shared_ptr<BitOrBlockErrorRateCurveDatabase> bleBitOrBlockErrorRateCurveDatabasePtr;
    shared_ptr<BitOrBlockErrorRateCurveDatabase> lteBitOrBlockErrorRateCurveDatabasePtr;

    shared_ptr<Lte::LteGlobalParameters> lteGlobalsPtr;

};//ChannelModelSet//

class MultiSystemsSimulator : public MultiAgentSimulator {
public:
    MultiSystemsSimulator(
        const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
        const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
        const RandomNumberGeneratorSeed& initRunSeed,
        const bool initRunSequentially,
        const bool initIsScenarioSettingOutputMode,
        const string& initInputConfigFileName,
        const string& initOutputConfigFileName);

    ~MultiSystemsSimulator() { (*this).DeleteAllNodes(); }

    virtual void CreateNewNode(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const shared_ptr<ObjectMobilityModel>& nodeMobilityModelPtr,
        const string& nodeTypeName = "");

    virtual void DeleteNode(const NodeId& theNodeId);
    virtual unsigned int LookupInterfaceIndex(
        const NodeId& theNodeId,
        const InterfaceId& interfaceName) const;

protected:
    virtual void CompleteSimulatorConstruction();

    virtual void SetupInterChannelModelInterference(
        const ParameterDatabaseReader& theParameterDatabaseReader);

private:
    shared_ptr<ChannelModelSet> channelModelSetPtr;

    map<NodeId, shared_ptr<SimNode> > simNodePtrs;

};//MultiSystemsSimulator//

class SimNode : public AgentCommunicationNode {
public:
    SimNode(
        const ParameterDatabaseReader& initParameterDatabaseReader,
        const GlobalNetworkingObjectBag& initGlobalNetworkingObjectBag,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeId& initNodeId,
        const RandomNumberGeneratorSeed& initRunSeed,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        const shared_ptr<AttachedAntennaMobilityModel>& initNodeMobilityModelPtr,
        const shared_ptr<ChannelModelSet>& initChannelModelSetPtr);

    ~SimNode() {}

    virtual const ObjectMobilityPosition GetCurrentLocation() const {
        const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
        ObjectMobilityPosition position;
        nodeMobilityModelPtr->GetPositionForTime(currentTime, position);
        return (position);
    }//GetCurrentLocation//

    virtual void CalculatePathlossToLocation(
        const PropagationInformationType& informationType,
        const unsigned int antennaNumber,
        const double& positionXMeters,
        const double& positionYMeters,
        const double& positionZMeters,
        PropagationStatisticsType& propagationStatistics) const;

    virtual void CalculatePathlossToNode(
        const PropagationInformationType& informationType,
        const unsigned int interfaceIndex,
        const ObjectMobilityPosition& rxAntennaPosition,
        const ObjectMobilityModel::MobilityObjectId& rxNodeId,
        const AntennaModel& rxAntennaModel,
        PropagationStatisticsType& propagationStatistics) const;

    virtual bool HasAntenna(const InterfaceId& channelId) const;
    virtual shared_ptr<AntennaModel> GetAntennaModelPtr(const unsigned int interfaceIndex) const;
    virtual ObjectMobilityPosition GetAntennaLocation(const unsigned int interfaceIndex) const;
    unsigned int GetAntennaNumber(const InterfaceId& interfaceName);

    shared_ptr<ObjectMobilityModel> GetPlatformMobilityModel() { return platformMobilityModelPtr->GetPlatformMobility(); }

    void DisconnectPropInterfaces();

    void SetupInterfaces(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const shared_ptr<GisSubsystem>& gisSubsystemPtr,
        const shared_ptr<ChannelModelSet>& channelModelSetPtr);

private:
    virtual void Attach(const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr);
    virtual void Detach();

    enum StationType { BS, UE, OTHER };
    StationType stationType;

    struct AntennaType {
        ChannelInstanceIdType channelModelId;
        unsigned int antennaNumber;

        shared_ptr<AntennaModel> antennaModelPtr;
        shared_ptr<ObjectMobilityModel> antennaMobilityModelPtr;

        shared_ptr<Aloha::PropagationModelInterface> alohaPropagationModelInterfacePtr;
        shared_ptr<Dot11::PropagationModelInterface> dot11PropagationModelInterfacePtr;
        shared_ptr<Dot11ad::PropagationModelInterface> dot11adPropagationModelInterfacePtr;
        shared_ptr<Dot11ah::PropagationModelInterface> dot11ahPropagationModelInterfacePtr;
        shared_ptr<T109::PropagationModelInterface> t109PropagationModelInterfacePtr;
        shared_ptr<Dot15::PropagationModelInterface> dot15PropagationModelInterfacePtr;
        shared_ptr<Ble::PropagationModelInterface> blePropagationModelInterfacePtr;
        shared_ptr<Lte::DownlinkPropagationModelInterface> lteDownlinkPropagationModelInterfacePtr;
        shared_ptr<Lte::UplinkPropagationModelInterface> lteUplinkPropagationModelInterfacePtr;

        shared_ptr<MimoChannelModelInterface> mimoChannelModelInterfacePtr;
        shared_ptr<FrequencySelectiveFadingModelInterface> frequencySelectiveFadingModelInterfacePtr;

        AntennaType(const ChannelInstanceIdType& initChannelModelId)
            :
            channelModelId(initChannelModelId),
            antennaNumber(static_cast<unsigned int>(-1))
        {}
    };

    struct InterfaceType {
        shared_ptr<Wave::WsmpLayer> wsmpLayerPtr;

        shared_ptr<MacLayer> macPtr;
        vector<shared_ptr<AntennaType> > antennaPtrs;
    };

    shared_ptr<AttachedAntennaMobilityModel> platformMobilityModelPtr;

    vector<InterfaceType> interfaces;

    // For Gateway only.
    shared_ptr<Lte::LteGatewayController> gatewayControllerPtr;

    void SetupStationType(const ParameterDatabaseReader& theParameterDatabaseReader);

    void SetupNetworkProtocols(
        const ParameterDatabaseReader& theParameterDatabaseReader);

    void SetupGatewayInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader);

    void SetupMovingObject(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<GisSubsystem>& gisSubsystemPtr);

    shared_ptr<ObjectMobilityModel> GetNodeOrAttachedMobility(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceId& theInterfaceId);

    void SetupWiredInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex);

    void SetupAlohaInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<ChannelModelSet>& channelModelSetPtr,
        InterfaceType& interfaceToBeInitialized);

    void SetupDot11Interface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<ChannelModelSet>& channelModelSetPtr,
        InterfaceType& interfaceToBeInitialized);

    void SetupDot11adInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<ChannelModelSet>& channelModelSetPtr,
        InterfaceType& interfaceToBeInitialized);

    void SetupDot11ahInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<ChannelModelSet>& channelModelSetPtr,
        InterfaceType& interfaceToBeInitialized);

    void SetupDot15Interface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<ChannelModelSet>& channelModelSetPtr,
        InterfaceType& interfaceToBeInitialized);

    shared_ptr<Ble::BleHost> bleHostPtr;
    void SetupBleInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const InterfaceId& interfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<ChannelModelSet>& channelModelSetPtr,
        InterfaceType& interfaceToBeInitialized);

    void SetupLteInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<ChannelModelSet>& channelModelSetPtr,
        InterfaceType& interfaceToBeInitialized);

    void SetupWaveInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<ChannelModelSet>& channelModelSetPtr,
        InterfaceType& interfaceToBeInitialized);

    void SetupGeoNetInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<ChannelModelSet>& channelModelSetPtr,
        InterfaceType& interfaceToBeInitialized);

    void SetupT109Interface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const InterfaceId& theInterfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<ChannelModelSet>& channelModelSetPtr,
        InterfaceType& interfaceToBeInitialized);

    void CompleteAntennaNumberAssignment();

    shared_ptr<AntennaType> GetAntennaPtr(const unsigned int antennaNumber) const;
};//SimNode//

#endif
