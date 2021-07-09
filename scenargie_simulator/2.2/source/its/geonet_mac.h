// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef GEONET_MAC_H
#define GEONET_MAC_H

#include "scensim_engine.h"
#include "scensim_netsim.h"

#include "geonet_dcc.h"
#include "its_queues.h"

namespace GeoNet {

using ScenSim::SimpleMacPacketHandler;
using ScenSim::PacketId;
using ScenSim::Packet;
using ScenSim::SimTime;
using ScenSim::MacLayer;
using ScenSim::NetworkLayer;
using ScenSim::NetworkAddress;
using ScenSim::RealStatistic;
using ScenSim::ConvertToNonDb;
using ScenSim::RandomNumberGeneratorSeed;
using ScenSim::PacketPriority;
using ScenSim::SimulationEvent;
using ScenSim::EtherTypeField;
using ScenSim::ETHERTYPE_GEONET;
using ScenSim::EnqueueResultType;

using Dot11::Dot11Phy;
using Dot11::Dot11Mac;
using Dot11::CongestionMonitoringHandler;
using Dot11::ItsOutputQueueWithPrioritySubqueues;
using Dot11::DatarateBitsPerSec;

using std::unique_ptr;
using std::move;


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

class GeoNetMac : public MacLayer {
public:
    static const string dccModelName;

    GeoNetMac(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeId& initNodeId,
        const InterfaceId& initInterfaceId,
        const unsigned int initInterfaceIndex,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const shared_ptr<Dot11Phy>& initDot11PhyPtr,
        const RandomNumberGeneratorSeed& nodeSeed);


    void NetworkLayerQueueChangeNotification();
    void DisconnectFromOtherLayers();

    void SetGeoNetPacketHandler(
        const shared_ptr<SimpleMacPacketHandler>& geoNetPacketHandlerPtr);

    void InsertPacketIntoAnOutputQueue(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriority trafficClass,
        const EtherTypeField& etherType,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDropPtr);

private:
    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;

    shared_ptr<GeoNetCongestionMonitor> congestionMonitorPtr;

    SimTime controlLoopInterval;
    void PeriodicCongestionControlExecution() const;

    //---------------------------------
    class ControlLoopEvent : public SimulationEvent {
        public:
            ControlLoopEvent(GeoNetMac* initGeoNetMacPtr)
                :
                geoNetMacPtr(initGeoNetMacPtr)
            {}
            void ExecuteEvent()
            {
                geoNetMacPtr->PeriodicCongestionControlExecution();
            }

        private:
            GeoNetMac* geoNetMacPtr;
    };
    //---------------------------------

    shared_ptr<ControlLoopEvent> controlLoopEventPtr;

    void ScheduleControlLoopEvent() const;

    shared_ptr<TransmitDatarateController> datarateControllerPtr;
    shared_ptr<TransmitPowerController> powerControllerPtr;
    shared_ptr<DCCSensitivityController> sensitivityControllerPtr;

    //Dot11Mac interface
    shared_ptr<ItsOutputQueueWithPrioritySubqueues> outputQueuePtr;
    shared_ptr<Dot11Mac> dot11MacPtr;

    shared_ptr<Dot11Phy> dot11PhyPtr;

    //statistics
    shared_ptr<RealStatistic> transmitDatarateBpsStatPtr;
    shared_ptr<RealStatistic> transmitPowerDbmStatPtr;
    shared_ptr<RealStatistic> ccaThresholdDbmStatPtr;

};//GeoNetMac//


inline
GeoNetMac::GeoNetMac(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeId& initNodeId,
    const InterfaceId& initInterfaceId,
    const unsigned int initInterfaceIndex,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const shared_ptr<Dot11Phy>& initDot11PhyPtr,
    const RandomNumberGeneratorSeed& nodeSeed)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    controlLoopInterval(INFINITE_TIME),
    dot11PhyPtr(initDot11PhyPtr),
    transmitDatarateBpsStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
        dccModelName + '_' + initInterfaceId + "_TxDatarateBps")),
    transmitPowerDbmStatPtr(
        simulationEngineInterfacePtr->CreateRealStatWithDbConversion(
        dccModelName + '_' + initInterfaceId + "_TxPowerDbm")),
    ccaThresholdDbmStatPtr(
        simulationEngineInterfacePtr->CreateRealStatWithDbConversion(
        dccModelName + '_' + initInterfaceId + "_CCAThresholdDbm"))

{
    dot11MacPtr =
        Dot11Mac::Create(
            theParameterDatabaseReader,
            initSimulationEngineInterfacePtr,
            initNodeId,
            initInterfaceId,
            initInterfaceIndex,
            initNetworkLayerPtr,
            initDot11PhyPtr,
            nodeSeed);

    //create network queue
    (*this).outputQueuePtr.reset(
        new ItsOutputQueueWithPrioritySubqueues(
            theParameterDatabaseReader,
            initInterfaceId,
            initSimulationEngineInterfacePtr,
            dot11MacPtr->GetMaxPacketPriority()));

    dot11MacPtr->SetNetworkOutputQueue(outputQueuePtr);
    initNetworkLayerPtr->SetInterfaceOutputQueue(initInterfaceIndex, outputQueuePtr);

    //set congestion monitor
    congestionMonitorPtr.reset(
        new GeoNetCongestionMonitor(
            initSimulationEngineInterfacePtr,
            theParameterDatabaseReader,
            initNodeId,
            initInterfaceId));
    dot11MacPtr->SetCongestionMonitoringHandler(congestionMonitorPtr);

    //TDC
    datarateControllerPtr.reset(
        new TransmitDatarateController(
            theParameterDatabaseReader,
            initNodeId,
            initInterfaceId));

    //TPC
    powerControllerPtr.reset(
        new TransmitPowerController(
            theParameterDatabaseReader,
            initNodeId,
            initInterfaceId));

    //DCC Sensitivity Control
    sensitivityControllerPtr.reset(
        new DCCSensitivityController(
            theParameterDatabaseReader,
            initNodeId,
            initInterfaceId));

    //control loop
    if ((theParameterDatabaseReader.ParameterExists(
            "its-geonet-dcc-control-loop-interval", initNodeId, initInterfaceId)) &&
        (sensitivityControllerPtr->IsEnabled())) {

        controlLoopInterval =
            theParameterDatabaseReader.ReadTime(
                "its-geonet-dcc-control-loop-interval", initNodeId, initInterfaceId);

        controlLoopEventPtr = shared_ptr<ControlLoopEvent>(new ControlLoopEvent(this));

        //execute initial loop
        PeriodicCongestionControlExecution();

    }//if//

 }


inline
void GeoNetMac::SetGeoNetPacketHandler(
    const shared_ptr<SimpleMacPacketHandler>& geoNetPacketHandlerPtr)
{

    dot11MacPtr->SetMacPacketHandler(ETHERTYPE_GEONET, geoNetPacketHandlerPtr);

}//SetGeoNetPacketHandler//


inline
void GeoNetMac::ScheduleControlLoopEvent() const
{

    const SimTime nextControlTime =
        simulationEngineInterfacePtr->CurrentTime() + controlLoopInterval;

    simulationEngineInterfacePtr->ScheduleEvent(
        controlLoopEventPtr, nextControlTime);

}//ScheduleControlLoopEvent//


inline
void GeoNetMac::PeriodicCongestionControlExecution() const
{

    const double congestionValue = congestionMonitorPtr->GetCurrentCongestionValue();

    const double newCCAThresholdDbm =
        sensitivityControllerPtr->GetCCAThresholdDbm(congestionValue);

    ccaThresholdDbmStatPtr->RecordStatValue(ConvertToNonDb(newCCAThresholdDbm));

    dot11PhyPtr->UpdateCcaEnergyDetectionThresholdDbm(newCCAThresholdDbm);

    (*this).ScheduleControlLoopEvent();

}//PeriodicCongestionControlExecution//


inline
void GeoNetMac::InsertPacketIntoAnOutputQueue(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriority trafficClass,
    const EtherTypeField& etherType,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDropPtr)
{

    const double congestionValue = congestionMonitorPtr->GetCurrentCongestionValue();

    const DatarateBitsPerSec theDatarateBitsPerSec =
        datarateControllerPtr->GetDatarateBps(congestionValue);

    transmitDatarateBpsStatPtr->RecordStatValue(
        static_cast<double>(theDatarateBitsPerSec));

    const double txPowerDbm =
        powerControllerPtr->GetTxPowerDbm(congestionValue);

    transmitPowerDbmStatPtr->RecordStatValue(ConvertToNonDb(txPowerDbm));

    (*outputQueuePtr).InsertWithEtherTypeAndDatarateAndTxPower(
        packetPtr,
        nextHopAddress,
        trafficClass,
        ETHERTYPE_GEONET,
        theDatarateBitsPerSec,
        txPowerDbm,
        enqueueResult,
        packetToDropPtr);

    dot11MacPtr->NetworkLayerQueueChangeNotification();

}//InsertPacketIntoAnOutputQueue//


inline
void GeoNetMac::NetworkLayerQueueChangeNotification()
{

    dot11MacPtr->NetworkLayerQueueChangeNotification();

}//NetworkLayerQueueChangeNotification//


inline
void GeoNetMac::DisconnectFromOtherLayers()
{
    dot11MacPtr->DisconnectFromOtherLayers();
    dot11PhyPtr.reset();

}//DisconnectFromOtherLayers//


}//namespace//

#endif
