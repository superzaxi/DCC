// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef GEONET_DCC_H
#define GEONET_DCC_H

#include "dot11_mac.h"

namespace GeoNet {

using std::map;
using std::make_pair;
using std::shared_ptr;
using std::cerr;
using std::endl;

using ScenSim::SimulationEngineInterface;
using ScenSim::SimTime;
using ScenSim::INFINITE_TIME;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeId;
using ScenSim::InterfaceId;
using ScenSim::Packet;
using ScenSim::PacketId;
using ScenSim::ConvertAStringSequenceOfRealValuedPairsIntoAMap;


using Dot11::Dot11Mac;
using Dot11::CongestionMonitoringHandler;
using Dot11::DatarateBitsPerSec;

//Decentralized Congestion Control
//ETSI 102 687 v1.1.1 (2011-07)
//Transmit Datarate Control (TDC)
//Transmit Power Control (TPC)
//DCC Sensitivity Control (DSC)


class GeoNetCongestionMonitor : public CongestionMonitoringHandler {
public:
    GeoNetCongestionMonitor(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& initNodeId,
        const InterfaceId& initInterfaceId);

    void ScanAReceivedFrame(
        const Packet& aFrame,
        const SimTime& scannedTime);

    double GetCurrentCongestionValue();

private:
    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;

    SimTime congestionMonitoringTimeoutTime;
    unsigned int maxVehicles;

    map<NodeId, SimTime> transmitterNodeIdRecords;

    void ClearObsoleteRecords();

};//GeoNetCongestionMonitor//


inline
GeoNetCongestionMonitor::GeoNetCongestionMonitor(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& initNodeId,
    const InterfaceId& initInterfaceId)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    congestionMonitoringTimeoutTime(INFINITE_TIME),
    maxVehicles(UINT_MAX)
{

    if (theParameterDatabaseReader.ParameterExists(
            "its-geonet-congestion-monitoring-timeout-time", initNodeId, initInterfaceId)) {
        congestionMonitoringTimeoutTime =
            theParameterDatabaseReader.ReadTime(
                "its-geonet-congestion-monitoring-timeout-time", initNodeId, initInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            "its-geonet-congestion-monitoring-max-vehicles", initNodeId, initInterfaceId)) {
        maxVehicles =
            theParameterDatabaseReader.ReadNonNegativeInt(
                "its-geonet-congestion-monitoring-max-vehicles", initNodeId, initInterfaceId);

        if (maxVehicles <= 0) {
            cerr << "\"ITS-GEONET-CONGESTION-MONITORING-MAX-VEHCLES\" should be 1 or more." << endl;
            exit(1);
        }//if//

    }//if//

}


inline
void GeoNetCongestionMonitor::ScanAReceivedFrame(
    const Packet& aFrame,
    const SimTime& scannedTime)
{

    const PacketId& thePacketId = aFrame.GetPacketId();
    const NodeId theNodeId = thePacketId.GetSourceNodeId();

    transmitterNodeIdRecords[theNodeId] = scannedTime;

}//ScanReceivedFrame//


inline
double GeoNetCongestionMonitor::GetCurrentCongestionValue()
{

    assert(maxVehicles != UINT_MAX);

    ClearObsoleteRecords();

    return ((double)transmitterNodeIdRecords.size() / maxVehicles);

}//GetCurrentCongestionValue//


inline
void GeoNetCongestionMonitor::ClearObsoleteRecords()
{

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    typedef map<NodeId, SimTime>::iterator IterType;
    IterType iter = transmitterNodeIdRecords.begin();

    while(iter != transmitterNodeIdRecords.end()) {
        if (((iter->second) + congestionMonitoringTimeoutTime) < currentTime) {
            IterType deleteIter = iter;
            ++iter;
            transmitterNodeIdRecords.erase(deleteIter);
        }
        else {
          ++iter;
        }//if//
    }//while//

}//ClearObsoleteRecords//



class TransmitDatarateController {
public:
    TransmitDatarateController(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& initNodeId,
        const InterfaceId& initInterfaceId);

    DatarateBitsPerSec GetDatarateBps(const double& congestionValue) const;


private:
    bool tdcIsEnabled;

    map<double, DatarateBitsPerSec> datarateControlTable;
    DatarateBitsPerSec constantDatarateBps;

    void ReadCongestionVsDatarateTable(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& initNodeId,
        const InterfaceId& initInterfaceId);

};//TransmitDatarateController//


inline
TransmitDatarateController::TransmitDatarateController(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& initNodeId,
    const InterfaceId& initInterfaceId)
    :
    tdcIsEnabled(false),
    constantDatarateBps(0)
{

    if (theParameterDatabaseReader.ParameterExists(
            "its-geonet-enable-tranmit-datarate-control", initNodeId, initInterfaceId)) {
        tdcIsEnabled =
            theParameterDatabaseReader.ReadBool(
                "its-geonet-enable-tranmit-datarate-control", initNodeId, initInterfaceId);
    }//if//

    if (tdcIsEnabled) {

        ReadCongestionVsDatarateTable(
            theParameterDatabaseReader,
            initNodeId,
            initInterfaceId);

    }
    else {

        const string modAndCodingString =
            theParameterDatabaseReader.ReadString("dot11-modulation-and-coding", initNodeId, initInterfaceId);

        //convert to datarate: assume 10MHz channel bandwidth
        if (modAndCodingString == "BPSK_0.5") {
            constantDatarateBps = 3000000;
        }
        else if (modAndCodingString == "BPSK_0.75") {
            constantDatarateBps = 4500000;
        }
        else if (modAndCodingString == "QPSK_0.5") {
            constantDatarateBps = 6000000;
        }
        else if (modAndCodingString == "QPSK_0.75") {
            constantDatarateBps = 9000000;
        }
        else if (modAndCodingString == "16QAM_0.5") {
            constantDatarateBps = 12000000;
        }
        else if (modAndCodingString == "16QAM_0.75") {
            constantDatarateBps = 18000000;
        }
        else if (modAndCodingString == "64QAM_0.67") {
            constantDatarateBps = 24000000;
        }
        else if (modAndCodingString == "64QAM_0.75") {
            constantDatarateBps = 27000000;
        }
    }//if//

}


inline
void TransmitDatarateController::ReadCongestionVsDatarateTable(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& initNodeId,
    const InterfaceId& initInterfaceId)
{

    const string datarateControlTableString =
        theParameterDatabaseReader.ReadString(
            "its-geonet-tdc-congestion-vs-datarate-mbps-table", initNodeId, initInterfaceId);

    bool success;

    map<double, double> datarateControlTableInDouble;
    ConvertAStringSequenceOfRealValuedPairsIntoAMap<double>(
        datarateControlTableString,
        success,
        datarateControlTableInDouble);

    if (!success) {
        cerr << "its-geonet-tdc-congestion-vs-datarate-mbps-table format:" << endl;
        cerr << "   " << datarateControlTableString << endl;
        exit(1);
    }//if//


    const DatarateBitsPerSec mbps = 1000 * 1000;
    for(map<double, double>::const_iterator iter = datarateControlTableInDouble.begin();
        iter != datarateControlTableInDouble.end(); ++iter) {

        const DatarateBitsPerSec datarateBps = static_cast<DatarateBitsPerSec>(iter->second * mbps);

        datarateControlTable.insert(make_pair(iter->first/100.0, datarateBps));

    }//for//


}//ReadCongestionVsDatarateTable//


inline
DatarateBitsPerSec TransmitDatarateController::GetDatarateBps(
    const double& congestionValue) const
{

    //constant
    if (!tdcIsEnabled) {
        return constantDatarateBps;
    }

    assert(!datarateControlTable.empty());

    map<double, DatarateBitsPerSec>::const_iterator iter =
        datarateControlTable.lower_bound(congestionValue);

    if (iter == datarateControlTable.end()) {
        --iter;
    }//if//

    return (iter->second);

}//GetDatarateBps//




class TransmitPowerController {
public:
    TransmitPowerController(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& initNodeId,
        const InterfaceId& initInterfaceId);

    double GetTxPowerDbm(const double& congestionValue) const;


private:
    bool tpcIsEnabled;

    map<double, double> txPowerControlTable;
    double interpolationStepDbm;
    double constantTxPowerDbm;

    void ReadCongestionVsTxPowerTable(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& initNodeId,
        const InterfaceId& initInterfaceId);

};//TransmitPowerController//


inline
TransmitPowerController::TransmitPowerController(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& initNodeId,
    const InterfaceId& initInterfaceId)
    :
    tpcIsEnabled(false),
    interpolationStepDbm(0.0),
    constantTxPowerDbm(-DBL_MAX)
{

    if (theParameterDatabaseReader.ParameterExists(
            "its-geonet-enable-tranmit-power-control", initNodeId, initInterfaceId)) {
        tpcIsEnabled =
            theParameterDatabaseReader.ReadBool(
                "its-geonet-enable-tranmit-power-control", initNodeId, initInterfaceId);
    }//if//

    if (tpcIsEnabled) {

        ReadCongestionVsTxPowerTable(
            theParameterDatabaseReader,
            initNodeId,
            initInterfaceId);


        interpolationStepDbm =
            theParameterDatabaseReader.ReadDouble(
                "its-geonet-tpc-interpolation-step-dbm", initNodeId, initInterfaceId);
        if (interpolationStepDbm <= 0.0) {
            cerr << "\"its-geonet-tpc-interpolation-step-dbm\" should be more than 0.0." << endl;
            exit(1);
        }

    }
    else {

        constantTxPowerDbm =
            theParameterDatabaseReader.ReadDouble("dot11-default-tx-power-dbm-when-not-specified", initNodeId, initInterfaceId);

    }//if//

}


inline
void TransmitPowerController::ReadCongestionVsTxPowerTable(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& initNodeId,
    const InterfaceId& initInterfaceId)
{

    const string txPowerControlTableString =
        theParameterDatabaseReader.ReadString(
            "its-geonet-tpc-congestion-vs-tx-power-dbm-table", initNodeId, initInterfaceId);

    bool success;

    map<double, double> txPowerControlTableInDouble;
    ConvertAStringSequenceOfRealValuedPairsIntoAMap<double>(
        txPowerControlTableString,
        success,
        txPowerControlTableInDouble);

    if (!success) {
        cerr << "its-geonet-tpc-congestion-vs-tx-power-dbm-table format:" << endl;
        cerr << "   " << txPowerControlTableString << endl;
        exit(1);
    }//if//

    for(map<double, double>::const_iterator iter = txPowerControlTableInDouble.begin();
        iter != txPowerControlTableInDouble.end(); ++iter) {

        txPowerControlTable.insert(make_pair(iter->first/100.0, iter->second));

    }//for//


}//ReadCongestionVsDatarateTable//


inline
double TransmitPowerController::GetTxPowerDbm(
    const double& congestionValue) const
{

    //constant
    if (!tpcIsEnabled) {
        return constantTxPowerDbm;
    }

    assert(!txPowerControlTable.empty());

    map<double, double>::const_iterator iter =
        txPowerControlTable.lower_bound(congestionValue);


    if (iter == txPowerControlTable.begin()) {
        return (iter->second);
    }
    else if (iter == txPowerControlTable.end()) {
        --iter;
        return (iter->second);
    }
    else if ((iter->first) == congestionValue) {
        //perfect match
        return (iter->second);
    }
    else {
        //linear interpolation with discrete step
        const double x2 = iter->first;
        const double y2 = iter->second;
        --iter;
        const double x1 = iter->first;
        const double y1 = iter->second;

        assert((x1 < congestionValue) && (congestionValue < x2));

        const double actualDeltaValue = (y2 - y1) * ((congestionValue - x1) / (x2 - x1));

        const double discreteDeltaValue =
            (int)(actualDeltaValue / interpolationStepDbm) * interpolationStepDbm;

        return (y1 + discreteDeltaValue);

    }//if//


}//GetTxPowerDbm//




class DCCSensitivityController {
public:
    DCCSensitivityController(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& initNodeId,
        const InterfaceId& initInterfaceId);

    double GetCCAThresholdDbm(
        const double& congestionValue) const;

    bool IsEnabled() const {
        return dscIsEnabled;
    }

private:
    bool dscIsEnabled;

    map<double, double> sensitivityControlTable;
    double interpolationStepDbm;

    void ReadCongestionVsSensitivityTable(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& initNodeId,
        const InterfaceId& initInterfaceId);

};//DCCSensitivityController//


inline
DCCSensitivityController::DCCSensitivityController(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& initNodeId,
    const InterfaceId& initInterfaceId)
    :
    dscIsEnabled(false)
{

    if (theParameterDatabaseReader.ParameterExists(
            "its-geonet-enable-dcc-sensitivity-control", initNodeId, initInterfaceId)) {
        dscIsEnabled =
            theParameterDatabaseReader.ReadBool(
                "its-geonet-enable-dcc-sensitivity-control", initNodeId, initInterfaceId);
    }//if//

    if (!dscIsEnabled) {
        return;
    }

    if (!theParameterDatabaseReader.ParameterExists(
            "its-geonet-dcc-control-loop-interval", initNodeId, initInterfaceId)) {
        cerr << "Please specify \"its-geonet-dcc-control-loop-interval\" for DCC Sensitivity Control" << endl;
        exit(1);
    }//if//

    ReadCongestionVsSensitivityTable(
        theParameterDatabaseReader,
        initNodeId,
        initInterfaceId);

    interpolationStepDbm =
        theParameterDatabaseReader.ReadDouble(
            "its-geonet-dsc-interpolation-step-dbm", initNodeId, initInterfaceId);
    if (interpolationStepDbm <= 0.0) {
            cerr << "\"its-geonet-dsc-interpolation-step-dbm\" should be more than 0.0." << endl;
            exit(1);
    }//if//

}


inline
void DCCSensitivityController::ReadCongestionVsSensitivityTable(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& initNodeId,
    const InterfaceId& initInterfaceId)
{

    const string sensitivityControlTableString =
        theParameterDatabaseReader.ReadString(
            "its-geonet-dsc-congestion-vs-sensitivity-dbm-table", initNodeId, initInterfaceId);

    bool success;

    map<double, double> sensitivityControlTableInDouble;
    ConvertAStringSequenceOfRealValuedPairsIntoAMap<double>(
        sensitivityControlTableString,
        success,
        sensitivityControlTableInDouble);

    if (!success) {
        cerr << "its-geonet-dsc-congestion-vs-sensitivity-dbm-table format:" << endl;
        cerr << "   " << sensitivityControlTableString << endl;
        exit(1);
    }//if//

    for(map<double, double>::const_iterator iter = sensitivityControlTableInDouble.begin();
        iter != sensitivityControlTableInDouble.end(); ++iter) {

        sensitivityControlTable.insert(make_pair(iter->first/100.0, iter->second));

    }//for//


}//ReadCongestionVsDatarateTable//


inline
double DCCSensitivityController::GetCCAThresholdDbm(
    const double& congestionValue) const
{
    assert(dscIsEnabled);

    assert(!sensitivityControlTable.empty());

    map<double, double>::const_iterator iter =
        sensitivityControlTable.lower_bound(congestionValue);

    if (iter == sensitivityControlTable.begin()) {
        return (iter->second);
    }
    else if (iter == sensitivityControlTable.end()) {
        --iter;
        return (iter->second);
    }
    else if ((iter->first) == congestionValue) {
        //perfect match
        return (iter->second);
    }
    else {
        //linear interpolation with discrete step
        const double x2 = iter->first;
        const double y2 = iter->second;
        --iter;
        const double x1 = iter->first;
        const double y1 = iter->second;

        assert((x1 < congestionValue) && (congestionValue < x2));

        const double actualDeltaValue = (y2 - y1) * ((congestionValue - x1) / (x2 - x1));

        const double discreteDeltaValue =
            (int)(actualDeltaValue / interpolationStepDbm) * interpolationStepDbm;

        return (y1 + discreteDeltaValue);

    }//if//


}//GetCCAThresholdDbm//





}//namespace//

#endif
