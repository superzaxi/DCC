// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11AD_TXPOWERCONTROL_H
#define DOT11AD_TXPOWERCONTROL_H

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "dot11ad_common.h"

namespace Dot11ad{

using std::shared_ptr;
using std::cerr;
using std::endl;

using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeId;
using ScenSim::InterfaceId;
using ScenSim::SimulationEngineInterface;
using ScenSim::MakeLowerCaseString;


class AdaptiveTxPowerController {
public:
    AdaptiveTxPowerController(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId);

    virtual ~AdaptiveTxPowerController() { }

    virtual double GetDmgBeaconTransmitPowerDbm() const { return (dmgBeaconTransmitPowerDbm); }
    virtual double GetCurrentTransmitPowerDbm(const MacAddress& macAddress) const;
    virtual void SetForcedLinkTxPower(const MacAddress& macAddress, const double& powerDbm);

    bool TxPowerIsSpecifiedByPhyLayer() const { return txPowerIsSpecifiedByPhyLayer; }


protected:
    map<MacAddress, double> forcedLinkPowerDbmMap;

    double transmitPowerDbm;
    double dmgBeaconTransmitPowerDbm;
    bool txPowerIsSpecifiedByPhyLayer;

};//AdaptiveTxPowerController//


inline
AdaptiveTxPowerController::AdaptiveTxPowerController(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId)
    :
    txPowerIsSpecifiedByPhyLayer(true)
{
    string powerSpecifiedBy = "phylayer";

    if (theParameterDatabaseReader.ParameterExists("dot11-tx-power-specified-by", theNodeId, theInterfaceId)) {
        powerSpecifiedBy = MakeLowerCaseString(
            theParameterDatabaseReader.ReadString("dot11-tx-power-specified-by", theNodeId, theInterfaceId));
    }//if//

    if (powerSpecifiedBy == "phylayer") {

        txPowerIsSpecifiedByPhyLayer = true;

        transmitPowerDbm =
            theParameterDatabaseReader.ReadDouble("dot11-tx-power-dbm", theNodeId, theInterfaceId);
    }
    else if (powerSpecifiedBy == "upperlayer") {

        txPowerIsSpecifiedByPhyLayer = false;

        transmitPowerDbm =
            theParameterDatabaseReader.ReadDouble("dot11-default-tx-power-dbm-when-not-specified", theNodeId, theInterfaceId);
    }
    else {
        cerr << "Error: Unknown dot11-tx-power-specified-by = " << powerSpecifiedBy << endl;
        exit(1);
    }//if//


    dmgBeaconTransmitPowerDbm = transmitPowerDbm;

    if (theParameterDatabaseReader.ParameterExists("dot11ad-dmg-beacon-tx-power-dbm", theNodeId, theInterfaceId)) {
        dmgBeaconTransmitPowerDbm =
            theParameterDatabaseReader.ReadDouble("dot11ad-dmg-beacon-tx-power-dbm", theNodeId, theInterfaceId);
    }//if//

}//AdaptiveTxPowerController//


inline
void AdaptiveTxPowerController::SetForcedLinkTxPower(
    const MacAddress& macAddress,
    const double& powerDbm)
{
    forcedLinkPowerDbmMap[macAddress] = powerDbm;
}

inline
double AdaptiveTxPowerController::GetCurrentTransmitPowerDbm(const MacAddress& macAddress) const
{
    typedef map<MacAddress, double>::const_iterator IterType;

    if ((forcedLinkPowerDbmMap.empty()) || (macAddress.IsABroadcastAddress())) {
        return (transmitPowerDbm);
    }//if//

    const IterType findIter = forcedLinkPowerDbmMap.find(macAddress);

    if (findIter == forcedLinkPowerDbmMap.end()) {
        return (transmitPowerDbm);
    }
    else {
        return (findIter->second);
    }//if//

}//CurrentTransmitPowerDbm//


//--------------------------------------------------------------------------------------------------

inline
shared_ptr<AdaptiveTxPowerController> CreateAdaptiveTxPowerController(
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId)
{

    return (shared_ptr<AdaptiveTxPowerController>(
        new AdaptiveTxPowerController(
            theParameterDatabaseReader,
            theNodeId,
            theInterfaceId)));

}//CreateAdaptiveTxPowerController//


}//namespace

#endif
