// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_NODEID_H
#define SCENSIM_NODEID_H

#include <string>
#include <limits.h>

namespace ScenSim {

using std::string;


// NodeId type :
//    A "Simulation Unit" is defined as an inseparable blob of submodels with
//    respect to the simulation engine.  A "Simulation Unit ID" identifies the
//    unit.  Submodels within a "Simulation Unit" communicate by direct C++
//    function calls at the current simulation time.  Communication to model
//    code in a different "Simulation Unit" should be done through simulation
//    events, specifically the ScheduleExternalEvent*() method calls.  Note this
//    closely corresponds to the network node ID in a network simulation, as
//    communication between models (layers) within a node can be seen as local
//    and communication to other nodes non-local.  This complexity (Simulation
//    Units) is driven by the desire to support parallel execution, with the
//    "Simulation Units" mapped to different processors arbitrarily and the
//    ScheduleExternalEvent*() providing the communication between
//    units on different processors.
//

typedef unsigned int NodeId;

typedef NodeId NodeIdType;  // Deprecated Style


const NodeId InvalidNodeId = UINT_MAX;
const NodeId INVALID_NODEID = InvalidNodeId;
const NodeId AnyNodeId = 0;
const NodeId ANY_NODEID = AnyNodeId;

const NodeId GISOBJECT_ROAD_START_NODEID = 100000000;
const NodeId GISOBJECT_INTERSECTION_START_NODEID = 101000000;
const NodeId GISOBJECT_BUILDING_START_NODEID = 102000000;
const NodeId GISOBJECT_WALL_START_NODEID = 102500000;
const NodeId GISOBJECT_RAIL_START_NODEID = 103000000;
const NodeId GISOBJECT_WAY_STATION_START_NODEID = 104000000;
const NodeId GISOBJECT_NODE_STATION_START_NODEID = 104500000;
const NodeId GISOBJECT_SIGNAL_START_NODEID = 105000000;
const NodeId GISOBJECT_BUSSTOP_START_NODEID = 105500000;
const NodeId GISOBJECT_AREA_START_NODEID = 106000000;
const NodeId GISOBJECT_PARK_START_NODEID = 106500000;
const NodeId GISOBJECT_ENTRANCE_START_NODEID = 107000000;
const NodeId GISOBJECT_SERVICEAREA_START_NODEID = 108000000;
const NodeId GISOBJECT_GENERIC_POLYGON_START_NODEID = 109000000;

const NodeId MAX_COMMUNICATION_NODEID = GISOBJECT_ROAD_START_NODEID - 1;

const unsigned int NODEID_MIN_STREAM_OUTPUT_WIDTH = 3;

typedef string ModelObjectInstanceId;
typedef ModelObjectInstanceId ModelObjectInstanceIdType;  // Deprecated Style



typedef string InterfaceOrInstanceId;
typedef InterfaceOrInstanceId InterfaceOrInstanceIdType;  // Deprecated Style

const InterfaceOrInstanceId nullInstanceId("");

typedef InterfaceOrInstanceId InterfaceId;
typedef InterfaceId InterfaceIdType;  // Deprecated Style

const InterfaceId nullInterfaceId(nullInstanceId);

}//namespace//

#endif
