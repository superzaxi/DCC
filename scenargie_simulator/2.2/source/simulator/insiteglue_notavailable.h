// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef INSITEGLUE_NOTAVAILABLE_H
#define INSITEGLUE_NOTAVAILABLE_H

#ifndef INSITEGLUE_CHOOSER_H
#error "Include insiteglue_chooser.h not insiteglue_notavailable.h (indirect include only)"
#endif

#include "scensim_proploss.h"

namespace ScenSim {

class BuildingLayer;

class InsiteGeometry {
public:
    InsiteGeometry() {}

    void RegisterBuildingLayerPtr(const shared_ptr<BuildingLayer>& buildingLayerPtr) {}

    void AddObject(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const SimTime& currentTime,
        const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
        const string& stationTypeString = "") {}

    void DeleteObject(const NodeId& theNodeId) {}

    void SyncMovingObjectTime(const SimTime& currentTime) {}
};


};//namespace//

#endif
