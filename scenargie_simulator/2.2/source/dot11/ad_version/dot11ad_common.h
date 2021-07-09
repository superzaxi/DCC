// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11AD_COMMON_H
#define DOT11AD_COMMON_H

#include <memory>
#include <string>
#include "scensim_mac.h"

namespace Dot11ad {

using ScenSim::Packet;

using ScenSim::SimulationEngineInterface;
using std::shared_ptr;
using std::make_shared;
using ScenSim::NodeId;
using std::string;

typedef ScenSim::SixByteMacAddress MacAddress;

const string parameterNamePrefix("dot11-");
const string adParamNamePrefix("dot11ad-");

typedef unsigned long long int DatarateBitsPerSec;


// 802.11ad:

// For Max Sectors for protocol sanity checks.

const unsigned int MaxDirectionalSectors = 63;
const unsigned int QuasiOmniSectorId = 63;
const unsigned int InvalidSectorId = UINT_MAX;

}//namespace//

#endif
