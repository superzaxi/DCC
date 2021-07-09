// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11AH_INFO_INTERFACE_H
#define DOT11AH_INFO_INTERFACE_H

#include "scensim_time.h"
#include "scensim_mac.h"

namespace Dot11ah {

using ScenSim::SimTime;
using ScenSim::MacAndPhyInfoInterface;

class Dot11Phy;

class Dot11InfoInterface : public MacAndPhyInfoInterface {
public:
    double GetRssiOfLastFrameDbm() const;
    double GetSinrOfLastFrameDb() const;

    unsigned int GetNumberOfReceivedFrames() const;
    unsigned int GetNumberOfFramesWithErrors() const;
    unsigned int GetNumberOfSignalCaptures() const;

    SimTime GetTotalIdleChannelTime() const;
    SimTime GetTotalBusyChannelTime() const;
    SimTime GetTotalTransmissionTime() const;

private:
    Dot11InfoInterface(const Dot11Phy* initPhyLayerPtr)
        : phyLayerPtr(initPhyLayerPtr) { }

    const Dot11Phy* phyLayerPtr;

    friend class Dot11Phy;

    //Disable:
    Dot11InfoInterface(const Dot11InfoInterface&);
    void operator=(const Dot11InfoInterface&);

};//Dot11InfoInterface//

}//namespace//

#endif
