// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef GEONET_TRACEDEF_H
#define GEONET_TRACEDEF_H

#include <stdint.h>

#include "scensim_time.h"
#include "scensim_nodeid.h"

namespace GeoNet {

using ScenSim::NodeId;
using ScenSim::SimTime;


//transport

static const size_t BTP_SEND_TRACE_RECORD_BYTES = 16;
struct BtpSendTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    unsigned char padding[4];
};

static const size_t BTP_RECEIVE_TRACE_RECORD_BYTES = 16;
struct BtpReceiveTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    unsigned char padding[4];
};


//network

static const size_t GEONET_PACKET_TRACE_RECORD_BYTES = 16;
struct GeoNetPacketTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    unsigned char padding[4];
};

// after format version 3
static const size_t GEONET_FULL_QUEUE_DROP_TRACE_RECORD_BYTES = 16;
struct GeoNetFullQueueDropTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    uint16_t enqueueResult;
    unsigned char padding[2];
};


}//namespace//


#endif

