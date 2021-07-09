// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef T109_TRACEDEF_H
#define T109_TRACEDEF_H

#include <stdint.h>

#include "scensim_time.h"
#include "scensim_nodeid.h"

namespace T109 {

using ScenSim::NodeId;
using ScenSim::SimTime;


//mac

static const size_t T109_MAC_FRAME_RECEIVE_TRACE_RECORD_BYTES = 16;
struct T109MacFrameReceiveTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    unsigned char frameType; // const static defined
    unsigned char padding[3];
};

static const size_t T109_MAC_IFS_AND_BACKOFF_START_TRACE_RECORD_BYTES = 16;
struct T109MacIfsAndBackoffStartTraceRecord {
    SimTime duration;
    uint32_t accessCategory;
    bool frameCorrupt:8; //0:No, 1:Yes
    unsigned char padding[3];
};

static const size_t T109_MAC_IFS_AND_BACKOFF_PAUSE_TRACE_RECORD_BYTES = 16;
struct T109MacIfsAndBackoffPauseTraceRecord {
    SimTime leftDuration;
    uint32_t accessCategory;
    unsigned char padding[4];
};

static const size_t T109_MAC_TX_BROADCAST_DATA_TRACE_RECORD_BYTES = 16;
struct T109MacTxBroadcastDataTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    uint32_t accessCategory;
};


//phy

static const size_t T109_NOISE_START_TRACE_RECORD_BYTES = 24;
struct T109NoiseStartTraceRecord {
    NodeId sourceNodeId;
    unsigned char padding[4];
    double rxPower;
    double interferenceAndNoisePower;

};

static const size_t T109_NOISE_END_TRACE_RECORD_BYTES = 16;
struct T109NoiseEndTraceRecord {
    double rxPower;
    double interferenceAndNoisePower;

};

static const size_t T109_TX_START_TRACE_RECORD_BYTES = 40;
struct T109TxStartTraceRecord {
    NodeId sourceNodeId;
    unsigned char padding[4];
    uint64_t sourceNodeSequenceNumber;
    double txPower;
    uint64_t dataRate;//actual: DatarateBitsPerSec
    SimTime duration; //long long int

};

static const size_t T109_RX_START_TRACE_RECORD_BYTES = 24;
struct T109RxStartTraceRecord {
    NodeId sourceNodeId;
    unsigned char padding[4];
    uint64_t sourceNodeSequenceNumber;
    double rxPower;
};

static const size_t T109_RX_END_TRACE_RECORD_BYTES = 16;
struct T109RxEndTraceRecord {
    NodeId sourceNodeId;
    bool error;
    bool captured;
    unsigned char padding[2];
    uint64_t sourceNodeSequenceNumber;
};


}//namespace//


#endif

