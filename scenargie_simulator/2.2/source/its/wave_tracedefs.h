// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef WAVE_TRACEDEF_H
#define WAVE_TRACEDEF_H

#include <stdint.h>

#include "scensim_nodeid.h"

namespace Wave {

using ScenSim::NodeId;

//network

static const size_t WSMP_PACKET_TRACE_RECORD_BYTES = 16;
struct WsmpPacketTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    unsigned char padding[4];
};

// after format version 3
static const size_t WSMP_FULL_QUEUE_DROP_TRACE_RECORD_BYTES = 16;
struct WsmpFullQueueDropTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    uint16_t enqueueResult;
    unsigned char padding[2];
};

//mac

static const size_t WAVE_NEW_CH_INTERVAL_TRACE_RECORD_BYTES = 16;
struct WaveNewChIntervalTraceRecord {
    int32_t previousChannelNumber;
    int32_t nextChannelNumber;
    uint8_t previousChannelCategory;
    uint8_t nextChannelCategory;
    unsigned char padding[6];
};

static const size_t WAVE_PHY_CH_SWITCH_TRACE_RECORD_BYTES = 8;
struct WavePhyChSwitchTraceRecord {
    int32_t channelNumber;
    uint8_t channelCategory;
    unsigned char padding[3];
};


}//namespace//


#endif

