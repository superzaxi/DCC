// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef ALOHA_TRACEDEF_H
#define ALOHA_TRACEDEF_H

#include <stdint.h>

#include "scensim_time.h"
#include "scensim_nodeid.h"

namespace Aloha {

using ScenSim::NodeId;
using ScenSim::SimTime;


//mac

static const size_t ALOHA_MAC_PACKET_DEQUEUE_TRACE_RECORD_BYTES = 16;
struct AlohaMacPacketDequeueTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    unsigned char padding[4];
};

static const size_t ALOHA_MAC_TX_DATA_TRACE_RECORD_BYTES = 16;
struct AlohaMacTxDataTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    uint32_t retryCount;
};

static const size_t ALOHA_MAC_PACKET_RETRY_EXCEEDED_TRACE_RECORD_BYTES = 16;
struct AlohaMacPacketRetryExceededTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    unsigned char padding[4];
};


static const size_t ALOHA_MAC_FRAME_RECEIVE_TRACE_RECORD_BYTES = 16;
struct AlohaMacFrameReceiveTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    unsigned char frameType; // const static defined
    unsigned char padding[3];
};


//phy

static const size_t ALOHA_PHY_TX_START_TRACE_RECORD_BYTES = 40;
struct AlohaPhyTxStartTraceRecord {
    NodeId sourceNodeId;
    unsigned char padding[4];
    uint64_t sourceNodeSequenceNumber;
    double txPower;
    uint64_t dataRate;//actual: DatarateBitsPerSec
    SimTime duration; //long long int

};

static const size_t ALOHA_PHY_SIGNAL_INTERFERENCE_TRACE_RECORD_BYTES = 16;
struct AlohaPhySignalInterferenceTraceRecord {
    NodeId sourceNodeId;
    unsigned char padding[4];
    uint64_t sourceNodeSequenceNumber;
};

static const size_t ALOHA_PHY_RX_END_TRACE_RECORD_BYTES = 16;
struct AlohaPhyRxEndTraceRecord {
    NodeId sourceNodeId;
    bool error;
    unsigned char padding[2];
    uint64_t sourceNodeSequenceNumber;
};

static const size_t ALOHA_PHY_RX_START_TRACE_RECORD_BYTES = 24;
struct AlohaPhyRxStartTraceRecord {
    NodeId sourceNodeId;
    unsigned char padding[4];
    uint64_t sourceNodeSequenceNumber;
    double rxPower;
};


}//namespace//


#endif

