// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11AD_TRACEDEF_H
#define DOT11AD_TRACEDEF_H

#include <stdint.h>

#include "scensim_time.h"
#include "scensim_nodeid.h"

#include "dot11ad_common.h"
#include "dot11ad_headers.h"

namespace Dot11ad {

using std::string;

using ScenSim::NodeId;
using ScenSim::SimTime;

//mac

const size_t MAC_FRAME_RECEIVE_TRACE_RECORD_BYTES = 16;
struct MacFrameReceiveTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    unsigned char frameType; // const static defined
    unsigned char padding[1];
    uint16_t packetLengthBytes; // after format version 2
};

const size_t DOT11_MAC_TX_MANAGEMENT_TRACE_RECORD_BYTES = 24;
struct Dot11MacTxManagementTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    NodeId destNodeId; // after format version 4
    unsigned char frameType; // const static defined
    unsigned char padding[7];
};

const size_t MAC_IFS_AND_BACKOFF_START_TRACE_RECORD_BYTES = 16;
struct MacIfsAndBackoffStartTraceRecord {
    SimTime duration;
    uint32_t accessCategory;
    bool frameCorrupt:8; //0:No, 1:Yes
    unsigned char padding[3];
};

const size_t MAC_IFS_AND_BACKOFF_PAUSE_TRACE_RECORD_BYTES = 16;
struct MacIfsAndBackoffPauseTraceRecord {
    SimTime leftDuration;
    uint32_t accessCategory;
    unsigned char padding[4];
};

const size_t MAC_PACKET_DEQUEUE_TRACE_RECORD_BYTES = 16;
struct MacPacketDequeueTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    uint32_t accessCategory;
};

const size_t MAC_TX_RTS_TRACE_RECORD_BYTES = 16;
struct MacTxRtsTraceRecord {
    uint32_t accessCategory;
    int32_t retry;
    NodeId destNodeId; // after format version 4
    unsigned char padding[4];
};

const size_t MAC_TX_BROADCAST_DATA_TRACE_RECORD_BYTES = 16;
struct MacTxBroadcastDataTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    uint32_t accessCategory;
};

const size_t MAC_TX_UNICAST_DATA_TRACE_RECORD_BYTES = 32;
struct MacTxUnicastDataTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    uint32_t accessCategory;
    int32_t shortFrameRetry;
    int32_t longFrameRetry;
    NodeId destNodeId; // after format version 4
    uint32_t numSubframes; // after format version 4
};

const size_t DOT11_MAC_TX_DMG_TRACE_RECORD_BYTES = 24;
struct Dot11MacTxDMGTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    NodeId destNodeId;
    unsigned char frameType;
    unsigned char padding[7];
};

const size_t MAC_CTSORACK_TIMEOUT_TRACE_RECORD_BYTES = 24;
struct MacCtsOrAckTimeoutTraceRecord {
    uint32_t accessCategory;
    int32_t windowSlot;
    int32_t shortFrameRetry;
    int32_t longFrameRetry;
    bool shortFrameOrNot:8; //1:short, 0:long
    unsigned char padding[7];
};

const size_t MAC_PACKET_RETRY_EXCEEDED_TRACE_RECORD_BYTES = 16;
struct MacPacketRetryExceededTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeId sourceNodeId;
    unsigned char padding[4];
};

const size_t MAC_TX_DATARATE_UPDATE_TRACE_RECORD_BYTES = 16;
struct MacTxDatarateUpdateTraceRecord {
    uint64_t txDatarateBps;
    NodeId destNodeId;
    unsigned char padding[4];
};

const size_t MAC_DOWNLINK_BEAMFORMING_TRACE_RECORD_BYTES = 24;
struct MacDownlinkBeamformingTraceRecord {
    NodeId sourceNodeId;
    uint32_t sectorId;
    double transmissionSectorMetric;
    bool sourceIsStaOrNot:8; //1:STA, 0:AP
    unsigned char padding[7];
};

const size_t MAC_UPLINK_BEAMFORMING_TRACE_RECORD_BYTES = 16;
struct MacUplinkBeamformingTraceRecord {
    NodeId sourceNodeId;
    uint32_t sectorId;
    bool sourceIsStaOrNot:8; //1:STA, 0:AP
    unsigned char padding[7];
};

//phy

const size_t NOISE_START_TRACE_RECORD_BYTES = 24;
struct NoiseStartTraceRecord {
    NodeId sourceNodeId;
    unsigned char padding[4];
    double rxPower;
    double interferenceAndNoisePower;

};

const size_t NOISE_END_TRACE_RECORD_BYTES = 16;
struct NoiseEndTraceRecord {
    double rxPower;
    double interferenceAndNoisePower;

};

const size_t TX_START_TRACE_RECORD_BYTES = 40;
struct TxStartTraceRecord {
    NodeId sourceNodeId;
    unsigned char padding[4];
    uint64_t sourceNodeSequenceNumber;
    double txPower;
    uint64_t dataRate;//actual: DatarateBitsPerSec
    SimTime duration; //long long int

};

const size_t RX_START_TRACE_RECORD_BYTES = 24;
struct RxStartTraceRecord {
    NodeId sourceNodeId;
    unsigned char padding[4];
    uint64_t sourceNodeSequenceNumber;
    double rxPower;
};


// capture types
const unsigned char TRACE_NO_SIGNAL_CAPTURE = 0;
const unsigned char TRACE_SIGNAL_CAPTURE = 1;
const unsigned char TRACE_SIGNAL_CAPTURE_IN_SHORT_TRAINING_FIELD = 2;

const size_t RX_END_TRACE_RECORD_BYTES = 16;
struct RxEndTraceRecord {
    NodeId sourceNodeId;
    bool error;
    unsigned char captureType;
    unsigned char padding[2];
    uint64_t sourceNodeSequenceNumber;
};


}//namespace//


#endif

