// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11AH_HEADERS_H
#define DOT11AH_HEADERS_H

//--------------------------------------------------------------------------------------------------
// "NotUsed" data items in header structs are placeholders for standard
// 802.11 fields that are not currently used in the model.  The purpose of
// not including the unused standard field names is to make plain the
// features that are and are NOT implemented.  The "Not Used" fields should always be
// zeroed so that packets do not include random garbage.  Likewise, only
// frame types and codes that are used in model logic will be defined.
//
// This code ignores machine endian issues because it is a model, i.e. fields are the
// correct sizes but the actual bits will not be in "network order" on small endian machines.
//


#include <string>
#include <bitset>
#include "scensim_support.h"
#include "scensim_queues.h"
#include "scensim_prop.h"
#include "dot11ah_common.h"

namespace Dot11ah {

using std::string;
using std::vector;
using std::array;

using ScenSim::EtherTypeField;
using ScenSim::MaxNumBondedChannels;
using ScenSim::OneZeroedByteStruct;
using ScenSim::TwoZeroedBytesStruct;
using ScenSim::FourZeroedBytesStruct;
using ScenSim::EightZeroedBytesStruct;
using ScenSim::CalcTwelveBitSequenceNumberDifference;
using ScenSim::SimTime;
using ScenSim::MICRO_SECOND;

const SimTime TimeUnitDuration = 1024 * MICRO_SECOND;

// Duration in us.

typedef uint16_t DurationField;
const DurationField MaxDurationFieldValue = 32768;


typedef uint16_t AssociationId;
// 802.11ah
const AssociationId MaxAssociationId = 8191;
//const AssociationId MaxAssociationId = 2007;


const unsigned char ASSOCIATION_REQUEST_FRAME_TYPE_CODE = 0x00; // 00 0000;
const unsigned char ASSOCIATION_RESPONSE_FRAME_TYPE_CODE = 0x01; // 00 0001;
const unsigned char REASSOCIATION_REQUEST_FRAME_TYPE_CODE = 0x02; // 00 0010;
const unsigned char REASSOCIATION_RESPONSE_FRAME_TYPE_CODE = 0x03; // 00 0011;
const unsigned char BEACON_FRAME_TYPE_CODE = 0x08; // 00 1000;
const unsigned char DISASSOCIATION_FRAME_TYPE_CODE = 0x0A; // 00 1010;
const unsigned char AUTHENTICATION_FRAME_TYPE_CODE = 0x0B; // 00 1011
const unsigned char NULL_FRAME_TYPE_CODE = 0x24; // 10 0100;
const unsigned char BLOCK_ACK_REQUEST_FRAME_TYPE_CODE = 0x18;  // 01 1000
const unsigned char BLOCK_ACK_FRAME_TYPE_CODE = 0x19;  // 01 1001
const unsigned char RTS_FRAME_TYPE_CODE  = 0x1B; // 01 1011;
const unsigned char CTS_FRAME_TYPE_CODE  = 0x1C; // 01 1100;
const unsigned char ACK_FRAME_TYPE_CODE  = 0x1D; // 01 1101;
const unsigned char QOS_DATA_FRAME_TYPE_CODE = 0x28; // 10 1000;
const unsigned char POWER_SAVE_POLL_FRAME_TYPE_CODE = 0x1A;  // 01 1010;

//NotUsed const unsigned char ACTION_FRAME_TYPE_CODE = 0x0D; // 00 1101


// 802.11ah defines different protocol version,
// instead just picking arbitrary non-overlapping values.

//const unsigned char AH_S1G_BEACON_FRAME_TYPE_CODE = 0x09; //00 1001
const unsigned char AH_RESOURCE_ALLOCATION_FRAME_TYPE_CODE = 0x10; //00 1010


inline
bool IsAManagementFrameTypeCode(const unsigned char frameTypeCode)
{
    // True if top 2 bits (out of 6) are 0.

    return ((frameTypeCode & 0x30) == 0x0);
}



inline
string ConvertToDot11FrameTypeName(const unsigned char frameTypeCode)
{
    switch (frameTypeCode) {
    case ASSOCIATION_REQUEST_FRAME_TYPE_CODE: return "Association-Request";
    case ASSOCIATION_RESPONSE_FRAME_TYPE_CODE: return "Association-Response";
    case REASSOCIATION_REQUEST_FRAME_TYPE_CODE: return "Reassociation-Request";
    case REASSOCIATION_RESPONSE_FRAME_TYPE_CODE: return "Reassociation-Response";
    case BEACON_FRAME_TYPE_CODE: return "Beacon";
    case DISASSOCIATION_FRAME_TYPE_CODE: return "Disassociation";
    case AUTHENTICATION_FRAME_TYPE_CODE: return "Authentication";
    case RTS_FRAME_TYPE_CODE: return "RTS";
    case CTS_FRAME_TYPE_CODE: return "CTS";
    case ACK_FRAME_TYPE_CODE: return "ACK";
    case QOS_DATA_FRAME_TYPE_CODE: return "Data";
    case NULL_FRAME_TYPE_CODE: return "Null";
    case BLOCK_ACK_REQUEST_FRAME_TYPE_CODE: return "BlockACK-Request";
    case BLOCK_ACK_FRAME_TYPE_CODE: return "BlockACK";

    default:
        assert(false); abort();
        break;
    }//switch//

    return "";

}//ConvertToDot11FrameTypeName//


struct FrameControlField {
    enum ToDsFromDsChoicesType {
        NotWirelessDistributionSystemFrame = 0,
        WirelessDistributionSystemFrame = 3
    };

    unsigned char notUsed1:2;

    unsigned char frameTypeAndSubtype:6;

    unsigned char toDsFromDs:2;

    unsigned char notUsed2:1;

    unsigned char isRetry:1;

    unsigned char powerManagement:1;

    unsigned char notUsed_moreData:1;

    unsigned char notUsed3:1;
    unsigned char notUsed4:1;

    FrameControlField()
        : isRetry(0), toDsFromDs(NotWirelessDistributionSystemFrame),
          powerManagement(0),
          notUsed1(0), notUsed2(0), notUsed3(0), notUsed4(0),
          notUsed_moreData(0)
    {}
};//FrameControlField//


struct CommonFrameHeader {
    FrameControlField theFrameControlField;
    DurationField duration;
    MacAddress receiverAddress;

    CommonFrameHeader() : duration(0) {}
};


// Request To Send aka RTS

struct RequestToSendFrame {
    CommonFrameHeader header;
    MacAddress transmitterAddress;

    FourZeroedBytesStruct notUsed_FCS;
};


// Clear To Send aka CTS

struct ClearToSendFrame {
    CommonFrameHeader header;

    FourZeroedBytesStruct notUsed_FCS;

};


// Power Save Poll aka PS-Poll

struct PowerSavePollFrame {
    CommonFrameHeader header;
    MacAddress transmitterAddress;
    FourZeroedBytesStruct notUsed_FCS;
};


inline
AssociationId GetAssociationIdFromCommonFrameHeaderDurationField(
    const CommonFrameHeader& header)
{
    assert(header.theFrameControlField.frameTypeAndSubtype == POWER_SAVE_POLL_FRAME_TYPE_CODE);
    assert(header.duration <= MaxAssociationId);

    return (static_cast<AssociationId>(header.duration));
}


// Data Frame Types

const unsigned short int MaxSequenceNumber = 4095;

struct SequenceControlField {
    unsigned short int sequenceNumber:12;

    unsigned short int notUsed:4;

};


// QoS = "Quality of Service"
// "Traffic ID" aka TID.

// Jay: Traffic ID seems only useful if 802.11 is used for multiple "link layer" hops
//      without going to the official network layer (which has the priority).
//      The only real use of ackPolicy currently would be to send unicast packets
//      without ack-ing.  Thus, will leave the QoS Control Field stuff commented out for now.
//
// const unsigned short NORMAL_ACK_POLICY = 0x0; // 0b00;
// const unsigned short DONT_ACK_POLICY = 0x2;   // 0b10;
//

struct QosControlField {
    unsigned char trafficId:4;
    unsigned char notUsed1:1;
    unsigned char notUsed_ackPolicy:2;
    unsigned char reserved:1;
    unsigned char notUsed2:8;

    QosControlField() : reserved(0), notUsed_ackPolicy(0), notUsed1(0), notUsed2(0) { }
};

struct Ieee802p2LinkLayerHeader {
    TwoZeroedBytesStruct notUsed;
    EtherTypeField cheatingVlanTagTpid;
    unsigned char cheatingVlanTagPcp:3;
    unsigned char cheatingVlanTagCfi:1;
    unsigned char cheatingVlanTagVidHigh:4;
    unsigned char cheatingVlanTagVidLow:8;
    EtherTypeField etherType;
};

struct QosDataFrameHeader {
    CommonFrameHeader header;
    MacAddress transmitterAddress;
    MacAddress notUsed_Address3;
    SequenceControlField theSequenceControlField;
    QosControlField qosControlField;
    FourZeroedBytesStruct notUsed_FCS;
    Ieee802p2LinkLayerHeader linkLayerHeader;
};

struct QosNullFrameHeader {
    CommonFrameHeader header;
    MacAddress transmitterAddress;
    MacAddress notUsed_Address3;
    SequenceControlField theSequenceControlField;
    QosControlField qosControlField;
    FourZeroedBytesStruct notUsed_FCS;
};

struct DistributionSystemQosDataFrameHeader {
    CommonFrameHeader header;
    MacAddress transmitterAddress;
    MacAddress destinationAddress;
    SequenceControlField theSequenceControlField;
    MacAddress sourceAddress;
    QosControlField qosControlField;
    FourZeroedBytesStruct notUsed_FCS;
    Ieee802p2LinkLayerHeader linkLayerHeader;
};


// Acknowledge aka ACK

struct AcknowledgementAkaAckFrame {
    CommonFrameHeader header;

    FourZeroedBytesStruct notUsed_FCS;
};


//---------------------------------------------------------

const unsigned int BlockAckBitMapNumBits = 64;


struct BlockAckOrBlockAckRequestControlField {
    unsigned char notUsed_blockAckPolicy:1;    // Always "Normal".
    unsigned char notUsed_multiTid:1;  // Never multi-TID.
    unsigned char notUsed_compressedBitmap:1;  // Always compressed.
    unsigned char reserved1:5;
    unsigned char reserved2:4;
    unsigned char trafficId:4;

    BlockAckOrBlockAckRequestControlField() : notUsed_blockAckPolicy(0),  notUsed_multiTid(0),
        notUsed_compressedBitmap(0), reserved1(0), reserved2(0), trafficId(0) { }
};


struct BlockAcknowledgementFrame {
    CommonFrameHeader header;
    MacAddress transmitterAddress;
    BlockAckOrBlockAckRequestControlField blockAckControl;
    unsigned short reserved:4;
    unsigned short startingSequenceControl:12;
    FourZeroedBytesStruct notUsed_FCS;
    std::bitset<BlockAckBitMapNumBits> blockAckBitmap;  // order in struct swapped for 8 byte alignment.

    BlockAcknowledgementFrame() : reserved(0) {
        assert(sizeof(blockAckBitmap) == 8);
        assert(sizeof(*this) == 32);
    }

    bool IsAcked(const unsigned short sequenceNumber) const;
};


inline
bool BlockAcknowledgementFrame::IsAcked(const unsigned short int sequenceNumber) const
{
    const int offset =
        CalcTwelveBitSequenceNumberDifference(sequenceNumber, startingSequenceControl);

    assert(offset >= 0);
    if (offset >= BlockAckBitMapNumBits) {
        return false;
    }//if//

    return (blockAckBitmap[offset]);
}


struct BlockAcknowledgementRequestFrame {
    CommonFrameHeader header;
    MacAddress transmitterAddress;
    BlockAckOrBlockAckRequestControlField blockAckRequestControl;
    unsigned short reserved:4;
    unsigned short startingSequenceControl:12;
    FourZeroedBytesStruct notUsed_FCS;

    BlockAcknowledgementRequestFrame() : reserved(0) { }
};



//---------------------------------------------------------


struct ManagementFrameHeader {
    CommonFrameHeader header;
    MacAddress transmitterAddress;
    MacAddress notUsed_Address3;
    SequenceControlField theSequenceControlField;
    //11ah: FourZeroedBytesStruct notUsed_Fcs;
    TwoZeroedBytesStruct notUsed_Fcs;
    unsigned char fcsRandomBitsHigh;
    unsigned char fcsRandomBitsLow;
};

//---------------------------------------------------------


struct MpduDelimiterFrame {
    unsigned short int notUsed_EndOfFrameBitAkaEof:1;
    unsigned short int reserved:1;
    // Assume expansion 12->14 bit (11AC). Do not limit to 4k.
    unsigned short int lengthBytes:14;
    OneZeroedByteStruct notUsed_Crc;
    OneZeroedByteStruct notUsed_signature;

    MpduDelimiterFrame() : notUsed_EndOfFrameBitAkaEof(0), reserved(0) { }
};


//---------------------------------------------------------


struct BlockAckParameterSetFieldType
{
    unsigned short notUsed_blockAckPolicy:1;  // Never delayed.
    unsigned short trafficId:4;
    unsigned short notUsed_bufferSize:10;
};

// ADDBA Support.
//NotUsed const unsigned char ActionFrameBlockAckCategoryCode = 3;
//NotUsed const unsigned char AddBlockAckSessionRequestActionCode = 0;
//NotUsed const unsigned char AddBlockAckSessionResponseActionCode = 1;
//NotUsed
//NotUsed // Action Frame types
//NotUsed
//NotUsed struct ActionFrameHeaderType {
//NotUsed     CommonFrameHeader header;
//NotUsed     unsigned char categoryCode;
//NotUsed     unsigned char actionCode;
//NotUsed
//NotUsed     ActionFrameHeaderType() {
//NotUsed         header.theFrameControlField.frameTypeAndSubtype = ACTION_FRAME_TYPE_CODE;
//NotUsed     }
//NotUsed };
//NotUsed
//NotUsed
//NotUsed // Aka ADDBA Request.
//NotUsed
//NotUsed struct AddBlockAckSessionRequestFrameType {
//NotUsed     ActionFrameHeaderType actionFrameHeader;
//NotUsed
//NotUsed     AddBlockAckSessionRequestFrameType() {
//NotUsed         actionFrameHeader.categoryCode = ActionFrameBlockAckCategoryCode;
//NotUsed         actionFrameHeader.actionCode = AddBlockAckSessionRequestActionCode;
//NotUsed     }
//NotUsed };
//NotUsed
//NotUsed
//NotUsed // Aka ADDBA Response.
//NotUsed
//NotUsed struct AddBlockAckSessionResponseFrameType {
//NotUsed     ActionFrameHeaderType actionFrameHeader;
//NotUsed
//NotUsed     AddBlockAckSessionResponseFrameType() {
//NotUsed         actionFrameHeader.categoryCode = ActionFrameBlockAckCategoryCode;
//NotUsed         actionFrameHeader.actionCode = AddBlockAckSessionResponseActionCode;
//NotUsed     }
//NotUsed };



const unsigned char TrafficIndicatorMapInfoElementId = 1;
const unsigned char RestrictedAccessWindowParmInfoElementId = 2;

struct InfoElementHeaderType {

    unsigned char elementId;
    unsigned char length;

    InfoElementHeaderType() : elementId(UCHAR_MAX), length(0) { }

};//InfoElementHeaderType//



const string SsidWildcardString = "";
const int SSID_LENGTH = 32;
const int SUPPORTED_RATES_LENGTH = 8;


//fixed length: 34(1+1+32) bytes
struct SsidField {
    unsigned char elementId;
    unsigned char length;
    char ssid[SSID_LENGTH];

    bool IsWildcardSsid() const { return (length == 0); }

    bool IsEqualTo(const string& aString) const {
        if (aString.length() != length) {
            return false;
        }
        for(unsigned int i = 0; (i < length); i++) {
            if (aString[i] != ssid[i]) {
                return false;
            }
        }//for//
        return true;
    }//IsEqualTo//

    SsidField()
        :
        length(0) {}

    SsidField(const string& ssidString) {

        assert(ssidString.length() <= SSID_LENGTH);
        length = static_cast<unsigned char>(ssidString.length());
        ssidString.copy(ssid, length);
    }

};//SsidField//


//fixed length: 10(1+1+8) bytes
struct SupportedRatesField {
    unsigned char elementId;
    unsigned char length;
    unsigned char supportedRates[SUPPORTED_RATES_LENGTH];
};//SupportedRatesField//


// "Legacy" = pre-802.11n.

struct LegacyBeaconFrameType {
    ManagementFrameHeader managementHeader;
    EightZeroedBytesStruct notUsed_Timestamp;
    TwoZeroedBytesStruct notUsed_BeaconInterval;
    TwoZeroedBytesStruct notUsed_CapabilityInformation;

    SsidField ssidElement;
    SupportedRatesField notUsed_SupportedRatesElement;

    LegacyBeaconFrameType(const string& ssidString) : ssidElement(ssidString) {}

};//LegacyBeaconFrameType//


struct HtCapabilitiesFrameElementType {
    static const unsigned int elementSize = 28;
    static const unsigned int zeroedBytesSize = elementSize - 1;

    unsigned char aggregateMpdusAreEnabled;

    array<unsigned char, zeroedBytesSize> zeroedBytes;

    HtCapabilitiesFrameElementType() : aggregateMpdusAreEnabled(0) {
        assert(sizeof(HtCapabilitiesFrameElementType) == elementSize);
        zeroedBytes.fill(0);
    }

};//HtCapabilitiesFrameElementType//



struct HtOperationFrameElement {
    static const unsigned int elementSize = 24;
    static const unsigned int zeroedBytesSize = elementSize - MaxNumBondedChannels;
    array<unsigned char, zeroedBytesSize> zeroedBytes;

    // Abstracted:
    array<unsigned char, MaxNumBondedChannels> bondedChannelList;
    unsigned int GetNumberBondedChannels() const
    {
        for(unsigned int i = 0; (i < MaxNumBondedChannels); i++) {
            if (bondedChannelList[i] == UCHAR_MAX) {
                return (i);
            }//if//
        }//for//
        return (MaxNumBondedChannels);
    }

    HtOperationFrameElement() {
        assert(sizeof(HtOperationFrameElement) == elementSize);
        zeroedBytes.fill(0);
        bondedChannelList.fill(UCHAR_MAX);
    }

};//HtOperationFrameElement//


struct BeaconFrame {
    ManagementFrameHeader managementHeader;
    EightZeroedBytesStruct notUsed_Timestamp;
    TwoZeroedBytesStruct notUsed_BeaconInterval;
    TwoZeroedBytesStruct notUsed_CapabilityInformation;

    SsidField ssidElement;
    SupportedRatesField notUsed_SupportedRatesElement;

    HtCapabilitiesFrameElementType theHtCapabilitiesFrameElement;
    HtOperationFrameElement htOperationFrameElement;

    BeaconFrame() {}

    BeaconFrame(const string& ssidString) : ssidElement(ssidString) {}

};//BeaconFrame//


//----------------------------------------------------------
// 802.11ah


struct ResourceAllocationWindowGroupFieldType {
    unsigned char pageIndex:2;
    unsigned char rawStartAidHighBits:6;
    unsigned char rawStartAidLowBits:5;
    unsigned char rawEndAidHighBits:3;
    unsigned char rawEndAidLowBits:8;

    void SetRawAssociationIdRange(
        const AssociationId& startAid,
        const AssociationId& endAid)
    {
        assert((startAid <= endAid) && (endAid <= 8192));
        assert((startAid / 2048) == (endAid / 2048));
        pageIndex = (startAid / 2048);
        rawStartAidHighBits = (startAid % 2048) / 32;
        rawStartAidLowBits = (startAid % 32);
        rawEndAidHighBits = (endAid % 2048) / 256;
        rawEndAidLowBits = (endAid % 256);
    }

    AssociationId GetRawStartAssociationId() const {
        return (
            (static_cast<AssociationId>(pageIndex) * 2048) +
            (static_cast<AssociationId>(rawStartAidHighBits) * 32) +
            static_cast<AssociationId>(rawStartAidLowBits));
    }

    AssociationId GetRawEndAssociationId() const {
        return (
            (static_cast<AssociationId>(pageIndex) * 2048) +
            (static_cast<AssociationId>(rawEndAidHighBits) * 256) +
            static_cast<AssociationId>(rawEndAidLowBits));
    }

};//ResourceAllocationWindowGroupFieldType//



inline
bool IsAMemberOfRawGroupSet(
    const ResourceAllocationWindowGroupFieldType& rawGroupField,
    const AssociationId& theAssociationId)
{
    return ((theAssociationId >= rawGroupField.GetRawStartAssociationId()) &&
            (theAssociationId <= rawGroupField.GetRawEndAssociationId()));
}


struct AhFrameControlFieldType {
    unsigned char notUsedProtocolVersion:2;
    unsigned char frameTypeAndSubtype:6;

    unsigned char nextTbttPresent:1;
    unsigned char compressedSsidPresent:1;
    unsigned char anoPresent:1;
    unsigned char bssBw:3;
    unsigned char security:1;
    unsigned char reserved:1;

    AhFrameControlFieldType() : notUsedProtocolVersion(0), nextTbttPresent(0),
        compressedSsidPresent(0), anoPresent(0), bssBw(0), security(0)
    {
        assert(sizeof(AhFrameControlFieldType) == sizeof(FrameControlField));
    }

};//AhFrameControlFieldType//


// Acronym: RAW = 802.11ah Restricted Access Window


struct RestrictedAccessWindowControlFieldType {
    //unsigned char rawType:2;
    //unsigned char rawTypeOptions:2;
    //unsigned char startTimeIndication:1;
    //unsigned char sameGroupIndication:1;
    //unsigned char channelIndicationPresence:1;
    //unsigned char periodicRawIndication:1;

    // Abstracted:
    unsigned char usesResourceAllocationFrames:1;
    unsigned char notUsed:7;

    RestrictedAccessWindowControlFieldType() : usesResourceAllocationFrames(0), notUsed(0) { }

};


struct RawSlotDefinitionFieldType {
    unsigned char notUsedSlotDefinitionFormatIndication:1;
    unsigned char notUsedCrossSlotBoundary:1;
    unsigned char numberOfSlots:6;
    unsigned char slotDurationCount:8;

    static const SimTime minDuration = 500 * MICRO_SECOND;
    static const SimTime durationPerCount = 120 * MICRO_SECOND;
    static const SimTime maxDuration =
        (minDuration + (durationPerCount * UCHAR_MAX));
    static const unsigned int maxNumberOfSlots = 63;


    void SetSlotDuration(const SimTime& time) {
        assert(time <= maxDuration);
        slotDurationCount =
            static_cast<unsigned char>(
                (time - minDuration) / durationPerCount);
    }

    SimTime GetSlotDuration() const {
        return (minDuration + (slotDurationCount * durationPerCount));
    }

    RawSlotDefinitionFieldType() :
        notUsedSlotDefinitionFormatIndication(0), notUsedCrossSlotBoundary(0) {}
};

//struct RawSlotDefinition1FieldType {
//    uint16_t slotDefinitionFormatIndication:1;
//    uint16_t crossSlotBoundary:1;
//    uint16_t slotDurationCount:11;
//    uint16_t numberOfSlots:3;
//};



// "RPS Element" Format is a standard Element header (InfoElementHeaderType)
// with a list of RAW assignment fields:

struct RestrictedAccessWindowAssignmentFieldType {
    RestrictedAccessWindowControlFieldType rawControl;
    RawSlotDefinitionFieldType rawSlotDefinition;
    ResourceAllocationWindowGroupFieldType rawGroup;

};//RestrictedAccessWindowAssignmentFieldType//



//------------------------------------------------------------------------------


struct SlotAssignmentFieldType {
    uint16_t uplinkIndicator:1;
    // Using a partial ID here "Makes no sense" Using Full AID.

    uint16_t theAssociationId:13;
    uint16_t reserved:2;
    uint16_t slotStartOffset;

};//SlotAssignmentFieldType//


struct AhResourceAllocationFrameHeaderType {
    AhFrameControlFieldType frameControl;
    MacAddress bssId;
    ResourceAllocationWindowGroupFieldType rawGroup;
    uint16_t rawDuration;
    FourZeroedBytesStruct notUsed_FCS;

};//AhResourceAllocationFrameType//


inline
bool IsABeaconFrame(const Packet& aFrame)
{
    const CommonFrameHeader& header =
        aFrame.GetAndReinterpretPayloadData<CommonFrameHeader>();

    return (header.theFrameControlField.frameTypeAndSubtype == BEACON_FRAME_TYPE_CODE);
}


//NotUsed: struct ProbeRequestWildcardSsidFrameType {
//NotUsed:
//NotUsed:     ManagementFrameHeader managementHeader;
//NotUsed:     SupportedRatesField NotUsed:_SupportedRatesElement;
//NotUsed:     TwoZeroedBytesStruct wildcardSsid;
//NotUsed:
//NotUsed:
//NotUsed: };//ProbeRequestWildcardSsidFrameType//


struct ProbeRequestFrame {

    ManagementFrameHeader managementHeader;
    SupportedRatesField notUsed_SupportedRatesElement;
    SsidField ssidElement;

    HtCapabilitiesFrameElementType theHtCapabilitiesFrameElement;

};//ProbeRequestFrame//


typedef BeaconFrame ProbeResponseFrame;


// Modeling Abstraction
struct AhCapabilityInformationType {
    unsigned char usesRestrictedAccessWindows:1;
    unsigned char notUsed:7;
};


struct AssociationRequestFrame {
    ManagementFrameHeader managementHeader;
    OneZeroedByteStruct notUsed_CapabilityInformation;
    AhCapabilityInformationType ahCapabilityInfo;

    TwoZeroedBytesStruct notUsed_ListenInterval;
    SsidField ssidElement;
    SupportedRatesField notUsed_SupportedRatesElement;

    HtCapabilitiesFrameElementType theHtCapabilitiesFrameElement;

    AssociationRequestFrame() { }

    AssociationRequestFrame(const string& ssidString)
        :
        ssidElement(ssidString)
    { }

};//AssociationRequestFrame//


struct AssociationResponseFrame {
    ManagementFrameHeader managementHeader;
    TwoZeroedBytesStruct notUsed_CapabilityInformation;
    TwoZeroedBytesStruct notUsed_StatusCode;
    AssociationId theAssociationId;
    SupportedRatesField notUsed_SupportedRatesElement;

    HtCapabilitiesFrameElementType theHtCapabilitiesFrameElement;

};//AssociationResponseFrame//


struct ReassociationRequestFrame {
    ManagementFrameHeader managementHeader;
    OneZeroedByteStruct notUsed_CapabilityInformation;
    AhCapabilityInformationType ahCapabilityInfo;

    TwoZeroedBytesStruct notUsed_ListenInterval;
    MacAddress currentApAddress;
    SsidField ssidElement;
    SupportedRatesField notUsed_SupportedRatesElement;

    HtCapabilitiesFrameElementType theHtCapabilitiesFrameElement;

    ReassociationRequestFrame() { }

    ReassociationRequestFrame(const string& ssidString)
        :
        ssidElement(ssidString)
    { }

};//ReassociationRequestFrame




typedef AssociationResponseFrame ReassociationResponseFrame;

struct DisassociationFrame {
    ManagementFrameHeader managementHeader;
    TwoZeroedBytesStruct notUsed_reasonCode;

}; //DisassociationFrame


struct AuthenticationFrame {
    ManagementFrameHeader managementHeader;
    TwoZeroedBytesStruct notUsed_AlgorithmNumber;
    TwoZeroedBytesStruct notUsed_SequenceNumber;
    TwoZeroedBytesStruct notUsed_StatusCode;

};//AuthenticationFrame//


struct CommonFrameHeaderWithTransmitterAddressFieldOverlayType {
    CommonFrameHeader header;
    MacAddress transmitterAddress;
};


inline
void CheckFrameHeaderDefinitions()
{
    assert((sizeof(QosNullFrameHeader) + sizeof(Ieee802p2LinkLayerHeader)) ==
           sizeof(QosDataFrameHeader));

    QosDataFrameHeader h1;
    QosNullFrameHeader& h2 = *reinterpret_cast<QosNullFrameHeader*>(&h1);
    assert(&h1.header == &h2.header);
    assert(sizeof(h1.header) == sizeof(h2.header));
    assert(&h1.transmitterAddress == &h2.transmitterAddress);
    assert(sizeof(h1.transmitterAddress) == sizeof(h2.transmitterAddress));
    assert(&h1.theSequenceControlField == &h2.theSequenceControlField);
    assert(sizeof(h1.theSequenceControlField) == sizeof(h2.theSequenceControlField));
    assert(&h1.qosControlField == &h2.qosControlField);
    assert(sizeof(h1.qosControlField) == sizeof(h2.qosControlField));

}//CheckQosNullFrameHeaderDefinition//


inline
MacAddress GetTransmitterAddressFromFrame(const Packet& aFrame)
{
    const CommonFrameHeaderWithTransmitterAddressFieldOverlayType& aHeader =
        aFrame.GetAndReinterpretPayloadData<CommonFrameHeaderWithTransmitterAddressFieldOverlayType>();

    if (aHeader.header.theFrameControlField.frameTypeAndSubtype == CTS_FRAME_TYPE_CODE) {

        // Transmitter address in header.
        return MacAddress::invalidMacAddress;
    }//if//

    return (aHeader.transmitterAddress);

}//GetTransmitterAddressFromFrame//



struct TrafficIndicationMapElementHeader {
    FourZeroedBytesStruct notUsed;
    unsigned char bitMapByteOffset;
};


class TrafficIndicationBitMap {
public:
    TrafficIndicationBitMap() { (*this).Clear(); }

    void Clear();

    bool IsEmpty() const { return (bitVector.empty()); }

    void AddBit(const AssociationId theAssociationId);
    bool BitIsSet(const AssociationId theAssociationId) const;

    static
    bool BitIsSetInRawBitMap(
        const unsigned char rawBitMapData[],
        const unsigned int rawBitMapSizeBytes,
        const unsigned char bitMapByteOffset,
        const AssociationId theAssociationId);

    const vector<unsigned char>& GetBitMapByteVector() const { return bitVector; }

    unsigned char GetStartByteOffset() const { return (static_cast<unsigned char>(startByteOffset)); }

private:
    unsigned int startByteOffset;
    vector<unsigned char> bitVector;

    void SetBit(const AssociationId theAssociationId);

};//TrafficIndicationBitMap//


inline
void TrafficIndicationBitMap::Clear()
{
    startByteOffset = 0;
    bitVector.clear();
}


inline
void TrafficIndicationBitMap::SetBit(const AssociationId theAssociationId)
{
    const unsigned int byteOffset = theAssociationId / 8;
    assert(byteOffset >= startByteOffset);

    const unsigned int byteIndex = (byteOffset - startByteOffset);
    const unsigned int bitPos = theAssociationId % 8;

    assert(byteIndex < bitVector.size());

    bitVector[byteIndex] |= (1 << bitPos);

}//SetBit//


inline
void TrafficIndicationBitMap::AddBit(const AssociationId theAssociationId)
{
    assert(theAssociationId != 0);

    const unsigned int byteOffset = theAssociationId / 8;
    const unsigned int endbyteOffset = static_cast<unsigned int>(startByteOffset + bitVector.size() - 1);

    if (bitVector.empty()) {
        startByteOffset = byteOffset;
        bitVector.resize(1, 0);
    }
    else if (startByteOffset > byteOffset) {

        const unsigned int oldstartByteOffset = startByteOffset;
        (*this).startByteOffset = byteOffset;
        const size_t originalSize = bitVector.size();

        bitVector.resize((bitVector.size() + (oldstartByteOffset - startByteOffset)), 0);

        std::copy_backward(
            bitVector.begin(),
            (bitVector.begin() + originalSize),
            (bitVector.begin() + (oldstartByteOffset - startByteOffset)));
    }
    else if (endbyteOffset < byteOffset) {
        bitVector.resize(((byteOffset - startByteOffset) + 1), 0);
    }//if/

    (*this).SetBit(theAssociationId);

    assert(startByteOffset <= UCHAR_MAX);

}//AddBit//


inline
bool TrafficIndicationBitMap::BitIsSet(const AssociationId theAssociationId) const
{
    const unsigned int byteOffset = theAssociationId / 8;
    const unsigned int endByteOffset = static_cast<unsigned int>(startByteOffset + bitVector.size() - 1);

    if ((byteOffset < startByteOffset) || (byteOffset > endByteOffset)) {
         return false;
    }
    else {
        const unsigned int bitPos = theAssociationId % 8;
        return ((bitVector[(byteOffset - startByteOffset)] & (1 << bitPos)) == 1);
    }//if//


}//BitIsSet//

inline
bool TrafficIndicationBitMap::BitIsSetInRawBitMap(
    const unsigned char rawBitMapData[],
    const unsigned int rawBitMapSizeBytes,
    const unsigned char bitMapByteOffset,
    const AssociationId theAssociationId)
{
    const unsigned int byteOffset = theAssociationId / 8;
    const unsigned int endByteOffset = bitMapByteOffset + rawBitMapSizeBytes - 1;

    if ((byteOffset < bitMapByteOffset) || (byteOffset > endByteOffset)) {
         return false;
    }
    else {
        const unsigned int bitPos = theAssociationId % 8;
        return ((rawBitMapData[(byteOffset - bitMapByteOffset)] & (1 << bitPos)) == 1);
    }//if//

}//BitIsSetInRawBitMap//



}//namespace//


#endif
