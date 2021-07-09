// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "geonet_net.h"

const string GeoNet::BasicTransportProtocol::modelName = "Btp";
const string GeoNet::GeoNetworkingProtocol::modelName = "GeoNet";
const string GeoNet::GeoNetMac::dccModelName = "GeoNetDCC";

namespace GeoNet {


void GeoNetworkingProtocol::SendSHBPacket(
    unique_ptr<Packet>& packetPtr,
    const PacketPriority& trafficClass,
    const SimTime& maxPacketLifetime,
    const SimTime& repetitionInterval)
{

    //TBD
    CommonHeaderType commonHeaderType(
        NH_BTP_B, HT_TSB, HST_SINGLE_HOP, trafficClass, 1);

    packetPtr->AddPlainStructHeader(
        commonHeaderType);

    (*this).InsertPacketIntoTransmitQueue(packetPtr, NetworkAddress::broadcastAddress, trafficClass);

    //TBD: debug
    //(*this).InsertPacketIntoTransmitQueueAndScheduleEventIfNecessary(
    //    packetPtr, NetworkAddress::broadcastAddress, trafficClass, maxPacketLifetime, repetitionInterval);

}//SendSHBPacket//




}//namespace//
