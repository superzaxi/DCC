// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scensim_netsim.h"
#include "scensim_mobileip.h"
#include "scensim_arp.h"
#include "scensim_netif.h"
#include "scensim_dhcp.h"
#include "scensim_iscdhcp.h"

#include <iostream>//20210604

using std::cout;//20210604
using std::endl;//20210604

namespace ScenSim {


// Could be put in own "scensim_qoscontrol.cpp" file.

const FlowId FlowId::nullFlowId;


void BasicNetworkLayer::InitMobileIpMobileNodeSubsystem(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    assert(mobileIpHomeAgentSubsystemPtr == nullptr);

    mobileIpMobileNodeSubsystemPtr.reset(
        new MobileIpMobileNodeSubsystem(
            theParameterDatabaseReader,
            simulationEngineInterfacePtr,
            shared_from_this(),
            hopLimit));

    (*this).primaryNetworkAddress = mobileIpMobileNodeSubsystemPtr->GetHomeAddress();
}


void BasicNetworkLayer::InitMobileIpHomeAgentSubsystem(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    assert(mobileIpMobileNodeSubsystemPtr == nullptr);

    mobileIpHomeAgentSubsystemPtr.reset(
        new MobileIpHomeAgentSubsystem(
            theParameterDatabaseReader,
            simulationEngineInterfacePtr,
            shared_from_this()));
}


bool BasicNetworkLayer::IsOneOfMyMobileIpHomeAddresses(
    const NetworkAddress& anAddress, const unsigned int interfaceIndex)
{
    if (mobileIpMobileNodeSubsystemPtr == nullptr) {
        return false;
    }//if//
    return (mobileIpMobileNodeSubsystemPtr->GetHomeAddress() == anAddress);
}


void BasicNetworkLayer::SetInterfaceIpAddress(
    const unsigned int interfaceIndex,
    const NetworkAddress& newInterfaceAddress,
    const unsigned int subnetMaskLengthBits)
{
    NetworkInterfaceInfoType& interfaceInfo = networkInterfaces.at(interfaceIndex);

    if (interfaceInfo.address != newInterfaceAddress) {

        OutputTraceAndStatsForIpAddressChanged(
            interfaceIndex, newInterfaceAddress, subnetMaskLengthBits);

        interfaceInfo.address = newInterfaceAddress;
        interfaceInfo.subnetMaskLengthBits = subnetMaskLengthBits;
        interfaceInfo.subnetMask = NetworkAddress::MakeSubnetMask(subnetMaskLengthBits);
        if (interfaceInfo.isPrimary) {
            primaryNetworkAddress = newInterfaceAddress;
        }//if//

        if (mobileIpMobileNodeSubsystemPtr != nullptr) {
            mobileIpMobileNodeSubsystemPtr->HandleMajorInterfaceStatusChange(interfaceIndex);
        }//if//

        interfaceInfo.networkInterfaceManagerPtr->NotifyProtocolAddressChanged();

        for(size_t i = 0; i < networkAddressInterfaces.size(); ++i) {
            networkAddressInterfaces[i]->NotifyNetworkAddressIsChanged(
                interfaceIndex,
                interfaceInfo.address,
                interfaceInfo.subnetMaskLengthBits);
        }//for//
    }//if//

}//SetInterfaceIpAddress//


void BasicNetworkLayer::SetupInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceId& theInterfaceId)
{
    networkInterfaces.push_back(NetworkInterfaceInfoType());

    NetworkInterfaceInfoType& interfaceInfo = networkInterfaces.back();

    interfaceInfo.networkInterfaceManagerPtr.reset(
        new NetworkInterfaceManager(
            theParameterDatabaseReader,
            theNodeId,
            theInterfaceId,
            static_cast<unsigned int>((networkInterfaces.size() - 1)),
            simulationEngineInterfacePtr,
            shared_from_this(),
            nodeSeed));

    interfaceInfo.interfaceName = theInterfaceId;
    string networkAddressString;
    if (theParameterDatabaseReader.ParameterExists("network-address", theNodeId, theInterfaceId)) {
        networkAddressString = theParameterDatabaseReader.ReadString("network-address", theNodeId, theInterfaceId);
    }
    else {
        cerr << "Error: No network-address for node and interface: "
             << theNodeId << ' ' << theInterfaceId << endl;
        exit(1);
    }//if//

    bool success = false;

    interfaceInfo.address.SetAddressFromString(networkAddressString, theNodeId, success);

    if (!success) {
        cerr << "Error: bad network-address format: " << networkAddressString;
        cerr << " for node: " << theNodeId << endl;
        exit(1);

    }//if//

    interfaceInfo.networkInterfaceManagerPtr->NotifyProtocolAddressChanged();

    if (mobileIpMobileNodeSubsystemPtr != nullptr) {
        mobileIpMobileNodeSubsystemPtr->AddInterfaceIfMobileIpEnabled(
            theParameterDatabaseReader,
            static_cast<unsigned int>((networkInterfaces.size() - 1)),
            theInterfaceId);
    }//if//

    interfaceInfo.isPrimary = false;

    if (theParameterDatabaseReader.ParameterExists("network-address-is-primary", theNodeId, theInterfaceId)) {

        if (theParameterDatabaseReader.ReadBool("network-address-is-primary", theNodeId, theInterfaceId) == true) {

            if (!primaryNetworkAddress.IsAnyAddress()) {
                cerr << "Error: Too many primary network addresses for node: " << theNodeId << endl;
                exit(1);
            }//if//

           interfaceInfo.isPrimary = true;
           (*this).primaryNetworkAddress = interfaceInfo.address;
        }//if//

    }//if//

    unsigned int prefixLengthBits;
    if (theParameterDatabaseReader.ParameterExists("network-prefix-length-bits", theNodeId, theInterfaceId)) {
        prefixLengthBits = theParameterDatabaseReader.ReadPositiveInt("network-prefix-length-bits", theNodeId, theInterfaceId);
    }
    else {
        cerr << "Error: No network-prefix-length-bits for node and interface: "
             << theNodeId << ' ' << theInterfaceId << endl;
        exit(1);
    }//if//

    if (prefixLengthBits >= NetworkAddress::numberBits) {
        cerr << "Error: Bad network-prefix-length-bits parameter, value = " << prefixLengthBits << endl;
        exit(1);

    }//if//

    if ((NetworkAddress::numberBits == 128) && (NetworkAddress::IsIpv4StyleAddressString(networkAddressString))) {
        //ajust prefix length from IPv4(32bit) style to IPv6(128bit) style
        prefixLengthBits += 96;
    }//if//

    interfaceInfo.subnetMaskLengthBits = prefixLengthBits;
    interfaceInfo.subnetMask = NetworkAddress::MakeSubnetMask(prefixLengthBits);

    interfaceInfo.subnetIsMultiHop = false;

    if (theParameterDatabaseReader.ParameterExists("network-subnet-is-multihop", theNodeId, theInterfaceId)) {
        interfaceInfo.subnetIsMultiHop =
            theParameterDatabaseReader.ReadBool("network-subnet-is-multihop", theNodeId, theInterfaceId);
    }//if//

    interfaceInfo.gatewayIsForcedNextHop = false;

    interfaceInfo.allowRoutingBackOutSameInterface = false;

    if (theParameterDatabaseReader.ParameterExists(
        "network-allow-routing-back-out-same-interface", theNodeId, theInterfaceId)) {

        interfaceInfo.allowRoutingBackOutSameInterface =
            theParameterDatabaseReader.ReadBool(
            "network-allow-routing-back-out-same-interface", theNodeId, theInterfaceId);
    }//if//

    interfaceInfo.ignoreUnregisteredProtocol = false;

    if (theParameterDatabaseReader.ParameterExists(
        "network-ignore-unregistered-protocol", theNodeId, theInterfaceId)) {

        interfaceInfo.ignoreUnregisteredProtocol =
            theParameterDatabaseReader.ReadBool(
            "network-ignore-unregistered-protocol", theNodeId, theInterfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("network-gateway-address", theNodeId, theInterfaceId)) {

        string gatewayAddressString = theParameterDatabaseReader.ReadString("network-gateway-address", theNodeId, theInterfaceId);

        interfaceInfo.gatewayAddress.SetAddressFromString(gatewayAddressString, theNodeId, success);

        if (!success) {
            cerr << "Error: bad network-gateway-address format: " << gatewayAddressString;
            cerr << " for node: " << theNodeId << endl;
            exit(1);

        }//if//

        if (gatewayAddressExists) {
            cerr << "Error: two interfaces on node: " << theNodeId << " have specified default gateways." << endl;
            exit(1);

        }//if//


        if(!interfaceInfo.gatewayAddress.IsInSameSubnetAs(interfaceInfo.address, interfaceInfo.subnetMask)) {
            cerr << "Error: on node/interface: " << theNodeId << "/" << theInterfaceId
                 << " specified gateway is not in the interface's subnet." << endl;
            exit(1);

        }//if//

        (*this).gatewayAddressExists = true;
        (*this).gatewayInterfaceIndex = static_cast<unsigned int>(networkInterfaces.size() - 1);

        if (primaryNetworkAddress != NetworkAddress::invalidAddress) {
            (*this).primaryNetworkAddress = interfaceInfo.address;
        }//if//

    }//if//

    if (theParameterDatabaseReader.ParameterExists("network-mtu-bytes", theNodeId, theInterfaceId)) {
        interfaceInfo.maxIpPacketSizeAkaMtuBytes =
            theParameterDatabaseReader.ReadNonNegativeInt("network-mtu-bytes", theNodeId, theInterfaceId);
    }//if//

    OutputTraceAndStatsForIpAddressChanged(
        static_cast<unsigned int>((networkInterfaces.size() - 1)),
        interfaceInfo.address,
        interfaceInfo.subnetMaskLengthBits);

    CheckTheNecessityOfMacAddressResolution(
        theParameterDatabaseReader,
        theInterfaceId,
        interfaceInfo.address,
        interfaceInfo.subnetMask);

}//SetupInterface//



void BasicNetworkLayer::ReceiveOutgoingPreformedNetworkPacket(unique_ptr<Packet>& packetPtr)
{
    const IpHeaderOverlayModel ipHeader(packetPtr->GetRawPayloadData(), packetPtr->LengthBytes());

    if ((mobileIpMobileNodeSubsystemPtr != nullptr) &&
        (mobileIpMobileNodeSubsystemPtr->GetHomeAddress() == ipHeader.GetSourceAddress())) {

        mobileIpMobileNodeSubsystemPtr->TunnelPacketToCorrespondentNode(
            packetPtr,
            ipHeader.GetSourceAddress(),
            ipHeader.GetDestinationAddress(),
            ipHeader.GetTrafficClass(),
            ipHeader.GetNextHeaderProtocolCode());

        return;

    }//if//


    bool foundARoute;
    unsigned int interfaceIndex;
    NetworkAddress nextHopAddress;

    GetNextHopAddressAndInterfaceIndexForDestination(
        ipHeader.GetDestinationAddress(), foundARoute, nextHopAddress, interfaceIndex);

    if (!foundARoute) {
        bool wasAcceptedForRouting = false;

        (*this).GiveNetworkPacketToOnDemandRoutingProtocol(
            packetPtr,
            ipHeader.GetSourceAddress(),
            ipHeader.GetDestinationAddress(),
            wasAcceptedForRouting);

        if (!wasAcceptedForRouting) {
            // Drop packet.

            OutputTraceAndStatsForNoRouteDrop(*packetPtr, ipHeader.GetDestinationAddress());

            packetPtr = nullptr;
        }//if//

        return;
    }//if//

    (*this).InsertPacketIntoAnOutputQueue(
        packetPtr,
        interfaceIndex,
        nextHopAddress,
        ipHeader.GetTrafficClass());

}//ReceiveOutgoingPreformedNetworkPacket//


inline
void BasicNetworkLayer::SendBroadcastOrMulticastPacket(
    unique_ptr<Packet>& packetPtr,
    const unsigned int interfaceIndex,
    const NetworkAddress& destinationAddress,
    const PacketPriority trafficClass,
    const unsigned char protocol)
{
    //insert queue
    //cout << "SendBroadcastOrMulticastPacket" << endl;

    IpHeaderModel
        header(
            trafficClass,
            packetPtr->LengthBytes(),
            hopLimit,
            protocol,
            networkInterfaces.at(interfaceIndex).address,
            destinationAddress);

    packetPtr->AddRawHeader(header.GetPointerToRawBytes(), header.GetNumberOfRawBytes());
    packetPtr->AddTrailingPadding(header.GetNumberOfTrailingBytes());

    (*this).InsertPacketIntoAnOutputQueue(
        packetPtr, interfaceIndex, destinationAddress, trafficClass);

}//SendBroadcastOrMulticastPacket//


void BasicNetworkLayer::BroadcastPacketOnAllInterfaces(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& destinationAddress,
    const PacketPriority trafficClass,
    const unsigned char protocol)
{
    //insert queue
    //cout << "BroadcastPacketOnAllInterfaces" << endl;
    
    if (networkInterfaces.size() == 0) {
        packetPtr = nullptr;
        return;
    }//if//

    for(unsigned int interfaceIndex = 0; (interfaceIndex < networkInterfaces.size()); interfaceIndex++) {
        unique_ptr<Packet> packetToSendPtr;

        if (interfaceIndex < (networkInterfaces.size() - 1)) {
            // Copy packet
            packetToSendPtr = unique_ptr<Packet>(new Packet(*packetPtr));
        }
        else {
            // Last interface, send original packet.

            packetToSendPtr = move(packetPtr);
        }//if//

        SendBroadcastOrMulticastPacket(
            packetToSendPtr,
            interfaceIndex,
            destinationAddress,
            trafficClass,
            protocol);

    }//for//

}//BroadcastPacketOnAllInterfaces//


NetworkAddress BasicNetworkLayer::GetSourceAddressForDestination(const NetworkAddress& destinationAddress) const
{
    if (mobileIpMobileNodeSubsystemPtr != nullptr) {
        return (mobileIpMobileNodeSubsystemPtr->GetHomeAddress());
    }//if//

    bool success;
    NetworkAddress notUsed;
    unsigned int interfaceIndex;
    (*this).GetNextHopAddressAndInterfaceIndexForDestination(
        destinationAddress,
        success,
        notUsed,
        interfaceIndex);

    if (!success) {
        cerr << "Error in GetSourceAddressForDestination: At node " << theNodeId << " Destination Address: "
             << destinationAddress.ConvertToString() << " is not reachable." << endl;
        exit(1);
    }//if//

    return networkInterfaces.at(interfaceIndex).address;

}//GetSourceAddressForDestination//


void BasicNetworkLayer::ReceivePacketFromUpperLayer(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& initialSourceAddress,
    const NetworkAddress& destinationAddress,
    PacketPriority trafficClass,
    const unsigned char protocol)
{
    //insert queue
    //cout << "ReceivePacketFromUpperLayer" << endl;
    if (NetworkAddressIsMyAddress(destinationAddress)) {
        ProcessLoopbackPacket(packetPtr, initialSourceAddress, destinationAddress, trafficClass, protocol);
        return;
    }//if//

    NetworkAddress sourceAddress = initialSourceAddress;
    //cout << "ReceivePacketFromUpperLayer2" << endl;//yes

    // Check Mobile IP to see if we are a Mobile Node and the packet must go through Home Agent.

    if ((mobileIpMobileNodeSubsystemPtr != nullptr) &&
        (mobileIpMobileNodeSubsystemPtr->GetHomeAddress() == sourceAddress)) {

        mobileIpMobileNodeSubsystemPtr->TunnelPacketToCorrespondentNode(
            packetPtr, sourceAddress, destinationAddress, trafficClass, protocol);

        return;

    }//if//

    //insert queue

    //Future Feature// // Check Mobile IP to see if we are a Home Agent for the address.
    //Future Feature//
    //Future Feature// if (mobileIpHomeAgentSubsystemPtr != nullptr) {
    //Future Feature//     bool packetWasRouted = false;
    //Future Feature//
    //Future Feature//     mobileIpHomeAgentSubsystemPtr->TunnelPacketIfHaveCareOfAddress(
    //Future Feature//         destinationAddress, packetPtr, packetWasRouted);
    //Future Feature//
    //Future Feature//     if (packetWasRouted) {
    //Future Feature//         return;
    //Future Feature//     }//if//
    //Future Feature//
    //Future Feature// }//if//

    if ((sourceAddress.IsAnyAddress()) &&
        (destinationAddress.IsTheBroadcastOrAMulticastAddress())) {

        (*this).BroadcastPacketOnAllInterfaces(
            packetPtr, destinationAddress, trafficClass, protocol);

        return;

    }//if//

    //cout << "ReceivePacketFromUpperLayer4" << endl;//no

    bool foundARoute = false;
    unsigned int interfaceIndex;
    NetworkAddress nextHopAddress;

    if (destinationAddress.IsTheBroadcastOrAMulticastAddress() ||
        destinationAddress.IsLinkLocalAddress()) {

        foundARoute = true;
        nextHopAddress = destinationAddress;
        interfaceIndex = LookupInterfaceIndex(sourceAddress);
    }
    else {
        GetNextHopAddressAndInterfaceIndexForDestination(
            destinationAddress, foundARoute, nextHopAddress, interfaceIndex);
    }//if//

    //cout << "ReceivePacketFromUpperLayer5" << endl;//no

    if (foundARoute) {
        (*this).NotifySendingOrForwardingDataPacketToOnDemandRoutingProtocol(sourceAddress, sourceAddress, nextHopAddress, destinationAddress);
    }
    else {
        bool wasAcceptedForRouting = false;

        IpHeaderModel
            header(
                trafficClass,
                packetPtr->LengthBytes(),
                hopLimit,
                protocol,
                NetworkAddress::anyAddress, // Null Address, will be set later.
                destinationAddress);

        packetPtr->AddRawHeader(header.GetPointerToRawBytes(), header.GetNumberOfRawBytes());
        packetPtr->AddTrailingPadding(header.GetNumberOfTrailingBytes());

        (*this).GiveNetworkPacketToOnDemandRoutingProtocol(
            packetPtr,
            sourceAddress,
            destinationAddress,
            wasAcceptedForRouting);

        if (!wasAcceptedForRouting) {
            // Drop packet.

            OutputTraceAndStatsForNoRouteDrop(*packetPtr, destinationAddress);

            packetPtr = nullptr;
        }//if//

        return;
    }//if//

    //cout << "ReceivePacketFromUpperLayer6" << endl;//no

    if (sourceAddress == NetworkAddress::anyAddress) {
        sourceAddress = networkInterfaces.at(interfaceIndex).address;
    }//if//

    IpHeaderModel
        header(
            trafficClass,
            packetPtr->LengthBytes(),
            hopLimit,
            protocol,
            sourceAddress,
            destinationAddress);

    //cout << "ReceivePacketFromUpperLayer7" << endl;//no

    packetPtr->AddRawHeader(header.GetPointerToRawBytes(), header.GetNumberOfRawBytes());
    packetPtr->AddTrailingPadding(header.GetNumberOfTrailingBytes());

    (*this).InsertPacketIntoAnOutputQueue(packetPtr, interfaceIndex, nextHopAddress, trafficClass);

}//ReceivePacketFromUpperLayer//


void BasicNetworkLayer::ReceiveOutgoingBroadcastPacket(
    unique_ptr<Packet>& packetPtr,
    const unsigned int interfaceIndex,
    const PacketPriority trafficClass,
    const unsigned char protocol)
{
    (*this).SendBroadcastOrMulticastPacket(
        packetPtr,
        interfaceIndex,
        NetworkAddress::broadcastAddress,
        trafficClass,
        protocol);

}//ReceiveOutgoingBroadcastPacket//



void BasicNetworkLayer::ReceivePacketFromMac(
    const unsigned int interfaceIndex,
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& lastHopAddress,
    const EtherTypeField etherType)
{
    if (etherType == ETHERTYPE_ARP) {
        NetworkInterfaceInfoType& interface = networkInterfaces.at(interfaceIndex);
        interface.networkInterfaceManagerPtr->ProcessArpPacket(packetPtr);
        assert(packetPtr == nullptr);
        return;
    }//if//

    OutputTraceAndStatsForReceivePacketFromMac(*packetPtr);

    IpHeaderOverlayModel ipHeader(packetPtr->GetRawPayloadData(), packetPtr->LengthBytes());

    NetworkAddress sourceAddress(ipHeader.GetSourceAddress());
    NetworkAddress destinationAddress(ipHeader.GetDestinationAddress());
    PacketPriority trafficClass(ipHeader.GetTrafficClass());
    unsigned char currentHopLimit(ipHeader.GetHopLimit());
    assert(currentHopLimit != 0);

    if (NetworkAddressIsForThisNode(destinationAddress, interfaceIndex)) {
        // Packet is for me or everyone, send it up the stack.

        unsigned char protocolNum;
        unsigned int ipHeaderLength;
        ipHeader.GetHeaderTotalLengthAndNextHeaderProtocolCode(ipHeaderLength, protocolNum);

        if ((mobileIpHomeAgentSubsystemPtr != nullptr) &&
            (ipHeader.MobilityExtensionHeaderExists())) {

            mobileIpHomeAgentSubsystemPtr->ProcessMobileIpProtocolOptions(ipHeader);
            assert(protocolNum == IpHeaderModel::ipProtoNoneProtocolNumber);
            packetPtr = nullptr;
            return;
        }//if//

        ipHeader.StopOverlayingHeader();
        packetPtr->DeleteHeader(ipHeaderLength);

        if (protocolNum == IpHeaderModel::ipInIpProtocolNumber) {

            (*this).ReceivePacketFromMac(interfaceIndex, packetPtr, lastHopAddress, etherType);
            return;

        }//if//

        if (protocolNum == IP_PROTOCOL_NUMBER_ICMP) {
            NetworkInterfaceInfoType& interface = networkInterfaces.at(interfaceIndex);
            interface.networkInterfaceManagerPtr->ProcessIcmpPacket(
                packetPtr,
                sourceAddress,
                destinationAddress,
                trafficClass,
                lastHopAddress);
            if (packetPtr != nullptr) {
                if (!networkInterfaces.at(interfaceIndex).ignoreUnregisteredProtocol) {
                    assert(false && "An upper layer cannot be prepared.");
                }//if//
                packetPtr = nullptr;
            }//if//
            return;
        }//if//

        map<unsigned char, shared_ptr<ProtocolPacketHandler> >::iterator mapIter =
            protocolPacketHandlerMap.find(protocolNum);

        if (mapIter != protocolPacketHandlerMap.end()) {
            mapIter->second->ReceivePacketFromNetworkLayer(
                packetPtr,
                sourceAddress,
                destinationAddress,
                trafficClass,
                lastHopAddress,
                currentHopLimit,
                interfaceIndex);
        }
        else {
            if (!networkInterfaces.at(interfaceIndex).ignoreUnregisteredProtocol) {
                assert(false && "An upper layer cannot be prepared.");
            }//if//

            packetPtr = nullptr;
        }//if//
    }
    else {
        // "Routing" packet to new interface.

        currentHopLimit -= 1;
        if (currentHopLimit == 0) {
            OutputTraceAndStatsForHopLimitDrop(*packetPtr, destinationAddress);

            packetPtr = nullptr;

            return;
        }
        else {
            ipHeader.SetHopLimit(currentHopLimit);
        }//if//

        bool foundARoute;
        unsigned int newInterfaceIndex;
        NetworkAddress nextHopAddress;

        // Try Mobile IP to see if we are a Home Agent for the address.

        if (mobileIpHomeAgentSubsystemPtr != nullptr) {
            bool packetWasRouted = false;

            mobileIpHomeAgentSubsystemPtr->RoutePacketIfHaveCareOfAddress(
                destinationAddress, packetPtr, packetWasRouted);

            if (packetWasRouted) {
                return;
            }//if//

        }//if//

        GetNextHopAddressAndInterfaceIndexForDestination(
            destinationAddress, foundARoute, nextHopAddress, newInterfaceIndex);

        if (foundARoute) {
            (*this).NotifySendingOrForwardingDataPacketToOnDemandRoutingProtocol(lastHopAddress, sourceAddress, nextHopAddress, destinationAddress);
        }
        else {
            bool wasAcceptedForRouting = false;

            (*this).GiveNetworkPacketToOnDemandRoutingProtocol(
                packetPtr,
                sourceAddress,
                destinationAddress,
                lastHopAddress,
                wasAcceptedForRouting);

            if (!wasAcceptedForRouting) {
                // Drop packet.

                OutputTraceAndStatsForNoRouteDrop(*packetPtr, destinationAddress);

                packetPtr = nullptr;

            }//if//

            return;

        }//if//

        if ((newInterfaceIndex == interfaceIndex) &&
            (!networkInterfaces.at(interfaceIndex).allowRoutingBackOutSameInterface)) {

            cerr << "Packet routed back out same interface and " << endl
                 << "   network-allow-routing-back-out-same-interface not set (true)." << endl;
            exit(1);

        }//if//

        (*this).InsertPacketIntoAnOutputQueue(packetPtr, newInterfaceIndex, nextHopAddress, trafficClass);

    }//if//

    assert(packetPtr == nullptr);

}//ReceivePacketFromMac//


void BasicNetworkLayer::GetTransportLayerPortNumbersFromIpPacket(
    const Packet& aPacket,
    bool& portNumbersWereRetrieved,
    unsigned short int& sourcePort,
    unsigned short int& destinationPort)
{
    typedef map<unsigned char, shared_ptr<ProtocolPacketHandler> >::const_iterator IterType;
    const IpHeaderOverlayModel ipHeader(aPacket.GetRawPayloadData(), aPacket.LengthBytes());

    portNumbersWereRetrieved = false;

    unsigned char protocolNum;
    unsigned int ipHeaderLength;
    ipHeader.GetHeaderTotalLengthAndNextHeaderProtocolCode(ipHeaderLength, protocolNum);

    IterType foundIter = protocolPacketHandlerMap.find(protocolNum);
    if (foundIter != protocolPacketHandlerMap.end()) {
        const ProtocolPacketHandler& protocolHandler = *foundIter->second;

        protocolHandler.GetPortNumbersFromPacket(
            aPacket,
            ipHeaderLength,
            portNumbersWereRetrieved,
            sourcePort,
            destinationPort);

    }//if//

}//GetTransportLayerPortNumbersFromIpPacket//


void BasicNetworkLayer::ProcessLinkIsUpNotification(const unsigned int interfaceIndex)
{
    NetworkInterfaceInfoType& interface = networkInterfaces.at(interfaceIndex);
    interface.networkInterfaceManagerPtr->ProcessLinkIsUpNotification();

    if (interface.dhcpClientInterfacePtr != nullptr) {
        interface.dhcpClientInterfacePtr->HandleLinkIsUpNotification(
            interfaceIndex,
            interface.macLayerPtr->GetGenericMacAddress());
    }//if//

}//ProcessLinkIsUpNotification//


void BasicNetworkLayer::ProcessLinkIsDownNotification(const unsigned int interfaceIndex)
{
    NetworkInterfaceInfoType& interface = networkInterfaces.at(interfaceIndex);
    interface.networkInterfaceManagerPtr->ProcessLinkIsDownNotification();
}


void BasicNetworkLayer::ProcessNewLinkToANodeNotification(
    const unsigned int interfaceIndex,
    const GenericMacAddress& newNodeMacAddress)
{
    NetworkInterfaceInfoType& interface = networkInterfaces.at(interfaceIndex);
    interface.networkInterfaceManagerPtr->ProcessNewLinkToANodeNotification(newNodeMacAddress);

    if (interface.networkInterfaceManagerPtr->IsAddressResolutionEnabled()) {
        if (interfaceIndex == gatewayInterfaceIndex) {
            interface.gatewayAddress =
                NetworkAddress(
                    GetSubnetAddress(interfaceIndex),
                    NetworkAddress(CalcNodeId(newNodeMacAddress)));
        }//if//
    }//if//

}//ProcessNewLinkToANodeNotification//


void BasicNetworkLayer::InsertPacketIntoAnOutputQueue(
    unique_ptr<Packet>& packetPtr,
    const unsigned int interfaceIndex,
    const NetworkAddress& nextHopAddress,
    const PacketPriority initialTrafficClass,
    const EtherTypeField etherType)
{
    //insert queue
    //cout << "InsertPacketIntoAnOutputQueue: " << theNodeId << endl;
    
    NetworkInterfaceInfoType interface = networkInterfaces.at(interfaceIndex);

    if ((interface.outputQueuePtr == nullptr) ||
        (interface.macLayerPtr == nullptr)) {
        cerr << "Error in Network Layer Model: No communication interface(or mac) to send packet." << endl;
        exit(1);
    }//if//


    if (packetPtr->LengthBytes() > interface.maxIpPacketSizeAkaMtuBytes) {
        cerr << "Error in Network Layer Model: IP Packet size is too large for interface." << endl;
        cerr << "  IP Packet Size = " << packetPtr->LengthBytes()
             << "  Max MTU = " << interface.maxIpPacketSizeAkaMtuBytes << endl;
        exit(1);
    }//if//

    //cout << "InsertPacketIntoAnOutputQueue2" << endl;

    PacketPriority trafficClass = initialTrafficClass;
    if (trafficClass == MAX_AVAILABLE_PACKET_PRIORITY) {
        trafficClass = interface.outputQueuePtr->MaxPossiblePacketPriority();
    }//if//

    //cout << "InsertPacketIntoAnOutputQueue3" << endl;

    if (!interface.networkInterfaceManagerPtr->IsAddressResolutionCompleted(nextHopAddress)) {
        interface.networkInterfaceManagerPtr->SendAddressResolutionRequest(
            packetPtr, nextHopAddress, trafficClass);
        return;
    }//if//

    //cout << "InsertPacketIntoAnOutputQueue4" << endl;

    EnqueueResultType enqueueResult;
    unique_ptr<Packet> packetToDropPtr;

    OutputTraceAndStatsForInsertPacketIntoQueue(*packetPtr);

    InterfaceOutputQueue& outputQueue = *interface.outputQueuePtr;

    if (!outputQueue.InsertWithFullPacketInformationModeIsOn()) {
        //insert queue
        outputQueue.Insert(packetPtr, nextHopAddress, trafficClass, enqueueResult, packetToDropPtr, etherType);
        //outputQueue.Insert(packetPtr, nextHopAddress, trafficClass, enqueueResult, packetToDropPtr, etherType, theNodeId);
        //cout << "insert_before_1" << endl;//yes
    }
    else {
        //cout << "InsertPacketIntoAnOutputQueue_else" << endl;//no
        const IpHeaderOverlayModel ipHeader(packetPtr->GetRawPayloadData(), packetPtr->LengthBytes());

        bool portNumbersWereRetrieved;
        unsigned short int sourcePort = ANY_PORT;
        unsigned short int destinationPort = ANY_PORT;

        GetTransportLayerPortNumbersFromIpPacket(
            *packetPtr,
            portNumbersWereRetrieved,
            sourcePort,
            destinationPort);

        unsigned short int ipv6FlowLabel = NULL_FLOW_LABEL;

        if (IpHeaderModel::usingVersion6) {
            ipv6FlowLabel = ipHeader.GetFlowLabel();
        }//if//

        outputQueue.InsertWithFullPacketInformation(
            packetPtr,
            nextHopAddress,
            ipHeader.GetSourceAddress(),
            sourcePort,
            ipHeader.GetDestinationAddress(),
            destinationPort,
            ipHeader.GetNextHeaderProtocolCode(),
            trafficClass,
            ipv6FlowLabel,
            enqueueResult,
            packetToDropPtr);
        //cout << "insert_before_2" << endl;//no

    }//if//

    //cout << "InsertPacketIntoAnOutputQueue5" << endl;

    if (enqueueResult != ENQUEUE_SUCCESS) {

        OutputTraceAndStatsForFullQueueDrop(*packetToDropPtr, enqueueResult);

        packetToDropPtr = nullptr;
        packetPtr = nullptr;
    }//if//

    //cout << "InsertPacketIntoAnOutputQueue6" << endl;

    interface.macLayerPtr->NetworkLayerQueueChangeNotification();

    //insert queue

}//InsertPacketIntoAnOutputQueue//


void BasicNetworkLayer::SetupDhcpServerAndClientIfNecessary(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<ApplicationLayer>& appLayerPtr)
{

    assert(appLayerPtr != nullptr);

    shared_ptr<DhcpClientInterface> dhcpClientInterfacePtr;
    shared_ptr<IscDhcpApplication> iscDhcpServerAppPtr;

    for(unsigned int interfaceIndex = 0; (interfaceIndex < networkInterfaces.size()); interfaceIndex++) {

        const InterfaceId theInterfaceId = (*this).GetInterfaceId(interfaceIndex);

        if ((theParameterDatabaseReader.ParameterExists("network-enable-dhcp-server", theNodeId, theInterfaceId)) &&
            (theParameterDatabaseReader.ReadBool("network-enable-dhcp-server", theNodeId, theInterfaceId))) {

            const string dhcpModel = MakeLowerCaseString(
                theParameterDatabaseReader.ReadString("network-dhcp-model", theNodeId, theInterfaceId));

            if (dhcpModel == "abstract") {
                shared_ptr<DhcpServerApplication> dhcpServerAppPtr(
                    new DhcpServerApplication(
                        theParameterDatabaseReader,
                        simulationEngineInterfacePtr,
                        theNodeId,
                        theInterfaceId,
                        interfaceIndex,
                        networkInterfaces[interfaceIndex].address,
                        networkInterfaces[interfaceIndex].subnetMaskLengthBits));
                appLayerPtr->AddApp(dhcpServerAppPtr);
                dhcpServerAppPtr->CompleteInitialization();
            }
            else if (dhcpModel == "isc") {
                if (iscDhcpServerAppPtr == nullptr) {
                    iscDhcpServerAppPtr = shared_ptr<IscDhcpApplication>(
                        new IscDhcpApplication(simulationEngineInterfacePtr, theNodeId, true));
                    appLayerPtr->AddApp(iscDhcpServerAppPtr);
                    iscDhcpServerAppPtr->CompleteInitialization(theParameterDatabaseReader);
                }//if//
                assert(iscDhcpServerAppPtr);
                iscDhcpServerAppPtr->EnableForThisInterface(
                    theParameterDatabaseReader,
                    theInterfaceId,
                    interfaceIndex);
            }
            else {
                cerr << "Error: network-dhcp-model(" << dhcpModel
                     << ") should be abstract or isc" << endl;
                exit(1);
            }//if//
        }//if//

        if ((theParameterDatabaseReader.ParameterExists("network-enable-dhcp-client", theNodeId, theInterfaceId)) &&
            (theParameterDatabaseReader.ReadBool("network-enable-dhcp-client", theNodeId, theInterfaceId))) {

            const string dhcpModel = MakeLowerCaseString(
                theParameterDatabaseReader.ReadString("network-dhcp-model", theNodeId, theInterfaceId));

            if (dhcpModel == "abstract") {
                if (dhcpClientInterfacePtr == nullptr) {
                    shared_ptr<DhcpClientApplication> dhcpClientAppPtr(
                        new DhcpClientApplication(
                            theParameterDatabaseReader,
                            simulationEngineInterfacePtr,
                            theNodeId,
                            networkInterfaces.size()));
                    appLayerPtr->AddApp(dhcpClientAppPtr);
                    dhcpClientAppPtr->CompleteInitialization();
                    dhcpClientInterfacePtr = dhcpClientAppPtr;
                }//if//
                networkInterfaces.at(interfaceIndex).dhcpClientInterfacePtr = dhcpClientInterfacePtr;
                dhcpClientInterfacePtr->EnableForThisInterface(
                    theParameterDatabaseReader,
                    theInterfaceId,
                    interfaceIndex);
            }
            else if (dhcpModel == "isc") {
                if (dhcpClientInterfacePtr == nullptr) {
                    shared_ptr<IscDhcpApplication> iscDhcpClientAppPtr(
                        new IscDhcpApplication(simulationEngineInterfacePtr, theNodeId, false));
                    appLayerPtr->AddApp(iscDhcpClientAppPtr);
                    iscDhcpClientAppPtr->CompleteInitialization(theParameterDatabaseReader);
                    dhcpClientInterfacePtr = iscDhcpClientAppPtr;
                }//if//
                networkInterfaces.at(interfaceIndex).dhcpClientInterfacePtr = dhcpClientInterfacePtr;
                dhcpClientInterfacePtr->EnableForThisInterface(
                    theParameterDatabaseReader,
                    theInterfaceId,
                    interfaceIndex);
            }
            else {
                cerr << "Error: network-dhcp-model(" << dhcpModel
                     << ") should be abstract or isc" << endl;
                exit(1);
            }//if//
        }//if//
    }//for//

}//SetupDhcpServerAndClientIfNecessary//


void BasicNetworkLayer::CheckTheNecessityOfMacAddressResolution(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceId& theInterfaceId,
    const NetworkAddress& networkAddress,
    const NetworkAddress& networkAddressMask) const
{
    const NetworkAddress hostAddress = networkAddress.MakeAddressWithZeroedSubnetBits(networkAddressMask);
    const uint32_t hostAddressValue = hostAddress.GetRawAddressLow32Bits();

    if (theNodeId == hostAddressValue) {
        return;
    }//if//

    if ((theParameterDatabaseReader.ParameterExists("network-enable-dhcp-client", theNodeId, theInterfaceId)) &&
        (theParameterDatabaseReader.ReadBool("network-enable-dhcp-client", theNodeId, theInterfaceId))) {
        return;
    }//if//

    const string macProtocol = MakeLowerCaseString(
        theParameterDatabaseReader.ReadString("mac-protocol", theNodeId, theInterfaceId));

    if ((macProtocol == "aloha") || (macProtocol == "dot11ad")) {
        // MAC address resolution(ARP or NDP) is not supported.
        cerr << "Error: Host address of " << networkAddress.ConvertToString()
             << " is different from theNodeId(" << theNodeId << ")." << endl;
        exit(1);
    }
    else if ((macProtocol == "dot11") || (macProtocol == "dot11ah")) {
        bool enableArp = false;
        if (theParameterDatabaseReader.ParameterExists("network-enable-arp", theNodeId, theInterfaceId)) {
            enableArp = theParameterDatabaseReader.ReadBool("network-enable-arp", theNodeId, theInterfaceId);
        }//if//

        bool enableNdp = false;
        if (theParameterDatabaseReader.ParameterExists("network-enable-ndp", theNodeId, theInterfaceId)) {
            enableNdp = theParameterDatabaseReader.ReadBool("network-enable-ndp", theNodeId, theInterfaceId);
        }//if//

        if ((!enableArp) && (!enableNdp)) {
            cerr << "Error: Host address of " << networkAddress.ConvertToString()
                 << " is different from theNodeId(" << theNodeId << ")."
                 << " Please enable MAC address resolution(ARP or NDP). " << endl;
            exit(1);
        }//if//
    }//if//

}//CheckTheNecessityOfMacAddressResolution//


}//namespace//
