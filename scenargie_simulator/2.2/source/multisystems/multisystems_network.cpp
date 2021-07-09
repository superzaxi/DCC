// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "sim.h"
#include "scensim_routing_support.h"

void SimNode::SetupNetworkProtocols(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    (*this).SetupGatewayInterface(
        theParameterDatabaseReader);

    SetupRoutingProtocol(
        theParameterDatabaseReader,
        theNodeId,
        simulationEngineInterfacePtr,
        networkLayerPtr,
        nodeSeed);

    // Add (routing)/transport/application protocols from here. -----

}//SetupNetworkProtocols//

void SimNode::SetupGatewayInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    bool hasLteInterface = false;

    const unsigned int numberInterfaces =  networkLayerPtr->NumberOfInterfaces();

    for(unsigned int interfaceIndex = 0; interfaceIndex < numberInterfaces; interfaceIndex++) {
        const InterfaceId theInterfaceId = networkLayerPtr->GetInterfaceId(interfaceIndex);
        const string macProtocol = MakeLowerCaseString(
            theParameterDatabaseReader.ReadString("mac-protocol", theNodeId, theInterfaceId));

        if (macProtocol.find("lte") != string::npos) {
            hasLteInterface = true;
            break;
        }//if//
    }//for//

    if (hasLteInterface) {
        if(!theParameterDatabaseReader.ParameterExists(
               Lte::parameterNamePrefix + "gateway-network-address")) {
            cerr << "Error: Please set a gateway(MME/SGW) network address. "
                 << Lte::parameterNamePrefix << "gateway-network-address"
                 << endl;
            exit(1);
        }//if//
    }

    if(theParameterDatabaseReader.ParameterExists(
           Lte::parameterNamePrefix + "gateway-network-address")) {

        string gatewayNetworkAddressString =
            theParameterDatabaseReader.ReadString(Lte::parameterNamePrefix + "gateway-network-address");

        NetworkAddress gatewayNetworkAddress;
        bool success;
        gatewayNetworkAddress.SetAddressFromString(gatewayNetworkAddressString, success);

        if (!success) {
            cerr << "Error in " << (Lte::parameterNamePrefix + "gateway-network-address")
                 << " parameter:" << endl;
            cerr << "      " << gatewayNetworkAddressString << endl;
            exit(1);
        }//if//

        for(unsigned int interfaceIndex = 0; interfaceIndex < numberInterfaces; interfaceIndex++) {

            if (networkLayerPtr->GetNetworkAddress(interfaceIndex) == gatewayNetworkAddress) {
                gatewayControllerPtr =
                    Lte::LteGatewayController::Create(
                        theParameterDatabaseReader,
                        simulationEngineInterfacePtr,
                        networkLayerPtr,
                        gatewayNetworkAddress,
                        *appLayerPtr);
            }//if//
        }//for//
    }//if//

}//SetupGatewayInterface//
