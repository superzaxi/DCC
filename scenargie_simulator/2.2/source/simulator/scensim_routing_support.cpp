// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scensim_routing_support.h"
#include "nrlolsrglue.h"
#include "kernelaodvglue.h"
#include "nuolsrv2glue.h"


namespace ScenSim {

void SetupRoutingProtocol(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const shared_ptr<NetworkLayer>& networkLayerPtr,
    const RandomNumberGeneratorSeed& nodeSeed)
{
    shared_ptr<NrlolsrProtocol> nrlolsrProtocolPtr;
    shared_ptr<KernelAodvProtocol> kernelAodvProtocolPtr;
    shared_ptr<NuOLSRv2Protocol> nuOLSRv2ProtocolPtr;

    const unsigned int numberInterfaces = networkLayerPtr->NumberOfInterfaces();

    for (unsigned int i = 0; i < numberInterfaces; i++) {

        const InterfaceId theInterfaceId = networkLayerPtr->GetInterfaceId(i);

        if (!theParameterDatabaseReader.ParameterExists("network-routing-protocol-name", theNodeId, theInterfaceId)) {
            continue;
        }//if//

        string routingProtocol =
            theParameterDatabaseReader.ReadString("network-routing-protocol-name", theNodeId, theInterfaceId);

        ConvertStringToLowerCase(routingProtocol);

        if (routingProtocol == "nrl_olsr") {
            if (nrlolsrProtocolPtr == nullptr) {

                nrlolsrProtocolPtr.reset(
                    new NrlolsrProtocol(
                        simulationEngineInterfacePtr,
                        networkLayerPtr,
                        networkLayerPtr->LookupInterfaceIndex(theInterfaceId),
                        theInterfaceId,
                        nodeSeed));

                nrlolsrProtocolPtr->CompleteInitialization(theParameterDatabaseReader);

                networkLayerPtr->RegisterPacketHandlerForProtocol(
                    IP_PROTOCOL_NUMBER_OLSR, nrlolsrProtocolPtr);
            }
            else {
                cerr << "Error: nrl_olsr doesn't support multiple interfaces" << endl;
                exit(1);
            }//if//
        }
        else if (routingProtocol == "kernel_aodv") {
            if (kernelAodvProtocolPtr == nullptr) {

                kernelAodvProtocolPtr.reset(
                    new KernelAodvProtocol(
                        theParameterDatabaseReader,
                        theNodeId,
                        simulationEngineInterfacePtr,
                        networkLayerPtr,
                        networkLayerPtr->LookupInterfaceIndex(theInterfaceId),
                        theInterfaceId,
                        nodeSeed));

                networkLayerPtr->RegisterOnDemandRoutingProtocolInterface(kernelAodvProtocolPtr);
                networkLayerPtr->RegisterPacketHandlerForProtocol(
                    IP_PROTOCOL_NUMBER_AODV, kernelAodvProtocolPtr);
            }
            else {
                cerr << "Error: kernel_aodv doesn't support multiple interfaces" << endl;
                exit(1);
            }//if//
        }
        else if (routingProtocol == "nu_olsrv2") {
            if (nuOLSRv2ProtocolPtr == nullptr) {

                nuOLSRv2ProtocolPtr.reset(
                    new NuOLSRv2Protocol(
                        theNodeId,
                        simulationEngineInterfacePtr,
                        networkLayerPtr,
                        nodeSeed));

                nuOLSRv2ProtocolPtr->CompleteInitialization(theParameterDatabaseReader);

                networkLayerPtr->RegisterNetworkAddressInterface(nuOLSRv2ProtocolPtr);
                networkLayerPtr->RegisterPacketHandlerForProtocol(
                    IP_PROTOCOL_NUMBER_OLSRV2, nuOLSRv2ProtocolPtr);
            }//if//

            nuOLSRv2ProtocolPtr->SetInterfaceId(theParameterDatabaseReader, theInterfaceId);
        }
        else {
            cerr << "Error: Don't support routing protocol: " << routingProtocol << endl;
            exit(1);
        }//if//

    }//for//

    if (nuOLSRv2ProtocolPtr != nullptr) {
        nuOLSRv2ProtocolPtr->Start();
    }//if//

}//SetupRoutingProtocol//

}//namespace//

