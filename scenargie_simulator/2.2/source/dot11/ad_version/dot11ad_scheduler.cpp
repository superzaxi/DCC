// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "dot11ad_mac.h"

namespace Dot11ad {

using ScenSim::InvalidNodeId;
using ScenSim::ConvertToString;

StaticScheduleScheduler::StaticScheduleScheduler(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& initNodeId,
    const InterfaceOrInstanceId& theInterfaceId,
    const shared_ptr<Dot11Mac>& macLayerPtr)
    :
    theNodeId(initNodeId),
    shouldDoReceiveSectorSweepInAbft(false),
    numberAntennaSectors(macLayerPtr->GetNumberAntennaSectors()),
    beaconCount(0),
    apReceiveSectorSweepInterval(0),
    staReceiveSectorSweepInterval(0)
{
    using std::max;

    beaconSuperframeIntervalDuration =
        theParameterDatabaseReader.ReadTime(
                (adParamNamePrefix + "beacon-superframe-interval-duration"), theNodeId, theInterfaceId);

    beaconStartTime = ZERO_TIME;

    if (theParameterDatabaseReader.ParameterExists(
            (adParamNamePrefix + "beacon-start-time"), theNodeId, theInterfaceId)) {
        beaconStartTime =
             theParameterDatabaseReader.ReadTime(
                 (adParamNamePrefix + "beacon-start-time"), theNodeId, theInterfaceId);
    }//if//

    beaconStartTimeMaxJitter = beaconSuperframeIntervalDuration;

    if (theParameterDatabaseReader.ParameterExists(
            (adParamNamePrefix + "beacon-start-time-max-jitter"), theNodeId, theInterfaceId)) {
        beaconStartTimeMaxJitter =
             theParameterDatabaseReader.ReadTime(
                 (adParamNamePrefix + "beacon-start-time-max-jitter"), theNodeId, theInterfaceId);
    }//if//

    isDistributedBeaconTransmissionIntervalAllowed = true;

    if (theParameterDatabaseReader.ParameterExists(
            (adParamNamePrefix + "distributed-beacon-transmission-interval-is-allowed"), theNodeId, theInterfaceId)) {
        isDistributedBeaconTransmissionIntervalAllowed =
            theParameterDatabaseReader.ReadBool(
                (adParamNamePrefix + "distributed-beacon-transmission-interval-is-allowed"), theNodeId, theInterfaceId);
    }//if//

    dmgBeaconTransmissionIntervalAkaBtiDuration =
        theParameterDatabaseReader.ReadTime(
                (adParamNamePrefix + "beacon-transmission-interval-aka-bti-duration"), theNodeId, theInterfaceId);

    const SimTime associationBeamformingTrainingAkaAbftDuration =
        theParameterDatabaseReader.ReadTime(
                (adParamNamePrefix + "association-beamforming-training-aka-abft-duration"),
                theNodeId, theInterfaceId);

    maxNumResponderTransmitSectorSweepFrames = 0;
    associationBeamformingTrainingAkaAbftNumTxssSlots = 0;
    associationBeamformingTrainingAkaAbftNumRxssSlots = 0;

    if (associationBeamformingTrainingAkaAbftDuration != ZERO_TIME) {

        maxNumResponderTransmitSectorSweepFrames =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (adParamNamePrefix + "abft-max-num-responder-txss-frames"),
                    theNodeId, theInterfaceId);

        if (maxNumResponderTransmitSectorSweepFrames > UCHAR_MAX) {
            cerr << "Error in \"" << (adParamNamePrefix + "abft-max-num-responder-txss-frames")
                 << "\" parameter: Must be less than or equal to " << UCHAR_MAX << "." << endl;

            exit(1);
        }//if//

        if (theParameterDatabaseReader.ParameterExists(adParamNamePrefix + "ap-receive-sector-sweep-interval-to-beacon", theNodeId, theInterfaceId)) {
            apReceiveSectorSweepInterval =
                theParameterDatabaseReader.ReadNonNegativeInt(
                    adParamNamePrefix + "ap-receive-sector-sweep-interval-to-beacon", theNodeId, theInterfaceId);
        }//if//

        (*this).associationBeamformingTrainingAkaAbftNumTxssSlotTime =
             macLayerPtr->CalcAssociationBeamformingTrainingSlotTime(
                 maxNumResponderTransmitSectorSweepFrames);

        associationBeamformingTrainingAkaAbftNumTxssSlots = static_cast<unsigned int>(
            (associationBeamformingTrainingAkaAbftDuration / associationBeamformingTrainingAkaAbftNumTxssSlotTime));

        (*this).txssAssociationBeamformingTrainingAkaAbftDuration =
            (associationBeamformingTrainingAkaAbftNumTxssSlots * associationBeamformingTrainingAkaAbftNumTxssSlotTime);

        (*this).associationBeamformingTrainingAkaAbftNumRxssSlotTime =
             macLayerPtr->CalcAssociationBeamformingTrainingSlotTime(
                 macLayerPtr->GetNumberAntennaSectors());

        associationBeamformingTrainingAkaAbftNumRxssSlots = static_cast<unsigned int>(
            (associationBeamformingTrainingAkaAbftDuration /
             associationBeamformingTrainingAkaAbftNumRxssSlotTime));

        (*this).rxssAssociationBeamformingTrainingAkaAbftDuration =
            (associationBeamformingTrainingAkaAbftNumRxssSlots * associationBeamformingTrainingAkaAbftNumRxssSlotTime);

    }//if//

    if (theParameterDatabaseReader.ParameterExists(adParamNamePrefix + "sta-receive-sector-sweep-interval-to-beacon", theNodeId, theInterfaceId)) {
        staReceiveSectorSweepInterval =
            theParameterDatabaseReader.ReadNonNegativeInt(
                adParamNamePrefix + "sta-receive-sector-sweep-interval-to-beacon", theNodeId, theInterfaceId);
    }//if//

    const unsigned int maxDataTransmissionIntervalPeriods = 100;

    const SimTime startOfDtiTime =
        dmgBeaconTransmissionIntervalAkaBtiDuration +
        associationBeamformingTrainingAkaAbftDuration;

    //Future if needed: announcementTransmissionIntervalAkaAtiDuration;

    (*this).dataTransferIntervalAkaDtiDuration = ZERO_TIME;
    bool foundLast = false;

    SimTime lastStartTime = ZERO_TIME;

    for(unsigned int parmNum = 1; (parmNum <= maxDataTransmissionIntervalPeriods); parmNum++) {
        const string parmName =
            adParamNamePrefix + "data-transfer-interval-" + ConvertToString(parmNum);

        if (theParameterDatabaseReader.ParameterExists(parmName + "-duration", theNodeId, theInterfaceId)) {
            if (foundLast) {
                cerr << "Error: Found gap in " << adParamNamePrefix + "data-transfer-interval-<num>-*"
                     << " parameters." << endl;
                exit(1);
            }//if//

            const SimTime startTime =
                theParameterDatabaseReader.ReadTime(parmName + "-relative-start-time", theNodeId, theInterfaceId);

            if (lastStartTime > startTime) {
                cerr << "Error in 802.11ad schedule parameter: \""
                     << (parmName + "-relative-start-time") << "\"." << endl;
                cerr << "    Period start times must be in increasing order." << endl;
                exit(1);
            }//if//

            lastStartTime = startTime;

            const SimTime duration =
                theParameterDatabaseReader.ReadTime(parmName + "-duration", theNodeId, theInterfaceId);

            (*this).dataTransferIntervalAkaDtiDuration =
                max(dataTransferIntervalAkaDtiDuration, (startTime + duration));

            const bool isCbap =
                theParameterDatabaseReader.ReadBool(
                    parmName + "-is-contention-based-access-period",
                    theNodeId, theInterfaceId);

            NodeId sourceNodeId = InvalidNodeId;
            NodeId destinationNodeId = InvalidNodeId;

            if (!isCbap) {
                sourceNodeId =
                    theParameterDatabaseReader.ReadNonNegativeInt(
                        parmName + "-source-nodeid", theNodeId, theInterfaceId);

                destinationNodeId =
                    theParameterDatabaseReader.ReadNonNegativeInt(
                        parmName + "-destination-nodeid", theNodeId, theInterfaceId);

            }//if//

            (*this).dmgDataTransferIntervalList.push_back(
                DataTransferIntervalListElementType(
                    isCbap, startTime, duration, sourceNodeId, destinationNodeId));
        }
        else {
           foundLast = true;
        }//if//
    }//for//


    if ((startOfDtiTime + dataTransferIntervalAkaDtiDuration) != beaconSuperframeIntervalDuration) {
        cerr << "Error 802.11ad Superframe parameter duration: "
             << (adParamNamePrefix + "beacon-superframe-interval-duration") << " parameter" << endl;
        cerr << "    does not match end of final data interval." << endl;
        exit(1);
    }//if//

}//StaticScheduleScheduler()//


}//namespace//
