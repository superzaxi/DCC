// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "t109_phy.h"
#include "t109_mac.h"

namespace T109 {

const string T109Phy::modelName = "T109Phy";

inline
void T109Phy::ProcessStatsForEndCurrentTransmission()
{
    SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    (*this).totalTransmissionTime += (currentTime - lastChannelStateTransitionTime);
    (*this).lastChannelStateTransitionTime = currentTime;
}


void T109Phy::EndCurrentTransmission()
{
    assert(phyState == PHY_TRANSMITTING);
    phyState = PHY_SCANNING;

    (*this).ProcessStatsForEndCurrentTransmission();

    macLayerPtr->TransmissionIsCompleteNotification();

    // Note that the MAC may (theoretically) have issued a transmit command
    // in the immediately previous statement (TransmissionIsCompleteNotification call).

    if ((phyState == PHY_SCANNING) &&
        (currentInterferencePower < energyDetectionPowerThreshold)) {

        (*this).currentlySensingBusyMedium = false;
        macLayerPtr->ClearChannelAtPhysicalLayerNotification();

    }//if//

}//EndCurrentTransmission//



inline
void T109Phy::ProcessStatsForTransitionToBusyChannel()
{
    SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    (*this).totalIdleChannelTime += (currentTime - lastChannelStateTransitionTime);
    (*this).lastChannelStateTransitionTime = currentTime;
}

inline
void T109Phy::ProcessStatsForTransitionToIdleChannel()
{
    SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    (*this).totalBusyChannelTime += (currentTime - lastChannelStateTransitionTime);
    (*this).lastChannelStateTransitionTime = currentTime;
}


void T109Phy::ProcessNewSignal(const IncomingSignal& aSignal)
{
    if (phyState == PHY_SCANNING) {
        if ((aSignal.HasACompleteFrame()) &&
            (aSignal.GetReceivedPowerDbm() >= preambleDetectionPowerThresholdDbm)) {

            phyState = PHY_RECEIVING;

            (*this).StartReceivingThisSignal(aSignal);

            if (!currentlySensingBusyMedium) {

                (*this).currentlySensingBusyMedium = true;
                (*this).ProcessStatsForTransitionToBusyChannel();

                macLayerPtr->BusyChannelAtPhysicalLayerNotification();

            }//if//
        }
        else {

            (*this).AddSignalToInterferenceLevel(aSignal);

            if ((!currentlySensingBusyMedium) &&
                (currentInterferencePower >= energyDetectionPowerThreshold)) {

                (*this).currentlySensingBusyMedium = true;
                (*this).ProcessStatsForTransitionToBusyChannel();

                macLayerPtr->BusyChannelAtPhysicalLayerNotification();

            }//if//
        }//if//
    }
    else if ((phyState == PHY_RECEIVING) && (aSignal.HasACompleteFrame()) &&
             (aSignal.GetReceivedPowerDbm() > (currentSignalPowerDbm + signalCaptureRatioThresholdDb))) {

        // Signal capture.

        (*this).currentPacketHasAnError = true;

        OutputTraceAndStatsForRxEnd(aSignal, true/*Capture*/);

        const double adjustedSignalPowerMw =
            (ConvertToNonDb(currentSignalPowerDbm) *
             CalcPilotSubcarriersPowerAdjustmentFactor());

        (*this).AddSignalPowerToInterferenceLevelAndDoStats(
            currentSignalPowerDbm,
            adjustedSignalPowerMw,
            currentIncomingSignalSourceNodeId,
            currentLockedOnFramePacketId,
            currentFrameSizeBytes);

        (*this).StartReceivingThisSignal(aSignal);
    }
    else {
        // Present signal keeps going

        (*this).AddSignalToInterferenceLevel(aSignal);

    }//if//

}//ProcessNewSignal//



void T109Phy::ProcessEndOfTheSignalCurrentlyBeingReceived(const IncomingSignal& aSignal)
{
    assert(phyState == PHY_RECEIVING);

    (*this).UpdatePacketReceptionCalculation();

    OutputTraceAndStatsForRxEnd(aSignal, false/*No Capture*/);

    phyState = PHY_SCANNING;

    if (!currentPacketHasAnError) {

        this->lastReceivedPacketRssiDbm = aSignal.GetReceivedPowerDbm();

        macLayerPtr->ReceiveFrameFromPhy(
            *aSignal.GetFrame().macFramePtr,
            this->lastReceivedPacketRssiDbm,
            aSignal.GetFrame().datarateBitsPerSecond);
    }

    // Note that the MAC may have issued a transmit command in the immediately
    // previous statement (ReceiveFrameFromPhy call).

    if ((phyState == PHY_SCANNING) &&
        (currentInterferencePower < energyDetectionPowerThreshold)) {

        assert(currentlySensingBusyMedium);
        (*this).currentlySensingBusyMedium = false;
        (*this).ProcessStatsForTransitionToIdleChannel();
        macLayerPtr->ClearChannelAtPhysicalLayerNotification();
    }//if//

    (*this).currentSignalPowerDbm = -DBL_MAX;
    (*this).currentAdjustedSignalPowerMilliwatts = 0.0;
    (*this).currentIncomingSignalSourceNodeId = 0;
    (*this).currentLockedOnFramePacketId = PacketId::nullPacketId;

}//ProcessEndOfTheSignalCurrentlyBeingReceived//



void T109Phy::SubtractSignalFromInterferenceLevelAndNotifyMacIfMediumIsClear(
    const IncomingSignal& aSignal)
{
    (*this).SubtractSignalFromInterferenceLevel(aSignal);

    if ((phyState == PHY_SCANNING) &&
        (currentlySensingBusyMedium) &&
        (currentInterferencePower < energyDetectionPowerThreshold)) {

        (*this).currentlySensingBusyMedium = false;
        (*this).ProcessStatsForTransitionToIdleChannel();

        macLayerPtr->ClearChannelAtPhysicalLayerNotification();

    }//if//

}//SubtractSignalFromInterferenceLevelAndNotifyMacIfMediumIsClear//


}//namespace
