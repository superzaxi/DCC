// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "t109_mac.h"
#include "t109_app.h"

namespace T109 {

const string T109Mac::modelName = "T109Mac";

void T109Mac::ProcessDataFrame(
    const Packet& dataFrame,
    const double receiveRssiDbm,
    const DatarateBitsPerSec& datarateBitsPerSecond)
{
    const MacFrameHeaderType& dataMacFrameHeader =
        dataFrame.GetAndReinterpretPayloadData<MacFrameHeaderType>();

    const bool haveAlreadySeenThisPacket =
        HaveAlreadySeenThisPacketBefore(
            dataMacFrameHeader.transmitterAddress,
            dataMacFrameHeader.sequenceNumber);

    if (!haveAlreadySeenThisPacket) {
        (*this).RememberNewSequenceNumber(
            dataMacFrameHeader.transmitterAddress,
            dataMacFrameHeader.sequenceNumber);

        unique_ptr<Packet> dataPacketPtr(new Packet(dataFrame));

        dataPacketPtr->DeleteHeader(sizeof(MacFrameHeaderType));
        dataPacketPtr->DeleteHeader(sizeof(LLcHeaderType));

        const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

        const RsuControlHeaderType& rsuControlHeader =
            dataPacketPtr->GetAndReinterpretPayloadData<RsuControlHeaderType>();

        if (accessMode == ACCESS_MODE_OBE) {

            if (currentTime >= allSlotInformationExpirationTime) {
                for(size_t i = 0; i < rsuFrameSlots.size(); i++) {
                    rsuFrameSlots[i].informationExpirationTime = ZERO_TIME;
                }
            }

            if (rsuControlHeader.id == 1) {
                rsuFrameSlotSyncCount = 4;
            } else {
                if (rsuFrameSlotSyncCount == 0 ||
                    rsuFrameSlotSyncCount > rsuControlHeader.sync+1) {
                    rsuFrameSlotSyncCount = rsuControlHeader.sync+1;
                }
            }

            if (rsuFrameSlotSyncCount >= 4) {
                assert(rsuFrameSlotSyncCount <= 8);

                allSlotInformationExpirationTime =
                    currentTime +
                    (8 - rsuFrameSlotSyncCount)*rsuSlotInformationAvailableDuration;
            }
        }

        for(size_t i = 0; i < NUMBER_RSU_FRAME_SLOTS; i++) {
            const RsuControlHeaderType::RsuSlotInformation& slotInformation =
                rsuControlHeader.slotInformations[i];

            if (slotInformation.durationCount == 0) {
                continue;
            }

            const SimTime expirationTime =
                currentTime +
                ((int(slotInformation.forwardCount)+1)*rsuSlotInformationAvailableDuration);

            RsuFrameSlot& rsuFrameSlot = rsuFrameSlots.at(i);

            if (expirationTime > rsuFrameSlot.informationExpirationTime) {
                rsuFrameSlot.durationCount = slotInformation.durationCount;
                rsuFrameSlot.informationExpirationTime = expirationTime;

                rsuFrameSlot.endTimeFromBase =
                    rsuFrameSlot.startTimeFromBase +
                   rsuFrameSlot.durationCount * (rsuControlUnitDuration*3) +
                    guardTime*2;

                assert(rsuFrameSlot.startTimeFromBase + guardTime*2 < rsuFrameSlot.endTimeFromBase);
            }
        }

        dataPacketPtr->DeleteHeader(sizeof(RsuControlHeaderType));

        appLayerPtr->ReceivePacketFromMac(
            dataPacketPtr,
            receiveRssiDbm,
            dataMacFrameHeader.transmitterAddress);

    }//if//

}//ProcessDataFrame//

}//namespace//
