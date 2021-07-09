// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11_INCOMING_BUFFER_H
#define DOT11_INCOMING_BUFFER_H

#include <memory>
#include <map>
#include "scensim_support.h"
#include "dot11_headers.h"

namespace Dot11 {

using std::unique_ptr;
using std::move;
using std::map;

using ScenSim::PacketPriority;
using ScenSim::CalcTwelveBitSequenceNumberDifference;


//--------------------------------------------------------------------------------------------------

class IncomingFrameBuffer {
public:
    void AddBlockAckSession(
        const MacAddress& transmitterAddress,
        const PacketPriority& trafficId);

    void DeleteBlockAckSession();

    void ProcessIncomingFrame(
        const MacAddress& transmitterAddress,
        const PacketPriority& trafficId,
        const unsigned short int headerSequenceNumber,
        bool& haveAlreadySeenThisFrame,
        vector<unique_ptr<Packet> >& bufferedPacketsToSendUp);

    void ProcessIncomingSubframe(
        unique_ptr<Packet>& framePtr,
        const MacAddress& transmitterAddress,
        const PacketPriority& trafficId,
        const unsigned short int headerSequenceNumber,
        bool& frameIsInOrder,
        bool& haveAlreadySeenThisFrame,
        vector<unique_ptr<Packet> >& bufferedPacketsToSendUp);

    bool HasFramesToSendUpStack(
        const MacAddress& transmitterAddress,
        const PacketPriority& trafficId) const;

    void RetrieveFramesToSendUpStack(
        const MacAddress& transmitterAddress,
        const PacketPriority& trafficId,
        vector<unique_ptr<Packet> >& framePtrs);

    void ProcessBlockAckRequestFrame(
        const BlockAcknowledgementRequestFrame& blockAckRequest,
        vector<unique_ptr<Packet> >& bufferedPacketsToSendUp);

    void GetBlockAckInfo(
        const MacAddress& transmitterAddress,
        const PacketPriority& trafficId,
        bool& success,
        unsigned short int& startSequenceNumber,
        std::bitset<BlockAckBitMapNumBits>& blockAckBitmap) const;

private:

    struct LinkInfoMapKey {
        MacAddress transmitterAddress;
        PacketPriority trafficId;

        LinkInfoMapKey(
            const MacAddress& initTransmitterAddress, const PacketPriority& initTrafficId)
            :
            transmitterAddress(initTransmitterAddress), trafficId(initTrafficId) {}

        bool operator<(const LinkInfoMapKey& right) const {
            return ((transmitterAddress < right.transmitterAddress) ||
                    ((transmitterAddress == right.transmitterAddress) && (trafficId < right.trafficId)));
        }
    };//LinkInfoMapKey//


    struct PacketBufferElement {
        unsigned long long int sequenceNumber;
        unique_ptr<Packet> framePtr;

        PacketBufferElement() : sequenceNumber(0) { }

        PacketBufferElement(
            unique_ptr<Packet>& initFramePtr,
            const unsigned long long int initSequenceNumber)
        :
            framePtr(move(initFramePtr)),
            sequenceNumber(initSequenceNumber)
        {
        }

        void operator=(PacketBufferElement&& right)
        {
            sequenceNumber = right.sequenceNumber;
            framePtr = move(right.framePtr);
        }

        PacketBufferElement(PacketBufferElement&& right) { (*this) = move(right); }

        bool operator<(const PacketBufferElement& right) const
        {
            return ((*this).sequenceNumber > right.sequenceNumber);
        }

    };//PacketBufferElement//


    struct BufferLinkInfo {
        bool blockAckSessionIsActive;
        std::priority_queue<PacketBufferElement> frameBuffer;
        unsigned long long int currentLowestUnreceivedSequenceNumber;
        unsigned long long int bitmapWindowStartSequenceNumber;
        std::bitset<BlockAckBitMapNumBits> blockAckBitmap;

        BufferLinkInfo() :
            blockAckSessionIsActive(false),
            currentLowestUnreceivedSequenceNumber(1),
            bitmapWindowStartSequenceNumber(1) {}

        void operator=(BufferLinkInfo&& right)
        {
            blockAckSessionIsActive = right.blockAckSessionIsActive;
            currentLowestUnreceivedSequenceNumber = right.currentLowestUnreceivedSequenceNumber;
            bitmapWindowStartSequenceNumber = right.bitmapWindowStartSequenceNumber;
            blockAckBitmap = right.blockAckBitmap;
            frameBuffer = move(right.frameBuffer);
        }

        BufferLinkInfo(BufferLinkInfo&& right) { (*this) = move(right); }

    };//BufferLinkInfo//

    map<LinkInfoMapKey, BufferLinkInfo> linkInfoMap;

    void AdvanceFirstUnreceivedFrameSequenceNumber(
        BufferLinkInfo& linkInfo,
        const unsigned long long int newStartingSequenceNumber,
        vector<unique_ptr<Packet> >& bufferedPacketsToSendUp);

    void UpdateReceivedFrameBitmap(
        BufferLinkInfo& linkInfo,
        const unsigned long long int headerSequenceNumber,
        bool& frameIsInOrder,
        bool& haveAlreadySeenThisFrame,
        vector<unique_ptr<Packet> >& bufferedPacketsToSendUp);

};//IncomingFrameBuffer//


inline
unsigned short int ConvertNonWrappingSequenceNumberToRealOne(
    const unsigned long long int bigSequenceNumber)
{
    return (static_cast<unsigned short int>(bigSequenceNumber & 0xFFF));

}//ConvertSequenceNumberToNonWrappingOne//


inline
unsigned long long int ConvertSequenceNumberToNonWrappingOne(
    const unsigned long long int referenceSequenceNumber,
    const unsigned short int sequenceNumber)
{
    const int difference =
        CalcTwelveBitSequenceNumberDifference(
            sequenceNumber,
            ConvertNonWrappingSequenceNumberToRealOne(referenceSequenceNumber));

    assert((difference >= 0) || (referenceSequenceNumber >= -difference));

    return (referenceSequenceNumber + difference);

}//ConvertSequenceNumberToNonWrappingOne//


inline
void IncomingFrameBuffer::AdvanceFirstUnreceivedFrameSequenceNumber(
    BufferLinkInfo& linkInfo,
    const unsigned long long int newStartingSequenceNumber,
    vector<unique_ptr<Packet> >& bufferedPacketsToSendUp)
{
    if (linkInfo.bitmapWindowStartSequenceNumber == newStartingSequenceNumber) {
        linkInfo.currentLowestUnreceivedSequenceNumber = newStartingSequenceNumber;
        return;
    }//if//

    assert(linkInfo.bitmapWindowStartSequenceNumber < newStartingSequenceNumber);

    const unsigned int shiftCount = static_cast<unsigned int>(
        (newStartingSequenceNumber - linkInfo.bitmapWindowStartSequenceNumber));

    linkInfo.bitmapWindowStartSequenceNumber = newStartingSequenceNumber;
    linkInfo.blockAckBitmap >>= shiftCount;

    if (linkInfo.currentLowestUnreceivedSequenceNumber < newStartingSequenceNumber) {

        // Deliver buffered packets with Seq # before new start sequence number.

        while ((!linkInfo.frameBuffer.empty()) &&
               (linkInfo.frameBuffer.top().sequenceNumber < newStartingSequenceNumber)) {

            // std::priority_queue is broken with respect to movable types (unique_ptr)

            bufferedPacketsToSendUp.push_back(
                move(const_cast<PacketBufferElement&>(linkInfo.frameBuffer.top()).framePtr));

            linkInfo.frameBuffer.pop();

        }//while//

        linkInfo.currentLowestUnreceivedSequenceNumber = newStartingSequenceNumber;

        // Deliver buffered packets that are now "inorder" (gap was removed).

        unsigned int bufferedFrameShiftCount = 0;
        unsigned int i = 0;
        while ((i < BlockAckBitMapNumBits) && (linkInfo.blockAckBitmap[i])) {
            assert(!linkInfo.frameBuffer.empty());
            assert(linkInfo.frameBuffer.top().sequenceNumber ==
                   linkInfo.currentLowestUnreceivedSequenceNumber);

            bufferedPacketsToSendUp.push_back(
                move(const_cast<PacketBufferElement&>(linkInfo.frameBuffer.top()).framePtr));

            linkInfo.frameBuffer.pop();

            linkInfo.currentLowestUnreceivedSequenceNumber++;
            bufferedFrameShiftCount++;
            i++;
        }//while//

        if (bufferedFrameShiftCount > 0) {
            linkInfo.bitmapWindowStartSequenceNumber += bufferedFrameShiftCount;
            linkInfo.blockAckBitmap >>= bufferedFrameShiftCount;
            assert(linkInfo.bitmapWindowStartSequenceNumber == linkInfo.currentLowestUnreceivedSequenceNumber);
        }//if//
    }//if//

}//AdvanceFirstUnreceivedFrameSequenceNumber//



inline
void IncomingFrameBuffer::UpdateReceivedFrameBitmap(
    BufferLinkInfo& linkInfo,
    const unsigned long long int sequenceNumber,
    bool& frameIsInOrder,
    bool& haveAlreadySeenThisFrame,
    vector<unique_ptr<Packet> >& bufferedPacketsToSendUp)
{
    frameIsInOrder = false;
    haveAlreadySeenThisFrame = false;
    bufferedPacketsToSendUp.clear();

    if (sequenceNumber < linkInfo.currentLowestUnreceivedSequenceNumber) {
        haveAlreadySeenThisFrame = true;
        return;
    }//if//

    assert(linkInfo.bitmapWindowStartSequenceNumber <= linkInfo.currentLowestUnreceivedSequenceNumber);

    if ((sequenceNumber >= linkInfo.bitmapWindowStartSequenceNumber) &&
        (sequenceNumber < (linkInfo.bitmapWindowStartSequenceNumber + BlockAckBitMapNumBits))) {

        const unsigned int offset =
            static_cast<unsigned int>(sequenceNumber - linkInfo.bitmapWindowStartSequenceNumber);

        if (linkInfo.blockAckBitmap[offset]) {
            haveAlreadySeenThisFrame = true;
            return;
        }//if//

        linkInfo.blockAckBitmap.set(offset);

        if (sequenceNumber == linkInfo.currentLowestUnreceivedSequenceNumber) {
            frameIsInOrder = true;
            linkInfo.currentLowestUnreceivedSequenceNumber++;
            unsigned int shiftBy = 1;
            unsigned int i = offset + 1;
            while ((i < BlockAckBitMapNumBits) && (linkInfo.blockAckBitmap[i])) {
                assert(!linkInfo.frameBuffer.empty());
                assert(linkInfo.frameBuffer.top().sequenceNumber ==
                       linkInfo.currentLowestUnreceivedSequenceNumber);

                // const_cast due to priority_queue conflict with movable types (unique_ptr).

                bufferedPacketsToSendUp.push_back(
                    move(const_cast<PacketBufferElement&>(linkInfo.frameBuffer.top()).framePtr));

                linkInfo.frameBuffer.pop();
                linkInfo.currentLowestUnreceivedSequenceNumber++;
                shiftBy++;
                i++;
            }//while//

            // Shift left "array order"
            linkInfo.blockAckBitmap >>= shiftBy;
            linkInfo.bitmapWindowStartSequenceNumber = linkInfo.currentLowestUnreceivedSequenceNumber;

        }//if//
    }
    else {

        assert(false && "Should never happen in model (sender more restricted than standard)"); abort();

        if ((sequenceNumber == linkInfo.currentLowestUnreceivedSequenceNumber) &&
            (sequenceNumber == (linkInfo.bitmapWindowStartSequenceNumber + BlockAckBitMapNumBits))) {

            // Totally inorder packet receives special case (currently no missing frames).

            assert(linkInfo.frameBuffer.empty());

            frameIsInOrder = true;
            linkInfo.currentLowestUnreceivedSequenceNumber++;
            linkInfo.bitmapWindowStartSequenceNumber++;
            // Set everything to received.
            linkInfo.blockAckBitmap.set();
        }
        else {
            // Shift bitmap

            const unsigned int shiftCount = static_cast<unsigned int>(
                (sequenceNumber - (linkInfo.bitmapWindowStartSequenceNumber + BlockAckBitMapNumBits - 1)));

            linkInfo.blockAckBitmap >>= shiftCount;
            linkInfo.blockAckBitmap.set(BlockAckBitMapNumBits - 1);
            linkInfo.bitmapWindowStartSequenceNumber += shiftCount;
            assert(linkInfo.bitmapWindowStartSequenceNumber <= linkInfo.currentLowestUnreceivedSequenceNumber);
        }//if//
    }//if//

}//UpdateReceivedFrameBitmap//



inline
void IncomingFrameBuffer::ProcessIncomingFrame(
    const MacAddress& transmitterAddress,
    const PacketPriority& trafficId,
    const unsigned short int headerSequenceNumber,
    bool& haveAlreadySeenThisFrame,
    vector<unique_ptr<Packet> >& bufferedPacketsToSendUp)
{
    haveAlreadySeenThisFrame = false;

    BufferLinkInfo& linkInfo = (*this).linkInfoMap[LinkInfoMapKey(transmitterAddress, trafficId)];

    if (!linkInfo.blockAckSessionIsActive) {
        // Regular Ack Processing (No buffering).

        const int sequenceNumberDifference =
            CalcTwelveBitSequenceNumberDifference(
                headerSequenceNumber,
                ConvertNonWrappingSequenceNumberToRealOne(
                    linkInfo.currentLowestUnreceivedSequenceNumber));

        if (sequenceNumberDifference < 0) {
            haveAlreadySeenThisFrame = true;
        }
        else {
            linkInfo.currentLowestUnreceivedSequenceNumber += (1 + sequenceNumberDifference);
        }//if//

        return;

    }//if//

    // Block Ack Mode Processing.

    const unsigned long long int sequenceNumber =
        ConvertSequenceNumberToNonWrappingOne(
            (linkInfo.bitmapWindowStartSequenceNumber + BlockAckBitMapNumBits),
            headerSequenceNumber);

    if (sequenceNumber < linkInfo.currentLowestUnreceivedSequenceNumber) {
        haveAlreadySeenThisFrame = true;
    }
    else {
        // Assuming single acked frames are always the lowest sequence number.

        (*this).AdvanceFirstUnreceivedFrameSequenceNumber(
            linkInfo,
            (sequenceNumber + 1),
            bufferedPacketsToSendUp);
    }//if//

}//ProcessIncomingFrame//



inline
void IncomingFrameBuffer::ProcessIncomingSubframe(
    unique_ptr<Packet>& framePtr,
    const MacAddress& transmitterAddress,
    const PacketPriority& trafficId,
    const unsigned short int headerSequenceNumber,
    bool& frameIsInOrder,
    bool& haveAlreadySeenThisFrame,
    vector<unique_ptr<Packet> >& bufferedPacketsToSendUp)
{
    frameIsInOrder = false;
    haveAlreadySeenThisFrame = false;

    BufferLinkInfo& linkInfo = (*this).linkInfoMap[LinkInfoMapKey(transmitterAddress, trafficId)];

    if (!linkInfo.blockAckSessionIsActive) {

        assert(false && "This should not happen with new Forced BAR (Simple ADDBA)"); abort();

        //NotUsed // Modelling Hack to not have to do ADDBA Request/ADDBA Response.
        //NotUsed
        //NotUsed linkInfo.blockAckSessionIsActive = true;
        //NotUsed linkInfo.bitmapWindowStartSequenceNumber = linkInfo.currentLowestUnreceivedSequenceNumber;

    }//if//

    // Block Ack Mode Processing.

    const unsigned long long int sequenceNumber =
        ConvertSequenceNumberToNonWrappingOne(
            (linkInfo.bitmapWindowStartSequenceNumber + BlockAckBitMapNumBits),
            headerSequenceNumber);

    (*this).UpdateReceivedFrameBitmap(
        linkInfo, sequenceNumber, frameIsInOrder, haveAlreadySeenThisFrame, bufferedPacketsToSendUp);

    if ((!frameIsInOrder) && (!haveAlreadySeenThisFrame)) {
        linkInfo.frameBuffer.push(PacketBufferElement(framePtr, sequenceNumber));
    }//if//

}//ProcessIncomingSubframe//



inline
void IncomingFrameBuffer::ProcessBlockAckRequestFrame(
    const BlockAcknowledgementRequestFrame& blockAckRequest,
    vector<unique_ptr<Packet> >& bufferedPacketsToSendUp)
{
    bufferedPacketsToSendUp.clear();

    BufferLinkInfo& linkInfo =
        (*this).linkInfoMap[
            LinkInfoMapKey(
                blockAckRequest.transmitterAddress,
                blockAckRequest.blockAckRequestControl.trafficId)];

    if (!linkInfo.blockAckSessionIsActive) {

        // Modelling Hack to not have to do ADDBA Request/ADDBA Response (use BAR instead).

        linkInfo.blockAckSessionIsActive = true;
        linkInfo.bitmapWindowStartSequenceNumber = blockAckRequest.startingSequenceControl;
        linkInfo.currentLowestUnreceivedSequenceNumber = blockAckRequest.startingSequenceControl;
    }
    else {
        const unsigned long long int startingSequenceNumber =
            ConvertSequenceNumberToNonWrappingOne(
                (linkInfo.bitmapWindowStartSequenceNumber + BlockAckBitMapNumBits),
                blockAckRequest.startingSequenceControl);

        if (startingSequenceNumber > linkInfo.currentLowestUnreceivedSequenceNumber) {
            (*this).AdvanceFirstUnreceivedFrameSequenceNumber(
                linkInfo,
                startingSequenceNumber,
                bufferedPacketsToSendUp);
        }//if//
    }//if//

}//ProcessBlockAckRequest//



inline
void IncomingFrameBuffer::GetBlockAckInfo(
    const MacAddress& transmitterAddress,
    const PacketPriority& trafficId,
    bool& success,
    unsigned short int& startSequenceNumber,
    std::bitset<BlockAckBitMapNumBits>& blockAckBitmap) const
{
    typedef map<LinkInfoMapKey, BufferLinkInfo>::const_iterator IterType;

    success = false;

    IterType iter = linkInfoMap.find(LinkInfoMapKey(transmitterAddress, trafficId));
    if (iter == linkInfoMap.end()) {
        return;
    }//if//

    const BufferLinkInfo& linkInfo = iter->second;

    startSequenceNumber =
        ConvertNonWrappingSequenceNumberToRealOne(linkInfo.bitmapWindowStartSequenceNumber);
    blockAckBitmap = linkInfo.blockAckBitmap;
    success = true;

}//GetBlockAckInfo//


inline
bool IncomingFrameBuffer::HasFramesToSendUpStack(
    const MacAddress& transmitterAddress,
    const PacketPriority& trafficId) const
{
    typedef map<LinkInfoMapKey, BufferLinkInfo>::const_iterator IterType;

    IterType iter = linkInfoMap.find(LinkInfoMapKey(transmitterAddress, trafficId));
    if (iter == linkInfoMap.end()) {
        return false;
    }//if//

    const BufferLinkInfo& linkInfo = iter->second;

    return
        ((!linkInfo.frameBuffer.empty()) &&
         (linkInfo.frameBuffer.top().sequenceNumber < linkInfo.currentLowestUnreceivedSequenceNumber));

}//HasFramesToSendUpStack//

inline
void IncomingFrameBuffer::RetrieveFramesToSendUpStack(
    const MacAddress& transmitterAddress,
    const PacketPriority& trafficId,
    vector<unique_ptr<Packet> >& framePtrs)
{
    framePtrs.clear();
    BufferLinkInfo& linkInfo = (*this).linkInfoMap[LinkInfoMapKey(transmitterAddress, trafficId)];

    while((!linkInfo.frameBuffer.empty()) &&
          (linkInfo.frameBuffer.top().sequenceNumber < linkInfo.currentLowestUnreceivedSequenceNumber)) {

        // const_cast due to priority_queue conflict with movable types.
        framePtrs.push_back(
            move(const_cast<PacketBufferElement&>(linkInfo.frameBuffer.top()).framePtr));

        linkInfo.frameBuffer.pop();
    }//while//

}//RetrieveFramesToSendUpStack//


}//namespace//

#endif
