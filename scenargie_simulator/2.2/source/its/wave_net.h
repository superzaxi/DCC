// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef WSMP_H
#define WSMP_H

#include <cassert>

#include "wave_mac.h"

namespace Wave {

using ScenSim::ENQUEUE_SUCCESS;
using ScenSim::ENQUEUE_FAILURE_BY_MAX_PACKETS;
using ScenSim::ENQUEUE_FAILURE_BY_MAX_BYTES;
using ScenSim::ConvertToUChar;
using ScenSim::ConvertToUShortInt;

class WaveMac;

// See IEEE 1609.3
// WAVE = "Wireless Access in Vehicular Environment"
// WSMP = "WAVE Short Message Protocol"

// WSM = "WAVE Short Message"
// WSA = "WAVE Service Advertisement"

typedef int WaveElementIdType;
enum {
    WSA_SERVICE_INFO = 1,
    WSA_CHANNEL_INFO = 2,
    WSA_WRA = 3,

    EXTENSION_TX_POWER = 4,
    EXTENSION_2D_LOCATION = 5,
    EXTENSION_3DLOCATION_AND_CONFIDENCE = 6,
    EXTENSION_ADVERTISER_IDENTITIER = 7,
    EXTENSION_PROVIDER_SERVICE_CONTEXT = 8,
    EXTENSION_IPV6_ADDRESS = 9,
    EXTENSION_SERVICE_PORT = 10,
    EXTENSION_PROVIDER_MAC_ADDRESS = 11,
    EXTENSION_EDCA_PARAMETERS_SET = 12,
    EXTENSION_SECONDARY_DNS = 13,
    EXTENSION_GATEWAY_MAC_ADDRESS = 14,
    EXTENSION_CHANNEL_NUMBER = 15,
    EXTENSION_DATARATE = 16,
    EXTENSION_REPEAT_RATE = 17,
    EXTENSION_COUNTRY_STRNIG = 18,
    EXTENSION_RCPI_THRESHOLD = 19,
    EXTENSION_WSA_COUNT_THRESHOLD = 20,
    EXTENSION_CHANNEL_ACCESS = 21,
    EXTENSION_WSA_COUNT_THRESHOLD_INTERVAL = 22,

    // 23-127 reserved.

    WSMP_WAVE_SHORT_MESSAGE = 128,
    WSMP_WSMP_S = 129,
    WSMP_WSMP_I = 139,

    // 131-255 reserved.
};//WaveElementIdType//

struct EdcaParameterType {
    bool admissionControlIsEnabled;
    uint8_t arbitrationInterframeSpaceNumber;
    uint16_t contentionWindowMin;
    uint16_t contentionWindowMax;
    uint16_t txopDurationUs;

    EdcaParameterType()
        :
        admissionControlIsEnabled(false),
        arbitrationInterframeSpaceNumber(2),
        contentionWindowMin(3),
        contentionWindowMax(7),
        txopDurationUs(0)
    {}
};//EdcaParameterType//

class WsmpLayer {
public:
    static const string modelName;

    WsmpLayer(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeId& initNodeId,
        const InterfaceId& initInterfaceId,
        const RandomNumberGeneratorSeed& initNodeSeed,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const shared_ptr<WaveMac>& initWaveMacPtr);

    // WSM-WaveShortMessage.request
    // no extension field in default

    void SendWsm(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& destinationAddress,
        const ChannelNumberIndexType& channelNumberId,
        const string& providerServiceId,
        const PacketPriority& priority);

    void SendWsm(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& destinationAddress,
        const ChannelNumberIndexType& channelNumberId,
        const string& providerServiceId,
        const PacketPriority& priority,
        const DatarateBitsPerSec& datarateBps,
        const double txPowerDbm,
        const uint64_t enabledWaveElementExtensionBitSet);

    // WME-ProviderService.request (Secured WSA is not supported.)

    void SetWsaHeaderExtension(
        const uint64_t initEnabledWsaHeaderExtensionBitSet);

    void AddOrUpdateWsaServiceInfo(
        const int localServiceInfoId, // 0 to 65535 value.
        const string& providerServiceId,
        const uint8_t servicePriority,
        const ChannelNumberIndexType& channelNumberId,
        const ChannelAccessType& channelAccess,
        const int numberTransmissionPer5sec,
        const uint64_t enabledWaveElementExtensionBitSet,
        const string& providerServiceContext,
        const NetworkAddress& ipv6Address,
        const unsigned short portNumber,
        const GenericMacAddress& providerMacAddress,
        const uint8_t rcpiThreshold,
        const uint8_t wsaCountThreshold,
        const uint8_t wsaCountThresholdInterval);

    void DeleteWsaServiceInfo(
        const int localServiceInfoId);

    void ReceivePacketFromMac(
        unique_ptr<Packet>& packetPtr,
        const GenericMacAddress& peerMacAddress);

    void SetWaveMacLayer(
        const shared_ptr<MacLayer>& macLayerPtr);

    void SetInterfaceOutputQueue(
        const shared_ptr<InterfaceOutputQueue>& outputQueuePtr);

    class WsmApplicationHandler {
    public:
        virtual ~WsmApplicationHandler() {}
        virtual void ReceiveWsm(unique_ptr<Packet>& packetPtr) = 0;
    };

    void SetWsmApplicationHandler(
        const string& providerServiceId,
        const shared_ptr<WsmApplicationHandler>& wsmApplicationHandlerPtr);

    DatarateBitsPerSec GetDatarateBps(const ChannelNumberIndexType& channelNumberId);
    double GetTxPowerDbm(const ChannelNumberIndexType& channelNumberId);

private:
    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    shared_ptr<ObjectMobilityModel> mobilityModelPtr;

    shared_ptr<NetworkLayer> networkLayerPtr;
    shared_ptr<WaveMac> waveMacPtr;

    map<string, shared_ptr<WsmApplicationHandler> > wsmApplicationHandlerPtrs;

    struct ServiceInfo {
        string providerServiceId;
        uint8_t servicePriority;
        ChannelNumberIndexType channelNumberId;

        int numberTransmissionsPer5sec;
        SimTime earliestNextTransmissionTime;

        uint64_t enabledWaveElementExtensionBitSet;
        string providerServiceContext;
        NetworkAddress ipv6Address;
        uint16_t portNumber;
        GenericMacAddress providerMacAddress;

        uint8_t rcpiThreshold;
        uint8_t wsaCountThreshold;
        uint8_t wsaCountThresholdInterval;

        ServiceInfo()
            :
            servicePriority(0),
            channelNumberId(CHANNEL_NUMBER_172),
            numberTransmissionsPer5sec(0),
            earliestNextTransmissionTime(ZERO_TIME),
            enabledWaveElementExtensionBitSet(0),
            portNumber(0),
            providerMacAddress(0),
            rcpiThreshold(0),
            wsaCountThreshold(0),
            wsaCountThresholdInterval(0)
        {}

        bool IsEnabled() const { return !providerServiceId.empty(); }
    };//ServiceInfo//

    struct ChannelInfo {
        uint8_t operatingClass;
        uint8_t adaptable;
        uint8_t datarate500Kbps;
        int8_t txPowerDbm;

        uint64_t enabledWaveElementExtensionBitSet;
        vector<EdcaParameterType> edcaParameters;
        ChannelAccessType channelAccess;

        ChannelInfo()
            :
            operatingClass(17),
            adaptable(0),
            datarate500Kbps(0),
            txPowerDbm(0),
            enabledWaveElementExtensionBitSet(0),
            channelAccess(CHANNEL_ACCESS_ALTERNATING)
        {}
    };//ChannelInfo//

    struct RoutingAdvertisement {
        uint16_t lifeTime;
        NetworkAddress ipv6PrefixNetworkAddress;
        uint8_t prefixLength;
        NetworkAddress defaultGatewayNetworkAddress;
        NetworkAddress primaryDnsNetworkAddress;

        uint64_t enabledWaveElementExtensionBitSet;
        NetworkAddress secondaryDnsNetworkAddress;
        GenericMacAddress defaultGatewayMacAddress;

        RoutingAdvertisement()
            :
            lifeTime(0),
            prefixLength(0),
            enabledWaveElementExtensionBitSet(0),
            defaultGatewayMacAddress(0)
        {}
    };//RoutingAdvertisement//

    // - Provider advertisement
    // - Users
    // The provider role is assumed by a device
    // transmitting WAVE Service Advertisements (WSAs)
    // indicating its availability for data exchange on
    // one or more SCHs. The user role is assumed by
    // a device monitoring for received WSAs, with the
    // potential to participate in the SCH data exchange.
    // A WAVE device may assume one, both, or neither role.

    vector<ServiceInfo> serviceInfos;
    vector<ChannelInfo> channelInfos;
    vector<RoutingAdvertisement> routingAdvertisements;

    bool isProvider;
    int numberMaxTransmissionPer5sec;
    PacketPriority wsaPriority;
    SimTime wsaTransmissionInterval;
    string wsaAdvertiserId;
    uint64_t enabledWsaHeaderExtensionBitSet;
    int wsaChangeCount;
    ChannelNumberIndexType wsaChannelNumberId;

    struct WsaReceptionInfo {
        uint8_t waveVersionAndChangeCount;

        uint64_t enabledHeaderExtensionBitSet;
        uint8_t numberTransmissionPer5sec;
        int8_t txPowerDbm;

        ObjectMobilityPosition position;
        uint8_t positionAndElevationConfidence;
        uint32_t positionAccuracy;

        string advertiserId;
        string countryString;

        WsaReceptionInfo()
            :
            waveVersionAndChangeCount(0),
            enabledHeaderExtensionBitSet(0),
            numberTransmissionPer5sec(0),
            txPowerDbm(0),
            positionAndElevationConfidence(0),
            positionAccuracy(0)
        {}
    };//WsaReceptionInfo//

    WsaReceptionInfo currentWsaReceptionInfo;

    class PeriodicWsaTransmissionEvent : public SimulationEvent {
    public:
        PeriodicWsaTransmissionEvent(WsmpLayer* initWsmpLayer) : wsmpLayer(initWsmpLayer) {}
        virtual void ExecuteEvent() {
            wsmpLayer->wsaTimerTicket.Clear();
            wsmpLayer->PeriodicallyTransmitWsa();
        }
    private:
        WsmpLayer* wsmpLayer;
    };//PeriodicWsaTransmissionEvent//

    shared_ptr<PeriodicWsaTransmissionEvent> wsaTransmissionTimerPtr;
    EventRescheduleTicket wsaTimerTicket;


    class WsmpPacketHandler : public SimpleMacPacketHandler {
    public:
        WsmpPacketHandler(WsmpLayer* initWsmpLayer) : wsmpLayer(initWsmpLayer) {}
        void ReceivePacketFromMac(
            unique_ptr<Packet>& packetPtr,
            const GenericMacAddress& transmitterAddress) {
            wsmpLayer->ReceivePacketFromMac(packetPtr, transmitterAddress);
        }
    private:
        WsmpLayer* wsmpLayer;
    };//WsmpPacketHandler//

    void UpdateWsaMaxTransmissionInterval();
    void PeriodicallyTransmitWsa();
    void RescheduleNextWsaTransmission();
    bool AtLeastOneServiceInfoShouldBeSent() const;

    void InsertPacketIntoQueue(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& destinationAddress,
        const ChannelNumberIndexType& channelNumberId,
        const PacketPriority& priority,
        const DatarateBitsPerSec& datarateBps,
        const double txPowerDbm);

    void ReceiveWsm(
        unique_ptr<Packet>& packetPtr,
        const GenericMacAddress& peerMacAddress);

    void ReceiveWsa(
        unique_ptr<Packet>& packetPtr,
        const GenericMacAddress& providerMacAddress);

    //trace and stats
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<CounterStatistic> packetsSentStatPtr;

    shared_ptr<CounterStatistic> bytesReceivedStatPtr;
    shared_ptr<CounterStatistic> packetsReceivedStatPtr;
    shared_ptr<CounterStatistic> packetMaxPacketsQueueDropsStatPtr;
    shared_ptr<CounterStatistic> packetMaxBytesQueueDropsStatPtr;

    void OutputTraceAndStatsForInsertPacketIntoQueue(const Packet& packet) const;
    void OutputTraceAndStatsForReceivePacketFromMac(const Packet& packet) const;
    void OutputTraceAndStatsForFullQueueDrop(
        const Packet& packet, const EnqueueResultType enqueueResult) const;

};//WsmpLayer

//---------------------------------------------------------------------------

static inline
bool ProviderServiceIdIsCorrectFormat(const string& providerServiceId)
{
    const unsigned char firstOctet = static_cast<unsigned char>(providerServiceId[0]);

    if (!(0 < providerServiceId.length() && providerServiceId.length() <= 4)) {
        return false;
    }//if//

    switch(providerServiceId.length()) {
    case 1: return (firstOctet < 0x80);
    case 2: return (0x80 <= firstOctet && firstOctet < 0xC0);
    case 3: return (0xC0 <= firstOctet && firstOctet < 0xE0);
    case 4: return (0xE0 <= firstOctet);
    default: break;
    }//switch//

    return false;
}//ProviderServiceIdIsCorrectFormat//

static inline
string ConvertToProviderServiceIdString(const uint32_t providerServiceId)
{
    list<char> aString;

    for(size_t i = 0; i < 4; i++) {
        const size_t bitShift = (8*i);

        aString.push_front(char((providerServiceId & (0xFF << bitShift)) >> bitShift));

        if (providerServiceId <= uint32_t(0xFF << bitShift)) {
            break;
        }
    }//for//

    const string providerServiceIdString(aString.begin(), aString.end());

    if (!ProviderServiceIdIsCorrectFormat(providerServiceIdString)) {
        cerr << "Error: invalid provider service id " << providerServiceId << "(";
        for(size_t i = 0; i < providerServiceIdString.length(); i++) {
            cerr << " " << int(uint8_t(providerServiceIdString[i]));
        }//for//
        cerr << ")" << endl
             << "valid id 0-127,32768-49151,12582912-14680063,3758096384-4026531839" << endl;
        exit(1);
    }//if//

    return providerServiceIdString;
}//ConvertToProviderServiceIdString//

static inline
bool ExtensionFieldIsEnabled(
    const uint64_t enabledWaveElementExtensionBitSet,
    const WaveElementIdType& wsmElementId)
{
    const uint64_t extensionBit = (uint64_t(1) << wsmElementId);

    return ((enabledWaveElementExtensionBitSet & extensionBit) == extensionBit);
}//ExtensionFieldIsEnabled//

static inline
void EnableExtensionField(
    const WaveElementIdType& wsmElementId,
    uint64_t& enabledWaveElementExtensionBitSet)
{
    const uint64_t extensionBit = (uint64_t(1) << wsmElementId);

    enabledWaveElementExtensionBitSet = extensionBit;
}//EnableExtensionField//

// Write

template <typename T> static inline
void WriteBasicField(
    const T& value,
    size_t& currentWriteBytes,
    unsigned char buf[])
{
    const size_t valueSize = sizeof(value);

    const unsigned char* valuePtr = reinterpret_cast<const unsigned char* >(&value);

    for(size_t i = 0; i < valueSize; i++) {
        buf[currentWriteBytes+i] = valuePtr[i];
    }//for

    currentWriteBytes += valueSize;
}//WriteBasicField//

static inline
void WriteProviderServiceId(
    const string& providerServiceId,
    size_t& currentWriteBytes,
    unsigned char buf[])
{
    assert(ProviderServiceIdIsCorrectFormat(providerServiceId));

    for(size_t i = 0; i < providerServiceId.length(); i++) {
        buf[currentWriteBytes+i] = providerServiceId[i];
    }//for

    currentWriteBytes += providerServiceId.length();
}//WriteProviderServiceId//

static inline
void WriteExtensionFieldHeader(
    const WaveElementIdType& wsmElementId,
    const size_t fieldSize,
    size_t& currentWriteBytes,
    unsigned char buf[])
{
    buf[currentWriteBytes] = static_cast<unsigned char>(wsmElementId);
    buf[currentWriteBytes+1] = static_cast<unsigned char>(fieldSize);

    currentWriteBytes += 2;
}//WriteExtensionFieldHeader//

template <typename T> static inline
void WriteBasicExtensionFieldIfNecessary(
    const uint64_t enabledWaveElementExtensionBitSet,
    const WaveElementIdType& wsmElementId,
    const T& value,
    size_t& currentWriteBytes,
    unsigned char buf[])
{
    if (ExtensionFieldIsEnabled(enabledWaveElementExtensionBitSet, wsmElementId)) {
        const size_t valueSize = sizeof(value);

        WriteExtensionFieldHeader(wsmElementId, valueSize, currentWriteBytes, buf);
        WriteBasicField(value, currentWriteBytes, buf);
    }//if//
}//WriteBasicExtensionFieldIfNecessary//

static inline
void WriteStringField(
    const string& aString,
    size_t& currentWriteBytes,
    unsigned char buf[])
{
    for(size_t i = 0; i < aString.length(); i++) {
        buf[currentWriteBytes+i] = aString[i];
    }//for//

    currentWriteBytes += aString.length();
}//WriteStringField//

static inline
void WriteStringExtensionFieldIfNecessary(
    const uint64_t enabledWaveElementExtensionBitSet,
    const WaveElementIdType& wsmElementId,
    const string& aString,
    size_t& currentWriteBytes,
    unsigned char buf[])
{
    if (ExtensionFieldIsEnabled(enabledWaveElementExtensionBitSet, wsmElementId)) {
        WriteExtensionFieldHeader(wsmElementId, aString.length(), currentWriteBytes, buf);
        WriteStringField(aString, currentWriteBytes, buf);
    }//if//
}//WriteStringExtensionFieldIfNecessary//

static inline
void WriteIpv6AddressField(
    const NetworkAddress& ipv6Address,
    size_t& currentWriteBytes,
    unsigned char buf[])
{
    const uint64_t lowAddressBits = ipv6Address.GetRawAddressLowBits();
    const uint64_t highAddressBits = ipv6Address.GetRawAddressLowBits();
    WriteBasicField(lowAddressBits, currentWriteBytes, buf);
    WriteBasicField(highAddressBits, currentWriteBytes, buf);
}//WriteIpv6AddressField//

static inline
void WriteIpv6AddressExtensionFieldIfNecessary(
    const uint64_t enabledWaveElementExtensionBitSet,
    const WaveElementIdType& wsmElementId,
    const NetworkAddress& ipv6Address,
    size_t& currentWriteBytes,
    unsigned char buf[])
{
    if (ExtensionFieldIsEnabled(enabledWaveElementExtensionBitSet, wsmElementId)) {
        WriteExtensionFieldHeader(wsmElementId, 16, currentWriteBytes, buf);
        WriteIpv6AddressField(ipv6Address, currentWriteBytes, buf);
    }//if//
}//WriteIpv6AddressExtensionFieldIfNecessary//

static inline
void WriteMacAddressExtensionFieldIfNecessary(
    const uint64_t enabledWaveElementExtensionBitSet,
    const WaveElementIdType& wsmElementId,
    const GenericMacAddress& macAddress,
    size_t& currentWriteBytes,
    unsigned char buf[])
{
    if (ExtensionFieldIsEnabled(enabledWaveElementExtensionBitSet, wsmElementId)) {
        WriteExtensionFieldHeader(wsmElementId, 6, currentWriteBytes, buf);
        WriteBasicField(uint16_t(macAddress >> 32), currentWriteBytes, buf);
        WriteBasicField(uint32_t(macAddress & 0xFFFFFFFF), currentWriteBytes, buf);
    }//if//
}//WriteMacAddressExtensionFieldIfNecessary//

static inline
void Write2dLocationExtensionFieldIfNecessary(
    const uint64_t enabledWaveElementExtensionBitSet,
    const WaveElementIdType& wsmElementId,
    const ObjectMobilityPosition& position,
    size_t& currentWriteBytes,
    unsigned char buf[])
{
    if (ExtensionFieldIsEnabled(enabledWaveElementExtensionBitSet, wsmElementId)) {
        WriteExtensionFieldHeader(wsmElementId, 8, currentWriteBytes, buf);
        WriteBasicField(float(position.X_PositionMeters()), currentWriteBytes, buf);
        WriteBasicField(float(position.Y_PositionMeters()), currentWriteBytes, buf);
    }//if//
}//Write2dLocationExtensionFieldIfNecessary//

static inline
void Write3dLocationExtensionFieldIfNecessary(
    const uint64_t enabledWaveElementExtensionBitSet,
    const WaveElementIdType& wsmElementId,
    const ObjectMobilityPosition& position,
    size_t& currentWriteBytes,
    unsigned char buf[])
{
    if (ExtensionFieldIsEnabled(enabledWaveElementExtensionBitSet, wsmElementId)) {
        const uint8_t positionConfidence = 0xf0;//0.01m -> See J2735 PositionConfidence
        const uint8_t elevationConfidence = 0x0f;// 0.01m
        const uint32_t positionAccuracy = 0x00ffffff;

        WriteExtensionFieldHeader(wsmElementId, 15, currentWriteBytes, buf);
        WriteBasicField(float(position.X_PositionMeters()), currentWriteBytes, buf);
        WriteBasicField(float(position.Y_PositionMeters()), currentWriteBytes, buf);
        WriteBasicField(uint16_t(position.HeightFromGroundMeters()*10), currentWriteBytes, buf);
        WriteBasicField(uint8_t(positionConfidence + elevationConfidence), currentWriteBytes, buf);
        WriteBasicField(uint32_t(positionAccuracy), currentWriteBytes, buf);
    }//if//
}//Write3dLocationExtensionFieldIfNecessary//

static inline
void WriteEdcaExtensionFieldIfNecessary(
    const uint64_t enabledWaveElementExtensionBitSet,
    const WaveElementIdType& wsmElementId,
    const vector<EdcaParameterType>& edcaParams,
    size_t& currentWriteBytes,
    unsigned char buf[])
{
    if (ExtensionFieldIsEnabled(enabledWaveElementExtensionBitSet, wsmElementId)) {
        for(size_t i = 0; i < edcaParams.size(); i++) {
            const EdcaParameterType& edcaParam = edcaParams[i];

            const uint8_t acIndexAndAcmAndAifsn =
                (uint8_t(i) << 5) +
                (uint8_t(edcaParam.admissionControlIsEnabled) << 5) +
                (uint8_t(edcaParam.arbitrationInterframeSpaceNumber));

            const uint8_t ecwMaxAndMin =
                (uint8_t(edcaParam.contentionWindowMax) << 4) +
                (uint8_t(edcaParam.contentionWindowMax));

            WriteExtensionFieldHeader(wsmElementId, 4, currentWriteBytes, buf);
            WriteBasicField(uint8_t(acIndexAndAcmAndAifsn), currentWriteBytes, buf);
            WriteBasicField(uint8_t(ecwMaxAndMin), currentWriteBytes, buf);
            WriteBasicField(uint16_t(edcaParam.txopDurationUs), currentWriteBytes, buf);
        }//for//
    }//if//
}//WriteEdcaExtensionFieldIfNecessary//

// Read

template <typename T> static inline
void ReadBasicField(
    const unsigned char buf[],
    T& value,
    size_t& currentReadBytes)
{
    const size_t valueSize = sizeof(value);

    unsigned char* valuePtr = reinterpret_cast<unsigned char* >(&value);

    for(size_t i = 0; i < valueSize; i++) {
        valuePtr[i] = buf[currentReadBytes+i];
    }//for//

    currentReadBytes += valueSize;
}//ReadBasicField//

static inline
void ReadProviderServiceId(
    const unsigned char buf[],
    string& providerServiceId,
    size_t& currentReadBytes)
{
    providerServiceId.clear();

    const unsigned char firstOctet = buf[currentReadBytes];

    int serviceIdLength = 1;
    while (firstOctet & (1 << (8 - (serviceIdLength)))) {
        serviceIdLength++;
    }//while//

    providerServiceId.resize(serviceIdLength);

    for(size_t i = 0; i < providerServiceId.length(); i++) {
        providerServiceId[i] = buf[currentReadBytes+i];
    }//for//

    currentReadBytes += serviceIdLength;
}//ReadProviderServiceId//

static inline
void ReadExtensionFieldHeader(
    const unsigned char buf[],
    WaveElementIdType& wsmElementId,
    size_t& fieldSize,
    size_t& currentReadBytes)
{
    wsmElementId = buf[currentReadBytes];
    fieldSize = buf[currentReadBytes+1];

    currentReadBytes += 2;
}//ReadExtensionFieldHeader//

static inline
WaveElementIdType PeekElementIdField(
    const unsigned char buf[],
    size_t& currentReadBytes)
{
    return buf[currentReadBytes];
}//PeekElementIdField//


template <typename T> static inline
void ReadBasicExtensionFieldIfNecessary(
    const unsigned char buf[],
    const WaveElementIdType& wsmElementId,
    uint64_t& enabledWaveElementExtensionBitSet,
    T& value,
    size_t& currentReadBytes)
{
    if (PeekElementIdField(buf, currentReadBytes) == wsmElementId) {
        EnableExtensionField(wsmElementId, enabledWaveElementExtensionBitSet);

        const size_t valueSize = sizeof(value);

        WaveElementIdType wsmElementId_NotUsed;
        size_t fieldSize;

        ReadExtensionFieldHeader(buf, wsmElementId_NotUsed, fieldSize, currentReadBytes);
        assert(wsmElementId == wsmElementId_NotUsed);
        assert(fieldSize == valueSize);
        ReadBasicField(buf, value, currentReadBytes);
    }//if//
}//ReadBasicExtensionFieldIfNecessary//

static inline
void ReadStringField(
    const unsigned char buf[],
    string& aString,
    size_t& currentReadBytes)
{
    for(size_t i = 0; i < aString.length(); i++) {
        aString[i] = buf[currentReadBytes+i];
    }//for//

    currentReadBytes += aString.length();
}//ReadStringField//

static inline
void ReadStringExtensionFieldIfNecessary(
    const unsigned char buf[],
    const WaveElementIdType& wsmElementId,
    uint64_t& enabledWaveElementExtensionBitSet,
    string& aString,
    size_t& currentReadBytes)
{
    if (PeekElementIdField(buf, currentReadBytes) == wsmElementId) {
        EnableExtensionField(wsmElementId, enabledWaveElementExtensionBitSet);

        WaveElementIdType wsmElementId_NotUsed;
        size_t fieldSize;

        ReadExtensionFieldHeader(buf, wsmElementId_NotUsed, fieldSize, currentReadBytes);
        assert(wsmElementId == wsmElementId_NotUsed);
        aString.resize(fieldSize);
        ReadStringField(buf, aString, currentReadBytes);
    }//if//
}//ReadStringExtensionFieldIfNecessary//

static inline
void ReadIpv6AddressField(
    const unsigned char buf[],
    NetworkAddress& ipv6Address,
    size_t& currentReadBytes)
{
    uint64_t addressLowBits;
    uint64_t addressHighBits;

    ReadBasicField(buf, addressLowBits, currentReadBytes);
    ReadBasicField(buf, addressHighBits, currentReadBytes);

    ipv6Address = NetworkAddress(addressHighBits, addressLowBits);
}//ReadIpv6AddressField//

static inline
void ReadIpv6AddressExtensionFieldIfNecessary(
    const unsigned char buf[],
    const WaveElementIdType& wsmElementId,
    uint64_t& enabledWaveElementExtensionBitSet,
    NetworkAddress& ipv6Address,
    size_t& currentReadBytes)
{
    if (PeekElementIdField(buf, currentReadBytes) == wsmElementId) {
        EnableExtensionField(wsmElementId, enabledWaveElementExtensionBitSet);

        WaveElementIdType wsmElementId_NotUsed;
        size_t fieldSize;

        ReadExtensionFieldHeader(buf, wsmElementId_NotUsed, fieldSize, currentReadBytes);
        assert(wsmElementId == wsmElementId_NotUsed);
        assert(fieldSize == 16);
        ReadIpv6AddressField(buf, ipv6Address, currentReadBytes);
    }//if//
}//ReadIpv6AddressExtensionFieldIfNecessary//

static inline
void ReadMacAddressExtensionFieldIfNecessary(
    const unsigned char buf[],
    const WaveElementIdType& wsmElementId,
    uint64_t& enabledWaveElementExtensionBitSet,
    GenericMacAddress& macAddress,
    size_t& currentReadBytes)
{
    if (PeekElementIdField(buf, currentReadBytes) == wsmElementId) {
        EnableExtensionField(wsmElementId, enabledWaveElementExtensionBitSet);

        WaveElementIdType wsmElementId_NotUsed;
        size_t fieldSize;

        ReadExtensionFieldHeader(buf, wsmElementId_NotUsed, fieldSize, currentReadBytes);
        assert(wsmElementId == wsmElementId_NotUsed);
        assert(fieldSize == 6);

        uint16_t rawAddressHighBits;
        uint32_t rawAddressLowBits;

        ReadBasicField(buf, rawAddressHighBits, currentReadBytes);
        ReadBasicField(buf, rawAddressLowBits, currentReadBytes);

        macAddress =
            GenericMacAddress(GenericMacAddress(rawAddressHighBits) << 32) + rawAddressLowBits;
    }//if//
}//ReadMacAddressExtensionFieldIfNecessary//

static inline
void Read2dLocationExtensionFieldIfNecessary(
    const unsigned char buf[],
    const WaveElementIdType& wsmElementId,
    uint64_t& enabledWaveElementExtensionBitSet,
    ObjectMobilityPosition& position,
    size_t& currentReadBytes)
{
    if (PeekElementIdField(buf, currentReadBytes) == wsmElementId) {
        EnableExtensionField(wsmElementId, enabledWaveElementExtensionBitSet);

        WaveElementIdType wsmElementId_NotUsed;
        size_t fieldSize;

        ReadExtensionFieldHeader(buf, wsmElementId_NotUsed, fieldSize, currentReadBytes);
        assert(wsmElementId == wsmElementId_NotUsed);
        assert(fieldSize == 8);

        float xMeters;
        float yMeters;

        ReadBasicField(buf, xMeters, currentReadBytes);
        ReadBasicField(buf, yMeters, currentReadBytes);

        position.SetX_PositionMeters(xMeters);
        position.SetY_PositionMeters(yMeters);
    }//if//
}//Read2dLocationExtensionFieldIfNecessary//

static inline
void Read3dLocationExtensionFieldIfNecessary(
    const unsigned char buf[],
    const WaveElementIdType& wsmElementId,
    uint64_t& enabledWaveElementExtensionBitSet,
    ObjectMobilityPosition& position,
    uint8_t& positionAndElevationConfidence,
    uint32_t& positionAccuracy,
    size_t& currentReadBytes)
{
    if (PeekElementIdField(buf, currentReadBytes) == wsmElementId) {
        EnableExtensionField(wsmElementId, enabledWaveElementExtensionBitSet);

        WaveElementIdType wsmElementId_NotUsed;
        size_t fieldSize;

        ReadExtensionFieldHeader(buf, wsmElementId_NotUsed, fieldSize, currentReadBytes);
        assert(wsmElementId == wsmElementId_NotUsed);
        assert(fieldSize == 15);

        float xMeters;
        float yMeters;
        uint16_t heightMeters;

        ReadBasicField(buf, xMeters, currentReadBytes);
        ReadBasicField(buf, yMeters, currentReadBytes);
        ReadBasicField(buf, heightMeters, currentReadBytes);
        ReadBasicField(buf, positionAndElevationConfidence, currentReadBytes);
        ReadBasicField(buf, positionAccuracy, currentReadBytes);

        position.SetX_PositionMeters(xMeters);
        position.SetY_PositionMeters(yMeters);
        position.SetHeightFromGroundMeters(heightMeters);
    }//if//
}//Read3dLocationExtensionFieldIfNecessary//

static inline
void ReadEdcaExtensionFieldIfNecessary(
    const unsigned char buf[],
    const WaveElementIdType& wsmElementId,
    uint64_t& enabledWaveElementExtensionBitSet,
    vector<EdcaParameterType>& edcaParams,
    size_t& currentReadBytes)
{
    edcaParams.resize(4);

    while (PeekElementIdField(buf, currentReadBytes) == wsmElementId) {
        WaveElementIdType wsmElementId_NotUsed;
        size_t fieldSize;
        uint8_t acIndexAndAcmAndAifsn;
        uint8_t ecwMaxAndMin;

        ReadExtensionFieldHeader(buf, wsmElementId_NotUsed, fieldSize, currentReadBytes);
        assert(wsmElementId == wsmElementId_NotUsed);
        ReadBasicField(buf, acIndexAndAcmAndAifsn, currentReadBytes);
        ReadBasicField(buf, ecwMaxAndMin, currentReadBytes);

        const uint8_t accessCategoryIndex = ((acIndexAndAcmAndAifsn & 0x60) >> 5);
        EdcaParameterType& edcaParam = edcaParams.at(accessCategoryIndex);

        ReadBasicField(buf, edcaParam.txopDurationUs, currentReadBytes);

        edcaParam.admissionControlIsEnabled = ((acIndexAndAcmAndAifsn & 0x10) != 0);
        edcaParam.arbitrationInterframeSpaceNumber = (acIndexAndAcmAndAifsn & 0xF);
        edcaParam.contentionWindowMax = (ecwMaxAndMin >> 4);
        edcaParam.contentionWindowMax = (ecwMaxAndMin & 0xF);
    }//while//
}//ReadEdcaExtensionFieldIfNecessary//

static inline
SimTime ConvertToTranssmisionInterval(const int numberTransmissionsPer5sec)
{
    assert(numberTransmissionsPer5sec > 0);
    return static_cast<SimTime>((5*SECOND) / numberTransmissionsPer5sec);
}//ConvertToTranssmisionInterval//

//--------------------------------------------------------------------------

#pragma warning(disable:4355)

inline
WsmpLayer::WsmpLayer(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeId& initNodeId,
    const InterfaceId& initInterfaceId,
    const RandomNumberGeneratorSeed& initNodeSeed,
    const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const shared_ptr<WaveMac>& initWaveMacPtr)
    :
    simEngineInterfacePtr(initSimulationEngineInterfacePtr),
    mobilityModelPtr(initNodeMobilityModelPtr),
    networkLayerPtr(initNetworkLayerPtr),
    waveMacPtr(initWaveMacPtr),
    channelInfos(NUMBER_CHANNELS),
    isProvider(false),
    numberMaxTransmissionPer5sec(0),
    wsaPriority(3),
    wsaTransmissionInterval(INFINITE_TIME),
    enabledWsaHeaderExtensionBitSet(0),
    wsaChangeCount(0),
    wsaChannelNumberId(CHANNEL_NUMBER_178),
    wsaTransmissionTimerPtr(
        new PeriodicWsaTransmissionEvent(this)),
    bytesSentStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_BytesSent")),
    packetsSentStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_PacketsSent")),
    bytesReceivedStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_BytesReceived")),
    packetsReceivedStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_PacketsReceived")),
    packetMaxPacketsQueueDropsStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_MaxPacketsQueueDrops")),
    packetMaxBytesQueueDropsStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_MaxBytesQueueDrops"))
{
    initWaveMacPtr->SetWsmpPacketHandler(
        shared_ptr<WsmpPacketHandler>(new WsmpPacketHandler(this)));

    if (theParameterDatabaseReader.ParameterExists(
            "its-wsmp-wsa-packet-priority", initNodeId, initInterfaceId)) {
        wsaPriority =
            ConvertToUChar(
                theParameterDatabaseReader.ReadNonNegativeInt(
                    "its-wsmp-wsa-packet-priority", initNodeId, initInterfaceId),
                "Error in parameter: \"its-wsmp-wsa-packet-priority\": out of range");
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            "its-wsmp-wsa-packet-channel-number", initNodeId, initInterfaceId)) {
        wsaChannelNumberId =
            ConvertToChannelNumberIndex(
                theParameterDatabaseReader.ReadInt(
                    "its-wsmp-wsa-packet-channel-number", initNodeId, initInterfaceId));
    }//if//

    for(size_t i = 0; i < channelInfos.size(); i++) {
        ChannelInfo& channelInfo = channelInfos[i];
        const string channelNumberString = ConvertChannelNumberIdToString(ChannelNumberIndexType(i));

        if (theParameterDatabaseReader.ParameterExists(
                "its-wsmp-ch" + channelNumberString + "-default-datarate-bits-per-second", initNodeId, initInterfaceId)) {

            channelInfo.datarate500Kbps = static_cast<uint8_t>(
                theParameterDatabaseReader.ReadBigInt(
                    "its-wsmp-ch" + channelNumberString + "-default-datarate-bits-per-second", initNodeId, initInterfaceId) / 500000);
        } else {
            channelInfo.datarate500Kbps = static_cast<uint8_t>(
                theParameterDatabaseReader.ReadBigInt(
                    "its-wsmp-default-datarate-bits-per-second", initNodeId, initInterfaceId) / 500000);
        }//if//

        double txPowerDbm;

        if (theParameterDatabaseReader.ParameterExists(
                "its-wsmp-ch" + channelNumberString + "-default-tx-power", initNodeId, initInterfaceId)) {

            txPowerDbm =
                theParameterDatabaseReader.ReadDouble(
                    "its-wsmp-ch" + channelNumberString + "-default-tx-power", initNodeId, initInterfaceId);

            if (txPowerDbm < -127. || 127. < txPowerDbm) {
                cerr << "Set: its-wsmp-ch" << channelNumberString << "-default-tx-power range is -127 to 127" << endl;
                exit(1);
            }

        } else {
            txPowerDbm =
                theParameterDatabaseReader.ReadDouble(
                    "its-wsmp-default-tx-power", initNodeId, initInterfaceId);

            if (txPowerDbm < -127. || 127. < txPowerDbm) {
                cerr << "Set: its-wsmp-default-tx-power range is -127 to 127" << endl;
                exit(1);
            }

        }//if//

        channelInfo.txPowerDbm = static_cast<int8_t>(txPowerDbm);

    }//for//
}//WsmpLayer//

#pragma warning(default:4355)

inline
void WsmpLayer::AddOrUpdateWsaServiceInfo(
    const int localServiceId,
    const string& providerServiceId,
    const uint8_t servicePriority,
    const ChannelNumberIndexType& channelNumberId,
    const ChannelAccessType& channelAccess,
    const int numberTransmissionPer5sec,
    const uint64_t enabledWaveElementExtensionBitSet,
    const string& providerServiceContext,
    const NetworkAddress& ipv6Address,
    const unsigned short portNumber,
    const GenericMacAddress& providerMacAddress,
    const uint8_t rcpiThreshold,
    const uint8_t wsaCountThreshold,
    const uint8_t wsaCountThresholdInterval)
{
    isProvider = true;

    assert(localServiceId < 65535);
    assert(!providerServiceId.empty());

    serviceInfos.resize(std::max<size_t>(serviceInfos.size(), localServiceId + 1));

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
    ServiceInfo& serviceInfo = serviceInfos[localServiceId];

    serviceInfo.providerServiceId = providerServiceId;
    serviceInfo.servicePriority = servicePriority;
    serviceInfo.channelNumberId = channelNumberId;

    serviceInfo.numberTransmissionsPer5sec = numberTransmissionPer5sec;
    serviceInfo.earliestNextTransmissionTime = currentTime;

    serviceInfo.enabledWaveElementExtensionBitSet = enabledWaveElementExtensionBitSet;
    serviceInfo.providerServiceContext = providerServiceContext;
    serviceInfo.ipv6Address = ipv6Address;
    serviceInfo.portNumber = portNumber;
    serviceInfo.providerMacAddress = providerMacAddress;

    serviceInfo.rcpiThreshold = rcpiThreshold;
    serviceInfo.wsaCountThreshold = wsaCountThreshold;
    serviceInfo.wsaCountThresholdInterval = wsaCountThresholdInterval;

    channelInfos.at(channelNumberId).channelAccess = channelAccess;

    (*this).UpdateWsaMaxTransmissionInterval();
    (*this).PeriodicallyTransmitWsa();
}//AddOrUpdateWsaServiceInfo//

inline
void WsmpLayer::DeleteWsaServiceInfo(const int localServiceId)
{
    assert(localServiceId < int(serviceInfos.size()));
    serviceInfos[localServiceId].providerServiceId.clear();

    (*this).UpdateWsaMaxTransmissionInterval();
    (*this).RescheduleNextWsaTransmission();
}//DeleteWsaServiceInfo//

inline
void WsmpLayer::UpdateWsaMaxTransmissionInterval()
{
    numberMaxTransmissionPer5sec = 0;

    if (serviceInfos.empty()) {
        wsaTransmissionInterval = INFINITE_TIME;
        return;
    }//if//

    for(size_t i = 0; i < serviceInfos.size(); i++) {
        const ServiceInfo& serviceInfo = serviceInfos[i];

        numberMaxTransmissionPer5sec = std::max(
            numberMaxTransmissionPer5sec,
            serviceInfo.numberTransmissionsPer5sec);
    }//for//
}//UpdateWsaMaxTransmissionInterval//

inline
void WsmpLayer::RescheduleNextWsaTransmission()
{
    if (numberMaxTransmissionPer5sec == 0) {
        return;
    }//if//

    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
    const SimTime nextTransmissionTime =
        currentTime + ConvertToTranssmisionInterval(numberMaxTransmissionPer5sec);

    assert(wsaTransmissionInterval > ZERO_TIME);

    if (wsaTimerTicket.IsNull()) {
        simEngineInterfacePtr->ScheduleEvent(
            wsaTransmissionTimerPtr, nextTransmissionTime, wsaTimerTicket);
    } else {
        simEngineInterfacePtr->RescheduleEvent(
            wsaTimerTicket, nextTransmissionTime);
    }//if//
}//RescheduleNextWsaTransmission//

inline
bool WsmpLayer::AtLeastOneServiceInfoShouldBeSent() const
{
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();

    for(size_t i = 0; i < serviceInfos.size(); i++) {
        const ServiceInfo& serviceInfo = serviceInfos[i];

        if (serviceInfo.IsEnabled() &&
            currentTime >= serviceInfo.earliestNextTransmissionTime) {
            return true;
        }//if//
    }//for//

    return false;
}//AtLeastOneServiceInfoShouldBeSent//

inline
void WsmpLayer::SetWsaHeaderExtension(
    const uint64_t initEnabledWsaHeaderExtensionBitSet)
{
    enabledWsaHeaderExtensionBitSet = initEnabledWsaHeaderExtensionBitSet;
}//SetWsaHeaderExtension//

inline
void WsmpLayer::PeriodicallyTransmitWsa()
{
    if (!(*this).AtLeastOneServiceInfoShouldBeSent()) {
        (*this).RescheduleNextWsaTransmission();
        return;
    }//if//

    const ChannelInfo& cchInfo = channelInfos.at(wsaChannelNumberId);
    const SimTime currentTime = simEngineInterfacePtr->CurrentTime();
    ObjectMobilityPosition position;
    mobilityModelPtr->GetPositionForTime(currentTime, position);

    static const int maxWsaSizeBytes = 2312;
    static const uint8_t wsmpHeaderVersion = (1 << 2);

    unsigned char wsaBuf[maxWsaSizeBytes];
    size_t currentWriteBytes = 0;

    WriteBasicField(uint8_t(wsmpHeaderVersion + wsaChangeCount), currentWriteBytes, wsaBuf);
    wsaChangeCount = ((wsaChangeCount + 1) % 4);

    // WSA header

    WriteBasicExtensionFieldIfNecessary(
        enabledWsaHeaderExtensionBitSet,
        EXTENSION_REPEAT_RATE, uint8_t(numberMaxTransmissionPer5sec),
        currentWriteBytes, wsaBuf);

    WriteBasicExtensionFieldIfNecessary(
        enabledWsaHeaderExtensionBitSet,
        EXTENSION_TX_POWER, uint8_t(cchInfo.txPowerDbm),
        currentWriteBytes, wsaBuf);

    Write2dLocationExtensionFieldIfNecessary(
        enabledWsaHeaderExtensionBitSet,
        EXTENSION_2D_LOCATION, position,
        currentWriteBytes, wsaBuf);

    Write3dLocationExtensionFieldIfNecessary(
        enabledWsaHeaderExtensionBitSet,
        EXTENSION_3DLOCATION_AND_CONFIDENCE,
        position,
        currentWriteBytes, wsaBuf);

    WriteStringExtensionFieldIfNecessary(
        enabledWsaHeaderExtensionBitSet,
        EXTENSION_ADVERTISER_IDENTITIER, wsaAdvertiserId,
        currentWriteBytes, wsaBuf);

    WriteStringExtensionFieldIfNecessary(
        enabledWsaHeaderExtensionBitSet,
        EXTENSION_COUNTRY_STRNIG, "---",
        currentWriteBytes, wsaBuf);

    // service information

    for(size_t i = 0; i < serviceInfos.size(); i++) {
        ServiceInfo& serviceInfo = serviceInfos[i];

        if (!serviceInfo.IsEnabled() ||
            currentTime < serviceInfo.earliestNextTransmissionTime) {
            continue;
        }//if//

        WriteBasicField(uint8_t(WSA_SERVICE_INFO), currentWriteBytes, wsaBuf);
        WriteProviderServiceId(serviceInfo.providerServiceId, currentWriteBytes, wsaBuf);
        WriteBasicField(uint8_t(serviceInfo.servicePriority), currentWriteBytes, wsaBuf);
        WriteBasicField(uint8_t(serviceInfo.channelNumberId), currentWriteBytes, wsaBuf);

        WriteStringExtensionFieldIfNecessary(
            serviceInfo.enabledWaveElementExtensionBitSet,
            EXTENSION_PROVIDER_SERVICE_CONTEXT, serviceInfo.providerServiceContext,
            currentWriteBytes, wsaBuf);

        WriteIpv6AddressExtensionFieldIfNecessary(
            serviceInfo.enabledWaveElementExtensionBitSet,
            EXTENSION_IPV6_ADDRESS, serviceInfo.ipv6Address,
            currentWriteBytes, wsaBuf);

        WriteBasicExtensionFieldIfNecessary(
            serviceInfo.enabledWaveElementExtensionBitSet,
            EXTENSION_SERVICE_PORT, uint16_t(serviceInfo.portNumber),
            currentWriteBytes, wsaBuf);

        WriteMacAddressExtensionFieldIfNecessary(
            serviceInfo.enabledWaveElementExtensionBitSet,
            EXTENSION_PROVIDER_MAC_ADDRESS, serviceInfo.providerMacAddress,
            currentWriteBytes, wsaBuf);

        WriteBasicExtensionFieldIfNecessary(
            serviceInfo.enabledWaveElementExtensionBitSet,
            EXTENSION_RCPI_THRESHOLD, uint8_t(serviceInfo.rcpiThreshold),
            currentWriteBytes, wsaBuf);

        WriteBasicExtensionFieldIfNecessary(
            serviceInfo.enabledWaveElementExtensionBitSet,
            EXTENSION_WSA_COUNT_THRESHOLD, uint8_t(serviceInfo.wsaCountThreshold),
            currentWriteBytes, wsaBuf);

        WriteBasicExtensionFieldIfNecessary(
            serviceInfo.enabledWaveElementExtensionBitSet,
            EXTENSION_WSA_COUNT_THRESHOLD_INTERVAL, uint8_t(serviceInfo.wsaCountThresholdInterval),
            currentWriteBytes, wsaBuf);

        serviceInfo.earliestNextTransmissionTime =
            currentTime + ConvertToTranssmisionInterval(serviceInfo.numberTransmissionsPer5sec);
    }//for//

    // channel information

    for(size_t i = 0; i < channelInfos.size(); i++) {
        const ChannelInfo& channelInfo = channelInfos[i];

        WriteBasicField(uint8_t(WSA_CHANNEL_INFO), currentWriteBytes, wsaBuf);
        WriteBasicField(uint8_t(channelInfo.operatingClass), currentWriteBytes, wsaBuf);
        WriteBasicField(uint8_t(i), currentWriteBytes, wsaBuf);
        WriteBasicField(uint8_t(channelInfo.adaptable), currentWriteBytes, wsaBuf);
        WriteBasicField(uint8_t(channelInfo.datarate500Kbps), currentWriteBytes, wsaBuf);
        WriteBasicField(uint8_t(channelInfo.txPowerDbm), currentWriteBytes, wsaBuf);

        WriteEdcaExtensionFieldIfNecessary(
            channelInfo.enabledWaveElementExtensionBitSet,
            EXTENSION_EDCA_PARAMETERS_SET, channelInfo.edcaParameters,
            currentWriteBytes, wsaBuf);

        WriteBasicExtensionFieldIfNecessary(
            channelInfo.enabledWaveElementExtensionBitSet,
            EXTENSION_CHANNEL_ACCESS, uint8_t(channelInfo.channelAccess),
            currentWriteBytes, wsaBuf);
    }//for//

    // WAVE Routing Advertisement

    for(size_t i = 0; i < routingAdvertisements.size(); i++) {
        const RoutingAdvertisement& routingAdvertisement = routingAdvertisements[i];

        WriteBasicField(uint8_t(WSA_WRA), currentWriteBytes, wsaBuf);
        WriteBasicField(uint16_t(routingAdvertisement.lifeTime), currentWriteBytes, wsaBuf);
        WriteIpv6AddressField(routingAdvertisement.ipv6PrefixNetworkAddress, currentWriteBytes, wsaBuf);
        WriteBasicField(uint8_t(routingAdvertisement.prefixLength), currentWriteBytes, wsaBuf);
        WriteIpv6AddressField(routingAdvertisement.defaultGatewayNetworkAddress, currentWriteBytes, wsaBuf);
        WriteIpv6AddressField(routingAdvertisement.primaryDnsNetworkAddress, currentWriteBytes, wsaBuf);

        WriteIpv6AddressExtensionFieldIfNecessary(
            routingAdvertisement.enabledWaveElementExtensionBitSet,
            EXTENSION_SECONDARY_DNS, routingAdvertisement.secondaryDnsNetworkAddress,
            currentWriteBytes, wsaBuf);

        WriteMacAddressExtensionFieldIfNecessary(
            routingAdvertisement.enabledWaveElementExtensionBitSet,
            EXTENSION_GATEWAY_MAC_ADDRESS, routingAdvertisement.defaultGatewayMacAddress,
            currentWriteBytes, wsaBuf);
    }//for//

    unique_ptr<Packet> packetPtr = Packet::CreatePacket(*simEngineInterfacePtr, wsaBuf, static_cast<unsigned int>(currentWriteBytes));

    (*this).InsertPacketIntoQueue(
        packetPtr,
        NetworkAddress::broadcastAddress,
        wsaChannelNumberId,
        wsaPriority,
        static_cast<DatarateBitsPerSec>(cchInfo.datarate500Kbps) * 500000,
        static_cast<double>(cchInfo.txPowerDbm));

    (*this).RescheduleNextWsaTransmission();
}//PeriodicallyTransmitWsa//

inline
void WsmpLayer::InsertPacketIntoQueue(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& destinationAddress,
    const ChannelNumberIndexType& channelNumberId,
    const PacketPriority& priority,
    const DatarateBitsPerSec& datarateBps,
    const double txPowerDbm)
{

    OutputTraceAndStatsForInsertPacketIntoQueue(*packetPtr);

    EnqueueResultType enqueueResult;
    unique_ptr<Packet> packetToDropPtr;

    waveMacPtr->InsertPacektIntoCchOrSchQueueWhichSupportsChannelIdOf(
        channelNumberId,
        packetPtr,
        destinationAddress,
        priority,
        ETHERTYPE_WSMP,
        datarateBps,
        txPowerDbm,
        enqueueResult,
        packetToDropPtr);

    if (enqueueResult != ENQUEUE_SUCCESS) {

        OutputTraceAndStatsForFullQueueDrop(*packetToDropPtr, enqueueResult);

        packetToDropPtr = nullptr;
        packetPtr = nullptr;

    }//if//

    waveMacPtr->NetworkLayerQueueChangeNotification();

}//InsertPacketIntoQueue//

inline
void WsmpLayer::SendWsm(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& destinationAddress,
    const ChannelNumberIndexType& channelNumberId,
    const string& providerServiceId,
    const PacketPriority& priority)
{
    const ChannelInfo& channelInfo = channelInfos.at(channelNumberId);
    const uint64_t noWaveElementExtensionBitSet = 0;

    (*this).SendWsm(
        packetPtr,
        destinationAddress,
        channelNumberId,
        providerServiceId,
        priority,
        static_cast<DatarateBitsPerSec>(channelInfo.datarate500Kbps)*500000,
        static_cast<double>(channelInfo.txPowerDbm),
        noWaveElementExtensionBitSet);
}//SendWsm//

inline
void WsmpLayer::SendWsm(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& destinationAddress,
    const ChannelNumberIndexType& channelNumberId,
    const string& providerServiceId,
    const PacketPriority& priority,
    const DatarateBitsPerSec& datarateBps,
    const double txPowerDbm,
    const uint64_t enabledWaveElementExtensionBitSet)
{
    static const int maxHeaderSizeBytes = 8 + 3*3;
    static const uint8_t wsmpHeaderVersion = 2;

    unsigned char header[maxHeaderSizeBytes];
    size_t currentWriteBytes = 0;

    WriteBasicField(uint8_t(wsmpHeaderVersion), currentWriteBytes, header);
    WriteProviderServiceId(providerServiceId, currentWriteBytes, header);

    WriteBasicExtensionFieldIfNecessary(
        enabledWaveElementExtensionBitSet,
        EXTENSION_CHANNEL_NUMBER, uint8_t(channelNumberId),
        currentWriteBytes, header);

    WriteBasicExtensionFieldIfNecessary(
        enabledWaveElementExtensionBitSet,
        EXTENSION_DATARATE, uint8_t(datarateBps / 500000),
        currentWriteBytes, header);

    WriteBasicExtensionFieldIfNecessary(
        enabledWaveElementExtensionBitSet,
        EXTENSION_TX_POWER, uint8_t(txPowerDbm),
        currentWriteBytes, header);

    WriteBasicField(uint8_t(WSMP_WAVE_SHORT_MESSAGE), currentWriteBytes, header);
    WriteBasicField(uint16_t(packetPtr->LengthBytes()), currentWriteBytes, header);

    packetPtr->AddRawHeader(header, static_cast<unsigned int>(currentWriteBytes));

    (*this).InsertPacketIntoQueue(
            packetPtr,
            destinationAddress,
            channelNumberId,
            priority,
            datarateBps,
            txPowerDbm);
}//SendWsm//

inline
void WsmpLayer::ReceivePacketFromMac(
    unique_ptr<Packet>& packetPtr,
    const GenericMacAddress& peerMacAddress)
{
    assert(packetPtr->LengthBytes() > 0);

    OutputTraceAndStatsForReceivePacketFromMac(*packetPtr);

    const unsigned char version = packetPtr->GetRawPayloadData(0, 1)[0];

    if (version == 2) {

        (*this).ReceiveWsm(packetPtr, peerMacAddress);

    } else if ((version >> 2) == 1 && !isProvider) {

        (*this).ReceiveWsa(packetPtr, peerMacAddress);

    } else {
        cerr << "Received unexpected WAVE packet." << endl;
        packetPtr = nullptr;
    }//if//

}//ReceivePacketFromMac//

inline
void WsmpLayer::ReceiveWsm(
    unique_ptr<Packet>& packetPtr,
    const GenericMacAddress& peerMacAddress)
{
    const unsigned char* header = packetPtr->GetRawPayloadData();
    size_t currentReadBytes = 0;

    uint8_t headerVersion;
    string providerServiceId;
    uint8_t channelNumberId = 0;
    uint8_t datarate500KBps = 0;
    uint8_t txPowerDbm = 0;
    uint8_t wsmElementId = 0;
    uint16_t payloadLengthBytes = 0;
    uint64_t enabledWaveElementExtensionBitSet = 0;

    ReadBasicField(header, headerVersion, currentReadBytes);
    assert(headerVersion == 2);

    ReadProviderServiceId(header, providerServiceId, currentReadBytes);

    ReadBasicExtensionFieldIfNecessary(
        header,
        EXTENSION_CHANNEL_NUMBER,
        enabledWaveElementExtensionBitSet,
        channelNumberId,
        currentReadBytes);

    ReadBasicExtensionFieldIfNecessary(
        header,
        EXTENSION_DATARATE,
        enabledWaveElementExtensionBitSet,
        datarate500KBps,
        currentReadBytes);

    ReadBasicExtensionFieldIfNecessary(
        header,
        EXTENSION_TX_POWER,
        enabledWaveElementExtensionBitSet,
        txPowerDbm,
        currentReadBytes);

    ReadBasicField(header, wsmElementId, currentReadBytes);
    ReadBasicField(header, payloadLengthBytes, currentReadBytes);

    assert(wsmElementId == WSMP_WAVE_SHORT_MESSAGE);
    assert(packetPtr->LengthBytes() > payloadLengthBytes);
    packetPtr->DeleteHeader(packetPtr->LengthBytes() - payloadLengthBytes);

    if (wsmApplicationHandlerPtrs.find(providerServiceId) != wsmApplicationHandlerPtrs.end()) {
        wsmApplicationHandlerPtrs[providerServiceId]->ReceiveWsm(packetPtr);
    }
    else {
        packetPtr = nullptr;
    }//if//

}//ReceiveWsm//

inline
void WsmpLayer::ReceiveWsa(
    unique_ptr<Packet>& packetPtr,
    const GenericMacAddress& providerMacAddress)
{
    const unsigned char* payload = packetPtr->GetRawPayloadData();
    size_t currentReadBytes = 0;

    ReadBasicField(payload, currentWsaReceptionInfo.waveVersionAndChangeCount, currentReadBytes);

    // WSA header

    ReadBasicExtensionFieldIfNecessary(
        payload,
        EXTENSION_REPEAT_RATE,
        currentWsaReceptionInfo.enabledHeaderExtensionBitSet,
        currentWsaReceptionInfo.numberTransmissionPer5sec,
        currentReadBytes);

    ReadBasicExtensionFieldIfNecessary(
        payload,
        EXTENSION_TX_POWER,
        currentWsaReceptionInfo.enabledHeaderExtensionBitSet,
        currentWsaReceptionInfo.txPowerDbm,
        currentReadBytes);

    Read2dLocationExtensionFieldIfNecessary(
        payload,
        EXTENSION_2D_LOCATION,
        currentWsaReceptionInfo.enabledHeaderExtensionBitSet,
        currentWsaReceptionInfo.position,
        currentReadBytes);

    Read3dLocationExtensionFieldIfNecessary(
        payload,
        EXTENSION_3DLOCATION_AND_CONFIDENCE,
        currentWsaReceptionInfo.enabledHeaderExtensionBitSet,
        currentWsaReceptionInfo.position,
        currentWsaReceptionInfo.positionAndElevationConfidence,
        currentWsaReceptionInfo.positionAccuracy,
        currentReadBytes);

    ReadStringExtensionFieldIfNecessary(
        payload,
        EXTENSION_ADVERTISER_IDENTITIER,
        currentWsaReceptionInfo.enabledHeaderExtensionBitSet,
        currentWsaReceptionInfo.advertiserId,
        currentReadBytes);

    ReadStringExtensionFieldIfNecessary(
        payload,
        EXTENSION_COUNTRY_STRNIG,
        currentWsaReceptionInfo.enabledHeaderExtensionBitSet,
        currentWsaReceptionInfo.countryString,
        currentReadBytes);

    const size_t packetLengthBytes = packetPtr->LengthBytes();

    // service information

    serviceInfos.clear();
    while (currentReadBytes < packetLengthBytes &&
           PeekElementIdField(payload, currentReadBytes) == WSA_SERVICE_INFO) {

        serviceInfos.push_back(ServiceInfo());
        ServiceInfo& serviceInfo = serviceInfos.back();

        uint8_t serviceInfoElementId;
        ReadBasicField(payload, serviceInfoElementId, currentReadBytes);
        ReadProviderServiceId(payload, serviceInfo.providerServiceId, currentReadBytes);
        ReadBasicField(payload, serviceInfo.servicePriority, currentReadBytes);
        ReadBasicField(payload, serviceInfo.channelNumberId, currentReadBytes);

        ReadStringExtensionFieldIfNecessary(
            payload,
            EXTENSION_PROVIDER_SERVICE_CONTEXT,
            serviceInfo.enabledWaveElementExtensionBitSet,
            serviceInfo.providerServiceContext,
            currentReadBytes);

        ReadIpv6AddressExtensionFieldIfNecessary(
            payload,
            EXTENSION_IPV6_ADDRESS,
            serviceInfo.enabledWaveElementExtensionBitSet,
            serviceInfo.ipv6Address,
            currentReadBytes);

        ReadBasicExtensionFieldIfNecessary(
            payload,
            EXTENSION_SERVICE_PORT,
            serviceInfo.enabledWaveElementExtensionBitSet,
            serviceInfo.portNumber,
            currentReadBytes);

        ReadMacAddressExtensionFieldIfNecessary(
            payload,
            EXTENSION_PROVIDER_MAC_ADDRESS,
            serviceInfo.enabledWaveElementExtensionBitSet,
            serviceInfo.providerMacAddress,
            currentReadBytes);

        ReadBasicExtensionFieldIfNecessary(
            payload,
            EXTENSION_RCPI_THRESHOLD,
            serviceInfo.enabledWaveElementExtensionBitSet,
            serviceInfo.rcpiThreshold,
            currentReadBytes);

        ReadBasicExtensionFieldIfNecessary(
            payload,
            EXTENSION_WSA_COUNT_THRESHOLD,
            serviceInfo.enabledWaveElementExtensionBitSet,
            serviceInfo.wsaCountThreshold,
            currentReadBytes);

        ReadBasicExtensionFieldIfNecessary(
            payload,
            EXTENSION_WSA_COUNT_THRESHOLD_INTERVAL,
            serviceInfo.enabledWaveElementExtensionBitSet,
            serviceInfo.wsaCountThresholdInterval,
            currentReadBytes);
    }//while//

    // channel information
    while (currentReadBytes < packetLengthBytes &&
           PeekElementIdField(payload, currentReadBytes) == WSA_CHANNEL_INFO) {

        uint8_t channelInfoElementId;
        uint8_t operatingClass;
        uint8_t channelNumberId;

        ReadBasicField(payload, channelInfoElementId, currentReadBytes);
        ReadBasicField(payload, operatingClass, currentReadBytes);
        ReadBasicField(payload, channelNumberId, currentReadBytes);

        ChannelInfo& channelInfo = channelInfos.at(channelNumberId);

        channelInfo.operatingClass = operatingClass;
        ReadBasicField(payload, channelInfo.adaptable, currentReadBytes);
        ReadBasicField(payload, channelInfo.datarate500Kbps, currentReadBytes);
        ReadBasicField(payload, channelInfo.txPowerDbm, currentReadBytes);

        ReadEdcaExtensionFieldIfNecessary(
            payload,
            EXTENSION_EDCA_PARAMETERS_SET,
            channelInfo.enabledWaveElementExtensionBitSet,
            channelInfo.edcaParameters,
            currentReadBytes);

        ReadBasicExtensionFieldIfNecessary(
            payload,
            EXTENSION_CHANNEL_ACCESS,
            channelInfo.enabledWaveElementExtensionBitSet,
            channelInfo.channelAccess,
            currentReadBytes);
    }//while//

    // WAVE Routing Advertisement

    routingAdvertisements.clear();
    while (currentReadBytes < packetLengthBytes &&
           PeekElementIdField(payload, currentReadBytes) == WSA_WRA) {

        routingAdvertisements.push_back(RoutingAdvertisement());
        RoutingAdvertisement& routingAdvertisement = routingAdvertisements.back();

        uint8_t routingAdvertisementElementId;
        ReadBasicField(payload, routingAdvertisementElementId, currentReadBytes);
        ReadBasicField(payload, routingAdvertisement.lifeTime, currentReadBytes);
        ReadIpv6AddressField(payload, routingAdvertisement.ipv6PrefixNetworkAddress, currentReadBytes);
        ReadBasicField(payload, routingAdvertisement.prefixLength, currentReadBytes);
        ReadIpv6AddressField(payload, routingAdvertisement.defaultGatewayNetworkAddress, currentReadBytes);
        ReadIpv6AddressField(payload, routingAdvertisement.primaryDnsNetworkAddress, currentReadBytes);

        ReadIpv6AddressExtensionFieldIfNecessary(
            payload,
            EXTENSION_SECONDARY_DNS,
            routingAdvertisement.enabledWaveElementExtensionBitSet,
            routingAdvertisement.secondaryDnsNetworkAddress,
            currentReadBytes);

        ReadMacAddressExtensionFieldIfNecessary(
            payload,
            EXTENSION_GATEWAY_MAC_ADDRESS,
            routingAdvertisement.enabledWaveElementExtensionBitSet,
            routingAdvertisement.defaultGatewayMacAddress,
            currentReadBytes);
    }//while//
}//ReceiveWsa//

inline
void WsmpLayer::SetWsmApplicationHandler(
    const string& providerServiceId,
    const shared_ptr<WsmApplicationHandler>& wsmApplicationHandlerPtr)
{
    wsmApplicationHandlerPtrs[providerServiceId] = wsmApplicationHandlerPtr;
}//SetWsmApplicationHandler//


inline
void WsmpLayer::OutputTraceAndStatsForInsertPacketIntoQueue(const Packet& packet) const
{

    if (simEngineInterfacePtr->TraceIsOn(TraceNetwork)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            WsmpPacketTraceRecord traceData;
            const PacketId& thePacketId = packet.GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == WSMP_PACKET_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, "", "WsmpSend", traceData);

        }
        else {

            ostringstream outStream;
            const PacketId& thePacketId = packet.GetPacketId();
            outStream << "PktId= " << thePacketId;
            simEngineInterfacePtr->OutputTrace(modelName, "", "WsmpSend", outStream.str());

        }//if//
    }//if//

    packetsSentStatPtr->IncrementCounter();

    const size_t packetLengthBytes = packet.LengthBytes();
    bytesSentStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForInsertPacketIntoQueue//


inline
void WsmpLayer::OutputTraceAndStatsForReceivePacketFromMac(const Packet& packet) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceNetwork)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            WsmpPacketTraceRecord traceData;
            const PacketId& thePacketId = packet.GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == WSMP_PACKET_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, "", "WsmpRecv", traceData);

        }
        else {

            ostringstream outStream;
            const PacketId& thePacketId = packet.GetPacketId();
            outStream << "PktId= " << thePacketId;
            simEngineInterfacePtr->OutputTrace(modelName, "", "WsmpRecv", outStream.str());

        }//if//
    }//if//

    packetsReceivedStatPtr->IncrementCounter();

    const size_t packetLengthBytes = packet.LengthBytes();
    bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForReceivePacketFromMac//


inline
void WsmpLayer::OutputTraceAndStatsForFullQueueDrop(
    const Packet& packet, const EnqueueResultType enqueueResult) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceNetwork)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            WsmpFullQueueDropTraceRecord traceData;
            const PacketId& thePacketId = packet.GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();
            traceData.enqueueResult= ConvertToUShortInt(enqueueResult);

            assert(sizeof(traceData) == WSMP_FULL_QUEUE_DROP_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, "", "FullQueueDrop", traceData);

        }
        else {

            ostringstream outStream;
            outStream << "PktId= " << packet.GetPacketId();
            outStream << " Result= " << ConvertToEnqueueResultString(enqueueResult);

            simEngineInterfacePtr->OutputTrace(modelName, "", "FullQueueDrop", outStream.str());

        }//if//

    }//if//

    if (enqueueResult == ENQUEUE_FAILURE_BY_MAX_PACKETS) {
        packetMaxPacketsQueueDropsStatPtr->IncrementCounter();
    }
    else if (enqueueResult == ENQUEUE_FAILURE_BY_MAX_BYTES) {
        packetMaxBytesQueueDropsStatPtr->IncrementCounter();
    }//if//

}//OutputTraceAndStatsForFullQueueDrop//

} //namespace Wave//

#endif
