// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include <iomanip>

#include "scensim_netsim.h"
#include "tcp_porting.h"

namespace ScenSim {

using FreeBsd9Port::callout;
using FreeBsd9Port::cdg_cc_algo;
using FreeBsd9Port::chd_cc_algo;
using FreeBsd9Port::cubic_cc_algo;
using FreeBsd9Port::curvnet;
using FreeBsd9Port::hd_cc_algo;
using FreeBsd9Port::htcp_cc_algo;
using FreeBsd9Port::hz;
using FreeBsd9Port::inpcb;
#ifdef USE_IPV6
using FreeBsd9Port::ip6_hdr;
#else
using FreeBsd9Port::ip;
#endif
using FreeBsd9Port::m_get;
using FreeBsd9Port::mbuf;
using FreeBsd9Port::mbuf_porting;
using FreeBsd9Port::newreno_cc_algo;
using FreeBsd9Port::saved_vnet;
using FreeBsd9Port::sockaddr;
#ifdef USE_IPV6
using FreeBsd9Port::sockaddr_in6;
#else
using FreeBsd9Port::sockaddr_in;
#endif
using FreeBsd9Port::sockbuf;
using FreeBsd9Port::socket;
using FreeBsd9Port::ticks;
using FreeBsd9Port::time_uptime;
using FreeBsd9Port::tcphdr;
using FreeBsd9Port::tcp_destroy;
using FreeBsd9Port::tcp_init;
using FreeBsd9Port::tcpcb;
using FreeBsd9Port::tcpstat;
using FreeBsd9Port::vegas_cc_algo;
using FreeBsd9Port::vnet;


static void CheckIfErrorInFreeBsd9Port(
    int error,
    const char* funcName)
{
    if (error == 0) {
        return;
    }//if//

    cerr << "Error: TCP socket API(" << funcName << ") failed. (errno=" << error << ")" << endl;
    exit(1);

}//CheckIfErrorInFreeBsd9Port//


//--------------------------------------------------------------------------------------------------

class TcpProtocolImplementation {
public:
    ~TcpProtocolImplementation();

    int32_t GenerateRandomInt();

    void RegisterCallout(
        struct FreeBsd9Port::callout *c);

    void ScheduleCallout(
        struct FreeBsd9Port::callout *c);

    void CreatePacket(
        struct socket *so,
        unsigned int totalPayloadLength,
        Packet*& packetPtr,
        unsigned char*& rawPacketDataPtr);

    void AddVirtualFragmentData(
        ScenSim::Packet& packet,
        const unsigned int offset,
        const unsigned int length,
        const unsigned char* rawFragmentDataPtr);

    void GetVirtualFragmentData(
        const ScenSim::Packet& packet,
        const unsigned int dataBegin,
        const unsigned int dataEnd,
        unsigned int& length,
        const unsigned char*& rawFragmentDataPtr,
        unsigned int& nextOffset) const;

    void SendToNetworkLayer(
        const struct FreeBsd9Port::tcpcb *tp,
        const struct FreeBsd9Port::mbuf *m,
        unique_ptr<Packet>& packetPtr);

    void HandleNewConnection(
        struct FreeBsd9Port::socket *lso,
        struct FreeBsd9Port::socket **so);

    void NotifyIsConnected(
        struct FreeBsd9Port::socket *so);

    void NotifyBufferAvailable(
        struct FreeBsd9Port::socket *so);

    void NotifyDataArrival(
        struct FreeBsd9Port::socket *so);

    void NotifyIsFinReceived(
        struct FreeBsd9Port::socket *so);

    void NotifyIsSocketDisconnected(
        struct FreeBsd9Port::socket *so);

    void NotifyIsSocketClosing(
        struct FreeBsd9Port::socket *so);

    void UpdateRttForStatistics(int rtt);

    void UpdateCwndForStatistics(unsigned long int cwnd);

    void CountRetransmissionForStatistics();

    void OutputDebugLogForPacket(
        const struct FreeBsd9Port::tcpcb *tp,
        const struct FreeBsd9Port::mbuf *m,
        const unsigned char packetData[],
        const unsigned int packetDataLength,
        const bool isSent);

    void OutputDebugLogForReassembleQueue(
        const struct FreeBsd9Port::tcpcb *tp);

    void OutputDebugLogForAppendSockBuf(
        const struct FreeBsd9Port::sockbuf *sb,
        const struct FreeBsd9Port::mbuf *m);

    void OutputDebugLogForDropSockBuf(
        const struct FreeBsd9Port::sockbuf *sb,
        int len);

    void OutputDebugLogForStat(
        const struct FreeBsd9Port::tcpstat *stat);

private:
    friend class TcpProtocol;
    friend class TcpConnectionImplementation;

    TcpProtocolImplementation(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeId& initNodeId,
        const RandomNumberGeneratorSeed& nodeSeed);

    void InitializeOnDemand();

    void InitializeSettings();

    void ReadBool(
        const string& parameter, int& value)
    {
        if (theParameterDatabaseReader.ParameterExists(parameter, theNodeId)) {
            value = theParameterDatabaseReader.ReadBool(parameter, theNodeId) ? 1 : 0;
        }//if//

    }//ReadBool//

    void ReadInt(
        const string& parameter, int& value)
    {
        if (theParameterDatabaseReader.ParameterExists(parameter, theNodeId)) {
            value = theParameterDatabaseReader.ReadInt(parameter, theNodeId);
        }//if//

    }//ReadInt//

    void ReadUnsignedInt(
        const string& parameter, unsigned int& value)
    {
        if (theParameterDatabaseReader.ParameterExists(parameter, theNodeId)) {
            value = theParameterDatabaseReader.ReadNonNegativeInt(parameter, theNodeId);
        }//if//

    }//ReadUnsignedInt//

    void ReadString(
        const string& parameter, string& value)
    {
        if (theParameterDatabaseReader.ParameterExists(parameter, theNodeId)) {
            value = theParameterDatabaseReader.ReadString(parameter, theNodeId);
        }//if//

    }//ReadString//

    void ReadTime(
        const string& parameter, int& value)
    {
        if (theParameterDatabaseReader.ParameterExists(parameter, theNodeId)) {
            const SimTime time =
                theParameterDatabaseReader.ReadTime(parameter, theNodeId);
            value = time / (SECOND / FreeBsd9Port::hz);
        }//if//

    }//ReadTime//

    void SetTcpProtocolPtr(TcpProtocol* initTcpProtocolPtr);

    void ConnectToNetworkLayer(const shared_ptr<NetworkLayer>& initNetworkLayerPtr);

    void DisconnectFromOtherLayers();

    TcpConnection& GetTcpConnectionRef(const BsdTcpConnectionIdType& connectionId);
    const TcpConnection& GetTcpConnectionRef(const BsdTcpConnectionIdType& connectionId) const;
    shared_ptr<TcpConnection> GetTcpConnectionPtr(const BsdTcpConnectionIdType& connectionId) const;


    void CreateOutgoingTcpConnection(
        const NetworkAddress& localAddress,
        const unsigned short int localPort,
        const NetworkAddress& foreignAddress,
        const unsigned short int foreignPort,
        const PacketPriority& priority,
        const shared_ptr<TcpConnection::AppTcpEventHandler>& appEventHandlerPtr,
        shared_ptr<TcpConnection>& newTcpConnectionPtr);

    bool PortIsAvailable(const int portNumber) const;

    void OpenSpecificTcpPort(
        const NetworkAddress& localAddress,
        const unsigned short int localPort,
        const shared_ptr<ConnectionFromTcpProtocolHandler>& connectionHandlerPtr);

    void DisconnectConnectionHandlerForPort(
        const NetworkAddress& localAddress,
        const unsigned short int localPort);

    void ReceivePacketFromNetworkLayer(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const PacketPriority trafficClass,
        const NetworkAddress& lastHopAddress,
        const unsigned char hopLimit,
        const unsigned int interfaceIndex);

    void GetPortNumbersFromPacket(
        const Packet& aPacket,
        const unsigned int transportHeaderOffset,
        bool& portNumbersWereRetrieved,
        unsigned short int& sourcePort,
        unsigned short int& destinationPort) const;

    static void SetChecksum(Packet& aPacket)
    {
        if (aPacket.LengthBytes() != aPacket.ActualLengthBytes()) {
            return;
        }//if//

        const uint16_t *buffer = (uint16_t*)aPacket.GetRawPayloadData();
        unsigned int length = aPacket.LengthBytes();
        unsigned long sum = 0;

        while (length > 1) {
            sum += *buffer++;
            length -= sizeof(*buffer);
        }//while//

        if (length == 1) {
            sum += *reinterpret_cast<const uint8_t*>(buffer);
            length -= sizeof(uint8_t);
        }//if//

        assert(length == 0);
        sum  = (sum & 0xffff) + (sum >> 16);
        sum  = (sum & 0xffff) + (sum >> 16);

        struct tcphdr& th = aPacket.GetAndReinterpretPayloadData<struct tcphdr>();

        th.th_sum = ~sum;

    }//SetChecksum//

    class CalloutEvent : public SimulationEvent {
    public:
        CalloutEvent(
            TcpProtocolImplementation *initTcpProtocolImplementationPtr,
            struct FreeBsd9Port::callout *initCalloutPtr)
            :
            tcpProtocolImplementationPtr(initTcpProtocolImplementationPtr),
            calloutPtr(initCalloutPtr)
        {}

        void ExecuteEvent()
        {
            tcpProtocolImplementationPtr->ProcessCalloutEvent(calloutPtr);

        }//ExecuteEvent//

    private:
        TcpProtocolImplementation *tcpProtocolImplementationPtr;
        struct FreeBsd9Port::callout *calloutPtr;

    };//CalloutEvent//

    static const ExtrinsicPacketInfoId virtualFragmentDataInfoId;

    class VirtualFragmentData
        :
        public ExtrinsicPacketInformation,
        public enable_shared_from_this<VirtualFragmentData>
    {
    public:
        VirtualFragmentData() {}

        virtual shared_ptr<ExtrinsicPacketInformation> Clone()
        {
            return shared_from_this();

        }//Clone//

        void Add(
            const unsigned int offset,
            const unsigned int length,
            const unsigned char* rawFragmentDataPtr)
        {
            assert(data.find(offset) == data.end());
            data[offset].assign(rawFragmentDataPtr, rawFragmentDataPtr + length);

        }//Add//

        void Get(
            const unsigned int dataBegin,
            const unsigned int dataEnd,
            unsigned int& length,
            const unsigned char*& rawFragmentDataPtr,
            unsigned int& nextOffset) const
        {
            map<unsigned int, vector<unsigned char> >::const_iterator iter;
            iter = data.begin();

            while (iter != data.end()) {
                const unsigned int fragmentOffset = iter->first;
                const unsigned int fragmentSize = iter->second.size();
                if (dataBegin < fragmentOffset) {
                    length = 0;
                    rawFragmentDataPtr = nullptr;
                    if (fragmentOffset < dataEnd) {
                        nextOffset = fragmentOffset;
                    }
                    else {
                        nextOffset = dataEnd;
                    }//if//
                    return;
                }
                else if ((fragmentOffset <= dataBegin) && (dataBegin < (fragmentOffset + fragmentSize))) {
                    const unsigned int offset = dataBegin - fragmentOffset;
                    rawFragmentDataPtr = &iter->second[offset];
                    if (dataEnd < (fragmentOffset + fragmentSize)) {
                        length = dataEnd - (fragmentOffset + offset);
                        nextOffset = fragmentOffset + offset + length;
                    }
                    else {
                        length = fragmentSize - offset;
                        ++iter;
                        if ((iter == data.end()) || (dataEnd < iter->first)) {
                            nextOffset = dataEnd;
                        }
                        else {
                            nextOffset = iter->first;
                        }//if//
                    }//if//
                    return;
                }//if//
                ++iter;
            }//while//

            length = 0;
            rawFragmentDataPtr = nullptr;
            nextOffset = dataEnd;

        }//Get//

    private:
        map<unsigned int, vector<unsigned char> > data;

    };//VirtualFragmentData//

    void ProcessCalloutEvent(struct FreeBsd9Port::callout *calloutPtr);

    void BeginTcpProcess();
    void EndTcpProcess();

    static const int SEED_HASH = 23733567;

    const ParameterDatabaseReader& theParameterDatabaseReader;
    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    NodeId theNodeId;
    RandomNumberGenerator aRandomNumberGenerator;
    TcpProtocol *tcpProtocolPtr;
    shared_ptr<NetworkLayer> networkLayerPtr;

    struct FreeBsd9Port::vnet *vnet_porting;

    struct SocketAddress {
        NetworkAddress networkAddress;
        unsigned short int portNumber;

        SocketAddress(
            const NetworkAddress& initNetworkAddress,
            unsigned short int initPortNumber)
            :
            networkAddress(initNetworkAddress),
            portNumber(initPortNumber)
        {
        }//SocketAddress//

        bool operator<(const SocketAddress& obj) const
        {
            return ((this->portNumber < obj.portNumber) ||
                    ((this->portNumber == obj.portNumber) &&
                     (this->networkAddress < obj.networkAddress)));

        }//operator<//

    };//SocketAddress//

    struct ListeningSocket {
        shared_ptr<TcpConnection> tcpConnectionPtr;
        shared_ptr<ConnectionFromTcpProtocolHandler> connectionHandlerPtr;

        ListeningSocket(
            const shared_ptr<TcpConnection>& initTcpConnectionPtr,
            const shared_ptr<ConnectionFromTcpProtocolHandler>& initConnectionHandlerPtr)
            :
            tcpConnectionPtr(initTcpConnectionPtr),
            connectionHandlerPtr(initConnectionHandlerPtr)
        {
        }//ListeningSocket//

    };//ListeningSocket//

    // The Unique Connection ID used for connection access and memory management from BSD
    // C code.

    BsdTcpConnectionIdType currentConnectionId;
    map<BsdTcpConnectionIdType, shared_ptr<TcpConnection> > tcpConnectionMap;

    map<SocketAddress, ListeningSocket> mapOfListeningSocket;

    struct CalloutEntry {
        shared_ptr<CalloutEvent> calloutEventPtr;
        shared_ptr<EventRescheduleTicket> ticketPtr;

        CalloutEntry(
            const shared_ptr<CalloutEvent>& initCalloutEventPtr,
            const shared_ptr<EventRescheduleTicket>& initTicketPtr)
            :
            calloutEventPtr(initCalloutEventPtr),
            ticketPtr(initTicketPtr)
        {
        }//CalloutEntry//

    };//CalloutEntry//

    vector<CalloutEntry> vectorOfCallout;

    unsigned long int prevCwnd;

    shared_ptr<CounterStatistic> bytesReceivedFromUpperLayerStatPtr;
    shared_ptr<CounterStatistic> bytesSentToUpperLayerStatPtr;
    shared_ptr<CounterStatistic> bytesAckedStatPtr;

};//TcpProtocolImplementation//


//--------------------------------------------------------------------------------------------------


class TcpConnectionImplementation {
public:
    ~TcpConnectionImplementation();

private:
    friend class TcpConnection;
    friend class TcpProtocolImplementation;

    TcpConnectionImplementation(
        const shared_ptr<TcpProtocol>& initTcpProtocolPtr,
        const shared_ptr<TcpConnection::AppTcpEventHandler>& initAppEventHandlerPtr,
        const BsdTcpConnectionIdType& connectionId,
        const NetworkAddress& localAddress,
        const unsigned short int localPort,
        const NetworkAddress& foreignAddress,
        const unsigned short int foreignPort);

    bool IsConnected() const;

    void EnableVirtualPayload();

    void SetPacketPriority(const PacketPriority& priority);

    void SetAppTcpEventHandler(
        const shared_ptr<TcpConnection::AppTcpEventHandler>& newAppEventHandlerPtr);

    void ClearAppTcpEventHandler();

    void SendDataBlock(
        shared_ptr<vector<unsigned char> >& dataBlockPtr,
        const unsigned int dataLength);

    NetworkAddress GetForeignAddress() const;

    void Close();

    enum DelayKind {
        READY_TO_ACCEPT,
        READY_TO_SEND,
        READY_TO_RECEIVE,
        READY_TO_CLOSE_REMOTE
    };//DelayKind//

    class CallbackDelayEvent : public SimulationEvent {
    public:
        CallbackDelayEvent(
            const shared_ptr<TcpConnection>& initTcpConnectionPtr,
            const DelayKind& initDelayKind)
            :
            tcpConnectionPtr(initTcpConnectionPtr),
            delayKind(initDelayKind)
        {}

        void ExecuteEvent()
        {
            tcpConnectionPtr->implPtr->ProcessCallbackDelayEvent(delayKind);

        }//ExecuteEvent//

    private:
        shared_ptr<TcpConnection> tcpConnectionPtr;
        DelayKind delayKind;

    };//CallbackDelayEvent//


    void ProcessCallbackDelayEvent(const DelayKind& delayKind);
    void ReceiveDataBlock();

    shared_ptr<TcpProtocol> tcpProtocolPtr;
    shared_ptr<TcpConnection::AppTcpEventHandler> appEventHandlerPtr;

    // The Unique Connection ID used for connection access and memory management from BSD
    // C code.

    BsdTcpConnectionIdType connectionId;

    struct FreeBsd9Port::socket *socket_porting;
    bool useVirtualPayload;
    PacketPriority packetPriority;

    unsigned long long int currentNumReceivedBytes;
    unsigned long long int currentNumSentBytes;
    unsigned long long int currentNumDeliveredBytes;

    inline unsigned long long int GetNumberOfReceivedBytes() const
        { return currentNumReceivedBytes; }
    inline unsigned long long int GetNumberOfSentBytes() const
        { return currentNumSentBytes; }
    inline unsigned long long int GetNumberOfDeliveredBytes() const
        { return currentNumDeliveredBytes; }
    inline unsigned long long int GetCurrentNumberOfUnsentBufferedBytes() const
        { return socket_porting->so_snd.sb_cc; }
    inline unsigned long long int GetCurrentNumberOfAvailableBufferBytes() const
        { return sbspace(&socket_porting->so_snd); }

};//TcpConnectionImplementation//



//------------------------------------------------------------------------------
// TcpProtocol
//------------------------------------------------------------------------------


TcpProtocol::TcpProtocol(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeId& theNodeId,
    const RandomNumberGeneratorSeed& nodeSeed)
    :
    implPtr(
        new TcpProtocolImplementation(
            theParameterDatabaseReader,
            initSimulationEngineInterfacePtr,
            theNodeId,
            nodeSeed)),
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BytesSentToLowerLayer"))),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BytesReceivedFromLowerLayer"))),
    rttStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_Rtt"))),
    cwndStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_Cwnd"))),
    retransmissionStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_Retransmission")))
{
    implPtr->SetTcpProtocolPtr(this);

}//TcpProtocol//


TcpProtocol::~TcpProtocol()
{
}//~TcpProtocol//


void TcpProtocol::ConnectToNetworkLayer(
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr)
{
    implPtr->ConnectToNetworkLayer(initNetworkLayerPtr);

}//ConnectToNetworkLayer//


void TcpProtocol::DisconnectFromOtherLayers()
{
    implPtr->DisconnectFromOtherLayers();

}//DisconnectFromOtherLayers//


void TcpProtocol::CreateOutgoingTcpConnection(
    const NetworkAddress& localAddress,
    const unsigned short int localPort,
    const NetworkAddress& foreignAddress,
    const unsigned short int foreignPort,
    const PacketPriority& priority,
    const shared_ptr<TcpConnection::AppTcpEventHandler>& appEventHandlerPtr,
    shared_ptr<TcpConnection>& newTcpConnectionPtr)
{
    implPtr->CreateOutgoingTcpConnection(
        localAddress,
        localPort,
        foreignAddress,
        foreignPort,
        priority,
        appEventHandlerPtr,
        newTcpConnectionPtr);

}//CreateOutgoingTcpConnection//



bool TcpProtocol::PortIsAvailable(
    const int portNumber) const
{
    return implPtr->PortIsAvailable(portNumber);

}//PortIsAvailable//


void TcpProtocol::OpenSpecificTcpPort(
    const NetworkAddress& localAddress,
    const unsigned short int localPort,
    const shared_ptr<ConnectionFromTcpProtocolHandler>& connectionHandlerPtr)
{
    implPtr->OpenSpecificTcpPort(
        localAddress,
        localPort,
        connectionHandlerPtr);

}//OpenSpecificTcpPort//


void TcpProtocol::DisconnectConnectionHandlerForPort(
    const NetworkAddress& localAddress,
    const unsigned short int localPort)
{
    implPtr->DisconnectConnectionHandlerForPort(
        localAddress,
        localPort);

}//DisconnectConnectionHandlerForPort//


void TcpProtocol::ReceivePacketFromNetworkLayer(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress,
    const PacketPriority trafficClass,
    const NetworkAddress& lastHopAddress,
    const unsigned char hopLimit,
    const unsigned int interfaceIndex)
{
    implPtr->ReceivePacketFromNetworkLayer(
        packetPtr,
        sourceAddress,
        destinationAddress,
        trafficClass,
        lastHopAddress,
        hopLimit,
        interfaceIndex);

}//ReceivePacketFromNetworkLayer//


void TcpProtocol::GetPortNumbersFromPacket(
    const Packet& aPacket,
    const unsigned int transportHeaderOffset,
    bool& portNumbersWereRetrieved,
    unsigned short int& sourcePort,
    unsigned short int& destinationPort) const
{
    implPtr->GetPortNumbersFromPacket(
        aPacket,
        transportHeaderOffset,
        portNumbersWereRetrieved,
        sourcePort,
        destinationPort);

}//GetPortNumbersFromPacket//


//------------------------------------------------------------------------------
// TcpConnection
//------------------------------------------------------------------------------

TcpConnection::TcpConnection(
    const shared_ptr<TcpProtocol>& tcpProtocolPtr,
    const shared_ptr<TcpConnection::AppTcpEventHandler>& appEventHandlerPtr,
    const unsigned int& connectionId,
    const NetworkAddress& localAddress,
    const unsigned short int localPort,
    const NetworkAddress& foreignAddress,
    const unsigned short int foreignPort)
    :
    implPtr(
        new TcpConnectionImplementation(
            tcpProtocolPtr,
            appEventHandlerPtr,
            connectionId,
            localAddress,
            localPort,
            foreignAddress,
            foreignPort))
{
    // static assert on types possible in the future.

    assert(sizeof(connectionId) == sizeof(BsdTcpConnectionIdType));

}//TcpConnection//



TcpConnection::~TcpConnection()
{
}//~TcpConnection//



bool TcpConnection::IsConnected() const
{
    return implPtr->IsConnected();

}//IsConnected//


void TcpConnection::EnableVirtualPayload()
{
    implPtr->EnableVirtualPayload();

}//EnableVirtualPayload//


void TcpConnection::SetPacketPriority(
    const PacketPriority& priority)
{
    implPtr->SetPacketPriority(priority);

}//SetPacketPriority//



void TcpConnection::SetAppTcpEventHandler(
    const shared_ptr<TcpConnection::AppTcpEventHandler>& newAppEventHandlerPtr)
{
    implPtr->SetAppTcpEventHandler(newAppEventHandlerPtr);

}//SetAppTcpEventHandler//



void TcpConnection::ClearAppTcpEventHandler()
{
    implPtr->ClearAppTcpEventHandler();

}//ClearAppTcpEventHandler//



void TcpConnection::SendDataBlock(
    shared_ptr<vector<unsigned char> >& dataBlockPtr,
    const unsigned int dataLength)
{
    implPtr->SendDataBlock(dataBlockPtr, dataLength);

}//SendDataBlock//



unsigned long long int TcpConnection::GetNumberOfReceivedBytes() const
{
    return implPtr->GetNumberOfReceivedBytes();

}//GetNumberOfReceivedBytes//



unsigned long long int TcpConnection::GetNumberOfSentBytes() const
{
    return implPtr->GetNumberOfSentBytes();

}//GetNumberOfSentBytes//



unsigned long long int TcpConnection::GetNumberOfDeliveredBytes() const
{
    return implPtr->GetNumberOfDeliveredBytes();

}//GetNumberOfDeliveredBytes//



unsigned long long int TcpConnection::GetCurrentNumberOfUnsentBufferedBytes() const
{
    return implPtr->GetCurrentNumberOfUnsentBufferedBytes();

}//GetCurrentNumberOfUnsentBufferedBytes//



unsigned long long int TcpConnection::GetCurrentNumberOfAvailableBufferBytes() const
{
    return implPtr->GetCurrentNumberOfAvailableBufferBytes();

}//GetCurrentNumberOfAvailableBufferBytes//



NetworkAddress TcpConnection::GetForeignAddress() const
{
    return implPtr->GetForeignAddress();

}//GetForeignAddress//



void TcpConnection::Close()
{
    implPtr->Close();

}//Close//



//------------------------------------------------------------------------------
// TcpProtocolImplementation
//------------------------------------------------------------------------------

TcpProtocolImplementation::TcpProtocolImplementation(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeId& initNodeId,
    const RandomNumberGeneratorSeed& nodeSeed)
    :
    theParameterDatabaseReader(parameterDatabaseReader),
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    theNodeId(initNodeId),
    aRandomNumberGenerator(HashInputsToMakeSeed(nodeSeed, SEED_HASH)),
    currentConnectionId(0),
    vnet_porting(nullptr),
    prevCwnd(0)
{
}//TcpProtocolImplementation//



TcpProtocolImplementation::~TcpProtocolImplementation()
{
    if (vnet_porting == nullptr) {
        return;
    }//if//

    vnet_porting->ctx = nullptr;

    BeginTcpProcess();
    tcp_destroy();
    if (debug_tcpstat) {
        OutputDebugLogForStat(vnet_porting->tcpstat);
        delete vnet_porting->tcpstat;
    }//if//
    EndTcpProcess();

    delete vnet_porting;

}//~TcpProtocolImplementation//



void TcpProtocolImplementation::InitializeOnDemand()
{
    assert(vnet_porting == nullptr);
    vnet_porting = new vnet();
    assert(vnet_porting);

    vnet_porting->ctx = this;

    InitializeSettings();

    BeginTcpProcess();
    if (debug_tcpstat) {
        vnet_porting->tcpstat = new tcpstat();
    }//if//
    tcp_init();
    EndTcpProcess();

}//InitializeOnDemand//



void TcpProtocolImplementation::InitializeSettings()
{
    BeginTcpProcess();

    string ccModuleName = "newreno";
    ReadString("tcp-cc-module-name", ccModuleName);
    ConvertStringToLowerCase(ccModuleName);

    if (ccModuleName == "newreno") {
        V_default_cc_ptr = &newreno_cc_algo;
    }
    else if (ccModuleName == "cubic") {
        V_default_cc_ptr = &cubic_cc_algo;
    }
    else if (ccModuleName == "h-tcp" || ccModuleName == "htcp") {
        V_default_cc_ptr = &htcp_cc_algo;
        ReadBool("tcp-enable-cc-htcp-adaptive-backoff", V_htcp_adaptive_backoff);
        ReadBool("tcp-enable-cc-htcp-rtt-scaling", V_htcp_rtt_scaling);
    }
    else if (ccModuleName == "vegas") {
        V_default_cc_ptr = &vegas_cc_algo;
        ReadInt("tcp-cc-vegas-alpha", V_vegas_alpha);
        ReadInt("tcp-cc-vegas-beta", V_vegas_beta);
    }
    else if (ccModuleName == "hamilton-delay" || ccModuleName == "hd") {
        V_default_cc_ptr = &hd_cc_algo;
        ReadInt("tcp-cc-hd-qthresh", V_hd_qthresh);
        ReadInt("tcp-cc-hd-qmin", V_hd_qmin);
        ReadInt("tcp-cc-hd-pmax", V_hd_pmax);
    }
    else if (ccModuleName == "caia-hamilton-delay" || ccModuleName == "chd") {
        V_default_cc_ptr = &chd_cc_algo;
        ReadInt("tcp-cc-chd-qmin", V_chd_qmin);
        ReadInt("tcp-cc-chd-pmax", V_chd_pmax);
        ReadBool("tcp-enable-cc-chd-loss-fair", V_chd_loss_fair);
        ReadBool("tcp-enable-cc-chd-use-max", V_chd_use_max);
        ReadInt("tcp-cc-chd-qthresh", V_chd_qthresh);
    }
    else if (ccModuleName == "caia-delay-gradient" || ccModuleName == "cdg") {
        V_default_cc_ptr = &cdg_cc_algo;
        ReadInt("tcp-cc-cdg-wif", V_cdg_wif);
        ReadInt("tcp-cc-cdg-wdf", V_cdg_wdf);
        ReadInt("tcp-cc-cdg-loss-wdf", V_cdg_loss_wdf);
        ReadInt("tcp-cc-cdg-smoothing-factor", V_cdg_smoothing_factor);
        ReadInt("tcp-cc-cdg-exp-backoff-scale", V_cdg_exp_backoff_scale);
        ReadInt("tcp-cc-cdg-consec-cong", V_cdg_consec_cong);
        ReadInt("tcp-cc-cdg-hold-backoff", V_cdg_hold_backoff);
    }
    else {
        assert(false);
    }

    ReadUnsignedInt("tcp-hostcache-hash-size", V_tcp_hostcache.hashsize);
    ReadUnsignedInt("tcp-hostcache-bucket-limit", V_tcp_hostcache.bucket_limit);
    ReadBool("tcp-enable-blackhole", V_blackhole);
    ReadBool("tcp-enable-delayed-ack", V_tcp_delack_enabled);
    if (V_tcp_delack_enabled) {
        ReadTime("tcp-timer-delayed-ack-time", tcp_delacktime);
    }//if//
    ReadBool("tcp-enable-drop-synfin", V_drop_synfin);
    ReadBool("tcp-enable-rfc3042", V_tcp_do_rfc3042);
    ReadBool("tcp-enable-rfc3390", V_tcp_do_rfc3390);
    if (!V_tcp_do_rfc3390) {
        ReadInt("tcp-ss-flight-size-segments", V_ss_fltsz);
        ReadInt("tcp-ss-local-flight-size-segments", V_ss_fltsz_local);
    }//if//
    ReadBool("tcp-enable-rfc3465-abc", V_tcp_do_rfc3465);
    if (V_tcp_do_rfc3465) {
        ReadInt("tcp-rfc3465-abc-l-var", V_tcp_abc_l_var);
    }//if//
//TBD: ECN is not supported
//    ReadBool("tcp-enable-rfc3168-ecn", V_tcp_do_ecn);
//    ReadInt("tcp-rfc3168-ecn-max-retries", V_tcp_ecn_maxretries);
    ReadBool("tcp-enable-insecure-rst", V_tcp_insecure_rst);
    ReadBool("tcp-enable-auto-receive-buffer", V_tcp_do_autorcvbuf);
    if (V_tcp_do_autorcvbuf) {
        ReadInt("tcp-auto-receive-buffer-increment-bytes", V_tcp_autorcvbuf_inc);
        ReadInt("tcp-auto-receive-buffer-max-bytes", V_tcp_autorcvbuf_max);
    }//if//
//TBD: ICMP is not supported
//    ReadBool("tcp-enable-path-mtu-discovery", V_path_mtu_discovery);
    ReadBool("tcp-enable-auto-send-buffer", V_tcp_do_autosndbuf);
    if (V_tcp_do_autosndbuf) {
        ReadInt("tcp-auto-send-buffer-increment-bytes", V_tcp_autosndbuf_inc);
        ReadInt("tcp-auto-send-buffer-max-bytes", V_tcp_autosndbuf_max);
    }//if//
    ReadTime("tcp-timer-keep-init-time", tcp_keepinit);
    ReadBool("tcp-enable-keep-alive", always_keepalive);
    if (always_keepalive) {
        ReadTime("tcp-timer-keep-idle-time", tcp_keepidle);
        ReadTime("tcp-timer-keep-interval-time", tcp_keepintvl);
        ReadInt("tcp-timer-keep-count", tcp_keepcnt);
    }//if//
    ReadTime("tcp-timer-msl-time", tcp_msl);
    ReadTime("tcp-timer-retransmit-min-time", tcp_rexmit_min);
    ReadTime("tcp-timer-retransmit-slop-time", tcp_rexmit_slop);
    ReadBool("tcp-enable-timer-fast-finwait2-timeout", tcp_fast_finwait2_recycle);
    if (tcp_fast_finwait2_recycle) {
        ReadTime("tcp-timer-finwait2-timeout-time", tcp_finwait2_timeout);
    }//if//
    ReadTime("tcp-timer-max-persist-idle-time", tcp_maxpersistidle);
    ReadInt("tcp-reassemble-max-segments", V_tcp_reass_maxseg);
    ReadBool("tcp-enable-rfc2018-sack", V_tcp_do_sack);
    if (V_tcp_do_sack) {
        ReadInt("tcp-rfc2018-sack-max-holes", V_tcp_sack_maxholes);
        ReadInt("tcp-rfc2018-sack-global-max-holes", V_tcp_sack_globalmaxholes);
    }//if//
    ReadInt("tcp-max-timewait-count", maxtcptw);
    ReadInt("tcp-mss-bytes", V_tcp_mssdflt);
    ReadInt("tcp-v6-mss-bytes", V_tcp_v6mssdflt);
    ReadInt("tcp-min-mss-bytes", V_tcp_minmss);
    ReadBool("tcp-enable-rfc1323", V_tcp_do_rfc1323);
//TBD: ICMP is not supported
//    ReadBool("tcp-enable-icmp-may-rst", V_icmp_may_rst);
    ReadTime("tcp-isn-reseed-interval-time", V_tcp_isn_reseed_interval);
    ReadBool("tcp-enable-sc-syncookies", V_tcp_syncookies);
    if (V_tcp_syncookies) {
        ReadBool("tcp-enable-sc-syncookies-only", V_tcp_syncookiesonly);
    }//if//
    ReadUnsignedInt("tcp-syncache-hash-size", V_tcp_syncache.hashsize);
    ReadUnsignedInt("tcp-syncache-bucket-limit", V_tcp_syncache.bucket_limit);
    ReadBool("tcp-enable-sc-rst-sock-fail", V_tcp_sc_rst_sock_fail);
    ReadInt("tcp-send-buffer-bytes", tcp_sendspace);
    ReadInt("tcp-receive-buffer-bytes", tcp_recvspace);
    ReadInt("tcp-max-sockets", maxsockets);
    ReadInt("tcp-buffer-max-bytes", sb_max);
    ReadBool("tcp-enable-nagle", tcp_nagle);
    ReadBool("tcp-enable-options", tcp_options);
    ReadBool("tcp-enable-debug-iss-zero", debug_iss_zero);
    ReadBool("tcp-enable-debug-packet", debug_packet);
    ReadBool("tcp-enable-debug-packet-option", debug_packet_option);
    ReadBool("tcp-enable-debug-packet-window", debug_packet_window);
    ReadBool("tcp-enable-debug-packet-file", debug_packet_file);
    ReadBool("tcp-enable-debug-reassemble", debug_reass);
    ReadBool("tcp-enable-debug-reassemble-file", debug_reass_file);
    ReadBool("tcp-enable-debug-sockbuf", debug_sockbuf);
    ReadBool("tcp-enable-debug-sockbuf-file", debug_sockbuf_file);
    ReadBool("tcp-enable-debug-statistics", debug_tcpstat);

    EndTcpProcess();

}//InitializeSettings//



void TcpProtocolImplementation::SetTcpProtocolPtr(
    TcpProtocol* initTcpProtocolPtr)
{
    tcpProtocolPtr = initTcpProtocolPtr;

    bytesReceivedFromUpperLayerStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            (tcpProtocolPtr->modelName + "_BytesReceivedFromUpperLayer"));
    bytesSentToUpperLayerStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            (tcpProtocolPtr->modelName + "_BytesSentToUpperLayer"));
    bytesAckedStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            (tcpProtocolPtr->modelName + "_BytesAcked"));

}//SetTcpProtocolPtr//



void TcpProtocolImplementation::ConnectToNetworkLayer(
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr)
{
    assert(networkLayerPtr == nullptr);

    networkLayerPtr = initNetworkLayerPtr;
    networkLayerPtr->RegisterPacketHandlerForProtocol(
        IP_PROTOCOL_NUMBER_TCP, tcpProtocolPtr->shared_from_this());

}//ConnectToNetworkLayer//



void TcpProtocolImplementation::DisconnectFromOtherLayers()
{
    networkLayerPtr.reset();
    mapOfListeningSocket.clear();
    tcpConnectionMap.clear();

}//DisconnectFromOtherLayers//



TcpConnection& TcpProtocolImplementation::GetTcpConnectionRef(
    const BsdTcpConnectionIdType& connectionId)
{
    typedef map<BsdTcpConnectionIdType, shared_ptr<TcpConnection> >::iterator IterType;

    IterType iter = tcpConnectionMap.find(connectionId);

    assert(iter != tcpConnectionMap.end());

    return (*iter->second);

}//GetTcpConnectionRef//



const TcpConnection& TcpProtocolImplementation::GetTcpConnectionRef(
    const BsdTcpConnectionIdType& connectionId) const
{
    typedef map<BsdTcpConnectionIdType, shared_ptr<TcpConnection> >::const_iterator IterType;

    IterType iter = tcpConnectionMap.find(connectionId);

    assert(iter != tcpConnectionMap.end());

    return (*iter->second);

}//GetTcpConnectionRef//



shared_ptr<TcpConnection> TcpProtocolImplementation::GetTcpConnectionPtr(
    const BsdTcpConnectionIdType& connectionId) const
{
    typedef map<BsdTcpConnectionIdType, shared_ptr<TcpConnection> >::const_iterator IterType;

    IterType iter = tcpConnectionMap.find(connectionId);

    if (iter == tcpConnectionMap.end()) {
        return (shared_ptr<TcpConnection>(/*nullptr*/));
    }//if//

    return (iter->second);

}//GetTcpConnectionPtr//



void TcpProtocolImplementation::CreateOutgoingTcpConnection(
    const NetworkAddress& localAddress,
    const unsigned short int localPort,
    const NetworkAddress& foreignAddress,
    const unsigned short int foreignPort,
    const PacketPriority& priority,
    const shared_ptr<TcpConnection::AppTcpEventHandler>& appEventHandlerPtr,
    shared_ptr<TcpConnection>& newTcpConnectionPtr)
{
    (*this).currentConnectionId++;

    newTcpConnectionPtr = shared_ptr<TcpConnection>(
        new TcpConnection(
            tcpProtocolPtr->shared_from_this(),
            appEventHandlerPtr,
            currentConnectionId,
            localAddress, localPort,
            foreignAddress, foreignPort));

    tcpConnectionMap[currentConnectionId] = newTcpConnectionPtr;

    newTcpConnectionPtr->SetPacketPriority(priority);

#ifdef USE_IPV6
    struct sockaddr_in6 sin;
    sin.sin6_len = sizeof(sin);
    sin.sin6_family = AF_INET6;
    sin.sin6_port = htons(localPort);
    sin.sin6_flowinfo = 0;
    sin.sin6_scope_id = 0;

    const uint64_t localAddressHighBits = localAddress.GetRawAddressHighBits();
    const uint64_t localAddressLowBits = localAddress.GetRawAddressLowBits();
    ConvertTwoHost64ToNet128(localAddressHighBits, localAddressLowBits, sin.sin6_addr.s6_addr);
#else
    struct sockaddr_in sin;
    sin.sin_len = sizeof(sin);
    sin.sin_family = AF_INET;
    sin.sin_port = htons(localPort);
    sin.sin_addr.s_addr = htonl(localAddress.GetRawAddressLow32Bits());
#endif

    BeginTcpProcess();
    int error = sobind(
        newTcpConnectionPtr->implPtr->socket_porting, (struct sockaddr *)&sin);
    EndTcpProcess();
    CheckIfErrorInFreeBsd9Port(error, "sobind");

#ifdef USE_IPV6
    sin.sin6_len = sizeof(sin);
    sin.sin6_family = AF_INET6;
    sin.sin6_port = htons(foreignPort);
    sin.sin6_flowinfo = 0;
    sin.sin6_scope_id = 0;

    const uint64_t foreignAddressHighBits = foreignAddress.GetRawAddressHighBits();
    const uint64_t foreignAddressLowBits = foreignAddress.GetRawAddressLowBits();
    ConvertTwoHost64ToNet128(foreignAddressHighBits, foreignAddressLowBits, sin.sin6_addr.s6_addr);
#else
    sin.sin_len = sizeof(sin);
    sin.sin_family = AF_INET;
    sin.sin_port = htons(foreignPort);
    sin.sin_addr.s_addr = htonl(foreignAddress.GetRawAddressLow32Bits());
#endif

    BeginTcpProcess();
    error = soconnect(
        newTcpConnectionPtr->implPtr->socket_porting, (struct sockaddr *)&sin);
    EndTcpProcess();
    CheckIfErrorInFreeBsd9Port(error, "soconnect");

}//CreateOutgoingTcpConnection//



bool TcpProtocolImplementation::PortIsAvailable(
    const int portNumber) const
{
    map<SocketAddress, ListeningSocket>::const_iterator iter;

//TBD: a pair of networkAddress and portNumber should be used as a key of map
    for (
        iter = mapOfListeningSocket.begin();
        iter != mapOfListeningSocket.end();
        ++iter) {

        if (iter->first.portNumber == portNumber) {
            return false;
        }//if//

    }//for//

    return true;

}//PortIsAvailable//



void TcpProtocolImplementation::OpenSpecificTcpPort(
    const NetworkAddress& localAddress,
    const unsigned short int localPort,
    const shared_ptr<ConnectionFromTcpProtocolHandler>& connectionHandlerPtr)
{
    (*this).currentConnectionId++;

    shared_ptr<TcpConnection> newTcpConnectionPtr =
        shared_ptr<TcpConnection>(
            new TcpConnection(
                tcpProtocolPtr->shared_from_this(),
                shared_ptr<TcpConnection::AppTcpEventHandler>(),
                currentConnectionId,
                localAddress, localPort,
                NetworkAddress::invalidAddress, 0));

    tcpConnectionMap[currentConnectionId] = newTcpConnectionPtr;

    SocketAddress socketAddress(localAddress, localPort);
    ListeningSocket listeningSocket(newTcpConnectionPtr, connectionHandlerPtr);

    assert(PortIsAvailable(localPort));
    mapOfListeningSocket.insert(make_pair(socketAddress, listeningSocket));

#ifdef USE_IPV6
    struct sockaddr_in6 sin;
    sin.sin6_len = sizeof(sin);
    sin.sin6_family = AF_INET6;
    sin.sin6_port = htons(localPort);
    sin.sin6_flowinfo = 0;
    sin.sin6_scope_id = 0;

    uint64_t localAddressHighBits = localAddress.GetRawAddressHighBits();
    uint64_t localAddressLowBits = localAddress.GetRawAddressLowBits();
    ConvertTwoHost64ToNet128(localAddressHighBits, localAddressLowBits, sin.sin6_addr.s6_addr);
#else
    struct sockaddr_in sin;
    sin.sin_len = sizeof(sin);
    sin.sin_family = AF_INET;
    sin.sin_port = htons(localPort);
    sin.sin_addr.s_addr = htonl(localAddress.GetRawAddressLow32Bits());
#endif

    BeginTcpProcess();
    int error = sobind(
        newTcpConnectionPtr->implPtr->socket_porting, (struct sockaddr *)&sin);
    EndTcpProcess();
    CheckIfErrorInFreeBsd9Port(error, "sobind");

    BeginTcpProcess();
    error = solisten(newTcpConnectionPtr->implPtr->socket_porting, -1);
    EndTcpProcess();
    CheckIfErrorInFreeBsd9Port(error, "solisten");

}//OpenSpecificTcpPort//



void TcpProtocolImplementation::DisconnectConnectionHandlerForPort(
    const NetworkAddress& localAddress,
    const unsigned short int localPort)
{
    SocketAddress socketAddress(localAddress, localPort);
    mapOfListeningSocket.erase(socketAddress);

}//DisconnectConnectionHandlerForPort//



void TcpProtocolImplementation::ReceivePacketFromNetworkLayer(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress,
    const PacketPriority trafficClass,
    const NetworkAddress& lastHopAddress,
    const unsigned char hopLimit,
    const unsigned int interfaceIndex)
{
    tcpProtocolPtr->
        OutputTraceAndStatsForReceivePacketFromNetworkLayer(*packetPtr);

//TBD: IP header should not be passed to TCP layer
    const unsigned int len = packetPtr->LengthBytes();
#ifdef USE_IPV6
    const int off0 = sizeof(struct ip6_hdr);
#else
    const int off0 = sizeof(struct ip);
#endif

//TBD: set M_BCAST or M_MCAST
//     if link-level broadcast or multicast packet is received

    struct mbuf *m = m_get(new mbuf_porting(packetPtr.release(), -off0), MT_DATA);
    assert(m);
    m->m_len = len + off0;
    m->m_pkthdr.len = len + off0;

    BeginTcpProcess();
    if (debug_packet) {
        unsigned char notUsed[1];
        OutputDebugLogForPacket(NULL, m, notUsed, 0, false);
    }//if//
    tcp_input(m, off0);
    EndTcpProcess();

}//ReceivePacketFromNetworkLayer//



void TcpProtocolImplementation::GetPortNumbersFromPacket(
    const Packet& aPacket,
    const unsigned int transportHeaderOffset,
    bool& portNumbersWereRetrieved,
    unsigned short int& sourcePort,
    unsigned short int& destinationPort) const
{
    const struct tcphdr& th =
        aPacket.GetAndReinterpretPayloadData<struct tcphdr>(
            transportHeaderOffset);

    sourcePort = ntohs(th.th_sport);
    destinationPort = ntohs(th.th_dport);
    portNumbersWereRetrieved = true;

}//GetPortNumbersFromPacket//



void TcpProtocolImplementation::ProcessCalloutEvent(
    struct callout *c)
{
    vectorOfCallout[c->c_index].ticketPtr->Clear();
    c->c_flags &= ~CALLOUT_PENDING;

    BeginTcpProcess();
    (*c->c_func)(c->c_arg);
    EndTcpProcess();

}//ProcessCalloutEvent//



void TcpProtocolImplementation::BeginTcpProcess()
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
    ticks = currentTime / (SECOND / hz);
    time_uptime = ticks / hz + 1;

    assert(time_uptime > 0);

    if (vnet_porting == nullptr) {
        InitializeOnDemand();
    }//if//

    CURVNET_SET(vnet_porting);

}//BeginTcpProcess//



void TcpProtocolImplementation::EndTcpProcess()
{
    CURVNET_RESTORE();

}//EndTcpProcess//



//------------------------------------------------------------------------------
// Functions called by TCP
//------------------------------------------------------------------------------

int32_t TcpProtocolImplementation::GenerateRandomInt()
{
    return aRandomNumberGenerator.GenerateRandomInt(0, RANDOM_MAX);

}//GenerateRandomInt//



void TcpProtocolImplementation::RegisterCallout(
    struct callout *c)
{
    assert(c);

    shared_ptr<CalloutEvent> calloutEventPtr(new CalloutEvent(this, c));
    shared_ptr<EventRescheduleTicket> ticketPtr(new EventRescheduleTicket());

    vectorOfCallout.push_back(CalloutEntry(calloutEventPtr, ticketPtr));

    c->c_index = vectorOfCallout.size() - 1;

}//GetCallout//



void TcpProtocolImplementation::ScheduleCallout(
    struct callout *c)
{
    assert(c);

    shared_ptr<CalloutEvent>& calloutEventPtr =
        vectorOfCallout[c->c_index].calloutEventPtr;
    shared_ptr<EventRescheduleTicket>& ticketPtr =
        vectorOfCallout[c->c_index].ticketPtr;

    assert(c->c_time >= 0);

    if (c->c_time == 0) {
        if (ticketPtr->IsNull()) {
            assert(callout_pending(c) == 0);
        }
        else {
            assert(callout_active(c) != 0);
            assert(callout_pending(c) != 0);
            simulationEngineInterfacePtr->CancelEvent(*ticketPtr);
        }//if//
        c->c_flags &= ~(CALLOUT_ACTIVE|CALLOUT_PENDING);
    }
    else {
        const SimTime currentTime =
            simulationEngineInterfacePtr->CurrentTime();
        const SimTime intervalTime = c->c_time * (SECOND / hz);
        const SimTime wakeupTime = currentTime + intervalTime;
        if (ticketPtr->IsNull()) {
            assert(callout_pending(c) == 0);
            simulationEngineInterfacePtr->
                ScheduleEvent(calloutEventPtr, wakeupTime, *ticketPtr);
        }
        else {
            assert(callout_active(c) != 0);
            assert(callout_pending(c) != 0);
            simulationEngineInterfacePtr->
                RescheduleEvent(*ticketPtr, wakeupTime);
        }//if//
        c->c_flags |= (CALLOUT_ACTIVE|CALLOUT_PENDING);
    }//if//

}//ScheduleCallout//



void TcpProtocolImplementation::CreatePacket(
    struct socket *so,
    unsigned int totalPayloadLength,
    Packet*& packetPtr,
    unsigned char*& rawPacketDataPtr)
{
    bool useVirtualPayload = false;

    if ((so != nullptr) && (so->so_ctx_id != InvalidBsdTcpConnectionId)) {
        useVirtualPayload = GetTcpConnectionRef(so->so_ctx_id).implPtr->useVirtualPayload;
    }//if//

    unique_ptr<Packet> tempPacketPtr =
        Packet::CreatePacket(*simulationEngineInterfacePtr, string(""), totalPayloadLength, useVirtualPayload);

    packetPtr = tempPacketPtr.release();

    rawPacketDataPtr = nullptr;

    if ((totalPayloadLength != 0) && (packetPtr->LengthBytes() == packetPtr->ActualLengthBytes())) {
        rawPacketDataPtr = packetPtr->GetRawPayloadData();
    }//if//

}//CreatePacket//


const ExtrinsicPacketInfoId TcpProtocolImplementation::virtualFragmentDataInfoId = "VirtualFragmentData";


void TcpProtocolImplementation::AddVirtualFragmentData(
    ScenSim::Packet& packet,
    const unsigned int offset,
    const unsigned int length,
    const unsigned char* rawFragmentDataPtr)
{
    if (!packet.CheckExtrinsicPacketInformationExist(virtualFragmentDataInfoId)) {
        shared_ptr<ExtrinsicPacketInformation> newVirtualFlagmentDataPtr =
            make_shared<VirtualFragmentData>();
        packet.AddExtrinsicPacketInformation(virtualFragmentDataInfoId, newVirtualFlagmentDataPtr);
    }//if//

    VirtualFragmentData& virtualFragmentData =
        packet.GetExtrinsicPacketInformation<VirtualFragmentData>(virtualFragmentDataInfoId);
    virtualFragmentData.Add(offset, length, rawFragmentDataPtr);

}//AddVirtualFragmentData//



void TcpProtocolImplementation::GetVirtualFragmentData(
    const ScenSim::Packet& packet,
    const unsigned int dataBegin,
    const unsigned int dataEnd,
    unsigned int& length,
    const unsigned char*& rawFragmentDataPtr,
    unsigned int& nextOffset) const
{
    if (!packet.CheckExtrinsicPacketInformationExist(virtualFragmentDataInfoId)) {
        length = 0;
        rawFragmentDataPtr = nullptr;
        nextOffset = dataEnd;
    }
    else {
        VirtualFragmentData& virtualFragmentData =
            packet.GetExtrinsicPacketInformation<VirtualFragmentData>(virtualFragmentDataInfoId);
        virtualFragmentData.Get(dataBegin, dataEnd, length, rawFragmentDataPtr, nextOffset);
    }//if//

}//GetVirtualFragmentData//



void TcpProtocolImplementation::SendToNetworkLayer(
    const struct tcpcb *tp,
    const struct mbuf *m,
    unique_ptr<Packet>& packetPtr)
{
    assert(m);

    PacketPriority packetPriority = 0;

    if (tp) {
        struct socket *so = tp->t_inpcb->inp_socket;
        if ((so->so_ctx_id == InvalidBsdTcpConnectionId) ||
            (GetTcpConnectionRef(so->so_ctx_id).implPtr->socket_porting == nullptr)) {

            packetPtr = nullptr;
            return;
        }//if//
        packetPriority = GetTcpConnectionRef(so->so_ctx_id).implPtr->packetPriority;
    }//if//

    if (debug_packet) {
        OutputDebugLogForPacket(tp, m, packetPtr->GetRawPayloadData(), packetPtr->LengthBytes(), true);
    }//if//

    const int tcpPayloadLength = packetPtr->LengthBytes();

#ifdef USE_IPV6
    packetPtr->AddRawHeader(
        (const unsigned char *)(m->m_data + sizeof(struct ip6_hdr)),
        m->m_len - sizeof(struct ip6_hdr));
#else
    packetPtr->AddRawHeader(
        (const unsigned char *)(m->m_data + sizeof(struct ip)),
        m->m_len - sizeof(struct ip));
#endif

    SetChecksum(*packetPtr);

    if (tcpPayloadLength == 0) {
        tcpProtocolPtr->
            OutputTraceAndStatsForSendControlPacketToNetworkLayer(*packetPtr);
    }
    else {
        tcpProtocolPtr->
            OutputTraceAndStatsForSendDataPacketToNetworkLayer(*packetPtr);
    }//if//

#ifdef USE_IPV6
    const struct ip6_hdr *ip = mtod(m, struct ip6_hdr *);

    uint64_t sourceAddressHighBits;
    uint64_t sourceAddressLowBits;
    ConvertNet128ToTwoHost64(ip->ip6_src.s6_addr, sourceAddressHighBits, sourceAddressLowBits);
    const NetworkAddress sourceAddress(sourceAddressHighBits, sourceAddressLowBits);

    uint64_t destinationAddressHighBits;
    uint64_t destinationAddressLowBits;
    ConvertNet128ToTwoHost64(ip->ip6_dst.s6_addr, destinationAddressHighBits, destinationAddressLowBits);
    const NetworkAddress destinationAddress(destinationAddressHighBits, destinationAddressLowBits);
#else
    const struct ip *ip = mtod(m, struct ip *);

    const NetworkAddress sourceAddress(ntohl(ip->ip_src.s_addr));
    const NetworkAddress destinationAddress(ntohl(ip->ip_dst.s_addr));
#endif

    networkLayerPtr->ReceivePacketFromUpperLayer(
        packetPtr,
        sourceAddress,
        destinationAddress,
        packetPriority,
        IP_PROTOCOL_NUMBER_TCP);

}//SendToNetworkLayer//



void TcpProtocolImplementation::HandleNewConnection(
    struct socket *lso,
    struct socket **so)
{
    assert(lso);
    assert(so);

    if (lso->so_ctx_id == InvalidBsdTcpConnectionId) {
        return;
    }//if//

    shared_ptr<TcpConnection> tcpConnectionPtr = GetTcpConnectionPtr(lso->so_ctx_id);

    assert(tcpConnectionPtr->implPtr->socket_porting != nullptr); //add logic to return, if this asserts.

    (*this).currentConnectionId++;

    shared_ptr<TcpConnection> newTcpConnectionPtr(
        new TcpConnection(
            tcpProtocolPtr->shared_from_this(),
            shared_ptr<TcpConnection::AppTcpEventHandler>(),
            currentConnectionId,
            NetworkAddress::invalidAddress, 0,
            NetworkAddress::invalidAddress, 0));

    tcpConnectionMap[currentConnectionId] = newTcpConnectionPtr;

    *so = newTcpConnectionPtr->implPtr->socket_porting;

    const struct inpcb *inp = (const struct inpcb *)lso->so_pcb;
#ifdef USE_IPV6
    uint64_t localAddressHighBits;
    uint64_t localAddressLowBits;
    ConvertNet128ToTwoHost64(inp->in6p_laddr.s6_addr, localAddressHighBits, localAddressLowBits);
    const NetworkAddress localAddress(localAddressHighBits, localAddressLowBits);
#else
    const NetworkAddress localAddress(ntohl(inp->inp_laddr.s_addr));
#endif
    const unsigned short int localPort(ntohs(inp->inp_lport));
    const SocketAddress socketAddress(localAddress, localPort);

    map<SocketAddress, ListeningSocket>::const_iterator iter;
    iter = mapOfListeningSocket.find(socketAddress);
    assert(iter != mapOfListeningSocket.end());

    assert(iter->second.connectionHandlerPtr.get());
    iter->second.connectionHandlerPtr->
        HandleNewConnection(newTcpConnectionPtr);

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    simulationEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(
            new TcpConnectionImplementation::CallbackDelayEvent(
                tcpConnectionPtr,
                TcpConnectionImplementation::READY_TO_ACCEPT)),
        currentTime);

}//HandleNewConnection//



void TcpProtocolImplementation::NotifyIsConnected(
    struct socket *so)
{
    NotifyBufferAvailable(so);

}//NotifyIsConnected//



void TcpProtocolImplementation::NotifyBufferAvailable(
    struct socket *so)
{
    assert(so);

    if (so->so_ctx_id == InvalidBsdTcpConnectionId) {
        return;
    }//if//

    shared_ptr<TcpConnection> tcpConnectionPtr = GetTcpConnectionPtr(so->so_ctx_id);

    assert(tcpConnectionPtr->implPtr->socket_porting != nullptr); //add logic to return, if this asserts.

    tcpConnectionPtr->implPtr->currentNumDeliveredBytes += so->so_acked;
    bytesAckedStatPtr->IncrementCounter(so->so_acked);
    so->so_acked = 0;

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    simulationEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(
            new TcpConnectionImplementation::CallbackDelayEvent(
                tcpConnectionPtr,
                TcpConnectionImplementation::READY_TO_SEND)),
        currentTime);

}//NotifyBufferAvailable//



void TcpProtocolImplementation::NotifyDataArrival(
    struct socket *so)
{
    assert(so);

    if (so->so_ctx_id == InvalidBsdTcpConnectionId) {
        return;
    }//if//

    shared_ptr<TcpConnection> tcpConnectionPtr = GetTcpConnectionPtr(so->so_ctx_id);

    assert(tcpConnectionPtr->implPtr->socket_porting != nullptr); //add logic to return, if this asserts.

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    simulationEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(
            new TcpConnectionImplementation::CallbackDelayEvent(
                tcpConnectionPtr,
                TcpConnectionImplementation::READY_TO_RECEIVE)),
        currentTime);

}//NotifyDataArrival//



void TcpProtocolImplementation::NotifyIsFinReceived(
    struct socket *so)
{
    assert(so);

    if (so->so_ctx_id == InvalidBsdTcpConnectionId) {
        return;
    }//if//

    shared_ptr<TcpConnection> tcpConnectionPtr = GetTcpConnectionPtr(so->so_ctx_id);

    assert(tcpConnectionPtr->implPtr->socket_porting != nullptr); //add logic to return, if this asserts.

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    simulationEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(
            new TcpConnectionImplementation::CallbackDelayEvent(
                tcpConnectionPtr,
                TcpConnectionImplementation::READY_TO_CLOSE_REMOTE)),
        currentTime);

}//NotifyIsFinReceived//



void TcpProtocolImplementation::NotifyIsSocketDisconnected(
    struct socket *so)
{
    assert(so);

    if (so->so_ctx_id == InvalidBsdTcpConnectionId) {
        return;
    }//if//

    shared_ptr<TcpConnection> tcpConnectionPtr = GetTcpConnectionPtr(so->so_ctx_id);

    assert(tcpConnectionPtr->implPtr->socket_porting != nullptr); //add logic to return, if this asserts.

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    simulationEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(
            new TcpConnectionImplementation::CallbackDelayEvent(
                tcpConnectionPtr,
                TcpConnectionImplementation::READY_TO_CLOSE_REMOTE)),
        currentTime);

}//NotifyIsSocketDisconnected//



void TcpProtocolImplementation::NotifyIsSocketClosing(struct socket *so)
{
    assert(so);

    if (so->so_ctx_id == InvalidBsdTcpConnectionId) {
        return;
    }//if//

    const BsdTcpConnectionIdType tcpConnectionId = so->so_ctx_id;

    shared_ptr<TcpConnection> tcpConnectionPtr = GetTcpConnectionPtr(tcpConnectionId);

    assert(tcpConnectionPtr->implPtr->socket_porting != nullptr); //add logic to return, if this asserts.

    assert(tcpConnectionPtr->implPtr->socket_porting == so);

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (tcpConnectionPtr->implPtr->appEventHandlerPtr.get()) {
        tcpConnectionPtr->implPtr->appEventHandlerPtr->DoTcpLocalHostClosedAction();
    }//if//

    if (tcpConnectionPtr->implPtr->socket_porting != nullptr) {
        tcpConnectionPtr->implPtr->socket_porting->so_ctx_id = InvalidBsdTcpConnectionId;
        tcpConnectionPtr->implPtr->socket_porting = nullptr;
    }//if//

    tcpConnectionMap.erase(tcpConnectionId);

}//NotifyIsSocketClosing//



void TcpProtocolImplementation::UpdateRttForStatistics(int rtt)
{
    const SimTime roundTripTime = rtt * (SECOND / hz);

    tcpProtocolPtr->rttStatPtr->
        RecordStatValue(ConvertTimeToDoubleSecs(roundTripTime));

}//UpdateRttForStatistics//



void TcpProtocolImplementation::UpdateCwndForStatistics(unsigned long int cwnd)
{
    if (prevCwnd != cwnd) {
        tcpProtocolPtr->cwndStatPtr->RecordStatValue(cwnd);
        prevCwnd = cwnd;
    }//if//

}//UpdateCwndForStatistics//



void TcpProtocolImplementation::CountRetransmissionForStatistics()
{
    tcpProtocolPtr->retransmissionStatPtr->IncrementCounter();

}//CountRetransmissionForStatistics//



//------------------------------------------------------------------------------
// Functions for debug
//------------------------------------------------------------------------------

using FreeBsd9Port::bcopy;
using FreeBsd9Port::min;
using FreeBsd9Port::sackblk;
using FreeBsd9Port::u_int8_t;
using FreeBsd9Port::u_int16_t;
using FreeBsd9Port::u_int32_t;

static void OutputDebugLogForOption(const struct tcphdr *th)
{
    const unsigned char *head = (const unsigned char *)(th + 1);
    int rest = (th->th_off << 2) - sizeof(struct tcphdr);

    while (rest > 0) {
        unsigned char opt = head[0];
        int len;

        if (opt == TCPOPT_EOL) {
            break;
        }//if//

        if (opt == TCPOPT_NOP) {
            len = 1;
        }
        else {
            if (rest < 2) {
                break;
            }//if//
            len = head[1];
            if (len < 2 || len > rest) {
                break;
            }//if//
        }//if//

        switch (opt) {
        case TCPOPT_NOP:
            break;
        case TCPOPT_MAXSEG:
        {
            assert(len == TCPOLEN_MAXSEG);

            u_int16_t mss;
            bcopy((char *)head + 2, (char *)&mss, sizeof(mss));
            mss = ntohs(mss);
            std::cout << " MSS=" << mss;

            break;
        }
        case TCPOPT_WINDOW:
        {
            assert(len == TCPOLEN_WINDOW);

            const u_int8_t wscale = min(head[2], TCP_MAX_WINSHIFT);
            std::cout << " WS=" << (int)wscale;

            break;
        }
        case TCPOPT_TIMESTAMP:
        {
            assert(len == TCPOLEN_TIMESTAMP);

            u_int32_t tsval;
            bcopy((char *)head + 2, (char *)&tsval, sizeof(tsval));
            tsval = ntohl(tsval);
            std::cout << " TSV=" << tsval;

            u_int32_t tsecr;
            bcopy((char *)head + 6, (char *)&tsecr, sizeof(tsecr));
            tsecr = ntohl(tsecr);
            std::cout << " TSE=" << tsecr;

            break;
        }
        case TCPOPT_SACK_PERMITTED:
        {
            assert(len == TCPOLEN_SACK_PERMITTED);

            std::cout << " SACK";

            break;
        }
        case TCPOPT_SACK:
        {
            assert(len > 2);
            assert((len - 2) % TCPOLEN_SACK == 0);

            const u_int8_t nsacks = (len - 2) / TCPOLEN_SACK;
            const struct sackblk *sacks = (const struct sackblk *)(head + 2);

            std::cout << " SACK=";
            for (int i = 0; i < nsacks; ++i) {
                std::cout << " " << sacks[i].start;
                std::cout << "-" << sacks[i].end;
            }//for//

            break;
        }
        default:
            assert(false);
        }//switch//

        rest -= len;
        head += len;

    }//while//

}//OutputDebugLogForOptions//



void TcpProtocolImplementation::OutputDebugLogForPacket(
    const struct tcpcb *tp,
    const struct mbuf *m,
    const unsigned char packetData[],
    const unsigned int packetDataLength,
    const bool isSent)
{
    assert(m);

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

#ifdef USE_IPV6
    const struct ip6_hdr *ip = (struct ip6_hdr *)m->m_data;

    uint64_t sourceAddressHighBits;
    uint64_t sourceAddressLowBits;
    ConvertNet128ToTwoHost64(ip->ip6_src.s6_addr, sourceAddressHighBits, sourceAddressLowBits);
    const NetworkAddress srcAddress(sourceAddressHighBits, sourceAddressLowBits);

    uint64_t destinationAddressHighBits;
    uint64_t destinationAddressLowBits;
    ConvertNet128ToTwoHost64(ip->ip6_dst.s6_addr, destinationAddressHighBits, destinationAddressLowBits);
    const NetworkAddress dstAddress(destinationAddressHighBits, destinationAddressLowBits);
#else
    const struct ip *ip = (struct ip *)m->m_data;
    const NetworkAddress srcAddress(ntohl(ip->ip_src.s_addr));
    const NetworkAddress dstAddress(ntohl(ip->ip_dst.s_addr));
#endif

    const struct tcphdr *th = (struct tcphdr *)(ip + 1);
    const int srcPort = ntohs(th->th_sport);
    const int dstPort = ntohs(th->th_dport);

    string header, id;
    const char *data;
    int len;

    if (isSent) {
        header = " [SEND]      ";
        id = "out";
        data = (const char *)&packetData[0];
        len = packetDataLength;
    }
    else {
        header = "       [RECV]";
        id = "in";
#ifdef USE_IPV6
        const int off0 = sizeof(struct ip6_hdr);
#else
        const int off0 = sizeof(struct ip);
#endif
        const int off = th->th_off << 2;
        data = (const char *)(m->m_data + off0 + off);
        len = m->m_len - off0 - off;
    }

    std::cout << ConvertTimeToStringSecs(currentTime);
    std::cout << header;
    std::cout << " " << std::setw(15) << srcAddress.ConvertToString();
    std::cout << ":" << std::setw(5) << srcPort;
    std::cout << " " << std::setw(15) << dstAddress.ConvertToString();
    std::cout << ":" << std::setw(5) << dstPort;
    std::cout << " seq=" << std::setw(10) << ntohl(th->th_seq);
    std::cout << " ack=" << std::setw(10) << ntohl(th->th_ack);
    std::cout << " win=" << std::setw(10) << ntohs(th->th_win);
    std::cout << " payload=" << std::setw(4) << len;
    std::cout << ((th->th_flags & TH_FIN) != 0 ? " F": "  ");
    std::cout << ((th->th_flags & TH_SYN) != 0 ? "S": " ");
    std::cout << ((th->th_flags & TH_RST) != 0 ? "R": " ");
    std::cout << ((th->th_flags & TH_PUSH) != 0 ? "P": " ");
    std::cout << ((th->th_flags & TH_ACK) != 0 ? "A": " ");
    std::cout << ((th->th_flags & TH_URG) != 0 ? "U": " ");
    std::cout << ((th->th_flags & TH_ECE) != 0 ? "E": " ");
    std::cout << ((th->th_flags & TH_CWR) != 0 ? "C": " ");
    if (debug_packet_option) {
        OutputDebugLogForOption(th);
    }//if//
    if (debug_packet_window) {
        if (tp) {
            std::cout << " snd_(una,nxt,max)=(" << tp->snd_una;
            std::cout << "," << tp->snd_nxt;
            std::cout << "," << tp->snd_max << ")";
            std::cout << " rcv_(nxt,adv,wnd)=(" << tp->rcv_nxt;
            std::cout << "," << tp->rcv_adv;
            std::cout << "," << tp->rcv_wnd << ")";
        }//if//
    }//if//
    std::cout << std::endl;

    if (debug_packet_file) {
        if (len > 0) {
            std::ostringstream fname;
            fname << "stream_" << id << "_" << theNodeId << "_";
            fname << srcAddress.ConvertToString() << "_" << srcPort << "_";
            fname << dstAddress.ConvertToString() << "_" << dstPort;
            fname << ".bin";
            std::ofstream ofs;
            ofs.open(fname.str().c_str(), std::ios::binary|std::ios::app);
            ofs.write(data, len);
        }//if//
    }//if//

}//OutputDebugLogForPacket//


using FreeBsd9Port::tseg_qent;

void TcpProtocolImplementation::OutputDebugLogForReassembleQueue(
    const struct tcpcb *tp)
{
    assert(tp);

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    struct tseg_qent *q;

    std::cout << ConvertTimeToStringSecs(currentTime);

    std::cout << " [REASS] tp=" << tp;
    LIST_FOREACH(q, &tp->t_segq, tqe_q) {
        std::cout << " " << q->tqe_th->th_seq;
        std::cout << "-" << q->tqe_th->th_seq + q->tqe_len;
    }//LIST_FOREACH//
    std::cout << std::endl;

    if (debug_reass_file) {
        if (!LIST_EMPTY(&tp->t_segq)) {
            std::ostringstream fname;
            fname << "reass_" << theNodeId << "_" << tp << ".bin";
            std::ofstream ofs;
            ofs.open(fname.str().c_str(), std::ios::binary|std::ios::trunc);
            LIST_FOREACH(q, &tp->t_segq, tqe_q) {
                ofs.write((const char *)q->tqe_m->m_data, q->tqe_len);
            }//LIST_FOREACH//
        }//if//
    }//if//

}//OutputDebugLogForReassembleQueue//



void TcpProtocolImplementation::OutputDebugLogForAppendSockBuf(
    const struct sockbuf *sb,
    const struct mbuf *m)
{
    assert(sb);
    assert(m);

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    std::cout << ConvertTimeToStringSecs(currentTime);
    std::cout << " [SB][ADD]      sockbuf=" << sb;
    std::cout << " len=" << sb->sb_cc + m->m_len << "(+" << m->m_len << ")";
    std::cout << std::endl;

    if (debug_sockbuf_file) {
        if (m->m_len > 0) {
            std::ostringstream fname;
            fname << "sb_append_" << theNodeId << "_" << sb << ".bin";
            std::ofstream ofs;
            ofs.open(fname.str().c_str(), std::ios::binary|std::ios::app);
            ofs.write((const char *)m->m_data, m->m_len);
        }//if//
    }//if//

}//OutputDebugLogForAppendSockBuf//



void TcpProtocolImplementation::OutputDebugLogForDropSockBuf(
    const struct sockbuf *sb,
    int len)
{
    assert(sb);

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    std::cout << ConvertTimeToStringSecs(currentTime);
    std::cout << " [SB]     [DEL] sockbuf=" << sb;
    std::cout << " len=" << sb->sb_cc - len << "(-" << len << ")";
    std::cout << std::endl;

    if (debug_sockbuf_file) {
        if (len > 0) {
            std::ostringstream fname;
            fname << "sb_drop_" << theNodeId << "_" << sb << ".bin";
            std::ofstream ofs;
            ofs.open(fname.str().c_str(), std::ios::binary|std::ios::app);
            struct mbuf *m = sb->sb_mb;
            while (m && len > 0) {
                if (m->m_len > len) {
                    ofs.write((const char *)m->m_data, len);
                    len -= len;
                }
                else {
                    ofs.write((const char *)m->m_data, m->m_len);
                    len -= m->m_len;
                }//if//
                m = m->m_next;
            }//while//
        }//if//
    }//if//

}//OutputDebugLogForDropSockBuf//



void TcpProtocolImplementation::OutputDebugLogForStat(
    const struct tcpstat *stat)
{
    assert(stat);

    std::ostringstream fname;
    fname << "tcpstat_" << theNodeId << ".txt";
    std::ofstream ofs;
    ofs.open(fname.str().c_str(), std::ios::trunc);
    ofs << "tcps_connattempt=" << stat->tcps_connattempt << " // connections initiated" << std::endl;
    ofs << "tcps_accepts=" << stat->tcps_accepts << " // connections accepted" << std::endl;
    ofs << "tcps_connects=" << stat->tcps_connects << " // connections established" << std::endl;
    ofs << "tcps_drops=" << stat->tcps_drops << " // connections dropped" << std::endl;
    ofs << "tcps_conndrops=" << stat->tcps_conndrops << " // embryonic connections dropped" << std::endl;
    ofs << "tcps_minmssdrops=" << stat->tcps_minmssdrops << " // average minmss too low drops" << std::endl;
    ofs << "tcps_closed=" << stat->tcps_closed << " // conn. closed (includes drops)" << std::endl;
    ofs << "tcps_segstimed=" << stat->tcps_segstimed << " // segs where we tried to get rtt" << std::endl;
    ofs << "tcps_rttupdated=" << stat->tcps_rttupdated << " // times we succeeded" << std::endl;
    ofs << "tcps_delack=" << stat->tcps_delack << " // delayed acks sent" << std::endl;
    ofs << "tcps_timeoutdrop=" << stat->tcps_timeoutdrop << " // conn. dropped in rxmt timeout" << std::endl;
    ofs << "tcps_rexmttimeo=" << stat->tcps_rexmttimeo << " // retransmit timeouts" << std::endl;
    ofs << "tcps_persisttimeo=" << stat->tcps_persisttimeo << " // persist timeouts" << std::endl;
    ofs << "tcps_keeptimeo=" << stat->tcps_keeptimeo << " // keepalive timeouts" << std::endl;
    ofs << "tcps_keepprobe=" << stat->tcps_keepprobe << " // keepalive probes sent" << std::endl;
    ofs << "tcps_keepdrops=" << stat->tcps_keepdrops << " // connections dropped in keepalive" << std::endl;
    ofs << "tcps_sndtotal=" << stat->tcps_sndtotal << " // total packets sent" << std::endl;
    ofs << "tcps_sndpack=" << stat->tcps_sndpack << " // data packets sent" << std::endl;
    ofs << "tcps_sndbyte=" << stat->tcps_sndbyte << " // data bytes sent" << std::endl;
    ofs << "tcps_sndrexmitpack=" << stat->tcps_sndrexmitpack << " // data packets retransmitted" << std::endl;
    ofs << "tcps_sndrexmitbyte=" << stat->tcps_sndrexmitbyte << " // data bytes retransmitted" << std::endl;
    ofs << "tcps_sndrexmitbad=" << stat->tcps_sndrexmitbad << " // unnecessary packet retransmissions" << std::endl;
    ofs << "tcps_sndacks=" << stat->tcps_sndacks << " // ack-only packets sent" << std::endl;
    ofs << "tcps_sndprobe=" << stat->tcps_sndprobe << " // window probes sent" << std::endl;
    ofs << "tcps_sndurg=" << stat->tcps_sndurg << " // packets sent with URG only" << std::endl;
    ofs << "tcps_sndwinup=" << stat->tcps_sndwinup << " // window update-only packets sent" << std::endl;
    ofs << "tcps_sndctrl=" << stat->tcps_sndctrl << " // control (SYN|FIN|RST) packets sent" << std::endl;
    ofs << "tcps_rcvtotal=" << stat->tcps_rcvtotal << " // total packets received" << std::endl;
    ofs << "tcps_rcvpack=" << stat->tcps_rcvpack << " // packets received in sequence" << std::endl;
    ofs << "tcps_rcvbyte=" << stat->tcps_rcvbyte << " // bytes received in sequence" << std::endl;
    ofs << "tcps_rcvbadsum=" << stat->tcps_rcvbadsum << " // packets received with ccksum errs" << std::endl;
    ofs << "tcps_rcvbadoff=" << stat->tcps_rcvbadoff << " // packets received with bad offset" << std::endl;
    ofs << "tcps_rcvmemdrop=" << stat->tcps_rcvmemdrop << " // packets dropped for lack of memory" << std::endl;
    ofs << "tcps_rcvshort=" << stat->tcps_rcvshort << " // packets received too short" << std::endl;
    ofs << "tcps_rcvduppack=" << stat->tcps_rcvduppack << " // duplicate-only packets received" << std::endl;
    ofs << "tcps_rcvdupbyte=" << stat->tcps_rcvdupbyte << " // duplicate-only bytes received" << std::endl;
    ofs << "tcps_rcvpartduppack=" << stat->tcps_rcvpartduppack << " // packets with some duplicate data" << std::endl;
    ofs << "tcps_rcvpartdupbyte=" << stat->tcps_rcvpartdupbyte << " // dup. bytes in part-dup. packets" << std::endl;
    ofs << "tcps_rcvoopack=" << stat->tcps_rcvoopack << " // out-of-order packets received" << std::endl;
    ofs << "tcps_rcvoobyte=" << stat->tcps_rcvoobyte << " // out-of-order bytes received" << std::endl;
    ofs << "tcps_rcvpackafterwin=" << stat->tcps_rcvpackafterwin << " // packets with data after window" << std::endl;
    ofs << "tcps_rcvbyteafterwin=" << stat->tcps_rcvbyteafterwin << " // bytes rcvd after window" << std::endl;
    ofs << "tcps_rcvafterclose=" << stat->tcps_rcvafterclose << " // packets rcvd after close" << std::endl;
    ofs << "tcps_rcvwinprobe=" << stat->tcps_rcvwinprobe << " // rcvd window probe packets" << std::endl;
    ofs << "tcps_rcvdupack=" << stat->tcps_rcvdupack << " // rcvd duplicate acks" << std::endl;
    ofs << "tcps_rcvacktoomuch=" << stat->tcps_rcvacktoomuch << " // rcvd acks for unsent data" << std::endl;
    ofs << "tcps_rcvackpack=" << stat->tcps_rcvackpack << " // rcvd ack packets" << std::endl;
    ofs << "tcps_rcvackbyte=" << stat->tcps_rcvackbyte << " // bytes acked by rcvd acks" << std::endl;
    ofs << "tcps_rcvwinupd=" << stat->tcps_rcvwinupd << " // rcvd window update packets" << std::endl;
    ofs << "tcps_pawsdrop=" << stat->tcps_pawsdrop << " // segments dropped due to PAWS" << std::endl;
    ofs << "tcps_predack=" << stat->tcps_predack << " // times hdr predict ok for acks" << std::endl;
    ofs << "tcps_preddat=" << stat->tcps_preddat << " // times hdr predict ok for data pkts" << std::endl;
    ofs << "tcps_pcbcachemiss=" << stat->tcps_pcbcachemiss;
    ofs << "tcps_cachedrtt=" << stat->tcps_cachedrtt << " // times cached RTT in route updated" << std::endl;
    ofs << "tcps_cachedrttvar=" << stat->tcps_cachedrttvar << " // times cached rttvar updated" << std::endl;
    ofs << "tcps_cachedssthresh=" << stat->tcps_cachedssthresh << " // times cached ssthresh updated" << std::endl;
    ofs << "tcps_usedrtt=" << stat->tcps_usedrtt << " // times RTT initialized from route" << std::endl;
    ofs << "tcps_usedrttvar=" << stat->tcps_usedrttvar << " // times RTTVAR initialized from rt" << std::endl;
    ofs << "tcps_usedssthresh=" << stat->tcps_usedssthresh << " // times ssthresh initialized from r" << std::endl;
    ofs << "tcps_persistdrop=" << stat->tcps_persistdrop << " // timeout in persist state" << std::endl;
    ofs << "tcps_badsyn=" << stat->tcps_badsyn << " // bogus SYN, e.g. premature ACK" << std::endl;
    ofs << "tcps_mturesent=" << stat->tcps_mturesent << " // resends due to MTU discovery" << std::endl;
    ofs << "tcps_listendrop=" << stat->tcps_listendrop << " // listen queue overflows" << std::endl;
    ofs << "tcps_badrst=" << stat->tcps_badrst << " // ignored RSTs in the window" << std::endl;
    ofs << "tcps_sc_added=" << stat->tcps_sc_added << " // entry added to syncache" << std::endl;
    ofs << "tcps_sc_retransmitted=" << stat->tcps_sc_retransmitted << " // syncache entry was retransmitted" << std::endl;
    ofs << "tcps_sc_dupsyn=" << stat->tcps_sc_dupsyn << " // duplicate SYN packet" << std::endl;
    ofs << "tcps_sc_dropped=" << stat->tcps_sc_dropped << " // could not reply to packet" << std::endl;
    ofs << "tcps_sc_completed=" << stat->tcps_sc_completed << " // successful extraction of entry" << std::endl;
    ofs << "tcps_sc_bucketoverflow=" << stat->tcps_sc_bucketoverflow << " // syncache per-bucket limit hit" << std::endl;
    ofs << "tcps_sc_cacheoverflow=" << stat->tcps_sc_cacheoverflow << " // syncache cache limit hit" << std::endl;
    ofs << "tcps_sc_reset=" << stat->tcps_sc_reset << " // RST removed entry from syncache" << std::endl;
    ofs << "tcps_sc_stale=" << stat->tcps_sc_stale << " // timed out or listen socket gone" << std::endl;
    ofs << "tcps_sc_aborted=" << stat->tcps_sc_aborted << " // syncache entry aborted" << std::endl;
    ofs << "tcps_sc_badack=" << stat->tcps_sc_badack << " // removed due to bad ACK" << std::endl;
    ofs << "tcps_sc_unreach=" << stat->tcps_sc_unreach << " // ICMP unreachable received" << std::endl;
    ofs << "tcps_sc_zonefail=" << stat->tcps_sc_zonefail << " // zalloc() failed" << std::endl;
    ofs << "tcps_sc_sendcookie=" << stat->tcps_sc_sendcookie << " // SYN cookie sent" << std::endl;
    ofs << "tcps_sc_recvcookie=" << stat->tcps_sc_recvcookie << " // SYN cookie received" << std::endl;
    ofs << "tcps_hc_added=" << stat->tcps_hc_added << " // entry added to hostcache" << std::endl;
    ofs << "tcps_hc_bucketoverflow=" << stat->tcps_hc_bucketoverflow << " // hostcache per bucket limit hit" << std::endl;
    ofs << "tcps_finwait2_drops=" << stat->tcps_finwait2_drops << " // Drop FIN_WAIT_2 connection after time limit" << std::endl;
    ofs << "tcps_sack_recovery_episode=" << stat->tcps_sack_recovery_episode << " // SACK recovery episodes" << std::endl;
    ofs << "tcps_sack_rexmits=" << stat->tcps_sack_rexmits << " // SACK rexmit segments  " << std::endl;
    ofs << "tcps_sack_rexmit_bytes=" << stat->tcps_sack_rexmit_bytes << " // SACK rexmit bytes     " << std::endl;
    ofs << "tcps_sack_rcv_blocks=" << stat->tcps_sack_rcv_blocks << " // SACK blocks (options) received" << std::endl;
    ofs << "tcps_sack_send_blocks=" << stat->tcps_sack_send_blocks << " // SACK blocks (options) sent    " << std::endl;
    ofs << "tcps_sack_sboverflow=" << stat->tcps_sack_sboverflow << " // times scoreboard overflowed" << std::endl;
    ofs << "tcps_ecn_ce=" << stat->tcps_ecn_ce << " // ECN Congestion Experienced" << std::endl;
    ofs << "tcps_ecn_ect0=" << stat->tcps_ecn_ect0 << " // ECN Capable Transport" << std::endl;
    ofs << "tcps_ecn_ect1=" << stat->tcps_ecn_ect1 << " // ECN Capable Transport" << std::endl;
    ofs << "tcps_ecn_shs=" << stat->tcps_ecn_shs << " // ECN successful handshakes" << std::endl;
    ofs << "tcps_ecn_rcwnd=" << stat->tcps_ecn_rcwnd << " // # times ECN reduced the cwnd" << std::endl;

}//OutputDebugLogForStat//



//------------------------------------------------------------------------------
// TcpConnectionImplementation
//------------------------------------------------------------------------------

TcpConnectionImplementation::TcpConnectionImplementation(
    const shared_ptr<TcpProtocol>& initTcpProtocolPtr,
    const shared_ptr<TcpConnection::AppTcpEventHandler>& initAppEventHandlerPtr,
    const BsdTcpConnectionIdType& initConnectionId,
    const NetworkAddress& localAddress,
    const unsigned short int localPort,
    const NetworkAddress& foreignAddress,
    const unsigned short int foreignPort)
    :
    tcpProtocolPtr(initTcpProtocolPtr),
    appEventHandlerPtr(initAppEventHandlerPtr),
    connectionId(initConnectionId),
    socket_porting(nullptr),
    useVirtualPayload(false),
    packetPriority(0),
    currentNumReceivedBytes(0),
    currentNumSentBytes(0),
    currentNumDeliveredBytes(0)
{
    tcpProtocolPtr->implPtr->BeginTcpProcess();
#ifdef USE_IPV6
    int error = socreate(AF_INET6, &socket_porting);
#else
    int error = socreate(AF_INET, &socket_porting);
#endif
    tcpProtocolPtr->implPtr->EndTcpProcess();
    CheckIfErrorInFreeBsd9Port(error, "socreate");

    assert(socket_porting);
    socket_porting->so_ctx_id = connectionId;

}//TcpConnectionImplementation//



TcpConnectionImplementation::~TcpConnectionImplementation()
{
    if (socket_porting == nullptr) {
        return;
    }//if//

    struct socket *so = socket_porting;

    socket_porting->so_ctx_id = InvalidBsdTcpConnectionId;
    socket_porting = nullptr;

    if (((so->so_state & SS_ISDISCONNECTING) != 0) ||
        ((so->so_state & SS_ISDISCONNECTED) != 0)) {
        return;
    }//if//

    tcpProtocolPtr->implPtr->BeginTcpProcess();
    int error = soclose(so);
    tcpProtocolPtr->implPtr->EndTcpProcess();
    CheckIfErrorInFreeBsd9Port(error, "soclose");

}//~TcpConnectionImplementation//



bool TcpConnectionImplementation::IsConnected() const
{
    if (socket_porting == nullptr) {
        return false;
    }//if//

    return (socket_porting->so_state & SS_ISCONNECTED) != 0;

}//IsConnected//



void TcpConnectionImplementation::EnableVirtualPayload()
{
    useVirtualPayload = true;

}//EnableVirtualPayload//



void TcpConnectionImplementation::SetPacketPriority(
    const PacketPriority& priority)
{
    packetPriority = priority;

}//SetPacketPriority//



void TcpConnectionImplementation::SetAppTcpEventHandler(
    const shared_ptr<TcpConnection::AppTcpEventHandler>& newAppEventHandlerPtr)
{
    appEventHandlerPtr = newAppEventHandlerPtr;

}//SetAppTcpEventHandler//



void TcpConnectionImplementation::ClearAppTcpEventHandler()
{
    appEventHandlerPtr.reset();

}//ClearAppTcpEventHandler//



void TcpConnectionImplementation::SendDataBlock(
    shared_ptr<vector<unsigned char> >& dataBlockPtr,
    const unsigned int dataLength)
{
    if (socket_porting == nullptr) {
        return;
    }//if//

    assert(dataLength >= dataBlockPtr->size());

    if (!useVirtualPayload) {
        dataBlockPtr->resize(dataLength);
    }//if//

    currentNumSentBytes += dataLength;
    tcpProtocolPtr->implPtr->bytesReceivedFromUpperLayerStatPtr->IncrementCounter(dataLength);

    struct mbuf *m = m_get(new mbuf_porting(dataBlockPtr, dataLength), MT_DATA);
    assert(m);
    m->m_len = dataLength;
    m->m_pkthdr.len = dataLength;

    tcpProtocolPtr->implPtr->BeginTcpProcess();
    int error = sosend(socket_porting, 0, m);
    tcpProtocolPtr->implPtr->EndTcpProcess();
    CheckIfErrorInFreeBsd9Port(error, "sosend");

}//SendDataBlock//



NetworkAddress TcpConnectionImplementation::GetForeignAddress() const
{
    if (socket_porting == nullptr) {
        return NetworkAddress::invalidAddress;
    }//if//

    struct inpcb *inp = (struct inpcb *)socket_porting->so_pcb;
    assert(inp);

#ifdef USE_IPV6
    uint64_t networkAddressHighBits;
    uint64_t networkAddressLowBits;
    ConvertNet128ToTwoHost64(inp->in6p_faddr.s6_addr, networkAddressHighBits, networkAddressLowBits);
    NetworkAddress networkAddress(networkAddressHighBits, networkAddressLowBits);
#else
    NetworkAddress networkAddress(ntohl(inp->inp_faddr.s_addr));
#endif

    return networkAddress;

}//GetForeignAddress//



void TcpConnectionImplementation::Close()
{
    if (socket_porting == nullptr) {
        return;
    }//if//

    tcpProtocolPtr->implPtr->BeginTcpProcess();
    int error = soclose(socket_porting);
    tcpProtocolPtr->implPtr->EndTcpProcess();
    CheckIfErrorInFreeBsd9Port(error, "soclose");

}//Close//



void TcpConnectionImplementation::ProcessCallbackDelayEvent(
    const DelayKind& delayKind)
{
    if (socket_porting == nullptr) {
        return;
    }//if//

    switch (delayKind) {
    case READY_TO_ACCEPT:
    {
        struct socket *so, *head;

        head = socket_porting;
        so = TAILQ_FIRST(&head->so_comp);
        assert(so);
        TAILQ_REMOVE(&head->so_comp, so, so_list);
        head->so_qlen--;
        so->so_qstate &= ~SQ_COMP;
        so->so_head = NULL;

        tcpProtocolPtr->implPtr->BeginTcpProcess();
        int error = soaccept(so);
        tcpProtocolPtr->implPtr->EndTcpProcess();
        CheckIfErrorInFreeBsd9Port(error, "soaccept");

        break;
    }
    case READY_TO_SEND:
    {
        if (appEventHandlerPtr.get()) {
            appEventHandlerPtr->DoTcpIsReadyForMoreDataAction();
        }//if//

        break;
    }
    case READY_TO_RECEIVE:
    {
        if (appEventHandlerPtr.get()) {
            ReceiveDataBlock();
        }//if//

        break;
    }
    case READY_TO_CLOSE_REMOTE:
    {
        if (appEventHandlerPtr.get()) {
            appEventHandlerPtr->DoTcpRemoteHostClosedAction();
        }//if//

        break;
    }
    default:
        assert(false);
    }//switch/

}//ProcessCallbackDelayEvent//



void TcpConnectionImplementation::ReceiveDataBlock()
{
    if (socket_porting == nullptr) {
        return;
    }//if//

    while (socket_porting->so_rcv.sb_cc > 0) {
        struct mbuf *mb = socket_porting->so_rcv.sb_mb;
        assert(mb);

        currentNumReceivedBytes += mb->m_len;
        tcpProtocolPtr->implPtr->bytesSentToUpperLayerStatPtr->IncrementCounter(mb->m_len);

        if (mb->m_datalen != mb->m_vdatalen) {
            assert(mb->m_extptr);
            assert(mb->m_databuf + mb->m_datalen <= mb->m_data);

            const unsigned int virtualDataBegin = mb->m_data - (mb->m_databuf + mb->m_datalen);
            const unsigned int virtualDataEnd = virtualDataBegin + mb->m_len;
            unsigned int offset = virtualDataBegin;
            unsigned int nextOffset = 0;

            while (offset < virtualDataEnd) {
                unsigned int length;
                const unsigned char* rawFragmentDataPtr;

                tcpProtocolPtr->implPtr->BeginTcpProcess();
                mb->m_datasource->get_data(offset, virtualDataEnd, length, rawFragmentDataPtr, nextOffset);
                tcpProtocolPtr->implPtr->EndTcpProcess();
                assert(offset < nextOffset);
                assert(length <= (nextOffset - offset));

                bool stallIncomingData = false;

                assert(appEventHandlerPtr.get());
                appEventHandlerPtr->ReceiveDataBlock(
                    rawFragmentDataPtr, nextOffset - offset, length, stallIncomingData);

                assert(!stallIncomingData);//TBD//

                offset = nextOffset;
            }//while//
        }
        else {
            assert(mb->m_datalen == mb->m_vdatalen);

            bool stallIncomingData = false;

            assert(appEventHandlerPtr.get());
            appEventHandlerPtr->ReceiveDataBlock(
                (const unsigned char *)mb->m_data, mb->m_len, mb->m_len, stallIncomingData);

            assert(!stallIncomingData);//TBD//
        }//if//

        tcpProtocolPtr->implPtr->BeginTcpProcess();
        sbdrop_locked(&socket_porting->so_rcv, mb->m_len);
        tcpProtocolPtr->implPtr->EndTcpProcess();

    }//while//

}//ReceiveDataBlock//

}//namespace//

using std::unique_ptr;
using std::move;


FreeBsd9Port::mbuf_porting::~mbuf_porting()
{
    if (m_packet != NULL) {
        assert(m_vector.get() == NULL);
        delete m_packet;
        m_packet = nullptr;
    }
    else {
        assert(m_vector.get() != NULL);
    }
}



int FreeBsd9Port::mbuf_porting::datalen()
{
    if (m_packet != NULL) {
        assert(m_vector.get() == NULL);
        return m_packet->ActualLengthBytes() - m_off;
    }
    else {
        assert(m_vector.get() != NULL);
        return m_vector->size();
    }
}



int FreeBsd9Port::mbuf_porting::vdatalen()
{
    if (m_packet != NULL) {
        assert(m_vector.get() == NULL);
        return m_packet->LengthBytes() - m_off;
    }
    else {
        assert(m_vector.get() != NULL);
        return m_vlen;
    }
}



FreeBsd9Port::caddr_t FreeBsd9Port::mbuf_porting::databuf()
{
    if (m_packet != NULL) {
        assert(m_vector.get() == NULL);
        return (caddr_t)m_packet->GetRawPayloadData() + m_off;
    }
    else {
        assert(m_vector.get() != NULL);
        if (m_vector->empty()) {
            assert(m_vlen > 0);
            return NULL;//TBD: all payloads are virtual
        }
        else {
            return (caddr_t)&(*m_vector)[0];
        }
    }
}


void FreeBsd9Port::mbuf_porting::get_data(
    const unsigned int dataBegin,
    const unsigned int dataEnd,
    unsigned int& length,
    const unsigned char*& rawFragmentDataPtr,
    unsigned int& nextOffset) const
{
    assert(dataBegin < dataEnd);

    if (m_packet != NULL) {
        GetVirtualFragmentData(
            curvnet->ctx, *m_packet, dataBegin, dataEnd, length, rawFragmentDataPtr, nextOffset);
    }
    else {
        assert(dataEnd <= m_vlen);

        if (m_vector->size() <= dataBegin) {
            length = 0;
            rawFragmentDataPtr = nullptr;
            nextOffset = dataEnd;
        }
        else if (dataBegin < m_vector->size() && m_vector->size() <= dataEnd) {
            length = m_vector->size() - dataBegin;
            rawFragmentDataPtr = &(*m_vector)[dataBegin];
            nextOffset = dataEnd;
        }
        else {
            length = dataEnd - dataBegin;
            rawFragmentDataPtr = &(*m_vector)[dataBegin];
            nextOffset = dataEnd;
        }
    }
}



// Glue code so that don't have to directly include implementation definition in Free BSD.

int32_t FreeBsd9Port::GenerateRandomInt(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr)
{
    return (tcpProtocolPtr->GenerateRandomInt());
}



void FreeBsd9Port::RegisterCallout(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::callout *c)
{
    tcpProtocolPtr->RegisterCallout(c);
}



void FreeBsd9Port::ScheduleCallout(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::callout *c)
{
    tcpProtocolPtr->ScheduleCallout(c);
}



void FreeBsd9Port::CreatePacket(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *so,
    unsigned int totalPayloadLength,
    ScenSim::Packet*& packetPtr,
    unsigned char*& rawPacketDataPtr)
{
    tcpProtocolPtr->CreatePacket(
        so, totalPayloadLength, packetPtr, rawPacketDataPtr);
}



void FreeBsd9Port::AddVirtualFragmentData(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    ScenSim::Packet& packet,
    const unsigned int offset,
    const unsigned int length,
    const char* rawFragmentDataPtr)
{
    tcpProtocolPtr->AddVirtualFragmentData(
        packet, offset, length, reinterpret_cast<const unsigned char*>(rawFragmentDataPtr));
}

void FreeBsd9Port::GetVirtualFragmentData(
    const ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    const ScenSim::Packet& packet,
    const unsigned int dataBegin,
    const unsigned int dataEnd,
    unsigned int& length,
    const unsigned char*& rawFragmentDataPtr,
    unsigned int& nextOffset)
{
    tcpProtocolPtr->GetVirtualFragmentData(packet, dataBegin, dataEnd, length,
        rawFragmentDataPtr, nextOffset);
}



void FreeBsd9Port::SendToNetworkLayer(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    const struct FreeBsd9Port::tcpcb *tp,
    const struct FreeBsd9Port::mbuf *m,
    ScenSim::Packet*& packetPtr)
{
    unique_ptr<ScenSim::Packet> uniquePacketPtr(packetPtr);
    packetPtr = nullptr;
    tcpProtocolPtr->SendToNetworkLayer(tp, m, uniquePacketPtr);
}



void FreeBsd9Port::HandleNewConnection(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *lso,
    struct FreeBsd9Port::socket **so)
{
    tcpProtocolPtr->HandleNewConnection(lso, so);
}



void FreeBsd9Port::NotifyIsConnected(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *so)
{
    tcpProtocolPtr->NotifyIsConnected(so);
}



void FreeBsd9Port::NotifyBufferAvailable(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *so)
{
    tcpProtocolPtr->NotifyBufferAvailable(so);
}



void FreeBsd9Port::NotifyDataArrival(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *so)
{
    tcpProtocolPtr->NotifyDataArrival(so);
}




void FreeBsd9Port::NotifyIsFinReceived(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *so)
{
    tcpProtocolPtr->NotifyIsFinReceived(so);
}




void FreeBsd9Port::NotifyIsSocketDisconnected(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *so)
{
    tcpProtocolPtr->NotifyIsSocketDisconnected(so);
}


void FreeBsd9Port::NotifyIsSocketClosing(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *so)
{
    tcpProtocolPtr->NotifyIsSocketClosing(so);
}



void FreeBsd9Port::UpdateRttForStatistics(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    int rtt)
{
    tcpProtocolPtr->UpdateRttForStatistics(rtt);
}



void FreeBsd9Port::UpdateCwndForStatistics(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    unsigned long int cwnd)
{
    tcpProtocolPtr->UpdateCwndForStatistics(cwnd);
}



void FreeBsd9Port::CountRetransmissionForStatistics(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr)
{
    tcpProtocolPtr->CountRetransmissionForStatistics();
}



void FreeBsd9Port::OutputDebugLogForPacket(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    const struct FreeBsd9Port::tcpcb *tp,
    const struct FreeBsd9Port::mbuf *m,
    const ScenSim::Packet& aPacket,
    const bool isSent)
{
    tcpProtocolPtr->OutputDebugLogForPacket(
        tp, m, aPacket.GetRawPayloadData(), aPacket.LengthBytes(), isSent);
}



void FreeBsd9Port::OutputDebugLogForReassembleQueue(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    const struct FreeBsd9Port::tcpcb *tp)
{
    tcpProtocolPtr->OutputDebugLogForReassembleQueue(tp);
}



void FreeBsd9Port::OutputDebugLogForAppendSockBuf(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    const struct FreeBsd9Port::sockbuf *sb,
    const struct FreeBsd9Port::mbuf *m)
{
    tcpProtocolPtr->OutputDebugLogForAppendSockBuf(sb, m);
}



void FreeBsd9Port::OutputDebugLogForDropSockBuf(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    const struct FreeBsd9Port::sockbuf *sb,
    int len)
{
    tcpProtocolPtr->OutputDebugLogForDropSockBuf(sb, len);
}



void FreeBsd9Port::OutputDebugLogForStat(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    const struct FreeBsd9Port::tcpstat *stat)
{
    tcpProtocolPtr->OutputDebugLogForStat(stat);
}









