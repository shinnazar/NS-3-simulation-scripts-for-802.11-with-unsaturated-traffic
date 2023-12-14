#include "ns3/ampdu-subframe-header.h"
#include "ns3/application-container.h"
#include "ns3/boolean.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/gnuplot.h"
#include "ns3/integer.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-list.h"
#include "ns3/packet-socket-client.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/packet-socket-server.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/queue-size.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-net-device.h"
#include "ns3/yans-wifi-helper.h"

#include <fstream>

/// Avoid std::numbers::pi because it's C++20
#define PI 3.1415926535
#define SIFS 10e-6          //sec            
#define SLOT 20e-6          //sec             
#define DIFS SIFS + 2*SLOT  //sec
#define PROP 2e-6           //sec
#define PHY_HDR 192e-6      //sec
// #define MAC_HDR 34          //bytes
#define MAC_HDR 62          //bytes
#define DATA_RATE 11e6      //bps
#define BASIC_RATE 1e6      //bps
#define ACK round((PHY_HDR + 14*8.0 / BASIC_RATE) * 1e6)/1.0e6    //sec

NS_LOG_COMPONENT_DEFINE("wifi_11b");

using namespace ns3;

std::ofstream cwTraceFile;      ///< File that traces CW over time
std::ofstream backoffTraceFile; ///< File that traces backoff over time
std::ofstream phyTxTraceFile;   ///< File that traces PHY transmissions  over time
std::ofstream macTxTraceFile;   ///< File that traces MAC transmissions  over time
std::ofstream macRxTraceFile;   ///< File that traces MAC receptions  over time

std::map<Mac48Address, uint64_t> packetsReceived; ///< Map that stores the total packets received
                                                  ///< per STA (and addressed to that STA)
std::map<Mac48Address, uint64_t> bytesReceived; ///< Map that stores the total bytes received per STA (and addressed to that STA)
std::map<Mac48Address, uint64_t> packetsTransmitted; ///< Map that stores the total packets transmitted per STA
std::map<Mac48Address, uint64_t> psduFailed; ///< Map that stores the total number of unsuccessfuly received PSDUS (for which
                ///< the PHY header was successfully received)  per STA (including PSDUs not
                ///< addressed to that STA)
std::map<Mac48Address, uint64_t> psduSucceeded; ///< Map that stores the total number of successfully received PSDUs per STA
                   ///< (including PSDUs not addressed to that STA)
std::map<Mac48Address, uint64_t> phyHeaderFailed; ///< Map that stores the total number of
                                                  ///< unsuccessfuly received PHY headers per STA
std::map<Mac48Address, uint64_t> rxEventWhileTxing; ///< Map that stores the number of reception events per STA that occurred
                       ///< while PHY was already transmitting a PPDU
std::map<Mac48Address, uint64_t> rxEventWhileRxing; ///< Map that stores the number of reception events per STA that occurred
                       ///< while PHY was already receiving a PPDU
std::map<Mac48Address, uint64_t> rxEventWhileDecodingPreamble; ///< Map that stores the number of reception events per STA that
                                  ///< occurred while PHY was already decoding a preamble
std::map<Mac48Address, uint64_t> rxEventAbortedByTx; ///< Map that stores the number of reception events aborted per STA because
                        ///< the PHY has started to transmit


std::set<uint32_t> associated; ///< Contains the IDs of the STAs that successfully associated to the
                               ///< access point (in infrastructure mode only)

bool tracing = false;    ///< Flag to enable/disable generation of tracing files
uint32_t pktSize = 1024; ///< packet size used for the simulation (in bytes)
uint8_t maxMpdus = 0;    ///< The maximum number of MPDUs in A-MPDUs (0 to disable MPDU aggregation)

/**
 * Parse context strings of the form "/NodeList/x/DeviceList/x/..." to extract the NodeId integer
 *
 * \param context The context to parse.
 * \return the NodeId
 */
uint32_t
ContextToNodeId(std::string context)
{
    std::string sub = context.substr(10);
    uint32_t pos = sub.find("/Device");
    return std::stoi(sub.substr(0, pos));
}

/**
 * Parse context strings of the form "/NodeList/x/DeviceList/x/..." and fetch the Mac address
 *
 * \param context The context to parse.
 * \return the device MAC address
 */
Mac48Address
ContextToMac(std::string context)
{
    std::string sub = context.substr(10);
    uint32_t pos = sub.find("/Device");
    uint32_t nodeId = std::stoi(sub.substr(0, pos));
    Ptr<Node> n = NodeList::GetNode(nodeId);
    Ptr<WifiNetDevice> d;
    for (uint32_t i = 0; i < n->GetNDevices(); i++)
    {
        d = n->GetDevice(i)->GetObject<WifiNetDevice>();
        if (d)
        {
            break;
        }
    }
    return Mac48Address::ConvertFrom(d->GetAddress());
}
// Functions for tracing.

/**
 * Incremement the counter for a given address.
 *
 * \param [out] counter The counter to increment.
 * \param addr The address to incremement the counter for.
 * \param increment The incremement (1 if omitted).
 */
void
IncrementCounter(std::map<Mac48Address, uint64_t>& counter,
                 Mac48Address addr,
                 uint64_t increment = 1)
{
    auto it = counter.find(addr);
    if (it != counter.end())
    {
        it->second += increment;
    }
    else
    {
        counter.insert(std::make_pair(addr, increment));
    }
}

/**
 * Trace a packet reception.
 *
 * \param context The context.
 * \param p The packet.
 * \param channelFreqMhz The channel frequqncy.
 * \param txVector The TX vector.
 * \param aMpdu The AMPDU.
 * \param signalNoise The signal and noise dBm.
 * \param staId The STA ID.
 */
void
TracePacketReception(std::string context,
                     Ptr<const Packet> p,
                     uint16_t channelFreqMhz,
                     WifiTxVector txVector,
                     MpduInfo aMpdu,
                     SignalNoiseDbm signalNoise,
                     uint16_t staId)
{
    Ptr<Packet> packet = p->Copy();
    if (txVector.IsAggregation())
    {
        AmpduSubframeHeader subHdr;
        uint32_t extractedLength;
        packet->RemoveHeader(subHdr);
        extractedLength = subHdr.GetLength();
        packet = packet->CreateFragment(0, static_cast<uint32_t>(extractedLength));
    }
    WifiMacHeader hdr;
    packet->PeekHeader(hdr);
    // hdr.GetAddr1() is the receiving MAC address
    if (hdr.GetAddr1() != ContextToMac(context))
    {
        return;
    }
    // hdr.GetAddr2() is the sending MAC address
    if (packet->GetSize() >= pktSize) // ignore non-data frames
    {
        IncrementCounter(packetsReceived, hdr.GetAddr2());
        IncrementCounter(bytesReceived, hdr.GetAddr2(), pktSize);
    }
}
/**
 * Contention window trace.
 *
 * \param context The context.
 * \param cw The contention window.
 */
void
CwTrace(std::string context, uint32_t cw, uint8_t /* linkId */)
{
    NS_LOG_INFO("CW time=" << Simulator::Now() << " node=" << ContextToNodeId(context)
                           << " val=" << cw);
    if (tracing)
    {
        cwTraceFile << Simulator::Now().GetSeconds() << " " << ContextToNodeId(context) << " " << cw
                    << std::endl;
    }
}

/**
 * Backoff trace.
 *
 * \param context The context.
 * \param newVal The backoff value.
 */
void
BackoffTrace(std::string context, uint32_t newVal, uint8_t /* linkId */)
{
    NS_LOG_INFO("Backoff time=" << Simulator::Now() << " node=" << ContextToNodeId(context)
                                << " val=" << newVal);
    if (tracing)
    {
        backoffTraceFile << Simulator::Now().GetSeconds() << " " << ContextToNodeId(context) << " "
                         << newVal << std::endl;
    }
}

/**
 * PHY Rx trace.
 *
 * \param context The context.
 * \param p The packet.
 * \param power The Rx power.
 */
void
PhyRxTrace(std::string context, Ptr<const Packet> p, RxPowerWattPerChannelBand power)
{
    NS_LOG_INFO("PHY-RX-START time=" << Simulator::Now() << " node=" << ContextToNodeId(context)
                                     << " size=" << p->GetSize());
}

/**
 * PHY Rx trace.
 *
 * \param context The context.
 * \param txVector The TX vector.
 * \param psduDuration The PDSU diration.
 */
void
PhyRxPayloadTrace(std::string context, WifiTxVector txVector, Time psduDuration)
{
    NS_LOG_INFO("PHY-RX-PAYLOAD-START time=" << Simulator::Now()
                                             << " node=" << ContextToNodeId(context)
                                             << " psduDuration=" << psduDuration);
}

/**
 * PHY Drop trace.
 *
 * \param context The context.
 * \param p The packet.
 * \param reason The drop reason.
 */
void
PhyRxDropTrace(std::string context, Ptr<const Packet> p, WifiPhyRxfailureReason reason)
{
    NS_LOG_INFO("PHY-RX-DROP time=" << Simulator::Now() << " node=" << ContextToNodeId(context)
                                    << " size=" << p->GetSize() << " reason=" << reason);
    Mac48Address addr = ContextToMac(context);
    switch (reason)
    {
    case UNSUPPORTED_SETTINGS:
        NS_FATAL_ERROR("RX packet with unsupported settings!");
        break;
    case CHANNEL_SWITCHING:
        NS_FATAL_ERROR("Channel is switching!");
        break;
    case BUSY_DECODING_PREAMBLE: {
        if (p->GetSize() >= pktSize) // ignore non-data frames
        {
            IncrementCounter(rxEventWhileDecodingPreamble, addr);
        }
        break;
    }
    case RXING: {
        if (p->GetSize() >= pktSize) // ignore non-data frames
        {
            IncrementCounter(rxEventWhileRxing, addr);
        }
        break;
    }
    case TXING: {
        if (p->GetSize() >= pktSize) // ignore non-data frames
        {
            IncrementCounter(rxEventWhileTxing, addr);
        }
        break;
    }
    case SLEEPING:
        NS_FATAL_ERROR("Device is sleeping!");
        break;
    case PREAMBLE_DETECT_FAILURE:
        NS_FATAL_ERROR("Preamble should always be detected!");
        break;
    case RECEPTION_ABORTED_BY_TX: {
        if (p->GetSize() >= pktSize) // ignore non-data frames
        {
            IncrementCounter(rxEventAbortedByTx, addr);
        }
        break;
    }
    case L_SIG_FAILURE: {
        if (p->GetSize() >= pktSize) // ignore non-data frames
        {
            IncrementCounter(phyHeaderFailed, addr);
        }
        break;
    }
    case HT_SIG_FAILURE:
    case SIG_A_FAILURE:
    case SIG_B_FAILURE:
        NS_FATAL_ERROR("Unexpected PHY header failure!");
    case PREAMBLE_DETECTION_PACKET_SWITCH:
        NS_FATAL_ERROR("All devices should send with same power, so no packet switch during "
                       "preamble detection should occur!");
        break;
    case FRAME_CAPTURE_PACKET_SWITCH:
        NS_FATAL_ERROR("Frame capture should be disabled!");
        break;
    case OBSS_PD_CCA_RESET:
        NS_FATAL_ERROR("Unexpected CCA reset!");
        break;
    case UNKNOWN:
    default:
        NS_FATAL_ERROR("Unknown drop reason!");
        break;
    }
}

/**
 * PHY RX end trace
 *
 * \param context The context.
 * \param p The packet.
 */
void
PhyRxDoneTrace(std::string context, Ptr<const Packet> p)
{
    NS_LOG_INFO("PHY-RX-END time=" << Simulator::Now() << " node=" << ContextToNodeId(context)
                                   << " size=" << p->GetSize());
}

/**
 * PHY successful RX trace
 *
 * \param context The context.
 * \param p The packet.
 * \param snr The SNR.
 * \param mode The WiFi mode.
 * \param preamble The preamble.
 */
void
PhyRxOkTrace(std::string context,
             Ptr<const Packet> p,
             double snr,
             WifiMode mode,
             WifiPreamble preamble)
{
    uint8_t nMpdus = (p->GetSize() / pktSize);
    NS_LOG_INFO("PHY-RX-OK time=" << Simulator::Now().As(Time::S) << " node="
                                  << ContextToNodeId(context) << " size=" << p->GetSize()
                                  << " nMPDUs=" << +nMpdus << " snr=" << snr << " mode=" << mode
                                  << " preamble=" << preamble);
    if ((maxMpdus != 0) && (nMpdus != 0) && (nMpdus != maxMpdus))
    {
        if (nMpdus > maxMpdus)
        {
            NS_FATAL_ERROR("A-MPDU settings not properly applied: maximum configured MPDUs is "
                           << +maxMpdus << " but received an A-MPDU containing " << +nMpdus
                           << " MPDUs");
        }
        NS_LOG_WARN("Warning: less MPDUs aggregated in a received A-MPDU ("
                    << +nMpdus << ") than configured (" << +maxMpdus << ")");
    }
    if (p->GetSize() >= pktSize) // ignore non-data frames
    {
        Mac48Address addr = ContextToMac(context);
        IncrementCounter(psduSucceeded, addr);
    }
}

/**
 * PHY RX error trace
 *
 * \param context The context.
 * \param p The packet.
 * \param snr The SNR.
 */
void
PhyRxErrorTrace(std::string context, Ptr<const Packet> p, double snr)
{
    NS_LOG_INFO("PHY-RX-ERROR time=" << Simulator::Now() << " node=" << ContextToNodeId(context)
                                     << " size=" << p->GetSize() << " snr=" << snr);
    if (p->GetSize() >= pktSize) // ignore non-data frames
    {
        Mac48Address addr = ContextToMac(context);
        IncrementCounter(psduFailed, addr);
    }
}

/**
 * PHY TX trace
 *
 * \param context The context.
 * \param p The packet.
 * \param txPowerW The TX power.
 */
void
PhyTxTrace(std::string context, Ptr<const Packet> p, double txPowerW)
{
    NS_LOG_INFO("PHY-TX-START time=" << Simulator::Now() << " node=" << ContextToNodeId(context)
                                     << " size=" << p->GetSize() << " " << txPowerW);
    if (tracing)
    {
        phyTxTraceFile << Simulator::Now().GetSeconds() << " " << ContextToNodeId(context)
                       << " size=" << p->GetSize() << " " << txPowerW << std::endl;
    }
    if (p->GetSize() >= pktSize) // ignore non-data frames
    {
        Mac48Address addr = ContextToMac(context);
        IncrementCounter(packetsTransmitted, addr);
    }
}

/**
 * PHY TX end trace.
 *
 * \param context The context.
 * \param p The packet.
 */
void
PhyTxDoneTrace(std::string context, Ptr<const Packet> p)
{
    NS_LOG_INFO("PHY-TX-END time=" << Simulator::Now() << " node=" << ContextToNodeId(context)
                                   << " " << p->GetSize());
}

/**
 * MAC TX trace.
 *
 * \param context The context.
 * \param p The packet.
 */
void
MacTxTrace(std::string context, Ptr<const Packet> p)
{
    if (tracing)
    {
        macTxTraceFile << Simulator::Now().GetSeconds() << " " << ContextToNodeId(context) << " "
                       << p->GetSize() << std::endl;
    }
}

/**
 * MAC RX trace.
 *
 * \param context The context.
 * \param p The packet.
 */
void
MacRxTrace(std::string context, Ptr<const Packet> p)
{
    if (tracing)
    {
        macRxTraceFile << Simulator::Now().GetSeconds() << " " << ContextToNodeId(context) << " "
                       << p->GetSize() << std::endl;
    }
}

/**
 * Reset the stats.
 */
void
RestartCalc()
{
    bytesReceived.clear();
    packetsReceived.clear();
    packetsTransmitted.clear();
    psduFailed.clear();
    psduSucceeded.clear();
    phyHeaderFailed.clear();
    rxEventWhileDecodingPreamble.clear();
    rxEventWhileRxing.clear();
    rxEventWhileTxing.clear();
    rxEventAbortedByTx.clear();
}
/**
 * Get the Counter associated with a MAC address.
 *
 * \param counter The map of counters to inspect.
 * \param addr The MAC address.
 * \return the value of the counter,
 */
uint64_t
GetCount(const std::map<Mac48Address, uint64_t>& counter, Mac48Address addr)
{
    uint64_t count = 0;
    auto it = counter.find(addr);
    if (it != counter.end())
    {
        count = it->second;
    }
    return count;
}

double GetCollisionProb(){
    int sum_rx = 0, sum_tx_attempt = 0;
    double p_col = -1;
    for (auto it = packetsReceived.begin(); it != packetsReceived.end(); it++){
        sum_rx = sum_rx + it->second;
        sum_tx_attempt = sum_tx_attempt + GetCount(packetsTransmitted, it->first); 
    }
    if(sum_tx_attempt)
        p_col = (sum_tx_attempt - sum_rx) / double(sum_tx_attempt);
    return p_col;
}

double get_T_success(int packet_size){
    return round((PHY_HDR + (MAC_HDR + packet_size)*8.0/DATA_RATE + SIFS + ACK + 2*PROP + DIFS)*1e6)/1.0e6;
}
