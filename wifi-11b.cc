/*
 * Copyright (c) 2009 MIRKO BANCHI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Mirko Banchi <mk.banchi@gmail.com>
 *          Sebastien Deronne <sebastien.deronne@gmail.com>
 */

#include "ns3/boolean.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/ht-phy.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/tuple.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/uinteger.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "wifi-11b.h"
#include "ns3/rng-seed-manager.h"

/*
* This script is to simulate the 802.11b Wi-Fi with n + 1 nodes
* where n nodes contend to send the given traffic load to a receiver node.
* Receiver node is access point (AP) if infrastructure mode is chosen, it's simple ad-hoc node if ad-hoc mode is chosen.
* Some of the codes were re-used from following scripts: wifi-ht-network.cc, wifi-bianchi.cc.
* Inputs: you can specify some input arguments such as 
*   - number of nodes, duration of simulation traffic load, queue size, seed value, mode (infrastructure or adhoc), etc
* Outputs: you'll get the normalized throughput (according to Liu's approach) and collision probability.
*/ 


using namespace ns3;

// NS_LOG_COMPONENT_DEFINE("ht-wifi-network");

int
main(int argc, char* argv[])
{
    // RngSeedManager::SetRun(10);
    uint32_t verbose = 0;   ///< verbosity level that increases the number of debugging traces
    bool infra = false;
    std::string dataMode = "DsssRate11Mbps";        ///< the constant PHY mode string used to transmit frames
    std::string ctrlMode = "DsssRate1Mbps";        ///< the constant PHY mode string used to transmit frames
    bool useRts = false;
    double simulationTime = 10; // seconds
    double distance = 1.0;      // meters
    double load = 1.0; // network offered load at STAs
    int nStas = 1; // number of contending stations to send uplink traffic
    int payloadSize = 1024; // Bytes
    int queueSize = 10; //packets
    int seed = 0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose",
                 "Logging level (0: no log - 1: simulation script logs - 2: all logs)",
                 verbose);
    cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);
    cmd.AddValue("useRts", "Enable/disable RTS/CTS", useRts);
    cmd.AddValue("nStas", "number of contending stations to send uplink traffic", nStas);
    cmd.AddValue("load", "offered load of the network", load);
    cmd.AddValue("infra", "True to use infrastructure mode, false to use ring adhoc mode", infra);
    cmd.AddValue("queueSize", "Size of the MAC queue at STAs", queueSize);
    cmd.AddValue("seed", "seed value for this simulation", seed);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(seed);
    if (useRts)
    {
        Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("0"));
    }
    
    Config::SetDefault("ns3::WifiRemoteStationManager::MaxSlrc", UintegerValue(7));
    Config::SetDefault("ns3::WifiRemoteStationManager::MaxSsrc", UintegerValue(7));
    Config::SetDefault("ns3::WifiMacQueue::MaxSize", QueueSizeValue(QueueSize(QueueSizeUnit::PACKETS, queueSize)));

    std::cout << "Nodes\tLoad\tNet_norm_load\tseed\tNet_norm_thrpt\tp_col\n";
    NodeContainer wifiNodes;
    wifiNodes.Create(nStas+1);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.DisablePreambleDetectionModel();
    phy.SetChannel(channel.Create());
    WifiMacHelper mac;
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                    "DataMode",
                                    StringValue(dataMode),
                                    "ControlMode",
                                    StringValue(ctrlMode));
    NetDeviceContainer devices;
    if (infra){
        Ssid ssid = Ssid("ns3-80211b");
        mac.SetType("ns3::ApWifiMac",
                    "EnableBeaconJitter",
                    BooleanValue(false),
                    "Ssid",
                    SsidValue(ssid));
        devices = wifi.Install(phy, mac, wifiNodes.Get(0));
        mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
        for (int i = 1; i <= nStas; i++)
            devices.Add(wifi.Install(phy, mac, wifiNodes.Get(i)));
    }
    else{
        mac.SetType("ns3::AdhocWifiMac");
        devices = wifi.Install(phy, mac, wifiNodes);
    }

    // mobility.
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();

    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    positionAlloc->Add(Vector(distance, 0.0, 0.0));
    mobility.SetPositionAllocator(positionAlloc);

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    // mobility.Install(wifiApNode);
    mobility.Install(wifiNodes);

    /* Internet stack*/
    InternetStackHelper stack;
    // stack.Install(wifiApNode);
    stack.Install(wifiNodes);

    Ipv4AddressHelper address;
    address.SetBase("192.168.1.0", "255.255.255.0");
    Ipv4InterfaceContainer nodeInterface;
    // Ipv4InterfaceContainer apNodeInterface;

    nodeInterface = address.Assign(devices);
    // apNodeInterface = address.Assign(apDevice);

    /* Setting applications */
    ApplicationContainer serverApp;
    
    // UDP flow
    uint16_t port = 9;
    UdpServerHelper server(port);
    serverApp = server.Install(wifiNodes.Get(0));
    serverApp.Start(Seconds(0.0));
    serverApp.Stop(Seconds(simulationTime + 1));
    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
    uv->SetAttribute ("Min", DoubleValue (0.0));
    uv->SetAttribute ("Max", DoubleValue (0.01));
    double net_load = load / get_T_success(payloadSize); // convert the load according to Liu's paper
    double pktInterval = 1 / (net_load / nStas);
    for(int i = 1; i <= nStas; i++){
        UdpClientHelper client(nodeInterface.GetAddress(0), port);
        client.SetAttribute("MaxPackets", UintegerValue(4294967295U));
        client.SetAttribute("Interval", TimeValue(Seconds(pktInterval))); // packets/s
        client.SetAttribute("PacketSize", UintegerValue(payloadSize));
        ApplicationContainer clientApp = client.Install(wifiNodes.Get(i));
        clientApp.Start(Seconds(1.0 + uv->GetValue())); //randomize app start time
        clientApp.Stop(Seconds(simulationTime + 1));
    }
    if (verbose >= 1)
    {
        LogComponentEnable("wifi_11b", LOG_LEVEL_ALL);
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
    }
    else
    {
        LogComponentEnable("wifi_11b", LOG_LEVEL_WARN);
    }
    if (verbose >= 2)
    {
        WifiHelper::EnableLogComponents();
    }
    // Log packet receptions
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/MonitorSnifferRx",
        MakeCallback(&TracePacketReception));
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();
        std::string txop = StaticCast<WifiNetDevice>(wifiNodes.Get(0)->GetDevice(0))->GetMac()->GetQosSupported()
        ? "BE_Txop"
        : "Txop";
    // Trace CW evolution
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::WifiMac/" + txop +
                        "/CwTrace",
                    MakeCallback(&CwTrace));
    // Trace backoff evolution
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::WifiMac/" + txop +
                        "/BackoffTrace",
                    MakeCallback(&BackoffTrace));
    // Trace PHY Tx start events
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyTxBegin",
                    MakeCallback(&PhyTxTrace));
    // Trace PHY Tx end events
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyTxEnd",
                    MakeCallback(&PhyTxDoneTrace));
    // Trace PHY Rx start events
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxBegin",
                    MakeCallback(&PhyRxTrace));
    // Trace PHY Rx payload start events
    Config::Connect(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxPayloadBegin",
        MakeCallback(&PhyRxPayloadTrace));
    // Trace PHY Rx drop events
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxDrop",
                    MakeCallback(&PhyRxDropTrace));
    // Trace PHY Rx end events
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxEnd",
                    MakeCallback(&PhyRxDoneTrace));
    // Trace PHY Rx error events
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/State/RxError",
                    MakeCallback(&PhyRxErrorTrace));
    // Trace PHY Rx success events
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/State/RxOk",
                    MakeCallback(&PhyRxOkTrace));
    // Trace packet transmission by the device
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&MacTxTrace));
    // Trace packet receptions to the device
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx", MakeCallback(&MacRxTrace));
    // Trace packets transmitted by the application
    Simulator::Stop(Seconds(simulationTime + 1));
    
    Simulator::Run();

    uint64_t rxBytes = 0;
    rxBytes = payloadSize * DynamicCast<UdpServer>(serverApp.Get(0))->GetReceived();
    double throughput = (rxBytes * 8) / (simulationTime * 1000000.0); // Mbit/s
    double p_col = GetCollisionProb();
    Simulator::Destroy();
    RestartCalc();

    std::cout << nStas << "\t" << load << "\t" << net_load * payloadSize * 8.0/11.0e6 << "\t" << seed << "\t" << throughput/11.0 << "\t" << p_col << std::endl;

    return 0;
}
