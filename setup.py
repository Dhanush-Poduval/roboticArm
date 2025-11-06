import ns.network
import ns.wifi
import ns.mobility
import ns.internet
import ns.applications
import ns.core

# --- Create nodes: PC (0), Relay (1), Robot (2)
nodes = ns.network.NodeContainer()
nodes.Create(3)

# --- Wi-Fi Channel setup for Hop 1 (Base <-> Relay)
wifi1 = ns.wifi.WifiHelper.Default()
phy1 = ns.wifi.YansWifiPhyHelper.Default()
channel1 = ns.wifi.YansWifiChannelHelper.Default()
phy1.SetChannel(channel1.Create())

mac1 = ns.wifi.WifiMacHelper()
ssid1 = ns.wifi.Ssid("Hop1")
mac1.SetType("ns3::StaWifiMac", "Ssid", ns.wifi.SsidValue(ssid1))

# Base = AP, Relay = STA
wifi1.SetRemoteStationManager("ns3::AarfWifiManager")
apMac1 = ns.wifi.WifiMacHelper()
apMac1.SetType("ns3::ApWifiMac", "Ssid", ns.wifi.SsidValue(ssid1))

# Install on nodes
devices1_ap = wifi1.Install(phy1, apMac1, nodes.Get(0))   # PC/Base
devices1_sta = wifi1.Install(phy1, mac1, nodes.Get(1))    # Relay

# --- Wi-Fi Channel setup for Hop 2 (Relay <-> Robot)
wifi2 = ns.wifi.WifiHelper.Default()
phy2 = ns.wifi.YansWifiPhyHelper.Default()
channel2 = ns.wifi.YansWifiChannelHelper.Default()
phy2.SetChannel(channel2.Create())

mac2 = ns.wifi.WifiMacHelper()
ssid2 = ns.wifi.Ssid("Hop2")
mac2.SetType("ns3::StaWifiMac", "Ssid", ns.wifi.SsidValue(ssid2))
apMac2 = ns.wifi.WifiMacHelper()
apMac2.SetType("ns3::ApWifiMac", "Ssid", ns.wifi.SsidValue(ssid2))

devices2_ap = wifi2.Install(phy2, apMac2, nodes.Get(1))   # Relay acts as AP
devices2_sta = wifi2.Install(phy2, mac2, nodes.Get(2))    # Robot

# --- Internet stack + IPs
stack = ns.internet.InternetStackHelper()
stack.Install(nodes)

address = ns.internet.Ipv4AddressHelper()
address.SetBase(ns.network.Ipv4Address("10.0.1.0"), ns.network.Ipv4Mask("255.255.255.0"))
interfaces1 = address.Assign(ns.network.NetDeviceContainer(devices1_ap, devices1_sta))

address.SetBase(ns.network.Ipv4Address("10.0.2.0"), ns.network.Ipv4Mask("255.255.255.0"))
interfaces2 = address.Assign(ns.network.NetDeviceContainer(devices2_ap, devices2_sta))

# --- Applications (simulate your UDP telemetry)
# Robot -> Base (UDP)
port = 5005
udp_sender = ns.applications.UdpEchoClientHelper(interfaces1.GetAddress(0), port)
udp_sender.SetAttribute("MaxPackets", ns.core.UintegerValue(5))
udp_sender.SetAttribute("Interval", ns.core.TimeValue(ns.core.Seconds(2)))
udp_sender.SetAttribute("PacketSize", ns.core.UintegerValue(64))
app_sender = udp_sender.Install(nodes.Get(2))
app_sender.Start(ns.core.Seconds(1.0))
app_sender.Stop(ns.core.Seconds(10.0))

udp_server = ns.applications.UdpEchoServerHelper(port)
app_server = udp_server.Install(nodes.Get(0))
app_server.Start(ns.core.Seconds(0.5))
app_server.Stop(ns.core.Seconds(10.0))

# --- Run simulation
ns.core.Simulator.Stop(ns.core.Seconds(10))
ns.core.Simulator.Run()
ns.core.Simulator.Destroy()
