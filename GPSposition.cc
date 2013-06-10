/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 The Boeing Company
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
 * Author: Guangyu Pei <guangyu.pei@boeing.com>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/tools-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <math.h>

NS_LOG_COMPONENT_DEFINE ("Main");

using namespace ns3;

class Experiment
{
public:
  Experiment ();
  Experiment (std::string name);
  uint32_t Run (const WifiHelper &wifi, const YansWifiPhyHelper &wifiPhy,
                const NqosWifiMacHelper &wifiMacSta, 
                const NqosWifiMacHelper &wifiMacAp,
                const YansWifiChannelHelper &wifiChannel,double xPos);
private:
  void ReceivePacket (Ptr<Socket> socket);
  void SetPosition (Ptr<Node> node, Vector position);
  Vector GetPosition (Ptr<Node> node);
  Ptr<Socket> SetupPacketReceive (Ptr<Node> node);
  void GenerateTraffic (Ptr<Socket> socket, Ptr<Packet> packet, uint32_t pktSize, 
                        uint32_t pktCount, Time pktInterval );

  void ShowPosition(Ptr<Node> node, double deltaTime);
  void ShowNodeInformation(NodeContainer c, int numOfNode);
  uint32_t m_pktsTotal;
  Gnuplot2dDataset m_output;

  double m_prevPos;
  uint32_t m_pktCount;
};

Experiment::Experiment ()
{
  m_prevPos=0;
  m_pktCount=0;
}

Experiment::Experiment (std::string name)
  : m_output (name)
{
  m_output.SetStyle (Gnuplot2dDataset::LINES);
}

void
Experiment::SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

Vector
Experiment::GetPosition (Ptr<Node> node)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  return mobility->GetPosition ();
}

void
Experiment::ShowPosition(Ptr<Node> node, double deltaTime)
{
  /* dummy, reserved for debugging */
  return;
}

void
Experiment::ShowNodeInformation(NodeContainer c, int numOfNode)
{
   
  for (int i = 0 ; i < numOfNode ; ++i){
    Ptr<MobilityModel> mobility = c.Get(i)->GetObject<MobilityModel> ();
    Vector nodePos = mobility->GetPosition ();
    Ptr<Ipv4> ipv4 = c.Get(i)->GetObject<Ipv4> (); // Get Ipv4 instance of the node
    Ipv4Address addr = ipv4->GetAddress (1, 0).GetLocal (); // Get Ipv4InterfaceAddress of xth interface.
    std::cout << c.Get(i)->GetId() << " " << addr << " (" << nodePos.x << ", " <<  
                 nodePos.y << ")" << std::endl;
  }
}

void
Experiment::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while ((packet = socket->Recv ()))
    {
      //std::cout<< (char*)packet->PeekData() << std::endl;
      double *buffer = new double[3];
      //std::stringstream buffer;
      packet->CopyData((uint8_t*)buffer, 3*sizeof(*buffer));
      Ptr<Node> associateNode = socket->GetNode();
      Ptr<Ipv4> ipv4 = associateNode->GetObject<Ipv4> (); // Get Ipv4 instance of the node
      Ipv4Address addr = ipv4->GetAddress (1, 0).GetLocal (); // Get Ipv4InterfaceAddress of xth interface.
      std::cout << "Node-" << associateNode->GetId() << " " << addr
                << " receive GPS position from Node-" 
                << buffer[2] << "(" << buffer[0] <<", " << buffer[1] << ") at " <<
                Simulator::Now ().GetSeconds () << " second" << std::endl;
      m_pktsTotal++;
      m_pktCount++;
      delete[] buffer;
    }
}

Ptr<Socket>
Experiment::SetupPacketReceive (Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback (&Experiment::ReceivePacket, this));
  return sink;
}

void
Experiment::GenerateTraffic (Ptr<Socket> socket, Ptr<Packet> packet, uint32_t pktSize, 
                             uint32_t pktCount, Time pktInterval )
{
 
  if (pktCount > 0)
    {      
      socket->Send (packet);
      Simulator::Schedule (pktInterval, &Experiment::GenerateTraffic, this, 
                           socket, packet , pktSize, pktCount-1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

uint32_t
Experiment::Run (const WifiHelper &wifi, const YansWifiPhyHelper &wifiPhy,
                 const NqosWifiMacHelper &wifiMacSta, // for different type of mac station 
                 const NqosWifiMacHelper &wifiMacAp,  // for different type of mac station
                 const YansWifiChannelHelper &wifiChannel,double xPos)
{
  m_pktsTotal = 0;
  int numOfNode = 10;
  NodeContainer c;
  c.Create (numOfNode);

  InternetStackHelper internet;
  internet.Install (c);

  YansWifiPhyHelper phy = wifiPhy;
  phy.SetChannel (wifiChannel.Create ());


  NetDeviceContainer devices;
  devices.Add( wifi.Install (phy, wifiMacAp, c.Get(0))); // AP
  for (int i = 1 ; i < numOfNode ; ++i) {
    devices.Add( wifi.Install (phy, wifiMacAp, c.Get(i)));
  }
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector(0, 0, 0)); // AP
  for (int i = 1 ; i < numOfNode ; ++i) {
    positionAlloc->Add (Vector (rand()%100, rand()%100, rand()%100));
  }
  mobility.SetPositionAllocator (positionAlloc);
 
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  for (int i = 0 ; i < numOfNode ; ++i){
    mobility.Install (c.Get(i));  
  }

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);
  
  std::vector <Ptr<Socket> > vecRecvSink;
  for (int i = 0 ; i < numOfNode ; ++i){
    Ptr<Socket> recvSink = SetupPacketReceive (c.Get (i));
    vecRecvSink.push_back(recvSink);
  }
  
  ShowNodeInformation(c, numOfNode);
  
  for (int i = 1 ; i < numOfNode ; ++i){
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    Ptr<Socket> source = Socket::CreateSocket (c.Get (i), tid);
    InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
    source->SetAllowBroadcast (true);
    source->Connect (remote);
    Ptr<MobilityModel> mobility = c.Get(i)->GetObject<MobilityModel> ();
    Vector nodePos = mobility->GetPosition ();
    double pos[3] = {nodePos.x, nodePos.y, (double)i};
    Ptr<Packet> packet = Create<Packet>((uint8_t*)pos,3*sizeof(*pos));
    source->Send (packet);
    uint32_t packetSize = 1014;
    uint32_t maxPacketCount = 10;
    Time interPacketInterval = Seconds ((double)i*0.1);
    Simulator::Schedule (Seconds ((double)i*0.1), &Experiment::GenerateTraffic, 
                       this, source, packet, packetSize, maxPacketCount, interPacketInterval);
  }
  /* Setup the stop time */
  Simulator::Stop(Seconds(10.0));

  Simulator::Run ();

  Simulator::Destroy ();

  return m_pktsTotal;
}

int main (int argc, char *argv[])
{
  std::ofstream outfile ("clear-channel.plt");
  std::vector <std::string> modes;

  /* according to the  */
  // disable fragmentation
  //Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("100"));

  CommandLine cmd;
  cmd.Parse (argc, argv);

  Gnuplot gnuplot = Gnuplot ("clear-channel.eps");

  Experiment experiment;

  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211g);
  //Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode");
  /* To change the wifi rate adaptation algorithm , rate should not to be 
   * specisfied */

  wifi.SetRemoteStationManager ("ns3::MinstrelWifiManager");

  // setup stas.
  NqosWifiMacHelper wifiMacSta = NqosWifiMacHelper::Default ();
  Ssid ssid = Ssid ("wifi-default");
  wifiMacSta.SetType ("ns3::StaWifiMac",
                   "Ssid", SsidValue (ssid),
                   "ActiveProbing", BooleanValue (false));
  // setup ap.
  NqosWifiMacHelper wifiMacAp = NqosWifiMacHelper::Default();
  wifiMacSta.SetType ("ns3::ApWifiMac",
                   "Ssid", SsidValue (ssid));

  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  /* Fixed Rss loss model simply fix the receive power which is not feasible here*/
  /* Change to the free space model*/
  wifiChannel.AddPropagationLoss ("ns3::TwoRayGroundPropagationLossModel",
                                  "Lambda", DoubleValue(300000000.0/2.400e9),
                                  "SystemLoss",DoubleValue (1.0),
                                  "MinDistance",DoubleValue (0.5),
                                  "HeightAboveZ",DoubleValue (1) );
  //NS_LOG_DEBUG (modes[i]);
  experiment = Experiment ();
  wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-110.0) );
  wifiPhy.Set ("CcaMode1Threshold", DoubleValue (-110.0) );
  wifiPhy.Set ("TxPowerStart", DoubleValue (15.0) );
  wifiPhy.Set ("TxPowerEnd", DoubleValue (15.0) );
  wifiPhy.Set ("RxGain", DoubleValue (0) );
  wifiPhy.Set ("RxNoiseFigure", DoubleValue (7) );
  uint32_t pktsRecvd = experiment.Run (wifi, wifiPhy, wifiMacSta,wifiMacAp, wifiChannel, 100.0);
  std::cout << "total receive: " << pktsRecvd << std::endl;
//  gnuplot.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
//  gnuplot.SetLegend ("RSS(dBm)", "Number of packets received");
//  gnuplot.SetExtra  ("set xrange [-102:-83]");
//  gnuplot.GenerateOutput (outfile);
//  outfile.close ();

  return 0;
}
