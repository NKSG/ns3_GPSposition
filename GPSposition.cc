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
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/athstats-helper.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <math.h>
#include <set>
#include <cmath>

NS_LOG_COMPONENT_DEFINE ("Main");

using namespace ns3;

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

std::vector<std::string> split(const std::string &s, char delim) {
  std::vector<std::string> elems;
  split(s, delim, elems);
  return elems;
}

bool comparePair(std::pair<int, std::pair<int,double> > a, std::pair<int,std::pair<int,double> > b)
{
  return a.second.second < b.second.second;
}


class Experiment
{
public:
  Experiment ();
  Experiment (std::string name);
  uint32_t Run (const WifiHelper &wifi, const YansWifiPhyHelper &wifiPhy,
                const NqosWifiMacHelper &wifiMacSta, 
                const NqosWifiMacHelper &wifiMacAp,
                const YansWifiChannelHelper &wifiChannel,double xPos);
  inline uint32_t getPktSent(){ return m_pktSend; }
  void PhyRxOkTrace (std::string context, Ptr<const Packet> packet, double snr, 
      WifiMode mode, enum WifiPreamble preamble);
  
private:
  void ReceivePacket (Ptr<Socket> socket);
  void SetPosition (Ptr<Node> node, Vector position);
  Vector GetPosition (Ptr<Node> node);
  Ptr<Socket> SetupPacketReceive (Ptr<Node> node);
  void GenerateTraffic (Ptr<Node> node, int id, Ptr<Socket> socket, 
                        uint32_t pktCount, Time pktInterval );

  void ShowPosition(Ptr<Node> node, double deltaTime);
  void ShowNodeInformation(NodeContainer c, int numOfNode);
  std::vector<double> RtsSnrThrldAssign(NodeContainer GpsContainer, int numOfGpsNode );
  uint32_t m_pktsTotal;
  Gnuplot2dDataset m_output;

  double m_prevPos;
  uint32_t m_pktCount;
  uint32_t m_pktSend;
  //double m_rtsSnrThrld;
  int m_numOfGpsNode;
  int m_numOfRtsNode;
  uint32_t m_pktSize;
  NetDeviceContainer devices;
  //std::vector<std::vector<uint8_t* > >  m_nodeListenList;
  std::vector<std::set<uint32_t> > m_receiveRtsCount;
  std::vector<std::pair<double,double> > m_receiveGpsInfo;
  std::vector<double> m_rtsSnrThrld;
  std::vector<double> m_sensingRadius;
};

Experiment::Experiment ()
{
  m_prevPos = 0;
  m_pktCount = 0;
  m_pktSend = 0;
  //m_rtsSnrThrld = 1000.0;
  m_numOfRtsNode = 20;
  m_numOfGpsNode = 8;
  m_pktSize = 1014;
  m_sensingRadius.resize(m_numOfGpsNode);
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
    std::cout << std::setw(5) << nodePos.x << std::setw(5) << nodePos.y << std::endl;
    //Ptr<Ipv4> ipv4 = c.Get(i)->GetObject<Ipv4> (); // Get Ipv4 instance of the node
    // Ptr<MacLow> mac48 = c.Get(i)->GetObject<MacLow> (); // Get Ipv4 instance of the node
    //Ipv4Address addr = ipv4->GetAddress (1, 0).GetLocal (); // Get Ipv4InterfaceAddress of xth interface.
    // Mac48Address macAddr = mac48->GetAddress();
    // std::cout << c.Get(i)->GetId() << " " << macAddr << " (" << nodePos.x << ", " <<  
    //              nodePos.y << ")" << std::endl;
  }
}

std::vector<double>
Experiment::RtsSnrThrldAssign( NodeContainer GpsContainer, int numOfGpsNode )
{
  Vector nodePos;
   for (int i = 0;  i < numOfGpsNode; i++) {
     nodePos = GpsContainer.Get(i)->GetObject<MobilityModel>()->GetPosition();
   }
   
  Vector pos1, pos2;
  double distance, dx, dy, minDistance;
  int minNode;
  std::set<int> remainNodeList;
  std::set<int>::iterator it;
  std::vector<std::pair<int, std::pair<int,double> > > minDisNodeList(numOfGpsNode);   
  std::vector<std::pair<int, double> > assignedNode;
  for (int i = 0; i < numOfGpsNode; i++) {
    remainNodeList.insert(i);
  }

  for (int i = 0; i < numOfGpsNode ; i++) {
    pos1 = GpsContainer.Get(i)->GetObject<MobilityModel>()->GetPosition();
    minDistance = 1000;
    minNode = -1;

    for ( int  j = 0; j < numOfGpsNode; j++) {
      if ( j == i ) continue;
      pos2 = GpsContainer.Get(j)->GetObject<MobilityModel>()->GetPosition();
      dx = pos2.x - pos1.x; 
      dy = pos2.y - pos1.y;
      distance = sqrt(dx*dx + dy*dy);
      if ( distance < minDistance ) {
        minDistance = distance;
        minNode = j;
        
      }
    }
    minDisNodeList[i].first = i;
    minDisNodeList[i].second.first = minNode;
    minDisNodeList[i].second.second = minDistance;
  }
//  for (size_t i = 0; i < minDisNodeList.size(); i++) {
//    std::cout << minDisNodeList[i].first << ": " << minDisNodeList[i].second.first << ' ' << minDisNodeList[i].second.second << ' ';
//  }
//  std::cout << std::endl;
  std::sort(minDisNodeList.begin(),minDisNodeList.end(),comparePair);

  assignedNode.push_back(std::make_pair<int,double>(minDisNodeList[0].first, 
        minDisNodeList[0].second.second/2.0));
  assignedNode.push_back(std::make_pair<int,double>(minDisNodeList[0].second.first, 
        minDisNodeList[0].second.second/2.0));
  remainNodeList.erase(minDisNodeList[0].first);
  remainNodeList.erase(minDisNodeList[0].second.first);

  int idx1, idx2, minInsideNode=-1,  minOutsideNode;
  double radius, assignedRadius;

  while( assignedNode.size() != (size_t)numOfGpsNode ) {
    minDistance = 100000.0;
    minNode = -1;
    assignedRadius = 100000.0;
    for (size_t i = 0; i < assignedNode.size(); i++) {
      idx1 = assignedNode[i].first;
      pos1 = GpsContainer.Get(idx1)->GetObject<MobilityModel>()->GetPosition();

      for (it = remainNodeList.begin(); it != remainNodeList.end(); it++) {
        idx2 = *it;
        pos2 = GpsContainer.Get(idx2)->GetObject<MobilityModel>()->GetPosition(); 
        dx = pos2.x - pos1.x; 
        dy = pos2.y - pos1.y;
        distance = sqrt(dx*dx+dy*dy);
        if ( distance < minDistance ) {
          minDistance = distance;
          minInsideNode  = idx1;
          minOutsideNode = idx2;
          assignedRadius = assignedNode[i].second; 
        }
      }

    }
    remainNodeList.erase(minOutsideNode);
    radius = minDistance - assignedRadius;
    if ( radius < 0.0 ) {
      radius = minDistance/2.0;
      size_t i;
      for (i = 0; i < assignedNode.size(); i++) {
        if ( assignedNode[i].first == minInsideNode ) {
          assignedNode[i].second = radius; 
          break;
        }
      }
      assignedNode.push_back(std::make_pair<int,double>(minOutsideNode,radius));
    }
    else {
      assignedNode.push_back(std::make_pair<int,double>(minOutsideNode,radius));
    }
  }
//  for (size_t i = 0; i < assignedNode.size(); i++) {
//    std::cout << assignedNode[i].first << ": " << assignedNode[i].second << std::endl;
//  }

  std::vector<double> rtsSnrThrld(numOfGpsNode);
  for (int i = 0; i < numOfGpsNode; i++) {
    rtsSnrThrld[assignedNode[i].first] = pow(10,(15.0-10*log10(assignedNode[i].second/1000.0))/9.0);
    m_sensingRadius[ assignedNode[i].first ] = assignedNode[i].second;
//    std::cout << 15.0-40.0*log10(assignedNode[i].second)+482.14 << ' ';
  }
//  std::cout << std::endl;


  return rtsSnrThrld;
  

}

void
Experiment::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while ((packet = socket->Recv ()))
    {
      //std::cout<< (char*)packet->PeekData() << std::endl;
      double *buffer = new double[5];
      //std::stringstream buffer;
      packet->CopyData((uint8_t*)buffer, 5*sizeof(*buffer));
      if ((uint8_t)buffer[2] == 1) {
        std::cout << std::setw(5) << buffer[0] 
                  << std::setw(5) << buffer[1] 
                  << std::setw(5) << (int)buffer[3] 
                  << std::setw(10) << m_sensingRadius[(int)buffer[4]-1]
                  << std::setw(14) << m_rtsSnrThrld[(int)buffer[4]-1]
                  << std::endl; 
        m_receiveGpsInfo[int(buffer[4])] = std::pair<double,double>(buffer[0],buffer[1]);
      }
      m_pktsTotal++;
      m_pktCount++;
//      delete[] buffer;
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
Experiment::PhyRxOkTrace (std::string context, Ptr<const Packet> packet,
 double snr, WifiMode mode, enum WifiPreamble preamble)
{
  Ptr<Packet> m_currentPacket;
  WifiMacHeader hdr;
  m_currentPacket = packet->Copy();
  m_currentPacket->RemoveHeader (hdr);

  std::vector<std::string> x = split(context,'/');
  std::stringstream ss;
  int nodeId;
  uint8_t buff[6];
  ss << x[2];
  ss >> nodeId; 
  hdr.GetAddr2().CopyTo(buff);
//  for (int i = 0; i < 6; i++) {
//    std::cout << (int)buffer[i]; 
//  }
//  std::cout << std::endl;
  //std::cout << nodeId << std::endl;

  // std:: cout << "recv snr = " <<  snr << " ; thrld = " << m_rtsSnrThrld << std::endl;
  uint32_t var = (buff[2]<<24)|(buff[3]<<16)|(buff[4]<<8)|buff[5];  
  //if ( hdr.IsRts() && snr > m_rtsSnrThrld[nodeId-1] && (int)var > m_numOfGpsNode + 1)
  if ( hdr.IsRts() && snr > m_rtsSnrThrld[nodeId-1] )
  {
    // std::cout << "rtsAddr = " << hdr.GetAddr2();
    //std::cout << var << std::endl;
    m_receiveRtsCount[nodeId].insert(var);
    //m_receiveRtsCount[nodeId]++;
    //std::cout << x[2] << std::endl;
     //std::cout << "Dst: " << hdr.GetAddr1() << " Src: " << hdr.GetAddr2() << ' ' << snr << std::endl;
  }
//    uint32_t var = (buff[2]<<24)|(buff[3]<<16)|(buff[4]<<8)|buff[5];  
//  if (nodeId == 0 && var != 1) {
//     std::cout << "Dst: " << hdr.GetAddr1() << " Src: " << hdr.GetAddr2() << ' ' << snr << std::endl;
//  }
}

void
Experiment::GenerateTraffic (Ptr<Node> node, int id, Ptr<Socket> socket,
                             uint32_t pktCount, Time pktInterval )
{

  Vector nodePos = node->GetObject<MobilityModel> () -> GetPosition();
  double content[5] = {nodePos.x, nodePos.y, (double)2,-1.0, 0};

  bool isGpsNode = id < m_numOfGpsNode+1 && id != 0;
  if ( isGpsNode )
  {
    content[2] = (double)1;
    content[3] = (double)m_receiveRtsCount[id].size();
    content[4] = (double)id;
  }


  if ( pktCount > 1 )  {
    if ( isGpsNode ) {
      

      Ptr<Packet> packet = Create<Packet>((uint8_t*)content,m_pktSize);
      socket->Send (packet);
      ++m_pktSend;
      Simulator::Schedule (pktInterval, &Experiment::GenerateTraffic, this, 
                           node, id, socket, pktCount-1, pktInterval);
    }
    else {
      Ptr<Packet> packet = Create<Packet>((uint8_t*)content,m_pktSize);
      socket->Send (packet);
      ++m_pktSend;
      Simulator::Schedule (pktInterval, &Experiment::GenerateTraffic, this, 
                           node, id, socket, pktCount-1, pktInterval);
    }


  }
  else if ( pktCount > 0 && pktCount <= 1 ){

      Ptr<Packet> packet = Create<Packet>((uint8_t*)content,m_pktSize);
      socket->Send (packet);
      ++m_pktSend;
      Simulator::Schedule (pktInterval, &Experiment::GenerateTraffic, this, 
                           node, id, socket, pktCount-1, pktInterval);
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
  /* number of ap node is 1 */
  NodeContainer GpsContainer;
  NodeContainer RtsContainer;
  NodeContainer ApContainer;

  GpsContainer.Create(m_numOfGpsNode);
  RtsContainer.Create(m_numOfRtsNode);
  ApContainer.Create(1);


  InternetStackHelper internet;
  internet.Install (GpsContainer);
  internet.Install (RtsContainer);
  internet.Install (ApContainer);

  YansWifiPhyHelper phy = wifiPhy;
  phy.SetChannel (wifiChannel.Create ());

  /* Initialize the receiving information data structure */
  m_receiveRtsCount.resize( m_numOfRtsNode + m_numOfGpsNode + 1 );
  m_receiveGpsInfo.resize( m_numOfGpsNode + 1 );

  devices.Add( wifi.Install (phy, wifiMacAp, ApContainer.Get(0))); // AP
  for (int i = 0 ; i < m_numOfGpsNode ; ++i) {
    devices.Add( wifi.Install (phy, wifiMacSta, GpsContainer.Get(i)));
  }
  for (int i = 0; i < m_numOfRtsNode; ++i) {
    devices.Add( wifi.Install (phy, wifiMacSta, RtsContainer.Get(i)));
  }
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  Ptr<ListPositionAllocator> ApPosAlloc = CreateObject<ListPositionAllocator> ();
  Ptr<ListPositionAllocator> GpsPosAlloc = CreateObject<ListPositionAllocator> ();
  Ptr<ListPositionAllocator> RtsPosAlloc = CreateObject<ListPositionAllocator> ();

  ApPosAlloc->Add (Vector(0, 0, 0)); // AP
  mobility.SetPositionAllocator(ApPosAlloc);
  mobility.Install(ApContainer);

  for (int i = 0 ; i < m_numOfGpsNode ; ++i) {
    GpsPosAlloc->Add (Vector (rand()%250-125, rand()%250-125, 1));
  }
  mobility.SetPositionAllocator(GpsPosAlloc);
  for (int i = 0 ; i < m_numOfGpsNode ; ++i){
    mobility.Install (GpsContainer.Get(i));  
  }

  for (int i = 0; i < m_numOfRtsNode; i++) {
    RtsPosAlloc->Add (Vector (rand()%250-125, rand()%250-125,1));
  }
  mobility.SetPositionAllocator(RtsPosAlloc);
  for (int i = 0; i < m_numOfRtsNode; i++) {
    mobility.Install (RtsContainer.Get(i));  
  }

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);
  
//  std::vector <Ptr<Socket> > vecRecvSink;
//  for (int i = 0 ; i < numOfNode ; ++i){
//    Ptr<Socket> recvSink = SetupPacketReceive (.Get (i));
//    vecRecvSink.push_back(recvSink);
//  }
  Ptr<Socket> recvSink = SetupPacketReceive (ApContainer.Get (0));
  
  
  m_rtsSnrThrld = RtsSnrThrldAssign(GpsContainer,m_numOfGpsNode);
  /* output real topology */
  std::cout << "---------- real topology -----------" << std::endl;
  std::cout << "# AP " << std::endl;
  ShowNodeInformation(ApContainer, 1);
  std::cout << "# GPS " << std::endl;
  ShowNodeInformation(GpsContainer, m_numOfGpsNode);
  std::cout << "# RTS " << std::endl;
  ShowNodeInformation(RtsContainer, m_numOfRtsNode);
  std::cout << std::endl;

  
  std::cout << "---------- approx topology ----------" << std::endl;
  std::cout << "# AP " << std::endl;
  ShowNodeInformation(ApContainer, 1);
  std::cout << "# GPS " << std::endl; 
  //Time interPacketInterval = Seconds (1);
  for (uint8_t i = 0 ; i < m_numOfGpsNode ; ++i){
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    Ptr<Socket> source = Socket::CreateSocket (GpsContainer.Get (i), tid);
    InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
    source->SetAllowBroadcast (true);
    source->Connect (remote);
    uint32_t maxPacketCount = 1;
    Time interPacketInterval = Seconds ((double)i*0.1);
    Simulator::Schedule (Seconds ((double)i*0.1), &Experiment::GenerateTraffic, 
                       this, GpsContainer.Get(i), i+1, source, maxPacketCount, interPacketInterval);
  }

  for (uint8_t i = 0 ; i < m_numOfRtsNode ; ++i){
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    Ptr<Socket> source = Socket::CreateSocket (RtsContainer.Get (i), tid);
    InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
    source->SetAllowBroadcast (true);
    source->Connect (remote);
    uint32_t maxPacketCount = 12;
    Time interPacketInterval = Seconds ((double)i*0.1);
    Simulator::Schedule (Seconds ((double)i*0.1), &Experiment::GenerateTraffic, 
                       this, RtsContainer.Get(i), i+1+m_numOfGpsNode, source, maxPacketCount, interPacketInterval);

  }

  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxOk", 
                  MakeCallback (&Experiment::PhyRxOkTrace,this));
  
  /* Setup the stop time */
  Simulator::Stop(Seconds(50.0));

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
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("50"));
    

  int numOfGpsNodes = 0;
  int numOfRtsNodes = 0;
  CommandLine cmd;
  cmd.AddValue("nGpsNodes","Number of GPS node", numOfGpsNodes);
  cmd.AddValue("nRtsNodes","Number of Rts node", numOfRtsNodes);
  cmd.Parse (argc, argv);
  Gnuplot gnuplot = Gnuplot ("clear-channel.eps");

  std::cout << numOfGpsNodes << ' '<< numOfRtsNodes << std::endl;
  Experiment experiment;

  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211g);
  //Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode");
  /* To change the wifi rate adaptation algorithm , rate should not to be 
   * specisfied */
  std::string phyMode("ErpOfdmRate6Mbps");
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue(phyMode),"ControlMode",StringValue(phyMode));

  // setup stas.
  NqosWifiMacHelper wifiMacSta = NqosWifiMacHelper::Default ();
  Ssid ssid = Ssid ("wifi-default");
  wifiMacSta.SetType ("ns3::StaWifiMac",
                   "Ssid", SsidValue (ssid),
                   "ActiveProbing", BooleanValue (false));
  // setup ap.
  NqosWifiMacHelper wifiMacAp = NqosWifiMacHelper::Default();
  wifiMacAp.SetType ("ns3::ApWifiMac",
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
  std::cout << "total send: " << experiment.getPktSent() << std::endl;
//  gnuplot.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
//  gnuplot.SetLegend ("RSS(dBm)", "Number of packets received");
//  gnuplot.SetExtra  ("set xrange [-102:-83]");
//  gnuplot.GenerateOutput (outfile);
//  outfile.close ();

  return 0;
}
