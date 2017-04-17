/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008,2009 IITP RAS
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
 * Author: Kirill Andreev <andreev@iitp.ru>
 *
 *
 * By default this script creates m_xSize * m_ySize square grid topology with
 * IEEE802.11s stack installed at each node with peering management
 * and HWMP protocol.
 * The side of the square cell is defined by m_step parameter.
 * When topology is created, UDP ping is installed to opposite corners
 * by diagonals. packet size of the UDP ping and interval between two
 * successive packets is configurable.
 * 
 *  m_xSize * step
 *  |<--------->|
 *   step
 *  |<--->|
 *  * --- * --- * <---Ping sink  _
 *  | \   |   / |                ^
 *  |   \ | /   |                |
 *  * --- * --- * m_ySize * step |
 *  |   / | \   |                |
 *  | /   |   \ |                |
 *  * --- * --- *                _
 *  ^ Ping source
 *
 *  See also MeshTest::Configure to read more about configurable
 *  parameters.
 */


#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/trace-helper.h"

#include "ns3/energy-module.h"

//added lib
#include "ns3/gnuplot.h"
#include "ns3/flow-monitor-module.h"
#include <ns3/flow-monitor-helper.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <map>
#include <string>
#include <dirent.h>

using namespace ns3;

typedef std::vector<Ptr<BasicEnergySource> > esList;
esList m_esList;

struct SimInputs
{
  int minB;
  double gamma;
};

struct CbrConnection
{
  Mac48Address destination;
  Mac48Address source;
  uint8_t cnnType;
  Ipv4Address srcIpv4Addr;
  Ipv4Address dstIpv4Addr;
  uint16_t srcPort;
  uint16_t dstPort;
  Time whenExpires;
  Mac48Address prevMac;
  Mac48Address nextMac;
  inline bool operator == (const CbrConnection &o) const {
        return ( o.dstIpv4Addr == dstIpv4Addr && o.dstPort == dstPort && o.srcIpv4Addr == srcIpv4Addr && o.srcPort == srcPort );
                        //||    		   ( o.dstIpv4Addr == srcIpv4Addr && o.dstPort == srcPort && o.srcIpv4Addr == dstIpv4Addr && o.srcPort == dstPort );
  }
};

typedef std::vector<CbrConnection> CbrConnectionsVector;

static int simPeriods=30;//900

NS_LOG_COMPONENT_DEFINE ("TestMeshScript");
class MeshTest
{
public:
  /// Init test
  MeshTest ();

  void Txed();
  void WannaTx();
  void BufferredAtSource(Ptr<Packet> packet);
  void Rxed(Time delay,Ptr<Packet> packet);
  void RxedElastic(Ptr<const Packet> packet, const Address &address);
  void TcpCnnEstablished(uint32_t srcId,Address dstIp);
  void CbrConnectionStateChanged (Ipv4Address srcIp, Ipv4Address dstIp,uint16_t srcPort,uint16_t dstPort,bool establishedClosed);

  void Every1Second();

  void SimInputChange();

  int GetNodeIdFromIP(Ipv4Address ip);

  /// Configure test from command line arguments
  void Configure (int argc, char ** argv);
  std::string GetNthWord(std::string s, std::size_t n);
  /// Run test
  int Run ();
  struct nodeInfo {
          double remainingE;
          Time BT; ///Beacon Time
          Time BI; ///Beacon interval
  };
private:
  int       m_xSize;
  int       m_ySize;
  double    m_step;
  double    m_randomStart;
  double    m_totalTime;
  double    m_packetInterval;
  uint16_t  m_packetSize;
  uint32_t  m_nIfaces;
  bool      m_chan;
  bool      m_pcap;

  int m_seed;
  double m_initialEnergy;
  double m_gamma;

  int m_cbrDurSec;

  nodeInfo *ni;

  std::string m_stack;
  std::string m_root;
  /// List of network nodes
  NodeContainer nodes;
  /// List of all mesh point devices
  NetDeviceContainer meshDevices;
  //Addresses of interfaces:
  Ipv4InterfaceContainer interfaces;
  // MeshHelper. Report is not static methods
  MeshHelper mesh;
  std::string m_xml_prefix;

  std::string m_resFolder;

  int totalTxed;
  int totalWannaTx;
  int totalRxed;
  int lastTotalRxed;
  Time totalDelay;
  int totalCnns;
  int totalAcceptedCnns;

  int totalRxedElasticBytes;
  int lastTotalRxedElasticBytes;

  int deadNodesNumber;

  CbrConnectionsVector m_currentSystemCnns;

  std::ofstream resFile;
  std::ofstream remEFile;
  std::ofstream delayFile;
  std::ofstream connectionsFile;
  std::ofstream tcpConnectionsFile;
  std::ofstream tcpAllCnnsFile;
  std::ofstream cnnNumsFile;
  std::ofstream throughtputFile;
  std::ofstream deadNodesFile;

  std::ifstream initEFile;
  int num15minPeriods;
  std::ifstream simInputFile;
  std::vector<SimInputs> simInputs;
  std::vector<SimInputs>::iterator simInsIterator;

  std::vector<uint64_t> m_bufferredAtSourcePacketIds;

private:
  /// Create nodes and setup their mobility
  void CreateNodes ();
  /// Install internet m_stack on nodes
  void InstallInternetStack ();
  void StartUdpApp(int srcId,int dstId,double dur);
  void StartTcpApp(int srcId,int dstId,double dur);
  /// Install applications
  void InstallApplication ();
  /// Print mesh devices diagnostics
  void Report ();
  void PopulateArpCache ();
};
MeshTest::MeshTest () :
  m_xSize (4),
  m_ySize (4),
  m_step (100.0),
  m_randomStart (0.01),
  m_totalTime (150.0),
  m_packetInterval (0.02),
  m_packetSize (160),
  m_nIfaces (1),
  m_chan (true),
  m_pcap (false),
  m_seed(1),
  m_initialEnergy(0.027),
  m_gamma(0.053),
  m_cbrDurSec(300),
  m_stack ("ns3::Dot11sStack"),
  m_root ("ff:ff:ff:ff:ff:ff"),
  m_xml_prefix("mp-report-"),
  m_resFolder("/home/hadi/hadi_results/hadi_report_shen4x4/")
{
}

void MeshTest::Txed(){//Ptr<OutputStreamWrapper> stream
	totalTxed++;
}
void MeshTest::WannaTx(){
	totalWannaTx++;
}
void MeshTest::BufferredAtSource(Ptr<Packet> packet){
	m_bufferredAtSourcePacketIds.push_back(packet->GetUid());
}
void MeshTest::Rxed(Time delay,Ptr<Packet> packet) {
	//write delay in a file
	std::vector<uint64_t>::iterator pidit = std::find(m_bufferredAtSourcePacketIds.begin(),m_bufferredAtSourcePacketIds.end(),packet->GetUid());
	if(pidit != m_bufferredAtSourcePacketIds.end()){
		delayFile << Simulator::Now().GetSeconds() << " " << (int)packet->GetUid() << " " << (double)delay.GetMilliSeconds() << std::endl;
	}else{
		delayFile << Simulator::Now().GetSeconds() << " " << (int)packet->GetUid() << " " << (double)delay.GetMilliSeconds() << " " << (double)delay.GetMilliSeconds() << std::endl;
	}
	totalDelay = totalDelay + delay;
	totalRxed++;
	//at end totalRxed * m_packetSize + totalRxedElasticBytes is traffic volume => write to a file
	//and totalRxed / totalTxed is PDR of CBR traffics => write to a file
}

void MeshTest::RxedElastic(Ptr<const Packet> packet, const Address &address) {
	totalRxedElasticBytes += packet->GetSize();
	//std::cout << Simulator::Now().GetSeconds() << " elasticRxedFrom " << address << " " << (int)packet->GetSize() << std::endl;
}

void MeshTest::TcpCnnEstablished(uint32_t srcId,Address dstIp){
			uint8_t b[20];
			int id;
			dstIp.CopyTo(b);
			id=(int)b[3];
			id-=1;
	tcpConnectionsFile << (int)srcId << " " << id << " " << Simulator::Now().GetSeconds() << std::endl;
}

void MeshTest::CbrConnectionStateChanged (Ipv4Address srcIp, Ipv4Address dstIp,uint16_t srcPort,uint16_t dstPort,bool establishedClosed){
	CbrConnection cnn;
	CbrConnectionsVector::iterator it;
	cnn.srcIpv4Addr=srcIp;
	cnn.dstIpv4Addr=dstIp;
	cnn.srcPort=srcPort;
	cnn.dstPort=dstPort;
	if(establishedClosed){
		it= find(m_currentSystemCnns.begin(),m_currentSystemCnns.end(),cnn);
		if(it==m_currentSystemCnns.end()){
			cnn.whenExpires=Simulator::Now();
			m_currentSystemCnns.push_back(cnn);
			totalAcceptedCnns++;
			cnnNumsFile << Simulator::Now().GetSeconds() << " " << (int)m_currentSystemCnns.size() << " + " << GetNodeIdFromIP(srcIp) << " " << srcPort << " " << dstPort << std::endl;
			std::cout << Simulator::Now().GetSeconds() << " connectionEstablished " <<  GetNodeIdFromIP(srcIp) << " " << srcPort << " " << dstPort << std::endl;
		}
	}else{
		it= find(m_currentSystemCnns.begin(),m_currentSystemCnns.end(),cnn);
		if(it!=m_currentSystemCnns.end()){
			connectionsFile << GetNodeIdFromIP(srcIp) << " " << srcPort <<" " << GetNodeIdFromIP(dstIp) << " " << dstPort <<" " << it->whenExpires.GetSeconds() << " " << Simulator::Now().GetSeconds() << std::endl;
			m_currentSystemCnns.erase(it);
			cnnNumsFile << Simulator::Now().GetSeconds() << " " << (int)m_currentSystemCnns.size() << " - " << GetNodeIdFromIP(srcIp) << " " << srcPort << " " << dstPort << std::endl;
			std::cout << Simulator::Now().GetSeconds() << " connectionFinished " <<  GetNodeIdFromIP(srcIp) << " " << srcPort << " " << dstPort << std::endl;
		}
	}
	//write current simulation time and number of connections to a file  std::cout << Simulator::Now() << std::endl
}

void MeshTest::SimInputChange(){
  simInsIterator++;
  if(simInsIterator!=simInputs.end ()){
      m_gamma=simInsIterator->gamma;
      double minB=1500;
      int minBnode=0;
      remEFile << Simulator::Now().GetSeconds() << " ";
      for(int var=0;var<m_xSize*m_ySize;var++){
          nodes.Get(var)->GetDevice(0)->GetObject<MeshPointDevice>()->GetInterface(1)->GetObject<WifiNetDevice>()->GetMac()->GetObject<RegularWifiMac>()->SetGamma(m_gamma);
          //nodes.Get(var)->GetObject<BasicEnergySource>()->SetAttribute ("BasicEnergySourceGamma",DoubleValue(m_gamma));
          remEFile << ni[var].remainingE << " ";
          if(ni[var].remainingE<minB)
            {
              minB=ni[var].remainingE;
              minBnode=var;
            }
        }
      remEFile << "; " << minBnode << " " << minB << std::endl;
      for (esList::const_iterator i = m_esList.begin(); i != m_esList.end(); i++) {
              Ptr<BasicEnergySource> es = (*i);
              es->SetAttribute ("BasicEnergySourceGamma",DoubleValue(m_gamma));
        }
      Simulator::Schedule(Seconds(simPeriods), &MeshTest::SimInputChange,this);
    }

}

void MeshTest::Every1Second() {
  int elasticThroughtput = totalRxedElasticBytes - lastTotalRxedElasticBytes;
  int cbrThroughtput = (totalRxed - lastTotalRxed)*m_packetSize;
  int sumThroughtput = elasticThroughtput + cbrThroughtput;
  //write elasticThroughtput with current simulation time to a file
  throughtputFile << Simulator::Now().GetSeconds() << " " << elasticThroughtput << " " << cbrThroughtput << " " << sumThroughtput << std::endl;
  lastTotalRxedElasticBytes = totalRxedElasticBytes;
  lastTotalRxed = totalRxed;

  for (esList::const_iterator i = m_esList.begin(); i != m_esList.end(); i++) {
          Ptr<BasicEnergySource> es = (*i);
          double remE = es->GetRemainingEnergy();
          if((ni[es->GetNode()->GetId()].remainingE > 0.5)&&(remE<=0.5)){//minRemainingEnergy = 0.5
                  deadNodesNumber++;
                  //write current simulation time and dead nodes number to a file
                  deadNodesFile << Simulator::Now().GetSeconds() << " " << deadNodesNumber << std::endl;
                  if(deadNodesNumber==1){
                          //write current simulation time as the network lifetime to a file
                          resFile << Simulator::Now().GetSeconds() << " ";
                  }
          }
          //at end write min node energy to a file
          ni[es->GetNode()->GetId()].remainingE = remE;
  }
  Simulator::Schedule(Seconds(1), &MeshTest::Every1Second,this);
}

int MeshTest::GetNodeIdFromIP(Ipv4Address ip){
	uint8_t b[4];
	int id;
	ip.Serialize(b);
	id=(int)b[3];
	id-=1;
	return id;
}

void
MeshTest::PopulateArpCache ()
{
  Ptr<ArpCache> arp = CreateObject<ArpCache> ();
  arp->SetAliveTimeout (Seconds(3600 * 24 * 365));
  for (NodeList::Iterator i = NodeList::Begin(); i != NodeList::End(); ++i)
  {
    Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
    NS_ASSERT(ip !=0);
    ObjectVectorValue interfaces;
    ip->GetAttribute("InterfaceList", interfaces);
    for(ObjectVectorValue::Iterator ifaces = interfaces.Begin(); ifaces != interfaces.End (); ifaces ++)//ObjectVectorValue::Iterator
    {
      Ptr<Ipv4Interface> ipIface = ifaces->second->GetObject<Ipv4Interface>();
      NS_ASSERT(ipIface != 0);
      Ptr<NetDevice> device = ipIface->GetDevice();
      NS_ASSERT(device != 0);
      Mac48Address addr = Mac48Address::ConvertFrom(device->GetAddress ());
      for(uint32_t k = 0; k < ipIface->GetNAddresses (); k ++)
      {
        Ipv4Address ipAddr = ipIface->GetAddress (k).GetLocal();
        if(ipAddr == Ipv4Address::GetLoopback())
          continue;
        ArpCache::Entry * entry = arp->Add(ipAddr);
        entry->MarkWaitReply(0);
        entry->MarkAlive(addr);
      }
    }
  }
  for (NodeList::Iterator i = NodeList::Begin(); i != NodeList::End(); ++i)
  {
    Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
    NS_ASSERT(ip !=0);
    ObjectVectorValue interfaces;
    ip->GetAttribute("InterfaceList", interfaces);
    for(ObjectVectorValue::Iterator j = interfaces.Begin(); j != interfaces.End (); j ++)
    {
      Ptr<Ipv4Interface> ipIface = j->second->GetObject<Ipv4Interface> ();
      ipIface->SetAttribute("ArpCache", PointerValue(arp));
    }
  }
}

void
MeshTest::Configure (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.AddValue ("x-size", "Number of nodes in a row grid. [6]", m_xSize);
  cmd.AddValue ("y-size", "Number of rows in a grid. [6]", m_ySize);
  cmd.AddValue ("step",   "Size of edge in our grid, meters. [100 m]", m_step);
  /*
   * As soon as starting node means that it sends a beacon,
   * simultaneous start is not good.
   */
  cmd.AddValue ("start",  "Maximum random start delay, seconds. [0.1 s]", m_randomStart);
  cmd.AddValue ("time",  "Simulation time, seconds [100 s]", m_totalTime);
  cmd.AddValue ("packet-interval",  "Interval between packets in UDP ping, seconds [0.001 s]", m_packetInterval);
  cmd.AddValue ("packet-size",  "Size of packets in UDP ping", m_packetSize);
  cmd.AddValue ("interfaces", "Number of radio interfaces used by each mesh point. [1]", m_nIfaces);
  cmd.AddValue ("channels",   "Use different frequency channels for different interfaces. [0]", m_chan);
  cmd.AddValue ("pcap",   "Enable PCAP traces on interfaces. [0]", m_pcap);
  cmd.AddValue ("stack",  "Type of protocol stack. ns3::Dot11sStack by default", m_stack);
  cmd.AddValue ("root", "Mac address of root mesh point in HWMP", m_root);
  cmd.AddValue ("xml_prefix",  "prefix of xml file for outputs", m_xml_prefix);
  cmd.AddValue ("res_folder",  "prefix of xml file for outputs", m_resFolder);

  cmd.AddValue ("InitialEnergy",  "InitialEnergy of all nodes", m_initialEnergy);
  cmd.AddValue ("Gamma",  "Gamma of all nodes", m_gamma);
  cmd.AddValue ("Seed",  "seed", m_seed);
  cmd.AddValue ("CbrDurSec",  "CbrDurSec", m_cbrDurSec);

  cmd.Parse (argc, argv);
  NS_LOG_DEBUG ("Grid:" << m_xSize << "*" << m_ySize);
  NS_LOG_DEBUG ("Simulation time: " << m_totalTime << " s");
}
void
MeshTest::CreateNodes ()
{ 
  /*
   * Create m_ySize*m_xSize stations to form a grid topology
   */
  nodes.Create (m_ySize*m_xSize);
  // Configure YansWifiChannel
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  /*
   * Create mesh helper and set stack installer to it
   * Stack installer creates all needed protocols and install them to
   * mesh point device
   */
  mesh = MeshHelper::Default ();
  if (!Mac48Address (m_root.c_str ()).IsBroadcast ())
    {
      mesh.SetStackInstaller (m_stack, "Root", Mac48AddressValue (Mac48Address (m_root.c_str ())));
    }
  else
    {
      //If root is not set, we do not use "Root" attribute, because it
      //is specified only for 11s
      mesh.SetStackInstaller (m_stack);
    }
  if (m_chan)
    {
      mesh.SetSpreadInterfaceChannels (MeshHelper::SPREAD_CHANNELS);
    }
  else
    {
      mesh.SetSpreadInterfaceChannels (MeshHelper::ZERO_CHANNEL);
    }
  mesh.SetMacType ("RandomStart", TimeValue (Seconds (m_randomStart)));
  // Set number of interfaces - default is single-interface mesh point
  mesh.SetNumberOfInterfaces (m_nIfaces);
  // Install protocols and return container if MeshPointDevices
  meshDevices = mesh.Install (wifiPhy, nodes);
  // Setup mobility - static grid topology
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (m_step),
                                 "DeltaY", DoubleValue (m_step),
                                 "GridWidth", UintegerValue (m_xSize),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);

  /** Energy Model **/
  Ptr<Node> node;
  Ptr<BasicEnergySource> es;
  EnergySourceContainer::Iterator esi;
  /***************************************************************************/
  std::ostringstream os;
  os << "/home/hadi/hadi_results/hadi_report/seed" << m_seed << "initEFile.txt";

  initEFile.open (os.str ().c_str ());
  std::string tempStr,initEVal;
  if(initEFile.is_open ())
    {
      std::getline(initEFile,tempStr);
    }

  for (NodeContainer::Iterator i = nodes.Begin(); i < nodes.End(); i++) {
          node = *i;
          Ptr<WifiNetDevice> wnd;
          wnd = node->GetDevice(0)->GetObject<MeshPointDevice>()->GetInterface(1)->GetObject<WifiNetDevice>();

          /* energy source */
          BasicEnergySourceHelper basicSourceHelper;
          // configure energy source
          if(node->GetId()==0)
            {
                  basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ",DoubleValue(1500));
            }
          else
            {
                  initEVal=GetNthWord (tempStr,node->GetId ()+1);

		  if(initEVal=="")
		    basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ",DoubleValue(m_initialEnergy));
		  else
		    {
		      double initEDouble=std::atof(initEVal.c_str ());
		      basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ",DoubleValue(initEDouble));
		    }
	    }
          basicSourceHelper.Set ("BasicEnergySourceGamma",DoubleValue(m_gamma));
          // install source
          EnergySourceContainer sources = basicSourceHelper.Install (node);
          esi = sources.Begin();
          es = (*esi)->GetObject<BasicEnergySource>();
          /* device energy model */
          WifiRadioEnergyModelHelper radioEnergyHelper;
          radioEnergyHelper.Install(wnd, es);
          m_esList.push_back(es);
          // configure radio energy model
          //radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
          // install device model
          //DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (meshDevices, sources);
    }
  Simulator::Schedule(Seconds(1), &MeshTest::Every1Second,this);
  /***************************************************************************/

  for (int var = 0; var < m_xSize*m_ySize; ++var) {
          nodes.Get(var)->GetDevice(0)->GetObject<MeshPointDevice>()->GetInterface(1)->GetObject<WifiNetDevice>()->GetMac()->GetObject<RegularWifiMac>()->SetGamma(m_gamma);
  }

  if (m_pcap)
    wifiPhy.EnablePcapAll (std::string ("mp-"));
}
void
MeshTest::InstallInternetStack ()
{
  InternetStackHelper internetStack;
  internetStack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  interfaces = address.Assign (meshDevices);
}
void MeshTest::StartUdpApp(int srcId,int dstId,double dur){
	std::cout << Simulator::Now().GetSeconds() << " StartUdpApp from node " << srcId << " to " << dstId << std::endl;
	totalCnns++;
	UdpClientHelper echoClient(interfaces.GetAddress(dstId), 10000);
	echoClient.SetAttribute("MaxPackets",UintegerValue((uint32_t) (m_totalTime * (1 / m_packetInterval))));
	echoClient.SetAttribute("Interval", TimeValue(Seconds(m_packetInterval)));
	echoClient.SetAttribute("PacketSize", UintegerValue(m_packetSize));
	ApplicationContainer clientApps = echoClient.Install(nodes.Get(srcId));
	clientApps.Start(Seconds(0.0));
	clientApps.Stop(Seconds(dur));
}

void MeshTest::StartTcpApp(int srcId,int dstId,double dur){
	std::cout << Simulator::Now().GetSeconds() << " StartTcpApp from node " << srcId << " to " << dstId << std::endl;

	  tcpAllCnnsFile << srcId << " " << dstId << " " << Simulator::Now().GetSeconds() << std::endl;

	  BulkSendHelper source ("ns3::TcpSocketFactory", InetSocketAddress (interfaces.GetAddress (dstId), 9));
	  // Set the amount of data to send in bytes.  Zero is unlimited.
	  source.SetAttribute ("MaxBytes", UintegerValue (40000));
	  ApplicationContainer sourceApps = source.Install (nodes.Get (srcId));
	  sourceApps.Start (Seconds (0.0));
	  sourceApps.Stop (Seconds (70.0));

	  uint32_t n=sourceApps.GetN();

	  sourceApps.Get(n-1)->TraceConnectWithoutContext("Established",MakeCallback (&MeshTest::TcpCnnEstablished,this));

}
void
MeshTest::InstallApplication ()
{
	    LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
	    LogComponentEnable("UdpServer", LOG_LEVEL_INFO);

	    UniformVariable rng = UniformVariable();

	    for (int var = 0; var < m_xSize*m_ySize; ++var) {

                    UdpServerHelper server(10000);
                    ApplicationContainer serverApps = server.Install(nodes.Get(var));
                    serverApps.Start(Simulator::Now());
                    serverApps.Stop(Seconds(m_totalTime));

                    Ptr<UdpServer> us=server.GetServer();
                    us->TraceConnectWithoutContext("ReceivedAtDestination",MakeCallback(&MeshTest::Rxed,this));

                    //hadi eo94 - 4 critical connections
                    UdpServerHelper criticalServer(11000);
                    ApplicationContainer criticalServerApps = criticalServer.Install(nodes.Get(var));
                    criticalServerApps.Start(Simulator::Now());
                    criticalServerApps.Stop(Seconds(m_totalTime));

                    Ptr<UdpServer> ucriticalS=criticalServer.GetServer();
                    ucriticalS->TraceConnectWithoutContext("ReceivedAtDestination",MakeCallback(&MeshTest::Rxed,this));
                    //hadi eo94 - 4 critical connections

                    //
                    // Create a PacketSinkApplication and install it on node 1
                    //
                    PacketSinkHelper sink ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), 9));
                    ApplicationContainer sinkApps = sink.Install (nodes.Get (var));
                    sinkApps.Start (Seconds(0));
                    sinkApps.Stop (Seconds(m_totalTime)-Simulator::Now());
            }

            double du_time = 50;
            uint64_t st_time = 21;
            uint8_t node_src_num=0,node_dst_num;
            //int connection_Number=30;
            node_dst_num=0;


               ExponentialVariable cbrInterArrivalMilliseconds (500);//500
               ExponentialVariable cbrDurationSeconds (m_cbrDurSec);//40
               ExponentialVariable elasticInterArrivalMilliseconds (500);
               ExponentialVariable elasticDurationSeconds (10);
               st_time =Seconds(20).GetNanoSeconds();

                std::ostringstream os;
                os << m_resFolder << "seed" << m_seed << "/cnnsFile.txt";
               std::ofstream cnnsFile;
               cnnsFile.open (os.str ().c_str ());
                if (!cnnsFile.is_open ())
                  {
                    std::cerr << "Error: Can't open file cnnsFile.txt \n";
                    return;
                  }
            for (int i = 0; NanoSeconds(st_time).GetSeconds() < m_totalTime - 40 ; ++i) {
                    st_time =(NanoSeconds(st_time)+MilliSeconds(cbrInterArrivalMilliseconds.GetValue())).GetNanoSeconds();
                    du_time=m_totalTime;
                    while(NanoSeconds (st_time)+Seconds (du_time)>Seconds (m_totalTime-30))
                    //while(du_time>m_cbrDurSec)
                      du_time =  cbrDurationSeconds.GetValue();

                            node_src_num=rng.GetInteger(0,m_xSize*m_ySize-1);
                            while(node_src_num==node_dst_num)
                                    node_src_num=rng.GetInteger(0,m_xSize*m_ySize-1);
                            cnnsFile << st_time << " " << du_time << " " << (int)node_src_num << std::endl;
                            Simulator::Schedule(NanoSeconds(st_time),&MeshTest::StartUdpApp,this,node_src_num,node_dst_num,du_time);
            }

            cnnsFile.close();
            st_time = Seconds(20).GetNanoSeconds();
            for (int i = 0; NanoSeconds(st_time).GetSeconds() < m_totalTime - 50  ; ++i) {

                    //st_time = Seconds(20.25+i*0.5).GetNanoSeconds()+rng.GetValue(50000,500000);;//rng.GetValue(20.0,m_totalTime);  //((double) rand() / (RAND_MAX)) * m_totalTime+10; // 10-simulationDuration
                    st_time =(NanoSeconds(st_time)+MilliSeconds(elasticInterArrivalMilliseconds.GetValue())).GetNanoSeconds();
                    //du_time = 10;//rng.GetValue(100,200) ;   // 0-20
                    du_time =  elasticDurationSeconds.GetValue();
                            node_src_num=rng.GetInteger(0,m_xSize*m_ySize-1);
                            node_dst_num=node_src_num;
                            while(node_src_num==node_dst_num)
                                    node_dst_num=rng.GetInteger(0,m_xSize*m_ySize-1);

                            Simulator::Schedule(NanoSeconds(st_time),&MeshTest::StartTcpApp,this,node_src_num,node_dst_num,du_time);
            }

}

std::string
MeshTest::GetNthWord(std::string s, std::size_t n)
{
    std::istringstream iss (s);
    while(n > 0 && (iss >> s))
        n--;
    if(n==0)
        return s;
    else
        return "";
}

int
MeshTest::Run ()
{
  ns3::RngSeedManager::SetSeed(m_seed);
  IntegerValue rr;
  GlobalValue::GetValueByNameFailSafe("RngRun",rr);

  ni=new nodeInfo[m_xSize*m_ySize];
  totalRxed=0;
  totalTxed=0;
  totalWannaTx=0;
  totalDelay=Seconds(0);
  totalRxedElasticBytes=0;
  lastTotalRxedElasticBytes=0;
  lastTotalRxed=0;
  totalCnns=0;
  totalAcceptedCnns=0;
  deadNodesNumber=0;

  DIR *pDir = opendir(m_resFolder.c_str());

  if(pDir == NULL){
              std::stringstream makeResFolder;
              makeResFolder << "mkdir -p " << m_resFolder;
              system(makeResFolder.str().c_str());
  }

  std::ostringstream os;
  os << "/home/hadi/hadi_results/hadi_report/simInput.txt";

  simInputFile.open (os.str ().c_str ());
  std::string tempStr;
  num15minPeriods=0;
  struct SimInputs simIns;
  if(simInputFile.is_open ())
    {
      while(!simInputFile.eof())
      {
              std::getline(simInputFile,tempStr);
              simIns.minB=std::atoi(GetNthWord(tempStr,1).c_str ());
              simIns.gamma=std::atof(GetNthWord(tempStr,2).c_str ());
              std::cout << tempStr << std::endl;
              if(tempStr!="")
                {
                  simInputs.push_back (simIns);
                  num15minPeriods++;
                }
              //
      }
    }

  for(std::vector<SimInputs>::iterator itt=simInputs.begin ();itt!=simInputs.end ();itt++)
    {
      std::cout << " === " << itt->minB << " " << itt->gamma << std::endl;
    }
  m_totalTime=num15minPeriods*simPeriods;
  simInsIterator=simInputs.begin ();
  m_gamma=simInsIterator->gamma;
  Simulator::Schedule(Seconds(simPeriods), &MeshTest::SimInputChange,this);

  CreateNodes ();
  InstallInternetStack ();
  InstallApplication ();
  Simulator::Schedule (Seconds (m_totalTime), &MeshTest::Report, this);
  Simulator::Stop (Seconds (m_totalTime));

  PopulateArpCache();

  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::dot11s::HwmpProtocol/TransmittingFromSource", MakeCallback (&MeshTest::Txed,this));
  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::dot11s::HwmpProtocol/WannaTransmittingFromSource", MakeCallback (&MeshTest::WannaTx,this));
  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::dot11s::HwmpProtocol/CbrCnnStateChanged", MakeCallback (&MeshTest::CbrConnectionStateChanged,this));
  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::dot11s::HwmpProtocol/PacketBufferredAtSource", MakeCallback (&MeshTest::BufferredAtSource,this));
  Config::ConnectWithoutContext ("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx", MakeCallback (&MeshTest::RxedElastic,this));

  os.str ("");
  os.clear();
  os << m_resFolder << "seed" << m_seed << "/resFile.txt";
  resFile.open (os.str ().c_str ());
  if (!resFile.is_open ())
    {
      std::cerr << "Error: Can't open file resFile.txt \n";
      return -1;
    }

  os.str ("");
  os.clear();
  os << m_resFolder << "seed" << m_seed << "/remEFile.txt";
  remEFile.open (os.str ().c_str ());
  if (!remEFile.is_open ())
    {
      std::cerr << "Error: Can't open file remEFile.txt \n";
      return -1;
    }

  os.str("");
  os.clear();
  os << m_resFolder << "seed" << m_seed << "/delayFile.txt";
  delayFile.open (os.str ().c_str ());
  if (!delayFile.is_open ())
    {
      std::cerr << "Error: Can't open file delayFile.txt \n";
      return -1;
    }

  os.str("");
  os.clear();
  os << m_resFolder << "seed" << m_seed << "/connectionsFile.txt";
  connectionsFile.open (os.str ().c_str ());
  if (!connectionsFile.is_open ())
    {
      std::cerr << "Error: Can't open file connectionsFile.txt \n";
      return -1;
    }

  os.str("");
  os.clear();
  os << m_resFolder << "seed" << m_seed << "/tcpConnectionsFile.txt";
  tcpConnectionsFile.open (os.str ().c_str ());
  if (!tcpConnectionsFile.is_open ())
    {
      std::cerr << "Error: Can't open file tcpConnectionsFile.txt \n";
      return -1;
    }

  os.str("");
  os.clear();
  os << m_resFolder << "seed" << m_seed << "/tcpAllCnnsFile.txt";
  tcpAllCnnsFile.open (os.str ().c_str ());
  if (!tcpAllCnnsFile.is_open ())
    {
      std::cerr << "Error: Can't open file tcpAllCnnsFile.txt \n";
      return -1;
    }

  os.str("");
  os.clear();
  os << m_resFolder << "seed" << m_seed << "/cnnNumsFile.txt";
  cnnNumsFile.open (os.str ().c_str ());
  if (!cnnNumsFile.is_open ())
    {
      std::cerr << "Error: Can't open file cnnNumsFile.txt \n";
      return -1;
    }

  os.str("");
  os.clear();
  os << m_resFolder << "seed" << m_seed << "/throughtputFile.txt";
  throughtputFile.open (os.str ().c_str ());
  if (!throughtputFile.is_open ())
    {
      std::cerr << "Error: Can't open file throughtputFile.txt \n";
      return -1;
    }

  os.str("");
  os.clear();
  os << m_resFolder << "seed" << m_seed << "/deadNodesFile.txt";
  deadNodesFile.open (os.str ().c_str ());
  if (!deadNodesFile.is_open ())
    {
      std::cerr << "Error: Can't open file deadNodesFile.txt \n";
      return -1;
    }

  Simulator::Run ();


  int trafficVolume = totalRxed * m_packetSize + totalRxedElasticBytes;
  double PDR = (double)totalRxed / totalTxed;
  double cacPackets = (double)totalTxed / totalWannaTx;
  double cacFlows = (double)totalAcceptedCnns / totalCnns;
  double minEnergy=10000;
  for (int var = 0; var < m_xSize*m_ySize; ++var) {
          if(minEnergy>ni[var].remainingE)
                  minEnergy=ni[var].remainingE;
  }

  double aveDelay,delayCIMin,delayCIMax;

  for(CbrConnectionsVector::iterator it=m_currentSystemCnns.begin();it!=m_currentSystemCnns.end();it++)
          connectionsFile << it->srcIpv4Addr << " " << it->dstIpv4Addr << " " << it->whenExpires.GetSeconds() << " " << Simulator::Now().GetSeconds() << std::endl;

delayFile.close ();

os.str("");
os.clear();
os << m_resFolder << "seed" << m_seed << "/delayFile.txt";
std::ifstream fin(os.str().c_str());
double sumDelay=0;
double delayS=0,delayS2=0,delaySE;
int numDelaySamples=0;

  if(fin.is_open())
  {
          while(!fin.eof())
          {
                  std::getline(fin,tempStr);
                  sumDelay += std::atof(GetNthWord(tempStr,3).c_str());
                  numDelaySamples++;
          }
          numDelaySamples--;
          aveDelay = sumDelay / numDelaySamples ;
          fin.clear();
          fin.seekg(0,std::ios::beg);
          while(!fin.eof())
          {
                  std::getline(fin,tempStr);
                  delayS2 += std::pow(aveDelay - std::atof(GetNthWord(tempStr,3).c_str()) , 2);
          }
          delayS2 = delayS2 / numDelaySamples ;
          delayS = std::sqrt(delayS2);
          delaySE = delayS / std::sqrt(numDelaySamples) ;
          delayCIMin = aveDelay - 1.96 * delaySE ;
          delayCIMax = aveDelay + 1.96 * delaySE ;
  }else{
          aveDelay = 0;
          delayCIMin = 0 ;
          delayCIMax = 0 ;
  }

  if(deadNodesNumber==0)
          resFile << m_totalTime << " " ;
  resFile << totalRxed * m_packetSize <<  " " << totalRxedElasticBytes << " "  << trafficVolume << " " << PDR << " " << cacPackets << " " << cacFlows << " " << minEnergy << " " << aveDelay << " " << delayCIMin << " " << delayCIMax << " ";
  for (int var = 0; var < m_xSize*m_ySize; ++var) {
      resFile << ni[var].remainingE << " ";
    }
  resFile << std::endl;

  double minB=1500;
  int minBnode=0;
  remEFile << Simulator::Now().GetSeconds() << " ";
  for(int var=0;var<m_xSize*m_ySize;var++){
      remEFile << ni[var].remainingE << " ";
      if(ni[var].remainingE<minB)
        {
          minB=ni[var].remainingE;
          minBnode=var;
        }
    }
  remEFile << "; " << minBnode << " " << minB << std::endl;


  fin.close();
  resFile.close ();
  remEFile.close ();
  connectionsFile.close ();
  tcpConnectionsFile.close ();
  tcpAllCnnsFile.close();
cnnNumsFile.close ();
throughtputFile.close ();
deadNodesFile.close ();

  	Simulator::Destroy ();
  	return 0;
}
void
MeshTest::Report ()
{
  unsigned n (0);
  for (NetDeviceContainer::Iterator i = meshDevices.Begin (); i != meshDevices.End (); ++i, ++n)
    {
      std::ostringstream os;
      os << m_xml_prefix << n << ".xml";
      std::cerr << "Printing mesh point device #" << n << " diagnostics to " << os.str () << "\n";
      std::ofstream of;
      of.open (os.str ().c_str ());
      if (!of.is_open ())
        {
          std::cerr << "Error: Can't open file " << os.str () << "\n";
          return;
        }
      mesh.Report (*i, of);
      of.close ();
    }
}
int
main (int argc, char *argv[])
{
  MeshTest t;
  t.Configure (argc, argv);
  return t.Run ();
}
