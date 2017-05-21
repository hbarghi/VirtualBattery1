/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
//
// Copyright (c) 2006 Georgia Tech Research Corporation
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// Author: George F. Riley<riley@ece.gatech.edu>
//

// ns3 - Poisson Data Source Application class
// Hadi Barghi, Iran University of Science and Technology, Spring 2017

#include "ns3/log.h"
#include "ns3/address.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/packet-socket-address.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/data-rate.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/trace-source-accessor.h"
#include "poisson-application.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/string.h"
#include "ns3/pointer.h"

#include "seq-ts-header.h"

#include "ns3/ipv4.h"
#include "ns3/rhoSigma-tag.h"

NS_LOG_COMPONENT_DEFINE ("PoissonApplication");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (PoissonApplication)
  ;

TypeId
PoissonApplication::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PoissonApplication")
    .SetParent<Application> ()
    .AddConstructor<PoissonApplication> ()
    .AddAttribute ("DataRate", "The data rate in on state.",
                   DataRateValue (DataRate ("64kb/s")),
                   MakeDataRateAccessor (&PoissonApplication::m_averageRate),
                   MakeDataRateChecker ())
    .AddAttribute ("MaxPacketSize", "Max size of packets sent in on state",
                   UintegerValue (640),
                   MakeUintegerAccessor (&PoissonApplication::m_maxPktSize),
                   MakeUintegerChecker<uint32_t> (1))
      .AddAttribute ("MinPacketSize", "Max size of packets sent in on state",
                     UintegerValue (16),
                     MakeUintegerAccessor (&PoissonApplication::m_minPktSize),
                     MakeUintegerChecker<uint32_t> (1))
      .AddAttribute ("PacketSize", "Max size of packets sent in on state",
                     UintegerValue (160),
                     MakeUintegerAccessor (&PoissonApplication::m_pktSize),
                     MakeUintegerChecker<uint32_t> (1))
      .AddAttribute ("SigmaPackets", "Sigma",
                     UintegerValue (10),
                     MakeUintegerAccessor (&PoissonApplication::m_sigmaPackets),
                     MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("Remote", "The address of the destination",
                   AddressValue (),
                   MakeAddressAccessor (&PoissonApplication::m_peer),
                   MakeAddressChecker ())
    .AddAttribute ("Protocol", "The type of protocol to use.",
                   TypeIdValue (UdpSocketFactory::GetTypeId ()),
                   MakeTypeIdAccessor (&PoissonApplication::m_tid),
                   MakeTypeIdChecker ())
    .AddTraceSource ("Tx", "A new packet is created and is sent",
                     MakeTraceSourceAccessor (&PoissonApplication::m_txTrace))
  ;
  return tid;
}


PoissonApplication::PoissonApplication ()
  : m_socket (0),
    m_connected (false),    
    m_lastStartTime (Seconds (0))
{
  m_sent = 0;  
  m_remainedBytes=0;
  NS_LOG_FUNCTION (this);
}

PoissonApplication::~PoissonApplication()
{
  NS_LOG_FUNCTION (this);
}

Ptr<Socket>
PoissonApplication::GetSocket (void) const
{
  NS_LOG_FUNCTION (this);
  return m_socket;
}

int64_t 
PoissonApplication::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  //m_onTime->SetStream (stream);
  //m_offTime->SetStream (stream + 1);
  return 2;
}

void
PoissonApplication::DoDispose (void)
{
  NS_LOG_FUNCTION (this);

  m_socket = 0;
  // chain up
  Application::DoDispose ();
}

// Application Methods
void PoissonApplication::StartApplication () // Called at time specified by Start
{
  NS_LOG_FUNCTION (this);

  m_packetSizeBytes=CreateObject<UniformRandomVariable>();
  double bitrate = (double)m_averageRate.GetBitRate ();
  double interval = (double)((m_maxPktSize+m_minPktSize)/2)/(bitrate/8);
  m_nextTimeSecond=ExponentialVariable(interval,2);

  // Create the socket if not already
  if (!m_socket)
    {
      m_socket = Socket::CreateSocket (GetNode (), m_tid);
      if (Inet6SocketAddress::IsMatchingType (m_peer))
        {
          m_socket->Bind6 ();
        }
      else if (InetSocketAddress::IsMatchingType (m_peer) ||
               PacketSocketAddress::IsMatchingType (m_peer))
        {
          m_socket->Bind ();
        }
      m_socket->Connect (m_peer);
      m_socket->SetAllowBroadcast (true);
      m_socket->ShutdownRecv ();

      m_socket->SetConnectCallback (
        MakeCallback (&PoissonApplication::ConnectionSucceeded, this),
        MakeCallback (&PoissonApplication::ConnectionFailed, this));
    }  

  m_realStopTime=m_stopTime+Simulator::Now ();

  SendPacket ();
  // Insure no pending event
//  CancelEvents ();
  // If we are not yet connected, there is nothing to do here
  // The ConnectionComplete upcall will start timers at that time
  //if (!m_connected) return;
//  ScheduleStartEvent ();
}

void PoissonApplication::StopApplication () // Called at time specified by Stop
{
  NS_LOG_FUNCTION (this);

  CancelEvents ();
  if(m_socket != 0)
    {
      m_socket->Close ();
    }
  else
    {
      NS_LOG_WARN ("PoissonApplication found null socket to close in StopApplication");
    }
}

void PoissonApplication::CancelEvents ()
{
  NS_LOG_FUNCTION (this);

  Simulator::Cancel (m_sendEvent);
}

// Private helpers

void PoissonApplication::SendPacket ()
{
  NS_LOG_FUNCTION (this);

  int newPktSize = (int)m_packetSizeBytes->GetValue (m_minPktSize,m_maxPktSize);

  m_remainedBytes+=newPktSize;

  while(m_remainedBytes>=m_pktSize)
    {
      Ptr<Packet> packet = Create<Packet> (m_pktSize);

      SeqTsHeader seqTs;
      seqTs.SetSeq (m_sent++);
      packet->AddHeader (seqTs);

      RhoSigmaTag rsTag;

      double bitrate = (double)m_averageRate.GetBitRate ();
      uint16_t rhoPpm=((bitrate/8)/m_pktSize)*60;///TODO: calculate
      //uint16_t sigma=1;///TODO: calculate
      uint16_t maxPktSizePackets=(m_maxPktSize+m_pktSize-1)/m_pktSize;

      rsTag.SetRho (rhoPpm);
      rsTag.SetStopTime(m_realStopTime);
      rsTag.SetSigma(m_sigmaPackets);
      rsTag.setDelayBound (MilliSeconds (200));
      rsTag.setMaxPktSize (maxPktSizePackets);
      packet->AddPacketTag(rsTag);


      Address addr;
      m_socket->GetSockName (addr);
      InetSocketAddress iaddr = InetSocketAddress::ConvertFrom (addr);

      Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
       Ipv4InterfaceAddress iaddr1 = ipv4->GetAddress (1,0);
       Ipv4Address addri = iaddr1.GetLocal ();

       NS_LOG_CAC("SendingAPacket " << addri << ":" << iaddr.GetPort () << "=>" << InetSocketAddress::ConvertFrom(m_peer).GetIpv4 () << ":" << InetSocketAddress::ConvertFrom (m_peer).GetPort () << " " << packet->GetUid ());

       m_txTrace (packet);
      m_socket->Send (packet);


      if (InetSocketAddress::IsMatchingType (m_peer))
        {
          NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds ()
                       << "s on-off application sent "
                       <<  packet->GetSize () << " bytes to "
                       << InetSocketAddress::ConvertFrom(m_peer).GetIpv4 ()
                       << " port " << InetSocketAddress::ConvertFrom (m_peer).GetPort ());
        }
      else if (Inet6SocketAddress::IsMatchingType (m_peer))
        {
          NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds ()
                       << "s on-off application sent "
                       <<  packet->GetSize () << " bytes to "
                       << Inet6SocketAddress::ConvertFrom(m_peer).GetIpv6 ()
                       << " port " << Inet6SocketAddress::ConvertFrom (m_peer).GetPort ());
        }
      m_lastStartTime = Simulator::Now ();
      m_remainedBytes-=m_pktSize;
    }
  ///TODO:use poisson distribution

  Time nextTime (Seconds (m_nextTimeSecond.GetValue ())); // Time till next packet
  NS_LOG_LOGIC ("nextTime = " << nextTime);
  m_sendEvent = Simulator::Schedule (nextTime, &PoissonApplication::SendPacket, this);
}


void PoissonApplication::ConnectionSucceeded (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  m_connected = true;
}

void PoissonApplication::ConnectionFailed (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
}


} // Namespace ns3
