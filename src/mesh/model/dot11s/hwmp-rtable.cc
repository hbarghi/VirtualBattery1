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
 */

#include "ns3/object.h"
#include "ns3/assert.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/log.h"

#include "hwmp-rtable.h"

namespace ns3 {
namespace dot11s {

NS_LOG_COMPONENT_DEFINE ("HwmpRtable");

NS_OBJECT_ENSURE_REGISTERED (HwmpRtable)
  ;

TypeId
HwmpRtable::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::dot11s::HwmpRtable")
    .SetParent<Object> ()
    .AddConstructor<HwmpRtable> ();
  return tid;
}
HwmpRtable::HwmpRtable ()
{
  DeleteProactivePath ();
  Simulator::Schedule(MilliSeconds (1),&HwmpRtable::UpdateToken,this);
}
HwmpRtable::~HwmpRtable ()
{
}
void
HwmpRtable::DoDispose ()
{
  m_routes.clear ();
  m_cnnBasedRoutes.clear ();
}
void
HwmpRtable::AddReactivePath (Mac48Address destination, Mac48Address retransmitter, uint32_t interface,
                             uint32_t metric, Time lifetime, uint32_t seqnum)
{
  std::map<Mac48Address, ReactiveRoute>::iterator i = m_routes.find (destination);
  if (i == m_routes.end ())
    {
      ReactiveRoute newroute;
      m_routes[destination] = newroute;
    }
  i = m_routes.find (destination);
  NS_ASSERT (i != m_routes.end ());
  i->second.retransmitter = retransmitter;
  i->second.interface = interface;
  i->second.metric = metric;
  i->second.whenExpire = Simulator::Now () + lifetime;
  i->second.seqnum = seqnum;
}
void
HwmpRtable::AddProactivePath (uint32_t metric, Mac48Address root, Mac48Address retransmitter,
                              uint32_t interface, Time lifetime, uint32_t seqnum)
{
  m_root.root = root;
  m_root.retransmitter = retransmitter;
  m_root.metric = metric;
  m_root.whenExpire = Simulator::Now () + lifetime;
  m_root.seqnum = seqnum;
  m_root.interface = interface;
}
void
HwmpRtable::AddPrecursor (Mac48Address destination, uint32_t precursorInterface,
                          Mac48Address precursorAddress, Time lifetime)
{
  Precursor precursor;
  precursor.interface = precursorInterface;
  precursor.address = precursorAddress;
  precursor.whenExpire = Simulator::Now () + lifetime;
  std::map<Mac48Address, ReactiveRoute>::iterator i = m_routes.find (destination);
  if (i != m_routes.end ())
    {
      bool should_add = true;
      for (unsigned int j = 0; j < i->second.precursors.size (); j++)
        {
          //NB: Only one active route may exist, so do not check
          //interface ID, just address
          if (i->second.precursors[j].address == precursorAddress)
            {
              should_add = false;
              i->second.precursors[j].whenExpire = precursor.whenExpire;
              break;
            }
        }
      if (should_add)
        {
          i->second.precursors.push_back (precursor);
        }
    }
}
void
HwmpRtable::DeleteProactivePath ()
{
  m_root.precursors.clear ();
  m_root.interface = INTERFACE_ANY;
  m_root.metric = MAX_METRIC;
  m_root.retransmitter = Mac48Address::GetBroadcast ();
  m_root.seqnum = 0;
  m_root.whenExpire = Simulator::Now ();
}
void
HwmpRtable::DeleteProactivePath (Mac48Address root)
{
  if (m_root.root == root)
    {
      DeleteProactivePath ();
    }
}
void
HwmpRtable::DeleteReactivePath (Mac48Address destination)
{
  std::map<Mac48Address, ReactiveRoute>::iterator i = m_routes.find (destination);
  if (i != m_routes.end ())
    {
      m_routes.erase (i);
    }
}
HwmpRtable::LookupResult
HwmpRtable::LookupReactive (Mac48Address destination)
{
  std::map<Mac48Address, ReactiveRoute>::iterator i = m_routes.find (destination);
  if (i == m_routes.end ())
    {
      return LookupResult ();
    }
  if ((i->second.whenExpire < Simulator::Now ()) && (i->second.whenExpire != Seconds (0)))
    {
      NS_LOG_DEBUG ("Reactive route has expired, sorry.");
      return LookupResult ();
    }
  return LookupReactiveExpired (destination);
}
HwmpRtable::LookupResult
HwmpRtable::LookupReactiveExpired (Mac48Address destination)
{
  std::map<Mac48Address, ReactiveRoute>::iterator i = m_routes.find (destination);
  if (i == m_routes.end ())
    {
      return LookupResult ();
    }
  return LookupResult (i->second.retransmitter, i->second.interface, i->second.metric, i->second.seqnum,
                       i->second.whenExpire - Simulator::Now ());
}
HwmpRtable::LookupResult
HwmpRtable::LookupProactive ()
{
  if (m_root.whenExpire < Simulator::Now ())
    {
      NS_LOG_DEBUG ("Proactive route has expired and will be deleted, sorry.");
      DeleteProactivePath ();
    }
  return LookupProactiveExpired ();
}
HwmpRtable::LookupResult
HwmpRtable::LookupProactiveExpired ()
{
  return LookupResult (m_root.retransmitter, m_root.interface, m_root.metric, m_root.seqnum,
                       m_root.whenExpire - Simulator::Now ());
}

std::vector<HwmpProtocol::FailedDestination>
HwmpRtable::GetUnreachableDestinations (Mac48Address peerAddress)
{
  HwmpProtocol::FailedDestination dst;
  std::vector<HwmpProtocol::FailedDestination> retval;
  for (std::map<Mac48Address, ReactiveRoute>::iterator i = m_routes.begin (); i != m_routes.end (); i++)
    {
      if (i->second.retransmitter == peerAddress)
        {
          dst.destination = i->first;
          i->second.seqnum++;
          dst.seqnum = i->second.seqnum;
          retval.push_back (dst);
        }
    }
  //Lookup a path to root
  if (m_root.retransmitter == peerAddress)
    {
      dst.destination = m_root.root;
      dst.seqnum = m_root.seqnum;
      retval.push_back (dst);
    }
  return retval;
}
HwmpRtable::PrecursorList
HwmpRtable::GetPrecursors (Mac48Address destination)
{
  //We suppose that no duplicates here can be
  PrecursorList retval;
  std::map<Mac48Address, ReactiveRoute>::iterator route = m_routes.find (destination);
  if (route != m_routes.end ())
    {
      for (std::vector<Precursor>::const_iterator i = route->second.precursors.begin ();
           i != route->second.precursors.end (); i++)
        {
          if (i->whenExpire > Simulator::Now ())
            {
              retval.push_back (std::make_pair (i->interface, i->address));
            }
        }
    }
  return retval;
}
bool
HwmpRtable::LookupResult::operator== (const HwmpRtable::LookupResult & o) const
{
  return (retransmitter == o.retransmitter && ifIndex == o.ifIndex && metric == o.metric && seqnum
          == o.seqnum);
}
HwmpRtable::LookupResult::LookupResult (Mac48Address r, uint32_t i, uint32_t m, uint32_t s, Time l) :
  retransmitter (r), ifIndex (i), metric (m), seqnum (s), lifetime (l)
{
}
bool
HwmpRtable::LookupResult::IsValid () const
{
  return !(retransmitter == Mac48Address::GetBroadcast () && ifIndex == INTERFACE_ANY && metric == MAX_METRIC
           && seqnum == 0);
}

void
HwmpRtable::AddCnnBasedReactivePath (
    Mac48Address destination,
    Mac48Address retransmitter,
    Mac48Address source,
    Mac48Address precursor,
    uint32_t interface,
    uint8_t cnnType,
    Ipv4Address srcIpv4Addr,
    Ipv4Address dstIpv4Addr,
    uint16_t srcPort,
    uint16_t dstPort,
    uint16_t rho,
    uint16_t sigma,
    Time  lifetime,
    uint32_t seqnum
    )
{
    for(std::vector<CnnBasedReactiveRoute>::iterator i=m_cnnBasedRoutes.begin();i<m_cnnBasedRoutes.end();i++)
    {
        if(
                (i->destination==destination)&&
                (i->source==source)&&
                (i->cnnType==cnnType)&&
                (i->srcIpv4Addr==srcIpv4Addr)&&
                (i->srcPort==srcPort)&&
                (i->dstIpv4Addr==dstIpv4Addr)&&
                (i->dstPort==dstPort)
          )//route exists, will be updated
        {
            i->retransmitter=retransmitter;
            i->precursor=precursor;
            i->interface=interface;
            i->seqnum=seqnum;
            i->whenExpire=Simulator::Now()+lifetime;
            return;
        }
    }
    CnnBasedReactiveRoute route;
    route.destination=destination;
    route.retransmitter=retransmitter;
    route.source=source;
    route.precursor=precursor;
    route.interface=interface;
    route.cnnType=cnnType;
    route.srcIpv4Addr=srcIpv4Addr;
    route.dstIpv4Addr=dstIpv4Addr;
    route.srcPort=srcPort;
    route.dstPort=dstPort;
    route.seqnum=seqnum;
    route.whenExpire=Simulator::Now()+lifetime;


    route.tokenBucketVirtualBattery=CreateObject<TokenBucketVirtualBattery>();
    route.tokenBucketVirtualBattery->m_numTokensPerMillisecond=rho/60000;//rho : p/ms
    route.tokenBucketVirtualBattery->m_maxTokenPacket=sigma;//sigma
    route.tokenBucketVirtualBattery->m_numTokenPacket=sigma;//rho : p/ms

    route.tokenBucketVirtualBattery->m_bMax = route.tokenBucketVirtualBattery->m_maxTokenPacket * (m_maxEnergyPerDataPacket+m_maxEnergyPerAckPacket);
    route.tokenBucketVirtualBattery->m_gamma = route.tokenBucketVirtualBattery->m_numTokensPerMillisecond * ((m_maxEnergyPerDataPacket+m_maxEnergyPerAckPacket)*1000);
    route.tokenBucketVirtualBattery->m_b=route.tokenBucketVirtualBattery->m_bMax;

    m_assignedGamma+=route.tokenBucketVirtualBattery->m_gamma;
    m_gammaPrim-=route.tokenBucketVirtualBattery->m_gamma;
    m_bPrim-=route.tokenBucketVirtualBattery->m_b;
    m_bPrimMax-=route.tokenBucketVirtualBattery->m_bMax;

    m_cnnBasedRoutes.push_back(route);
}


/////////  TokenBucketVirtualBattery methods start
TypeId
TokenBucketVirtualBattery::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::dot11s::TokenBucketVirtualBattery")
    .SetParent<Object> ()
    .AddConstructor<TokenBucketVirtualBattery> ();
  return tid;
}
TokenBucketVirtualBattery::TokenBucketVirtualBattery () :
  m_maxQueueSize(255)
{
}
TokenBucketVirtualBattery::~TokenBucketVirtualBattery ()
{
}
void
TokenBucketVirtualBattery::DoDispose ()
{  
  m_rqueue.clear ();
}
bool
TokenBucketVirtualBattery::QueuePacket (QueuedPacket packet)
{
  if (m_rqueue.size () > m_maxQueueSize)
    {
      return false;
    }
  m_rqueue.push_back (packet);
  return true;
}

TokenBucketVirtualBattery::QueuedPacket::QueuedPacket () :
  pkt (0),
  protocol (0),
  interface (0)
{
}

void
TokenBucketVirtualBattery::UpdateToken ()
{
  m_numTokenPacket+=m_numTokensPerMillisecond;
  if(m_numTokenPacket>m_maxTokenPacket)
    m_numTokenPacket=m_maxTokenPacket;

  double tempBatterLevel=m_b;
  while(m_rqueue.size ()!=0)
    {
      if((tempBatterLevel>=m_maxEnergyPerDataPacket+m_maxEnergyPerAckPacket)&&(m_numTokenPacket>=1))
        {
          QueuedPacket packet = m_rqueue[0];
          m_rqueue.erase (m_rqueue.begin ());
          packet.reply (true, packet.pkt, packet.src, packet.dst, packet.protocol, packet.interface);
          tempBatterLevel-=(m_maxEnergyPerDataPacket+m_maxEnergyPerAckPacket);
          m_numTokenPacket--;
        }
      else
        {
          return;
        }
    }

}

void
HwmpRtable::QueueCnnBasedPacket (
    Mac48Address destination,
    Mac48Address source,
    uint8_t cnnType,
    Ipv4Address srcIpv4Addr,
    Ipv4Address dstIpv4Addr,
    uint16_t srcPort,
    uint16_t dstPort,
    Ptr<Packet> packet,
    uint16_t protocolType,
    uint32_t sourceIface,
    Callback<void, /* return type */
                       bool, /* flag */
                       Ptr<Packet>, /* packet */
                       Mac48Address, /* src */
                       Mac48Address, /* dst */
                       uint16_t, /* protocol */
                       uint32_t /* out interface ID */
                       > routeReply ///< how to reply
    )
{
  for(std::vector<CnnBasedReactiveRoute>::iterator i=m_cnnBasedRoutes.begin();i<m_cnnBasedRoutes.end();i++)
  {
      if(
              (i->destination==destination)&&
              (i->source==source)&&
              (i->cnnType==cnnType)&&
              (i->srcIpv4Addr==srcIpv4Addr)&&
              (i->srcPort==srcPort)&&
              (i->dstIpv4Addr==dstIpv4Addr)&&
              (i->dstPort==dstPort)
        )//route exists, will be deleted
        {
          TokenBucketVirtualBattery::QueuedPacket pkt;
          pkt.pkt = packet;
          pkt.dst = destination;
          pkt.src = source;
          pkt.protocol = protocolType;
          pkt.reply = routeReply;
          pkt.interface = sourceIface;
          pkt.cnnType=cnnType;
          pkt.srcIpv4Addr=srcIpv4Addr;
          pkt.dstIpv4Addr=dstIpv4Addr;
          pkt.srcPort=srcPort;
          pkt.dstPort=dstPort;
          i->tokenBucketVirtualBattery->QueuePacket (pkt);
          return;
        }
    }
}
/////////  TokenBucketVirtualBattery methods end


void
HwmpRtable::SetMaxEnergyPerDataPacket(double energy)
{
  m_maxEnergyPerDataPacket = energy;
  for(std::vector<CnnBasedReactiveRoute>::iterator i=m_cnnBasedRoutes.begin();i<m_cnnBasedRoutes.end();i++)
    {
      i->tokenBucketVirtualBattery->m_maxEnergyPerDataPacket=energy;
    }
}
double
HwmpRtable::GetMaxEnergyPerDataPacket()
{
  return m_maxEnergyPerDataPacket;
}
void
HwmpRtable::SetMaxEnergyPerAckPacket(double energy)
{
  m_maxEnergyPerAckPacket = energy;
  for(std::vector<CnnBasedReactiveRoute>::iterator i=m_cnnBasedRoutes.begin();i<m_cnnBasedRoutes.end();i++)
    {
      i->tokenBucketVirtualBattery->m_maxEnergyPerAckPacket=energy;
    }
}
double
HwmpRtable::GetMaxEnergyPerAckPacket()
{
  return m_maxEnergyPerAckPacket;
}

void
HwmpRtable::TotalEnergyIncreasedByGamma(double energy)
{
  double remained2IncreaseEnergy=energy;
  double increasingEnergy=0;
  for(std::vector<CnnBasedReactiveRoute>::iterator i=m_cnnBasedRoutes.begin();i<m_cnnBasedRoutes.end();i++)
    {
      increasingEnergy=(i->tokenBucketVirtualBattery->m_gamma/m_systemGamma)*energy;
      if(i->tokenBucketVirtualBattery->m_b+increasingEnergy<=i->tokenBucketVirtualBattery->m_bMax)
        {
          remained2IncreaseEnergy-=increasingEnergy;
          i->tokenBucketVirtualBattery->m_b+=increasingEnergy;
        }
      else
        {
          remained2IncreaseEnergy-=(i->tokenBucketVirtualBattery->m_bMax-i->tokenBucketVirtualBattery->m_b);
          i->tokenBucketVirtualBattery->m_b=i->tokenBucketVirtualBattery->m_bMax;
        }
    }
  if(m_bPrim+remained2IncreaseEnergy<=m_bPrimMax)
    {
      m_bPrim+=remained2IncreaseEnergy;
    }
  else
    {
      remained2IncreaseEnergy-=(m_bPrimMax-m_bPrim);
      m_bPrim=m_bPrimMax;
      bool thereIsNotFullVB=true;
      while((remained2IncreaseEnergy>0)&&(thereIsNotFullVB))
        {
          thereIsNotFullVB=false;
          double newEnergy=remained2IncreaseEnergy;
          for(std::vector<CnnBasedReactiveRoute>::iterator i=m_cnnBasedRoutes.begin();i<m_cnnBasedRoutes.end();i++)
            {
              increasingEnergy=(i->tokenBucketVirtualBattery->m_gamma/m_systemGamma)*newEnergy;
              if(i->tokenBucketVirtualBattery->m_b+increasingEnergy<i->tokenBucketVirtualBattery->m_bMax)
                {
                  remained2IncreaseEnergy-=increasingEnergy;
                  i->tokenBucketVirtualBattery->m_b+=increasingEnergy;
                  thereIsNotFullVB=true;
                }
              else
                {
                  remained2IncreaseEnergy-=(i->tokenBucketVirtualBattery->m_bMax-i->tokenBucketVirtualBattery->m_b);
                  i->tokenBucketVirtualBattery->m_b=i->tokenBucketVirtualBattery->m_bMax;
                }
            }
        }
    }

}

void HwmpRtable::TotalEnergyDecreasedByOtherPackets(double energy)
{
  double remained2DecreaseEnergy=energy;
  double decreasingEnergy=0;
  if(m_bPrim>=energy)
    {
      m_bPrim-=energy;
    }
  else
    {
      remained2DecreaseEnergy-=m_bPrim;
      m_bPrim=0;
      bool thereIsNotEmptyVB=true;
      while((remained2DecreaseEnergy>0)&&(thereIsNotEmptyVB))
        {
          double newEnergy = remained2DecreaseEnergy;
          thereIsNotEmptyVB=false;
          for(std::vector<CnnBasedReactiveRoute>::iterator i=m_cnnBasedRoutes.begin();i<m_cnnBasedRoutes.end();i++)
            {
              decreasingEnergy = (i->tokenBucketVirtualBattery->m_gamma/m_assignedGamma)*newEnergy;
              if(i->tokenBucketVirtualBattery->m_b>decreasingEnergy)
                {
                  thereIsNotEmptyVB=true;
                  remained2DecreaseEnergy-=decreasingEnergy;
                  i->tokenBucketVirtualBattery->m_b-=decreasingEnergy;
                }
              else
                {
                  remained2DecreaseEnergy-=i->tokenBucketVirtualBattery->m_b;
                  i->tokenBucketVirtualBattery->m_b=0;
                }
            }
        }
    }
}

void HwmpRtable::ChangeEnergy4aConnection(uint8_t cnnType, Ipv4Address srcIpv4Addr, Ipv4Address dstIpv4Addr, uint16_t srcPort, uint16_t dstPort, double energy, bool incDec)
{
  for(std::vector<CnnBasedReactiveRoute>::iterator i=m_cnnBasedRoutes.begin();i<m_cnnBasedRoutes.end();i++)
    {
      if(
              (i->cnnType==cnnType)&&
              (i->srcIpv4Addr==srcIpv4Addr)&&
              (i->srcPort==srcPort)&&
              (i->dstIpv4Addr==dstIpv4Addr)&&
              (i->dstPort==dstPort)
        )
        {
          if(incDec)
            {
              i->tokenBucketVirtualBattery->m_b+=energy;
            }
          else
            {
              i->tokenBucketVirtualBattery->m_b-=energy;
            }
        }
    }
}

void
HwmpRtable::DeleteCnnBasedReactivePath (
    Mac48Address destination,
    Mac48Address source,
    uint8_t cnnType,
    Ipv4Address srcIpv4Addr,
    Ipv4Address dstIpv4Addr,
    uint16_t srcPort,
    uint16_t dstPort
    )
{
    for(std::vector<CnnBasedReactiveRoute>::iterator i=m_cnnBasedRoutes.begin();i<m_cnnBasedRoutes.end();i++)
    {
        if(
                (i->destination==destination)&&
                (i->source==source)&&
                (i->cnnType==cnnType)&&
                (i->srcIpv4Addr==srcIpv4Addr)&&
                (i->srcPort==srcPort)&&
                (i->dstIpv4Addr==dstIpv4Addr)&&
                (i->dstPort==dstPort)
          )//route exists, will be deleted
        {
            m_assignedGamma-=i->tokenBucketVirtualBattery->m_gamma;
            m_gammaPrim+=i->tokenBucketVirtualBattery->m_gamma;
            m_bPrim+=i->tokenBucketVirtualBattery->m_b;
            m_bPrimMax+=i->tokenBucketVirtualBattery->m_bMax;
            i->tokenBucketVirtualBattery->Dispose ();
            m_cnnBasedRoutes.erase(i);
        }
    }
}

HwmpRtable::CnnBasedLookupResult
HwmpRtable::LookupCnnBasedReactive (
    Mac48Address destination,
    Mac48Address source,
    uint8_t cnnType,
    Ipv4Address srcIpv4Addr,
    Ipv4Address dstIpv4Addr,
    uint16_t srcPort,
    uint16_t dstPort
    )
{
    for(std::vector<CnnBasedReactiveRoute>::iterator i=m_cnnBasedRoutes.begin();i<m_cnnBasedRoutes.end();i++)
    {
        if(
                (i->destination==destination)&&
                (i->source==source)&&
                (i->cnnType==cnnType)&&
                (i->srcIpv4Addr==srcIpv4Addr)&&
                (i->srcPort==srcPort)&&
                (i->dstIpv4Addr==dstIpv4Addr)&&
                (i->dstPort==dstPort)
          )//route exists, will be deleted
        {
            return CnnBasedLookupResult(i->retransmitter, i->precursor, i->interface, i->seqnum,i->whenExpire - Simulator::Now ());
        }
    }
    return CnnBasedLookupResult();
}

void
HwmpRtable::AddCnnBasedReversePath (
    Mac48Address destination,
    Mac48Address retransmitter,
    uint32_t interface,
    uint8_t cnnType,
    Ipv4Address srcIpv4Addr,
    Ipv4Address dstIpv4Addr,
    uint16_t srcPort,
    uint16_t dstPort,
    Time  lifetime,
    uint32_t seqnum
    )
{
    for(std::vector<CnnBasedReverseRoute>::iterator i=m_cnnBasedReverse.begin();i<m_cnnBasedReverse.end();i++)
    {
        if(
                (i->destination==destination)&&
                (i->cnnType==cnnType)&&
                (i->srcIpv4Addr==srcIpv4Addr)&&
                (i->srcPort==srcPort)&&
                (i->dstIpv4Addr==dstIpv4Addr)&&
                (i->dstPort==dstPort)
          )//route exists, will be updated
        {
            i->retransmitter=retransmitter;
            i->interface=interface;
            i->seqnum=seqnum;
            i->whenExpire=Simulator::Now()+lifetime;
            return;
        }
    }
    CnnBasedReverseRoute route;
    route.destination=destination;
    route.retransmitter=retransmitter;
    route.interface=interface;
    route.cnnType=cnnType;
    route.srcIpv4Addr=srcIpv4Addr;
    route.dstIpv4Addr=dstIpv4Addr;
    route.srcPort=srcPort;
    route.dstPort=dstPort;
    route.seqnum=seqnum;
    route.whenExpire=Simulator::Now()+lifetime;
    m_cnnBasedReverse.push_back(route);
}

void
HwmpRtable::DeleteCnnBasedReversePath (
    Mac48Address destination,
    uint8_t cnnType,
    Ipv4Address srcIpv4Addr,
    Ipv4Address dstIpv4Addr,
    uint16_t srcPort,
    uint16_t dstPort
    )
{
    for(std::vector<CnnBasedReverseRoute>::iterator i=m_cnnBasedReverse.begin();i<m_cnnBasedReverse.end();i++)
    {
        if(
                (i->destination==destination)&&
                (i->cnnType==cnnType)&&
                (i->srcIpv4Addr==srcIpv4Addr)&&
                (i->srcPort==srcPort)&&
                (i->dstIpv4Addr==dstIpv4Addr)&&
                (i->dstPort==dstPort)
          )//route exists, will be deleted
        {
            m_cnnBasedReverse.erase(i);
        }
    }
}

HwmpRtable::CnnBasedLookupResult
HwmpRtable::LookupCnnBasedReverse (
    Mac48Address destination,
    uint8_t cnnType,
    Ipv4Address srcIpv4Addr,
    Ipv4Address dstIpv4Addr,
    uint16_t srcPort,
    uint16_t dstPort
    )
{
    for(std::vector<CnnBasedReverseRoute>::iterator i=m_cnnBasedReverse.begin();i<m_cnnBasedReverse.end();i++)
    {
        if(
                (i->destination==destination)&&
                (i->cnnType==cnnType)&&
                (i->srcIpv4Addr==srcIpv4Addr)&&
                (i->srcPort==srcPort)&&
                (i->dstIpv4Addr==dstIpv4Addr)&&
                (i->dstPort==dstPort)
          )//route exists, will be deleted
        {
            return CnnBasedLookupResult(i->retransmitter, Mac48Address::GetBroadcast(), i->interface, i->seqnum,i->whenExpire - Simulator::Now ());
        }
    }
    return CnnBasedLookupResult();
}

void HwmpRtable::UpdateToken()
{
  for(std::vector<CnnBasedReactiveRoute>::iterator i=m_cnnBasedRoutes.begin();i<m_cnnBasedRoutes.end();i++)
    {
      i->tokenBucketVirtualBattery->UpdateToken ();
    }
  Simulator::Schedule(MilliSeconds (1),&HwmpRtable::UpdateToken,this);
}

double HwmpRtable::systemGamma() const
{
  return m_systemGamma;
}

void HwmpRtable::setSystemGamma(double systemGamma)
{
  m_systemGamma = systemGamma;
}

double HwmpRtable::systemB() const
{
  return m_systemB;
}

void HwmpRtable::setSystemB(double systemB)
{
  m_systemB = systemB;
}

double HwmpRtable::systemBMax() const
{
  return m_systemBMax;
}

void HwmpRtable::setSystemBMax(double systemBMax)
{
  m_systemBMax = systemBMax;
}

double HwmpRtable::gammaPrim() const
{
  return m_gammaPrim;
}

void HwmpRtable::setGammaPrim(double gammaPrim)
{
  m_gammaPrim = gammaPrim;
}

double HwmpRtable::bPrim() const
{
  return m_bPrim;
}

void HwmpRtable::setBPrim(double bPrim)
{
  m_bPrim = bPrim;
}

double HwmpRtable::bPrimMax() const
{
  return m_bPrimMax;
}

void HwmpRtable::setBPrimMax(double bPrimMax)
{
  m_bPrimMax = bPrimMax;
}

double HwmpRtable::assignedGamma() const
{
  return m_assignedGamma;
}

void HwmpRtable::setAssignedGamma(double assignedGamma)
{
  m_assignedGamma = assignedGamma;
}
bool
HwmpRtable::CnnBasedLookupResult::operator== (const HwmpRtable::CnnBasedLookupResult & o) const
{
  return (retransmitter == o.retransmitter && precursor == o.precursor && ifIndex == o.ifIndex && seqnum == o.seqnum);
}
HwmpRtable::CnnBasedLookupResult::CnnBasedLookupResult (Mac48Address r, Mac48Address p, uint32_t i, uint32_t s, Time l) :
  retransmitter (r), precursor(p), ifIndex (i), seqnum (s), lifetime (l)
{
}
bool
HwmpRtable::CnnBasedLookupResult::IsValid () const
{
  return !(retransmitter == Mac48Address::GetBroadcast () && precursor == Mac48Address::GetBroadcast () && ifIndex == INTERFACE_ANY && seqnum == 0);
}


} // namespace dot11s
} // namespace ns3
