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
}
HwmpRtable::~HwmpRtable ()
{
}
void
HwmpRtable::DoDispose ()
{
  m_routes.clear ();
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
    uint32_t metric,
    uint8_t cnnType,
    Ipv4Address srcIpv4Addr,
    Ipv4Address dstIpv4Addr,
    uint16_t srcPort,
    uint16_t dstPort,
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
            i->metric=metric;
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
    route.metric=metric;
    route.cnnType=cnnType;
    route.srcIpv4Addr=srcIpv4Addr;
    route.dstIpv4Addr=dstIpv4Addr;
    route.srcPort=srcPort;
    route.dstPort=dstPort;
    route.seqnum=seqnum;
    route.whenExpire=Simulator::Now()+lifetime;
    m_cnnBasedRoutes.push_back(route);
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
            return CnnBasedLookupResult(i->retransmitter, i->precursor, i->interface, i->metric, i->seqnum,i->whenExpire - Simulator::Now ());
        }
    }
    return CnnBasedLookupResult();
}

void
HwmpRtable::AddCnnBasedReversePath (
    Mac48Address destination,
    Mac48Address retransmitter,
    uint32_t interface,
    uint32_t metric,
    uint32_t dProb,
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
            i->metric=metric;
            i->dProb=dProb;
            i->seqnum=seqnum;
            i->whenExpire=Simulator::Now()+lifetime;
            return;
        }
    }
    CnnBasedReverseRoute route;
    route.destination=destination;
    route.retransmitter=retransmitter;
    route.interface=interface;
    route.metric=metric;
    route.dProb=dProb;
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
            return CnnBasedLookupResult(i->retransmitter, Mac48Address::GetBroadcast(), i->interface, i->metric, i->seqnum,i->whenExpire - Simulator::Now ());
        }
    }
    return CnnBasedLookupResult();
}
bool
HwmpRtable::CnnBasedLookupResult::operator== (const HwmpRtable::CnnBasedLookupResult & o) const
{
  return (retransmitter == o.retransmitter && precursor == o.precursor && ifIndex == o.ifIndex && metric == o.metric && seqnum == o.seqnum);
}
HwmpRtable::CnnBasedLookupResult::CnnBasedLookupResult (Mac48Address r, Mac48Address p, uint32_t i, uint32_t m, uint32_t s, Time l) :
  retransmitter (r), precursor(p), ifIndex (i), metric (m), seqnum (s), lifetime (l)
{
}
bool
HwmpRtable::CnnBasedLookupResult::IsValid () const
{
  return !(retransmitter == Mac48Address::GetBroadcast () && precursor == Mac48Address::GetBroadcast () && ifIndex == INTERFACE_ANY && metric == MAX_METRIC
           && seqnum == 0);
}


} // namespace dot11s
} // namespace ns3
