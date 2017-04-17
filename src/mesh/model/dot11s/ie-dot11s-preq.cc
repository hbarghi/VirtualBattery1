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

#include "ie-dot11s-preq.h"
#include "ns3/address-utils.h"
#include "ns3/assert.h"
#include "ns3/packet.h"

namespace ns3 {
namespace dot11s {
/*************************
 * DestinationAddressUnit
 ************************/
DestinationAddressUnit::DestinationAddressUnit () :
  m_do (false), m_rf (false), m_usn (false), m_destinationAddress (Mac48Address ()), m_destSeqNumber (0)
{
}
void
DestinationAddressUnit::SetFlags (bool doFlag, bool rfFlag, bool usnFlag)
{
  m_do = doFlag;
  m_rf = rfFlag;
  m_usn = usnFlag;
}

void
DestinationAddressUnit::SetDestSeqNumber (uint32_t dest_seq_number)
{
  m_destSeqNumber = dest_seq_number;
  if (m_destSeqNumber != 0)
    {
      m_usn = true;
    }
}
void
DestinationAddressUnit::SetDestinationAddress (Mac48Address dest_address)
{
  m_destinationAddress = dest_address;
}
bool
DestinationAddressUnit::IsDo ()
{
  return m_do;
}

bool
DestinationAddressUnit::IsRf ()
{
  return m_rf;
}
bool
DestinationAddressUnit::IsUsn ()
{
  return m_usn;
}
uint32_t
DestinationAddressUnit::GetDestSeqNumber () const
{
  return m_destSeqNumber;
}
Mac48Address
DestinationAddressUnit::GetDestinationAddress () const
{
  return m_destinationAddress;
}
/********************************
 * IePreq
 *******************************/
IePreq::~IePreq ()
{
}
IePreq::IePreq () :
  m_maxSize (32), m_flags (0), m_hopCount (0), m_ttl (0), m_preqId (0), m_originatorAddress (
    Mac48Address::GetBroadcast ()), m_originatorSeqNumber (0), m_lifetime (0), m_metric (0),
        m_cnnType(1),
        m_srcIpv4Addr("10.0.0.1"),
        m_dstIpv4Addr("10.0.0.2"),
        m_srcPort(25000),
        m_dstPort(25001),
        m_rho (0),
        m_sigma (0),
        m_stopTime (Seconds(0)),
        m_gammaPrimMilliwatts(0),//for sum:0, for min:maxuint32=0xffffffff
        m_bPrimMillijoules(0),//for sum:0, for min:maxuint32=0xffffffff
  m_destCount (0)
{
}
WifiInformationElementId
IePreq::ElementId () const
{
  return IE11S_PREQ;
}
void
IePreq::SetUnicastPreq ()
{
  m_flags |= 1 << 1;
}

void
IePreq::SetNeedNotPrep ()
{
  m_flags |= 1 << 2;
}
void
IePreq::SetHopcount (uint8_t hopcount)
{
  m_hopCount = hopcount;
}
void
IePreq::SetTTL (uint8_t ttl)
{
  m_ttl = ttl;
}
void
IePreq::SetPreqID (uint32_t preq_id)
{
  m_preqId = preq_id;
}
void
IePreq::SetMetric (uint32_t metric)
{
  m_metric = metric;
}
void
IePreq::SetCnnParams(uint8_t cnnType,Ipv4Address srcIpv4Addr,Ipv4Address dstIpv4Addr,uint16_t srcPort,uint16_t dstPort)
{
    m_cnnType=cnnType;
    m_srcIpv4Addr=srcIpv4Addr;
    m_srcPort=srcPort;
    m_dstIpv4Addr=dstIpv4Addr;
    m_dstPort=dstPort;
}
void
IePreq::SetRho (uint16_t rho)
{
  m_rho = rho;
}
void
IePreq::SetSigma (uint16_t sigma)
{
  m_sigma = sigma;
}
void
IePreq::SetStopTime (Time stop)
{
  m_stopTime = stop;
}
void
IePreq::SetOriginatorAddress (Mac48Address originator_address)
{
  m_originatorAddress = originator_address;
}
void
IePreq::SetOriginatorSeqNumber (uint32_t originator_seq_number)
{
  m_originatorSeqNumber = originator_seq_number;
}
void
IePreq::SetLifetime (uint32_t lifetime)
{
  m_lifetime = lifetime;
}
void
IePreq::SetDestCount (uint8_t dest_count)
{
  m_destCount = dest_count;
}
bool
IePreq::IsUnicastPreq () const
{
  return (m_flags & (1 << 1));
}
bool
IePreq::IsNeedNotPrep () const
{
  return (m_flags & (1 << 2));
}
uint8_t
IePreq::GetHopCount () const
{
  return m_hopCount;
}
uint8_t
IePreq::GetTtl () const
{
  return m_ttl;
}
uint32_t
IePreq::GetPreqID () const
{
  return m_preqId;
}
uint32_t
IePreq::GetMetric () const
{
  return m_metric;
}
uint8_t
IePreq::GetCnnType () const
{
    return m_cnnType;
}
Ipv4Address
IePreq::GetSrcIpv4Addr () const
{
    return m_srcIpv4Addr;
}
Ipv4Address
IePreq::GetDstIpv4Addr () const
{
    return m_dstIpv4Addr;
}
uint16_t
IePreq::GetSrcPort () const
{
    return m_srcPort;
}
uint16_t
IePreq::GetDstPort () const
{
    return m_dstPort;
}
Mac48Address
IePreq::GetOriginatorAddress () const
{
  return m_originatorAddress;
}
uint32_t
IePreq::GetOriginatorSeqNumber () const
{
  return m_originatorSeqNumber;
}
uint32_t
IePreq::GetLifetime () const
{
  return m_lifetime;
}
uint16_t
IePreq::GetRho ()
{
  return m_rho;
}
uint16_t
IePreq::GetSigma ()
{
  return m_sigma;
}
Time
IePreq::GetStopTime ()
{
  return m_stopTime;
}
uint8_t
IePreq::GetDestCount () const
{
  return m_destCount;
}
void
IePreq::DecrementTtl ()
{
  m_ttl--;
  m_hopCount++;
}
void
IePreq::IncrementMetric (uint32_t metric)
{
  m_metric += metric;
}
void
IePreq::UpdateVBMetricSum (uint32_t GammPrimMilliwatts, uint32_t BPrimMillijoules)
{
  m_gammaPrimMilliwatts+=GammPrimMilliwatts;
  m_bPrimMillijoules+=BPrimMillijoules;
}
void
IePreq::UpdateVBMetricMin (uint32_t GammPrimMilliwatts, uint32_t BPrimMillijoules)
{
  if(m_gammaPrimMilliwatts>GammPrimMilliwatts)
    m_gammaPrimMilliwatts=GammPrimMilliwatts;
  if(m_bPrimMillijoules>BPrimMillijoules)
    m_bPrimMillijoules=BPrimMillijoules;
}
uint32_t
IePreq::GetGammaPrim()
{
  return m_gammaPrimMilliwatts;
}

uint32_t
IePreq::GetBPrim()
{
  return m_bPrimMillijoules;
}

void
IePreq::SerializeInformationField (Buffer::Iterator i) const
{
  i.WriteU8 (m_flags);
  i.WriteU8 (m_hopCount);
  i.WriteU8 (m_ttl);
  i.WriteHtolsbU32 (m_preqId);
  WriteTo (i, m_originatorAddress);
  i.WriteHtolsbU32 (m_originatorSeqNumber);
  i.WriteHtolsbU32 (m_lifetime);
  i.WriteHtolsbU32 (m_metric);
  i.WriteU8(m_cnnType);
  WriteTo(i,m_srcIpv4Addr);
  WriteTo(i,m_dstIpv4Addr);
  i.WriteHtolsbU16(m_srcPort);
  i.WriteHtolsbU16(m_dstPort);
  i.WriteU8 (m_destCount);
  i.WriteHtolsbU16 (m_rho);
  i.WriteHtolsbU16 (m_sigma);
  i.WriteHtolsbU64 (m_stopTime.GetNanoSeconds());
  i.WriteU32 (m_gammaPrimMilliwatts);
  i.WriteU32 (m_bPrimMillijoules);
  int written = 0;
  for (std::vector<Ptr<DestinationAddressUnit> >::const_iterator j = m_destinations.begin (); j
       != m_destinations.end (); j++)
    {
      uint8_t flags = 0;
      if ((*j)->IsDo ())
        {
          flags |= 1 << 0;
        }
      if ((*j)->IsRf ())
        {
          flags |= 1 << 1;
        }
      if ((*j)->IsUsn ())
        {
          flags |= 1 << 2;
        }
      i.WriteU8 (flags);
      WriteTo (i, (*j)->GetDestinationAddress ());
      i.WriteHtolsbU32 ((*j)->GetDestSeqNumber ());
      written++;
      if (written > m_maxSize)
        {
          break;
        }
    }
}
uint8_t
IePreq::DeserializeInformationField (Buffer::Iterator start, uint8_t length)
{
  Buffer::Iterator i = start;
  m_flags = i.ReadU8 ();
  m_hopCount = i.ReadU8 ();
  m_ttl = i.ReadU8 ();
  m_preqId = i.ReadLsbtohU32 ();
  ReadFrom (i, m_originatorAddress);
  m_originatorSeqNumber = i.ReadLsbtohU32 ();
  m_lifetime = i.ReadLsbtohU32 ();
  m_metric = i.ReadLsbtohU32 ();
  m_cnnType = i.ReadU8();
  ReadFrom(i, m_srcIpv4Addr);
  ReadFrom(i, m_dstIpv4Addr);
  m_srcPort = i.ReadLsbtohU16();
  m_dstPort = i.ReadLsbtohU16();
  m_destCount = i.ReadU8 ();
  m_rho = i.ReadLsbtohU16 ();
  m_sigma =i.ReadLsbtohU16 ();
  m_stopTime = NanoSeconds(i.ReadLsbtohU64 ());
  m_gammaPrimMilliwatts=i.ReadU32 ();
  m_bPrimMillijoules=i.ReadU32 ();
  for (int j = 0; j < m_destCount; j++)
    {
      Ptr<DestinationAddressUnit> new_element = Create<DestinationAddressUnit> ();
      bool doFlag = false;
      bool rfFlag = false;
      bool usnFlag = false;
      uint8_t flags = i.ReadU8 ();
      if (flags & (1 << 0))
        {
          doFlag = true;
        }
      if (flags & (1 << 1))
        {
          rfFlag = true;
        }
      if (flags & (1 << 2))
        {
          usnFlag = true;
        }
      new_element->SetFlags (doFlag, rfFlag, usnFlag);
      Mac48Address addr;
      ReadFrom (i, addr);
      new_element->SetDestinationAddress (addr);
      new_element->SetDestSeqNumber (i.ReadLsbtohU32 ());
      m_destinations.push_back (new_element);
      NS_ASSERT (40 + j * 11 < length);
    }
  return i.GetDistanceFrom (start);
}
uint8_t
IePreq::GetInformationFieldSize () const
{
  uint8_t retval = 1 //Flags
    + 1   //Hopcount
    + 1   //TTL
    + 4   //PREQ ID
    + 6   //Source address (originator)
    + 4   //Originator seqno
    + 4   //Lifetime
    + 4   //metric
    + 1   //cnnType
    + 4   //srcIpv4Addr
    + 4   //dstIpv4Addr
    + 2   //srcPort
    + 2   //dstPort
    + 2   // Rho
    + 2   // Sigma
    + 8   // Stop
    + 4   // gammaPrim
    + 4   // bPrim
    + 1;   //destination count
  if (m_destCount > m_maxSize)
    {
      retval += (m_maxSize * 11);
    }
  else
    {
      retval += (m_destCount * 11);
    }
  return retval;
}
void
IePreq::Print (std::ostream &os) const
{
  os << std::endl << "<information_element id=" << ElementId () << ">" << std::endl;
  os << " originator address  = " << m_originatorAddress << std::endl;
  os << " TTL                 = " << (uint16_t) m_ttl << std::endl;
  os << " hop count           = " << (uint16_t) m_hopCount << std::endl;
  os << " metric              = " << m_metric << std::endl;
  os << " seqno               = " << m_originatorSeqNumber << std::endl;
  os << " lifetime            = " << m_lifetime << std::endl;
  os << " preq ID             = " << m_preqId << std::endl;
  os << " Destinations are:" << std::endl;
  for (int j = 0; j < m_destCount; j++)
    {
      os << "    " << m_destinations[j]->GetDestinationAddress () << std::endl;
    }
  os << "</information_element>" << std::endl;
}
std::vector<Ptr<DestinationAddressUnit> >
IePreq::GetDestinationList ()
{
  return m_destinations;
}
void
IePreq::AddDestinationAddressElement (bool doFlag, bool rfFlag, Mac48Address dest_address,
                                      uint32_t dest_seq_number)
{
  for (std::vector<Ptr<DestinationAddressUnit> >::const_iterator i = m_destinations.begin (); i
       != m_destinations.end (); i++)
    {
      if ((*i)->GetDestinationAddress () == dest_address)
        {
          return;
        }
    }
  /// \todo check overflow
  Ptr<DestinationAddressUnit> new_element = Create<DestinationAddressUnit> ();
  new_element->SetFlags (doFlag, rfFlag, (dest_seq_number == 0));
  new_element->SetDestinationAddress (dest_address);
  new_element->SetDestSeqNumber (dest_seq_number);
  m_destinations.push_back (new_element);
  m_destCount++;
}
void
IePreq::DelDestinationAddressElement (Mac48Address dest_address)
{
  for (std::vector<Ptr<DestinationAddressUnit> >::iterator i = m_destinations.begin (); i
       != m_destinations.end (); i++)
    {
      if ((*i)->GetDestinationAddress () == dest_address)
        {
          m_destinations.erase (i);
          m_destCount--;
          break;
        }
    }
}
void
IePreq::ClearDestinationAddressElements ()
{
  for (std::vector<Ptr<DestinationAddressUnit> >::iterator j = m_destinations.begin (); j
       != m_destinations.end (); j++)
    {
      (*j) = 0;
    }
  m_destinations.clear ();
  m_destCount = 0;
}
bool
operator== (const DestinationAddressUnit & a, const DestinationAddressUnit & b)
{
  return (a.m_do == b.m_do && a.m_rf == b.m_rf && a.m_usn == b.m_usn && a.m_destinationAddress
          == b.m_destinationAddress && a.m_destSeqNumber == b.m_destSeqNumber);
}
bool
operator== (const IePreq & a, const IePreq & b)
{
  bool ok = (a.m_flags == b.m_flags && a.m_hopCount == b.m_hopCount && a.m_ttl == b.m_ttl && a.m_preqId
             == b.m_preqId && a.m_originatorAddress == b.m_originatorAddress && a.m_originatorSeqNumber
             == b.m_originatorSeqNumber && a.m_lifetime == b.m_lifetime && a.m_metric == b.m_metric && a.m_destCount
             == b.m_destCount);

  if (!ok)
    {
      return false;
    }
  if (a.m_destinations.size () != b.m_destinations.size ())
    {
      return false;
    }
  for (size_t i = 0; i < a.m_destinations.size (); ++i)
    {
      if (!(*(PeekPointer (a.m_destinations[i])) == *(PeekPointer (b.m_destinations[i]))))
        {
          return false;
        }
    }
  return true;
}
bool
IePreq::MayAddAddress (Mac48Address originator)
{
  if (m_originatorAddress != originator)
    {
      return false;
    }
  if (m_destinations[0]->GetDestinationAddress () == Mac48Address::GetBroadcast ())
    {
      return false;
    }
  if ((GetInformationFieldSize () + 11) > 255)
    {
      return false;
    }
  return true;
}
bool
IePreq::IsFull () const
{
  return ((GetInformationFieldSize () + 11) > 255);
}
std::ostream &
operator << (std::ostream &os, const IePreq &a)
{
  a.Print (os);
  return os;
}
} // namespace dot11s
} // namespace ns3

