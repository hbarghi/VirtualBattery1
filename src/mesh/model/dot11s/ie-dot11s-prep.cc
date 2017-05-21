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

#include "ie-dot11s-prep.h"
#include "ns3/address-utils.h"
#include "ns3/assert.h"
#include "ns3/packet.h"
namespace ns3 {
namespace dot11s {
/********************************
 * IePrep
 *******************************/
IePrep::~IePrep ()
{
}
IePrep::IePrep () :
  m_flags (0), m_hopcount (0), m_ttl (0), m_destinationAddress (Mac48Address::GetBroadcast ()),
  m_destSeqNumber (0), m_lifetime (0), m_metric (0), 
        m_cnnType(1),
        m_rho (0),
        m_sigma (0),
        m_stopTime (Seconds(0)),
        m_srcIpv4Addr("10.0.0.1"),
        m_dstIpv4Addr("10.0.0.2"),
        m_srcPort(25000),
        m_dstPort(25001),
  m_originatorAddress (Mac48Address::GetBroadcast ()),
  m_originatorSeqNumber (0)
{
}
WifiInformationElementId
IePrep::ElementId () const
{
  return IE11S_PREP;
}
void
IePrep::SetFlags (uint8_t flags)
{
  m_flags = flags;
}
void
IePrep::SetHopcount (uint8_t hopcount)
{
  m_hopcount = hopcount;
}
void
IePrep::SetTtl (uint8_t ttl)
{
  m_ttl = ttl;
}
void
IePrep::SetDestinationSeqNumber (uint32_t destSeqNumber)
{
  m_destSeqNumber = destSeqNumber;
}
void
IePrep::SetDestinationAddress (Mac48Address destAddress)
{
  m_destinationAddress = destAddress;
}
void
IePrep::SetMetric (uint32_t metric)
{
  m_metric = metric;
}
void
IePrep::SetCnnParams(uint8_t cnnType,Ipv4Address srcIpv4Addr,Ipv4Address dstIpv4Addr,uint16_t srcPort,uint16_t dstPort)
{
    m_cnnType=cnnType;
    m_srcIpv4Addr=srcIpv4Addr;
    m_srcPort=srcPort;
    m_dstIpv4Addr=dstIpv4Addr;
    m_dstPort=dstPort;
}
void
IePrep::SetRho (uint16_t rho)
{
  m_rho = rho;
}
void
IePrep::SetSigma (uint16_t sigma)
{
  m_sigma = sigma;
}
void
IePrep::SetStopTime (Time stop)
{
  m_stopTime = stop;
}
void
IePrep::SetOriginatorAddress (Mac48Address originatorAddress)
{
  m_originatorAddress = originatorAddress;
}
void
IePrep::SetOriginatorSeqNumber (uint32_t originatorSeqNumber)
{
  m_originatorSeqNumber = originatorSeqNumber;
}
void
IePrep::SetLifetime (uint32_t lifetime)
{
  m_lifetime = lifetime;
}
uint8_t
IePrep::GetFlags () const
{
  return m_flags;
}
uint8_t
IePrep::GetHopcount () const
{
  return m_hopcount;
}
uint32_t
IePrep::GetTtl () const
{
  return m_ttl;
}
uint32_t
IePrep::GetDestinationSeqNumber () const
{
  return m_destSeqNumber;
}
Mac48Address
IePrep::GetDestinationAddress () const
{
  return m_destinationAddress;
}
uint32_t
IePrep::GetMetric () const
{
  return m_metric;
}
uint8_t
IePrep::GetCnnType () const
{
    return m_cnnType;
}
Ipv4Address
IePrep::GetSrcIpv4Addr () const
{
    return m_srcIpv4Addr;
}
Ipv4Address
IePrep::GetDstIpv4Addr () const
{
    return m_dstIpv4Addr;
}
uint16_t
IePrep::GetSrcPort () const
{
    return m_srcPort;
}
uint16_t
IePrep::GetDstPort () const
{
    return m_dstPort;
}
Mac48Address
IePrep::GetOriginatorAddress () const
{
  return m_originatorAddress;
}
uint32_t
IePrep::GetOriginatorSeqNumber () const
{
  return m_originatorSeqNumber;
}
uint32_t
IePrep::GetLifetime () const
{
  return m_lifetime;
}
uint16_t
IePrep::GetRho ()
{
  return m_rho;
}
uint16_t
IePrep::GetSigma ()
{
  return m_sigma;
}
Time
IePrep::GetStopTime ()
{
  return m_stopTime;
}
void
IePrep::DecrementTtl ()
{
  m_ttl--;
  m_hopcount++;
}

void
IePrep::IncrementMetric (uint32_t metric)
{
  m_metric += metric;
}

void
IePrep::SerializeInformationField (Buffer::Iterator i) const
{
  i.WriteU8 (m_flags);
  i.WriteU8 (m_hopcount);
  i.WriteU8 (m_ttl);
  WriteTo (i, m_destinationAddress);
  i.WriteHtolsbU32 (m_destSeqNumber);
  i.WriteHtolsbU32 (m_lifetime);
  i.WriteHtolsbU32 (m_metric);
  i.WriteU8(m_cnnType);
  WriteTo(i,m_srcIpv4Addr);
  WriteTo(i,m_dstIpv4Addr);
  i.WriteHtolsbU16(m_srcPort);
  i.WriteHtolsbU16(m_dstPort);
  WriteTo (i, m_originatorAddress);
  i.WriteHtolsbU32 (m_originatorSeqNumber);
  i.WriteHtolsbU16 (m_rho);
  i.WriteHtolsbU16 (m_sigma);
  i.WriteHtolsbU64 (m_stopTime.GetNanoSeconds());
  i.WriteHtolsbU64 (m_delayBound.GetNanoSeconds ());
  i.WriteHtolsbU16 (m_maxPktSize);
}
uint8_t
IePrep::DeserializeInformationField (Buffer::Iterator start, uint8_t length)
{
  Buffer::Iterator i = start;
  m_flags = i.ReadU8 ();
  m_hopcount = i.ReadU8 ();
  m_ttl = i.ReadU8 ();
  ReadFrom (i, m_destinationAddress);
  m_destSeqNumber = i.ReadLsbtohU32 ();
  m_lifetime = i.ReadLsbtohU32 ();
  m_metric = i.ReadLsbtohU32 ();
  m_cnnType = i.ReadU8();
  ReadFrom(i, m_srcIpv4Addr);
  ReadFrom(i, m_dstIpv4Addr);
  m_srcPort = i.ReadLsbtohU16();
  m_dstPort = i.ReadLsbtohU16();
  ReadFrom (i, m_originatorAddress);
  m_originatorSeqNumber = i.ReadLsbtohU32 ();
  m_rho = i.ReadLsbtohU16 ();
  m_sigma =i.ReadLsbtohU16 ();
  m_stopTime = NanoSeconds(i.ReadLsbtohU64 ());
  m_delayBound=NanoSeconds(i.ReadLsbtohU64 ());
  m_maxPktSize=i.ReadLsbtohU16 ();
  return i.GetDistanceFrom (start);
}
uint8_t
IePrep::GetInformationFieldSize () const
{
  uint32_t retval = 1 //Flags
    + 1   //Hopcount
    + 1   //Ttl
    + 6   //Dest address
    + 4   //Dest seqno
    + 4   //Lifetime
    + 4   //metric
    + 1   //cnnType
    + 2   // Rho
    + 2   // Sigma
    + 8   // Stop
    + 8   // delay bound
    + 2   //max packet size
    + 4   //srcIpv4Addr
    + 4   //dstIpv4Addr
    + 2   //srcPort
    + 2   //dstPort
    + 6   //Originator address
    + 4;   //Originator seqno
  return retval;
}
void
IePrep::Print (std::ostream& os) const
{
  os << std::endl << "<information_element id=" << ElementId () << ">" << std::endl;
  os << "Flags:        = " << m_flags << std::endl << "Hopcount:     = " << m_hopcount << std::endl << "TTL:          = " << m_ttl
     << std::endl<< "Destination:  = " << m_destinationAddress << std::endl << "Dest. seqnum: = " << m_destSeqNumber
     << std::endl << "Lifetime:     = " << m_lifetime << std::endl<< "Metric:       = " << m_metric << std::endl << "Originator:   = "
     << m_originatorAddress << std::endl << "Orig. seqnum: = " << m_originatorSeqNumber << std::endl;
  os << "</information_element>" << std::endl;
}

Time IePrep::GetDelayBound() const
{
    return m_delayBound;
}

void IePrep::SetDelayBound(const Time &delayBound)
{
    m_delayBound = delayBound;
}

uint16_t IePrep::GetMaxPktSize() const
{
    return m_maxPktSize;
}

void IePrep::SetMaxPktSize(const uint16_t &maxPktSize)
{
    m_maxPktSize = maxPktSize;
}
bool
operator== (const IePrep & a, const IePrep & b)
{
    return ((a.m_flags == b.m_flags) && (a.m_hopcount == b.m_hopcount) && (a.m_ttl == b.m_ttl)
          && (a.m_destinationAddress == b.m_destinationAddress) && (a.m_destSeqNumber == b.m_destSeqNumber)
          && (a.m_lifetime == b.m_lifetime) && (a.m_metric == b.m_metric) && (a.m_originatorAddress
                                                                              == b.m_originatorAddress) && (a.m_originatorSeqNumber == b.m_originatorSeqNumber));
}
std::ostream &
operator << (std::ostream &os, const IePrep &a)
{
  a.Print (os);
  return os;
}
} // namespace dot11s
} // namespace ns3

