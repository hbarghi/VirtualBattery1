/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 Saeed Asaiyan
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
 * Author: Saeed Asaiyan <s.asaiyan@gmail.com>
 */


#include "ns3/tag.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "rhoSigma-tag.h"

#include <iostream>

namespace ns3{

NS_OBJECT_ENSURE_REGISTERED (RouSigmaTag);

TypeId
RouSigmaTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RouSigmaTag")
    .SetParent<Tag> ()
    .AddConstructor<RouSigmaTag> ()
    .AddAttribute ("Rou",
                   "A Rou value",
                   EmptyAttributeValue (),
                   MakeUintegerAccessor (&RouSigmaTag::SetRou,
                                         &RouSigmaTag::GetRou),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("Sigma",
                   "A Sigma value",
                   EmptyAttributeValue (),
                   MakeUintegerAccessor (&RouSigmaTag::SetSigma,
                                         &RouSigmaTag::GetSigma),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("ConnectionTime",
                  "The connection duration",
                  EmptyAttributeValue(),
                  MakeTimeAccessor (&RouSigmaTag::SetStopTime,
                                    &RouSigmaTag::GetStopTime),
                  MakeTimeChecker (Seconds(0)))
  ;
  return tid;
}
TypeId
RouSigmaTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
uint32_t
RouSigmaTag::GetSerializedSize (void) const
{
  return 12;
}
void
RouSigmaTag::Serialize (TagBuffer i) const
{
  i.WriteU16 (m_rou);
  i.WriteU64 (m_stopTime.GetMicroSeconds());
  i.WriteU16 (m_sigma);
}
void
RouSigmaTag::Deserialize (TagBuffer i)
{
  m_rou = i.ReadU16 ();
  m_stopTime = MicroSeconds(i.ReadU64());
  m_sigma = i.ReadU16 ();
}
void
RouSigmaTag::Print (std::ostream &os) const
{
  os << "rou=" << m_rou;
  os << " sigma=" << m_sigma;
  os << " Connection Time=" << m_stopTime;
}
void
RouSigmaTag::SetRou (uint16_t value)
{
  m_rou = value;
}
uint16_t
RouSigmaTag::GetRou (void) const
{
  return m_rou;
}
void
RouSigmaTag::SetSigma(uint16_t value)
{
  m_sigma = value;
}
uint16_t
RouSigmaTag::GetSigma (void) const
{
  return m_sigma;
}

void
RouSigmaTag::SetStopTime(Time value)
{
  m_stopTime = value;
}
Time
RouSigmaTag::GetStopTime (void) const
{
  return m_stopTime;
}

} // namespace ns3
