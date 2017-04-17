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

#ifndef ROU_SIGMA_TAG_H_
#define ROU_SIGMA_TAG_H_

#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/nstime.h"
#include <iostream>

namespace ns3
{
class Tag;

class RouSigmaTag : public Tag
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual void Print (std::ostream &os) const;

  // these are our accessors to our tag structure
  void SetRou (uint16_t value);
  uint16_t GetRou (void) const;
  void SetSigma (uint16_t value);
  uint16_t GetSigma (void) const;
  void SetStopTime (Time value);
  Time GetStopTime (void) const;
private:
  uint16_t m_rou;
  uint16_t m_sigma;
  Time m_stopTime;
};

}

#endif /* ROU_SIGMA_TAG_H_ */
