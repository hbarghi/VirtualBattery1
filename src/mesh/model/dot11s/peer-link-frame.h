/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
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
 * Authors: Kirill Andreev <andreev@iitp.ru>
 */

#ifndef PEER_LINK_FRAME_START_H
#define PEER_LINK_FRAME_START_H
#include "ns3/header.h"
#include "ns3/supported-rates.h"
#include "dot11s-mac-header.h"
#include "ie-dot11s-configuration.h"
#include "ie-dot11s-peering-protocol.h"
#include "ie-dot11s-id.h"
namespace ns3
{
class MeshWifiInterfaceMac;
namespace dot11s
{
/**
 * \ingroup dot11s
 *
 * \brief 802.11s Peer link management frame
 * 
 * Peer link management frame included the following (see chapters 7.4.12.1-7.4.12.3 of 802.11s):
 * - Subtype field
 * - Association ID field
 * - Supported rates
 * - Mesh ID of mesh
 */

class PeerLinkOpenFrame : public Header
{
public:
  PeerLinkOpenFrame ();
  PeerLinkOpenFrame (const IePeeringProtocol & protocol,
                     uint16_t capability,
                     const SupportedRates & supportedRates,
                     const IeMeshId & meshId,
                    const IeConfiguration & config);
  ///\name Inherited from header:
  ///\{
  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const;
  void Print (std::ostream &os) const;
  uint32_t GetSerializedSize () const;
  void Serialize (Buffer::Iterator start) const;
  uint32_t Deserialize (Buffer::Iterator start);
  ///\}
private:
  /// 3 octets protocol version
  IePeeringProtocol m_protocolVersion;
  /// capability fiels
  uint16_t m_capability;
  /// Supported rates:
  SupportedRates m_supportedRates;
  /// Mesh network identificator
  IeMeshId m_meshId;
  /// Mesh configuration:
  IeConfiguration m_config;     //open and confirm
};

class PeerLinkConfirmFrame : public Header
{
public:
  PeerLinkConfirmFrame ();
  PeerLinkConfirmFrame (const IePeeringProtocol & protocol,
                        uint16_t capability,
                        uint16_t assocId,
                        const SupportedRates & supportedRates,
                        const IeConfiguration & config);
  ///\name Inherited from header:
  ///\{
  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const;
  void Print (std::ostream &os) const;
  uint32_t GetSerializedSize () const;
  void Serialize (Buffer::Iterator start) const;
  uint32_t Deserialize (Buffer::Iterator start);
  ///\}
private:
  /// 3 octets protocol version
  IePeeringProtocol m_protocolVersion;
  /// capability fiels
  uint16_t m_capability;
  uint16_t m_aid;
  SupportedRates m_supportedRates;
  IeConfiguration m_config;
};

class PeerLinkCloseFrame : public Header
{
public:
  PeerLinkCloseFrame ();
  PeerLinkCloseFrame (const IePeeringProtocol & protocol,
                      const IeMeshId & meshId,
                      uint16_t reason);
  ///\name Inherited from header:
  ///\{
  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const;
  void Print (std::ostream &os) const;
  uint32_t GetSerializedSize () const;
  void Serialize (Buffer::Iterator start) const;
  uint32_t Deserialize (Buffer::Iterator start);
  ///\}
private:
  IePeeringProtocol m_protocolVersion;
  IeMeshId m_meshId;
  uint16_t m_reasonCode;
};
class PeerLinkFrameStart : public Header
{
public:
  PeerLinkFrameStart ();
  ///\brief fields:
  struct PlinkFrameStartFields
  {
    uint8_t subtype;
    IePeeringProtocol protocol; //Peering protocol version - in all subtypes - 3 octets
    uint16_t capability;        //open and confirm
    uint16_t aid;               //confirm only
    SupportedRates rates;       //open and confirm
    IeMeshId meshId;            //open and close
    IeConfiguration config;     //open and confirm
    uint16_t reasonCode;        //close only
  };
  ///\attention: must be set before deserialize, before only multihop
  //action header knows about subtype
  void SetPlinkFrameSubtype (uint8_t subtype);
  void SetPlinkFrameStart (PlinkFrameStartFields);
  PlinkFrameStartFields GetFields () const;
  /** \name Inherited from header:
   * \{
   */
  static  TypeId   GetTypeId ();
  virtual TypeId   GetInstanceTypeId () const;
  virtual void     Print (std::ostream &os) const;
  virtual uint32_t GetSerializedSize () const;
  virtual void     Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  /**
   * \}
   */
private:
  uint8_t m_subtype;
  IePeeringProtocol m_protocol;
  uint16_t m_capability;
  uint16_t m_aid;
  SupportedRates m_rates;
  IeMeshId m_meshId;
  IeConfiguration m_config;
  uint16_t m_reasonCode;

  friend bool operator== (const PeerLinkFrameStart & a, const PeerLinkFrameStart & b);

  PeerLinkFrameStart& operator= (const PeerLinkFrameStart &);
  PeerLinkFrameStart (const PeerLinkFrameStart &);

};
bool operator== (const PeerLinkFrameStart & a, const PeerLinkFrameStart & b);
} // namespace dot11s
} // namespace ns3
#endif
