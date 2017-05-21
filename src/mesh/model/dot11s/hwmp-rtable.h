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

#ifndef HWMP_RTABLE_H
#define HWMP_RTABLE_H

#include <map>
#include "ns3/nstime.h"
#include "ns3/mac48-address.h"
#include "ns3/hwmp-protocol.h"

namespace ns3 {
namespace dot11s {

  class TokenBucketVirtualBattery : public Object
  {
  public:
    static TypeId GetTypeId ();
    TokenBucketVirtualBattery ();
    ~TokenBucketVirtualBattery ();
    void DoDispose ();

    void UpdateToken();
    double m_numTokenPacket;//current tokens
    double m_maxTokenPacket;//sigma
    double m_numTokensPerMillisecond;//rho

    double m_gamma;
    double m_b;
    double m_bMax;

    double m_maxEnergyPerDataPacket;
    double m_maxEnergyPerAckPacket;

    uint8_t cnnType;
    Ipv4Address srcIpv4Addr;
    Ipv4Address dstIpv4Addr;
    uint16_t srcPort;
    uint16_t dstPort;

    uint64_t m_id;

    int m_predictedNumberOfPackets,m_realNumberOfPackets;
    double m_predictedEnergy,m_realEnergy;

    struct QueuedPacket
    {
      Ptr<Packet> pkt; ///< the packet
      Mac48Address src; ///< src address
      Mac48Address dst; ///< dst address
      uint16_t protocol; ///< protocol number
      uint8_t cnnType;
      Ipv4Address srcIpv4Addr;
      Ipv4Address dstIpv4Addr;
      uint16_t srcPort;
      uint16_t dstPort;
      Time whenQueued;
      uint32_t interface; ///< incoming device interface ID. (if packet has come from upper layers, this is Mesh point ID)
      Callback<void, /* return type */
                         bool, /* flag */
                         Ptr<Packet>, /* packet */
                         Mac48Address, /* src */
                         Mac48Address, /* dst */
                         uint16_t, /* protocol */
                         uint32_t /* out interface ID */
                         > reply; ///< how to reply

      QueuedPacket ();
    };

    uint16_t m_maxQueueSize;
    double m_delayBoundSeconds;

    std::vector<QueuedPacket> m_rqueue;
    bool QueuePacket (QueuedPacket packet);

  };


/**
 * \ingroup dot11s
 *
 * \brief Routing table for HWMP -- 802.11s routing protocol
 */
class HwmpRtable : public Object
{
public:
  /// Means all interfaces
  const static uint32_t INTERFACE_ANY = 0xffffffff;
  /// Maximum (the best?) path metric
  const static uint32_t MAX_METRIC = 0xffffffff;
  static const uint8_t CNN_TYPE_MAC_ONLY = 1;
  static const uint8_t CNN_TYPE_IP_ONLY = 2;
  static const uint8_t CNN_TYPE_IP_PORT = 3;
  static const uint8_t CNN_TYPE_PKT_BASED = 4;

  /// Route lookup result, return type of LookupXXX methods
  struct CnnBasedLookupResult
  {
    Mac48Address retransmitter;
    Mac48Address precursor;
    uint32_t ifIndex;
    uint32_t seqnum;
    Time lifetime;
    CnnBasedLookupResult (Mac48Address r = Mac48Address::GetBroadcast (),
                  Mac48Address p = Mac48Address::GetBroadcast (),
                  uint32_t i = INTERFACE_ANY,
                  uint32_t s = 0,
                  Time l = Seconds (0.0));
    /// True for valid route
    bool IsValid () const;
    /// Compare route lookup results, used by tests
    bool operator== (const CnnBasedLookupResult & o) const;
  };

  struct LookupResult
  {
    Mac48Address retransmitter;
    uint32_t ifIndex;
    uint32_t metric;
    uint32_t seqnum;
    Time lifetime;
    LookupResult (Mac48Address r = Mac48Address::GetBroadcast (),
                  uint32_t i = INTERFACE_ANY,
                  uint32_t m = MAX_METRIC,
                  uint32_t s = 0,
                  Time l = Seconds (0.0));
    /// True for valid route
    bool IsValid () const;
    /// Compare route lookup results, used by tests
    bool operator== (const LookupResult & o) const;
  };
  /// Path precursor = {MAC, interface ID}
  typedef std::vector<std::pair<uint32_t, Mac48Address> > PrecursorList;

public:
  static TypeId GetTypeId ();
  HwmpRtable ();
  ~HwmpRtable ();
  void DoDispose ();

  ///\name Add/delete paths
  //\{
  void AddReactivePath (
    Mac48Address destination,
    Mac48Address retransmitter,
    uint32_t interface,
    uint32_t metric,
    Time  lifetime,
    uint32_t seqnum
    );
  void AddProactivePath (
    uint32_t metric,
    Mac48Address root,
    Mac48Address retransmitter,
    uint32_t interface,
    Time  lifetime,
    uint32_t seqnum
    );
  void AddPrecursor (Mac48Address destination, uint32_t precursorInterface, Mac48Address precursorAddress, Time lifetime);
  PrecursorList GetPrecursors (Mac48Address destination);
  void DeleteProactivePath ();
  void DeleteProactivePath (Mac48Address root);
  void DeleteReactivePath (Mac48Address destination);
  //\}

  ///\name Lookup
  //\{
  /// Lookup path to destination
  LookupResult LookupReactive (Mac48Address destination);
  /// Return all reactive paths, including expired
  LookupResult LookupReactiveExpired (Mac48Address destination);
  /// Find proactive path to tree root. Note that calling this method has side effect of deleting expired proactive path
  LookupResult LookupProactive ();
  /// Return all proactive paths, including expired
  LookupResult LookupProactiveExpired ();
  //\}
  void SetMaxEnergyPerDataPacket(double energy);
  double GetMaxEnergyPerDataPacket();
  void SetMaxEnergyPerAckPacket(double energy);
  double GetMaxEnergyPerAckPacket();

  void TotalEnergyIncreasedByGamma (double energy);
  void ControlEnergyIncreasedByCollisionEnergyBack (double energy);
  void BPrimPacketsEnergyDecreased(double energy);
  void ControlPacketsEnergyDecreased (double energy);

  void ChangeEnergy4aConnection (
      uint8_t cnnType,
      Ipv4Address srcIpv4Addr,
      Ipv4Address dstIpv4Addr,
      uint16_t srcPort,
      uint16_t dstPort,
      double energy,
      bool incDec
    );

  /// When peer link with a given MAC-address fails - it returns list of unreachable destination addresses
  std::vector<HwmpProtocol::FailedDestination> GetUnreachableDestinations (Mac48Address peerAddress);

  bool AddCnnBasedReactivePath (Mac48Address destination,
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
    Time stopTime,
    Time delayBound,
    uint16_t maxPktSize,
    Time  lifetime,
    uint32_t seqnum,
    bool intermediate,
    bool doCAC);
  void QueueCnnBasedPacket(
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
      );
  void DeleteCnnBasedReactivePath (
    Mac48Address destination,
    Mac48Address source,
    uint8_t cnnType,
    Ipv4Address srcIpv4Addr,
    Ipv4Address dstIpv4Addr,
    uint16_t srcPort,
    uint16_t dstPort
    );
  CnnBasedLookupResult LookupCnnBasedReactive (
    Mac48Address destination,
    Mac48Address source,
    uint8_t cnnType,
    Ipv4Address srcIpv4Addr,
    Ipv4Address dstIpv4Addr,
    uint16_t srcPort,
    uint16_t dstPort
    );

  void AddCnnBasedReversePath (
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
    );
  void DeleteCnnBasedReversePath (
    Mac48Address destination,
    uint8_t cnnType,
    Ipv4Address srcIpv4Addr,
    Ipv4Address dstIpv4Addr,
    uint16_t srcPort,
    uint16_t dstPort
    );
  CnnBasedLookupResult LookupCnnBasedReverse (
    Mac48Address destination,
    uint8_t cnnType,
    Ipv4Address srcIpv4Addr,
    Ipv4Address dstIpv4Addr,
    uint16_t srcPort,
    uint16_t dstPort
    );

  double m_maxEnergyPerDataPacket;
  double m_maxEnergyPerAckPacket;

  double m_energyAlpha;

  void UpdateToken();

  double systemGamma() const;
  void setSystemGamma(double systemGamma);

  double systemB() const;
  void setSystemB(double systemB);

  double systemBMax() const;
  void setSystemBMax(double systemBMax);

  double gammaPrim() const;
  void setGammaPrim(double gammaPrim);

  double bPrim() const;
  void setBPrim(double bPrim);

  double bPrimMax() const;
  void setBPrimMax(double bPrimMax);

  double assignedGamma() const;
  void setAssignedGamma(double assignedGamma);

  double controlGamma() const;
  void setControlGamma(double controlGamma);

  double controlB() const;
  void setControlB(double controlB);

  double controlBMax() const;
  void setControlBMax(double controlBMax);

  uint16_t Gppm() const;
  void setGppm(const uint16_t &Gppm);

private:
  /// Route found in reactive mode
  struct Precursor
  {
    Mac48Address address;
    uint32_t interface;
    Time whenExpire;
  };
  struct ReactiveRoute
  {
    Mac48Address retransmitter;
    uint32_t interface;
    uint32_t metric;
    Time whenExpire;
    uint32_t seqnum;
    std::vector<Precursor> precursors;
  };
  /// Route fond in proactive mode
  struct ProactiveRoute
  {
    Mac48Address root;
    Mac48Address retransmitter;
    uint32_t interface;
    uint32_t metric;
    Time whenExpire;
    uint32_t seqnum;
    std::vector<Precursor> precursors;
  };

  /// List of routes
  std::map<Mac48Address, ReactiveRoute>  m_routes;
  /// Path to proactive tree root MP
  ProactiveRoute  m_root;
  struct CnnBasedReactiveRoute
  {
    Mac48Address destination;
    Mac48Address retransmitter;
    Mac48Address source;
    Mac48Address precursor;
    uint32_t interface;
    uint8_t cnnType;
    Ipv4Address srcIpv4Addr;
    Ipv4Address dstIpv4Addr;
    uint16_t srcPort;
    uint16_t dstPort;
    Time whenExpire;
    uint32_t seqnum;

    Ptr<TokenBucketVirtualBattery> tokenBucketVirtualBattery;


  };
  struct CnnBasedReverseRoute
  {
    Mac48Address destination;
    Mac48Address retransmitter;
    uint32_t interface;
    uint8_t cnnType;
    Ipv4Address srcIpv4Addr;
    Ipv4Address dstIpv4Addr;
    uint16_t srcPort;
    uint16_t dstPort;
    Time whenExpire;
    uint32_t seqnum;
  };
  std::vector<CnnBasedReactiveRoute>  m_cnnBasedRoutes;
  std::vector<CnnBasedReverseRoute>  m_cnnBasedReverse;

  double m_systemGamma;
  double m_systemB;
  double m_systemBMax;//battery capacity
  double m_gammaPrim;
  double m_bPrim;
  double m_bPrimMax;
  double m_assignedGamma;

  uint16_t m_Gppm;

  double m_controlGamma;
  double m_controlB;
  double m_controlBMax;

};
} // namespace dot11s
} // namespace ns3
#endif
