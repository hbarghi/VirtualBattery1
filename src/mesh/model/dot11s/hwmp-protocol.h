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
 * Authors: Kirill Andreev <andreev@iitp.ru>
 */

#ifndef HWMP_PROTOCOL_H
#define HWMP_PROTOCOL_H

#include "ns3/mesh-l2-routing-protocol.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/traced-value.h"
#include <vector>
#include <map>
#include <float.h>
#include <algorithm>

#include "ns3/rhoSigma-tag.h"


#define CBR_ROUTE_EXPIRE_SECONDS 3
#define SOURCE_CBR_ROUTE_EXPIRE_MILLISECONDS 3000

namespace ns3 {
class MeshPointDevice;
class Packet;
class Mac48Address;
class UniformRandomVariable;
namespace dot11s {
class HwmpProtocolMac;
class HwmpRtable;
class IePerr;
class IePreq;
class IePrep;
/**
 * \ingroup dot11s
 *
 * \brief Hybrid wireless mesh protocol -- a routing protocol of IEEE 802.11s draft.
 */

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

class HwmpProtocol : public MeshL2RoutingProtocol
{
public:
  static TypeId GetTypeId ();
  HwmpProtocol ();
  ~HwmpProtocol ();
  void DoDispose ();
  /**
   * \brief structure of unreachable destination - address and sequence number
   */
  typedef struct
  {
    Mac48Address destination;
    uint32_t seqnum;
  } FailedDestination;
  /// Route request, inherited from MeshL2RoutingProtocol
  bool RequestRoute (uint32_t  sourceIface, const Mac48Address source, const Mac48Address destination,
                     Ptr<const Packet>  packet, uint16_t  protocolType, RouteReplyCallback  routeReply);
  /// Cleanup packet from all tags
  bool RemoveRoutingStuff (uint32_t fromIface, const Mac48Address source,
                           const Mac48Address destination, Ptr<Packet>  packet, uint16_t&  protocolType);
  /**
   * \brief Install HWMP on given mesh point.
   *
   * Installing protocol cause installing its interface MAC plugins.
   *
   * Also MP aggregates all installed protocols, HWMP protocol can be accessed
   * via MeshPointDevice::GetObject<dot11s::HwmpProtocol>();
   */
  bool Install (Ptr<MeshPointDevice>);
  void PeerLinkStatus (Mac48Address meshPontAddress, Mac48Address peerAddress, uint32_t interface,bool status);
  ///\brief This callback is used to obtain active neighbours on a given interface
  ///\param cb is a callback, which returns a list of addresses on given interface (uint32_t)
  void SetNeighboursCallback (Callback<std::vector<Mac48Address>, uint32_t> cb);
  ///\name Proactive PREQ mechanism:
  ///\{
  void SetRoot ();
  void UnsetRoot ();
  ///\}
  ///\brief Statistics:
  void Report (std::ostream &) const;
  void ResetStats ();
  /**
   * Assign a fixed random variable stream number to the random variables
   * used by this model.  Return the number of streams (possibly zero) that
   * have been assigned.
   *
   * \param stream first stream index to use
   * \return the number of stream indices assigned by this model
   */
  int64_t AssignStreams (int64_t stream);

  void SetRoutingType(int routingType);

private:
  friend class HwmpProtocolMac;

  virtual void DoInitialize ();

  HwmpProtocol& operator= (const HwmpProtocol &);
  HwmpProtocol (const HwmpProtocol &);

  /**
   * \brief Structure of path error: IePerr and list of receivers:
   * interfaces and MAC address
   */
  struct PathError
  {
    std::vector<FailedDestination> destinations; ///< destination list: Mac48Address and sequence number
    std::vector<std::pair<uint32_t, Mac48Address> > receivers; ///< list of PathError receivers (in case of unicast PERR)
  };
  /// Packet waiting its routing information
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
    uint32_t inInterface; ///< incoming device interface ID. (if packet has come from upper layers, this is Mesh point ID)
    RouteReplyCallback reply; ///< how to reply

    QueuedPacket ();
  };
  struct CnnBasedPreqEvent {
    EventId preqTimeout;
    Time whenScheduled;
    Mac48Address destination;
    Mac48Address source;
    uint8_t cnnType;
    Ipv4Address srcIpv4Addr;
    Ipv4Address dstIpv4Addr;
    uint16_t srcPort;
    uint16_t dstPort;
    uint16_t rho;
    uint16_t sigma;
    Time stopTime;
    Time delayBound;
    uint16_t maxPktSize;
  };
  std::vector<CnnBasedPreqEvent> m_cnnBasedPreqTimeouts;

  struct DelayedPrepStruct {
    EventId prepTimeout;
    Time whenScheduled;
    Mac48Address destination;
    Mac48Address source;
    uint8_t cnnType;
    Ipv4Address srcIpv4Addr;
    Ipv4Address dstIpv4Addr;
    uint16_t srcPort;
    uint16_t dstPort;
    uint16_t rho;
    uint16_t sigma;
    Time stopTime;
    Time delayBound;
    uint16_t maxPktSize;
    uint32_t initMetric;
    uint32_t originatorDsn;
    uint32_t destinationSN;
    uint32_t lifetime;
    uint32_t interface;
    inline bool operator == (const DelayedPrepStruct &o) const {
          return ( o.dstIpv4Addr == dstIpv4Addr && o.dstPort == dstPort && o.srcIpv4Addr == srcIpv4Addr && o.srcPort == srcPort );
    }
  };
  std::vector<DelayedPrepStruct> m_delayedPrepStruct;

  typedef std::map<uint32_t, Ptr<HwmpProtocolMac> > HwmpProtocolMacMap;
  /// Like RequestRoute, but for unicast packets
  bool ForwardUnicast (uint32_t  sourceIface, const Mac48Address source, const Mac48Address destination,
                       Ptr<Packet>  packet, uint16_t  protocolType, RouteReplyCallback  routeReply, uint32_t ttl);

  ///\name Interaction with HWMP MAC plugin
  //\{
  void ReceivePreq (IePreq preq, Mac48Address from, uint32_t interface, Mac48Address fromMp, uint32_t metric);
  void Schedule2sendPrep(
      Mac48Address src,
      Mac48Address dst,
      uint32_t initMetric,
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
      uint32_t originatorDsn,
      uint32_t destinationSN,
      uint32_t lifetime,
      uint32_t interface);
  void SendDelayedPrep(DelayedPrepStruct dps);
  void ReceivePrep (IePrep prep, Mac48Address from, uint32_t interface, Mac48Address fromMp, uint32_t metric);
  void ReceivePerr (std::vector<FailedDestination>, Mac48Address from, uint32_t interface, Mac48Address fromMp);
  void SendPrep (
    Mac48Address src,
    Mac48Address dst,
    Mac48Address retransmitter,
    uint32_t initMetric,
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
    uint32_t originatorDsn,
    uint32_t destinationSN,
    uint32_t lifetime,
    uint32_t interface);
  void InsertCbrCnnAtSourceIntoSourceCbrCnnsVector(
                        Mac48Address destination,
                        Mac48Address source,
                        uint8_t cnnType,
                    Ipv4Address srcIpv4Addr,
                    Ipv4Address dstIpv4Addr,
                    uint16_t srcPort,
                    uint16_t dstPort,
                    Mac48Address prevHop,
                    Mac48Address nextHop
                  );
  void SourceCbrRouteExtend(
                        Mac48Address destination,
                        Mac48Address source,
                        uint8_t cnnType,
                    Ipv4Address srcIpv4Addr,
                    Ipv4Address dstIpv4Addr,
                    uint16_t srcPort,
                    uint16_t dstPort
                  );
  void InsertCbrCnnIntoCbrCnnsVector(
                    Mac48Address destination,
                    Mac48Address source,
                    uint8_t cnnType,
                    Ipv4Address srcIpv4Addr,
                    Ipv4Address dstIpv4Addr,
                    uint16_t srcPort,
                    uint16_t dstPort,
                    Mac48Address prevHop,
                    Mac48Address nextHop
                  );
  void CbrRouteExtend(
                    Mac48Address destination,
                    Mac48Address source,
                    uint8_t cnnType,
                    Ipv4Address srcIpv4Addr,
                    Ipv4Address dstIpv4Addr,
                    uint16_t srcPort,
                    uint16_t dstPort
                  );
  void CbrRouteExpire(CbrConnection cbrCnn);
  void CheckCbrRoutes4Expiration();
  /**
   * \brief forms a path error information element when list of destination fails on a given interface
   * \attention removes all entries from routing table!
   */
  PathError MakePathError (std::vector<FailedDestination> destinations);
  ///\brief Forwards a received path error
  void ForwardPathError (PathError perr);
  ///\brief Passes a self-generated PERR to interface-plugin
  void InitiatePathError (PathError perr);
  /// \return list of addresses where a PERR should be sent to
  std::vector<std::pair<uint32_t, Mac48Address> > GetPerrReceivers (std::vector<FailedDestination> failedDest);

  /// \return list of addresses where a PERR should be sent to
  std::vector<Mac48Address> GetPreqReceivers (uint32_t interface);
  /// \return list of addresses where a broadcast should be
  //retransmitted
  std::vector<Mac48Address> GetBroadcastReceivers (uint32_t interface);
  /**
   * \brief MAC-plugin asks whether the frame can be dropped. Protocol automatically updates seqno.
   *
   * \return true if frame can be dropped
   * \param seqno is the sequence number of source
   * \param source is the source address
   */
  bool DropDataFrame (uint32_t seqno, Mac48Address source);
  //\}
  /// Route discovery time:
  TracedCallback<Time> m_routeDiscoveryTimeCallback;
  TracedCallback<> m_txed4mSourceCallback;
  TracedCallback<> m_wannaTx4mSourceCallback;
  TracedCallback<Ipv4Address,Ipv4Address,uint16_t,uint16_t,bool> m_CbrCnnStateChanged;
  TracedCallback<Ptr<Packet> > m_packetBufferredAtSource;
  ///\name Methods related to Queue/Dequeue procedures
  ///\{
  bool QueuePacket (QueuedPacket packet);
  QueuedPacket  DequeueFirstPacketByCnnParams (
                        Mac48Address dst,
                        Mac48Address src,
                        uint8_t cnnType,
                        Ipv4Address srcIpv4Addr,
                        Ipv4Address dstIpv4Addr,
                        uint16_t srcPort,
                        uint16_t dstPort
  );
  QueuedPacket  DequeueFirstPacketByDst (Mac48Address dst);
  QueuedPacket  DequeueFirstPacket ();
  void ReactivePathResolved (Mac48Address dst);
  void CnnBasedReactivePathResolved (
      Mac48Address dst,
      Mac48Address src,
      uint8_t cnnType,
      Ipv4Address srcIpv4Addr,
      Ipv4Address dstIpv4Addr,
      uint16_t srcPort,
      uint16_t dstPort
                    );
  void ProactivePathResolved ();
  ///\}
  ///\name Methods responsible for path discovery retry procedure:
  ///\{
  /**
   * \brief checks when the last path discovery procedure was started for a given destination.
   *
   * If the retry counter has not achieved the maximum level - preq should not be sent
   */
  bool  ShouldSendPreq (Mac48Address dst);
  bool  CnnBasedShouldSendPreq (
      RhoSigmaTag rsTag,
            Mac48Address dst,
            Mac48Address src,
            uint8_t cnnType,
            Ipv4Address srcIpv4Addr,
            Ipv4Address dstIpv4Addr,
            uint16_t srcPort,
            uint16_t dstPort
  );

  /**
   * \brief Generates PREQ retry when retry timeout has expired and route is still unresolved.
   *
   * When PREQ retry has achieved the maximum level - retry mechanism should be canceled
   */
  void  RetryPathDiscovery (Mac48Address dst, uint8_t numOfRetry);
  void  CnnBasedRetryPathDiscovery (
                CnnBasedPreqEvent preqEvent,
                uint8_t numOfRetry
  );
  /// Proactive Preq routines:
  void SendProactivePreq ();
  ///\}
  ///\return address of MeshPointDevice
  Mac48Address GetAddress ();
  ///\name Methods needed by HwmpMacLugin to access protocol parameters:
  ///\{
  bool GetDoFlag ();
  bool GetRfFlag ();
  Time GetPreqMinInterval ();
  Time GetPerrMinInterval ();
  uint8_t GetMaxTtl ();
  uint32_t GetNextPreqId ();
  uint32_t GetNextHwmpSeqno ();
  uint32_t GetActivePathLifetime ();
  uint8_t GetUnicastPerrThreshold ();

  void EnergyChange(Ptr<Packet> packet,bool isAck, bool incDec,double energy,double remainedEnergy,uint32_t packetSize);
  void GammaChange(double gamma,double totalSimmTime);

  ///\}
private:
  ///\name Statistics:
  ///\{
  struct Statistics
  {
    uint16_t txUnicast;
    uint16_t txBroadcast;
    uint32_t txBytes;
    uint16_t droppedTtl;
    uint16_t totalQueued;
    uint16_t totalDropped;
    uint16_t initiatedPreq;
    uint16_t initiatedPrep;
    uint16_t initiatedPerr;

    void Print (std::ostream & os) const;
    Statistics ();
  };
  Statistics m_stats;
  ///\}
  HwmpProtocolMacMap m_interfaces;
  Mac48Address m_address;
  uint32_t m_dataSeqno;
  uint32_t m_hwmpSeqno;
  uint32_t m_preqId;
  ///\name Sequence number filters
  ///\{
  /// Data sequence number database
  std::map<Mac48Address, uint32_t> m_lastDataSeqno;

  struct CnnBasedSeqnoMetricDatabase {
    Mac48Address originatorAddress;
    uint32_t originatorSeqNumber;
    Mac48Address destinationAddress;
    uint32_t destinationSeqNumber;
    uint32_t metric;
    uint8_t cnnType;
    uint32_t gammaPrim;
    uint32_t bPrim;
    uint32_t totalE;
    Ipv4Address srcIpv4Addr;
    Ipv4Address dstIpv4Addr;
    uint16_t srcPort;
    uint16_t dstPort;
  };
  std::vector<CnnBasedSeqnoMetricDatabase> m_hwmpSeqnoMetricDatabase;

  /// keeps HWMP seqno (first in pair) and HWMP metric (second in pair) for each address
//  std::map<Mac48Address, std::pair<uint32_t, uint32_t> > m_hwmpSeqnoMetricDatabase;
//  std::map<Mac48Address, std::pair<uint32_t, std::pair<uint32_t, uint32_t> > > m_hwmpSeqnoMetricDatabase4prep;
  ///\}

  /// Routing table
  Ptr<HwmpRtable> m_rtable;

  ///\name Timers:
  //\{
  struct PreqEvent {
    EventId preqTimeout;
    Time whenScheduled;
  };
  std::map<Mac48Address, PreqEvent> m_preqTimeouts;
  EventId m_proactivePreqTimer;
  /// Random start in Proactive PREQ propagation
  Time m_randomStart;
  ///\}
  /// Packet Queue
  std::vector<QueuedPacket> m_rqueue;
  ///\name HWMP-protocol parameters (attributes of GetTypeId)
  ///\{
  uint16_t m_maxQueueSize;
  uint8_t m_dot11MeshHWMPmaxPREQretries;
  Time m_dot11MeshHWMPnetDiameterTraversalTime;
  Time m_dot11MeshHWMPpreqMinInterval;
  Time m_dot11MeshHWMPperrMinInterval;
  Time m_dot11MeshHWMPactiveRootTimeout;
  Time m_dot11MeshHWMPactivePathTimeout;
  Time m_dot11MeshHWMPpathToRootInterval;
  Time m_dot11MeshHWMPrannInterval;
  bool m_isRoot;
  uint8_t m_maxTtl;
  uint8_t m_unicastPerrThreshold;
  uint8_t m_unicastPreqThreshold;
  uint8_t m_unicastDataThreshold;
  bool m_doFlag;
  bool m_rfFlag;
  ///\}
  /// Random variable for random start time
  Ptr<UniformRandomVariable> m_coefficient;
  Callback <std::vector<Mac48Address>, uint32_t> m_neighboursCallback;

  CbrConnectionsVector m_cbrConnections;
  CbrConnectionsVector m_sourceCbrConnections;
  CbrConnectionsVector m_notRoutedCbrConnections;

  uint32_t m_VBMetricMargin;
  uint16_t m_Gppm;

  bool m_noDataPacketYet;
  double m_energyPerByte;

  double m_totalSimulationTime;
};
} // namespace dot11s
} // namespace ns3
#endif
