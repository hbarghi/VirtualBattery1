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

#ifndef HWMP_STATE_H
#define HWMP_STATE_H

#include "ns3/mesh-wifi-interface-mac-plugin.h"
#include "ns3/hwmp-protocol.h"

namespace ns3 {

class MeshWifiInterfaceMac;
class WifiActionHeader;

namespace dot11s {

class HwmpProtocol;
class IePreq;
class IePrep;
class IePerr;

/**
 * \ingroup dot11s
 *
 * \brief Interface MAC plugin for HWMP -- 802.11s routing protocol
 */
class HwmpProtocolMac : public MeshWifiInterfaceMacPlugin
{
public:
  HwmpProtocolMac (uint32_t, Ptr<HwmpProtocol>);
  ~HwmpProtocolMac ();
  ///\name Inherited from MAC plugin
  //\{
  void SetParent (Ptr<MeshWifiInterfaceMac> parent);
  bool Receive (Ptr<Packet> packet, const WifiMacHeader & header);
  bool UpdateOutcomingFrame (Ptr<Packet> packet, WifiMacHeader & header, Mac48Address from, Mac48Address to);
  /// Update beacon is empty, because HWMP does not know anything about beacons
  void UpdateBeacon (MeshWifiBeacon & beacon) const {};
  int64_t AssignStreams (int64_t stream);
  double GetEres() const;
  double GetBatteryCapacity() const;
  double GetGamma() const;
  void SetEnergyChangeCallback(Callback<void, Ptr<Packet>, bool,bool,double,double,uint32_t> callback);
  void SetGammaChangeCallback(Callback<void, double, double> callback);
  //\}
  bool HasEnoughCapacity4NewConnection(Mac48Address from, Mac48Address to,int hopCount,Mac48Address prevHop,uint16_t rhoPpm) const;//hadi eo94
private:
  friend class HwmpProtocol;
  ///\returns a path selection action header
  static WifiActionHeader GetWifiActionHeader ();
  ///\name Interaction with HWMP:
  //\{
  void SendPreq (IePreq preq);
  void SendPreq (std::vector<IePreq> preq);
  void SendPrep (IePrep prep, Mac48Address receiver);
  //Forward a path error
  void ForwardPerr (std::vector<HwmpProtocol::FailedDestination> destinations, std::vector<Mac48Address> receivers);
  // initiate my own path error
  void InitiatePerr (std::vector<HwmpProtocol::FailedDestination> destinations, std::vector<Mac48Address> receivers);
  /** \brief Request a destination. If can not send preq immediately -
   * add a destination to existing PREQ generated by me and stored in
   * PREQ queue
   * \param dest is the destination to be resolved
   * \param originator_seqno is a sequence number that shall be preq originator sequenece number
   * \param dst_seqno is a sequence number taken from routing table
   */
  void RequestDestination (Mac48Address dst, uint32_t originator_seqno, uint32_t dst_seqno,
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
                           uint32_t gammaPrim,
                           uint32_t bPrim,
                           uint32_t totalE
                     );
  //\}

  /// Sends one PREQ when PreqMinInterval after last PREQ expires (if any PREQ exists in rhe queue)
  void SendMyPreq ();
  void SendMyPerr ();
  /// \return metric to HWMP protocol, needed only by metrics to add
  //peer as routing entry
  uint32_t GetLinkMetric (Mac48Address peerAddress) const;
  uint16_t GetChannelId () const;
  /// Report statistics
  void Report (std::ostream &) const;
  void ResetStats ();
private:
  Ptr<MeshWifiInterfaceMac> m_parent;
  uint32_t m_ifIndex;
  Ptr<HwmpProtocol> m_protocol;

  ///\name my PREQ and PREQ timer:
  //\{
  EventId m_preqTimer;
  std::vector<IePreq>  m_myPreq;
  //\}
  ///\name PERR timer and stored path error
  //\{
  EventId m_perrTimer;
  struct MyPerr {
    std::vector<HwmpProtocol::FailedDestination> destinations;
    std::vector<Mac48Address> receivers;
  };
  MyPerr m_myPerr;
  ///\name Statistics:
  //\{
  struct Statistics
  {
    uint16_t txPreq;
    uint16_t rxPreq;
    uint16_t txPrep;
    uint16_t rxPrep;
    uint16_t txPerr;
    uint16_t rxPerr;
    uint16_t txMgt;
    uint32_t txMgtBytes;
    uint16_t rxMgt;
    uint32_t rxMgtBytes;
    uint16_t txData;
    uint32_t txDataBytes;
    uint16_t rxData;
    uint32_t rxDataBytes;
    void Print (std::ostream & os) const;
    Statistics ();
  };
  Statistics m_stats;
  //\}
private:
  /// Receive data frame
  bool ReceiveData (Ptr<Packet> packet, const WifiMacHeader & header);
  /// Receive action management frame
  bool ReceiveAction (Ptr<Packet> packet, const WifiMacHeader & header);

  uint32_t m_lastPrepUid;
};
} // namespace dot11s
} // namespace ns3
#endif
