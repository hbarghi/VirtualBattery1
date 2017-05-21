/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
//
// Copyright (c) 2006 Georgia Tech Research Corporation
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// Author: George F. Riley<riley@ece.gatech.edu>
//

// ns3 - Poisson Data Source Application class
// Hadi Barghi, Iran University of Science and Technology, Spring 2017

#ifndef POISSON_APPLICATION_H
#define POISSON_APPLICATION_H

#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/data-rate.h"
#include "ns3/traced-callback.h"
#include "ns3/random-variable-stream.h"
#include "ns3/random-variable.h"

namespace ns3 {

class Address;
class RandomVariableStream;
class Socket;

/**
 * \ingroup applications 
 * \defgroup poisson PoissonApplication
 *
 * This traffic generator follows an On/Off pattern: after 
 * Application::StartApplication
 * is called, ...
 */
/**
* \ingroup poisson
*
* \brief Generate traffic to a single destination according to an
*        poisson pattern.
*
* This traffic generator follows an Poisson pattern: after
* Application::StartApplication
* is called, ...
*
* If the underlying socket type supports broadcast, this application
* will automatically enable the SetAllowBroadcast(true) socket option.
*/
class PoissonApplication : public Application
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  PoissonApplication ();

  virtual ~PoissonApplication();

  /**
   * \brief Return a pointer to associated socket.
   * \return pointer to associated socket
   */
  Ptr<Socket> GetSocket (void) const;

 /**
  * \brief Assign a fixed random variable stream number to the random variables
  * used by this model.
  *
  * \param stream first stream index to use
  * \return the number of stream indices assigned by this model
  */
  int64_t AssignStreams (int64_t stream);

protected:
  virtual void DoDispose (void);
private:
  // inherited from Application base class.
  virtual void StartApplication (void);    // Called at time specified by Start
  virtual void StopApplication (void);     // Called at time specified by Stop

  //helpers
  /**
   * \brief Cancel all pending events.
   */
  void CancelEvents ();

  // Event handlers
  void SendPacket ();

  Ptr<Socket>     m_socket;       //!< Associated socket
  Address         m_peer;         //!< Peer address
  bool            m_connected;    //!< True if connected
  DataRate        m_averageRate;      //!< Rate that data is generated
  uint32_t        m_maxPktSize;      //!< Size of packets
  uint32_t        m_minPktSize;      //!< Size of packets
  uint32_t        m_pktSize;      //!< Size of packets
  uint16_t        m_sigmaPackets;      //!< sigma in packets
  Time            m_lastStartTime; //!< Time last packet sent
  EventId         m_sendEvent;    //!< Event id of pending "send packet" event
  TypeId          m_tid;          //!< Type of the socket used

  Ptr<UniformRandomVariable> m_packetSizeBytes;
  uint32_t m_remainedBytes;

  uint32_t m_sent; //!< Counter for sent packets
  Time m_realStopTime;

  ExponentialVariable m_nextTimeSecond;

  /// Traced Callback: transmitted packets.
  TracedCallback<Ptr<const Packet> > m_txTrace;

private:
  /**
   * \brief Handle a Connection Succeed event
   * \param socket the connected socket
   */
  void ConnectionSucceeded (Ptr<Socket> socket);
  /**
   * \brief Handle a Connection Failed event
   * \param socket the not connected socket
   */
  void ConnectionFailed (Ptr<Socket> socket);
};

} // namespace ns3

#endif /* Poisson_APPLICATION_H */
