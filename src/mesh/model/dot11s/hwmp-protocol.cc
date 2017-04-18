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

#include "hwmp-protocol.h"
#include "hwmp-protocol-mac.h"
#include "hwmp-tag.h"
#include "hwmp-rtable.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/mesh-point-device.h"
#include "ns3/wifi-net-device.h"
#include "ns3/mesh-point-device.h"
#include "ns3/mesh-wifi-interface-mac.h"
#include "ns3/random-variable-stream.h"
#include "airtime-metric.h"
#include "ie-dot11s-preq.h"
#include "ie-dot11s-prep.h"
#include "ns3/trace-source-accessor.h"
#include "ie-dot11s-perr.h"
#include "ns3/arp-l3-protocol.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/udp-l4-protocol.h"
#include "ns3/tcp-l4-protocol.h"
#include "ns3/arp-header.h"
#include "ns3/ipv4-header.h"
#include "ns3/tcp-header.h"
#include "ns3/udp-header.h"

#include <gsl/gsl_integration.h>

#define PI 3.141592653589793

NS_LOG_COMPONENT_DEFINE ("HwmpProtocol");

namespace ns3 {
namespace dot11s {

NS_OBJECT_ENSURE_REGISTERED (HwmpProtocol)
  ;

struct f_params {
  double alpha;
  double beta;
  double x0;
};

double f(double x, void *p)
{
  f_params &params= *reinterpret_cast<f_params *>(p);
  return (params.x0/(std::sqrt(2*PI*params.alpha*std::pow(x,3))))*std::exp(-(std::pow(params.x0+params.beta*x,2)/(2*params.alpha*x)));
}

/* integration/qng.c
*
* Copyright (C) 1996, 1997, 1998, 1999, 2000, 2007 Brian Gough
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3 of the License, or (at
* your option) any later version.
*
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

//#include <config.h>
#include <math.h>
#include <float.h>

 double
 HwmpProtocol::rescale_error (double err, const double result_abs, const double result_asc)
 {
   err = fabs(err) ;

   if (result_asc != 0 && err != 0)
       {
         double scale = pow((200 * err / result_asc), 1.5) ;

         if (scale < 1)
           {
             err = result_asc * scale ;
           }
         else
           {
             err = result_asc ;
           }
       }
   if (result_abs > GSL_DBL_MIN / (50 * GSL_DBL_EPSILON))
     {
       double min_err = 50 * GSL_DBL_EPSILON * result_abs ;

       if (min_err > err)
         {
           err = min_err ;
         }
     }

   return err ;
 }

int
HwmpProtocol::mygsl_integration_qng (const gsl_function *f,
                     double a, double b,
                     double epsabs, double epsrel,
                     double * result, double * abserr, size_t * neval)
{
  double fv1[5], fv2[5], fv3[5], fv4[5];
  double savfun[21];  /* array of function values which have been computed */
  double res10, res21, res43, res87;    /* 10, 21, 43 and 87 point results */
  double result_kronrod, err ;
  double resabs; /* approximation to the integral of abs(f) */
  double resasc; /* approximation to the integral of abs(f-i/(b-a)) */

  const double half_length =  0.5 * (b - a);
  const double abs_half_length = fabs (half_length);
  const double center = 0.5 * (b + a);
  const double f_center = GSL_FN_EVAL(f, center);

  int k ;

  if (epsabs <= 0 && (epsrel < 50 * GSL_DBL_EPSILON || epsrel < 0.5e-28))
    {
      * result = 0;
      * abserr = 0;
      * neval = 0;
      GSL_ERROR ("tolerance cannot be achieved with given epsabs and epsrel",
                 GSL_EBADTOL);
    };

  /* Compute the integral using the 10- and 21-point formula. */

  res10 = 0;
  res21 = w21b[5] * f_center;
  resabs = w21b[5] * fabs (f_center);

  for (k = 0; k < 5; k++)
    {
      const double abscissa = half_length * x1[k];
      const double fval1 = GSL_FN_EVAL(f, center + abscissa);
      const double fval2 = GSL_FN_EVAL(f, center - abscissa);
      const double fval = fval1 + fval2;
      res10 += w10[k] * fval;
      res21 += w21a[k] * fval;
      resabs += w21a[k] * (fabs (fval1) + fabs (fval2));
      savfun[k] = fval;
      fv1[k] = fval1;
      fv2[k] = fval2;
    }

  for (k = 0; k < 5; k++)
    {
      const double abscissa = half_length * x2[k];
      const double fval1 = GSL_FN_EVAL(f, center + abscissa);
      const double fval2 = GSL_FN_EVAL(f, center - abscissa);
      const double fval = fval1 + fval2;
      res21 += w21b[k] * fval;
      resabs += w21b[k] * (fabs (fval1) + fabs (fval2));
      savfun[k + 5] = fval;
      fv3[k] = fval1;
      fv4[k] = fval2;
    }

  resabs *= abs_half_length ;

  {
    const double mean = 0.5 * res21;

    resasc = w21b[5] * fabs (f_center - mean);

    for (k = 0; k < 5; k++)
       {
         resasc +=
           (w21a[k] * (fabs (fv1[k] - mean) + fabs (fv2[k] - mean))
           + w21b[k] * (fabs (fv3[k] - mean) + fabs (fv4[k] - mean)));
       }
     resasc *= abs_half_length ;
   }

   result_kronrod = res21 * half_length;

   err = rescale_error ((res21 - res10) * half_length, resabs, resasc) ;

   /*   test for convergence. */

   if (err < epsabs || err < epsrel * fabs (result_kronrod))
     {
       * result = result_kronrod ;
       * abserr = err ;
       * neval = 21;
       return GSL_SUCCESS;
     }

   /* compute the integral using the 43-point formula. */

   res43 = w43b[11] * f_center;

   for (k = 0; k < 10; k++)
     {
       res43 += savfun[k] * w43a[k];
     }

   for (k = 0; k < 11; k++)
     {
       const double abscissa = half_length * x3[k];
       const double fval = (GSL_FN_EVAL(f, center + abscissa)
                            + GSL_FN_EVAL(f, center - abscissa));
       res43 += fval * w43b[k];
       savfun[k + 10] = fval;
     }

   /*  test for convergence */

   result_kronrod = res43 * half_length;
   err = rescale_error ((res43 - res21) * half_length, resabs, resasc);

   if (err < epsabs || err < epsrel * fabs (result_kronrod))
     {
       * result = result_kronrod ;
       * abserr = err ;
       * neval = 43;
       return GSL_SUCCESS;
     }

   /* compute the integral using the 87-point formula. */

   res87 = w87b[22] * f_center;

   for (k = 0; k < 21; k++)
     {
       res87 += savfun[k] * w87a[k];
     }

   for (k = 0; k < 22; k++)
     {
       const double abscissa = half_length * x4[k];
       res87 += w87b[k] * (GSL_FN_EVAL(f, center + abscissa)
                           + GSL_FN_EVAL(f, center - abscissa));
     }

   /*  test for convergence */

   result_kronrod = res87 * half_length ;

   err = rescale_error ((res87 - res43) * half_length, resabs, resasc);

   if (err < epsabs || err < epsrel * fabs (result_kronrod))
     {
       * result = result_kronrod ;
       * abserr = err ;
       * neval = 87;
       return GSL_SUCCESS;
     }

   /* failed to converge */

   * result = result_kronrod ;
   * abserr = err ;
   * neval = 87;

   //GSL_ERROR("failed to reach tolerance with highest-order rule", GSL_ETOL) ;
   NS_LOG_HADI("failed to reach tolerance with highest-order rule, res: " << result_kronrod << " " << err);
   return GSL_SUCCESS;
 }

class DepletionProbabilityDensityFunction
{
public:
	double operator()(double t,double alpha,double beta,double x0) const
	{
		//return exp(-x/5.0)*(2.0 + sin(2.0*x));
	  return (x0/(std::sqrt(2*PI*alpha*std::pow(t,3))))*std::exp(-(std::pow(x0+beta*t,2)/(2*alpha*t)));
	}
};


TypeId
HwmpProtocol::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::dot11s::HwmpProtocol")
    .SetParent<MeshL2RoutingProtocol> ()
    .AddConstructor<HwmpProtocol> ()
    .AddAttribute ( "RandomStart",
                    "Random delay at first proactive PREQ",
                    TimeValue (Seconds (0.1)),
                    MakeTimeAccessor (
                      &HwmpProtocol::m_randomStart),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "MaxQueueSize",
                    "Maximum number of packets we can store when resolving route",
                    UintegerValue (255),
                    MakeUintegerAccessor (
                      &HwmpProtocol::m_maxQueueSize),
                    MakeUintegerChecker<uint16_t> (1)
                    )
    .AddAttribute ( "Dot11MeshHWMPmaxPREQretries",
                    "Maximum number of retries before we suppose the destination to be unreachable",
                    UintegerValue (3),
                    MakeUintegerAccessor (
                      &HwmpProtocol::m_dot11MeshHWMPmaxPREQretries),
                    MakeUintegerChecker<uint8_t> (1)
                    )
    .AddAttribute ( "Dot11MeshHWMPnetDiameterTraversalTime",
                    "Time we suppose the packet to go from one edge of the network to another",
                    TimeValue (MicroSeconds (1024*100)),
                    MakeTimeAccessor (
                      &HwmpProtocol::m_dot11MeshHWMPnetDiameterTraversalTime),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "Dot11MeshHWMPpreqMinInterval",
                    "Minimal interval between to successive PREQs",
                    TimeValue (MicroSeconds (1024*100)),
                    MakeTimeAccessor (
                      &HwmpProtocol::m_dot11MeshHWMPpreqMinInterval),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "Dot11MeshHWMPperrMinInterval",
                    "Minimal interval between to successive PREQs",
                    TimeValue (MicroSeconds (1024*100)),
                    MakeTimeAccessor (&HwmpProtocol::m_dot11MeshHWMPperrMinInterval),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "Dot11MeshHWMPactiveRootTimeout",
                    "Lifetime of poractive routing information",
                    TimeValue (MicroSeconds (1024*5000)),
                    MakeTimeAccessor (
                      &HwmpProtocol::m_dot11MeshHWMPactiveRootTimeout),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "Dot11MeshHWMPactivePathTimeout",
                    "Lifetime of reactive routing information",
                    TimeValue (MicroSeconds (1024*5000)),
                    MakeTimeAccessor (
                      &HwmpProtocol::m_dot11MeshHWMPactivePathTimeout),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "Dot11MeshHWMPpathToRootInterval",
                    "Interval between two successive proactive PREQs",
                    TimeValue (MicroSeconds (1024*2000)),
                    MakeTimeAccessor (
                      &HwmpProtocol::m_dot11MeshHWMPpathToRootInterval),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "Dot11MeshHWMPrannInterval",
                    "Lifetime of poractive routing information",
                    TimeValue (MicroSeconds (1024*5000)),
                    MakeTimeAccessor (
                      &HwmpProtocol::m_dot11MeshHWMPrannInterval),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "MaxTtl",
                    "Initial value of Time To Live field",
                    UintegerValue (32),
                    MakeUintegerAccessor (
                      &HwmpProtocol::m_maxTtl),
                    MakeUintegerChecker<uint8_t> (2)
                    )
    .AddAttribute ( "UnicastPerrThreshold",
                    "Maximum number of PERR receivers, when we send a PERR as a chain of unicasts",
                    UintegerValue (32),
                    MakeUintegerAccessor (
                      &HwmpProtocol::m_unicastPerrThreshold),
                    MakeUintegerChecker<uint8_t> (1)
                    )
    .AddAttribute ( "UnicastPreqThreshold",
                    "Maximum number of PREQ receivers, when we send a PREQ as a chain of unicasts",
                    UintegerValue (1),
                    MakeUintegerAccessor (
                      &HwmpProtocol::m_unicastPreqThreshold),
                    MakeUintegerChecker<uint8_t> (1)
                    )
    .AddAttribute ( "UnicastDataThreshold",
                    "Maximum number ofbroadcast receivers, when we send a broadcast as a chain of unicasts",
                    UintegerValue (1),
                    MakeUintegerAccessor (
                      &HwmpProtocol::m_unicastDataThreshold),
                    MakeUintegerChecker<uint8_t> (1)
                    )
    .AddAttribute ( "DoFlag",
                    "Destination only HWMP flag",
                    BooleanValue (true),
                    MakeBooleanAccessor (
                      &HwmpProtocol::m_doFlag),
                    MakeBooleanChecker ()
                    )
    .AddAttribute ( "RfFlag",
                    "Reply and forward flag",
                    BooleanValue (false),
                    MakeBooleanAccessor (
                      &HwmpProtocol::m_rfFlag),
                    MakeBooleanChecker ()
                    )
    .AddTraceSource ( "RouteDiscoveryTime",
                      "The time of route discovery procedure",
                      MakeTraceSourceAccessor (
                        &HwmpProtocol::m_routeDiscoveryTimeCallback)
                      )
    //by hadi
    .AddTraceSource ( "TransmittingFromSource",
                                  "",
                      MakeTraceSourceAccessor (
                         &HwmpProtocol::m_txed4mSourceCallback)
                                  )
    .AddTraceSource ( "WannaTransmittingFromSource",
                          "",
                              MakeTraceSourceAccessor (
                                  &HwmpProtocol::m_wannaTx4mSourceCallback)
                              )
     .AddTraceSource( "CbrCnnStateChanged",
                          "",
                          MakeTraceSourceAccessor(
                                          &HwmpProtocol::m_CbrCnnStateChanged))
         .AddTraceSource( "PacketBufferredAtSource",
                                          "",
                                          MakeTraceSourceAccessor(
                                                          &HwmpProtocol::m_packetBufferredAtSource))
  ;
  return tid;
}

HwmpProtocol::HwmpProtocol () :
  m_dataSeqno (1),
  m_hwmpSeqno (1),
  m_preqId (0),
  m_rtable (CreateObject<HwmpRtable> ()),
  m_randomStart (Seconds (0.1)),
  m_maxQueueSize (255),
  m_dot11MeshHWMPmaxPREQretries (3),
  m_dot11MeshHWMPnetDiameterTraversalTime (MicroSeconds (1024*100)),
  m_dot11MeshHWMPpreqMinInterval (MicroSeconds (1024*100)),
  m_dot11MeshHWMPperrMinInterval (MicroSeconds (1024*100)),
  m_dot11MeshHWMPactiveRootTimeout (MicroSeconds (1024*5000)),
  m_dot11MeshHWMPactivePathTimeout (MicroSeconds (1024*5000)),
  m_dot11MeshHWMPpathToRootInterval (MicroSeconds (1024*2000)),
  m_dot11MeshHWMPrannInterval (MicroSeconds (1024*5000)),
  m_isRoot (false),
  m_maxTtl (32),
  m_unicastPerrThreshold (32),
  m_unicastPreqThreshold (1),
  m_unicastDataThreshold (1),
  m_doFlag (true),
  m_rfFlag (false)
{
  NS_LOG_FUNCTION_NOARGS ();
  m_coefficient = CreateObject<UniformRandomVariable> ();
}

HwmpProtocol::~HwmpProtocol ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
HwmpProtocol::DoInitialize ()
{
  m_coefficient->SetAttribute ("Max", DoubleValue (m_randomStart.GetSeconds ()));
  if (m_isRoot)
    {
      SetRoot ();
    }
  Simulator::Schedule(Seconds(0.5),&HwmpProtocol::CheckCbrRoutes4Expiration,this);//hadi eo94
}

void
HwmpProtocol::DoDispose ()
{
  NS_LOG_FUNCTION_NOARGS ();
  for (std::map<Mac48Address, PreqEvent>::iterator i = m_preqTimeouts.begin (); i != m_preqTimeouts.end (); i++)
    {
      i->second.preqTimeout.Cancel ();
    }
  m_proactivePreqTimer.Cancel ();
  m_preqTimeouts.clear ();
  m_lastDataSeqno.clear ();
  m_hwmpSeqnoMetricDatabase.clear ();
  for (std::vector<CnnBasedPreqEvent>::iterator cbpei = m_cnnBasedPreqTimeouts.begin (); cbpei != m_cnnBasedPreqTimeouts.end (); cbpei++)
  {
      cbpei->preqTimeout.Cancel();
  }
  m_cnnBasedPreqTimeouts.clear();
  m_interfaces.clear ();
  m_rqueue.clear ();
  m_rtable = 0;
  m_mp = 0;
}

bool
HwmpProtocol::RequestRoute (
  uint32_t sourceIface,
  const Mac48Address source,
  const Mac48Address destination,
  Ptr<const Packet> constPacket,
  uint16_t protocolType, //ethrnet 'Protocol' field
  MeshL2RoutingProtocol::RouteReplyCallback routeReply
  )
{
  Ptr <Packet> packet = constPacket->Copy ();
  HwmpTag tag;
  if (sourceIface == GetMeshPoint ()->GetIfIndex ())
    {
      // packet from level 3
      if (packet->PeekPacketTag (tag))
        {
          NS_FATAL_ERROR ("HWMP tag has come with a packet from upper layer. This must not occur...");
        }
      //Filling TAG:
      if (destination == Mac48Address::GetBroadcast ())
        {
          tag.SetSeqno (m_dataSeqno++);
        }
      tag.SetTtl (m_maxTtl);
    }
  else
    {
      if (!packet->RemovePacketTag (tag))
        {
          NS_FATAL_ERROR ("HWMP tag is supposed to be here at this point.");
        }
      tag.DecrementTtl ();
      if (tag.GetTtl () == 0)
        {
          m_stats.droppedTtl++;
          return false;
        }
    }
  if (destination == Mac48Address::GetBroadcast ())
    {
      m_stats.txBroadcast++;
      m_stats.txBytes += packet->GetSize ();
      //channel IDs where we have already sent broadcast:
      std::vector<uint16_t> channels;
      for (HwmpProtocolMacMap::const_iterator plugin = m_interfaces.begin (); plugin != m_interfaces.end (); plugin++)
        {
          bool shouldSend = true;
          for (std::vector<uint16_t>::const_iterator chan = channels.begin (); chan != channels.end (); chan++)
            {
              if ((*chan) == plugin->second->GetChannelId ())
                {
                  shouldSend = false;
                }
            }
          if (!shouldSend)
            {
              continue;
            }
          channels.push_back (plugin->second->GetChannelId ());
          std::vector<Mac48Address> receivers = GetBroadcastReceivers (plugin->first);
          for (std::vector<Mac48Address>::const_iterator i = receivers.begin (); i != receivers.end (); i++)
            {
              Ptr<Packet> packetCopy = packet->Copy ();
              //
              // 64-bit Intel valgrind complains about tag.SetAddress (*i).  It
              // likes this just fine.
              //
              Mac48Address address = *i;
              tag.SetAddress (address);
              packetCopy->AddPacketTag (tag);
              routeReply (true, packetCopy, source, destination, protocolType, plugin->first);
            }
        }
    }
  else
    {
      return ForwardUnicast (sourceIface, source, destination, packet, protocolType, routeReply, tag.GetTtl ());
    }
  return true;
}
bool
HwmpProtocol::RemoveRoutingStuff (uint32_t fromIface, const Mac48Address source,
                                  const Mac48Address destination, Ptr<Packet>  packet, uint16_t&  protocolType)
{
  HwmpTag tag;
  if (!packet->RemovePacketTag (tag))
    {
      NS_FATAL_ERROR ("HWMP tag must exist when packet received from the network");
    }
  return true;
}
bool
HwmpProtocol::ForwardUnicast (uint32_t  sourceIface, const Mac48Address source, const Mac48Address destination,
                              Ptr<Packet>  packet, uint16_t  protocolType, RouteReplyCallback  routeReply, uint32_t ttl)
{
  Ptr<Packet> pCopy=packet->Copy();
  uint8_t cnnType;//1:mac only, 2:ip only , 3:ip port
  Ipv4Address srcIpv4Addr;
  Ipv4Address dstIpv4Addr;
  uint16_t srcPort;
  uint16_t dstPort;
  if(protocolType==ArpL3Protocol::PROT_NUMBER)
  {
      ArpHeader arpHdr;
      pCopy->RemoveHeader(arpHdr);
      srcIpv4Addr = arpHdr.GetSourceIpv4Address();
      dstIpv4Addr = arpHdr.GetDestinationIpv4Address();
      cnnType=HwmpRtable::CNN_TYPE_IP_ONLY;
//      NS_LOG_HADI(m_address << " ARP packet have seen");
      NS_ASSERT(true);
  }
  else if(protocolType==Ipv4L3Protocol::PROT_NUMBER)
  {
      Ipv4Header ipv4Hdr;
      pCopy->RemoveHeader(ipv4Hdr);
      srcIpv4Addr = ipv4Hdr.GetSource();
      dstIpv4Addr = ipv4Hdr.GetDestination();
      uint8_t protocol = ipv4Hdr.GetProtocol();
      if(protocol==TcpL4Protocol::PROT_NUMBER)
      {
          TcpHeader tcpHdr;
          pCopy->RemoveHeader (tcpHdr);
          srcPort=tcpHdr.GetSourcePort ();
          dstPort=tcpHdr.GetDestinationPort ();
          cnnType=HwmpRtable::CNN_TYPE_PKT_BASED;
      }
      else if(protocol==UdpL4Protocol::PROT_NUMBER)
      {
          UdpHeader udpHdr;
          pCopy->RemoveHeader(udpHdr);
          srcPort=udpHdr.GetSourcePort();
          dstPort=udpHdr.GetDestinationPort();
          cnnType=HwmpRtable::CNN_TYPE_IP_PORT;
//          NS_LOG_HADI(m_address << " UDP packet have seen " << source << "->" << destination << " " << srcIpv4Addr << ":" << (int)srcPort << "=>" << dstIpv4Addr << ":" << (int)dstPort);
      }
      else
      {
          cnnType=HwmpRtable::CNN_TYPE_IP_ONLY;
//          NS_LOG_HADI(m_address << " non TCP or UDP packet have seen");
          NS_ASSERT(true);
      }
  }
  else
  {
      cnnType=HwmpRtable::CNN_TYPE_MAC_ONLY;
//      NS_LOG_HADI(m_address << " non IP packet have seen");
      NS_ASSERT(true);
  }
  if((source==GetAddress())&&(cnnType==HwmpRtable::CNN_TYPE_IP_PORT)){
          NS_LOG_HADI(" hwmp forwardUnicast4mSource " << (int)packet->GetUid() << " " << srcIpv4Addr << ":" << (int)srcPort << "=>" << dstIpv4Addr << ":" << (int)dstPort);
          m_wannaTx4mSourceCallback();
}
  NS_ASSERT (destination != Mac48Address::GetBroadcast ());
  CbrConnection connection;
  connection.destination=destination;
  connection.source=source;
  connection.cnnType=cnnType;
  connection.dstIpv4Addr=dstIpv4Addr;
  connection.srcIpv4Addr=srcIpv4Addr;
  connection.dstPort=dstPort;
  connection.srcPort=srcPort;
  if(cnnType==HwmpRtable::CNN_TYPE_IP_PORT){
      CbrConnectionsVector::iterator nrccvi=std::find(m_notRoutedCbrConnections.begin(),m_notRoutedCbrConnections.end(),connection);
      if(nrccvi!=m_notRoutedCbrConnections.end()){
          if(source==GetAddress()){
                  NS_LOG_HADI(" hwmp cnnRejectedDrop " << (int)packet->GetUid() << " " << srcIpv4Addr << ":" << (int)srcPort << "=>" << dstIpv4Addr << ":" << (int)dstPort);
          }
          return false;
      }
    }
  HwmpRtable::CnnBasedLookupResult cnnBasedResult = m_rtable->LookupCnnBasedReactive(destination,source,cnnType,srcIpv4Addr,dstIpv4Addr,srcPort,dstPort);
  NS_LOG_DEBUG ("Requested src = "<<source<<", dst = "<<destination<<", I am "<<GetAddress ()<<", RA = "<<cnnBasedResult.retransmitter);
  HwmpTag tag;
  tag.SetAddress (cnnBasedResult.retransmitter);
  tag.SetTtl (ttl);
  //seqno and metric is not used;
  packet->AddPacketTag (tag);
  if (cnnBasedResult.retransmitter != Mac48Address::GetBroadcast ())
    {
          if((source==GetAddress())&&(cnnType==HwmpRtable::CNN_TYPE_IP_PORT)){
		  NS_LOG_HADI(m_address << " tx4mSource " << (int)packet->GetUid());
		  m_txed4mSourceCallback();
	  }
	  CbrRouteExtend(destination,source,cnnType,srcIpv4Addr,dstIpv4Addr,srcPort,dstPort);
      //reply immediately:
      routeReply (true, packet, source, destination, protocolType, cnnBasedResult.ifIndex);
      m_stats.txUnicast++;
      m_stats.txBytes += packet->GetSize ();
      return true;
    }
  if (sourceIface != GetMeshPoint ()->GetIfIndex ())
    {
      //Start path error procedure:
      NS_LOG_DEBUG ("Must Send PERR");
      m_stats.totalDropped++;
      return false;
    }
  if(source==GetAddress ())
    {
      NS_LOG_HADI("ReceivePreqCAC");
      //std::cout << integral << "\n";
      if(false)
        {
          NS_LOG_HADI("cac rejected at source the connection");
          m_notRoutedCbrConnections.push_back (connection);
          return false;
        }

    }
  //Request a destination:
  if (CnnBasedShouldSendPreq (destination, source, cnnType, srcIpv4Addr, dstIpv4Addr, srcPort, dstPort))
    {
	  NS_LOG_HADI(m_address << " sendingPathRequest " << source << " " << destination);
      uint32_t originator_seqno = GetNextHwmpSeqno ();
      uint32_t dst_seqno = 0;
      m_stats.initiatedPreq++;
      for (HwmpProtocolMacMap::const_iterator i = m_interfaces.begin (); i != m_interfaces.end (); i++)
        {
          i->second->RequestDestination (destination, originator_seqno, dst_seqno, cnnType, srcIpv4Addr, dstIpv4Addr, srcPort, dstPort,0,0,Seconds (0));
        }
    }
  QueuedPacket pkt;
  pkt.pkt = packet;
  pkt.dst = destination;
  pkt.src = source;
  pkt.protocol = protocolType;
  pkt.reply = routeReply;
  pkt.inInterface = sourceIface;
  pkt.cnnType=cnnType;
  pkt.srcIpv4Addr=srcIpv4Addr;
  pkt.dstIpv4Addr=dstIpv4Addr;
  pkt.srcPort=srcPort;
  pkt.dstPort=dstPort;
  if (QueuePacket (pkt))
    {
      if((source==GetAddress ())&&(cnnType==HwmpRtable::CNN_TYPE_IP_PORT))
        m_packetBufferredAtSource(packet);
      m_stats.totalQueued++;
      return true;
    }
  else
    {
      m_stats.totalDropped++;
      return false;
    }
}
void
HwmpProtocol::ReceivePreq (IePreq preq, Mac48Address from, uint32_t interface, Mac48Address fromMp, uint32_t metric)
{  
  preq.IncrementMetric (metric);
  //acceptance cretirea:
  bool duplicatePreq=false;
  //bool freshInfo (true);
  for(std::vector<CnnBasedSeqnoMetricDatabase>::iterator i=m_hwmpSeqnoMetricDatabase.begin();i!=m_hwmpSeqnoMetricDatabase.end();i++)
  {
      if(
              (i->originatorAddress==preq.GetOriginatorAddress())       &&
              (i->cnnType==preq.GetCnnType())                           &&
              (i->srcIpv4Addr==preq.GetSrcIpv4Addr())                   &&
              (i->srcPort==preq.GetSrcPort())                           &&
              (i->dstIpv4Addr==preq.GetDstIpv4Addr())                   &&
              (i->dstPort==preq.GetDstPort())
        )
      {
          duplicatePreq=true;
          if ((int32_t)(i->originatorSeqNumber - preq.GetOriginatorSeqNumber ())  > 0)
            {
              return;
            }
          if (i->originatorSeqNumber == preq.GetOriginatorSeqNumber ())
            {
              //freshInfo = false;
              if (i->metric <= preq.GetMetric ())
                {
                  return;
                }
            }
          m_hwmpSeqnoMetricDatabase.erase (i);
          break;
        }
    }
  if(!duplicatePreq)
    {
      NS_LOG_HADI("ReceivePreqCAC");
      //std::cout << integral << "\n";
      if(false)
        {
          NS_LOG_HADI("cac rejected the connection");
          return;
        }
    }
  CnnBasedSeqnoMetricDatabase newDb;
  newDb.originatorAddress=preq.GetOriginatorAddress();
  newDb.originatorSeqNumber=preq.GetOriginatorSeqNumber();
  newDb.metric=preq.GetMetric();
  newDb.cnnType=preq.GetCnnType();
  newDb.srcIpv4Addr=preq.GetSrcIpv4Addr();
  newDb.dstIpv4Addr=preq.GetDstIpv4Addr();
  newDb.srcPort=preq.GetSrcPort();
  newDb.dstPort=preq.GetDstPort();
  m_hwmpSeqnoMetricDatabase.push_back(newDb);
  NS_LOG_DEBUG ("I am " << GetAddress () << "Accepted preq from address" << from << ", preq:" << preq);
  std::vector<Ptr<DestinationAddressUnit> > destinations = preq.GetDestinationList ();
  //Add reverse path to originator:
  m_rtable->AddCnnBasedReversePath (preq.GetOriginatorAddress(),from,interface,0,0,preq.GetCnnType(),preq.GetSrcIpv4Addr(),preq.GetDstIpv4Addr(),preq.GetSrcPort(),preq.GetDstPort(),Seconds(1),preq.GetOriginatorSeqNumber());
  //Add reactive path to originator:
/*  if (
    (freshInfo) ||
    (
      (m_rtable->LookupReactive (preq.GetOriginatorAddress ()).retransmitter == Mac48Address::GetBroadcast ()) ||
      (m_rtable->LookupReactive (preq.GetOriginatorAddress ()).metric > preq.GetMetric ())
    )
    )
    {
      m_rtable->AddReactivePath (
        preq.GetOriginatorAddress (),
        from,
        interface,
        preq.GetMetric (),
        MicroSeconds (preq.GetLifetime () * 1024),
        preq.GetOriginatorSeqNumber ()
        );
      ReactivePathResolved (preq.GetOriginatorAddress ());
    }
  if (
    (m_rtable->LookupReactive (fromMp).retransmitter == Mac48Address::GetBroadcast ()) ||
    (m_rtable->LookupReactive (fromMp).metric > metric)
    )
    {
      m_rtable->AddReactivePath (
        fromMp,
        from,
        interface,
        metric,
        MicroSeconds (preq.GetLifetime () * 1024),
        preq.GetOriginatorSeqNumber ()
        );
      ReactivePathResolved (fromMp);
    }*/
  for (std::vector<Ptr<DestinationAddressUnit> >::const_iterator i = destinations.begin (); i != destinations.end (); i++)
    {
	  NS_LOG_HADI(m_address << " receivePReq " << preq.GetOriginatorAddress() << " " << from << " " << (*i)->GetDestinationAddress ());
      if ((*i)->GetDestinationAddress () == Mac48Address::GetBroadcast ())
        {
          //only proactive PREQ contains destination
          //address as broadcast! Proactive preq MUST
          //have destination count equal to 1 and
          //per destination flags DO and RF
          NS_ASSERT (preq.GetDestCount () == 1);
          NS_ASSERT (((*i)->IsDo ()) && ((*i)->IsRf ()));
          //Add proactive path only if it is the better then existed
          //before
/*          if (
            ((m_rtable->LookupProactive ()).retransmitter == Mac48Address::GetBroadcast ()) ||
            ((m_rtable->LookupProactive ()).metric > preq.GetMetric ())
            )
            {
              m_rtable->AddProactivePath (
                preq.GetMetric (),
                preq.GetOriginatorAddress (),
                from,
                interface,
                MicroSeconds (preq.GetLifetime () * 1024),
                preq.GetOriginatorSeqNumber ()
                );
              ProactivePathResolved ();
            }*/
          if (!preq.IsNeedNotPrep ())
            {
              SendPrep (
                GetAddress (),
                preq.GetOriginatorAddress (),
                from,
                preq.GetMetric(),
                preq.GetCnnType(),
                preq.GetSrcIpv4Addr(),
                preq.GetDstIpv4Addr(),
                preq.GetSrcPort(),
                preq.GetDstPort(),
                preq.GetOriginatorSeqNumber (),
                GetNextHwmpSeqno (),
                preq.GetLifetime (),
                interface
                );
            }
          break;
        }
      if ((*i)->GetDestinationAddress () == GetAddress ())
        {
          SendPrep (
            GetAddress (),
            preq.GetOriginatorAddress (),
            from,
            preq.GetMetric(),
            preq.GetCnnType(),
            preq.GetSrcIpv4Addr(),
            preq.GetDstIpv4Addr(),
            preq.GetSrcPort(),
            preq.GetDstPort(),
            preq.GetOriginatorSeqNumber (),
            GetNextHwmpSeqno (),
            preq.GetLifetime (),
            interface
            );
          //NS_ASSERT (m_rtable->LookupReactive (preq.GetOriginatorAddress ()).retransmitter != Mac48Address::GetBroadcast ());
          preq.DelDestinationAddressElement ((*i)->GetDestinationAddress ());
          continue;
        }
      //check if can answer:
      HwmpRtable::LookupResult result = m_rtable->LookupReactive ((*i)->GetDestinationAddress ());
      if ((!((*i)->IsDo ())) && (result.retransmitter != Mac48Address::GetBroadcast ()))
        {
          //have a valid information and can answer
          uint32_t lifetime = result.lifetime.GetMicroSeconds () / 1024;
          if ((lifetime > 0) && ((int32_t)(result.seqnum - (*i)->GetDestSeqNumber ()) >= 0))
            {
              SendPrep (
                (*i)->GetDestinationAddress (),
                preq.GetOriginatorAddress (),
                from,
                preq.GetMetric(),
                preq.GetCnnType(),
                preq.GetSrcIpv4Addr(),
                preq.GetDstIpv4Addr(),
                preq.GetSrcPort(),
                preq.GetDstPort(),
                preq.GetOriginatorSeqNumber (),
                result.seqnum,
                lifetime,
                interface
                );
              m_rtable->AddPrecursor ((*i)->GetDestinationAddress (), interface, from,
                                      MicroSeconds (preq.GetLifetime () * 1024));
              if ((*i)->IsRf ())
                {
                  (*i)->SetFlags (true, false, (*i)->IsUsn ()); //DO = 1, RF = 0
                }
              else
                {
                  preq.DelDestinationAddressElement ((*i)->GetDestinationAddress ());
                  continue;
                }
            }
        }
    }
  //check if must retransmit:
  if (preq.GetDestCount () == 0)
    {
      return;
    }
  //Forward PREQ to all interfaces:
  NS_LOG_DEBUG ("I am " << GetAddress () << "retransmitting PREQ:" << preq);
  for (HwmpProtocolMacMap::const_iterator i = m_interfaces.begin (); i != m_interfaces.end (); i++)
    {
      i->second->SendPreq (preq);
    }
}
void
HwmpProtocol::ReceivePrep (IePrep prep, Mac48Address from, uint32_t interface, Mac48Address fromMp, uint32_t metric)
{
  if(prep.GetDestinationAddress () == GetAddress ()){
    CbrConnection connection;
    connection.cnnType=prep.GetCnnType ();
    connection.dstIpv4Addr=prep.GetDstIpv4Addr ();
    connection.srcIpv4Addr=prep.GetSrcIpv4Addr ();
    connection.dstPort=prep.GetDstPort ();
    connection.srcPort=prep.GetSrcPort ();
    CbrConnectionsVector::iterator nrccvi=std::find(m_notRoutedCbrConnections.begin(),m_notRoutedCbrConnections.end(),connection);
    if(nrccvi!=m_notRoutedCbrConnections.end()){
        NS_LOG_HADI("sourceCnnHasDropped " << prep.GetSrcIpv4Addr() << ":" << (int)prep.GetSrcPort() << "=>" << prep.GetDstIpv4Addr() << ":" << (int)prep.GetDstPort() << " " << from);
        return;
      }
    }
  prep.IncrementMetric (metric);
  //acceptance cretirea:
  bool freshInfo (true);
  std::vector<CnnBasedSeqnoMetricDatabase>::iterator dbit;
  for(std::vector<CnnBasedSeqnoMetricDatabase>::iterator i=m_hwmpSeqnoMetricDatabase.begin();i!=m_hwmpSeqnoMetricDatabase.end();i++)
  {
      if(
              (i->originatorAddress==prep.GetOriginatorAddress())       &&
              (i->cnnType==prep.GetCnnType())                           &&
              (i->srcIpv4Addr==prep.GetSrcIpv4Addr())                   &&
              (i->srcPort==prep.GetSrcPort())                           &&
              (i->dstIpv4Addr==prep.GetDstIpv4Addr())                   &&
              (i->dstPort==prep.GetDstPort())
        )
      {
        if ((int32_t)(i->destinationSeqNumber - prep.GetDestinationSeqNumber())  > 0)
          {
            /*BarghiTest 1392/08/02 add for get result start*/
            //commented for hadireports std::cout << "t:" << Simulator::Now() << " ,Im " << m_address << " returning because of older preq" << std::endl;
            /*BarghiTest 1392/08/02 add for get result end*/
                NS_LOG_HADI(" hwmp droppedCPREP     seqnum       " << prep.GetSrcIpv4Addr() << ":" << (int)prep.GetSrcPort() << "=>" << prep.GetDstIpv4Addr() << ":" << (int)prep.GetDstPort() << " " << from);
            return;
          }
        dbit=i;
        freshInfo=false;
        break;
        }
    }
  if(freshInfo)
    {
      CnnBasedSeqnoMetricDatabase newDb;
      newDb.originatorAddress=prep.GetOriginatorAddress();
      newDb.originatorSeqNumber=prep.GetOriginatorSeqNumber();
      newDb.destinationAddress=prep.GetDestinationAddress();
      newDb.destinationSeqNumber=prep.GetDestinationSeqNumber();
      newDb.metric=prep.GetMetric();
      newDb.cnnType=prep.GetCnnType();
      newDb.srcIpv4Addr=prep.GetSrcIpv4Addr();
      newDb.dstIpv4Addr=prep.GetDstIpv4Addr();
      newDb.srcPort=prep.GetSrcPort();
      newDb.dstPort=prep.GetDstPort();
      m_hwmpSeqnoMetricDatabase.push_back(newDb);
      if (prep.GetDestinationAddress () == GetAddress ())
        {
          m_rtable->AddCnnBasedReactivePath (
                prep.GetOriginatorAddress (),
                from,
                GetAddress (),
                GetAddress (),
                interface,
                prep.GetMetric (),
                prep.GetCnnType (),
                prep.GetSrcIpv4Addr (),
                prep.GetDstIpv4Addr (),
                prep.GetSrcPort (),
                prep.GetDstPort (),
                Seconds (10000),
                prep.GetOriginatorSeqNumber ());
          m_rtable->AddPrecursor (prep.GetDestinationAddress (), interface, from,
                                  MicroSeconds (prep.GetLifetime () * 1024));
          /*if (result.retransmitter != Mac48Address::GetBroadcast ())
            {
              m_rtable->AddPrecursor (prep.GetOriginatorAddress (), interface, result.retransmitter,
                                      result.lifetime);
            }*/
          //ReactivePathResolved (prep.GetOriginatorAddress ());
          CnnBasedReactivePathResolved(prep.GetOriginatorAddress (),GetAddress (),prep.GetCnnType (),prep.GetSrcIpv4Addr (),prep.GetDstIpv4Addr (),prep.GetSrcPort (),prep.GetDstPort ());
          m_CbrCnnStateChanged(prep.GetSrcIpv4Addr(),prep.GetDstIpv4Addr(),prep.GetSrcPort(),prep.GetDstPort(),true);
          NS_LOG_DEBUG ("I am "<<GetAddress ()<<", resolved "<<prep.GetOriginatorAddress ());
          return;
        }
    }else
    {
          m_hwmpSeqnoMetricDatabase.erase (dbit);
          CnnBasedSeqnoMetricDatabase newDb;
          newDb.originatorAddress=prep.GetOriginatorAddress();
          newDb.originatorSeqNumber=prep.GetOriginatorSeqNumber();
          newDb.destinationAddress=prep.GetDestinationAddress();
          newDb.destinationSeqNumber=prep.GetDestinationSeqNumber();
          newDb.metric=prep.GetMetric();
          newDb.cnnType=prep.GetCnnType();
          newDb.srcIpv4Addr=prep.GetSrcIpv4Addr();
          newDb.dstIpv4Addr=prep.GetDstIpv4Addr();
          newDb.srcPort=prep.GetSrcPort();
          newDb.dstPort=prep.GetDstPort();
          m_hwmpSeqnoMetricDatabase.push_back(newDb);
          if (prep.GetDestinationAddress () == GetAddress ())
            {
              m_rtable->AddCnnBasedReactivePath (
                    prep.GetOriginatorAddress (),
                    from,
                    GetAddress (),
                    GetAddress (),
                    interface,
                    prep.GetMetric (),                    
                    prep.GetCnnType (),
                    prep.GetSrcIpv4Addr (),
                    prep.GetDstIpv4Addr (),
                    prep.GetSrcPort (),
                    prep.GetDstPort (),
                    Seconds (10000),
                    prep.GetOriginatorSeqNumber ());
              m_rtable->AddPrecursor (prep.GetDestinationAddress (), interface, from,
                                      MicroSeconds (prep.GetLifetime () * 1024));
              /*if (result.retransmitter != Mac48Address::GetBroadcast ())
                {
                  m_rtable->AddPrecursor (prep.GetOriginatorAddress (), interface, result.retransmitter,
                                          result.lifetime);
                }*/
              //ReactivePathResolved (prep.GetOriginatorAddress ());
              CnnBasedReactivePathResolved(prep.GetOriginatorAddress (),GetAddress (),prep.GetCnnType (),prep.GetSrcIpv4Addr (),prep.GetDstIpv4Addr (),prep.GetSrcPort (),prep.GetDstPort ());
              m_CbrCnnStateChanged(prep.GetSrcIpv4Addr(),prep.GetDstIpv4Addr(),prep.GetSrcPort(),prep.GetDstPort(),true);
              NS_LOG_DEBUG ("I am "<<GetAddress ()<<", resolved "<<prep.GetOriginatorAddress ());
              return;
            }

    }



  //update routing info
  //Now add a path to destination and add precursor to source
  NS_LOG_DEBUG ("I am " << GetAddress () << ", received prep from " << prep.GetOriginatorAddress () << ", receiver was:" << from);
  HwmpRtable::CnnBasedLookupResult result=m_rtable->LookupCnnBasedReverse(prep.GetDestinationAddress(),prep.GetCnnType(),prep.GetSrcIpv4Addr(),prep.GetDstIpv4Addr(),prep.GetSrcPort(),prep.GetDstPort());
  if (result.retransmitter == Mac48Address::GetBroadcast ())
    {
      return;
    }
  m_rtable->AddCnnBasedReactivePath (                    prep.GetOriginatorAddress (),
                                                         from,
                                                         prep.GetDestinationAddress (),
                                                         result.retransmitter,
                                                         interface,
                                                         prep.GetMetric (),
                                                         prep.GetCnnType (),
                                                         prep.GetSrcIpv4Addr (),
                                                         prep.GetDstIpv4Addr (),
                                                         prep.GetSrcPort (),
                                                         prep.GetDstPort (),
                                                         Seconds (10000),
                                                         prep.GetOriginatorSeqNumber ());
  InsertCbrCnnIntoCbrCnnsVector(prep.GetOriginatorAddress(),prep.GetDestinationAddress(),prep.GetCnnType(),prep.GetSrcIpv4Addr(),prep.GetDstIpv4Addr(),prep.GetSrcPort(),prep.GetDstPort(),result.retransmitter,from);
  //Forward PREP
  HwmpProtocolMacMap::const_iterator prep_sender = m_interfaces.find (result.ifIndex);
  NS_ASSERT (prep_sender != m_interfaces.end ());
  prep_sender->second->SendPrep (prep, result.retransmitter);
}


void
HwmpProtocol::InsertCbrCnnAtSourceIntoSourceCbrCnnsVector(
			Mac48Address destination,
			Mac48Address source,
			uint8_t cnnType,
		    Ipv4Address srcIpv4Addr,
		    Ipv4Address dstIpv4Addr,
		    uint16_t srcPort,
		    uint16_t dstPort,
		    Mac48Address prevHop,
		    Mac48Address nextHop
		  ){
	NS_LOG_HADI(" hwmp inserting cnn into cnnsvector at source " << srcIpv4Addr << ":" << (int)srcPort << "=>" << dstIpv4Addr << ":" << (int)dstPort << " n " << nextHop << " p " << prevHop);
    CbrConnection connection;
    connection.destination=destination;
    connection.source=source;
    connection.cnnType=cnnType;
    connection.dstIpv4Addr=dstIpv4Addr;
    connection.srcIpv4Addr=srcIpv4Addr;
    connection.dstPort=dstPort;
    connection.srcPort=srcPort;
    connection.prevMac=prevHop;
    connection.nextMac=nextHop;
    connection.whenExpires=Simulator::Now()+MilliSeconds(SOURCE_CBR_ROUTE_EXPIRE_MILLISECONDS);
    CbrConnectionsVector::iterator ccvi=std::find(m_sourceCbrConnections.begin(),m_sourceCbrConnections.end(),connection);
    if(ccvi==m_sourceCbrConnections.end()){
        NS_LOG_HADI(" hwmp new, inserted at source " << srcIpv4Addr << ":" << (int)srcPort << "=>" << dstIpv4Addr << ":" << (int)dstPort << " n " << nextHop << " p " << prevHop);
        m_sourceCbrConnections.push_back(connection);
    }else{
        NS_LOG_HADI(" hwmp exist, expiration extended at source " << srcIpv4Addr << ":" << (int)srcPort << "=>" << dstIpv4Addr << ":" << (int)dstPort << " n " << nextHop << " p " << prevHop);
        ccvi->whenExpires=Simulator::Now()+Seconds(SOURCE_CBR_ROUTE_EXPIRE_MILLISECONDS);
      }
}

void
HwmpProtocol::SourceCbrRouteExtend(
			Mac48Address destination,
			Mac48Address source,
			uint8_t cnnType,
		    Ipv4Address srcIpv4Addr,
		    Ipv4Address dstIpv4Addr,
		    uint16_t srcPort,
		    uint16_t dstPort
		  ){
    NS_LOG_HADI(" hwmp cbr route extend at source " << srcIpv4Addr << ":" << (int)srcPort << "=>" << dstIpv4Addr << ":" << (int)dstPort);
    CbrConnection connection;
    connection.destination=destination;
    connection.source=source;
    connection.cnnType=cnnType;
    connection.dstIpv4Addr=dstIpv4Addr;
    connection.srcIpv4Addr=srcIpv4Addr;
    connection.dstPort=dstPort;
    connection.srcPort=srcPort;
    CbrConnectionsVector::iterator ccvi=std::find(m_sourceCbrConnections.begin(),m_sourceCbrConnections.end(),connection);
    if(ccvi!=m_sourceCbrConnections.end()){
        NS_LOG_HADI(" hwmp cbr route really found and extended at source " << srcIpv4Addr << ":" << (int)srcPort << "=>" << dstIpv4Addr << ":" << (int)dstPort << " n " << ccvi->nextMac << " p " << ccvi->prevMac);
        ccvi->whenExpires=Simulator::Now()+MilliSeconds(SOURCE_CBR_ROUTE_EXPIRE_MILLISECONDS);
    }else{
        NS_LOG_HADI(" hwmp cbr route not found and not extended at source " << srcIpv4Addr << ":" << (int)srcPort << "=>" << dstIpv4Addr << ":" << (int)dstPort);
      }
}

void
HwmpProtocol::InsertCbrCnnIntoCbrCnnsVector(
			Mac48Address destination,
			Mac48Address source,
			uint8_t cnnType,
		    Ipv4Address srcIpv4Addr,
		    Ipv4Address dstIpv4Addr,
		    uint16_t srcPort,
		    uint16_t dstPort,
		    Mac48Address prevHop,
		    Mac48Address nextHop
		  ){
	NS_LOG_HADI(" hwmp inserting cnn into cnnsvector " << srcIpv4Addr << ":" << (int)srcPort << "=>" << dstIpv4Addr << ":" << (int)dstPort << " n " << nextHop << " p " << prevHop);
    CbrConnection connection;
    connection.destination=destination;
    connection.source=source;
    connection.cnnType=cnnType;
    connection.dstIpv4Addr=dstIpv4Addr;
    connection.srcIpv4Addr=srcIpv4Addr;
    connection.dstPort=dstPort;
    connection.srcPort=srcPort;
    connection.prevMac=prevHop;
    connection.nextMac=nextHop;
    connection.whenExpires=Simulator::Now()+Seconds(CBR_ROUTE_EXPIRE_SECONDS);
    //connection.routeExpireEvent=Simulator::Schedule(Seconds(CBR_ROUTE_EXPIRE_SECONDS),&HwmpProtocol::CbrRouteExpire,this,connection);
    CbrConnectionsVector::iterator ccvi=std::find(m_cbrConnections.begin(),m_cbrConnections.end(),connection);
    if(ccvi==m_cbrConnections.end()){
        NS_LOG_HADI(" hwmp new, inserted " << srcIpv4Addr << ":" << (int)srcPort << "=>" << dstIpv4Addr << ":" << (int)dstPort << " n " << nextHop << " p " << prevHop);
          m_cbrConnections.push_back(connection);
    }else{
        NS_LOG_HADI(" hwmp exist, expiration extended " << srcIpv4Addr << ":" << (int)srcPort << "=>" << dstIpv4Addr << ":" << (int)dstPort << " n " << nextHop << " p " << prevHop);
        ccvi->whenExpires=Simulator::Now()+Seconds(CBR_ROUTE_EXPIRE_SECONDS);
        ccvi->nextMac=nextHop;
        ccvi->prevMac=prevHop;
        //m_cbrConnections.erase(ccvi);
        //m_cbrConnections.push_back(connection);
        //ccvi->routeExpireEvent.Cancel();
        //ccvi->routeExpireEvent=Simulator::Schedule(Seconds(CBR_ROUTE_EXPIRE_SECONDS),&HwmpProtocol::CbrRouteExpire,this,connection);

    }
}
void
HwmpProtocol::CbrRouteExtend(
			Mac48Address destination,
			Mac48Address source,
			uint8_t cnnType,
		    Ipv4Address srcIpv4Addr,
		    Ipv4Address dstIpv4Addr,
		    uint16_t srcPort,
		    uint16_t dstPort
		  ){
    NS_LOG_HADI(" hwmp cbr route extend " << srcIpv4Addr << ":" << (int)srcPort << "=>" << dstIpv4Addr << ":" << (int)dstPort);
    CbrConnection connection;
    connection.destination=destination;
    connection.source=source;
    connection.cnnType=cnnType;
    connection.dstIpv4Addr=dstIpv4Addr;
    connection.srcIpv4Addr=srcIpv4Addr;
    connection.dstPort=dstPort;
    connection.srcPort=srcPort;
    CbrConnectionsVector::iterator ccvi=std::find(m_cbrConnections.begin(),m_cbrConnections.end(),connection);
    if(ccvi!=m_cbrConnections.end()){
        NS_LOG_HADI(" hwmp cbr route really found and extended " << srcIpv4Addr << ":" << (int)srcPort << "=>" << dstIpv4Addr << ":" << (int)dstPort << " n " << ccvi->nextMac << " p " << ccvi->prevMac);
        ccvi->whenExpires=Simulator::Now()+Seconds(CBR_ROUTE_EXPIRE_SECONDS);
        //ccvi->routeExpireEvent.Cancel();
        //ccvi->routeExpireEvent=Simulator::Schedule(Seconds(CBR_ROUTE_EXPIRE_SECONDS),&HwmpProtocol::CbrRouteExpire,this,connection);
    }else{
        NS_LOG_HADI(" hwmp cbr route not found and not extended " << srcIpv4Addr << ":" << (int)srcPort << "=>" << dstIpv4Addr << ":" << (int)dstPort);
    }
}
void
HwmpProtocol::CbrRouteExpire(CbrConnection cbrCnn){
        NS_LOG_HADI(" hwmp cbr route expired " << cbrCnn.srcIpv4Addr << ":" << (int)cbrCnn.srcPort << "=>" << cbrCnn.dstIpv4Addr << ":" << (int)cbrCnn.dstPort << " n " << cbrCnn.nextMac << " p " << cbrCnn.prevMac);
    CbrConnectionsVector::iterator ccvi=std::find(m_cbrConnections.begin(),m_cbrConnections.end(),cbrCnn);
    if(ccvi!=m_cbrConnections.end()){
        m_cbrConnections.erase(ccvi);
        m_rtable->DeleteCnnBasedReactivePath(cbrCnn.destination,cbrCnn.source,cbrCnn.cnnType,cbrCnn.srcIpv4Addr,cbrCnn.dstIpv4Addr,cbrCnn.srcPort,cbrCnn.dstPort);
        NS_LOG_HADI(" hwmp cbr route deleted " << cbrCnn.srcIpv4Addr << ":" << (int)cbrCnn.srcPort << "=>" << cbrCnn.dstIpv4Addr << ":" << (int)cbrCnn.dstPort << " n " << cbrCnn.nextMac << " p " << cbrCnn.prevMac);
    }
}
void
HwmpProtocol::CheckCbrRoutes4Expiration(){
	CbrConnectionsVector tempvector;
	bool changed=false;
	for(CbrConnectionsVector::iterator ccvi=m_cbrConnections.begin();ccvi!=m_cbrConnections.end();ccvi++){
		if(Simulator::Now()<ccvi->whenExpires){
			tempvector.push_back(*ccvi);
		}else{
			changed = true;
			m_rtable->DeleteCnnBasedReactivePath(ccvi->destination,ccvi->source,ccvi->cnnType,ccvi->srcIpv4Addr,ccvi->dstIpv4Addr,ccvi->srcPort,ccvi->dstPort);
			NS_LOG_HADI(" hwmp cbr route expired and deleted " << ccvi->srcIpv4Addr << ":" << (int)ccvi->srcPort << "=>" << ccvi->dstIpv4Addr << ":" << (int)ccvi->dstPort << " n " << ccvi->nextMac << " p " << ccvi->prevMac);
		}
	}
	if(changed){
		m_cbrConnections.clear();
		m_cbrConnections=tempvector;
		NS_LOG_HADI(" hwmp num connections " << m_cbrConnections.size());
	}

	tempvector.clear();
	for(CbrConnectionsVector::iterator ccvi=m_sourceCbrConnections.begin();ccvi!=m_sourceCbrConnections.end();ccvi++){
		if(Simulator::Now()<ccvi->whenExpires){
			tempvector.push_back(*ccvi);
		}else{
			changed = true;
			m_CbrCnnStateChanged(ccvi->srcIpv4Addr,ccvi->dstIpv4Addr,ccvi->srcPort,ccvi->dstPort,false);
		}
	}
	if(changed){
		m_sourceCbrConnections.clear();
		m_sourceCbrConnections=tempvector;
	}

	Simulator::Schedule(MilliSeconds(50),&HwmpProtocol::CheckCbrRoutes4Expiration,this);
}


void
HwmpProtocol::ReceivePerr (std::vector<FailedDestination> destinations, Mac48Address from, uint32_t interface, Mac48Address fromMp)
{
  //Acceptance cretirea:
  NS_LOG_DEBUG ("I am "<<GetAddress ()<<", received PERR from "<<from);
  std::vector<FailedDestination> retval;
  HwmpRtable::LookupResult result;
  for (unsigned int i = 0; i < destinations.size (); i++)
    {
      result = m_rtable->LookupReactiveExpired (destinations[i].destination);
      if (!(
            (result.retransmitter != from) ||
            (result.ifIndex != interface) ||
            ((int32_t)(result.seqnum - destinations[i].seqnum) > 0)
            ))
        {
          retval.push_back (destinations[i]);
        }
    }
  if (retval.size () == 0)
    {
      return;
    }
  ForwardPathError (MakePathError (retval));
}
void
HwmpProtocol::SendPrep (
  Mac48Address src,
  Mac48Address dst,
  Mac48Address retransmitter,
  uint32_t initMetric,
  uint8_t cnnType,
  Ipv4Address srcIpv4Addr,
  Ipv4Address dstIpv4Addr,
  uint16_t srcPort,
  uint16_t dstPort,
  uint32_t originatorDsn,
  uint32_t destinationSN,
  uint32_t lifetime,
  uint32_t interface)
{
  IePrep prep;

  prep.SetHopcount (0);
  prep.SetTtl (m_maxTtl);
  prep.SetDestinationAddress (dst);
  prep.SetDestinationSeqNumber (destinationSN);
  prep.SetLifetime (lifetime);
  prep.SetMetric (initMetric);
  prep.SetCnnParams(cnnType,srcIpv4Addr,dstIpv4Addr,srcPort,dstPort);
  prep.SetOriginatorAddress (src);
  prep.SetOriginatorSeqNumber (originatorDsn);
  HwmpProtocolMacMap::const_iterator prep_sender = m_interfaces.find (interface);
  NS_ASSERT (prep_sender != m_interfaces.end ());
  prep_sender->second->SendPrep (prep, retransmitter);
  m_stats.initiatedPrep++;
}
bool
HwmpProtocol::Install (Ptr<MeshPointDevice> mp)
{
  m_mp = mp;
  std::vector<Ptr<NetDevice> > interfaces = mp->GetInterfaces ();
  for (std::vector<Ptr<NetDevice> >::const_iterator i = interfaces.begin (); i != interfaces.end (); i++)
    {
      // Checking for compatible net device
      Ptr<WifiNetDevice> wifiNetDev = (*i)->GetObject<WifiNetDevice> ();
      if (wifiNetDev == 0)
        {
          return false;
        }
      Ptr<MeshWifiInterfaceMac>  mac = wifiNetDev->GetMac ()->GetObject<MeshWifiInterfaceMac> ();
      if (mac == 0)
        {
          return false;
        }
      // Installing plugins:
      Ptr<HwmpProtocolMac> hwmpMac = Create<HwmpProtocolMac> (wifiNetDev->GetIfIndex (), this);
      m_interfaces[wifiNetDev->GetIfIndex ()] = hwmpMac;
      mac->InstallPlugin (hwmpMac);
      //Installing airtime link metric:
      Ptr<AirtimeLinkMetricCalculator> metric = CreateObject <AirtimeLinkMetricCalculator> ();
      mac->SetLinkMetricCallback (MakeCallback (&AirtimeLinkMetricCalculator::CalculateMetric, metric));
    }
  mp->SetRoutingProtocol (this);
  // Mesh point aggregates all installed protocols
  mp->AggregateObject (this);
  m_address = Mac48Address::ConvertFrom (mp->GetAddress ()); // address;
  return true;
}
void
HwmpProtocol::PeerLinkStatus (Mac48Address meshPointAddress, Mac48Address peerAddress, uint32_t interface, bool status)
{
  if (status)
    {
      return;
    }
  std::vector<FailedDestination> destinations = m_rtable->GetUnreachableDestinations (peerAddress);
  InitiatePathError (MakePathError (destinations));
}
void
HwmpProtocol::SetNeighboursCallback (Callback<std::vector<Mac48Address>, uint32_t> cb)
{
  m_neighboursCallback = cb;
}
bool
HwmpProtocol::DropDataFrame (uint32_t seqno, Mac48Address source)
{
  if (source == GetAddress ())
    {
      return true;
    }
  std::map<Mac48Address, uint32_t,std::less<Mac48Address> >::const_iterator i = m_lastDataSeqno.find (source);
  if (i == m_lastDataSeqno.end ())
    {
      m_lastDataSeqno[source] = seqno;
    }
  else
    {
      if ((int32_t)(i->second - seqno)  >= 0)
        {
          return true;
        }
      m_lastDataSeqno[source] = seqno;
    }
  return false;
}
HwmpProtocol::PathError
HwmpProtocol::MakePathError (std::vector<FailedDestination> destinations)
{
  PathError retval;
  //HwmpRtable increments a sequence number as written in 11B.9.7.2
  retval.receivers = GetPerrReceivers (destinations);
  if (retval.receivers.size () == 0)
    {
      return retval;
    }
  m_stats.initiatedPerr++;
  for (unsigned int i = 0; i < destinations.size (); i++)
    {
      retval.destinations.push_back (destinations[i]);
      m_rtable->DeleteReactivePath (destinations[i].destination);
    }
  return retval;
}
void
HwmpProtocol::InitiatePathError (PathError perr)
{
  for (HwmpProtocolMacMap::const_iterator i = m_interfaces.begin (); i != m_interfaces.end (); i++)
    {
      std::vector<Mac48Address> receivers_for_interface;
      for (unsigned int j = 0; j < perr.receivers.size (); j++)
        {
          if (i->first == perr.receivers[j].first)
            {
              receivers_for_interface.push_back (perr.receivers[j].second);
            }
        }
      i->second->InitiatePerr (perr.destinations, receivers_for_interface);
    }
}
void
HwmpProtocol::ForwardPathError (PathError perr)
{
  for (HwmpProtocolMacMap::const_iterator i = m_interfaces.begin (); i != m_interfaces.end (); i++)
    {
      std::vector<Mac48Address> receivers_for_interface;
      for (unsigned int j = 0; j < perr.receivers.size (); j++)
        {
          if (i->first == perr.receivers[j].first)
            {
              receivers_for_interface.push_back (perr.receivers[j].second);
            }
        }
      i->second->ForwardPerr (perr.destinations, receivers_for_interface);
    }
}

std::vector<std::pair<uint32_t, Mac48Address> >
HwmpProtocol::GetPerrReceivers (std::vector<FailedDestination> failedDest)
{
  HwmpRtable::PrecursorList retval;
  for (unsigned int i = 0; i < failedDest.size (); i++)
    {
      HwmpRtable::PrecursorList precursors = m_rtable->GetPrecursors (failedDest[i].destination);
      m_rtable->DeleteReactivePath (failedDest[i].destination);
      m_rtable->DeleteProactivePath (failedDest[i].destination);
      for (unsigned int j = 0; j < precursors.size (); j++)
        {
          retval.push_back (precursors[j]);
        }
    }
  //Check if we have dublicates in retval and precursors:
  for (unsigned int i = 0; i < retval.size (); i++)
    {
      for (unsigned int j = i+1; j < retval.size (); j++)
        {
          if (retval[i].second == retval[j].second)
            {
              retval.erase (retval.begin () + j);
            }
        }
    }
  return retval;
}
std::vector<Mac48Address>
HwmpProtocol::GetPreqReceivers (uint32_t interface)
{
  std::vector<Mac48Address> retval;
  if (!m_neighboursCallback.IsNull ())
    {
      retval = m_neighboursCallback (interface);
    }
  if ((retval.size () >= m_unicastPreqThreshold) || (retval.size () == 0))
    {
      retval.clear ();
      retval.push_back (Mac48Address::GetBroadcast ());
    }
  return retval;
}
std::vector<Mac48Address>
HwmpProtocol::GetBroadcastReceivers (uint32_t interface)
{
  std::vector<Mac48Address> retval;
  if (!m_neighboursCallback.IsNull ())
    {
      retval = m_neighboursCallback (interface);
    }
  if ((retval.size () >= m_unicastDataThreshold) || (retval.size () == 0))
    {
      retval.clear ();
      retval.push_back (Mac48Address::GetBroadcast ());
    }
  return retval;
}

bool
HwmpProtocol::QueuePacket (QueuedPacket packet)
{
  if (m_rqueue.size () > m_maxQueueSize)
    {
      return false;
    }
  m_rqueue.push_back (packet);
  return true;
}

HwmpProtocol::QueuedPacket
HwmpProtocol::DequeueFirstPacketByCnnParams (
                        Mac48Address dst,
                        Mac48Address src,
                        uint8_t cnnType,
                        Ipv4Address srcIpv4Addr,
                        Ipv4Address dstIpv4Addr,
                        uint16_t srcPort,
                        uint16_t dstPort
                                            )
{
  QueuedPacket retval;
  retval.pkt = 0;
  NS_LOG_HADI(" hwmp DequeueFirstPacketByCnnParams " << (int)m_rqueue.size());
  for (std::vector<QueuedPacket>::iterator i = m_rqueue.begin (); i != m_rqueue.end (); i++)
    {
      if (
              ((*i).dst == dst)                 &&
              ((*i).src == src)                 &&
              ((*i).cnnType == cnnType)         &&
              ((*i).srcIpv4Addr == srcIpv4Addr) &&
              ((*i).dstIpv4Addr == dstIpv4Addr) &&
              ((*i).srcPort == srcPort)         &&
              ((*i).dstPort == dstPort)
         )
        {
          retval = (*i);
          m_rqueue.erase (i);
          break;
        }
    }
  //std::cout << Simulator::Now().GetSeconds() << " " << m_address << " SourceQueueSize " << m_rqueue.size() << std::endl;
  return retval;
}

HwmpProtocol::QueuedPacket
HwmpProtocol::DequeueFirstPacketByDst (Mac48Address dst)
{
  QueuedPacket retval;
  retval.pkt = 0;
  for (std::vector<QueuedPacket>::iterator i = m_rqueue.begin (); i != m_rqueue.end (); i++)
    {
      if ((*i).dst == dst)
        {
          retval = (*i);
          m_rqueue.erase (i);
          break;
        }
    }
  return retval;
}

HwmpProtocol::QueuedPacket
HwmpProtocol::DequeueFirstPacket ()
{
  QueuedPacket retval;
  retval.pkt = 0;
  if (m_rqueue.size () != 0)
    {
      retval = m_rqueue[0];
      m_rqueue.erase (m_rqueue.begin ());
    }
  return retval;
}

void
HwmpProtocol::ReactivePathResolved (Mac48Address dst)
{
  std::map<Mac48Address, PreqEvent>::iterator i = m_preqTimeouts.find (dst);
  if (i != m_preqTimeouts.end ())
    {
      m_routeDiscoveryTimeCallback (Simulator::Now () - i->second.whenScheduled);
    }

  HwmpRtable::LookupResult result = m_rtable->LookupReactive (dst);
  NS_ASSERT (result.retransmitter != Mac48Address::GetBroadcast ());
  //Send all packets stored for this destination
  QueuedPacket packet = DequeueFirstPacketByDst (dst);
  while (packet.pkt != 0)
    {
          if(packet.src==GetAddress()){
                  NS_LOG_HADI(m_address << " tx4mSource2 " << (int)packet.pkt->GetUid());
          }
      //set RA tag for retransmitter:
      HwmpTag tag;
      packet.pkt->RemovePacketTag (tag);
      tag.SetAddress (result.retransmitter);
      packet.pkt->AddPacketTag (tag);
      m_stats.txUnicast++;
      m_stats.txBytes += packet.pkt->GetSize ();
      packet.reply (true, packet.pkt, packet.src, packet.dst, packet.protocol, result.ifIndex);

      packet = DequeueFirstPacketByDst (dst);
    }
}

void
HwmpProtocol::CnnBasedReactivePathResolved (
                              Mac48Address dst,
                              Mac48Address src,
                              uint8_t cnnType,
                              Ipv4Address srcIpv4Addr,
                              Ipv4Address dstIpv4Addr,
                              uint16_t srcPort,
                              uint16_t dstPort
                                            )
{

  HwmpRtable::CnnBasedLookupResult result = m_rtable->LookupCnnBasedReactive(dst,src,cnnType,srcIpv4Addr,dstIpv4Addr,srcPort,dstPort);
  NS_ASSERT (result.retransmitter != Mac48Address::GetBroadcast ());
  //Send all packets stored for this destination
  QueuedPacket packet = DequeueFirstPacketByCnnParams (dst,src,cnnType,srcIpv4Addr,dstIpv4Addr,srcPort,dstPort);
  while (packet.pkt != 0)
    {
          if((packet.src==GetAddress())&&(cnnType==HwmpRtable::CNN_TYPE_IP_PORT)){
		  NS_LOG_HADI(m_address << " tx4mSource2 " << (int)packet.pkt->GetUid());
		  m_txed4mSourceCallback();
	  }
      //set RA tag for retransmitter:
      HwmpTag tag;
      packet.pkt->RemovePacketTag (tag);
      tag.SetAddress (result.retransmitter);
      packet.pkt->AddPacketTag (tag);
      m_stats.txUnicast++;
      m_stats.txBytes += packet.pkt->GetSize ();
      packet.reply (true, packet.pkt, packet.src, packet.dst, packet.protocol, result.ifIndex);

      packet = DequeueFirstPacketByCnnParams (dst,src,cnnType,srcIpv4Addr,dstIpv4Addr,srcPort,dstPort);
    }
}
double
HwmpProtocol::IntegrateNumerical(double a,double b,double alpha,double beta,double x0,double threshold)
{
  //(x0/(std::sqrt(2*PI*alpha*std::pow(t,3))))*std::exp(-(std::pow(x0+beta*t,2)/(2*alpha*t)));
  double integral=0;
  double newVal;
  double oldVal=(x0/(std::sqrt(2*PI*alpha*std::pow(a,3))))*std::exp(-(std::pow(x0+beta*a,2)/(2*alpha*a)));
  bool increasing=true;
  for(double t=a+0.01;t<b;t+=0.01)
    {
      newVal=(x0/(std::sqrt(2*PI*alpha*std::pow(t,3))))*std::exp(-(std::pow(x0+beta*t,2)/(2*alpha*t)));
      if(newVal<oldVal)
        increasing=false;
      if(!increasing)
        {
          if((integral+(b-t)*newVal)<threshold)
            return (threshold-0.01);
        }
      integral+=(newVal+oldVal)*0.005;
      if(integral>threshold)
        return integral;
      oldVal=newVal;
    }
  return integral;
}

void
HwmpProtocol::ProactivePathResolved ()
{
  //send all packets to root
  HwmpRtable::LookupResult result = m_rtable->LookupProactive ();
  NS_ASSERT (result.retransmitter != Mac48Address::GetBroadcast ());
  QueuedPacket packet = DequeueFirstPacket ();
  while (packet.pkt != 0)
    {
      //set RA tag for retransmitter:
      HwmpTag tag;
      if (!packet.pkt->RemovePacketTag (tag))
        {
          NS_FATAL_ERROR ("HWMP tag must be present at this point");
        }
      tag.SetAddress (result.retransmitter);
      packet.pkt->AddPacketTag (tag);
      m_stats.txUnicast++;
      m_stats.txBytes += packet.pkt->GetSize ();
      packet.reply (true, packet.pkt, packet.src, packet.dst, packet.protocol, result.ifIndex);

      packet = DequeueFirstPacket ();
    }
}

bool
HwmpProtocol::ShouldSendPreq (Mac48Address dst)
{
  std::map<Mac48Address, PreqEvent>::const_iterator i = m_preqTimeouts.find (dst);
  if (i == m_preqTimeouts.end ())
    {
      m_preqTimeouts[dst].preqTimeout = Simulator::Schedule (
          Time (m_dot11MeshHWMPnetDiameterTraversalTime * 2),
          &HwmpProtocol::RetryPathDiscovery, this, dst, 1);
      m_preqTimeouts[dst].whenScheduled = Simulator::Now ();
      return true;
    }
  return false;
}
bool
HwmpProtocol::CnnBasedShouldSendPreq (
            Mac48Address dst,
            Mac48Address src,
            uint8_t cnnType,
            Ipv4Address srcIpv4Addr,
            Ipv4Address dstIpv4Addr,
            uint16_t srcPort,
            uint16_t dstPort
  )
{
  for(std::vector<CnnBasedPreqEvent>::iterator cbpei = m_cnnBasedPreqTimeouts.begin (); cbpei != m_cnnBasedPreqTimeouts.end (); cbpei++)
  {
      if(
                (cbpei->destination==dst)               &&
                (cbpei->source==src)                    &&
                (cbpei->cnnType==cnnType)               &&
                (cbpei->srcIpv4Addr==srcIpv4Addr)       &&
                (cbpei->dstIpv4Addr==dstIpv4Addr)       &&
                (cbpei->srcPort==srcPort)               &&
                (cbpei->dstPort==dstPort)
        )
      {
          return false;
      }
  }
  CnnBasedPreqEvent cbpe;
  cbpe.destination=dst;
  cbpe.source=src;
  cbpe.cnnType=cnnType;
  cbpe.srcIpv4Addr=srcIpv4Addr;
  cbpe.dstIpv4Addr=dstIpv4Addr;
  cbpe.srcPort=srcPort;
  cbpe.dstPort=dstPort;
  cbpe.whenScheduled=Simulator::Now();
  cbpe.preqTimeout=Simulator::Schedule(
                  Time (m_dot11MeshHWMPnetDiameterTraversalTime * 2),
                  &HwmpProtocol::CnnBasedRetryPathDiscovery,this,cbpe,1);
  m_cnnBasedPreqTimeouts.push_back(cbpe);
  return true;
}
void
HwmpProtocol::RetryPathDiscovery (Mac48Address dst, uint8_t numOfRetry)
{
  HwmpRtable::LookupResult result = m_rtable->LookupReactive (dst);
  if (result.retransmitter == Mac48Address::GetBroadcast ())
    {
      result = m_rtable->LookupProactive ();
    }
  if (result.retransmitter != Mac48Address::GetBroadcast ())
    {
      std::map<Mac48Address, PreqEvent>::iterator i = m_preqTimeouts.find (dst);
      NS_ASSERT (i != m_preqTimeouts.end ());
      m_preqTimeouts.erase (i);
      return;
    }
  if (numOfRetry > m_dot11MeshHWMPmaxPREQretries)
    {
	  NS_LOG_HADI(m_address << " givingUpPathRequest " << dst);
      QueuedPacket packet = DequeueFirstPacketByDst (dst);
      //purge queue and delete entry from retryDatabase
      while (packet.pkt != 0)
        {
          m_stats.totalDropped++;
          packet.reply (false, packet.pkt, packet.src, packet.dst, packet.protocol, HwmpRtable::MAX_METRIC);
          packet = DequeueFirstPacketByDst (dst);
        }
      std::map<Mac48Address, PreqEvent>::iterator i = m_preqTimeouts.find (dst);
      NS_ASSERT (i != m_preqTimeouts.end ());
      m_routeDiscoveryTimeCallback (Simulator::Now () - i->second.whenScheduled);
      m_preqTimeouts.erase (i);
      return;
    }
  numOfRetry++;
  uint32_t originator_seqno = GetNextHwmpSeqno ();
  uint32_t dst_seqno = m_rtable->LookupReactiveExpired (dst).seqnum;
  NS_LOG_HADI(m_address << " retryPathRequest " << dst);
  for (HwmpProtocolMacMap::const_iterator i = m_interfaces.begin (); i != m_interfaces.end (); i++)
    {
      Ipv4Address tempadd;
      i->second->RequestDestination (dst, originator_seqno, dst_seqno,HwmpRtable::CNN_TYPE_PKT_BASED,tempadd,tempadd,0,0,0,0,Seconds (0));
    }
  m_preqTimeouts[dst].preqTimeout = Simulator::Schedule (
      Time ((2 * (numOfRetry + 1)) *  m_dot11MeshHWMPnetDiameterTraversalTime),
      &HwmpProtocol::RetryPathDiscovery, this, dst, numOfRetry);
}
void
HwmpProtocol::CnnBasedRetryPathDiscovery (
                        CnnBasedPreqEvent preqEvent,
                        uint8_t numOfRetry
                                  )
{
  HwmpRtable::CnnBasedLookupResult result = m_rtable->LookupCnnBasedReactive(preqEvent.destination,preqEvent.source,preqEvent.cnnType,preqEvent.srcIpv4Addr,preqEvent.dstIpv4Addr,preqEvent.srcPort,preqEvent.dstPort);
  if (result.retransmitter != Mac48Address::GetBroadcast ())
    {
        for(std::vector<CnnBasedPreqEvent>::iterator cbpei = m_cnnBasedPreqTimeouts.begin (); cbpei != m_cnnBasedPreqTimeouts.end (); cbpei++)
        {
            if(
                      (cbpei->destination==preqEvent.destination)       &&
                      (cbpei->source==preqEvent.source)                 &&
                      (cbpei->cnnType==preqEvent.cnnType)               &&
                      (cbpei->srcIpv4Addr==preqEvent.srcIpv4Addr)       &&
                      (cbpei->dstIpv4Addr==preqEvent.dstIpv4Addr)       &&
                      (cbpei->srcPort==preqEvent.srcPort)               &&
                      (cbpei->dstPort==preqEvent.dstPort)
              )
            {
                m_cnnBasedPreqTimeouts.erase(cbpei);
                return;
            }
        }
      NS_ASSERT (false);
      return;
    }
  if (numOfRetry > m_dot11MeshHWMPmaxPREQretries)
    {
      //hadireport reject connection
          NS_LOG_HADI(" hwmp connectionRejected            " << preqEvent.destination << " " << preqEvent.srcIpv4Addr << ":" << (int)preqEvent.srcPort << "=>" << preqEvent.dstIpv4Addr << ":" << (int)preqEvent.dstPort);
      QueuedPacket packet = DequeueFirstPacketByCnnParams (preqEvent.destination,preqEvent.source,preqEvent.cnnType,preqEvent.srcIpv4Addr,preqEvent.dstIpv4Addr,preqEvent.srcPort,preqEvent.dstPort);
      CbrConnection connection;
      connection.destination=preqEvent.destination;
      connection.source=preqEvent.source;
      connection.cnnType=preqEvent.cnnType;
      connection.dstIpv4Addr=preqEvent.dstIpv4Addr;
      connection.srcIpv4Addr=preqEvent.srcIpv4Addr;
      connection.dstPort=preqEvent.dstPort;
      connection.srcPort=preqEvent.srcPort;
      CbrConnectionsVector::iterator nrccvi=std::find(m_notRoutedCbrConnections.begin(),m_notRoutedCbrConnections.end(),connection);
      if(nrccvi==m_notRoutedCbrConnections.end()){
          m_notRoutedCbrConnections.push_back(connection);
      }

      //purge queue and delete entry from retryDatabase
      while (packet.pkt != 0)
        {
          if(packet.src==GetAddress()){
                  NS_LOG_HADI(" hwmp noRouteDrop2 " << (int)packet.pkt->GetUid() << " " << preqEvent.srcIpv4Addr << ":" << (int)preqEvent.srcPort << "=>" << preqEvent.dstIpv4Addr << ":" << (int)preqEvent.dstPort);
          }
          m_stats.totalDropped++;
          packet.reply (false, packet.pkt, packet.src, packet.dst, packet.protocol, HwmpRtable::MAX_METRIC);
          packet = DequeueFirstPacketByCnnParams (preqEvent.destination,preqEvent.source,preqEvent.cnnType,preqEvent.srcIpv4Addr,preqEvent.dstIpv4Addr,preqEvent.srcPort,preqEvent.dstPort);
        }
        for(std::vector<CnnBasedPreqEvent>::iterator cbpei = m_cnnBasedPreqTimeouts.begin (); cbpei != m_cnnBasedPreqTimeouts.end (); cbpei++)
        {
            if(
                      (cbpei->destination==preqEvent.destination)               &&
                      (cbpei->cnnType==preqEvent.cnnType)               &&
                      (cbpei->srcIpv4Addr==preqEvent.srcIpv4Addr)       &&
                      (cbpei->dstIpv4Addr==preqEvent.dstIpv4Addr)       &&
                      (cbpei->srcPort==preqEvent.srcPort)               &&
                      (cbpei->dstPort==preqEvent.dstPort)
              )
            {
                m_cnnBasedPreqTimeouts.erase(cbpei);
                return;
            }
        }
      NS_ASSERT (false);
      return;
    }
  numOfRetry++;
  uint32_t originator_seqno = GetNextHwmpSeqno ();
  uint32_t dst_seqno = 0;
  for (HwmpProtocolMacMap::const_iterator i = m_interfaces.begin (); i != m_interfaces.end (); i++)
    {
      i->second->RequestDestination (preqEvent.destination, originator_seqno, dst_seqno, preqEvent.cnnType, preqEvent.srcIpv4Addr, preqEvent.dstIpv4Addr, preqEvent.srcPort, preqEvent.dstPort,0,0,Seconds (0));
    }
  for(std::vector<CnnBasedPreqEvent>::iterator cbpei = m_cnnBasedPreqTimeouts.begin (); cbpei != m_cnnBasedPreqTimeouts.end (); cbpei++)
  {
      if(
                (cbpei->destination==preqEvent.destination)               &&
                (cbpei->cnnType==preqEvent.cnnType)               &&
                (cbpei->srcIpv4Addr==preqEvent.srcIpv4Addr)       &&
                (cbpei->dstIpv4Addr==preqEvent.dstIpv4Addr)       &&
                (cbpei->srcPort==preqEvent.srcPort)               &&
                (cbpei->dstPort==preqEvent.dstPort)
        )
      {
          cbpei->preqTimeout=Simulator::Schedule(
                  Time ((2 * (numOfRetry + 1)) *  m_dot11MeshHWMPnetDiameterTraversalTime),
                  &HwmpProtocol::CnnBasedRetryPathDiscovery,this,(*cbpei),numOfRetry);
          cbpei->whenScheduled=Simulator::Now();
          return;
      }
  }
  CnnBasedPreqEvent cbpe;
  cbpe.destination=preqEvent.destination;
  cbpe.cnnType=preqEvent.cnnType;
  cbpe.srcIpv4Addr=preqEvent.srcIpv4Addr;
  cbpe.dstIpv4Addr=preqEvent.dstIpv4Addr;
  cbpe.srcPort=preqEvent.srcPort;
  cbpe.dstPort=preqEvent.dstPort;
  cbpe.whenScheduled=Simulator::Now();
  cbpe.preqTimeout=Simulator::Schedule(
                  Time ((2 * (numOfRetry + 1)) *  m_dot11MeshHWMPnetDiameterTraversalTime),
                  &HwmpProtocol::CnnBasedRetryPathDiscovery,this,cbpe,numOfRetry);
  m_cnnBasedPreqTimeouts.push_back(cbpe);
}
//Proactive PREQ routines:
void
HwmpProtocol::SetRoot ()
{
  Time randomStart = Seconds (m_coefficient->GetValue ());
  m_proactivePreqTimer = Simulator::Schedule (randomStart, &HwmpProtocol::SendProactivePreq, this);
  NS_LOG_DEBUG ("ROOT IS: " << m_address);
  m_isRoot = true;
}
void
HwmpProtocol::UnsetRoot ()
{
  m_proactivePreqTimer.Cancel ();
}
void
HwmpProtocol::SendProactivePreq ()
{
  IePreq preq;
  //By default: must answer
  preq.SetHopcount (0);
  preq.SetTTL (m_maxTtl);
  preq.SetLifetime (m_dot11MeshHWMPactiveRootTimeout.GetMicroSeconds () /1024);
  //\attention: do not forget to set originator address, sequence
  //number and preq ID in HWMP-MAC plugin
  preq.AddDestinationAddressElement (true, true, Mac48Address::GetBroadcast (), 0);
  preq.SetOriginatorAddress (GetAddress ());
  preq.SetPreqID (GetNextPreqId ());
  preq.SetOriginatorSeqNumber (GetNextHwmpSeqno ());
  for (HwmpProtocolMacMap::const_iterator i = m_interfaces.begin (); i != m_interfaces.end (); i++)
    {
      i->second->SendPreq (preq);
    }
  m_proactivePreqTimer = Simulator::Schedule (m_dot11MeshHWMPpathToRootInterval, &HwmpProtocol::SendProactivePreq, this);
}
bool
HwmpProtocol::GetDoFlag ()
{
  return m_doFlag;
}
bool
HwmpProtocol::GetRfFlag ()
{
  return m_rfFlag;
}
Time
HwmpProtocol::GetPreqMinInterval ()
{
  return m_dot11MeshHWMPpreqMinInterval;
}
Time
HwmpProtocol::GetPerrMinInterval ()
{
  return m_dot11MeshHWMPperrMinInterval;
}
uint8_t
HwmpProtocol::GetMaxTtl ()
{
  return m_maxTtl;
}
uint32_t
HwmpProtocol::GetNextPreqId ()
{
  m_preqId++;
  return m_preqId;
}
uint32_t
HwmpProtocol::GetNextHwmpSeqno ()
{
  m_hwmpSeqno++;
  return m_hwmpSeqno;
}
uint32_t
HwmpProtocol::GetActivePathLifetime ()
{
  return m_dot11MeshHWMPactivePathTimeout.GetMicroSeconds () / 1024;
}
uint8_t
HwmpProtocol::GetUnicastPerrThreshold ()
{
  return m_unicastPerrThreshold;
}
Mac48Address
HwmpProtocol::GetAddress ()
{
  return m_address;
}
//Statistics:
HwmpProtocol::Statistics::Statistics () :
  txUnicast (0),
  txBroadcast (0),
  txBytes (0),
  droppedTtl (0),
  totalQueued (0),
  totalDropped (0),
  initiatedPreq (0),
  initiatedPrep (0),
  initiatedPerr (0)
{
}
void HwmpProtocol::Statistics::Print (std::ostream & os) const
{
  os << "<Statistics "
  "txUnicast=\"" << txUnicast << "\" "
  "txBroadcast=\"" << txBroadcast << "\" "
  "txBytes=\"" << txBytes << "\" "
  "droppedTtl=\"" << droppedTtl << "\" "
  "totalQueued=\"" << totalQueued << "\" "
  "totalDropped=\"" << totalDropped << "\" "
  "initiatedPreq=\"" << initiatedPreq << "\" "
  "initiatedPrep=\"" << initiatedPrep << "\" "
  "initiatedPerr=\"" << initiatedPerr << "\"/>" << std::endl;
}
void
HwmpProtocol::Report (std::ostream & os) const
{
  os << "<Hwmp "
  "address=\"" << m_address << "\"" << std::endl <<
  "maxQueueSize=\"" << m_maxQueueSize << "\"" << std::endl <<
  "Dot11MeshHWMPmaxPREQretries=\"" << (uint16_t)m_dot11MeshHWMPmaxPREQretries << "\"" << std::endl <<
  "Dot11MeshHWMPnetDiameterTraversalTime=\"" << m_dot11MeshHWMPnetDiameterTraversalTime.GetSeconds () << "\"" << std::endl <<
  "Dot11MeshHWMPpreqMinInterval=\"" << m_dot11MeshHWMPpreqMinInterval.GetSeconds () << "\"" << std::endl <<
  "Dot11MeshHWMPperrMinInterval=\"" << m_dot11MeshHWMPperrMinInterval.GetSeconds () << "\"" << std::endl <<
  "Dot11MeshHWMPactiveRootTimeout=\"" << m_dot11MeshHWMPactiveRootTimeout.GetSeconds () << "\"" << std::endl <<
  "Dot11MeshHWMPactivePathTimeout=\"" << m_dot11MeshHWMPactivePathTimeout.GetSeconds () << "\"" << std::endl <<
  "Dot11MeshHWMPpathToRootInterval=\"" << m_dot11MeshHWMPpathToRootInterval.GetSeconds () << "\"" << std::endl <<
  "Dot11MeshHWMPrannInterval=\"" << m_dot11MeshHWMPrannInterval.GetSeconds () << "\"" << std::endl <<
  "isRoot=\"" << m_isRoot << "\"" << std::endl <<
  "maxTtl=\"" << (uint16_t)m_maxTtl << "\"" << std::endl <<
  "unicastPerrThreshold=\"" << (uint16_t)m_unicastPerrThreshold << "\"" << std::endl <<
  "unicastPreqThreshold=\"" << (uint16_t)m_unicastPreqThreshold << "\"" << std::endl <<
  "unicastDataThreshold=\"" << (uint16_t)m_unicastDataThreshold << "\"" << std::endl <<
  "doFlag=\"" << m_doFlag << "\"" << std::endl <<
  "rfFlag=\"" << m_rfFlag << "\">" << std::endl;
  m_stats.Print (os);
  for (HwmpProtocolMacMap::const_iterator plugin = m_interfaces.begin (); plugin != m_interfaces.end (); plugin++)
    {
      plugin->second->Report (os);
    }
  os << "</Hwmp>" << std::endl;
}
void
HwmpProtocol::ResetStats ()
{
  m_stats = Statistics ();
  for (HwmpProtocolMacMap::const_iterator plugin = m_interfaces.begin (); plugin != m_interfaces.end (); plugin++)
    {
      plugin->second->ResetStats ();
    }
}

int64_t
HwmpProtocol::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_coefficient->SetStream (stream);
  return 1;
}

HwmpProtocol::QueuedPacket::QueuedPacket () :
  pkt (0),
  protocol (0),
  inInterface (0)
{
}
} // namespace dot11s
} // namespace ns3
