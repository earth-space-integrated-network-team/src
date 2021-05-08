#include <vector>
#include <iomanip>
#include "ns3/names.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/net-device.h"
#include "ns3/ipv4-route.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/boolean.h"
#include "ns3/node.h"
#include "satellite-global-routing.h"


namespace ns3{

TypeId
SatelliteGlobalRouting::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SatelliteGlobalRouting")
    .SetParent<Object> ()
    .SetGroupName ("Internet");
  return tid;
}

bool
SatelliteGlobalRouting::RouteInput  (Ptr<const Packet> p, const Ipv4Header &header, Ptr<const NetDevice> idev,
		                             UnicastForwardCallback ucb, MulticastForwardCallback mcb,
                                     LocalDeliverCallback lcb, ErrorCallback ecb)
{
  NS_LOG_FUNCTION (this << p << header << header.GetSource () << header.GetDestination () << idev << &lcb << &ecb);
  // Check if input device supports IP
  NS_ASSERT (m_ipv4->GetInterfaceForDevice (idev) >= 0);
  uint32_t iif = m_ipv4->GetInterfaceForDevice (idev);

  if (m_ipv4->IsDestinationAddress (header.GetDestination (), iif))
    {
      if (!lcb.IsNull ())
        {
          NS_LOG_LOGIC ("Local delivery to " << header.GetDestination ());
          lcb (p, header, iif);
          return true;
        }
      else
        {
          // The local delivery callback is null.  This may be a multicast
          // or broadcast packet, so return false so that another
          // multicast routing protocol can handle it.  It should be possible
          // to extend this to explicitly check whether it is a unicast
          // packet, and invoke the error callback if so
          return false;
        }
    }

  // Check if input device supports IP forwarding
  if (m_ipv4->IsForwarding (iif) == false)
    {
      NS_LOG_LOGIC ("Forwarding disabled for this interface");
      ecb (p, header, Socket::ERROR_NOROUTETOHOST);
      return true;
    }
  // Next, try to find a route
  NS_LOG_LOGIC ("Unicast destination- looking up global route");
  Ptr<Ipv4Route> rtentry = LookupForwarding(header.GetDestination ());
  if (rtentry != 0)
    {
      NS_LOG_LOGIC ("Found unicast destination- calling unicast callback");
      ucb (rtentry, p, header);
      return true;
    }
  else
    {
      NS_LOG_LOGIC ("Did not find unicast destination- returning false");
      return false; // Let other routing protocols try to handle this
                    // route request.
    }
}

Ptr<Ipv4Route>
SatelliteGlobalRouting::RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr)
{
  NS_LOG_FUNCTION (this << p << &header << oif << &sockerr);
//
// First, see if this is a multicast packet we have a route for.  If we
// have a route, then send the packet down each of the specified interfaces.
//
  if (header.GetDestination ().IsMulticast ())
    {
      NS_LOG_LOGIC ("Multicast destination-- returning false");
      return 0; // Let other routing protocols try to handle this
    }
//
// See if this is a unicast packet we have a route for.
//
  NS_LOG_LOGIC ("Unicast destination- looking up");
  Ptr<Ipv4Route> rtentry = LookupForwarding (header.GetDestination (), oif);
  if (rtentry)
    {
      sockerr = Socket::ERROR_NOTERROR;
    }
  else
    {
      sockerr = Socket::ERROR_NOROUTETOHOST;
    }
  return rtentry;
}

void
SatelliteGlobalRouting::NotifyInterfaceUp (uint32_t i)
{
  NS_LOG_FUNCTION (this << i);
//  if (m_respondToInterfaceEvents && Simulator::Now ().GetSeconds () > 0)  // avoid startup events
//    {
////      GlobalRouteManager::DeleteGlobalRoutes ();
////      GlobalRouteManager::BuildGlobalRoutingDatabase ();
////      GlobalRouteManager::InitializeRoutes ();
//    }
}

void
SatelliteGlobalRouting::NotifyInterfaceDown (uint32_t i)
{
  NS_LOG_FUNCTION (this << i);
//  if (m_respondToInterfaceEvents && Simulator::Now ().GetSeconds () > 0)  // avoid startup events
//    {
////      GlobalRouteManager::DeleteGlobalRoutes ();
////      GlobalRouteManager::BuildGlobalRoutingDatabase ();
////      GlobalRouteManager::InitializeRoutes ();
//    }
}

void
SatelliteGlobalRouting::NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address)
{
  NS_LOG_FUNCTION (this << interface << address);
//  if (m_respondToInterfaceEvents && Simulator::Now ().GetSeconds () > 0)  // avoid startup events
//    {
////      GlobalRouteManager::DeleteGlobalRoutes ();
////      GlobalRouteManager::BuildGlobalRoutingDatabase ();
////      GlobalRouteManager::InitializeRoutes ();
//    }
}

void
SatelliteGlobalRouting::NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address)
{
  NS_LOG_FUNCTION (this << interface << address);
//  if (m_respondToInterfaceEvents && Simulator::Now ().GetSeconds () > 0)  // avoid startup events
//    {
////      GlobalRouteManager::DeleteGlobalRoutes ();
////      GlobalRouteManager::BuildGlobalRoutingDatabase ();
////      GlobalRouteManager::InitializeRoutes ();
//    }
}

void
SatelliteGlobalRouting::SetIpv4 (Ptr<Ipv4> ipv4)
{
  NS_LOG_FUNCTION (this << ipv4);
  NS_ASSERT (m_ipv4 == 0 && ipv4 != 0);
  m_ipv4 = ipv4;
}

void
SatelliteGlobalRouting::PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit) const
{
	  NS_LOG_FUNCTION (this << stream);
	  std::ostream* os = stream->GetStream ();

	  *os << "Node: " << m_ipv4->GetObject<Node> ()->GetId ()
	      << ", Time: " << Now().As (unit)
	      << ", Local time: " << GetObject<Node> ()->GetLocalTime ().As (unit)
	      << ", Ipv4GlobalRouting table" << std::endl;

	  if (GetNRoutes () > 0)
	    {
	      *os << "Destination     Gateway         Genmask         Flags Metric Ref    Use Iface" << std::endl;
	      for (uint32_t j = 0; j < GetNRoutes (); j++)
	        {
	          std::ostringstream dest, gw, mask, flags;
	          Ipv4RoutingTableEntry route = GetRoute (j);
	          dest << route.GetDest ();
	          *os << std::setiosflags (std::ios::left) << std::setw (16) << dest.str ();
	          gw << route.GetGateway ();
	          *os << std::setiosflags (std::ios::left) << std::setw (16) << gw.str ();
	          mask << route.GetDestNetworkMask ();
	          *os << std::setiosflags (std::ios::left) << std::setw (16) << mask.str ();
	          flags << "U";
	          if (route.IsHost ())
	            {
	              flags << "H";
	            }
	          else if (route.IsGateway ())
	            {
	              flags << "G";
	            }
	          *os << std::setiosflags (std::ios::left) << std::setw (6) << flags.str ();
	          // Metric not implemented
	          *os << "-" << "      ";
	          // Ref ct not implemented
	          *os << "-" << "      ";
	          // Use not implemented
	          *os << "-" << "   ";
	          if (Names::FindName (m_ipv4->GetNetDevice (route.GetInterface ())) != "")
	            {
	              *os << Names::FindName (m_ipv4->GetNetDevice (route.GetInterface ()));
	            }
	          else
	            {
	              *os << route.GetInterface ();
	            }
	          *os << std::endl;
	        }
	    }
	  *os << std::endl;
}

uint32_t
SatelliteGlobalRouting::GetNRoutes (void) const
{
  NS_LOG_FUNCTION (this);
  return m_SatelliteRoutes.size();
}

Ipv4RoutingTableEntry *
SatelliteGlobalRouting::GetRoute (uint32_t index) const
{
  NS_LOG_FUNCTION (this << index);
  if (index < m_SatelliteRoutes.size ())
    {
      uint32_t tmp = 0;
      for (SatelliteRoutesCI i = m_SatelliteRoutes.begin ();
           i != m_SatelliteRoutes.end ();
           i++)
        {
          if (tmp  == index)
            {
              return *i;
            }
          tmp++;
        }
    }
  NS_ASSERT (false);
  // quiet compiler.
  return 0;
}

Ptr<Ipv4Route>
SatelliteGlobalRouting::LookupForwarding(Ipv4Address dest, Ptr< NetDevice > oif=0)
{
	  NS_LOG_FUNCTION (this << dest << oif);
	  NS_LOG_LOGIC ("Looking for route for destination " << dest);
	  Ptr<Ipv4Route> rtentry = 0;
	  // store all available routes that bring packets to their destination
	  typedef std::vector<Ipv4RoutingTableEntry*> RouteVec_t;
	  RouteVec_t allRoutes;

	  NS_LOG_LOGIC ("Number of m_hostRoutes = " << m_SatelliteRoutes.size ());
	  for (SatelliteRoutesCI i = m_SatelliteRoutes.begin ();
	       i != m_SatelliteRoutes.end ();
	       i++)
	    {
	      NS_ASSERT ((*i)->IsHost ());
	      if ((*i)->GetDest ().IsEqual (dest))
	        {
	          if (oif != 0)
	            {
	              if (oif != m_ipv4->GetNetDevice ((*i)->GetInterface ()))
	                {
	                  NS_LOG_LOGIC ("Not on requested interface, skipping");
	                  continue;
	                }
	            }
	          allRoutes.push_back (*i);
	          NS_LOG_LOGIC (allRoutes.size () << "Found global host route" << *i);
	        }
	    }
	  if (allRoutes.size () > 0 ) // if route(s) is found
	    {
	      // pick up one of the routes uniformly at random if random
	      // ECMP routing is enabled, or always select the first route
	      // consistently if random ECMP routing is disabled
	      uint32_t selectIndex=0;
//	      if (m_randomEcmpRouting)
//	        {
//	          selectIndex = m_rand->GetInteger (0, allRoutes.size ()-1);
//	        }
//	      else
//	        {
//	          selectIndex = 0;
//	        }
	      Ipv4RoutingTableEntry* route = allRoutes.at (selectIndex);
	      // create a Ipv4Route object from the selected routing table entry
	      rtentry = Create<Ipv4Route> ();
	      rtentry->SetDestination (route->GetDest ());
	      /// \todo handle multi-address case
	      rtentry->SetSource (m_ipv4->GetAddress (route->GetInterface (), 0).GetLocal ());
	      rtentry->SetGateway (route->GetGateway ());
	      uint32_t interfaceIdx = route->GetInterface ();
	      rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));
	      return rtentry;
	    }
	  else
	    {
	      return 0;
	    }
}

std::pair<uint32_t,uint32_t>
SatelliteGlobalRouting::GetLogicalSatelliteAddress(Ptr<Node> node)const
{

	return std::make_pair(0,0);
}

void
SatelliteGlobalRouting::GenerateRoutingTable(NodeContainer c)
{
	Ptr<Node> current_node=m_ipv4->GetObject<Node>();
//	std::pair<uint32_t,uint32_t> logical_satellite_address;
//	logical_satellite_address = GetLogicalSatelliteAddress(current_node);

	for(uint32_t i=0;i<c.GetN();i++)
	{
		if(c.Get(i)==current_node)continue;
		if(current_node->GetObject<MobilityModel>()->GetPosition().z<0.1)	//this node is ground station
		{
			Ptr<PointToPointNetDevice> d_l=current_node->GetDevice(1)->GetObject<PointToPointNetDevice>();
			Ptr<PointToPointChannel> c_l=d_l->GetChannel()->GetObject<PointToPointChannel>();
			Ptr<PointToPointNetDevice> d_a=(c_l->GetPointToPointDevice(0)==dl)?c_l->GetPointToPointDevice(1):c_l->GetPointToPointDevice(0);
			Ptr<Ipv4L3Protocol> ipv4_a=d_a->GetObject<Ipv4L3Protocol>();
			Ipv4InterfaceAddress interface_a=ipv4_a->GetAddress((ipv4_a->GetInterfaceForDevice(d_a)),0);
			Ipv4Address ipv4address_a=interface_a.GetLocal();
			m_SatelliteRoutes.push_back(&Ipv4RoutingTableEntry::CreateHostRouteTo(Ipv4Address("0.0.0.0"),ipv4address_a,m_ipv4->GetInterfaceForDevice(d_l)));
		}
		else
		{
			//this node is LEO satellite
			std::pair<uint32_t,uint32_t> current_logical_address = GetLogicalSatelliteAddress(current_node);
			std::pair<uint32_t,uint32_t> another_logical_address = GetLogicalSatelliteAddress(c.Get(i));
		}

	}
}


}
