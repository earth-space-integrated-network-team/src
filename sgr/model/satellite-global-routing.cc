#include <vector>
#include <iomanip>

#include "ns3/ipv4-routing-table-entry.h"

#include "ns3/satellite-global-routing.h"


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
  //NS_LOG_FUNCTION (this << p << header << header.GetSource () << header.GetDestination () << idev << &lcb << &ecb);
  // Check if input device supports IP
  //NS_ASSERT (m_ipv4->GetInterfaceForDevice (idev) >= 0);
  uint32_t iif = m_ipv4->GetInterfaceForDevice (idev);

  if (m_ipv4->IsDestinationAddress (header.GetDestination (), iif))
    {
      if (!lcb.IsNull ())
        {
//          NS_LOG_LOGIC ("Local delivery to " << header.GetDestination ());
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
//      NS_LOG_LOGIC ("Forwarding disabled for this interface");
      ecb (p, header, Socket::ERROR_NOROUTETOHOST);
      return true;
    }
  // Next, try to find a route
//  NS_LOG_LOGIC ("Unicast destination- looking up global route");
  Ptr<Ipv4Route> rtentry = LookupForwarding(header.GetDestination ());
  if (rtentry != 0)
    {
//      NS_LOG_LOGIC ("Found unicast destination- calling unicast callback");
      ucb (rtentry, p, header);
      return true;
    }
  else
    {
//      NS_LOG_LOGIC ("Did not find unicast destination- returning false");
      return false; // Let other routing protocols try to handle this
                    // route request.
    }
}

Ptr<Ipv4Route>
SatelliteGlobalRouting::RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr)
{
//  NS_LOG_FUNCTION (this << p << &header << oif << &sockerr);
//
// First, see if this is a multicast packet we have a route for.  If we
// have a route, then send the packet down each of the specified interfaces.
//
  if (header.GetDestination ().IsMulticast ())
    {
//      NS_LOG_LOGIC ("Multicast destination-- returning false");
      return 0; // Let other routing protocols try to handle this
    }
//
// See if this is a unicast packet we have a route for.
//
//  NS_LOG_LOGIC ("Unicast destination- looking up");
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
//  NS_LOG_FUNCTION (this << i);
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
//  NS_LOG_FUNCTION (this << i);
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
//  NS_LOG_FUNCTION (this << interface << address);
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
//  NS_LOG_FUNCTION (this << interface << address);
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
//  NS_LOG_FUNCTION (this << ipv4);
  NS_ASSERT (m_ipv4 == 0 && ipv4 != 0);
  m_ipv4 = ipv4;
}

void
SatelliteGlobalRouting::PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit) const
{
//	  NS_LOG_FUNCTION (this << stream);
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
//  NS_LOG_FUNCTION (this);
  return m_SatelliteRoutes.size();
}

Ipv4RoutingTableEntry *
SatelliteGlobalRouting::GetRoute (uint32_t index) const
{
//  NS_LOG_FUNCTION (this << index);
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
SatelliteGlobalRouting::LookupForwarding(Ipv4Address dest, Ptr< NetDevice > oif)
{
//	  NS_LOG_FUNCTION (this << dest << oif);
//	  NS_LOG_LOGIC ("Looking for route for destination " << dest);
	  Ptr<Ipv4Route> rtentry = 0;
	  // store all available routes that bring packets to their destination
	  typedef std::vector<Ipv4RoutingTableEntry*> RouteVec_t;
	  RouteVec_t allRoutes;

//	  NS_LOG_LOGIC ("Number of m_hostRoutes = " << m_SatelliteRoutes.size ());
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
//	                  NS_LOG_LOGIC ("Not on requested interface, skipping");
	                  continue;
	                }
	            }
	          allRoutes.push_back (*i);
//	          NS_LOG_LOGIC (allRoutes.size () << "Found global host route" << *i);
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

//std::pair<uint32_t,uint32_t>
//SatelliteGlobalRouting::GetLogicalSatelliteAddress(Ptr<Node> node)const
//{
//	double x=node->GetObject<MobilityModel>()->GetPosition().x;
//	double y=node->GetObject<MobilityModel>()->GetPosition().y;
//	double z=node->GetObject<MobilityModel>()->GetPosition().z;
//	if（z<0.1) return;
//	double current_time=Simulator::Now().GetSeconds();
//	double G = 6.673e-11; // gravitational constant [Nm^2/kg^2]
//	double M = 5.972e24; // mass of Earth [kg]
//	double altitude = z;
//	double R = 6378.1;
//	double derta_x=180*current_time*sqrt(G*M/pow(z+R,3))/3.1416;
//	double origin_x=x-derta_x;
//
//	return std::make_pair(0,0);
//}

std::vector<double>
SatelliteGlobalRouting::SatelliteRoutingDistance(std::vector<std::vector<std::pair<uint32_t,uint32_t>>> routing,double h,uint32_t num,uint32_t plane)const
{
	std::vector<double> distance_table;
	for(std::vector<std::vector<std::pair<uint32_t,uint32_t>>>::iterator it=routing.begin();it!=routing.end();it++)
	{
		double distance=0;
		for(std::vector<std::pair<uint32_t,uint32_t>>::iterator node=it->begin();node!=it->end()-1;node++)
		{
			distance=distance+DistanceBetweenSatellites(*node,*(node+1),h,num,plane);
		}
		distance_table.push_back(distance);
	}
	return distance_table;
}

double
SatelliteGlobalRouting::DistanceBetweenSatellites(std::pair<uint32_t,uint32_t> s1,std::pair<uint32_t,uint32_t> s2,double h,uint32_t num,uint32_t plane)const
{
	double current_time=Simulator::Now().GetSeconds();
	double R = 6378.1;
	double x_0 = 50;
	double x_1 = 60;
	double G = 6.673e-11; // gravitational constant [Nm^2/kg^2]
	double M = 5.972e24; // mass of Earth [kg]
	double distance;
	if (s1.first == s2.first)//同轨相邻卫星
		return sin(180 / num) * (R + h) * 2;
	else//异轨相邻卫星
	{
		double s1_x;
//		double s1_y;
		double s2_x;
//		double s2_y;

		while (current_time > 2 * 3.1416 * sqrt(pow((R + h)*1000, 3) / G / M))
			current_time -= 2 * 3.1416 * sqrt(pow((R + h)*1000, 3) / G / M);

		double derta_x = 180 * current_time * sqrt(G * M / pow((R + h)*1000, 3)) / 3.1416;
		double derta_y = 360 / plane;


		if (s1.first % 2 < 0.5)
			s1_x = x_0 - s1.second * 360 / num - derta_x;
		else
			s1_x = x_1 - s1.second * 360 / num - derta_x;
		if (s2.first % 2 < 0.5)
			s2_x = x_0 - s2.second * 360 / num - derta_x;
		else
			s2_x = x_1 - s2.second * 360 / num - derta_x;

		if (s1_x > 90 && s1_x <= 270)s1_x = 180 - s1_x;
		if (s2_x > 90 && s2_x <= 270)s2_x = 180 - s2_x;

		if (s1_x > -270 && s1_x < -90)s1_x = -(180 + s1_x);
		if (s2_x > -270 && s2_x < -90)s2_x = -(180 + s2_x);

		if (s1_x > 270)s1_x = -(360 - s1_x);
		if (s2_x > 270)s2_x = -(360 - s2_x);

		if (s1_x < -270)s1_x = s1_x + 360;
		if (s2_x < -270)s2_x = s2_x + 360;

		s1_x = (90 - s1_x) / 180 * 3.1416;
		s2_x = (90 - s2_x) / 180 * 3.1416;

		distance= sqrt(pow((R+h) * sin(s1_x) * cos(0) - (R+h) * sin(s2_x) * cos(derta_y), 2) +
				pow((R+h) * sin(s1_x) * sin(0) - (R+h) * sin(s2_x) * sin(derta_y), 2) +
				pow((R+h) * cos(s1_x) - (R+h) * cos(s2_x), 2));

		return distance;
	}
}

void
SatelliteGlobalRouting::GenerateSatelliteRoutingTable(NodeContainer c,std::vector<std::pair<uint32_t,uint32_t>> logical_address_table)
{
	Ptr<Node> current_node=m_ipv4->GetObject<Node>();
	double h=current_node->GetObject<MobilityModel>()->GetPosition().z;
	uint32_t num=(logical_address_table.end()-1)->second;
	uint32_t plane =(logical_address_table.end()-1)->first;
	std::pair<uint32_t,uint32_t> source_logical_address;
	std::pair<uint32_t,uint32_t> destination_logical_address;
//	std::pair<uint32_t,uint32_t> logical_satellite_address;
//	logical_satellite_address = GetLogicalSatelliteAddress(current_node);
	uint32_t i;
	enum HopDirection {POSITIVE,NEGATIVE,ZERO};
	std::pair<HopDirection,HopDirection> direction;
//	uint32_t num_satellite_per_plane = (logical_address_table.end()-1)->second()+1;
	std::vector<std::pair<uint32_t, uint32_t>>::iterator temp = logical_address_table.end() - 1;
	uint32_t num_satellite_per_plane = temp->second+1;

	for(i=0;i<c.GetN();i++)
	{
		if(c.Get(i)==current_node)
		{
			source_logical_address=logical_address_table[i];
			break;
		}
		else continue;
	}
	if(i==c.GetN())return;

	for(i=0;i<c.GetN();i++)
	{
		if(c.Get(i)==current_node) continue;
		destination_logical_address=logical_address_table[i];
		if(destination_logical_address.first>source_logical_address.first)
			direction.first=POSITIVE;
		else if(destination_logical_address.first<source_logical_address.first)
			direction.first=NEGATIVE;
		else direction.first=ZERO;

		if(destination_logical_address.second>source_logical_address.second)
			direction.second=POSITIVE;
		else if(destination_logical_address.second<source_logical_address.second)
			direction.second=NEGATIVE;
		else direction.second=ZERO;

		std::vector<std::vector<std::vector<std::pair<uint32_t,uint32_t>>>> path_table1;
		std::vector<std::vector<std::vector<std::pair<uint32_t,uint32_t>>>> path_table2;
		std::vector<std::vector<std::pair<uint32_t,uint32_t>>> routing;

		//path_table[i]表示从source到第i个节点的路径表
		//path_table[i][j]表示从source到第i个节点的第j条路径
		//path_table[i][j]储存从source到第i个节点的第j条路径上的节点逻辑地址
		//routing维护从source到destination的若干条路径

		uint32_t h_hop=std::max(destination_logical_address.first,source_logical_address.first)-
				       std::min(destination_logical_address.first,source_logical_address.first);
		uint32_t v_hop1=std::max(destination_logical_address.second,source_logical_address.second)-
				        std::min(destination_logical_address.second,source_logical_address.second);
		uint32_t v_hop2=(num_satellite_per_plane-
				        std::max(destination_logical_address.second,source_logical_address.second)-
						std::min(destination_logical_address.second,source_logical_address.second))%
						num_satellite_per_plane;

		for(uint32_t hh=0;hh<h_hop+1;hh++)
		{
			for(uint32_t vv=0;vv<v_hop1+1;vv++)
			{
				std::vector<std::pair<uint32_t,uint32_t>> a;
				std::vector<std::vector<std::pair<uint32_t,uint32_t>>> b;
				uint32_t h_logical,v_logical;
				if(hh==0&&vv==0)//无下、无左
				{

					a.push_back(source_logical_address);
					b.push_back(a);
					path_table1.push_back(b);
					continue;
				}
				if(vv==0)//无下，有左
				{
					if(direction.first==POSITIVE) h_logical=source_logical_address.first+hh;
					else h_logical=source_logical_address.first-hh;
					a=path_table1[(hh-1)*(v_hop1+1)][0];
					a.push_back(std::make_pair(h_logical,source_logical_address.second));
					b.push_back(a);
					path_table1.push_back(b);
					continue;
				}
				if(hh==0)//有下、无左
				{
					if(direction.second==POSITIVE) v_logical=source_logical_address.second+vv;
					else v_logical=source_logical_address.second-vv;
					a=(*(path_table1.end()-1))[0];
					a.push_back(std::make_pair(source_logical_address.first,v_logical));
					b.push_back(a);
					path_table1.push_back(b);
					continue;
				}
				//有下、有左
				if(direction.first==POSITIVE) h_logical=source_logical_address.first+hh;
				else h_logical=source_logical_address.first-hh;
				if(direction.second==POSITIVE) v_logical=source_logical_address.second+vv;
				else v_logical=source_logical_address.second-vv;
				for(std::vector<std::vector<std::pair<uint32_t,uint32_t>>>::iterator it=path_table1[(hh-1)*(v_hop1+1)+vv].begin();
						it!=path_table1[(hh-1)*(v_hop1+1)+vv].end();it++)
				{
					a=*it;
					a.push_back(std::make_pair(h_logical,v_logical));
					b.push_back(a);
				}

				for(std::vector<std::vector<std::pair<uint32_t,uint32_t>>>::iterator it=(path_table1.end()-1)->begin();
						it!=(path_table1.end()-1)->end();it++)
				{
					a=*it;
					a.push_back(std::make_pair(h_logical,v_logical));
					b.push_back(a);
				}
				path_table1.push_back(b);
			}
		}


		//环形拓扑

		for(uint32_t hh=0;hh<h_hop+1;hh++)
		{
			for(uint32_t vv=0;vv<v_hop2+1;vv++)
			{
				std::vector<std::pair<uint32_t,uint32_t>> a;
				std::vector<std::vector<std::pair<uint32_t,uint32_t>>> b;
				uint32_t h_logical,v_logical;
				if(hh==0&&vv==0)//无下、无左
				{

					a.push_back(source_logical_address);
					b.push_back(a);
					path_table2.push_back(b);
					continue;
				}
				if(vv==0)//无下，有左
				{
					if(direction.first==POSITIVE) h_logical=source_logical_address.first+hh;
					else h_logical=source_logical_address.first-hh;
					a=path_table2[(hh-1)*(v_hop2+1)][0];
					a.push_back(std::make_pair(h_logical,source_logical_address.second));
					b.push_back(a);
					path_table2.push_back(b);
					continue;
				}
				if(hh==0)//有下、无左
				{
					if(direction.second==NEGATIVE) v_logical=(source_logical_address.second+vv)%num_satellite_per_plane;
					else v_logical = (source_logical_address.second+num_satellite_per_plane - vv)%num_satellite_per_plane;
					a=(*(path_table2.end()-1))[0];
					a.push_back(std::make_pair(source_logical_address.first,v_logical));
					b.push_back(a);
					path_table2.push_back(b);
					continue;
				}
				//有下、有左
				if(direction.first==POSITIVE) h_logical=source_logical_address.first+hh;
				else h_logical=source_logical_address.first-hh;
				if(direction.second==NEGATIVE) v_logical=(source_logical_address.second+vv)%num_satellite_per_plane;
				else v_logical=(source_logical_address.second+num_satellite_per_plane - vv)%num_satellite_per_plane;;
				for(std::vector<std::vector<std::pair<uint32_t,uint32_t>>>::iterator it=path_table2[(hh-1)*(v_hop2+1)+vv].begin();
						it!=path_table2[(hh-1)*(v_hop2+1)+vv].end();it++)
				{
					a=*it;
					a.push_back(std::make_pair(h_logical,v_logical));
					b.push_back(a);
				}

				for(std::vector<std::vector<std::pair<uint32_t,uint32_t>>>::iterator it=(path_table2.end()-1)->begin();
						it!=(path_table2.end()-1)->end();it++)
				{
					a=*it;
					a.push_back(std::make_pair(h_logical,v_logical));
					b.push_back(a);
				}
				path_table2.push_back(b);
			}
		}

		for(std::vector<std::vector<std::pair<uint32_t,uint32_t>>>::iterator it=(path_table1.end()-1)->begin();
			it!=(path_table1.end()-1)->end();it++)
		{
			routing.push_back(*it);
		}
		for(std::vector<std::vector<std::pair<uint32_t,uint32_t>>>::iterator it=(path_table2.end()-1)->begin();
			it!=(path_table2.end()-1)->end();it++)
		{
			routing.push_back(*it);
		}

		std::vector<double> distance_table=SatelliteRoutingDistance(routing,h,num,plane);

		//最短路径排序
		for(uint32_t jj=0;jj<distance_table.size()-1;jj++)
		{
			for(uint32_t kk=0;kk<distance_table.size()-1-jj;kk++)
			{
				if(distance_table[kk]>distance_table[kk+1])
				{
					std::swap(distance_table[kk],distance_table[kk+1]);
					std::swap(routing[kk],routing[kk+1]);
				}
			}
		}

		if(distance_table[0]>0.5)
		{
			std::pair<uint32_t,uint32_t> gateway_logical = routing[0][1];
			uint32_t aa=0;
			for(std::vector<std::pair<uint32_t,uint32_t>>::iterator it=logical_address_table.begin();it!=logical_address_table.end();it++)
			{
				if(gateway_logical.first==it->first&&gateway_logical.second==it->second)
					break;
				aa+=1;
			}
			if(aa==c.GetN())return;
			Ptr<Node> node_a=c.Get(aa);
			for(aa=0;aa<current_node->GetNDevices();aa++)
			{
				Ptr<PointToPointNetDevice> d_l=current_node->GetDevice(aa)->GetObject<PointToPointNetDevice>();
				Ptr<PointToPointChannel> c_l=d_l->GetChannel()->GetObject<PointToPointChannel>();
				Ptr<PointToPointNetDevice> d_a=(c_l->GetPointToPointDevice(0)==d_l)?c_l->GetPointToPointDevice(1):c_l->GetPointToPointDevice(0);
				if(d_a->GetNode()==node_a)
				{
					Ptr<Ipv4L3Protocol> ipv4_a=d_a->GetObject<Ipv4L3Protocol>();
					Ipv4InterfaceAddress interface_a=ipv4_a->GetAddress((ipv4_a->GetInterfaceForDevice(d_a)),0);
					Ipv4Address ipv4address_a=interface_a.GetLocal();
					Ptr<Node> DesNode=c.Get(i);
					uint32_t des_ip_num=DesNode->GetObject<Ipv4L3Protocol>()->GetNInterfaces();
					for(uint32_t ind=0;ind<des_ip_num;ind++)
					{
						Ipv4Address DesIp=DesNode->GetObject<Ipv4L3Protocol>()->GetInterface(ind)->GetAddress(0).GetLocal();
						Ipv4RoutingTableEntry *rou=new Ipv4RoutingTableEntry (Ipv4RoutingTableEntry::CreateHostRouteTo(DesIp,ipv4address_a,m_ipv4->GetInterfaceForDevice(d_l)));

						m_SatelliteRoutes.push_back(rou);
					}
				}
				else continue;
			}
		}
		//m_SatelliteRoutes.push_back(&Ipv4RoutingTableEntry::CreateHostRouteTo(Ipv4Address("0.0.0.0"),ipv4address_a,m_ipv4->GetInterfaceForDevice(d_l)));
	}
}
void
SatelliteGlobalRouting::GenerateSatToGroRoutingTable(NodeContainer c)
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
			Ptr<PointToPointNetDevice> d_a=(c_l->GetPointToPointDevice(0)==d_l)?c_l->GetPointToPointDevice(1):c_l->GetPointToPointDevice(0);
			Ptr<Ipv4L3Protocol> ipv4_a=d_a->GetObject<Ipv4L3Protocol>();
			Ipv4InterfaceAddress interface_a=ipv4_a->GetAddress((ipv4_a->GetInterfaceForDevice(d_a)),0);
			Ipv4Address ipv4address_a=interface_a.GetLocal();
			Ipv4RoutingTableEntry *rou=new Ipv4RoutingTableEntry (Ipv4RoutingTableEntry::CreateHostRouteTo(Ipv4Address("0.0.0.0"),ipv4address_a,m_ipv4->GetInterfaceForDevice(d_l)));

			m_SatelliteRoutes.push_back(rou);
		}
		else
		{
			//this node is LEO satellite
			//std::pair<uint32_t,uint32_t> current_logical_address = GetLogicalSatelliteAddress(current_node);
			//std::pair<uint32_t,uint32_t> another_logical_address = GetLogicalSatelliteAddress(c.Get(i));
		}

	}

}


}
