#ifndef SATELLITE_GLOBAL_ROUTING_H
#define SATELLITE_GLOBAL_ROUTING_H

#include <list>
#include <stdint.h>
#include "ns3/ipv4-address.h"
#include "ns3/ipv4-header.h"
#include "ns3/ptr.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/random-variable-stream.h"
#include "ns3/node-container.h"
#include "ns3/leo-satellite-mobility.h"
#include "ns3/point-to-point-module.h"
#include <typeinfo>
#include "ns3/core-module.h"
#include <cmath>

namespace ns3{

class Packet;
class NetDevice;
class Ipv4Interface;
class Ipv4Address;
class Ipv4Header;
class Ipv4RoutingTableEntry;
class Ipv4MulticastRoutingTableEntry;
class Node;

class SatelliteGlobalRouting : public Ipv4RoutingProtocol
{
public:
	static TypeId GetTypeId (void);
	SatelliteGlobalRouting();
	virtual ~SatelliteGlobalRouting();
	virtual Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr);

	virtual bool RouteInput  (Ptr<const Packet> p, const Ipv4Header &header, Ptr<const NetDevice> idev,
	                          UnicastForwardCallback ucb, MulticastForwardCallback mcb,
	                          LocalDeliverCallback lcb, ErrorCallback ecb);
	virtual void NotifyInterfaceUp (uint32_t interface);
	virtual void NotifyInterfaceDown (uint32_t interface);
	virtual void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address);
	virtual void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address);
	virtual void SetIpv4 (Ptr<Ipv4> ipv4);
	virtual void PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit = Time::S) const;
private:
	Ptr<Ipv4> m_ipv4;
	Ptr<Ipv4Route> LookupForwarding(Ipv4Address dest, Ptr< NetDevice > oif=0);

	typedef std::list<Ipv4RoutingTableEntry *> SatelliteRoutes;
	SatelliteRoutes m_SatelliteRoutes;

	typedef std::list<Ipv4RoutingTableEntry *>::const_iterator SatelliteRoutesCI;

	std::pair<uint32_t,uint32_t> GetLogicalSatelliteAddress(Ptr<Node> node)const;
	std::vector<double> SatelliteRoutingDistance(std::vector<std::vector<std::pair<uint32_t,uint32_t>>> routing,NodeContainer c)const;
	double DistanceBetweenSatellites(std::pair<uint32_t,uint32_t> s1,std::pair<uint32_t,uint32_t> s2,double h,uint32_t num,uint32_t plane)const;


public:
	uint32_t GetNRoutes (void) const;
	Ipv4RoutingTableEntry *GetRoute (uint32_t i) const;
	void GenerateSatelliteRoutingTable(NodeContainer c,std::vector<std::pair<uint32_t,uint32_t>> logical_address_table);
	void GenerateSatToGroRoutingTable(NodeContainer c);
};






}
