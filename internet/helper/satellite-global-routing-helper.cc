/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008 INRIA
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */
#include "satellite-global-routing-helper.h"



namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatelliteGlobalRoutingHelper");

SatelliteGlobalRoutingHelper::SatelliteGlobalRoutingHelper ()
{
}

SatelliteGlobalRoutingHelper::SatelliteGlobalRoutingHelper (const SatelliteGlobalRoutingHelper &o)
{
}

SatelliteGlobalRoutingHelper*
SatelliteGlobalRoutingHelper::Copy (void) const
{
  return new SatelliteGlobalRoutingHelper (*this);
}

Ptr<Ipv4RoutingProtocol>
SatelliteGlobalRoutingHelper::Create (Ptr<Node> node) const
{
  NS_LOG_LOGIC ("Adding GlobalRouter interface to node " <<
                node->GetId ());

  //Ptr<GlobalRouter> globalRouter = CreateObject<GlobalRouter> ();
  //node->AggregateObject (globalRouter);

//  NS_LOG_LOGIC ("Adding GlobalRouting Protocol to node " << node->GetId ());
  Ptr<SatelliteGlobalRouting> SatelliteglobalRouting= CreateObject<SatelliteGlobalRouting> ();
//  globalRouter->SetRoutingProtocol (SatelliteglobalRouting);

  return SatelliteglobalRouting;
}


} // namespace ns3
