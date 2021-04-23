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
  * Author: Liu Shuyang <hello_world@sjtu.edu.cn>
  */

#include "ns3/abort.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/queue.h"
#include "ns3/net-device-queue-interface.h"
#include "ns3/config.h"
#include "ns3/packet.h"
#include "ns3/names.h"

#include "free-point-to-point-helper.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("FreePointToPointHelper");

FreePointToPointHelper::FreePointToPointHelper()
{
    ob_queue.SetTypeId("ns3::DropTailQueue<Packet>");
    ob_device.SetTypeId("ns3::PointToPointNetDevice");
}

NetDeviceContainer
FreePointToPointHelper::Install(NodeContainer c,const AttributeValue &datarate)
{
    uint32_t num = c.GetN();
    NetDeviceContainer container;
    for (uint32_t i = 0; i < num; i++)
    {

        Ptr<PointToPointNetDevice> dev = ob_device.Create<PointToPointNetDevice>();
        //ob_device.Set("DataRate",datarate);
        dev->SetAttribute("DataRate",datarate);
        dev->SetAddress(Mac48Address::Allocate());
        c.Get(i)->AddDevice(dev);
        Ptr<Queue<Packet> > queue = ob_queue.Create<Queue<Packet> >();
        dev->SetQueue(queue);
        Ptr<NetDeviceQueueInterface> ndqi = CreateObject<NetDeviceQueueInterface>();
        ndqi->GetTxQueue(0)->ConnectQueueTraces(queue);
        dev->AggregateObject(ndqi);
        container.Add(dev);

    }
    return container;
}


}
