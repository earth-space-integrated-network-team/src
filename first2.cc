/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("FirstScriptExample");



int
main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);
  
  Time::SetResolution (Time::NS);
  LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
  LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);

  NodeContainer nodes;
  NodeContainer mynodes;
  nodes.Create (2);
  mynodes.Create(2);





  //Ptr<PointToPointNetDevice> Free_PointToPointNetDevice(std::string datarate)
  //{
  	  std::string datarate = "5Mbps";
  	  std::string delay = "2ms";

  	 ObjectFactory ob_queue;
  	 ObjectFactory ob_device;
  	 ob_queue.SetTypeId("ns3::DropTailQueue<Packet>");
  	 ob_device.SetTypeId("ns3::PointToPointNetDevice");
  	 ob_device.Set("DataRate",StringValue (datarate));
  	 Ptr<PointToPointNetDevice> device1 = ob_device.Create<PointToPointNetDevice>();
  	 device1->SetAddress(Mac48Address::Allocate ());
  	 Ptr<Queue<Packet>> device_queue1 = ob_queue.Create<Queue<Packet>>();
  	 device1->SetQueue(device_queue1);
  	 Ptr<NetDeviceQueueInterface> device_queue_interface1 = CreateObject<NetDeviceQueueInterface> ();
  	 device_queue_interface1->GetTxQueue (0)->ConnectQueueTraces (device_queue1);
  	 device1->AggregateObject(device_queue_interface1);

  //	 return device;
  //}



//  	ObjectFactory ob_queue2;
//  	  	 ObjectFactory ob_device2;
//  	  	 ob_queue2.SetTypeId("ns3::DropTailQueue<Packet>");
//  	  	 ob_device2.SetTypeId("ns3::PointToPointNetDevice");
//  	  	 ob_device2.Set("DataRate",StringValue (datarate));
  	  	 Ptr<PointToPointNetDevice> device2 = ob_device.Create<PointToPointNetDevice>();
  	  	 device2->SetAddress(Mac48Address::Allocate ());
  	  	 Ptr<Queue<Packet>> device_queue2 = ob_queue.Create<Queue<Packet>>();
  	  	 device2->SetQueue(device_queue2);
  	  	 Ptr<NetDeviceQueueInterface> device_queue_interface2 = CreateObject<NetDeviceQueueInterface> ();
  	  	 device_queue_interface2->GetTxQueue (0)->ConnectQueueTraces (device_queue2);
  	  	 device2->AggregateObject(device_queue_interface2);

  //Ptr<PointToPointChannel> Free_PointToPointChannel(std::string delay)
  //{
  	ObjectFactory ob_channel0;
  	ob_channel0.SetTypeId("ns3::PointToPointChannel");
  	ob_channel0.Set("Delay",StringValue(delay));
  	Ptr<PointToPointChannel> p2pchannel = ob_channel0.Create<PointToPointChannel> ();
  //	return channel;
  //}




//  Ptr<PointToPointNetDevice> a2;
//
//  PointToPointNetDevice b1;
//  PointToPointNetDevice b2;
//  a1=&b1;
//  a2=&b2;
//  std::cout<<a1->IsInitialized()<<"546456465465465465465465465465465465465"<<std::endl;
//
//  DataRate a_data("5Mbps");
//
//  a1->SetDataRate(a_data);
//  a2->SetDataRate(a_data);
//  Ptr<PointToPointNetDevice> a1;
//  a1=Free_PointToPointNetDevice("5Mbps");
//  Ptr<PointToPointNetDevice> a2;
//  a2=Free_PointToPointNetDevice("5Mbps");


  nodes.Get(0)->AddDevice(device1);
  nodes.Get(1)->AddDevice(device2);

//  Ptr<PointToPointChannel> p2pchannel;
//  p2pchannel = Free_PointToPointChannel("2ms");
//  PointToPointChannel channel;
//  p2pchannel=&channel;
//  Time delay1("2ms");
//
//  p2pchannel->SetDelay(delay1);
//
//  p2pchannel->Detach();
//  p2pchannel->Attach(device1);
//  p2pchannel->Attach(device2);
  device1->Attach(p2pchannel);
  device2->Attach(p2pchannel);
  NetDeviceContainer devices;
  devices.Add(device1);
  devices.Add(device2);




//  PointToPointHelper pointToPoint;
//  PointToPointHelper mychannel;
//  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
//  pointToPoint.SetChannelAttribute ("Delay", StringValue ("2ms"));
//
//  mychannel.SetDeviceAttribute("DataRate",StringValue("5Mbps"));
//  mychannel.SetChannelAttribute("Delay",StringValue("2ms"));
//
//
//  NetDeviceContainer devices;
//  NetDeviceContainer mydevices;
//  devices = pointToPoint.Install (nodes);
//  mydevices = mychannel.Install(mynodes);
//
//  Ptr<PointToPointChannel> p2pchannel;
//  p2pchannel=mydevices.Get(0)->GetChannel()->GetObject<PointToPointChannel>();
//
//  p2pchannel->Detach();
//  Time delay("2s");
//  p2pchannel->SetDelay(delay);
//
//  p2pchannel->Attach(devices.Get(0)->GetObject<PointToPointNetDevice>());
//  p2pchannel->Attach(devices.Get(1)->GetObject<PointToPointNetDevice>());
//



  InternetStackHelper stack;
  stack.Install (nodes);


  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");

  Ipv4InterfaceContainer interfaces = address.Assign (devices);

  std::cout<<interfaces.GetN()<<"/////////////////////////"<<std::endl;


  for(uint32_t jj=0;jj<2;jj++)
  {
	  std::cout<<nodes.Get(0)->GetDevice(jj)->GetAddress()<<"*******************"<<std::endl;
  }

  UdpEchoServerHelper echoServer (9);

  ApplicationContainer serverApps = echoServer.Install (nodes.Get (1));
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (10.0));

  UdpEchoClientHelper echoClient (interfaces.GetAddress (1), 9);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (1));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (1.0)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer clientApps = echoClient.Install (nodes.Get (0));
  clientApps.Start (Seconds (2.0));
  clientApps.Stop (Seconds (10.0));

  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
