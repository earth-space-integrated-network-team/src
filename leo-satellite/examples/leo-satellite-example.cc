/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * LEO Satellite Example
 * Runs traffic through a configurable LEO satellite constellation
 *
 * ENSC 427: Communication Networks
 * Spring 2020
 * Team 11
 */

#include "ns3/core-module.h"
#include "ns3/leo-satellite-config.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-helper.h"
#include <string>
#include <sstream>
#include "ns3/flow-monitor-module.h"//swd

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LeoSatelliteExample");

int 
main (int argc, char *argv[])
{
  uint32_t n_planes = 3;			//jishu
  uint32_t n_sats_per_plane = 4;	//oushu
  double altitude = 2000;

  CommandLine cmd;
  cmd.AddValue ("n_planes", "Number of planes in satellite constellation", n_planes);
  cmd.AddValue ("n_sats_per_plane", "Number of satellites per plane in the satellite constellation", n_sats_per_plane);
  cmd.AddValue ("altitude", "Altitude of satellites in constellation in kilometers ... must be between 500 and 2000", altitude);

  cmd.Parse (argc,argv);

  LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
  LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

  LeoSatelliteConfig sat_network(n_planes, n_sats_per_plane, altitude);
  

  UdpEchoServerHelper echoServer (9);

  ApplicationContainer serverApps = echoServer.Install(sat_network.ground_stations.Get(1));
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (10.0));

  UdpEchoClientHelper echoClient (sat_network.ground_station_interfaces[1].GetAddress(0), 9);
  echoClient.SetAttribute("MaxPackets", UintegerValue (20));
  echoClient.SetAttribute("Interval", TimeValue(Seconds(100.0)));
  echoClient.SetAttribute("PacketSize", UintegerValue (1024));

  ApplicationContainer clientApps = echoClient.Install (sat_network.ground_stations.Get(0));
  clientApps.Start (Seconds (2.0));
  clientApps.Stop (Seconds (10.0));

  FlowMonitorHelper flowmonHelper;
  flowmonHelper.InstallAll();


  //ban the update function, just for static topology

  /* for(uint32_t i=0; i<19; i++)
  {
    Simulator::Stop(Seconds(100));
    Simulator::Run();
    sat_network.UpdateLinks();
  }



  uint16_t port = 9;   // Discard port (RFC 863)

  //  OnOffHelper onoff1 ("ns3::UdpSocketFactory",
  //                     InetSocketAddress (sat_network.ground_station_interfaces[1].GetAddress (0), port));//satellite(1,0)
  //  onoff1.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=0.001]"));
  //  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ExponentialRandomVariable[Mean=0.001|Bound=1]"));
  //
  //  ApplicationContainer onOffApp1 = onoff1.Install (sat_network.ground_stations.Get(0));//satellite(1,3)
  //  onOffApp1.Start (Seconds (1.0));
  //  onOffApp1.Stop (Seconds (20.0));

    // Create a similar flow from n3 to n1, starting at time 1.1 seconds
    OnOffHelper onoff2 ("ns3::UdpSocketFactory",
                       InetSocketAddress (sat_network.intra_plane_interfaces[9].GetAddress (0), port));//satellite(0,3)
    onoff2.SetConstantRate (DataRate ("448kb/s"));
    onoff2.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=0.001]"));
    onoff2.SetAttribute ("OffTime", StringValue ("ns3::ExponentialRandomVariable[Mean=0.001|Bound=1]"));

    ApplicationContainer onOffApp2 = onoff2.Install (sat_network.plane[0].Get (0));//satellite(0,0)
    onOffApp2.Start (Seconds (2.1));
    onOffApp2.Stop (Seconds (900.0));//swd

    Ptr<FlowMonitor> flowmon;//swd
      FlowMonitorHelper flowmonHelper;
      flowmon = flowmonHelper.InstallAll ();
      */

  Simulator::Stop(Seconds(10));
  Simulator::Run();

  /*flowmon->CheckForLostPackets ();
       	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmonHelper.GetClassifier ());
               std::map<FlowId, FlowMonitor::FlowStats> stats = flowmon->GetFlowStats ();

               uint32_t txPacketsum = 0;
               uint32_t rxPacketsum = 0;
               uint32_t DropPacketsum = 0;
               uint32_t LostPacketsum = 0;
               double Delaysum = 0;
               //double Ovhd=0;

               for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i =stats.begin (); i != stats.end (); ++i)
               {
                 //NS_LOG_UNCOND(i->second.txPackets);
               	Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
               	{
                  txPacketsum += i->second.txPackets;
                  rxPacketsum += i->second.rxPackets;
                  LostPacketsum += i->second.lostPackets;
                  DropPacketsum += i->second.packetsDropped.size();
                  Delaysum += i->second.delaySum.GetSeconds();
                  double pdr=double(i->second.rxPackets)/i->second.txPackets;
                  std::cout<<t.sourceAddress<<"---"<<t.destinationAddress<<" :"<<pdr<<","<<i->second.rxPackets
                 		 <<","<<i->second.txPackets<<std::endl;
                 }
               }
               std ::cout<< "lost packet:"<<LostPacketsum<<std::endl;
               std ::cout<< "drop packet:"<<DropPacketsum<<std::endl;
               std ::cout<< "txPacketsum:"<<txPacketsum<<std::endl;
               std ::cout<< "rxPacketsum:"<<rxPacketsum<<std::endl;
               std ::cout<< "pdr"<<double(rxPacketsum)/txPacketsum<<std::endl;
               std::cout<<"delay"<<Delaysum/double(rxPacketsum)<<std::endl;
               */
 
  Simulator::Destroy ();

  std::stringstream ss;
  std::string file_title;
  ss << "leo-satellite-example-"<<n_planes<<"-"<<n_sats_per_plane<<"-"<<altitude<<".flowmon";
  ss >> file_title;

  flowmonHelper.SerializeToXmlFile (file_title, false, false);
  return 0; 
}


