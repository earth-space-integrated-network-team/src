/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * LEO Satellite Constellation Config
 * Creates and maintains all satellites and links within a satellite communication network
 *
 * ENSC 427: Communication Networks
 * Spring 2020
 * Team 11
 */

#include "leo-satellite-config.h"
#include <typeinfo>
#include "ns3/net-device-container.h"
#include <cmath>
#include "ns3/free-point-to-point-helper.h"



namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (LeoSatelliteConfig);
NS_LOG_COMPONENT_DEFINE ("LeoSatelliteConfig");

typedef std::vector<Ipv4RoutingTableEntry *> SatelliteRoutes;
SatelliteRoutes GenerateSatelliteRoutingTable(SatelliteRoutes m_SatelliteRoutes,Ptr<Node> node,NodeContainer c,
											  std::vector<std::pair<uint32_t, uint32_t>> logical_address_table);//,
//											  std::vector<std::pair<std::pair<uint32_t,uint32_t>,std::vector<uint32_t>>> satellite_load_table);
//SatelliteRoutes GenerateSatelliteRoutingTable(SatelliteRoutes m_SatelliteRoutes,Ptr<Ipv4>m_ipv4,NodeContainer c, std::vector<std::pair<uint32_t, uint32_t>> logical_address_table);


//extern double CalculateDistanceGroundToSat (const Vector &a, const Vector &b);

double speed_of_light = 299792458; //in m/s

//typeid
TypeId LeoSatelliteConfig::GetTypeId (void)			///asdfasdfasdf
{
  static TypeId tid = TypeId ("ns3::LeoSatelliteConfig")  //[][[p[p;
  .SetParent<Object> ()
  .SetGroupName("LeoSatellite")		//asdfsadf
  ;
  return tid;
}

LeoSatelliteConfig::~LeoSatelliteConfig ()
{
}

TypeId LeoSatelliteConfig::GetInstanceTypeId (void) const
{
  TypeId tid = this->GetTypeId();
  return tid;
}

//constructor

LeoSatelliteConfig::LeoSatelliteConfig (uint32_t num_planes, uint32_t num_satellites_per_plane, double altitude)
{

  //pr = 0;//pr = 0 : no satellite in the polar region ; pr = 1 : more than one satellite in the polar region
  this->num_planes = num_planes;
  this->num_satellites_per_plane = num_satellites_per_plane;
  this->m_altitude = altitude;


  uint32_t total_num_satellites = num_planes*num_satellites_per_plane;
  NodeContainer temp;
  temp.Create(total_num_satellites);


  //assign mobility model to all satellites
  MobilityHelper mobility;

  mobility.SetMobilityModel ("ns3::LeoSatelliteMobilityModel",
                             "NPerPlane", IntegerValue (num_satellites_per_plane),
                             "NumberofPlanes", IntegerValue (num_planes),
                             "Altitude", DoubleValue(altitude),
                             "Time", DoubleValue(Simulator::Now().GetSeconds()));

  mobility.Install(temp);
  
  for (NodeContainer::Iterator j = temp.Begin ();
       j != temp.End (); ++j)
    {
      Ptr<Node> object = *j;
      Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      Vector null = Vector(0.0, 0.0, 0.0);
      position->SetPosition(null); // needs to be done to initialize
      NS_ASSERT (position != 0);
    }

  //assigning nodes to e/ plane's node container as necessary
  for (uint32_t i=0; i<num_planes; i++)
  {
     NodeContainer temp_plane;

     for(uint32_t j=0; j<num_satellites_per_plane/2; j++)
     {
       Vector pos = temp.Get(i*num_satellites_per_plane/2 + j)->GetObject<MobilityModel> ()->GetPosition();
       std::cout << Simulator::Now().GetSeconds() << ": plane # "<< i << " node # " <<j<< ": x = " << pos.x << ", y = " << pos.y << ", z = " << pos.z << std::endl;
       temp_plane.Add(temp.Get(i*num_satellites_per_plane/2 + j));

     }
     for(uint32_t j=num_satellites_per_plane/2; j> 0; j--) //?????????
     {
       Vector pos = temp.Get(total_num_satellites/2 + i*num_satellites_per_plane/2 + j - 1)->GetObject<MobilityModel> ()->GetPosition();
       std::cout << Simulator::Now().GetSeconds() << ": plane # "<< i << " node # " <<num_satellites_per_plane - j<< ": x = " << pos.x << ", y = " << pos.y << ", z = " << pos.z << std::endl;
       temp_plane.Add(temp.Get(total_num_satellites/2 + i*num_satellites_per_plane/2 + j - 1));
     }



     InternetStackHelper stack;
//     AodvHelper aodv;

//     stack.SetRoutingHelper(aodv);

     stack.Install(temp_plane);
     //std::cout<<"---------------------"<< temp_plane.Get(0)->GetNDevices()<<std::endl;
     this->plane.push_back(temp_plane);

  }

  //setting up all intraplane links
  //以轨道面0的1,2卫星为初始节点
  Vector nodeAPosition = this->plane[0].Get(0)->GetObject<MobilityModel>()->GetPosition();
  Vector nodeBPosition = this->plane[0].Get(1)->GetObject<MobilityModel>()->GetPosition();
  //计算出这两颗卫星之间距离和传播时延
  double distance = CalculateDistance(nodeAPosition, nodeBPosition);
  double delay = (distance * 1000)/speed_of_light; //should get delay in seconds
  //构造点对点信道，以方便接下来适配于所有轨道内节点间上
  PointToPointHelper intraplane_link_helper;
  //
  //
  //intraplane_link_helper.EnablePcapAll("intraplane_link");
  //
  //
  intraplane_link_helper.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  intraplane_link_helper.SetChannelAttribute ("Delay", TimeValue(Seconds (delay)));

  std::cout<<"Setting up intra-plane links with distance of "<<distance<<" km and delay of "<<delay<<" seconds."<<std::endl;

  //遍历所有轨道所有节点，以为所有轨道内，节点配置p2p信道

  for (uint32_t i=0; i<num_planes; i++)
  {
    for (uint32_t j=0; j<num_satellites_per_plane; j++)
    {

    	//(0,0)->(0,1)->(0,2)->(0,...)->(0,num_satellites_per_plane-1)->(0,0)
    	//(...,0)->(...,1)->(...,2)->(...,...)->(...,num_satellites_per_plane-1)->(...,0)
    	//(num_panes-1,0)->(num_panes-1,1)->(num_panes-1,2)->(num_panes-1,...)->(num_panes-1,num_satellites_per_plane-1)->(0,0)
      this->intra_plane_devices.push_back(intraplane_link_helper.Install(plane[i].Get(j), plane[i].Get((j+1)%num_satellites_per_plane)));

      std::cout<<"Plane "<<i<<": channel between node "<<j<<" and node "<<(j+1)%num_satellites_per_plane<<std::endl;
      std::cout<<"With Position of" <<std::endl;
      std::cout<<"("<<i<<","<<j<<"): "<<"x:"<<plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition().x
    		                          <<"	y:"<<plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition().y
									  <<"	z:"<<plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition().z
									  <<std::endl;
      std::cout<<"("<<i<<","<<(j+1)%num_satellites_per_plane<<"): "<<"x:"<<plane[i].Get((j+1)%num_satellites_per_plane)->GetObject<MobilityModel>()->GetPosition().x
        		                      <<"	y:"<<plane[i].Get((j+1)%num_satellites_per_plane)->GetObject<MobilityModel>()->GetPosition().y
    							      <<"	z:"<<plane[i].Get((j+1)%num_satellites_per_plane)->GetObject<MobilityModel>()->GetPosition().z
    								  <<std::endl;

    }
  }

	 // std::cout<<intra_plane_devices.size()<<"********************************"<<std::endl;

  //setting up interplane links except for reverse seams
  std::cout<<"Setting up inter-plane links"<<std::endl;
  for (uint32_t i=0; i<num_planes-1; i++)
  {
    for (uint32_t j=0; j<num_satellites_per_plane; j++)
    {
    	//(0,0)->(1,0)	(0,1)->(1,1) ... (0,num_satellites_per_plane-1)->(1,num_satellites_per_plane-1)
    	//|
    	//(n0,0)->(n0+1,0)	(n0,1)->(n0+1,1) ... (n0,num_satellites_per_plane-1)->(n0+1,num_satellites_per_plane-1)
    	//|
    	//(num_planes-2,0)->(num_planes-1,0)	(num_planes-2,1)->(num_planes-1,1) ... (num_planes-2,num_satellites_per_plane-1)->(num_planes-1,num_satellites_per_plane-1)

      uint32_t nodeBIndex; //ID of adjacent satellite
      (i == num_planes - 1) ? nodeBIndex = num_satellites_per_plane - j - 1: nodeBIndex = j;
      Vector nodeAPos = this->plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition();
      Vector nodeBPos = this->plane[i+1].Get(j)->GetObject<MobilityModel>()->GetPosition();
      double distance = CalculateDistance(nodeAPos, nodeBPos);
      double delay = (distance*1000)/speed_of_light;

      PointToPointHelper interplane_link_helper;
      PointToPointHelper badlink;


     // interplane_link_helper.EnablePcapAll("interplane_link");

      interplane_link_helper.SetDeviceAttribute("DataRate", StringValue ("5Mbps"));
      interplane_link_helper.SetChannelAttribute("Delay", TimeValue(Seconds(delay)));

      badlink.SetDeviceAttribute("DataRate", StringValue ("1bps"));
      badlink.SetChannelAttribute("Delay", TimeValue(Seconds(2000)));


      NodeContainer temp_node_container;
      temp_node_container.Add(this->plane[i].Get(j));
      temp_node_container.Add(this->plane[i+1].Get(j));//?????????

      NetDeviceContainer temp_netdevice_container;

    	  temp_netdevice_container = interplane_link_helper.Install(temp_node_container.Get(0),temp_node_container.Get(1));
    	  std::cout<<"Channel open between plane "<<i<<" satellite "<<j<<" and plane "<<(i+1)%num_planes<<" satellite "<<nodeBIndex<< " with distance "<<distance<< "km and delay of "<<delay<<" seconds"<<std::endl;
          std::cout<<"With Position of" <<std::endl;
          std::cout<<"("<<i<<","<<j<<"): "<<"x:"<<plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition().x
        		                          <<"	y:"<<plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition().y
    									  <<"	z:"<<plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition().z
    									  <<std::endl;
          std::cout<<"("<<i+1<<","<<j<<"): "<<"x:"<<plane[i+1].Get(j)->GetObject<MobilityModel>()->GetPosition().x
            		                      <<"	y:"<<plane[i+1].Get(j)->GetObject<MobilityModel>()->GetPosition().y
        							      <<"	z:"<<plane[i+1].Get(j)->GetObject<MobilityModel>()->GetPosition().z
        								  <<std::endl;
//      }

      Ptr<PointToPointChannel> p2p_channel;
      Ptr<Channel> channel;
      channel = temp_netdevice_container.Get(0)->GetChannel();
      channel = temp_netdevice_container.Get(1)->GetChannel();

      p2p_channel = channel->GetObject<PointToPointChannel> ();

      this->inter_plane_devices.push_back(temp_netdevice_container);
      this->inter_plane_channels.push_back(p2p_channel);
      this->inter_plane_channel_tracker.push_back(nodeBIndex);
    }
    //std::cout<<inter_plane_devices.size()<<"***********************************"<<std::endl;
  }

  //setting up two ground stations for now
  std::cout << "Setting up two ground stations" << std::endl;
  ground_stations.Create(2);
  //assign mobility model to ground stations
  MobilityHelper groundMobility;
  groundMobility.SetMobilityModel ("ns3::GroundStationMobilityModel",
                             "NPerPlane", IntegerValue (num_satellites_per_plane),
                             "NumberofPlanes", IntegerValue (num_planes),
							 "Time", DoubleValue(Simulator::Now().GetSeconds()));

  groundMobility.Install(ground_stations);

  //Install IP stack
  InternetStackHelper stack;
  //AodvHelper aodv;

  //stack.SetRoutingHelper(aodv);

  stack.Install(ground_stations);
  for (int j = 0; j<2; j++)
  {
    Vector temp = ground_stations.Get(j)->GetObject<MobilityModel> ()->GetPosition();
    std::cout << "Current Time: " << Simulator::Now().GetSeconds() << ": ground station # " << j << ": x = " << temp.x << ", y = " << temp.y <<std::endl;
  }

  NodeContainer all_nodes;
  all_nodes.Add(ground_stations.Get(0));
  all_nodes.Add(ground_stations.Get(1));
  for(uint32_t i=0;i<num_planes;i++)
  {
	  for(uint32_t j=0;j<num_satellites_per_plane;j++)
	  {
		  all_nodes.Add(plane[i].Get(j));
	  }
  }
  FreePointToPointHelper p2p_for_all_nodes;
  NetDeviceContainer all_nodes_p2p_devices;
  all_nodes_p2p_devices = p2p_for_all_nodes.Install(all_nodes,StringValue("5Mbps"));

  for(uint32_t kk =0;kk<all_nodes_p2p_devices.GetN();kk++)
  {
	  all_nodes_p2p_devices.Get(kk)->GetObject<PointToPointNetDevice>()->Detach();
  }

  //setting up links between ground stations and their closest satellites

  for(uint32_t kk=0;kk<2;kk++)
  {
	  double distance;
	  double min_distance=99999.0;
	  uint32_t min_i;
	  uint32_t min_j;
	  for(uint32_t i = 0;i<num_planes;i++)
	  {
		  for(uint32_t j = 0;j<num_satellites_per_plane;j++)
		  {
			  double sat_x=plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition().x;
			  double sat_y=plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition().y;
			  double sat_z=plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition().z;
			  double gro_x=ground_stations.Get(kk)->GetObject<MobilityModel> ()->GetPosition().x;
			  double gro_y=ground_stations.Get(kk)->GetObject<MobilityModel> ()->GetPosition().y;


			  //使用球坐标，计算两点间距离
			  //（r,xita,fai）
			  sat_x=(90-sat_x)/180*3.1416;
			  sat_y=(sat_y+180)/180*3.1416;
			  sat_z+=6371.8;
			  gro_x=(90-gro_x)/180*3.1416;
			  gro_y=(180+gro_y)/180*3.1416;
			  double gro_z=6371.8;
			  distance=DistanceOfTwoPoints(sat_z,sat_x,sat_y,gro_z,gro_x,gro_y);
			  if(distance<min_distance)
			  {
				  min_distance=distance;
				  min_i=i;
				  min_j=j;
			  }
		  }
	  }
	  //std::cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"<<std::endl;


	  ObjectFactory ob_channel;
	  ob_channel.SetTypeId("ns3::PointToPointChannel");
	  ob_channel.Set("Delay",TimeValue(Seconds(min_distance*1000/speed_of_light)));
	  Ptr<PointToPointChannel> gro_sat_channel = ob_channel.Create<PointToPointChannel> ();

	  gro_sat_channel->Detach();
	  //gro_sat_channel->Attach(all_nodes_p2p_devices.Get(kk)->GetObject<PointToPointNetDevice>());
	  //gro_sat_channel->Attach(all_nodes_p2p_devices.Get(2+min_i*num_satellites_per_plane+min_j)->GetObject<PointToPointNetDevice>());
	  //gro_sat_channel->Detach();
	  all_nodes_p2p_devices.Get(kk)->GetObject<PointToPointNetDevice>()->Attach(gro_sat_channel);
	  all_nodes_p2p_devices.Get(2+min_i*num_satellites_per_plane+min_j)->GetObject<PointToPointNetDevice>()->Attach(gro_sat_channel);

	  this->ground_station_channels.push_back(gro_sat_channel);
	  this->ground_station_channel_tracker.push_back(min_i);
	  this->ground_station_channel_tracker.push_back(min_j);
  }
  this->ground_station_devices.push_back(all_nodes_p2p_devices);

  //Configure IP Addresses for all NetDevices
  Ipv4AddressHelper address;
  //address.SetBase ("10.1.0.0", "255.255.255.0");
  address.SetBase ("10.2.0.0", "255.255.255.0");//swd

  //configuring IP Addresses for IntraPlane devices
  for(uint32_t i=0; i< this->intra_plane_devices.size(); i++)
  {
    address.NewNetwork();
    this->intra_plane_interfaces.push_back(address.Assign(this->intra_plane_devices[i]));
  }
  //address.NewNetwork();
  
  //configuring IP Addresses for InterPlane devices
  for(uint32_t i=0; i< this->inter_plane_devices.size(); i++)
    {
      address.NewNetwork();
      this->inter_plane_interfaces.push_back(address.Assign(this->inter_plane_devices[i]));
    }
  //address.NewNetwork();

  //configuring IP Addresses for Ground devices
//  for(uint32_t i =0;i< this->ground_station_devices.size();i++)
//  {
	  address.NewNetwork();
	  this->ground_station_interfaces.push_back(address.Assign(this->ground_station_devices[0]));
//  }








  //Populate Routing Tables
//	  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
	NodeContainer c;

	for(uint32_t i=0;i<num_planes;i++)
	{
		for(uint32_t j=0;j<num_satellites_per_plane;j++)
		{

			logical_address_table.push_back(std::make_pair(i,j));
			Ptr<Node> node=plane[i].Get(j);
//			uint32_t device_num=node->GetNDevices();
			std::vector<uint32_t> n_device_queue;
//			for(uint32_t k=0;k<device_num;k++)
//			{
//				uint32_t load=node->GetDevice(k)->GetObject<PointToPointNetDevice>()->GetQueue()->GetNPackets();
//				n_device_queue.push_back(load);
//			}
//			satellite_load_table.push_back(std::make_pair(std::make_pair(i,j),n_device_queue));

			c.Add(node);
			Ptr<Ipv4>ipv4_temp=node->GetObject<Ipv4>();
			int16_t priority;
			Ptr<Ipv4GlobalRouting>routing=ipv4_temp->
					  	  	  	  	  	  GetRoutingProtocol()->
					                      GetObject<Ipv4ListRouting>()->
										  GetRoutingProtocol(0,priority)->
										  GetObject<Ipv4GlobalRouting>();
			priority=10;
			uint32_t num_routes=routing->GetNRoutes();
			for(uint32_t k=0;k<num_routes;k++)
			{
				routing->RemoveRoute(routing->GetNRoutes()-1);
			}
		}
	}

	for(uint32_t i=0;i<ground_stations.GetN();i++)
	{
		Ptr<Node> node=ground_stations.Get(i);
		Ptr<Ipv4>ipv4_temp=node->GetObject<Ipv4>();
		int16_t priority;
		Ptr<Ipv4GlobalRouting>routing=ipv4_temp->
				  	  	  	  	  	  GetRoutingProtocol()->
				                      GetObject<Ipv4ListRouting>()->
									  GetRoutingProtocol(0,priority)->
									  GetObject<Ipv4GlobalRouting>();

		uint32_t num_routes=routing->GetNRoutes();
		for(uint32_t k=0;k<num_routes;k++)
		{
			routing->RemoveRoute(routing->GetNRoutes()-1);
		}
	}



	for(uint32_t i=0;i<num_planes;i++)
	{
		for(uint32_t j=0;j<num_satellites_per_plane;j++)
		{
			Ptr<Node> node =plane[i].Get(j);
			SatelliteRoutes m_SatelliteRoutes;
			//std::cout<<"GenerateSatelliteRoutingTable(m_SatelliteRoutes,node,c,logical_address_table)"<<std::endl;
			m_SatelliteRoutes=GenerateSatelliteRoutingTable(m_SatelliteRoutes,node,c,logical_address_table);//,satellite_load_table);
			Ptr<Ipv4>ipv4_temp=node->GetObject<Ipv4>();
			int16_t priority;
			Ptr<Ipv4GlobalRouting>routing=ipv4_temp->
					  	  	  	  	  	  GetRoutingProtocol()->
					                      GetObject<Ipv4ListRouting>()->
										  GetRoutingProtocol(0,priority)->
										  GetObject<Ipv4GlobalRouting>();
			priority=10;

			for(uint32_t k=0;k<m_SatelliteRoutes.size();k++)
			{
				routing->AddHostRouteTo(m_SatelliteRoutes[k]->GetDest(), m_SatelliteRoutes[k]->GetGateway(), m_SatelliteRoutes[k]->GetInterface());
			}
		}
	}

	for(uint32_t i=0;i<ground_stations.GetN();i++)
	{
		c.Add(ground_stations.Get(i));
	}
	for(uint32_t i=0;i<c.GetN();i++)
	{
		Ptr<Node> current_node=c.Get(i);
		if (current_node->GetObject<MobilityModel>()->GetPosition().z < 0.1)	//this node is ground station
		{
			Ptr<PointToPointNetDevice> d_l = current_node->GetDevice(1)->GetObject<PointToPointNetDevice>();
			Ptr<PointToPointChannel> c_l = d_l->GetChannel()->GetObject<PointToPointChannel>();
			Ptr<PointToPointNetDevice> d_a = (c_l->GetPointToPointDevice(0) == d_l) ? c_l->GetPointToPointDevice(1) : c_l->GetPointToPointDevice(0);
			Ptr<Ipv4L3Protocol> ipv4_a = d_a->GetNode()->GetObject<Ipv4L3Protocol>();
			Ipv4InterfaceAddress interface_a = ipv4_a->GetAddress((ipv4_a->GetInterfaceForDevice(d_a)), 0);
			Ipv4Address ipv4address_a = interface_a.GetLocal();
			Ptr<Ipv4> m_ipv4=current_node->GetObject<Ipv4>();
			int16_t priority;
			Ptr<Ipv4GlobalRouting>routing=m_ipv4->
				  	  	  	  	  	  GetRoutingProtocol()->
				                      GetObject<Ipv4ListRouting>()->
									  GetRoutingProtocol(0,priority)->
									  GetObject<Ipv4GlobalRouting>();
			priority=10;
//			routing->AddHostRouteTo(Ipv4Address("0.0.0.0"),ipv4address_a,m_ipv4->GetInterfaceForDevice(d_l));
			routing->AddNetworkRouteTo(Ipv4Address("0.0.0.0"), Ipv4Mask("0.0.0.0"), ipv4address_a, m_ipv4->GetInterfaceForDevice(d_l));
		}
		else
		{
			Ptr<Ipv4> m_ipv4=current_node->GetObject<Ipv4>();
			int16_t priority;
			Ptr<Ipv4GlobalRouting>routing=m_ipv4->
				  	  	  	  	  	  GetRoutingProtocol()->
				                      GetObject<Ipv4ListRouting>()->
									  GetRoutingProtocol(0,priority)->
									  GetObject<Ipv4GlobalRouting>();
			for(uint32_t k=0;k<ground_stations.GetN();k++)
			{
				Ptr<Node> gs=ground_stations.Get(k);
				Ptr<Ipv4L3Protocol> gs_ipv4=gs->GetObject<Ipv4L3Protocol>();
				Ptr<PointToPointNetDevice> d_l = gs->GetDevice(1)->GetObject<PointToPointNetDevice>();
				Ptr<PointToPointChannel> c_l = d_l->GetChannel()->GetObject<PointToPointChannel>();
				Ptr<PointToPointNetDevice> d_a = (c_l->GetPointToPointDevice(0) == d_l) ? c_l->GetPointToPointDevice(1) : c_l->GetPointToPointDevice(0);
				Ptr<Ipv4L3Protocol> ipv4_a = d_a->GetNode()->GetObject<Ipv4L3Protocol>();
				Ipv4InterfaceAddress interface_a = ipv4_a->GetAddress((ipv4_a->GetInterfaceForDevice(d_a)), 0);
				Ipv4Address ipv4address_a = interface_a.GetLocal();
				Ipv4Address gs_ipadd=gs->GetObject<Ipv4>()->GetAddress((gs_ipv4->GetInterfaceForDevice(d_l)), 0).GetLocal();
				if(d_a->GetNode()==current_node)
				{
					routing->AddHostRouteTo(gs_ipadd,gs_ipadd,m_ipv4->GetInterfaceForDevice(d_a));
					continue;
				}

//				for(uint32_t ind=0;ind<routing->GetNRoutes();ind++)
//				{
//					if(routing->GetRoute(ind)->GetDest()==gs_ipadd)
//						routing->RemoveRoute(ind);
//				}
				for(uint32_t ind=0;ind<routing->GetNRoutes();ind++)
				{
//						Ptr<Ipv4RoutingTableEntry> irte=routing->GetRoute(ind);
					if(routing->GetRoute(ind)->GetDest()==ipv4address_a)
					{
						routing->AddHostRouteTo(gs_ipadd, routing->GetRoute(ind)->GetGateway(), routing->GetRoute(ind)->GetInterface());
					}
				}
			}
//				routing->AddHostRouteTo(dest, nextHop, interface)
		}
//		current_node->GetObject<Ipv4L3Protocol>()->
//  	  	  	  	      GetRoutingProtocol()->
//                      GetObject<Ipv4ListRouting>()->ChangePriority(priority);
	}





  std::cout<<"Populating Routing Tables"<<std::endl;
//  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
//  double time=Simulator::Now().GetSeconds();
//  std::string routename="routes_at_nownownow";
//  routename+=std::to_string(time);
//  Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(routename,std::ios::out);
//  Ipv4GlobalRoutingHelper::PrintRoutingTableAllAt(Seconds(0),routingStream);
//  Ipv4GlobalRoutingHelper::Print(ground_stations.Get(0),routingStream,Seconds(3.0));

//

//  PrintGlobalNetInfo();
  std::cout<<"Finished Populating Routing Tables"<<std::endl;
  std::cout<<Simulator::Now().GetSeconds()<<std::endl;
  for(uint32_t i=0;i<2000;i++)
  {
	  Simulator::Schedule(Seconds(i/10.0),&LeoSatelliteConfig::UpdateLinks,this);
//	  std::cout<<"--------------------"<<i<<Simulator::Now().GetSeconds()<<std::endl;
  }

}






void LeoSatelliteConfig::UpdateLinks()  //swd
{
	std::cout<<std::endl<<std::endl<<std::endl<<"Updating Links"<<std::endl;

	std::cout<<"The location of all satellites:"<<std::endl;
	//The location of all satellites
	for(uint32_t i=0; i < this->num_planes; i++)
	{
		for(uint32_t j=0; j < this->num_satellites_per_plane; j++)
		{
			Vector SatellitePos = this->plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition();
			std::cout << Simulator::Now().GetSeconds() << ": plane # "<< i << " node # " <<j<< ": x = " << SatellitePos.x << ", y = " << SatellitePos.y << ", z = " << SatellitePos.z << std::endl;
		}
	}
	std::cout<<"The location of all stations:"<<std::endl;
	for(uint32_t i=0;i<2;i++)
	{
		Vector StationPos = this->ground_stations.Get(i)->GetObject<MobilityModel>()->GetPosition();
		std::cout << Simulator::Now().GetSeconds() << ": station # "<< i << ": x = " << StationPos.x << ", y = " << StationPos.y << ", z = " << StationPos.z << std::endl;
	}

	std::cout<<"Current situation of intra-plane links:"<<std::endl;
	//Current situation of intra-plane links except for reverse seams
	Vector nodeAPosition = this->plane[0].Get(0)->GetObject<MobilityModel>()->GetPosition();
	Vector nodeBPosition = this->plane[0].Get(1)->GetObject<MobilityModel>()->GetPosition();
	double distance = CalculateDistance(nodeAPosition, nodeBPosition);
	double delay = (distance * 1000)/speed_of_light; //should get delay in seconds
	std::cout<<"Intra-plane links with distance of "<<distance<<" km and delay of "<<delay<<" seconds."<<std::endl;

	std::cout<<"Current situation of inter-plane links:"<<std::endl;
	//Current situation of inter-plane links
	for(uint32_t i=0; i < this->num_planes-1; i++)
	{
		for(uint32_t j=0; j < this->num_satellites_per_plane; j++)
		{
			uint32_t nodeBIndex; //Used to establish a connection between reverse seams.
			(i == num_planes - 1) ? nodeBIndex = num_satellites_per_plane - j - 1: nodeBIndex = j;
			Vector nodeAPos = this->plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition();
			Vector nodeBPos = this->plane[i+1].Get(j)->GetObject<MobilityModel>()->GetPosition();
			double distance = CalculateDistance(nodeAPos, nodeBPos);
			double delay = (distance*1000)/speed_of_light;

		    if(nodeAPos.x < -70 || nodeAPos.x >70 || nodeBPos.x < -70 || nodeBPos.x > 70) // swd
		    {
//		    	node_pr[i*num_satellites_per_plane+j] = 0;
		    	this->inter_plane_devices[i*num_satellites_per_plane+j].Get(0)->SetAttribute("DataRate", StringValue("1bps"));
		    	this->inter_plane_devices[i*num_satellites_per_plane+j].Get(1)->SetAttribute("DataRate", StringValue("1bps"));
//		    	this->inter_plane_channels[i*num_satellites_per_plane+j]->SetAttribute("Delay", TimeValue(Seconds(2000)));
		    	//temp_netdevice_container = badlink.Install(temp_node_container);
		    	//std::cout<<"Channel can not open between plane "<<i<<" satellite "<<j<<" and plane "<<(i+1)%num_planes<<" satellite "<<nodeBIndex<< std::endl;
		    }
		    else
		    {
//		    	node_pr[i*num_satellites_per_plane+j] = 1;
		    	this->inter_plane_devices[i*num_satellites_per_plane+j].Get(0)->SetAttribute("DataRate", StringValue("5Mbps"));
		        this->inter_plane_devices[i*num_satellites_per_plane+j].Get(1)->SetAttribute("DataRate", StringValue("5Mbps"));
		    	this->inter_plane_channels[i*num_satellites_per_plane+j]->SetAttribute("Delay", TimeValue(Seconds(delay)));
		    	//temp_netdevice_container = interplane_link_helper.Install(temp_node_container);
		    	//std::cout<<"Channel open between plane "<<i<<" satellite "<<j<<" and plane "<<(i+1)%num_planes<<" satellite "<<nodeBIndex<< " with distance "<<distance<< "km and delay of "<<delay<<" seconds"<<std::endl;
		    }
		}

	}

	std::cout<<"Updating links between ground stations and their closest satellites:"<<std::endl;
	//updating links between ground stations and their closest satellites


	  for(uint32_t kk=0;kk<2;kk++)
	  {
		  double distance;
		  double min_distance=99999.0;
		  uint32_t min_i;
		  uint32_t min_j;
		  for(uint32_t i = 0;i<num_planes;i++)
		  {
			  for(uint32_t j = 0;j<num_satellites_per_plane;j++)
			  {
				  double sat_x=plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition().x;
				  double sat_y=plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition().y;
				  double sat_z=plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition().z;
				  double gro_x=ground_stations.Get(kk)->GetObject<MobilityModel> ()->GetPosition().x;
				  double gro_y=ground_stations.Get(kk)->GetObject<MobilityModel> ()->GetPosition().y;


				  //使用球坐标，计算两点间距离
				  //（r,xita,fai）
				  sat_x=(90-sat_x)/180*3.1416;
				  sat_y=(sat_y+180)/180*3.1416;
				  sat_z+=6371.8;
				  gro_x=(90-gro_x)/180*3.1416;
				  gro_y=(180+gro_y)/180*3.1416;
				  double gro_z=6371.8;
				  distance=DistanceOfTwoPoints(sat_z,sat_x,sat_y,gro_z,gro_x,gro_y);
				  if(distance<min_distance)
				  {
					  min_distance=distance;
					  min_i=i;
					  min_j=j;
				  }
			  }
		  }
		  if(min_i==ground_station_channel_tracker[kk*2]&&min_j==ground_station_channel_tracker[kk*2+1])
			  ground_station_channels[kk]->SetAttribute("Delay", TimeValue(Seconds(min_distance*1000/speed_of_light)));
		  else
		  {

			  ground_station_devices[0].Get(kk)->GetObject<PointToPointNetDevice>()->Detach();
			  ground_station_devices[0].Get(2+ground_station_channel_tracker[kk*2]*num_satellites_per_plane+ground_station_channel_tracker[kk*2+1])->GetObject<PointToPointNetDevice>()->Detach();
			  ground_station_channels[kk]->Detach();
			  ground_station_channels[kk]->SetAttribute("Delay", TimeValue(Seconds(min_distance*1000/speed_of_light)));
			  ground_station_devices[0].Get(kk)->GetObject<PointToPointNetDevice>()->Attach(ground_station_channels[kk]);
			  ground_station_devices[0].Get(2+min_i*num_satellites_per_plane+min_j)->GetObject<PointToPointNetDevice>()->Attach(ground_station_channels[kk]);
			  ground_station_channel_tracker[kk*2]=min_i;
			  ground_station_channel_tracker[kk*2+1]=min_j;
		  }
	  }

	//Recompute Routing Tables
	std::cout<<"Recomputing Routing Tables"<<std::endl;
//	Ipv4GlobalRoutingHelper::RecomputeRoutingTables ();


	NodeContainer c;
	for(uint32_t i=0;i<num_planes;i++)
	{
		for(uint32_t j=0;j<num_satellites_per_plane;j++)
		{

			Ptr<Node> node=plane[i].Get(j);
			c.Add(node);
			Ptr<Ipv4>ipv4_temp=node->GetObject<Ipv4>();
			int16_t priority;
			Ptr<Ipv4GlobalRouting>routing=ipv4_temp->
					  	  	  	  	  	  GetRoutingProtocol()->
					                      GetObject<Ipv4ListRouting>()->
										  GetRoutingProtocol(0,priority)->
										  GetObject<Ipv4GlobalRouting>();

			uint32_t num_routes=routing->GetNRoutes();
			for(uint32_t k=0;k<num_routes;k++)
			{
				routing->RemoveRoute(routing->GetNRoutes()-1);
			}
			priority=10;
		}
	}


	for(uint32_t i=0;i<ground_stations.GetN();i++)
	{
		Ptr<Node> node=ground_stations.Get(i);
		Ptr<Ipv4>ipv4_temp=node->GetObject<Ipv4>();
		int16_t priority;
		Ptr<Ipv4GlobalRouting>routing=ipv4_temp->
				  	  	  	  	  	  GetRoutingProtocol()->
				                      GetObject<Ipv4ListRouting>()->
									  GetRoutingProtocol(0,priority)->
									  GetObject<Ipv4GlobalRouting>();

		uint32_t num_routes=routing->GetNRoutes();
		for(uint32_t k=0;k<num_routes;k++)
		{
			routing->RemoveRoute(routing->GetNRoutes()-1);
		}
	}



	for(uint32_t i=0;i<num_planes;i++)
	{
		for(uint32_t j=0;j<num_satellites_per_plane;j++)
		{
			Ptr<Node> node =plane[i].Get(j);
			SatelliteRoutes m_SatelliteRoutes;
			m_SatelliteRoutes=GenerateSatelliteRoutingTable(m_SatelliteRoutes,node,c,logical_address_table);//,satellite_load_table);
			Ptr<Ipv4>ipv4_temp=node->GetObject<Ipv4>();
			int16_t priority;
			Ptr<Ipv4GlobalRouting>routing=ipv4_temp->
					  	  	  	  	  	  GetRoutingProtocol()->
					                      GetObject<Ipv4ListRouting>()->
										  GetRoutingProtocol(0,priority)->
										  GetObject<Ipv4GlobalRouting>();
			priority=10;

			for(uint32_t k=0;k<m_SatelliteRoutes.size();k++)
			{
				routing->AddHostRouteTo(m_SatelliteRoutes[k]->GetDest(), m_SatelliteRoutes[k]->GetGateway(), m_SatelliteRoutes[k]->GetInterface());
			}
		}
	}

	for(uint32_t i=0;i<ground_stations.GetN();i++)
	{
		c.Add(ground_stations.Get(i));
	}
	for(uint32_t i=0;i<c.GetN();i++)
	{
		Ptr<Node> current_node=c.Get(i);
		if (current_node->GetObject<MobilityModel>()->GetPosition().z < 0.1)	//this node is ground station
		{
			Ptr<PointToPointNetDevice> d_l = current_node->GetDevice(1)->GetObject<PointToPointNetDevice>();
			Ptr<PointToPointChannel> c_l = d_l->GetChannel()->GetObject<PointToPointChannel>();
			Ptr<PointToPointNetDevice> d_a = (c_l->GetPointToPointDevice(0) == d_l) ? c_l->GetPointToPointDevice(1) : c_l->GetPointToPointDevice(0);
			Ptr<Ipv4L3Protocol> ipv4_a = d_a->GetNode()->GetObject<Ipv4L3Protocol>();
			Ipv4InterfaceAddress interface_a = ipv4_a->GetAddress((ipv4_a->GetInterfaceForDevice(d_a)), 0);
			Ipv4Address ipv4address_a = interface_a.GetLocal();
			Ptr<Ipv4> m_ipv4=current_node->GetObject<Ipv4>();
			int16_t priority;
			Ptr<Ipv4GlobalRouting>routing=m_ipv4->
				  	  	  	  	  	  GetRoutingProtocol()->
				                      GetObject<Ipv4ListRouting>()->
									  GetRoutingProtocol(0,priority)->
									  GetObject<Ipv4GlobalRouting>();
			priority=10;
//			routing->AddHostRouteTo(Ipv4Address("0.0.0.0"),ipv4address_a,m_ipv4->GetInterfaceForDevice(d_l));
			routing->AddNetworkRouteTo(Ipv4Address("0.0.0.0"), Ipv4Mask("0.0.0.0"), ipv4address_a, m_ipv4->GetInterfaceForDevice(d_l));
		}
		else
		{
			Ptr<Ipv4> m_ipv4=current_node->GetObject<Ipv4>();
			int16_t priority;
			Ptr<Ipv4GlobalRouting>routing=m_ipv4->
				  	  	  	  	  	  GetRoutingProtocol()->
				                      GetObject<Ipv4ListRouting>()->
									  GetRoutingProtocol(0,priority)->
									  GetObject<Ipv4GlobalRouting>();
			for(uint32_t k=0;k<ground_stations.GetN();k++)
			{
				Ptr<Node> gs=ground_stations.Get(k);
				Ptr<Ipv4L3Protocol> gs_ipv4=gs->GetObject<Ipv4L3Protocol>();
				Ptr<PointToPointNetDevice> d_l = gs->GetDevice(1)->GetObject<PointToPointNetDevice>();
				Ptr<PointToPointChannel> c_l = d_l->GetChannel()->GetObject<PointToPointChannel>();
				Ptr<PointToPointNetDevice> d_a = (c_l->GetPointToPointDevice(0) == d_l) ? c_l->GetPointToPointDevice(1) : c_l->GetPointToPointDevice(0);
				Ptr<Ipv4L3Protocol> ipv4_a = d_a->GetNode()->GetObject<Ipv4L3Protocol>();
				Ipv4InterfaceAddress interface_a = ipv4_a->GetAddress((ipv4_a->GetInterfaceForDevice(d_a)), 0);
				Ipv4Address ipv4address_a = interface_a.GetLocal();
				Ipv4Address gs_ipadd=gs->GetObject<Ipv4>()->GetAddress((gs_ipv4->GetInterfaceForDevice(d_l)), 0).GetLocal();
				if(d_a->GetNode()==current_node)
				{
					routing->AddHostRouteTo(gs_ipadd,gs_ipadd,m_ipv4->GetInterfaceForDevice(d_a));
					continue;
				}

//				for(uint32_t ind=0;ind<routing->GetNRoutes();ind++)
//				{
//					if(routing->GetRoute(ind)->GetDest()==gs_ipadd)
//						routing->RemoveRoute(ind);
//				}
				for(uint32_t ind=0;ind<routing->GetNRoutes();ind++)
				{
//						Ptr<Ipv4RoutingTableEntry> irte=routing->GetRoute(ind);
					if(routing->GetRoute(ind)->GetDest()==ipv4address_a)
					{
						routing->AddHostRouteTo(gs_ipadd, routing->GetRoute(ind)->GetGateway(), routing->GetRoute(ind)->GetInterface());
						break;
					}
				}
			}
//				routing->AddHostRouteTo(dest, nextHop, interface)
		}
//		current_node->GetObject<Ipv4L3Protocol>()->
//  	  	  	  	      GetRoutingProtocol()->
//                      GetObject<Ipv4ListRouting>()->ChangePriority(priority);
	}
//	  double time=Simulator::Now().GetSeconds();
//	  std::string routename="routes_at";
//	  routename+=std::to_string(time);
//	  Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(routename,std::ios::out);
//	  Ipv4GlobalRoutingHelper::PrintRoutingTableAllAt(Seconds(0),routingStream);




	std::cout<<"Finished Recomputing Routing Tables"<<std::endl;


}


void LeoSatelliteConfig::PrintGlobalNetInfo()
{

	for(uint32_t i=0;i<num_planes;i++)
	{
		std::cout<<std::endl<<std::endl;
		std::cout<<"plane "<<i <<" satellites:"<<std::endl;
		for(uint32_t j=0;j<num_satellites_per_plane;j++)
		{
			std::cout<<"("<<i<<","<<j<<"):"<<std::endl;
			Vector pos= this->plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition();
			std::cout<<"["<<pos.x<<","<<pos.y<<","<<pos.z<<"]"<<std::endl;
			if(i==0)
			{
				std::cout<<"satellites beside the seam have 3 ISL device:"<<std::endl;
				std::cout<<"("<<i<<","<<j<<")->"<<"("<<i<<","<<(j+1)%num_satellites_per_plane<<"):	"
						 <<intra_plane_interfaces[i*num_satellites_per_plane+j].GetAddress(0)<<"->"
						 <<intra_plane_interfaces[i*num_satellites_per_plane+j].GetAddress(1)<<std::endl;
				uint32_t temp = (j==0 ? (i+1)*num_satellites_per_plane-1 : j-1);
				std::cout<<"("<<i<<","<<j<<")->"<<"("<<i<<","<<temp<<"):	"
						 <<intra_plane_interfaces[temp].GetAddress(1)<<"->"
						 <<intra_plane_interfaces[temp].GetAddress(0)<< std::endl;
				std::cout<<"("<<i<<","<<j<<")->"<<"("<<i+1<<","<<j<<"):	"
						 <<inter_plane_interfaces[i*num_satellites_per_plane+j].GetAddress(0)<<"->"
						 <<inter_plane_interfaces[i*num_satellites_per_plane+j].GetAddress(1)<<std::endl;
			}
			else if(i==num_planes-1)
			{
				std::cout<<"satellites beside the seam have 3 ISL device:"<<std::endl;
				std::cout<<"("<<i<<","<<j<<")->"<<"("<<i<<","<<(j+1)%num_satellites_per_plane<<"):	"
						 <<intra_plane_interfaces[i*num_satellites_per_plane+j].GetAddress(0)<<"->"
						 <<intra_plane_interfaces[i*num_satellites_per_plane+j].GetAddress(1)<<std::endl;
				uint32_t temp = (j==0 ? (i+1)*num_satellites_per_plane-1 : j-1);
				std::cout<<"("<<i<<","<<j<<")->"<<"("<<i<<","<<temp%num_satellites_per_plane<<"):	"
						 <<intra_plane_interfaces[i*num_satellites_per_plane+temp%num_satellites_per_plane].GetAddress(1)<<"->"
						 <<intra_plane_interfaces[i*num_satellites_per_plane+temp%num_satellites_per_plane].GetAddress(0)<< std::endl;
				std::cout<<"("<<i<<","<<j<<")->"<<"("<<i-1<<","<<j<<"):	"
						 <<inter_plane_interfaces[(i-1)*num_satellites_per_plane+j].GetAddress(1)<<"->"
						 <<inter_plane_interfaces[(i-1)*num_satellites_per_plane+j].GetAddress(0)<<std::endl;
			}
			else
			{
				std::cout<<"satellites NOT beside the seam have 4 ISL device:"<<std::endl;
				std::cout<<"("<<i<<","<<j<<")->"<<"("<<i<<","<<(j+1)%num_satellites_per_plane<<"):	"
						 <<intra_plane_interfaces[i*num_satellites_per_plane+j].GetAddress(0)<<"->"
						 <<intra_plane_interfaces[i*num_satellites_per_plane+j].GetAddress(1)<<std::endl;
				uint32_t temp = (j==0 ? (i+1)*num_satellites_per_plane-1 : j-1);
				std::cout<<"("<<i<<","<<j<<")->"<<"("<<i<<","<<temp%num_satellites_per_plane<<"):	"
						 <<intra_plane_interfaces[i*num_satellites_per_plane+temp%num_satellites_per_plane].GetAddress(1)<<"->"
						 <<intra_plane_interfaces[i*num_satellites_per_plane+temp%num_satellites_per_plane].GetAddress(0)<< std::endl;
				std::cout<<"("<<i<<","<<j<<")->"<<"("<<i+1<<","<<j<<"):	"
						 <<inter_plane_interfaces[i*num_satellites_per_plane+j].GetAddress(0)<<"->"
						 <<inter_plane_interfaces[i*num_satellites_per_plane+j].GetAddress(1)<<std::endl;
				std::cout<<"("<<i<<","<<j<<")->"<<"("<<i-1<<","<<j<<"):	"
						 <<inter_plane_interfaces[(i-1)*num_satellites_per_plane+j].GetAddress(1)<<"->"
						 <<inter_plane_interfaces[(i-1)*num_satellites_per_plane+j].GetAddress(0)<<std::endl;
			}
			std::cout<<std::endl;


		}
	}

	std::cout<<std::endl<<std::endl;
	uint32_t num_station=ground_stations.GetN();
	std::cout<<"Now the network have "<<num_station<<" stations."<<std::endl;
	uint32_t num_interface=ground_station_interfaces[0].GetN();
	for(uint32_t i=0;i<num_interface;i++)
	{
		if(i==0)
		{
			std::cout<<"ground station"<<i<<":	"
					 <<ground_station_interfaces[0].GetAddress(i)<<"->"
					 <<ground_station_interfaces[0].GetAddress(ground_station_channel_tracker[0]*num_satellites_per_plane+ground_station_channel_tracker[1]+2)<<std::endl;

			continue;
		}
		if(i==1)
		{
			std::cout<<"ground station"<<i<<":	"
					 <<ground_station_interfaces[0].GetAddress(i)<<"->"
					 <<ground_station_interfaces[0].GetAddress(ground_station_channel_tracker[2]*num_satellites_per_plane+ground_station_channel_tracker[3]+2)<<std::endl;
			continue;
		}
		std::cout<<"satellite to ground station interface ("<<(i-2-(i-2)%num_satellites_per_plane)/num_satellites_per_plane<<","<<(i-2)%num_satellites_per_plane<<"):	"
				 <<ground_station_interfaces[0].GetAddress(i)<<std::endl;
	}
}

double
LeoSatelliteConfig::DistanceOfTwoPoints(double r1,double xita1,double fai1,double r2,double xita2,double fai2) const
{
	double distance;
	distance=sqrt(pow(r1*sin(xita1)*cos(fai1)-r2*sin(xita2)*cos(fai2),2)+
				  pow(r1*sin(xita1)*sin(fai1)-r2*sin(xita2)*sin(fai2),2)+
				  pow(r1*cos(xita1)-r2*cos(xita2),2));
	return distance;
}


double DistanceBetweenSatellites(std::pair<uint32_t, uint32_t> s1, std::pair<uint32_t, uint32_t> s2, NodeContainer c, std::vector<std::pair<uint32_t, uint32_t>> logical_address_table)
{
//	double current_time = Simulator::Now().GetSeconds();
	double R = 6378.1;
//	double x_0 = 50;
//	double x_1 = 60;
//	double G = 6.673e-11; // gravitational constant [Nm^2/kg^2]
//	double M = 5.972e24; // mass of Earth [kg]
	double distance;


	uint32_t num=(logical_address_table.end()-1)->second+1;
//	uint32_t plane=(logical_address_table.end()-1)->first+1;
	uint32_t ind_s1;
	uint32_t ind_s2;
	for(ind_s1=0;ind_s1<c.GetN();ind_s1++)
	{
		if(logical_address_table[ind_s1].first==s1.first
		 &&logical_address_table[ind_s1].second==s1.second)
			break;
	}
	if(ind_s1==c.GetN())return 999999999;

	for(ind_s2=0;ind_s2<c.GetN();ind_s2++)
	{
		if(logical_address_table[ind_s2].first==s2.first
		 &&logical_address_table[ind_s2].second==s2.second)
			break;
	}
	if(ind_s2==c.GetN())return 999999999;

	Ptr<Node> node1=c.Get(ind_s1);
	Ptr<Node> node2=c.Get(ind_s2);

	double h=node1->GetObject<MobilityModel>()->GetPosition().z;
	double s1x=node1->GetObject<MobilityModel>()->GetPosition().x;
	double s1y=node1->GetObject<MobilityModel>()->GetPosition().y;
	double s2x=node2->GetObject<MobilityModel>()->GetPosition().x;
	double s2y=node2->GetObject<MobilityModel>()->GetPosition().y;

	if(abs(s1x)>=70||abs(s2x)>=70) return 99999;

	if(s1.first==s2.first) return sin(3.1416/num)*(R+h)*2;

	s1x=(90+s1x)*3.1416/180;
	s1y=(180+s1y)*3.1416/180;
	s2x=(90+s2x)*3.1416/180;
	s2y=(180+s2y)*3.1416/180;

	double r1=R+h;
	double xita1=s1x;
	double fai1=s1y;
	double r2=r1;
	double xita2=s2x;
	double fai2=s2y;

	distance=sqrt(pow(r1*sin(xita1)*cos(fai1)-r2*sin(xita2)*cos(fai2),2)+
				  pow(r1*sin(xita1)*sin(fai1)-r2*sin(xita2)*sin(fai2),2)+
				  pow(r1*cos(xita1)-r2*cos(xita2),2));
	return distance;
}

std::vector<double> SatelliteRoutingDistance(std::vector<std::vector<std::pair<uint32_t, uint32_t>>> routing, NodeContainer c,std::vector<std::pair<uint32_t, uint32_t>> logical_address_table)
{
	std::vector<double> distance_table;
	for (std::vector<std::vector<std::pair<uint32_t, uint32_t>>>::iterator it = routing.begin(); it != routing.end(); it++)
	{
		double distance = 0;
		for (std::vector<std::pair<uint32_t, uint32_t>>::iterator node = it->begin(); node != it->end() - 1; node++)
		{
			distance = distance + DistanceBetweenSatellites(*node, *(node + 1), c,logical_address_table);
		}
		distance_table.push_back(distance);
	}
	return distance_table;
}





SatelliteRoutes GenerateSatelliteRoutingTable(SatelliteRoutes m_SatelliteRoutes,Ptr<Node>current_node,NodeContainer c,
		  	  	  	  	  	  	  	  	  	  std::vector<std::pair<uint32_t, uint32_t>> logical_address_table)//,
//											  std::vector<std::pair<std::pair<uint32_t,uint32_t>,std::vector<uint32_t>>> satellite_load_table)
{
			Ptr<Ipv4>m_ipv4=current_node->GetObject<Ipv4>();
//			double h = current_node->GetObject<MobilityModel>()->GetPosition().z;
//			uint32_t num = (logical_address_table.end() - 1)->second;
//			uint32_t plane = (logical_address_table.end() - 1)->first;
			std::pair<uint32_t, uint32_t> source_logical_address;
			std::pair<uint32_t, uint32_t> destination_logical_address;
			//	std::pair<uint32_t,uint32_t> logical_satellite_address;
			//	logical_satellite_address = GetLogicalSatelliteAddress(current_node);
			uint32_t i;
			enum HopDirection { POSITIVE, NEGATIVE, ZERO };
			std::pair<HopDirection, HopDirection> direction;
			//	uint32_t num_satellite_per_plane = (logical_address_table.end()-1)->second()+1;
			std::vector<std::pair<uint32_t, uint32_t>>::iterator temp = logical_address_table.end() - 1;
			uint32_t num_satellite_per_plane = temp->second + 1;

			for (i = 0; i < c.GetN(); i++)
			{
				if (c.Get(i) == current_node)
				{
					source_logical_address = logical_address_table[i];
					break;
				}
				else continue;
			}
			if (i == c.GetN())return m_SatelliteRoutes;

			for (i = 0; i < c.GetN(); i++)
			{
				if (c.Get(i) == current_node) continue;
				destination_logical_address = logical_address_table[i];
				if (destination_logical_address.first > source_logical_address.first)
					direction.first = POSITIVE;
				else if (destination_logical_address.first < source_logical_address.first)
					direction.first = NEGATIVE;
				else direction.first = ZERO;

				if (destination_logical_address.second > source_logical_address.second)
					direction.second = POSITIVE;
				else if (destination_logical_address.second < source_logical_address.second)
					direction.second = NEGATIVE;
				else direction.second = ZERO;

				std::vector<std::vector<std::vector<std::pair<uint32_t, uint32_t>>>> path_table1;
				std::vector<std::vector<std::vector<std::pair<uint32_t, uint32_t>>>> path_table2;
				std::vector<std::vector<std::pair<uint32_t, uint32_t>>> routing;

				//path_table[i]表示从source到第i个节点的路径表
				//path_table[i][j]表示从source到第i个节点的第j条路径
				//path_table[i][j]储存从source到第i个节点的第j条路径上的节点逻辑地址
				//routing维护从source到destination的若干条路径

				uint32_t h_hop = std::max(destination_logical_address.first, source_logical_address.first) -
					std::min(destination_logical_address.first, source_logical_address.first);
				uint32_t v_hop1 = std::max(destination_logical_address.second, source_logical_address.second) -
					std::min(destination_logical_address.second, source_logical_address.second);
				uint32_t v_hop2 = (num_satellite_per_plane -
					(std::max(destination_logical_address.second, source_logical_address.second) -
					std::min(destination_logical_address.second, source_logical_address.second))) %
					num_satellite_per_plane;

				for (uint32_t hh = 0; hh < h_hop + 1; hh++)
				{
					for (uint32_t vv = 0; vv < v_hop1 + 1; vv++)
					{
						std::vector<std::pair<uint32_t, uint32_t>> a;
						std::vector<std::vector<std::pair<uint32_t, uint32_t>>> b;
						uint32_t h_logical, v_logical;
						if (hh == 0 && vv == 0)//无下、无左
						{

							a.push_back(source_logical_address);
							b.push_back(a);
							path_table1.push_back(b);
							continue;
						}
						if (vv == 0)//无下，有左
						{
							if (direction.first == POSITIVE) h_logical = source_logical_address.first + hh;
							else h_logical = source_logical_address.first - hh;
							a = path_table1[(hh - 1) * (v_hop1 + 1)][0];
							a.push_back(std::make_pair(h_logical, source_logical_address.second));
							b.push_back(a);
							path_table1.push_back(b);
							continue;
						}
						if (hh == 0)//有下、无左
						{
							if (direction.second == POSITIVE) v_logical = source_logical_address.second + vv;
							else v_logical = source_logical_address.second - vv;
							a = (*(path_table1.end() - 1))[0];
							a.push_back(std::make_pair(source_logical_address.first, v_logical));
							b.push_back(a);
							path_table1.push_back(b);
							continue;
						}
						//有下、有左
						if (direction.first == POSITIVE) h_logical = source_logical_address.first + hh;
						else h_logical = source_logical_address.first - hh;
						if (direction.second == POSITIVE) v_logical = source_logical_address.second + vv;
						else v_logical = source_logical_address.second - vv;
						for (std::vector<std::vector<std::pair<uint32_t, uint32_t>>>::iterator it = path_table1[(hh - 1) * (v_hop1 + 1) + vv].begin();
							it != path_table1[(hh - 1) * (v_hop1 + 1) + vv].end(); it++)
						{
							a = *it;
							a.push_back(std::make_pair(h_logical, v_logical));
							b.push_back(a);
						}

						for (std::vector<std::vector<std::pair<uint32_t, uint32_t>>>::iterator it = (path_table1.end() - 1)->begin();
							it != (path_table1.end() - 1)->end(); it++)
						{
							a = *it;
							a.push_back(std::make_pair(h_logical, v_logical));
							b.push_back(a);
						}
						path_table1.push_back(b);
					}
				}


				//环形拓扑

				for (uint32_t hh = 0; hh < h_hop + 1; hh++)
				{
					for (uint32_t vv = 0; vv < v_hop2 + 1; vv++)
					{
						std::vector<std::pair<uint32_t, uint32_t>> a;
						std::vector<std::vector<std::pair<uint32_t, uint32_t>>> b;
						uint32_t h_logical, v_logical;
						if (hh == 0 && vv == 0)//无下、无左
						{

							a.push_back(source_logical_address);
							b.push_back(a);
							path_table2.push_back(b);
							continue;
						}
						if (vv == 0)//无下，有左
						{
							if (direction.first == POSITIVE) h_logical = source_logical_address.first + hh;
							else h_logical = source_logical_address.first - hh;
							a = path_table2[(hh - 1) * (v_hop2 + 1)][0];
							a.push_back(std::make_pair(h_logical, source_logical_address.second));
							b.push_back(a);
							path_table2.push_back(b);
							continue;
						}
						if (hh == 0)//有下、无左
						{
							if (direction.second == NEGATIVE) v_logical = (source_logical_address.second + vv) % num_satellite_per_plane;
							else v_logical = (source_logical_address.second + num_satellite_per_plane - vv) % num_satellite_per_plane;
							a = (*(path_table2.end() - 1))[0];
							a.push_back(std::make_pair(source_logical_address.first, v_logical));
							b.push_back(a);
							path_table2.push_back(b);
							continue;
						}
						//有下、有左
						if (direction.first == POSITIVE) h_logical = source_logical_address.first + hh;
						else h_logical = source_logical_address.first - hh;
						if (direction.second == NEGATIVE) v_logical = (source_logical_address.second + vv) % num_satellite_per_plane;
						else v_logical = (source_logical_address.second + num_satellite_per_plane - vv) % num_satellite_per_plane;;
						for (std::vector<std::vector<std::pair<uint32_t, uint32_t>>>::iterator it = path_table2[(hh - 1) * (v_hop2 + 1) + vv].begin();
							it != path_table2[(hh - 1) * (v_hop2 + 1) + vv].end(); it++)
						{
							a = *it;
							a.push_back(std::make_pair(h_logical, v_logical));
							b.push_back(a);
						}

						for (std::vector<std::vector<std::pair<uint32_t, uint32_t>>>::iterator it = (path_table2.end() - 1)->begin();
							it != (path_table2.end() - 1)->end(); it++)
						{
							a = *it;
							a.push_back(std::make_pair(h_logical, v_logical));
							b.push_back(a);
						}
						path_table2.push_back(b);
					}
				}

				for (std::vector<std::vector<std::pair<uint32_t, uint32_t>>>::iterator it = (path_table1.end() - 1)->begin();
					it != (path_table1.end() - 1)->end(); it++)
				{
					routing.push_back(*it);
				}
				for (std::vector<std::vector<std::pair<uint32_t, uint32_t>>>::iterator it = (path_table2.end() - 1)->begin();
					it != (path_table2.end() - 1)->end(); it++)
				{
					routing.push_back(*it);
				}

				std::vector<double> distance_table = SatelliteRoutingDistance(routing,c,logical_address_table);

//				for(std::vector<std::vector<std::pair<uint32_t, uint32_t>>>::iterator it=routing.begin();it!=routing.end();it++)
//				{
//					for(std::vector<std::pair<uint32_t, uint32_t>>::iterator that=it->begin();that!=it->end();that++)
//					{
//						if(that->)
//					}
//
//				}


				//最短路径排序
				for (uint32_t jj = 0; jj < distance_table.size() - 1; jj++)
				{
					for (uint32_t kk = 0; kk < distance_table.size() - 1 - jj; kk++)
					{
						if (distance_table[kk] > distance_table[kk + 1])
						{
							std::swap(distance_table[kk], distance_table[kk + 1]);
							std::swap(routing[kk], routing[kk + 1]);
						}
					}
				}

//				for(uint32_t path_ind=0;path_ind<routing[0].size()-1;path_ind++)
//				{
//					std::cout<<"("<<routing[0][path_ind].first<<","<<routing[0][path_ind].second<<")->";
//				}
//				std::cout<<"("<<(routing[0].end()-1)->first<<","<<(routing[0].end()-1)->second<<")"<<std::endl;

				if (distance_table[0] > 0.5)
				{
					std::pair<uint32_t, uint32_t> gateway_logical = routing[0][1];
					uint32_t aa = 0;
					for (std::vector<std::pair<uint32_t, uint32_t>>::iterator it = logical_address_table.begin(); it != logical_address_table.end(); it++)
					{
						if (gateway_logical.first == it->first && gateway_logical.second == it->second)
							break;
						aa += 1;
					}
					if (aa == c.GetN())return m_SatelliteRoutes;;
					Ptr<Node> node_a = c.Get(aa);
					for (aa = 0; aa < current_node->GetNDevices(); aa++)
					{
						if(!current_node->GetDevice(aa)->IsPointToPoint()) continue;
						Ptr<PointToPointNetDevice> d_l = current_node->GetDevice(aa)->GetObject<PointToPointNetDevice>();
						if(d_l->GetChannel()==0)continue;
						Ptr<PointToPointChannel> c_l = d_l->GetChannel()->GetObject<PointToPointChannel>();

						Ptr<PointToPointNetDevice> d_a = (c_l->GetPointToPointDevice(0) == d_l) ? c_l->GetPointToPointDevice(1) : c_l->GetPointToPointDevice(0);
						if (d_a->GetNode() == node_a)
						{

							Ptr<Ipv4L3Protocol> ipv4_a = node_a->GetObject<Ipv4L3Protocol>();
							Ipv4InterfaceAddress interface_a = ipv4_a->GetAddress((ipv4_a->GetInterfaceForDevice(d_a)), 0);
							Ipv4Address ipv4address_a = interface_a.GetLocal();
							Ptr<Node> DesNode = c.Get(i);
							uint32_t des_ip_num = DesNode->GetObject<Ipv4L3Protocol>()->GetNInterfaces();
							for (uint32_t ind = 0; ind < des_ip_num; ind++)
							{
								Ipv4Address DesIp = DesNode->GetObject<Ipv4L3Protocol>()->GetInterface(ind)->GetAddress(0).GetLocal();
								Ipv4RoutingTableEntry* rou = new Ipv4RoutingTableEntry(Ipv4RoutingTableEntry::CreateHostRouteTo(DesIp, ipv4address_a, m_ipv4->GetInterfaceForDevice(d_l)));

								m_SatelliteRoutes.push_back(rou);
							}
						}
						else continue;
					}
				}
//				m_SatelliteRoutes.push_back(&Ipv4RoutingTableEntry::CreateHostRouteTo(Ipv4Address("0.0.0.0"),ipv4address_a,m_ipv4->GetInterfaceForDevice(d_l)));
			}
			return m_SatelliteRoutes;
}






}

