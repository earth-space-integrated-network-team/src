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
#include <string>//swd

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (LeoSatelliteConfig);
NS_LOG_COMPONENT_DEFINE ("LeoSatelliteConfig");



extern double CalculateDistanceGroundToSat (const Vector &a, const Vector &b);

double speed_of_light = 299792458; //in m/s

//typeid
TypeId LeoSatelliteConfig::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LeoSatelliteConfig")
  .SetParent<Object> ()
  .SetGroupName("LeoSatellite")
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
  //pr = 0 : no satellite in the polar region ; pr = 1 : more than one satellite in the polar region
  pr = 0;
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
     for(uint32_t j=num_satellites_per_plane/2; j> 0; j--)
     {
       Vector pos = temp.Get(total_num_satellites/2 + i*num_satellites_per_plane/2 + j - 1)->GetObject<MobilityModel> ()->GetPosition();
       std::cout << Simulator::Now().GetSeconds() << ": plane # "<< i << " node # " <<num_satellites_per_plane - j<< ": x = " << pos.x << ", y = " << pos.y << ", z = " << pos.z << std::endl;
       temp_plane.Add(temp.Get(total_num_satellites/2 + i*num_satellites_per_plane/2 + j - 1));
     }
//     InternetStackHelper stack;
//     //OlsrHelper olsr;    //swd
//     //Ipv4ListRoutingHelper list;//swd
//     //list.Add(olsr,100);//swd
//     //stack.SetRoutingHelper(list);//swd
//     stack.Install(temp_plane);
     this->plane.push_back(temp_plane);//存放所有卫星，（0,0）~（0,3），（1,0）~（1,3）......
  }

  //setting up all intraplane links
  Vector nodeAPosition = this->plane[0].Get(0)->GetObject<MobilityModel>()->GetPosition();
  Vector nodeBPosition = this->plane[0].Get(1)->GetObject<MobilityModel>()->GetPosition();
  double distance = CalculateDistance(nodeAPosition, nodeBPosition);
  double delay = (distance * 1000)/speed_of_light; //should get delay in seconds

  PointToPointHelper intraplane_link_helper;
  intraplane_link_helper.SetDeviceAttribute ("DataRate", StringValue ("50Gbps"));
  intraplane_link_helper.SetChannelAttribute ("Delay", TimeValue(Seconds (delay)));

  std::cout<<"Setting up intra-plane links with distance of "<<distance<<" km and delay of "<<delay<<" seconds."<<std::endl;

  //遍历所有轨道所有节点，以为所有轨道内，节点配置p2p信道
  //(0,0)->(0,1)->(0,2)->(0,...)->(0,num_satellites_per_plane-1)->(0,0)
  //(...,0)->(...,1)->(...,2)->(...,...)->(...,num_satellites_per_plane-1)->(...,0)
  //(num_panes-1,0)->(num_panes-1,1)->(num_panes-1,2)->(num_panes-1,...)->(num_panes-1,num_satellites_per_plane-1)->(0,0)
  for (uint32_t i=0; i<num_planes; i++)
  {
    for (uint32_t j=0; j<num_satellites_per_plane; j++)
    {
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

  //setting up interplane links except for reverse seams
  //(0,0)->(1,0)	(0,1)->(1,1) ... (0,num_satellites_per_plane-1)->(1,num_satellites_per_plane-1)
  //(n0,0)->(n0+1,0)	(n0,1)->(n0+1,1) ... (n0,num_satellites_per_plane-1)->(n0+1,num_satellites_per_plane-1)
  //(num_planes-2,0)->(num_planes-1,0)	(num_planes-2,1)->(num_planes-1,1) ... (num_planes-2,num_satellites_per_plane-1)->(num_planes-1,num_satellites_per_plane-1)
  std::cout<<"Setting up inter-plane links"<<std::endl;
  for (uint32_t i=0; i<num_planes-1; i++)
  {
    for (uint32_t j=0; j<num_satellites_per_plane; j++)
    {
      uint32_t nodeBIndex; //ID of adjacent satellite
      (i == num_planes - 1) ? nodeBIndex = num_satellites_per_plane - j - 1: nodeBIndex = j;
      Vector nodeAPos = this->plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition();
      Vector nodeBPos = this->plane[(i+1)%num_planes].Get(nodeBIndex)->GetObject<MobilityModel>()->GetPosition();
      double distance = CalculateDistance(nodeAPos, nodeBPos);
      double delay = (distance*1000)/speed_of_light;

      //当卫星进入极区后,会使用badlink的特性,否则则使用interplane_link_helper的特性
      PointToPointHelper interplane_link_helper;
      PointToPointHelper badlink;
      interplane_link_helper.SetDeviceAttribute("DataRate", StringValue ("50Gbps"));
      interplane_link_helper.SetChannelAttribute("Delay", TimeValue(Seconds(delay)));
      badlink.SetDeviceAttribute("DataRate", StringValue ("1bps"));
      badlink.SetChannelAttribute("Delay", TimeValue(Seconds(delay)));

      //按照（0,0）<->（1,0）的方式存放入NodeContainer
      NodeContainer temp_node_container;
      temp_node_container.Add(this->plane[i].Get(j));
      temp_node_container.Add(this->plane[i+1].Get(j));

      NetDeviceContainer temp_netdevice_container;

      //判断卫星是否位于极区中
      if(nodeAPos.x < -70 || nodeAPos.x >70 || nodeBPos.x < -70 || nodeBPos.x > 70) // swd
      {
    	  pr = 1;
    	  temp_netdevice_container = badlink.Install(temp_node_container.Get(0),temp_node_container.Get(1));

    	  std::cout<<"Channel can not open between plane "<<i<<" satellite "<<j<<" and plane "<<(i+1)%num_planes<<" satellite "<<nodeBIndex<< std::endl;
          std::cout<<"With Position of" <<std::endl;
          std::cout<<"("<<i<<","<<j<<"): "<<"x:"<<plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition().x
        		                          <<"	y:"<<plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition().y
    									  <<"	z:"<<plane[i].Get(j)->GetObject<MobilityModel>()->GetPosition().z
    									  <<std::endl;
          std::cout<<"("<<i+1<<","<<j<<"): "<<"x:"<<plane[i+1].Get(j)->GetObject<MobilityModel>()->GetPosition().x
            		                      <<"	y:"<<plane[i+1].Get(j)->GetObject<MobilityModel>()->GetPosition().y
        							      <<"	z:"<<plane[i+1].Get(j)->GetObject<MobilityModel>()->GetPosition().z
        								  <<std::endl;
      }
      else
      {
    	  pr = 0;
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
      }

      Ptr<PointToPointChannel> p2p_channel;
      Ptr<Channel> channel;
      channel = temp_netdevice_container.Get(0)->GetChannel();
      p2p_channel = channel->GetObject<PointToPointChannel> ();
      this->inter_plane_channels.push_back(p2p_channel);
      channel = temp_netdevice_container.Get(1)->GetChannel();
      p2p_channel = channel->GetObject<PointToPointChannel> ();

      this->inter_pr.push_back(pr); //swd
      this->inter_plane_devices.push_back(temp_netdevice_container);
      this->inter_plane_channels.push_back(p2p_channel);
      this->inter_plane_channel_tracker.push_back(nodeBIndex);
    }
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

  //std::cout << "groundMobility.Install(ground_stations);" << std::endl;//swd
  groundMobility.Install(ground_stations);

//  //Install IP stack
//  InternetStackHelper stack;
//  //OlsrHelper olsr2;    //swd
//  //Ipv4ListRoutingHelper list2;//swd
//  //list2.Add(olsr2,100);//swd
//  //stack.SetRoutingHelper(list2);//swd
//  stack.Install(ground_stations);
  for (int j = 0; j<2; j++)
  {
    Vector temp = ground_stations.Get(j)->GetObject<MobilityModel> ()->GetPosition();
    std::cout<< "Current Time: " << Simulator::Now().GetSeconds() << ": ground station # " << j << ": x = " << temp.x << ", y = " << temp.y <<std::endl;
  }

  NodeContainer all_nodes;//将地面站与卫星节点全部加入到all_nodes中
  all_nodes.Add(ground_stations.Get(0));
  all_nodes.Add(ground_stations.Get(1));
  for(uint32_t i=0;i<num_planes;i++)
  {
	  for(uint32_t j=0;j<num_satellites_per_plane;j++)
	  {
		  all_nodes.Add(plane[i].Get(j));
	  }
  }

  //Install IP stack
  OlsrHelper olsr;
  //AodvHelper aodv;
  Ipv4StaticRoutingHelper staticRouting;
  InternetStackHelper stack;
  Ipv4ListRoutingHelper list;

  //list.Add(aodv,100);
  list.Add(olsr,100);
  list.Add(staticRouting,0);
  stack.SetRoutingHelper(list);
  stack.Install(all_nodes);
  Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("dynamic-global-routing.routes", std::ios::out);
  olsr.PrintRoutingTableAllEvery(Seconds(2.0), routingStream,  Time::S);

  FreePointToPointHelper p2p_for_all_nodes;
  NetDeviceContainer all_nodes_p2p_devices;
  all_nodes_p2p_devices = p2p_for_all_nodes.Install(all_nodes,StringValue("5.36Gbps"));

  for(uint32_t kk =0;kk<all_nodes_p2p_devices.GetN();kk++)
  {
	  all_nodes_p2p_devices.Get(kk)->GetObject<PointToPointNetDevice>()->Detach();
  }

  //setting up links between ground stations and their closest satellites
  std::cout<<"Setting links between ground stations and satellites"<<std::endl;

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

	  std::cout<<"Ground_station: "<<kk<<" <-> "<<"Satellite: "<<"("<<min_i<<","<<min_j<<")"<<std::endl;

	  ObjectFactory ob_channel;
	  ob_channel.SetTypeId("ns3::PointToPointChannel");
	  ob_channel.Set("Delay",TimeValue(Seconds(min_distance*1000/speed_of_light)));
	  Ptr<PointToPointChannel> gro_sat_channel = ob_channel.Create<PointToPointChannel> ();
	  gro_sat_channel->Detach();
	  all_nodes_p2p_devices.Get(kk)->GetObject<PointToPointNetDevice>()->Attach(gro_sat_channel);
	  all_nodes_p2p_devices.Get(2+min_i*num_satellites_per_plane+min_j)->GetObject<PointToPointNetDevice>()->Attach(gro_sat_channel);
	  this->ground_station_channels.push_back(gro_sat_channel);
	  this->ground_station_channel_tracker.push_back(min_i);
	  this->ground_station_channel_tracker.push_back(min_j);
  }
  this->ground_station_devices.push_back(all_nodes_p2p_devices);



//  for (uint32_t i=0; i<2; i++)
//  {
//    Vector gndPos = ground_stations.Get(i)->GetObject<MobilityModel> ()->GetPosition();
//    std::cout << Simulator::Now().GetSeconds() << ": ground station # " << i << ": x = " << gndPos.x << ", y = " << gndPos.y << std::endl;//swd
//    uint32_t closestAdjSat = 0;
//    uint32_t closestAdjSatDist = 0;
//    uint32_t planeIndex;  //未考虑地球自转情况下，基站设于planeIndex轨道正下方
//    if (i == 0)
//      planeIndex = 0;
//    else
//      planeIndex = floor(3*num_planes/7);
//
//    //find closest adjacent satellite for ground station
//    for (uint32_t j=0; j<this->num_satellites_per_plane; j++)
//    {
//      Vector pos = this->plane[planeIndex].Get(j)->GetObject<MobilityModel>()->GetPosition();
//      double temp_dist = CalculateDistanceGroundToSat(gndPos,pos);
//      if((temp_dist < closestAdjSatDist) || (j==0))
//      {
//        closestAdjSatDist = temp_dist;
//        closestAdjSat = j;
//      }
//    }
//    double delay = (closestAdjSatDist*1000)/speed_of_light;
//    CsmaHelper ground_station_link_helper;
//    ground_station_link_helper.SetChannelAttribute("DataRate", StringValue ("50Mbps"));//swd
//    ground_station_link_helper.SetChannelAttribute("Delay", TimeValue(Seconds(delay)));
//
//    std::cout<<"Channel open between ground station " << i << " and plane " << planeIndex << " satellite "<<closestAdjSat<<" with distance "<<closestAdjSatDist<< "km and delay of "<<delay<<" seconds"<<std::endl;
//
//    NodeContainer temp_node_container;
//    temp_node_container.Add(ground_stations.Get(i));
//    temp_node_container.Add(this->plane[planeIndex]);
//    NetDeviceContainer temp_netdevice_container;
//    temp_netdevice_container = ground_station_link_helper.Install(temp_node_container);
//    Ptr<CsmaChannel> csma_channel;
//    Ptr<Channel> channel;
//    channel = temp_netdevice_container.Get(0)->GetChannel();
//    csma_channel = channel->GetObject<CsmaChannel> ();
//
//    for (uint32_t k=0; k<num_satellites_per_plane; k++)
//    {
//      if (closestAdjSat != k)
//      {
//        csma_channel->Detach(temp_netdevice_container.Get(k+1)->GetObject<CsmaNetDevice> ());//Decoupling redundant ground-satellite links
//      }
//    }
//
//
//    this->ground_station_devices.push_back(temp_netdevice_container);
//    this->ground_station_channels.push_back(csma_channel);
//    this->ground_station_channel_tracker.push_back(closestAdjSat);
//  }

  //Configure IP Addresses for all NetDevices
  Ipv4AddressHelper address;
  //address.SetBase ("10.1.0.0", "255.255.255.0");
  address.SetBase ("10.2.0.0", "255.255.255.0");//swd

  //configuring IP Addresses for IntraPlane devices
  for(uint32_t i=0; i< this->intra_plane_devices.size(); i++)
  {
    address.NewNetwork();
    this->intra_plane_interfaces.push_back(address.Assign(this->intra_plane_devices[i]));
    //std::cout<<" "<<intra_plane_interfaces[i].GetAddress(0)<<" "<<intra_plane_interfaces[i].GetAddress(1)<<std::endl;//swd
  }
  
  //configuring IP Addresses for InterPlane devices
  for(uint32_t i=0; i< this->inter_plane_devices.size(); i++)
  {
    address.NewNetwork();
    this->inter_plane_interfaces.push_back(address.Assign(this->inter_plane_devices[i]));
    //std::cout<<" "<<inter_plane_interfaces[i].GetAddress(0)<<" "<<inter_plane_interfaces[i].GetAddress(1)<<std::endl;//swd
  }

  //configuring IP Addresses for Ground devices
  for(uint32_t i =0;i< this->ground_station_devices.size();i++)
  {
	  address.NewNetwork();
	  this->ground_station_interfaces.push_back(address.Assign(this->ground_station_devices[i]));
  }

  //Populate Routing Tables
 //std::cout<<"Populating Routing Tables"<<std::endl;
 // Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
  //Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("dynamic-global-routing.routes", std::ios::out);
  //Ipv4GlobalRoutingHelper::PrintRoutingTableAllEvery(Seconds(2.0), routingStream,  Time::S);
  //std::cout<<"Finished Populating Routing Tables"<<std::endl;
  PrintGlobalNetInfo();
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
			Vector nodeBPos = this->plane[(i+1)%num_planes].Get(nodeBIndex)->GetObject<MobilityModel>()->GetPosition();
			double distance = CalculateDistance(nodeAPos, nodeBPos);
			double delay = (distance*1000)/speed_of_light;

		    if(nodeAPos.x < -70 || nodeAPos.x >70 || nodeBPos.x < -70 || nodeBPos.x > 70) // swd
		    {
		    	pr = 1;
		    	this->inter_plane_devices[i*num_satellites_per_plane+j].Get(0)->SetAttribute("DataRate", StringValue("1Gbps"));
		    	this->inter_plane_devices[i*num_satellites_per_plane+j].Get(1)->SetAttribute("DataRate", StringValue("1Gbps"));
		    	this->inter_plane_channels[i*num_satellites_per_plane+j]->SetAttribute("Delay", TimeValue(Seconds(delay)));
		    	std::cout<<"Channel can not open between plane "<<i<<" satellite "<<j<<" and plane "<<(i+1)%num_planes<<" satellite "<<nodeBIndex<< std::endl;
		    }
		    else
		    {
		    	pr = 0;
		    	this->inter_plane_devices[i*num_satellites_per_plane+j].Get(0)->SetAttribute("DataRate", StringValue("50Gbps"));
		        this->inter_plane_devices[i*num_satellites_per_plane+j].Get(1)->SetAttribute("DataRate", StringValue("50Gbps"));
		    	this->inter_plane_channels[i*num_satellites_per_plane+j]->SetAttribute("Delay", TimeValue(Seconds(delay)));
		    	std::cout<<"Channel open between plane "<<i<<" satellite "<<j<<" and plane "<<(i+1)%num_planes<<" satellite "<<nodeBIndex<< " with distance "<<distance<< "km and delay of "<<delay<<" seconds"<<std::endl;
		    }
		    this->inter_pr[i*j+j] = pr;
		}
	}

	std::cout<<"Updating links between ground stations and their closest satellites:"<<std::endl;
	//updating links between ground stations and their closest satellites

	std::cout<<"The location of all stations:"<<std::endl;
	for(uint32_t i=0;i<2;i++)
	{
		Vector StationPos = this->ground_stations.Get(i)->GetObject<MobilityModel>()->GetPosition();
		std::cout << Simulator::Now().GetSeconds() << ": station # "<< i << ": x = " << StationPos.x << ", y = " << StationPos.y << ", z = " << StationPos.z << std::endl;
	}


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
		std::cout<<"Ground_station: "<<kk<<" <-> "<<"Satellite: "<<"("<<min_i<<","<<min_j<<")"<<std::endl;
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




//	for (uint32_t i=0; i<2; i++)
//	{
//		Vector gndPos = ground_stations.Get(i)->GetObject<MobilityModel> ()->GetPosition();
//	    uint32_t closestAdjSat = 0;
//	    uint32_t closestAdjSatDist = 0;
//	    uint32_t planeIndex;
//	    if (i == 0)
//	      planeIndex = 0;
//	    else
//	      planeIndex = floor(3*num_planes/7);
//	    //find closest adjacent satellite for ground station
//	    for (uint32_t j=0; j<this->num_satellites_per_plane; j++)
//	    {
//	      Vector pos = this->plane[planeIndex].Get(j)->GetObject<MobilityModel>()->GetPosition();
//	      double temp_dist = CalculateDistanceGroundToSat(gndPos,pos);
//	      if((temp_dist < closestAdjSatDist) || (j==0))
//	      {
//	        closestAdjSatDist = temp_dist;
//	        closestAdjSat = j;
//	      }
//	    }
//
//	    uint32_t currAdjNodeID = this->ground_station_channel_tracker[i];
//	    if(currAdjNodeID == closestAdjSat)
//	    {
//	      double new_delay = (closestAdjSatDist*1000)/speed_of_light;
//	      this->ground_station_channels[i]->SetAttribute("Delay", TimeValue(Seconds(new_delay)));
//	      std::cout<<"Channel updated between ground station : "<<i<<" and plane : "<<planeIndex<<" satellite : "<<closestAdjSat<< " with distance "<<closestAdjSatDist<< "km and delay of "<<new_delay<<" seconds"<<std::endl;
//	    }
//	    else
//	    {
//	    	this->ground_station_channels[i]->Detach(this->ground_station_devices[i].Get(currAdjNodeID+1)->GetObject<CsmaNetDevice> ());
//	        std::pair< Ptr< Ipv4 >, uint32_t> interface = this->ground_station_interfaces[i].Get(currAdjNodeID+1);
//	        interface.first->SetDown(interface.second);
//	        this->ground_station_channels[i]->Reattach(this->ground_station_devices[i].Get(closestAdjSat+1)->GetObject<CsmaNetDevice> ());
//	        interface = this->ground_station_interfaces[i].Get(closestAdjSat+1);
//	        interface.first->SetUp(interface.second);
//	        this->ground_station_channel_tracker[i] = closestAdjSat;
//	        double new_delay = (closestAdjSatDist*1000)/speed_of_light;
//	        this->ground_station_channels[i]->SetAttribute("Delay", TimeValue(Seconds(new_delay)));
//	        std::cout<<"New channel between ground station : "<<i<<" and plane : "<<planeIndex<<" satellite : "<<closestAdjSat<< " with distance "<<closestAdjSatDist<< "km and delay of "<<new_delay<<" seconds"<<std::endl;
//	    }
//	}

	//Recompute Routing Tables
//	std::cout<<"Recomputing Routing Tables"<<std::endl;
//	Ipv4GlobalRoutingHelper::RecomputeRoutingTables ();
//	std::cout<<"Finished Recomputing Routing Tables"<<std::endl;
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


void LeoSatelliteConfig::OutputFlowInformation(Ptr<FlowMonitor> flowmon, FlowMonitorHelper &flowmonHelper)
{
	  //flowmon->SerializeToXmlFile ((tr_name + ".flowmon").c_str(), false, false);//swd
	  flowmon->CheckForLostPackets ();
	      	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmonHelper.GetClassifier ());
	              std::map<FlowId, FlowMonitor::FlowStats> stats = flowmon->GetFlowStats ();

	              uint32_t txPacketsum = 0;
	              uint32_t rxPacketsum = 0;
	              uint32_t DropPacketsum = 0;
	              uint32_t LostPacketsum = 0;
	              double Delaysum = 0;

	              std::cout<< "**********************************"<<std::endl;
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
}


}

