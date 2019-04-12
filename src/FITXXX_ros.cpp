/*
 * FITXXX ROS.cpp
 * Author: Zhigang Wu
 * Date: 2018-01-12
*/

#include <csignal>
#include <cstdio>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "FITXXX/FITXXX.h"
#include <std_srvs/Trigger.h>


class FITXXX_ros_node
{

private:
  ros::NodeHandle node_;

  // Subscriber
  //ros::Subscriber topic_;

  // Publisher
  ros::Publisher laser_scan_pub_;

  // Service
  //ros::ServiceServer connect_laser_service_;
  ros::ServiceServer disconnect_laser_service_;
  ros::ServiceServer start_laser_service_;
  ros::ServiceServer stop_laser_service_;

  // Params
  double scan_range_min_;
  double scan_range_max_;
  double angle_min_;
  double angle_max_;
  int port_;
  int scan_seq_;

  std::string host_ip_;
  std::string frame_;
  std::string frame_id_;

  // Laser status
  bool connect_status_;
  bool get_config_;
  bool scanParamInitialized_;
  FITXXX laser;

  // Laser config
  ULDINI_Type uld_config_;
  // Laser message
  sensor_msgs::LaserScan scan_msg_;

  ros::Time HB_time;

public:

  FITXXX_ros_node();
  ~FITXXX_ros_node();

  void initialize_scan_param();
  bool connect();
  bool getConfig();
  void startMesure();
  void sendHB2Lidar();
  void check_valid_scan_data();
  bool connect_laser(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool disconnect_laser(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool stop_scan_msg(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool start_scan_msg(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  void update();
};

FITXXX_ros_node::FITXXX_ros_node()
: scan_seq_(0)
, connect_status_(false)
, get_config_(true)
, scanParamInitialized_(false)
{
  // parameters
  ros::param::get("/fitxxx/scan_range_min", scan_range_min_);
  ros::param::get("/fitxxx/scan_range_max", scan_range_max_);

  ros::param::get("/fitxxx/angle_min", angle_min_);
  ros::param::get("/fitxxx/angle_max", angle_max_);

  ros::param::get("/fitxxx/frame", frame_id_);

  ros::param::get("/fitxxx/host_ip", host_ip_);
  ros::param::get("/fitxxx/port", port_);

  laser.setScanAngles(angle_min_, angle_max_);
  
  // subscribers

  // publishers
  laser_scan_pub_ = node_.advertise<sensor_msgs::LaserScan>("/scan", 1);

  // Services
  //connect_laser_service_ = node_.advertiseService("/fitxxx/connect_laser_srv", &FITXXX_ros_node::connect_laser, this);
  disconnect_laser_service_ = node_.advertiseService("/fitxxx/disconnect_laser_srv", &FITXXX_ros_node::disconnect_laser, this);
  start_laser_service_ = node_.advertiseService("/fitxxx/start_laser_srv", &FITXXX_ros_node::start_scan_msg, this);
  stop_laser_service_ = node_.advertiseService("/fitxxx/stop_laser_srv", &FITXXX_ros_node::stop_scan_msg, this);
  
}

FITXXX_ros_node::~FITXXX_ros_node()
{
    laser.stopMeas();
    laser.disconnect();
}

// load from laser config
void FITXXX_ros_node::initialize_scan_param()
{
    scan_msg_.header.frame_id = frame_id_;
    scan_msg_.range_min = 0;
    scan_msg_.range_max = uld_config_.nMR/ 100.;
    scan_msg_.angle_min = angle_min_;
    scan_msg_.angle_max = angle_max_;

    ROS_INFO("scan_range_min: %f", scan_msg_.range_min);
    ROS_INFO("scan_range_max: %f", scan_msg_.range_max);
    ROS_INFO("angle_min: %f", scan_msg_.angle_min);
    ROS_INFO("angle_max: %f", scan_msg_.angle_max);
    ROS_INFO("lase RPM: %d", uld_config_.nSAV);
    ROS_INFO("lase nSAP: %d", uld_config_.nSAP);  
    ROS_INFO("lase nPF: %d", uld_config_.nPF);  

    scanParamInitialized_ = true;
}

// Connect laser.
bool FITXXX_ros_node::connect()
{
    ROS_INFO_STREAM("Connecting to laser at " << host_ip_);
    laser.connect(host_ip_, port_);
    if (!laser.isConnected())
    {
      ROS_WARN("Unable to connect, retrying.");
      ros::Duration(1).sleep();
      return false;
    }

    connect_status_ = true;
    ROS_INFO("Connected to laser.");
    return true;
}

// Get Laser config.
bool FITXXX_ros_node::getConfig()
{
  return laser.getConfig();
}

// Start continous laser scan msgs
void FITXXX_ros_node::startMesure()
{
  laser.startMeas();
}

// Send Heart Beat to laser.
void FITXXX_ros_node::sendHB2Lidar()
{
  laser.sendHB();
}

// service: Connect laser.
/*bool FITXXX_ros_node::connect_laser(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
    resp.success = true;
    resp.message = "Disconnect Laser.";
    connect();
    return resp.success;
}*/

// service: Disconnect laser.
bool FITXXX_ros_node::disconnect_laser(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
    resp.success = true;
    resp.message = "Disconnect Laser.";
    laser.disconnect();
    return resp.success;
}

// service: start laser.
bool FITXXX_ros_node::start_scan_msg(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
    resp.success = true;
    resp.message = "Start receiving LaserScan msg.";
    startMesure();
    return resp.success;
}

// service: Stop laser.
bool FITXXX_ros_node::stop_scan_msg(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
    resp.success = true;
    resp.message = "Stop receiving LaserScan msg.";
    laser.stopMeas();
    return resp.success;
}

void FITXXX_ros_node::update()
{
  if(!connect_status_)
  {
    ROS_INFO("connect----------");
    connect();
  }
  else
  {
    // Send laser configuration request.
    if(get_config_)
    {
      getConfig();
      ROS_INFO("SEND: Request laser config ...");
      get_config_ = false;
    }

    // if receive packet from tcp
    if(laser.packetDecodeExt(&uld_config_))
    {
       // decode laser packet.
       while(laser.GetALim(&uld_config_, &scan_msg_))
       {     
          if(laser.initializedLaserConfig())
          {
            ros::Time scan_time = ros::Time::now();

            // if not intialized laser config.
            if(!scanParamInitialized_)
            {
              initialize_scan_param();
              startMesure();
              HB_time = ros::Time::now();
            }
            else
            {
              if(scan_time.toSec() - HB_time.toSec() > 5.0)
              {
                // Send Heart beat to laser every 5 sec.
                sendHB2Lidar();
                HB_time = scan_time;
              }
            }
        
            scan_msg_.header.stamp = scan_time;
            ++scan_msg_.header.seq;
        
            // publish sensor_msgs/LaserScan
            laser_scan_pub_.publish(scan_msg_);         
          }
      }
    }
  }
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "FITXXX_ros_node");
  FITXXX_ros_node node;

  ROS_INFO("FIT laser driver node.");

  //ros::Rate rate(30);

  while(ros::ok())
  {
    ros::spinOnce();
    node.update();
    //rate.sleep();
  }
  return 0;
}
