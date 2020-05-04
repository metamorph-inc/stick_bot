#include <string>
#include <math.h>
#include <thread>
#include <boost/assign/list_of.hpp>
#include <boost/algorithm/string.hpp> 
#include "Eigen/Dense"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SendCommandWithAcknowledgementSrv.h"
#include "hebiros/CommandMsg.h"


// Global variables
// TODO: Should probably rewrite this as a class to avoid globals.
std::vector<double> g_position_fbk;
double g_wrist_vel_target = 0.0;
double g_spool_pos_target = 0.0;

// Function declarations
std::vector<std::string> getHebiFamilyNameFromMapping(const std::string &hebi_mapping);

void wrist_twist_callback(const geometry_msgs::TwistConstPtr& twist);

void spool_twist_callback(const geometry_msgs::TwistConstPtr& twist);

void feedback_callback(const sensor_msgs::JointState::ConstPtr& js);


int main(int argc, char *argv[])
{
  // ROS node setup
  ros::init(argc, argv, "stick_bot_gripper");
  ros::NodeHandle nh, private_nh("~");

  // ROS parameters
  std::string hebi_mapping_wrist_str;
  std::string hebi_mapping_spool_str;
  private_nh.param<std::string>("hebi_mapping_wrist", hebi_mapping_wrist_str, "Gripper/Wrist");
  private_nh.param<std::string>("hebi_mapping_spool", hebi_mapping_spool_str, "Gripper/Spool");

  // ROS publishers & subscribers
  ros::Rate r(200); // 200 hz
  ros::Subscriber sub_wrist = nh.subscribe("gripper/wrist/cmd_vel", 1, wrist_twist_callback);

  // Get HEBI families and names
  std::vector<std::string> hebi_mapping;
  std::vector<std::string> hebi_mapping_wrist;
  std::vector<std::string> hebi_mapping_spool;
  std::vector<std::string> hebi_families;
  std::vector<std::string> hebi_names;
  hebi_mapping = {hebi_mapping_wrist};
  hebi_mapping_wrist = getHebiFamilyNameFromMapping(hebi_mapping_wrist_str);
  hebi_families.push_back(hebi_mapping_wrist[0]);
  hebi_names.push_back(hebi_mapping_wrist[1]);

  // Look up HEBI modules
  std::string hebi_group_name = "hebi_gripper";

  //Create a client which uses the service to create a group
  ros::ServiceClient add_group_client = nh.serviceClient<hebiros::AddGroupFromNamesSrv>(
    "/hebiros/add_group_from_names");

  ros::ServiceClient send_command_with_acknowledgement = nh.serviceClient<hebiros::SendCommandWithAcknowledgementSrv>(
    "/hebiros/"+hebi_group_name+"/send_command_with_acknowledgement");

  //Create a subscriber to receive feedback from a group
  //Register feedback_callback as a callback which runs when feedback is received
  ros::Subscriber feedback_subscriber = nh.subscribe<sensor_msgs::JointState>(
    "/hebiros/"+hebi_group_name+"/feedback/joint_state", 100, &feedback_callback);

  //Create a publisher to send desired commands to a group
  ros::Publisher hebi_publisher = nh.advertise<sensor_msgs::JointState>(
    "/hebiros/"+hebi_group_name+"/command/joint_state", 100);

  sensor_msgs::JointState cmd_msg;
  cmd_msg.name = hebi_mapping;

  hebiros::AddGroupFromNamesSrv add_group_srv;
  add_group_srv.request.group_name = hebi_group_name;
  add_group_srv.request.families = hebi_families;
  add_group_srv.request.names = hebi_names;
  while(!add_group_client.call(add_group_srv)) {}

  // Wait for HEBIROS Feedback
  while(g_position_fbk.empty())
  {
    r.sleep();
    ros::spinOnce();
  }

  std::vector<double> cmd_positions = g_position_fbk;
  double wrist_velocity_target;
  double wrist_position_target;
  double spool_position_target;

  // main loop
  while (ros::ok())
  {
    // get velocity target
    wrist_velocity_target = g_wrist_vel_target;

    // calculate position target
    wrist_position_target += (wrist_velocity_target * r.expectedCycleTime().toSec());

    // send position command to hardware
    // TODO: Maybe mix in velocity command
    cmd_positions[0] = wrist_position_target;
    cmd_msg.position = cmd_positions;
    hebi_publisher.publish(cmd_msg);

    ros::spinOnce();
    r.sleep();
  }

    return 0;
}

// Function definitions
std::vector<std::string> getHebiFamilyNameFromMapping(const std::string &hebi_mapping_string)
{
  std::vector<std::string> hebi_mapping_vector;
  boost::split(hebi_mapping_vector, hebi_mapping_string, boost::is_any_of("/"));
  
  return hebi_mapping_vector;
}

void wrist_twist_callback(const geometry_msgs::TwistConstPtr& twist)
{
  g_wrist_vel_target = twist->angular.z;
}

void spool_twist_callback(const geometry_msgs::TwistConstPtr& twist)
{
  g_spool_pos_target = twist->angular.z;
}

void feedback_callback(const sensor_msgs::JointState::ConstPtr& js)
{
  g_position_fbk = js->position;
}