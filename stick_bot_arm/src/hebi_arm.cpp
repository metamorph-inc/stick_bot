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
double g_shoulder_vel_target = 0.0;
double g_elbow_vel_target = 0.0;

// Function declarations
std::vector<std::string> getHebiFamilyNameFromMapping(const std::string &hebi_mapping);

void shoulder_twist_callback(const geometry_msgs::TwistConstPtr& twist);

void elbow_twist_callback(const geometry_msgs::TwistConstPtr& twist);

void feedback_callback(const sensor_msgs::JointState::ConstPtr& js);


int main(int argc, char *argv[])
{
  // ROS node setup
  ros::init(argc, argv, "stick_bot_arm");
  ros::NodeHandle nh, private_nh("~");

  // ROS parameters
  std::string hebi_mapping_shoulder_str;
  std::string hebi_mapping_elbow_str;
  std::string hebi_gains_fname;
  double rotate_min;
  double rotate_max;
  private_nh.param<std::string>("hebi_mapping_shoulder", hebi_mapping_shoulder_str, "Arm/Shoulder");
  private_nh.param<std::string>("hebi_mapping_elbow", hebi_mapping_elbow_str, "Arm/Elbow");
  private_nh.param<std::string>("hebi_gains_fname", hebi_gains_fname, "gains/arm/arm_2d.xml");
  private_nh.param<double>("rotate_min", rotate_min, -M_PI);
  private_nh.param<double>("rotate_max", rotate_max, M_PI);

  // ROS publishers & subscribers
  ros::Rate r(200); // 200 hz
  ros::Subscriber sub_shoulder = nh.subscribe("arm/shoulder/cmd_vel", 1, shoulder_twist_callback);
  ros::Subscriber sub_elbow = nh.subscribe("arm/elbow/cmd_vel", 1, elbow_twist_callback);

  // Get HEBI families and names
  std::vector<std::string> hebi_mapping;
  std::vector<std::string> hebi_mapping_shoulder;
  std::vector<std::string> hebi_mapping_elbow;
  std::vector<std::string> hebi_families;
  std::vector<std::string> hebi_names;
  hebi_mapping = {hebi_mapping_shoulder_str, hebi_mapping_elbow_str};
  hebi_mapping_shoulder = getHebiFamilyNameFromMapping(hebi_mapping_shoulder_str);
  hebi_mapping_elbow = getHebiFamilyNameFromMapping(hebi_mapping_elbow_str);
  hebi_families.push_back(hebi_mapping_shoulder[0]);
  hebi_families.push_back(hebi_mapping_elbow[0]);
  hebi_names.push_back(hebi_mapping_shoulder[1]);
  hebi_names.push_back(hebi_mapping_elbow[1]);

  // Look up HEBI modules
  std::string hebi_group_name = "hebi_arm";

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

  // Set HEBI Gains
  hebiros::SendCommandWithAcknowledgementSrv send_cmd_w_ack_srv;
  send_cmd_w_ack_srv.request.command.name = hebi_mapping;
  send_cmd_w_ack_srv.request.command.settings.name = hebi_mapping;
  send_cmd_w_ack_srv.request.command.settings.control_strategy = {4, 4, 4, 4};
  send_cmd_w_ack_srv.request.command.settings.position_gains.kp = {24, 45};
  send_cmd_w_ack_srv.request.command.settings.velocity_gains.kp = {0.05, 0.03};
  send_command_with_acknowledgement.call(send_cmd_w_ack_srv);

  // Wait for HEBIROS Feedback
  while(g_position_fbk.empty())
  {
    r.sleep();
    ros::spinOnce();
  }

  std::vector<double> cmd_positions = {0.0, 0.0};
  double shoulder_velocity_target;
  double shoulder_position_target = g_position_fbk[0];
  double elbow_velocity_target;
  double elbow_position_target;
  double elbow_offset_target = 0.0;

  // main loop
  while (ros::ok())
  {
    // get velocity target
    shoulder_velocity_target = g_shoulder_vel_target;
    elbow_velocity_target = g_elbow_vel_target;

    // calculate position target
    shoulder_position_target += (shoulder_velocity_target * r.expectedCycleTime().toSec());
    shoulder_position_target = fmax(rotate_min, fmin(rotate_max, shoulder_position_target));

    elbow_offset_target += (elbow_velocity_target * r.expectedCycleTime().toSec());
    elbow_position_target = (shoulder_position_target + (M_PI/2.0)) + elbow_offset_target;

    // send position command to hardware
    // TODO: Maybe mix in velocity command
    cmd_positions[0] = shoulder_position_target;
    cmd_positions[1] = elbow_position_target;
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

void shoulder_twist_callback(const geometry_msgs::TwistConstPtr& twist)
{
  g_shoulder_vel_target = twist->angular.z;
}

void elbow_twist_callback(const geometry_msgs::TwistConstPtr& twist)
{
  g_elbow_vel_target = twist->angular.z;
}

void feedback_callback(const sensor_msgs::JointState::ConstPtr& js)
{
  g_position_fbk = js->position;
}