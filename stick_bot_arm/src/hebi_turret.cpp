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


double VELOCITY_SCALAR = 100.0;

// Global variables
// TODO: Should probably rewrite this as a class to avoid globals.
std::vector<double> g_position_fbk;
double g_turret_vel_target = 0.0;

// Function declarations
std::vector<std::string> getHebiFamilyNameFromMapping(const std::string &hebi_mapping);

void twist_callback(const geometry_msgs::TwistConstPtr& twist);

void feedback_callback(const sensor_msgs::JointState::ConstPtr& js);


int main(int argc, char *argv[])
{
  // ROS node setup
  ros::init(argc, argv, "stick_bot_turret");
  ros::NodeHandle nh, private_nh("~");

  // ROS parameters
  std::string hebi_mapping_str;
  std::string hebi_gains_fname;
  double rotate_min;
  double rotate_max;
  private_nh.param<std::string>("hebi_mapping", hebi_mapping_str, "Turret/Base");
  private_nh.param<std::string>("hebi_gains_fname", hebi_gains_fname, "gains/turret/x8_16.xml");
  private_nh.param<double>("rotate_min", rotate_min, -M_PI);
  private_nh.param<double>("rotate_max", rotate_max, M_PI);

  // ROS publishers & subscribers
  ros::Rate r(200); // 200 hz
  ros::Subscriber sub = nh.subscribe("turret/cmd_vel", 1, twist_callback);

  // Get HEBI families and names
  std::vector<std::string> hebi_mapping = {hebi_mapping_str};
  std::vector<std::string> hebi_mapping_turret;
  std::vector<std::string> hebi_families;
  std::vector<std::string> hebi_names;
  hebi_mapping_turret = getHebiFamilyNameFromMapping(hebi_mapping_str);
  hebi_families.push_back(hebi_mapping_turret[0]);
  hebi_names.push_back(hebi_mapping_turret[1]);

  // Look up HEBI modules
  std::string hebi_group_name = "hebi_turret";

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
  send_cmd_w_ack_srv.request.command.settings.control_strategy = {4};
  send_cmd_w_ack_srv.request.command.settings.position_gains.kp = {5};
  send_cmd_w_ack_srv.request.command.settings.velocity_gains.kp = {0.03};
  send_command_with_acknowledgement.call(send_cmd_w_ack_srv);

  // Wait for HEBIROS Feedback
  while(g_position_fbk.empty())
  {
    r.sleep();
    ros::spinOnce();
  }

  std::vector<double> cmd_positions = g_position_fbk;
  double velocity_target;
  double position_target;

  // main loop
  while (ros::ok())
  {
    // get velocity target
    velocity_target = g_turret_vel_target;

    // calculate position target
    position_target = g_position_fbk[0] + (VELOCITY_SCALAR * velocity_target * r.expectedCycleTime().toSec());
    position_target = fmax(rotate_min, fmin(rotate_max, position_target));

    // send position command to hardware
    // TODO: Maybe mix in velocity command
    cmd_positions[0] = position_target;
    cmd_msg.position = cmd_positions;
    //cmd_velocities[0] = velocity_target;
    //hebi_command->setVelocity(cmd_velocities);
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

void twist_callback(const geometry_msgs::TwistConstPtr& twist)
{
  g_turret_vel_target = twist->angular.z;
}

void feedback_callback(const sensor_msgs::JointState::ConstPtr& js)
{
  g_position_fbk = js->position;
}