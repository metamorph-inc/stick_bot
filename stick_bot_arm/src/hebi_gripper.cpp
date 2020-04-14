#include <string>
#include <math.h>
#include <thread>
#include <boost/assign/list_of.hpp>
#include <boost/algorithm/string.hpp> 
#include "Eigen/Dense"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"


// Global variables
// TODO: Should probably rewrite this as a class to avoid globals.
double g_wrist_vel_target = 0.0;
double g_spool_pos_target = 0.0;

// Function declarations
std::vector<std::string> getHebiFamilyNameFromMapping(const std::string &hebi_mapping);

void setHebiGainsFromFile(const std::string &hebi_gains_fname, hebi::Group &hebi_group);

void printHebiLookup(hebi::Lookup &hebi_lookup);

void wrist_twist_callback(const geometry_msgs::TwistConstPtr& twist);

void spool_twist_callback(const geometry_msgs::TwistConstPtr& twist);


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
  ros::Subscriber sub_spool = nh.subscribe("gripper/spool/cmd_pos", 1, spool_twist_callback);

  // Get HEBI families and names
  std::vector<std::string> hebi_mapping_wrist;
  std::vector<std::string> hebi_mapping_spool;
  std::vector<std::string> hebi_families;
  std::vector<std::string> hebi_names;
  hebi_mapping_wrist = getHebiFamilyNameFromMapping(hebi_mapping_wrist_str);
  hebi_mapping_spool = getHebiFamilyNameFromMapping(hebi_mapping_spool_str);
  hebi_families.push_back(hebi_mapping_wrist[0]);
  hebi_families.push_back(hebi_mapping_spool[0]);
  hebi_names.push_back(hebi_mapping_wrist[1]);
  hebi_names.push_back(hebi_mapping_spool[1]);

  // Look up HEBI modules
  hebi::Lookup lookup;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  printHebiLookup(lookup);
  std::shared_ptr<hebi::Group> hebi_group;
  while (ros::ok() && !hebi_group)
  {
    hebi_group = lookup.getGroupFromNames(hebi_families, hebi_names, 10000);
    ROS_WARN("Looking for HEBI Group...");
    r.sleep();
  }
  ROS_WARN("Found HEBI Group.");

  // HEBI Command and Feedback
  constexpr double feedback_frequency = 200;
  hebi_group->setFeedbackFrequencyHz(feedback_frequency);
  std::shared_ptr<hebi::GroupFeedback> hebi_feedback;
  hebi_feedback = std::make_shared<hebi::GroupFeedback>(hebi_group->size());
  constexpr long command_lifetime = 250;
  hebi_group->setCommandLifetimeMs(command_lifetime);
  std::shared_ptr<hebi::GroupCommand> hebi_command;
  hebi_command = std::make_shared<hebi::GroupCommand>(hebi_group->size());

  // Try to get HEBI feedback
  constexpr int max_attempts = 20;
  int num_attempts = 0;
  while (!hebi_group->getNextFeedback(*hebi_feedback)) {
    if (num_attempts++ > max_attempts) {
      ROS_ERROR("Unable to get HEBI Group Feedback after %s attempts...", std::to_string(num_attempts).c_str());
      // do nothing... for now;
    }
  }

  bool feedback_received = false;
  Eigen::VectorXd cmd_positions = Eigen::VectorXd(hebi_group->size()).setZero();
  double wrist_velocity_target;
  double wrist_position_target;
  double spool_position_target;

  // main loop
  while (ros::ok())
  {
    // get hebi feedback
    if (hebi_group->getNextFeedback(*hebi_feedback))
    {
      feedback_received = true;
    } else {
      feedback_received = false;
    }

    if (feedback_received)
    {
      // get velocity target
      wrist_velocity_target = g_wrist_vel_target;
      spool_position_target = g_spool_pos_target;

      // calculate position target
      wrist_position_target += (wrist_velocity_target * r.expectedCycleTime().toSec());

      // send position command to hardware
      // TODO: Maybe mix in velocity command
      cmd_positions[0] = wrist_position_target;
      cmd_positions[1] = spool_position_target;
      hebi_command->setPosition(cmd_positions);
      hebi_group->sendCommand(*hebi_command);
    }

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

void setHebiGainsFromFile(const std::string &hebi_gains_fname, hebi::Group &hebi_group)
{
  hebi::GroupCommand gains_command(hebi_group.size());
  if (!gains_command.readGains(hebi_gains_fname)) {
    ROS_WARN("Could not read %s", hebi_gains_fname.c_str());
  } else {
    if (!hebi_group.sendCommandWithAcknowledgement(gains_command)) {
      ROS_WARN("Could not send gains");
    }
  }
}

void printHebiLookup(hebi::Lookup &hebi_lookup)
{
  // Print snapshot to screen
  auto entry_list = hebi_lookup.getEntryList();
  ROS_INFO("Modules found on network (Family | Name):");
  for (auto entry : *entry_list)
  {
    ROS_INFO("%s | %s", entry.family_.c_str(), entry.name_.c_str());
  }
}

void wrist_twist_callback(const geometry_msgs::TwistConstPtr& twist)
{
  g_wrist_vel_target = twist->angular.z;
}

void spool_twist_callback(const geometry_msgs::TwistConstPtr& twist)
{
  g_spool_pos_target = twist->angular.z;
}