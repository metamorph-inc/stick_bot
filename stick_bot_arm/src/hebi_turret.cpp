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


double VELOCITY_SCALAR = 100.0;

// Global variables
// TODO: Should probably rewrite this as a class to avoid globals.
double g_turret_vel_target = 0.0;

// Function declarations
std::vector<std::string> getHebiFamilyNameFromMapping(const std::string &hebi_mapping);

void setHebiGainsFromFile(const std::string &hebi_gains_fname, hebi::Group &hebi_group);

void printHebiLookup(hebi::Lookup &hebi_lookup);

void twist_callback(const geometry_msgs::TwistConstPtr& twist);


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
  std::vector<std::string> hebi_mapping;
  std::vector<std::string> hebi_families;
  std::vector<std::string> hebi_names;
  hebi_mapping = getHebiFamilyNameFromMapping(hebi_mapping_str);
  hebi_families.push_back(hebi_mapping[0]);
  hebi_names.push_back(hebi_mapping[1]);

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

  // Set HEBI Gains from file
  setHebiGainsFromFile(hebi_gains_fname, *hebi_group);

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
  Eigen::VectorXd hebi_positions;
  Eigen::VectorXd cmd_positions = Eigen::VectorXd(hebi_group->size()).setZero();
  //Eigen::VectorXd cmd_velocities = Eigen::VectorXd(hebi_group->size()).setZero();
  double velocity_target;
  double position_target;

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
      // get current position
      hebi_positions = hebi_feedback->getPosition();

      // get velocity target
      velocity_target = g_turret_vel_target;

      // calculate position target
      position_target = hebi_positions[0] + (VELOCITY_SCALAR * velocity_target * r.expectedCycleTime().toSec());
      position_target = fmax(rotate_min, fmin(rotate_max, position_target));

      // send position command to hardware
      // TODO: Maybe mix in velocity command
      cmd_positions[0] = position_target;
      hebi_command->setPosition(cmd_positions);
      //cmd_velocities[0] = velocity_target;
      //hebi_command->setVelocity(cmd_velocities);
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

void twist_callback(const geometry_msgs::TwistConstPtr& twist)
{
  g_turret_vel_target = twist->angular.z;
}