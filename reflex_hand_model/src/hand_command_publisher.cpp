#include <ros/ros.h>
#include <ihmc_msgs/HandDesiredConfigurationRosMessage.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

ros::Publisher *left_hand_msg_pub, *right_hand_msg_pub;
std_msgs::Float64MultiArray hand_msg;
const std::vector<double> CLOSE_HAND_BASIC = { 2.95, 2.95, 2.95, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5 };
const std::vector<double> OPEN_HAND_BASIC = { 0.05, 0.05, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

void callback(ihmc_msgs::HandDesiredConfigurationRosMessage hand_desired_config)
{
  ros::Publisher* publisher;
  if (hand_desired_config.robot_side == ihmc_msgs::HandDesiredConfigurationRosMessage::LEFT)
  {
    publisher = left_hand_msg_pub;
  }
  else if (hand_desired_config.robot_side == ihmc_msgs::HandDesiredConfigurationRosMessage::RIGHT)
  {
    publisher = right_hand_msg_pub;
  }
  else
  {
    return;
  }

  switch (hand_desired_config.hand_desired_configuration)
  {
    case ihmc_msgs::HandDesiredConfigurationRosMessage::CLOSE:
      hand_msg.data = CLOSE_HAND_BASIC;
      break;
    case ihmc_msgs::HandDesiredConfigurationRosMessage::OPEN:
      hand_msg.data = OPEN_HAND_BASIC;
      break;
    default:
      return;
  }
  publisher->publish(hand_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_message_relay");
  ros::NodeHandle nh;

  hand_msg.layout.dim.resize(1);
  hand_msg.layout.dim[0].size = 9;
  hand_msg.layout.dim[0].stride = 1;
  hand_msg.layout.dim[0].label = "relay";

  ros::Publisher publish_left =
      nh.advertise<std_msgs::Float64MultiArray>("/reflex_hands/left_hand_controller/command", 1);
  left_hand_msg_pub = &publish_left;

  ros::Publisher publish_right =
      nh.advertise<std_msgs::Float64MultiArray>("/reflex_hands/right_hand_controller/command", 1);
  right_hand_msg_pub = &publish_right;

  ros::Subscriber subscribe = nh.subscribe("/ihmc_ros/atlas/control/hand_desired_configuration", 10, callback);

  ros::spin();
  return 0;
}