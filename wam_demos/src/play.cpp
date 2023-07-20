#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <thread>
#include <stdlib.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"
#include "wam_msgs/RTJointPos.h"
#include "wam_msgs/RTJointVel.h"
#include "wam_srvs/Hold.h"
#include "wam_srvs/JointMove.h"

void playJointPositions(std::string bag_name, ros::Publisher rt_joint_position_publisher,
          ros::ServiceClient joint_position_move_client) {
  rosbag::Bag wam_bag;
  wam_bag.open(bag_name);
  ros::Time current_time = ros::Time::now();
  ros::Time prevTime = current_time;
  ros::Time current_msg_time;
  ros::Time prev_msg_time;
  bool first_check = true;
  for (rosbag::MessageInstance const message_instance : rosbag::View(wam_bag)) {
    ros::Time current_time = ros::Time::now();
    current_msg_time = message_instance.getTime();
    if (first_check) {
      wam_srvs::JointMove wam_jp_srv;
      sensor_msgs::JointState::ConstPtr start_joint_position = message_instance.instantiate<sensor_msgs::JointState>();
      std::vector<float> joints(start_joint_position->position.begin(), start_joint_position->position.end());
      wam_jp_srv.request.joints = joints;
      ROS_INFO("Moving WAM to start pose");
      if (!joint_position_move_client.call(wam_jp_srv)) { //move to start position at first check
        ROS_ERROR("Could not move to start Pose");
        exit(0);
      }
      first_check = false;
      prev_msg_time = current_msg_time;
    }
    wam_msgs::RTJointPos::ConstPtr rt_bag_msg =
        message_instance.instantiate<wam_msgs::RTJointPos>();
    if (rt_bag_msg != nullptr) {
      while ((current_time - prevTime).toSec() <
             (current_msg_time - prev_msg_time).toSec()) {  // wait until (current_msg_time - prev_msg_time) has passed
        current_time = ros::Time::now();
      }
      prev_msg_time = current_msg_time;
      prevTime = current_time;
      rt_joint_position_publisher.publish(rt_bag_msg);
    }
  }
  wam_bag.close();
}

void playJointVelocities(std::string bag_name, ros::Publisher rt_jv_pub,
          ros::ServiceClient joint_position_move_client) {
  rosbag::Bag wam_bag;
  wam_bag.open(bag_name);
  ros::Time current_time = ros::Time::now();
  ros::Time prevTime = current_time;
  ros::Time current_msg_time;
  ros::Time prev_msg_time;
  bool first_check = true;
  for (rosbag::MessageInstance const message_instance : rosbag::View(wam_bag)) {
    ros::Time current_time = ros::Time::now();
    current_msg_time = message_instance.getTime();
    if (first_check) { //move to start position at first check
      wam_srvs::JointMove wam_jp_srv;
      sensor_msgs::JointState::ConstPtr start_joint_position = message_instance.instantiate<sensor_msgs::JointState>();
      std::vector<float> joints(start_joint_position->position.begin(), start_joint_position->position.end());
      wam_jp_srv.request.joints = joints;
      if (!joint_position_move_client.call(wam_jp_srv)) {
        ROS_ERROR("Could not move to start Pose");
        exit(0);
      }
      first_check = false;
      prev_msg_time = current_msg_time;
    }
    wam_msgs::RTJointVel::ConstPtr rt_bag_msg =
        message_instance.instantiate<wam_msgs::RTJointVel>();
    if (rt_bag_msg != nullptr) {
      while ((current_time - prevTime).toSec() <
             (current_msg_time - prev_msg_time).toSec()) {  //wait until (current_msg_time - prev_msg_time) has passed.
        current_time = ros::Time::now();
      }
      prev_msg_time = current_msg_time;
      prevTime = current_time;
      rt_jv_pub.publish(rt_bag_msg);
    }
  }
  wam_bag.close();
}

int getBagInfo(std::string ros_bag_name) {
  rosbag::Bag wam_bag;
  wam_bag.open(ros_bag_name);
  rosbag::View wam_bag_view(wam_bag);
  std::vector<const rosbag::ConnectionInfo *> connection_info = wam_bag_view.getConnections();
  std::string topic = connection_info.at(1)->topic; //get RT message topic, and use that to determine how the trajectory was recorded
  int option;
  if (topic.compare("wam/jnt_pos_cmd") == 0) {
    ROS_INFO("Rosbag recorded using Joint Positions");
    option = 0;
  } else if (topic.compare("wam/jnt_vel_cmd") == 0) {
    ROS_INFO("Rosbag recorded using Joint Velocities");
    option = 1;
  }
  wam_bag.close();
  return option;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wam_play");
  ros::NodeHandle n;
  std::string input;
  ros::Rate loop_rate(500);
  std::string ros_bag_name = "wam.bag";
  if (argc > 1) {
    ros_bag_name = argv[1];
  }
  ros_bag_name = "wam_rosbags/" + ros_bag_name;
  std::cout << "Name:" << ros_bag_name << std::endl;
  int option = getBagInfo(ros_bag_name); //jp:0, jv: 1
  ROS_INFO("Press any key to start Playing");
  getline(std::cin, input);
  ROS_INFO("Press ctrl-c to stop Playing");
  switch (option) {
    case 0:
      playJointPositions(
          ros_bag_name,
          n.advertise<wam_msgs::RTJointPos>("wam/jnt_pos_cmd", 100),
          n.serviceClient<wam_srvs::JointMove>("wam/joint_move"));
          break;
    case 1:
      playJointVelocities(
          ros_bag_name,
          n.advertise<wam_msgs::RTJointVel>("wam/jnt_vel_cmd", 100),
          n.serviceClient<wam_srvs::JointMove>("wam/joint_move"));
          break;
  }
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}