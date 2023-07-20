#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <thread>
#include <sys/stat.h>
#include <math.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"
#include "wam_srvs/JointMove.h"
#include "wam_srvs/Hold.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "wam_msgs/RTJointPos.h"
#include "wam_msgs/RTJointVel.h"
#include "teach.h"
/**Function to parse args 
 * Possible args are:
    * -n <bag_name>
    * -t <record_type>
        * "-t jp" -> record using joint positions
        * "-t jv" -> record using joint velocities
  * Option defines what record mode to use. Possible values are:
        * 0 -> record using joint positions
        * 1 -> record using joint velocities
*/
void parseArgs(int argc, char** argv, int& option, std::string& ros_bag_name) {
  if (argc == 3) {  //either one of the args specified
    char* flag(argv[1]);
    if (strcmp(flag, "-n") == 0) { //argv[2] is name
      ros_bag_name = argv[2];
      ROS_INFO("Rosbag name: %s", ros_bag_name.c_str());
    } else if (strcmp(flag, "-t") == 0) { //argv[2] is record_type
      char* record_mode(argv[2]);
      if (strcmp(record_mode, "jp") == 0) {
        option = 0;
        ROS_INFO("Recording using joint positions");
      } else if (strcmp(record_mode, "jv") == 0) {
        option = 1;
        ROS_INFO("Recording using joint velocities");
      } else {
        ROS_ERROR(
            "Incorrect recording data type specified: must be jp or jv.");
        exit(0);
      }
    } else {
      ROS_ERROR("Invalid flag. Usage: %s -n <bag-name> -t <record-type>",
                argv[0]);
      exit(0);
    }
  } else if (argc == 5) {  //both args specified
    char* flag1(argv[1]); //first flag
    if (strcmp(flag1, "-n") == 0) {
      ros_bag_name = argv[2];
      ROS_INFO("Rosbag name: %s", ros_bag_name.c_str());
    } else if (strcmp(flag1, "-t") == 0) {
      char* record_mode(argv[2]);
      if (strcmp(record_mode, "jp") == 0) {
        option = 0;
        ROS_INFO("Recording using joint positions");
      } else if (strcmp(record_mode, "jv") == 0) {
        option = 1;
        ROS_INFO("Recording using joint velocities");
      } else {
        ROS_ERROR(
            "Incorrect recording data type specified: must be jp or jv.");
        exit(0);
      }
    }
    char* flag(argv[3]); //second flag
    if (strcmp(flag, "-n") == 0) {
      ros_bag_name = argv[4];
      ROS_INFO("Rosbag name: %s", ros_bag_name.c_str());
    } else if (strcmp(flag, "-t") == 0) {
      char* record_mode(argv[4]);
      if (strcmp(record_mode, "jp") == 0) {
        option = 0;
        ROS_INFO("Recording using joint positions");
      } else if (strcmp(record_mode, "jv") == 0) {
        option = 1;
        ROS_INFO("Recording using joint velocities");
      } else {
        ROS_ERROR(
            "Incorrect recording data type specified: must be jp or jv.");
        exit(0);
      }
    }
     else {
      ROS_ERROR("Invalid flag. Usage: %s -n <bag-name> -t <record-type>",
                argv[0]);
      exit(0);
    }
  } else {
    // no args specified
    ROS_WARN("No args specified. Defaulting to record using joint positions");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wam_teach");
  ros::NodeHandle n;
  ros::Rate loop_rate(500);
  mkdir("wam_rosbags", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); //make a "wam_rosbags" directory to store rosbags
  std::string ros_bag_name = "wam.bag"; //default bag name
  int option = 0;
  parseArgs(argc, argv, option, ros_bag_name); 
  ros_bag_name = "wam_rosbags/" + ros_bag_name;
  Teach teach(
      ros_bag_name,
      n.serviceClient<wam_srvs::Hold>("wam/hold_joint_pos"), option);
  ros::Subscriber wam_jp_sub =
      n.subscribe("wam/joint_states", 1000, &Teach::wamJointPositionCb, &teach);
  std::string input;
  ROS_INFO("Press any key to start Teaching");
  getline(std::cin, input);
  ROS_INFO("Press any key to stop teaching");
  ros::spinOnce();
  std::thread teach_thread(&Teach::startTeaching, &teach);
  teach_thread.detach();
  getline(std::cin, input);
  teach.stopTeaching();
}