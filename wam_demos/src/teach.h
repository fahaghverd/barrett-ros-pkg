#ifndef SRC_WAM_DEMOS_SRC_TEACH_
#define SRC_WAM_DEMOS_SRC_TEACH_

const double kRateLimits = 1;

class Teach {
 private:
  rosbag::Bag wam_bag_;
  //command messages
  wam_msgs::RTJointPos rt_joint_position_cmd_;
  wam_msgs::RTJointVel rt_joint_velocity_cmd_;
  
  sensor_msgs::JointState current_joint_position_;
  //indicators for recording rosbags
  bool joint_position_teach_, joint_velocity_teach_;
  std::string bag_name_;
  bool teaching_;
  /**True if first message has not been written. 
   * First message contains start JP */
  bool first_write_;
  ros::ServiceClient hold_srv_;
  int option_;
 public:
  void wamJointPositionCb(const sensor_msgs::JointState::ConstPtr& msg);
  void startTeaching();
  void stopTeaching();
  Teach(std::string bag_name,
                 ros::ServiceClient hold_srv, int option) {
    bag_name_ = bag_name;
    option_ = option;
    joint_position_teach_ = joint_velocity_teach_ = false;
    teaching_ = false;
    hold_srv_ = hold_srv;
  }
};

//callback function for /joint_states topic. If joint_position_teach is set, write the joint states to wam_bag_
//Joint velocity here as well
void Teach::wamJointPositionCb(
    const sensor_msgs::JointState::ConstPtr& msg) {
  current_joint_position_ = *msg;
  if (joint_position_teach_) {
    std::vector<float> joints(msg->position.begin(), msg->position.end());
    rt_joint_position_cmd_.joints = joints;
    rt_joint_position_cmd_.rate_limits.resize(msg->position.size());
    for (int i = 0; i < msg->position.size(); i++) {
      rt_joint_position_cmd_.rate_limits.at(i) = kRateLimits;
    }
    wam_bag_.write("wam/jnt_pos_cmd", current_joint_position_.header.stamp, rt_joint_position_cmd_);
  } else if (joint_velocity_teach_) {
    std::vector<float> velocities(msg->velocity.begin(), msg->velocity.end());
    rt_joint_velocity_cmd_.velocities = velocities;
    wam_bag_.write("wam/jnt_vel_cmd",current_joint_position_.header.stamp, rt_joint_velocity_cmd_);
  }
}

//Callback of teach_thread. Idle wam and start teaching. Spin until teaching_ is set to false.
void Teach::startTeaching() {
  wam_bag_.open(bag_name_, rosbag::bagmode::Write);
  teaching_ = true;
  /*Write current_joint_position_ to a random topic. Will be used by the play program to move the WAM to the start position */
  wam_bag_.write("startJP", current_joint_position_.header.stamp, current_joint_position_); 
  switch (option_) {
    case 0:
      joint_position_teach_ = true;
      break;
    case 1:
      joint_velocity_teach_ = true;
      break;
  }
  wam_srvs::Hold hold_srv_call;
  hold_srv_call.request.hold = false;
  hold_srv_.call(hold_srv_call);
  ros::Rate loop_rate(500);
  while (ros::ok() && teaching_) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Teach::stopTeaching() {
  joint_position_teach_ = joint_velocity_teach_ = false;
  wam_bag_.close();
  teaching_ = false;
}
#endif // SRC_WAM_DEMOS_SRC_TEACH_