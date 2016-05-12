#include "perlin_noise_node/nao.h"

namespace perlin_noise_node
{

Nao::Nao(ros::NodeHandle nh) : PerlinNode::PerlinNode(nh) {
  // Create a publisher and name the topic.
  pub_ = nh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("joint_angles", 10);
}

void Nao::timerCallback(const ros::TimerEvent& event)
{
  int num_joints = 26;
  std::vector<float> joint_angles (num_joints, 0);

  naoqi_bridge_msgs::JointAnglesWithSpeed jointAngles;
  jointAngles.joint_names = joint_names_;
  jointAngles.joint_angles = joint_angles;

  std::vector<double> filt_in (3, 0);
  std::vector<double> filt_out (3, -1);

  for(int i = 0; i < perlin_joints_.size(); i++) {

    filt_in[0] = perlin_offset_[i] + perlin_param_[i];
    perlin_param_[i] += 1.1;

    if(filter_chain_.update(filt_in, filt_out)) {
      ROS_INFO("updated sucessfully");
    } else {
      ROS_INFO("updated unsucessfully");
    }

    // TODO change correct index
    jointAngles.joint_angles[i] = filt_out[0] * .75;
  }

  jointAngles.speed = speed_;
  jointAngles.relative = 1;

  pub_.publish(jointAngles);

  ROS_INFO("message is published");
}
}
