#include "perlin_noise_node/nao.h"

namespace perlin_noise_node
{

Nao::Nao(ros::NodeHandle nh) : PerlinNode::PerlinNode(nh) {
  // Create a publisher and name the topic.
  pub_ = nh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("joint_angles", 10);
}

void Nao::timerCallback(const ros::TimerEvent& event)
{
  std::vector<float> joint_angles (joint_names_.size(), 0);

  // Create new message to publish
  naoqi_bridge_msgs::JointAnglesWithSpeed jointAngles;
  jointAngles.joint_names = joint_names_;
  jointAngles.joint_angles = joint_angles;

  // Call Perlin Noise Filter, update joint_angles with results
  std::vector<double> filt_in (3, 0);
  std::vector<double> filt_out (3, -1);

  for(int i = 0; i < perlin_joints_.size(); i++) {

    filt_in[0] = perlin_offset_[i] + perlin_param_[i];
    perlin_param_[i] += 1.1;

    if(filter_chain_.update(filt_in, filt_out)) {
      ROS_INFO("Noise values updated sucessfully");
    } else {
      ROS_INFO("Noise values updated unsucessfully");
    }

    // Find specified joint's corresponding joint_angle's index
    std::vector<std::string>::iterator it = std::find(jointAngles.joint_names.begin(), jointAngles.joint_names.end(), perlin_joints_[i]);
    if (it != jointAngles.joint_names.end()) {
       int ind = std::distance(jointAngles.joint_names.begin(), it);
       jointAngles.joint_angles[ind] = filt_out[0] * .75;
    } else {
      ROS_ERROR("Couldn't find user specified joint name in joint_names");
    }
  }

  jointAngles.speed = speed_;
  jointAngles.relative = 1;

  pub_.publish(jointAngles);
}
}
