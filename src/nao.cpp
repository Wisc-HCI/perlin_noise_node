#include "perlin_noise_node/nao.h"

namespace perlin_noise_node
{

PerlinNode::PerlinNode(ros::NodeHandle nh) :
    filter_chain_("double")
{

  // Declare variables that can be modified by launch file or command line.
  // Frequency of message publishing (seconds), default .5
  double freq;

  // Seed rand
  std::srand(std::time(0));

  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
  ros::NodeHandle pnh("~");
  pnh.getParam("freq", freq);
  pnh.getParam("speed", speed_);
  pnh.getParam("perlin_joints", perlin_joints_);
  pnh.getParam("joint_names", joint_names_);

  // Create a publisher and name the topic.
  pub_ = nh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("joint_angles", 10);

  // Create timer.
  timer_ = nh.createTimer(ros::Duration(freq), &PerlinNode::timerCallback, this);

  // Create filter i/o vectors
  filter_chain_.configure(3, "filter_params");

  // TODO create num offsets corresponding to size of perlin_joints
  perlin_offset_.push_back(10000*((double) rand() / (RAND_MAX)));
  perlin_offset_.push_back(10000*((double) rand() / (RAND_MAX)));
  perlin_offset_.push_back(10000*((double) rand() / (RAND_MAX)));
  perlin_offset_.push_back(10000*((double) rand() / (RAND_MAX)));

  perlin_param_.push_back(0);
  perlin_param_.push_back(0);
  perlin_param_.push_back(0);
  perlin_param_.push_back(0);

}

void PerlinNode::init()
{

}

void PerlinNode::timerCallback(const ros::TimerEvent& event)
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
