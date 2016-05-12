#include "perlin_noise_node/perlin_base.h"

namespace perlin_noise_node
{

PerlinNode::PerlinNode(ros::NodeHandle nh) :
    filter_chain_("double")
{
  // Seed rand
  std::srand(std::time(0));

  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
  ros::NodeHandle pnh("~");
  pnh.getParam("freq", freq_);
  pnh.getParam("speed", speed_);
  pnh.getParam("perlin_joints", perlin_joints_);
  pnh.getParam("joint_names", joint_names_);

  // Create timer.
  timer_ = nh.createTimer(ros::Duration(freq_), &PerlinNode::timerCallback, this);

  // Create filter i/o vectors
  filter_chain_.configure(3, "filter_params");

  // Create offsets for each of the joints that Perlin is applied to
  for (int i = 0; i < perlin_joints_.size(); i++) {
    perlin_offset_.push_back(10000*((double) rand() / (RAND_MAX)));
    perlin_param_.push_back(0);
  }
}
}
