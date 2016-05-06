#include "perlin_noise_node/pn_gen.h"

namespace perlin_noise_node
{

PerlinNode::PerlinNode(ros::NodeHandle nh) :
    filter_chain_("double")
{

  // Declare variables that can be modified by launch file or command line.
  int rate;

  // Seed rand
  std::srand(std::time(0));

  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
  ros::NodeHandle pnh("~");
  pnh.param("rate", rate, 1);

  // Create a publisher and name the topic.
  pub_ = nh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("joint_angles", 10);

  // Create timer.
  timer_ = nh.createTimer(ros::Duration(.5), &PerlinNode::timerCallback, this);

  // Create filter i/o vectors
  filter_chain_.configure(3, "perlin_params");

  // TODO these will eventually be read in from paramServer
  paramNames_.push_back("HeadYaw");
  paramNames_.push_back("HeadPitch");
  // paramNames_.push_back("LShoulderPitch");
  // paramNames_.push_back("LShoulderRoll");


  perlinOffset_.push_back(10000*((double) rand() / (RAND_MAX)));
  perlinOffset_.push_back(10000*((double) rand() / (RAND_MAX)));
  perlinOffset_.push_back(10000*((double) rand() / (RAND_MAX)));
  perlinOffset_.push_back(10000*((double) rand() / (RAND_MAX)));

  perlinParam_.push_back(0);
  perlinParam_.push_back(0);
  perlinParam_.push_back(0);
  perlinParam_.push_back(0);

}

void PerlinNode::timerCallback(const ros::TimerEvent& event)
{

  int nameSize = 26;
  std::string x[26] = {"HeadYaw", "HeadPitch", "LShoulderPitch",
  "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
  "LHand", "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch",
   "LAnklePitch", "LAnkleRoll", "RHipYawPitch", "RHipRoll", "RHipPitch",
    "RKneePitch", "RAnklePitch", "RAnkleRoll", "RShoulderPitch",
     "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"};
  std::vector<std::string> name(x, x + sizeof(x)/sizeof(x[0]));
  std::vector<float> joint_angles (nameSize, 0);

  naoqi_bridge_msgs::JointAnglesWithSpeed jointAngles;
  jointAngles.joint_names = name;
  jointAngles.joint_angles = joint_angles;

  std::vector<double> filt_in (3, 0);
  std::vector<double> filt_out (3, -1);

  for(int i = 0; i < paramNames_.size(); i++) {

    filt_in[0] = perlinOffset_[i] + perlinParam_[i];
    perlinParam_[i] += 1.1;

    if(filter_chain_.update(filt_in, filt_out)) {
      ROS_INFO("updated sucessfully");
    } else {
      ROS_INFO("updated unsucessfully");
    }

    jointAngles.joint_angles[i] = filt_out[0] * .75;
  }

  jointAngles.speed = .025;
  jointAngles.relative = 1;

  pub_.publish(jointAngles);

  ROS_INFO("message is published");
}
}
