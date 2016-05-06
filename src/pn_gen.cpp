#include "perlin_noise_node/pn_gen.h"

namespace perlin_noise_node
{

PerlinNode::PerlinNode(ros::NodeHandle nh) :
    filter_chain_("double")
{
  // TODO remove toPublish_ after further confirmation that stack isn't needed

  // Declare variables that can be modified by launch file or command line.
  int rate;

  // Initialize rand
  std::srand(std::time(0));

  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
  ros::NodeHandle pnh("~");
  pnh.param("rate", rate, 1);

  // Create a publisher and name the topic.
  pub_ = nh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("joint_angles", 10);

  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  // sub_ = nh.subscribe("/joint_states", 1, &PerlinNode::messageCallback, this);

  // Count sub's CB calls, only act on every 100th for nao
  msgHits_ = 0;

  // Create timer.
  timer_ = nh.createTimer(ros::Duration(.5), &PerlinNode::timerCallback, this);

  // Create filter i/o vectors
  filter_chain_.configure(3, "perlin_params");

  // TODO these will eventually be read in from paramServer
  // paramNames_.push_back("HeadYaw");
  // paramNames_.push_back("HeadPitch");
  paramNames_.push_back("LShoulderPitch");
  paramNames_.push_back("LShoulderRoll");


  perlinOffset_.push_back(10000*((double) rand() / (RAND_MAX)));
  perlinOffset_.push_back(10000*((double) rand() / (RAND_MAX)));
  perlinOffset_.push_back(10000*((double) rand() / (RAND_MAX)));
  perlinOffset_.push_back(10000*((double) rand() / (RAND_MAX)));

  perlinParam_.push_back(0);
  perlinParam_.push_back(0);
  perlinParam_.push_back(0);
  perlinParam_.push_back(0);

}

void PerlinNode::messageCallback(const sensor_msgs::JointState::ConstPtr & msg)
{

  // if (msgHits_ == 100) {
  //   msgHits_ = 0;
  // } else {
  //   msgHits_++;
  //   return;
  // }

  // TODO eventually need seperate class to handle Update.
  // nao implementation needs class obj for each given param (joint name)
  // obj will have:
  // -index - index corresponding to joint_states name arr
  // -name
  // each robot's conf will need:
  // - sizeof(name)
  // - sub, pub topics
  // videos of nao,

  // int nameSize = 26;
  // std::vector<std::string> name (msg->name);
  // std::vector<float> joint_angles (nameSize, 0);
  //
  // naoqi_bridge_msgs::JointAnglesWithSpeed jointAngles;
  // jointAngles.joint_names = name;
  // jointAngles.joint_angles = joint_angles;
  //
  // std::vector<double> filt_in (3, 0);
  // std::vector<double> filt_out (3, -1);
  //
  // for(int i = 0; i < paramNames_.size(); i++) {
  //
  //   perlinParam_[i] += 0.01;
  //
  //   filt_in[0] = perlinOffset_[i] * perlinParam_[i];
  //
  //
  //
  //   if(filter_chain_.update(filt_in, filt_out)) {
  //     ROS_INFO("updated sucessfully");
  //   } else {
  //     ROS_INFO("updated unsucessfully");
  //   }
  //
  //   jointAngles.joint_angles[i] = filt_out[0];
  // }
  //
  // jointAngles.speed = .075;
  // jointAngles.relative = 1;
  //
  // //pub_.publish(jointAngles);
  //
  // toPublish_.push(jointAngles);
  //
  // ROS_INFO("message is published");
}

void PerlinNode::timerCallback(const ros::TimerEvent& event)
{
  // ROS_INFO("toPublish size: %lu", toPublish_.size());
  //
  // if(toPublish_.empty()){
  //   return;
  // }
  //
  // pub_.publish(toPublish_.top());
  // toPublish_.pop();

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

    jointAngles.joint_angles[i+2] = filt_out[0] * .75;
  }

  jointAngles.speed = .025;
  jointAngles.relative = 1;

  pub_.publish(jointAngles);

  // toPublish_.push(jointAngles);

  ROS_INFO("message is published");
}
}
