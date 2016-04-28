#include "perlin_noise_node/talker.h"

namespace node_example
{

ExampleTalker::ExampleTalker(ros::NodeHandle nh) :
    filter_chain_("double")
{

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
  sub_ = nh.subscribe("/joint_states", 1, &ExampleTalker::messageCallback, this);

  // Create timer.
  // timer_ = nh.createTimer(ros::Duration(1 / rate), &ExampleTalker::timerCallback, this);

  // Create filter i/o vectors
  filter_chain_.configure(3, "perlin_params");

  // TODO these will eventually be read in from paramServer
  paramNames_.push_back("HeadYaw");
  paramNames_.push_back("HeadPitch");
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

void ExampleTalker::messageCallback(const sensor_msgs::JointState::ConstPtr & msg)
{
  // TODO eventually need seperate class to handle Update.
  // nao implementation needs class obj for each given param (joint name)
  // obj will have:
  // -index - index corresponding to joint_states name arr
  // -name
  // each robot's conf will need:
  // - sizeof(name)
  // - sub, pub topics
  // videos of nao, 

  int nameSize = 26;
  std::vector<std::string> name (msg->name);
  std::vector<float> joint_angles (nameSize, 0);

  naoqi_bridge_msgs::JointAnglesWithSpeed jointAngles;
  jointAngles.joint_names = name;
  jointAngles.joint_angles = joint_angles;

  std::vector<double> filt_in (3, 0);
  std::vector<double> filt_out (3, -1);

  for(int i = 0; i < paramNames_.size(); i++) {

    // int ind = 0;
    // std::size_t found = std::find(jointAngles.joint_names.begin(), jointAngles.joint_names.end(), paramNames[i]); // replace with if(joint.perlin)
    // if (found != jointAngles.joint_names.end()) {
    //    ind = std::distance(jointAngles.joint_names.begin(), found);
    // }

    filt_in[0] = perlinOffset_[i] * perlinParam_[i];

    perlinParam_[i] += 0.01;

    if(filter_chain_.update(filt_in, filt_out)) {
      ROS_INFO("updated sucessfully");
    } else {
      ROS_INFO("updated unsucessfully");
    }

    jointAngles.joint_angles[i] = filt_out[0];
  }

  jointAngles.speed = 1;
  jointAngles.relative = 1;

  pub_.publish(jointAngles);

  ROS_INFO("message is published");
}

void ExampleTalker::timerCallback(const ros::TimerEvent& event)
{
  // double input = 0;
  // double result = 999;
  // ROS_INFO("before update, result is: %f", result);
  // if(filter_chain.update(input, result)) {
  //   ROS_INFO("update successful");
  // } else {
  //   ROS_INFO("update unsuccessful");
  // }

  // node_example::NodeExampleData msg;
  // msg.message = message_;

  // ROS_INFO("post update, result is: %f", result);

  // msg.a = 0;
  // msg.b = b_;

  // pub_.publish(msg);
}
}
