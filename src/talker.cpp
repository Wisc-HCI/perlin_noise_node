#include "perlin_noise_node/talker.h"

namespace node_example
{

ExampleTalker::ExampleTalker(ros::NodeHandle nh) :
    filter_chain_("double")
{

  // Declare variables that can be modified by launch file or command line.
  int rate;

  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
  ros::NodeHandle pnh("~");
  pnh.param("rate", rate, 1);

  // Create a publisher and name the topic.
  // pub_ = nh.advertise<node_example::NodeExampleData>("example", 10);

  // Create timer.
  timer_ = nh.createTimer(ros::Duration(1 / rate), &ExampleTalker::timerCallback, this);

  filter_chain_.configure(3, "perlin_params");

  // double filt_in = 0.9;
  // double filt_out = 9;
  std::vector<double> filt_in;
  filt_in.push_back(.9);
  filt_in.push_back(0);
  filt_in.push_back(0);

  std::vector<double> filt_out;
  filt_in.push_back(-1);
  filt_in.push_back(-1);
  filt_in.push_back(-1);


  if(filter_chain_.update(filt_in, filt_out)) {
    ROS_INFO("updated sucessfully");
  } else {
    ROS_INFO("updated unsucessfully");
  }
  ROS_INFO("filt_in is: %f", filt_in);
  ROS_INFO("filt_out is: %f", filt_out);

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
