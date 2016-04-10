#include "perlin_noise_node/talker.h"

namespace node_example
{

ExampleTalker::ExampleTalker(ros::NodeHandle nh) :
    a_(1),
    b_(2),
    message_("hello"),
    filter_chain("double")
{

  // Set up a dynamic reconfigure server.
  // Do this before parameter server, else some of the parameter server values can be overwritten.
  dynamic_reconfigure::Server<node_example::nodeExampleConfig>::CallbackType cb;
  cb = boost::bind(&ExampleTalker::configCallback, this, _1, _2);
  dr_srv_.setCallback(cb);

  // Declare variables that can be modified by launch file or command line.
  int rate;

  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
  ros::NodeHandle pnh("~");
  pnh.param("a", a_, 99);
  pnh.param("b", b_, b_);
  pnh.param("message", message_, message_);
  pnh.param("rate", rate, 1);

  // Create a publisher and name the topic.
  pub_ = nh.advertise<node_example::NodeExampleData>("example", 10);

  // Create timer.
  timer_ = nh.createTimer(ros::Duration(1 / rate), &ExampleTalker::timerCallback, this);

  if(filter_chain.configure("perlin_params")) {
    ROS_INFO("configured sucessfully");
  } else {
    ROS_INFO("configured unsucessfully");
  }

  double filt_in = 0.9;
  double filt_out = 9;

  if(filter_chain.update(filt_in, filt_out)) {
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

  node_example::NodeExampleData msg;
  msg.message = message_;

  // ROS_INFO("post update, result is: %f", result);

  msg.a = 0;
  msg.b = b_;

  pub_.publish(msg);
}

void ExampleTalker::configCallback(node_example::nodeExampleConfig& config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  message_ = config.message.c_str();
  a_ = config.a;
  b_ = config.b;
}

}
