#ifndef NODE_EXAMPLE_TALKER_H
#define NODE_EXAMPLE_TALKER_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// Filter includes
#include <vector>
#include "perlin_noise_filter/perlin_noise_filter.h"
#include "filters/filter_chain.h"
#include <sensor_msgs/JointState.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <algorithm>    // std::find
#include <iterator>     // std::distance
#include <cstdlib>       // std::rand
#include <time.h>

namespace node_example
{

class ExampleTalker
{
public:
  //! Constructor.
  ExampleTalker(ros::NodeHandle nh);

  //! Callback function for subscriber.
  void messageCallback(const sensor_msgs::JointState::ConstPtr &msg);

  //! Timer callback for publishing message.
  void timerCallback(const ros::TimerEvent& event);

private:
  //! Filter Chain
  filters::MultiChannelFilterChain<double> filter_chain_;

  //! The timer variable used to go to callback function at specified rate.
  ros::Timer timer_;

  //! Message publisher
  ros::Publisher pub_;

  //! Subscriber
  ros::Subscriber sub_;

  std::vector<std::string> paramNames_;

  std::vector<double> perlinOffset_;

  std::vector<double> perlinParam_;
};

}

#endif // NODE_EXAMPLE_TALKER_H
