#ifndef PERLIN_NOISE_NODE_PN_GEN_H
#define PERLIN_NOISE_NODE_PN_GEN_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// Filter includes
#include "perlin_noise_filter/perlin_noise_filter.h"
#include "filters/filter_chain.h"

// Node includes
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>

// C/C++ includes
#include <cstdlib>       // std::rand
#include <vector>
#include <time.h>
#include <string>

namespace perlin_noise_node
{

class PerlinNode
{
public:
  //! Constructor.
  PerlinNode(ros::NodeHandle nh);

  //! Timer callback for publishing message.
  void timerCallback(const ros::TimerEvent& event);

private:
  //! Filter Chain
  filters::MultiChannelFilterChain<double> filter_chain_;

  //! The timer variable used to go to callback function at specified rate.
  ros::Timer timer_;

  //! Message publisher
  ros::Publisher pub_;

  std::vector<std::string> paramNames_;

  std::vector<double> perlinOffset_;

  std::vector<double> perlinParam_;
};

}

#endif // PERLIN_NOISE_NODE_PN_GEN_H
