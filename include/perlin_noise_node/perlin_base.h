#ifndef PERLIN_NOISE_NODE_PERLIN_BASE_H
#define PERLIN_NOISE_NODE_PERLIN_BASE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// Filter includes
#include "perlin_noise_filter/perlin_noise_filter.h"
#include "filters/filter_chain.h"

// C/C++ includes
#include <cstdlib>       // std::rand
#include <vector>
#include <time.h>
#include <string>

namespace perlin_noise_node
{

class PerlinNode
{
protected:
  //! Constructor
  PerlinNode(ros::NodeHandle nh);

  //! Timer callback for publishing message (must be implemented)
  virtual void timerCallback(const ros::TimerEvent& event) = 0;

  //! Filter Chain
  filters::MultiChannelFilterChain<double> filter_chain_;

  //! The timer variable used to go to callback function at specified rate.
  ros::Timer timer_;

  //! Message publisher
  ros::Publisher pub_;

  //! User specified joints that Perlin Noise is applied to
  std::vector<std::string> perlin_joints_;

  //! All joints names (required for message being published)
  std::vector<std::string> joint_names_;

  //! Frequency of message publishing
  double freq_;

  //! Motion speed of robot
  double speed_;

  //! Increments at fixed rate along Perlin Gradient
  std::vector<double> perlin_offset_;

  //! Random value for Perlin input
  std::vector<double> perlin_param_;
};

}

#endif // PERLIN_NOISE_NODE_PERLIN_BASE_H
