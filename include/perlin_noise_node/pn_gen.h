#ifndef PERLIN_NOISE_NODE_PN_GEN_H
#define PERLIN_NOISE_NODE_PN_GEN_H

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
  //! Constructor.
  PerlinNode(ros::NodeHandle nh);

  //! Timer callback for publishing message.
  virtual void timerCallback(const ros::TimerEvent& event) = 0;

  //! Filter Chain
  filters::MultiChannelFilterChain<double> filter_chain_;

  //! The timer variable used to go to callback function at specified rate.
  ros::Timer timer_;

  //! Message publisher
  ros::Publisher pub_;

  std::vector<std::string> perlin_joints_;

  std::vector<std::string> joint_names_;

  double freq_;

  double speed_;

  std::vector<double> perlin_offset_;

  std::vector<double> perlin_param_;
};

}

#endif // PERLIN_NOISE_NODE_PN_GEN_H
