#ifndef NODE_EXAMPLE_TALKER_H
#define NODE_EXAMPLE_TALKER_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// Filter includes
#include <vector>
#include "perlin_noise_filter/perlin_noise_filter.h"
#include "filters/filter_chain.h"

namespace node_example
{

class ExampleTalker
{
public:
  //! Constructor.
  ExampleTalker(ros::NodeHandle nh);

  //! Timer callback for publishing message.
  void timerCallback(const ros::TimerEvent& event);

private:
  //! Filter Chain
  filters::MultiChannelFilterChain<double> filter_chain_;

  //! The timer variable used to go to callback function at specified rate.
  ros::Timer timer_;

  //! Message publisher.
  // ros::Publisher pub_;
};

}

#endif // NODE_EXAMPLE_TALKER_H
