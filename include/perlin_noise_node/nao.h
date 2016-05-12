#ifndef PERLIN_NOISE_NODE_NAO_H
#define PERLIN_NOISE_NODE_NAO_H

#include "pn_gen.h"

// Nao specific includes
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>

namespace perlin_noise_node
{

class Nao: public PerlinNode
{
public:
  //! Intialize PerlinNode along with everything specific to this robot
  Nao(ros::NodeHandle nh);

  //! Timer callback for publishing message.
  void timerCallback(const ros::TimerEvent& event);
};
}

#endif // PERLIN_NOISE_NODE_NAO_H
