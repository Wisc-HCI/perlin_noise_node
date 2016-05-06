#ifndef PERLIN_NOISE_NODE_NAO_H
#define PERLIN_NOISE_NODE_NAO_H

#include "pn_gen.h"

// Node includes
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>

namespace perlin_noise_node
{

class Nao: public PerlinNode
{
  //! Intialize everything specific to this robot
  void init();

  //! Timer callback for publishing message.
  void timerCallback(const ros::TimerEvent& event);
};

}

#endif // PERLIN_NOISE_NODE_NAO_H
