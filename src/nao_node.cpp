#include "perlin_noise_node/nao.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "pn_gen");
  ros::NodeHandle nh;

  // Create a new perlin_noise_node::PerlinNode object.
  perlin_noise_node::Nao node(nh);

  // Let ROS handle all callbacks.
  ros::spin();

  return 0;
} // end main()
