#include <smc_g2_operator/smc_g2_operator.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "smc_operator");
  SMCOperator smc_node;
  smc_node.initSMC();
  smc_node.initSubscriber();
  smc_node.initServiceServer();
  ros::spin();
  return 0;
}