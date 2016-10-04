/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * **********/

#include <planners_manager/planners_manager.h>

PlannersManager* pm_;


/**
 * \brief Main function
 * Executes the main loop of the roboergosum project
 * */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "planners_manager");
  ros::NodeHandle node;
  ros::Rate loop_rate(30);

  PlannersManager pm(&node);
  pm_ = &pm;

  ROS_INFO("[planners_manager] Init planners_manager");

  ros::Publisher statereward_pub = node.advertise<BP_experiment::StateReward>("/bp experiment/statereward", 1);

  ROS_INFO("[planners_manager] planners_manager ready");

  float reward = 0.0;
  while (node.ok()) {
     //we reset the environment if needed
     if(pm_->needEnvReset_){
        pm_->setEnvironment();
     }

     //we publish the world state and the previous reward
     long long int nbWS = pm_->computeWS();
     std::stringstream ss;
     ss << nbWS;
     std::string stateID = ss.str();
     BP_experiment::StateReward msg_state;
     msg_state.stateID = stateID;
     msg_state.stateType = "Bpe";
     msg_state.reward = reward;
     statereward_pub.publish(msg_state);

     //we ask to the metacontroller which expert to call

     //we call the expert

     //we execute the returned action and update HATP plan if one

     //we execute human action if needed and update HATP plan if one

     //we compute the reward

     ros::spinOnce();
     loop_rate.sleep();
  }

  return 0;
}
