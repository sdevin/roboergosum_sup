/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * **********/

#include <human_manager/human_manager.h>

HumanManager* hm_;

/**
 * @brief Service call to simulate a human action
 */
bool humaAction(roboergosum_msgs::HumanAction::Request  &req, roboergosum_msgs::HumanAction::Response &res){
	
	if(req.actionName == "pick"){
       hm_->humanPick(req.agent, req.object);
	}else if(req.actionName == "place"){
       hm_->humanPlace(req.agent, req.object, req.support);
	}else if(req.actionName == "drop"){
       hm_->humanDrop(req.agent, req.object, req.container);
	}else{
       ROS_ERROR("[human_manager] Unknown action name");
	}

	return true;
}

/**
 * \brief Main function
 * */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "human_manager");
  ros::NodeHandle node;

  HumanManager hm(&node);
  hm_ = &hm;

  ROS_INFO("[human_manager] Init human_manager");

  //Services declarations
  ros::ServiceServer service_action = node.advertiseService("human_manager/human_action", humaAction); //allows to execute a human action

  ROS_INFO("[human_manager] human_manager ready");

  ros::spin();

  return 0;
}
