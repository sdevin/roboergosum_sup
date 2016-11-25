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
 * @brief Service call to remove attachments
 */
bool addAttachment(roboergosum_msgs::String::Request  &req, roboergosum_msgs::String::Response &res){

    std::pair<std::string, std::string> attach;
    attach.first = "HERAKLES_HUMAN1";
    attach.second = req.data;
    hm_->attachments_.push_back(attach);

    return true;
}

/**
 * @brief Service call to remove attachments
 */
bool removeAttachment(roboergosum_msgs::String::Request  &req, roboergosum_msgs::String::Response &res){

    for(std::vector<std::pair<std::string, std::string> >::iterator it = hm_->attachments_.begin(); it != hm_->attachments_.end(); it++){
       if(it->second == req.data){
           hm_->attachments_.erase(it);
           break;
       }
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
  ros::ServiceServer remove_attach = node.advertiseService("human_manager/remove_attachment", removeAttachment); //allows to remove an attachment
  ros::ServiceServer add_attach = node.advertiseService("human_manager/add_attachment", addAttachment); //allows to add an attachment

  ROS_INFO("[human_manager] human_manager ready");

  ros::spin();

  return 0;
}
