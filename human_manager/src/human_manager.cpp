/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Main class of the node
 * **********/

#include <human_manager/human_manager.h>

/**
 * \brief Construction of the class
 * @param node pointer to the node handle
 * */
HumanManager::HumanManager(ros::NodeHandle* node){
    node_ = node;
    node_->getParam("roboergosum/humanRightHand", humanHand_);
    node_->getParam("/roboergosum/robotName", robotName_);
}

/**
 * \brief Function to execute a pick action
 * @param agent the agent who acts
 * @param object the object picked
 * */
void HumanManager::humanPick(std::string agent, std::string object){

    std::pair<bool, std::string> previousAttachment = hasInHand(agent);
    if(previousAttachment.first){
        ROS_WARN("[human_manager] %s has already %s in hand", agent.c_str(), previousAttachment.second.c_str());
        return;
    }
	
    ROS_INFO("[human_manager] %s has picked %s", agent.c_str(), object.c_str());

    ros::ServiceClient put_in_hand = node_->serviceClient<toaster_msgs::PutInHand>("pdg/put_in_hand");

    //put the object in the hand of the agent
	toaster_msgs::PutInHand srv_putInHand;
    srv_putInHand.request.objectId = object;
    srv_putInHand.request.agentId = agent;
    srv_putInHand.request.jointName = humanHand_;
    if (!put_in_hand.call(srv_putInHand)){
   	 ROS_ERROR("Failed to call service pdg/put_in_hand");
    }

    //we add the attachment
    std::pair<std::string, std::string> attach;
    attach.first = agent;
    attach.second = object;
    attachments_.push_back(attach);

    //add effects to the database
    std::vector<toaster_msgs::Fact> effects;
    toaster_msgs::Fact fact;
    fact.subjectId = object;
    fact.property = "isHoldBy";
    fact.targetId = agent;
    effects.push_back(fact);
    addFactsToDB(effects);
}

/**
 * \brief Function to execute a place action
 * @param agent the agent who acts
 * @param object the object placed
 * @param support the support where the object is placed
 * */
void HumanManager::humanPlace(std::string agent, std::string object, std::string support){


    std::pair<bool, std::string> previousAttachment = hasInHand(agent);
    if(!previousAttachment.first){
        ROS_WARN("[human_manager] %s has no object in hand", agent.c_str());
        return;
    }else{
        if(previousAttachment.second != object){
            ROS_WARN("[human_manager] %s has %s in hand, not %s", agent.c_str(), previousAttachment.second.c_str(), object.c_str());
            return;
        }
    }

    ROS_INFO("[human_manager] %s has placed %s on %s", agent.c_str(), object.c_str(), support.c_str());

    ros::ServiceClient remove_from_hand = node_->serviceClient<toaster_msgs::RemoveFromHand>("pdg/remove_from_hand");
    ros::ServiceClient set_entity_pose = node_->serviceClient<toaster_msgs::SetEntityPose>("toaster_simu/set_entity_pose");

	//remove the object from the hand of the agent
	toaster_msgs::RemoveFromHand srv_rmFromHand;
    srv_rmFromHand.request.objectId = object;
    if (!remove_from_hand.call(srv_rmFromHand)){
     ROS_ERROR("Failed to call service pdg/remove_from_hand");
    }

    //we remove the corresponding attachment
    for(std::vector<std::pair<std::string, std::string> >::iterator it = attachments_.begin(); it != attachments_.end(); it++){
       if(it->first == agent){
           attachments_.erase(it);
           break;
       }
    }

	//put the object on the placement
	double objectHeight, supportHeight;
    std::string objectHeightTopic = "entities/objectsHeight/bottom/";
	objectHeightTopic = objectHeightTopic + object;
    std::string supportHeightTopic = "entities/objectsHeight/top/";
	supportHeightTopic = supportHeightTopic + support;
    node_->getParam(objectHeightTopic, objectHeight);
    node_->getParam(supportHeightTopic, supportHeight);
    toaster_msgs::ObjectListStamped objectList;
	double x,y,z;
    try{
       objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
       for(std::vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
         if(it->meEntity.id == support){
            x = it->meEntity.pose.position.x;
            y = it->meEntity.pose.position.y;
            z = it->meEntity.pose.position.z;
            break;
         }
       }
       z = z + objectHeight + supportHeight;
       toaster_msgs::SetEntityPose srv_setPose;
       srv_setPose.request.id = object;
       srv_setPose.request.type = "object";
       srv_setPose.request.pose.position.x = x;
       srv_setPose.request.pose.position.y = y;
       srv_setPose.request.pose.position.z = z;
       srv_setPose.request.pose.orientation.x = 0.0;
       srv_setPose.request.pose.orientation.y = 0.0;
       srv_setPose.request.pose.orientation.z = 0.0;
       srv_setPose.request.pose.orientation.w = 1.0;
       if (!set_entity_pose.call(srv_setPose)){
         ROS_ERROR("Failed to call service pdg/set_entity_pose");
         }
    }
    catch(const std::exception & e){
       ROS_WARN("Failed to read %s pose from toaster", support.c_str());
    }

    //add effects to the database
    std::vector<toaster_msgs::Fact> effects;
    toaster_msgs::Fact fact;
    fact.subjectId = "NULL";
    fact.property = "isHoldBy";
    fact.targetId = agent;
    effects.push_back(fact);
    fact.subjectId = object;
    fact.property = "isOn";
    fact.targetId = support;
    effects.push_back(fact);
    addFactsToDB(effects);
}

/**
 * \brief Function to execute a drop action
 * @param agent the agent who acts
 * @param object the object droped
 * @param container the container where the object is droped
 * */
void HumanManager::humanDrop(std::string agent, std::string object, std::string container){
	
    std::pair<bool, std::string> previousAttachment = hasInHand(agent);
    if(!previousAttachment.first){
        ROS_WARN("[human_manager] %s has no object in hand", agent.c_str());
        return;
    }else{
        if(previousAttachment.second != object){
            ROS_WARN("[human_manager] %s has %s in hand, not %s", agent.c_str(), previousAttachment.second.c_str(), object.c_str());
            return;
        }
    }

    ROS_INFO("[human_manager] %s has droped %s in %s", agent.c_str(), object.c_str(), container.c_str());

    ros::ServiceClient remove_from_hand = node_->serviceClient<toaster_msgs::RemoveFromHand>("pdg/remove_from_hand");
    ros::ServiceClient set_entity_pose = node_->serviceClient<toaster_msgs::SetEntityPose>("toaster_simu/set_entity_pose");
	//remove the object from the hand of the agent
	toaster_msgs::RemoveFromHand srv_rmFromHand;
    srv_rmFromHand.request.objectId = object;
    if (!remove_from_hand.call(srv_rmFromHand)){
     ROS_ERROR("Failed to call service pdg/remove_from_hand");
    }

    //we remove the corresponding attachment
    for(std::vector<std::pair<std::string, std::string> >::iterator it = attachments_.begin(); it != attachments_.end(); it++){
       if(it->first == agent){
           attachments_.erase(it);
           break;
       }
    }

	//put the object in the container
    toaster_msgs::ObjectListStamped objectList;
	double x,y,z;
    try{
       objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
       for(std::vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
         if(it->meEntity.id == container){
            x = it->meEntity.pose.position.x;
            y = it->meEntity.pose.position.y;
            z = it->meEntity.pose.position.z;
            break;
         }
       }
       toaster_msgs::SetEntityPose srv_setPose;
       srv_setPose.request.id = object;
       srv_setPose.request.type = "object";
       srv_setPose.request.pose.position.x = x;
       srv_setPose.request.pose.position.y = y;
       srv_setPose.request.pose.position.z = z;
       srv_setPose.request.pose.orientation.x = 0.0;
       srv_setPose.request.pose.orientation.y = 0.0;
       srv_setPose.request.pose.orientation.z = 0.0;
       srv_setPose.request.pose.orientation.w = 1.0;
       if (!set_entity_pose.call(srv_setPose)){
         ROS_ERROR("Failed to call service pdg/set_entity_pose");
         }
    }
    catch(const std::exception & e){
       ROS_WARN("Failed to read %s pose from toaster", container.c_str());
    }

    //add effects to the database
    std::vector<toaster_msgs::Fact> effects;
    toaster_msgs::Fact fact;
    fact.subjectId = "NULL";
    fact.property = "isHoldBy";
    fact.targetId = agent;
    effects.push_back(fact);
    fact.subjectId = object;
    fact.property = "isIn";
    fact.targetId = container;
    effects.push_back(fact);
    addFactsToDB(effects);
}

/**
 * \brief Function which return true if an agent has already an object in hand
 * @param agent the agent tested
 * \return a boolean which specifies if the human has an object in hand, and if true the object in question
 * */
std::pair<bool, std::string> HumanManager::hasInHand(std::string agent){
    std::pair<bool, std::string> answer;
    for(std::vector<std::pair<std::string, std::string> >::iterator it = attachments_.begin(); it != attachments_.end(); it++){
        if(it->first == agent){
            answer.first = true;
            answer.second = it->second;
            return answer;
        }
    }
    answer.first = false;
    return answer;
}

/**
* \brief Function which add facts to the database
* @param facts the facts to add
*/
void HumanManager::addFactsToDB(std::vector<toaster_msgs::Fact> facts){

    ros::ServiceClient client = node_->serviceClient<toaster_msgs::SetInfoDB>("database_manager/set_info");
    std::vector<toaster_msgs::Fact> toAdd, toRm;

    for(std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
        if(it->subjectId == "NULL" || it->targetId == "NULL"){
            toRm.push_back(*it);
        }else{
            toAdd.push_back(*it);
        }
    }

    if(toRm.size() >0){
        toaster_msgs::SetInfoDB srv;
        srv.request.agentId = robotName_;
        srv.request.facts = facts;
        srv.request.infoType = "FACT";
        srv.request.add = false;
        if (!client.call(srv)){
         ROS_ERROR("[action_manager] Failed to call service database_manager/set_info");
        }
    }

    if(toAdd.size() >0){
        toaster_msgs::SetInfoDB srv;
        srv.request.agentId = robotName_;
        srv.request.facts = facts;
        srv.request.infoType = "FACT";
        srv.request.add = true;
        if (!client.call(srv)){
         ROS_ERROR("[action_manager] Failed to call service database_manager/set_info");
        }
    }
}

