/************
 * \author Sandra Devin (sdevin@laas.fr)
 *
 * Class of the give action
 * **********/

#include "action_manager/Actions/give.h"


/**
 * Constructor of the class
 * @param action the description of the action to execute
 * @param connector pointer to the connector object
 * */
Give::Give(roboergosum_msgs::Action action, Connector* connector): VirtualAction(connector){
    //
    if(action.parameters.size() == 2){
        object_ = action.parameters[0];
        receiver_ = action.parameters[1];
    }else{
        ROS_WARN("[action_manager] Wrong paramters for the give action, should be: object, receiver");
    }
    node_.getParam("/action_manager/handoverConfigurationRight", confNameRight_);
    node_.getParam("/action_manager/handoverConfigurationLeft", confNameLeft_);
}

/**
 * \brief Check the precondition of the give action
 *
 * For the give action the preconditions checked are:
 *  - the receiver should be an agent
 *  - the agent should be reachable by the robot
 *  - the robot should have the object in hand
 *
 * \return true if the preconditions are checked, else return false
 * */
bool Give::preconditions(){

    //First we check if the object is a known manipulable object
    if(!isAgent(receiver_)){
      ROS_WARN("[action_executor] The receiver of the give action is not an known agent");
      return false;
    }

    //Then we check if the robot has the object in hand and if the receiver is reachable
    std::vector<toaster_msgs::Fact> precsTocheck;
    toaster_msgs::Fact fact;
    fact.subjectId = object_;
    fact.property = "isHoldBy";
    fact.targetId = robotName_;
    precsTocheck.push_back(fact);
    fact.subjectId = receiver_;
    fact.property = "isReachableBy";
    fact.targetId = robotName_;
    precsTocheck.push_back(fact);

    return AreFactsInDB(precsTocheck);
}

/**
 * \brief Plan for the give action
 *
 * Verify if we need to add the graps of the pick.
 * Ask to gtp a solution to place the object (moveTo a give configuration)
 *
 * \return true if the planning is a success, else return false
 * */
bool Give::plan(){

    //if the previous gtp id is -1, we need to look for the id of the grasp
    if(connector_->previousGTPId_ == -1){
        if(connector_->idGrasp_ != -1){
            ROS_WARN("[action_manager] Give failed in planning: no previous id for the grasp");
        }else{
            //We add the grasp in gtp
            if(!addGTPAttachment(connector_->idGrasp_)){
                //Failure msg already in the function
                return false;
            }
        }
    }


    //We look in which arm the robot has the object
    std::string confName;
    if(armGrasp_ = 0){
        confName = confNameRight_;
    }else if(armGrasp_ = 1){
        confName = confNameLeft_;
    }else{
        ROS_WARN("[action_manager] Give failed in planning: impossible to get the robot arm where is the object");
    }


    //Now we can plan the place
    std::vector<gtp_ros_msg::Ag> agents;
    gtp_ros_msg::Ag agent;
    agent.actionKey = "mainAgent";
    agent.agentName = robotName_;
    agents.push_back(agent);
    std::vector<gtp_ros_msg::Obj> objects;
    objects.push_back(object);
    std::vector<gtp_ros_msg::Points> points;
    std::vector<gtp_ros_msg::Data> datas;
    gtp_ros_msg::Data data;
    data.dataKey = "hand";
    data.dataValue = "right";
    datas.push_back(data);
    data.dataKey = "confName";
    data.dataValue = confName;
    datas.push_back(data);

    GTPActionId_ = planGTP("moveTo", agents, objects, datas, points);

    if(GTPActionId_ == -1){
        return false;
    }else{
        return true;
    }

    return true;
}

/**
 * \brief Execute the give action
 *
 * Execute with pr2motion the solution found by gtp
 *
 * \return true if the execution is a success, else return false
 * */
bool Give::exec(){

    return executeTrajectory(GTPActionId_, 0, armGrasp_);

}

/**
 * \brief Post-conditions for the give action
 *
 * Transfer the object from robot to human hand
 *
 * \return true if the post-conditions are checked, else return false
 * */
bool Give::post(){

    RemoveFromHand(object_);
    PutInHumanHand(object_, receiver_);

    return true;
}
