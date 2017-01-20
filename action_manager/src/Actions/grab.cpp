/************
 * \author Sandra Devin (sdevin@laas.fr)
 *
 * Class of the grab action (handover from human to robot)
 * **********/

#include "action_manager/Actions/grab.h"


/**
 * Constructor of the class
 * @param action the description of the action to execute
 * @param connector pointer to the connector object
 * */
Grab::Grab(roboergosum_msgs::Action action, Connector* connector): VirtualAction(connector){

    if(action.parameters.size() == 2){
        object_ = action.parameters[0];
        giver_ = action.parameters[1];
    }else{
        ROS_WARN("[action_manager] Wrong paramters for the give action, should be: object, giver");
    }
}

/**
 * \brief Check the precondition of the grab action
 *
 * For the grab action the preconditions checked are:
 *  - the giver should be an agent
 *  - the giver should be reachable by the robot
 *  - the giver should have the object in hand
 *
 * \return true if the preconditions are checked, else return false
 * */
bool Grab::preconditions(){

    //First we check if the giver is a knwon agent
    if(!isAgent(giver_)){
      ROS_WARN("[action_manager] The giver of the grab action is not an known agent");
      return false;
    }

    //Then we check if the robot has the object in hand and if the giver is reachable
    std::vector<toaster_msgs::Fact> precsTocheck;
    toaster_msgs::Fact fact;
    fact.subjectId = object_;
    fact.property = "isHoldBy";
    fact.targetId = giver_;
    precsTocheck.push_back(fact);
    fact.subjectId = "NULL";
    fact.property = "isHoldBy";
    fact.targetId = robotName_;
    precsTocheck.push_back(fact);
    fact.subjectId = giver_;
    fact.property = "isReachableBy";
    fact.targetId = robotName_;
    precsTocheck.push_back(fact);

    return AreFactsInDB(precsTocheck);
}

/**
 * \brief Plan for the grab action
 *
 * Remove the object from human hand, place it in front of the robot
 * Ask to gtp a solution to grab the object (pick the object)
 *
 * \return true if the planning is a success, else return false
 * */
bool Grab::plan(){

    //we remove the object from the human hand
    RemoveFromHumanHand(object_);
    //we place it in front of the robot
    PutObjectInFrontRobot(object_);

    ros::Duration(1.0).sleep();

    connector_->previousGTPId_ = -1;

    //we plan a pick to grab the object
    std::vector<gtp_ros_msg::Ag> agents;
    gtp_ros_msg::Ag agent;
    agent.actionKey = "mainAgent";
    agent.agentName = robotName_;
    agents.push_back(agent);
    std::vector<gtp_ros_msg::Obj> objects;
    gtp_ros_msg::Obj object;
    object.actionKey = "mainObject";
    object.objectName = object_;
    objects.push_back(object);
    std::vector<gtp_ros_msg::Points> points;
    std::vector<gtp_ros_msg::Data> datas;

    if(shouldUseRightHand_){
        gtp_ros_msg::Data data;
        data.dataKey = "hand";
        data.dataValue = "right";
        datas.push_back(data);
    }

    GTPActionId_ = planGTP("pick", agents, objects, datas, points);

    if(GTPActionId_ == -1){
        //we place back the object in the human hand
        PutInHumanHand(object_, giver_);
        return false;
    }

    return true;
}

/**
 * \brief Execute the grab action
 *
 * Execute with pr2motion the solution found by gtp
 * If failure put back the object in the human hand
 *
 * \return true if the execution is a success, else return false
 * */
bool Grab::exec(){

    if(!execGTPAction(GTPActionId_, true, object_)){
        //we place back the object in the human hand
        PutInHumanHand(object_, giver_);
        return false;
    }
    //if we consider the human lazy, the action failed
    if(connector_->humanNoHandover_){
        PutInHumanHand(object_, giver_);
        return false;
    }

    return true;

}

/**
 * \brief Post-conditions for the grab action
 *
 * Check that the gripper of the robot is not completly closed (meaning that the robot misses the object, not in simu)
 * The effects are:
 *   - NULL isHoldBy receiver
 *   - object isHoldBy robot
 *
 * \return true if the post-conditions are checked, else return false
 * */
bool Grab::post(){

    //Check gripper position (completly close or not)
    if(gripperEmpty_  && !simu_){
        ROS_WARN("[action_manager] Robot failed to grab (gripper empty)");
        return false;
    }

    //remove human attachment
    ros::ServiceClient rm_attach = connector_->node_.serviceClient<roboergosum_msgs::String>("human_manager/remove_attachment");
    roboergosum_msgs::String srv;
    srv.request.data = object_;
    if (!rm_attach.call(srv)){
     ROS_ERROR("Failed to call service human_manager/remove_attachment");
     }

    //add effects to the database
    std::vector<toaster_msgs::Fact> effects;
    toaster_msgs::Fact fact;
    fact.subjectId = "NULL";
    fact.property = "isHoldBy";
    fact.targetId = giver_;
    effects.push_back(fact);
    fact.subjectId = object_;
    fact.property = "isHoldBy";
    fact.targetId = robotName_;
    effects.push_back(fact);
    addFactsToDB(effects);

    return true;
}
