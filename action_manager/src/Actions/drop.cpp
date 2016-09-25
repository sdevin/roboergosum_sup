/************
 * \author Sandra Devin (sdevin@laas.fr)
 *
 * Class of the drop action
 * **********/

#include "action_manager/Actions/drop.h"


/**
 * Constructor of the class
 * @param action the description of the action to execute
 * @param connector pointer to the connector object
 * */
Drop::Drop(roboergosum_msgs::Action action, Connector* connector): VirtualAction(connector){
    //
    if(action.parameters.size() == 2){
        object_ = action.parameters[0];
        container_ = action.parameters[1];
    }else{
        ROS_WARN("[action_manager] Wrong paramters for the place action, should be: object, container");
    }
}

/**
 * \brief Check the precondition of the drop action
 *
 * For the place action the preconditions checked are:
 *  - the container should be a container object
 *  - the container should be reachable by the robot
 *  - the robot should not have the object in hand
 *
 * \return true if the preconditions are checked, else return false
 * */
bool Drop::preconditions(){

    //First we check if the container is a known container object
    if(!isContainerObject(container_)){
      ROS_WARN("[action_executor] The container where to drop is not a known container object");
      return false;
    }

    //Then we check if the robot has the object in hand and if the container is reachable
    std::vector<toaster_msgs::Fact> precsTocheck;
    toaster_msgs::Fact fact;
    fact.subjectId = object_;
    fact.property = "isHoldBy";
    fact.targetId = robotName_;
    precsTocheck.push_back(fact);
    fact.subjectId = container_;
    fact.property = "isReachableBy";
    fact.targetId = robotName_;
    precsTocheck.push_back(fact);

    return AreFactsInDB(precsTocheck);
}

/**
 * \brief Plan for the drop action
 *
 * Verify if we need to add the graps of the pick.
 * Ask to gtp a solution to drop the object
 *
 * \return true if the planning is a success, else return false
 * */
bool Drop::plan(){

    //if the previous gtp id is -1, we need to look for the id of the grasp
    if(connector_->previousGTPId_ == -1){
        if(connector_->idGrasp_ != -1){
            ROS_WARN("[action_manager] Drop failed in planning: no previous id for the grasp");
        }else{
            //We add the grasp in gtp
            if(!addGTPAttachment(connector_->idGrasp_)){
                //Failure msg already in the function
                return false;
            }
        }
    }

    //Now we can plan the place
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
    object.actionKey = "supportObject";
    object.objectName = container_;
    objects.push_back(object);
    std::vector<gtp_ros_msg::Points> points;
    std::vector<gtp_ros_msg::Data> datas;

    if(shouldUseRightHand_){
        gtp_ros_msg::Data data;
        data.dataKey = "hand";
        data.dataValue = "right";
        datas.push_back(data);
    }

    GTPActionId_ = planGTP("drop", agents, objects, datas, points);

    if(GTPActionId_ == -1){
        return false;
    }else{
        return true;
    }

    return true;
}

/**
 * \brief Execute the drop action
 *
 * Execute with pr2motion the solution found by gtp
 *
 * \return true if the execution is a success, else return false
 * */
bool Drop::exec(){

    return execGTPAction(GTPActionId_, true, object_);
}

/**
 * \brief Post-conditions for the drop action
 *
 * Place the object in the container
 *
 * \return true if the post-conditions are checked, else return false
 * */
bool Drop::post(){

    PutInContainer(object_, container_);

    return true;
}
