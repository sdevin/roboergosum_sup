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
 *  - the robot should have the object in hand
 *
 * \return true if the preconditions are checked, else return false
 * */
bool Drop::preconditions(){

    //First we check if the container is a known container object
    if(!isContainerObject(container_)){
      ROS_WARN("[action_manager] The container where to drop is not a known container object");
      return false;
    }

    //Then we check if the robot has the object in hand and if the container is reachable
    std::vector<toaster_msgs::Fact> precsTocheck;
    toaster_msgs::Fact fact;
    fact.subjectId = object_;
    fact.property = "isHoldBy";
    fact.targetId = robotName_;
    precsTocheck.push_back(fact);
    /*fact.subjectId = container_;
    fact.property = "isReachableBy";
    fact.targetId = robotName_;
    precsTocheck.push_back(fact);*/

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

    /*
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

    //Now we can plan the drop
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
    }

    return true;*/
    ros::Duration(1.0).sleep();
    if(container_ == "GREEN_TRASHBIN"){
        std::vector<toaster_msgs::Fact> precsTocheck;
        toaster_msgs::Fact fact;
        fact.subjectId = robotName_;
        fact.property = "isAt";
        fact.targetId = "ROBOT_LOC";
        precsTocheck.push_back(fact);

        return AreFactsInDB(precsTocheck);
    }else if(container_ == "BLUE_TRASHBIN"){
        std::vector<toaster_msgs::Fact> precsTocheck;
        toaster_msgs::Fact fact;
        fact.subjectId = robotName_;
        fact.property = "isAt";
        fact.targetId = "SECOND_LOC";
        precsTocheck.push_back(fact);

        return AreFactsInDB(precsTocheck);
    }

    return false;

}

/**
 * \brief Execute the drop action
 *
 * Execute with pr2motion the solution found by gtp
 *
 * \return true if the execution is a success, else return false
 * */
bool Drop::exec(){

    //move the arm
    pr2motion::Arm_Right_MoveToQGoalGoal goalQ;
    goalQ.traj_mode.value=pr2motion::pr2motion_TRAJ_MODE::pr2motion_TRAJ_GATECH;
    goalQ.shoulder_pan_joint = -1.952888;
    goalQ.shoulder_lift_joint = -0.095935;
    goalQ.upper_arm_roll_joint = -0.601572;
    goalQ.elbow_flex_joint = -1.600124;
    goalQ.forearm_roll_joint = 0.018247;
    goalQ.wrist_flex_joint = -0.432897;
    goalQ.wrist_roll_joint = -1.730082;\
    connector_->PR2motion_arm_right_Q_->sendGoal(goalQ);
    connector_->rightArmMoving_ = true;
    ROS_INFO("[action_manager] Waiting for arms move");
    bool finishedBeforeTimeout = connector_->PR2motion_arm_right_Q_->waitForResult(ros::Duration(connector_->waitActionServer_));
    connector_->rightArmMoving_ = false;
    if (!finishedBeforeTimeout){
       ROS_INFO("Action PR2 go to Q did not finish before the time out.");
    }

    //drop the tape
    moveGripper(0, true); //open
    RemoveFromHand(object_);

    return true;

    //return execGTPAction(GTPActionId_, true, object_);
}

/**
 * \brief Post-conditions for the drop action and add effects
 *
 * Place the object in the container
 * The effects are:
 *   - NULL isHoldBy robot
 *   - object isIn container
 *
 * \return true if the post-conditions are checked, else return false
 * */
bool Drop::post(){

    PutInContainer(object_, container_);

    //add effects to the database
    std::vector<toaster_msgs::Fact> effects;
    toaster_msgs::Fact fact;
    fact.subjectId = "NULL";
    fact.property = "isHoldBy";
    fact.targetId = robotName_;
    effects.push_back(fact);
    fact.subjectId = object_;
    fact.property = "isIn";
    fact.targetId = container_;
    effects.push_back(fact);
    addFactsToDB(effects);

    return true;
}
