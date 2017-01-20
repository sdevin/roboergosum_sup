/************
 * \author Sandra Devin (sdevin@laas.fr)
 *
 * Class of the place action
 * **********/

#include "action_manager/Actions/place.h"


/**
 * Constructor of the class
 * @param action the description of the action to execute
 * @param connector pointer to the connector object
 * */
Place::Place(roboergosum_msgs::Action action, Connector* connector): VirtualAction(connector){

    if(action.parameters.size() == 2){
        object_ = action.parameters[0];
        support_ = action.parameters[1];
    }else{
        ROS_WARN("[action_manager] Wrong paramters for the place action, should be: object, support");
    }
}

/**
 * \brief Check the precondition of the place action
 *
 * For the place action the preconditions checked are:
 *  - the support should be a support object
 *  - the support should be reachable by the robot
 *  - the robot should have the object in hand
 *
 * \return true if the preconditions are checked, else return false
 * */
bool Place::preconditions(){

    //First we check if the object is a known support object
    if(!isSupportObject(support_)){
      ROS_WARN("[action_manager] The support where to place is not a known support object");
      return false;
    }

    //Then we check if the robot has the object in hand and if the support is reachable
    std::vector<toaster_msgs::Fact> precsTocheck;
    toaster_msgs::Fact fact;
    fact.subjectId = object_;
    fact.property = "isHoldBy";
    fact.targetId = robotName_;
    precsTocheck.push_back(fact);
    fact.subjectId = support_;
    fact.property = "isReachableBy";
    fact.targetId = robotName_;
    precsTocheck.push_back(fact);

    return AreFactsInDB(precsTocheck);
}

/**
 * \brief Plan for the place action
 *
 * Verify if we need to add the graps of the pick.
 * Ask to gtp a solution to place the object
 *
 * \return true if the planning is a success, else return false
 * */
bool Place::plan(){

    //if the previous gtp id is -1, we need to look for the id of the grasp
    /*if(connector_->previousGTPId_ == -1){
        if(connector_->idGrasp_ != -1){
            ROS_WARN("[action_manager] Place failed in planning: no previous id for the grasp");
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
    object.objectName = support_;
    objects.push_back(object);
    std::vector<gtp_ros_msg::Points> points;
    std::vector<gtp_ros_msg::Data> datas;

    if(shouldUseRightHand_){
        gtp_ros_msg::Data data;
        data.dataKey = "hand";
        data.dataValue = "right";
        datas.push_back(data);
    }

    GTPActionId_ = planGTP("place", agents, objects, datas, points);

    if(GTPActionId_ == -1){
        return false;
    }*/

    ros::Duration(1.0).sleep();
    return true;
}

/**
 * \brief Execute the place action
 *
 * Execute with pr2motion the solution found by gtp
 *
 * \return true if the execution is a success, else return false
 * */
bool Place::exec(){

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

    //place the tape
    moveGripper(0, true); //open
    RemoveFromHand(object_);

    return true;

    //return execGTPAction(GTPActionId_, true, object_);
}

/**
 * \brief Post-conditions for the place action and add effects
 *
 * Place the object on the support
 * The effects are:
 *   - NULL isHoldBy robot
 *   - object isOn support
 *
 * \return true if the post-conditions are checked, else return false
 * */
bool Place::post(){

    PutOnSupport(object_, support_);

    //add effects to the database
    std::vector<toaster_msgs::Fact> effects;
    toaster_msgs::Fact fact;
    fact.subjectId = "NULL";
    fact.property = "isHoldBy";
    fact.targetId = robotName_;
    effects.push_back(fact);
    fact.subjectId = object_;
    fact.property = "isOn";
    fact.targetId = support_;
    effects.push_back(fact);

    std::vector<std::string> supportLocations;
    std::string locationTopic = "/environment/locations/" + support_;
    connector_->node_.getParam(locationTopic, supportLocations);
    for(std::vector<std::string>::iterator itl = supportLocations.begin(); itl != supportLocations.end(); itl++){
        fact.subjectId = object_;
        fact.property = "isAt";
        fact.targetId = *itl;
        effects.push_back(fact);
    }

    addFactsToDB(effects);


    return true;
}
