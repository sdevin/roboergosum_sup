/************
 * \author Sandra Devin (sdevin@laas.fr)
 *
 * Class of the pick action
 * **********/

#include "action_manager/Actions/pick.h"


/**
 * Constructor of the class
 * @param action the description of the action to execute
 * @param connector pointer to the connector object
 * */
Pick::Pick(roboergosum_msgs::Action action, Connector* connector): VirtualAction(connector){
    //
    if(action.parameters.size() == 1){
        object_ = action.parameters[0];
    }else{
        ROS_WARN("[action_manager] Wrong paramters for the pick action, should be: object");
    }
}

/**
 * \brief Check the precondition of the pick action
 *
 * For the pick action the preconditions checked are:
 *  - the object should be a manipulable object
 *  - the object should be reachable by the robot
 *  - the robot should not have empty hands
 *
 * \return true if the preconditions are checked, else return false
 * */
bool Pick::preconditions(){

    //First we check if the object is a known manipulable object
    if(!isManipulableObject(object_)){
      ROS_WARN("[action_executor] The object to pick is not a known manipulable object");
      return false;
    }

    //Then we check if the robot has the hands free and if the object is reachable
    std::vector<toaster_msgs::Fact> precsTocheck;
    toaster_msgs::Fact fact;
    fact.subjectId = "NULL";
    fact.property = "isHoldBy";
    fact.targetId = robotName_;
    precsTocheck.push_back(fact);
    fact.subjectId = object_;
    fact.property = "isReachableBy";
    fact.targetId = robotName_;
    precsTocheck.push_back(fact);

    return AreFactsInDB(precsTocheck);
}

/**
 * \brief Plan for the pick action
 *
 * Ask to gtp a solution to pick the object
 *
 * \return true if the planning is a success, else return false
 * */
bool Pick::plan(){

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
        return false;
    }

    return true;
}

/**
 * \brief Execute the pick action
 *
 * Execute with pr2motion the solution found by gtp
 *
 * \return true if the execution is a success, else return false
 * */
bool Pick::exec(){

    return execGTPAction(GTPActionId_, true, object_);
}

/**
 * \brief Post-conditions for the pick action
 *
 * Check that the gripper of the robot is not completly closed (meaning that the robot misses the object, not in simu)
 *
 * \return true if the post-conditions are checked, else return false
 * */
bool Pick::post(){

    //Check gripper position (completly close or not)
    if(gripperEmpty_  && !simu_){
        ROS_WARN("[action_manager] Robot failed to pick (gripper empty)");
        return false;
    }

    return true;
}
