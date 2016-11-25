/************
 * \author Sandra Devin (sdevin@laas.fr)
 *
 * Class of the navigate action
 * **********/

#include "action_manager/Actions/navigate.h"


/**
 * Constructor of the class
 * @param action the description of the action to execute
 * @param connector pointer to the connector object
 * */
Navigate::Navigate(roboergosum_msgs::Action action, Connector* connector): VirtualAction(connector){

    if(action.parameters.size() == 1){
        location_ = action.parameters[0];
    }else{
        ROS_WARN("[action_manager] Wrong paramters for the pick action, should be: location");
    }

    //initialize move base action client
    client_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
    client_->waitForServer();
}

/**
 * \brief Check the precondition of the navigate action
 *
 * For the navigate action the preconditions checked are:
 *  - the location should be a known location
 *
 * \return true if the preconditions are checked, else return false
 * */
bool Navigate::preconditions(){

    //First we check if the object is a known manipulable object
    if(!isLocation(location_)){
      ROS_WARN("[action_manager] The location to navigate is not a known location");
      return false;
    }

    return true;
}

/**
 * \brief Plan for the navigate action
 *
 * No planning needed (when on real robot: will need tuck arm)
 *
 * \return true if the planning is a success, else return false
 * */
bool Navigate::plan(){

    return true;
}

/**
 * \brief Execute the navigate action
 *
 * Use move_base to go to the location
 *
 * \return true if the execution is a success, else return false
 * */
bool Navigate::exec(){

    //first we get the position of the location
    std::string xParam = "action_manager/locationsPose/" + location_ + "/x";
    std::string yParam = "action_manager/locationsPose/" + location_ + "/y";
    std::string xRotParam = "action_manager/locationsPose/" + location_ + "/x_rot";
    std::string yRotParam = "action_manager/locationsPose/" + location_ + "/y_rot";
    std::string zRotParam = "action_manager/locationsPose/" + location_ + "/z_rot";
    std::string wRotParam = "action_manager/locationsPose/" + location_ + "/w_rot";
    double x, y, x_rot, y_rot, z_rot, w_rot;
    connector_->node_.getParam(xParam, x);
    connector_->node_.getParam(yParam, y);
    connector_->node_.getParam(xRotParam, x_rot);
    connector_->node_.getParam(yRotParam, y_rot);
    connector_->node_.getParam(zRotParam, z_rot);
    connector_->node_.getParam(wRotParam, w_rot);

    //then we call move base
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.x = x_rot;
    goal.target_pose.pose.orientation.y = y_rot;
    goal.target_pose.pose.orientation.z = z_rot;
    goal.target_pose.pose.orientation.w = w_rot;
    client_->sendGoal(goal);
    ROS_INFO("[action_manager] Move base goal sent %f %f %f %f %f %f",goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.x , goal.target_pose.pose.orientation.y , goal.target_pose.pose.orientation.z , goal.target_pose.pose.orientation.w);
    client_->waitForResult(ros::Duration(300.0));
    if(client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_WARN("[action_manager] Move base failed in execution");
        return false;
    }

    return true;
}

/**
 * \brief Post-conditions for the navigate action and add effects
 *
 * The effects are:
 * - set the robot at the new location
 *
 * \return true if the post-conditions are checked, else return false
 * */
bool Navigate::post(){


    //add effects to the database
    std::vector<toaster_msgs::Fact> effects;
    toaster_msgs::Fact fact;
    fact.subjectId = robotName_;
    fact.property = "isAt";
    fact.targetId = "NULL";
    effects.push_back(fact);
    fact.subjectId = robotName_;
    fact.property = "isAt";
    fact.targetId = location_;
    effects.push_back(fact);
    addFactsToDB(effects);

    return true;
}
