/************
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Class for connection to others module
 * **********/

#ifndef CONNECTOR_H
#define CONNECTOR_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "roboergosum_msgs/ActionManagerAction.h"
#include <gtp_ros_msg/requestAction.h>
#include <pr2motion/Arm_Right_MoveAction.h>
#include <pr2motion/Arm_Left_MoveAction.h>
#include <pr2motion/Arm_Right_MoveToQGoalAction.h>
#include <pr2motion/Arm_Left_MoveToQGoalAction.h>
#include <pr2motion/Gripper_Right_OperateAction.h>
#include <pr2motion/Gripper_Left_OperateAction.h>
#include <pr2motion/InitAction.h>
#include <pr2motion/connect_port.h>
#include <pr2motion/Torso_MoveAction.h>


#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionServer<roboergosum_msgs::ActionManagerAction> Server;

/**
 * @brief contains the connection to other modules and informations to keep from an action to another
 */
class Connector{
public:
    Connector();

    ros::NodeHandle node_;/**< node handle*/
    Server* action_server_; /**< the action serveur*/
    std::string robotName_; /**< the name of the robot*/
    bool simu_; /**< flag which is true of working in simulation*/
    std::vector<std::string> manipulableObjects_; /**< list of manipulable objects*/
    std::vector<std::string> supportObjects_; /**< list of support objects*/
    std::vector<std::string> containerObjects_; /**< list of container objects*/
    std::vector<std::string> agentsList_; /**< list of existing agents*/
    std::vector<std::string> locationsList_; /**< list of existing locations*/
    int previousGTPId_; /**< gtp id of the previous action*/
    int nbPlanMax_; /**< nb of time we should try to call gtp to get a plan*/
    double waitActionServer_; /**< time we should wait for an action server*/
    int idGrasp_; /**< id of the gtp task where the robot grasps the last object*/
    int armGrasp_; /**< 0 if the robot has an object in the right hand, 1 in the left, -1 no object*/

    bool gripperRightOpen_; /**< flag which is true if the robot right gripper is open*/
    bool gripperLeftOpen_; /**< flag which is true if the robot left gripper is open*/
    bool rightArmMoving_; /**< flag which is true if the robot right arm is moving*/
    bool leftArmMoving_; /**< flag which is true if the robot left arm is moving*/
    bool rightGripperMoving_; /**< flag which is true if the robot right gripper is moving*/
    bool leftGripperMoving_; /**< flag which is true if the robot left gripper is moving*/
    bool torsoMoving_; /**< flag which is true if the robot torso is moving*/
    bool stopOrder_; /**< flag true when the robot has to stop its action*/
    bool humanNoHandover_; /**< flag true if we consider that the human will not contribute to handover*/

    std::string rightArmPose_; /**< actuel robot right arm pose*/
    std::string leftArmPose_; /**< actuel robot left arm pose*/
    std::string rightArmRestPose_; /**< rest robot right arm pose*/
    std::string leftArmRestPose_; /**< rest robot left arm pose*/

    actionlib::SimpleActionClient<gtp_ros_msg::requestAction>* acGTP_;/**< gtp action server*/
    actionlib::SimpleActionClient<pr2motion::InitAction>* PR2motion_init_;/**< pr2motion init action server*/
    actionlib::SimpleActionClient<pr2motion::Torso_MoveAction>* PR2motion_torso_;/**< pr2motion torso action server*/
    actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveAction>* PR2motion_arm_right_;/**< pr2motion right arm action server*/
    actionlib::SimpleActionClient<pr2motion::Arm_Left_MoveAction>* PR2motion_arm_left_;/**< pr2motion left arm action server*/
    actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveToQGoalAction>* PR2motion_arm_right_Q_;/**< pr2motion right arm action server for move to Q*/
    actionlib::SimpleActionClient<pr2motion::Arm_Left_MoveToQGoalAction>* PR2motion_arm_left_Q_;/**< pr2motion left arm action server for move to Q*/
    actionlib::SimpleActionClient<pr2motion::Gripper_Right_OperateAction>* PR2motion_gripper_right_;/**< pr2motion right gripper action server*/
    actionlib::SimpleActionClient<pr2motion::Gripper_Left_OperateAction>* PR2motion_gripper_left_;/**< pr2motion left gripper action server*/

};

#endif // CONNECTOR_H

