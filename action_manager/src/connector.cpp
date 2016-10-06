/************
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Class for connection to others module
 * **********/

#include <action_manager/connector.h>

/**
 * \brief Constructor of the class
 * */
Connector::Connector(){
    node_.getParam("/roboergosum/robotName", robotName_);
    node_.getParam("/roboergosum/simu", simu_);
    node_.getParam("/entities/objects", manipulableObjects_);
    node_.getParam("/entities/supports", supportObjects_);
    node_.getParam("/entities/containers", containerObjects_);
    node_.getParam("/entities/agents", agentsList_);
    node_.getParam("/entities/locations", locationsList_);
    node_.getParam("/roboergosum/waitActionServer", waitActionServer_);
    node_.getParam("/action_manager/nbPlanMaxGTP", nbPlanMax_);
    node_.getParam("roboergosum/humanLazy", humanLazy_);

    previousGTPId_ = -1;
    idGrasp_ = -1;
    gripperRightOpen_ = false;
    gripperLeftOpen_ = false;
    rightArmMoving_ = false;
    leftArmMoving_ = false;
    rightGripperMoving_ = false;
    leftGripperMoving_ = false;
    torsoMoving_ = false;
    stopOrder_ = false;
    rightArmPose_ = "unknown";
    leftArmPose_ = "unknown";

    //Init action clients
    ROS_INFO("Waiting for gtp action server.");
    acGTP_ = new actionlib::SimpleActionClient<gtp_ros_msg::requestAction>("gtp_ros_server", true);
    acGTP_->waitForServer();
    ROS_INFO("Waiting for pr2motion actions server.");
    PR2motion_init_ = new actionlib::SimpleActionClient<pr2motion::InitAction>("pr2motion/Init", true);
    PR2motion_init_->waitForServer();
    PR2motion_torso_ = new actionlib::SimpleActionClient<pr2motion::Torso_MoveAction>("pr2motion/Torso_Move", true);
    PR2motion_torso_->waitForServer();
    PR2motion_arm_right_ = new actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveAction>("pr2motion/Arm_Right_Move",true);
    PR2motion_arm_right_->waitForServer();
    PR2motion_arm_left_ = new actionlib::SimpleActionClient<pr2motion::Arm_Left_MoveAction>("pr2motion/Arm_Left_Move",true);
    PR2motion_arm_left_->waitForServer();
    PR2motion_gripper_right_ = new actionlib::SimpleActionClient<pr2motion::Gripper_Right_OperateAction>("pr2motion/Gripper_Right_Operate",true);
    PR2motion_gripper_right_->waitForServer();
    PR2motion_gripper_left_ = new actionlib::SimpleActionClient<pr2motion::Gripper_Left_OperateAction>("pr2motion/Gripper_Left_Operate",true);
    PR2motion_gripper_left_->waitForServer();
    ROS_INFO("[action_manager] Action clients started.");

    //Init PR2motion
    ros::ServiceClient connect = node_.serviceClient<pr2motion::connect_port>("pr2motion/connect_port");
    pr2motion::InitGoal goal_init;
    PR2motion_init_->sendGoal(goal_init);

    pr2motion::connect_port srv;
    srv.request.local = "joint_state";
    srv.request.remote = "joint_states";
    if (!connect.call(srv)){
       ROS_ERROR("[action_manager] Failed to call service pr2motion/connect_port");
    }
    srv.request.local = "head_controller_state";
    srv.request.remote = "/head_traj_controller/state";
    if (!connect.call(srv)){
        ROS_ERROR("[action_manager] Failed to call service pr2motion/connect_port");
    }
    srv.request.local = "traj";
    srv.request.remote = "gtp_trajectory";
    if (!connect.call(srv)){
        ROS_ERROR("[action_manager] Failed to call service pr2motion/connect_port");
    }

    if(simu_){//change torso position
       pr2motion::Torso_MoveGoal goal;
       goal.torso_position = 0.1;
       torsoMoving_ = true;
       PR2motion_torso_->sendGoal(goal);
       ROS_INFO("[action_manager] Waiting for Torso move");
       bool finishedBeforeTimeout = PR2motion_torso_->waitForResult(ros::Duration(waitActionServer_));
       torsoMoving_ = false;
       if (!finishedBeforeTimeout){
          ROS_INFO("Action PR2Torso did not finish before the time out.");
       }
    }
}
