/************
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Virtual class to describe an action
 * **********/

#include <action_manager/virtual_action.h>


/**
 * \brief Constructor of the class
 *
 * @param connector pointer to the connector object
 * */
VirtualAction::VirtualAction(Connector* connector){

    connector_ = connector;
    node_.getParam("/action_manager/shouldUseRightHand", shouldUseRightHand_);
    robotName_ = connector_->robotName_;
    simu_ = connector_->simu_;
    gripperEmpty_ = false;
}


/**
* \brief Function which return true if an object is a manipulable object (based on parameters)
* @param object the tested object
* \return true if the object is a manipulable object
*/
bool VirtualAction::isManipulableObject(std::string object){

   //We check if the object is in the list of manipulable objects
   for(std::vector<std::string>::iterator it = connector_->manipulableObjects_.begin(); it != connector_->manipulableObjects_.end(); it++){
      if(*it == object){
         return true;
      }
   }

   return false;

}

/**
* \brief Function which return true if an object is a support object (based on parameters)
* @param support the tested object
* \return true if the object is a support object
*/
bool VirtualAction::isSupportObject(std::string support){

    //We check if the object is in the list of supports objects
    for(std::vector<std::string>::iterator it = connector_->supportObjects_.begin(); it != connector_->supportObjects_.end(); it++){
       if(*it == support){
          return true;
       }
    }

   return false;

}

/**
* \brief Function which return true if an object is a container object (based on parameters)
* @param container the tested object
* \return true if the object is a container object
*/
bool VirtualAction::isContainerObject(std::string container){

    //We check if the object is in the list of container objects
    for(std::vector<std::string>::iterator it = connector_->containerObjects_.begin(); it != connector_->containerObjects_.end(); it++){
       if(*it == container){
          return true;
       }
    }

   return false;

}

/**
* \brief Function which return true if some facts are on the databases
* @param facts the facts
* \return true if the tested facts are in the robot table of the database
*/
bool VirtualAction::AreFactsInDB(std::vector<toaster_msgs::Fact> facts){

    ros::ServiceClient client = node_.serviceClient<toaster_msgs::ExecuteDB>("database_manager/execute");
    toaster_msgs::ExecuteDB srv;
    srv.request.command = "ARE_IN_TABLE";
    srv.request.agent = robotName_;
    srv.request.facts = facts;
    if (client.call(srv)){
        return srv.response.boolAnswer;
    }else{
       ROS_ERROR("[action_manager] Failed to call service mental_states/get_info");
    }
    return false;
}


/**
* \brief Function which update GTP world state with TOASTER
* \return true if success
*/
bool VirtualAction::updateGTP(){

  // send goal to GTP
  gtp_ros_msg::requestGoal goal;
  goal.req.requestType = "update";
  connector_->acGTP_->sendGoal(goal);

  //wait for the action to return
  bool finishedBeforeTimeout = connector_->acGTP_->waitForResult(ros::Duration(connector_->waitActionServer_));

  if (!finishedBeforeTimeout){
    ROS_INFO("[action_manager] Failed to update GTP: Action did not finish before the time out.");
    return false;
  }

   return true;
}

/**
* \brief Function which ask a plan to GTP and return the id of the solution (-1 if no solution)
* @param actionName name of the action
* @param agents agents involved in the action
* @param objects objects involved in the action
* @param datas datas involved in the action
* @param points points involved in the action
* \return the id of the task return by gtp, -1 if failure
*/
int VirtualAction::planGTP(std::string actionName, std::vector<gtp_ros_msg::Ag> agents, std::vector<gtp_ros_msg::Obj> objects, std::vector<gtp_ros_msg::Data> datas, std::vector<gtp_ros_msg::Points> points){

  updateGTP();

  gtp_ros_msg::requestGoal goal;
  goal.req.requestType = "planning";
  goal.req.actionName = actionName;
  goal.req.involvedAgents = agents;
  goal.req.involvedObjects = objects;
  goal.req.data = datas;
  goal.req.points = points;
  goal.req.predecessorId.actionId = connector_->previousGTPId_;
  goal.req.predecessorId.alternativeId = 0;

  //we try several time since GTP can take several try to find a solution
  int nbTry = 0;
  while(nbTry < connector_->nbPlanMax_){
     connector_->acGTP_->sendGoal(goal);
     bool finishedBeforeTimeout = connector_->acGTP_->waitForResult(ros::Duration(connector_->waitActionServer_));

     if (finishedBeforeTimeout)
     {
       if(connector_->acGTP_->getResult()->ans.success){
           return connector_->acGTP_->getResult()->ans.identifier.actionId;
       }
     }
     else{
         ROS_INFO("[action_manager] Failed to get plan: GTP Action did not finish before the time out.");
         return -1;
     }

      nbTry++;
   }

   return -1;
}

/**
* \brief Function which execute an action based on its GTP id
* @param GTPActionId gtp id for the action
* @param shouldOpen indicate if the gripper should be open before execution
* @param object object involved in the action if there is
* \return true if success
*/
bool VirtualAction::execGTPAction(int GTPActionId, bool shouldOpen, std::string object){

  gtp_ros_msg::requestGoal goal;
  goal.req.requestType = "details";
  goal.req.loadAction.actionId = GTPActionId;
  goal.req.loadAction.alternativeId = 0;

  connector_->acGTP_->sendGoal(goal);
  bool finishedBeforeTimeout = connector_->acGTP_->waitForResult(ros::Duration(connector_->waitActionServer_));

  if (finishedBeforeTimeout){
     std::vector<gtp_ros_msg::SubTraj> subTrajs = connector_->acGTP_->getResult()->ans.subTrajs;
     if(shouldOpen && ((subTrajs[0].armId== 0 && !connector_->gripperRightOpen_) || (subTrajs[0].armId== 1 && !connector_->gripperLeftOpen_))){//the robot should have the gripper open to execute the trajectory
        moveGripper(subTrajs[0].armId, true); //open
     }
     for(std::vector<gtp_ros_msg::SubTraj>::iterator it = subTrajs.begin(); it != subTrajs.end(); it++){
         if(connector_->action_server_->isPreemptRequested() || connector_->stopOrder_){
            return false;
         }
         if(it->agent == robotName_){
            if(it->subTrajName == "grasp"){
                moveGripper(it->armId, false); //close
                std::string hand;
                if(it->armId == 0){
                    hand = "right";
                }else{
                    hand = "left";
                }
                if(!gripperEmpty_  || simu_){
                    PutInHand(object, hand, GTPActionId);
                }
            }else if(it->subTrajName == "release"){
                moveGripper(it->armId, true); //open
                RemoveFromHand(object);
            }else{//this is a trajectory
                executeTrajectory(GTPActionId, it->subTrajId, it->armId);
            }
         }
     }
  }else{
     ROS_INFO("[action_manager] Failed to execute the action: GTP get details did not finish before the time out.");
     return false;
  }

  connector_->previousGTPId_ = GTPActionId;
  return true;

}

/**
* \brief Function which execute a GTP trajectory
* @param GTPActionId gtp id for the action
* @param actionSubId id of the trajectory
* @param armId id of the arm to move
* \return true if success
*/
bool VirtualAction::executeTrajectory(int GTPActionId, int actionSubId, int armId){

   gtp_ros_msg::requestGoal goal;
   goal.req.requestType = "load";
   goal.req.loadAction.actionId = GTPActionId;
   goal.req.loadAction.alternativeId = 0;
   goal.req.loadSubTraj = actionSubId;

   connector_->acGTP_->sendGoal(goal);
   bool finishedBeforeTimeout = connector_->acGTP_->waitForResult(ros::Duration(connector_->waitActionServer_));

   if (finishedBeforeTimeout){
     if(armId == 0){//right arm
        pr2motion::Arm_Right_MoveGoal arm_goal_right;
        arm_goal_right.traj_mode.value=pr2motion::pr2motion_TRAJ_MODE::pr2motion_TRAJ_GATECH;
        arm_goal_right.path_mode.value=pr2motion::pr2motion_PATH_MODE::pr2motion_PATH_PORT;
        connector_->rightArmMoving_ = true;
        connector_->PR2motion_arm_right_->sendGoal(arm_goal_right,  boost::bind(&VirtualAction::moveRightArm, this, _1, _2),
                                                   Client_Right_Arm::SimpleActiveCallback(),  Client_Right_Arm::SimpleFeedbackCallback());
        while(connector_->rightArmMoving_ == true){
            //wait for preempted request or end of the action
            if(connector_->action_server_->isPreemptRequested() || connector_->stopOrder_ ){
                connector_->PR2motion_arm_right_->cancelGoal();
                return false;
            }
        }
        connector_->rightArmPose_ = "unknown";
     }else{
        pr2motion::Arm_Left_MoveGoal arm_goal_left;
        arm_goal_left.traj_mode.value=pr2motion::pr2motion_TRAJ_MODE::pr2motion_TRAJ_GATECH;
        arm_goal_left.path_mode.value=pr2motion::pr2motion_PATH_MODE::pr2motion_PATH_PORT;
        connector_->leftArmMoving_ = true;
        connector_->PR2motion_arm_left_->sendGoal(arm_goal_left,  boost::bind(&VirtualAction::moveLeftArm, this, _1, _2),
                                                   Client_Left_Arm::SimpleActiveCallback(),  Client_Left_Arm::SimpleFeedbackCallback());
        while(connector_->leftArmMoving_ == true){
            //wait for preempted request or end of the action
            if(connector_->action_server_->isPreemptRequested() || connector_->stopOrder_){
                connector_->PR2motion_arm_left_->cancelGoal();
                return false;
            }
        }
        connector_->leftArmPose_ = "unknown";

     }
   }
   else{
    ROS_INFO("[action_manager] Failed to execute the trajectory: GTP load did not finish before the time out.");
    return false;
   }

   return true;

}

/**
* \brief Function to open or close a gripper
* @param armId id of the arm of the gripper to close
* @param open true if to open the gripper, false to close
* \return true if success
*/
bool VirtualAction::moveGripper(int armId, bool open){

    bool finishedBeforeTimeout;
    if(armId == 0){//right arm
       pr2motion::Gripper_Right_OperateGoal gripper_goal;
       if(open){
           gripper_goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_OPEN;
       }else{
           gripper_goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_CLOSE;
       }
       connector_->rightGripperMoving_ = true;
       connector_->PR2motion_gripper_right_->sendGoal(gripper_goal,  boost::bind(&VirtualAction::moveRightGripper, this, _1, _2),
                                                  Client_Right_Gripper::SimpleActiveCallback(),  Client_Right_Gripper::SimpleFeedbackCallback());
       while(connector_->rightGripperMoving_ == true){
           //wait for preempted request or end of the action
           if(connector_->action_server_->isPreemptRequested() || connector_->stopOrder_){
               connector_->PR2motion_gripper_right_->cancelGoal();
               return false;
           }
       }
       if(open){
           connector_->gripperRightOpen_ = true;
       }else{
            connector_->gripperRightOpen_ = false;
            if(!simu_){
                gripperEmpty_  = isGripperEmpty("right");
            }
       }
    }else{
       pr2motion::Gripper_Left_OperateGoal gripper_goal;
       if(open){
           gripper_goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_OPEN;
       }else{
           gripper_goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_CLOSE;
       }
       connector_->leftGripperMoving_ = true;
       connector_->PR2motion_gripper_left_->sendGoal(gripper_goal,  boost::bind(&VirtualAction::moveLeftGripper, this, _1, _2),
                                                  Client_Left_Gripper::SimpleActiveCallback(),  Client_Left_Gripper::SimpleFeedbackCallback());
       while(connector_->leftGripperMoving_ == true){
           //wait for preempted request or end of the action
           if(connector_->action_server_->isPreemptRequested() || connector_->stopOrder_){
               connector_->PR2motion_gripper_left_->cancelGoal();
               return false;
           }
       }
       if(open){
           connector_->gripperLeftOpen_ = true;
       }else{
            connector_->gripperLeftOpen_ = false;
            if(!simu_){
                gripperEmpty_  = isGripperEmpty("left");
            }
       }
    }


   return true;

}

/**
* \brief Check if the gripper is completely closed in order to know if it is empty or not
* @param arm arm of the gripper we want to check
* \return true if the gripper is completly closed
*/
bool VirtualAction::isGripperEmpty(std::string arm){

    std::string gripperJoint;
    double gripperThreshold;
    std::string gripperTopic = "action_manager/gripperJoint/";
    gripperTopic = gripperTopic + arm;
    node_.getParam(gripperTopic, gripperJoint);
    node_.getParam("action_manager/gripperThreshold", gripperThreshold);
    toaster_msgs::RobotListStamped list;
    try{
        list  = *(ros::topic::waitForMessage<toaster_msgs::RobotListStamped>("pdg/robotList",ros::Duration(1)));
        for(std::vector<toaster_msgs::Robot>::iterator it = list.robotList.begin(); it != list.robotList.end(); it++){
          if(it->meAgent.meEntity.id == robotName_){
              for(std::vector<toaster_msgs::Joint>::iterator itj = it->meAgent.skeletonJoint.begin(); itj != it->meAgent.skeletonJoint.end(); itj++){
                  if(itj->meEntity.id == gripperJoint){
                      if(itj->position < gripperThreshold){
                          return true;
                      }else{
                          return false;
                      }
                  }
              }
          }
        }
    }
    catch(const std::exception & e){
        ROS_WARN("[action_manager] Failed to check if gripper empty: Failed to read robot pose from toaster");
    }
    return true;
}

/**
* \brief Function which puts an object in the hand of the robot
* @param object the object to put
* @param hand the hand to attach to (right or left)
* @param gtpId id of the gtp task associated to the grasp
*/
void VirtualAction::PutInHand(std::string object, std::string hand, int gtpId){

   ros::ServiceClient client = node_.serviceClient<toaster_msgs::PutInHand>("pdg/put_in_hand");

    //put the object in the hand of the robot
    std::string robotHand;
    std::string handTopic = "/roboergosum/robotHands/";
    handTopic = handTopic + hand;
    node_.getParam(handTopic, robotHand);
    std::string robotToasterName;
    node_.getParam("roergosum/toasterRobotName", robotToasterName);
    toaster_msgs::PutInHand srv;
    srv.request.objectId = object;
    srv.request.agentId = robotToasterName;
    srv.request.jointName = robotHand;
    if (!client.call(srv)){
     ROS_ERROR("[action_manager] Failed to call service pdg/put_in_hand");
    }
    //remember the gtp id of the grasp
    connector_->idGrasp_ = gtpId;

}

/**
* \brief Function which remove an object from the hand of the robot
* @param object the object to remove
*/
void VirtualAction::RemoveFromHand(std::string object){

   ros::ServiceClient client = node_.serviceClient<toaster_msgs::RemoveFromHand>("pdg/remove_from_hand");

    //remove the object from the hand of the robot
    toaster_msgs::RemoveFromHand srv;
    srv.request.objectId = object;
    if (!client.call(srv)){
     ROS_ERROR("[action_manager] Failed to call service pdg/remove_from_hand");
    }

}

/**
* \brief Function which add to gtp the id of the grasp for the object the robot has in hand
* @param id id of the grasp
* \return true if success
*/
bool VirtualAction::addGTPAttachment(int id){

    // send goal to GTP
    gtp_ros_msg::requestGoal goal;
    goal.req.requestType = "addAttachemnt";
    goal.req.loadAction.actionId = id;
    goal.req.loadAction.alternativeId = 0;
    connector_->acGTP_->sendGoal(goal);

    //wait for the action to return
    bool finishedBeforeTimeout = connector_->acGTP_->waitForResult(ros::Duration(connector_->waitActionServer_));

    if (!finishedBeforeTimeout){
      ROS_INFO("[action_manager] Failed to add attachment to GTP: Action did not finish before the time out.");
      return false;
    }

     return true;
}

/**
* \brief Function which puts an object on a support
* @param object the object to put
* @param support the support where to place the object
*/
void VirtualAction::PutOnSupport(std::string object, std::string support){

    ros::ServiceClient client;
    if(connector_->simu_){
    client = node_.serviceClient<toaster_msgs::SetEntityPose>("toaster_simu/set_entity_pose");
    }else{
    client = node_.serviceClient<toaster_msgs::SetEntityPose>("pdg/set_entity_pose");
    }

    double objectHeight, supportHeight;
    std::string objectHeightTopic = "entities/objectsHeight/bottom/";
    objectHeightTopic = objectHeightTopic + object;
    std::string supportHeightTopic = "entities/objectsHeight/top/";
    supportHeightTopic = supportHeightTopic + support;
    node_.getParam(objectHeightTopic, objectHeight);
    node_.getParam(supportHeightTopic, supportHeight);
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
    toaster_msgs::SetEntityPose srv;
    srv.request.id = object;
    srv.request.type = "object";
    srv.request.pose.position.x = x;
    srv.request.pose.position.y = y;
    srv.request.pose.position.z = z;
    srv.request.pose.orientation.x = 0.0;
    srv.request.pose.orientation.y = 0.0;
    srv.request.pose.orientation.z = 0.0;
    srv.request.pose.orientation.w = 1.0;
    if (!client.call(srv)){
     ROS_ERROR("Failed to call service pdg/set_entity_pose");
     }
    }
    catch(const std::exception & e){
    ROS_WARN("[action_executor] Failed to read %s pose from toaster", support.c_str());
    }

}

/**
* \brief Called once when the goal of the right arm action client completes
* @param state state of the right arm action server
* @param result result of the right arm action server
*/
void VirtualAction::moveRightArm(const actionlib::SimpleClientGoalState& state, const pr2motion::Arm_Right_MoveResultConstPtr& result){

        connector_->rightArmMoving_ = false;
}

/**
* \brief Called once when the goal of the left arm action client completes
* @param state state of the left arm action server
* @param result result of the left arm action server
*/
void VirtualAction::moveLeftArm(const actionlib::SimpleClientGoalState& state, const pr2motion::Arm_Left_MoveResultConstPtr& result){

        connector_->leftArmMoving_ = false;
}

/**
* \brief Called once when the goal of the right gripper action client completes
* @param state state of the right arm gripper server
* @param result result of the right arm gripper server
*/
void VirtualAction::moveRightGripper(const actionlib::SimpleClientGoalState& state, const pr2motion::Gripper_Right_OperateResultConstPtr& result){

        connector_->rightGripperMoving_ = false;
}

/**
* \brief Called once when the goal of the left gripper action client completes
* @param state state of the left gripper action server
* @param result result of the left gripper action server
*/
void VirtualAction::moveLeftGripper(const actionlib::SimpleClientGoalState& state, const pr2motion::Gripper_Left_OperateResultConstPtr& result){

        connector_->leftGripperMoving_ = false;
}
