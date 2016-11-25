/************
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Virtual class to describe an action
 * **********/

#ifndef VIRTUALACTION_H
#define VIRTUALACTION_H

#include "action_manager/connector.h"
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/ExecuteDB.h"
#include "toaster_msgs/SetInfoDB.h"
#include <gtp_ros_msg/requestAction.h>
#include "toaster_msgs/RobotListStamped.h"
#include "toaster_msgs/ObjectListStamped.h"
#include "toaster_msgs/PutInHand.h"
#include "toaster_msgs/RemoveFromHand.h"
#include "toaster_msgs/SetEntityPose.h"
#include "roboergosum_msgs/String.h"

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveAction> Client_Right_Arm;
typedef actionlib::SimpleActionClient<pr2motion::Arm_Left_MoveAction> Client_Left_Arm;
typedef actionlib::SimpleActionClient<pr2motion::Gripper_Right_OperateAction> Client_Right_Gripper;
typedef actionlib::SimpleActionClient<pr2motion::Gripper_Left_OperateAction> Client_Left_Gripper;

/**
 * @brief virtual class from which each action class inherits
 */
class VirtualAction{
public:
    VirtualAction(Connector* connector);
    /**
     * Destructor of the class
     * */
	~VirtualAction() {};
    /**
     * virtual function to check preconditions of the action
     * */
	virtual bool preconditions() = 0;
    /**
     * virtual function to plan for the action
     * */
	virtual bool plan() = 0;
    /**
     * virtual function to execute the action
     * */
    virtual bool exec() = 0;
    /**
     * virtual function to apply and check post condition of the action
     * */
    virtual bool post() = 0;

    void moveRightArm(const actionlib::SimpleClientGoalState& state, const pr2motion::Arm_Right_MoveResultConstPtr& result);
    void moveLeftArm(const actionlib::SimpleClientGoalState& state, const pr2motion::Arm_Left_MoveResultConstPtr& result);
    void moveRightGripper(const actionlib::SimpleClientGoalState& state, const pr2motion::Gripper_Right_OperateResultConstPtr& result);
    void moveLeftGripper(const actionlib::SimpleClientGoalState& state, const pr2motion::Gripper_Left_OperateResultConstPtr& result);

protected:
   bool shouldUseRightHand_; /**< flag which indicates if the robot should mandatorily use the right hand*/
   Connector* connector_; /**< pointer to the connector object*/
   std::string robotName_; /**< the robot name*/
   bool simu_; /**< flag which is true of working in simulation*/
   int GTPActionId_; /**< gtp id for the action */
   bool gripperEmpty_; /**< flag which is true if the robot gripper is empty after a pick */

   bool isManipulableObject(std::string object);
   bool isSupportObject(std::string support);
   bool isContainerObject(std::string container);
   bool isLocation(std::string location);
   bool isAgent(std::string agent);
   bool AreFactsInDB(std::vector<toaster_msgs::Fact> precs);
   bool updateGTP();
   int  planGTP(std::string actionName, std::vector<gtp_ros_msg::Ag> agents, std::vector<gtp_ros_msg::Obj> objects, std::vector<gtp_ros_msg::Data> datas, std::vector<gtp_ros_msg::Points> points);
   bool execGTPAction(int GTPActionId, bool shouldOpen, std::string object);
   bool executeTrajectory(int GTPActionId, int actionSubId, int armId);
   bool moveGripper(int armId, bool open);
   bool isGripperEmpty(std::string arm);
   void PutInHand(std::string object, std::string hand, int gtpId);
   void RemoveFromHand(std::string object);
   void RemoveFromHumanHand(std::string object);
   void PutInHumanHand(std::string object, std::string agent);
   bool addGTPAttachment(int id);
   void PutOnSupport(std::string object, std::string support);
   void PutInContainer(std::string object, std::string container);
   void PutObjectInFrontRobot(std::string object);
   void addFactsToDB(std::vector<toaster_msgs::Fact> facts);

};

#endif // VIRTUALACTION_H

