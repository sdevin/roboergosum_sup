/** @file mainpage.h
* @brief Main page of the action manager doxygen documentation
*
*/
/** \mainpage Action Manager
*
* @authors Sandra Devin (sdevin@laas.fr)
*
* Status: maintained, documented
*
*This node allow to execute robot actions in the context of the roboergosum projet.
*
* The actions supported by this node are:
* - Pick
* - Place
* - Drop
* - Give (handover robot to human)
* - Grab (handover human to robot)
*
* This node is composed of the following classes:
* - ActionManager: main class of the node, contains the action server
* - Connector: class which contains all the connection of the node with other modules and keep information from one action to another
* - VirtualAction: virtual class from which each action class inherits
* - One class by action the node can execute (pick, place,...)
*
* \section action_API Action API
* The node contains an action server /roboergosum/action_manager.
*
* The goal of this action server is an action (roboergosum_msgs/Action)
*
* \section subscribed_topics Sucribed topics
* - /action_manager/resetGTPId (std_msgs/Bool)
*
* When a true bool is published in this topic, the gtp previous id is reset to -1
*
* \section Parameters
*
* In addition to general parameters of the roboergosum supervisor, the node needs these parameters:
* - shouldUseRightHand (bool, default true): true if the robot should use only the right hand
* - nbPlanMaxGTP (int, default 3): nb of time the robot will ask gtp a plan if failure
* - gripperJoint/right (string, default pr2r_gripper_joint): name of the joint of the right gripper,
* - gripperJoint/left (string, default pr2l_gripper_joint): name of the joint of the left gripper
* - gripperThreshold (double, default 0.01) minimum value the gripper joint should have not to be considered completly closed
* - handoverConfigurationRight (string, default RightTestNo): name of the gtp configuration for a handover with the right arm
* - handoverConfigurationLeft (string, default RightTestNo): name of the gtp configuration for a handover with the left arm
* - putInFrontObject (string, default TABLE_4): name of the object above which we put an object to be in front of the robot
* - putInFrontHeigth (double, default 1.5): height we put an object to be in front of the robot
*
* \section Dependencies
* Build dependecies:
* - toaster_msgs
* - gtp_ros_msgs
* - pr2motion
* - roboergosum_msgs
*
* Run dependencies
* - Toaster
* - GTP
* - PR2motion
*/
