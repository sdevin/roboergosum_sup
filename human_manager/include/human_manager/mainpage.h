/** @file mainpage.h
* @brief Main page of the action manager doxygen documentation
*
*/
/** \mainpage Human Manager
*
* @authors Sandra Devin (sdevin@laas.fr)
*
* Status: maintained, documented
*
* This node allows to execute simulated actions of the human
*
* The actions supported by this node are:
* - Pick
* - Place
* - Drop
* - Give (handover human to robot)
* - Grab (handover robot to human)
*
* This node is composed of the following classes:
* - HumanManager: main class of the node

*
* \section Services
*- /human_manager/human_action (string actionName, string agent, string object, string support, string container)
*
* Simulate an action from a human
*
* \section Parameters
*
* \section Dependencies
* Build dependecies:
* - toaster_msgs
* - roboergosum_msgs
*
* Run dependencies
* - Toaster
*/
