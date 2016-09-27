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
* \section provided_topics Provided topics
*- /human_manager//human_action (std_msgs/Bool)
*
* When a true bool is published in this topic, the gtp previous id is reset to -1
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
