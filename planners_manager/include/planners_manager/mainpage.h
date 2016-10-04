/** @file mainpage.h
* @brief Main page of the action manager doxygen documentation
*
*/
/** \mainpage Planners Manager
*
* @authors Sandra Devin (sdevin@laas.fr)
*
* Status: maintained, documented
*
* This node allows to execute the main loop of the roboergosum project. It:
* - set the environment if needed
* - compute the ws to send to planners
* - call the meta controller to know which planner call
* - call the planner
* - execute the robot action
* - simulate a human action if needed
* - update HATP plan if needed
* - compute the reward and send it
*
* This node is composed of the following classes:
* - PlannersManager: main class of the node

*
* \section Services
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
* - Action Manager
* - Human Manager
*/
