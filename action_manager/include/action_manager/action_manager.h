/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Main class of the node
 * **********/

#ifndef ACTIONMANAGER_H
#define ACTIONMANAGER_H

#include "roboergosum_msgs/ActionManagerAction.h"
#include "action_manager/virtual_action.h"
#include "action_manager/connector.h"
#include "action_manager/Actions/pick.h"
#include "action_manager/Actions/place.h"
#include "action_manager/Actions/drop.h"
#include "action_manager/Actions/give.h"
#include "action_manager/Actions/grab.h"
#include "action_manager/Actions/navigate.h"
#include <toaster_msgs/Fact.h>
#include <toaster_msgs/FactList.h>
#include <std_msgs/Bool.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/thread.hpp>


typedef actionlib::SimpleActionServer<roboergosum_msgs::ActionManagerAction> Server;

/**
 * @brief main class of the node, contains the action server.
 */
class ActionManager{
public:
    ActionManager(std::string name);

    std::string robotName_; /**< the name of the robot*/
    bool simu_; /**< flag which is true of working in simulation*/
protected:
    roboergosum_msgs::ActionManagerFeedback feedback_; /**< feedback of the action serveur*/
    roboergosum_msgs::ActionManagerResult result_; /**< result of the action serveur*/
    Server action_server_; /**< the action serveur*/

private:
    void execute(const roboergosum_msgs::ActionManagerGoalConstPtr& goal);
    VirtualAction* initializeAction(roboergosum_msgs::Action action);
};

#endif // ACTIONMANAGER_H

