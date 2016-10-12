/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Main class of the node
 * **********/

#ifndef PLANNERSMANAGER_H
#define PLANNERSMANAGER_H

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <fstream>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


#include "toaster_msgs/SetEntityPose.h"
#include "toaster_msgs/ObjectListStamped.h"
#include "toaster_msgs/ExecuteDB.h"
#include "toaster_msgs/SetInfoDB.h"
#include "roboergosum_msgs/Action.h"
#include "roboergosum_msgs/Plan.h"
#include "roboergosum_msgs/ActionManagerAction.h"
#include "roboergosum_msgs/HumanAction.h"
#include "hatp_msgs/PlanningRequest.h"


#include "BP_experiment/StateReward.h"
#include "BP_experiment/ExpertsActiv.h"
#include "BP_experiment/ExpertPlanning.h"


/**
 * @brief main class of the node
 */
class PlannersManager{
public:
    PlannersManager(ros::NodeHandle* node);
    ~PlannersManager();
    void setEnvironment();
    std::string computeWS(std::vector<toaster_msgs::Fact> WSFacts);
    bool areActionsIdentical(roboergosum_msgs::Action action1, roboergosum_msgs::Action action2);
    bool isIntInVector(int i, std::vector<int> vector);
    std::vector<std::string> executeSQL(std::string sql);
    bool AreFactsInDB(std::vector<toaster_msgs::Fact> facts);
    std::pair<bool, roboergosum_msgs::Plan> GetHATPPlan(bool toBlock, roboergosum_msgs::Action actionToBlock);
    void removeHATPFlags();
    roboergosum_msgs::Action getActionFromId(int id);

    bool needEnvReset_; /**< true if the environment need to be put reset to the initial set-up*/
    std::ofstream fileLog_; /**< the file whre to log info*/
    std::string objectInHand_; /**< the object the robot has in hand*/
    std::string objectInHumanHand_; /**< the object the human has in hand*/
    std::string robotPose_; /**< position of the robot*/
protected:

private:
    ros::NodeHandle* node_; /**< pointer to the node handle*/
    std::string robotName_; /**< name of the robot*/

    void resetDB();
    roboergosum_msgs::Plan convertPlan(hatp_msgs::Plan plan);
    void addHATPFlags(roboergosum_msgs::Action actionToBlock);
};

#endif // PLANNERSMANAGER_H

