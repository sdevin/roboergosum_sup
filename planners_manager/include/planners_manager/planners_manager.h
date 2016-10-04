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
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include "toaster_msgs/SetEntityPose.h"
#include "toaster_msgs/ObjectListStamped.h"
#include "toaster_msgs/ExecuteDB.h"

#include "BP_experiment/StateReward.h"


/**
 * @brief main class of the node
 */
class PlannersManager{
public:
    PlannersManager(ros::NodeHandle* node);
    ~PlannersManager() {};
    void setEnvironment();
    long long int computeWS();

    bool needEnvReset_; /**< true if the environment need to be put reset to the initial set-up*/
protected:

private:
    ros::NodeHandle* node_; /**< pointer to the node handle*/
    std::string robotName_; /**< name of the robot*/

    bool AreFactsInDB(std::vector<toaster_msgs::Fact> facts);
    void resetDB();
};

#endif // PLANNERSMANAGER_H

