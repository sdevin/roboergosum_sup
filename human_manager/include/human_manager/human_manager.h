/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Main class of the node
 * **********/

#ifndef HUMANMNANGER_H
#define HUMANMNANGER_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include "roboergosum_msgs/HumanAction.h"
#include "toaster_msgs/PutInHand.h"
#include "toaster_msgs/RemoveFromHand.h"
#include "toaster_msgs/SetEntityPose.h"
#include "toaster_msgs/ObjectListStamped.h"

/**
 * @brief main class of the node
 */
class HumanManager{
public:
    HumanManager(ros::NodeHandle* node);
    ~HumanManager() {};
    void humanPick(std::string agent, std::string object);
    void humanPlace(std::string agent, std::string object, std::string support);
    void humanDrop(std::string agent, std::string object, std::string container);
    std::pair<bool, std::string> hasInHand(std::string agent);
protected:

private:
    std::vector<std::pair<std::string, std::string> > attachments_; /**< list of the different attachments (agent, object)*/
    ros::NodeHandle* node_; /**< pointer to the node handle*/
    std::string humanHand_; /**< name of the right hand of the human*/
};

#endif // HUMANMNANGER_H

