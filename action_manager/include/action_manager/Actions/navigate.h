/************
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Class of the navigate action
 * **********/

#ifndef NAVIGATE_H
#define NAVIGATE_H

#include "action_manager/virtual_action.h"

#include <move_base_msgs/MoveBaseAction.h>

/**
 * @brief Class of the pick action
 */

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Navigate: public VirtualAction{

public:
    Navigate(roboergosum_msgs::Action action, Connector* connector);
    virtual bool preconditions();
    virtual bool plan();
    virtual bool exec();
    virtual bool post();

private:
    std::string location_; /**< location where to go*/
    MoveBaseClient* client_; /**< client to the move base action server*/
};

#endif // NAVIGATE_H
