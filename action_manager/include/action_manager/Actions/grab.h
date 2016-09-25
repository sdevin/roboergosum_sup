/************
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Class of the grab action (handover from human to robot)
 * **********/

#ifndef GRAB_H
#define GRAB_H

#include "action_manager/virtual_action.h"

/**
 * @brief Class of the grab action
 */
class Grab: public VirtualAction{

public:
    Grab(roboergosum_msgs::Action action, Connector* connector);
    virtual bool preconditions();
    virtual bool plan();
    virtual bool exec();
    virtual bool post();

private:
    std::string object_; /**< object to place*/
    std::string giver_; /**< agent who should receive the object*/
};

#endif // GRAB_H
