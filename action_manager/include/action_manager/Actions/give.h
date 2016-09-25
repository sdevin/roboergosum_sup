/************
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Class of the give action (handover from robot to human)
 * **********/

#ifndef GIVE_H
#define GIVE_H

#include "action_manager/virtual_action.h"

/**
 * @brief The Give class
 */
class Give: public VirtualAction{

public:
    Give(roboergosum_msgs::Action action, Connector* connector);
    virtual bool preconditions();
    virtual bool plan();
    virtual bool exec();
    virtual bool post();

private:
    std::string object_; /**< object to place*/
    std::string receiver_; /**< agent who should receive the object*/
    std::string confNameRight_; /**< gtp right arm configuration for the handover*/
    std::string confNameLeft_; /**< gtp left arm configuration for the handover*/
};

#endif // GIVE_H
