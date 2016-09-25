/************
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Class of the pick action
 * **********/

#ifndef PICK_H
#define PICK_H

#include "action_manager/virtual_action.h"

/**
 * @brief Class of the pick action
 */
class Pick: public VirtualAction{

public:
    Pick(roboergosum_msgs::Action action, Connector* connector);
    virtual bool preconditions();
    virtual bool plan();
    virtual bool exec();
    virtual bool post();

private:
    std::string object_; /**< object to pick*/
};

#endif // PICK_H
