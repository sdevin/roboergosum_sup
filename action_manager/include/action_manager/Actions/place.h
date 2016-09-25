/************
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Class of the place action
 * **********/

#ifndef PLACE_H
#define PLACE_H

#include "action_manager/virtual_action.h"

/**
 * @brief The Place class
 */
class Place: public VirtualAction{

public:
    Place(roboergosum_msgs::Action action, Connector* connector);
    virtual bool preconditions();
    virtual bool plan();
    virtual bool exec();
    virtual bool post();

private:
    std::string object_; /**< object to place*/
    std::string support_; /**< support where to place*/
};

#endif // PLACE_H
