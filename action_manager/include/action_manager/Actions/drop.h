/************
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Class of the drop action
 * **********/

#ifndef DROP_H
#define DROP_H

#include "action_manager/virtual_action.h"

/**
 * @brief The Drop class
 */
class Drop: public VirtualAction{

public:
    Drop(roboergosum_msgs::Action action, Connector* connector);
    virtual bool preconditions();
    virtual bool plan();
    virtual bool exec();
    virtual bool post();

private:
    std::string object_; /**< object to place*/
    std::string container_; /**< container where to drop*/
};

#endif // DROP_H
