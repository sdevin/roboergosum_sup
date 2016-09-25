/************
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Main class of the action manager
 * **********/

#include <action_manager/action_manager.h>

ros::NodeHandle* node_;
Connector* connector_;

/**
 * \brief Constructor of the class
 * @param name the name of the action server name
 * */
ActionManager::ActionManager(std::string name):
action_server_(node_, name, 
    boost::bind(&ActionManager::execute,this, _1), false)
 {
    action_server_.start();
    connector_->action_server_ = &action_server_;
    ROS_INFO("[action_manager] Action server ready");
}


/**
 * \brief Action server execute function
 *
 * Create a action based on the name of action to execute.
 * First checks the preconditions, then plans, then executes and finally applies post-conditions.
 * @param goal the goal is an action to execute
 * */
void ActionManager::execute(const roboergosum_msgs::ActionManagerGoalConstPtr& goal) {

    //Getting action informations
    std::string name = goal->action.name;
    int id = goal->action.id;
    ROS_INFO("[action_manager] Executing action %s with id %i with parameters:", name.c_str(), id);
    for(int i=0; i<goal->action.parameters.size();i++){
        ROS_INFO("  %s", goal->action.parameters[i].c_str());
    }

    //Creating the action
    VirtualAction* act;
    act = initializeAction(goal->action);
    if(!act){
        result_.report = false;
        result_.state = "NON_VALID";
        action_server_.setAborted(result_);
        ROS_INFO("[action_manager] Action failed at creation");
        return;
    }

    //Checking preconditions
    feedback_.state = "PREC";
    action_server_.publishFeedback(feedback_);
    if(!act->preconditions()){
        result_.report = false;
        if(!action_server_.isPreemptRequested()){
            result_.state = feedback_.state;
            action_server_.setAborted(result_);
        }else{
            result_.state = "PREEMPTED";
            action_server_.setPreempted(result_);
        }
        ROS_INFO("[action_manager] Action failed in preconditions");
        return;
    }

    if(action_server_.isPreemptRequested()){
        result_.state = "PREEMPTED";
        result_.report = false;
        action_server_.setPreempted(result_);
        ROS_INFO("[action_manager] Action stoped");
        return;
    }

    //Plan for the action
    feedback_.state = "PLAN";
    action_server_.publishFeedback(feedback_);
    if(!act->plan()){
        result_.report = false;
        if(!action_server_.isPreemptRequested()){
            result_.state = feedback_.state;
            action_server_.setAborted(result_);
        }else{
            result_.state = "PREEMPTED";
            action_server_.setPreempted(result_);

        }
        ROS_INFO("[action_manager] Action failed in planning");
        return;
    }

    if(action_server_.isPreemptRequested()){
        result_.state = "PREEMPTED";
        result_.report = false;
        action_server_.setPreempted(result_);
        ROS_INFO("[action_manager] Action stoped");
        return;
    }

    //Execution of the action
    feedback_.state = "EXEC";
    action_server_.publishFeedback(feedback_);
    if(!act->exec()){
        result_.report = false;
        if(!action_server_.isPreemptRequested()){
            result_.state = feedback_.state;
            action_server_.setAborted(result_);
        }else{
            result_.state = "PREEMPTED";
            action_server_.setPreempted(result_);

        }
        ROS_INFO("[action_manager] Action failed in execution");
        return;
    }

    //Apply/Check Post-conditions
    feedback_.state = "POST";
    action_server_.publishFeedback(feedback_);
    if(!act->post()){
        result_.report = false;
        if(!action_server_.isPreemptRequested()){
            result_.state = feedback_.state;
            action_server_.setAborted(result_);
        }else{
            result_.state = "PREEMPTED";
            action_server_.setPreempted(result_);

        }
        ROS_INFO("[action_manager] Action failed in post conditions");
        return;
    }

    result_.report = true;
    result_.state = "OK";
    action_server_.setSucceeded(result_);
    ROS_INFO("[action_manager] Action succeed");
}


/**
 * \brief Allow to create a virtual action based on its name
 * @param action the action to execute
 * \return the virtual action initialized
 * */
VirtualAction* ActionManager::initializeAction(roboergosum_msgs::Action action) {
    VirtualAction* act = NULL;

    if(action.name == "pick"){
        act = new Pick(action, connector_);
    }else if(action.name == "place"){
        act = new Place(action, connector_);
    }else if(action.name == "drop"){
        act = new Drop(action, connector_);
    }else if(action.name == "give"){
        act = new Give(action, connector_);
    }else{
        ROS_WARN("[action_manager] Unknown action");
    }

    return act;
}

/**
 * \brief Listen to the topic to reset the GTP id
 * @param msg bool msg, true to reset the id
 * */
void resetGTPIdCallback(const std_msgs::Bool::ConstPtr& msg){

    if(msg->data){
        connector_->previousGTPId_ = -1;
    }
}


/**
 * \brief Main function
 *
 * Create the node and the action server
 * */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "action_manager");
  ros::NodeHandle node;
  node_ = &node;

  ROS_INFO("[action_manager] Init action_manager");

  Connector connector;
  connector_ = &connector;

  ros::Subscriber sub = node.subscribe("action_manager/resetGTPId", 1, resetGTPIdCallback);

  ActionManager executor("roboergosum/action_manager");

  ros::spin();

  return 0;
}
