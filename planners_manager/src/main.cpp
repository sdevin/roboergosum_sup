/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * **********/

#include <planners_manager/planners_manager.h>

ros::NodeHandle* node_;
PlannersManager* pm_;
roboergosum_msgs::Plan hatpPlan;
std::vector<int> executedActions;
bool hasHATPPlan = false;
bool toBlockHATP = false;
roboergosum_msgs::Action actionToBlockHATP;
float reward = 0.0;
std::vector<toaster_msgs::Fact> WSFacts, rewardFacts;
bool humanLazy, humanProactive;
std::map<std::string, std::string> objectsColor;
std::vector<std::string> taskObjects, taskContainers;
std::string robotName, humanName;
std::string humanArea, middleArea, robotArea;
std::string expertsMode;

/**
 * \brief Choose an action to be perform by the human
 * @return true if there is an action, and if true the corresponding action
 * */
std::pair<bool, roboergosum_msgs::Action> getHumanAction(){

    std::pair<bool, roboergosum_msgs::Action> result;
    result.first = false;

    //we look the object in the human hand
    std::string sqlCommand = "SELECT subject_id from fact_table_" + robotName + " where predicate = 'isHoldBy' and target_id = '"+ humanName +"'";
    std::vector<std::string> objetsInHand = pm_->executeSQL(sqlCommand);
    for(std::vector<std::string>::iterator it = objetsInHand.begin(); it != objetsInHand.end(); it++){
        if(objectsColor[*it] == "green"){
            //the human drop the object in the box
            result.first = true;
            roboergosum_msgs::Action action;
            action.name = "drop";
            action.actors.push_back(humanName);
            action.parameters.push_back(*it);
            action.parameters.push_back("GREEN_TRASHBIN");
            result.second = action;
            return result;
        }else{
            //if the human has an object in hand which is not green, he waits
            return result;
        }
    }

    //then if the human has empty hands, we look in the object near him
    sqlCommand = "SELECT subject_id from fact_table_" + robotName + " where predicate = 'isInArea' and target_id = '"+ humanArea +"'";
    std::vector<std::string> objetsNearHuman = pm_->executeSQL(sqlCommand);
    //we look first for green objects
    for(std::vector<std::string>::iterator it = objetsNearHuman.begin(); it != objetsNearHuman.end(); it++){
        if(objectsColor[*it] == "green"){
            //the human pick the object
            result.first = true;
            roboergosum_msgs::Action action;
            action.name = "pick";
            action.actors.push_back(humanName);
            action.parameters.push_back(*it);
            result.second = action;
            return result;
        }
    }
    //then if the human is not lazy, we look for blue objects
    if(!humanLazy){
        for(std::vector<std::string>::iterator it = objetsNearHuman.begin(); it != objetsNearHuman.end(); it++){
            if(objectsColor[*it] == "blue"){
                //the human pick the object
                result.first = true;
                roboergosum_msgs::Action action;
                action.name = "pick";
                action.actors.push_back(humanName);
                action.parameters.push_back(*it);
                result.second = action;
                return result;
            }
        }
    }

    //finally, if the human is proactive we look for green objects in the middle of the table
    if(!humanProactive){
        sqlCommand = "SELECT subject_id from fact_table_" + robotName + " where predicate = 'isInArea' and target_id = '"+ middleArea +"'";
        std::vector<std::string> objetsMiddle = pm_->executeSQL(sqlCommand);
        for(std::vector<std::string>::iterator it = objetsMiddle.begin(); it != objetsMiddle.end(); it++){
            if(objectsColor[*it] == "green"){
                //the human pick the object
                result.first = true;
                roboergosum_msgs::Action action;
                action.name = "pick";
                action.actors.push_back(humanName);
                action.parameters.push_back(*it);
                result.second = action;
                return result;
            }
        }
    }


    return result;
}

/**
 * \brief Initialize  information on objects
 * */
void initObjects(){

    node_->getParam("entities/objects", taskObjects);
    node_->getParam("entities/containers", taskContainers);
    for(std::vector<std::string>::iterator it = taskObjects.begin(); it != taskObjects.end(); it++){
        std::string colorTopic = "entities/objectsColor/" + *it;
        node_->getParam(colorTopic, objectsColor[*it]);
    }
    for(std::vector<std::string>::iterator it = taskContainers.begin(); it != taskContainers.end(); it++){
        std::string colorTopic = "entities/objectsColor/" + *it;
        node_->getParam(colorTopic, objectsColor[*it]);
    }
}

/**
 * \brief Update the HATP plan according to an executed action
 * @param action the executed action
 * */
void updateHATPPlan(roboergosum_msgs::Action action){

    //we look if the action is part of the plan
    bool find = false;
    roboergosum_msgs::Action planAction;
    for(std::vector<roboergosum_msgs::Action>::iterator it = hatpPlan.actions.begin(); it != hatpPlan.actions.end(); it++){
        if(pm_->areActionsIdentical(action, *it)){
            find = true;
            planAction = *it;
            break;
        }
    }
    if(!find){
        //action not in the plan, we abort the plan
        toBlockHATP = false;
        hasHATPPlan = false;
        executedActions.clear();
        return;
    }

    //if the action has already been executed it is an unexpected action
    if(pm_->isIntInVector(planAction.id, executedActions)){
        toBlockHATP = false;
        hasHATPPlan = false;
        executedActions.clear();
        return;
    }

    //we check if the causal links of the action are ok
    for(std::vector<roboergosum_msgs::Link>::iterator it = hatpPlan.links.begin(); it != hatpPlan.links.end(); it++){
        if(it->following == planAction.id && !pm_->isIntInVector(it->origin, executedActions)){
            //a causal link is not checked, we abort the goal
            toBlockHATP = false;
            hasHATPPlan = false;
            executedActions.clear();
            return;
        }
    }

    //if we are here, the actionis considered ok so we add it to theexecuted actions
    executedActions.push_back(planAction.id);
}

/**
 * \brief Initialize the set of facts representing the world state
 * */
void initWSFacts(){

    int nbFacts;
    node_->getParam("worldState/nbFacts", nbFacts);
    long long int result = 0;
    for(int i = 0; i < nbFacts; i++){
        std::stringstream ss;
        ss << i;
        std::string nbFact = ss.str();
        //we retrive the fact
        toaster_msgs::Fact fact;
        std::string subjectTopic = "worldState/facts/" + nbFact + "subject";
        std::string propertyTopic = "worldState/facts/" + nbFact + "property";
        std::string targetTopic = "worldState/facts/" + nbFact + "target";
        node_->getParam(subjectTopic, fact.subjectId);
        node_->getParam(propertyTopic, fact.property);
        node_->getParam(targetTopic, fact.targetId);
        WSFacts.push_back(fact);
    }
}

/**
 * \brief Initialize the set of facts which need to be true for the robot to get a reward
 * */
void initRewardFacts(){

    int nbFacts;
    node_->getParam("reward/nbFacts", nbFacts);
    long long int result = 0;
    for(int i = 0; i < nbFacts; i++){
        std::stringstream ss;
        ss << i;
        std::string nbFact = ss.str();
        //we retrive the fact
        toaster_msgs::Fact fact;
        std::string subjectTopic = "reward/facts/" + nbFact + "subject";
        std::string propertyTopic = "reward/facts/" + nbFact + "property";
        std::string targetTopic = "reward/facts/" + nbFact + "target";
        node_->getParam(subjectTopic, fact.subjectId);
        node_->getParam(propertyTopic, fact.property);
        node_->getParam(targetTopic, fact.targetId);
        rewardFacts.push_back(fact);
    }
}


/**
 * \brief Main function
 * Executes the main loop of the roboergosum project
 * */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "planners_manager");
  ros::NodeHandle node;
  ros::Rate loop_rate(30);
  node_ = &node;

  ROS_INFO("[planners_manager] Init planners_manager");

  node_->getParam("roboergosum/humanLazy", humanLazy);
  node_->getParam("roboergosum/humanProactive", humanProactive);
  node_->getParam("roboergosum/robotName", robotName);
  node_->getParam("roboergosum/humanName", humanName);
  node_->getParam("environment/humanArea", humanArea);
  node_->getParam("environment/middleArea", middleArea);
  node_->getParam("environment/robotArea", robotArea);
  node_->getParam("roboergosum/expertsMode", expertsMode);

  PlannersManager pm(&node);
  pm_ = &pm;
  initWSFacts();
  initRewardFacts();
  initObjects();

  ros::Publisher statereward_pub = node.advertise<BP_experiment::StateReward>("/bp experiment/statereward", 1);

  actionlib::SimpleActionClient<roboergosum_msgs::ActionManagerAction> actionClient("roboergosum/action_manager", true);

  ROS_INFO("[planners_manager] planners_manager ready");

  while (node.ok()) {
     //we reset the environment if needed
     if(pm_->needEnvReset_){
        pm_->setEnvironment();
     }

     //we publish the world state and the previous reward
     long long int nbWS = pm_->computeWS(WSFacts);
     std::stringstream ss;
     ss << nbWS;
     std::string stateID = ss.str();
     BP_experiment::StateReward msg_state;
     msg_state.stateID = stateID;
     msg_state.stateType = "Bpe";
     msg_state.reward = reward;
     statereward_pub.publish(msg_state);

     std::string extertToCall;
     //we ask to the metacontroller which expert to call (if in both mode)
     if(expertsMode == "both"){
        //TODO
     }else{
         extertToCall = expertsMode;
     }

     //we call the expert
     roboergosum_msgs::Action action;
     if(extertToCall == "mf"){
        //TODO
     }else if(extertToCall == "hatp"){
        //TODO
     }

     bool humanActionNeeded = false;
     //if the return action is a wait action we wait 2 seconds
     if(action.name == "wait"){
         ros::Duration(2.0).sleep();
         humanActionNeeded = true;
     }else{
         //else we execute the returned action and update HATP plan if one
         roboergosum_msgs::ActionManagerGoal goal;
         goal.action = action;
         actionClient.sendGoal(goal);
         bool finishedBeforeTimeout = actionClient.waitForResult(ros::Duration(300.0));
         if(!finishedBeforeTimeout){
             ROS_ERROR("roboergosum/action_manager action did not finish before time out");
         }else if(actionClient.getResult()->report){
             if(!(action.name == "give" || action.name == "grab")){
                 //the human acts only if the robot performed an action (including wait) which was not an handover
                 humanActionNeeded = true;
             }
             //update HATP plan
             if(hasHATPPlan){
                updateHATPPlan(action);
             }
         }else{
             //the action failed
             if(extertToCall == "hatp"){
                 //if it was an action from hatp, a new plan is needed
                 toBlockHATP = true;
                 actionToBlockHATP = action;
                 hasHATPPlan = false;
             }
         }
     }

     //we execute human action if needed and update HATP plan if one
     if(humanActionNeeded){
        std::pair<bool, roboergosum_msgs::Action> humanAction = getHumanAction();
        if(humanAction.first){
            ros::ServiceClient client = node_->serviceClient<roboergosum_msgs::HumanAction>("human_manager/human_action");
            roboergosum_msgs::HumanAction srv;
            srv.request.actionName = humanAction.second.name;
            srv.request.agent = humanName;
            if(action.parameters.size()>0){
                srv.request.object = humanAction.second.parameters[0];
            }
            if(action.parameters.size()>1){
                srv.request.container = humanAction.second.parameters[1];
            }
            //Sending the action
            if (!client.call(srv)) {
               ROS_ERROR("Failed to call service human_manager/human_action");
            }
        }else{
            //if hatp was waiting for an action and the human does not perform it we need a new plan
            if(action.name == "wait" && extertToCall == "hatp"){
                toBlockHATP = true;
                roboergosum_msgs::Action actionToBlock;
                actionToBlock.name = "humanAction";
                actionToBlockHATP = actionToBlock;
                hasHATPPlan = false;
            }
        }
     }

     //we compute the reward
     if(pm_->AreFactsInDB(rewardFacts)){
         reward = 1.0;
         pm_->needEnvReset_ = true;
     }else{
         reward = 0.0;
     }

     ros::spinOnce();
     loop_rate.sleep();
  }

  return 0;
}
