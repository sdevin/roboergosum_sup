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
float reward;
std::vector<toaster_msgs::Fact> WSFacts, rewardFacts;
bool humanLazy, humanNoHandover;
std::map<std::string, std::string> objectsColor;
std::vector<std::string> taskObjects, taskContainers;
std::string robotName, humanName;
std::string humanArea, middleArea, robotArea;
std::string expertsMode;
bool firstLoop, actionPossible, shouldExecTraj;
float pred_error;

/**
 * \brief Choose an action to be perform by the human
 * @return true if there is an action, and if true the corresponding action
 * */
std::pair<bool, roboergosum_msgs::Action> getHumanAction(){


    std::pair<bool, roboergosum_msgs::Action> result;
    result.first = false;

    std::vector<toaster_msgs::Fact> WS;
    toaster_msgs::Fact fact;
    fact.subjectId = "BLUE_TAPE1";
    fact.property = "isHoldBy";
    fact.targetId = humanName;
    WS.push_back(fact);
    fact.subjectId = "BLUE_TAPE2";
    fact.property = "isHoldBy";
    fact.targetId = humanName;
    WS.push_back(fact);
    fact.subjectId = "BLUE_TAPE3";
    fact.property = "isHoldBy";
    fact.targetId = humanName;
    WS.push_back(fact);
    fact.subjectId = "GREEN_TAPE1";
    fact.property = "isHoldBy";
    fact.targetId = humanName;
    WS.push_back(fact);
    fact.subjectId = "GREEN_TAPE2";
    fact.property = "isHoldBy";
    fact.targetId = humanName;
    WS.push_back(fact);
    fact.subjectId = "GREEN_TAPE3";
    fact.property = "isHoldBy";
    fact.targetId = humanName;
    WS.push_back(fact);
    fact.subjectId = "BLUE_TAPE1";
    fact.property = "IsInArea";
    fact.targetId = humanArea;
    fact.targetOwnerId = "TABLE_4";
    WS.push_back(fact);
    fact.subjectId = "BLUE_TAPE2";
    fact.property = "IsInArea";
    fact.targetId = humanArea;
    fact.targetOwnerId = "TABLE_4";
    WS.push_back(fact);
    fact.subjectId = "BLUE_TAPE3";
    fact.property = "IsInArea";
    fact.targetId = humanArea;
    fact.targetOwnerId = "TABLE_4";
    WS.push_back(fact);
    fact.subjectId = "GREEN_TAPE1";
    fact.property = "IsInArea";
    fact.targetId = humanArea;
    fact.targetOwnerId = "TABLE_4";
    WS.push_back(fact);
    fact.subjectId = "GREEN_TAPE2";
    fact.property = "IsInArea";
    fact.targetId = humanArea;
    fact.targetOwnerId = "TABLE_4";
    WS.push_back(fact);
    fact.subjectId = "GREEN_TAPE3";
    fact.property = "IsInArea";
    fact.targetId = humanArea;
    fact.targetOwnerId = "TABLE_4";
    WS.push_back(fact);
    fact.subjectId = "BLUE_TAPE1";
    fact.property = "IsInArea";
    fact.targetId = robotArea;
    fact.targetOwnerId = "TABLE_4";
    WS.push_back(fact);
    fact.subjectId = "BLUE_TAPE2";
    fact.property = "IsInArea";
    fact.targetId = robotArea;
    fact.targetOwnerId = "TABLE_4";
    WS.push_back(fact);
    fact.subjectId = "BLUE_TAPE3";
    fact.property = "IsInArea";
    fact.targetId = robotArea;
    fact.targetOwnerId = "TABLE_4";
    WS.push_back(fact);
    fact.subjectId = "GREEN_TAPE1";
    fact.property = "IsInArea";
    fact.targetId = robotArea;
    fact.targetOwnerId = "TABLE_4";
    WS.push_back(fact);
    fact.subjectId = "GREEN_TAPE2";
    fact.property = "IsInArea";
    fact.targetId = robotArea;
    fact.targetOwnerId = "TABLE_4";
    WS.push_back(fact);
    fact.subjectId = "GREEN_TAPE3";
    fact.property = "IsInArea";
    fact.targetId = robotArea;
    fact.targetOwnerId = "TABLE_4";
    WS.push_back(fact);
    fact.targetOwnerId = "";
    fact.subjectId = "BLUE_TAPE1";
    fact.property = "isIn";
    fact.targetId = "BLUE_TRASHBIN";
    WS.push_back(fact);
    fact.subjectId = "BLUE_TAPE2";
    fact.property = "isIn";
    fact.targetId = "BLUE_TRASHBIN";
    WS.push_back(fact);
    fact.subjectId = "BLUE_TAPE3";
    fact.property = "isIn";
    fact.targetId = "BLUE_TRASHBIN";
    WS.push_back(fact);
    fact.subjectId = "GREEN_TAPE1";
    fact.property = "isIn";
    fact.targetId = "GREEN_TRASHBIN";
    WS.push_back(fact);
    fact.subjectId = "GREEN_TAPE2";
    fact.property = "isIn";
    fact.targetId = "GREEN_TRASHBIN";
    WS.push_back(fact);
    fact.subjectId = "GREEN_TAPE3";
    fact.property = "isIn";
    fact.targetId = "GREEN_TRASHBIN";
    WS.push_back(fact);
    fact.subjectId = "BLUE_TAPE1";
    fact.property = "isHoldBy";
    fact.targetId = "PR2_ROBOT";
    WS.push_back(fact);
    fact.subjectId = "BLUE_TAPE2";
    fact.property = "isHoldBy";
    fact.targetId = "PR2_ROBOT";
    WS.push_back(fact);
    fact.subjectId = "BLUE_TAPE3";
    fact.property = "isHoldBy";
    fact.targetId = "PR2_ROBOT";
    WS.push_back(fact);
    fact.subjectId = "GREEN_TAPE1";
    fact.property = "isHoldBy";
    fact.targetId = "PR2_ROBOT";
    WS.push_back(fact);
    fact.subjectId = "GREEN_TAPE2";
    fact.property = "isHoldBy";
    fact.targetId = "PR2_ROBOT";
    WS.push_back(fact);
    fact.subjectId = "GREEN_TAPE3";
    fact.property = "isHoldBy";
    fact.targetId = "PR2_ROBOT";
    WS.push_back(fact);

    std::vector<std::string> WSRes = pm_->AreFactsInDBIndiv(WS);

    //we look the blue objects in the human hand
    //isHoldBy facts for blue
    for(int i = 0; i<3; i++){
        if(WSRes[i] == "true"){
            //the human drop the object in the box
            result.first = true;
            roboergosum_msgs::Action action;
            action.name = "drop";
            action.actors.push_back(humanName);
            if(i == 0){
                action.parameters.push_back("BLUE_TAPE1");
            }else if(i == 1){
                action.parameters.push_back("BLUE_TAPE2");
            }else if(i == 2){
                action.parameters.push_back("BLUE_TAPE3");
            }
            action.parameters.push_back("BLUE_TRASHBIN");
            result.second = action;
            return result;
        }
    }

    //we look the green objects in the human hand
    for(int i = 3; i<6; i++){//isHoldBy facts for green
        if(WSRes[i] == "true"){
            return result;
        }
    }

    //then if the human has empty hands, we look in the object near him
    for(int i = 6; i<9; i++){//IsInArea human facts for blue
        if(WSRes[i] == "true" && WSRes[i+12] == "false" && WSRes[i+6] == "false" && WSRes[i+18] == "false"){//not in trashbin and in robotReachable
            result.first = true;
            roboergosum_msgs::Action action;
            action.name = "pick";
            action.actors.push_back(humanName);
            if(i == 6){
                action.parameters.push_back("BLUE_TAPE1");
            }else if(i == 7){
                action.parameters.push_back("BLUE_TAPE2");
            }else if(i == 8){
                action.parameters.push_back("BLUE_TAPE3");
            }
            result.second = action;
            return result;
        }
    }

    //finally, if the human is proactive we look for blue objects in the middle of the table
    if(!humanLazy){
        for(int i = 6; i<9; i++){//IsInArea middle facts for blue
            if(WSRes[i] == "true" && WSRes[i+18] == "false" && WSRes[i+12] == "false"){//not in trashbin
                result.first = true;
                roboergosum_msgs::Action action;
                action.name = "pick";
                action.actors.push_back(humanName);
                if(i == 6){
                    action.parameters.push_back("BLUE_TAPE1");
                }else if(i == 7){
                    action.parameters.push_back("BLUE_TAPE2");
                }else if(i == 8){
                    action.parameters.push_back("BLUE_TAPE3");
                }
                result.second = action;
                return result;
            }
        }
    }

    //then if the human is not lazy, we look for green objects
    if(!humanNoHandover){
        for(int i = 9; i<12; i++){//IsInArea human facts for green
            if(WSRes[i] == "true" && WSRes[i+6] == "false"&& WSRes[i+12] == "false" && WSRes[i+18] == "false"){//not in trashbin and in robotReachable
                result.first = true;
                roboergosum_msgs::Action action;
                action.name = "pick";
                action.actors.push_back(humanName);
                if(i == 9){
                    action.parameters.push_back("GREEN_TAPE1");
                }else if(i == 10){
                    action.parameters.push_back("GREEN_TAPE2");
                }else if(i == 11){
                    action.parameters.push_back("GREEN_TAPE3");
                }
                result.second = action;
                return result;
            }
        }
    }

    /*
    //we look the object in the human hand
    std::string sqlCommand = "SELECT subject_id from fact_table_" + robotName + " where predicate='isHoldBy' and target_id = '"+ humanName +"'";
    std::vector<std::string> objetsInHand = pm_->executeSQL(sqlCommand);
    for(std::vector<std::string>::iterator it = objetsInHand.begin(); it != objetsInHand.end(); it++){
        if(objectsColor[*it] == "blue"){
            //the human drop the object in the box
            result.first = true;
            roboergosum_msgs::Action action;
            action.name = "drop";
            action.actors.push_back(humanName);
            action.parameters.push_back(*it);
            action.parameters.push_back("BLUE_TRASHBIN");
            result.second = action;
            return result;
        }else{
            //if the human has an object in hand which is not blue, he waits
            return result;
        }
    }

    //then if the human has empty hands, we look in the object near him
    sqlCommand = "SELECT subject_id from fact_table_" + robotName + " where predicate='IsInArea' and target_id='"+ humanArea +"TABLE_4'";
    std::vector<std::string> objetsNearHuman = pm_->executeSQL(sqlCommand);
    ROS_ERROR("sql: %s, ", sqlCommand.c_str());
    //we look first for blue objects
    for(std::vector<std::string>::iterator it = objetsNearHuman.begin(); it != objetsNearHuman.end(); it++){
        ROS_ERROR("object: %s, ", it->c_str());
        if(objectsColor[*it] == "blue" && *it != "BLUE_TRASHBIN"){
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
    //then if the human is not lazy, we look for green objects
    if(!humanNoHandover){
        for(std::vector<std::string>::iterator it = objetsNearHuman.begin(); it != objetsNearHuman.end(); it++){
            if(objectsColor[*it] == "green" && *it != "GREEN_TRASHBIN"){
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

    //finally, if the human is proactive we look for blue objects in the middle of the table
    if(!humanLazy){
        sqlCommand = "SELECT subject_id from fact_table_" + robotName + " where predicate='IsInArea' and target_id='"+ middleArea +"TABLE_4'";
        std::vector<std::string> objetsMiddle = pm_->executeSQL(sqlCommand);
        for(std::vector<std::string>::iterator it = objetsMiddle.begin(); it != objetsMiddle.end(); it++){
            if(objectsColor[*it] == "blue" && *it != "BLUE_TRASHBIN"){
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
    }*/


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

    //if an action succeeds, we remove HATP flags
    if(toBlockHATP){
        pm_->removeHATPFlags();
        toBlockHATP = false;
    }

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
        ROS_WARN("Action not in the plan: %d", action.id);
        executedActions.clear();
        return;
    }

    //if the action has already been executed it is an unexpected action
    if(pm_->isIntInVector(planAction.id, executedActions)){
        ROS_WARN("Action already executed: %d", action.id);
        ROS_WARN("Executed actions:");
        for(int i = 0; i<executedActions.size(); i++){
            ROS_WARN("   %d", executedActions[i]);
        }
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
            ROS_WARN("Causal links not ok: %d", action.id);
            return;
        }
    }

    //if we are here, the action is considered ok so we add it to the executed actions
    executedActions.push_back(planAction.id);
}

/**
 * \brief Return the next HATP action to perform
 * @return next HATP action to perform
 * */
roboergosum_msgs::Action getNextHATP(){

    if(hasHATPPlan){
        for(std::vector<roboergosum_msgs::Action>::iterator it = hatpPlan.actions.begin(); it != hatpPlan.actions.end(); it++){
            if(it->actors[0] == robotName && !pm_->isIntInVector(it->id, executedActions)){
                bool linksOk = true;
                for(std::vector<roboergosum_msgs::Link>::iterator itl = hatpPlan.links.begin(); itl != hatpPlan.links.end(); itl++){
                    if(itl->following == it->id && !pm_->isIntInVector(itl->origin, executedActions)){
                        linksOk = false;
                        break;
                    }
                }
                if(linksOk){
                    return *it;
                }
            }
        }
    }

    //if we are here, we did not find an action so we return wait
    roboergosum_msgs::Action action;
    action.name = "wait";
    return action;
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
        std::string subjectTopic = "worldState/facts/nb" + nbFact + "/subject";
        std::string propertyTopic = "worldState/facts/nb" + nbFact + "/property";
        std::string targetTopic = "worldState/facts/nb" + nbFact + "/target";
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
        std::string subjectTopic = "reward/facts/nb" + nbFact + "/subject";
        std::string propertyTopic = "reward/facts/nb" + nbFact + "/property";
        std::string targetTopic = "reward/facts/nb" + nbFact + "/target";
        node_->getParam(subjectTopic, fact.subjectId);
        node_->getParam(propertyTopic, fact.property);
        node_->getParam(targetTopic, fact.targetId);
        rewardFacts.push_back(fact);
    }
}

/**
 * \brief Say if there is a human action to do
 * @return true if there is a human action to do
 * */
bool isHumanAction(){

    if(hasHATPPlan){
        for(std::vector<roboergosum_msgs::Action>::iterator it = hatpPlan.actions.begin(); it != hatpPlan.actions.end(); it++){
            if(it->actors[0] == humanName && !pm_->isIntInVector(it->id, executedActions)){
                bool linksOk = true;
                for(std::vector<roboergosum_msgs::Link>::iterator itl = hatpPlan.links.begin(); itl != hatpPlan.links.end(); itl++){
                    if(itl->following == it->id && !pm_->isIntInVector(itl->origin, executedActions)){
                        linksOk = false;
                        break;
                    }
                }
                if(linksOk){
                    return true;
                }
            }
        }
    }

    return false;
}

/**
 * \brief Say if the action is possible or not taking the current world state
 * @param action the action to test
 * @return true if the action is possible
 * */
bool getPossibleAction(roboergosum_msgs::Action action){

    std::vector<toaster_msgs::Fact> precs;

    if(action.name == "wait"){
        return isHumanAction();
    }else if(action.name == "pick"){
        toaster_msgs::Fact fact;
        fact.subjectId = action.parameters[0];
        fact.property = "isReachableBy";
        fact.targetId = robotName;
        precs.push_back(fact);
        fact.subjectId = "NULL";
        fact.property = "isHoldBy";
        fact.targetId = robotName;
        precs.push_back(fact);
    }else if(action.name == "place"){
        toaster_msgs::Fact fact;
        fact.subjectId = action.parameters[1];
        fact.property = "isReachableBy";
        fact.targetId = robotName;
        precs.push_back(fact);
        fact.subjectId = action.parameters[0];
        fact.property = "isHoldBy";
        fact.targetId = robotName;
        precs.push_back(fact);
    }else if(action.name == "drop"){
        toaster_msgs::Fact fact;
        fact.subjectId = action.parameters[0];
        fact.property = "isHoldBy";
        fact.targetId = robotName;
        precs.push_back(fact);
    }else if(action.name == "navigate"){
        return true;
    }else if(action.name == "give"){
        toaster_msgs::Fact fact;
        fact.subjectId = action.parameters[0];
        fact.property = "isHoldBy";
        fact.targetId = robotName;
        precs.push_back(fact);
        fact.subjectId = "NULL";
        fact.property = "isHoldBy";
        fact.targetId = humanName;
        precs.push_back(fact);
    }else if(action.name == "grab"){
        toaster_msgs::Fact fact;
        fact.subjectId = action.parameters[0];
        fact.property = "isHoldBy";
        fact.targetId = humanName;
        precs.push_back(fact);
        fact.subjectId = "NULL";
        fact.property = "isHoldBy";
        fact.targetId = robotName;
        precs.push_back(fact);
        fact.subjectId = action.parameters[1];
        fact.property = "isReachableBy";
        fact.targetId = robotName;
        precs.push_back(fact);
    }else{
        ROS_WARN("[planners_manager] Unknown action: %s", action.name.c_str());
        return false;
    }

    //check if the preconditions are in the robot knowledge
    return pm_->AreFactsInDB(precs);
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
  node_->getParam("roboergosum/humanNoHandover", humanNoHandover);
  node_->getParam("roboergosum/robotName", robotName);
  node_->getParam("roboergosum/humanName", humanName);
  node_->getParam("environment/humanArea", humanArea);
  node_->getParam("environment/middleArea", middleArea);
  node_->getParam("environment/robotArea", robotArea);
  node_->getParam("roboergosum/expertsMode", expertsMode);
  node_->getParam("action_manager/shouldExecTraj", shouldExecTraj);

  //set the seed
  int seed;
  node_->getParam("roboergosum/seed", expertsMode);
  srand(seed);

  firstLoop = true;

  PlannersManager pm(&node);
  pm_ = &pm;
  initWSFacts();
  initRewardFacts();
  initObjects();

  ros::Publisher statereward_pub = node.advertise<BP_experiment::StateReward>("/bp_experiment/premetacontroller/statereward", 1);
  ros::Publisher hatp_pub = node.advertise<BP_experiment::Actions>("/bp_experiment/goaldirectedAction", 1);
  ros::Publisher pred_pub = node.advertise<BP_experiment::StatePred>("/bp_experiment/pmc/MB/sp", 1);

  actionlib::SimpleActionClient<roboergosum_msgs::ActionManagerAction> actionClient("roboergosum/action_manager", true);
  actionClient.waitForServer();

  ROS_INFO("[planners_manager] planners_manager ready");

  while (node.ok()) {

      /** *********************ENV GESTION AND WS PUBLISHING*****************************/
     //we reset the environment if needed
     if(pm_->needEnvReset_){
        pm_->setEnvironment();
        hasHATPPlan = false;
        executedActions.clear();
        if(shouldExecTraj){
            ros::Duration(2.0).sleep();
        }
     }

     //we publish the world state and the previous reward
     std::string stateID = pm_->computeWS(WSFacts);
     BP_experiment::StateReward msg_state;
     msg_state.stateID = stateID;
     msg_state.stateType = "Bpe";
     msg_state.reward = reward;
     statereward_pub.publish(msg_state);

     //we publish the prediction error
     BP_experiment::StatePred msg_pred;
     msg_pred.pred_error = pred_error;
     pred_pub.publish(msg_pred);


     /** *********************EXPERT CALL*****************************/
     std::string expertToCall;
     //we ask to the metacontroller which expert to call (if in both mode)
     if(expertsMode == "both"){
        BP_experiment::ExpertsActiv MCanswer;
        MCanswer  = *(ros::topic::waitForMessage<BP_experiment::ExpertsActiv>("bp_experiment/metacontroller/controlsignal",ros::Duration(1)));
        if(MCanswer.modelfree){
            expertToCall = "mf";
        }else{
            expertToCall = "hatp";
        }
     }else{
         expertToCall = expertsMode;
     }

     //we call the expert
     int idAction;
     roboergosum_msgs::Action action;
     if(expertToCall == "mf"){
         BP_experiment::Actions MFanswer;
         MFanswer = *(ros::topic::waitForMessage<BP_experiment::Actions>("/bp_experiment/metacontroller/decidedAction",ros::Duration(1)));
         action = pm_->getActionFromId(MFanswer.actionID);
         idAction = MFanswer.actionID;
         ros::Time now = ros::Time::now();
         ros::Duration t = now - pm_->nodeStartTime_;
         float time = t.toSec();
         pm_->fileLogHATP_ << time << " " << pm_->nbActions_ << " 0.0 " << "MF" <<std::endl;

         //compute the predicion for the action (is it possible or not)
         actionPossible = getPossibleAction(action);
     }else if(expertToCall == "hatp"){
        if(!hasHATPPlan){
            executedActions.clear();
            //first we look for a plan
            std::pair<bool, roboergosum_msgs::Plan> hatpRequest = pm_->GetHATPPlan(toBlockHATP, actionToBlockHATP);
            if(hatpRequest.first){
                hatpPlan = hatpRequest.second;
                hasHATPPlan = true;
            }else{
                ROS_WARN("HATP did not find a plan for the current situation");
            }
        }else{
            ros::Time now = ros::Time::now();
            ros::Duration t = now - pm_->nodeStartTime_;
            float time = t.toSec();
            pm_->fileLogHATP_ << time << " " << pm_->nbActions_ << " 0.0 " << "ALREADY_PLAN" <<std::endl;
        }
        //then we get the corresponding action
        action = getNextHATP();
        //we publish the corresponding id
        idAction = pm_->getIdFromAction(action);
        int nbActions;
        node_->getParam("actions/nbActions", nbActions);
        BP_experiment::Actions HATPAction;
        HATPAction.actionID = idAction;
        HATPAction.source = "MB";
        for(int i = 0; i < nbActions; i++){
            if(i == idAction){
                HATPAction.actionProbDistribution.push_back(1-(nbActions-1)*0.01);
            }else{
                HATPAction.actionProbDistribution.push_back(0.01);
            }
        }
        hatp_pub.publish(HATPAction);
        actionPossible = true;
     }

     /** *********************ACTION EXECUTION*****************************/
     bool humanActionNeeded = false;
     std::string state;
     //if the return action is a wait action we wait 2 seconds
     if(action.name == "wait"){
         ros::Duration(2.0).sleep();
         humanActionNeeded = true;
         state = "SUCCEED";
     }else{
         //else we execute the returned action and update HATP plan if one
         roboergosum_msgs::ActionManagerGoal goal;
         goal.action = action;
         actionClient.sendGoal(goal);
         bool finishedBeforeTimeout = actionClient.waitForResult(ros::Duration(300.0));
         if(!finishedBeforeTimeout){
             ROS_ERROR("roboergosum/action_manager action did not finish before time out");
         }else if(actionClient.getResult()->report){
             state = "SUCCEED";
             if(!(action.name == "give" || action.name == "grab")){
                 //the human acts only if the robot performed an action (including wait) which was not an handover
                 humanActionNeeded = true;
             }
             //update HATP plan
             if(hasHATPPlan){
                updateHATPPlan(action);
             }
             if(action.name == "pick" || action.name == "grab"){
                 pm_->objectInHand_ = action.parameters[0];
                 if(action.name == "grab"){
                     pm_->objectInHumanHand_ = "NONE";
                 }
             }else if(action.name == "give" || action.name == "place" || action.name == "drop"){
                 pm_->objectInHand_ = "NONE";
                 if(action.name == "grab"){
                     pm_->objectInHumanHand_ = action.parameters[0];
                 }
             }else if(action.name == "navigate"){
                 pm_->robotPose_ = action.parameters[0];
             }
             if(actionPossible){
                 pred_error = 0.0;
             }else{
                 pred_error = 1.0;
             }
         }else{
             if(actionClient.getResult()->state == "PREC"){
                 state = "NO_EXEC";
             }else{
                 state = "FAILED";
             }
             //the action failed
             if(expertToCall == "hatp"){
                 //if it was an action from hatp, a new plan is needed
                 toBlockHATP = true;
                 actionToBlockHATP = action;
                 hasHATPPlan = false;
             }
             if(actionPossible){
                 pred_error = 1.0;
             }else{
                 pred_error = 0.0;
             }
         }
     }
     pm_->nbActions_ ++;

     //we log action result
     ros::Time now = ros::Time::now();
     ros::Duration t = now - pm_->nodeStartTime_;
     float time = t.toSec();
     pm_->fileLogRobotActions_ << time  << " " << pm_->nbActions_ << " " << expertToCall << " " << idAction << " " << state << std::endl;


     /** *********************HUMAN ACTION EXECUTION*****************************/
     //we execute human action if needed and update HATP plan if one
     if(humanActionNeeded){
        std::pair<bool, roboergosum_msgs::Action> humanAction = getHumanAction();
        if(humanAction.first){
            ros::ServiceClient client = node_->serviceClient<roboergosum_msgs::HumanAction>("human_manager/human_action");
            roboergosum_msgs::HumanAction srv;
            srv.request.actionName = humanAction.second.name;
            srv.request.agent = humanName;
            if(humanAction.second.parameters.size()>0){
                srv.request.object = humanAction.second.parameters[0];
            }
            if(humanAction.second.parameters.size()>1){
                if(humanAction.second.name == "drop"){
                    srv.request.container = humanAction.second.parameters[1];
                }else if(humanAction.second.name == "place"){
                    srv.request.support = humanAction.second.parameters[1];
                }
            }
            //Sending the action
            std::string humanState;
            if (client.call(srv)) {
                if(humanAction.second.name == "pick"){
                    pm_->objectInHumanHand_ = humanAction.second.parameters[0];
                }else if(humanAction.second.name == "place" || humanAction.second.name == "drop"){
                    pm_->objectInHumanHand_ = "NONE";
                }
                //update HATP plan
                if(hasHATPPlan){
                   updateHATPPlan(humanAction.second);
                   if(hasHATPPlan){
                       pred_error = 0.0;
                       humanState = "EXPECTED";
                   }else{
                       pred_error = 1.0;
                       humanState = "UNEXPECTED";
                   }
                }else{
                   humanState = "NO_PLAN";
                }
            }else{
                ROS_ERROR("Failed to call service human_manager/human_action");
            }

            //we log action result
            now = ros::Time::now();
            t = now - pm_->nodeStartTime_;
            time = t.toSec();
            pm_->fileLogHumanActions_ << time  << " " << pm_->nbActions_ << " " << humanState << std::endl;
        }else{
            //if hatp was waiting for an action and the human does not perform it we need a new plan
            if(action.name == "wait" && expertToCall == "hatp"){
                pred_error = 1.0;
                toBlockHATP = true;
                roboergosum_msgs::Action actionToBlock;
                actionToBlock.name = "humanAction";
                actionToBlockHATP = actionToBlock;
                hasHATPPlan = false;
            }else if(action.name == "wait"){
                if(actionPossible){
                    pred_error = 1.0;
                }else{
                    pred_error = 0.0;
                }
            }
        }
     }

     /** *********************REWARD*****************************/

     //we compute the reward
     //the robot needs to perform a wait action in the final state to get a reward
     if(pm_->AreFactsInDB(rewardFacts) && action.name == "wait"){
         reward = 1.0;
         pm_->needEnvReset_ = true;
     }else{
         reward = 0.0;
     }


     firstLoop = false;

     ros::spinOnce();
     loop_rate.sleep();
    }

    return 0;
}
