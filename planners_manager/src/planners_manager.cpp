/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * Main class of the node
 * **********/

#include <planners_manager/planners_manager.h>

/**
 * \brief Construction of the class
 * @param node pointer to the node handle
 * */
PlannersManager::PlannersManager(ros::NodeHandle* node){
    node_ = node;
    node_->getParam("/roboergosum/robotName", robotName_);
    needEnvReset_ = true;
    objectInHand_ = "NONE";
    objectInHumanHand_ = "NONE";
    robotPose_ = "initialLocation";

    std::string idExp;
    node_->getParam("/roboergosum/idExp", idExp);
    std::string filePath = "logs/Sup/exp_" + idExp;
    fileLog_.open(filePath.c_str(), std::ios::out|std::ios::trunc);
}

/**
 * \brief Destructor of the class
 * */
PlannersManager::~PlannersManager(){

    fileLog_.close();
}

/**
 * \brief Set the environment in toaster at the initial point (based on params)
 * */
void PlannersManager::setEnvironment(){

    //we reset the database to not have old facts in it
    resetDB();
    objectInHand_ = "NONE";
    objectInHumanHand_ = "NONE";
    robotPose_ = "initialLocation";

    ros::ServiceClient client = node_->serviceClient<toaster_msgs::SetEntityPose>("toaster_simu/set_entity_pose");

    toaster_msgs::ObjectListStamped objectList;
    toaster_msgs::SetEntityPose srv;
    try{
        //get list of entities to place
        std::vector<std::string> listObjects;
        node_->getParam("/environment/objectsToPlace", listObjects);
        for(std::vector<std::string>::iterator it = listObjects.begin(); it != listObjects.end(); it++){
               std::string posex = "/environment/objectsPose/" + *it + "/x" ;
               std::string posey = "/environment/objectsPose/" + *it + "/y" ;
               std::string posez = "/environment/objectsPose/" + *it + "/z" ;
               std::string rotationTopic = "/environment/objectsPose/" + *it + "/rotation" ;
               double x, y, z;
               bool rotation;
               node_->getParam(posex, x);
               node_->getParam(posey, y);
               node_->getParam(posez, z);
               node_->getParam(rotationTopic, rotation);
               srv.request.id = *it;
               srv.request.type = "object";
               srv.request.pose.position.x = x;
               srv.request.pose.position.y = y;
               srv.request.pose.position.z = z;
               srv.request.pose.orientation.x = 0.0;
               srv.request.pose.orientation.y = 0.0;
               if(rotation){
                   srv.request.pose.orientation.z = 0.7;
                   srv.request.pose.orientation.w = 0.7;
               }else{
                   srv.request.pose.orientation.z = 0.0;
                   srv.request.pose.orientation.w = 1.0;
               }
               if (!client.call(srv)){
                 ROS_ERROR("Failed to call service toaster_simu/set_entity_pose");
               }
        }
    }
    catch(const std::exception & e){
        ROS_WARN("[action_executor] Failed to read toaster poster");
    }
}

/**
* \brief Function which return true if some facts are on the databases
* @param facts the facts
* \return true if the tested facts are in the robot table of the database
*/
bool PlannersManager::AreFactsInDB(std::vector<toaster_msgs::Fact> facts){

    ros::ServiceClient client = node_->serviceClient<toaster_msgs::ExecuteDB>("database_manager/execute");
    toaster_msgs::ExecuteDB srv;
    srv.request.command = "ARE_IN_TABLE";
    srv.request.agent = robotName_;
    srv.request.facts = facts;
    if (client.call(srv)){
        return srv.response.boolAnswer;
    }else{
       ROS_ERROR("[action_manager] Failed to call service database_manager/execute");
    }
    return false;
}

/**
* \brief Function which say if each facts are on the databases
* @param facts the facts
* \return a vector of bool representing for each fact if it is on the database
*/
std::vector<bool> PlannersManager::AreFactsInDBIndiv(std::vector<toaster_msgs::Fact> facts){

    std::vector<bool> res;
    ros::ServiceClient client = node_->serviceClient<toaster_msgs::ExecuteDB>("database_manager/execute");
    toaster_msgs::ExecuteDB srv;
    srv.request.command = "ARE_IN_TABLE";
    srv.request.type = "INDIV";
    srv.request.agent = robotName_;
    srv.request.facts = facts;
    if (client.call(srv)){
        res = srv.response.boolResults;
    }else{
       ROS_ERROR("[action_manager] Failed to call service database_manager/execute");
    }
    return res;
}

void PlannersManager::resetDB(){

    ros::ServiceClient client = node_->serviceClient<toaster_msgs::ExecuteDB>("database_manager/execute");
    toaster_msgs::ExecuteDB srv;
    srv.request.command = "EMPTY";
    srv.request.type = "ALL";
    if (!client.call(srv)){
        ROS_ERROR("[action_manager] Failed to call service database_manager/execute");
    }
}


/**
 * \brief Compute an int representing the world state to send to the MF
 * @param WSFacts the set of facts representing the world state
 * @return the int to send to the MF
 * */
std::string PlannersManager::computeWS(std::vector<toaster_msgs::Fact> WSFacts){

    std::string ws;
    //we look if the facts exists in the database
    std::vector<bool> results = AreFactsInDBIndiv(WSFacts);
    for(std::vector<bool>::iterator it = results.begin(); it != results.end(); it++){
        if(*it){
            ws = ws + "1";
        }else{
            ws = ws + "0";
        }
    }

    return ws;
}

/**
 * \brief Compare two action
 * @param action1 the first action to compare
 * @param action2 the second action to compare
 * @return true if the two actions are identical
 * */
bool PlannersManager::areActionsIdentical(roboergosum_msgs::Action action1, roboergosum_msgs::Action action2){

    if(action1.name != action2.name){
        return false;
    }else{
        if(action1.parameters.size() != action1.parameters.size()){
            return false;
        }else{
            for(int i = 0; i < action1.parameters.size(); i++){
                if(action1.parameters[i] != action2.parameters[i]){
                    return  false;
                }
            }
        }
        if(action1.actors.size() != action1.actors.size()){
            return false;
        }else{
            for(int i = 0; i < action1.actors.size(); i++){
                if(action1.actors[i] != action2.actors[i]){
                    return  false;
                }
            }
        }
    }

    return true;
}

/**
 * \brief Look if an integer is in a vector
 * @param i the int we look for
 * @param vector the vector where we look for
 * @return true if the int is in the vector
 * */
bool PlannersManager::isIntInVector(int i, std::vector<int> vector){

    for(std::vector<int>::iterator it = vector.begin(); it != vector.end(); it++){
        if(i == *it){
            return true;
        }
    }

    return false;
}

/**
 * \brief Execute a SQL command in the database
 * @param sql the command to execute
 * @return the result returned by the sql command
 * */
std::vector<std::string> PlannersManager::executeSQL(std::string sql){

    ros::ServiceClient client = node_->serviceClient<toaster_msgs::ExecuteDB>("database_manager/execute");
    toaster_msgs::ExecuteDB srv;
    srv.request.command = "SQL";
    if (client.call(srv)){
        return srv.response.results;
    }else{
        ROS_ERROR("[planners_manager] Failed to call service database_manager/execute");
        std::vector<std::string> result;
        return result;
    }
}

/**
 * \brief Ask a plan to HATP for the current goal
 * @return true if there is a plan and the plan
 * */
std::pair<bool, roboergosum_msgs::Plan> PlannersManager::GetHATPPlan(bool toBlock, roboergosum_msgs::Action actionToBlock){

    std::pair<bool, roboergosum_msgs::Plan> answer;

    ros::ServiceClient client = node_->serviceClient<hatp_msgs::PlanningRequest>("Planner");
    hatp_msgs::PlanningRequest service;

    //we add HATP flags
    if(toBlock){
        addHATPFlags(actionToBlock);
    }

    //We look for the HATP method name to call
    std::string methodTopic = "roboergosum/HATPMethodName";
    std::string methodName;
    node_->getParam(methodTopic, methodName);
    service.request.request.task=methodName;

    //We ask a plan to HATP
    service.request.request.type="plan";
    if(client.call(service)){
       if(service.response.solution.report == "OK"){
          answer.first = true;
          answer.second = convertPlan(service.response.solution);

       }else{
          answer.first = false;
      }
    }else{
        ROS_ERROR("[planners_manager] Failed to call service 'Planner'");
        answer.first = false;
    }

    return answer;
}

/**
 * \brief Function which convert a plan from HATP to a roboergosum plan
 * @param plan the HATP plan
 * @return a roboergosum plan
 * */
roboergosum_msgs::Plan PlannersManager::convertPlan(hatp_msgs::Plan plan){

   roboergosum_msgs::Plan newPlan;
   for(std::vector<hatp_msgs::Task>::iterator it = plan.tasks.begin(); it != plan.tasks.end(); it++){
      if(it->type){//the task is an action and not a method
          roboergosum_msgs::Action action;
          std::string nameTopic = "HATP_actions/";
          nameTopic = nameTopic + it->name;
          node_->getParam(nameTopic, action.name);
          action.id = it->id;
          action.actors = it->agents;
          //we remove the agents from the parameters
          int i;
          if(action.name == "give" || action.name == "grab"){
              i = 2;
          }else{
              i = 1;
          }
          for(i; i < it->parameters.size(); i++){
              action.parameters.push_back(it->parameters[i]);
          }
          newPlan.actions.push_back(action);
       }
    }
    for(std::vector<hatp_msgs::StreamNode>::iterator it = plan.streams.begin(); it != plan.streams.end(); it++){
       for(std::vector<unsigned int>::iterator itt = it->successors.begin(); itt != it->successors.end(); itt++){
          roboergosum_msgs::Link link;
          link.origin = it->taskId;
          link.following = *itt;
          newPlan.links.push_back(link);
       }
    }

    return newPlan;
}

/**
 * \brief Function which add hatp flags before asking for a plan
 * @param actionToBlock the action to block
 * */
void PlannersManager::addHATPFlags(roboergosum_msgs::Action actionToBlock){

    ros::ServiceClient client = node_->serviceClient<toaster_msgs::SetInfoDB>("database_manager/set_info");
    toaster_msgs::SetInfoDB srv;
    srv.request.infoType = "FACT";
    srv.request.agentId = robotName_;
    std::vector<toaster_msgs::Fact> toAddFacts;
    toaster_msgs::Fact fact;
    srv.request.add = true;

    //first we look for the handover flag
    fact.subjectId = "HERAKLES_HUMAN1";
    fact.property = "performHandover";
    fact.targetId = "true";
    if(actionToBlock.name == "give" || actionToBlock.name == "grab"){
        toAddFacts.push_back(fact);
        srv.request.facts = toAddFacts;
        if (!client.call(srv)){
         ROS_ERROR("[mental_state] Failed to call service database_manager/set_info");
        }
    }

    //then we look for human action
    fact.subjectId = "HERAKLES_HUMAN1";
    fact.property = "hasNotActed";
    fact.targetId = "true";
    if(actionToBlock.name == "humanAction"){
        toAddFacts.push_back(fact);
        srv.request.facts = toAddFacts;
        if (!client.call(srv)){
         ROS_ERROR("[mental_state] Failed to call service database_manager/set_info");
        }
    }

    //finally we look for pick failures
    if(actionToBlock.name == "pick" ){
        srv.request.add = true;
        fact.subjectId = actionToBlock.parameters[0];
        fact.property = "canNotBePicked";
        fact.targetId = "true";
        toAddFacts.push_back(fact);
        srv.request.facts = toAddFacts;
        if (!client.call(srv)){
         ROS_ERROR("[mental_state] Failed to call service database_manager/set_info");
        }
    }
}

/**
 * \brief Function which add hatp flags before asking for a plan
 * */
void PlannersManager::removeHATPFlags(){

    ros::ServiceClient client = node_->serviceClient<toaster_msgs::SetInfoDB>("database_manager/set_info");
    toaster_msgs::SetInfoDB srv;
    srv.request.infoType = "FACT";
    srv.request.agentId = robotName_;
    std::vector<toaster_msgs::Fact> toRmFacts;
    toaster_msgs::Fact fact;
    srv.request.add = false;

    //first we look for the handover flag
    fact.subjectId = "HERAKLES_HUMAN1";
    fact.property = "performHandover";
    fact.targetId = "true";
    toRmFacts.push_back(fact);

    //then we look for human action
    fact.subjectId = "HERAKLES_HUMAN1";
    fact.property = "hasNotActed";
    fact.targetId = "true";
    toRmFacts.push_back(fact);

    //finally we look for pick failures
    //we first remove the old ones
    fact.subjectId = "NULL";
    fact.property = "canNotBePicked";
    fact.targetId = "true";
    toRmFacts.push_back(fact);

    srv.request.add = false;
    srv.request.facts = toRmFacts;
    if (!client.call(srv)){
     ROS_ERROR("[mental_state] Failed to call service database_manager/set_info");
    }
}

/**
 * \brief Function which return the action corresponding to a given id
 * @param id the id of the action
 * @return the instantiated action corresponding to the id
 * */
roboergosum_msgs::Action PlannersManager::getActionFromId(int id){

    std::stringstream ss;
    ss << id;
    std::string idString = ss.str();

    roboergosum_msgs::Action action;
    action.actors.push_back(robotName_);

    //get the action name
    std::string nameTopic = "actions/" + idString + "/name";
    node_->getParam(nameTopic, action.name);
    if(action.name == "pick"){
        //we look for the object
        std::string objectTopic = "actions/" + idString + "/object";
        std::string object;
        node_->getParam(objectTopic, object);
        action.parameters.push_back(object);
        return action;
    }

    if(action.name == "place" || action.name == "drop" || action.name == "give"){
        //we retreive the object from attachment
        action.parameters.push_back(objectInHand_);
    }

    if(action.name == "grab"){
        //we get object form human hand
        action.parameters.push_back(objectInHumanHand_);
    }

    if(action.name == "place"){
        //we look for the support
        std::string supportTopic = "actions/" + idString + "/support";
        std::string support;
        node_->getParam(supportTopic, support);
        action.parameters.push_back(support);
        return action;
    }

    if(action.name == "drop"){
        //we get the good trashbin
        std::string colorTopic = "entities/objectsColor/" + objectInHand_;
        std::string objectColor;
        node_->getParam(colorTopic, objectColor);
        std::string trashbinTopic = "entities/trashbin/" + objectColor;
        std::string trashbin;
        node_->getParam(trashbinTopic, trashbin);
        action.parameters.push_back(trashbin);
        return action;
    }

    if(action.name == "give" || action.name == "grab"){
        //we add receiver/sender
        action.parameters.push_back("HERAKLES_HUMAN1");
        return action;
    }

    if(action.name == "navigate"){
        //we add the target location
        if(robotPose_ == "initialLocation"){
            action.parameters.push_back("secondLocation");
        }else{
            action.parameters.push_back("initialLocation");
        }
        return action;
    }

    return action;
}

/**
 * \brief Function which return the id corresponding to an action
 * @param action the instantiated action corresponding to the id
 * @return the id of the action
 * */
int PlannersManager::getIdFromAction(roboergosum_msgs::Action action){

    int nbActions;
    node_->getParam("actions/nbActions", nbActions);
    for(int i = 0; i < nbActions; i++){
        std::stringstream ss;
        ss << i;
        std::string idString = ss.str();

        //get the action name
        std::string name;
        std::string nameTopic = "actions/" + idString + "/name";
        node_->getParam(nameTopic, name);
        if(action.name == name){
            if(name == "pick"){
                std::string object;
                std::string objectTopic = "actions/" + idString + "/object";
                node_->getParam(objectTopic, object);
                if(object == action.parameters[0]){
                    return i;
                }
            }else if(name == "place"){
                std::string support;
                std::string supportTopic = "actions/" + idString + "/support";
                node_->getParam(supportTopic, support);
                if(support == action.parameters[1]){
                    return i;
                }
            }else{
                return i;
            }
        }
    }

}
