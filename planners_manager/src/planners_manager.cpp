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
}

/**
 * \brief Set the environment in toaster at the initial point (based on params)
 * */
void PlannersManager::setEnvironment(){

    //we reset the database to not have old facts in it
    resetDB();

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
long long int PlannersManager::computeWS(std::vector<toaster_msgs::Fact> WSFacts){

    long long int result = 0;
    int i = 0;
    for(std::vector<toaster_msgs::Fact>::iterator it = WSFacts.begin(); it != WSFacts.end(); it++){
        //we look if the fact exists in the database
        std::vector<toaster_msgs::Fact> facts;
        facts.push_back(*it);
        if(AreFactsInDB(facts)){
            result = result + pow(2, i);
        }
        i++;
    }

    return result;
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
        ROS_ERROR("[action_manager] Failed to call service database_manager/execute");
        std::vector<std::string> result;
        return result;
    }
}
