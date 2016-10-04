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
 * @return the int to send to the MF
 * */
long long int PlannersManager::computeWS(){

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
        //we look if the fact exists in the database
        std::vector<toaster_msgs::Fact> facts;
        facts.push_back(fact);
        if(AreFactsInDB(facts)){
            result = result + pow(2, i);
        }
    }

    return result;
}
