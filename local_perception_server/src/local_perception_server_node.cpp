/**\file
 * \brief
 * Node to estimate welding joint
 * @version 0.0
 * @author João Pedro Carvalho de Souza
 */

#include <local_perception_server/local_perception_server.h>


int main(int argc, char** argv)
{

    ros::init(argc, argv, "local_perception_server_node");

    ROS_INFO(">> Aperte enter");
    getchar();
    ROS_INFO(">> Enter pressionado");

    ros::NodeHandlePtr node_handle(new ros::NodeHandle());
    ros::NodeHandlePtr private_node_handle(new ros::NodeHandle("~"));

    local_perception_server::LocalPerception node;
    node.setupParameterServer(node_handle,private_node_handle);
    node.run();

    ros::spin();

}
