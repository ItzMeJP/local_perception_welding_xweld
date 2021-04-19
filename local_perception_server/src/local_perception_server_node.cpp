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

    ros::NodeHandlePtr node_handle(new ros::NodeHandle());
    ros::NodeHandlePtr private_node_handle(new ros::NodeHandle("~"));

    std::string ros_verbosity_level;
    private_node_handle->param("ros_verbosity_level", ros_verbosity_level, std::string("DEBUG"));
    local_perception_server::verbosity_levels::setVerbosityLevelROS(ros_verbosity_level);

    std::string pcl_verbosity_level;
    private_node_handle->param("pcl_verbosity_level", pcl_verbosity_level, std::string("DEBUG"));
    local_perception_server::verbosity_levels::setVerbosityLevelPCL(pcl_verbosity_level);

    local_perception_server::LocalPerception node;
    node.setupParameterServer(node_handle,private_node_handle);
    node.start();

    ros::spin();

}
