/**\file math_common.cpp
 * \brief Utils PCL standalone and PCL+ROS operations and functions
 *
 * @version 1.0
 * @author João Pedro Carvalho de Souza
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>

namespace local_perception_server {
    namespace pcl_complement {

        bool pubCloud(ros::Publisher& _pub, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _p);
        bool pubCloud(ros::Publisher& _pub, pcl::PointCloud<pcl::PointXYZRGBNormal> _p);

        bool applyCropbox(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& _cloud, std::vector<double> _min_arr, std::vector<double> _max_arr);
        bool applyCropbox(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& _inputCloud, Eigen::Vector4f _min_pt, Eigen::Vector4f _max_pt);
        bool applyCropbox (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloud, pcl::PointXYZRGBNormal _min_point, pcl::PointXYZRGBNormal _max_point);

        bool applyVoxelGrid(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& _cloud, std::vector<double> _voxel_leaf_sizes_arr);
        bool getCentroid(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _cloud, pcl::PointXYZRGBNormal _centroid_point);

    }
}