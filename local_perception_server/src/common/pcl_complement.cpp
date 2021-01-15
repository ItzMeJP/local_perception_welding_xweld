/**\file math_common.cpp
 * \brief Utils PCL standlone and PCL+ROS operations and functions
 *
 * @version 1.0
 * @author João Pedro Carvalho de Souza
 */

#include <local_perception_server/common/pcl_complement.h>

namespace local_perception_server {
    namespace pcl_complement {

        bool pubCloud (ros::Publisher &_pub, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _p) {

            pcl::PCLPointCloud2 point_cloud2;
            pcl::toPCLPointCloud2(*_p, point_cloud2);
            point_cloud2.header = _p->header;
            pcl_conversions::toPCL(ros::Time::now(), point_cloud2.header.stamp);
            _pub.publish(point_cloud2);

            return true;
        }

        bool pubCloud (ros::Publisher &_pub, pcl::PointCloud<pcl::PointXYZRGBNormal> _p) {

            pcl::PCLPointCloud2 point_cloud2;
            pcl::toPCLPointCloud2(_p, point_cloud2);
            point_cloud2.header = _p.header;
            pcl_conversions::toPCL(ros::Time::now(), point_cloud2.header.stamp);
            _pub.publish(point_cloud2);

            return true;
        }

        //this fuction was needded since, the current PCL version has an issue in cropbox.
        bool applyCropbox (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloud, std::vector<double> _min_arr,
                                            std::vector<double> _max_arr) {


            Eigen::Vector4f min(_min_arr.at(0), _min_arr.at(1), _min_arr.at(2), 1.0),
                            max(_max_arr.at(0), _max_arr.at(1), _max_arr.at(2), 1.0);

            if(!applyCropbox(_cloud, min, max))
                return false;

            return true;
        }

        //this fuction was needded since, the current PCL version has an issue in cropbox.
        bool applyCropbox (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& _inputCloud, Eigen::Vector4f _min_pt,
                                            Eigen::Vector4f _max_pt) {

            if (_inputCloud->width == 0) {
                ROS_ERROR_STREAM("Empty input cloud in cropbox");
                return false;
            }

            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _outputCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::PointXYZRGBNormal *point = new pcl::PointXYZRGBNormal;

            _outputCloud->clear();

            for (u_int j = 0; j < _inputCloud->width; ++j) {

                if ((_inputCloud->points.at(j).x < _min_pt[0] || _inputCloud->points.at(j).y < _min_pt[1] ||
                     _inputCloud->points.at(j).z < _min_pt[2]) ||
                    (_inputCloud->points.at(j).x > _max_pt[0] || _inputCloud->points.at(j).y > _max_pt[1] ||
                     _inputCloud->points.at(j).z > _max_pt[2])) {
                    continue;
                }

                point->x = _inputCloud->points.at(j).x;
                point->y = _inputCloud->points.at(j).y;
                point->z = _inputCloud->points.at(j).z;

                _outputCloud->points.push_back(*point);
                _outputCloud->width++;

            }
            delete point;

            _outputCloud->height          = 1;
            _outputCloud->header.frame_id = _inputCloud->header.frame_id;
            _outputCloud->header.stamp    = _inputCloud->header.stamp;

            _inputCloud->clear();
            *_inputCloud = *_outputCloud;
            return true;
        }

        bool applyVoxelGrid (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloud,
                                              std::vector<double> _voxel_leaf_sizes_arr) {


            if (_cloud->width == 0) {
                ROS_ERROR_STREAM("Empty input cloud in voxel grid filter");
                return false;
            }

            if (_voxel_leaf_sizes_arr.at(0) == 0 && _voxel_leaf_sizes_arr.at(1) == 0 && _voxel_leaf_sizes_arr.at(2) == 0) {
                ROS_WARN_STREAM("Voxel grid parameter is zero. Deactivating filter");
                return true;
            } else {
                pcl::VoxelGrid<pcl::PointXYZRGBNormal> sor;
                sor.setInputCloud(_cloud);
                sor.setLeafSize(_voxel_leaf_sizes_arr.at(0), _voxel_leaf_sizes_arr.at(1), _voxel_leaf_sizes_arr.at(2));
                sor.filter(*_cloud);
            }

            return true;
        }

        bool getCentroid(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _cloud, pcl::PointXYZRGBNormal _centroid_point){

            if (_cloud->width == 0) {
                ROS_ERROR_STREAM("Empty input cloud in centroid calculation");
                return false;
            }

            pcl::CentroidPoint<pcl::PointXYZRGBNormal> centroid;

            for (size_t it = 0; it < _cloud->points.size(); ++it) {
                centroid.add(_cloud->points.at(it));
            }

            centroid.get(_centroid_point);

            return true;

        }

    }//end namespace
}//end namespace