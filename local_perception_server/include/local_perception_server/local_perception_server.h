
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>
#include <vector>
#include <cstdlib>
#include <numeric>
#include <fstream>
#include <iostream>
#include <angles/angles.h>


//#include <pcl/types.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/colors.h>
#include <pcl_conversions/pcl_conversions.h>


#include <pcl/common/intersections.h>
#include <pcl/filters/project_inliers.h>

//#include <pcl/filters/crop_box.h>

//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/io/pcd_io.h>

#include <bits/stdc++.h>

#include <local_perception_server/common/pcl_complement.h>
#include <local_perception_server/common/verbosity_levels.h>

#include <actionlib/server/simple_action_server.h>
#include <local_perception_msgs/LocalPerceptionAction.h>
#include <local_perception_msgs/PointArr.h>
#include <local_perception_msgs/WeldingArr.h>

#ifndef LOCAL_PERCEPTION_SERVER_H
#define LOCAL_PERCEPTION_SERVER_H
namespace local_perception_server {
    class LocalPerception {
    public:

        typedef pcl::PointXYZRGBNormal PointLP;
        typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudLP;
        typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudPtrLP;
        typedef actionlib::SimpleActionServer<local_perception_msgs::LocalPerceptionAction> LocalPerceptionActionServer;

        struct InfiniteLineDescriptors{
            Eigen::Vector3d reference_point,
                            direction_vector;
        };

        struct BoxParametrization{
            std::vector<double> min_vertex_arr, max_vertex_arr;
        };

        enum ERROR_LIST{
            CLOUD_RECEPTION_TIMEOUT = 101,
            PLANE_SEG_SMALL_INPUT_CLOUD_SIZE,
            PLANE_SEG_UNABLE_TO_ESTIMATE_A_FIT_MODEL,
            INPUT_CLOUD_CROPBOX_ERROR,
            INPUT_CLOUD_VOXELGRID_ERROR,
            SMALL_NUMBER_OF_PLANES_FOUND,
            LINE_ESTIMATION_PLANE_PARALLEL_ERROR
        };

        enum CAMERA_FRAME_NORM_TYPE{
            OPTICAL,
            ROS
        };

        LocalPerception();
        ~LocalPerception();

        void start(); //start the server
        void executeCB (const local_perception_msgs::LocalPerceptionGoalConstPtr &goal); // recognize the goal request
        void feedback (float percentage);
        void setSucceeded (std::string outcome = "succeeded");
        void setAborted (std::string outcome);
        bool checkPreemption ();

        void setupParameterServer(ros::NodeHandlePtr &_node_handle,
                                  ros::NodeHandlePtr &_private_node_handle);

        void setupVoxelGridFilterConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle,
                                                                  ros::NodeHandlePtr &_private_node_handle);

        void setupRansacPlaneSegmentationConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle,
                                                                          ros::NodeHandlePtr &_private_node_handle);

        void setupCropboxFilterFilterConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle, ros::NodeHandlePtr &_private_node_handle);

        //void setupLineErrorAssesment(ros::NodeHandlePtr &_node_handle,
        //                             ros::NodeHandlePtr &_private_node_handle);

        void setupOutputPoseFilter(ros::NodeHandlePtr &_node_handle,
                                   ros::NodeHandlePtr &_private_node_handle);

        void setupPlaneIntersectionConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle,
                                                                    ros::NodeHandlePtr &_private_node_handle);

        bool run(local_perception_msgs::LocalPerceptionGoalConstPtr);


    protected:


        std::shared_ptr<LocalPerceptionActionServer>                             actionServer_; // to not init it in the constructor
        std::string                                                              action_server_name_ = "LocalPerceptionActionServer";
        local_perception_msgs::LocalPerceptionFeedback                      feedback_;
        local_perception_msgs::LocalPerceptionResult                        result_;

        double seg_thresehold_,
               normal_dist_weight_,
               eps_angle_,
               normal_radius_search_,
               inliers_threshold_weight_,
               left_rate_to_all_candidates_,
               distance_to_line_threshold_,
               gt_line_offset_,
               input_cloud_timeout_threshold_,
               hfov_,
               vfov_,
               distance_range_;

        int max_iterations_,
            normal_k_search_,
            tolerance_iterator_threshold_,
            segmented_index_,
            index_cloud_plane_a_,
            index_cloud_plane_b_,
            sac_type_,
            model_type_,
            welding_interval_,
            cropbox_frame_id_norm_,
            max_number_of_planes_;

        bool cropbox_activation_;


        std::string axis_,
                    input_cloud_topic_,
                    plane_gt_parent_frame_,
                    output_segmented_cloud_merged_topic_,
                    output_segmented_cloud_topic_,
                    output_plane_gt_topic_,
                    output_input_cloud_filtered_,
                    output_line_cloud_,
                    output_region_cloud_,
                    output_region_centroid_cloud_,
                    output_robot_poses_cloud_,
                    output_gt_line_cloud_cloud_,
                    gt_line_frame_id_,
                    gt_line_axis_,
                    aborted_msg_;

        std::vector<double> voxel_leaf_sizes_arr_;


        ros::NodeHandlePtr node_handle_,
                           private_node_handle_;

        pcl::PCLPointCloud2::Ptr cloud_raw;
        PointCloudPtrLP input_cloud_;

        ros::Subscriber input_cloud_sub_;

        ros::Publisher pub_seg_merged_,
                       pub_seg_,
                       pub_input_filtered_,
                       pub_plane_gt_,
                       pub_intersection_region_cloud_,
                       pub_intersection_centroid_cloud_,
                       pub_intersection_line_cloud_,
                       pub_robot_poses_cloud_,
                       pub_gt_line_cloud_;

        std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

        std::vector<geometry_msgs::TransformStamped> robot_pose_tf_arr_;

        tf2_ros::StaticTransformBroadcaster         static_broadcaster_; // check the problem of continuous tf publishing

        void CloudChatterCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);

        bool planeSegmentation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _input_cloud,
                               pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & _segmented_cloud_merged,
                               std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> & _output_arr,
                               std::vector<pcl::ModelCoefficients>& _coefs_output_arr);

        bool inputPointCloudVerification();


        bool generateIntersectionLine( pcl::ModelCoefficients coefs_plane_a,
                                       pcl::ModelCoefficients coefs_plane_b,
                                       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& line_cloud,
                                       InfiniteLineDescriptors &line);
                                       //Eigen::VectorXf& coefs_line); //first 3 parameters- initial point nearest to zero. last 3 parameters = direction vector

        bool calculateIntersectionRegion(PointCloudPtrLP input_cloud, InfiniteLineDescriptors _line , PointCloudPtrLP &intersection_region_cloud );

        bool projectPointCloudToLine(PointCloudPtrLP _input_cloud, InfiniteLineDescriptors _line, std::vector<Eigen::Vector3d> &_projected_point_arr);

        bool generateWeldingPoses (std::vector<Eigen::Vector3d> _projected_point_arr, InfiniteLineDescriptors _line,
                                   std::string _parent_frame_id, std::vector<geometry_msgs::TransformStamped> &_poses_arr);

        bool createLineCloud(Eigen::VectorXf& coefs_line, //first 3 parameters- initial point nearest to zero. last 3 parameters = direction vector
                             pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& line_cloud);

        bool setupCropboxParameters(double _linear_distance,BoxParametrization &_cropbox_parameters);

        bool applyCorrection(const std::vector<float> offset, std::vector<geometry_msgs::TransformStamped> &_poses_arr );

    };
}
#endif //LOCAL_PERCEPTION_SERVER_H
