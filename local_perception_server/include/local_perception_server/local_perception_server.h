
/**\file local_perception_server.h
 * \brief definition of local perception server in ROS
 *
 * @version 3.0.0
 * @author João Pedro Carvalho de Souza
 * @email: josouza@fe.up.pt
 */


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
#include <algorithm>

#include <std_vector_complement/std_vector_operations.h>


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
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>


#include <bits/stdc++.h>

#include <local_perception_server/common/pcl_complement.h>
#include <local_perception_server/common/verbosity_levels.h>

#include <actionlib/server/simple_action_server.h>
#include <local_perception_msgs/LocalPerceptionAction.h>
#include <local_perception_msgs/PointArr.h>
#include <local_perception_msgs/WeldingArr.h>
#include <local_perception_msgs/LocalQualityPerceptionAction.h>
#include <local_perception_msgs/QualityMsg.h>
#include <local_perception_msgs/QualityArr.h>

#ifndef LOCAL_PERCEPTION_SERVER_H
#define LOCAL_PERCEPTION_SERVER_H
namespace local_perception_server {
    class LocalPerception {
    public:

        typedef pcl::PointXYZRGBNormal PointLP;
        typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudLP;
        typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudPtrLP;

        typedef actionlib::SimpleActionServer<local_perception_msgs::LocalPerceptionAction> LocalPerceptionActionServer;
        typedef actionlib::SimpleActionServer<local_perception_msgs::LocalQualityPerceptionAction> LocalQualityPerceptionActionServer;

        struct InfiniteLineDescriptors{
            Eigen::Vector3d reference_point,
                            direction_vector;
        };

        struct BoxParametrization{
            std::vector<double> min_vertex_arr, max_vertex_arr;
        };

        struct PointDistances{
            std::string parent_frame;
            double x,y,z,distance;
        };

        enum ERROR_LIST{
            CLOUD_RECEPTION_TIMEOUT = 101,
            PLANE_SEG_SMALL_INPUT_CLOUD_SIZE,
            PLANE_SEG_UNABLE_TO_ESTIMATE_A_FIT_MODEL,
            INPUT_CLOUD_CROPBOX_ERROR,
            INPUT_CLOUD_VOXELGRID_ERROR,
            SMALL_NUMBER_OF_PLANES_FOUND,
            LINE_ESTIMATION_PLANE_PARALLEL_ERROR,
            ICP_NOT_CONVERGED,
        };

        enum CAMERA_FRAME_NORM_TYPE{
            OPTICAL,
            ROS
        };

        LocalPerception();
        ~LocalPerception();

        void start(); //start the server

        void executeCB (const local_perception_msgs::LocalPerceptionGoalConstPtr &goal); // recognize the goal request
        void executeQualityCB (const local_perception_msgs::LocalQualityPerceptionGoalConstPtr &_goal);

        void feedback (float percentage);
        void setSucceeded (std::string outcome = "succeeded");
        void setAborted (std::string outcome);
        bool checkPreemption ();

        void feedbackQuality (float percentage);
        void setQualitySucceeded (std::string outcome = "succeeded");
        void setQualityAborted (std::string outcome);
        bool checkQualityPreemption ();

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

        void setupWeldingQualityConfigurationFromParameterServer (ros::NodeHandlePtr &_node_handle, ros::NodeHandlePtr &_private_node_handle);

        void setupPlaneIntersectionConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle,
                                                                    ros::NodeHandlePtr &_private_node_handle);

        bool runWeldingEstimationPipeline(local_perception_msgs::LocalPerceptionGoalConstPtr);

        bool runQualityPipeline(local_perception_msgs::LocalQualityPerceptionGoalConstPtr);

        bool runBasicPipeline();


        bool runQualityEvaluation();

        bool getAlignment(Eigen::Matrix4d &T);

        bool buildWeldingLineFrame();
        bool buildWeldingLineFrame(geometry_msgs::TransformStamped _custom_position);

    protected:


        std::shared_ptr<LocalPerceptionActionServer>                             actionServer_; // to not init it in the constructor
        std::string                                                              action_server_name_ = "LocalPerceptionActionServer";

        local_perception_msgs::LocalPerceptionFeedback                      feedback_;
        local_perception_msgs::LocalPerceptionResult                        result_;

        std::shared_ptr<LocalQualityPerceptionActionServer>                      actionQualityServer_; // to not init it in the constructor
        std::string                                                              action_quality_server_name_ = "LocalQualityPerceptionActionServer";

        local_perception_msgs::LocalQualityPerceptionFeedback                      quality_feedback_;
        local_perception_msgs::LocalQualityPerceptionResult                        quality_result_;

        struct MergedGoals{
            double  acquisition_distance,
                    deposit_radius_tolerance,
                    edge_tolerance;
            std::vector<float> offset_compensation;
            std::string output_path;
        } merged_goals;

        Eigen::Affine3d transformation_aligned_line_frame_wrt_sensor_;

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

        InfiniteLineDescriptors estimated_welding_line_;

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

        bool cropbox_activation_,
             lock_input_pc_callback_mutex_ = true;

        std::vector<geometry_msgs::TransformStamped> welding_poses_arr_;

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
                    aborted_msg_,
                    output_welding_deposit_region_cloud_;

        std::vector<double> voxel_leaf_sizes_arr_;

        geometry_msgs::TransformStamped t_aligned_ref_wrt_sensor_;

        std::stringstream logStreamStr_, logStreamStrPC_;

        ros::NodeHandlePtr node_handle_,
                           private_node_handle_;

        pcl::PCLPointCloud2::Ptr cloud_raw;
        PointCloudPtrLP input_cloud_,
                        input_cloud_bkup_,
                        post_cropbox_input_cloud_bkup_,
                        all_segmented_planes_cloud_,
                        plane_cloud_a_, plane_cloud_b_, plane_cloud_ab_,
                        estimated_welding_line_cloud_,
                        intersection_region_cloud_,
                        welding_deposit_cloud_wrt_sensor_,
                        welding_deposit_cloud_wrt_line_axis_;
                        ;

        std::vector<PointCloudLP> all_segmented_planes_arr_;


        ros::Subscriber input_cloud_sub_;

        ros::Publisher pub_seg_merged_,
                       pub_seg_,
                       pub_input_filtered_,
                       pub_plane_gt_,
                       pub_intersection_region_cloud_,
                       pub_intersection_centroid_cloud_,
                       pub_intersection_line_cloud_,
                       pub_robot_poses_cloud_,
                       pub_gt_line_cloud_,
                       pub_welding_deposit_region_cloud_;

        std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

        std::vector<geometry_msgs::TransformStamped> robot_pose_tf_arr_;

        tf2_ros::StaticTransformBroadcaster         static_broadcaster_; // check the problem of continuous tf publishing

        void CloudChatterCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);

        bool planeSegmentation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _input_cloud,
                               pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & _segmented_cloud_merged,
                               std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> & _output_arr,
                               std::vector<pcl::ModelCoefficients>& _coefs_output_arr);

        bool cylinderSegmentation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _input_cloud,
                                  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & _segmented_cloud,
                                  pcl::ModelCoefficients& _coefs);

        bool inputPointCloudVerification();


        bool generateIntersectionLine( pcl::ModelCoefficients coefs_plane_a,
                                       pcl::ModelCoefficients coefs_plane_b,
                                       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& line_cloud,
                                       InfiniteLineDescriptors &line);
                                       //Eigen::VectorXf& coefs_line); //first 3 parameters- initial point nearest to zero. last 3 parameters = direction vector

        bool calculateIntersectionRegion(PointCloudPtrLP input_cloud, InfiniteLineDescriptors _line , double _dist_threshold, PointCloudPtrLP &intersection_region_cloud );

        bool projectPointCloudToLine(PointCloudPtrLP _input_cloud, InfiniteLineDescriptors _line, std::vector<Eigen::Vector3d> &_projected_point_arr);

        bool distFromLine(PointCloudPtrLP _input_cloud, InfiniteLineDescriptors _line, std::vector<double> &_distance_arr);

        bool generateWeldingPoses (std::vector<Eigen::Vector3d> _projected_point_arr, InfiniteLineDescriptors _line,
                                   std::string _parent_frame_id, std::vector<geometry_msgs::TransformStamped> &_poses_arr);

        bool createLineCloud(Eigen::VectorXf& coefs_line, //first 3 parameters- initial point nearest to zero. last 3 parameters = direction vector
                             pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& line_cloud);

        bool setupCropboxParameters(double _linear_distance,BoxParametrization &_cropbox_parameters);

        bool applyCorrection(const std::vector<float> offset, std::vector<geometry_msgs::TransformStamped> &_poses_arr );

        bool isInputCloudCallbackMutexLocked();

        void lockCloudCallbackMutex();

        void unlockCloudCallbackMutex();

        bool runDebugTools();

        bool exportQualityResult();

        bool buildResult(std::vector<double> _input_arr, std::string _post_prefix, local_perception_msgs::QualityArr &_result_arr);

        bool exportCSVFile();

        };
}
#endif //LOCAL_PERCEPTION_SERVER_H
