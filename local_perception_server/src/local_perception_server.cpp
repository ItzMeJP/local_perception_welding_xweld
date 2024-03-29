
/**\file local_perception_server.cpp
 * \brief implementation of local perception server in ROS
 *
 * @version 3.0.0
 * @author João Pedro Carvalho de Souza
 * @email: josouza@fe.up.pt
 */


#include <local_perception_server/local_perception_server.h>

namespace local_perception_server {

    LocalPerception::LocalPerception () {

        cloud_raw = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);
        input_cloud_ = PointCloudPtrLP (new PointCloudLP);
        input_cloud_bkup_ = PointCloudPtrLP (new PointCloudLP);
        post_cropbox_input_cloud_bkup_ = PointCloudPtrLP (new PointCloudLP);
        plane_cloud_a_= PointCloudPtrLP (new PointCloudLP);
        plane_cloud_b_ = PointCloudPtrLP (new PointCloudLP);
        plane_cloud_ab_ = PointCloudPtrLP (new PointCloudLP);
        estimated_welding_line_cloud_ = PointCloudPtrLP (new PointCloudLP);
        intersection_region_cloud_ = PointCloudPtrLP (new PointCloudLP);
        welding_deposit_cloud_wrt_sensor_ = PointCloudPtrLP (new PointCloudLP);
        welding_deposit_cloud_wrt_line_axis_ = PointCloudPtrLP (new PointCloudLP);
        all_segmented_planes_cloud_ = PointCloudPtrLP (new PointCloudLP);

        tf_buffer_       = std::make_shared<tf2_ros::Buffer>(ros::Duration(600));
        tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    }

    LocalPerception::~LocalPerception () {}

    /// ###############################################################################################################
    /// ROS action related
    /// ###############################################################################################################
    void LocalPerception::start() {

        actionServer_ = std::make_shared<LocalPerceptionActionServer>(*node_handle_, action_server_name_,
                                                                    boost::bind(&LocalPerception::executeCB,
                                                                                this, _1));

        actionQualityServer_ = std::make_shared<LocalQualityPerceptionActionServer>(*node_handle_, action_quality_server_name_,
                                                                             boost::bind(&LocalPerception::executeQualityCB,
                                                                                         this, _1));
        actionServer_->start();
        actionQualityServer_->start();
    }

    /// ##############################################
    /// Local perception server action definitions
    /// ##############################################

    void LocalPerception::executeCB (const local_perception_msgs::LocalPerceptionGoalConstPtr &_goal) {
        result_.welding_list.welding.clear();
        aborted_msg_ = "Aborted. Error(s):";
        runWeldingEstimationPipeline(_goal)?setSucceeded():setAborted(aborted_msg_);
    }

    void LocalPerception::setSucceeded (std::string outcome) {
        result_.percentage  = 100;
        result_.skillStatus = action_server_name_;
        result_.skillStatus += ": Succeeded";
        result_.outcome     = outcome;
        ROS_INFO_STREAM(action_server_name_ << ": Succeeded");
        actionServer_->setSucceeded(result_);
    }

    void LocalPerception::setAborted (std::string outcome) {
        result_.percentage  = 0;
        result_.skillStatus = action_server_name_;
        result_.skillStatus += ": Aborted";
        result_.outcome     = outcome;
        ROS_INFO_STREAM(action_server_name_ << ": Aborted");
        actionServer_->setAborted(result_);
    }


    void LocalPerception::feedback (float percentage) {
        feedback_.percentage  = percentage;
        feedback_.skillStatus = action_server_name_;
        feedback_.skillStatus += " Executing";
        ROS_INFO_STREAM(action_server_name_ << ": Executing. Percentage" << percentage);
        actionServer_->publishFeedback(feedback_);
    }

    bool LocalPerception::checkPreemption () {
        if (actionServer_->isPreemptRequested() || !ros::ok()) {
            result_.percentage  = 0;
            result_.skillStatus = action_server_name_;
            result_.skillStatus += ": Preempted";
            result_.outcome     = "preempted";
            ROS_INFO_STREAM(action_server_name_ << ": Preempted");
            actionServer_->setPreempted(result_);
            return true;
        } else {
            return false;
        }
    }
    /// ####################################################
    /// Local quality perception server action definitions
    /// ####################################################


    void LocalPerception::executeQualityCB (const local_perception_msgs::LocalQualityPerceptionGoalConstPtr &_goal) {
        quality_result_.output.quality_list.clear();
        aborted_msg_ = "Aborted. Error(s):";
        runQualityPipeline(_goal)?setQualitySucceeded():setQualityAborted(aborted_msg_);
    }

    void LocalPerception::setQualitySucceeded (std::string outcome) {
        quality_result_.percentage  = 100;
        quality_result_.skillStatus = action_quality_server_name_;
        quality_result_.skillStatus += ": Succeeded";
        quality_result_.outcome     = outcome;
        ROS_INFO_STREAM(action_quality_server_name_ << ": Succeeded");
        actionQualityServer_->setSucceeded(quality_result_);
    }

    void LocalPerception::setQualityAborted (std::string outcome) {
        quality_result_.percentage  = 0;
        quality_result_.skillStatus = action_quality_server_name_;
        quality_result_.skillStatus += ": Aborted";
        quality_result_.outcome     = outcome;
        ROS_INFO_STREAM(action_quality_server_name_ << ": Aborted");
        actionQualityServer_->setAborted(quality_result_);
    }

    void LocalPerception::feedbackQuality (float percentage) {
        quality_feedback_.percentage  = percentage;
        quality_feedback_.skillStatus = action_quality_server_name_;
        quality_feedback_.skillStatus += " Executing";
        ROS_INFO_STREAM(action_quality_server_name_ << ": Executing. Percentage" << percentage);
        actionQualityServer_->publishFeedback(quality_feedback_);
    }
    bool LocalPerception::checkQualityPreemption () {
        if (actionQualityServer_->isPreemptRequested() || !ros::ok()) {
            quality_result_.percentage  = 0;
            quality_result_.skillStatus = action_quality_server_name_;
            quality_result_.skillStatus += ": Preempted";
            quality_result_.outcome     = "preempted";
            ROS_INFO_STREAM(action_quality_server_name_ << ": Preempted");
            actionQualityServer_->setPreempted(quality_result_);
            return true;
        } else {
            return false;
        }
    }

    /// ###############################################################################################################
    /// Parameter server related
    /// ###############################################################################################################
    void LocalPerception::setupParameterServer(ros::NodeHandlePtr &_node_handle,
                              ros::NodeHandlePtr &_private_node_handle){

        node_handle_         = _node_handle;
        private_node_handle_ = _private_node_handle;

        this->setupPlaneIntersectionConfigurationFromParameterServer(_node_handle,_private_node_handle);
        this->setupRansacPlaneSegmentationConfigurationFromParameterServer(_node_handle,_private_node_handle);
        this->setupVoxelGridFilterConfigurationFromParameterServer(_node_handle,_private_node_handle);
        this->setupCropboxFilterFilterConfigurationFromParameterServer(_node_handle,_private_node_handle);
        this->setupOutputPoseFilter(_node_handle,_private_node_handle);
        //this->setupLineErrorAssesment(_node_handle,_private_node_handle);
        this->setupWeldingQualityConfigurationFromParameterServer(_node_handle,_private_node_handle);


    }

    void LocalPerception::setupCropboxFilterFilterConfigurationFromParameterServer (ros::NodeHandlePtr &_node_handle,
                                                                          ros::NodeHandlePtr &_private_node_handle) {

        _private_node_handle->param<bool>("cropbox/enable", cropbox_activation_, false);
        _private_node_handle->param<int>("cropbox/frame_id_norm", cropbox_frame_id_norm_, false);  // which convention is used to define the point cloud parent frame

        if (!cropbox_activation_) {
            ROS_WARN("Deactivating cropbox");
            return;
        }

        _private_node_handle->param<double>("cropbox/horizontal_fov", hfov_, 80.0);
        _private_node_handle->param<double>("cropbox/vertical_fov", vfov_, 60.0);
        _private_node_handle->param<double>("cropbox/distance_range", distance_range_, 0.025);

    }

    void LocalPerception::setupVoxelGridFilterConfigurationFromParameterServer (ros::NodeHandlePtr &_node_handle,
                                                                      ros::NodeHandlePtr &_private_node_handle) {

        voxel_leaf_sizes_arr_.clear();
        if (!_private_node_handle->param("voxel_grid_filter/leaf_sizes", voxel_leaf_sizes_arr_,
                                         std::vector<double>())) {
            voxel_leaf_sizes_arr_.push_back(0);
            voxel_leaf_sizes_arr_.push_back(0);
            voxel_leaf_sizes_arr_.push_back(0);
            ROS_WARN_STREAM("Error reading voxel leaf sizes. Using zero values");
        }

    }

    void LocalPerception::setupOutputPoseFilter (ros::NodeHandlePtr &_node_handle,
                                       ros::NodeHandlePtr &_private_node_handle) {

        _private_node_handle->param<int>("pose_filter/welding_interval", welding_interval_, 10);

    }

    /*
    void LocalPerception::setupLineErrorAssesment (ros::NodeHandlePtr &_node_handle,
                                         ros::NodeHandlePtr &_private_node_handle) {

        _private_node_handle->param<std::string>("line_error/gt_line_frame_id", gt_line_frame_id_, "wobj_weld");
        _private_node_handle->param<std::string>("line_error/gt_line_axis", gt_line_axis_, "x");
        _private_node_handle->param<double>("line_error/gt_line_offset", gt_line_offset_, 10);


    }
    */
    void LocalPerception::setupRansacPlaneSegmentationConfigurationFromParameterServer (ros::NodeHandlePtr &_node_handle,
                                                                              ros::NodeHandlePtr &_private_node_handle) {

        _private_node_handle->param<double>("sac_method/segmentation_threshold", seg_thresehold_, 3);
        _private_node_handle->param<double>("sac_method/normal_distance_weight", normal_dist_weight_, 0.1);
        _private_node_handle->param<double>("sac_method/eps_angle", eps_angle_, 0);
        _private_node_handle->param<double>("sac_method/normal_radius_search", normal_radius_search_, 0);
        _private_node_handle->param<int>("sac_method/max_iterations", max_iterations_, 10000);
        _private_node_handle->param<int>("sac_method/normal_k_search", normal_k_search_, 9);
        _private_node_handle->param<int>("sac_method/tolerance_iterator_threshold", tolerance_iterator_threshold_, 5);
        _private_node_handle->param<double>("sac_method/left_rate_to_all_candidates", left_rate_to_all_candidates_,
                                            0.1);
        _private_node_handle->param<int>("sac_method/max_number_of_planes", max_number_of_planes_, 2);
        _private_node_handle->param<double>("sac_method/inliers_threshold_weight", inliers_threshold_weight_, .25);
        _private_node_handle->param<std::string>("ransac_plane/axis", axis_, "z");
        _private_node_handle->param<int>("sac_method/segmented_index", segmented_index_, 0);
        _private_node_handle->param<int>("sac_method/method_type", sac_type_, pcl::SAC_RANSAC);
        _private_node_handle->param<int>("sac_method/model_type", model_type_,
                                         pcl::SACMODEL_NORMAL_PLANE); // SACMODEL_NORMAL_PLANE  SACMODEL_PLANE


    }

    void LocalPerception::setupPlaneIntersectionConfigurationFromParameterServer (ros::NodeHandlePtr &_node_handle,
                                                                        ros::NodeHandlePtr &_private_node_handle) {



        _private_node_handle->param<double>("plane_intersection/distance_to_line_threshold",
                                            distance_to_line_threshold_, 0.1);
        _private_node_handle->param<double>("input_cloud_timeout_threshold",
                                            input_cloud_timeout_threshold_, 5.0);

        _private_node_handle->param<int>("plane_intersection/index_cloud_plane_a", index_cloud_plane_a_, 0);
        _private_node_handle->param<int>("plane_intersection/index_cloud_plane_b", index_cloud_plane_b_, 1);

        _private_node_handle->param<std::string>("input_cloud_topic", input_cloud_topic_,
                                                 "/camera/depth_registered/points");

        _private_node_handle->param<std::string>("output_all_segmented_planes_cloud_topic",
                                                 output_segmented_cloud_merged_topic_, "/cloud_all_segmented_planes");
        _private_node_handle->param<std::string>("output_segmented_cloud_topic", output_segmented_cloud_topic_,
                                                 "/cloud_seg_reference");
        _private_node_handle->param<std::string>("output_cloud_filtered", output_input_cloud_filtered_,
                                                 "/cloud_filtered");
        _private_node_handle->param<std::string>("output_line_cloud", output_line_cloud_, "/output_line_cloud");
        _private_node_handle->param<std::string>("output_region_cloud", output_region_cloud_, "/output_region_cloud");
        _private_node_handle->param<std::string>("output_region_centroid_cloud", output_region_centroid_cloud_,
                                                 "/output_centroid_cloud");
        _private_node_handle->param<std::string>("output_welding_deposit_region_cloud", output_welding_deposit_region_cloud_,
                                                 "/welding_deposit_region_cloud");


        input_cloud_sub_ = private_node_handle_->subscribe(input_cloud_topic_, 1, &LocalPerception::CloudChatterCallback, this);

        pub_seg_merged_                  = private_node_handle_->advertise<pcl::PCLPointCloud2>(
                output_segmented_cloud_merged_topic_, 100, true);
        pub_seg_                         = private_node_handle_->advertise<pcl::PCLPointCloud2>(
                output_segmented_cloud_topic_, 100, true);
        pub_input_filtered_              = private_node_handle_->advertise<pcl::PCLPointCloud2>(
                output_input_cloud_filtered_, 100, true);
        pub_intersection_line_cloud_     = private_node_handle_->advertise<pcl::PCLPointCloud2>(output_line_cloud_, 100,
                                                                                                true);
        pub_intersection_region_cloud_   = private_node_handle_->advertise<pcl::PCLPointCloud2>(output_region_cloud_,
                                                                                                100, true);
        pub_intersection_centroid_cloud_ = private_node_handle_->advertise<pcl::PCLPointCloud2>(
                output_region_centroid_cloud_, 100, true);

        pub_welding_deposit_region_cloud_ = private_node_handle_->advertise<pcl::PCLPointCloud2>(
                output_welding_deposit_region_cloud_, 100, true);


    }

    void LocalPerception::setupWeldingQualityConfigurationFromParameterServer (ros::NodeHandlePtr &_node_handle,
                                                                                    ros::NodeHandlePtr &_private_node_handle) {

        //TODO:  to any future increment.
       // _private_node_handle->param<double>("quality/threshold", quality_threshold_, 0.15);

    }

    bool LocalPerception::inputPointCloudVerification()
    {
        float refresh_period = 0.5;
        int input_cloud_timeout_counter = 0;

        ROS_DEBUG_STREAM("Subscribe topic: " << input_cloud_topic_);

        unlockCloudCallbackMutex();

        while (ros::ok() && input_cloud_->points.size() == 0) {

            if(input_cloud_timeout_counter*refresh_period > input_cloud_timeout_threshold_){
                ROS_ERROR_STREAM(input_cloud_timeout_threshold_ << " [s] timout exceeded. ");
                aborted_msg_ += " " + std::to_string(ERROR_LIST::CLOUD_RECEPTION_TIMEOUT);
                lockCloudCallbackMutex();
                return false;
            }

            ROS_INFO_STREAM("Waiting for input cloud.");
            ros::Duration(refresh_period).sleep();
            //ros::spinOnce(); // Spin si already running in main node

            input_cloud_timeout_counter++;
        }

        lockCloudCallbackMutex();

        ROS_INFO_STREAM("Input cloud received.");

        if(!input_cloud_->is_dense) {
            ROS_WARN_STREAM("The input point cloud is NOT dense. Transforming to dense...");
            //boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*input_cloud_,*input_cloud_, indices);
            input_cloud_->is_dense = true;
            input_cloud_->height = 1;
            input_cloud_->width = input_cloud_->size();
        }
        return true;
    }

    bool LocalPerception::planeSegmentation (PointCloudPtrLP  _input,
                                   PointCloudPtrLP  &_segmented_cloud_merged,
                                   std::vector<PointCloudLP> &_output_arr,
                                   std::vector<pcl::ModelCoefficients> &_coefs_output_arr) {

        _output_arr.clear();
        _segmented_cloud_merged->clear();
        _coefs_output_arr.clear();

        PointCloudPtrLP  _input_cloud(new PointCloudLP);

        copyPointCloud(*_input, *_input_cloud);

        pcl::SACSegmentationFromNormals<pcl::PointXYZRGBNormal, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(model_type_);
        seg.setMethodType(sac_type_);
        seg.setDistanceThreshold(seg_thresehold_);
        seg.setNormalDistanceWeight(normal_dist_weight_);
        seg.setMaxIterations(max_iterations_);
        seg.setEpsAngle(eps_angle_ / 180.0 * M_PI); // used for angle deviation planes - in degrees

        if (axis_ == "x") // used for planes direction
            seg.setAxis(Eigen::Vector3f(1.0, 0.0, 0.0));
        else if (axis_ == "y")
            seg.setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));
        else if (axis_ == "xy" || axis_ == "yx")
            seg.setAxis(Eigen::Vector3f(1.0, 1.0, 0.0));
        else
            seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));

        pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr                          cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr           tree(
                new pcl::search::KdTree<pcl::PointXYZRGBNormal>());

        pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        pcl::ModelCoefficients coefficients;

        PointCloudPtrLP  seg_clouds(new PointCloudLP()),
                                                     full_seg_clouds(new PointCloudLP()),
                                                     complemented_seg_clouds(
                                                             new PointCloudLP());

        pcl::GlasbeyLUT colors;

        std::vector<pcl::PointCloud<pcl::PointXYZRGB> > vec_full_seg_clouds;

        u_int color_it = 0;
        int seg_cloud_num = 0, discarting_num = 0;
        int first_cloud_size = _input_cloud->size();
        int nr_points = (int) _input_cloud->points.size();
        int j                = 0;

        if (nr_points < 1) {
            ROS_ERROR_STREAM("Input cloud has size less than 1: " << nr_points);
            aborted_msg_ += " " + std::to_string(ERROR_LIST::PLANE_SEG_SMALL_INPUT_CLOUD_SIZE);
            return false;
        }

        while (ros::ok() && (_input_cloud->points.size() > left_rate_to_all_candidates_ * nr_points) ||
               ros::ok() && seg_cloud_num<max_number_of_planes_-1) {

            ROS_INFO_STREAM("Running SAC Method #" << sac_type_ << " iteration #" << seg_cloud_num);

            // Normals estimators
            ne.setSearchMethod(tree);
            ne.setInputCloud(_input_cloud);
            ne.setKSearch(normal_k_search_);
            ne.setRadiusSearch(normal_radius_search_);
            ne.compute(*cloud_normals);

            seg.setInputCloud(_input_cloud);
            seg.setInputNormals(cloud_normals);
            seg.segment(*inliers, coefficients);

            if (inliers->indices.size() == 0) {
                ROS_ERROR_STREAM("Could not estimate a model for the given dataset.");
                aborted_msg_ += " " + std::to_string(ERROR_LIST::PLANE_SEG_UNABLE_TO_ESTIMATE_A_FIT_MODEL);
                return false;
            }

            extract.setInputCloud(_input_cloud);
            extract.setIndices(inliers);
            extract.setNegative(false); // to extract the assimilated inlier
            extract.filter(*seg_clouds);
            seg_cloud_num++;

            if (discarting_num >= tolerance_iterator_threshold_) {
                ROS_WARN_STREAM("Finishing, not relevant cloud left");
                break;
            }

            if (inliers->indices.size() < inliers_threshold_weight_ * first_cloud_size) {
                ROS_WARN_STREAM("Not enough inliers (" << inliers->indices.size() << "). Discarding #" << seg_cloud_num
                                                       << " segmented cloud(s)");
                extract.setNegative(true); //to extract the complemented points
                extract.filter(*complemented_seg_clouds);
                _input_cloud.swap(
                        complemented_seg_clouds); // change full cloud to one without the first inlier detection
                discarting_num++;
                continue;
            }
            discarting_num = 0;

            if (color_it >= pcl::GlasbeyLUT::size()) {
                ROS_WARN_STREAM("Plane color out of range. Setting 0 back.");
                color_it = 0;
            }

            ROS_DEBUG_STREAM("Size of the #" << seg_cloud_num << " segmented cloud: " << seg_clouds->size());

            for (size_t i = 0; i < seg_clouds->size(); i++) {
                seg_clouds->points.at(i).r = colors.at(color_it).r;
                seg_clouds->points.at(i).g = colors.at(color_it).g;
                seg_clouds->points.at(i).b = colors.at(color_it).b;
            }

            color_it++;
            j++;

            //Concatenating all inlier cloud points associateds
            *full_seg_clouds = *full_seg_clouds + *seg_clouds;

            _output_arr.push_back(*seg_clouds);
            _coefs_output_arr.push_back(coefficients);


            extract.setNegative(true); //to extract the complemented points
            extract.filter(*complemented_seg_clouds);
            _input_cloud.swap(complemented_seg_clouds); // change full cloud to one without the first inlier detection

        }

        *_segmented_cloud_merged = *full_seg_clouds;
        _segmented_cloud_merged->header = _input_cloud->header;

        ROS_INFO("SAC method finished.");
        return true;
    }

    //Deprecated
    bool LocalPerception::cylinderSegmentation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _input_cloud,
                                                             pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & _segmented_cloud,
                                                             pcl::ModelCoefficients& _coefs){


        pcl::PassThrough<PointLP> pass;
        pcl::NormalEstimation<PointLP, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<PointLP, pcl::Normal> seg;
        pcl::ExtractIndices<PointLP> extract;
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        pcl::search::KdTree<PointLP>::Ptr tree (new pcl::search::KdTree<PointLP> ());

        // Datasets
        pcl::PointCloud<PointLP>::Ptr cloud_filtered (new pcl::PointCloud<PointLP>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<PointLP>::Ptr cloud_filtered2 (new pcl::PointCloud<PointLP>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);



/*
        // Build a passthrough filter to remove spurious NaNs and scene background
        pass.setInputCloud (_input_cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0, 1.5);
        pass.filter (*cloud_filtered);
        std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;
*/
        // Estimate point normals
        ne.setSearchMethod (tree);
        ne.setInputCloud (_input_cloud);
        ne.setKSearch (50);
        ne.compute (*cloud_normals);
/*
        // Create the segmentation object for the planar model and set all the parameters
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight (0.1);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.03);
        seg.setInputCloud (cloud_filtered);
        seg.setInputNormals (cloud_normals);
        // Obtain the plane inliers and coefficients
        seg.segment (*inliers_plane, *coefficients_plane);
        std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

        // Extract the planar inliers from the input cloud
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers_plane);
        extract.setNegative (false);

        // Write the planar inliers to disk
        pcl::PointCloud<PointLP>::Ptr cloud_plane (new pcl::PointCloud<PointLP> ());
        extract.filter (*cloud_plane);
        std::cerr << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_filtered2);
        extract_normals.setNegative (true);
        extract_normals.setInputCloud (cloud_normals);
        extract_normals.setIndices (inliers_plane);
        extract_normals.filter (*cloud_normals2);
*/
        // Create the segmentation object for cylinder segmentation and set all the parameters
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_CYLINDER);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight (0.1);
        seg.setMaxIterations (10000);
        seg.setDistanceThreshold (0.01);
        //seg.setRadiusLimits (0, quality_threshold_);
        seg.setInputCloud (_input_cloud);
        seg.setInputNormals (cloud_normals);

        // Obtain the cylinder inliers and coefficients
        seg.segment (*inliers_cylinder, *coefficients_cylinder);
        std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

        // Write the cylinder inliers to disk
        extract.setInputCloud (_input_cloud);
        extract.setIndices (inliers_cylinder);
        extract.setNegative (false);

        extract.filter (*_segmented_cloud);
        if (_segmented_cloud->points.empty ()) {
            std::cerr << "Can't find the cylindrical component." << std::endl;
            return false;
        }
        else
        {
            std::cerr << "PointCloud representing the cylindrical component: " << _segmented_cloud->size () << " data points." << std::endl;
            return true;
        }
    }

    void LocalPerception::CloudChatterCallback (const boost::shared_ptr<const sensor_msgs::PointCloud2> &input) {

        if(!isInputCloudCallbackMutexLocked()){
            //cloud_ros_=*input;
            pcl_conversions::toPCL(*input, *cloud_raw);
            pcl::fromPCLPointCloud2(*cloud_raw, *input_cloud_);
        }

        //ROS_WARN_STREAM("Mutex is locked? : " << isInputCloudCallbackMutexLocked() );

    }


    // Comparator function to sort pairs
    // according to second value
    bool cmp (std::pair<int, double> &a, std::pair<int, double> &b) {
        return a.second < b.second;
    }

    bool LocalPerception::runWeldingEstimationPipeline (local_perception_msgs::LocalPerceptionGoalConstPtr _goal) {

        merged_goals.acquisition_distance = _goal->acquisition_distance;
        merged_goals.edge_tolerance = _goal->edge_tolerance;

        if(!this->runBasicPipeline() ||
           !this->applyCorrection(_goal->offset_compensation, welding_poses_arr_))
            return false;

        local_perception_msgs::PointArr welding_pose;
        welding_pose.welding_line_index = 0;
        welding_pose.welding_point_stamped = welding_poses_arr_;
        result_.welding_list.welding.push_back(welding_pose);

        if(verbosity_levels::isROSDebug())
            runDebugTools();

        return true;

    }

    bool LocalPerception::runQualityPipeline(local_perception_msgs::LocalQualityPerceptionGoalConstPtr _goal){

        merged_goals.acquisition_distance = _goal->acquisition_distance;
        merged_goals.deposit_radius_tolerance = _goal->deposit_radius_tolerance;
        merged_goals.output_path = _goal->output_path;

        if(!this->runBasicPipeline() ||
           !this->runQualityEvaluation())
            return false;

        if(verbosity_levels::isROSDebug())
            runDebugTools();

        return true;
    }

    bool LocalPerception::runBasicPipeline(){

        std::vector<pcl::ModelCoefficients> coefs_output_arr;

        std::vector<Eigen::Vector3d> projected_point_arr;

        if(!inputPointCloudVerification())
            return false;

        input_cloud_bkup_->clear();
        pcl::copyPointCloud<PointLP>(*input_cloud_, *input_cloud_bkup_);

        if (cropbox_activation_) {
            BoxParametrization cropbox_parameters;
            if (!setupCropboxParameters(merged_goals.acquisition_distance, cropbox_parameters) ||
                !local_perception_server::pcl_complement::applyCropbox(input_cloud_, cropbox_parameters.min_vertex_arr,cropbox_parameters.max_vertex_arr)) {
                aborted_msg_ += " " + std::to_string(ERROR_LIST::INPUT_CLOUD_CROPBOX_ERROR);
                return false;
            }
        }

        post_cropbox_input_cloud_bkup_->clear();
        pcl::copyPointCloud<PointLP>(*input_cloud_, *post_cropbox_input_cloud_bkup_);

        if(!local_perception_server::pcl_complement::applyVoxelGrid(input_cloud_, voxel_leaf_sizes_arr_)){
            aborted_msg_ += " " + std::to_string(ERROR_LIST::INPUT_CLOUD_VOXELGRID_ERROR);
            return false;
        }

        local_perception_server::pcl_complement::pubCloud(pub_input_filtered_, input_cloud_);

        if(!this->planeSegmentation(input_cloud_, all_segmented_planes_cloud_, all_segmented_planes_arr_, coefs_output_arr))
            return false;

        local_perception_server::pcl_complement::pubCloud(pub_seg_merged_, all_segmented_planes_cloud_);
        ROS_INFO_STREAM(" # " << all_segmented_planes_arr_.size() << " planes had been found.");

        if(all_segmented_planes_arr_.size()<2){
            ROS_ERROR_STREAM("Not enough planes were found.");
            aborted_msg_ += " " + std::to_string(ERROR_LIST::SMALL_NUMBER_OF_PLANES_FOUND);
            return false;
        }

        // TODO: adapt it to generate all intersections

        *plane_cloud_a_ = all_segmented_planes_arr_.at(index_cloud_plane_a_);
        *plane_cloud_b_ = all_segmented_planes_arr_.at(index_cloud_plane_b_);
        *plane_cloud_ab_ = *plane_cloud_a_ + *plane_cloud_b_;

        local_perception_server::pcl_complement::pubCloud(pub_seg_, plane_cloud_ab_);

        if(!this->generateIntersectionLine(coefs_output_arr.at(index_cloud_plane_a_),coefs_output_arr.at(index_cloud_plane_b_), estimated_welding_line_cloud_,estimated_welding_line_)
           || !this->calculateIntersectionRegion(plane_cloud_ab_, estimated_welding_line_, distance_to_line_threshold_, intersection_region_cloud_)
           || !this->projectPointCloudToLine(intersection_region_cloud_, estimated_welding_line_, projected_point_arr)
           || !this->generateWeldingPoses (projected_point_arr, estimated_welding_line_, intersection_region_cloud_->header.frame_id, welding_poses_arr_)
                )
            return false;

        return true;
    }

    bool LocalPerception::runDebugTools(){

        for (size_t i = 0; i < welding_poses_arr_.size(); ++i) {
            static_broadcaster_.sendTransform(welding_poses_arr_.at(i));
        }

        geometry_msgs::TransformStamped pRefTf;
        pRefTf.header.frame_id         = intersection_region_cloud_->header.frame_id;
        pRefTf.header.stamp            = ros::Time::now();
        pRefTf.child_frame_id          = "welding_pose_reference";
        pRefTf.transform.translation.x = estimated_welding_line_.reference_point[0] ;
        pRefTf.transform.translation.y = estimated_welding_line_.reference_point[1] ;
        pRefTf.transform.translation.z = estimated_welding_line_.reference_point[2];
        pRefTf.transform.rotation.x    = 0;
        pRefTf.transform.rotation.y    = 0;
        pRefTf.transform.rotation.z    = 0;
        pRefTf.transform.rotation.w    = 1;
        static_broadcaster_.sendTransform(pRefTf);

        local_perception_server::pcl_complement::pubCloud(pub_intersection_line_cloud_, estimated_welding_line_cloud_);
        local_perception_server::pcl_complement::pubCloud(pub_intersection_region_cloud_,intersection_region_cloud_);

        return true;
    }

    bool LocalPerception::runQualityEvaluation()
    {

        this->calculateIntersectionRegion(post_cropbox_input_cloud_bkup_,
                                          estimated_welding_line_,
                                          merged_goals.deposit_radius_tolerance,
                                          welding_deposit_cloud_wrt_sensor_);


        ROS_WARN_STREAM("Total number of point in welding deposit: " << welding_deposit_cloud_wrt_sensor_->size());

        buildWeldingLineFrame();

        pcl::transformPointCloud(*welding_deposit_cloud_wrt_sensor_,*welding_deposit_cloud_wrt_line_axis_, transformation_aligned_line_frame_wrt_sensor_.inverse(), true);

        welding_deposit_cloud_wrt_line_axis_->header.frame_id = "welding_pose_reference_aligned";

        local_perception_server::pcl_complement::pubCloud(pub_welding_deposit_region_cloud_, welding_deposit_cloud_wrt_line_axis_);

        exportQualityResult();

        return true;
    }

    bool LocalPerception::getAlignment(Eigen::Matrix4d &T){

        Eigen::VectorXf parent_line_coefs (6);

        logStreamStr_.clear();
        if(cropbox_frame_id_norm_ == CAMERA_FRAME_NORM_TYPE::ROS){
            logStreamStr_  << "Frame Convention," << "ROS\n";
            parent_line_coefs << 0,0,0,0,1,0; // aligning the Y-axis since it is the baseline in ROS convention.
        }

        else if (cropbox_frame_id_norm_ == CAMERA_FRAME_NORM_TYPE::OPTICAL) {
            logStreamStr_  << "Frame Convention," << "Optical\n";
            parent_line_coefs << 0,0,0,1,0,0; // aligning the X-axis since it is the baseline OPTICAL convention.
        }

        PointCloudPtrLP     parent_line_cloud      (new PointCloudLP);
        PointCloudPtrLP     parent_aligned_line_cloud      (new PointCloudLP);


        createLineCloud(parent_line_coefs,parent_line_cloud);
        parent_line_cloud->header = estimated_welding_line_cloud_->header;

        if(!pcl_complement::applyICP(parent_line_cloud,estimated_welding_line_cloud_,parent_aligned_line_cloud,T)){
            ROS_ERROR("ICP did not converge");
            aborted_msg_ += " " + std::to_string(ERROR_LIST::ICP_NOT_CONVERGED);
            return false;
        }

        return true;

    }

    bool LocalPerception::buildWeldingLineFrame(){

        Eigen::Matrix4d T;
        getAlignment(T);

        transformation_aligned_line_frame_wrt_sensor_ = T;

        Eigen::Quaterniond q(transformation_aligned_line_frame_wrt_sensor_.rotation());


        t_aligned_ref_wrt_sensor_.header.frame_id         = estimated_welding_line_cloud_->header.frame_id;
        t_aligned_ref_wrt_sensor_.header.stamp            = ros::Time::now();
        t_aligned_ref_wrt_sensor_.child_frame_id          = "welding_pose_reference_aligned";
        t_aligned_ref_wrt_sensor_.transform.translation.x = transformation_aligned_line_frame_wrt_sensor_.translation().x();
        t_aligned_ref_wrt_sensor_.transform.translation.y = transformation_aligned_line_frame_wrt_sensor_.translation().y();
        t_aligned_ref_wrt_sensor_.transform.translation.z = transformation_aligned_line_frame_wrt_sensor_.translation().z();
        t_aligned_ref_wrt_sensor_.transform.rotation.x    = q.x();
        t_aligned_ref_wrt_sensor_.transform.rotation.y    = q.y();
        t_aligned_ref_wrt_sensor_.transform.rotation.z    = q.z();
        t_aligned_ref_wrt_sensor_.transform.rotation.w    = q.w();

        static_broadcaster_.sendTransform(t_aligned_ref_wrt_sensor_);

        return true;
    }

    //deprecated, need adjusts
    bool LocalPerception::buildWeldingLineFrame(geometry_msgs::TransformStamped _custom_position){

        Eigen::Matrix4d T;
        getAlignment(T);

        transformation_aligned_line_frame_wrt_sensor_ = T;

        Eigen::Quaterniond q(transformation_aligned_line_frame_wrt_sensor_.rotation());

        geometry_msgs::TransformStamped toPub;

        toPub.header.frame_id         = estimated_welding_line_cloud_->header.frame_id;
        toPub.header.stamp            = ros::Time::now();
        toPub.child_frame_id          = "welding_pose_reference_aligned";
        toPub.transform.translation.x = _custom_position.transform.translation.x ;
        toPub.transform.translation.y = _custom_position.transform.translation.y ;
        toPub.transform.translation.z = _custom_position.transform.translation.z;
        toPub.transform.rotation.x    = q.x();
        toPub.transform.rotation.y    = q.y();
        toPub.transform.rotation.z    = q.z();
        toPub.transform.rotation.w    = q.w();

        static_broadcaster_.sendTransform(toPub);

        return true;
    }

    bool LocalPerception::exportQualityResult(){

        std::vector<double> each_point_x_distance_to_welding_line,
                            each_point_y_distance_to_welding_line,
                            each_point_z_distance_to_welding_line,
                            each_point_distance_to_welding_line;

        local_perception_msgs::QualityArr q_arr;

        distFromLine(welding_deposit_cloud_wrt_sensor_,estimated_welding_line_, each_point_distance_to_welding_line);

        logStreamStrPC_.clear();
        logStreamStrPC_  << "Welding Deposit Cloud.\n";
        logStreamStrPC_  << "X,Y,Z," "\n";

        for (size_t i = 0; i < welding_deposit_cloud_wrt_line_axis_->size(); ++i) {

            PointLP& point = welding_deposit_cloud_wrt_line_axis_->at(i);

            each_point_x_distance_to_welding_line.push_back(point.x);
            each_point_y_distance_to_welding_line.push_back(point.y);
            each_point_z_distance_to_welding_line.push_back(point.z);

            logStreamStrPC_ << point.x << "," << point.y << "," << point.z << "\n";

        }

        logStreamStr_  << "Welding Deposit Cloud Parent Frame," << t_aligned_ref_wrt_sensor_.child_frame_id <<"\n";
        logStreamStr_  << "Welding Deposit Cloud Parent Frame Definition\n";
        logStreamStr_  << "Parent Frame," << t_aligned_ref_wrt_sensor_.header.frame_id <<"\n";
        logStreamStr_  << "Position [x | y | z][meters]," << t_aligned_ref_wrt_sensor_.transform.translation.x <<
                       "," << t_aligned_ref_wrt_sensor_.transform.translation.y <<
                       "," << t_aligned_ref_wrt_sensor_.transform.translation.z << "\n";

        logStreamStr_  << "Rotation [qx | qy | qz | qw]," << t_aligned_ref_wrt_sensor_.transform.rotation.x <<
                       "," << t_aligned_ref_wrt_sensor_.transform.rotation.y <<
                       "," << t_aligned_ref_wrt_sensor_.transform.rotation.z <<
                       "," << t_aligned_ref_wrt_sensor_.transform.rotation.w << "\n";

        logStreamStr_ << "Quality Params\n";

        if(cropbox_frame_id_norm_ == CAMERA_FRAME_NORM_TYPE::ROS)
        {
            q_arr.quality_list.clear();
            buildResult(each_point_distance_to_welding_line,"distance", q_arr);
            buildResult(each_point_x_distance_to_welding_line,"x", q_arr);
            buildResult(each_point_y_distance_to_welding_line,"y", q_arr);

        }

        else if (cropbox_frame_id_norm_ == CAMERA_FRAME_NORM_TYPE::OPTICAL) {

            q_arr.quality_list.clear();
            buildResult(each_point_distance_to_welding_line,"distance", q_arr);
            buildResult(each_point_y_distance_to_welding_line,"y", q_arr);
            buildResult(each_point_z_distance_to_welding_line,"z", q_arr);

        }

        quality_result_.output.quality_list.clear();
        quality_result_.output.quality_list = q_arr.quality_list;
        quality_result_.output.quality_reference_frame_definition = t_aligned_ref_wrt_sensor_;





        exportCSVFile();

        return true;
    }

    bool LocalPerception::buildResult(std::vector<double> _input_arr, std::string _post_prefix, local_perception_msgs::QualityArr &_result_arr){


        double std = StdVectorOperation::getStdDeviationValue(_input_arr);
        double avg = StdVectorOperation::getAverageValue(_input_arr);
        double min = StdVectorOperation::getMinimumValue(_input_arr);
        double max = StdVectorOperation::getMaximumValue(_input_arr);

        std::vector<std::string> str_arr;
        std::vector<double> val_arr;
        local_perception_msgs::QualityMsg q;

        val_arr.clear();
        str_arr.clear();

        val_arr.insert(val_arr.end(), {avg, std, min, max});
        str_arr =  {"avg_"+_post_prefix, "std_"+_post_prefix, "min_"+_post_prefix, "max_"+_post_prefix} ;

        for (size_t i = 0; i < str_arr.size() ; ++i) {
            q.param_name = str_arr.at(i);
            q.param_value = val_arr.at(i);
            _result_arr.quality_list.push_back(q);
            ROS_DEBUG_STREAM(q.param_name << " = " << q.param_value);
            logStreamStr_ << q.param_name << "," << q.param_value <<"\n";
        }

        return true;

    }

    bool LocalPerception::exportCSVFile(){

        std::ofstream file1, file2;
        ros::Time     t = ros::Time().now();

        std::string file_path_and_name_info = merged_goals.output_path+"/" + std::to_string(t.sec) + "_info.csv",
                    file_path_and_name_pc = merged_goals.output_path+"/" + std::to_string(t.sec) + "_point_cloud.csv";

        file1.open(file_path_and_name_info);
        file2.open(file_path_and_name_pc);

        if (file1.fail()) {
            char *homepath = getenv("HOME");
            std::string homepath_s(homepath);
            ROS_ERROR_STREAM("Failed to save the file: " << file_path_and_name_info << ". Saving log files to the home dir.");
            file1.open(homepath_s + +"/" + std::to_string(t.sec) + "_info.csv");
            file2.open(homepath_s + +"/" + std::to_string(t.sec) + "_point_cloud.csv");
        }

        file1 << logStreamStr_.str();
        file2 << logStreamStrPC_.str();

        file1.close();
        file2.close();

        return true;

    }

    bool LocalPerception::generateIntersectionLine (pcl::ModelCoefficients coefs_plane_a,
                                          pcl::ModelCoefficients coefs_plane_b,
                                          PointCloudPtrLP  &line_cloud,
                                          InfiniteLineDescriptors &line)
    {
        Eigen::Vector4f plane_coef_a, plane_coef_b;
        Eigen::VectorXf coefs_line;

        plane_coef_a << coefs_plane_a.values.at(0), coefs_plane_a.values.at(1), coefs_plane_a.values.at(
                2), coefs_plane_a.values.at(3);
        plane_coef_b << coefs_plane_b.values.at(0), coefs_plane_b.values.at(1), coefs_plane_b.values.at(
                2), coefs_plane_b.values.at(3);

        if (!pcl::planeWithPlaneIntersection(plane_coef_a, plane_coef_b, coefs_line, 0.1)) {
            ROS_ERROR("Error in line equation estimation. Be sure that both planes are not parallel.");
            aborted_msg_ += " " + std::to_string(ERROR_LIST::LINE_ESTIMATION_PLANE_PARALLEL_ERROR);
            return false;
        }

        line.reference_point <<  coefs_line[0],coefs_line[1], coefs_line[2];
        line.direction_vector << coefs_line[3], coefs_line[4], coefs_line[5];

        if (!createLineCloud(coefs_line, line_cloud)) //Be aware to setup the header
        {
            ROS_ERROR("Error while generating line cloud");
            return false;
        }

        line_cloud->header = coefs_plane_a.header;
        line_cloud->height = 1;
        line_cloud->width  = line_cloud->size();

        return true;
    }

    bool LocalPerception::calculateIntersectionRegion(PointCloudPtrLP _input_cloud, InfiniteLineDescriptors _line, double _dist_threshold, PointCloudPtrLP &intersection_region_cloud ){

        intersection_region_cloud->clear();

        Eigen::Vector3d pp;

        double d1, d2, dist, min_v = 1e18, max_v = -1e18;

        for (size_t it = 0; it < _input_cloud->points.size(); ++it) {

            Eigen::Vector3d current_p(_input_cloud->points.at(it).x, _input_cloud->points.at(it).y,
                                      _input_cloud->points.at(it).z);
            pp           = _line.reference_point - current_p;
            d1           = pp.cross(_line.direction_vector).norm();
            d2           = _line.direction_vector.norm(),
                    dist = d1 / d2;

            if (dist < min_v)
                min_v = dist;
            if (dist > max_v)
                max_v = dist;

            if (dist < _dist_threshold)
                intersection_region_cloud->push_back(_input_cloud->points.at(it));
        }

        intersection_region_cloud->header = input_cloud_->header;
        intersection_region_cloud->height = 1;
        intersection_region_cloud->width  = intersection_region_cloud->size();

        ROS_DEBUG_STREAM("Line reference point: [" << _line.reference_point.x() << ", "<< _line.reference_point.y() << ", " << _line.reference_point.z() << "]" );
        ROS_DEBUG_STREAM("Direction vector: [" << _line.direction_vector.x() << ", "<< _line.direction_vector.y() << ", " << _line.direction_vector.z() << "]" );
        ROS_DEBUG_STREAM("Maximum distance from line:" << max_v);
        ROS_DEBUG_STREAM("Minimum distance from line:" << min_v);
        ROS_DEBUG_STREAM("Intersection region size: " << intersection_region_cloud->points.size());


        return true;
    }

    bool LocalPerception::projectPointCloudToLine(PointCloudPtrLP _input_cloud, InfiniteLineDescriptors _line, std::vector<Eigen::Vector3d> &_projected_point_arr){

        _projected_point_arr.clear();

        for (size_t it = 0; it < _input_cloud->points.size(); ++it) {

            pcl::PointXYZRGBNormal point;

            Eigen::Vector3d current_p(_input_cloud->points.at(it).x,
                                      _input_cloud->points.at(it).y,
                                      _input_cloud->points.at(it).z),
                            ap, projected_point;

            ap = current_p - _line.reference_point;
            projected_point =
                    (ap.dot(_line.direction_vector) / _line.direction_vector.dot(_line.direction_vector) * _line.direction_vector) +
                            _line.reference_point;

            _projected_point_arr.push_back(projected_point);
        }
        return true;
    }

    bool LocalPerception::distFromLine(PointCloudPtrLP _input_cloud, InfiniteLineDescriptors _line, std::vector<double> &_distance_arr){

        _distance_arr.clear();

        for (size_t it = 0; it < _input_cloud->points.size(); ++it) {

            pcl::PointXYZRGBNormal point;

            Eigen::Vector3d current_p(_input_cloud->points.at(it).x,
                                      _input_cloud->points.at(it).y,
                                      _input_cloud->points.at(it).z),
                    ap, projected_point;

            ap = current_p - _line.reference_point;
            projected_point =
                    (ap.dot(_line.direction_vector) / _line.direction_vector.dot(_line.direction_vector) * _line.direction_vector) +
                    _line.reference_point;

            ap = current_p - projected_point;

            _distance_arr.push_back(ap.norm());
        }
        return true;
    }


    bool LocalPerception::generateWeldingPoses (std::vector<Eigen::Vector3d> _projected_point_arr, InfiniteLineDescriptors _line,
                                                std::string _parent_frame_id, std::vector<geometry_msgs::TransformStamped> &_poses_arr) {

        Eigen::Vector3d pp;

        _poses_arr.clear();

        std::vector<std::pair<int, double>> dist_arr;
        std::pair<int, double>              aux;

        for (size_t it = 0; it < _projected_point_arr.size(); ++it) {

            pp = _projected_point_arr.at(it) - _line.reference_point;
            aux.first = it;

            if (pp.dot(_line.direction_vector) < 0)
                aux.second = -pp.norm();
            else
                aux.second = pp.norm();

            dist_arr.push_back(aux);
        }

        std::sort(dist_arr.begin(), dist_arr.end(), cmp);

        Eigen::Vector3d initWeldingPose, lastWeldingPose, welding_length_vector;
        initWeldingPose = _projected_point_arr.at(dist_arr.at(0).first);
        lastWeldingPose = _projected_point_arr.at(dist_arr.at(dist_arr.size() - 1).first);

        welding_length_vector = lastWeldingPose - initWeldingPose;

        double welding_length = welding_length_vector.norm();

        geometry_msgs::TransformStamped pose_to_weld;
        pose_to_weld.header.frame_id = _parent_frame_id;
        pose_to_weld.header.stamp    = ros::Time::now();

        //pcl::PointXYZRGBNormal point;

        for (int it = 0; it < welding_interval_ + 1; ++it) {

            pose_to_weld.child_frame_id = "welding_pose_" + std::to_string(it);

            pose_to_weld.transform.translation.x =
                    initWeldingPose[0] + it * welding_length / welding_interval_ * _line.direction_vector[0];

            pose_to_weld.transform.translation.y =
                    initWeldingPose[1] + it * welding_length / welding_interval_ * _line.direction_vector[1];

            pose_to_weld.transform.translation.z =
                    initWeldingPose[2] + it * welding_length / welding_interval_ * _line.direction_vector[2];

            pose_to_weld.transform.rotation.w = 1.0; //TODO: set custom orientation.

            //static_broadcaster_.sendTransform(pose_to_weld);

            _poses_arr.push_back(pose_to_weld);

            //point.x = pose_to_weld.transform.translation.x;
            //point.y = pose_to_weld.transform.translation.y;
            //point.z = pose_to_weld.transform.translation.z;

            //robot_poses_cloud->push_back(point);

        }

        //robot_poses_cloud->header = intersection_region_cloud->header;
        //robot_poses_cloud->width  = robot_poses_cloud->size();
        //robot_poses_cloud->height = 1;

        return true;
    }

    bool LocalPerception::createLineCloud (
            Eigen::VectorXf &coefs_line, //first 3 parameters- initial point nearest to zero. last 3 parameters = direction vector
            PointCloudPtrLP  &line_cloud) {

        pcl::PointXYZRGB p1, p2, delta;
        double           k          = 10.0;
        double           scale_line = 10000;

        p1.x = coefs_line[0] - k * coefs_line[3];
        p1.y = coefs_line[1] - k * coefs_line[4];
        p1.z = coefs_line[2] - k * coefs_line[5];
        p2.x = coefs_line[0] + k * coefs_line[3];
        p2.y = coefs_line[1] + k * coefs_line[4];
        p2.z = coefs_line[2] + k * coefs_line[5];

        delta.x = (p2.x - p1.x) / scale_line;
        delta.y = (p2.y - p1.y) / scale_line;
        delta.z = (p2.z - p1.z) / scale_line;

        line_cloud->clear();
        for (int i = 0; i < scale_line; ++i) {
            pcl::PointXYZRGBNormal point;
            point.x = p1.x + i * delta.x;
            point.y = p1.y + i * delta.y;
            point.z = p1.z + i * delta.z;
            line_cloud->points.push_back(point);

        }

        return true;
    }

    bool LocalPerception::setupCropboxParameters(double _linear_distance, BoxParametrization &_cropbox_parameters){

        _cropbox_parameters.max_vertex_arr.clear();
        _cropbox_parameters.min_vertex_arr.clear();

        double vfov_rad = angles::from_degrees (vfov_),
               hfov_rad = angles::from_degrees (hfov_),
               h_projection, v_projection;

        h_projection = _linear_distance * atan(hfov_rad * .5);
        v_projection = _linear_distance * atan(vfov_rad * .5);

        if(cropbox_frame_id_norm_ == CAMERA_FRAME_NORM_TYPE::ROS)
        {
            _cropbox_parameters.max_vertex_arr.push_back(_linear_distance+distance_range_);
            _cropbox_parameters.max_vertex_arr.push_back(h_projection);
            _cropbox_parameters.max_vertex_arr.push_back(v_projection);

            _cropbox_parameters.min_vertex_arr.push_back(_linear_distance-distance_range_);
            _cropbox_parameters.min_vertex_arr.push_back(-h_projection);
            _cropbox_parameters.min_vertex_arr.push_back(-v_projection);
        }

        else if (cropbox_frame_id_norm_ == CAMERA_FRAME_NORM_TYPE::OPTICAL)
        {

            _cropbox_parameters.max_vertex_arr.push_back(h_projection);
            _cropbox_parameters.max_vertex_arr.push_back(v_projection);
            _cropbox_parameters.max_vertex_arr.push_back(_linear_distance+distance_range_);

            _cropbox_parameters.min_vertex_arr.push_back(-h_projection);
            _cropbox_parameters.min_vertex_arr.push_back(-v_projection);
            _cropbox_parameters.min_vertex_arr.push_back(_linear_distance-distance_range_);

        }

        ROS_DEBUG_STREAM("Cropbox maximum parameters defined: [" << _cropbox_parameters.max_vertex_arr.at(0) <<" ," << _cropbox_parameters.max_vertex_arr.at(1) <<" ," << _cropbox_parameters.max_vertex_arr.at(2) << "]");
        ROS_DEBUG_STREAM("Cropbox minimum parameters defined: [" << _cropbox_parameters.min_vertex_arr.at(0) <<" ," << _cropbox_parameters.min_vertex_arr.at(1) <<" ," << _cropbox_parameters.min_vertex_arr.at(2) << "]");

        return true;
    }

    bool LocalPerception::applyCorrection(const std::vector<float> offset, std::vector<geometry_msgs::TransformStamped> &_poses_arr ){

        if(offset.empty()){
            ROS_WARN("No offset vector defined. Not correcting. ");
            return true;
        }
        else if (offset.size() != 3){
            ROS_WARN("Offset should be a three-dimensional float vector. Not correcting. ");
            return true;
        }

        std::vector<geometry_msgs::TransformStamped>::iterator it  = _poses_arr.begin();

        while(it != _poses_arr.end() && ros::ok())
        {
            it->transform.translation.x  = it->transform.translation.x + offset.at(0);
            it->transform.translation.y  = it->transform.translation.y + offset.at(1);
            it->transform.translation.z  = it->transform.translation.z + offset.at(2);
            it++;
        }

        ROS_DEBUG_STREAM("Offset of [" << offset.at(0) << ", "<< offset.at(1) << ", " << offset.at(2)  << "] applied");
        return true;

    }

    bool LocalPerception::isInputCloudCallbackMutexLocked(){
        return lock_input_pc_callback_mutex_;
    }

    void LocalPerception::lockCloudCallbackMutex(){

        lock_input_pc_callback_mutex_ = true;
    }

    void LocalPerception::unlockCloudCallbackMutex(){

        lock_input_pc_callback_mutex_ = false;
    }
}
