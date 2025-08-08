/**
 * @file monocular-slam-node.cpp
 * @brief Implementation of the MonocularSlamNode Wrapper class.
 * @author Based on RgbdSlamNode by Suchetan R S (rssuchetan@gmail.com)
 */
#include "monocular-slam-node.hpp"

#include <opencv2/core/core.hpp>

namespace ORB_SLAM3_Wrapper
{
    MonocularSlamNode::MonocularSlamNode(const std::string &strVocFile,
                                       const std::string &strSettingsFile,
                                       ORB_SLAM3::System::eSensor sensor)
        : Node("ORB_SLAM3_MONO_ROS2")
    {
        // Declare parameters (topic names)
        this->declare_parameter("image_topic_name", rclcpp::ParameterValue("camera/image_raw"));
        this->declare_parameter("imu_topic_name", rclcpp::ParameterValue("imu"));
        this->declare_parameter("odom_topic_name", rclcpp::ParameterValue("odom"));

        // ROS Subscribers
        imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
            this->get_parameter("image_topic_name").as_string(), 10, 
            std::bind(&MonocularSlamNode::ImageCallback, this, std::placeholders::_1));

        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            this->get_parameter("imu_topic_name").as_string(), 1000, 
            std::bind(&MonocularSlamNode::ImuCallback, this, std::placeholders::_1));
        
        odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_topic_name").as_string(), 1000, 
            std::bind(&MonocularSlamNode::OdomCallback, this, std::placeholders::_1));

        // ROS Publishers
        //---- the following is published when a service is called
        mapPointsPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);
        visibleLandmarksPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visible_landmarks", 10);
        visibleLandmarksPose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("visible_landmarks_pose", 10);
        slamInfoPub_ = this->create_publisher<slam_msgs::msg::SlamInfo>("slam_info", 10);
        //---- the following is published continously
        mapDataPub_ = this->create_publisher<slam_msgs::msg::MapData>("map_data", 10);
        robotPoseMapFrame_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose_slam", 10);

        // ROS Services
        mapDataCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        mapPointsCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        pointsInViewCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        getMapDataService_ = this->create_service<slam_msgs::srv::GetMap>("get_map_data", 
            std::bind(&MonocularSlamNode::getMapDataServer, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
            rmw_qos_profile_services_default, mapDataCallbackGroup_);
        
        mapPointsService_ = this->create_service<slam_msgs::srv::GetAllLandmarksInMap>("get_all_map_points", 
            std::bind(&MonocularSlamNode::getAllMapPointsServer, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
            rmw_qos_profile_services_default, mapPointsCallbackGroup_);
        
        getMapPointsService_ = this->create_service<slam_msgs::srv::GetLandmarksInView>("get_map_points_in_view", 
            std::bind(&MonocularSlamNode::getMapPointsInViewServer, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
            rmw_qos_profile_services_default, pointsInViewCallbackGroup_);
        
        resetLocalMapSrv_ = this->create_service<std_srvs::srv::SetBool>("reset_local_map", 
            std::bind(&MonocularSlamNode::resetLocalMapServer, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // TF
        tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        bool bUseViewer;
        this->declare_parameter("visualization", rclcpp::ParameterValue(true));
        this->get_parameter("visualization", bUseViewer);

        this->declare_parameter("robot_base_frame", "base_footprint");
        this->get_parameter("robot_base_frame", robot_base_frame_id_);

        this->declare_parameter("global_frame", "map");
        this->get_parameter("global_frame", global_frame_);

        this->declare_parameter("odom_frame", "odom");
        this->get_parameter("odom_frame", odom_frame_id_);

        this->declare_parameter("publish_tf", rclcpp::ParameterValue(true));
        this->get_parameter("publish_tf", publish_tf_);

        this->declare_parameter("odometry_mode", rclcpp::ParameterValue(false));
        this->get_parameter("odometry_mode", odometry_mode_);

        this->declare_parameter("do_loop_closing", rclcpp::ParameterValue(true));
        this->get_parameter("do_loop_closing", do_loop_closing_);

        this->declare_parameter("map_data_publish_frequency", rclcpp::ParameterValue(10));
        this->get_parameter("map_data_publish_frequency", map_data_publish_frequency_);

        // Robot initial pose
        this->declare_parameter("robot_x", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_x", robot_x_);
        this->declare_parameter("robot_y", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_y", robot_y_);
        this->declare_parameter("robot_z", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_z", robot_z_);
        this->declare_parameter("robot_qx", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_qx", robot_qx_);
        this->declare_parameter("robot_qy", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_qy", robot_qy_);
        this->declare_parameter("robot_qz", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_qz", robot_qz_);
        this->declare_parameter("robot_qw", rclcpp::ParameterValue(1.0));
        this->get_parameter("robot_qw", robot_qw_);

        initial_pose.position.x = robot_x_;
        initial_pose.position.y = robot_y_;
        initial_pose.position.z = robot_z_;
        initial_pose.orientation.x = robot_qx_;
        initial_pose.orientation.y = robot_qy_;
        initial_pose.orientation.z = robot_qz_;
        initial_pose.orientation.w = robot_qw_;

        // ROS Timer
        mapDataTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / map_data_publish_frequency_),
            std::bind(&MonocularSlamNode::publishMapData, this),
            mapDataCallbackGroup_);

        interface_ = std::make_shared<ORB_SLAM3_Wrapper::ORBSLAM3Interface>(strVocFile, strSettingsFile,
                                                                            sensor, bUseViewer, do_loop_closing_, initial_pose, global_frame_, odom_frame_id_, robot_base_frame_id_);

        frequency_tracker_count_ = 0;
        frequency_tracker_clock_ = std::chrono::high_resolution_clock::now();

        RCLCPP_INFO(this->get_logger(), "CONSTRUCTOR END!");
    }

    MonocularSlamNode::~MonocularSlamNode()
    {
        imageSub_.reset();
        imuSub_.reset();
        odomSub_.reset();
        interface_.reset();
        RCLCPP_INFO(this->get_logger(), "DESTRUCTOR!");
    }

    void MonocularSlamNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgIMU)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "ImuCallback");
        // push value to imu buffer.
        interface_->handleIMU(msgIMU);
    }

    void MonocularSlamNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msgOdom)
    {
        if (odometry_mode_)
        {   // populate map to odom tf if odometry is being used
            RCLCPP_DEBUG_STREAM(this->get_logger(), "OdomCallback");
            interface_->getMapToOdomTF(msgOdom, tfMapOdom_);
        }
        else
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "Odometry msg recorded but no odometry mode is true, set to false to use this odometry");
    }

    void MonocularSlamNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msgImage)
    {
        Sophus::SE3f Tcw;
        bool tracked = false;

        // Use IMU-enabled tracking if IMU data is available, otherwise use monocular-only
        if (interface_->trackMonoculari(msgImage, Tcw))
        {
            tracked = true;
        }
        else if (interface_->trackMonocular(msgImage, Tcw))
        {
            tracked = true;
        }

        if (tracked)
        {
            isTracked_ = true;
            if (publish_tf_)
            {
                // populate map to base_footprint tf if odometry is not being used
                if (!odometry_mode_)
                {
                    tfMapOdom_ = geometry_msgs::msg::TransformStamped();
                    tfMapOdom_.header.stamp = msgImage->header.stamp;
                    tfMapOdom_.header.frame_id = global_frame_;
                    tfMapOdom_.child_frame_id = odom_frame_id_;
                    tfBroadcaster_->sendTransform(tfMapOdom_);
                    interface_->getDirectOdomToRobotTF(msgImage->header, tfMapOdom_);
                }
                // publish the tf if publish_tf_ is true
                tfBroadcaster_->sendTransform(tfMapOdom_);
            }

            // Publish robot pose
            publishRobotPose();
            publishSlamInfo();

            // Frequency tracking
            frequency_tracker_count_++;
            auto current_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - frequency_tracker_clock_);
            if (duration.count() >= 1000) // 1 second
            {
                double frequency = frequency_tracker_count_ / (duration.count() / 1000.0);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Tracking frequency: %.2f Hz", frequency);
                frequency_tracker_count_ = 0;
                frequency_tracker_clock_ = current_time;
            }
        }
        else
        {
            isTracked_ = false;
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Tracking failed");
        }
    }

    void MonocularSlamNode::publishMapData()
    {
        if (isTracked_)
        {
            slam_msgs::msg::MapData mapDataMsg;
            interface_->mapDataToMsg(mapDataMsg, true, false);
            mapDataPub_->publish(mapDataMsg);
        }
    }

    void MonocularSlamNode::publishRobotPose()
    {
        geometry_msgs::msg::PoseStamped robotPose;
        interface_->getRobotPose(robotPose);
        robotPoseMapFrame_->publish(robotPose);
    }

    void MonocularSlamNode::publishSlamInfo()
    {
        slam_msgs::msg::SlamInfo slamInfo;
        slamInfo.num_maps = interface_->getNumberOfMaps();
        slamInfo.num_keyframes_in_current_map = 0; // TODO: implement if needed
        slamInfo.tracking_frequency = frequency_tracker_count_; // Approximate frequency
        slamInfoPub_->publish(slamInfo);
    }

    void MonocularSlamNode::getMapDataServer(std::shared_ptr<rmw_request_id_t> /*request_header*/,
                                            std::shared_ptr<slam_msgs::srv::GetMap::Request> request,
                                            std::shared_ptr<slam_msgs::srv::GetMap::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Map data service called");
        interface_->mapDataToMsg(response->data, true, request->tracked_points, request->kf_id_for_landmarks);
    }

    void MonocularSlamNode::getAllMapPointsServer(std::shared_ptr<rmw_request_id_t> /*request_header*/,
                                                 std::shared_ptr<slam_msgs::srv::GetAllLandmarksInMap::Request> /*request*/,
                                                 std::shared_ptr<slam_msgs::srv::GetAllLandmarksInMap::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Get all map points service called");
        interface_->getCurrentMapPoints(response->landmarks);
    }

    void MonocularSlamNode::resetLocalMapServer(std::shared_ptr<rmw_request_id_t> /*request_header*/,
                                               std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                               std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Reset local map service called");
        if (request->data)
        {
            interface_->resetLocalMapping();
            response->success = true;
            response->message = "Local map reset successfully";
        }
        else
        {
            response->success = false;
            response->message = "Reset not requested";
        }
    }

    void MonocularSlamNode::getMapPointsInViewServer(std::shared_ptr<rmw_request_id_t> /*request_header*/,
                                                    std::shared_ptr<slam_msgs::srv::GetLandmarksInView::Request> request,
                                                    std::shared_ptr<slam_msgs::srv::GetLandmarksInView::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Get map points in view service called");
        std::vector<ORB_SLAM3::MapPoint*> mapPoints;
        interface_->mapPointsVisibleFromPose(request->pose, mapPoints, 1000, request->max_dist_pose_observation, request->max_angle_pose_observation);

        // Convert to slam_msgs/MapPoint array
        response->map_points.clear();
        for (auto* mp : mapPoints)
        {
            if (mp && !mp->isBad())
            {
                slam_msgs::msg::MapPoint mapPoint;
                auto pos = mp->GetWorldPos();
                mapPoint.position.x = pos[0];
                mapPoint.position.y = pos[1];
                mapPoint.position.z = pos[2];
                // observing_keyframes can be left empty for now
                response->map_points.push_back(mapPoint);
            }
        }

        // Publish for visualization
        geometry_msgs::msg::PoseStamped poseStamped;
        poseStamped.header.frame_id = global_frame_;
        poseStamped.header.stamp = this->get_clock()->now();
        poseStamped.pose = request->pose;
        visibleLandmarksPose_->publish(poseStamped);
    }
}
