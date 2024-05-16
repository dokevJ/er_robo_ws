#include "hybrid_a_star_ros.h"

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

PLUGINLIB_EXPORT_CLASS(hybrid_a_star::HybridAStarROS, nav_core::BaseGlobalPlanner)

namespace hybrid_a_star
{
    HybridAStarROS::HybridAStarROS()
    : costmap_ros_(NULL), initialized_(false) {}
    
    HybridAStarROS::HybridAStarROS(std::string name, costmap_2d::Costmap2DROS *costmap_ros) 
    : costmap_ros_(NULL), initialized_(false)
    {
        initialize(name, costmap_ros);
    }

    void HybridAStarROS::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_) {

            if (!costmap_ros) {
                ROS_ERROR("Costmap is null, cannot initialize");
                return;
            }

            if (!kinodynamic_astar_searched_ptr_) {
                ros::NodeHandle nh;
                double steering_angle = nh.param("planner/steering_angle", 10);
                int steering_angle_discrete_num = nh.param("planner/steering_angle_discrete_num", 1);
                double wheel_base = nh.param("planner/wheel_base", 0.4);
                double segment_length = nh.param("planner/segment_length", 1.6);
                int segment_length_discrete_num = nh.param("planner/segment_length_discrete_num", 8);
                double steering_penalty = nh.param("planner/steering_penalty", 1.05);
                double steering_change_penalty = nh.param("planner/steering_change_penalty", 1.5);
                double reversing_penalty = nh.param("planner/reversing_penalty", 2.0);
                double shot_distance = nh.param("planner/shot_distance", 5.0);

                kinodynamic_astar_searched_ptr_ = std::make_shared<HybridAStar>(
                        steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
                        steering_penalty, reversing_penalty, steering_change_penalty, shot_distance
                );
            }

            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            ros::NodeHandle private_nh("~/" + name);
            
            const double map_resolution = 0.2;
            double origin_x = costmap_->getOriginX();
            double origin_y = costmap_->getOriginY();
            double map_width_meters = costmap_->getSizeInMetersX();
            double map_height_meters = costmap_->getSizeInMetersY();

            ROS_INFO("Initialize Global planner with map size : %.2f X %.2f meters, origin : (%.2f, %.2f)",
                     map_width_meters, map_height_meters, origin_x, origin_y);

            kinodynamic_astar_searched_ptr_->Init(origin_x, map_width_meters,
                                                  origin_y, map_height_meters,
                                                  costmap_->getResolution(),
                                                  map_resolution);
            
            for (unsigned int mx=0; mx<costmap_->getSizeInCellsX(); ++mx) {
                for (unsigned int my=0; my<costmap_->getSizeInCellsY(); ++my) {
                    unsigned char cost = costmap_->getCost(mx, my);
                    if (cost != costmap_2d::FREE_SPACE) {
                        unsigned int w = std::floor(mx * costmap_->getResolution() / map_resolution);
                        unsigned int h = std::floor(my * costmap_->getResolution() / map_resolution);
                        kinodynamic_astar_searched_ptr_->SetObstacle(w, h);
                    }
                }
            }

            initialized_ = true;
            ROS_INFO("Global Planner initialized!");
        }
        else {
            ROS_WARN("This planner has already been initialized... doing nothing");
        }
    }

    bool HybridAStarROS::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                               std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if(!initialized_) {
            ROS_ERROR("This planner has not been initialized, please call initailize() to use the planner");
            return false;
        }

        ROS_DEBUG("Got a start : (%.2f, %.2f), and Goal : (%.2f, %.2f)", start.pose.position.x, start.pose.position.y,
                  goal.pose.position.x, goal.pose.position.y);
        
        plan.clear();

        if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                      costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }

        tf2::Quaternion start_quat_tf;
        tf2::convert(start.pose.orientation, start_quat_tf);
        tf2::Quaternion goal_quat_tf;
        tf2::convert(goal.pose.orientation, goal_quat_tf);

        double start_yaw = tf2::getYaw(start_quat_tf);
        double goal_yaw = tf2::getYaw(goal_quat_tf);

        Vec3d start_state = Vec3d(start.pose.position.x, start.pose.position.y, start_yaw);
        
        Vec3d goal_state = Vec3d(goal.pose.position.x, goal.pose.position.y, goal_yaw);

        if (kinodynamic_astar_searched_ptr_->Search(start_state, goal_state)) {
            auto path = kinodynamic_astar_searched_ptr_->GetPath();
            publishPath(path);
            publishVehiclePath(path, 0.6, 0.4, 5u);
            publishSearchedTree(kinodynamic_astar_searched_ptr_->GetSearchedTree());

            geometry_msgs::PoseStamped pose_stamped;

            for (const auto &pose: path) {
                pose_stamped.header.frame_id = "map";
                pose_stamped.pose.position.x = pose.x();
                pose_stamped.pose.position.y = pose.y();
                pose_stamped.pose.position.z = 0.0;

                tf2::Quaternion q;
                q.setRPY(0, 0, pose.z());
                pose_stamped.pose.orientation = tf2::toMsg(q);

                plan.push_back(pose_stamped);
            }
            return true;
        }
        else {
            ROS_WARN("The planner failed to find a path, choose other goal position");
            return false;
        }
    }

    void HybridAStarROS::publishPath(const VectorVec3d &path) 
    {
        nav_msgs::Path nav_path;
        geometry_msgs::PoseStamped pose_stamped;

        for (const auto &pose: path) {
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = pose.x();
            pose_stamped.pose.position.y = pose.y();
            pose_stamped.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, pose.z());
            pose_stamped.pose.orientation = tf2::toMsg(q);

            nav_path.poses.emplace_back(pose_stamped);
        }

        nav_path.header.frame_id = "map";
        nav_path.header.stamp = timestamp_;

        path_pub_.publish(nav_path);
    }

    void HybridAStarROS::publishVehiclePath(const VectorVec3d &path, double width,
                                            double length, unsigned int vehicle_interval = 5u) 
    {
        visualization_msgs::MarkerArray vehicle_array;

        for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
            visualization_msgs::Marker vehicle;

            if (i == 0) {
                vehicle.action = 3;
            }

            vehicle.header.frame_id = "map";
            vehicle.header.stamp = ros::Time::now();
            vehicle.type = visualization_msgs::Marker::CUBE;
            vehicle.id = static_cast<int>(i / vehicle_interval);
            vehicle.scale.x = width;
            vehicle.scale.y = length;
            vehicle.scale.z = 0.01;
            vehicle.color.a = 0.1;

            vehicle.color.r = 1.0;
            vehicle.color.b = 0.0;
            vehicle.color.g = 0.0;

            vehicle.pose.position.x = path[i].x();
            vehicle.pose.position.y = path[i].y();
            vehicle.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, path[i].z());
            vehicle.pose.orientation = tf2::toMsg(q);
            vehicle_array.markers.emplace_back(vehicle);
        }

        vehicle_path_pub_.publish(vehicle_array);
    }

    void HybridAStarROS::publishSearchedTree(const VectorVec4d &searched_tree) 
    {
        visualization_msgs::Marker tree_list;
        tree_list.header.frame_id = "map";
        tree_list.header.stamp = ros::Time::now();
        tree_list.type = visualization_msgs::Marker::LINE_LIST;
        tree_list.action = visualization_msgs::Marker::ADD;
        tree_list.ns = "searched_tree";
        tree_list.scale.x = 0.02;

        tree_list.color.a = 1.0;
        tree_list.color.r = 0;
        tree_list.color.g = 0;
        tree_list.color.b = 0;

        tree_list.pose.orientation.w = 1.0;
        tree_list.pose.orientation.x = 0.0;
        tree_list.pose.orientation.y = 0.0;
        tree_list.pose.orientation.z = 0.0;

        geometry_msgs::Point point;

        for (const auto &i: searched_tree) {
            point.x = i.x();
            point.y = i.y();
            point.z = 0.0;
            tree_list.points.emplace_back(point);

            point.x = i.z();
            point.y = i.w();
            point.z = 0.0;
            tree_list.points.emplace_back(point);
        }

        searched_tree_pub_.publish(tree_list);
    }
} // namespace hybrid_a_star
