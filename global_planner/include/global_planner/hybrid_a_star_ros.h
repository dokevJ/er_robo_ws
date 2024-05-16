#ifndef HYBRID_A_STAR_ROS_H
#define HYBRID_A_STAR_ROS_H

#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <cmath>

#include <memory>
#include <boost/thread.hpp>

#include <tf2_ros/buffer.h>

#include <dynamic_reconfigure/server.h>

#include <angles/angles.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>

#include <vector>
#include "hybrid_a_star.h"
#include "type.h"
#include <ros/ros.h>

namespace hybrid_a_star 
{
    class HybridAStarROS : public nav_core::BaseGlobalPlanner {
    public:
        HybridAStarROS();
        HybridAStarROS(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);
        ~HybridAStarROS();

    private:
        void publishPath(const VectorVec3d &path);
        void publishVehiclePath(const VectorVec3d &path, double width,
                                                double length, unsigned int vehicle_interval);
        void publishSearchedTree(const VectorVec4d &searched_tree);

        ros::Publisher path_pub_;
        ros::Publisher searched_tree_pub_;
        ros::Publisher vehicle_path_pub_;

        std::shared_ptr<HybridAStar> kinodynamic_astar_searched_ptr_;
        costmap_2d::Costmap2DROS* costmap_ros_;
        costmap_2d::Costmap2D* costmap_;

        ros::Time timestamp_;

        bool initialized_;
    };
}

#endif