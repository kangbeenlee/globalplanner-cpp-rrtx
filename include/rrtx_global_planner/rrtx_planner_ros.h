#ifndef RRTX_PLANNER_ROS_H
#define RRTX_PLANNER_ROS_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <base_local_planner/costmap_model.h>

#include <rrtx_global_planner/rrtx_planner.h>



namespace rrtx_global_planner
{
    class RRTXPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        RRTXPlanner();
        ~RRTXPlanner();

        // default functions for plugin:
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        // for visualization
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        void publishTree();
        void publishNodes();

        // rrtx functions
        void resetRRTX();
        bool isStateValid(const NodePtr& node);
        bool isCollision(const NodePtr& v, const NodePtr& u);
        std::list<std::pair<int, int>> computeBresenhamLine(int x0, int y0, int x1, int y1);

    private:
        costmap_2d::Costmap2DROS* costmap_ros_; 
        costmap_2d::Costmap2D* costmap_;
        std::shared_ptr<base_local_planner::WorldModel> world_model_;
        // for plugin setup
        bool initialized_;

        boost::mutex mutex_;

        // for rrtx
        std::shared_ptr<RRTX> rrtx_;
        // cell cost threshold
        double cost_threshold_{50.0};
        // to check initialize rrtx (rrtx must be initialized when goal is changed)
        double goal_x_, goal_y_;
        bool first_flag_{true};

        // visualize path
        ros::Publisher plan_pub_;
        ros::Publisher nodes_pub_;
        ros::Publisher graph_pub_;
    };
}

#endif