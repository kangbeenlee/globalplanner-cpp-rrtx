#include <rrtx_global_planner/rrtx_planner_ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrtx_global_planner::RRTXPlanner, nav_core::BaseGlobalPlanner)

namespace rrtx_global_planner
{
    RRTXPlanner::RRTXPlanner() : costmap_ros_(NULL), initialized_(false)
    {
    }

    RRTXPlanner::~RRTXPlanner()
    {
    }

    void RRTXPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if (!initialized_)
        {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            nodes_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("nodes", 30);
            graph_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("tree", 30);
            world_model_ = std::make_shared<base_local_planner::CostmapModel>(*costmap_);

            resetRRTX();
            initialized_ = true;
            ROS_INFO("RRTX global planner initialized!");
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }

    void RRTXPlanner::resetRRTX()
    {
        rrtx_ = std::make_shared<RRTX>();
        rrtx_->setDelta(1.8);
        rrtx_->setGlobalWidth(-10.0, 10.0);
        rrtx_->setGlobalHeigth(-10.0, 10.0);
        rrtx_->setLocalCostMapSize(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
        rrtx_->setStateValidityChecker(boost::bind(&RRTXPlanner::isStateValid, this, _1));
        rrtx_->setEdgeCollsionChecker(boost::bind(&RRTXPlanner::isCollision, this, _1, _2));
    }

    bool RRTXPlanner::isStateValid(const NodePtr& node)
    {
        double x = node->getX();
        double y = node->getY();

        unsigned int mx, my;

        costmap_->worldToMap(x, y, mx, my);
        double cell_cost = costmap_->getCost(mx, my);

        if (cell_cost >= 0 && cell_cost < cost_threshold_)
            return true;
        return false;
    }

    bool RRTXPlanner::isCollision(const NodePtr& v, const NodePtr& u)
    {
        unsigned int m_vx, m_vy, m_ux, m_uy;
        costmap_->worldToMap(v->getX(), v->getY(), m_vx, m_vy);
        costmap_->worldToMap(u->getX(), u->getY(), m_ux, m_uy);

        auto bresenham_line = computeBresenhamLine(m_vx, m_vy, m_ux, m_uy);

        for (const auto& point : bresenham_line)
        {
            double cell_cost = costmap_->getCost(point.first, point.second);
            if (cell_cost < 0 || cell_cost >= cost_threshold_)
                return true;
        }
        return false;
    }

    // Bresenham's algorithm (http://members.chello.at/~easyfilter/bresenham.html)
    std::list<std::pair<int, int>> RRTXPlanner::computeBresenhamLine(int x0, int y0, int x1, int y1)
    {
        std::list<std::pair<int, int>> line;
        int dx =  abs(x1-x0), sx = x0<x1 ? 1 : -1;
        int dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1; 
        int err = dx+dy, e2; /* error value e_xy */
    
        for(;;)
        {  /* loop */
            line.emplace_back(x0, y0);
            if (x0==x1 && y0==y1) break;
            e2 = 2*err;
            if (e2 >= dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
            if (e2 <= dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
        }
        return line;
    }

    bool RRTXPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                    std::vector<geometry_msgs::PoseStamped>& plan)
    {
        boost::mutex::scoped_lock lock(mutex_);

        if (!initialized_)
        {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        if(goal.header.frame_id != costmap_ros_->getGlobalFrameID())
        {
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
            costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }

        // ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
        // ROS_DEBUG("Check boundary size %.2f, %.2f, %.2f, %.2f", costmap_->getOriginX(), costmap_->getOriginY(),
        //            costmap_->getOriginX() + costmap_->getSizeInCellsX() * costmap_->getResolution(),
        //            costmap_->getOriginY() + costmap_->getSizeInCellsX() * costmap_->getResolution());
        ROS_INFO("Try to find global path with RRTX ...");

        if (goal_x_ != goal.pose.position.x && goal_y_ != goal.pose.position.y)
        {
            ROS_INFO("New goal is received ... RRTX is on intialization ...");

            rrtx_->initialize();
            goal_x_ = goal.pose.position.x;
            goal_y_ = goal.pose.position.y;
            rrtx_->setGoal(goal_x_, goal_y_);
            rrtx_->setStart(start.pose.position.x, start.pose.position.y);
            std::cout << ">>> " << rrtx_->isPathValid() << std::endl;
        }

        rrtx_->updateRobotPosition(start.pose.position.x, start.pose.position.y);
        rrtx_->planning(0.01);

        // clear the plan, just in case
        plan.clear();
        plan.push_back(start);

        bool check = rrtx_->isPathValid();

        // convert path into ROS messages:
        if (rrtx_->isPathValid())
        {
            ROS_INFO("Successed to find global path with RRTX!");

            std::list<std::pair<double, double>> path = rrtx_->getPath();
            for (const auto& point : path)
            {
                geometry_msgs::PoseStamped ps = goal;
                ps.header.stamp = ros::Time::now();
                ps.pose.position.x = point.first;
                ps.pose.position.y = point.second;
                plan.push_back(ps);
            }
            plan.push_back(goal);
            publishPlan(plan);
        }
        else
        {
            ROS_ERROR("Failed to compute global path with RRTX");
        }

        // publish the plan to visualize
        publishNodes();
        publishTree();

        return rrtx_->isPathValid();
    }

    void RRTXPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
    {
        if (!initialized_)
        {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return;
        }

        // create a message for the plan 
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        if (!path.empty())
            gui_path.header = path[0].header;

        gui_path.poses = path;
        plan_pub_.publish(gui_path);
    }

    void RRTXPlanner::publishNodes()
    {
        if (!initialized_)
        {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return;
        }

        visualization_msgs::Marker node;
        node.type = visualization_msgs::Marker::SPHERE;
        node.header.frame_id = costmap_ros_->getGlobalFrameID();
        node.action = visualization_msgs::Marker::ADD;

        std::set<NodePtr> NodeSet = rrtx_->getNodeSet();
        visualization_msgs::MarkerArray nodes;
        unsigned int id = 0;
        for (const auto& n : NodeSet)
        {   
            node.id = id;
            id++;
            node.header.stamp = ros::Time::now();
            node.pose.orientation.w = 1.0;
            node.pose.position.x = n->getX();
            node.pose.position.y = n->getY();
            node.color.b = 0.5;
            node.color.r = 0.5;
            node.color.g = 0.5;
            node.color.a = 1.0;
            node.scale.x = 0.05;
            node.scale.y = 0.05;
            node.scale.z = 0.05;
            nodes.markers.push_back(node);
        }
        nodes_pub_.publish(nodes);
    }

    void RRTXPlanner::publishTree()
    {
        visualization_msgs::Marker edge;
        edge.type = visualization_msgs::Marker::LINE_LIST;
        edge.header.frame_id = costmap_ros_->getGlobalFrameID();
        // edge.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point ps, pe;

        std::set<NodePtr> tree_nodes = rrtx_->getTreeNodes();

        visualization_msgs::MarkerArray edges;
        unsigned int id = 0;

        for (const auto node : tree_nodes)
        {
            edge.points.clear();
            ps.x = node->getX();
            ps.y = node->getY();

            for (const auto& child : *node->getChildren())
            {
                pe.x = child->getX();
                pe.y = child->getY();
                edge.id = id;
                id++;
                edge.header.stamp = ros::Time::now();
                edge.pose.orientation.w = 1;
                edge.color.g = 1.0;
                edge.color.a = 1.0;
                edge.scale.x = 0.03;
                edge.points.push_back(ps);
                edge.points.push_back(pe);
                edges.markers.push_back(edge);
            }
        }
        graph_pub_.publish(edges);
    }
} // end namespace global_planner