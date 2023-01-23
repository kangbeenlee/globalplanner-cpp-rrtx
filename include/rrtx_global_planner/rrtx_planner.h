#ifndef RRTX_PLANNER_H
#define RRTX_PLANNER_H

#include <iostream>
#include <functional>
#include <algorithm>
#include <boost/thread.hpp>
#include <limits>
#include <chrono>
#include <memory>
#include <cmath>
#include <random>
#include <deque>
#include <list>
#include <set>

#include <rrtx_global_planner/rrtx_time.h>



namespace rrtx_global_planner
{
    class Node;
    using NodePtr = std::shared_ptr<Node>;
    using NodeSetPtr = std::shared_ptr<std::set<NodePtr>>;
    using StateValidityCheckerFn = std::function<bool(const NodePtr&)>;
    using EdgeValidityCheckerFn = std::function<bool(const NodePtr&, const NodePtr&)>;

    // for priority queue
    struct Queuekey
    {
        Queuekey(double main_cost, double sub_cost, std::weak_ptr<Node> node)
        : main_cost(main_cost), sub_cost(sub_cost), node(node) {}

        double main_cost, sub_cost;
        std::weak_ptr<Node> node;
    };

    // key comparison criterion
    struct compare
    {
        bool operator()(const Queuekey& key1, const Queuekey& key2)
        {
            if (key1.main_cost == key2.main_cost)
                return key1.sub_cost < key2.sub_cost;

            return key1.main_cost < key2.main_cost;
        }
    };

    class Node : public std::enable_shared_from_this<Node>
    {
    public:
        Node();
        Node(double x, double y,
            double lmc = std::numeric_limits<double>::infinity(),
            double cost_to_goal = std::numeric_limits<double>::infinity());
        ~Node();
        
        double getX() const;
        double getY() const;
        void setX(double new_x);
        void setY(double new_y);

        std::list<NodePtr> getAllOutNeighbors() const;
        std::list<NodePtr> getAllInNeighbors() const;
        std::list<NodePtr> getAllNeighbors() const;

        const NodeSetPtr& getOriginalOutNeighbors() const;
        const NodeSetPtr& getOriginalInNeighbors() const;
        const NodeSetPtr& getRunningOutNeighbors() const;
        const NodeSetPtr& getRunningInNeighbors() const;

        void addNodeToOriginalOutNeighbors(const NodePtr& node);
        void addNodeToOriginalInNeighbors(const NodePtr& node);
        void addNodeToRunningOutNeighbors(const NodePtr& node);
        void addNodeToRunningInNeighbors(const NodePtr& node);
        void removeNodeFromRunningOutNeighbors(const NodePtr& node);
        void removeNodeFromRunningInNeighbors(const NodePtr& node);

        void addNodeInObstacleEdgeSet(const NodePtr& node);
        void removeNodeInObstacleEdgeSet(const NodePtr& node);
        const NodeSetPtr& getObstacleEdgeSet() const;
        bool isInObstacleEdgeSet(const NodePtr& node) const;

        void setParent(const NodePtr& new_parent);
        bool haveParent() const;
        std::weak_ptr<Node> getParent() const;
        // const NodePtr& getParent() const;
        void addChild(const NodePtr& child);
        const NodeSetPtr& getChildren() const;
        void removeAllChildren();

        void setLMC(double new_lmc);
        double getLMC() const;
        void setCostToGoal(double cost_to_goal);
        double getCostToGoal() const;
        double computeDistance(const NodePtr& other) const;

    private:
        double x_;
        double y_;

        std::weak_ptr<Node> parent_;
        NodeSetPtr children_;

        double cost_to_goal_;
        double lmc_;
        
        NodeSetPtr E_O_;
        NodeSetPtr N_o_plus_;
        NodeSetPtr N_o_minus_;
        NodeSetPtr N_r_plus_;
        NodeSetPtr N_r_minus_;
    };

    class RRTX
    {
    public:
        RRTX();
        ~RRTX();

        static double computeAngle(const NodePtr& to, const NodePtr& from);

        // Algorithm 1
        void planning(double time);
        // Algorithm 2
        void extend(const NodePtr& v, const double& hyper_ball_radius);
        // Algorithm 3
        void cullNeighbors(const NodePtr& v, const double& radius);
        // Algorithm 4
        void rewireNeighbours(const NodePtr& v);
        // Algorithm 5
        void reduceInconsistency();
        // Algorithm 6; finds the best parent for v from the node set U
        void findParent(const NodePtr& v, const std::list<NodePtr>& U);
        // Algorithm 7
        double shrinkingBallRadius();
        // Algorithm 8; slightly different from pseudo code, only condiser the obstacle in local cost map
        void updateObstacles();
        // Algorithm 9; perform a cost-to-goal increase cascade leaf-ward through tree after an obstacle has been added
        void propagateDescendants();
        // Algorithm 10
        void verifyOrphan(const NodePtr& v);
        // Algorithm 11
        void removeObstacle();
        // Algorithm 12
        void addNewObstacle();
        // Algorithm 13
        void verifyQueue(const NodePtr& v);
        // Algorithm 14; update lmc(v) based on v's out-neighbors
        void updateLMC(const NodePtr& v);

        // whether key is less than key of node or not
        bool isKeyLess(const Queuekey& key, const NodePtr& node) const;
        // generate random node
        NodePtr randomNode() const;
        // add node in node set
        void addNode(const NodePtr& new_node);
        // find near neighbors within search radiuss
        std::list<NodePtr> near(const NodePtr& v, const double& hyper_ball_radius) const;
        // get nearest neighbor
        NodePtr nearest(const NodePtr& v) const;
        // NodePtr nearest(const NodePtr& v, const std::list<NodePtr>& node_set) const;
        // modify the position of random node within step length
        void saturate(const NodePtr& v, const NodePtr& v_nearest);
        void updateRobot();

        // compute and update gamma required for shrinking ball radius
        void updateGamma();
        // check whether node is in priority queue
        std::multiset<Queuekey>::iterator isInQueue(const NodePtr& v) const;
        std::list<NodePtr> computeDifferenceSet(const std::set<NodePtr>& set1,
                                                const std::set<NodePtr>& set2) const;
        std::list<NodePtr> computeDifferenceSet(const std::list<NodePtr>& set1,
                                                const std::set<NodePtr>& set2) const;
        std::list<NodePtr> computeUnionSet(const std::list<NodePtr>& set1,
                                        const std::list<NodePtr>& set2) const;
        void removeNodeFromTree(const NodePtr& node);
        void addNodeToTree(const NodePtr& node);
        bool isNodeInLocalCostMap(const NodePtr& v, const NodePtr& node) const;
        bool isInNodeSet(const NodePtr& node) const;
        bool isPathValid() const;

        // below functions must be called from outside
        void setStateValidityChecker(const StateValidityCheckerFn& svc);
        void setEdgeCollsionChecker(const EdgeValidityCheckerFn& evc);
        
        void setRobotBias(double robot_bias);
        void setDelta(double delta);
        void setGlobalWidth(double low_x, double high_x);
        void setGlobalHeigth(double low_y, double high_y);
        void setLocalCostMapSize(double local_width, double local_height);
        void setGoal(double goal_x, double goal_y);
        void setStart(double start_x, double start_y);
        void initialize();
        void updateRobotPosition(double bot_x, double bot_y);
        NodePtr findTargetNode(double robot_x, double robot_y) const;

        NodePtr nearestLMC(const double robot_x, const double robot_y) const;
        std::list<std::pair<double, double>> getPath() const;
        const std::list<NodePtr> getNodeSetWithOutOrphans() const;
        const std::set<NodePtr>& getNodeSet() const;
        const std::set<NodePtr>& getTreeNodes() const;
        const NodePtr& getGoalNode() const;

    private:
        StateValidityCheckerFn isStateValid_;
        EdgeValidityCheckerFn isCollision_;

        // robotUpdate() should be done after initial path is generated
        unsigned int max_nodes_{100};
        
        bool path_to_goal_{false};
        double hyper_ball_radius_;
        
        // node set in graph
        std::set<NodePtr> node_set_;
        // nodes in shortest-path sub-tree
        std::set<NodePtr> tree_nodes_;
        // nodes that have become disconnected from tree due to obstacles
        std::set<NodePtr> orphan_nodes_;

        // priority queue
        std::multiset<Queuekey, compare> queue_;

        NodePtr goal_node_;
        NodePtr start_node_;
        NodePtr robot_node_;
        double robot_x_;
        double robot_y_;
        double robot_bias_{0.1};

        // saturation distance
        double delta_;
        double ball_constant_;
        std::pair<double, double> global_width_;
        std::pair<double, double> global_height_;
        double local_half_width_;
        double local_half_height_;
        // for epsilon consistent graph
        double epsilon_{0.05};
        // to compute search radius
        double gamma_;
        // factor of safety so that gamma > expression from Theorem 38 of RRT* paper
        double gamma_FOS_{1.1};
        // RRTX is implemented only in 2-Dimensional space
        double dimension_{2.0};
        // volume of the unit d-ball in the 2-dimensional Euclidean space
        double zeta_d_;
    };
};

#endif