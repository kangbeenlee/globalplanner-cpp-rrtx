#include <rrtx_global_planner/rrtx_planner.h>

////////////////////////////////////////////////////////////////////////////////
////////////////////////// Node class member functions /////////////////////////
////////////////////////////////////////////////////////////////////////////////

namespace rrtx_global_planner
{
    Node::Node()
    {
        // parent_.lock() = nullptr;
        children_ = std::make_shared<std::set<NodePtr>>();

        E_O_ = std::make_shared<std::set<NodePtr>>();
        N_o_plus_ = std::make_shared<std::set<NodePtr>>();
        N_o_minus_ = std::make_shared<std::set<NodePtr>>();
        N_r_plus_ = std::make_shared<std::set<NodePtr>>();
        N_r_minus_ = std::make_shared<std::set<NodePtr>>();
    }
    
    Node::Node(double x, double y, double lmc, double cost_to_goal)
    : x_(x), y_(y), lmc_(lmc), cost_to_goal_(cost_to_goal)
    {
        children_ = std::make_shared<std::set<NodePtr>>();

        E_O_ = std::make_shared<std::set<NodePtr>>();
        N_o_plus_ = std::make_shared<std::set<NodePtr>>();
        N_o_minus_ = std::make_shared<std::set<NodePtr>>();
        N_r_plus_ = std::make_shared<std::set<NodePtr>>();
        N_r_minus_ = std::make_shared<std::set<NodePtr>>();
    }

    Node::~Node()
    {
    }

    double Node::getX() const
    {
        return x_;
    }

    double Node::getY() const
    {
        return y_;
    }

    void Node::setX(double new_x)
    {
        x_ = new_x;
    }

    void Node::setY(double new_y)
    {
        y_ = new_y;
    }

    std::list<NodePtr> Node::getAllOutNeighbors() const
    {
        std::list<NodePtr> all_out_neighbors;
        std::set_union(N_o_plus_->begin(), N_o_plus_->end(),
                    N_r_plus_->begin(), N_r_plus_->end(),
                    std::inserter(all_out_neighbors, all_out_neighbors.begin()));

        return all_out_neighbors;
    }

    std::list<NodePtr> Node::getAllInNeighbors() const
    {
        std::list<NodePtr> all_in_neighbors;
        std::set_union(N_o_minus_->begin(), N_o_minus_->end(),
                    N_r_minus_->begin(), N_r_minus_->end(),
                    std::inserter(all_in_neighbors, all_in_neighbors.begin()));

        return all_in_neighbors;
    }

    std::list<NodePtr> Node::getAllNeighbors() const
    {
        std::list<NodePtr> all_neighbors;
        std::list<NodePtr> all_out_neighbors = getAllOutNeighbors();
        std::list<NodePtr> all_in_neighbors = getAllInNeighbors();

        std::set_union(all_out_neighbors.begin(), all_out_neighbors.end(),
                    all_in_neighbors.begin(), all_in_neighbors.end(),
                    std::inserter(all_neighbors, all_neighbors.begin()));

        return all_neighbors;
    }

    const NodeSetPtr& Node::getOriginalOutNeighbors() const
    {
        return N_o_plus_;
    }

    const NodeSetPtr& Node::getOriginalInNeighbors() const
    {
        return N_o_minus_;
    }

    const NodeSetPtr& Node::getRunningOutNeighbors() const
    {
        return N_r_plus_;
    }

    const NodeSetPtr& Node::getRunningInNeighbors() const
    {
        return N_r_minus_;
    }

    void Node::addNodeToOriginalOutNeighbors(const NodePtr& node)
    {
        N_o_plus_->insert(node);
    }

    void Node::addNodeToOriginalInNeighbors(const NodePtr& node)
    {
        N_o_minus_->insert(node);
    }

    void Node::addNodeToRunningOutNeighbors(const NodePtr& node)
    {
        N_r_plus_->insert(node);
    }

    void Node::addNodeToRunningInNeighbors(const NodePtr& node)
    {
        N_r_minus_->insert(node);
    }

    void Node::removeNodeFromRunningOutNeighbors(const NodePtr& node)
    {
        auto itr = N_r_plus_->find(node);
        if (itr != N_r_plus_->end())
            N_r_plus_->erase(itr);
    }

    void Node::removeNodeFromRunningInNeighbors(const NodePtr& node)
    {
        auto itr = N_r_minus_->find(node);
        if (itr != N_r_minus_->end())
            N_r_minus_->erase(itr);
    }

    void Node::addNodeInObstacleEdgeSet(const NodePtr& node)
    {
        E_O_->insert(node);
    }

    void Node::removeNodeInObstacleEdgeSet(const NodePtr& node)
    {
        auto itr = E_O_->find(node);
        if (itr != E_O_->end())
            E_O_->erase(itr);
    }

    const NodeSetPtr& Node::getObstacleEdgeSet() const
    {
        return E_O_;
    }

    bool Node::isInObstacleEdgeSet(const NodePtr& node) const
    {
        auto itr = E_O_->find(node);
        if (itr != E_O_->end())
            return true;
        return false;
    }

    void Node::setParent(const NodePtr& new_parent)
    {
        std::weak_ptr<Node> unlock_parent = getParent();
        NodePtr parent = unlock_parent.lock();

        // if a parent exists already then remove node-self from node's previous parent
        if (parent)
        {
            auto itr = parent->children_->find(shared_from_this());
            if (itr != children_->end())
                parent->children_->erase(itr);
        }

        // if new_parent is not nullptr
        if (new_parent)
        {
            parent_ = new_parent;
            new_parent->addChild(shared_from_this());
            // new_parent->children_->insert(shared_from_this());
        }
        // if new_parent is nullptr, initialize parent
        else
            parent_ = std::weak_ptr<Node>();
    }

    bool Node::haveParent() const
    {
        if (parent_.lock() == nullptr)
            return false;
        return true;
    }

    std::weak_ptr<Node> Node::getParent() const
    {
        return parent_;
    }

    void Node::addChild(const NodePtr& child)
    {
        children_->insert(child);
    }

    const NodeSetPtr& Node::getChildren() const
    {
        return children_;
    }

    void Node::removeAllChildren()
    {
        std::list<NodePtr> children_data;
        std::copy(children_->begin(), children_->end(), std::back_inserter(children_data));

        // remove child
        for (const auto& child : children_data)
        {
            auto itr = children_->find(child);
            if (itr != children_->end())
                children_->erase(itr);
        }
    }

    void Node::setLMC(double new_lmc)
    {
        lmc_ = new_lmc;
    }

    double Node::getLMC() const
    {
        return lmc_;
    }

    void Node::setCostToGoal(double cost_to_goal)
    {
        cost_to_goal_ = cost_to_goal;
    }

    double Node::getCostToGoal() const
    {
        return cost_to_goal_;
    }

    double Node::computeDistance(const NodePtr& other) const
    {
        if (isInObstacleEdgeSet(other))
            return std::numeric_limits<double>::infinity();
        return std::hypot(x_ - other->getX(), y_ - other->getY());
    }



    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////// RRTX class member functions /////////////////////////
    ////////////////////////////////////////////////////////////////////////////////

    RRTX::RRTX()
    {
        goal_node_ = std::make_shared<Node>();
        start_node_ = std::make_shared<Node>();

        node_set_ = {};
        tree_nodes_ = {};
        orphan_nodes_ = {};

        // node_set_ = std::make_shared<std::set<NodePtr>>();
        // tree_nodes_ = std::make_shared<std::set<NodePtr>>();
        // orphan_nodes_ = std::make_shared<std::set<NodePtr>>();

        zeta_d_ = std::acos(-1);
    }
    RRTX::~RRTX()
    {
    }

    double RRTX::computeAngle(const NodePtr& to, const NodePtr& from)
    {
        double dx = to->getX() - from->getX();
        double dy = to->getY() - from->getY();

        return std::atan2(dy, dx);
    }

    void RRTX::planning(double time)
    {
        rrtx_time::point start = rrtx_time::now();
        double first_flag = true;
        double obstacle_flag = true;

        while (rrtx_time::seconds(rrtx_time::now() - start) < time)
        {
            hyper_ball_radius_ = shrinkingBallRadius();

            // given global planning time, assume that obstacles are not changed
            if (obstacle_flag)
            {
                updateObstacles();
                obstacle_flag = false;
            }

            // given global planning time, assume that robot position is not changed
            if (path_to_goal_ && first_flag)
            {
                updateRobot();
                first_flag = false;
            }

            NodePtr v = randomNode();
            NodePtr v_nearest = nearest(v);

            if (v->computeDistance(v_nearest) > delta_)
                saturate(v, v_nearest);

            if (isStateValid_(v))
                extend(v, hyper_ball_radius_);
    
            if (isInNodeSet(v))
                rewireNeighbours(v);
                reduceInconsistency();
        }
    }

    void RRTX::extend(const NodePtr& v, const double& hyper_ball_radius)
    {
        std::list<NodePtr> V_near = near(v, hyper_ball_radius);
        findParent(v, V_near);

        std::weak_ptr<Node> unlock_v_parent = v->getParent();
        NodePtr v_parent = unlock_v_parent.lock();
        if (v_parent == nullptr)
            return;

        addNode(v);
        // child v has already been added to children of v's parent with setParent() in findParent()

        for (const auto& u : V_near)
        {
            // collision check from v to u and from u to v are same
            if (!isCollision_(v, u))
            {
                v->addNodeToOriginalOutNeighbors(u);
                v->addNodeToOriginalInNeighbors(u);
                u->addNodeToRunningOutNeighbors(v);
                u->addNodeToRunningInNeighbors(v);
            }
        }
    }

    void RRTX::cullNeighbors(const NodePtr& v, const double& hyper_ball_radius_)
    {
        std::list<NodePtr> N_r_plus_list;
        std::copy(v->getRunningOutNeighbors()->begin(), v->getRunningOutNeighbors()->end(),
                  std::back_inserter(N_r_plus_list));

        for (const auto& u : N_r_plus_list)
        {
            std::weak_ptr<Node> unlock_v_parent = v->getParent();
            NodePtr v_parent = unlock_v_parent.lock();

            if (v_parent != u && hyper_ball_radius_ < v->computeDistance(u))
            {
                // delete u from v's running out neighbors
                v->removeNodeFromRunningOutNeighbors(u);
                // delete v from u's running in neighbors
                u->removeNodeFromRunningInNeighbors(v);
            }
        }
    }

    void RRTX::rewireNeighbours(const NodePtr& v)
    {
        if (v->getCostToGoal() - v->getLMC() <= epsilon_)
            return;

        cullNeighbors(v, hyper_ball_radius_);

        std::weak_ptr<Node> unlock_v_parent = v->getParent();
        NodePtr v_parent = unlock_v_parent.lock();

        for (const auto& u : v->getAllInNeighbors())
        {
            if (u != v_parent)
            {
                double lmc_prime = u->computeDistance(v) + v->getLMC();
                if (u->getLMC() > lmc_prime && !isCollision_(u, v))
                    {
                        u->setLMC(lmc_prime);
                        u->setParent(v);
                        if (u->getCostToGoal() - u->getLMC() > epsilon_)
                            verifyQueue(u);
                    }
            }                    
        }
    }

    void RRTX::reduceInconsistency()
    {
        // refer julia code
        while (queue_.size() && (
               isKeyLess(*queue_.begin(), goal_node_) // keyLess(top(Q), v_bot)
            || goal_node_->getLMC()        == std::numeric_limits<double>::infinity()
            || goal_node_->getCostToGoal() == std::numeric_limits<double>::infinity()
            || isInQueue(goal_node_)       != queue_.end()))
        {
            // pop top key from priority queue
            auto itr = queue_.begin();

            std::weak_ptr<Node> unlock_node = (*itr).node;
            NodePtr v = unlock_node.lock();
            queue_.erase(itr);

            if (v->getCostToGoal() - v->getLMC() > epsilon_)
            {
                updateLMC(v);
                rewireNeighbours(v);
            }
            v->setCostToGoal(v->getLMC());
        }
    }

    void RRTX::findParent(const NodePtr& v, const std::list<NodePtr>& U)
    {
        for (const auto& u : U)
        {
            double lmc_prime = v->computeDistance(u) + u->getLMC();
            if (v->getLMC() > lmc_prime && !isCollision_(v, u))
            {
                v->setParent(u);
                v->setLMC(lmc_prime);
            }
        }
    }

    double RRTX::shrinkingBallRadius()
    {
        // refer julia code
        return std::min(delta_, ball_constant_ * pow(log(node_set_.size() + 1) / (node_set_.size()), 1 / dimension_));
    }

    void RRTX::updateObstacles()
    {
        // removeObstacle();
        addNewObstacle();
        propagateDescendants();
        verifyQueue(robot_node_);
        reduceInconsistency();
    }

    void RRTX::propagateDescendants()
    {
        // recursively add children of node in orphan_nodes to orphan_node
        std::deque<NodePtr> orphan_deque;

        for (const auto& v : orphan_nodes_)
            orphan_deque.emplace_back(v);

        while (orphan_deque.size())
        {
            NodePtr orphan = orphan_deque.front();
            orphan_deque.pop_front();

            for (const auto& child : *(orphan->getChildren()))
            {
                orphan_deque.emplace_back(child);
                orphan_nodes_.insert(child);
            }
        }

        // check if robot node got orphaned, not in pseudo code
        auto itr = orphan_nodes_.find(robot_node_);
        if (itr != orphan_nodes_.end())
            path_to_goal_ = false;

        for (const auto& v : orphan_nodes_)
        {
            std::list<NodePtr> parent_set;

            std::weak_ptr<Node> unlock_v_parent = v->getParent();
            NodePtr v_parent = unlock_v_parent.lock();

            if (v_parent == goal_node_)
                continue;

            if (v_parent)
                parent_set = {v_parent};

            std::list<NodePtr> all_out_neighbors_of_v_with_parent =
                computeUnionSet(v->getAllOutNeighbors(), parent_set);

            std::list<NodePtr> all_out_neighbors_of_v_without_orphans =
                computeDifferenceSet(all_out_neighbors_of_v_with_parent, orphan_nodes_);

            for (const auto& u : all_out_neighbors_of_v_without_orphans)
            {
                u->setCostToGoal(std::numeric_limits<double>::infinity());
                verifyQueue(u);
            }
        }

        for (const auto& v : orphan_nodes_)
        {
            v->setCostToGoal(std::numeric_limits<double>::infinity());
            v->setLMC(std::numeric_limits<double>::infinity());

            // remove v from tree
            std::weak_ptr<Node> unlock_v_parent = v->getParent();
            NodePtr v_parent = unlock_v_parent.lock();

            if (v_parent)
            {
                // v is already removed from children of v's parent in setParent()
                v->setParent(nullptr);
                // orphan node shouldn't be removed from node set
                removeNodeFromTree(v);
            }
        }
        // reset orphan nodes to empty set
        orphan_nodes_ = {};
    }

    void RRTX::verifyOrphan(const NodePtr& v)
    {
        auto itr = isInQueue(v);
        if (itr != queue_.end())
            queue_.erase(itr);

        orphan_nodes_.insert(v);
    }

    void RRTX::removeObstacle()
    {
        std::set<std::pair<NodePtr, NodePtr>> E_free;

        for (const auto& node : node_set_)
        {
            if (node == goal_node_)
                continue;

            if (!isNodeInLocalCostMap(robot_node_, node))
                continue;

            // check whether this node is included 
            std::weak_ptr<Node> unlock_parent = node->getParent();
            NodePtr parent = unlock_parent.lock();
            if (parent == nullptr)
                continue;
            
            std::list<NodePtr> all_out_neighbors = node->getAllOutNeighbors();
            for (const auto& out_neighbor : all_out_neighbors)
            {
                if (!isCollision_(node, out_neighbor))
                    E_free.emplace(node, out_neighbor);
            }
        }
        
        for (const auto& edge : E_free)
        {
            NodePtr v = edge.first;
            NodePtr u = edge.second;

            v->removeNodeInObstacleEdgeSet(u);
            // u->removeNodeInObstacleEdgeSet(v);

            updateLMC(v);

            if (v->getLMC() != v->getCostToGoal())
                verifyQueue(v);
        }
    }

    void RRTX::addNewObstacle()
    {
        // how to find edges under collision with obstacle?
        // only consider the local cost map area (check how far from robot node)
        // if isCollision_() is true then add egde in E_O

        std::set<std::pair<NodePtr, NodePtr>> E_O;

        for (const auto& node : tree_nodes_)
        {
            if (node == goal_node_)
                continue;

            if (!isNodeInLocalCostMap(robot_node_, node))
                continue;

            std::weak_ptr<Node> unlock_parent = node->getParent();
            NodePtr parent = unlock_parent.lock();
            if (parent == nullptr)
                continue;
            
            if (isCollision_(node, parent))
                E_O.emplace(node, parent);
        }
        
        // 모든 E_O에 대해서
        for (const auto& edge : E_O)
        {
            NodePtr v = edge.first;
            NodePtr u = edge.second;

            v->addNodeInObstacleEdgeSet(u);
            // u->addNodeInObstacleEdgeSet(v);

            // u is already parent of v
            verifyOrphan(v);
        }
    }

    void RRTX::verifyQueue(const NodePtr& v)
    {
        auto itr = isInQueue(v);
        if (itr != queue_.end())
        {
            queue_.erase(itr);
            queue_.emplace(std::min(v->getCostToGoal(), v->getLMC()), v->getCostToGoal(), v);
        }
        else
            queue_.emplace(std::min(v->getCostToGoal(), v->getLMC()), v->getCostToGoal(), v);
    }

    void RRTX::updateLMC(const NodePtr& v)
    {
        cullNeighbors(v, hyper_ball_radius_);

        std::list<NodePtr> all_out_neighbors_of_v_without_orphans =
            computeDifferenceSet(v->getAllOutNeighbors(), orphan_nodes_);

        for(const auto& u : all_out_neighbors_of_v_without_orphans)
        {
            std::weak_ptr<Node> unlock_u_parent = u->getParent();
            NodePtr u_parent = unlock_u_parent.lock();

            if (u_parent != v)
            {
                double lmc_prime = v->computeDistance(u) + u->getLMC();
                if (v->getLMC() > lmc_prime && !isCollision_(v, u))
                {
                    v->setParent(u);
                    v->setLMC(lmc_prime);
                }
            }
        }
        addNodeToTree(v);
    }

    bool RRTX::isKeyLess(const Queuekey& key, const NodePtr& node) const
    {
        if (key.main_cost == std::min(node->getCostToGoal(), node->getLMC()))
            return key.sub_cost < node->getCostToGoal();
        return key.main_cost < std::min(node->getCostToGoal(), node->getLMC());
    }

    NodePtr RRTX::randomNode() const
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> generate_random_x(global_width_.first, global_width_.second);
        std::uniform_real_distribution<double> generate_random_y(global_height_.first, global_height_.second);

        std::uniform_real_distribution<double> generate_biased_sampling_value(0, 1);

        // for biased sampling
        if (!path_to_goal_ && generate_biased_sampling_value(gen) < robot_bias_)
            return std::make_shared<Node>(robot_x_, robot_y_);
            
        // for uniform sampling
        double random_x = generate_random_x(gen);
        double random_y = generate_random_y(gen);

        return std::make_shared<Node>(random_x, random_y);
    }

    void RRTX::addNode(const NodePtr& new_node)
    {
        // if new node is at robot, then path to goal is found
        if (robot_x_ == new_node->getX() && robot_y_ == new_node->getY())
        {
            robot_node_ = new_node;
            path_to_goal_ = true;
        }

        node_set_.insert(new_node);
        tree_nodes_.insert(new_node);
    }

    std::list<NodePtr> RRTX::near(const NodePtr& v, const double& hyper_ball_radius) const
    {
        std::list<NodePtr> V_near;
        for (const auto& u : node_set_)
        {
            if (v->computeDistance(u) <= hyper_ball_radius)
                V_near.push_back(u);
        }

        return V_near;
    }

    NodePtr RRTX::nearest(const NodePtr& v) const
    {
        NodePtr nearest_node;
        double min_distance = std::numeric_limits<double>::infinity();
        for (const auto& u : node_set_)
        {
            double distance = v->computeDistance(u);
            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_node = u;
            }
        }

        return nearest_node;
    }

    void RRTX::saturate(const NodePtr& v, const NodePtr& v_nearest)
    {
        double angle = computeAngle(v, v_nearest);
        v->setX(v_nearest->getX() + delta_ * std::cos(angle));
        v->setY(v_nearest->getY() + delta_ * std::sin(angle));
    }

    void RRTX::updateRobot()
    {
        std::weak_ptr<Node> unlock_robot_parent = robot_node_->getParent();
        NodePtr robot_parent = unlock_robot_parent.lock();

        if (robot_parent == nullptr)
            return;

        double distance = std::hypot(robot_parent->getX() - robot_x_, robot_parent->getY() - robot_y_);
        double edge_length = robot_node_->computeDistance(robot_parent);
        if (distance <= edge_length / 2.0)
            robot_node_ = robot_parent;
    }

    void RRTX::updateGamma()
    {
        // assume that all space is free
        double mu_X_free = (global_width_.second - global_width_.first) * (global_height_.second - global_height_.first);
        gamma_ = gamma_FOS_ * pow(2 * (1 + 1 / dimension_), 1 / dimension_) * pow(mu_X_free / zeta_d_, 1 / dimension_);
    }

    std::multiset<Queuekey>::iterator RRTX::isInQueue(const NodePtr& v) const
    {
        for (auto itr = queue_.begin(); itr != queue_.end(); ++itr)
        {
            auto unlock_node = (*itr).node;
            auto node_in_Q = unlock_node.lock();

            if (v == node_in_Q)
                return itr;
        }
        return queue_.end();
    }

    std::list<NodePtr> RRTX::computeDifferenceSet(const std::set<NodePtr>& set1,
                                                  const std::set<NodePtr>& set2) const
    {
        std::list<NodePtr> different_set;
        std::set_difference(set1.begin(), set1.end(), set2.begin(), set2.end(),
                            std::inserter(different_set, different_set.end()));

        return different_set;
    }

    std::list<NodePtr> RRTX::computeDifferenceSet(const std::list<NodePtr>& set1,
                                                  const std::set<NodePtr>& set2) const
    {
        std::list<NodePtr> different_set;
        std::set_difference(set1.begin(), set1.end(), set2.begin(), set2.end(),
                            std::inserter(different_set, different_set.end()));

        return different_set;
    }

    std::list<NodePtr> RRTX::computeUnionSet(const std::list<NodePtr>& set1,
                                            const std::list<NodePtr>& set2) const
    {
        std::list<NodePtr> union_set;
        std::set_union(set1.begin(), set1.end(), set2.begin(), set2.end(),
                    std::inserter(union_set, union_set.end()));

        return union_set;
    }

    void RRTX::removeNodeFromTree(const NodePtr& node)
    {
        // this function must is called when setParent w/ nullptr
        // remove from tree node & initialize node-self children
        auto itr = tree_nodes_.find(node);
        if (itr != tree_nodes_.end())
            tree_nodes_.erase(itr);

        // node->removeAllChildren();
    }

    void RRTX::addNodeToTree(const NodePtr& node)
    {
        // this function is called when setParent w/o nullptr
        tree_nodes_.insert(node);
    }

    bool RRTX::isNodeInLocalCostMap(const NodePtr& v, const NodePtr& node) const
    {
        double dx = node->getX() - v->getX();
        double dy = node->getY() - v->getY();
        double distance = std::hypot(dx, dy);

        // local_half_width_ and local_half_height_ should be same
        if (distance <= local_half_width_)
            return true;
        return false;
    }

    bool RRTX::isInNodeSet(const NodePtr& node) const
    {
        auto itr = node_set_.find(node);
        if (itr != node_set_.end())
            return true;
        return false;
    }

    bool RRTX::isPathValid() const
    {
        return path_to_goal_;
    }

    // below functions must be called from outside
    void RRTX::setStateValidityChecker(const StateValidityCheckerFn& svc)
    {
        isStateValid_ = svc;
    }
    
    // below functions must be called from outside
    void RRTX::setEdgeCollsionChecker(const EdgeValidityCheckerFn& evc)
    {
        isCollision_ = evc;
    }

    void RRTX::setRobotBias(double robot_bias)
    {
        robot_bias_ = robot_bias;
    }

    void RRTX::setDelta(double delta)
    {
        delta_ = delta;
        ball_constant_ = delta * 10;
    }

    void RRTX::setGlobalWidth(double low_x, double high_x)
    {
        global_width_.first = low_x;
        global_width_.second = high_x;
    }

    void RRTX::setGlobalHeigth(double low_y, double high_y)
    {
        global_height_.first = low_y;
        global_height_.second = high_y;
    }

    void RRTX::setLocalCostMapSize(double local_width, double local_height)
    {
        local_half_width_ = local_width / 2.0;
        local_half_height_ = local_height / 2.0;
    }

    void RRTX::setGoal(double goal_x, double goal_y)
    {
        goal_node_->setX(goal_x);
        goal_node_->setY(goal_y);
        goal_node_->setLMC(0.0);
        goal_node_->setCostToGoal(0.0);
    }

    void RRTX::setStart(double start_x, double start_y)
    {
        start_node_->setX(start_x);
        start_node_->setY(start_y);
        start_node_->setLMC(std::numeric_limits<double>::infinity());
        start_node_->setCostToGoal(std::numeric_limits<double>::infinity());
    }

    void RRTX::initialize()
    {
        addNode(goal_node_);
        robot_node_ = start_node_;
    }

    void RRTX::updateRobotPosition(double robot_x, double robot_y)
    {
        robot_x_ = robot_x;
        robot_y_ = robot_y;
    }

    NodePtr RRTX::findTargetNode(double robot_x, double robot_y) const
    {
        NodePtr target_node;
        double min_distance = std::numeric_limits<double>::infinity();
        for (const auto& u : tree_nodes_)
        {
            double distance = u->getLMC() + std::hypot(u->getX() - robot_x, u->getY() - robot_y);
            if (distance < min_distance)
                min_distance = distance;
                target_node = u;
        }

        return target_node;
    }

    NodePtr RRTX::nearestLMC(const double robot_x, const double robot_y) const
    {
        NodePtr nearest_from_robot;
        double min_LMC = std::numeric_limits<double>::infinity();
        for (const auto& u : tree_nodes_)
        {
            double distance = std::hypot(robot_x - u->getX(), robot_y - u->getY());
            if (distance > hyper_ball_radius_)
                continue;
            
            double LMC = distance + u->getLMC();
            if (LMC < min_LMC)
            {
                min_LMC = LMC;
                nearest_from_robot = u;
            }
        }
        return nearest_from_robot;
    }

    std::list<std::pair<double, double>> RRTX::getPath() const
    {
        std::list<std::pair<double, double>> path;
        path.push_back(std::make_pair(robot_x_, robot_y_));

        NodePtr this_node = nearestLMC(robot_x_, robot_y_);
        while (this_node != goal_node_)
        {
            path.push_back(std::make_pair(this_node->getX(), this_node->getY()));
            std::weak_ptr<Node> unlock_parent = this_node->getParent();
            this_node = unlock_parent.lock();
        }
        path.push_back(std::make_pair(goal_node_->getX(), goal_node_->getY()));

        return path;
    }

    // std::list<std::pair<double, double>> RRTX::getPath() const
    // {
    //     std::list<std::pair<double, double>> path;
    //     path.push_back(std::make_pair(robot_x_, robot_y_));

    //     NodePtr this_node = robot_node_;
    //     while (this_node != goal_node_)
    //     {
    //         path.push_back(std::make_pair(this_node->getX(), this_node->getY()));
    //         std::weak_ptr<Node> unlock_parent = this_node->getParent();
    //         this_node = unlock_parent.lock();
    //     }
    //     path.push_back(std::make_pair(goal_node_->getX(), goal_node_->getY()));

    //     return path;
    // }

    const std::list<NodePtr> RRTX::getNodeSetWithOutOrphans() const
    {
        return computeDifferenceSet(node_set_, orphan_nodes_);
    }

    const std::set<NodePtr>& RRTX::getNodeSet() const
    {
        return node_set_;
    }

    const std::set<NodePtr>& RRTX::getTreeNodes() const
    {
        return tree_nodes_;
    }

    const NodePtr& RRTX::getGoalNode() const
    {
        return goal_node_;
    }
}