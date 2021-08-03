#include <chrono>
#include <queue>
#include <unordered_set>
#include <boost/unordered_set.hpp>

#include "CBSHSearch.h"
#include "log.h"
#include "LLNode.h"
#include "cbsh_config.h"

namespace mapf{
    namespace CBSH {

        CBSHSearch::CBSHSearch(const Map::ConstPtr &map, const std::vector<Agent::Ptr> &agents, bool block)
            : map_(map), HL_expend_num_(0), block_(block), solution_cost_(-2),
              cost_upperbound_(std::numeric_limits<int>::max()), rl_done_(false)
        {
            auto root_s = std::chrono::system_clock::now();
            std::vector<int> starts;
            std::vector<int> goals;
            // 计算astar h
            for(auto a: agents) {
                ComputeH(a->GetGoal(), a->GetId());
                starts.push_back(a->GetStart());
                goals.push_back(a->GetGoal());
                agent_ids_.push_back(a->GetId());
            }
            auto mp_ch_e = std::chrono::system_clock::now();

            CBSHConfig::Ptr cbsh_config;
            cbsh_config.reset(new CBSHConfig());
            strategy_ = cbsh_config->GetStrategy();
            focal_w_ = cbsh_config->GetFocal();
            rectangle_reasoning_ = cbsh_config->GetRectangle();

            open_list_.clear();
            focal_list_.clear();

            // 规划根节点
            // 构造根节点时进行初始规划、识别冲突
            root_.reset(new CBSHNode(agent_ids_, starts, goals, strategy_, rectangle_reasoning_,
                map_, mddtable_, astar_h_, block_));

            root_->open_handle_ = open_list_.push(root_);
            root_->focal_handle_ = focal_list_.push(root_);
            min_f_cost_ = root_->GetTotalCost();
            focal_list_threshold_ = min_f_cost_ * focal_w_;

            LOG_DEBUG_STREAM("High level build root.");
            auto mp_root_d = std::chrono::duration_cast<std::chrono::microseconds>(mp_ch_e - root_s);
            compute_astarh_time_ = double(mp_root_d.count()) * std::chrono::microseconds::period::num /
            std::chrono::microseconds::period::den;
        }

        CBSHSearch::CBSHSearch(const Map::ConstPtr &map, const std::vector<std::string> &agent_ids,
        std::map<std::string, CBSHPath> init_paths, double f_w, int init_h, std::string strategy,
        bool rectangle_reasoning, int cost_upperbound, double time_limit, bool block)
            : map_(map), strategy_(strategy), agent_ids_(agent_ids),
              HL_expend_num_(0), focal_w_(f_w), rectangle_reasoning_(rectangle_reasoning),
              block_(block), cost_upperbound_(cost_upperbound), time_limit_(time_limit),
              solution_cost_(-2)
        {
            // 计算astar h
            for(auto p: init_paths) {
                std::string a = p.first;
                ComputeH(p.second.GetGoalLoc(), a);
            }

            open_list_.clear();
            focal_list_.clear();

            // 规划根节点
            // 构造根节点时进行初始规划、识别冲突
            root_.reset(new CBSHNode(agent_ids_, strategy_, rectangle_reasoning_,
                map_, mddtable_, astar_h_, block_, init_paths, init_h));

            root_->open_handle_ = open_list_.push(root_);
            root_->focal_handle_ = focal_list_.push(root_);
            min_f_cost_ = root_->GetTotalCost();
            focal_list_threshold_ = min_f_cost_ * focal_w_;
        }

        void CBSHSearch::Reset() {
            LOG_DEBUG_STREAM("Reset state");
            open_list_.clear();
            focal_list_.clear();
            curr_node_ = root_;
            min_f_cost_ = root_->GetTotalCost();
            focal_list_threshold_ = min_f_cost_ * focal_w_;
            if (curr_node_->GetCollisionNum() == 0) rl_done_ = true;
            else rl_done_ = false;
        }

        void CBSHSearch::Step(int a, int t) {
            //if (!IsCons(a, t)) return false;
            std::string conf_agent = agent_ids_.at(a);
            int loc = curr_node_->GetPaths().at(conf_agent).GetLoc(t);
            // 扩展左右节点
            CBSHNode::Ptr next;

            next.reset(new CBSHNode(curr_node_, conf_agent));
            LOG_DEBUG_STREAM("Conflict loc1: " << loc << "; timestep: " << t);
            Constraint cons(conf_agent, "vertex");
            cons.SetConstraint(loc, -1, t);
            next->AddConstraint(conf_agent, cons);

            BuildChild(next, conf_agent, curr_node_->GetConflictGraph());
            LOG_DEBUG_STREAM("Finish build left child. Agent id: " << conf_agent);
            UpdateFocalList();

            curr_node_ = focal_list_.top();
            focal_list_.pop();
            open_list_.erase(curr_node_->open_handle_);
            if (curr_node_->GetCollisionNum() == 0) { // 无冲突，规划完成
                solution_cost_ = curr_node_->GetGCost();
                rl_done_ = true;
            }
        }

        bool CBSHSearch::IsCons(int a, int t) const { // TODO:
            auto path = curr_node_->GetPaths();
            std::string id1 = agent_ids_[a], id2;
            if (path.at(id1).GetLoc(t) == path.at(id2).GetLoc(t) ||
                (path.at(id1).GetLoc(t - 1) == path.at(id2).GetLoc(t) &&
                path.at(id1).GetLoc(t) == path.at(id2).GetLoc(t - 1))) {
                return true;
            }
            return false;
        }

        std::vector<std::vector<int> > CBSHSearch::GetState() const {
            auto map = map_->GetMap();
            std::vector<std::vector<int> > res;
            res.push_back(map);
            for (auto a: agent_ids_) {
                auto path = curr_node_->GetPaths().at(a).GetPaths();
                for (int i = 0; i < path.size(); ++i) {
                    //
                }
            }
        }

        bool CBSHSearch::MakePlan() {
            auto mp_s = std::chrono::system_clock::now();

            while(!focal_list_.empty()){
                if(min_f_cost_ >= cost_upperbound_) {
                    solution_cost_ = min_f_cost_;
                    return false;
                }
                CBSHNode::Ptr current_node = focal_list_.top();
                focal_list_.pop();
                open_list_.erase(current_node->open_handle_);

                if(current_node->GetCollisionNum() == 0){ // 无冲突，规划完成
                    solution_cost_ = current_node->GetGCost();
                    RecordPlan(current_node);
                    auto mp_e = std::chrono::system_clock::now();
                    auto mp_d = std::chrono::duration_cast<std::chrono::microseconds>(mp_e - mp_s);
                    LOG_DEBUG_STREAM("Finish make plan, time: " << double(mp_d.count()) *
                    std::chrono::microseconds::period::num / std::chrono::microseconds::period::den);
                    LOG_DEBUG_STREAM("Generate High Level nodes: " << HL_expend_num_);
                    LOG_DEBUG_STREAM("High level root, time: " << compute_astarh_time_);
                    LOG_DEBUG_STREAM("G cost: " << current_node->GetGCost());
                    LOG_DEBUG_STREAM("Makespan: " << current_node->GetMakespan());
                    return true;
                }
                else if(strategy_ == "NONE") {
                    current_node->ClassifyConflicts();
                    current_node->ChooseConflict();
                }
                else if(!current_node->HasComputeH()){
                    current_node->ClassifyConflicts();
                    // 计算冲突启发式函数，更新节点cost
                    int h = ComputeHeuristics(current_node);
                    if(h < 0){  // no solution
                        UpdateFocalList();
                        continue;
                    }
                    else {
                        current_node->UpdateCost(h);
                    }
                    current_node->ChooseConflict();
                    if(current_node->GetTotalCost() > focal_list_threshold_){
                        current_node->open_handle_ = open_list_.push(current_node);
                        UpdateFocalList();
                        continue;
                    }
                }

                // 扩展左右节点
                CBSHNode::Ptr left;
                CBSHNode::Ptr right;
                ++HL_expend_num_;

                // 生成限制
                Conflict curr_conf = current_node->GetLastestConflict();
                LOG_DEBUG_STREAM("Conflict type: " << curr_conf.Type());
                std::string conf_agent1 = curr_conf.GetAgent(0);
                std::string conf_agent2 = curr_conf.GetAgent(1);
                left.reset(new CBSHNode(current_node, conf_agent1));
                right.reset(new CBSHNode(current_node, conf_agent2));
                if(curr_conf.Type() == "rectangle") {
                    Constraint cons1(conf_agent1, "rectangle");
                    Constraint cons2(conf_agent2, "rectangle");
                    std::pair<int, int> Rg = curr_conf.GetRg();
                    int s1_time = curr_conf.GetRectTime(0);
                    int s2_time = curr_conf.GetRectTime(1);
                    LOG_DEBUG_STREAM("Rectangle conflict Rg: " << Rg.first << ", " << Rg.second << "; s1 time: " <<
                    s1_time << "; s2 time: " << s2_time);
                    MDD::Ptr mdd1 = current_node->BuildMDD(conf_agent1);
                    MDD::Ptr mdd2 = current_node->BuildMDD(conf_agent2);
                    AddModifiedBarrierCons(current_node->GetPath(conf_agent1).GetPaths(),
                    current_node->GetPath(conf_agent2).GetPaths(), mdd1, mdd2, s1_time, s2_time, Rg, cons1, cons2);
                    left->AddConstraint(conf_agent1, cons1);
                    right->AddConstraint(conf_agent2, cons2);
                }
                else if(curr_conf.Type() == "vertex"){
                    LOG_DEBUG_STREAM("Vertex conflict loc: " << curr_conf.GetLoc(0) << "; timestep: " << curr_conf.GetTimestep());
                    Constraint cons1(conf_agent1, "vertex");
                    Constraint cons2(conf_agent2, "vertex");
                    cons1.SetConstraint(curr_conf.GetLoc(0), -1, curr_conf.GetTimestep());
                    left->AddConstraint(conf_agent1, cons1);
                    cons2.SetConstraint(curr_conf.GetLoc(0), -1, curr_conf.GetTimestep());
                    right->AddConstraint(conf_agent2, cons2);
                }
                else {
                    LOG_DEBUG_STREAM("Edge conflict loc1: " << curr_conf.GetLoc(0) << "; loc2: " << curr_conf.GetLoc(1) <<
                    "; timestep: " << curr_conf.GetTimestep());
                    Constraint cons1(conf_agent1, "edge");
                    Constraint cons2(conf_agent2, "edge");
                    cons1.SetConstraint(curr_conf.GetLoc(0), curr_conf.GetLoc(1), curr_conf.GetTimestep());
                    left->AddConstraint(conf_agent1, cons1);
                    cons2.SetConstraint(curr_conf.GetLoc(1), curr_conf.GetLoc(0), curr_conf.GetTimestep());
                    right->AddConstraint(conf_agent2, cons2);
                }

                BuildChild(left, conf_agent1, current_node->GetConflictGraph());
                LOG_DEBUG_STREAM("Finish build left child. Agent id: " << conf_agent1);
                BuildChild(right, conf_agent2, current_node->GetConflictGraph());
                LOG_DEBUG_STREAM("Finish build left child. Agent id: " << conf_agent2);

                UpdateFocalList();
            } // end of while loop
            return false;
        }

        std::vector<Agent::Ptr> CBSHSearch::GetPlan() {
            return plan_result_;
        }

        bool CBSHSearch::BuildChild(CBSHNode::Ptr &node, std::string cons_agent,
                                    const std::map<std::pair<std::string, std::string>, int> &conflict_graph) {
            auto bc_s = std::chrono::system_clock::now();
            int lower_bound;
            Conflict parent_conf = node->GetLastestConflict();

            if (parent_conf.Type() == "rectangle") {
                lower_bound = 0;
            } else if (parent_conf.GetTimestep() >= node->GetPath(cons_agent).Size()) {
                lower_bound = parent_conf.GetTimestep() + 1;
            } else {
                lower_bound = node->GetPath(cons_agent).Size() - 1;
            }

            if (!node->LLPlan(cons_agent, lower_bound)) return false;
            // 评估h cost
            node->EstimateH(cons_agent, conflict_graph);
            node->FindConflict(cons_agent);
            node->CopyConflictGraph(conflict_graph);

            node->open_handle_ = open_list_.push(node);
            if (node->GetTotalCost() <= focal_list_threshold_) {
                node->focal_handle_ = focal_list_.push(node);
            }
            auto bc_e = std::chrono::system_clock::now();
            auto bc_d = std::chrono::duration_cast<std::chrono::microseconds>(bc_e - bc_s);
            LOG_DEBUG_STREAM("Finish build child, time: " << double(bc_d.count()) *
            std::chrono::microseconds::period::num / std::chrono::microseconds::period::den);
            return true;
        }

        void CBSHSearch::RecordPlan(const CBSHNode::Ptr &node) {
            plan_result_.clear();
            for(auto path: node->GetPaths()) {
                Agent::Ptr a;
                a.reset(new Agent(path.first));
                a->SetStart(path.second.GetStartLoc());
                a->SetGoal(path.second.GetGoalLoc());
                a->SetPaths(path.second.GetPaths());
                plan_result_.push_back(a);

                LOG_DEBUG_STREAM("Agent id: " << path.first);
                for(auto loc: path.second.GetPaths()) {
                    std::pair<int, int> l;
                    l = map_->ToXY(loc.first);
                    LOG_DEBUG_STREAM(l.first << ", " << l.second);
                }
            }
        }

        // 向focal list中添加新node（处于新旧focal_list_threshold之间）
        // 更新focal list threshold
        void CBSHSearch::UpdateFocalList() {
            float new_f_cost = open_list_.top()->GetTotalCost();
            if(new_f_cost > min_f_cost_) {
                min_f_cost_ = new_f_cost;
                float new_focal_list_threshold = min_f_cost_ *focal_w_;
                for(auto &n: open_list_) {
                    float n_f_cost = n->GetTotalCost();
                    if(n_f_cost > focal_list_threshold_ && n_f_cost <= new_focal_list_threshold) {
                        n->focal_handle_ = focal_list_.push(n);
                    }
                }
                focal_list_threshold_ = new_focal_list_threshold;
            }
        }

        void CBSHSearch::AddModifiedBarrierCons(std::vector<std::pair<int, bool> > path1,
            std::vector<std::pair<int, bool> > path2, MDD::Ptr mdd1,
            MDD::Ptr mdd2, int s1, int s2, std::pair<int, int> Rg, Constraint &cons1, Constraint &cons2){
            std::pair<int, int> s1_cor = map_->ToYX(path1[s1].first);
            std::pair<int, int> s2_cor = map_->ToYX(path2[s2].first);
            int Rg_t = s1 + abs(Rg.first - s1_cor.first) + abs(Rg.second - s1_cor.second);

            std::pair<int, int> R1, R2;
            if((s1_cor.first == s2_cor.first && (s1_cor.second - s2_cor.second) * (s2_cor.second - Rg.second) < 0) ||
            (s1_cor.first != s2_cor.first && (s1_cor.first - s2_cor.first) * (s2_cor.first - Rg.first) >= 0)) {
                R1 = std::make_pair(Rg.first, s1_cor.second);
                R2 = std::make_pair(s2_cor.first, Rg.second);
                AddModifiedBarrierConsH(mdd1, Rg, R1, Rg_t, cons1);
                AddModifiedBarrierConsV(mdd2, Rg, R2, Rg_t, cons2);
            }
            else {
                R1 = std::make_pair(s1_cor.first, Rg.second);
                R2 = std::make_pair(Rg.first, s2_cor.second);
                AddModifiedBarrierConsV(mdd1, Rg, R1, Rg_t, cons1);
                AddModifiedBarrierConsH(mdd2, Rg, R2, Rg_t, cons2);
            }
        }

        void CBSHSearch::AddModifiedBarrierConsH(MDD::Ptr mdd, std::pair<int, int> Rg, std::pair<int, int> R,
                int Rg_t, Constraint &cons){
            int sign = R.second < Rg.second ? 1: -1;
            int Rt = Rg_t - abs(R.second - Rg.second);
            int t1 = -1;
            for(int t2 = Rt; t2 <= Rg_t; ++t2) {
                int loc = R.second + (t2 - Rt) * sign + Rg.first * map_->GetWidth();
                MDDNode::Ptr it = nullptr;
                for(MDDNode::Ptr n: mdd->GetLevel(t2)){
                    if(n->GetLoc() == loc) {
                        it = n;
                        break;
                    }
                }

                if(it == nullptr && t1 >= 0) {
                    int loc1 = R.second + (t1 - Rt) * sign + Rg.first * map_->GetWidth();
                    int loc2 = R.second + (t2 - 1 - Rt) * sign + Rg.first * map_->GetWidth();
                    cons.SetConstraint(loc1, loc2, t2 - 1);
                    t1 = -1;
                    continue;
                }
                else if(it != nullptr && t1 < 0) {
                    t1 = t2;
                }

                if(it != nullptr && t2 == Rg_t) {
                    int loc1 = R.second + (t1 - Rt) * sign + Rg.first * map_->GetWidth();
                    cons.SetConstraint(loc1, loc, t2);
                }
            }
        }

        void CBSHSearch::AddModifiedBarrierConsV(MDD::Ptr mdd, std::pair<int, int> Rg, std::pair<int, int> R,
            int Rg_t, Constraint &cons){
            int sign = R.first < Rg.first ? 1 : -1;
            int Rt = Rg_t - abs(R.first - Rg.first);
            int t1 = -1;
            for(int t2 = Rt; t2 <= Rg_t; ++t2) {
                int loc = (R.first + (t2 - Rt) * sign) * map_->GetWidth() + Rg.second;
                MDDNode::Ptr it = nullptr;
                for(MDDNode::Ptr n: mdd->GetLevel(t2)) {
                    if(n->GetLoc() == loc) {
                        it = n;
                        break;
                    }
                }

                if(it == nullptr && t1 >= 0) {
                    int loc1 = (R.first + (t1 - Rt) * sign) * map_->GetWidth() + Rg.second;
                    int loc2 = (R.first + (t2 - 1 - Rt) * sign) * map_->GetWidth() + Rg.second;
                    cons.SetConstraint(loc1, loc2, t2 - 1);
                    t1 = -1;
                    continue;
                }
                else if(it != nullptr && t1 < 0) {
                    t1 = t2;
                }

                if(it != nullptr && t2 == Rg_t) {
                    int loc1 = (R.first + (t1 - Rt) * sign) * map_->GetWidth() + Rg.second;
                    cons.SetConstraint(loc1, loc, t2);
                }
            }
        }

        void CBSHSearch::ComputeH(int goal_loc, std::string agent_id){
            int map_size = map_->GetMapSize();
            std::vector<int> temp_h(map_size, std::numeric_limits<int>::max());

            boost::heap::fibonacci_heap<LLNode::Ptr, boost::heap::compare<LLNode::Open_compare> > open_list;
            boost::unordered_set<LLNode::Ptr, LLNode::NodeHasher, LLNode::Eqnode> nodes;

            LLNode::Ptr root;
            root.reset(new LLNode(goal_loc, 0, 0, nullptr, 0, 0));
            root->open_handle_ = open_list.push(root);
            nodes.insert(root);
            std::vector<int> moveoffset = map_->GetMoveOffset();
            while(!open_list.empty()) {
                LLNode::Ptr curr_node = open_list.top();
                open_list.pop();

                for(int i = 0; i < 4; ++i) {
                    int curr_loc = curr_node->GetLoc();
                    int next_loc = curr_loc + moveoffset[i];
                    if(block_ && map_->IsBlocked(next_loc)) {
                        continue;
                    }
                    if(map_->ValidMove(curr_loc, next_loc)) {
                        int next_g_cost = curr_node->GetGCost() + 1;
                        LLNode::Ptr next;
                        next.reset(new LLNode(next_loc, next_g_cost, 0, nullptr, 0, 0));
                        auto iter = nodes.find(next);
                        if(iter == nodes.end()) {
                            next->open_handle_ = open_list.push(next);
                            nodes.insert(next);
                        }
                        else {
                            next = *iter;
                            if(next->GetGCost() > next_g_cost) {
                                next->SetGCost(next_g_cost);
                                open_list.update(next->open_handle_);
                            }
                        }
                    }
                } // end for loop
            } // end while loop
            for(auto iter = nodes.begin(); iter != nodes.end(); ++iter) {
                temp_h[iter->get()->GetLoc()] = iter->get()->GetGCost();
            }
            astar_h_[agent_id] = temp_h;
        }

        int CBSHSearch::ComputeHeuristics(CBSHNode::Ptr node) {
            node->SetComputeH(true);
            int agent_num = agent_ids_.size();
            int edge_num = 0;
            std::map<std::pair<std::string, std::string>, int> conf_graph;
            if(strategy_ == "CG") {  // 使用CG
                auto cardinal_conf = node->GetCardinalConf();
                for(auto iter = cardinal_conf.begin(); iter != cardinal_conf.end(); ++iter) {
                    std::string agent_id1 = iter->GetAgent(0);
                    std::string agent_id2 = iter->GetAgent(1);
                    auto iter_cg = conf_graph.find(std::make_pair(agent_id1, agent_id2));
                    if(iter_cg == conf_graph.end() || iter_cg->second == 0) {
                        conf_graph[std::make_pair(agent_id1, agent_id2)] = 1;
                        conf_graph[std::make_pair(agent_id2, agent_id1)] = 1;
                        ++edge_num;
                    }
                }
            }
            else {  // 使用DG或WDG
                if(!BuildDependenceGraph(node)) {
                    return -1;
                }
                for(const auto a1: agent_ids_) {
                    for(const auto a2: agent_ids_) {
                        auto conflict_graph = node->GetConflictGraph();
                        auto iter = conflict_graph.find(std::make_pair(a1, a2));
                        if(iter != conflict_graph.end() && iter->second > 0) {
                            conf_graph[std::make_pair(a1, a2)] = iter->second;
                            conf_graph[std::make_pair(a2, a1)] = iter->second;
                            ++edge_num;
                        }
                    }
                }
            }

            int result;
            if(strategy_ == "WDG") {
                result = node->WeightedVertexCorver(conf_graph);
            }
            else {
                result = node->MinimumVertexCover(conf_graph, edge_num);
            }
            return result;
        }

        // 每条边代表agent之间有冲突，DG用1表示，WDG计算权重
        bool CBSHSearch::BuildDependenceGraph(CBSHNode::Ptr node) {
            for(auto conf: node->cardinal_conf_) {
                std::string agent_id1 = conf.GetAgent(0) < conf.GetAgent(1) ? conf.GetAgent(0): conf.GetAgent(1);
                std::string agent_id2 = conf.GetAgent(0) < conf.GetAgent(1) ? conf.GetAgent(1): conf.GetAgent(0);
                if(strategy_ == "DG") {
                    node->conflict_graph_[std::make_pair(agent_id1, agent_id2)] = 1;
                }
                else if (node->conflict_graph_.find(std::make_pair(agent_id1, agent_id2)) == node->conflict_graph_.end()) {
                    std::pair<int, bool> w_hit = GetEdgeWeight(agent_id1, agent_id2, true, node);
                    if(w_hit.first < 0) {   // no solution
                        return false;
                    }
                    node->conflict_graph_[std::make_pair(agent_id1, agent_id2)] = w_hit.first;
                }
            }
            for(auto conf: node->semi_conf_) {
                std::string agent_id1 = conf.GetAgent(0) < conf.GetAgent(1) ? conf.GetAgent(0): conf.GetAgent(1);
                std::string agent_id2 = conf.GetAgent(0) < conf.GetAgent(1) ? conf.GetAgent(1): conf.GetAgent(0);
                if (node->conflict_graph_.find(std::make_pair(agent_id1, agent_id2)) == node->conflict_graph_.end()) {
                    std::pair<int, bool> w_hit = GetEdgeWeight(agent_id1, agent_id2, false, node);
                    if(w_hit.first < 0) {   // no solution
                        return false;
                    }
                    node->conflict_graph_[std::make_pair(agent_id1, agent_id2)] = w_hit.first;
                }
            }
            for(auto conf: node->non_conf_) {
                std::string agent_id1 = conf.GetAgent(0) < conf.GetAgent(1) ? conf.GetAgent(0): conf.GetAgent(1);
                std::string agent_id2 = conf.GetAgent(0) < conf.GetAgent(1) ? conf.GetAgent(1): conf.GetAgent(0);
                if (node->conflict_graph_.find(std::make_pair(agent_id1, agent_id2)) == node->conflict_graph_.end()) {
                    std::pair<int, bool> w_hit = GetEdgeWeight(agent_id1, agent_id2, false, node);
                    if(w_hit.first < 0) {   // no solution
                        return false;
                    }
                    node->conflict_graph_[std::make_pair(agent_id1, agent_id2)] = w_hit.first;
                }
            }
            return true;
        }

        std::pair<int, bool> CBSHSearch::GetEdgeWeight(std::string agent1, std::string agent2, bool cardinal,
        CBSHNode::Ptr node) {
            std::string a1 = agent1 < agent2 ? agent1: agent2;
            std::string a2 = agent1 < agent2 ? agent2: agent1;
            std::set<Constraint> s1, s2;
            for(auto c_step: node->paths_.at(a1).GetConstraints()) {
                for(auto c: c_step){
                    Constraint con(c);
                    s1.insert(con);
                }
            }
            for(auto c_step: node->paths_.at(a2).GetConstraints()) {
                for(auto c: c_step){
                    Constraint con(c);
                    s2.insert(c);
                }
            }
            Htable h1(a1, s1);
            Htable h2(a2, s2);
            if(strategy_ != "NONE") {
                auto iter = htable_.find(h1);
                if(iter != htable_.end()) {
                    auto iter2 = iter->second.find(h2);
                    if(iter2 != iter->second.end()) {
                        return std::make_pair(iter2->second, true);
                    }
                }
            }
            int result = 0;
            if(cardinal) {
                result = 1;
            }
            else if (strategy_ == "DG" || strategy_ == "WDG") {
                MDD::Ptr mdd1 = node->BuildMDD(a1);
                MDD::Ptr mdd2 = node->BuildMDD(a2);
                if(mdd1->GetLevelSize() > mdd2->GetLevelSize()){
                    MDD::Ptr temp = mdd1;
                    mdd1 = mdd2;
                    mdd2 = temp;
                }

                if(!node->SyncMDDs(*mdd1, *mdd2)) {
                    result = 1;
                }
                else{
                    result = 0;
                }
            }

            if(strategy_ == "WDG" && result > 0) { // WDG
                std::vector<std::string> agentids = {agent1, agent2};
                std::map<std::string, CBSHPath> initpaths;
                initpaths[agent1] = node->GetPath(agent1);
                initpaths[agent2] = node->GetPath(agent2);
                int cost_shortestpath = initpaths[agent1].Size() + initpaths[agent2].Size() - 2;
                int upperbound = initpaths[agent1].Size() + initpaths[agent2].Size() + 10;
                CBSHSearch temp_plan(map_, agentids, initpaths, 1.0, std::max(result, 0), "DG",
                rectangle_reasoning_, upperbound, 0.0, block_ );
                temp_plan.MakePlan();
                //TODO:超时处理
                if(temp_plan.solution_cost_ < 0) {
                    result = temp_plan.solution_cost_;
                }
                else {
                    result = temp_plan.solution_cost_ - cost_shortestpath;
                }
            }

            htable_[h1][h2] = result;
            return std::make_pair(result, false);
        }

    } // namespace CBSH
} // namespace mapf