#include <chrono>

#include "CBSHNode.h"
// #include "log.h"

namespace mapf {
    namespace CBSH {

        CBSHNode::CBSHNode(const std::vector<std::string> &agent_ids, const std::vector<int> &starts,
                const std::vector<int> &goals, std::string strategy,
                const bool &rectangle_reasoning, const Map::ConstPtr &map,
                std::map<MDDTable, MDD::Ptr> &mddtable,
                std::unordered_map<std::string, std::vector<int> > &astar_h,
                const bool &block)
                : makespan_(0), total_cost_(0), node_g_cost_(0), node_h_cost_(0),
                  agent_ids_(agent_ids), strategy_(strategy),
                  rectangle_reasoning_(rectangle_reasoning),
                  map_(map), parent_h_cost_(-1), mddtable_(mddtable),
                  depth_(0), astar_h_(astar_h), block_(block), has_compute_h_(false)
        {
            // 初始规划
            int agent_num = agent_ids.size();
            conflict_graph_.clear();
            for(int i = 0; i < agent_num; ++i) {
                CBSHPath path(agent_ids[i], starts[i], goals[i]);
                paths_[agent_ids[i]] = path;
            }
            for(int i = 0; i < agent_num; ++i) {
                LLPlan(agent_ids[i], 0);
            }
            total_cost_ = node_g_cost_;

            // 识别冲突
            FindConflictRoot();
        }

        CBSHNode::CBSHNode(const std::vector<std::string> &agent_ids, std::string strategy,
                const bool &rectangle_reasoning, const Map::ConstPtr &map,
                std::map<MDDTable, MDD::Ptr> &mddtable,
                std::unordered_map<std::string, std::vector<int> > &astar_h,
                const bool &block, std::map<std::string, CBSHPath> init_paths,
                int init_h)
                : makespan_(0), total_cost_(0), node_g_cost_(0), node_h_cost_(init_h),
                  agent_ids_(agent_ids), strategy_(strategy),
                  rectangle_reasoning_(rectangle_reasoning),
                  map_(map), parent_h_cost_(-1), mddtable_(mddtable),
                  depth_(0), astar_h_(astar_h), block_(block), has_compute_h_(false)
        {
            paths_.clear();
            for(auto path: init_paths) {
                paths_[path.first] = path.second;
                makespan_ = std::max(makespan_, path.second.Size() - 1);
                node_g_cost_ += path.second.Size() - 1;
            }
            total_cost_ = node_g_cost_;
            depth_ = 0;
            FindConflictRoot();
        }

        CBSHNode::CBSHNode(CBSHNode::Ptr &node, std::string cons_agent)
            : makespan_(node->makespan_),
              total_cost_(node->total_cost_), node_g_cost_(node->node_g_cost_), node_h_cost_(node->node_h_cost_), // cost复制父节点
              agent_ids_(node->agent_ids_), paths_(node->paths_),
              num_of_collisions_(node->num_of_collisions_), conflict_graph_(node->conflict_graph_),
              strategy_(node->strategy_),
              rectangle_reasoning_(node->rectangle_reasoning_),
              map_(node->map_), parent_h_cost_(node->node_h_cost_),
              mddtable_(node->mddtable_),
              conflicts_(node->conflicts_), astar_h_(node->astar_h_),
              block_(node->block_), has_compute_h_(false)
        {
            depth_ = node->depth_ + 1;
            constraint_agent_ = cons_agent;
            CopyList(unknown_conf_, node->unknown_conf_);
            CopyList(cardinal_conf_, node->cardinal_conf_);
            CopyList(rectSemi_conf_, node->rectSemi_conf_);
            CopyList(semi_conf_, node->semi_conf_);
            CopyList(rectNon_conf_, node->rectNon_conf_);
            CopyList(non_conf_, node->non_conf_);
            conflict_graph_.clear();
        }

        void CBSHNode::CopyList(std::list<Conflict> & to, const std::list<Conflict> & from) {
            for(const auto c: from) {
                if(c.GetAgent(0) != constraint_agent_ && c.GetAgent(1) != constraint_agent_) {
                    to.push_back(c);
                }
            }
        }

        float CBSHNode::GetTotalCost() const {
            return total_cost_;
        }

        int CBSHNode::GetCollisionNum() const {
            return num_of_collisions_;
        }

        void CBSHNode::FindConflictRoot() {
            for(int a1_idx = 0; a1_idx < agent_ids_.size(); ++a1_idx) {
                std::string a1 = agent_ids_[a1_idx];
                for(int a2_idx = a1_idx + 1; a2_idx < agent_ids_.size(); ++a2_idx) {
                    std::string a2 = agent_ids_[a2_idx];
                    int min_len = std::min(paths_.at(a1).Size(), paths_.at(a2).Size());
                    for(int t = 0; t < min_len; ++t) {
                        int loc1 = paths_.at(a1).GetLoc(t);
                        int loc2 = paths_.at(a2).GetLoc(t);
                        if(loc1 == loc2) {
                            Conflict conf(a1, a2, "vertex", loc1, -1, t);
                            unknown_conf_.push_back(conf);
                        }
                        else if(t < min_len - 1 && loc1 == paths_.at(a2).GetLoc(t + 1) &&
                        loc2 == paths_.at(a1).GetLoc(t + 1)) {
                            Conflict conf(a1, a2, "edge", loc1, loc2, t + 1);
                            unknown_conf_.push_back(conf);
                        }
                    }
                    if(paths_.at(a1).Size() != paths_.at(a2).Size()) {
                        std::string a1_ = paths_.at(a1).Size() < paths_.at(a2).Size() ? a1 : a2;
                        std::string a2_ = paths_.at(a1).Size() < paths_.at(a2).Size() ? a2 : a1;
                        if((a1_ == "85" && a2_ == "99") || (a1_ == "99" && a2_ == "85")) {
                            int pathsize1_d = paths_.at(a1_).Size();
                            int pathsize2_d = paths_.at(a2_).Size();
                            int temp =1;
                        }
                        int loc1 = paths_.at(a1_).GetLastLoc();
                        for(int t = min_len; t < paths_.at(a2_).Size(); ++t) {
                            int loc2 = paths_.at(a2_).GetLoc(t);
                            if(loc1 == loc2) {
                                Conflict conf(a2_, a1_, "vertex", loc1, -1, t);
                                unknown_conf_.push_front(conf);
                            }
                        }
                    }
                }
            }
            num_of_collisions_ = unknown_conf_.size() + cardinal_conf_.size() + rectSemi_conf_.size() +
                semi_conf_.size() + rectNon_conf_.size() + non_conf_.size();
        }

        void CBSHNode::FindConflict(std::string agent_id) {
            for(const auto a: agent_ids_) {
                if(a == agent_id) {
                    continue;
                }
                else if(num_of_collisions_ == 0) {
                    if(paths_.at(a).Size() + 1 < paths_.at(agent_id).Size()) {
                        int loc1 = paths_.at(a).GetLastLoc();
                        for(int t = paths_.at(a).Size(); t < paths_.at(agent_id).Size(); ++t) {
                            int loc2 = paths_.at(agent_id).GetLoc(t);
                            if(loc1 == loc2) {
                                Conflict conf(a, agent_id, "vertex", loc1, -1, t);
                                unknown_conf_.push_front(conf);
                            }
                        }
                    }
                    continue;
                }
                int min_len = std::min(paths_.at(a).Size(), paths_.at(agent_id).Size());
                for(int timestep = 0; timestep < min_len; ++timestep) {
                    int loc1 = paths_.at(agent_id).GetLoc(timestep);
                    int loc2 = paths_.at(a).GetLoc(timestep);
                    if(loc1 == loc2) {
                        Conflict conf(agent_id, a, "vertex", loc1, -1, timestep);
                        unknown_conf_.push_back(conf);
                    }
                    else if (timestep < min_len - 1 && loc1 == paths_.at(a).GetLoc(timestep + 1)
                    && loc2 == paths_.at(agent_id).GetLoc(timestep + 1)) {
                        Conflict conf(agent_id, a, "edge", loc1, loc2, timestep + 1);
                        unknown_conf_.push_back(conf);
                    }
                }
                // 一个agent已抵达目标点，至少为semi conflict
                if(paths_.at(agent_id).Size() < paths_.at(a).Size()) {
                    int loc1 = paths_.at(agent_id).GetLastLoc();
                    for(int timestep = min_len; timestep < paths_.at(a).Size(); ++timestep) {
                        if(loc1 == paths_.at(a).GetLoc(timestep)) {
                            Conflict conf(agent_id, a, "vertex", loc1, -1, timestep);
                            unknown_conf_.push_front(conf);
                        }
                    }
                }
                else if(paths_.at(a).Size() < paths_.at(agent_id).Size()) {
                    int loc1 = paths_.at(a).GetLastLoc();
                    for(int timestep = min_len; timestep < paths_.at(agent_id).Size(); ++timestep) {
                        if(loc1 == paths_.at(agent_id).GetLoc(timestep)) {
                            Conflict conf(a, agent_id, "vertex", loc1, -1, timestep);
                            unknown_conf_.push_front(conf);
                        }
                    }
                }
            }
            num_of_collisions_ = unknown_conf_.size() + cardinal_conf_.size() + rectSemi_conf_.size() +
                semi_conf_.size() + rectNon_conf_.size() + non_conf_.size();
        }

        bool CBSHNode::LLPlan(std::string agent_id, int lower_bound) {
            UpdateConfTable(agent_id);

            auto ll_s = std::chrono::system_clock::now();
            auto constraints = paths_.at(agent_id).GetConstraints();
            int start_loc = paths_.at(agent_id).GetStartLoc();
            if(!constraints.empty()) {
                for(auto con: constraints[0]) {
                    if(con.GetType() == "vertex" && con.GetLoc(0) == start_loc) {
                        return false;
                    }
                }
            }
            int goal_loc = paths_.at(agent_id).GetGoalLoc();
            std::vector<int> mapoffset = map_->GetMoveOffset();

            LLNode::Ptr start;
            start.reset(new LLNode(start_loc, 0, astar_h_.at(agent_id)[start_loc], nullptr, 0, 0));
            start->open_handle_ = open_list_.push(start);
            start->focal_handle_ = focal_list_.push(start);
            start->SetInOpenlist(true);
            all_LL_node_table_.insert(start);
            int min_f_cost = start->GetFCost();
            int min_len = std::max(lower_bound, min_f_cost);    // focal list阈值

            while (!focal_list_.empty()) {
                LLNode::Ptr curr_node = focal_list_.top();
                focal_list_.pop();
                open_list_.erase(curr_node->open_handle_);
                curr_node->SetInOpenlist(false);
                int curr_loc = curr_node->GetLoc();
                int curr_timestep = curr_node->GetTimeStep();

                // 找到规划
                if(curr_loc == goal_loc && curr_timestep >= lower_bound) {
                    // 更新路径
                    std::vector<std::pair<int, bool> > path_result;
                    LLNode::Ptr curr_path_point = curr_node;
                    path_result.resize(curr_timestep + 1);
                    for(int t = curr_timestep; t >= 0; --t) {
                        path_result[t] = std::make_pair(curr_path_point->GetLoc(), false);
                        curr_path_point = curr_path_point->GetParent();
                    }
                    node_g_cost_ = node_g_cost_ - paths_.at(agent_id).Size() + path_result.size();
                    makespan_ = std::max(makespan_, static_cast<int>(path_result.size() - 1));
                    paths_.at(agent_id).UpdatePath(path_result, makespan_);
                    for(auto &p: paths_) {
                        p.second.UpdateConstraints(makespan_);
                    }

                    open_list_.clear();
                    focal_list_.clear();
                    all_LL_node_table_.clear();
                    auto ll_e = std::chrono::system_clock::now();
                    auto ll_d = std::chrono::duration_cast<std::chrono::microseconds>(ll_e - ll_s);
                    auto ll_t = double(ll_d.count()) *
                    std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
                    // LOG_DEBUG_STREAM("Finish low level plan, time: " << double(ll_d.count()) *
                    //                  std::chrono::microseconds::period::num / std::chrono::microseconds::period::den);

                    return true;
                }

                for(int i = 0; i < 5; ++i) {
                    int next_loc = curr_loc + mapoffset[i];
                    int next_timestep = curr_timestep + 1;
                    // 如果该步运动可行，且不受限制
                    if(block_) {    // 对顶货架智能体，只有可行区域可规划，TODO::抵达目标点的判断应在地图中更新
                        if(!(!map_->IsBlocked(next_loc) || next_loc == goal_loc)) {
                            continue;
                        }
                    }
                    if(map_->ValidMove(curr_loc, next_loc) &&
                    !paths_.at(agent_id).IsConstrainted(curr_loc, next_loc, next_timestep)) {
                        // 计算该步代价
                        int next_g_cost = curr_node->GetGCost() + 1;
                        int next_h_cost = astar_h_.at(agent_id)[next_loc];
                        int next_conf_num = curr_node->GetConfNum() + NumOfConflictsForStep(curr_loc, next_loc, next_timestep);

                        LLNode::Ptr next;
                        next.reset(new LLNode(next_loc, next_g_cost, next_h_cost, curr_node, next_timestep, next_conf_num));

                        auto iter = all_LL_node_table_.find(next);
                        if(iter == all_LL_node_table_.end()) {  // 如果没有遍历过该位置
                            next->open_handle_ = open_list_.push(next);
                            next->SetInOpenlist(true);
                            if(next->GetFCost() <= min_len) {
                                next->focal_handle_ = focal_list_.push(next);
                            }
                            all_LL_node_table_.insert(next);
                        }
                        else {  // 如果已经遍历过该位置，进行节点更新
                            LLNode::Ptr exit_node = *iter;
                            if(exit_node->InOpenlist()) {  // next在open list中
                                // 如果新的路线f cost更小，或者f cost一致，冲突更少
                                if(exit_node->GetFCost() > next_g_cost + next_h_cost ||
                                (exit_node->GetFCost() == next_g_cost + next_h_cost && exit_node->GetConfNum() > next_conf_num)) {
                                    bool add_to_focal = false;
                                    bool update_focal = false;
                                    bool update_open = false;
                                    if((next_g_cost + next_h_cost) <= min_len) {
                                        if(exit_node->GetFCost() > min_len) {   // 新路径点在focal list中，旧的不在
                                            add_to_focal = true;
                                        }
                                        else {  // 新路径点在focal list中，旧的也在
                                            update_focal = true;
                                        }
                                    }
                                    if(exit_node->GetFCost() > next_g_cost + next_h_cost) {
                                        update_open = true;
                                    }
                                    exit_node->SetGCost(next_g_cost);
                                    exit_node->SetHCost(next_h_cost);
                                    exit_node->SetParent(curr_node);
                                    exit_node->SetConfNum(next_conf_num);

                                    if(update_open) {
                                        open_list_.increase(exit_node->open_handle_);
                                    }
                                    if(add_to_focal) {
                                        exit_node->focal_handle_ = focal_list_.push(exit_node);
                                    }
                                    if(update_focal) {
                                        focal_list_.update(exit_node->focal_handle_);
                                    }
                                }
                            }
                            else {  // next 不在open list中
                                if(exit_node->GetFCost() > next_g_cost + next_h_cost ||
                                (exit_node->GetFCost() == next_g_cost + next_h_cost && exit_node->GetConfNum() > next_conf_num)) {
                                    exit_node->SetGCost(next_g_cost);
                                    exit_node->SetHCost(next_h_cost);
                                    exit_node->SetParent(curr_node);
                                    exit_node->SetConfNum(next_conf_num);
                                    exit_node->open_handle_ = open_list_.push(exit_node);
                                    exit_node->SetInOpenlist(true);
                                    if(exit_node->GetFCost() <= min_len) {
                                        exit_node->focal_handle_ = focal_list_.push(exit_node);
                                    }
                                }
                            }
                        }
                    }
                }// end for loop

                if(open_list_.size() == 0) {
                    break;
                }
                LLNode::Ptr open_head = open_list_.top();
                if(open_head->GetFCost() > min_f_cost) {
                    min_f_cost = open_head->GetFCost();
                    int new_min_len = std::max(min_len, min_f_cost);
                    for(LLNode::Ptr n: open_list_) {
                        if(n->GetFCost() > min_len && n->GetFCost() <= new_min_len) {
                            n->focal_handle_ = focal_list_.push(n);
                        }
                    }
                    min_len = new_min_len;
                }
            }// end while loop
            return false;
        }

        void CBSHNode::ClassifyConflicts() {
            while (!unknown_conf_.empty()) {
                auto conf = unknown_conf_.front();
                unknown_conf_.pop_front();

                std::string agent_id1 = conf.GetAgent(0);
                std::string agent_id2 = conf.GetAgent(1);
                int timestep = conf.GetTimestep();
                bool cardinal1 = false;
                bool cardinal2 = false;
                if(timestep >= paths_.at(agent_id1).Size()) {
                    cardinal1 = true;
                }
                else if (!paths_.at(agent_id1).GetSingle(0)) {
                    MDD::Ptr mdd;
                    mdd = BuildMDD(agent_id1);
                    int pathsize_d = paths_.at(agent_id1).Size();
                    int mddsize_d = mdd->GetLevelSize();
                    for(int i = 0; i < paths_.at(agent_id1).Size(); ++i){
                        paths_.at(agent_id1).SetSingle(i, mdd->GetLevel(i).size() == 1);
                    }
                }

                if(timestep >= paths_.at(agent_id2).Size()) {
                    cardinal2 = true;
                }
                else if (!paths_.at(agent_id2).GetSingle(0)) {
                    MDD::Ptr mdd;
                    mdd = BuildMDD(agent_id2);
                    for(int i = 0; i < paths_.at(agent_id2).Size(); ++i){
                        paths_.at(agent_id2).SetSingle(i, mdd->GetLevel(i).size() == 1);
                    }
                }

                if(conf.Type() == "edge") {
                    cardinal1 = paths_.at(agent_id1).GetSingle(timestep) &&
                        paths_.at(agent_id1).GetSingle(timestep - 1);
                    cardinal2 = paths_.at(agent_id2).GetSingle(timestep) &&
                        paths_.at(agent_id2).GetSingle(timestep - 1);
                }
                else if (conf.Type() == "vertex") {
                    if(!cardinal1) {
                        cardinal1 = paths_.at(agent_id1).GetSingle(timestep);
                    }
                    if(!cardinal2) {
                        cardinal2 = paths_.at(agent_id2).GetSingle(timestep);
                    }
                }

                if(cardinal1 && cardinal2) {
                    cardinal_conf_.push_back(conf);
                    if(strategy_ == "NONE") {
                        return;
                    }
                    continue;
                }
                else if (rectangle_reasoning_ && conf.Type() == "vertex" &&
                    paths_.at(agent_id1).Size() > timestep &&
                    paths_.at(agent_id2).Size() > timestep) {  // conflict在两agent抵达目标点之前发生
                    //Rectangle reasoning for semi and non cardinal vertex conflicts
                    std::list<int> starts1 = GetStartCandidates(agent_id1, timestep);
                    std::list<int> goals1 = GetGoalCandidates(agent_id1, timestep);
                    std::list<int> starts2 = GetStartCandidates(agent_id2, timestep);
                    std::list<int> goals2 = GetGoalCandidates(agent_id2, timestep);

                    // 尝试所有组合
                    int type = -1;
                    int area = 0;
                    Conflict new_conf;
                    for(int s1: starts1) {
                        for(int g1: goals1) {
                            int s1_loc = paths_.at(agent_id1).GetLoc(s1);
                            int g1_loc = paths_.at(agent_id1).GetLoc(g1);
                            if(!IsManhattanOptimal(s1_loc, g1_loc, g1 - s1)) {
                                continue;
                            }
                            for(int s2: starts2) {
                                for(int g2: goals2) {
                                    int s2_loc = paths_.at(agent_id2).GetLoc(s2);
                                    int g2_loc = paths_.at(agent_id2).GetLoc(g2);
                                    if(!IsManhattanOptimal(s2_loc, g2_loc, g2 - s2)) {
                                        continue;
                                    }
                                    if(!IsRectangleConflict(s1_loc, s2_loc, g1_loc, g2_loc)) {
                                        continue;
                                    }
                                    std::pair<int, int> Rg = GetRg(s1_loc, g1_loc, g2_loc);
                                    std::pair<int, int> Rs = GetRs(s1_loc, s2_loc, g1_loc);
                                    int new_area = (abs(Rs.first - Rg.first) + 1) * (abs(Rs.second - Rg.second) + 1);
                                    int new_type = ClassifyRectangleConflict(s1_loc, s2_loc, g1_loc, g2_loc, Rg);
                                    if(new_type == 1) {
                                        int temp=1;
                                    }
                                    if(new_type > type || (new_type == type && new_area > area)) {
                                        new_conf.SetRectConflict(agent_id1, agent_id2, "rectangle", s1, s2, Rg);
                                        type = new_type;
                                        area = new_area;
                                    }
                                }
                            }
                        }
                    }
                    if(type == 2) {
                        cardinal_conf_.push_back(new_conf);
                        continue;
                    }
                    else if (type == 1 && !FindRectangleConflict(new_conf)) {
                        rectSemi_conf_.push_back(new_conf);
                    }
                    else if (type == 0 && !FindRectangleConflict(new_conf)) {
                        rectNon_conf_.push_back(new_conf);
                    }
                }
                if(cardinal1 ||cardinal2) {
                    semi_conf_.push_back(conf);
                }
                else {
                    non_conf_.push_back(conf);
                }
            }

            // 释放部分不用的冲突，节约存储空间，需按顺序进行处理
            RemoveLowPriorityConflict(cardinal_conf_);
            RemoveLowPriorityConflict(rectSemi_conf_);
            RemoveLowPriorityConflict(semi_conf_);
            RemoveLowPriorityConflict(rectNon_conf_);
            RemoveLowPriorityConflict(non_conf_);
            high_priority_.clear();

        }

        std::list<int> CBSHNode::GetStartCandidates(std::string agent_id, int timestep) {
            std::list<int> starts;
            for(int i = 0; i <= timestep; ++i) {
                int loc1 = paths_.at(agent_id).GetLoc(i);
                int loc2 = paths_.at(agent_id).GetLoc(timestep);
                if(paths_.at(agent_id).GetSingle(i) && IsManhattanOptimal(loc1, loc2, timestep - i)) {
                    starts.push_back(i);
                }
            }
            return starts;
        }

        std::list<int> CBSHNode::GetGoalCandidates(std::string agent_id, int timestep) {
            std::list<int> goals;
            for(int i = paths_.at(agent_id).Size(); i >=timestep; --i) {
                int loc1 = paths_.at(agent_id).GetLoc(i);
                int loc2 = paths_.at(agent_id).GetLoc(timestep);
                if(paths_.at(agent_id).GetSingle(i) && IsManhattanOptimal(loc1, loc2, i - timestep)) {
                    goals.push_back(i);
                }
            }
            return goals;
        }

        bool CBSHNode::IsManhattanOptimal(int loc1, int loc2, int dt) {
            int manhattan = abs(loc1 / map_->GetWidth() - loc2 / map_->GetWidth()) +
            abs(loc1 % map_->GetWidth() - loc2 % map_->GetWidth());
            return manhattan == dt;
        }

        bool CBSHNode::IsRectangleConflict(int start_loc1, int start_loc2, int goal_loc1, int goal_loc2) {
            if(start_loc1 == start_loc2) {
                return false;
            }
            else if (start_loc1 == goal_loc1 || start_loc2 == goal_loc2) {
                return false;
            }
            std::pair<int, int> s1 = map_->ToYX(start_loc1);
            std::pair<int, int> g1 = map_->ToYX(goal_loc1);
            std::pair<int, int> s2 = map_->ToYX(start_loc2);
            std::pair<int, int> g2 = map_->ToYX(goal_loc2);
            if((s1.first - g1.first) * (s2.first - g2.first) < 0 ||
            (s1.second - g1.second) * (s2.second - g2.second) < 0 ) { // 方向不同
                return false;
            }
            else if ((s2.first - s1.first) * (s1.first - g1.first) < 0 &&
            (s2.second - s1.second) * (s1.second - g1.second) < 0 ) { // s1在中间
                return false;
            }
            else if ((s1.first - s2.first) * (s2.first - g2.first) < 0 &&
            (s1.second - s2.second) * (s2.second - g2.second) < 0 ) { // s2在中间
                return false;
            }
            else if ((s1.first == g1.first && s2.second == g2.second) ||
            (s1.second == g1.second && s2.first == g2.first)) { // area=1
                return false;
            }
            else {
                return true;
            }
        }

        std::pair<int, int> CBSHNode::GetRg(int start_loc1, int goal_loc1, int goal_loc2) {
            std::pair<int, int> s1 = map_->ToYX(start_loc1);
            std::pair<int, int> g1 = map_->ToYX(goal_loc1);
            std::pair<int, int> g2 = map_->ToYX(goal_loc2);
            int x, y;
            if(s1.first == g1.first) {
                x = g1.first;
            }
            else if (s1.first < g1.first) {
                x = std::min(g1.first, g2.first);
            }
            else {
                x = std::max(g1.first, g2.first);
            }

            if(s1.second == g1.second) {
                y = g1.second;
            }
            else if (s1.second < g1.second) {
                y = std::min(g1.second, g2.second);
            }
            else {
                y = std::max(g1.second, g2.second);
            }
            return std::make_pair(x, y);
        }

        std::pair<int, int> CBSHNode::GetRs(int start_loc1, int start_loc2, int goal_loc1) {
            std::pair<int, int> s1 = map_->ToYX(start_loc1);
            std::pair<int, int> s2 = map_->ToYX(start_loc2);
            std::pair<int, int> g1 = map_->ToYX(goal_loc1);
            int x, y;
            if(s1.first == g1.first) {
                x = s1.first;
            }
            else if (s1.first < g1.first) {
                x = std::max(s1.first, s2.first);
            }
            else {
                x = std::min(s1.first, s2.first);
            }

            if(s1.second == g1.second) {
                y = s1.second;
            }
            else if (s1.second < g1.second) {
                y = std::max(s1.second, s2.second);
            }
            else {
                y = std::min(s1.second, s2.second);
            }
            return std::make_pair(x, y);
        }

        // return 2: cardinal rectangle conflict
        // return 1: semi-cardinal rectangle conflict
        // return 0: non-cardinal rectangle conflict
        int CBSHNode::ClassifyRectangleConflict(int start_loc1, int start_loc2, int goal_loc1,
                int goal_loc2, std::pair<int, int> Rg) {
            std::pair<int, int> s1 = map_->ToYX(start_loc1);
            std::pair<int, int> g1 = map_->ToYX(goal_loc1);
            std::pair<int, int> s2 = map_->ToYX(start_loc2);
            std::pair<int, int> g2 = map_->ToYX(goal_loc2);
            int cardinal1 = 0;
            int cardinal2 = 0;
            if((s1.first == s2.first && (s1.second - s2.second) * (s2.second - Rg.second) >= 0) ||
            (s1.first != s2.first && (s1.first - s2.first) * (s2.first - Rg.first) < 0)) {
                if(Rg.first == g1.first) {
                    cardinal1 = 1;
                }
                if(Rg.second == g2.second) {
                    cardinal2 = 1;
                }
            }
            else {
                if(Rg.second == g1.second) {
                    cardinal1 = 1;
                }
                if(Rg.first == g2.first) {
                    cardinal2 = 1;
                }
            }
            return cardinal1 + cardinal2;
        }

        bool CBSHNode::FindRectangleConflict(const Conflict &conf) const {
            for(const auto c: conflicts_) {
                if(c.GetRg() != conf.GetRg()) {
                    continue;
                }
                else if ((c.GetAgent(0) == conf.GetAgent(0) && c.GetAgent(1) == conf.GetAgent(1)) ||
                (c.GetAgent(1) == conf.GetAgent(0) && c.GetAgent(0) == conf.GetAgent(1))) {
                    return true;
                }
            }
            return false;
        }

        void CBSHNode::RemoveLowPriorityConflict(std::list<Conflict> & conflicts) {
            if(conflicts.empty()) {
                return;
            }
            for(auto iter = conflicts.begin(); iter != conflicts.end(); ) {
                std::string agent_id1 = iter->GetAgent(0);
                std::string agent_id2 = iter->GetAgent(1);
                std::pair<std::string, std::string> agent_pair;
                if(agent_id1 < agent_id2) {
                    agent_pair = std::make_pair(agent_id1, agent_id2);
                }
                else {
                    agent_pair = std::make_pair(agent_id2, agent_id1);
                }

                // 在高优先级中查找，存在则删除
                bool found = false;
                for(const auto p: high_priority_) {
                    if(p == agent_pair) {
                        found = true;
                        iter = conflicts.erase(iter);
                        break;
                    }
                }
                if(found) {
                    continue;
                }

                auto iter_behind = iter;
                ++iter_behind;
                bool keep = true;
                while(iter_behind != conflicts.end()) {
                    // 只有在冲突双方一致时，进行比较，时间短的保留
                    if((agent_id1 == iter_behind->GetAgent(0) && agent_id2 == iter_behind->GetAgent(1)) ||
                    agent_id1 == iter_behind->GetAgent(1) && agent_id2 == iter_behind->GetAgent(0)) {
                        if(std::max(iter->GetTimestep(), iter_behind->GetTimestep()) >=
                        std::min(paths_.at(agent_id1).Size(), paths_.at(agent_id2).Size())) {
                            keep = iter->GetTimestep() <= iter_behind->GetTimestep();
                        }
                        else {
                            int time1;
                            int time2;
                            if(iter->Type() == "rectangle") {
                                time1 = GetRectangleTime(*iter);
                            }
                            else {
                                time1 = iter->GetTimestep();
                            }
                            if(iter_behind->Type() == "rectangle") {
                                time2 = GetRectangleTime(*iter_behind);
                            }
                            else {
                                time2 = iter_behind->GetTimestep();
                            }
                            keep = time1 <= time2;
                        }
                        if(keep) {
                            iter_behind = conflicts.erase(iter_behind);
                        }
                        else {
                            iter = conflicts.erase(iter);
                            break;
                        }
                    }
                    else {
                        ++iter_behind;
                    }
                }   // end while loop
                if(keep) {
                    ++iter;
                }
            }

            // 保留的冲突放入高优先级队列
            for(auto conf: conflicts) {
                std::string agent_id1 = conf.GetAgent(0);
                std::string agent_id2 = conf.GetAgent(1);
                std::pair<std::string, std::string> agent_pair;
                if(agent_id1 < agent_id2) {
                    agent_pair = std::make_pair(agent_id1, agent_id2);
                }
                else {
                    agent_pair = std::make_pair(agent_id2, agent_id1);
                }
                high_priority_.push_back(agent_pair);
            }
        }

        int CBSHNode::GetRectangleTime(const Conflict& conflict) {
            std::string agent_id1 = conflict.GetAgent(0);
            std::string agent_id2 = conflict.GetAgent(1);
            int t1 = conflict.GetRectTime(0);
            int t2 = conflict.GetRectTime(1);
            int s1_loc = paths_.at(agent_id1).GetLoc(t1);
            int s2_loc = paths_.at(agent_id2).GetLoc(t2);
            std::pair<int, int> s1 = map_->ToYX(s1_loc);
            std::pair<int, int> s2 = map_->ToYX(s2_loc);
            std::pair<int, int> Rg = conflict.GetRg();
            std::pair<int, int> Rs = GetRs(s1_loc, s2_loc, map_->YXToLoc(Rg.first, Rg.second));
            return t1 - abs(s1.first - Rs.first) - abs(s1.second - Rs.second);
        }

        int CBSHNode::WeightedVertexCorver(const std::map<std::pair<std::string, std::string>, int> &conf_graph) const {
            int agent_num = agent_ids_.size();
            int result = 0;
            std::map<std::string, bool> done;
            for(auto a: agent_ids_) {
                done[a] = false;
            }
            for(auto a1: agent_ids_) {
                if(done.at(a1)) {
                    continue;
                }
                std::vector<int> range; // 每个有连接agent的最大edge值
                std::vector<std::string> indices; // 所有edge>0连接的agent
                std::queue<std::string> Q;
                Q.push(a1);
                done[a1] = true;
                int num = 0;
                while(!Q.empty()) { // 宽度优先搜索
                    std::string a_curr = Q.front();
                    Q.pop();
                    range.push_back(0);
                    indices.push_back(a_curr);
                    for(auto a2: agent_ids_) {
                        auto iter = conf_graph.find(std::make_pair(a_curr, a2));
                        if(iter != conf_graph.end() && iter->second > 0) {  // conf_graph成对修改，只查一种就可以
                            range[num] = std::max(range[num], iter->second);
                            if(!done[a2]) {
                                Q.push(a2);
                                done[a2] = true;
                            }
                        }
                        iter = conf_graph.find(std::make_pair(a2, a_curr));
                        if(iter != conf_graph.end() && iter->second > 0) {  // conf_graph成对修改，只查一种就可以
                            range[num] = std::max(range[num], iter->second);
                            if(!done[a2]) {
                                Q.push(a2);
                                done[a2] = true;
                            }
                        }
                    }
                    ++num;
                }// end while loop
                if(num == 1) { // no edge
                    continue;
                }
                else if(num == 2) {
                    // add edge weight
                    auto iter1 = conf_graph.find(std::make_pair(indices[0], indices[1]));
                    auto iter2 = conf_graph.find(std::make_pair(indices[1], indices[0]));
                    int cg1 = 0;
                    int cg2 = 0;
                    if(iter1 != conf_graph.end()) {
                        cg1 = iter1->second;
                    }
                    if(iter2 != conf_graph.end()) {
                        cg2 = iter2->second;
                    }
                    result += std::max(cg1, cg2);
                    continue;
                }
                std::vector<int> x(num, 0);
                std::map<std::pair<std::string, std::string>, int> G; // 记录agent互相连接的最大edge
                for(int i = 0; i < num; ++i) {
                    for(int j = i+1; j < num; ++j) {
                        auto iter1 = conf_graph.find(std::make_pair(indices[i], indices[j]));
                        auto iter2 = conf_graph.find(std::make_pair(indices[j], indices[i]));
                        int cg1 = 0;
                        int cg2 = 0;
                        if(iter1 != conf_graph.end()) {
                            cg1 = iter1->second;
                        }
                        if(iter2 != conf_graph.end()) {
                            cg2 = iter2->second;
                        }
                        G[std::make_pair(agent_ids_[i], agent_ids_[j])] = std::max(cg1, cg2);
                    }
                }
                int best_so_far = std::numeric_limits<int>::max();
                result += WeightedVertexCorver(x, 0, 0, G, range, best_so_far);
            }// end for loop
            return result;
        }

        int CBSHNode::WeightedVertexCorver(std::vector<int> &x, int i, int sum,
            std::map<std::pair<std::string, std::string>, int> &conf_graph,
            std::vector<int> range, int &best_so_far) const {
            if(sum >= best_so_far) {
                return std::numeric_limits<int>::max();
            }
            else if(i == x.size()) {
                best_so_far = sum;
                return sum;
            }
            else if (range[i] == 0) {
                int rst = WeightedVertexCorver(x, i + 1, sum, conf_graph, range, best_so_far);
                if (rst < best_so_far) {
                    best_so_far = rst;
                }
                return best_so_far;
            }
            int min_cost = 0;
            for(int j = 0; j < i; ++j) {    // find minimum cost
                auto iter = conf_graph.find(std::make_pair(agent_ids_[j], agent_ids_[i]));
                int cg;
                if(iter != conf_graph.end()) {
                    cg = iter->second;
                }
                else{
                    cg = 0;
                }
                if(min_cost + x[j] < cg ) {
                    min_cost = cg - x[j];
                }
            }

            int best_cost = -1;
            for(int cost = min_cost; cost <= range[i]; ++cost) {
                x[i] = cost;
                int rst = WeightedVertexCorver(x, i + 1, sum + x[i], conf_graph, range, best_so_far);
                if(rst < best_so_far) {
                    best_so_far = rst;
                    best_cost = cost;
                }
            }
            if(best_cost >= 0) {
                x[i] = best_cost;
            }
            return best_so_far;
        }

        // DG使用的最小顶点覆盖评价函数
        int CBSHNode::MinimumVertexCover(const std::map<std::pair<std::string, std::string>, int> &conf_graph,
                int edge_num) const {
            if(edge_num < 2) {
                return edge_num;
            }
            // 统计有冲突的agent数量
            int node_num = 0;
            for(auto a1: agent_ids_) {
                for(auto a2: agent_ids_) {
                    auto iter = conf_graph.find(std::make_pair(a1, a2));
                    if(iter != conf_graph.end() && iter->second > 0) {
                        ++node_num;
                        break;
                    }
                }
            }
            // 根节点从1开始寻找最小顶点覆盖
            if(parent_h_cost_ == -1) {
                for(int i = 1; i < node_num; ++i) {
                    if(KVertexCover(conf_graph, node_num, edge_num, i)) {
                        return i;
                    }
                }
            }
            // 子节点在父节点基础上去除冲突，可能产生新冲突，所以在parent h cost附近查找
            else {
                if(KVertexCover(conf_graph, node_num, edge_num, parent_h_cost_ - 1)) {
                    return parent_h_cost_ - 1;
                }
                else if(KVertexCover(conf_graph, node_num, edge_num, parent_h_cost_)) {
                    return parent_h_cost_;
                }
                else {
                    return parent_h_cost_ + 1;
                }
            }
        }

        bool CBSHNode::KVertexCover(const std::map<std::pair<std::string, std::string>, int> &conf_graph,
                int node_num, int edge_num, int k) const {
            // 如果边过少，或者边过多，不需要递归
            if(edge_num == 0) {
                return true;
            }
            else if(edge_num > k * node_num - k) {
                return false;
            }

            int agent_num = agent_ids_.size();
            // 找到一个存在冲突agent
            std::string to_remove_node;
            bool found = false;
            for(auto a1: agent_ids_) {
                for(auto a2: agent_ids_) {
                    auto iter = conf_graph.find(std::make_pair(a1, a2));
                    if (iter != conf_graph.end() && iter->second > 0) {
                        to_remove_node = a1;
                        found = true;
                        break;
                    }
                }
                if(found) {
                    break;
                }
            }
            // 从graph中去除该agent所有边
            std::map<std::pair<std::string, std::string>, int> conf_graph_reduce = conf_graph;
            int edge_num_reduce = edge_num;
            for(auto a1: agent_ids_) {
                auto iter = conf_graph_reduce.find(std::make_pair(to_remove_node, a1));
                if(iter != conf_graph_reduce.end() && iter->second > 0) {
                    conf_graph_reduce[std::make_pair(to_remove_node, a1)] = 0;
                    conf_graph_reduce[std::make_pair(a1, to_remove_node)] = 0;
                    --edge_num_reduce;
                }
            }
            // 递归求解
            if(KVertexCover(conf_graph_reduce, node_num - 1, edge_num_reduce, k - 1)) {
                return true;
            }
            return false;
        }

        void CBSHNode::ChooseConflict() {
            if(!cardinal_conf_.empty()) { // 选择cost最大cardinal,cost相同选最早
                int max_weight = -1;
                int min_time = std::numeric_limits<int>::max();
                mapf::CBSH::Conflict temp_conf;
                for(auto conf: cardinal_conf_) {
                    // weight为自定义，一方抵达目标点的冲突的优先级更高
                    int weight = 2;
                    std::string agent1 = conf.GetAgent(0);
                    std::string agent2 = conf.GetAgent(1);
                    int t = conf.GetTimestep();
                    if(t >= paths_.at(agent1).Size()) {
                        weight = t - paths_.at(agent1).Size() + 3;
                    }
                    else if(t >= paths_.at(agent2).Size()) {
                        weight = t - paths_.at(agent2).Size() + 3;
                    }
                    // 评估时间
                    int t_n;
                    if(conf.Type() == "rectangle") {
                        t_n = GetRectangleTime(conf);
                    }
                    else {
                        t_n = t;
                    }
                    if(max_weight < weight || (max_weight == weight && t_n < min_time)) {
                        temp_conf = conf;
                        max_weight = weight;
                        min_time = t_n;
                    }
                }
                conflicts_.push_back(temp_conf);
            }
            else if(!rectSemi_conf_.empty()) {   // 选择最早的
                Conflict earliest_conf = rectSemi_conf_.front();
                int min_time = std::numeric_limits<int>::max();
                for(auto conf: rectSemi_conf_) {
                    int t = GetRectangleTime(conf);
                    if(t < min_time) {
                        earliest_conf = conf;
                        min_time = t;
                    }
                }
                conflicts_.push_back(earliest_conf);
            }
            else if(!semi_conf_.empty()) {  // 选择最早的
                Conflict earliest_conf = semi_conf_.front();
                int min_time = std::numeric_limits<int>::max();
                for(auto conf: semi_conf_) {
                    int t = conf.GetTimestep();
                    if(t < min_time) {
                        earliest_conf = conf;
                        min_time = t;
                    }
                }
                conflicts_.push_back(earliest_conf);
            }
            else if(!rectNon_conf_.empty()) {   // 选择最早的
                Conflict earliest_conf = rectNon_conf_.front();
                int min_time = std::numeric_limits<int>::max();
                for(auto conf: rectNon_conf_) {
                    int t = GetRectangleTime(conf);
                    if(t < min_time) {
                        earliest_conf = conf;
                        min_time = t;
                    }
                }
                conflicts_.push_back(earliest_conf);
            }
            else if(!non_conf_.empty()) {  // 选择最早的
                Conflict earliest_conf = non_conf_.front();
                int min_time = std::numeric_limits<int>::max();
                for(auto conf: non_conf_) {
                    int t = conf.GetTimestep();
                    if(t < min_time) {
                        earliest_conf = conf;
                        min_time = t;
                    }
                }
                conflicts_.push_back(earliest_conf);
            }
            else if(!unknown_conf_.empty()) {  // 选择最早的
                Conflict earliest_conf = unknown_conf_.front();
                int min_time = std::numeric_limits<int>::max();
                for(auto conf: unknown_conf_) {
                    int t = conf.GetTimestep();
                    if(t < min_time) {
                        earliest_conf = conf;
                        min_time = t;
                    }
                }
                conflicts_.push_back(earliest_conf);
            }
        }

        Conflict CBSHNode::GetLastestConflict() const {
            return conflicts_.back();
        }

        void CBSHNode::AddConstraint(std::string agent_id, Constraint cons) {
            paths_.at(agent_id).AddConstraint(cons);
        }

        std::map<std::string, CBSHPath> CBSHNode::GetPaths() const {
            return paths_;
        }

        CBSHPath CBSHNode::GetPath(std::string agent_id) const {
            return paths_.at(agent_id);
        }

        void CBSHNode::UpdateConfTable(std::string agent_id) {
            conf_avoid_ver_table_.clear();
            conf_avoid_edge_table_.clear();
            conf_avoid_ver_table_.resize(makespan_ + 1);
            conf_avoid_edge_table_.resize(makespan_ + 1);
            for(auto a: agent_ids_) {
                if(a != agent_id && !paths_.at(a).IsEmpty()) {
                    for(int t = 0; t < makespan_ + 1; ++t) {
                        if(t >= paths_.at(a).Size()) {
                            conf_avoid_ver_table_[t].insert(paths_.at(a).GetLastLoc());
                        }
                        else {
                            int loc = paths_.at(a).GetLoc(t);
                            conf_avoid_ver_table_[t].insert(loc);
                            if(t > 0 && paths_.at(a).GetLoc(t - 1) != loc) {
                                int prev_loc = paths_.at(a).GetLoc(t - 1);
                                // 从前往后记录
                                conf_avoid_edge_table_[t].insert(std::make_pair(loc, prev_loc));
                            }
                        }
                    } // end for loop
                }
            }// end for loop
        }

        int CBSHNode::NumOfConflictsForStep(int curr_loc, int next_loc, int next_timestep) const {
            if(conf_avoid_edge_table_.empty() && conf_avoid_ver_table_.empty()) {
                return 0;
            }
            int result = 0;
            if(next_timestep >= conf_avoid_ver_table_.size()) {
                // 查找顶点限制
                auto iter = conf_avoid_ver_table_.back().find(next_loc);
                if(iter != conf_avoid_ver_table_.back().end()) {
                    ++result;
                }
            }
            else {
                // 查找顶点限制
                auto iter = conf_avoid_ver_table_[next_timestep].find(next_loc);
                if(iter != conf_avoid_ver_table_[next_timestep].end()) {
                    ++result;
                }
                // 查找边限制，从后往前查找，对向冲突
                auto iter_edge = conf_avoid_edge_table_[next_timestep].find(std::make_pair(curr_loc, next_loc));
                if(iter_edge != conf_avoid_edge_table_[next_timestep].end()) {
                    ++result;
                }
            }
            return result;
        }

        MDD::Ptr CBSHNode::BuildMDD(std::string agent_id) {
            // TODO:将计算的mdd保存备查，key的数据结构过于复杂，待实现、调试
            /*std::set<Constraint> cons;
            for(auto c_step: paths_.at(agent_id).GetConstraints()) {
                for(auto c: c_step){
                    cons.insert(c);
                }
            }
            MDDTable mddtable_key(agent_id, cons);
            if(!mddtable_.empty()) {
                auto iter = mddtable_.find(mddtable_key);
                if(iter != mddtable_.end()) {
                    return iter->second;
                }
            }*/

            MDD::Ptr mdd;
            mdd.reset(new MDD());
            mdd->BuildMDD(map_, paths_.at(agent_id), astar_h_);
            /*if(rectangle_reasoning_) {
                mddtable_[mddtable_key] = mdd;
            }*/
            return mdd;
        }

        void CBSHNode::UpdateCost(int h) {
            node_h_cost_ = std::max(h, node_h_cost_);
            total_cost_ = node_g_cost_ + node_h_cost_;
            if(node_h_cost_ == 6) {
                int temp = 1;
            }
        }

        void CBSHNode::EstimateH(std::string agent_id, const std::map<std::pair<std::string, std::string>, int> &conflict_graph) {
            int temp_h = 0;
            if(node_h_cost_ != 0) {
                if(conflict_graph.empty()) {
                    temp_h = node_h_cost_ - 1;
                }
                else {
                    int max_weight = 0;
                    for(auto ap: conflict_graph) {
                        if((ap.first.first == agent_id || ap.first.second == agent_id) && ap.second > max_weight) {
                            max_weight = ap.second;
                            if(max_weight >= node_h_cost_) {
                                break;
                            }
                        }
                    } // end for loop
                    if(max_weight < node_h_cost_) {
                        temp_h = node_h_cost_ - max_weight;
                    }
                }
            }
            node_h_cost_ = std::max(temp_h, total_cost_ - node_g_cost_);
            total_cost_ = node_g_cost_ + node_h_cost_;
        }

        bool CBSHNode::SyncMDDs(const MDD &mdd1, const MDD &mdd2) {
            if(mdd2.GetLevelSize() <= 1) {
                return false;
            }
            MDD copy(mdd1);
            if(copy.GetLevelSize() < mdd2.GetLevelSize()) {
                int i = copy.GetLevelSize();
                copy.ResizeLevel(mdd2.GetLevelSize());
                for(; i < mdd2.GetLevelSize(); ++i) {
                    MDDNode::Ptr parent = copy.GetLevel(i - 1).front();
                    MDDNode::Ptr node;
                    node.reset(new MDDNode(parent->GetLoc(), parent));
                    parent->AddChild(node);
                    copy.AddNode(i, node);
                }
            }

            copy.AddCoexistingNode(0, mdd2.GetLevel(0).front());
            int l = copy.GetLevel(0).front()->GetCoexistingNodes().size();

            for(int i = 1; i < copy.GetLevelSize(); ++i) {
                for(auto &n: copy.GetLevel(i)) {
                    for(auto p: n->GetParents()) {
                        for(auto p_coexit: p->GetCoexistingNodes()) {
                            for(auto ch_p_coexit: p_coexit->GetChildren()) {
                                if(n->GetLoc() == ch_p_coexit->GetLoc()) {  // vertex conflict
                                    continue;
                                }
                                else if(n->GetLoc() == p_coexit->GetLoc() && p->GetLoc() == ch_p_coexit->GetLoc()) {    // edge conflict
                                    continue;
                                }

                                auto coexit = n->GetCoexistingNodes();
                                auto iter = coexit.cbegin();
                                for(; iter != coexit.cend(); ++iter){
                                    if((*iter) == ch_p_coexit) {
                                        break;
                                    }
                                }
                                if(iter == coexit.cend()){
                                    n->AddCoexistingNode(ch_p_coexit);
                                }
                            }
                        }
                    }
                    if(n->GetCoexistingNodes().empty()) {
                        copy.DeletNode(n, i);
                    }
                }
                if(copy.GetLevel(i).empty()) {
                    return false;
                }
            } // end for loop
            return true;
        }

        void CBSHNode::CBSHNodeLog() const {
            // LOG_DEBUG_STREAM("CBSHNode log.");
        }

        std::list<Conflict> CBSHNode::GetConflicts() const {
            return conflicts_;
        }

        int CBSHNode::GetDepth() const {
            return depth_;
        }

        int CBSHNode::GetHCost() const {
            return node_h_cost_;
        }

        int CBSHNode::GetGCost() const {
            return node_g_cost_;
        }

        int CBSHNode::GetMakespan() const {
            return makespan_;
        }

        bool CBSHNode::HasComputeH() const {
            return has_compute_h_;
        }

        std::string CBSHNode::GetConsAgent() const {
            return constraint_agent_;
        }

        std::map<std::pair<std::string, std::string>, int> CBSHNode::GetConflictGraph() const {
            return conflict_graph_;
        }

        void CBSHNode::CopyConflictGraph(const std::map<std::pair<std::string, std::string>, int> &conf_graph) {
            if(strategy_ == "DG" || strategy_ == "WDG") {
                for(auto c: conf_graph) {
                    if(c.first.first != constraint_agent_ && c.first.second != constraint_agent_) {
                        conflict_graph_[c.first] = c.second;
                    }
                }
            }
        }

        void CBSHNode::SetComputeH(bool hascompute) {
            has_compute_h_ = hascompute;
        }

        std::list<Conflict> CBSHNode::GetCardinalConf() const{
            return cardinal_conf_;
        }

    } // namespace CBSH
} // namespace mapf