#ifndef CBSSEARCH_H
#define CBSSEARCH_H

#include <boost/heap/fibonacci_heap.hpp>
#include <unordered_map>

#include "plan.h"
#include "CBSHNode.h"
#include "agent/agent.h"
#include "mapf_map/mapf_map.h"
#include "Htable.h"
#include "MDD.h"

namespace mapf {
    namespace CBSH {

        class CBSHSearch //: public mapf::Plan
        {
        private:
            std::vector<std::string> agent_ids_;
            mapf::Map::Ptr map_;
            std::string strategy_;  //默认使用PC，范围：NONE,CG,DG,WDG
            bool rectangle_reasoning_; // TODO:rectangle reasoning using MDDs

            int HL_expend_num_;   // 计数，考察算法性能
            float focal_list_threshold_;    // 筛选部分节点的阈值
            float focal_w_; // 性能放松比例，范围>=1，设置为1.0则寻找最优解
            float min_f_cost_;  // 记录最小f cost

            int cost_upperbound_;
            double time_limit_;

            std::vector<Agent::Ptr> plan_result_;
            bool block_;    // 如果agent托着货架，block为true，不能钻货架

            CBSHNode::Ptr root_;
            double compute_astarh_time_;

            std::unordered_map<std::string, std::vector<int> > astar_h_;  // key为agent id，value为astar的h值，每次规划需重新计算

            std::map<Htable, std::map<Htable, int> > htable_;   // WDG使用，记录所有dege的weight

            std::map<MDDTable, MDD::Ptr> mddtable_; // MDD使用，记录所有构建的MDD

            // 全部节点
            boost::heap::fibonacci_heap<CBSHNode::Ptr, boost::heap::compare<CBSHNode::Open_compare> > open_list_;
            // 选择部分节点作为每次选取范围
            boost::heap::fibonacci_heap<CBSHNode::Ptr, boost::heap::compare<CBSHNode::Focal_compare> > focal_list_;
        public:
            typedef std::shared_ptr<CBSHSearch> Ptr;
            // 对应level会使用低level所有方法
            CBSHSearch();
            CBSHSearch(Map::Ptr map, const std::vector<Agent::Ptr> &agents, bool block);
            CBSHSearch(Map::Ptr map, const std::vector<std::string> &agent_ids,
                        std::map<std::string, CBSHPath> init_paths, double f_w, int init_h, std::string strategy,
                        bool rectangle_reasoning, int cost_upperbound, double time_limit, bool block);
            ~CBSHSearch()=default;

            bool MakePlan();
            float solution_cost_;

            // 将节点路径转化为输出路径
            void RecordPlan(const CBSHNode::Ptr &node);
            std::vector<Agent::Ptr> GetPlan();

            bool BuildChild(CBSHNode::Ptr &node, std::string cons_agent,
                            const std::map<std::pair<std::string, std::string>, int> &conflict_graph);
            void UpdateFocalList();
        private:
            // 计算AStar的h函数，所有点到目标点的h函数，加速AStar计算
            void ComputeH(int goal_loc, std::string agent_id);

            // MDD使用
            void AddModifiedBarrierCons(std::vector<std::pair<int, bool> > path1,
                                        std::vector<std::pair<int, bool> > path2, MDD::Ptr mdd1,
                                        MDD::Ptr mdd2, int s1, int s2, std::pair<int, int> Rg,
                                        Constraint &cons1, Constraint &cons2);
            void AddModifiedBarrierConsH(MDD::Ptr mdd, std::pair<int, int> Rg, std::pair<int, int> R,
                                        int Rg_t, Constraint &cons);
            void AddModifiedBarrierConsV(MDD::Ptr mdd, std::pair<int, int> Rg, std::pair<int, int> R,
                                        int Rg_t, Constraint &cons);

            int ComputeHeuristics(CBSHNode::Ptr node);
            bool BuildDependenceGraph(CBSHNode::Ptr node);
            std::pair<int, bool> GetEdgeWeight(std::string agent1, std::string agent2, bool cardinal,
                                               CBSHNode::Ptr node);
        public:
            // RL
            void Reset();
            float Step(int a, int t);
            bool IsCons(int a, int t) const;
            CBSHNode::Ptr curr_node_;
            bool rl_done_;
            std::vector<int> GetState() const;
            bool isDone() const;
            float GetReward() const;
            int reward_;
            std::vector<int> GetValidAction() const;
        };
    }
}

#endif // CBSSEARCH_H