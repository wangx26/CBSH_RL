#ifndef CBSHNODE_H
#define CBSHNODE_H

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_set.hpp>
#include <memory>
#include <list>
#include <map>
#include <unordered_map>
#include <set>

#include "CBSHPath.h"
#include "CBSHConstraint.h"
#include "CBSHConflict.h"
#include "mapf_map/mapf_map.h"
#include "Htable.h"
#include "LLNode.h"
#include "MDD.h"

namespace mapf {
    namespace CBSH {

        class CBSHNode
        {
        private:
            const std::vector<std::string> &agent_ids_;
            std::string strategy_;
            const bool &rectangle_reasoning_;
            const Map::ConstPtr &map_;
            const std::unordered_map<std::string, std::vector<int> > &astar_h_;
            std::list<Conflict> conflicts_;     // 所有选定扩展的冲突，最后为本次使用
            std::string constraint_agent_;
            int depth_;
            bool has_compute_h_;

            std::map<MDDTable, MDD::Ptr> &mddtable_;

            int total_cost_;
            int node_g_cost_;
            int node_h_cost_;
            int parent_h_cost_;

            int num_of_collisions_;
            int makespan_;
            bool block_;

            std::list<std::pair<std::string, std::string> > high_priority_;  // 去除低优先级冲突时使用

            // 底层规划
            std::vector<std::set<int> > conf_avoid_ver_table_;  // 记录已被占用的路径点，在底层规划中记录冲突进行评估，不做回避
            std::vector<std::set<std::pair<int, int> > > conf_avoid_edge_table_;  // 记录已被占用的边，pair<curr point, pre point>
            boost::unordered_set<LLNode::Ptr, LLNode::NodeHasher, LLNode::Eqnode> all_LL_node_table_;  //LLnode为next，pair<next_loc, next_timestep>

        public:
            typedef std::shared_ptr<CBSHNode> Ptr;

            std::list<Conflict> unknown_conf_;
            std::list<Conflict> cardinal_conf_;
            std::list<Conflict> rectSemi_conf_;
            std::list<Conflict> semi_conf_;
            std::list<Conflict> rectNon_conf_;
            std::list<Conflict> non_conf_;
            std::map<std::pair<std::string, std::string>, int> conflict_graph_; // map<<id1, id2>, weight >
            std::map<std::string, CBSHPath> paths_;    // 所有路径

            CBSHNode(const std::vector<std::string> &agent_ids, const std::vector<int> &starts,
                const std::vector<int> &goals, std::string strategy, 
                const bool &rectangle_reasoning, const Map::ConstPtr &map,
                std::map<MDDTable, MDD::Ptr> &mddtable, 
                std::unordered_map<std::string, std::vector<int> > &astar_h, 
                const bool &block);
            CBSHNode(const std::vector<std::string> &agent_ids, std::string strategy, 
                const bool &rectangle_reasoning, const Map::ConstPtr &map,
                std::map<MDDTable, MDD::Ptr> &mddtable, 
                std::unordered_map<std::string, std::vector<int> > &astar_h, 
                const bool &block, std::map<std::string, CBSHPath> init_paths, 
                int init_h);
            CBSHNode(CBSHNode::Ptr &node, std::string cons_agent);
            ~CBSHNode()=default;

            bool LLPlan(std::string agent_id, int lower_bound);

            void FindConflict(std::string agent_id);
            void FindConflictRoot();

            struct Open_compare {
                bool operator()(const CBSHNode::Ptr n1, const CBSHNode::Ptr n2) const {
                    return n1->total_cost_ >= n2->total_cost_;
                }
            };

            struct Focal_compare {
                bool operator()(const CBSHNode::Ptr n1, const CBSHNode::Ptr n2) const {
                    return n1->num_of_collisions_ >= n2->num_of_collisions_;
                }
            };

            // 高层规划使用handle type
            boost::heap::fibonacci_heap<CBSHNode::Ptr, boost::heap::compare<CBSHNode::Open_compare> >::handle_type open_handle_;
            boost::heap::fibonacci_heap<CBSHNode::Ptr, boost::heap::compare<CBSHNode::Focal_compare> >::handle_type focal_handle_;

            // 底层规划使用
            boost::heap::fibonacci_heap<LLNode::Ptr, boost::heap::compare<LLNode::Open_compare> > open_list_;
            boost::heap::fibonacci_heap<LLNode::Ptr, boost::heap::compare<LLNode::Focal_compare> > focal_list_;
            //std::set<LLNode::Ptr> all_LL_nodes_;

            int GetCollisionNum() const;
            float GetTotalCost() const;
            int GetHCost() const;
            int GetGCost() const;
            std::list<Conflict> GetConflicts() const;
            int GetDepth() const;
            int GetMakespan() const;
            bool HasComputeH() const;
            std::list<Conflict> GetCardinalConf() const;
            std::map<std::pair<std::string, std::string>, int> GetConflictGraph() const;

            void ClassifyConflicts();
            
            void ChooseConflict();

            void EstimateH(std::string agent_id, const std::map<std::pair<std::string, std::string>, int> &conflict_graph);

            Conflict GetLastestConflict() const;
            void AddConstraint(std::string agent_id, Constraint cons);

            std::map<std::string, CBSHPath> GetPaths() const;
            CBSHPath GetPath(std::string agent_id) const;
            void SetComputeH(bool hascompute);

            // 建立MDD，更新MDDtable，更新single
            MDD::Ptr BuildMDD(std::string agent_id);

            void UpdateCost(int h);

            void CBSHNodeLog() const;

            std::string GetConsAgent() const;
            void CopyConflictGraph(const std::map<std::pair<std::string, std::string>, int> &conf_graph);
        private:
            // CBSH-RM
            // 返回值int为index（timestep)
            std::list<int> GetStartCandidates(std::string agent_id, int timestep);
            std::list<int> GetGoalCandidates(std::string agent_id, int timestep);

            bool IsManhattanOptimal(int loc1, int loc2, int dt);
            bool IsRectangleConflict(int start_loc1, int start_loc2, int goal_loc1, int goal_loc2);

            // CBSH-RM
            std::pair<int, int> GetRg(int start_loc1, int goal_loc1, int goal_loc2);
            std::pair<int, int> GetRs(int start_loc1, int start_loc2, int goal_loc1);
            int ClassifyRectangleConflict(int start_loc1, int start_loc2, int goal_loc1, 
                int goal_loc2, std::pair<int, int> Rg);
            bool FindRectangleConflict(const Conflict &conf) const ;
            void RemoveLowPriorityConflict(std::list<Conflict> & conflicts);
            int GetRectangleTime(const Conflict& conflict);

            void CopyList(std::list<Conflict> & to, const std::list<Conflict> & from);

        public:
            // DG,WDG
            
            int WeightedVertexCorver(const std::map<std::pair<std::string, std::string>, int> &conf_graph) const;
            int WeightedVertexCorver(std::vector<int> &x, int i, int sum, 
            std::map<std::pair<std::string, std::string>, int> &conf_graph, 
            std::vector<int> range, int &best_so_far) const;
            int MinimumVertexCover(const std::map<std::pair<std::string, std::string>, int> &conf_graph, 
                int edge_num) const;
            // 是否存在k-vertex covers solution,每个点连接的边不大于k
            // 递归求解
            bool KVertexCover(const std::map<std::pair<std::string, std::string>, int> &conf_graph, 
                int node_num, int edge_num, int k) const;

            // 底层规划
            void UpdateConfTable(std::string agent_id);
            int NumOfConflictsForStep(int curr_loc, int next_loc, int next_timestep) const;\

            // MDD合并
            bool SyncMDDs(const MDD &mdd1, const MDD &mdd2);
        };
    }
}

#endif // CBSHNODE_H