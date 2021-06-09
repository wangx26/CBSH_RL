#ifndef MDD_H
#define MDD_H

#include <list>
#include <memory>
#include <set>
#include <unordered_map>

#include "CBSHConstraint.h"
#include "mapf_map/mapf_map.h"
#include "CBSHPath.h"

namespace mapf {
    namespace CBSH {
        class MDDNode
        {
        public:
            typedef std::shared_ptr<MDDNode> Ptr;
            MDDNode(int loc, MDDNode::Ptr parent);
            ~MDDNode()=default;

            int GetLevel() const;
            int GetLoc() const;
            std::list<MDDNode::Ptr> GetParents() const;
            std::list<MDDNode::Ptr> GetChildren() const;

            void AddParent(MDDNode::Ptr parent);
            void AddChild(MDDNode::Ptr child);

            void AddCoexistingNode(const MDDNode::Ptr node);
            std::list<MDDNode::Ptr> GetCoexistingNodes() const;

            void RemoveParent(MDDNode::Ptr parent);
            void RemoveChild(MDDNode::Ptr child);
        private:
            int loc_;
            int level_;

            std::list<MDDNode::Ptr> children_;
            std::list<MDDNode::Ptr> parents_;

            std::list<MDDNode::Ptr> coexisting_nodes_; // SyncMDD使用
        };
        
        class MDD
        {
        public:
            typedef std::shared_ptr<MDD> Ptr;

            MDD()=default;
            MDD(const MDD &mdd_cpy);    // 深度拷贝,SyncMDD使用
            ~MDD()=default;

            int GetLevelSize() const;
            std::list<MDDNode::Ptr> GetLevel(int index) const;
            std::vector<std::list<MDDNode::Ptr> > GetLevels() const;

            bool BuildMDD(const Map::ConstPtr &map, CBSHPath path, 
            const std::unordered_map<std::string, std::vector<int> > &astar_h);
            void ResizeLevel(int len);
            void AddNode(int level_index, MDDNode::Ptr node);

            // SyncMDD使用
            MDDNode::Ptr FindNode(int loc, int level) const;
            void AddCoexistingNode(int level_index, MDDNode::Ptr node);
            void DeletNode(MDDNode::Ptr node, int level_index);
        private:
            std::vector<std::list<MDDNode::Ptr> > levels_;
        };

        class MDDTable
        {
        private:
            std::string agent_id_;
            std::set<Constraint> cons_set_;
        public:
            MDDTable(std::string agent_id, std::set<Constraint> cons_set);
            ~MDDTable()=default;

            bool operator<(const MDDTable &m) const {
                if(agent_id_ < m.agent_id_ ) {
                    return true;
                }
                else if(cons_set_.size() < m.cons_set_.size()) {
                    return true;
                }
                else if(cons_set_.size() == m.cons_set_.size()) {
                    bool less = true;
                    auto iter1 = cons_set_.begin();
                    auto iter2 = m.cons_set_.begin();
                    for(; iter1 != cons_set_.end() && iter2 != m.cons_set_.end(); ++iter1, ++iter2) {
                        if(!(*iter1 < *iter2)) {
                            less = false;
                            break;
                        }
                    }
                    return less;
                }
                return false;
            }
        };
    } // namespace CBSH
} // namespace mapf
#endif //MDD_H