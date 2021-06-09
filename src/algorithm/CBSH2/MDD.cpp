#include <queue>
#include <list>

#include "MDD.h"

namespace mapf {
    namespace CBSH {
        int MDD::GetLevelSize() const {
            return levels_.size();
        }

        std::list<MDDNode::Ptr> MDD::GetLevel(int index) const {
            return levels_[index];
        }

        MDDTable::MDDTable(std::string agent_id, std::set<Constraint> cons_set)
            : agent_id_(agent_id), cons_set_(cons_set)
        {
            //
        }

        MDDNode::MDDNode(int loc, MDDNode::Ptr parent) {
            loc_ = loc;
            if(parent == nullptr) {
                level_ = 0;
            }
            else {
                level_ = parent->level_ + 1;
                parents_.push_back(parent);
            }
        }

        int MDDNode::GetLevel() const {
            return level_;
        }

        int MDDNode::GetLoc() const {
            return loc_;
        }

        void MDDNode::AddParent(MDDNode::Ptr parent) {
            parents_.push_back(parent);
        }

        std::list<MDDNode::Ptr> MDDNode::GetParents() const {
            return parents_;
        }

        std::list<MDDNode::Ptr> MDDNode::GetChildren() const {
            return children_;
        }

        void MDDNode::AddChild(MDDNode::Ptr child) {
            children_.push_back(child);
        }

        void MDDNode::AddCoexistingNode(const MDDNode::Ptr node) {
            coexisting_nodes_.push_back(node);
        }

        std::list<MDDNode::Ptr> MDDNode::GetCoexistingNodes() const {
            return coexisting_nodes_;
        }

        void MDDNode::RemoveParent(MDDNode::Ptr parent) {
            parents_.remove(parent);
        }

        void MDDNode::RemoveChild(MDDNode::Ptr child) {
            children_.remove(child);
        }

        bool MDD::BuildMDD(const Map::ConstPtr &map, CBSHPath path, 
        const std::unordered_map<std::string, std::vector<int> > &astar_h) {
            MDDNode::Ptr root;
            root.reset(new MDDNode(path.GetStartLoc(), nullptr));
            int num_of_levels = path.Size();
            std::queue<MDDNode::Ptr> open_list;
            std::list<MDDNode::Ptr> closed_list;
            open_list.push(root);
            closed_list.push_back(root);
            levels_.resize(num_of_levels);
            while (!open_list.empty()){
                MDDNode::Ptr curr_node = open_list.front();
                open_list.pop();
                if(curr_node->GetLevel() == num_of_levels - 1) {
                    levels_[num_of_levels - 1].push_back(curr_node);
                    if(!open_list.empty()) {
                        return false;
                    }
                    break;
                }
                // (g+1)+h <= f = num_of_levels-1，所以h上限num_of_levels-g-2
                float h_bound = num_of_levels - curr_node->GetLevel() -2;
                for(int i = 0; i < 5; ++i) {
                    int new_loc = curr_node->GetLoc() + map->GetMoveOffset()[i];
                    if(map->ValidMove(curr_node->GetLoc(), new_loc) && 
                    astar_h.at(path.GetId())[new_loc] <= h_bound &&
                    !path.IsConstrainted(curr_node->GetLoc(), new_loc, curr_node->GetLevel() + 1)) {    // 可行移动
                        bool find = false;
                        for(std::list<MDDNode::Ptr>::reverse_iterator child = closed_list.rbegin();
                        child != closed_list.rend() && child->get()->GetLevel() == curr_node->GetLevel() + 1;
                        ++child) {  // 遍历closed list，查找child是否存在
                            if(child->get()->GetLoc() == new_loc) {
                                child->get()->AddParent(curr_node);
                                find = true;
                                break;
                            }
                        }
                        if(!find) { // 如果不存在，新建child
                            MDDNode::Ptr child_node;
                            child_node.reset(new MDDNode(new_loc, curr_node));
                            open_list.push(child_node);
                            closed_list.push_back(child_node);
                        }
                    }
                }// end for loop
            } // end while loop
            
            for(int t = num_of_levels - 1; t > 0; --t) {
                for(auto it: levels_[t]) {
                    for(auto parent: it->GetParents()) {
                        if(parent->GetChildren().empty()) {
                            levels_[t - 1].push_back(parent);
                        }
                        parent->AddChild(it);
                    }
                }
            }
            return true;
        }

        void MDD::ResizeLevel(int len) {
            levels_.resize(len);
        }

        void MDD::AddNode(int level_index, MDDNode::Ptr node) {
            levels_[level_index].push_back(node);
        }

        void MDD::AddCoexistingNode(int level_index, MDDNode::Ptr node){
            levels_[level_index].front()->AddCoexistingNode(node);
        }

        void MDD::DeletNode(MDDNode::Ptr node, int level_index) {
            levels_[level_index].remove(node);
            for(auto child: node->GetChildren()){
                child->RemoveParent(node);
                if(child->GetParents().empty()) {
                    DeletNode(child, level_index + 1);
                }
            }
            for(auto parent: node->GetParents()) {
                parent->RemoveChild(node);
                if(parent->GetChildren().empty()) {
                    DeletNode(parent, level_index - 1);
                }
            }
        }

        // deep copy
        MDD::MDD(const MDD &mdd_cpy) {
            levels_.resize(mdd_cpy.GetLevelSize());
            MDDNode::Ptr root;
            root.reset(new MDDNode(mdd_cpy.GetLevel(0).front()->GetLoc(), nullptr));
            levels_[0].push_back(root);
            for(int t = 0; t < levels_.size() - 1; ++t) {
                for(auto &node: levels_[t]) {
                    MDDNode::Ptr cpy_node = mdd_cpy.FindNode(node->GetLoc(), t);
                    for(auto cpy_child: cpy_node->GetChildren()) {
                        MDDNode::Ptr child = FindNode(cpy_child->GetLoc(), cpy_child->GetLevel());
                        if(child == nullptr) {
                            child.reset(new MDDNode(cpy_child->GetLoc(), node));
                            levels_[t+1].push_back(child);
                            node->AddChild(child);
                        }
                        else {
                            child->AddParent(node);
                            node->AddChild(child);
                        }
                    }
                }
            }
        }

        MDDNode::Ptr MDD::FindNode(int loc, int level) const {
            if(level < levels_.size()) {
                for(auto iter: levels_[level]) {
                    if(iter->GetLoc() == loc) {
                        return iter;
                    }
                }
            }
            return nullptr;
        }

        std::vector<std::list<MDDNode::Ptr> > MDD::GetLevels() const {
            return levels_;
        }

    } // namespace CBSH
} // namespace mapf