#ifndef LLNODE_H
#define LLNODE_H

#include <vector>
#include <memory>
#include <boost/heap/fibonacci_heap.hpp>

namespace mapf {
    namespace CBSH {
        class LLNode
        {
        public:
            typedef std::shared_ptr<LLNode> Ptr;

            struct Open_compare {
                bool operator()(const LLNode::Ptr n1, const LLNode::Ptr n2) const {
                    if(n1->g_cost_ + n1->h_cost_ == n2->g_cost_ + n2->h_cost_) {
                        return n1->g_cost_ <= n2->g_cost_;
                    }
                    return n1->g_cost_ + n1->h_cost_ >= n2->g_cost_ +n2->h_cost_;
                }
            };

            struct Focal_compare {
                bool operator()(const LLNode::Ptr n1, const LLNode::Ptr n2) const {
                    if(n1->conf_num_ == n2->conf_num_) {
                        return n1->g_cost_ <= n2->g_cost_;
                    }
                    return n1->conf_num_ >= n2->conf_num_;
                }
            };

            boost::heap::fibonacci_heap<LLNode::Ptr, boost::heap::compare<Open_compare> >::handle_type open_handle_;
            boost::heap::fibonacci_heap<LLNode::Ptr, boost::heap::compare<Focal_compare> >::handle_type focal_handle_;

            bool operator<(const LLNode::Ptr &n) const {
                if(this->loc_ == n->loc_) {
                    return this->timestep_ < n->timestep_;
                }
                return this->loc_ < n->loc_;
            }

            LLNode(int loc, int g_cost, int h_cost, LLNode::Ptr parent, int timestep,
                int conf_num=0);
            ~LLNode()=default;

            int GetFCost() const;
            int GetLoc() const;
            int GetTimeStep() const;
            int GetConfNum() const;
            LLNode::Ptr GetParent() const;
            int GetGCost() const;
            bool InOpenlist() const;

            void SetGCost(int g_cost);
            void SetHCost(int h_cost);
            void SetParent(LLNode::Ptr parent);
            void SetConfNum(int conf_num);
            void SetInOpenlist(bool in_openlist);

            struct Eqnode {
                bool operator()(const LLNode::Ptr n1, const LLNode::Ptr n2) const {
                    return (n1 == n2 || 
                    (n1 && n2 && n1->loc_ == n2->loc_ && n1->timestep_ == n2->timestep_) );
                }
            };

            struct NodeHasher {
                std::size_t operator()(const LLNode::Ptr n) const {
                    std::size_t loc_hash = std::hash<int>()(n->loc_);
                    std::size_t timestep_hash = std::hash<int>()(n->timestep_);
                    return (loc_hash ^ (timestep_hash << 1));
                }
            };
            
        private:
            int loc_;
            int g_cost_;
            int h_cost_;
            LLNode::Ptr parent_;
            int timestep_;
            int conf_num_;
            bool in_openlist_;
        };

    } // CBSH
} //namespace mapf
#endif //LLNODE_H