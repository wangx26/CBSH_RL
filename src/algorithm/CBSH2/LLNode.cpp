#include "LLNode.h"

namespace mapf {
    namespace CBSH {
        int LLNode::GetFCost() const {
            return g_cost_ + h_cost_;
        }

        LLNode::LLNode(int loc, int g_cost, int h_cost, LLNode::Ptr parent, int timestep,
                int conf_num)
                : loc_(loc), g_cost_(g_cost), h_cost_(h_cost), parent_(parent),
                  timestep_(timestep), conf_num_(conf_num)
        {
            //
        }

        int LLNode::GetLoc() const {
            return loc_;
        }

        int LLNode::GetTimeStep() const {
            return timestep_;
        }

        int LLNode::GetConfNum() const {
            return conf_num_;
        }

        LLNode::Ptr LLNode::GetParent() const {
            return parent_;
        }

        int LLNode::GetGCost() const {
            return g_cost_;
        }

        bool LLNode::InOpenlist() const {
            return in_openlist_;
        }

        void LLNode::SetGCost(int g_cost) {
            g_cost_ = g_cost;
        }

        void LLNode::SetHCost(int h_cost) {
            h_cost_ = h_cost;
        }

        void LLNode::SetParent(LLNode::Ptr parent) {
            parent_ = parent;
        }

        void LLNode::SetConfNum(int conf_num) {
            conf_num_ = conf_num;
        }

        void LLNode::SetInOpenlist(bool in_openlist) {
            in_openlist_ = in_openlist;
        }

    } // namespace CBSH
} // namespace mapf