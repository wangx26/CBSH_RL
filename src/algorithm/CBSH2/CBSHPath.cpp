#include <limits>
#include <boost/heap/fibonacci_heap.hpp>
#include <set>

#include "CBSHPath.h"

namespace mapf {
    namespace CBSH {
        CBSHPath::CBSHPath(std::string id, int start_loc, int goal_loc)
            : id_(id), start_loc_(start_loc), goal_loc_(goal_loc) {
            path_.clear();
        }

        int CBSHPath::Size() const {
            return path_.size();
        }

        int CBSHPath::GetLoc(int index) const {
            return path_[index].first;
        }

        std::string CBSHPath::GetId() const {
            return id_;
        }

        int CBSHPath::GetLastLoc() const {
            return path_.back().first;
        }

        int CBSHPath::GetStartLoc() const {
            return start_loc_;
        }

        int CBSHPath::GetGoalLoc() const {
            return goal_loc_;
        }

        bool CBSHPath::GetSingle(int index) const {
            return path_[index].second;
        }

        std::vector<std::pair<int, bool> > CBSHPath::GetPaths() const {
            return path_;
        }

        bool CBSHPath::IsEmpty() const {
            return path_.empty();
        }

        std::vector<std::list<Constraint> > CBSHPath::GetConstraints() const {
            return constraints_;
        }

        void CBSHPath::AddConstraint(Constraint cons) {
            constraints_[cons.GetTimestep()].push_back(cons);
        }

        void CBSHPath::UpdatePath(std::vector<std::pair<int, bool> > path, int makespan) {
            path_ = path;
            //single_.resize(path.size(), false);
        }

        void CBSHPath::UpdateConstraints(int makespan) {
            for(int i = constraints_.size(); i < makespan + 1; ++i) {
                std::list<mapf::CBSH::Constraint> l;
                constraints_.push_back(l);
            }
        }

        bool CBSHPath::IsConstrainted(int curr_loc, int next_loc, int next_timestep) const {
            if(constraints_.empty()) {
                return false;
            }
            auto s = constraints_.size();
            if(next_timestep < constraints_.size()) {
                for(auto iter = constraints_[next_timestep].begin(); iter != constraints_[next_timestep].end(); ++iter) {
                    if((iter->GetType() == "vertex" && iter->GetLoc(0) == next_loc) ||
                        (iter->GetType() == "edge" && iter->GetLoc(0) == curr_loc && iter->GetLoc(1) == next_loc)) {
                        return true;
                    }
                }
            }
            return false;
        }

        void CBSHPath::SetSingle(int index, bool is_single) {
            path_[index].second = is_single;
        }

        void CBSHPath::InitConstraints(int len) {
            for(int i = constraints_.size(); i < len + 1; ++i) {
                std::list<mapf::CBSH::Constraint> l;
                constraints_.push_back(l);
            }
        }

    } // namespace CBSH
} // namespace mapf