#include "agent.h"


namespace mapf {

    Agent::Agent(std::string id)
        : id_(id)
    {
    }

    std::string Agent::GetId() const {
        return id_;
    }

    int Agent::GetStart() const {
        return start_loc_;
    }

    int Agent::GetGoal() const {
        return goal_loc_;
    }

    void Agent::SetStart(int start_loc) {
        start_loc_ = start_loc;
    }

    void Agent::SetGoal(int goal_loc) {
        goal_loc_ = goal_loc;
    }

    std::vector<int> Agent::GetPath() const {
        return path_;
    }

    void Agent::SetPaths(std::vector<std::pair<int, bool> > path) {
        path_.clear();
        for(auto p: path) {
            path_.push_back(p.first);
        }
    }

}