#include <iostream>
//#include <fstream>
//#include <chrono>
#include <pybind11/pybind11.h>

#include "common/mapf_map/mapf_map.h"
#include "common/agent/agent.h"
//#include "algorithm/CBSH2/CBSHSearch.h"

namespace py = pybind11;

class CBSRL{
private:
    //mapf::CBSH::CBSHSearch search_;
    mapf::Map::Ptr map_;
    int map_height_;
    int map_width_;
    int agent_num_;
public:
    CBSRL(){
        map_.reset(new mapf::Map());
        map_->LoadFileMap("/home/ld/CBSH_RL/data/test.map");
        map_->SetOffset();
        map_height_ = map_->GetHeight();
        map_width_ = map_->GetWidth();
        std::vector<mapf::Agent> agents;
        std::vector<int> all_start, all_goal;
        map_->LoadAgentFile("/home/ld/CBSH_RL/data/test.csv", all_start, all_goal, agent_num_);
        for(int i = 0; i < agent_num_;  ++i) {
            mapf::Agent agent_each(std::to_string(i));
            agent_each.SetStart(all_start[i]);
            agent_each.SetGoal(all_goal[i]);
            agents.emplace_back(agent_each);
            std::cout << all_start[i] << ", " << all_goal[i] << std::endl;
        }
        // search_.reset(new mapf::CBSH::CBSHSearch(map_, agents, true));
    }

    int init(){

        printf("inited ok\n");
        return 0;
    }
    int Reset() {
        //search_->Reset();
        return 0;
    }
    int Step(int a, int t) {
        //search_->Step(a, t);
        return 0;
    }
    bool isDone() {
        //return search_->rl_done_;
        printf("isDone\n");
        return true;
    }
    int FinalReward() {
        //return search_->solution_cost_;
        return 0;
    }
    int Reward() {
        //return search_->curr_node_->GetCollisionNum();
        return 0;
    }
    int* GetState() {
        int s[1];
        /*auto state = search_->GetState();
        int s[len];
        int index = 0;
        for (int k = 0; k <= agent_num_; ++k) {
            for (int i = 0; i < map_height_; ++i) {
                for (int j = 0; j < map_width_; ++j) {
                    s[index] = state[index];
                    ++index;
                }
            }
        }*/
        return s;
    }
    int GetMapHeight() {
        return map_height_;
    }
    int GetMapWidth() {
        return map_width_;
    }
    int GetAgentNum() {
        return agent_num_;
    }
};

PYBIND11_MODULE(cbsrl, m) {
    py::class_<CBSRL>(m, "CBSHRL")
        .def(py::init<>())
        .def("isdone", &CBSRL::isDone)
        .def("get_maph", &CBSRL::GetMapHeight);
}