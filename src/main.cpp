#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>
#include <chrono>

#include "mapf_map/mapf_map.h"
#include "agent/agent.h"
#include "plan.h"
#include "CBSH2/CBSHSearch.h"
#include "show.h"
#include "log.h"
#include "cbsh_config.h"

using namespace mapf;
/*
int main(int argc, char** argv) {
    CBSHConfig::Ptr cbsh_config;
    cbsh_config.reset(new CBSHConfig());

    // read map
    mapf::Map::Ptr map;
    map.reset(new mapf::Map());
    std::string map_path = cbsh_config->GetMapPath();
    map->LoadFileMap(map_path);
    map->SetOffset();

    // read agents info
    std::vector<mapf::Agent::Ptr> agents;
    int rand_seed = cbsh_config->GetRandomSeed();

    int agent_num;
    std::vector<int> all_start, all_goal;
    // 随机位置、目标
    //agent_num = cbsh_config->GetAgentNum();
    //all_start = map->RandStart(agent_num, rand_seed);
    //all_goal = map->RandGoal(agent_num, rand_seed);

    // 从文件读取
    map->LoadAgentFile(cbsh_config->GetAgentPath(), all_start, all_goal, agent_num);
    std::vector<std::string> agent_ids;
    for(int i = 0; i < agent_num;  ++i) {
        Agent::Ptr agent_each;
        agent_each.reset(new Agent(std::to_string(i)));
        agent_each->SetStart(all_start[i]);
        agent_each->SetGoal(all_goal[i]);
        agents.emplace_back(agent_each);
    }

    // record agents
    std::string agents_record_path = cbsh_config->GetRandomAgentPath();
    std::fstream f(agents_record_path, std::ios_base::out);
    f << agent_num << std::endl;
    for(int i = 0; i < agent_num; ++i) {
        f << map->ToXY(all_start[i]).second << "," << map->ToXY(all_start[i]).first << "," <<
        map->ToXY(all_goal[i]).second << "," << map->ToXY(all_goal[i]).first << std::endl;
    }
    f.close();

    // make plan
    mapf::Plan::Ptr plan;
    auto t_s = std::chrono::system_clock::now();
    plan.reset(new mapf::CBSH::CBSHSearch(map, agents, true ));
    if(plan->MakePlan()) {
        auto result = plan->GetPlan();
        auto t_e = std::chrono::system_clock::now();
        auto t_d = std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
        LOG_DEBUG_STREAM("Total plan time: " << double(t_d.count()) *
        std::chrono::microseconds::period::num / std::chrono::microseconds::period::den);
        //mapf::Show sh(map, "/home/wolf/CBSH_RL/data/images", "/home/wolf/CBSH_RL/data");
        //sh.GenerateImage(result);
        //sh.GenerateVideo();
    };

    // show the result
    return 0;
}*/
class CBSRL{
private:
    mapf::CBSH::CBSHSearch::Ptr search_;
public:
    CBSRL(){}

    void init(){
        CBSHConfig::Ptr cbsh_config;
        cbsh_config.reset(new CBSHConfig());
        // read map
        mapf::Map::Ptr map;
        map.reset(new mapf::Map());
        std::string map_path = cbsh_config->GetMapPath();
        map->LoadFileMap(map_path);
        map->SetOffset();
        // read agents info
        std::vector<mapf::Agent::Ptr> agents;
        int agent_num;
        std::vector<int> all_start, all_goal;
        map->LoadAgentFile(cbsh_config->GetAgentPath(), all_start, all_goal, agent_num);
        for(int i = 0; i < agent_num;  ++i) {
            Agent::Ptr agent_each;
            agent_each.reset(new Agent(std::to_string(i)));
            agent_each->SetStart(all_start[i]);
            agent_each->SetGoal(all_goal[i]);
            agents.emplace_back(agent_each);
        }
        search_.reset(new mapf::CBSH::CBSHSearch(map, agents, true));
        printf("inited ok\n");
    }

    void Reset() {
        search_->Reset();
    }

    bool Step(int a1, int a2, int t) {
        return search_->Step(a1, a2, t);
    }

    bool isDone() {
        return search_->rl_done_;
    }

    int FinalReward() {
        return search_->solution_cost_;
    }

    std::vector<std::vector<std::vector<int> > > GetState() {
        //return search_->;
    }
};
extern "C" {
    CBSRL rl;

    void init(){
        rl.init();
    }
    void print_msg(const char* s) {
        std::cout<<s<<std::endl;
    }
}