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

using namespace mapf;

int main(int argc, char** argv) {
    // get options
    namespace po = boost::program_options;
    po::options_description desc("Options");
    desc.add_options()
        ("help", "produce help message")
        ("mapFile,m", po::value<std::string>()->default_value("/home/ld/mapf/data/lak503d.map", "file path of map"))
        ("agentFile,a", po::value<std::string>()->default_value("/home/ld/mapf/data/agents.csv", "file path of agents"))
        ("agentFileToRead,p", po::value<std::string>()->default_value("/home/ld/mapf/data/lak503dmap-100agents-2.csv", "file path of agents"))
        ("agentNum,n", po::value<int>()->default_value(75), "num of agents")
        ("randSeed,r", po::value<int>()->default_value(10), "random seed")
        ("strategy,s", po::value<int>()->default_value(4), "strategy level, [0,1,2,3,4]对应[CBS,PC,CG,DG,WDG]")
        ("focal,f", po::value<float>()->default_value(1.0), "focal threshold")
        ("block,b", po::value<bool>()->default_value(true), "if the agent take goods, block is true")
    ;
    po::variables_map vm;
    try{
        po::store(po::parse_command_line(argc, argv, desc), vm);
    }
    catch(...) {
        std::cout << "Wrong options" << std::endl;
        return 0;
    }
    if(vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }

    // read map
    mapf::Map::Ptr map;
    map.reset(new mapf::Map());
    std::string map_path = vm.at("mapFile").as<std::string>();
    map->LoadFileMap(map_path);
    map->SetOffset();

    // read agents info
    std::vector<mapf::Agent::Ptr> agents;
    int rand_seed = vm.at("randSeed").as<int>();
    
    int agent_num;
    std::vector<int> all_start, all_goal;
    // 随机位置、目标
    //agent_num = vm.at("agentNum").as<int>();
    //all_start = map->RandStart(agent_num, rand_seed);
    //all_goal = map->RandGoal(agent_num, rand_seed);
    
    // 从文件读取
    map->LoadAgentFile(vm.at("agentFileToRead").as<std::string>(), all_start, all_goal, agent_num);
    std::vector<std::string> agent_ids;
    for(int i = 0; i < agent_num;  ++i) {
        agent_ids.push_back(std::to_string(i));
    }

    // record agents
    std::string agents_path = vm.at("agentFile").as<std::string>();
    std::fstream f(agents_path, std::ios_base::out);
    f << agent_num << std::endl;
    for(int i = 0; i < agent_num; ++i) {
        f << map->ToXY(all_start[i]).second << "," << map->ToXY(all_start[i]).first << "," << 
        map->ToXY(all_goal[i]).second << "," << map->ToXY(all_goal[i]).first << std::endl;
    }
    f.close();

    // make plan
    mapf::Plan::Ptr plan;
    auto map_offset = map->GetMoveOffset();
    auto t_s = std::chrono::system_clock::now();
    plan.reset(new mapf::CBSH::CBSHSearch(map, agent_ids, vm.at("strategy").as<int>(), vm.at("focal").as<float>(), 
    false, vm.at("block").as<bool>(), all_start, all_goal ));
    if(plan->MakePlan()) {
        agents = plan->GetPlan();
        auto t_e = std::chrono::system_clock::now();
        auto t_d = std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
        LOG_DEBUG_STREAM("Total plan time: " << double(t_d.count()) * 
        std::chrono::microseconds::period::num / std::chrono::microseconds::period::den);
        mapf::Show sh(map, "/home/ld/mapf/data/images", "/home/ld/mapf/data");
        sh.GenerateImage(agents);
        sh.GenerateVideo();
    };

    // show the result
    return 0;
}