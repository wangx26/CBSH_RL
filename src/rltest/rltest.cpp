#include <chrono>
#include <iostream>
#include <fstream>

#include "rltest.h"
#include "CBSH2/CBSHSearch.h"

namespace mapf {
    RLtest::RLtest() {
        conf_.reset(new CBSHConfig());
        LoadMap();
        LoadAgent();
    }

    void RLtest::LoadMap() {
        std::string test_map_path = conf_->GetTestMapPath();
        std::string test_map_name = conf_->GetTestMapName();
        map_.reset(new Map());
        map_->LoadFileMap(test_map_path + test_map_name);
    }

    void RLtest::LoadAgent() {
        agent_server_.reset(new AgentServer());
        std::string test_map_name = conf_->GetTestMapName();
        agent_server_->LoadAgentScenarios(test_map_name);
        std::vector<std::pair<int, int> > starts, goals;
        agent_server_->AgentTest(starts, goals);
        Agent::Ptr agent;
        agents_.clear();
        for (int i = 0; i < goals.size(); ++i) {
            agent.reset(new Agent(std::to_string(i)));
            agent->SetStart(map_->ToLoc(starts[i].first, starts[i].second));
            agent->SetGoal(map_->ToLoc(goals[i].first, goals[i].second));
            agents_.push_back(agent);
        }
    }

    void RLtest::TestCbs() const {
        CBSH::CBSHSearch::Ptr cbsh;
        cbsh.reset(new CBSH::CBSHSearch(map_, agents_, true));

        auto s_time = std::chrono::system_clock::now();
        auto success = cbsh->MakePlan();
        auto path = cbsh->GetPlan();
        auto end_time = std::chrono::system_clock::now();

        auto chrono_t = std::chrono::duration_cast<std::chrono::microseconds>(end_time - s_time);
        auto t = double(chrono_t.count()) * std::chrono::microseconds::period::num /
                 std::chrono::microseconds::period::den;
        auto l = cbsh->GetGCost();
        auto makespan = cbsh->GetMakeSpan();
        WriteData(t, l, makespan);
    }

    void RLtest::TestRand() const {
        //
    }

    void RLtest::TestRL() const {
        //
    }

    void RLtest::WriteData(double t, int g_cost, int make_span) const {
        auto data_path = conf_->GetTestDataPath();
        auto agent_num = conf_->GetAgentNum();
        auto strategy = conf_->GetStrategy();
        if (strategy == "NONE") {
            strategy = "CBS";
        } else if (strategy == "WDG") {
            strategy = "CBSH2";
        } else if (strategy == "DG") {
            strategy = "CBSH";
        } else if (strategy == "RL") {
            strategy = "RL";
        }
        std::ofstream f;
        f.open(data_path, std::ios::app);
        if (f.is_open()) f << strategy << "," << agent_num << "," << t << "," << g_cost << "," << make_span << ",\n";
        f.close();
    }
}