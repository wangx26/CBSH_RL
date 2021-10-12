#include <chrono>
#include <iostream>
#include <fstream>

#include "rltest.h"
#include "CBSH2/CBSHSearch.h"

namespace mapf {
    RLtest::RLtest() {
        conf_.reset(new CBSHConfig());
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