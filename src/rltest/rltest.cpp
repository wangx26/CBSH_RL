#include <chrono>

#include "rltest.h"
#include "CBSH2/CBSHSearch.h"
#include "config/cbsh_config.h"

namespace mapf {
    RLtest::RLtest() {
        //
    }

    void RLtest::TestCbs() {
        CBSHConfig::Ptr conf;
        conf.reset(new CBSHConfig());
        std::string map_name = conf->GetTestMapName();
        CBSH::CBSHSearch::Ptr cbsh;
        cbsh.reset(new CBSH::CBSHSearch(map_, agents_, true));

        auto s_time = std::chrono::system_clock::now();
        auto success = cbsh->MakePlan();
        auto path = cbsh->GetPlan();
        auto end_time = std::chrono::system_clock::now();
    }

    void RLtest::TestRL() {
        //
    }
}