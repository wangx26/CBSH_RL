#ifndef RLTEST_H
#define RLTEST_H

#include <memory>
#include <vector>

#include "mapf_map/mapf_map.h"
#include "agent/agent.h"
#include "config/cbsh_config.h"

namespace mapf {
    class RLtest {
    public:
        typedef std::shared_ptr<RLtest> Ptr;
        RLtest();
        ~RLtest()=default;
        void TestCbs() const;
        void TestRL() const;
        void WriteData(double t, int g_cost, int make_span) const;

    private:
        //地图
        Map::Ptr map_;
        //任务
        std::vector<Agent::Ptr> agents_;
        CBSHConfig::Ptr conf_;
    };
}

#endif //RLTEST_H