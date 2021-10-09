#ifndef RLTEST_H
#define RLTEST_H

#include <memory>
#include <vector>

#include "mapf_map/mapf_map.h"
#include "agent/agent.h"

namespace mapf {
    class RLtest {
    public:
        typedef std::shared_ptr<RLtest> Ptr;
        RLtest();
        ~RLtest()=default;
        void TestCbs();
        void TestRL();

    private:
        //地图
        Map::Ptr map_;
        //任务
        std::vector<Agent::Ptr> agents_;
    };
}

#endif //RLTEST_H