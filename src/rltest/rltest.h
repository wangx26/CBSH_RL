#ifndef RLTEST_H
#define RLTEST_H

#include <memory>
#include <vector>

#include "mapf_map/mapf_map.h"
#include "agent/agent.h"
#include "cbsh_config.h"
#include "agent/agent_server.h"

namespace mapf {
    class RLtest {
    public:
        typedef std::shared_ptr<RLtest> Ptr;
        RLtest();
        ~RLtest()=default;
        void TestCbs() const;
        void TestRand() const;
        void TestRL() const;
        void WriteData(double t, int g_cost, int make_span, int hl) const;

    private:
        //地图
        Map::Ptr map_;
        float map_obs_;
        //任务
        std::vector<Agent::Ptr> agents_;
        CBSHConfig::Ptr conf_;
        AgentServer::Ptr agent_server_;

        void LoadMap();
        void LoadAgent();
    };
}

#endif //RLTEST_H