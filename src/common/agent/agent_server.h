#ifndef AGENTSERVER_H
#define AGENTSERVER_H

#include <vector>

#include "agent.h"

namespace mapf {

    class AgentServer
    {
    private:
        int agent_num_;
        std::vector<Agent::Ptr> agents_;
    public:
        void LoadAgentScenarios();
    };
}

#endif // AGENTSERVER_H