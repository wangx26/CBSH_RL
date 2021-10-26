#ifndef AGENTSERVER_H
#define AGENTSERVER_H

#include <vector>
#include <memory>

namespace mapf {

    class AgentServer
    {
    public:
        typedef std::shared_ptr<AgentServer> Ptr;
        AgentServer();
        ~AgentServer()=default;
        void LoadAgentScenarios(const std::string &map_name);

        void AgentTrain(std::vector<std::pair<int, int> > &starts, std::vector<std::pair<int, int> > &goals) const;
        void AgentTest(std::vector<std::pair<int, int> > &starts, std::vector<std::pair<int, int> > &goals) const;
    private:
        int agent_num_;
        float train_rate_;
        std::string map_path_;
        int randseed_agent_;
        std::vector<std::vector<int> > agents_train_;
        std::vector<std::vector<int> > agents_test_;
        std::vector<std::string> GetFileList(std::string path) const;
    };
}

#endif // AGENTSERVER_H