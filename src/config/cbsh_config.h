#ifndef CBSHCONFIG_H
#define CBSHCONFIG_H

#include <memory>
namespace mapf {
    class CBSHConfig
    {
    private:
        std::string config_path_;

        std::string strategy_;  // CBS,PC,CG,DG,WDG
        float focal_;
        bool rectangle_reasoning_;
        std::string map_path_;
        std::string agent_path_;

        int agent_num_;
        std::string random_agent_path_;
        int random_seed_;
    public:
        typedef std::shared_ptr<CBSHConfig> Ptr;

        CBSHConfig();
        ~CBSHConfig()=default;

        void LoadConfig();

        std::string GetStrategy() const {
            return strategy_;
        }
        float GetFocal() const {
            return focal_;
        }
        bool GetRectangle() const {
            return rectangle_reasoning_;
        }
        std::string GetMapPath() const {
            return map_path_;
        }
        std::string GetAgentPath() const {
            return agent_path_;
        }
        int GetAgentNum() const {
            return agent_num_;
        }
        std::string GetRandomAgentPath() const {
            return random_agent_path_;
        }
        int GetRandomSeed() const {
            return random_seed_;
        }
    };
}
#endif  //CBSHCONFIG_H