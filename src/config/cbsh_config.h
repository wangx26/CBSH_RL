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
        std::string trainmap_path_;
        std::string testmap_path_;
        std::string testmap_name_;
        std::string agent_path_;
        std::string test_data_path_;

        int agent_num_;
        std::string random_agent_path_;
        int random_seed_;

        float train_rate_;
        bool train_;
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
        std::string GetTrainMapPath() const {
            return trainmap_path_;
        }
        std::string GetTestMapPath() const {
            return testmap_path_;
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
        float GetTrainRate() const {
            return train_rate_;
        }
        bool GetTrain() const {
            return train_;
        }
        std::string GetTestMapName() const {
            return testmap_name_;
        }
        std::string GetTestDataPath() const {
            return test_data_path_;
        }
    };
}
#endif  //CBSHCONFIG_H