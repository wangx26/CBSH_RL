#include "agent_server.h"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <regex>
#include <unordered_set>

#include "config/cbsh_config.h"

namespace mapf {
    AgentServer::AgentServer() {
        CBSHConfig::Ptr config;
        config.reset(new CBSHConfig());
        map_path_ = config->GetMapPath();
        train_rate_ = config->GetTrainRate();
        agent_num_ = config->GetAgentNum();
        rand_seed_ = config->GetRandomSeed();
    }

    void AgentServer::LoadAgentScenarios(const std::string &map_name) {
        agents_train_.clear();
        agents_test_.clear();
        std::vector<std::string> filelist;
        filelist = GetFileList(map_path_ + "/" + map_name);
        for (const auto &file: filelist) {
            std::fstream f(map_path_ + file, std::ios_base::in);
            if (f.is_open()) {
                std::vector<std::vector<int> > total;
                std::string line;
                std::getline(f, line);
                while (getline(f, line)) {
                    std::string num;
                    std::vector<int> xy;
                    std::istringstream readstr(line);
                    for (int i = 0; i < 4; ++i) readstr >> num;
                    for (int i = 0; i < 4; ++i) {
                        readstr >> num;
                        xy.push_back(atoi(num.c_str()));
                    }
                    total.push_back(xy);
                }
                int train_num = int(train_rate_ * int(total.size()));
                agents_train_.insert(agents_train_.end(), total.begin(), total.begin() + train_num);
                agents_test_.insert(agents_test_.end(), total.begin() + train_num, total.end());
            } else {
                std::cout << "Open file fail: " << file << std::endl;
            }
            f.close();
        }
    }

    std::vector<std::string> AgentServer::GetFileList(std::string path) const {
        std::vector<std::string> filelist;
        std::regex txt_regex(".*\\.scen");
        try {
            boost::filesystem::path dir(path);
            if (boost::filesystem::exists(dir) && boost::filesystem::is_directory(dir)) {
                boost::filesystem::directory_iterator end_iter;
                for (boost::filesystem::directory_iterator it(dir); it != end_iter; ++it) {
                    if (boost::filesystem::is_regular(it->path()) && std::regex_match(it->path().string(), txt_regex)) {
                        filelist.push_back(it->path().string());
                    }
                }
            } else {
                std::cout << "Agent Path not exist: " << path << std::endl;
            }
        }
        catch (...) {
            std::cout << "Path not a directory: " << path << std::endl;
        }
        return filelist;
    }

    void AgentServer::AgentTrain(std::vector<std::pair<int, int> > &starts, std::vector<std::pair<int, int> > &goals) const {
        std::srand(rand_seed_);
        std::unordered_set<int> st;
        while (st.size() < agent_num_) {
            int index = std::rand() % agents_train_.size();
            st.insert(index);
        }
        starts.clear();
        goals.clear();
        for (const auto &index: st) {
            auto a = agents_train_[index];
            starts.emplace_back(std::make_pair(a[0], a[1]));
            goals.emplace_back(std::make_pair(a[2], a[3]));
        }
    }

    void AgentServer::AgentTest(std::vector<std::pair<int, int> > &starts, std::vector<std::pair<int, int> > &goals) const {
        std::srand(rand_seed_);
        std::unordered_set<int> st;
        while (st.size() < agent_num_) {
            int index = std::rand() % agents_test_.size();
            st.insert(index);
        }
        starts.clear();
        goals.clear();
        for (const auto &index: st) {
            auto a = agents_test_[index];
            starts.emplace_back(std::make_pair(a[0], a[1]));
            goals.emplace_back(std::make_pair(a[2], a[3]));
        }
    }
}