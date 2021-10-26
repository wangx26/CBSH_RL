#include <jsoncpp/json/json.h>
#include <fstream>

#include "cbsh_config.h"
// #include "log.h"

namespace mapf {
    CBSHConfig::CBSHConfig() {
        config_path_ = "/home/ld/CBSH_RL/data/config/cbsh_config.json";
        LoadConfig();
    }

    void CBSHConfig::LoadConfig() {
        try {
            Json::Value root;
            Json::Reader reader;
            std::fstream fs(config_path_);
            if(fs.is_open()) {
                if (reader.parse(fs, root)) {
                    if(root["strategy"].isString()) {
                        strategy_ = root["strategy"].asString();
                    }
                    if(root["focal"].isDouble()) {
                        focal_ = root["focal"].asDouble();
                    }
                    if(root["rectangle_reasoning"].isBool()) {
                        rectangle_reasoning_ = root["rectangle_reasoning"].asBool();
                    }
                    if(root["trainmap_path"].isString()) {
                        trainmap_path_ = root["trainmap_path"].asString();
                    }
                    if(root["testmap_path"].isString()) {
                        testmap_path_ = root["testmap_path"].asString();
                    }
                    if(root["testmap_name"].isString()) {
                        testmap_name_ = root["testmap_name"].asString();
                    }
                    if(root["agent_path"].isString()) {
                        agent_path_ = root["agent_path"].asString();
                    }
                    if(root["agent_num"].isInt()) {
                        agent_num_ = root["agent_num"].asInt();
                    }
                    if(root["random_agent"]["random_agent_path"].isString()) {
                        random_agent_path_ = root["random_agent"]["random_agent_path"].asString();
                    }
                    if(root["random_agent"]["random_seed"].isInt()) {
                        randseed_agent_ = root["random_agent"]["random_seed"].asInt();
                    }
                    if (root["randseedmap"].isInt()) {
                        randseed_map_ = root["randseedmap"].asInt();
                    }
                    if(root["train_rate"].isDouble()) {
                        train_rate_ = root["train_rate"].asDouble();
                    }
                    if(root["train"].isBool()) {
                        train_ = root["train"].asBool();
                    }
                    if(root["test_data_path"].isString()) {
                        test_data_path_ = root["test_data_path"].asString();
                    }
                } else {
                    // LOG_ERROR_STREAM("CBSH config parse error.");
                }
            } else {
                // LOG_ERROR_STREAM("CBSH config file open error.");
            }
        }
        catch (...) {
            // LOG_ERROR_STREAM("CBSH config load catch error.");
        }
    }
}