#include <jsoncpp/json/json.h>
#include <fstream>

#include "cbsh_config.h"
#include "log.h"

namespace mapf {
    CBSHConfig::CBSHConfig() {
        config_path_ = "/home/ld/mapf/data/config/cbsh_config.json";
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
                    if(root["map_path"].isString()) {
                        map_path_ = root["map_path"].asString();
                    }
                    if(root["agent_path"].isString()) {
                        agent_path_ = root["agent_path"].asString();
                    }
                    if(root["random_agent"]["agent_num"].isInt()) {
                        agent_num_ = root["random_agent"]["agent_num"].asInt();
                    }
                    if(root["random_agent"]["random_agent_path"].isString()) {
                        random_agent_path_ = root["random_agent"]["random_agent_path"].asString();
                    }
                    if(root["random_agent"]["random_seed"].isInt()) {
                        random_seed_ = root["random_agent"]["random_seed"].asInt();
                    }
                } else {
                    LOG_ERROR_STREAM("CBSH config parse error.");
                }
            } else {
                LOG_ERROR_STREAM("CBSH config file open error.");
            }
        }
        catch (...) {
            LOG_ERROR_STREAM("CBSH config load catch error.");
        }
    }
}