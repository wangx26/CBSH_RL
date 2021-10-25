#include <string>

#include <rltest/rltest.h>
#include <config/cbsh_config.h>
#include <mapf_map/mapf_map.h>

int main() {
    mapf::CBSHConfig::Ptr conf;
    conf.reset(new mapf::CBSHConfig());
    // 载入地图
    

    // 载入任务
    std::vector<int> starts, goals;
    int agent_num = conf->GetAgentNum(); 

    // 使用CBS算法

    // 使用CBS随机剪枝算法

    // 使用rl算法
}