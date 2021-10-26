#include <string>

#include "rltest/rltest.h"

int main() {
    mapf::RLtest::Ptr  rltest;
    rltest.reset(new mapf::RLtest());

    // 使用CBS算法
    rltest->TestCbs();

    // 使用CBS随机剪枝算法

    // 使用rl算法
}