#ifndef RLTEST_H
#define RLTEST_H

#include <memory>
#include <vector>

namespace mapf {
    class RLtest {
    public:
        typedef std::shared_ptr<RLtest> Ptr;
        RLtest();
        ~RLtest()=default;
        void TestCbs();
        void TestRL();

    private:
        //地图
        std::vector<int> map_;
        //任务
    };
}

#endif //RLTEST_H