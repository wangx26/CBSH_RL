#ifndef CBSHCONFLICT_H
#define CBSHCONFLICT_H

#include <vector>
#include <string>

namespace mapf {
    namespace CBSH {

        class Conflict
        {
        public:
            Conflict()=default;
            Conflict(std::string agent1, std::string agent2, std::string type, int loc1, 
                int loc2, int timestep);
            ~Conflict()=default;

            std::string Type() const ;
            // 返回agent id，index范围0,1
            std::string GetAgent(int index) const;
            // 返回坐标，index范围0,1
            int GetLoc(int index) const;
            int GetTimestep() const;
            std::pair<int, int> GetRg() const;
            // 返回rectangle conflict的时间，index范围0,1
            int GetRectTime(int index) const;

            void SetRectConflict(std::string agent1, std::string agent2, std::string type, int time1, 
                int time2, std::pair<int, int> Rg);
        private:
            std::string agent1_;
            std::string agent2_;
            std::string conflict_type_; // "vertex","edge","rectangle"
            int loc1_;  // vertex conflict只有loc1
            int loc2_;  // edge conflict有loc2
            int timestep_;
            std::pair<int, int> Rg_;    // rectangle
            int time_start1_;   // rectangle
            int time_start2_;   // rectangle
        };

    }
}

#endif // CBSHCONFLICT_H