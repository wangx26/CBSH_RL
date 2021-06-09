#include "CBSHConflict.h"

namespace mapf {
    namespace CBSH {
        Conflict::Conflict(std::string agent1, std::string agent2, std::string type, int loc1, 
                int loc2, int timestep)
                : agent1_(agent1), agent2_(agent2), conflict_type_(type), loc1_(loc1),
                  loc2_(loc2), timestep_(timestep)
        {
            //
        }

        void Conflict::SetRectConflict(std::string agent1, std::string agent2, std::string type, int time1, 
                int time2, std::pair<int, int> Rg) 
        {
            agent1_ = agent1;
            agent2_ = agent2;
            conflict_type_ = type;
            time_start1_ = time1;
            time_start2_ = time2;
            Rg_ = Rg;
        }

        std::string Conflict::Type() const {
            return conflict_type_;
        }

        std::string Conflict::GetAgent(int index) const {
            if(index == 0) {
                return agent1_;
            }
            else {
                return agent2_;
            }
        }

        int Conflict::GetLoc(int index) const {
            if(index == 0) {
                return loc1_;
            }
            else {
                return loc2_;
            }
        }

        int Conflict::GetTimestep() const {
            return timestep_;
        }

        std::pair<int, int> Conflict::GetRg() const {
            return Rg_;
        }

        int Conflict::GetRectTime(int index) const {
            if(index == 0) {
                return time_start1_;
            }
            else {
                return time_start2_;
            }
        }

    } // namespace CBSH
} // namespace mapf