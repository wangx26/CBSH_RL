#ifndef CBSHPATH_H
#define CBSHPATH_H

#include <vector>
#include <string>
#include <list>
#include <map>

#include "CBSHConstraint.h"
#include "CBSHConflict.h"
#include "mapf_map/mapf_map.h"

namespace mapf {
    namespace CBSH {

        class CBSHPath
        {
        private:
            std::string id_;
            int start_loc_;
            int goal_loc_;
            std::vector<std::pair<int, bool> > path_;   // loc, single记录path点在MDD中是否只可以扩展一条路径
            //std::vector<bool> single_;  // 

            std::vector<std::list<Constraint> > constraints_;

        public:
            CBSHPath(std::string id, int start_loc, int goal_loc);
            CBSHPath()=default;
            ~CBSHPath()=default;

            int Size() const ;
            int GetLoc(int index) const;
            std::string GetId() const;
            int GetLastLoc() const;
            bool GetSingle(int index) const ;
            std::vector<std::pair<int, bool> > GetPaths() const;
            bool IsEmpty() const;
            std::vector<std::list<Constraint> > GetConstraints() const;

            int GetStartLoc() const;
            int GetGoalLoc() const;

            void AddConstraint(Constraint cons);

            void UpdatePath(std::vector<std::pair<int, bool> > path, int makespan);
            void UpdateConstraints(int makespan);
            bool IsConstrainted(int curr_loc, int next_loc, int next_timestep) const;

            void SetSingle(int index, bool is_single);
            void InitConstraints(int len);

            

        };

    }
}

#endif // CBSHPATH_H