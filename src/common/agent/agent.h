#ifndef AGENT_H
#define AGENT_H


#include <vector>
#include <string>
#include <boost/smart_ptr.hpp>

namespace mapf {

    class Agent
    {
    private:
        std::string id_;
        int start_loc_;
        int goal_loc_;
        std::vector<int> path_;
    public:
        typedef std::shared_ptr<Agent> Ptr;
        typedef std::shared_ptr<const Agent> ConstPtr;

        Agent(std::string id);
        ~Agent() = default;

        std::string GetId() const ;
        int GetStart() const ;
        int GetGoal() const ;
        std::vector<int> GetPath() const;

        void SetStart(int start_loc);
        void SetGoal(int goal_loc);
        void SetPaths(std::vector<std::pair<int, bool> > path);
    };
}

#endif // AGENT_H