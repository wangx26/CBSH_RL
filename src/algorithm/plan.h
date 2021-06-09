#ifndef PLAN_H
#define PLAN_H


#include <vector>

#include "agent/agent.h"
#include "mapf_map/mapf_map.h"

namespace mapf {
    
    class Plan
    {
    private:
        /* data */
    public:
        typedef std::shared_ptr<Plan> Ptr;

        Plan() = default;
        
        virtual bool MakePlan() {};

        virtual std::vector<Agent::Ptr> GetPlan() {};
    };    
}

#endif // PLAN_H