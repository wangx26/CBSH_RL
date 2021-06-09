#ifndef CBSHCONSTRAINT_H
#define CBSHCONSTRAINT_H

#include <vector>
#include <string>

namespace mapf {
    namespace CBSH {
        class Constraint
        {
        private:
            std::string agent_id_;
            std::string constraint_type_;   // "vertex","edge","rectangle"
            int loc1_;  // vertex conflict只有loc1
            int loc2_;  // edge conflict有loc2
            int timestep_;
        public:
            Constraint();
            ~Constraint()=default;
            Constraint(const Constraint &c);
            Constraint(std::string id, std::string type);

            std::string GetAgentId() const;
            int GetTimestep() const;
            int GetLoc(int index) const;
            std::string GetType() const;
            void SetConstraint(int loc1, int loc2, int timestep);

            bool operator<(const Constraint &c) const {
                return agent_id_ < c.agent_id_ && constraint_type_ < c.constraint_type_ &&
                    loc1_ < c.loc1_ && loc2_ < c.loc2_ && timestep_ < c.timestep_;
            }
        };
    }
}

#endif // CBSHCONSTRAINT_H