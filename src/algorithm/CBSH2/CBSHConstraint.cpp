#include "CBSHConstraint.h"

namespace mapf {
    namespace CBSH {
        Constraint::Constraint()
            : agent_id_(""), constraint_type_(""), loc1_(0), loc2_(0), timestep_(0)
        {
            //
        }

        Constraint::Constraint(std::string id, std::string type) {
            agent_id_ = id;
            constraint_type_ = type;
        }

        Constraint::Constraint(const Constraint &c) {
            agent_id_ = c.agent_id_;
            constraint_type_ = c.constraint_type_;
            loc1_ = c.loc1_;
            loc2_ = c.loc2_;
            timestep_ = c.timestep_;
        }

        std::string Constraint::GetAgentId() const {
            return agent_id_;
        }

        int Constraint::GetTimestep() const {
            return timestep_;
        }

        void Constraint::SetConstraint(int loc1, int loc2, int timestep) {
            loc1_ = loc1;
            loc2_ = loc2;
            timestep_ = timestep;
        }

        int Constraint::GetLoc(int index) const {
            if(index == 0) {
                return loc1_;
            }
            else{
                return loc2_;
            }
        }

        std::string Constraint::GetType() const {
            return constraint_type_;
        }

    } // namespace CBSH
} // namespace mapf