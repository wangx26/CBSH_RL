#include "Htable.h"

namespace mapf {
    namespace CBSH {
        Htable::Htable(std::string agent, std::set<CBSH::Constraint> agent_set)
            : agent_(agent), agent_set_(agent_set)
        {
            //
        }
    } // namespace CBSH
}// namespace mapf