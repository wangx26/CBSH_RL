#ifndef HTABLE_H
#define HTABLE_H

#include <set>

#include "CBSHConstraint.h"

namespace mapf {
    namespace CBSH {
        class Htable
        {
        private:
            std::string agent_;
            std::set<Constraint> agent_set_;
        public:
            Htable(std::string agent,
                std::set<Constraint> agent_set);
            ~Htable()=default;

            bool operator<(const Htable &h) const {
                if(agent_ < h.agent_ ) {
                    return true;
                }
                else if(agent_set_.size() < h.agent_set_.size()) {
                    return true;
                }
                else if(agent_set_.size() == h.agent_set_.size()) {
                    bool less = true;
                    auto iter1 = agent_set_.begin();
                    auto iter2 = h.agent_set_.begin();
                    for(; iter1 != agent_set_.end() && iter2 != h.agent_set_.end(); ++iter1, ++iter2) {
                        if(!(*iter1 < *iter2)) {
                            less = false;
                            break;
                        }
                    }
                    return less;
                }
                return false;
            }
        };
    } // namespace CBSH
} // namespace mapf
#endif //HTABLE_H