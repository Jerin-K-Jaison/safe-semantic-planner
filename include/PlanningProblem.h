#ifndef PLANNING_PROBLEM_H
#define PLANNING_PROBLEM_H

#include "State.h"
#include "Transition.h"

#include <cstdint>
#include <vector>

class PlanningProblem {
public:

    uint64_t initialState;
    uint64_t goalState;

    std::vector<uint64_t> badStates;

    std::vector<State> states;

    std::vector<Transition> transitions;

    bool isBadState(uint64_t stateId) const;
};

#endif
