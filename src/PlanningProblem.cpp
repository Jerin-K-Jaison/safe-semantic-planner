#include "PlanningProblem.h"

bool PlanningProblem::isBadState(
    uint64_t stateId
) const {

    for (uint64_t badState : badStates) {

        if (badState == stateId) {
            return true;
        }
    }

    return false;
}
