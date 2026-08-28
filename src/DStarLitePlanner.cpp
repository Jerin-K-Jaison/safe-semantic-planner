#include "DStarLitePlanner.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <unordered_set>

DStarLitePlanner::DStarLitePlanner()
    : km(0.0),
      start(0),
      goal(0) {
}


// --------------------------------------------------
// Get g value
// --------------------------------------------------

double DStarLitePlanner::getG(
    uint64_t state
) const {

    auto it = g.find(state);

    if (it == g.end())
        return INF;

    return it->second;
}


// --------------------------------------------------
// Get rhs value
// --------------------------------------------------

double DStarLitePlanner::getRHS(
    uint64_t state
) const {

    auto it = rhs.find(state);

    if (it == rhs.end())
        return INF;

    return it->second;
}


// --------------------------------------------------
// Euclidean heuristic
// --------------------------------------------------

double DStarLitePlanner::heuristic(
    uint64_t stateA,
    uint64_t stateB
) const {

    const State* a = nullptr;
    const State* b = nullptr;

    for (const auto& state : problem.states) {

        if (state.id == stateA)
            a = &state;

        if (state.id == stateB)
            b = &state;
    }

    if (a == nullptr || b == nullptr)
        return 0.0;

    double sum = 0.0;

    size_t dimensions =
        std::min(
            a->embedding.size(),
            b->embedding.size()
        );

    for (size_t i = 0;
         i < dimensions;
         ++i) {

        double difference =
            a->embedding[i] -
            b->embedding[i];

        sum += difference * difference;
    }

    return std::sqrt(sum);
}


// --------------------------------------------------
// First priority key
// --------------------------------------------------

double DStarLitePlanner::calculateKey1(
    uint64_t state
) const {

    double minimum =
        std::min(
            getG(state),
            getRHS(state)
        );

    return minimum
           + heuristic(start, state)
           + km;
}


// --------------------------------------------------
// Second priority key
// --------------------------------------------------

double DStarLitePlanner::calculateKey2(
    uint64_t state
) const {

    return std::min(
        getG(state),
        getRHS(state)
    );
}


// --------------------------------------------------
// Effective transition cost
// --------------------------------------------------

double DStarLitePlanner::transitionCost(
    const Transition& transition
) const {

    double safety =
        std::max(
            0.0,
            std::min(
                1.0,
                transition.safety
            )
        );

    double reliability =
        std::max(
            0.0,
            std::min(
                1.0,
                transition.reliability
            )
        );

    double safetyPenalty =
        1.0 - safety;

    double reliabilityPenalty =
        1.0 - reliability;

    return transition.cost
           + 2.0 * safetyPenalty
           + reliabilityPenalty;
}


// --------------------------------------------------
// Initialize D* Lite
// --------------------------------------------------

void DStarLitePlanner::initialize() {

    g.clear();
    rhs.clear();

    while (!open.empty())
        open.pop();

    km = 0.0;

    start =
        problem.initialState;

    goal =
        problem.goalState;

    // Every state initially has infinite cost
    for (const auto& state :
         problem.states) {

        g[state.id] = INF;
        rhs[state.id] = INF;
    }

    // Goal has zero cost to itself
    rhs[goal] = 0.0;

    open.push({
        calculateKey1(goal),
        calculateKey2(goal),
        goal
    });
}


// --------------------------------------------------
// Update vertex
// --------------------------------------------------

void DStarLitePlanner::updateVertex(
    uint64_t state
) {

    if (state != goal) {

        double best = INF;

        // Look at transitions leaving this state
        auto it =
            outgoing.find(state);

        if (it != outgoing.end()) {

            for (const auto& transition :
                 it->second) {

                if (!transition.available)
                    continue;

                if (problem.isBadState(
                        transition.from))
                    continue;

                if (problem.isBadState(
                        transition.to))
                    continue;

                double candidate =
                    transitionCost(
                        transition
                    )
                    + getG(
                        transition.to
                    );

                best =
                    std::min(
                        best,
                        candidate
                    );
            }
        }

        rhs[state] = best;
    }

    if (getG(state) !=
        getRHS(state)) {

        open.push({
            calculateKey1(state),
            calculateKey2(state),
            state
        });
    }
}


// --------------------------------------------------
// Compute shortest path
// --------------------------------------------------

void DStarLitePlanner::computeShortestPath() {

    size_t iterations = 0;

    const size_t MAX_ITERATIONS = 100000;

    while (!open.empty() && iterations < MAX_ITERATIONS) {

        QueueNode current = open.top();
        open.pop();

        uint64_t u = current.state;

        double oldK1 = current.k1;
        double oldK2 = current.k2;

        double newK1 = calculateKey1(u);
        double newK2 = calculateKey2(u);

        // If the key is outdated, insert the updated key.
        if (oldK1 < newK1 ||
            (oldK1 == newK1 && oldK2 < newK2)) {

            open.push({
                newK1,
                newK2,
                u
            });

            iterations++;
            continue;
        }

        if (getG(u) > getRHS(u)) {

            // State becomes locally consistent.
            g[u] = getRHS(u);

            auto it = incoming.find(u);

            if (it != incoming.end()) {

                for (const auto& transition : it->second) {

                    updateVertex(transition.from);
                }
            }

        } else {

            // State becomes over-consistent.
            g[u] = INF;

            updateVertex(u);

            auto it = incoming.find(u);

            if (it != incoming.end()) {

                for (const auto& transition : it->second) {

                    updateVertex(transition.from);
                }
            }
        }

        iterations++;

        // Once the start state has a consistent value,
        // the shortest path has been calculated.
        if (!std::isinf(getG(start)) &&
            getG(start) == getRHS(start)) {

            break;
        }
    }
}
// --------------------------------------------------
// Choose next state
// --------------------------------------------------

uint64_t DStarLitePlanner::chooseNextState(
    uint64_t current
) {

    double bestValue = INF;

    uint64_t bestState =
        current;

    auto it =
        outgoing.find(current);

    if (it == outgoing.end())
        return current;

    for (const auto& transition :
         it->second) {

        if (!transition.available)
            continue;

        if (problem.isBadState(
                transition.to))
            continue;

        double value =
            transitionCost(
                transition
            )
            + getG(
                transition.to
            );

        if (value < bestValue) {

            bestValue =
                value;

            bestState =
                transition.to;
        }
    }

    return bestState;
}


// --------------------------------------------------
// Main planning function
// --------------------------------------------------

PlanningResult DStarLitePlanner::plan(
    const PlanningProblem& inputProblem
) {

    auto begin =
        std::chrono::high_resolution_clock::now();

    problem =
        inputProblem;

    outgoing.clear();
    incoming.clear();

    // Build graph
    for (const auto& transition :
         problem.transitions) {

        outgoing[
            transition.from
        ].push_back(
            transition
        );

        incoming[
            transition.to
        ].push_back(
            transition
        );
    }

    PlanningResult result;

    // Start or goal cannot be bad
    if (problem.isBadState(
            problem.initialState) ||
        problem.isBadState(
            problem.goalState)) {

        return result;
    }

    // Initialize
    initialize();

    // Run D* Lite
    computeShortestPath();

    // No path exists
    if (std::isinf(
            getG(start))) {

        auto end =
            std::chrono::high_resolution_clock::now();

        result.planningTimeMs =
            std::chrono::duration<
                double,
                std::milli
            >(end - begin).count();

        return result;
    }

    uint64_t current =
        start;

    result.statePath.push_back(
        current
    );

    std::unordered_set<
        uint64_t
    > visited;

    visited.insert(current);

    double minimumSafety =
        1.0;

    // Construct path
    while (current != goal) {

        uint64_t next =
            chooseNextState(
                current
            );

        // No valid transition
        if (next == current)
            break;

        // Never enter a bad state
        if (problem.isBadState(next))
            break;

        // Prevent loops
        if (visited.count(next))
            break;

        visited.insert(next);

        // Find selected transition
        for (const auto& transition :
             outgoing[current]) {

            if (transition.to == next &&
                transition.available) {

                result.transitionPath.push_back(
                    transition.id
                );

                result.totalCost +=
                    transition.cost;

                minimumSafety =
                    std::min(
                        minimumSafety,
                        transition.safety
                    );

                break;
            }
        }

        current =
            next;

        result.statePath.push_back(
            current
        );
    }

    result.success =
        (current == goal);

    result.safetyScore =
        minimumSafety;

    result.exploredStates =
        visited.size();

    auto end =
        std::chrono::high_resolution_clock::now();

    result.planningTimeMs =
        std::chrono::duration<
            double,
            std::milli
        >(end - begin).count();

    return result;
}


// --------------------------------------------------
// Dynamic goal update
// --------------------------------------------------

void DStarLitePlanner::updateGoal(
    uint64_t newGoal
) {

    problem.goalState =
        newGoal;

    initialize();

    computeShortestPath();
}


// --------------------------------------------------
// Dynamic transition update
// --------------------------------------------------

void DStarLitePlanner::updateTransition(
    uint64_t transitionId,
    bool available
) {

    for (auto& transition :
         problem.transitions) {

        if (transition.id ==
            transitionId) {

            transition.available =
                available;

            break;
        }
    }

    outgoing.clear();
    incoming.clear();

    for (const auto& transition :
         problem.transitions) {

        outgoing[
            transition.from
        ].push_back(
            transition
        );

        incoming[
            transition.to
        ].push_back(
            transition
        );
    }

    initialize();

    computeShortestPath();
}


// --------------------------------------------------
// Dynamic bad-state update
// --------------------------------------------------

void DStarLitePlanner::updateBadStates(
    const std::vector<uint64_t>& badStates
) {

    problem.badStates =
        badStates;

    initialize();

    computeShortestPath();
}