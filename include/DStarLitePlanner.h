#ifndef DSTAR_LITE_PLANNER_H
#define DSTAR_LITE_PLANNER_H

#include "PlanningProblem.h"
#include "PlanningResult.h"

#include <cstdint>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>

class DStarLitePlanner {

private:

    // A node stored in the priority queue
    struct QueueNode {

        double k1;
        double k2;
        uint64_t state;

        bool operator>(const QueueNode& other) const {

            if (k1 != other.k1)
                return k1 > other.k1;

            return k2 > other.k2;
        }
    };

    using PriorityQueue =
        std::priority_queue<
            QueueNode,
            std::vector<QueueNode>,
            std::greater<QueueNode>
        >;

    PlanningProblem problem;

    // D* Lite values
    std::unordered_map<uint64_t, double> g;
    std::unordered_map<uint64_t, double> rhs;

    // Graph representation
    std::unordered_map<
        uint64_t,
        std::vector<Transition>
    > outgoing;

    std::unordered_map<
        uint64_t,
        std::vector<Transition>
    > incoming;

    PriorityQueue open;

    double km;

    uint64_t start;
    uint64_t goal;

    const double INF =
        std::numeric_limits<double>::infinity();

    // Calculate Euclidean distance
    double heuristic(
        uint64_t stateA,
        uint64_t stateB
    ) const;

    double getG(uint64_t state) const;

    double getRHS(uint64_t state) const;

    // Calculate priority key
    double calculateKey1(
        uint64_t state
    ) const;

    double calculateKey2(
        uint64_t state
    ) const;

    // Initialize D* Lite
    void initialize();

    // Update one vertex
    void updateVertex(
        uint64_t state
    );

    // Main D* Lite search
    void computeShortestPath();

    // Calculate effective transition cost
    double transitionCost(
        const Transition& transition
    ) const;

    // Select next state while constructing path
    uint64_t chooseNextState(
        uint64_t current
    );

public:

    DStarLitePlanner();

    PlanningResult plan(
        const PlanningProblem& problem
    );

    // Dynamic updates
    void updateGoal(
        uint64_t newGoal
    );

    void updateTransition(
        uint64_t transitionId,
        bool available
    );

    void updateBadStates(
        const std::vector<uint64_t>& badStates
    );
};

#endif
