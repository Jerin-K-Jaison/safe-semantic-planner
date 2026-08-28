#include "DStarLitePlanner.h"

#include <iostream>
#include <iomanip>

void printResult(
    const PlanningResult& result
) {

    std::cout << "\n";
    std::cout << "====================================\n";
    std::cout << "       PLANNING RESULT\n";
    std::cout << "====================================\n";

    std::cout
        << "Success: "
        << (result.success ? "YES" : "NO")
        << "\n";

    std::cout << "State Path: ";

    for (size_t i = 0;
         i < result.statePath.size();
         ++i) {

        std::cout
            << result.statePath[i];

        if (i + 1 <
            result.statePath.size()) {

            std::cout << " -> ";
        }
    }

    std::cout << "\n";

    std::cout << "Total Cost: "
              << result.totalCost
              << "\n";

    std::cout << "Safety Score: "
              << result.safetyScore
              << "\n";

    std::cout << "Explored States: "
              << result.exploredStates
              << "\n";

    std::cout << "Planning Time: "
              << result.planningTimeMs
              << " ms\n";

    std::cout << "====================================\n";
}


PlanningProblem createBasicProblem() {

    PlanningProblem problem;

    /*
        Test Case 1

        S -> A -> B -> G
    */

    problem.states = {

        State(0, {0.0, 0.0}),

        State(1, {1.0, 0.0}),

        State(2, {2.0, 0.0}),

        State(3, {3.0, 0.0})
    };

    problem.initialState = 0;

    problem.goalState = 3;

    problem.badStates = {};

    problem.transitions = {

        Transition(
            0,
            0,
            1,
            1.0,
            0.9,
            0.95
        ),

        Transition(
            1,
            1,
            2,
            1.0,
            0.9,
            0.95
        ),

        Transition(
            2,
            2,
            3,
            1.0,
            0.9,
            0.95
        )
    };

    return problem;
}


int main() {

    std::cout
        << "Safe Semantic Planner\n";

    std::cout
        << "D* Lite Implementation\n";

    PlanningProblem problem =
        createBasicProblem();

    DStarLitePlanner planner;

    PlanningResult result =
        planner.plan(problem);
        

    printResult(result);

    return 0;
}