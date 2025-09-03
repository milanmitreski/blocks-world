#include<iostream>
#include "blocks_world.hpp"
#include "sat.hpp"

using namespace std;

int main() {
    bw::State initial_state = bw::load_state("initial.bw");
    bw::State goal_state = bw::load_state("goal.bw");
    bw::BlocksWorld blocks_world(initial_state, goal_state);

    pln::PlanningProblem planning_problem = blocks_world.to_planning_problem();

    optional<sat::Valuation> result;
    unsigned plan_length;

    for(plan_length = 1; ; plan_length++) {
        sat::Solver solver = planning_problem.to_sat_solver(plan_length);
        result = solver.solve();
        if(result)
            break;
    }
    
    cout << blocks_world.plan_to_string(planning_problem.extract_plan(*result, plan_length));
    return 0;
}