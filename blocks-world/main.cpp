#include<iostream>
#include "blocks_world.hpp"
#include "sat.hpp"

using namespace std;

int main() {
    CNF cnf = {
        {1, -3, 4},
        {-1, 2},
        {-4, 3},
        {-2}
    };
    Solver solver(cnf, 4);
    solver.solve();
    return 0;
}