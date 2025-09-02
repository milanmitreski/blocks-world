#ifndef PLANING_HPP
#define PLANING_HPP

#include <functional>
#include <set>
#include "sat.hpp"

namespace pln {
    using StateVariable = unsigned;
    using StateVariables = std::set<StateVariable>;
    using Literal = std::pair<StateVariable, bool>;
    using State = std::function<bool(StateVariable)>;
    using Condition = std::set<Literal>;
    using Effect = std::set<Literal>;
    using Action = std::pair<Condition, Effect>;

    class PlanningProblem {
    public:
        PlanningProblem(StateVariables variables, State initial_state, Condition goal_condition):
            variables_(variables), initial_state_(initial_state), goal_condition_(goal_condition) {}
        State get_initial_state() const;
        Condition get_goal_condition() const;
    private:
        Condition literal_precondition(const Literal& literal) const;

        sat::CNF initial_state() const;
        sat::CNF goal_condition() const;
        sat::CNF action_to_cnf(const Action& action) const;


        StateVariables variables_;
        State initial_state_;
        Condition goal_condition_;
    };
}

#endif // PLANING_HPP