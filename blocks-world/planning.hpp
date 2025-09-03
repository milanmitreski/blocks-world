#ifndef PLANING_HPP
#define PLANING_HPP

#include <functional>
#include <set>
#include "sat.hpp"

namespace pln {
    using StateVariable = unsigned;
    using StateVariables = std::vector<StateVariable>;
    using Literal = std::pair<StateVariable, bool>;
    using State = std::function<bool(StateVariable)>;
    using Condition = std::vector<Literal>;
    using Effect = std::vector<Literal>;
    using Action = std::pair<Condition, Effect>;
    using Actions = std::vector<Action>;

    class PlanningProblem {
    public:
        PlanningProblem(StateVariables variables, State initial_state, Actions actions, Condition goal_condition):
            variables_(variables), initial_state_(initial_state), actions_(actions), goal_condition_(goal_condition) {}
        ~PlanningProblem() = default;
    
        sat::Solver to_sat_solver(const unsigned& plan_length) const;
        std::vector<Action> extract_plan(const sat::Valuation& valuation, const unsigned& plan_length) const;
    private:
        bool is_effect(const Literal& literal, const Action& action) const;

        sat::CNF initial_state() const;
        sat::CNF goal_condition(const unsigned& plan_length) const;
        sat::CNF action_to_cnf(const Action& action, const unsigned& step) const;
        sat::CNF state_transition(const unsigned& step, const unsigned& plan_length) const;
        sat::CNF plan(const unsigned& plan_length) const;

        StateVariables variables_;
        State initial_state_;
        Actions actions_;
        Condition goal_condition_;
    };
}

#endif // PLANING_HPP