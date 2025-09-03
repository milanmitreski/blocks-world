#include "planning.hpp"
#include <algorithm>

namespace pln {
    bool PlanningProblem::is_effect(const Literal& literal, const Action& action) const {
        return std::find(action.second.begin(), action.second.end(), literal) != action.second.end();
    }

    sat::CNF PlanningProblem::initial_state() const {
        sat::CNF cnf;
        for(const auto& var : variables_) {
            if(initial_state_(var)) {
                cnf.push_back(sat::Clause{static_cast<sat::Literal>(var)});
            } else {
                cnf.push_back(sat::Clause{-static_cast<sat::Literal>(var)});
            }
        } 
        return cnf;
    }

    sat::CNF PlanningProblem::goal_condition(const unsigned& plan_length) const {
        sat::CNF cnf;
        for(const auto& literal : goal_condition_) {
            sat::Clause clause;
            if(literal.second) {
                clause.push_back(static_cast<sat::Literal>(literal.first + plan_length * variables_.size()));
            } else {
                clause.push_back(-static_cast<sat::Literal>(literal.first + plan_length * variables_.size()));
            }
            cnf.push_back(clause);
        }
        return cnf;
    }

    sat::CNF PlanningProblem::action_to_cnf(const Action& action, const unsigned& step) const {
        sat::CNF cnf;
        unsigned offset = step * variables_.size();

        for(const auto& literal : action.first) {
            sat::Clause clause;
            if(literal.second) {
                clause.push_back(static_cast<sat::Literal>(literal.first + offset));
            } else {
                clause.push_back(-static_cast<sat::Literal>(literal.first + offset));
            }
            cnf.push_back(clause);
        }

        for(const auto& var : variables_) {
            sat::CNF cnf_literal;
            if(is_effect({var, true}, action)) {
                cnf_literal.push_back(sat::Clause{static_cast<sat::Literal>(var + offset + variables_.size())});
            } else if(is_effect({var, false}, action)) {
                cnf_literal.push_back(sat::Clause{-static_cast<sat::Literal>(var + offset + variables_.size())});
            } else {
                cnf_literal.push_back(sat::Clause{-static_cast<sat::Literal>(var + offset), static_cast<sat::Literal>(var + offset + variables_.size())});
                cnf_literal.push_back(sat::Clause{static_cast<sat::Literal>(var + offset), -static_cast<sat::Literal>(var + offset + variables_.size())});
            }
            cnf.insert(cnf.end(), cnf_literal.begin(), cnf_literal.end());
        }
        return cnf;
    }

    sat::CNF PlanningProblem::state_transition(const unsigned& step, const unsigned& plan_length) const {
        sat::CNF cnf;

        std::vector<unsigned> action_selectors;
        for (unsigned i = 1; i <= actions_.size(); ++i) {
            action_selectors.push_back(variables_.size() * (plan_length + 1) + step * actions_.size() + i);
        }

        sat::Clause at_least_one;
        for (const auto& selector : action_selectors) {
            at_least_one.push_back(static_cast<sat::Literal>(selector));
        }
        cnf.push_back(at_least_one);
        
        unsigned action_index = 0;
        for (const auto& action : actions_) {
            sat::CNF action_cnf = action_to_cnf(action, step);
            for (const auto& clause : action_cnf) {
                sat::Clause modified_clause = clause;
                modified_clause.push_back(-static_cast<sat::Literal>(action_selectors[action_index]));
                cnf.push_back(modified_clause);
            }
            action_index++;
        }
        return cnf;
    }

    sat::CNF PlanningProblem::plan(const unsigned& plan_length) const {
        sat::CNF cnf = initial_state();
        for(unsigned step = 0; step < plan_length; step++) {
            sat::CNF transition_cnf = state_transition(step, plan_length);
            cnf.insert(cnf.end(), transition_cnf.begin(), transition_cnf.end());
        }
        sat::CNF goal_cnf = goal_condition(plan_length);
        cnf.insert(cnf.end(), goal_cnf.begin(), goal_cnf.end());
        return cnf;
    }

    sat::Solver PlanningProblem::to_sat_solver(const unsigned& plan_length) const {
        sat::CNF cnf = plan(plan_length);
        unsigned num_vars = variables_.size() * (plan_length + 1) + actions_.size() * plan_length;
        sat::Solver solver(cnf, num_vars);
        return solver;
    }

    std::vector<Action> PlanningProblem::extract_plan(const sat::Valuation& valuation, const unsigned& plan_length) const {
        std::vector<Action> plan;
        unsigned num_vars = variables_.size();
        unsigned num_actions = actions_.size();
        for(unsigned step = 0; step < plan_length; step++) {
            unsigned action_start_index = num_vars * (plan_length + 1) + step * num_actions;
            for(unsigned action_index = 0; action_index < num_actions; action_index++) {
                if(valuation[action_start_index + action_index + 1]) {
                    plan.push_back(actions_[action_index]);
                    break;
                }
            }
        }
        return plan;
    }
}