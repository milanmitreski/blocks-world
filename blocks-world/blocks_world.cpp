#include <iostream>
#include "blocks_world.hpp"

namespace bw {
    State BlocksWorld::get_initial_state() const {
        return initial_state_;
    }

    State BlocksWorld::get_goal_state() const {
        return goal_state_;
    }

    unsigned BlocksWorld::num_blocks() const {
        unsigned count = 0;
        for(const auto& tower : initial_state_) {
            count += tower.size();
        }
        return count;
    }

    pln::StateVariable BlocksWorld::on_table(const Block& block) const {
        return on(block, block);
    }

    pln::StateVariable BlocksWorld::on(const Block& lower, const Block& upper) const {
        return (lower - 1) * num_blocks() + upper;
    }

    pln::StateVariables BlocksWorld::state_variables() const {
        pln::StateVariables vars;
        unsigned n = num_blocks();
        for(unsigned i = 1; i <= n; ++i) {
            for(unsigned j = 1; j <= n; ++j) {
                if(i != j) {
                    vars.insert(on(i, j));
                } else {
                    vars.insert(on_table(i));
                }
            }
        }
        return vars;
    }

    pln::Condition BlocksWorld::clear_condition(const Block& block) const {
        pln::Condition cond;
        for(unsigned i = 1; i <= num_blocks(); ++i) {
            if(i != block) {
                cond.insert({on(block, i), false});
            }
        }
        return cond;
    }

    pln::Action BlocksWorld::move_to_table_action(const Block& block, const Block& from) const {
        pln::Condition precond = clear_condition(block);
        precond.insert({on(from, block), true});
        pln::Effect effect;
        effect.insert({on(from, block), false});
        effect.insert({on_table(block), true});
        return pln::Action{precond, effect};
    }

    pln::Action BlocksWorld::move_from_table_action(const Block& block, const Block& to) const {
        pln::Condition precond = clear_condition(to);
        precond.insert({on_table(block), true});
        pln::Effect effect;
        effect.insert({on_table(block), false});
        effect.insert({on(to, block), true});
        return pln::Action{precond, effect};
    }

    pln::Action BlocksWorld::move_to_block_action(const Block& block, const Block& from, const Block& to) const {
        pln::Condition precondBlock = clear_condition(block);
        pln::Condition precondTo = clear_condition(to);
        pln::Condition precond;
        precond.insert(precondBlock.begin(), precondBlock.end());
        precond.insert(precondTo.begin(), precondTo.end());
        precond.insert({on(from, block), true});
        pln::Effect effect;
        effect.insert({on(from, block), false});
        effect.insert({on(to, block), true});
        return pln::Action{precond, effect};
    }

    pln::State BlocksWorld::initial_state() const {
        pln::State state = [this](pln::StateVariable var) {
            for(const auto& tower : initial_state_) {
                for(unsigned i = 0; i < tower.size(); ++i) {
                    if(i == 0) {
                        if(var == on_table(tower[i])) {
                            return true;
                        }
                    } else {
                        if(var == on(tower[i-1], tower[i])) {
                            return true;
                        }
                    }
                }
            }
            return false;
        };
        return state;
    }

    pln::Condition BlocksWorld::goal_condition() const {
        pln::Condition cond;
        for(const auto& tower : goal_state_) {
            for(unsigned i = 0; i < tower.size(); ++i) {
                if(i == 0) {
                    cond.insert({on_table(tower[i]), true});
                } else {
                    cond.insert({on(tower[i-1], tower[i]), true});
                }
            }
        }
        return cond;
    }

    pln::PlanningProblem BlocksWorld::to_planning_problem() const {
        return pln::PlanningProblem(state_variables(), initial_state(), goal_condition());
    }
}