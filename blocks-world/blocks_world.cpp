#include <iostream>
#include <fstream>
#include <sstream>
#include "blocks_world.hpp"

namespace bw {
    unsigned BlocksWorld::num_blocks() const {
        unsigned count = 0;
        for(const auto& tower : initial_state_) {
            count += tower.size();
        }
        return count;
    }

    State load_state(const std::string& filename) {
        State state;
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file: " + filename);
        }
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            Tower tower;
            Block block;
            while (iss >> block) {
                tower.push_back(block);
            }
            if (!tower.empty()) {
                state.push_back(tower);
            }
        }
        file.close();
        return state;
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
        for(unsigned i = 1; i <= n; i++) {
            for(unsigned j = 1; j <= n; j++) {
                if(i != j) {
                    vars.push_back(on(i, j));
                } else {
                    vars.push_back(on_table(i));
                }
            }
        }
        return vars;
    }

    pln::Condition BlocksWorld::clear_condition(const Block& block) const {
        pln::Condition cond;
        for(unsigned i = 1; i <= num_blocks(); i++) {
            if(i != block) {
                cond.push_back({on(block, i), false});
            }
        }
        return cond;
    }

    pln::Action BlocksWorld::move_to_table_action(const Block& block, const Block& from) const {
        pln::Condition precond = clear_condition(block);
        precond.push_back(pln::Literal{on(from, block), true});
        pln::Effect effect;
        effect.push_back(pln::Literal{on(from, block), false});
        effect.push_back(pln::Literal{on_table(block), true});
        return pln::Action{precond, effect};
    }

    pln::Action BlocksWorld::move_from_table_action(const Block& block, const Block& to) const {
        pln::Condition precondBlock = clear_condition(block);
        pln::Condition precondTo = clear_condition(to);
        pln::Condition precond;
        precond.insert(precond.end(), precondBlock.begin(), precondBlock.end());
        precond.insert(precond.end(), precondTo.begin(), precondTo.end());
        precond.push_back(pln::Literal{on_table(block), true});
        pln::Effect effect;
        effect.push_back(pln::Literal{on_table(block), false});
        effect.push_back(pln::Literal{on(to, block), true});
        return pln::Action{precond, effect};
    }

    pln::Action BlocksWorld::move_to_block_action(const Block& block, const Block& from, const Block& to) const {
        pln::Condition precondBlock = clear_condition(block);
        pln::Condition precondTo = clear_condition(to);
        pln::Condition precond;
        precond.insert(precond.end(), precondBlock.begin(), precondBlock.end());
        precond.insert(precond.end(), precondTo.begin(), precondTo.end());
        precond.push_back(pln::Literal{on(from, block), true});
        pln::Effect effect;
        effect.push_back(pln::Literal{on(from, block), false});
        effect.push_back(pln::Literal{on(to, block), true});
        return pln::Action{precond, effect};
    }

    pln::State BlocksWorld::initial_state() const {
        pln::State state = [this](pln::StateVariable var) {
            for(const auto& tower : initial_state_) {
                for(unsigned i = 0; i < tower.size(); i++) {
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
            for(unsigned i = 0; i < tower.size(); i++) {
                if(i == 0) {
                    cond.push_back(pln::Literal{on_table(tower[i]), true});
                } else {
                    cond.push_back(pln::Literal{on(tower[i-1], tower[i]), true});
                }
            }
        }
        return cond;
    }

    pln::PlanningProblem BlocksWorld::to_planning_problem() const {
        pln::StateVariables vars = state_variables();
        pln::State init_state = initial_state();
        pln::Condition goal_cond = goal_condition();
        pln::Actions actions;

        for(unsigned i = 1; i <= num_blocks(); i++) {
            for(unsigned j = 1; j <= num_blocks(); j++) {
                if(i != j) {
                    actions.push_back(move_to_table_action(i, j));
                    actions.push_back(move_from_table_action(i, j));
                    for(unsigned k = 1; k <= num_blocks(); k++) {
                        if(k != i && k != j) {
                            actions.push_back(move_to_block_action(i, j, k));
                        }
                    }
                }
            }
        }

        return pln::PlanningProblem{vars, init_state, actions, goal_cond};
    }

    std::string BlocksWorld::action_to_string(const pln::Action& action) const {
        for(unsigned i = 1; i <= num_blocks(); i++) {
            for(unsigned j = 1; j <= num_blocks(); j++) {
                if(i != j) {
                    if(action == move_to_table_action(i, j)) {
                        return "Move " + std::to_string(i) + " from " + std::to_string(j) + " to table";
                    }
                    if(action == move_from_table_action(i, j)) {
                        return "Move " + std::to_string(i) + " from table to " + std::to_string(j);
                    }
                    for(unsigned k = 1; k <= num_blocks(); k++) {
                        if(k != i && k != j) {
                            if(action == move_to_block_action(i, j, k)) {
                                return "Move " + std::to_string(i) + " from " + std::to_string(j) + " to " + std::to_string(k);
                            }
                        }
                    }
                }
            }
        }
        return "Unknown action";
    }
 
    std::string BlocksWorld::plan_to_string(const std::vector<pln::Action>& plan) const {
        std::string result;
        result += "Plan with " + std::to_string(plan.size()) + " steps: \n";
        for(unsigned i = 0; i < plan.size(); i++) {
            result += std::to_string(i+1) + ". action: " + action_to_string(plan[i]) + "\n";
        }
        return result;
    }   
}