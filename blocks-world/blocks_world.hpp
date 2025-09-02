#ifndef BLOCKS_WORLD_HPP
#define BLOCKS_WORLD_HPP

#include <vector>
#include "planning.hpp"

namespace bw {
    using Block = unsigned;
    using Tower = std::vector<Block>;
    using State = std::vector<Tower>;

    template<typename Collection, typename Element, typename... Args>
    Collection of(Args... args) {
        return Collection{static_cast<Element>(args)...}; 
    }

    class BlocksWorld {
    public:
        BlocksWorld(State initial_state, State goal_state): initial_state_(initial_state), goal_state_(goal_state) {};
        ~BlocksWorld() = default;

        State get_initial_state() const;
        State get_goal_state() const;
        unsigned num_blocks() const;

        pln::PlanningProblem to_planning_problem() const;
    private:
        State initial_state_;
        State goal_state_;

        pln::StateVariable on_table(const Block& block) const;
        pln::StateVariable on(const Block& lower, const Block& upper) const;
        pln::StateVariables state_variables() const;
        pln::State initial_state() const;
        pln::Condition goal_condition() const;
        pln::Condition clear_condition(const Block& block) const;
        pln::Action move_to_table_action(const Block& block, const Block& from) const;
        pln::Action move_from_table_action(const Block& block, const Block& to) const;
        pln::Action move_to_block_action(const Block& block, const Block& from, const Block& to) const;
    };
}

#endif // BLOCKS_WORLD_HPP