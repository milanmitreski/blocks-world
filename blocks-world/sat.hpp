#ifndef BLOCKS_WORLD_SAT_HPP
#define BLOCKS_WORLD_SAT_HPP

#include <vector>
#include <iostream>
#include <optional>

namespace sat {
    using Literal = int;
    using Clause = std::vector<Literal>;
    using CNF = std::vector<Clause>;
    using Valuation = std::vector<bool>;

    class Solver {
    public:
        Solver(CNF cnf, unsigned num_vars): cnf_(cnf), num_vars_(num_vars) {};
        ~Solver() = default;

        void add_clause(const Clause& clause);
        void print_dimacs(std::ostream& out = std::cout) const;
        std::optional<Valuation> solve() const;
    private:
        CNF cnf_;
        unsigned num_vars_;
    };
}
#endif // BLOCKS_WORLD_SAT_HPP