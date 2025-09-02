#ifndef BLOCKS_WORLD_SAT_HPP
#define BLOCKS_WORLD_SAT_HPP

#include <vector>
#include <iostream>

namespace sat {
    using Literal = int;
    using Clause = std::vector<Literal>;
    using CNF = std::vector<Clause>;

    class Solver {
    public:
        Solver(CNF cnf, int num_vars): cnf_(cnf), num_vars_(num_vars) {};
        ~Solver() = default;

        void add_clause(const Clause& clause);
        void clear();
        size_t num_clauses() const;
        size_t num_literals(const Clause& clause) const;
        void set_num_vars(int n);
        int get_num_vars() const;
        void print_dimacs(std::ostream& out = std::cout) const;
        void solve() const;
    private:
        CNF cnf_;
        int num_vars_;
    };
}
#endif // BLOCKS_WORLD_SAT_HPP