#include "sat.hpp"
#include <fstream>

namespace sat {
    void Solver::add_clause(const Clause& clause) {
        cnf_.push_back(clause);
    }

    void Solver::clear() {
        cnf_.clear();
    }

    size_t Solver::num_clauses() const {
        return cnf_.size();
    }

    size_t Solver::num_literals(const Clause& clause) const {
        return clause.size();
    }

    void Solver::set_num_vars(int n) {
        num_vars_ = n;
    }

    int Solver::get_num_vars() const {
        return num_vars_;
    }

    void Solver::print_dimacs(std::ostream& out) const {
        out << "p cnf " << num_vars_ << " " << cnf_.size() << "\n";
        for (const auto& clause : cnf_) {
            for (Literal lit : clause) {
                out << lit << " ";
            }
            out << "0\n";
        }
    }

    void Solver::solve() const {
        std::ofstream cnf_file("problem.cnf");
        print_dimacs(cnf_file);
        cnf_file.close();

        int ret = std::system("minisat problem.cnf result.txt");

        std::ifstream result_file("result.txt");
        std::string line;
        while (std::getline(result_file, line)) {
            std::cout << line << std::endl;
        }
        result_file.close();
    }
}