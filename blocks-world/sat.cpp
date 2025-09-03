#include "sat.hpp"
#include <fstream>
#include <sstream>

namespace sat {
    void Solver::add_clause(const Clause& clause) {
        cnf_.push_back(clause);
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

    void cleanup() {
        std::string cleanup_command = "rm plan.cnf plan.out minisat.log";
        system(cleanup_command.c_str());
    }
    
    std::optional<Valuation> Solver::solve() const {
        std::ofstream temp_cnf("plan.cnf");
        print_dimacs(temp_cnf);
        temp_cnf.close();

        std::string command = "minisat plan.cnf plan.out > minisat.log";
        system(command.c_str());

        std::ifstream result_file("plan.out");
        if (!result_file.is_open()) {
            cleanup();
            return std::nullopt;
        }

        std::string line;
        std::getline(result_file, line);
        if (line == "UNSAT") {
            cleanup();
            return std::nullopt;
        } else if (line != "SAT") {
            cleanup();
            return std::nullopt;
        }

        Valuation valuation(num_vars_ + 1, false);
        std::getline(result_file, line);
        std::istringstream iss(line);
        int var;
        while (iss >> var) {
            if (var == 0) break;
            if (var > 0 && var <= static_cast<int>(num_vars_)) {
                valuation[var] = true;
            }
        }
        result_file.close();
        cleanup();

        return valuation;
    }
}