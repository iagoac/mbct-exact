#ifndef SOLVER_HH_
#define SOLVER_HH_

#include <random>    // mtrand
#include <limits>    // std::numeric_limits
#include <algorithm> // std::min
#include "cplex_includes.hpp"
#include "argparse.hpp"

class Solver {
private:
  void create_variables(void);
  void add_objective(void);
  void add_constraints(void);

  void flow_constraints(void);
  void flow_on_tree_constraints(void);
  void number_of_edges_constraints(void);
  void max_error(void);

  /* Model building methods */
  void set_cplex_params(void);

public:
  /* Class variables declaration */
  Args *_args;      // The arguments class

  /* CPLEX common structures */
  IloEnv _env;
  IloModel _model;
  IloCplex _cplex;

  /* CPLEX variables */
  IloIntVarMatrix3D _x;
  IloNumVar _y;
  IloIntVarMatrix _z;

  /* Class constructor */
  Solver (Args* arg)
  :_args(arg), _model(this->_env), _cplex(this->_model) {
    this->initialize();
  }

  /* Method used by the constructor
  calls other methods to initialize
  the mathematical model         */
  void initialize();

  void solve(void) { this->_cplex.solve(); }
};

#endif // SOLVER_HH_
