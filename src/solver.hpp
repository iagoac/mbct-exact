#ifndef SOLVER_HH_
#define SOLVER_HH_

#include <vector>
#include "cplex_includes.hpp" // CPLEX bi-dimensional and tri-dimensional structures
#include "argparse.hpp"       // Arguments class
#include "cGraph/release/cgraph.hpp"  // cGraph --> My graph class

class Solver {
private:
  /* Method used by the constructor
  calls other methods to initialize
  the mathematical model         */
  void initialize(void);

  /* Methods used by the initialize method */
  void create_variables(void);
  void add_objective(void);
  void add_constraints(void);
  void set_cplex_params(void);

  /* Constraint's list */
  void flow_constraints(void);
  void flow_on_tree_constraints(void);
  void number_of_edges_constraints(void);
  void max_error(void);

public:
  /* Class variables declaration */
  Args  *args_;     // The arguments class
  // Graph<int, int> graph_;   // The graph instance
  Digraph<int, int> graph_; // The graph instance

  /* CPLEX common structures */
  IloEnv env_;
  IloModel *model_;
  IloCplex *cplex_;

  /* CPLEX modelling objects */
  IloObjective obj_;
  IloConstraintArray constraints_;
  int root;
  int terminal;

  /* CPLEX variables */
  IloIntVarMatrix3D x_;
  IloNumVar y_;
  IloIntVarMatrix z_;

  /* Class constructor */
  Solver(Args *arg, Digraph<int, int> g) {
    this->args_  = arg;
    this->graph_ = g;
    this->model_ = new IloModel(env_);
    this->cplex_ = new IloCplex(*model_);
    this->initialize();
  }

  void solve(void) { this->cplex_->solve(); }
};

#endif // SOLVER_HH_
