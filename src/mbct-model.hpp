#ifndef MBCT_HH_
#define MBCT_HH_

#include <vector>             // std::vector
#include <utility>            // std::pair
#include <algorithm>          // std::reverse
#include "cplex_includes.hpp" // CPLEX bi-dimensional and tri-dimensional structures
#include "argparse.hpp"       // Arguments class
#include "cGraph/release/cgraph.hpp"  // cGraph --> My graph class

class MBCT {
private:
  /* Method used by the constructor
  calls other methods to initialize
  the mathematical model         */
  void initialize(void);

  /* Methods used by the initialize method */
  void create_variables(void);
  void add_objective_z1(void);
  void add_objective_z2(void);
  void add_constraints(void);
  void set_cplex_params(void);

  /* Constraint's list */
  void flow_constraints(void);
  void flow_on_tree_constraints(void);
  void number_of_edges_constraints(void);
  void max_error(void);
  void big_slack_constraints(void);

  /* Methods used by Z(1) and Z(2) solvers */
  int compute_y(void);

public:
  /* Class variables declaration */
  Args  *args_;                             // Arguments class
  Graph<int, int> graph_;                   // Graph instance
  std::vector<std::pair<int, int>> points;  // Set of pareto points

  /* CPLEX common structures */
  IloEnv env_;
  IloModel *model_;
  IloCplex *cplex_;

  /* CPLEX modelling objects */
  IloObjective obj_;
  IloConstraintArray constraints_;
  int root = 0;
  int terminal;

  /* CPLEX variables */
  IloIntVarMatrix3D x_;
  IloNumVar y_;
  IloIntVarMatrix z_;
  IloIntVar master_slack_;
  IloIntVarArray slack_;

  /* Class constructor */
  MBCT() {}
  MBCT(Args *arg, Graph<int, int> g) {
    this->args_  = arg;
    this->graph_ = g;
    this->model_ = new IloModel(env_);
    this->cplex_ = new IloCplex(*model_);
    this->initialize();
  }

  void solve(void) { this->cplex_->solve(); }

  void solve_obj1(void);
  void solve_obj2(void);

  /* Postprocessing methods  */
  void process_pareto_points(void);
  uintmax_t hypervolume(std::pair<int, int> max_cost, std::pair<int, int> max_error);
  void print_points(void);
};

#endif // MBCT_HH_
