#include "mbct-model.hpp"

void MBCT::initialize(void) {
  #ifdef DEBUG
    std::cout << "Initializing the formulation..." << std::endl;
  #endif

  /* If debug is not defined then do not
  send the CPLEX  output to the std::out */
  #ifndef DEBUG // Please, be atent: 'ifndef'
    this->cplex_->setOut(this->env_.getNullStream());
    this->cplex_->setWarning(this->env_.getNullStream());
  #endif

  #ifdef DEBUG
    std::cout << "Constructing the problem variables: ";
  #endif

  if (this->args_->get<int>("-augmented")) {
    this->create_variables_augmented();
  } else {
    this->create_variables();
  }

  #ifdef DEBUG
    std::cout << "Done!" << std::endl;
    std::cout << "Constructing the objective function: ";
  #endif

  if (this->args_->get<int>("-objective") == 1 ) {
    if (this->args_->get<int>("-augmented") == 1) {
      this->add_objective_z1_augmented();
    } else {
      this->add_objective_z1();
    }
  } else {
    if (this->args_->get<int>("-augmented") == 1) {
      this->add_objective_z2_augmented();
    } else {
      this->add_objective_z2();
    }
  }

  #ifdef DEBUG
    std::cout << "Done!" << std::endl;
    std::cout << "Adding problem constraints:" << std::endl;
  #endif

  this->add_constraints();

  #ifdef DEBUG
    std::cout << "Done!" << std::endl;
    std::cout << "Setting CPLEX parameters: ";
  #endif

  this->set_cplex_params();

  #ifdef DEBUG
    std::cout << "Done!" << std::endl;
  #endif

  #ifdef DEBUG
    /* Then, export the CPLEX model into file model.lp */
    this->cplex_->exportModel("model.lp");
  #endif
}

/* Create and initialize the CPLEX variables */
void MBCT::create_variables(void) {
  /* Loop variables */
   uint i, j, k;

  /* x_{ij}^{k} \in {0,1} */
  x_ = IloIntVarMatrix3D(this->env_, this->graph_.num_nodes());
  for (i = 0; i < this->graph_.num_nodes(); i++) {
    x_[i] = IloIntVarMatrix(this->env_, this->graph_.num_nodes());
    for (j = 0; j < this->graph_.num_nodes(); j++) {
      x_[i][j] = IloIntVarArray(this->env_, this->graph_.num_nodes());
      for (k = 0; k < this->graph_.num_nodes(); k++) {
        char name[50];
        sprintf(name, "x_%d_%d_%d", i, j, k);
        x_[i][j][k] = IloIntVar(this->env_, 0, 1, name);
      }
    }
  }

  /* y \in R */
  y_ = IloNumVar(this->env_, 0, IloInfinity, "y");

  /* z_{ij} \in {0,1} */
  z_ = IloIntVarMatrix(this->env_, this->graph_.num_nodes());
  for (i = 0; i < this->graph_.num_nodes(); i++) {
    z_[i] = IloIntVarArray(this->env_, this->graph_.num_nodes());
    for (j = 0; j < this->graph_.num_nodes(); j++) {
      char name[50];
      sprintf(name, "z_%d_%d", i, j);
      z_[i][j] = IloIntVar(this->env_, 0, 1, name);
    }
  }
}

/* Create and initialize the CPLEX variables */
void MBCT::create_variables_augmented(void) {
  this->create_variables();

  slack_ = IloIntVarArray(this->env_, this->graph_.num_nodes());
  for (int i = 0; i < this->graph_.num_nodes(); i++) {
    char name[50];
    sprintf(name, "slack_%d", i);
    slack_[i] = IloIntVar(this->env_, 0, IloIntMax, name);
  }

  master_slack_ = IloIntVar(this->env_, 0, IloIntMax, "master_slack");
}

/* Create the objective function */
void MBCT::add_objective_z1(void) {
  IloNumExpr expr(this->env_);

  /* min \sum_{(ij) \in E} ( c_{ij} z_{ij}) + exp(slack) */
  for (int i = 0; i < this->graph_.num_nodes(); i++)  {
    for (auto j : this->graph_.adjacency_list[i]) {
      expr += this->graph_.edge[i][j]*z_[i][j];
    }
  }

  this->obj_ = IloObjective(this->env_, expr, IloObjective::Minimize);
  this->model_->add(this->obj_);
  expr.end();
}

/* Create the objective function */
void MBCT::add_objective_z1_augmented(void) {
  IloNumExpr expr(this->env_);

  /* min \sum_{(ij) \in E} ( c_{ij} z_{ij}) + exp(slack) */
  for (int i = 0; i < this->graph_.num_nodes(); i++)  {
    for (auto j : this->graph_.adjacency_list[i]) {
      expr += this->graph_.edge[i][j]*z_[i][j];
    }
  }

  expr -= 0.01 * master_slack_;

  this->obj_ = IloObjective(this->env_, expr, IloObjective::Minimize);
  this->model_->add(this->obj_);
  expr.end();
}

void MBCT::add_objective_z2(void) {
  IloNumExpr expr(this->env_);

  /* min y + exp(slack) */
  expr += this->y_;

  this->obj_ = IloObjective(this->env_, expr, IloObjective::Minimize);
  this->model_->add(this->obj_);
  expr.end();
}

void MBCT::add_objective_z2_augmented(void) {
  IloNumExpr expr(this->env_);

  /* min y + exp(slack) */
  expr += this->y_;

  expr -= 0.01 * master_slack_;

  this->obj_ = IloObjective(this->env_, expr, IloObjective::Minimize);
  this->model_->add(this->obj_);
  expr.end();
}

/* First constraint set. The classic multiflow constraints
\sum_{(j,i) \in A} x_{ji}^k - \sum_{(i,j) \in A} x_{ij}^k = {-1, 0, 1}
\forall j \in N, k \in N \ {r} */
void MBCT::flow_constraints(void) {
  int size = this->graph_.num_nodes();

  for (int k = 1; k < size; k++) {
    for (int j = 0; j < size; j++) {
      IloNumExpr expr(this->env_);
      for (auto l : this->graph_.adjacency_list[j]) {
        expr += x_[j][l][k];
      }

      for (auto i : this->graph_.adjacency_list[j]) {
          expr -= x_[i][j][k];
      }

      if (j == this->root) {
       this->model_->add(expr == 1);
      } else if (j == k) {
        this->model_->add(expr == -1);
      } else if (j != this->root && j != k) {
        this->model_->add(expr == 0);
      }

      expr.end();
    }
  }
}

/* Second constraint set
x_{ij}^k \leq z_{ij}
\forall(i,j) \in A, k \in N \ {r} */
void MBCT::flow_on_tree_constraints(void) {
  int size = this->graph_.num_nodes();

  for (int i = 0; i < size; i++)  {
    for (auto j : this->graph_.adjacency_list[i]) {
      for(int k = 1 ; k < size ; k++) {
        this->model_->add(x_[i][j][k] <= z_[i][j]);
      }
    }
  }
}

/* Third constraint set
\sum_{(i,j) \in A} z_{ij} = |A|-1 */
void MBCT::number_of_edges_constraints(void) {
  int size = this->graph_.num_nodes();

  IloNumExpr expr(this->env_);
  for (int i = 0; i < size; i++)  {
    for (auto j : this->graph_.adjacency_list[i]) {
      expr += z_[i][j];
    }
  }

  IloInt tamanho_arvore = size - 1;
  this->model_->add(expr == tamanho_arvore);
  expr.end();
}

/* Fourth constraint set
y >= \sum_{ij} \in A} \gamma_j x_{ij}^k
\forall k \in N \setminus {r}        */
void MBCT::max_error(void) {
  int size = this->graph_.num_nodes();

  for (int k = 1; k < size; k++) {
    IloNumExpr expr(this->env_);

    for (int j = 0; j < size; j++) {
      for (auto i : this->graph_.adjacency_list[j]) {
        expr += this->graph_.node[j]*this->x_[i][j][k];
      }
    }
    this->model_->add(this->y_ >= expr);
    expr.end();
  }
}

/* Fifth constriant set (optional)
It is only inserted when optimizing Z_1
and constraining Z_2
master_slack >= slack_i
\forall slack_i                      */
void MBCT::big_slack_constraints(void) {
  int size = this->graph_.num_nodes();
  for (int k = 1; k < size; k++) {
    this->model_->add(master_slack_ <= slack_[k]);
  }
}

/* Add all CPLEX constraints */
void MBCT::add_constraints() {
  #ifdef DEBUG
    std::cout << "Adding the flow constraints: ";
  #endif

  this->flow_constraints();

  #ifdef DEBUG
    std::cout << "Done!" << std::endl;
    std::cout << "Adding the tree constraint: ";
  #endif

  this->flow_on_tree_constraints();

  #ifdef DEBUG
    std::cout << "Done!" << std::endl;
    std::cout << "Adding the maximum number of edges constraint: ";
  #endif

  this->number_of_edges_constraints();

  #ifdef DEBUG
    std::cout << "Done!" << std::endl;
  #endif

  /* The augmented e-constraint method
  need this for constraining Z(2)   */
  if (this->args_->get<int>("-objective") == 1 && this->args_->get<int>("-augmented") == 1) {
    #ifdef DEBUG
      std::cout << "Adding the big slack constraints: ";
    #endif

    this->big_slack_constraints();

    #ifdef DEBUG
      std::cout << "Done!" << std::endl;
    #endif
  }

  /* The polytope of problem Z(2)
  has an additional constraint */
  if (this->args_->get<int>("-objective") == 2) {
    #ifdef DEBUG
      std::cout << "Adding the maximum cost constraints: ";
    #endif

    this->max_error();

    #ifdef DEBUG
      std::cout << "Done!" << std::endl;
    #endif
  }
}

void MBCT::process_pareto_points(void) {
  std::vector<std::pair<int, int>> new_points;

  /* If optimizing z_2 instead of z_1, then reverse the vector */
  if (this->points[0].first > this->points.back().first) {
    std::reverse(this->points.begin(), this->points.end());
  }

  int i;
  for (i = 1; i < this->points.size(); i++) {
    new_points.push_back(this->points[i-1]);
    if (this->points[i-1].first == this->points[i].first && this->points[i-1].second >= this->points[i].second) {
      new_points.pop_back();
    }
  }

  new_points.push_back(this->points[i-1]);

  if (this->points[i-2].first == this->points[i-1].first && this->points[i-2].second >= this->points[i-1].second) {
    new_points.pop_back();
  }

  if (this->points[i-2].first <= this->points[i-1].first && this->points[i-2].second == this->points[i-1].second) {
    new_points.pop_back();
  }

  this->points = new_points;
}

uintmax_t MBCT::hypervolume(std::pair<int, int> max_cost, std::pair<int, int> max_error) {
  std::pair<int, int> nadir;

  nadir.first = max_cost.first;
  nadir.second = max_cost.second;

  uintmax_t hyp = 0;
  hyp += ( (max_cost.first - this->points[0].first) * (this->points[0].second - max_cost.second) ) / 2;
  hyp += (max_cost.first - this->points[0].first) * (nadir.second - this->points[0].second);

  for (int i = 0; i < this->points.size() - 1; i++) {
    hyp += ( (this->points[i].first - this->points[i+1].first) * (this->points[i+1].second - this->points[i].second) ) / 2;
    hyp += (this->points[i].first - this->points[i+1].first) * (nadir.second - this->points[i+1].second);
  }

  hyp += ( (this->points.back().first - max_cost.first) * (max_cost.second - this->points.back().second) ) / 2;
  hyp += (this->points.back().first - max_cost.first) * (nadir.second - max_cost.second);

  return (hyp);
}

void MBCT::print_points(void) {
  for (auto p : this->points) {
    std::cout << p.first << " " << p.second << std::endl;
  }
}

void MBCT::solve_obj1(void) {
  /* Solve it the first time */
  this->cplex_->solve();

  /* Get the number of explored nodes */
  this->total_nodes += this->cplex_->getNnodes();

  /* Computes the pareto front */
  while (this->cplex_->getStatus() == IloAlgorithm::Optimal && (timer.count<std::chrono::seconds>() < this->args_->get<int>("-time"))) {
    std::pair<int, int> aux;
    aux.first = this->cplex_->getObjValue();

    /* First, computes the value of the second objective
    It will be stored on variable obj2_value */
    int obj2_value = 0;
    for (int k = 1; k < this->graph_.num_nodes(); k++) {
      int sum = 0;
      for (int i = 0; i < this->graph_.num_nodes(); i++) {
        for (auto j : this->graph_.adjacency_list[i]) {
          sum += this->graph_.node[j] * this->cplex_->getValue(this->x_[i][j][k]);
        }
      }
      if (sum > obj2_value) {
        obj2_value = sum;
      }
    }

    aux.second = obj2_value;

    /* Store the pareto point */
    this->points.push_back(aux);

    /* Add the new constraint's set */
    for (int k = 1; k < this->graph_.num_nodes(); k++) {
      IloNumExpr expr(this->env_);
      for (int i = 0; i < this->graph_.num_nodes(); i++) {
        for (auto j : this->graph_.adjacency_list[i]) {
          expr += this->graph_.node[j]*this->x_[i][j][k];
        }
      }
      this->model_->add(expr < obj2_value);
      expr.end();
    }

    /* Solves the problem again */
    this->cplex_->solve();

    #ifdef DEBUG
      std::cout << std::endl << std::endl << points.back().first << " - " << points.back().second;
      std::cout << std::endl << this->cplex_->getStatus() << std::endl << std::endl;
    #endif
  }
}

void MBCT::solve_obj1_augmented(void) {
  /* Auxiliary variable that stores a Pareto point */
  std::pair<int, int> aux;

  /* Computes all possible constraints of obj2
  and store them in an IloRangeArray      */
  IloRangeArray rangeArray(this->env_);

  for (int k = 1; k < this->graph_.num_nodes(); k++) {
    IloNumExpr expr(this->env_);
    for (int i = 0; i < this->graph_.num_nodes(); i++) {
      for (auto j : this->graph_.adjacency_list[i]) {
        expr += this->graph_.node[j]*this->x_[i][j][k];
      }
    }

    expr += slack_[k];

    char name[50];
    sprintf(name, "z2_constraint_%d", k);
    IloRange newIloRange(this->env_, -IloInfinity, expr, IloInfinity, name);
    rangeArray.add(newIloRange);
    expr.end();
  }

  /* Solve it the first time
  We need a small 'gambiarra':
  We set master_slack_ = 0  */
  IloExpr exp (this->env_);
  exp += master_slack_;
  IloRange range_slack(this->env_, 0, exp, 0, "gambs");
  this->model_->add(range_slack);
  this->solve();

  /* Get the number of explored nodes */
  this->total_nodes += this->cplex_->getNnodes();

  /* First, computes the value of the second objective
  It will be stored on variable obj2_value */
  int obj2_value = this->compute_z2_value();

  aux.first = std::ceil(this->cplex_->getObjValue());
  aux.second = obj2_value;

  this->model_->remove(range_slack);
  exp.end();

  /* Computes the pareto front */
  while (1) {
    /* Store the pareto point */
    this->points.push_back(aux);

    /* Set the new bounds of the IloRangeArray */
    IloInt bound_value = obj2_value - 1;
    IloIntArray new_bounds(this->env_);
    for (int k = 1; k < this->graph_.num_nodes(); k++) {
      new_bounds.add(bound_value);
    }
    rangeArray.setBounds(new_bounds, new_bounds);

    /* Add the constraints with
    the new computed bounds  */
    this->model_->add(rangeArray);

    /* Solves the problem again */
    this->solve();
    if ((this->cplex_->getStatus() != IloAlgorithm::Optimal) || (timer.count<std::chrono::seconds>() > this->args_->get<int>("-time"))) {
      this->ended = true;
      break;
    }

    /* Get the number of explored nodes */
    this->total_nodes += this->cplex_->getNnodes();

    /* Get the new Pareto point */
    obj2_value = this->compute_z2_value();
    aux.first = std::ceil(this->cplex_->getObjValue() - 0.01 * obj2_value);
    aux.second = obj2_value;

    /* Remove the constraints */
    this->model_->remove(rangeArray);

    #ifdef DEBUG
      std::cout << std::endl << std::endl << points.back().first << " - " << points.back().second;
      std::cout << std::endl << this->cplex_->getStatus() << std::endl << std::endl;
    #endif
  }
}

int MBCT::compute_z2_value(void) {
  int obj2_value = 0;
  for (int k = 1; k < this->graph_.num_nodes(); k++) {
    int sum = 0;
    for (int i = 0; i < this->graph_.num_nodes(); i++) {
      for (auto j : this->graph_.adjacency_list[i]) {
        sum += this->graph_.node[j] * this->cplex_->getValue(this->x_[i][j][k]);
      }
    }
    if (sum > obj2_value) {
      obj2_value = sum;
    }
  }
  return (obj2_value);
}

int MBCT::compute_z1_value(void) {
  int obj1_value = 0;
  for (int i = 0; i < this->graph_.num_nodes(); i++)  {
    for (auto j : this->graph_.adjacency_list[i]) {
      obj1_value += this->graph_.edge[i][j] * this->cplex_->getValue(this->z_[i][j]);
    }
  }
  return (obj1_value);
}

void MBCT::solve_obj2(void) {
  /* Solve it the first time */
  this->cplex_->solve();

  /* Get the number of explored nodes */
  this->total_nodes += this->cplex_->getNnodes();

  /* Computes the pareto front */
  while (this->cplex_->getStatus() == IloAlgorithm::Optimal && (timer.count<std::chrono::seconds>() < this->args_->get<int>("-time"))) {
    std::pair<int, int> aux;
    aux.second = this->cplex_->getObjValue();

    aux.first = this->compute_z1_value();

    /* Store the pareto point */
    this->points.push_back(aux);

    /* Add the new constraint's set */
    IloNumExpr expr(this->env_);
    /* min \sum_{(ij) \in E} ( c_{ij} z_{ij}) */
    for (int i = 0; i < this->graph_.num_nodes(); i++)  {
      for (auto j : this->graph_.adjacency_list[i]) {
        expr += this->graph_.edge[i][j]*z_[i][j];
      }
    }
    this->model_->add(expr < aux.first);
    expr.end();

    /* Solves the problem again */
    this->cplex_->solve();

    #ifdef DEBUG
      std::cout << std::endl << std::endl << points.back().first << " - " << points.back().second;
      std::cout << std::endl << this->cplex_->getStatus() << std::endl << std::endl;
    #endif
  }
}
void MBCT::solve_obj2_augmented(void) {
  /* Auxiliary variable that stores a Pareto point */
  std::pair<int, int> aux;

  /* Computes the constraint associated with
  with Z_1 and store it in an IloRange var */
  IloNumExpr expr(this->env_);

  for (int i = 0; i < this->graph_.num_nodes(); i++)  {
    for (auto j : this->graph_.adjacency_list[i]) {
      expr += this->graph_.edge[i][j]*z_[i][j];
    }
  }

  expr += master_slack_;
  IloRange z1_constraint(this->env_, -IloInfinity, expr, IloInfinity, "z1_constrained");
  expr.end();

  /* Solve it the first time
  We need a small 'gambiarra':
  We set master_slack_ = 0  */
  IloExpr exp (this->env_);
  exp += master_slack_;
  IloRange range_slack(this->env_, 0, exp, 0, "gambs");
  this->model_->add(range_slack);
  this->solve();

  /* Get the number of explored nodes */
  this->total_nodes += this->cplex_->getNnodes();

  /* First, computes the value of the second objective
  It will be stored on variable obj2_value */
  int obj1_value = this->compute_z1_value();

  aux.first  = obj1_value;
  aux.second = std::ceil(this->cplex_->getObjValue());

  this->model_->remove(range_slack);
  exp.end();

  /* Computes the pareto front */
  while (1) {
    /* Store the pareto point */
    this->points.push_back(aux);

    /* Set the new bounds of the IloRangeArray */
    IloInt bound_value = obj1_value - 1;
    z1_constraint.setBounds(bound_value, bound_value);

    /* Add the constraints with
    the new computed bounds  */
    this->model_->add(z1_constraint);

    /* Solves the problem again */
    this->solve();
    if ((this->cplex_->getStatus() != IloAlgorithm::Optimal) || (timer.count<std::chrono::seconds>() >> this->args_->get<int>("-time"))) {
      this->ended = true;
      break;
    }

    /* Get the number of explored nodes */
    this->total_nodes += this->cplex_->getNnodes();

    /* Get the new Pareto point */
    obj1_value = this->compute_z1_value();
    aux.first  = obj1_value;
    aux.second = std::ceil(this->cplex_->getObjValue());

    /* Remove the constraints */
    this->model_->remove(z1_constraint);

    #ifdef DEBUG
      std::cout << std::endl << std::endl << points.back().first << " - " << points.back().second;
      std::cout << std::endl << this->cplex_->getStatus() << std::endl << std::endl;
    #endif
  }
}

void MBCT::set_cplex_params(void) {
  // this->cplex_->setParam(IloCplex::TiLim, 7200);
  // this->cplex_->setParam(IloCplex::Threads, 1);
  // this->cplex_->setParam(IloCplex::Cliques, -1);
  // this->cplex_->setParam(IloCplex::Covers, -1);
  // this->cplex_->setParam(IloCplex::DisjCuts, -1);
  // this->cplex_->setParam(IloCplex::FlowCovers, -1);
  // this->cplex_->setParam(IloCplex::FlowPaths, -1);
  // this->cplex_->setParam(IloCplex::GUBCovers, -1);
  // this->cplex_->setParam(IloCplex::ImplBd, -1);
  // this->cplex_->setParam(IloCplex::MIRCuts, -1);
  // this->cplex_->setParam(IloCplex::FracCuts, -1);
  // this->cplex_->setParam(IloCplex::ZeroHalfCuts, -1);
  // this->cplex_->setParam(IloCplex::MIPSearch, CPX_MIPSEARCH_TRADITIONAL);
  // this->cplex_->setParam(IloCplex::MIPInterval, 100);
  // this->cplex_->setParam(IloCplex::PreInd, 0);
  // this->cplex_->setParam(IloCplex::AggInd, 0);
  // this->cplex_->setParam(IloCplex::HeurFreq, -1);
  // this->cplex_->setParam(IloCplex::EpAGap, 0);
}
