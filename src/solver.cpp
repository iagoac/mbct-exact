#include "solver.hpp"

void Solver::initialize(void) {
  #ifdef DEBUG
    std::cout << "Initializing the formulation..." << std::endl;
  #endif

  /* If debug is not defined
  then do not send the CPLEX
  output to the std::out  */
  #ifndef DEBUG // Please, be atent 'ifndef'
    this->cplex_->setOut(this->env_.getNullStream());
    this->cplex_->setWarning(this->env_.getNullStream());
  #endif

  #ifdef DEBUG
    std::cout << "Constructing the problem variables: ";
  #endif

  this->create_variables();

  #ifdef DEBUG
    std::cout << "Done!" << std::endl;
    std::cout << "Constructing the objective function: ";
  #endif

  this->add_objective();

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
void Solver::create_variables(void) {
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
        sprintf(name, "x_%d_%d", i, j);
        z_[i][j] = IloIntVar(this->env_, 0, 1, name);
      }
  }
}

/* Create the objective function */
void Solver::add_objective(void) {
  IloNumExpr expr(this->env_);

  /* min \sum_{(ij) \in E} ( c_{ij} z_{ij})  */
  for (int i = 0; i < this->graph_.num_nodes(); i++)  {
    for (auto j : this->graph_.adjacency_list[i]) {
      expr += this->graph_.edge[i][j]*z_[i][j];
    }
  }

  this->obj_ = IloObjective(this->env_, expr, IloObjective::Minimize);
  this->model_->add(this->obj_);
  expr.end();
}

/* First constraint set. The classic multiflow constraints
\sum_{(j,i) \in A} x_{ji}^k - \sum_{(i,j) \in A} x_{ij}^k = {-1, 0, 1}
\forall j \in N, k \in N \ {r} */
void Solver::flow_constraints(void) {
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
void Solver::flow_on_tree_constraints(void) {
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
void Solver::number_of_edges_constraints(void) {
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

/* Add all CPLEX constraints */
void Solver::add_constraints() {
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
}

void Solver::set_cplex_params(void) {
  this->cplex_->setParam(IloCplex::TiLim, 3600);
  this->cplex_->setParam(IloCplex::Threads, 1);
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
