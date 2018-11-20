#include "solver.hpp"

void Solver::initialize() {
  #ifdef DEBUG
    std::cout << "Initializing the formulation..." << std::endl;
  #endif

  /* If debug is not defined
  then do not send the CPLEX
  output to the std::out  */
  #ifndef DEBUG // Please, be atent 'ifndef'
    this->_cplex.setOut(this->_env.getNullStream());
    this->_cplex.setWarning(this->_env.getNullStream());
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
    /* Then, export the CPLEX model into file mymodel.lp */
    this->_cplex.exportModel("model.lp");
  #endif
}

/* Create and initialize the CPLEX variables */
void Solver::create_variables(void) {
  /* Loop variables */
   uint i, j, k;

  /* x_{ij}^{k} \in {0,1} */
  _x = IloIntVarMatrix3D(this->_env, this->_graph->size());
  for (i = 0; i < this->_graph->size(); i++) {
      _x[i] = IloIntVarMatrix(this->_env, this->_graph->size());
      for (j = 0; j < this->_graph->size(); j++) {
          _x[i][j] = IloIntVarArray(this->_env, this->_graph->size());
          for (k = 0; k < this->_graph->size(); k++) {
              char name[50];
              sprintf(name, "x_%d_%d_%d", i, j, k);
              _x[i][j][k] = IloIntVar(this->_env, 0, 1, name);
          }
      }
  }

  /* y \in R */
  _y = IloNumVar(this->_env, 0, IloInfinity, "y");

  /* z_{ij} \in {0,1} */
  _z = IloIntVarArray(this->_env, this->_graph->size());
  for (i = 0; i < this->_graph->size(); i++) {
      char name[50];
      sprintf(name, "x_%d", i);
      _z[i] = IloIntVar(this->_env, 0, 1, name);
  }
}

/* Create the objective function */
void Solver::add_objective_z1(void) {
  IloNumExpr expr(this->_env);
  int size = this->_graph->size();

  /* min sum_{k \in V'} [ \sum_{(ij) \in A} ( u_{ij} y_{ij}^k) - x_k] */
  for (int k = 1; k < size; k++) {
      for (int i = 0; i < size; i++)  {
          for (int j = 0; j < size; j++)  {
              if (_graph->get_arc_cost(i,j) >= 1) {
                  expr += this->_graph->get_arc_cost(i,j)*_x[i][j][k];
              }
          }
      }
  }

  this->_model.add(IloMinimize(this->_env, expr));
  expr.end();
}

void Solver::add_objective_z2(void) {
  this->_model.add(IloMinimize(this->_env, this->_y));
}

/* First constraint set. The classic multiflow constraints
\sum_{(j,i) \in A} x_{ji}^k - \sum_{(i,j) \in A} x_{ij}^k = {-1, 0, 1}
\forall j \in N, k \in N \ {r} */
void Solver::flow_constraints(void) {
  int size = this->_graph->size();

  for (int k = 1; k < size; k++) {
    for (int j = 0; j < size; j++) {
      IloNumExpr expr(this->_env);
      for (int l = 0; l < size; l++) {
        if (this->_graph->get_arc_cost(j,l) >= 1) {
          expr += _x[j][l][k];
        }
      }

      for (int i = 0; i < size; i++) {
        if (this->_graph->get_arc_cost(i,j) >= 1) {
          expr -= _x[i][j][k];
        }
      }

      if (j == this->_s) {
       this->_model.add(expr == 1);
      } else if (j == k) {
        this->_model.add(expr == -1);
      } else if (j != this->_s && j != k) {
        this->_model.add(expr == 0);
      }

      expr.end();
    }
  }
}

/* Second constraint set
x_{ij}^k \leq _z{ij}
\forall(i,j) \in A, k \in N \ {r} */
void Solver::flow_on_tree_constraints(void) {
  int size = this->_graph->size();

  for (int i = 0; i < size; i++)  {
    for (int j = 0; j < size; j++) {
      for(int k = 0 ; k < size ; k++) {
        if (this->_graph->get_arc_cost(i,j) >= 1) {
          this->_model.add(_x[i][j][k] <= _z[i][j]);
        }
      }
    }
  }
}

/* Third constraint set
\sum_{(i,j) \in A} _z{ij} = |A|-1
\forall(i,j) \in A,k \in N \ {r} */
void Solver::number_of_edges_constraints(void) {
  int size = this->_graph->size();

  IloNumExpr expr(this->_env);
  for (int i = 0; i < size; i++)  {
    for (int j = 0; j < size; j++) {
      if (_graph->get_arc_cost(i,j) >= 1) {
        expr += _z[i][j];
      }
    }
  }

  IloInt tamanho_arvore = size - 1;
  this->_model.add(expr == tamanho_arvore);
  expr.end();
}

/* Fourth constraint set
y >= \sum_{(i,j) \in A} \error_j x_{ij}^k
\forall k \in N {r} */
void Solver::max_error(void) {
  int size = this->_graph->size();

  for (int k = 1; k < size; k++) {
    IloNumExpr expr(this->_env);
    for (int i = 0; i < size; i++)  {
      for (int j = 0; j < size; j++) {
        expr += this->_graph->gamma[j]*_x[i][j][k];
      }
    }
    this->_model.add(_y >= expr);
    expr.end();
  }
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
  this->_cplex.setParam(IloCplex::TiLim, 3600);
  this->_cplex.setParam(IloCplex::Threads, 1);
  // this->_cplex.setParam(IloCplex::Cliques, -1);
  // this->_cplex.setParam(IloCplex::Covers, -1);
  // this->_cplex.setParam(IloCplex::DisjCuts, -1);
  // this->_cplex.setParam(IloCplex::FlowCovers, -1);
  // this->_cplex.setParam(IloCplex::FlowPaths, -1);
  // this->_cplex.setParam(IloCplex::GUBCovers, -1);
  // this->_cplex.setParam(IloCplex::ImplBd, -1);
  // this->_cplex.setParam(IloCplex::MIRCuts, -1);
  // this->_cplex.setParam(IloCplex::FracCuts, -1);
  // this->_cplex.setParam(IloCplex::ZeroHalfCuts, -1);
  // this->_cplex.setParam(IloCplex::MIPSearch, CPX_MIPSEARCH_TRADITIONAL);
  // this->_cplex.setParam(IloCplex::MIPInterval, 100);
  // this->_cplex.setParam(IloCplex::PreInd, 0);
  // this->_cplex.setParam(IloCplex::AggInd, 0);
  // this->_cplex.setParam(IloCplex::HeurFreq, -1);
  // this->_cplex.setParam(IloCplex::EpAGap, 0);
}
