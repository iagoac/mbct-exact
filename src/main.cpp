#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "cxxtimer.hpp"
#include "argparse.hpp"
#include "solver.hpp"
#include "cGraph/release/cgraph.hpp"

int main(int argc, char* const* argv) {
  /* Declare the arguments parser */
  ArgParse argparser;
  double solution;

  /* Get the arguments */
  argparser.add("-input", ArgParse::Opt::req);
  argparser.add("-nadir", ArgParse::Opt::req);
  argparser.add("-objective", ArgParse::Opt::req);
  Args args = argparser.parse(argc, argv);

  /* Debug initial message */
  #ifdef DEBUG
    std::cout << "Debug is activated!" << std::endl;
    std::cout << "Solving instance " << args.get<std::string>("-input") << std::endl;
  #endif

  Digraph<int, int> g;

  // variables used to read the instance
  int a, b, c;
  int numberOfVertex;

  // open the instancee file
  std::fstream file(args.get<char *>("-input"), std::fstream::in);

  // read both the vertex and edges number
  file >> numberOfVertex;

  /* Initialize the terminal's map */
  for (int i = 0; i < numberOfVertex; i++) {
    file >> a;
    g.add_node(a);
  }

  /* Read the instance edges */
  while (!file.eof()) {
    file >> a >> b >> c;
    g.add_arc(a, b, c);
  }

  file.close();

  /* Initializing the time counter */
  cxxtimer::Timer timer;
  timer.start();

  /* If it has some */
  // instance.preprocessing();
  /* Print pre-processing time */
  // std::cout << timer.count<std::chrono::seconds>() << std::endl;


  /* Solver constructor */
  Solver solver(&args, g);

  solver.solve();

  timer.stop();
  std::cout << args.get<std::string>("-input") << ",";
  std::cout << args.get<std::string>("-seed") << ",";
  std::cout << args.get<std::string>("-algorithm") << ",";
  std::cout << solution << ",";
  std::cout << timer.count<std::chrono::seconds>() << std::endl;


  // std::cout << solver._cplex.getBestObjValue() << ",";
  // std::cout << solver._cplex.getObjValue() << ",";
  // std::cout << solver._cplex.getMIPRelativeGap()*100 << ",";
  // std::cout << solver._cplex.getTime() << ",";
  // std::cout << solver._cplex.getStatus();
  // std::cout << std::endl;

  return (0);
}
