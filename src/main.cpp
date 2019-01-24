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
  argparser.add("-seed", ArgParse::Opt::req);
  Args args = argparser.parse(argc, argv);

  /* Debug initial message */
  #ifdef DEBUG
    std::cout << "Debug is activated!" << std::endl;
    std::cout << "Solving instance " << args.get<std::string>("-input") << std::endl;
  #endif

  Graph<int, int> g;

  // variables used to read the instance
  int a, b, c;
  uint numberOfVertex;
  int max_cost, max_error;

  // open the input file
  std::fstream file;


  #ifdef DEBUG
    std::cout << "Opening instance file" << std::endl;
  #endif

  file.open(args.get<std::string>("-input"), std::fstream::in);

  if (!file) {
    std::cout << "Instance file not found. Aborting" << std::endl;
    exit(0);
  }

  #ifdef DEBUG
    std::cout << "Input found! Let's read it :)" << std::endl;
  #endif

  // read both the vertex and edges number
  file >> numberOfVertex;

  /* Initialize the terminal's map */
  for (int i = 0; i < numberOfVertex; i++) {
    file >> a;
    g.add_node(a);
  }

  /* Read the instance edges */
  while (file >> a >> b >> c) {
    g.add_edge(a, b, c);
  }
  file.close();

  #ifdef DEBUG
    std::cout << "Input readed with sucess. Let's read the nadir file" << std::endl;
  #endif

  /* Read the file that contains the coordinates of the nadir point */
  file.open(args.get<std::string>("-nadir"), std::fstream::in);

  if (!file) {
    std::cout << "Nadir file not found. Aborting" << std::endl;
    exit(0);
  }

  #ifdef DEBUG
    std::cout << "Input found! Let's read it :)" << std::endl;
  #endif

  file >> max_cost >> max_error;
  file.close();

  #ifdef DEBUG
    std::cout << "Nadir point readed with sucess. Let's build the formulation" << std::endl;
  #endif

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

  /* Remove the dominated points */
  solver.process_pareto_points();

  solver.print_points();

  std::cout << std::endl << std::endl;
  
  /* Print the results */
  std::cout << args.get<std::string>("-input") << ",";
  // std::cout << args.get<std::string>("-algorithm") << ",";
  std::cout << solver.hypervolume(max_cost, max_error) << ",";
  std::cout << timer.count<std::chrono::seconds>() << std::endl;
  solver.print_points();

  return (0);
}
