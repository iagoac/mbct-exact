#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "argparse.hpp"
#include "mbct-model.hpp"
#include "cGraph/release/cgraph.hpp"

int main(int argc, char* const* argv) {
  /* Declare the arguments parser */
  ArgParse argparser;
  double solution;

  /* Get the arguments */
  argparser.add("-input",     ArgParse::Opt::req);
  argparser.add("-augmented", ArgParse::Opt::req, { "0", "1" });
  argparser.add("-objective", ArgParse::Opt::req);
  argparser.add("-time"     , ArgParse::Opt::req);
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
  std::pair<int, int> max_cost, max_error;

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
    std::cout << "Input readed with sucess. ";
    // std::cout << "Let's read the nadir file";
    std::cout << std::endl;
  #endif

  /* Read the file that contains the coordinates of the nadir point */
  // file.open(args.get<std::string>("-nadir"), std::fstream::in);
  //
  // if (!file) {
  //   std::cout << "Nadir file not found. Aborting" << std::endl;
  //   exit(0);
  // }
  //
  // #ifdef DEBUG
  //   std::cout << "Input found! Let's read it :)" << std::endl;
  // #endif
  //
  // file >> max_cost.first >> max_cost.second >> max_error.first >> max_error.second;
  // file.close();

  #ifdef DEBUG
    // std::cout << "Nadir point readed with sucess. ";
    std::cout << "Let's build the formulation :)" << std::endl;
  #endif

  /* If it has some */
  // instance.preprocessing();
  /* Print pre-processing time */
  // std::cout << timer.count<std::chrono::seconds>() << std::endl;

  /* Solver constructor */
  MBCT solver(&args, g);

  /* Initializing the time counter */
  solver.timer.start();

  if (args.get<int>("-objective") == 1) {
    if (args.get<int>("-augmented") == 1) {
      solver.solve_obj1_augmented();
    } else {
      solver.solve_obj1();
    }
  } else {
    if (args.get<int>("-augmented") == 1) {
      solver.solve_obj2_augmented();
    } else {
      solver.solve_obj2();
    }
  }

  solver.timer.stop();

  int computed_points = solver.points.size();

  /* Remove the dominated points */
  solver.process_pareto_points();

  /* Gets only the instance name */
  std::string instance_name = args.get<std::string>("-input").substr(args.get<std::string>("-input").find_last_of('/')+1);

  /* Print the results */
  std::cout << instance_name << ",";
  std::cout << args.get<std::string>("-objective") << ",";
  std::cout << computed_points << ",";
  std::cout << solver.points.size() << ",";
  std::cout << solver.total_nodes << ",";
  std::cout << solver.timer.count<std::chrono::seconds>() << std::endl;
  solver.print_points();

  return (0);
}
