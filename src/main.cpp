#include <stdio.h>
#include <stdlib.h>
#include "cxxtimer.hpp"
#include "argparse.hpp"
#include "solver.hpp"

int main(int argc, char* const* argv) {
  /* Declare the arguments parser */
  ArgParse argparser;
  double solution;

  /* Get the arguments */
  argparser.add("-input", ArgParse::Opt::req);
  // argparser.add("-robust", ArgParse::Opt::ena);
  argparser.add("-formulation", "");
  Args args = argparser.parse(argc, argv);

  /* Debug initial message */
  #ifdef DEBUG
    std::cout << "Debug is activated!" << std::endl;
    std::cout << "Solving instance " << args.get<std::string>("-input") << std::endl;
  #endif

  /* Initializing the time counter */
  cxxtimer::Timer timer;
  timer.start();

  /* Solver constructor
  it reads the .mps file */
  Solver solver(&args);

  if (args.get<std::string>("-formulation") == "compact") {
    // solution = solver.compact();
  }

  timer.stop();
  std::cout << args.get<std::string>("-input") << ",";
  std::cout << args.get<std::string>("-seed") << ",";
  std::cout << args.get<std::string>("-algorithm") << ",";
  std::cout << solution << ",";
  std::cout << timer.count<std::chrono::seconds>() << std::endl;


  // solver.benders();

  // std::cout << solver._cplex.getBestObjValue() << ",";
  // std::cout << solver._cplex.getObjValue() << ",";
  // std::cout << solver._cplex.getMIPRelativeGap()*100 << ",";
  // std::cout << solver._cplex.getTime() << ",";
  // std::cout << solver._cplex.getStatus();
  // std::cout << std::endl;

  return (0);
}
