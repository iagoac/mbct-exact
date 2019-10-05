#include <vector>
#include <algorithm>
#include <iterator>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <iomanip>

#include<math.h>

#include <limits>


auto cmp = [](std::pair<int, int> lhs, std::pair<int, int> rhs) {
  return ((lhs.first == rhs.first) && (lhs.second == rhs.second));
};

void printPairs(std::vector<std::pair<int,int>> points) {
  for (auto p : points) {
    std::cout << p.first << " " << p.second << std::endl;
  }
}

double euclidian(std::pair<int,int> a, std::pair<int, int> b) {
  double x = a.first - b.first;
  double y = a.first - b.first;

  double dist = x*x + y*y;
  dist = std::sqrt(dist);
  return (dist);
}

int main(int argc, char* const* argv) {
  // std::cout << "intersection,npe,m2,m3" << std::endl;
  /* Opening the files */
  std::fstream ex;
  std::fstream np;
  std::string filename = argv[1];
  ex.open(filename, std::fstream::in);
  np.open(filename + ".npe", std::fstream::in);
  if (!ex) {
    std::cout << "Exact file not found. Aborting" << std::endl;
    exit(0);
  }
  if (!np) {
    std::cout << "NPE file not found. Aborting" << std::endl;
    exit(0);
  }

  /* Constructing the vector which store the pareto frontiers */
  std::vector<std::pair<int, int>> exact;
  std::vector<std::pair<int, int>> npe;
  std::vector<std::pair<int, int>> intersection;

  int a, b;

  while (ex >> a >> b) {
    exact.push_back(std::make_pair(a, b));
  }
  while (np >> a >> b) {
    npe.push_back(std::make_pair(a, b));
  }

  std::cout << filename << ",";

  // printPairs(exact);
  // std::cout << std::endl << std::endl;
  // printPairs(npe);

  /***** M_1 ******/
  for (auto pointA : npe) {
    for (auto pointB : exact) {
      if (pointA.first == pointB.first && pointA.second == pointB.second) {
        intersection.push_back(pointA);
        break;
      }
    }
  }

  // std::cout << std::endl << std::endl;
  // printPairs(intersection);
  // exit(0);

  /* Print M_1 */
  std::cout << static_cast<float>(exact.size()) << "," << static_cast<float>(npe.size()) << ",";
  std::cout << static_cast<float>(intersection.size()) << ",";
  std::cout << static_cast<float>(intersection.size())/static_cast<float>(npe.size()) << ",";


  /****** M_2 ******/
  double sum = 0;
  for (auto pointA : exact) {
    double aux = std::numeric_limits<double>::max();
    for (auto pointB : npe) {
      // std::cout << euclidian(pointA, pointB) << std::endl;
      if (euclidian(pointA, pointB) < aux) {
        aux = euclidian(pointA, pointB);
      }
    }
    // std::cout << sum << " + " << aux << std::endl;
    sum += aux;
  }

  /* Print M_2 */
  std::cout << static_cast<float>(1) / static_cast<float>(exact.size()) * sum << ",";

  /***** M_3 ******/
  sum = 0;

  for (auto pointA : npe) {
    double aux = std::numeric_limits<double>::max();
    for (auto pointB : exact) {
      // std::cout << euclidian(pointA, pointB) << std::endl;
      if (euclidian(pointA, pointB) < aux) {
        aux = euclidian(pointA, pointB);
      }
    }
    // std::cout << sum << " + " << aux << std::endl;
    sum += aux;
  }

  /* Print M_3 */
  std::cout << std::sqrt(sum) / static_cast<float>(npe.size()) << std::endl;
}
