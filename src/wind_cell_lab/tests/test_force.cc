#include "../plugin/wind_math.hh"
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
  using namespace aircells;
  auto calm=WindIncrement({3,4,0},{0,0,0},1.225,0.08,5);
  assert(Norm(calm.force)<1e-12 && !calm.limited);
  auto wind=WindIncrement({0,0,0},{6,0,0},1.225,0.08,5);
  assert(std::abs(wind.force[0]-1.764)<1e-9);
  assert(wind.force[1]==0 && wind.force[2]==0);
  auto reverse=WindIncrement({0,0,0},{-6,0,0},1.225,0.08,5);
  assert(std::abs(reverse.force[0]+1.764)<1e-9);
  auto capped=WindIncrement({0,0,0},{100,0,0},1.225,0.08,5);
  assert(capped.limited && std::abs(Norm(capped.force)-5)<1e-9);
  assert(EdgeWeight(-1,10)==0 && EdgeWeight(0,10)==0);
  assert(EdgeWeight(10,10)==1 && EdgeWeight(50,10)==1);
  assert(std::abs(EdgeWeight(5,10)-0.5)<1e-12);
  std::cout << "Force kernel and spatial edge weights pass\n";
}
