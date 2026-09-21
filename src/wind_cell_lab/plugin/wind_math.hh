#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace aircells {
using Triple = std::array<double, 3>;
inline double Norm(const Triple &v) {
  return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}
struct ForceResult { Triple force; bool limited; };
inline ForceResult WindIncrement(const Triple &v, const Triple &wind,
                                 double rho, double cda, double cap) {
  Triple relative{v[0]-wind[0],v[1]-wind[1],v[2]-wind[2]};
  const double relativeSpeed=Norm(relative), speed=Norm(v);
  Triple force{};
  for (int i=0;i<3;++i) force[i]=0.5*rho*cda*(-relativeSpeed*relative[i]+speed*v[i]);
  const double magnitude=Norm(force);
  const bool limited=magnitude>cap;
  if (limited) for (auto &f:force) f*=cap/magnitude;
  return {force,limited};
}
inline double EdgeWeight(double margin,double blend) {
  if (margin<=0) return 0;
  const double a=std::clamp(margin/blend,0.0,1.0);
  return a*a*(3-2*a);
}
}
