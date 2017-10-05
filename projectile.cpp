#include "projectile.hpp"
#include <cmath>


const double dt = 0.001;

const double k = 0.5;
const double q = 1;
const double m = 1;
const double Eo = 1;
const double w = 2;

auto force(TState s) { return VecR2<double>{cos(k*-s.position.z-w*s.t), sin(k*s.position.z-w*s.t),0}*(Eo/sqrt(2)); }

auto euler_step(TState s, VecR2<double> accel) {
  TState next;
  next.t = s.t + dt;
  next.position = s.position + (s.velocity * dt);
  next.velocity = s.velocity + (accel * dt);
  return next;
}

void n_steps(auto n, TState state0) {
  print_tstate(state0);
  if (n == 0)
    return;
  else {
    auto state = state0;
    for (unsigned k = 0; k < n; ++k) {
      state = euler_step(state, force(state) / m);
      print_tstate(state);
    }
  }
}

int main() {
  n_steps(10000, TState{0., {(-((Eo)/((pow(w,2))*sqrt(2)))),0,0}, {0,Eo/(w*sqrt(2)),-0.25}});
  return 0;
}
