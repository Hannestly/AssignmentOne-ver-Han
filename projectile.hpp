#pragma once

#include <cstdio>

template <typename T>
struct VecR2 {
  T x{0};
  T y{0};
  T z{0};
};

template <typename T>
auto operator+(VecR2<T> a, VecR2<T> b) {
  return VecR2<T>{a.x + b.x, a.y + b.y, a.z + b.z};
};

template <typename T>
auto operator*(VecR2<T> a, T c) {
  return VecR2<T>{c * a.x, c * a.y, c * a.z};
}

template <typename T>
auto operator*(T c, VecR2<T> a) {
  return a * c;
}

template <typename T>
auto operator/(VecR2<T> a, T c) {
  return VecR2<T>{a.x / c, a.y / c, a.z / c};
}

struct TState {
  double t{0};
  VecR2<double> position{0, 0, 0};
  VecR2<double> velocity{0, 0, 0};
};

void print_tstate(TState s) {
  printf("%f %f %f %f %f %f %f\n", s.t, s.position.x, s.position.y, s.position.z, s.velocity.x,
         s.velocity.y, s.velocity.z);
}
