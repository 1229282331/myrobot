// Timing contract only. This adapter never owns device IO.
#pragma once
#include <cstdint>
#include <ostream>
#include <stdexcept>

struct SolverTiming {
  int callback_ms, request_ms, retreat_ms, geometry_ms;
  static constexpr int version=1, period_ms=50;

  static SolverTiming select(int64_t callback,int64_t request,int64_t retreat,
                             int64_t geometry,double publish_period,double update_period) {
    if(publish_period!=.05 || update_period!=.05)
      throw std::runtime_error("Solver timing requires the 50 ms integration step");
    if(!(0<geometry && geometry<retreat && retreat<callback && callback<request && request<=100))
      throw std::runtime_error("Invalid solver timing order or source lifetime");
    return {int(callback),int(request),int(retreat),int(geometry)};
  }

  void json(std::ostream& out) const {
    out<<"{\"version\":"<<version<<",\"period_ms\":"<<period_ms
       <<",\"callback_budget_ms\":"<<callback_ms<<",\"request_timeout_ms\":"<<request_ms
       <<",\"retreat_budget_ms\":"<<retreat_ms<<",\"geometry_budget_ms\":"<<geometry_ms<<"}";
  }
};
