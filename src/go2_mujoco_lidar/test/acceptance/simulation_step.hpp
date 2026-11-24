// Advance native acceptance physics on an epoch step clock.
// mj_step accumulates a double time; at ~257 s with dt=.002 that drift rounds
// SportModeState to one ns past LowState.tick*1 ms, breaking exact state pairing.
// Derive publication time from integer steps, leaving the integrator unchanged.
#pragma once
#include <cstdint>
#include <mujoco/mujoco.h>

inline void step_epoch(const mjModel *model, mjData *data, std::uint64_t &steps)
{
  mj_step(model, data);
  data->time = static_cast<double>(++steps) * model->opt.timestep;
}
