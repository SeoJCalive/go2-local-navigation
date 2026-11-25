// Advance native acceptance physics on an epoch step clock.
// mj_step accumulates a double time; at ~257 s with dt=.002 that drift rounds
// SportModeState to one ns past LowState.tick*1 ms, breaking exact state pairing.
// Derive publication time from integer steps, leaving the integrator unchanged.
#pragma once
#include "simulation_step.h"
