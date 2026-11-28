// G03 v2 per-physics-frame stand-up gate and subsequent fall detection.
// Threshold provenance and pre-registration: docs/G03_SAVED_MAP_NAV2_CRITERIA.md.
// Inputs are freejoint base height/roll/pitch and simulation seconds.
// Completion and timeout are latched; a late stand cannot erase a timeout.
#pragma once

#include <algorithm>
#include <cmath>

struct G03Posture
{
  double stable_since = -1.;
  double completed_at = -1.;
  bool timed_out = false;

  bool observe(double time, double z, double roll, double pitch)
  {
    const double tilt = std::max(std::abs(roll), std::abs(pitch));
    if (completed_at < 0. && !timed_out) {
      if (z >= .22 && tilt <= M_PI / 12.) {
        if (stable_since < 0.) stable_since = time;
      } else {
        stable_since = -1.;
      }
      if (stable_since >= 0. && time - stable_since >= 1. && time <= 10.)
        completed_at = time;
      if (completed_at < 0. && time >= 10.) timed_out = true;
    }
    return completed_at >= 0. && (z < .18 || tilt > M_PI / 3.);
  }
};
