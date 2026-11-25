// Independent integer-nanosecond oracle over 600 s and a fresh reset epoch.
// The negative control uses unmodified mj_step accumulation and must diverge.
#include "simulation_step.hpp"
#include <cmath>
#include <iostream>

int main(int argc, char **argv)
{
  if (argc != 2) return 2;
  char error[1024]{};
  auto *model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!model) {std::cerr << error << '\n'; return 2;}
  auto *data = mj_makeData(model);
  bool old_drift_observed = false;
  std::uint64_t first_bad_step = 0;
  for (std::uint64_t i = 1; i <= 300000; ++i) {
    mj_step(model, data);
    if (!first_bad_step && std::llround(data->time * 1000000000) !=
      std::llround(data->time / .001) * 1000000) first_bad_step = i;
    old_drift_observed |= std::llround(data->time * 1000000000) !=
      static_cast<long long>(i * 2000000);
  }
  bool passed = old_drift_observed && first_bad_step != 0;
  std::uint64_t steps = 0;
  for (int epoch = 0; epoch < 2; ++epoch) {
    mj_resetData(model, data);
    for (std::uint64_t i = 1; i <= 300000; ++i) {
      step_epoch(model, data, steps);
      // The oracle is the fixture's exact 2 ms, not double multiplication.
      const auto expected_ns = static_cast<long long>(i * 2000000);
      passed &= std::llround(data->time * 1000000000) == expected_ns;
      passed &= std::llround(data->time / .001) * 1000000 == expected_ns;
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
  std::cout << "CLOCK_600S_RESET " << (passed ? "PASS" : "FAIL")
            << " old_accumulation_drift=" << old_drift_observed
            << " first_bad_step=" << first_bad_step
            << " first_bad_seconds=" << first_bad_step * .002 << '\n';
  return passed ? 0 : 1;
}
