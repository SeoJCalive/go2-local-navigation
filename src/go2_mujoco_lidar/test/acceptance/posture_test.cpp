// Pure G03 posture state transitions. No ROS, simulator or physical actuation.
// Exercise exact boundaries, one-frame interruptions and latched timeout.
#include "posture.hpp"
#include <iostream>

int main()
{
  bool passed = true;
  G03Posture gate;
  passed &= !gate.observe(0., .1, 0., 0.);  // initial squat is not a fall
  gate.observe(1., .22, M_PI / 12., 0.);
  gate.observe(1.5, .219, 0., 0.);  // one bad height frame resets the hold
  gate.observe(2., .22, 0., 0.);
  gate.observe(2.5, .27, 0., M_PI / 12. + .001);  // tilt also resets it
  gate.observe(3., .22, 0., 0.);
  gate.observe(3.998, .22, 0., 0.);
  passed &= gate.completed_at < 0.;
  passed &= !gate.observe(4., .22, 0., 0.);
  passed &= gate.completed_at == 4. && !gate.timed_out;
  passed &= !gate.observe(4.002, .18, M_PI / 3., 0.);
  passed &= gate.observe(4.004, .179, 0., 0.);
  passed &= gate.observe(4.006, .27, M_PI / 3. + .001, 0.);
  passed &= gate.observe(4.008, .27, 0., -M_PI / 3. - .001);

  G03Posture timeout;
  timeout.observe(0., .1, 0., 0.);
  timeout.observe(10., .27, 0., 0.);
  timeout.observe(11., .27, 0., 0.);
  passed &= timeout.timed_out && timeout.completed_at < 0.;
  G03Posture boundary;
  boundary.observe(9., .27, 0., 0.);
  boundary.observe(10., .27, 0., 0.);
  passed &= boundary.completed_at == 10. && !boundary.timed_out;
  G03Posture late;
  late.observe(9.5, .27, 0., 0.);
  late.observe(10.5, .27, 0., 0.);
  passed &= late.timed_out && late.completed_at < 0.;
  std::cout << "G03_POSTURE " << (passed ? "PASS" : "FAIL") << '\n';
  return passed ? 0 : 1;
}
