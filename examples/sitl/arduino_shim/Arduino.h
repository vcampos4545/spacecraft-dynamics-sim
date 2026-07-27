#pragma once
// Minimal native stand-in for the handful of Arduino core symbols
// BalanceController.cpp/IMUFilter.cpp actually reference (DEG_TO_RAD,
// micros(), delay()) -- just enough for those two real firmware files to
// compile natively, unmodified. See examples/sitl/sitl.cpp for why this
// exists and what it's for.
#include <chrono>
#include <thread>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308232
#endif

// Monotonic microsecond clock, matching the real micros()'s contract
// (elapsed time since some arbitrary start, not wall-clock time) closely
// enough for IMUFilter's dt calculation -- it only ever differences two
// consecutive calls.
inline unsigned long micros()
{
  using namespace std::chrono;
  static const auto start = steady_clock::now();
  return (unsigned long)duration_cast<microseconds>(steady_clock::now() - start).count();
}

// calibrateGyro() really does call this 1000x (GYRO_CAL_SAMPLES) with a
// real sleep -- letting that actually take ~2 real seconds at startup
// matches the physical firmware's own calibration timing instead of
// silently making it instant, which is worth preserving for a SITL demo.
inline void delay(unsigned long ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
