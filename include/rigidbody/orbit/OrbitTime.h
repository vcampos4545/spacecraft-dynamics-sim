#pragma once
#include <cmath>

// Julian Date epoch handling -- lets a scenario set a real mission epoch
// (e.g. 2026-01-01T00:00:00Z) that OrbitFrames::gmstRad and SunModel need,
// and advance it by elapsed mission time.
namespace OrbitTime
{
constexpr double SECONDS_PER_DAY = 86400.0;

// Calendar (UTC, Gregorian) -> Julian Date. Standard integer JDN formula
// (Fliegel & Van Flandern) extended with a fractional day.
inline double julianDate(int year, int month, int day,
                         int hour = 0, int minute = 0, double second = 0.0)
{
  int a = (14 - month) / 12;
  int y = year + 4800 - a;
  int m = month + 12 * a - 3;

  long long jdn = day + (153 * m + 2) / 5 + 365LL * y + y / 4 - y / 100 + y / 400 - 32045;

  double dayFrac = (hour - 12) / 24.0 + minute / 1440.0 + second / SECONDS_PER_DAY;
  return static_cast<double>(jdn) + dayFrac;
}

// Julian Date epochJd plus elapsedSeconds of simulated mission time.
inline double advance(double epochJd, double elapsedSeconds)
{
  return epochJd + elapsedSeconds / SECONDS_PER_DAY;
}
} // namespace OrbitTime
