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

// Julian Date -> calendar (UTC, Gregorian). Inverse of julianDate() above
// -- standard algorithm (Meeus, "Astronomical Algorithms" ch. 7; the same
// one Fliegel & Van Flandern's forward JDN formula inverts), valid for
// the Gregorian calendar (post-1582-10-15, the only range julianDate()
// itself produces since it always applies the Gregorian formula). Used
// wherever a computed instant (e.g. a predicted ground-station pass AOS/
// LOS time) needs to be shown as a real clock time rather than a raw JD.
inline void calendarDate(double jd, int &year, int &month, int &day,
                         int &hour, int &minute, double &second)
{
  double jdShifted = jd + 0.5;
  long long z = static_cast<long long>(std::floor(jdShifted));
  double dayFrac = jdShifted - static_cast<double>(z); // 0..1 from local JDN midnight

  long long alpha = static_cast<long long>((static_cast<double>(z) - 1867216.25) / 36524.25);
  long long a = z + 1 + alpha - alpha / 4;
  long long b = a + 1524;
  long long c = static_cast<long long>((static_cast<double>(b) - 122.1) / 365.25);
  long long d = static_cast<long long>(365.25 * static_cast<double>(c));
  long long e = static_cast<long long>((static_cast<double>(b - d)) / 30.6001);

  day = static_cast<int>(b - d - static_cast<long long>(30.6001 * static_cast<double>(e)));
  month = static_cast<int>(e < 14 ? e - 1 : e - 13);
  year = static_cast<int>(month > 2 ? c - 4716 : c - 4715);

  double hoursFrac = dayFrac * 24.0;
  hour = static_cast<int>(hoursFrac);
  double minutesFrac = (hoursFrac - hour) * 60.0;
  minute = static_cast<int>(minutesFrac);
  second = (minutesFrac - minute) * 60.0;
  // Guard against the fractional-second roundoff pushing seconds to
  // exactly 60 (e.g. 59.9999999997) -- carries cleanly since 60s/60m/24h
  // are themselves exact.
  if (second >= 60.0)
  {
    second -= 60.0;
    minute += 1;
    if (minute >= 60)
    {
      minute -= 60;
      hour += 1;
    }
  }
}
} // namespace OrbitTime
