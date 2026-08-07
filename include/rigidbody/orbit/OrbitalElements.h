#pragma once
#include <rigidbody/orbit/OrbitState.h>

// Classical (Keplerian) orbital elements -> ECI state vector conversion
// (the standard "COE2RV" algorithm), so a scenario can specify an orbit
// the way it's naturally described (e.g. "500 km circular, 51.6 deg
// inclination") instead of hand-computing a position/velocity vector.
struct OrbitalElements
{
  double semiMajorAxisM;
  double eccentricity = 0.0;
  double inclinationRad = 0.0;
  double raanRad = 0.0;        // right ascension of ascending node
  double argPeriapsisRad = 0.0;
  double trueAnomalyRad = 0.0;

  // Convenience factory for the common "circular orbit at a given
  // altitude and inclination" case -- eccentricity/argument of periapsis
  // are meaningless for a circular orbit, left at zero; trueAnomalyRad
  // becomes the orbit's starting phase angle.
  static OrbitalElements circular(double altitudeM, double inclinationRad,
                                  double planetRadiusM, double raanRad = 0.0,
                                  double trueAnomalyRad = 0.0);

  // Converts to an ECI state vector for a central body with gravitational
  // parameter mu (m^3/s^2), at the given mission time.
  OrbitState toState(double mu, double missionTimeS = 0.0) const;

  // The inverse conversion ("RV2COE") -- classical elements from a real
  // ECI state vector, for displaying/logging what orbit a propagated
  // state actually represents (a scenario's true position/velocity
  // rarely stays exactly circular once real perturbations like J2 are
  // in the force model, even if it started that way).
  static OrbitalElements fromState(const OrbitState &state, double mu);

  // Orbital period (s) and mean motion (rad/s) for a central body with
  // gravitational parameter mu -- both functions of semiMajorAxisM alone
  // (Kepler's third law), independent of where in the orbit `state` was
  // sampled.
  double periodS(double mu) const;
  double meanMotionRadS(double mu) const;
};
