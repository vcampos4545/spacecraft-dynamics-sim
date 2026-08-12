// Headless validation for rigidbody/orbit/ (double-precision truth
// propagator) and CentralBodyGravity (float32 ForceGenerator). No VGL/
// imgui dependency -- this checks the math itself, not rendering, so it
// builds and runs without a display. Prints PASS/FAIL per check and
// returns the failure count as the process exit code, the same
// convention satellite-adcs-sim's own tests/test_common.h CHECK() macro
// follows, so this reads the same way if you've seen that project.
#include <rigidbody/orbit/OrbitState.h>
#include <rigidbody/orbit/OrbitalElements.h>
#include <rigidbody/orbit/OrbitForceModel.h>
#include <rigidbody/orbit/OrbitPropagator.h>
#include <rigidbody/orbit/OrbitFrames.h>
#include <rigidbody/orbit/OrbitTime.h>
#include <rigidbody/orbit/SunModel.h>
#include <rigidbody/orbit/EclipseModel.h>
#include <rigidbody/orbit/MoonModel.h>
#include <rigidbody/orbit/ThirdBodyGravity.h>
#include <rigidbody/orbit/AtmosphericDrag.h>
#include <rigidbody/orbit/SolarRadiationPressure.h>
#include <rigidbody/orbit/CelestialSystem.h>
#include <rigidbody/orbit/CelestialPerturbation.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/environment/central_body/CentralBodyGravity.h>
#include <cstdio>
#include <cmath>
#include <memory>

namespace
{
int g_failures = 0;

void check(bool pass, const char *description)
{
  std::printf("[%s] %s\n", pass ? "PASS" : "FAIL", description);
  if (!pass)
    g_failures++;
}

constexpr double MU_EARTH = 3.986004418e14;
constexpr double EARTH_RADIUS_M = 6.371e6;

void checkOrbitalElementsCircular()
{
  double altitudeM = 500e3;
  double inclinationRad = 51.6 * OrbitFrames::DEG2RAD;
  OrbitalElements elements = OrbitalElements::circular(altitudeM, inclinationRad, EARTH_RADIUS_M);
  OrbitState state = elements.toState(MU_EARTH);

  double expectedRadius = EARTH_RADIUS_M + altitudeM;
  double actualRadius = glm::length(state.position);
  check(std::abs(actualRadius - expectedRadius) < 1.0,
        "OrbitalElements::circular: |position| matches semi-major axis");

  double expectedSpeed = std::sqrt(MU_EARTH / expectedRadius); // circular velocity
  double actualSpeed = glm::length(state.velocity);
  check(std::abs(actualSpeed - expectedSpeed) < 1e-6 * expectedSpeed,
        "OrbitalElements::circular: |velocity| matches circular-orbit speed");

  double radialDot = glm::dot(glm::normalize(state.position), glm::normalize(state.velocity));
  check(std::abs(radialDot) < 1e-9,
        "OrbitalElements::circular: velocity is perpendicular to position");
}

void checkOrbitalElementsRoundTrip()
{
  // Deliberately eccentric/inclined/non-zero RAAN+argPeriapsis, to
  // exercise every branch of fromState() (the circular check above can't
  // -- eccentricity and argument of periapsis are degenerate at e=0).
  OrbitalElements original;
  original.semiMajorAxisM = EARTH_RADIUS_M + 800e3;
  original.eccentricity = 0.15;
  original.inclinationRad = 63.4 * OrbitFrames::DEG2RAD;
  original.raanRad = 40.0 * OrbitFrames::DEG2RAD;
  original.argPeriapsisRad = 75.0 * OrbitFrames::DEG2RAD;
  original.trueAnomalyRad = 110.0 * OrbitFrames::DEG2RAD;

  OrbitState state = original.toState(MU_EARTH);
  OrbitalElements recovered = OrbitalElements::fromState(state, MU_EARTH);

  check(std::abs(recovered.semiMajorAxisM - original.semiMajorAxisM) < 1.0,
        "OrbitalElements::fromState: semi-major axis round-trips");
  check(std::abs(recovered.eccentricity - original.eccentricity) < 1e-9,
        "OrbitalElements::fromState: eccentricity round-trips");
  check(std::abs(recovered.inclinationRad - original.inclinationRad) < 1e-9,
        "OrbitalElements::fromState: inclination round-trips");
  check(std::abs(recovered.raanRad - original.raanRad) < 1e-9,
        "OrbitalElements::fromState: RAAN round-trips");
  check(std::abs(recovered.argPeriapsisRad - original.argPeriapsisRad) < 1e-9,
        "OrbitalElements::fromState: argument of periapsis round-trips");
  check(std::abs(recovered.trueAnomalyRad - original.trueAnomalyRad) < 1e-9,
        "OrbitalElements::fromState: true anomaly round-trips");
}

void checkGeodeticRoundTrip()
{
  double latDeg = -33.4;
  double lonDeg = 151.2; // an arbitrary real-world-scale point (Sydney-ish)
  glm::dvec3 ecef = OrbitFrames::geodeticToECEF(latDeg, lonDeg);
  OrbitFrames::Geodetic recovered = OrbitFrames::ecefToGeodetic(ecef);

  check(std::abs(recovered.latDeg - latDeg) < 1e-9,
        "OrbitFrames::ecefToGeodetic: latitude round-trips through geodeticToECEF");
  check(std::abs(recovered.lonDeg - lonDeg) < 1e-9,
        "OrbitFrames::ecefToGeodetic: longitude round-trips through geodeticToECEF");

  // eciToGeodeticDeg at thetaGst=0 should match ecefToGeodetic directly
  // (ECI and ECEF coincide when the Earth-rotation angle is zero).
  OrbitFrames::Geodetic viaEci = OrbitFrames::eciToGeodeticDeg(ecef, 0.0);
  check(std::abs(viaEci.latDeg - latDeg) < 1e-9 && std::abs(viaEci.lonDeg - lonDeg) < 1e-9,
        "OrbitFrames::eciToGeodeticDeg: matches ecefToGeodetic at thetaGst=0");
}

void checkTwoBodyKeplerianPeriod()
{
  double altitudeM = 500e3;
  double semiMajorAxisM = EARTH_RADIUS_M + altitudeM;
  OrbitalElements elements = OrbitalElements::circular(altitudeM, 51.6 * OrbitFrames::DEG2RAD, EARTH_RADIUS_M);
  OrbitState state = elements.toState(MU_EARTH);
  OrbitState initial = state;

  OrbitPropagator propagator;
  propagator.addForceModel(std::make_unique<TwoBodyGravity>());

  double period = 2.0 * M_PI * std::sqrt(semiMajorAxisM * semiMajorAxisM * semiMajorAxisM / MU_EARTH);
  double dt = 1.0; // 1s RK4 steps
  int numSteps = static_cast<int>(period / dt);
  for (int i = 0; i < numSteps; i++)
    propagator.step(state, dt);

  double posError = glm::length(state.position - initial.position);
  double velError = glm::length(state.velocity - initial.velocity);
  // Loose tolerance (km-scale over a ~500km-altitude, ~7.6km/s orbit) --
  // this is a coarse sanity check on period/closure, not a precision claim.
  check(posError < 5000.0,
        "TwoBodyGravity: circular orbit returns to start after one period (position)");
  check(velError < 5.0,
        "TwoBodyGravity: circular orbit returns to start after one period (velocity)");
}

double computeRaanRad(const OrbitState &state)
{
  glm::dvec3 h = glm::cross(state.position, state.velocity); // orbit angular momentum / normal
  glm::dvec3 nodeVec = glm::cross(glm::dvec3(0.0, 0.0, 1.0), h); // ascending node direction
  return std::atan2(nodeVec.y, nodeVec.x);
}

void checkJ2NodalRegression()
{
  double altitudeM = 500e3;
  double semiMajorAxisM = EARTH_RADIUS_M + altitudeM;
  double inclinationRad = 51.6 * OrbitFrames::DEG2RAD;
  OrbitalElements elements = OrbitalElements::circular(altitudeM, inclinationRad, EARTH_RADIUS_M);
  OrbitState state = elements.toState(MU_EARTH);

  auto j2 = std::make_unique<J2Perturbation>();
  double j2Coeff = j2->j2;
  double planetRadius = j2->planetRadiusM;

  OrbitPropagator propagator;
  propagator.addForceModel(std::make_unique<TwoBodyGravity>());
  propagator.addForceModel(std::move(j2));

  double raan0 = computeRaanRad(state);

  double propagateSeconds = 5.0 * 86400.0; // 5 days
  double dt = 5.0;
  int numSteps = static_cast<int>(propagateSeconds / dt);
  for (int i = 0; i < numSteps; i++)
    propagator.step(state, dt);

  double raan1 = computeRaanRad(state);
  double measuredRateRadS = (raan1 - raan0) / propagateSeconds;

  // Closed-form secular nodal regression rate (Vallado eq. 9-39):
  // dRAAN/dt = -1.5 * n * J2 * (Re/p)^2 * cos(i), n = mean motion.
  double meanMotion = std::sqrt(MU_EARTH / (semiMajorAxisM * semiMajorAxisM * semiMajorAxisM));
  double p = semiMajorAxisM; // e = 0 -> semi-latus rectum equals semi-major axis
  double analyticRateRadS = -1.5 * meanMotion * j2Coeff * (planetRadius / p) * (planetRadius / p) * std::cos(inclinationRad);

  double relError = std::abs(measuredRateRadS - analyticRateRadS) / std::abs(analyticRateRadS);
  check(relError < 0.05,
        "J2Perturbation: nodal regression rate matches closed-form analytic rate (within 5%)");
}

void checkSunAndEclipse()
{
  double jd = OrbitTime::julianDate(2026, 3, 20, 12, 0, 0.0); // roughly equinox
  glm::dvec3 sunDir = SunModel::directionEci(jd);
  check(std::abs(glm::length(sunDir) - 1.0) < 1e-9,
        "SunModel::directionEci returns a unit vector");

  // Satellite directly behind Earth from the Sun's perspective, at LEO
  // altitude -- must be in eclipse.
  glm::dvec3 satBehindEarth = -sunDir * (EARTH_RADIUS_M + 500e3);
  check(EclipseModel::inEclipse(satBehindEarth, sunDir),
        "EclipseModel::inEclipse: satellite directly behind Earth is in shadow");

  // Satellite on the sunward side -- must not be in eclipse.
  glm::dvec3 satSunward = sunDir * (EARTH_RADIUS_M + 500e3);
  check(!EclipseModel::inEclipse(satSunward, sunDir),
        "EclipseModel::inEclipse: satellite on the sunward side is not in shadow");
}

void checkCalendarDateRoundTrip()
{
  // Several dates across a year (including a leap-year Feb 29 and a
  // year boundary) to exercise calendarDate()'s month/year carry logic,
  // not just a single easy case.
  struct Case
  {
    int year, month, day, hour, minute;
    double second;
  };
  const Case cases[] = {
      {2026, 1, 1, 0, 0, 0.0},
      {2026, 3, 20, 12, 0, 0.0},
      {2024, 2, 29, 23, 59, 59.5}, // leap day, near midnight rollover
      {2025, 12, 31, 23, 59, 59.999},
      {2030, 7, 4, 14, 21, 3.0},
  };

  for (const Case &c : cases)
  {
    double jd = OrbitTime::julianDate(c.year, c.month, c.day, c.hour, c.minute, c.second);
    int y, mo, d, h, mi;
    double s;
    OrbitTime::calendarDate(jd, y, mo, d, h, mi, s);

    // 1ms tolerance, not 1e-6s: a Julian Date near 2.46e6 has ~15-16
    // significant decimal digits of double precision to work with, so
    // sub-millisecond round-off through julianDate's own chain of
    // divisions/multiplications (and calendarDate's inverse) is expected,
    // not a bug -- nowhere near the whole-second resolution anything
    // consuming this (e.g. a pass-schedule AOS/LOS display) actually needs.
    bool match = (y == c.year && mo == c.month && d == c.day && h == c.hour &&
                  mi == c.minute && std::abs(s - c.second) < 1e-3);
    // The 23:59:59.999 case may legitimately carry into the next second
    // (60.0 - 59.999 = 0.001, within float round-off of the carry
    // threshold in calendarDate's own guard) -- accept a carry to the
    // next day as equally correct, not just an exact field match.
    if (!match && c.second > 59.9)
    {
      double jdRecovered = OrbitTime::julianDate(y, mo, d, h, mi, s);
      match = std::abs(jdRecovered - jd) < (0.5 / OrbitTime::SECONDS_PER_DAY);
    }
    char msg[256];
    std::snprintf(msg, sizeof(msg),
                  "OrbitTime::calendarDate: round-trips %04d-%02d-%02d %02d:%02d:%06.3f (got %04d-%02d-%02d %02d:%02d:%06.3f)",
                  c.year, c.month, c.day, c.hour, c.minute, c.second, y, mo, d, h, mi, s);
    check(match, msg);
  }
}

void checkMoonModel()
{
  double jd = OrbitTime::julianDate(2026, 3, 20, 12, 0, 0.0);
  glm::dvec3 dir = MoonModel::directionEci(jd);
  check(std::abs(glm::length(dir) - 1.0) < 1e-9,
        "MoonModel::directionEci returns a unit vector");

  glm::dvec3 pos = MoonModel::positionEci(jd);
  double dist = glm::length(pos);
  // Earth-Moon distance ranges from ~356,500 km (perigee) to ~406,700 km
  // (apogee); a low-precision model should still land well inside this
  // range for any date.
  check(dist > 3.5e8 && dist < 4.1e8,
        "MoonModel::positionEci: distance is within the real perigee/apogee range");
}

void checkThirdBodyGravity()
{
  // LEO state (position only matters for the acceleration call; velocity
  // is unused by ThirdBodyGravity).
  OrbitState state;
  state.position = glm::dvec3(EARTH_RADIUS_M + 500e3, 0.0, 0.0);
  double epochJd = OrbitTime::julianDate(2026, 3, 20, 12, 0, 0.0);

  ThirdBodyGravity sunGravity(ThirdBodyType::Sun);
  sunGravity.epochJd = epochJd;
  glm::dvec3 sunAccel = sunGravity.acceleration(state, 0.0);

  ThirdBodyGravity moonGravity(ThirdBodyType::Moon);
  moonGravity.epochJd = epochJd;
  glm::dvec3 moonAccel = moonGravity.acceleration(state, 0.0);

  // Leading-order tidal approximation (valid since d_third-body >>
  // r_satellite): |a| ~= 2 * mu_third * r_sat / d^3. Loose tolerance
  // since this is a first-order approximation, not the exact formula.
  double rSat = glm::length(state.position);
  double dSun = glm::length(SunModel::positionEci(epochJd));
  double dMoon = glm::length(MoonModel::positionEci(epochJd));
  constexpr double GM_SUN = 1.32712440018e20;
  constexpr double GM_MOON = 4.9048695e12;
  double sunTidalApprox = 2.0 * GM_SUN * rSat / (dSun * dSun * dSun);
  double moonTidalApprox = 2.0 * GM_MOON * rSat / (dMoon * dMoon * dMoon);

  double sunRelErr = std::abs(glm::length(sunAccel) - sunTidalApprox) / sunTidalApprox;
  double moonRelErr = std::abs(glm::length(moonAccel) - moonTidalApprox) / moonTidalApprox;
  check(sunRelErr < 0.5,
        "ThirdBodyGravity(Sun): magnitude at LEO matches leading-order tidal approximation");
  check(moonRelErr < 0.5,
        "ThirdBodyGravity(Moon): magnitude at LEO matches leading-order tidal approximation");

  // Known real-world orders of magnitude (~1e-7 to ~1e-6 m/s^2 at LEO).
  check(glm::length(sunAccel) > 1e-8 && glm::length(sunAccel) < 1e-5,
        "ThirdBodyGravity(Sun): magnitude at LEO is the expected order of magnitude");
  check(glm::length(moonAccel) > 1e-8 && glm::length(moonAccel) < 1e-5,
        "ThirdBodyGravity(Moon): magnitude at LEO is the expected order of magnitude");
}

void checkAtmosphericDrag()
{
  AtmosphericDrag drag(0.01, 1.33); // representative 1U-ish cross-section/mass

  auto accelAtAltitude = [&](double altitudeM) {
    OrbitState state;
    state.position = glm::dvec3(EARTH_RADIUS_M + altitudeM, 0.0, 0.0);
    // Circular-ish velocity in +Y, so there's real relative motion vs. the
    // co-rotating atmosphere for drag to act against.
    state.velocity = glm::dvec3(0.0, std::sqrt(MU_EARTH / (EARTH_RADIUS_M + altitudeM)), 0.0);
    return drag.acceleration(state, 0.0);
  };

  glm::dvec3 accel300 = accelAtAltitude(300e3);
  glm::dvec3 accel500 = accelAtAltitude(500e3);
  glm::dvec3 accel800 = accelAtAltitude(800e3);
  check(glm::length(accel300) > glm::length(accel500) && glm::length(accel500) > glm::length(accel800),
        "AtmosphericDrag: acceleration magnitude decreases monotonically with altitude");

  glm::dvec3 accel1200 = accelAtAltitude(1200e3);
  check(glm::length(accel1200) == 0.0,
        "AtmosphericDrag: acceleration vanishes above 1000km (out of model range)");

  OrbitState state300;
  state300.position = glm::dvec3(EARTH_RADIUS_M + 300e3, 0.0, 0.0);
  state300.velocity = glm::dvec3(0.0, std::sqrt(MU_EARTH / (EARTH_RADIUS_M + 300e3)), 0.0);
  glm::dvec3 accel = drag.acceleration(state300, 0.0);
  check(glm::dot(accel, state300.velocity) < 0.0,
        "AtmosphericDrag: acceleration opposes velocity (decelerating)");
}

void checkSolarRadiationPressure()
{
  SolarRadiationPressure srp(0.01, 1.33);
  double epochJd = OrbitTime::julianDate(2026, 3, 20, 12, 0, 0.0);
  srp.epochJd = epochJd;
  glm::dvec3 sunDir = SunModel::directionEci(epochJd);

  OrbitState sunwardState;
  sunwardState.position = sunDir * (EARTH_RADIUS_M + 500e3);
  glm::dvec3 sunwardAccel = srp.acceleration(sunwardState, 0.0);
  check(glm::length(sunwardAccel) > 0.0,
        "SolarRadiationPressure: nonzero acceleration when sunlit");
  check(glm::dot(sunwardAccel, sunDir) < 0.0,
        "SolarRadiationPressure: acceleration points away from the Sun");

  OrbitState eclipsedState;
  eclipsedState.position = -sunDir * (EARTH_RADIUS_M + 500e3);
  glm::dvec3 eclipsedAccel = srp.acceleration(eclipsedState, 0.0);
  check(glm::length(eclipsedAccel) == 0.0,
        "SolarRadiationPressure: zero acceleration while in eclipse");
}

void checkCentralBodyGravityClosedOrbit()
{
  float altitudeM = 500e3f;
  float radiusM = 6.371e6f + altitudeM;
  float speed = std::sqrt(3.986004418e14f / radiusM); // circular velocity, float32

  PhysicsWorld world;
  world.fixedTimestep = 1.0f;
  auto gravityOwned = std::make_unique<CentralBodyGravity>();
  CentralBodyGravity *gravity = gravityOwned.get();
  gravity->mu = 3.986004418e14f;
  world.addGlobalForceGenerator(std::move(gravityOwned));

  RigidBody *body = world.createBody(RigidBodyShape::SPHERE, glm::vec3(0.1f, 0.0f, 0.0f), 1.0f);
  body->position = glm::vec3(radiusM, 0.0f, 0.0f);
  body->velocity = glm::vec3(0.0f, speed, 0.0f);
  glm::vec3 startPos = body->position;

  float period = 2.0f * static_cast<float>(M_PI) * std::sqrt(radiusM * radiusM * radiusM / gravity->mu);
  int numSteps = static_cast<int>(period / world.fixedTimestep);
  for (int i = 0; i < numSteps; i++)
    world.step(world.fixedTimestep);

  float posError = glm::length(body->position - startPos);
  // Float32 over ~5700 steps at LEO radius -- looser tolerance than the
  // double-precision OrbitPropagator check above, by design (see
  // CentralBodyGravity.h's precision note).
  check(posError < 50000.0f,
        "CentralBodyGravity: RigidBody under real 1/r^2 gravity returns near start after one period");
}

// Builds a Sun (root) -> Earth -> Moon hierarchy, reusing the existing
// analytic SunModel/MoonModel formulas as each child's parent-relative
// ephemeris -- Earth's position relative to the Sun is exactly
// -SunModel::positionEci(jd) (SunModel already gives the Sun's position
// relative to Earth), and Moon's relative to Earth is exactly
// MoonModel::positionEci(jd).
CelestialSystem buildSunEarthMoonSystem(CelestialBody *&outSun, CelestialBody *&outEarth, CelestialBody *&outMoon)
{
  constexpr double GM_SUN = 1.32712440018e20;
  constexpr double GM_MOON = 4.9048695e12;

  CelestialSystem system;

  CelestialBodyParams sunParams;
  sunParams.mu = GM_SUN;
  outSun = system.addBody("Sun", sunParams);
  system.starBody = outSun;

  CelestialBodyParams earthParams;
  earthParams.mu = MU_EARTH;
  earthParams.radiusM = EARTH_RADIUS_M;
  earthParams.dipoleTiltDeg = 11.0;
  earthParams.dipoleScaleTm3 = 7.94e15;
  outEarth = system.addBody("Earth", earthParams, outSun);
  outEarth->analyticPositionFn = [](double jd) { return -SunModel::positionEci(jd); };

  CelestialBodyParams moonParams;
  moonParams.mu = GM_MOON;
  outMoon = system.addBody("Moon", moonParams, outEarth);
  outMoon->analyticPositionFn = [](double jd) { return MoonModel::positionEci(jd); };

  return system;
}

void checkCelestialSystemHierarchy()
{
  double epochJd = OrbitTime::julianDate(2026, 3, 20, 12, 0, 0.0);

  CelestialBody *sun, *earth, *moon;
  CelestialSystem system = buildSunEarthMoonSystem(sun, earth, moon);

  check(glm::length(system.absolutePosition(sun, epochJd)) == 0.0,
        "CelestialSystem: root body stays at the system origin");

  glm::dvec3 expectedEarthPos = -SunModel::positionEci(epochJd);
  check(glm::length(system.absolutePosition(earth, epochJd) - expectedEarthPos) < 1.0,
        "CelestialSystem: Earth's absolute position matches -SunModel::positionEci (Sun-relative)");

  glm::dvec3 expectedMoonPos = expectedEarthPos + MoonModel::positionEci(epochJd);
  check(glm::length(system.absolutePosition(moon, epochJd) - expectedMoonPos) < 1.0,
        "CelestialSystem: Moon's absolute position sums Earth's + its own parent-relative offset");
}

void checkCelestialPerturbationMatchesThirdBodyGravity()
{
  double epochJd = OrbitTime::julianDate(2026, 3, 20, 12, 0, 0.0);

  CelestialBody *sun, *earth, *moon;
  CelestialSystem system = buildSunEarthMoonSystem(sun, earth, moon);

  OrbitState state;
  state.position = glm::dvec3(EARTH_RADIUS_M + 500e3, 0.0, 0.0);

  CelestialPerturbation sunPerturbation(system, *earth, *sun);
  sunPerturbation.jd = epochJd;
  ThirdBodyGravity thirdBodySun(ThirdBodyType::Sun);
  thirdBodySun.epochJd = epochJd;
  double sunRelErr = glm::length(sunPerturbation.acceleration(state, 0.0) - thirdBodySun.acceleration(state, 0.0)) /
                      glm::length(thirdBodySun.acceleration(state, 0.0));
  check(sunRelErr < 1e-9,
        "CelestialPerturbation(Earth, Sun): reproduces ThirdBodyGravity(Sun) exactly");

  CelestialPerturbation moonPerturbation(system, *earth, *moon);
  moonPerturbation.jd = epochJd;
  ThirdBodyGravity thirdBodyMoon(ThirdBodyType::Moon);
  thirdBodyMoon.epochJd = epochJd;
  double moonRelErr = glm::length(moonPerturbation.acceleration(state, 0.0) - thirdBodyMoon.acceleration(state, 0.0)) /
                       glm::length(thirdBodyMoon.acceleration(state, 0.0));
  check(moonRelErr < 1e-9,
        "CelestialPerturbation(Earth, Moon): reproduces ThirdBodyGravity(Moon) exactly");
}

void checkPhysicsWorldOrbitalMode()
{
  double epochJd = OrbitTime::julianDate(2026, 3, 20, 12, 0, 0.0);

  CelestialBody *sun, *earth, *moon;
  CelestialSystem system = buildSunEarthMoonSystem(sun, earth, moon);

  OrbitalElements elements = OrbitalElements::circular(500e3, 51.6 * OrbitFrames::DEG2RAD, EARTH_RADIUS_M);
  OrbitState initialState = elements.toState(MU_EARTH);

  // Reference: the exact same force list (primary + Sun/Moon perturbers),
  // propagated standalone -- proves PhysicsWorld's internal bridging
  // doesn't silently diverge from calling OrbitPropagator directly.
  OrbitState referenceState = initialState;
  OrbitPropagator referenceProp;
  referenceProp.addForceModel(std::make_unique<TwoBodyGravity>());
  auto sunPerturbRef = std::make_unique<CelestialPerturbation>(system, *earth, *sun);
  auto moonPerturbRef = std::make_unique<CelestialPerturbation>(system, *earth, *moon);
  CelestialPerturbation *sunPerturbRefPtr = sunPerturbRef.get();
  CelestialPerturbation *moonPerturbRefPtr = moonPerturbRef.get();
  referenceProp.addForceModel(std::move(sunPerturbRef));
  referenceProp.addForceModel(std::move(moonPerturbRef));

  PhysicsWorld world;
  world.attachCelestialSystem(&system, epochJd);
  RigidBody *body = world.createBody(RigidBodyShape::SPHERE, glm::vec3(0.1f, 0.0f, 0.0f), 1.0f);
  world.setOrbitalMode(body, earth, {sun, moon}, initialState);

  double dt = 60.0;
  int numSteps = 200;
  double jd = epochJd;
  for (int i = 0; i < numSteps; i++)
  {
    sunPerturbRefPtr->jd = jd;
    moonPerturbRefPtr->jd = jd;
    world.step(static_cast<float>(dt));
    referenceProp.step(referenceState, dt);
    jd = OrbitTime::advance(jd, dt);
  }

  float posError = glm::length(body->position - glm::vec3(referenceState.position));
  check(posError < 10.0f,
        "PhysicsWorld::setOrbitalMode: internal bridging matches standalone OrbitPropagator with the same force list");

  // isInEclipse/sunDirectionAt: cross-check against the same computation
  // done manually from the reference state.
  glm::dvec3 sunRelToEarth = system.absolutePosition(sun, jd) - system.absolutePosition(earth, jd);
  glm::dvec3 expectedSunDir = glm::normalize(sunRelToEarth - referenceState.position);
  bool expectedEclipse = EclipseModel::inShadow(referenceState.position, expectedSunDir, glm::dvec3(0.0), EARTH_RADIUS_M);
  check(world.isInEclipse(body) == expectedEclipse,
        "PhysicsWorld::isInEclipse: matches EclipseModel::inShadow computed manually from the same state");

  glm::vec3 sunDirErr = world.sunDirectionAt(body) - glm::vec3(expectedSunDir);
  check(glm::length(sunDirErr) < 1e-4f,
        "PhysicsWorld::sunDirectionAt: matches the direction computed manually from the same state");

  glm::vec3 field = world.ambientFieldAt(body);
  check(glm::length(field) > 0.0f,
        "PhysicsWorld::ambientFieldAt: returns a nonzero field for an orbital-mode body around Earth");
}
} // namespace

int main()
{
  checkOrbitalElementsCircular();
  checkOrbitalElementsRoundTrip();
  checkGeodeticRoundTrip();
  checkTwoBodyKeplerianPeriod();
  checkJ2NodalRegression();
  checkSunAndEclipse();
  checkCalendarDateRoundTrip();
  checkMoonModel();
  checkThirdBodyGravity();
  checkAtmosphericDrag();
  checkSolarRadiationPressure();
  checkCentralBodyGravityClosedOrbit();
  checkCelestialSystemHierarchy();
  checkCelestialPerturbationMatchesThirdBodyGravity();
  checkPhysicsWorldOrbitalMode();

  std::printf("\n%s\n", g_failures == 0 ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED");
  return g_failures == 0 ? 0 : 1;
}
