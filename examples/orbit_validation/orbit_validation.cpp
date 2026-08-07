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
} // namespace

int main()
{
  checkOrbitalElementsCircular();
  checkOrbitalElementsRoundTrip();
  checkGeodeticRoundTrip();
  checkTwoBodyKeplerianPeriod();
  checkJ2NodalRegression();
  checkSunAndEclipse();
  checkMoonModel();
  checkThirdBodyGravity();
  checkAtmosphericDrag();
  checkSolarRadiationPressure();
  checkCentralBodyGravityClosedOrbit();

  std::printf("\n%s\n", g_failures == 0 ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED");
  return g_failures == 0 ? 0 : 1;
}
