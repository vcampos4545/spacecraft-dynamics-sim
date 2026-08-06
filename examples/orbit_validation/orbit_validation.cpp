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
  checkTwoBodyKeplerianPeriod();
  checkJ2NodalRegression();
  checkSunAndEclipse();
  checkCentralBodyGravityClosedOrbit();

  std::printf("\n%s\n", g_failures == 0 ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED");
  return g_failures == 0 ? 0 : 1;
}
