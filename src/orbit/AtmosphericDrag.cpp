#include <rigidbody/orbit/AtmosphericDrag.h>
#include <array>
#include <cmath>

namespace
{
// Piecewise exponential fit to the US Standard Atmosphere 1976, covering
// 0-1000 km with emphasis on LEO accuracy (100-800 km) -- each layer is
// [base altitude m, reference density kg/m^3, scale height m], with scale
// heights derived from adjacent-layer density ratios
// (H = delta_h / ln(rho1/rho2)). A multi-layer model, unlike
// UniformDrag/CentralBodyDrag's single fixed scale height -- accurate
// across the full LEO altitude range rather than just near one reference
// altitude, which matters here since this is the truth propagator's own
// drag term, not a short-duration/local-effect approximation.
struct Layer
{
  double baseAltitudeM;
  double refDensityKgM3;
  double scaleHeightM;
};

constexpr std::array<Layer, 23> LAYERS = {{
    {0.0, 1.225000e+00, 8500.0},
    {25000.0, 4.008000e-02, 6600.0},
    {30000.0, 1.841000e-02, 6700.0},
    {40000.0, 3.996000e-03, 7100.0},
    {50000.0, 1.027000e-03, 7900.0},
    {60000.0, 3.097000e-04, 8100.0},
    {70000.0, 8.283000e-05, 7900.0},
    {80000.0, 1.846000e-05, 7200.0},
    {100000.0, 5.600000e-07, 5900.0},
    {110000.0, 9.750000e-08, 7350.0},
    {120000.0, 2.420000e-08, 7800.0},
    {130000.0, 8.680000e-09, 8200.0},
    {140000.0, 3.960000e-09, 8700.0},
    {150000.0, 2.076000e-09, 8900.0},
    {180000.0, 5.194000e-10, 10200.0},
    {200000.0, 2.541000e-10, 11200.0},
    {250000.0, 6.073000e-11, 31100.0},
    {300000.0, 1.916000e-11, 43700.0},
    {350000.0, 6.703000e-12, 54900.0},
    {400000.0, 2.803000e-12, 58800.0},
    {500000.0, 5.215000e-13, 63700.0},
    {600000.0, 8.770000e-14, 71400.0},
    {700000.0, 3.070000e-14, 88800.0},
}};

double atmosphericDensity(double altitudeM)
{
  if (altitudeM >= 1.0e6)
    return 0.0; // above 1000km: negligible
  if (altitudeM < 0.0)
    altitudeM = 0.0;

  const Layer *layer = &LAYERS[0];
  for (int i = static_cast<int>(LAYERS.size()) - 1; i >= 0; i--)
  {
    if (altitudeM >= LAYERS[i].baseAltitudeM)
    {
      layer = &LAYERS[i];
      break;
    }
  }
  return layer->refDensityKgM3 * std::exp(-(altitudeM - layer->baseAltitudeM) / layer->scaleHeightM);
}
} // namespace

AtmosphericDrag::AtmosphericDrag(double areaM2, double massKg) : areaM2(areaM2), massKg(massKg)
{
}

glm::dvec3 AtmosphericDrag::acceleration(const OrbitState &state, double /*t*/) const
{
  double altitudeM = glm::length(state.position) - planetRadiusM;
  double rho = atmosphericDensity(altitudeM);
  if (rho < 1e-30)
    return glm::dvec3(0.0);

  // Atmosphere co-rotates with the planet: v_atm = omega x r.
  glm::dvec3 omega(0.0, 0.0, planetRotationRateRadS);
  glm::dvec3 vAtm = glm::cross(omega, state.position);
  glm::dvec3 vRel = state.velocity - vAtm;
  double vRelMag = glm::length(vRel);
  if (vRelMag < 1e-10)
    return glm::dvec3(0.0);

  double beta = dragCoefficient * areaM2 / massKg; // ballistic coefficient
  double accelMag = 0.5 * rho * beta * vRelMag * vRelMag;
  return (-accelMag / vRelMag) * vRel;
}
