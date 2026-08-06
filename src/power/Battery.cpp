#include <rigidbody/power/Battery.h>
#include <algorithm>

namespace
{
constexpr float JOULES_PER_WATT_HOUR = 3600.0f;
}

Battery::Battery(float capacityWh, float minVoltageIn, float maxVoltageIn, float initialSoc)
    : capacityJ(capacityWh * JOULES_PER_WATT_HOUR),
      energyJ(capacityWh * JOULES_PER_WATT_HOUR * std::clamp(initialSoc, 0.0f, 1.0f)),
      minVoltage(minVoltageIn),
      maxVoltage(maxVoltageIn)
{
}

void Battery::update(float netPowerW, float dt)
{
  energyJ = std::clamp(energyJ + netPowerW * dt, 0.0f, capacityJ);
}

float Battery::stateOfCharge() const
{
  return capacityJ > 1e-9f ? energyJ / capacityJ : 0.0f;
}

float Battery::voltage() const
{
  return minVoltage + (maxVoltage - minVoltage) * stateOfCharge();
}
