#pragma once

// A simple Coulomb-counting battery: stores energy (not charge) in
// joules, since every consumer/generator in this project's power model
// already reports watts, and integrating power directly avoids introducing
// a separate current/voltage-conversion step this sim has no real need
// for. State of charge and voltage are both derived from the same energy
// value, not tracked independently, so they can never drift out of sync
// with each other.
//
// Voltage is a simple linear function of state of charge (real Li-ion
// discharge curves are flatter in the middle and steeper at the ends, but
// a straight line between the empty/full endpoints is the honest
// equivalent here without modeling real cell chemistry) -- good enough to
// show "voltage sags as the battery depletes," which is the property
// anything reacting to it (FDIR's low-battery fault, a UI panel) actually
// cares about.
class Battery
{
public:
  float capacityJ;
  float energyJ;
  float minVoltage; // V, at 0% state of charge
  float maxVoltage; // V, at 100% state of charge

  // capacityWh: nameplate capacity in watt-hours (how batteries are
  // actually rated) -- converted to joules internally (1 Wh = 3600 J)
  // since that's the unit update() integrates in.
  explicit Battery(float capacityWh = 10.0f, float minVoltageIn = 6.0f,
                   float maxVoltageIn = 8.4f, float initialSoc = 1.0f);

  // Integrates net power (generated minus consumed, watts; positive
  // charges, negative discharges) over dt seconds, clamped to
  // [0, capacityJ] -- a battery can't go negative or overcharge past its
  // nameplate capacity, the same hard limits a real charge controller
  // enforces.
  void update(float netPowerW, float dt);

  float stateOfCharge() const; // 0-1
  float voltage() const;       // V, linear interpolation between minVoltage/maxVoltage by state of charge
  bool isDepleted() const { return stateOfCharge() <= 0.0f; }
};
