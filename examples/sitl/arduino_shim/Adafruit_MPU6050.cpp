#include "Adafruit_MPU6050.h"

namespace
{
sensors_vec_t g_accel;
sensors_vec_t g_gyro;
bool g_ok = true;
} // namespace

bool Adafruit_MPU6050::getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp)
{
  (void)temp;
  if (!g_ok)
  {
    // Real getEvent() also fails without updating accel/gyro -- IMUFilter's
    // caller (main.cpp) is expected to leave the last good theta/thetaDot
    // in place either way, so a stale event value doesn't matter.
    g_ok = true; // one-shot: SITLBridge::injectFailure() arms exactly one failed read
    return false;
  }
  if (accel)
    accel->acceleration = g_accel;
  if (gyro)
    gyro->gyro = g_gyro;
  return true;
}

void SITLBridge::injectReading(sensors_vec_t accel, sensors_vec_t gyro)
{
  g_accel = accel;
  g_gyro = gyro;
}

void SITLBridge::injectFailure()
{
  g_ok = false;
}
