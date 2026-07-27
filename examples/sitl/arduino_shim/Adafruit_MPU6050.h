#pragma once
// Minimal native stand-in for the two Adafruit libraries IMUFilter.cpp
// depends on (Adafruit_MPU6050 + the sensors_event_t/sensors_vec_t types
// normally pulled in from Adafruit_Sensor), providing only the members
// IMUFilter.cpp actually touches. Real sensing (I2C, register reads, raw
// count -> physical unit conversion) is replaced entirely: sitl.cpp
// injects already-physical-unit accel/gyro readings from this project's
// own IMU sensor model (rigidbody/sensors/IMU.h) once per control tick,
// and getEvent() just hands back whatever was last injected.
// IMUFilter's own complementary-filter fusion math -- the actual thing
// under test -- runs completely unmodified on top of that.
//
// Injection is a free-function/global API (SITLBridge namespace below)
// rather than a method on Adafruit_MPU6050 itself, because IMUFilter.h
// keeps its `mpu` member private (correctly so -- that's the real
// firmware's own encapsulation, not something this project should reach
// into). There's only ever one physical MPU6050 in the real firmware
// anyway, so one global injection slot is exactly as much state as the
// real hardware has.

struct sensors_vec_t
{
  float x = 0.0f, y = 0.0f, z = 0.0f;
};

struct sensors_event_t
{
  sensors_vec_t acceleration;
  sensors_vec_t gyro;
};

// Values are arbitrary -- Adafruit_MPU6050's setX() calls become no-ops
// below, so nothing ever reads these back.
enum mpu6050_accel_range_t { MPU6050_RANGE_4_G };
enum mpu6050_gyro_range_t { MPU6050_RANGE_500_DEG };
enum mpu6050_bandwidth_t { MPU6050_BAND_21_HZ };

class Adafruit_MPU6050
{
public:
  bool begin() { return true; }
  void setAccelerometerRange(mpu6050_accel_range_t) {}
  void setGyroRange(mpu6050_gyro_range_t) {}
  void setFilterBandwidth(mpu6050_bandwidth_t) {}

  // Hands back the last SITLBridge::injectReading() value. Real
  // Adafruit_MPU6050::getEvent() also fills a temperature event and can
  // fail (I2C error); IMUFilter.cpp only ever uses accel/gyro and does
  // check the return value (see SITLBridge::injectFailure()).
  bool getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp);
};

namespace SITLBridge
{
// Call once per control tick, before IMUFilter::update()/calibrateGyro().
void injectReading(sensors_vec_t accel, sensors_vec_t gyro);

// Makes the next getEvent() call return false, exercising the real
// firmware's imuFailStreak / I2C-bus-recovery logic (see main.cpp) against
// a simulated dropout instead of a real wedged bus.
void injectFailure();
} // namespace SITLBridge
