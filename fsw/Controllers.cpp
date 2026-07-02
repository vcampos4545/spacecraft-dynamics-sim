#include "Controllers.h"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/common.hpp>
#include <cmath>

void PIDController::autoTune(glm::mat3 inertiaTensor,
                             float settlingTime,
                             float dampingRatio)
{
  // Average moment of inertia
  float I =
      (inertiaTensor[0][0] +
       inertiaTensor[1][1] +
       inertiaTensor[2][2]) /
      3.0f;

  // Natural frequency
  float omega_n = 4.0f / (dampingRatio * settlingTime);

  // Gains
  Kp = I * omega_n * omega_n;
  Kd = 2.0f * dampingRatio * I * omega_n;
  Ki = Kp * 0.01f;

  // Anti-windup limit
  if (Ki > 0.0f)
  {
    maxIntegral = (Kp / Ki) * 0.5f;
  }
}

glm::vec3 PIDController::computeControlTorque(
    glm::quat targetAttitude,
    const glm::quat &q_current,
    const glm::vec3 &omega_current,
    float dt)
{
  // Quaternion attitude error in BODY frame
  glm::quat q_error = glm::inverse(q_current) * targetAttitude;

  // Shortest rotation
  if (q_error.w < 0.0f)
  {
    q_error = -q_error;
  }

  // Axis-angle approximation
  glm::vec3 attitudeError(q_error.x, q_error.y, q_error.z);
  attitudeError *= 2.0f;

  // Integral update (anti-windup)
  integralError += attitudeError * dt;
  integralError = glm::clamp(
      integralError,
      -glm::vec3(maxIntegral),
      glm::vec3(maxIntegral));

  // Derivative (target angular velocity = 0)
  glm::vec3 derivativeError = -omega_current;

  glm::vec3 torque =
      Kp * attitudeError +
      Ki * integralError +
      Kd * derivativeError;

  return -torque;
}

void PIDController::reset()
{
  integralError = glm::vec3(0.0f);
}

// class CascadedController
// {
// public:
//   float settlingTime = 5.0f;
//   float dampingRatio = 1.0f;

//   CascadedController() = default;

//   glm::vec3 computeControlTorque(
//       const glm::quat targetAttitude,
//       const glm::quat &q_current,
//       const glm::vec3 &omega_current,
//       float omega_max,
//       const glm::mat3 &I,
//       float dt)
//   {
//     // -------------------------------------------------
//     // Quaternion error (body frame)
//     // q_err rotates CURRENT -> TARGET
//     // -------------------------------------------------
//     glm::quat q_error = glm::inverse(q_current) * targetAttitude;

//     // Ensure shortest rotation path
//     if (q_error.w < 0.0f)
//       q_error = -q_error;

//     // -------------------------------------------------
//     // Extract quaternion vector part
//     // Small-angle approx: 2*q_vec ≈ rotation error
//     // -------------------------------------------------
//     glm::vec3 attitudeError = 2.0f * glm::vec3(q_error.x, q_error.y, q_error.z);

//     // -------------------------------------------------
//     // Natural frequency from settling time
//     // -------------------------------------------------
//     float omega_n = 4.0f / (dampingRatio * settlingTime);

//     // -------------------------------------------------
//     // OUTER LOOP: attitude error -> rate command
//     // -------------------------------------------------
//     float k_q = -1.0f * omega_n;
//     glm::vec3 omega_cmd = k_q * attitudeError;

//     // ----- Rate saturation -----
//     float omega_cmd_mag = glm::length(omega_cmd);
//     if (omega_cmd_mag > omega_max && omega_cmd_mag > 0.0f)
//     {
//       omega_cmd *= (omega_max / omega_cmd_mag);
//     }

//     // -------------------------------------------------
//     // INNER LOOP: rate tracking -> torque
//     // -------------------------------------------------
//     float omega_rate = 5.0f * omega_n;

//     glm::vec3 rate_error = omega_current - omega_cmd;

//     glm::vec3 tau = -(I * rate_error) * omega_rate;

//     return tau;
//   }
// };

// class MPCController
// {

//   glm::vec3 computeControlTorque() { return {0, 0, 0}; }
// };

void LQRController::autoTune(glm::mat3 inertiaTensor,
                             glm::vec3 Q_att_,
                             glm::vec3 Q_rate_,
                             glm::vec3 R_)
{
  Q_att = Q_att_;
  Q_rate = Q_rate_;
  R = R_;

  // Solve the CARE per-axis for A = [[0,1],[0,0]], B = [[0],[b]], b = 1/I_axis
  // Closed form (see header comment):
  //   p12 = sqrt(q1*r) / b
  //   p22 = sqrt(r * (2*p12 + q2)) / b
  //   K   = [b*p12/r, b*p22/r]
  for (int i = 0; i < 3; i++)
  {
    float I = inertiaTensor[i][i];
    float b = (I > 1e-9f) ? (1.0f / I) : 0.0f;

    float q1 = Q_att[i];
    float q2 = Q_rate[i];
    float r = R[i];

    if (b <= 0.0f || r <= 0.0f)
    {
      Kp[i] = 0.0f;
      Kd[i] = 0.0f;
      continue;
    }

    float p12 = std::sqrt(q1 * r) / b;
    float p22 = std::sqrt(r * (2.0f * p12 + q2)) / b;

    Kp[i] = b * p12 / r;
    Kd[i] = b * p22 / r;
  }
}

glm::vec3 LQRController::computeControlTorque(
    glm::quat targetAttitude,
    const glm::quat &q_current,
    const glm::vec3 &omega_current,
    float dt)
{
  // Quaternion attitude error in BODY frame
  glm::quat q_error = glm::inverse(q_current) * targetAttitude;

  // Shortest rotation
  if (q_error.w < 0.0f)
  {
    q_error = -q_error;
  }

  // Axis-angle approximation
  glm::vec3 attitudeError(q_error.x, q_error.y, q_error.z);
  attitudeError *= 2.0f;

  // u = -K x, x = [attitudeError, omega]
  glm::vec3 torque = Kp * attitudeError - Kd * omega_current;

  return -torque;
}