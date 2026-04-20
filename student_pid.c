#include "student_pid.h"
#include "num.h"
#include <math.h>
#include <float.h>

/**
 * PID object initialization.
 *
 * @param[out] pid   A pointer to the pid object to initialize.
 * @param[in] desired  The initial set point.
 * @param[in] kp        The proportional gain
 * @param[in] ki        The integral gain
 * @param[in] kd        The derivative gain
 * @param[in] dt        Delta time since the last call
 * @param[in] samplingRate Frequency the update will be called
 * @param[in] cutoffFreq   Frequency to set the low pass filter cutoff at
 * @param[in] enableDFilter Enable setting for the D lowpass filter
 */
void studentPidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt,
             const float samplingRate, const float cutoffFreq,
             bool enableDFilter)
{
  pid->error = 0.0f;
  pid->prevError = 0.0f;
  pid->integ = 0.0f;
  pid->deriv = 0.0f;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->dt = dt;
  pid->setpoint = desired;
  pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->outputLimit = DEFAULT_PID_OUTPUT_LIMIT;
  pid->outP = 0.0f;
  pid->outI = 0.0f;
  pid->outD = 0.0f;
 
  //additional initialization for optional low pass filter
  pid->enableDFilter = enableDFilter;
  if (pid->enableDFilter)
  {
    lpf2pInit(&pid->dFilter, samplingRate, cutoffFreq);
  }
}

/**
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if studentPidSetError() has been used.
 * @return PID algorithm output
 */
float studentPidUpdate(PidObject* pid, const float measured, const bool updateError)
{
  // calculate error unless caller already set it manually
  if (updateError) {
    studentPidSetError(pid, pid->setpoint - measured);
  }

  // accumulate integral, clamp to prevent windup
  pid->integ += pid->error * pid->dt;
  if (pid->iLimit != 0.0f) {
    pid->integ = constrain(pid->integ, -pid->iLimit, pid->iLimit);
  }

  // filter if enabled to reduce noise sensitivity
  float deriv = 0.0f;
  if (pid->dt > 0.0f) {
    deriv = (pid->error - pid->prevError) / pid->dt;
  }

  if (pid->enableDFilter)
  {
    pid->deriv = lpf2pApply(&pid->dFilter, deriv);
    if (isnan(pid->deriv)) {
      pid->deriv = 0.0f;
    }
  } else {
    pid->deriv = deriv;
  }

  // store individual terms for GUI logging
  pid->outP = pid->kp * pid->error;
  pid->outI = pid->ki * pid->integ;
  pid->outD = pid->kd * pid->deriv;

  // clamp output if limit is set
  float update = pid->outP + pid->outI + pid->outD;
  if (pid->outputLimit != 0.0f) {
    update = constrain(update, -pid->outputLimit, pid->outputLimit);
  }

  // store error for next derivative calculation
  pid->prevError = pid->error;

  return update;
}

/**
 * Set the integral limit for this PID in deg.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] limit Pid integral swing limit.
 */
void studentPidSetIntegralLimit(PidObject* pid, const float limit) {
  pid->iLimit = fabsf(limit);
}

/**
 * Reset the PID error values
 *
 * @param[in] pid   A pointer to the pid object.
 */
void studentPidReset(PidObject* pid)
{
  pid->error = 0.0f;
  pid->prevError = 0.0f;
  pid->integ = 0.0f;
  pid->deriv = 0.0f;
  pid->outP = 0.0f;
  pid->outI = 0.0f;
  pid->outD = 0.0f;

  if (pid->enableDFilter) {
    lpf2pReset(&pid->dFilter, 0.0f);
  }
}

/**
 * Set a new error. Use if a special error calculation is needed.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] error The new error
 */
void studentPidSetError(PidObject* pid, const float error)
{
  pid->error = error;
}

/**
 * Set a new set point for the PID to track.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] angle The new set point
 */
void studentPidSetDesired(PidObject* pid, const float desired)
{
  pid->setpoint = desired;
}

/**
 * Get the current desired setpoint
 * 
 * @param[in] pid  A pointer to the pid object.
 * @return The set point
 */
float studentPidGetDesired(PidObject* pid)
{
  return pid->setpoint;
}


/**
 * Find out if PID is active
 * @return TRUE if active, FALSE otherwise
 */
bool studentPidIsActive(PidObject* pid)
{
  return fabsf(pid->kp) > FLT_EPSILON ||
         fabsf(pid->ki) > FLT_EPSILON ||
         fabsf(pid->kd) > FLT_EPSILON;
}

/**
 * Set a new proportional gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kp    The new proportional gain
 */
void studentPidSetKp(PidObject* pid, const float kp)
{
  pid->kp = kp;
}

/**
 * Set a new integral gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] ki    The new integral gain
 */
void studentPidSetKi(PidObject* pid, const float ki)
{
  pid->ki = ki;
}

/**
 * Set a new derivative gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kd    The derivative gain
 */
void studentPidSetKd(PidObject* pid, const float kd)
{
  pid->kd = kd;
}

/**
 * Set a new dt gain for the PID. Defaults to IMU_UPDATE_DT upon construction
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] dt    Delta time
 */
void studentPidSetDt(PidObject* pid, const float dt) {
  pid->dt = dt;
}