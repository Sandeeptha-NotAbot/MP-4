/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * student_attitude_pid_controller.c: Attitude controller using PID correctors
 */
#include <stdbool.h>

#include "FreeRTOS.h"

#include "student_attitude_controller.h"
#include "student_pid.h"
#include "param.h"
#include "log.h"

//low pass filter settings
#define ATTITUDE_LPF_CUTOFF_FREQ      15.0f
#define ATTITUDE_LPF_ENABLE      false
#define ATTITUDE_RATE_LPF_CUTOFF_FREQ 30.0f
#define ATTITUDE_RATE_LPF_ENABLE false

/**
 * @brief Convert float to 16 bit integer
 * Use this for converting final value to store in the control struct
 * 
 * @param in float
 * @return int16_t 
 */
static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

static PidObject pidRollRate;
static PidObject pidPitchRate;
static PidObject pidYawRate;
static PidObject pidRoll;
static PidObject pidPitch;
static PidObject pidYaw;

static bool isInit;

/**
 * @brief Initialize all PID data structures with PID coefficients defined in student_pid.h
 * 
 * @param updateDt expected delta time since last call for all PID loops
 */
void studentAttitudeControllerInit(const float updateDt)
{
  if(isInit)
    return;

  const float samplingRate = 1.0f / updateDt;

  studentPidInit(&pidRollRate, 0.0f, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD,
                 updateDt, samplingRate, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  studentPidInit(&pidPitchRate, 0.0f, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD,
                 updateDt, samplingRate, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  studentPidInit(&pidYawRate, 0.0f, PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD,
                 updateDt, samplingRate, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);

  studentPidSetIntegralLimit(&pidRollRate, PID_ROLL_RATE_INTEGRATION_LIMIT);
  studentPidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  studentPidSetIntegralLimit(&pidYawRate, PID_YAW_RATE_INTEGRATION_LIMIT);

  studentPidInit(&pidRoll, 0.0f, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD,
                 updateDt, samplingRate, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
  studentPidInit(&pidPitch, 0.0f, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD,
                 updateDt, samplingRate, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
  studentPidInit(&pidYaw, 0.0f, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD,
                 updateDt, samplingRate, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);

  studentPidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRATION_LIMIT);
  studentPidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
  studentPidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRATION_LIMIT);

  isInit = true;
}

/**
 * @brief Simple test to make sure controller is initialized
 * 
 * @return true/false
 */
bool studentAttitudeControllerTest()
{
  return isInit;
}

/**
 * Make the controller run an update of the attitude PID. The output is
 * the desired rate which should be fed into a rate controller. The
 * attitude controller can be run in a slower update rate then the rate
 * controller.
 * 
 * @param eulerRollActual input
 * @param eulerPitchActual input
 * @param eulerYawActual input
 * @param eulerRollDesired input
 * @param eulerPitchDesired input
 * @param eulerYawDesired input
 * @param rollRateDesired output
 * @param pitchRateDesired output
 * @param yawRateDesired output
 */
void studentAttitudeControllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
  studentPidSetDesired(&pidRoll, eulerRollDesired);
  studentPidSetDesired(&pidPitch, eulerPitchDesired);
  studentPidSetDesired(&pidYaw, eulerYawDesired);

  *rollRateDesired = studentPidUpdate(&pidRoll, eulerRollActual, true);
  *pitchRateDesired = studentPidUpdate(&pidPitch, eulerPitchActual, true);

  float yawError = eulerYawDesired - eulerYawActual;
  while (yawError > 180.0f) {
    yawError -= 360.0f;
  }
  while (yawError < -180.0f) {
    yawError += 360.0f;
  }

  studentPidSetError(&pidYaw, yawError);
  *yawRateDesired = studentPidUpdate(&pidYaw, eulerYawActual, false);
}

/**
 * Make the controller run an update of the rate PID. Input comes from the 
 * correct attitude function. The output is the actuator force. 
 *  * 
 * @param rollRateActual input
 * @param pitchRateActual input
 * @param yawRateActual input
 * @param rollRateDesired input
 * @param pitchRateDesired input
 * @param yawRateDesired input
 * @param rollCmd output
 * @param pitchCmd output
 * @param yawCmd
 */
void studentAttitudeControllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired,
       int16_t* rollCmd, int16_t* pitchCmd, int16_t* yawCmd
       )
{
  studentPidSetDesired(&pidRollRate, rollRateDesired);
  studentPidSetDesired(&pidPitchRate, pitchRateDesired);
  studentPidSetDesired(&pidYawRate, yawRateDesired);

  *rollCmd = saturateSignedInt16(studentPidUpdate(&pidRollRate, rollRateActual, true));
  *pitchCmd = saturateSignedInt16(studentPidUpdate(&pidPitchRate, pitchRateActual, true));
  *yawCmd = saturateSignedInt16(studentPidUpdate(&pidYawRate, yawRateActual, true));

}

void studentAttitudeControllerResetRollAttitudePID(void)
{
  studentPidReset(&pidRoll);
}

void studentAttitudeControllerResetYawAttitudePID(void)
{
  studentPidReset(&pidYaw);
}

void studentAttitudeControllerResetPitchAttitudePID(void)
{
  studentPidReset(&pidPitch);
}

void studentAttitudeControllerResetAllPID(void)
{
  studentAttitudeControllerResetRollAttitudePID();
  studentAttitudeControllerResetPitchAttitudePID();
  studentAttitudeControllerResetYawAttitudePID();
  studentPidReset(&pidRollRate);
  studentPidReset(&pidPitchRate);
  studentPidReset(&pidYawRate);
}

/**
 *  Log variables of attitude PID controller
 */ 
LOG_GROUP_START(s_pid_attitude)
/**
 * @brief Proportional output roll
 */
LOG_ADD(LOG_FLOAT, roll_outP, &pidRoll.outP)
/**
 * @brief Integral output roll
 */
LOG_ADD(LOG_FLOAT, roll_outI, &pidRoll.outI)
/**
 * @brief Derivative output roll
 */
LOG_ADD(LOG_FLOAT, roll_outD, &pidRoll.outD)
/**
 * @brief Proportional output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitch.outP)
/**
 * @brief Integral output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitch.outI)
/**
 * @brief Derivative output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitch.outD)
/**
 * @brief Proportional output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outP, &pidYaw.outP)
/**
 * @brief Intergal output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outI, &pidYaw.outI)
/**
 * @brief Derivative output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outD, &pidYaw.outD)
LOG_GROUP_STOP(s_pid_attitude)

/**
 *  Log variables of attitude rate PID controller
 */
LOG_GROUP_START(s_pid_rate)
/**
 * @brief Proportional output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outP, &pidRollRate.outP)
/**
 * @brief Integral output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outI, &pidRollRate.outI)
/**
 * @brief Derivative output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outD, &pidRollRate.outD)
/**
 * @brief Proportional output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitchRate.outP)
/**
 * @brief Integral output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitchRate.outI)
/**
 * @brief Derivative output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitchRate.outD)
/**
 * @brief Proportional output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outP, &pidYawRate.outP)
/**
 * @brief Integral output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outI, &pidYawRate.outI)
/**
 * @brief Derivative output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outD, &pidYawRate.outD)
LOG_GROUP_STOP(s_pid_rate)

/**
 * Tuning settings for the gains of the PID
 * controller for the attitude of the Crazyflie which consists
 * of the Yaw Pitch and Roll 
 */
PARAM_GROUP_START(s_pid_attitude)
/**
 * @brief Proportional gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRoll.kp)
/**
 * @brief Integral gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRoll.ki)
/**
 * @brief Derivative gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRoll.kd)
/**
 * @brief Proportional gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitch.kp)
/**
 * @brief Integral gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitch.ki)
/**
 * @brief Derivative gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitch.kd)
/**
 * @brief Proportional gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYaw.kp)
/**
 * @brief Integral gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYaw.ki)
/**
 * @brief Derivative gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYaw.kd)
PARAM_GROUP_STOP(s_pid_attitude)

/**
 * Tuning settings for the gains of the PID controller for the rate angles of
 * the Crazyflie, which consists of the yaw, pitch and roll rates 
 */
PARAM_GROUP_START(s_pid_rate)
/**
 * @brief Proportional gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRollRate.kp)
/**
 * @brief Integral gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRollRate.ki)
/**
 * @brief Derivative gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRollRate.kd)
/**
 * @brief Proportional gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitchRate.kp)
/**
 * @brief Integral gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitchRate.ki)
/**
 * @brief Derivative gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitchRate.kd)
/**
 * @brief Proportional gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYawRate.kp)
/**
 * @brief Integral gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYawRate.ki)
/**
 * @brief Derivative gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYawRate.kd)
PARAM_GROUP_STOP(s_pid_rate)