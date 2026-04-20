
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "student_attitude_controller.h"
#include "sensfusion6.h"
#include "controller_student.h"

#include "log.h"
#include "debug.h"

#include "param.h"
#include "math3d.h"

//delta time between calls to the update function
#define STUDENT_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

//desired vehicle state as calculated by PID controllers
static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float thrustDesired;

//variables used only for logging PID command outputs
static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

//Dummy variables for test stand
static float angle;
static float rate;
static bool yawWasRateMode;
static bool rollWasRateMode;  
static bool pitchWasRateMode;

void controllerStudentInit(void)
{
  studentAttitudeControllerInit(STUDENT_UPDATE_DT);
}

bool controllerStudentTest(void)
{
  bool pass = true;
  //controller passes check if attitude controller passes
  pass &= studentAttitudeControllerTest();

  return pass;
}


/**
 * Limit the input angle between -180 and 180
 * 
 * @param angle 
 * @return float 
 */
static float capAngle(float angle) {
  while (angle > 180.0f) {
    angle -= 360.0f;
  }
  while (angle < -180.0f) {
    angle += 360.0f;
  }
  return angle;
}


/**
 * This function is called periodically to update the PID loop,
 * Reads state estimate and setpoint values and passes them
 * to the functions that perform PID calculations,
 * attitude PID and attitude rate PID
 * 
 * @param control Output, struct is modified as the ouput of the control loop
 * @param setpoint Input, setpoints for thrust, attitude, position, velocity etc. of the quad
 * @param sensors Input, Raw sensor values (typically want to use the state estimated instead) includes gyro, 
 * accelerometer, barometer, magnatometer 
 * @param state Input, A more robust way to measure the current state of the quad, allows for direct
 * measurements of the orientation of the quad. Includes attitude, position, velocity,
 * and acceleration
 * @param tick Input, system clock
 */
void controllerStudent(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick)
{

  // Main Controller Function

  // check if time to update the attutide controller
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {

    //only support attitude and attitude rate control
    if(setpoint->mode.x != modeDisable || setpoint->mode.y != modeDisable || setpoint->mode.z != modeDisable){
      DEBUG_PRINT("Student controller does not support vehicle position or velocity mode. Check flight mode.");
      control->thrust = 0;
      control->roll = 0;
      control->pitch = 0;
      control->yaw = 0;
      return;
    }

    if (setpoint->mode.yaw == modeVelocity) {
      if (!yawWasRateMode) {
        attitudeDesired.yaw = capAngle(state->attitude.yaw);
      }
      attitudeDesired.yaw = capAngle(attitudeDesired.yaw + setpoint->attitudeRate.yaw * STUDENT_UPDATE_DT);
    } else {
      attitudeDesired.yaw = capAngle(setpoint->attitude.yaw);
    }
    yawWasRateMode = (setpoint->mode.yaw == modeVelocity);

    attitudeDesired.roll = capAngle(setpoint->attitude.roll);
    attitudeDesired.pitch = capAngle(setpoint->attitude.pitch);
    thrustDesired = setpoint->thrust;

    studentAttitudeControllerCorrectAttitudePID(
      state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
      attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
      &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      studentAttitudeControllerResetRollAttitudePID();
    }

    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      studentAttitudeControllerResetPitchAttitudePID();
    }

    if (setpoint->mode.yaw == modeVelocity) {
      rateDesired.yaw = setpoint->attitudeRate.yaw;
      // yawWasRateMode is already false on first entry, so reset fires exactly once
    }

    studentAttitudeControllerCorrectRatePID(
      sensors->gyro.x, sensors->gyro.y, sensors->gyro.z,
      rateDesired.roll, rateDesired.pitch, rateDesired.yaw,
      &control->roll, &control->pitch, &control->yaw);
  }

  control->thrust = thrustDesired;

  if (control->thrust <= 0.0f) {
    control->thrust = 0.0f;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    thrustDesired = 0.0f;
    yawWasRateMode = false;
    rollWasRateMode  = false;
    pitchWasRateMode = false;
    studentAttitudeControllerResetAllPID();
  }


  //copy values for logging
  cmd_thrust = control->thrust;
  cmd_roll = control->roll;
  cmd_pitch = control->pitch;
  cmd_yaw = control->yaw;
  r_roll = sensors->gyro.x;
  r_pitch = sensors->gyro.y;
  r_yaw = sensors->gyro.z;
  accelz = sensors->acc.z;
}

/**
 * Logging variables for the command and reference signals for the
 * student PID controller
 */
LOG_GROUP_START(ctrlStdnt)

/**
 * @brief Thrust command output
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command output
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command output
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command output
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in degrees
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in degrees
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Gyro yaw rate measurement in degrees
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the z axis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll, &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch, &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw, &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate, &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate, &rateDesired.yaw)

LOG_GROUP_STOP(ctrlStdnt)

/**
 *  Test Stand Logging Variables
 */

LOG_GROUP_START(Test_Stand)

/**
 * @brief Test_Stand.angle
 */
LOG_ADD(LOG_FLOAT, angle, &angle)
/**
 * @brief Test_Stand.rate
 */
LOG_ADD(LOG_FLOAT, rate, &rate)

LOG_GROUP_STOP(Test_Stand)



/**
 * Controller parameters
 */
PARAM_GROUP_START(ctrlStdnt)

PARAM_ADD(PARAM_FLOAT, placeHolder, &angle)

PARAM_GROUP_STOP(ctrlStdnt)