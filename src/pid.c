/**
 * @file pid.c
 * @brief Test PID control algorithm featuring a band-limited differentiator and dynamic integrator
 * clamping.
 * @version 0.1
 * @date 2022-12
 * 
 * @copyright Copyright (c) 2022-2022 Alouette (Skylark)
 * 
 */

#include "pid.h"

/**
 * @brief Initialize pid parameters
 * 
 * @param pid 
 */
void pid_init(pid_t *pid) {
  // reset pid gains
  pid->ki = PID_DEFAULT_KI;
  pid->kd = PID_DEFAULT_KD;
  pid->kp = PID_DEFAULT_KP;
  // reset memory
  pid->memory.error = 0.0f;
  pid->memory.integrator = 0.0f;
  pid->memory.measurement = 0.0f;
  pid->memory.differentiator = 0.0f;
  // reset clamps
  pid->int_clamp.max = PID_DEFAULT_INT_CLAMP_MAX;
  pid->int_clamp.min = PID_DEFAULT_INT_CLAMP_MIN;
  pid->out_clamp.max = PID_DEFAULT_OUTPUT_CLAMP_MAX;
  pid->out_clamp.min = PID_DEFAULT_OUTPUT_CLAMP_MIN;
  // reset sequence params
  pid->tau = PID_DEFAULT_TAU;
  pid->time = PID_DEFAULT_FREQ;
  pid->output = 0.0f;
}

/**
 * @brief Compute control signal on discrete time scale using Tustin Transform.
 * The derivative branch implements a low pass filter to circumvent high frequency noise
 * The integration branch uses dynamic clamping based on proportional branch output to avoid 
 * saturation
 * 
 * @param pid pid model
 * @param setpoint target setpoint
 * @param measurement sensor feedback (input)
 * @return float 
 */
float pid_update(pid_t *pid, float setpoint, float measurement) {
  // initialize instantaneous parameters
  float error = 0.0f;
  float proportional = 0.0f;
  float differentiator = 0.0f;
  float integrator = 0.0f;
  float lim_max_int = 0.0f;
  float lim_min_int = 0.0f;

  // compute error signal
  error = setpoint - measurement;
  // compute proportional branch
  proportional = pid->kp * error;
  // compute integration branch
  integrator = pid->memory.integrator + 0.5f * pid->ki * pid->time * (error + pid->memory.error);

  // dynamic integrator clamping
  // compute integrator limits
  lim_max_int = pid->int_clamp.max > proportional ? pid->int_clamp.max - proportional : 0.0f;
  lim_min_int = pid->int_clamp.min < proportional ? pid->int_clamp.min - proportional : 0.0f;
  // clamp integrator
  if (integrator > lim_max_int) {
    integrator = lim_max_int;
  } else if (integrator < lim_min_int) {
    integrator = lim_min_int;
  }

  // derivative (band-limited differentiator)
  differentiator = (2.0f * pid->kd * (measurement - pid->memory.measurement)
                             + (2.0f * pid->tau - pid->time) * pid->memory.differentiator)
                             / (2.0f * pid->tau + pid->time);

  // compute output and apply limits
  pid->output = proportional + integrator + differentiator;
  if (pid->output > pid->out_clamp.max) {
    pid->output = pid->out_clamp.max;
  } else if (pid->output < pid->out_clamp.min) {
    pid->output = pid->out_clamp.min;
  }

  // store persistant data
  pid->memory.differentiator = differentiator;
  pid->memory.integrator = integrator;
  pid->memory.measurement = measurement;
  pid->memory.error = error;
  // update integration limits
  pid->int_clamp.max = lim_max_int;
  pid->int_clamp.min = lim_min_int;

  // return controller output
  return pid->output;
}
