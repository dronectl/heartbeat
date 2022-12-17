/**
 * @file pid.h
 * @brief PID controller datastructures and definitions
 * @version 0.1
 * @date 2022-12
 * 
 * @copyright Copyright (c) 2022-2022 Alouette (Skylark)
 * 
 */

#ifndef __PID_H__
#define __PID_H__

#define PID_DEFAULT_KP 2.0f
#define PID_DEFAULT_KI 0.5f
#define PID_DEFAULT_KD 0.25f
#define PID_DEFAULT_TAU 0.02f
#define PID_DEFAULT_INT_CLAMP_MIN -5.0f
#define PID_DEFAULT_INT_CLAMP_MAX 5.0f
#define PID_DEFAULT_OUTPUT_CLAMP_MIN -10.0f
#define PID_DEFAULT_OUTPUT_CLAMP_MAX 10.0f
#define PID_DEFAULT_FREQ 0.01f

/**
 * @brief PID clamping parameters
 * 
 */
typedef struct pid_clamp_t {
  float min; // clamp minimum
  float max; // clamp maximum
} pid_clamp_t;

/**
 * @brief Persistant pid memory parameters. These values represent previously computed values.
 * 
 */
typedef struct pid_mem_t {
  float error; // error 
  float integrator; // integrator term
  float differentiator; // differentiator term
  float measurement; // measurement
} pid_mem_t;

/**
 * @brief PID coefficient struct
 * 
 */
typedef struct pid_t {
  float kp; // proportional gain 
  float ki; // integral gain
  float kd; // derivative gain
  float tau; // derivative low-pass filter time constant
  float time; // sample time in seconds
  float output; // controller output
  pid_clamp_t out_clamp; // output clamp
  pid_clamp_t int_clamp; // integrator clamp
  pid_mem_t memory; // persistant pid memory parameters
} pid_t;

void pid_init(pid_t *pid);
float pid_update(pid_t *pid, float setpoint, float measurement);

#endif // __PID_H__
