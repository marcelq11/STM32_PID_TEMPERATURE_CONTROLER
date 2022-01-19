/*
 * PID_discrete_simple.h
 *
 *  Created on: Jan 19, 2022
 *      Author: marcel
 */

#ifndef INC_PID_DISCRETE_SIMPLE_H_
#define INC_PID_DISCRETE_SIMPLE_H_

#include <stdio.h>
#include <stdint.h>

typedef float float32_t;

typedef struct{
	float32_t Kp;
	float32_t Ki;
	float32_t Kd;
	float32_t dt;
}pid_parameters_t;

typedef struct{
	pid_parameters_t p;
	float32_t previous_error, previous_integral;
}pid_data_t;

float32_t calculate_discrete_pid(pid_data_t* pid, float32_t setpoint, float32_t measured);




#endif /* INC_PID_DISCRETE_SIMPLE_H_ */
