/**
 * @author Dominik Luczak
 */
/**
* Calculation of discrete PID
* @param[in,out] s A pointer to PID parameters and history
* @param[in] setpoint Input setpoint value
* @param[in] measured Input measured value
* @return PID output value
*/
#include "PID_discrete_simple.h"

float32_t calculate_discrete_pid(pid_data_t* pid, float32_t setpoint, float32_t measured){
	float32_t u=0, P, I, D, error, integral, derivative;
	
	error = setpoint-measured;
	
	//proportional part
	P = pid->p.Kp * error;

	//integral part
	integral = pid->previous_integral + (error+pid->previous_error) ; //numerical integrator without anti-windup
	pid->previous_integral = integral;
	I = pid->p.Ki*integral*(pid->p.dt/2.0);

	//derivative part
	derivative = (error - pid->previous_error)/pid->p.dt; //numerical derivative without filter
	pid->previous_error = error;
	D = pid->p.Kd*derivative;
	
	//sum of all parts
	u = P  + I + D; //without saturation
	
	return u;
}

  
