#include <math.h>
#include "ZPID.h"

ZPID::ZPID(float Kp, float Ki, float Kd, float Kb, float Nd, float Ts, float out_min, float out_max, float int_init, float deriv_init, ZBlockMethod int_method, ZBlockMethod deriv_method)
{
    this->_Kp = Kp;
    this->_Ki = Ki;
    this->_Kd = Kd;
    this->_Kb = Kb;
    if(this->_Kb > 0) this->_back_propagation_anti_windup = true;
    else this->_back_propagation_anti_windup = false;
    this->_Nd = Nd;
    this->_Ts = Ts;
    this->_out_min = out_min;
    this->_out_max = out_max;
    this->_int_init = int_init;
    this->_deriv_init = deriv_init;
    this->_int_method = int_method;
    this->_deriv_method = deriv_method;

    this->_ticks = 0;
    this->_integrator = new ZBlock(this->_int_init, this->_Ts, 1.0, ZBlockType::Z_TYPE_INT, this->_int_method);
    this->_derivative = new ZBlock(this->_deriv_init, this->_Ts, this->_Nd, ZBlockType::Z_TYPE_DER, this->_deriv_method);
}

float ZPID::update(float setpoint, float input) 
{
    if(this->_back_propagation_anti_windup) return this->update_back_progation(setpoint, input);
    return this->update_cut_off(setpoint, input);
}

float ZPID::update_back_progation(float setpoint, float input)
{
	// error
	this->_setpoint = setpoint;
	this->_input = input;
	this->_error = this->_setpoint - this->_input;

	// PID gains
	this->_P_out = this->_Kp * this->_error;
	this->_I_out = this->_Ki * this->_error;
	this->_D_out = this->_Kd * this->_error;

	// derivative block
	this->_deriv_out = this->_derivative->forwardStep(this->_D_out);

	// integral with backpropagation
	// - the do-while loop is here to handle the algebraic loop caused by the back-propagation feature
	// - a hardcoded tolerance and maximum number of iterations are in place
	float _B_out_last;
	int num_backprop_iters = 0;
	do {
		_B_out_last = this->_B_out;

		// back step integrator after first pass
		if (num_backprop_iters > 0) 
            this->_integrator->backStep();

		// step forward
        this->_int_out = this->_integrator->forwardStep(this->_I_out + _B_out_last);

		// total controller output before saturation block
        this->_sat_in = this->_P_out + this->_int_out + this->_deriv_out;

		// saturation and back prop
        this->_output = this->saturate(this->_sat_in);
        this->_sum_sat = this->_output - this->_sat_in;
        this->_B_out = this->_Kb * this->_sum_sat;

		num_backprop_iters++;
	} while (fabs(this->_B_out - _B_out_last) > BACKPROP_LOOP_TOL && num_backprop_iters < BACKPROP_MAX_ITERS);

	this->_ticks++;

	return this->_output;    
}

float ZPID::update_cut_off(float setpoint, float input)
{
  // error
  this->_setpoint = setpoint;
  this->_input = input;
  this->_error = this->_setpoint - this->_input;

  // PID gains
  this->_P_out = this->_Kp * this->_error;
  this->_I_out = this->_Ki * this->_error;
  this->_D_out = this->_Kd * this->_error;

  // derivative block
  this->_deriv_out = this->_derivative->forwardStep(this->_D_out);

  // step forward
  this->_int_out = this->_integrator->forwardStep(this->_I_out + this->_B_out);
  
  // total controller output before saturation block
  this->_sat_in = this->_P_out + this->_int_out + this->_deriv_out;
  
  // saturation and back prop
  this->_output = this->saturate(this->_sat_in);
  this->_sum_sat = this->_output - this->_sat_in;

  this->_ticks++;

  return 0;    
}

float ZPID::saturate(float input)
{
    if(input < this->_out_min) return this->_out_min;
    if(input > this->_out_max) return this->_out_max;
    return input;
}
