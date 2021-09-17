/**
 * zpid is a simple module written in C for performing discrete PID control loop calculations.
 * 
 * Dependencies:
	- "zblock.h", a lower-level module handling discrete-time calculations
 */

#ifndef ZPID_H
#define ZPID_H

#include "ZBlock.h"

#define BACKPROP_LOOP_TOL 0.001
#define BACKPROP_MAX_ITERS 1000

class ZPID{
    public:
        /**
        * \brief        Class constructor, creates a new controller given a set of parameters.
        * \param[in]    Kp      Proportional gain.
        * \param[in]    Ki      Integral gain.
        * \param[in]    Kd      Derivative gain.
        * \param[in]    Kb      Back-propogation gain (integral anti-windup).
        *                       Kb > 0 - Back-propogation gain.
        *                       Kb <= 0 - Cut-Off - integral anti-windup.
        * \param[in]    N       Derivative filter parameter.
        * \param[in]    Ts      Sample time [seconds].
        * \param[in]    out_min     Output minimum (saturation block).
        * \param[in]    out_max     Output maximum (saturation block).
        * \param[in]    int_init    Integral calculation initial condition.
        * \param[in]    deriv_init  Derivative calculation initial condition.
        * \param[in]    int_method    Integral solver method.
        * \param[in]    deriv_method  Derivative solver method.
        * \Details
        * 
        */
        ZPID(float Kp, float Ki, float Kd, float Kb, float Nd, float Ts, float out_min, float out_max, float int_init, float deriv_init, ZBlockMethod int_method, ZBlockMethod deriv_method);
        /**
        * \brief        Steps the controller forward in time, given a target value and actual value
        * \param[in]    setpoint    Setpoint value for controller.
        * \param[in]    input       Actual value, typically from feedback.
        * \return       output      The control signal.
        * \Details
        * 
        */
       float update(float setpoint, float input);
        /**
        * \brief        Reset the PID Controller to the initial state.
        * \return       None.
        * \Details
        * 
        */
       //void reset();

    private:
        float _Kp;						// proportional gain
        float _Ki;						// integral gain
        float _Kd;						// derivative gain
        float _Kb;						// back-propogation gain (integral anti-windup)
        float _Nd;						// derivative filter parameter
        float _Ts;						// sample time [seconds]
        float _out_min;				    // output minimum (saturation block)
        float _out_max;				    // output maximum (saturation block)
        float _int_init;				// integral calculation initial condition
        float _deriv_init;				// derivative calculation initial condition
        ZBlockMethod _int_method;		// integral solver method
        ZBlockMethod _deriv_method;		// derivative solver method

        bool _back_propagation_anti_windup;

        long _ticks;

        float _setpoint;
        float _input;
        float _error;
        float _P_out;
        float _I_out;
        float _D_out;
        float _B_out;
        float _int_out;
        float _deriv_out;
        float _sum_I;
        float _sum_sat;
        float _sat_in;
        float _output;
        ZBlock *_integrator;
        ZBlock *_derivative;

        /**
        * \brief        Steps the controller forward in time, given a target value and actual value
        * \param[in]    setpoint    Setpoint value for controller.
        * \param[in]    input       Actual value, typically from feedback.
        * \return       output      The control signal.
        * \Details
        * Back propagation algorithm for anti-windup
        */
       float update_back_progation(float setpoint, float input);
               /**
        * \brief        Steps the controller forward in time, given a target value and actual value
        * \param[in]    setpoint    Setpoint value for controller.
        * \param[in]    input       Actual value, typically from feedback.
        * \return       output      The control signal.
        * \Details
        * Cut-Off algorithm for anti-windup
        */
       float update_cut_off(float setpoint, float input);

        /**
        * \brief        Saturate the signal introduced on this function.
        * \param[in]    input       Actual value of control signal.
        * \return       output      The control signal.
        * \Details
        * 
        */
       float saturate(float input);
};

#endif /* ZPID_H */
