/**
   zblock is module that handles simple discrete-time systems and their solutions in time.
   Integral, derivative, and first order systems are currently supported, with various
   solver options available (e.g. forward euler, backward euler, trapezoidal) as well as
   derivative filtering.
   Example usage:
        // create first order system
        ZBlock G = ZBlock(0.0, period_s, tau, ZBlockType::Z_TYPE_FIRSTORDER, ZBlockMethod::Z_METHOD_STD);

        // update
        float K; // Proporcional Constant of the plant G
        output = G.forwardStep(input*K);
*/

#ifndef ZBLOCK_H
#define ZBLOCK_H

/* enums for block type and solver method */
/**
   Z_TYPE_INT: Integral.
   Z_TYPE_DER: Derivative.
   Z_TYPE_FIRSTORDER: First order system.
*/
enum class ZBlockType { Z_TYPE_INT, Z_TYPE_DER, Z_TYPE_FIRSTORDER};
/**
   Z_METHOD_STD: Standard, used for a dervative block type only
   Z_METHOD_FE: Forward Euler. Best for small sampling times. Large sampling times can lead to instability.
   Z_METHOD_BE: Backward Euler. Guarenteed stability if corresponding continous-time system is stable.
   Z_METHOD_TRAP: Trapezoidal. Closet match in the frequency domain to the corresponding continuous-time
 				system. Guarenteed stability if corresponding continous-time system is stable.
*/
enum class ZBlockMethod { Z_METHOD_STD, Z_METHOD_FE, Z_METHOD_BE, Z_METHOD_TRAP};

class ZBlock {
  public:
    /**
      \brief        Class constructor, creates a new block given set of parameters.
      \param[in]    x0      Initial condition for internal state.
      \param[in]    Ts      Sample time in [sec].
      \param[in]    K       Gain.
                                For an integral block, this is a multiplier on the output
    	                        For a derivative block, this is a filtering parameter.
    		                        As N -> inf, behavior approaches ideal unfiltered derivative.
    	                        For a first order block, this is a time constant in [sec].
      \param[in]    type    The windup upper limit action control restriction.
      \param[in]    method  The execution period of the controller (in seconds).
      \Details

    */
    ZBlock(float x0, float Ts, float K, ZBlockType type, ZBlockMethod method);
    /**
      \brief        Executes the steps discrete system forward in time, given an input.
      \param[in]    uk      Input to system at time current time step k.
      \return       output  Output at current time step k.
      \Details

    */
    float forwardStep(float uk);
    /**
      \brief        Executes the steps discrete system backwards in time.
      \return       None.
      \Details

    */
    void backStep();
  private:
    float _uk;  // input, time k
    float _yk;  // output, time k
    float _xk;  // state, time k
    float _xk1; // state, time k+1
    float _x0;  // intial state, time 0
    float _Ts;  // sample time [s]
    float _K;   // gain
    ZBlockType _type;     // block type (Z_TYPE_)
    ZBlockMethod _method; // discrete step method (Z_METHOD_)

    float forwardStepIntegration(float uk);
    float forwardStepDerivative(float uk);
    float forwardStepFirstOrder(float uk);
};

#endif /* ZBLOCK_H */
