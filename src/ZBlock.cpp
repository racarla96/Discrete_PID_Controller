#include "ZBlock.h"
#include "Arduino.h"

ZBlock::ZBlock(float x0, float Ts, float K, ZBlockType type, ZBlockMethod method) {
  this->_uk = 0.0;
  this->_yk = 0.0;
  this->_xk = 0.0;
  this->_xk1 = x0;
  this->_x0 = x0;
  this->_Ts = Ts;
  this->_K = K;
  this->_type = type;
  this->_method = method;
}

float ZBlock::forwardStep(float uk) {
  float yk;
  switch (this->_type) {
    case ZBlockType::Z_TYPE_INT:
      yk = forwardStepIntegration(uk);
      break;
    case ZBlockType::Z_TYPE_DER:
      yk = forwardStepDerivative(uk);
      break;
    case ZBlockType::Z_TYPE_FIRSTORDER:
      yk = forwardStepFirstOrder(uk);
      break;
    default:
      yk = 0.0;
      break;
  }
  return yk;
}

void ZBlock::backStep() {
  this->_xk1 = this->_xk;
}

float ZBlock::forwardStepIntegration(float uk) {
  this->_uk = uk;
  this->_xk = this->_xk1;

  switch (this->_method) {
    case ZBlockMethod::Z_METHOD_FE:
      this->_yk = this->_xk;
      this->_xk1 = this->_xk + this->_K * this->_Ts * this->_uk;
      Serial.println(uk);
      break;
    case ZBlockMethod::Z_METHOD_BE:
      this->_yk = this->_xk + this->_K * this->_Ts * this->_uk;
      this->_xk1 = this->_yk;
      break;
    case ZBlockMethod::Z_METHOD_TRAP:
      this->_yk = this->_xk + this->_K * this->_Ts / 2.0 * this->_uk;
      this->_xk1 = this->_yk + this->_K * this->_Ts / 2.0 * this->_uk;
      break;
    default:
      this->_yk = this->_uk;
      this->_xk1 = this->_xk;
      break;
  }
  return this->_yk;
}

float ZBlock::forwardStepDerivative(float uk) {
  this->_uk = uk;
  this->_xk = this->_xk1;

  // K here is the derivative filter gain, N (rdefined for clarity)
  double N = this->_K;

  switch (this->_method) {
    case ZBlockMethod::Z_METHOD_STD:
      this->_yk = (1.0 / this->_Ts) * this->_uk + this->_xk;
      this->_xk1 = (-1.0 / this->_Ts) * this->_uk;
      break;
    case ZBlockMethod::Z_METHOD_FE:
      this->_yk = this->_xk + N * this->_uk;
      this->_xk1 = (1.0 - N * this->_Ts) * this->_yk - N * this->_uk;
      break;
    case ZBlockMethod::Z_METHOD_BE:
      this->_yk = N / (1.0 + N * this->_Ts) * this->_uk + this->_xk;
      this->_xk1 = (this->_yk - N * this->_uk) / (1.0 + N * this->_Ts);
      break;
    case ZBlockMethod::Z_METHOD_TRAP:
      this->_yk = 2.0 * N / (2.0 + N * this->_Ts) * this->_uk + this->_xk;
      this->_xk1 = ((1.0 - N * this->_Ts / 2.0) * this->_yk - N * this->_uk) / (1.0 + N * this->_Ts / 2.0);
      break;
    default:
      this->_yk = this->_uk;
      this->_xk1 = this->_xk;
      break;
  }
  return this->_yk;
}

float ZBlock::forwardStepFirstOrder(float uk) {
  this->_uk = uk;
  this->_xk = this->_xk1;

  // K here is a first order time constant, tau (redefined for clarity)
  double tau = this->_K;

  this->_yk = this->_Ts / (tau + this->_Ts) * this->_uk + this->_xk;
  this->_xk1 = tau / (tau + this->_Ts) * this->_yk;

  return this->_yk;
}
