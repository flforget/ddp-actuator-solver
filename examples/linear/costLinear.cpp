#include "costLinear.hh"

CostLinear::CostLinear()
{
  Q <<  1.0, 0.0,
        0.0, 0.0;
  R << 0.1 * 0.01;

  lxx = Q;
  luu = R;
  lux << 0.0, 0.0;
  lxu << 0.0, 0.0;
  lx.setZero();
  final_cost = 0;
  running_cost = 0;
}

void CostLinear::computeCostAndDeriv(const stateVec_t& X,
                                     const stateVec_t& Xdes, const commandVec_t& U)
{
  running_cost = ((X - Xdes).transpose() * Q * (X - Xdes) + U.transpose() * R * U)
                 (0, 0);
  lx = Q * (X - Xdes);
  lu = R * U;
}

void CostLinear::computeFinalCostAndDeriv(const stateVec_t& X,
    const stateVec_t& Xdes)
{
  final_cost = ((X - Xdes).transpose() * 1.0 * Q * (X - Xdes))(0, 0);
  lx = 1.0 * Q * (X - Xdes);
  lxx = 1.0 * Q;
}
