#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "SearchTools.h"
#include "debug.h"
#include <map>

/**
    Tools for evaluating states and state cost
*/
int * TransformIndex(int x, int y, int z, state6D * state);

state6D * EvaluateState(state6D * new_state, state6D * goal_state, float ParetoWeight, float (*HeuristicFn)(state6D * stateToEval, state6D * goalState, int debug_level), int debug_level);

std::map<float, uint32_t> EvaluateStateOverlap(state6D * new_state, int debug_level);

float LookupCostCalc(dVoxelArray * Environment, int iX, int iY, int iZ, float iDeform, float iCost, int debug_level);
