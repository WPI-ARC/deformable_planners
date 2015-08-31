#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "SearchFramework.h"

StateList * AstarSearch(state6D * startState, state6D * goalState, float ParetoWeight, float (*HeuristicFn)(state6D * stateToEval, state6D * goalState, int debug_level), StateList * (*GenerateChildren)(state6D * state, int debug_level), int ctrl);

StateList * BuildPath(state6D * top_state);
