#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "Astar.h"

float IrosDistanceHeuristic3D(state6D * stateToEval, state6D * goalState, int debug_level);

StateList * IrosGenerateChildren3D(state6D * state, int debug_level);

int IrosAstar3D(state6D * startState, state6D * goalState, float pareto_weight, char * path_save_name);
