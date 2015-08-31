#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "deformable_astar/SearchFramework.h"

//#define snap(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))
#define USE_PERFORMANCE_TIMERS

int * TransformIndex(int x, int y, int z, state6D * state)
{
    float Xcenter = state->XT;
    float Ycenter = state->YT;
    float Zcenter = state->ZT;
    float Xcurrent = Xcenter + (float)(x - state->RefShape->xCenter);
    float Ycurrent = Ycenter + (float)(y - state->RefShape->yCenter);
    float Zcurrent = Zcenter + (float)(z - state->RefShape->zCenter);
    double * transformed_point = RotatePoint(Xcurrent, Ycurrent, Zcurrent, state->XR, state->YR, state->ZR, Xcenter, Ycenter, Zcenter);
    int * new_coords = (int *)malloc(sizeof(int) * 3);
    //Snap the floating point values to the 3D grid
    new_coords[0] = snap(transformed_point[0]);
    new_coords[1] = snap(transformed_point[1]);
    new_coords[2] = snap(transformed_point[2]);
    //Get rid of stuff we don't need
    free(transformed_point);
    //printf("DEBUG - Transformed original index %d|%d|%d, relative %d|%d|%d, to %d|%d|%d\n", x,y,z,(int)Xcurrent,(int)Ycurrent,(int)Zcurrent,new_coords[0],new_coords[1],new_coords[2]);
    return new_coords;
}

std::map<float, u_int32_t> EvaluateStateOverlap(state6D * new_state, int debug_level)
{
#ifdef USE_PERFORMANCE_TIMERS
    struct timespec st, et;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &st);
#endif
    std::map<float, u_int32_t> sensivity_counts;
    for (int i = 0; i < new_state->RefShape->xDim; i++)
    {
        for (int j = 0; j < new_state->RefShape->yDim; j++)
        {
            for (int k = 0; k < new_state->RefShape->zDim; k++)
            {
                int * TransformedIndex = TransformIndex(i, j, k, new_state);
                // Get the sensitivity and deformability of the matching environment cell
                float environment_sensitvity = new_state->Environment->dVoxels[TransformedIndex[0]][TransformedIndex[1]][TransformedIndex[2]].sensitivity;
                //float environment_deformability = new_state->Environment->dVoxels[TransformedIndex[0]][TransformedIndex[1]][TransformedIndex[2]].deformability;
                if (environment_sensitvity != 0.0 && environment_sensitvity != -0.0)
                {
                    sensivity_counts[environment_sensitvity] += 1;
                }
                free(TransformedIndex);
            }
        }
    }
#ifdef USE_PERFORMANCE_TIMERS
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &et);
    float secs = (float)(et.tv_sec - st.tv_sec);
    secs = secs + (float)(et.tv_nsec - st.tv_nsec) / 1000000000.0;
    debug(debug_level, 2, "Overlap computation took %f seconds\n", secs);
#endif
    return sensivity_counts;
}

state6D * EvaluateState(state6D * new_state, state6D * goal_state, float ParetoWeight, float (*HeuristicFn)(state6D * stateToEval, state6D * goalState, int debug_level), int debug_level)
{
#ifdef USE_PERFORMANCE_TIMERS
    struct timespec st, et;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &st);
#endif
    float state_cost = 0.0;
    for (int i = 0; i < new_state->RefShape->xDim; i++)
    {
        for (int j = 0; j < new_state->RefShape->yDim; j++)
        {
            for (int k = 0; k < new_state->RefShape->zDim; k++)
            {
                int * TransformedIndex = TransformIndex(i, j, k, new_state);
                float index_sensitivity = new_state->RefShape->dVoxels[i][j][k].sensitivity;
                float index_deform = new_state->RefShape->dVoxels[i][j][k].deformability;
                state_cost = state_cost + LookupCostCalc(new_state->Environment, TransformedIndex[0], TransformedIndex[1], TransformedIndex[2], index_deform, index_sensitivity, debug_level);
                free(TransformedIndex);
            }
        }
    }
#ifdef USE_PERFORMANCE_TIMERS
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &et);
    float secs = (float)(et.tv_sec - st.tv_sec);
    secs = secs + (float)(et.tv_nsec - st.tv_nsec) / 1000000000.0;
    debug(debug_level, 2, "DVXL cost computation took %f seconds\n", secs);
#endif
    //You should probably know here that NAN "poisons" float math, so once NAN is added in, the output is always NAN
    //Also, more importantly, NAN != NAN according to the IEEE float specs, hence the use of isnan()
    debug(debug_level, 2, "Assessed state deformation cost: %f\n", state_cost);
    if (isnan(state_cost) || isnanf(state_cost))
    {
        debug(debug_level, 2, "Invalid state - returning NULL\n");
        return NULL;
    }
    state_cost = state_cost + new_state->COST;
    float heuristicVal = HeuristicFn(new_state, goal_state, debug_level);
    //float VAL = heuristicVal + (1.0 - ParetoWeight) * new_state->DIST + ParetoWeight * state_cost; - BAD
    //float VAL = heuristicVal + new_state->DIST + state_cost;
    float VAL = (1.0 - ParetoWeight) * (heuristicVal + new_state->DIST) + ParetoWeight * state_cost;
    //Set values in the state(float)Env->xDim
    new_state->COST = state_cost;
    new_state->VAL = VAL;
    return new_state;
}

float LookupCostCalc(dVoxelArray * Environment, int iX, int iY, int iZ, float iDeform, float iSensitivity, int debug_level)
{
    if (iX < 0 || iY < 0 || iZ < 0)
    {
        debug(debug_level, 3, "Rejecting index due to environment bounds [TOO LOW - %d|%d|%d]\n", iX, iY, iZ);
        return NAN;
    }
    else if (iX >= Environment->xDim || iY >= Environment->yDim || iZ >= Environment->zDim)
    {
        debug(debug_level, 3, "Rejecting index due to environment bounds [TOO HIGH - %d|%d|%d]\n", iX, iY, iZ);
        return NAN;
    }
    else
    {
        float env_deform = Environment->dVoxels[iX][iY][iZ].deformability;
        float env_sensitivity = Environment->dVoxels[iX][iY][iZ].sensitivity;
        if (env_deform == 0.0 && iDeform == 0.0)
        {
            debug(debug_level, 3, "Rejecting index due to impossible collision\n");
            return NAN;
        }
        else if (env_sensitivity == 0.0 || iSensitivity == 0.0)
        {
            return 0.0;
        }
        float real_env_cost = (env_deform / (env_deform + iDeform)) * env_sensitivity;
        float real_i_cost = (iDeform / (iDeform + env_deform)) * iSensitivity;
        return real_env_cost + real_i_cost;
    }
}
