#include "stdlib.h"
#include "stdio.h"
#include <string>
#include "math.h"
#include "deformable_astar/IrosPlanner.h"

#define _USE_MATH_DEFINES
#define UNUSED(x) (void)(x)

using namespace std;

float IrosDistanceHeuristic3D(state6D * stateToEval, state6D * goalState, int debug_level)
{
    UNUSED(debug_level);
    float deltaX = goalState->XT - stateToEval->XT;
    float deltaY = goalState->YT - stateToEval->YT;
    float deltaZ = goalState->ZT - stateToEval->ZT;
    float distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2));
    return distance;
}

StateList * IrosGenerateChildren3D(state6D * state, int debug_level)
{
    debug(debug_level, 2, "Generating children...\n");
    if (state == NULL)
    {
        debug(debug_level, 1, "\033[91m *** Passed a null state, unable to generate children! *** \033[0m\n");
        fflush(stdout);
        return AllocateStateList();
    }
    StateList * children = AllocateStateList();
    float Xmax = (float)state->Environment->xDim;
    float Ymax = (float)state->Environment->yDim;
    float Zmax = (float)state->Environment->zDim;
    //float k = state->ZT;
    for (float i = (state->XT - 1.0); i <= (state->XT + 1.0); i+=1.0)
    {
        for (float j = (state->YT - 1.0); j <= (state->YT + 1.0); j+=1.0)
        {
            for (float k = (state->ZT - 1.0); k <= (state->ZT + 1.0); k+=1.0)
            {
                //Check if we're within the acceptable bounds of the environment
                if (i >= 0.0 && j >= 0.0 && k >= 0.0 && i < Xmax && j < Ymax && k < Zmax)
                {
                    //Check to make sure we're not duplicating the parent
                    if (i != state->XT || j != state->YT || k != state->ZT)
                    {
                        float xdiff = i - state->XT;
                        float ydiff = j - state->YT;
                        float zdiff = k - state->ZT;
                        float diff = sqrt(pow(xdiff, 2) + pow(ydiff, 2) + pow(zdiff, 2));
                        state6D * childState = NewState6D(state, state->RefShape, state->Environment, i, j, k, 0.0, 0.0, 0.0, diff);
                        StateAppend(children, childState);
                    }
                }
            }
        }
    }
    debug(debug_level, 1, "Generated %d possible new children\n", children->Length);
    return children;
}

int IrosAstar3D(state6D * startState, state6D * goalState, float pareto_weight, char * path_save_name)
{
    //Control Params
    int control = 0;
    //Run an A* search
    printf("Running A* [hybrid 3D] search...\n");
    StateList * path = AstarSearch(startState, goalState, pareto_weight, IrosDistanceHeuristic3D, IrosGenerateChildren3D, control);
    if (path == NULL)
    {
        printf("A* [hybrid 3D] failed to plan a path!\n");
        return -1;
    }
    //Print the path to the console
    for (int index = 0; index < path->Length; index++)
    {
        state6D * PN = GetIndex(path, index);
        printf("Path state X:%f|Y:%f|Z:%f|Dist:%f|Deform:%f\n", PN->XT, PN->YT, PN->ZT, PN->DIST, PN->COST);
    }
    //Save the path planned to a CSV file
    printf("Writing the path to CSV...\n");
    std::string basepath("/home/calderpg/Dropbox/ROS_workspace/src/Research/deformable_planning/deformable_astar/data/");
    std::string csv_filepath = basepath + std::string(path_save_name) + std::string("_path.csv");
    FILE * path_file = fopen(csv_filepath.c_str(), "w");
    if (path_file == NULL)
    {
        printf("Unable to open CSV file!\n");
        fflush(stdout);
    }
    fprintf(path_file, "XT,YT,ZT,XR,YR,ZR,COST,DIST");
    for (int index = 0; index < path->Length; index++)
    {
        state6D * PN = GetIndex(path, index);
        fprintf(path_file, "%f,%f,%f,%f,%f,%f,%f,%f\n", PN->XT, PN->YT, PN->ZT, PN->XR, PN->YR, PN->ZR, PN->COST, PN->DIST);
    }
    //Flush & close file
    fclose(path_file);
    //Save a copy of the environment with manipulators at regular intervals on the planned path
    printf("Saving a copy of the planning environment...\n");
    dVoxelArray * savecopy = CloneDVoxelArray(startState->Environment);
    //Show the swept volume
    for (int index = 0; index < path->Length; index+=2)
    {
        state6D * PN = GetIndex(path, index);
        TransformMergeDVoxelArrays(savecopy, PN->RefShape, PN->XT, PN->YT, PN->ZT, 0, 0, 0, 100);
    }
    //Draw in start and goal states
    int sx = startState->XT;
    int sy = startState->YT;
    int sz = startState->ZT;
    int gx = goalState->XT;
    int gy = goalState->YT;
    int gz = goalState->ZT;
    savecopy->dVoxels[sx][sy][sz].R = 255;
    savecopy->dVoxels[sx][sy][sz].G = 0;
    savecopy->dVoxels[sx][sy][sz].B = 255;
    savecopy->dVoxels[sx][sy][sz].A = 255;
    savecopy->dVoxels[gx][gy][gz].R = 0;
    savecopy->dVoxels[gx][gy][gz].G = 255;
    savecopy->dVoxels[gx][gy][gz].B = 0;
    savecopy->dVoxels[gx][gy][gz].A = 255;
    std::string base_env_filepath = basepath + std::string(path_save_name) + std::string("_env.dvxl");
    std::string robot_filepath = basepath + std::string(path_save_name) + std::string("_robot.dvxl");
    std::string planned_filepath = basepath + std::string(path_save_name) + std::string("_planned.dvxl");
    WriteDVoxelArrayToFile(savecopy, (char*)planned_filepath.c_str());
    WriteDVoxelArrayToFile(startState->RefShape, (char*)robot_filepath.c_str());
    WriteDVoxelArrayToFile(startState->Environment, (char*)base_env_filepath.c_str());
    printf("Cleaning up...\n");
    int length = path->Length;
    DestroyStateList(path);
    DestroyDVoxelArray(savecopy);
    printf("A* [3D] search complete, returning the length of the path planned...\n");
    //Return the number of states in the planned path
    return length;
}
