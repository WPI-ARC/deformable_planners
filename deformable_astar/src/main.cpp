#include <iostream>
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "deformable_astar/IrosPlanner.h"

#define _USE_MATH_DEFINES

int OldIrosTest(float Xstart, float Ystart, float Zstart, float Xgoal, float Ygoal, float Zgoal, int options, char * path_save)
{
    //File names - Not used
    //Load environment
    printf("Building a test environment...\n");
    dVoxelArray * Environment = AllocateDVoxelArray(200, 200, 10, 100, 100, 5, 0.0, 1.0);
    //Add three walls in the middle of the environment
    for (int j = 0; j < 200; j++)
    {
        for (int k = 0; k < 10; k++)
        {
            Environment->dVoxels[50][j][k].sensitivity = 1.0;
            Environment->dVoxels[50][j][k].deformability = 0.0;
            Environment->dVoxels[50][j][k].A = 255;
            Environment->dVoxels[100][j][k].sensitivity = 1.0;
            Environment->dVoxels[100][j][k].deformability = 0.0;
            Environment->dVoxels[100][j][k].A = 255;
            Environment->dVoxels[150][j][k].sensitivity = 1.0;
            Environment->dVoxels[150][j][k].deformability = 0.0;
            Environment->dVoxels[150][j][k].A = 255;
        }
    }
    //Make a hole in the first wall
    for (int j = 15; j < 20; j++)
    {
        for (int k = 0; k < 10; k++)
        {
            Environment->dVoxels[50][j][k].sensitivity = Environment->defaultSensitivity;
            Environment->dVoxels[50][j][k].deformability = Environment->defaultDeformability;
            Environment->dVoxels[50][j][k].A = 0;
        }
    }
    //Make a hole in the second wall
    for (int j = 0; j < 10; j++)
    {
        for (int k = 0; k < 10; k++)
        {
            Environment->dVoxels[100][j][k].sensitivity = Environment->defaultSensitivity;
            Environment->dVoxels[100][j][k].deformability = Environment->defaultDeformability;
            Environment->dVoxels[100][j][k].A = 0;
        }
    }
    //One single hole in the third wall
    Environment->dVoxels[150][40][4].sensitivity = 0.0;
    Environment->dVoxels[150][40][4].deformability = 1.0;
    Environment->dVoxels[150][40][4].A = 0;
    Environment->dVoxels[150][41][4].sensitivity = 0.0;
    Environment->dVoxels[150][41][4].deformability = 1.0;
    Environment->dVoxels[150][41][4].A = 0;
    Environment->dVoxels[150][40][5].sensitivity = 0.0;
    Environment->dVoxels[150][40][5].deformability = 1.0;
    Environment->dVoxels[150][40][5].A = 0;
    Environment->dVoxels[150][41][5].sensitivity = 0.0;
    Environment->dVoxels[150][41][5].deformability = 1.0;
    Environment->dVoxels[150][41][5].A = 0;
    //Load reference manipulator
    printf("Building a test manipulator...\n");
    dVoxelArray * Manipulator = AllocateDVoxelArray(7, 7, 7, 3, 3, 3, 1.0, 0.5);
    //Color the manipulator red
    for (int i = 0; i < Manipulator->xDim; i++)
    {
        for (int j = 0; j < Manipulator->yDim; j++)
        {
            for (int k = 0; k < Manipulator->zDim; k++)
            {
                Manipulator->dVoxels[i][j][k].A = 128;
                Manipulator->dVoxels[i][j][k].R = 255;
            }
        }
    }
    //Set the center of the manipulator to a higher cost with no deformity
    Manipulator->dVoxels[3][3][3].deformability = 0.0;
    Manipulator->dVoxels[3][3][3].sensitivity = 2.0;
    Manipulator->dVoxels[3][3][3].A = 255;
    //Create start and goal states
    printf("Building test and goal states...\n");
    state6D * startState = NewState6D(NULL, Manipulator, Environment, Xstart, Ystart, Zstart, 0.0, 0.0, 0.0, 0.0);
    state6D * goalState = NewState6D(NULL, Manipulator, Environment, Xgoal, Ygoal, Zgoal, 0.0, 0.0, 0.0, 0.0);
    printf("Running sanity checks...\n");
    if (EvaluateState(startState, goalState, 0.5, IrosDistanceHeuristic3D, 1) == NULL)
    {
        printf("FATAL ERROR - start state is not feasible! - exiting\n");
        return -1;
    }
    if (EvaluateState(goalState, goalState, 0.5, IrosDistanceHeuristic3D, 1) == NULL)
    {
        printf("FATAL ERROR - goal state is not feasible! - exiting\n");
        return -1;
    }
    printf("Press ENTER to run the planner...\n");
    getchar();
    printf("Running full-connected Astar\n");
    return IrosAstar3D(startState, goalState, 0.5, path_save);
}

int inCircle(float xc, float yc, float x, float y, float r)
{
    float dx = x - xc;
    float dy = y - yc;
    float d = sqrt(pow(dx, 2) + pow(dy, 2));
    if (d <= r)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int AddCube(dVoxelArray * Environment, int Xs, int Ys, int Zs, int cube_dim, float cost, float deformity, uint8_t R, uint8_t G, uint8_t B, uint8_t A)
{
    int added = 0;
    for (int i = Xs; i < Xs + cube_dim; i++)
    {
        for (int j = Ys; j < Ys + cube_dim; j++)
        {
            for (int k = Zs; k < Zs + cube_dim; k++)
            {
                Environment->dVoxels[i][j][k].sensitivity = cost;
                Environment->dVoxels[i][j][k].deformability = deformity;
                Environment->dVoxels[i][j][k].R = R;
                Environment->dVoxels[i][j][k].G = G;
                Environment->dVoxels[i][j][k].B = B;
                Environment->dVoxels[i][j][k].A = A;
                added++;
            }
        }
    }
    return added;
}

int IrosTest(int Xdim, int Ydim, int Zdim, float Xstart, float Ystart, float Zstart, float Xgoal, float Ygoal, float Zgoal, int options, char * path_save)
{
    //File names - Not used
    //Load environment
    printf("Building a test environment...\n");
    dVoxelArray * Environment = AllocateDVoxelArray(Xdim, Ydim, Zdim, 0, 0, 0, 0.0, 1.0);
    //Make the walls of the environment
    for (int i = 0; i < Xdim; i++)
    {
        for (int j = 0; j < Ydim; j++)
        {
            for (int k = 0; k < Zdim; k++)
            {
                if ((i < 10 || i >= 80) || (j < 10 || j >= 105))
                {
                    Environment->dVoxels[i][j][k].sensitivity = 1.0;
                    Environment->dVoxels[i][j][k].deformability = 0.0;
                    Environment->dVoxels[i][j][k].R = 128;
                    Environment->dVoxels[i][j][k].G = 128;
                    Environment->dVoxels[i][j][k].B = 128;
                    Environment->dVoxels[i][j][k].A = 255;
                }
            }
        }
    }
    //Add two walls in the middle of the environment
    for (int i = 0; i < Xdim; i++)
    {
        for (int k = 0; k < Zdim; k++)
        {
            for (int j = 30; j < 45; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = 1.0;
                Environment->dVoxels[i][j][k].deformability = 0.5;
                Environment->dVoxels[i][j][k].A = 255;
            }
            for (int j = 70; j < 85; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = 1.0;
                Environment->dVoxels[i][j][k].deformability = 0.5;
                Environment->dVoxels[i][j][k].A = 255;
            }
        }
    }
    //Make a hole in the first wall
    for (int j = 30; j < 45; j++)
    {
        for (int k = 0; k < Zdim; k++)
        {
            for (int i = 10; i < 28; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
            for (int i = 38; i < 52; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
            for (int i = 62; i < 75; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    //Make a hole in the second wall
    for (int j = 70; j < 85; j++)
    {
        for (int k = 0; k < Zdim; k++)
        {
            for (int i = 40; i < 54; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
            for (int i = 64; i < 80; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
//    Environment->dVoxels[60][41][4].cost = 0.0;
//    Environment->dVoxels[60][41][4].deformity = 1.0;
//    Environment->dVoxels[60][41][4].A = 0;
//    Environment->dVoxels[60][40][5].cost = 0.0;
//    Environment->dVoxels[60][40][5].deformity = 1.0;
//    Environment->dVoxels[60][40][5].A = 0;
//    Environment->dVoxels[60][41][5].cost = 0.0;
//    Environment->dVoxels[60][41][5].deformity = 1.0;
//    Environment->dVoxels[60][41][5].A = 0;
    //Load reference manipulator
    printf("Building a test manipulator...\n");
    dVoxelArray * Manipulator = AllocateDVoxelArray(16, 16, 5, 8, 8, 2, 0.0, 1.0);
    //Color the manipulator red and make it rigid
    for (int i = 0; i < Manipulator->xDim; i++)
    {
        for (int j = 0; j < Manipulator->yDim; j++)
        {
            for (int k = 0; k < Manipulator->zDim; k++)
            {
                if (inCircle(7.5, 7.5, i, j, 8.0))
                {
                    Manipulator->dVoxels[i][j][k].deformability = 0.0;
                    Manipulator->dVoxels[i][j][k].sensitivity = 1.0;
                    Manipulator->dVoxels[i][j][k].A = 128;
                    Manipulator->dVoxels[i][j][k].R = 255;
                }
            }
        }
    }
    //Set the center of the manipulator to a higher cost with no deformity
    //Manipulator->dVoxels[7][7][7].deformity = 0.0;
    //Manipulator->dVoxels[7][7][7].cost = 2.0;
    //Manipulator->dVoxels[7][7][7].A = 255;
    //Create start and goal states
    printf("Building test and goal states...\n");
    state6D * startState = NewState6D(NULL, Manipulator, Environment, Xstart, Ystart, Zstart, 0.0, 0.0, 0.0, 0.0);
    state6D * goalState = NewState6D(NULL, Manipulator, Environment, Xgoal, Ygoal, Zgoal, 0.0, 0.0, 0.0, 0.0);
    printf("Running sanity checks...\n");
    if (EvaluateState(startState, goalState, 0.5, IrosDistanceHeuristic3D, 1) == NULL)
    {
        printf("FATAL ERROR - start state is not feasible! - exiting\n");
        return -1;
    }
    if (EvaluateState(goalState, goalState, 0.5, IrosDistanceHeuristic3D, 1) == NULL)
    {
        printf("FATAL ERROR - goal state is not feasible! - exiting\n");
        return -1;
    }
    printf("Press ENTER to run the planner...\n");
    getchar();
    printf("Running full-connected Astar\n");
    return IrosAstar3D(startState, goalState, 0.5, path_save);
}

dVoxelArray * MakeTripleWallEnv(float env_cost, float env_deform)
{
    int Xdim = 50;
    int Ydim = 110;
    int Zdim = 50;
    printf("Building the standard 3-wall test environment...\n");
    dVoxelArray * Environment = AllocateDVoxelArray(Xdim, Ydim, Zdim, 0, 0, 0, 0.0, 1.0);
    //Add three walls in the middle of the environment
    for (int i = 0; i < Xdim; i++)
    {
        for (int k = 0; k < Zdim; k++)
        {
            for (int j = 20; j < 30; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = env_cost;
                Environment->dVoxels[i][j][k].deformability = env_deform;
                Environment->dVoxels[i][j][k].A = 255;
                Environment->dVoxels[i][j][k].R = 100;
                Environment->dVoxels[i][j][k].G = 100;
                Environment->dVoxels[i][j][k].B = 100;
            }
            for (int j = 50; j < 60; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = env_cost;
                Environment->dVoxels[i][j][k].deformability = env_deform;
                Environment->dVoxels[i][j][k].A = 255;
                Environment->dVoxels[i][j][k].R = 100;
                Environment->dVoxels[i][j][k].G = 100;
                Environment->dVoxels[i][j][k].B = 100;
            }
            for (int j = 80; j < 90; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = env_cost;
                Environment->dVoxels[i][j][k].deformability = env_deform;
                Environment->dVoxels[i][j][k].A = 255;
                Environment->dVoxels[i][j][k].R = 100;
                Environment->dVoxels[i][j][k].G = 100;
                Environment->dVoxels[i][j][k].B = 100;
            }
        }
    }
    //Make a big hole in the first wall
    for (int j = 20; j < 30; j++)
    {
        for (int k = 0; k < 20; k++)
        {
            for (int i = 0; i < Xdim; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    //Make a big hole in the second wall
    for (int j = 50; j < 60; j++)
    {
        for (int k = Zdim - 20; k < Zdim; k++)
        {
            for (int i = 0; i < Xdim; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    //Make a big hole in the third wall
    for (int j = 80; j < 90; j++)
    {
        for (int k = 0; k < 20; k++)
        {
            for (int i = 0; i < Xdim; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    //Make a little hole in the first wall
    for (int j = 20; j < 30; j++)
    {
        for (int k = 20; k < 30; k++)
        {
            for (int i = 20; i < 30; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    //Make a little hole in the second wall
    for (int j = 50; j < 60; j++)
    {
        for (int k = 20; k <30; k++)
        {
            for (int i = 20; i < 30; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    //Make a little hole in the third wall
    for (int j = 80; j < 90; j++)
    {
        for (int k = 20; k < 30; k++)
        {
            for (int i = 20; i < 30; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    //Make a tiny hole in the first wall
    for (int j = 20; j < 30; j++)
    {
        for (int k = 45; k < 50; k++)
        {
            for (int i = 25; i < 30; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    //Make a tiny hole in the third wall
    for (int j = 80; j < 90; j++)
    {
        for (int k = 45; k < 50; k++)
        {
            for (int i = 20; i < 25; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    return Environment;
}

dVoxelArray * MakeLowResTripleWallEnv(float env_cost, float env_deform)
{
    int Xdim = 30;
    int Ydim = 66;
    int Zdim = 30;
    printf("Building the standard 3-wall test environment...\n");
    dVoxelArray * Environment = AllocateDVoxelArray(Xdim, Ydim, Zdim, 0, 0, 0, 0.0, 1.0);
    //Add three walls in the middle of the environment
    for (int i = 0; i < Xdim; i++)
    {
        for (int k = 0; k < Zdim; k++)
        {
            for (int j = 12; j < 18; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = env_cost;
                Environment->dVoxels[i][j][k].deformability = env_deform;
                Environment->dVoxels[i][j][k].A = 255;
                Environment->dVoxels[i][j][k].R = 100;
                Environment->dVoxels[i][j][k].G = 100;
                Environment->dVoxels[i][j][k].B = 100;
            }
            for (int j = 30; j < 36; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = env_cost;
                Environment->dVoxels[i][j][k].deformability = env_deform;
                Environment->dVoxels[i][j][k].A = 255;
                Environment->dVoxels[i][j][k].R = 100;
                Environment->dVoxels[i][j][k].G = 100;
                Environment->dVoxels[i][j][k].B = 100;
            }
            for (int j = 48; j < 54; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = env_cost;
                Environment->dVoxels[i][j][k].deformability = env_deform;
                Environment->dVoxels[i][j][k].A = 255;
                Environment->dVoxels[i][j][k].R = 100;
                Environment->dVoxels[i][j][k].G = 100;
                Environment->dVoxels[i][j][k].B = 100;
            }
        }
    }
    //Make a big hole in the first wall
    for (int j = 12; j < 18; j++)
    {
        for (int k = 0; k < 12; k++)
        {
            for (int i = 0; i < Xdim; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    //Make a big hole in the second wall
    for (int j = 30; j < 36; j++)
    {
        for (int k = Zdim - 12; k < Zdim; k++)
        {
            for (int i = 0; i < Xdim; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    //Make a big hole in the third wall
    for (int j = 48; j < 54; j++)
    {
        for (int k = 0; k < 12; k++)
        {
            for (int i = 0; i < Xdim; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    //Make a little hole in the first wall
    for (int j = 12; j < 18; j++)
    {
        for (int k = 12; k < 18; k++)
        {
            for (int i = 12; i < 18; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    //Make a little hole in the second wall
    for (int j = 30; j < 36; j++)
    {
        for (int k = 12; k < 18; k++)
        {
            for (int i = 12; i < 18; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    //Make a little hole in the third wall
    for (int j = 48; j < 54; j++)
    {
        for (int k = 12; k < 18; k++)
        {
            for (int i = 12; i < 18; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    //Make a tiny hole in the first wall
    for (int j = 12; j < 18; j++)
    {
        for (int k = Zdim - 3; k < Zdim; k++)
        {
            for (int i = 15; i < 18; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    //Make a tiny hole in the third wall
    for (int j = 48; j < 54; j++)
    {
        for (int k = Zdim - 3; k < Zdim; k++)
        {
            for (int i = 12; i < 15; i++)
            {
                Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                Environment->dVoxels[i][j][k].A = 0;
            }
        }
    }
    return Environment;
}

int InHole(int i, int j, int k, int hole_num)
{
    if (hole_num == 1)
    {
        if (i >= 10 && i < 20 && k >= 5 && k < 10)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else if (hole_num == 2)
    {
        if (i >= 30 && i < 40 && k >= 5 && k < 10)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

int InLowResHole(int i, int j, int k, int hole_num)
{
    if (hole_num == 1)
    {
        if (i >= 6 && i < 12 && k >= 4 && k < 8)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else if (hole_num == 2)
    {
        if (i >= 18 && i < 24 && k >= 4 && k < 8)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

dVoxelArray * MakeMultiCostEnv(float env_cost, float env_deform)
{
    int Xdim = 50;
    int Ydim = 110;
    int Zdim = 20;
    printf("Building the standard 3-wall test environment...\n");
    dVoxelArray * Environment = AllocateDVoxelArray(Xdim, Ydim, Zdim, 0, 0, 0, 0.0, 1.0);
    //Add two walls in the middle of the environment
    //Make the first half with different costs
    for (int i = 0; i < 25; i++)
    {
        for (int k = 0; k < Zdim; k++)
        {
            for (int j = 30; j < 40; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = 0.5 * env_cost;
                Environment->dVoxels[i][j][k].deformability = 0.5 * env_deform;
                Environment->dVoxels[i][j][k].A = 255;
                Environment->dVoxels[i][j][k].B = 255;
            }
            for (int j = 70; j < 80; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = env_cost;
                Environment->dVoxels[i][j][k].deformability = env_deform;
                Environment->dVoxels[i][j][k].A = 255;
                Environment->dVoxels[i][j][k].R = 100;
                Environment->dVoxels[i][j][k].G = 100;
                Environment->dVoxels[i][j][k].B = 100;
            }
        }
    }
    //Make the other half w/ reversed costs
    for (int i = 25; i < Xdim; i++)
    {
        for (int k = 0; k < Zdim; k++)
        {
            for (int j = 30; j < 40; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = env_cost;
                Environment->dVoxels[i][j][k].deformability = env_deform;
                Environment->dVoxels[i][j][k].A = 255;
                Environment->dVoxels[i][j][k].R = 100;
                Environment->dVoxels[i][j][k].G = 100;
                Environment->dVoxels[i][j][k].B = 100;
            }
            for (int j = 70; j < 80; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = 0.5 * env_cost;
                Environment->dVoxels[i][j][k].deformability = 0.5 * env_deform;
                Environment->dVoxels[i][j][k].A = 255;
                Environment->dVoxels[i][j][k].B = 255;
            }
        }
    }
    //Make four holes, one in each wall
    for (int i = 0; i < Xdim; i++)
    {
        for (int k = 0; k < Zdim; k++)
        {
            for (int j = 30; j < 40; j++)
            {
                if(InHole(i, j, k, 1) == 1)
                {
                    Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                    Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                    Environment->dVoxels[i][j][k].A = 0;
                }
                else if(InHole(i, j, k, 2) == 1)
                {
                    Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                    Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                    Environment->dVoxels[i][j][k].A = 0;
                }
            }
            for (int j = 70; j < 80; j++)
            {
                if(InHole(i, j, k, 1) == 1)
                {
                    Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                    Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                    Environment->dVoxels[i][j][k].A = 0;
                }
                else if(InHole(i, j, k, 2) == 1)
                {
                    Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                    Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                    Environment->dVoxels[i][j][k].A = 0;
                }
            }
        }
    }
    //Done
    return Environment;
}

dVoxelArray * MakeLowResMultiCostEnv(float env_cost, float env_deform)
{
    int Xdim = 30;
    int Ydim = 66;
    int Zdim = 12;
    printf("Building the standard 3-wall test environment...\n");
    dVoxelArray * Environment = AllocateDVoxelArray(Xdim, Ydim, Zdim, 0, 0, 0, 0.0, 1.0);
    //Add two walls in the middle of the environment
    //Make the first half with different costs
    for (int i = 0; i < 15; i++)
    {
        for (int k = 0; k < Zdim; k++)
        {
            for (int j = 18; j < 24; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = 0.5 * env_cost;
                Environment->dVoxels[i][j][k].deformability = 0.5 * env_deform;
                Environment->dVoxels[i][j][k].A = 255;
                Environment->dVoxels[i][j][k].B = 255;
            }
            for (int j = 42; j < 48; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = env_cost;
                Environment->dVoxels[i][j][k].deformability = env_deform;
                Environment->dVoxels[i][j][k].A = 255;
                Environment->dVoxels[i][j][k].R = 100;
                Environment->dVoxels[i][j][k].G = 100;
                Environment->dVoxels[i][j][k].B = 100;
            }
        }
    }
    //Make the other half w/ reversed costs
    for (int i = 15; i < Xdim; i++)
    {
        for (int k = 0; k < Zdim; k++)
        {
            for (int j = 18; j < 24; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = env_cost;
                Environment->dVoxels[i][j][k].deformability = env_deform;
                Environment->dVoxels[i][j][k].A = 255;
                Environment->dVoxels[i][j][k].R = 100;
                Environment->dVoxels[i][j][k].G = 100;
                Environment->dVoxels[i][j][k].B = 100;
            }
            for (int j = 42; j < 48; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = 0.5 * env_cost;
                Environment->dVoxels[i][j][k].deformability = 0.5 * env_deform;
                Environment->dVoxels[i][j][k].A = 255;
                Environment->dVoxels[i][j][k].B = 255;
            }
        }
    }
    //Make four holes, one in each wall
    for (int i = 0; i < Xdim; i++)
    {
        for (int k = 0; k < Zdim; k++)
        {
            for (int j = 18; j < 24; j++)
            {
                if(InLowResHole(i, j, k, 1) == 1)
                {
                    Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                    Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                    Environment->dVoxels[i][j][k].A = 0;
                }
                else if(InLowResHole(i, j, k, 2) == 1)
                {
                    Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                    Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                    Environment->dVoxels[i][j][k].A = 0;
                }
            }
            for (int j = 42; j < 48; j++)
            {
                if(InLowResHole(i, j, k, 1) == 1)
                {
                    Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                    Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                    Environment->dVoxels[i][j][k].A = 0;
                }
                else if(InLowResHole(i, j, k, 2) == 1)
                {
                    Environment->dVoxels[i][j][k].sensitivity = Environment->defaultSensitivity;
                    Environment->dVoxels[i][j][k].deformability = Environment->defaultDeformability;
                    Environment->dVoxels[i][j][k].A = 0;
                }
            }
        }
    }
    //Done
    return Environment;
}

dVoxelArray * MakeHockeyPuck(float robot_cost, float robot_deform)
{
    printf("Building the hockey puck robot...\n");
    dVoxelArray * Manipulator = AllocateDVoxelArray(16, 16, 5, 8, 8, 2, 0.0, 1.0);
    //Color the manipulator red and make it rigid
    for (int i = 0; i < Manipulator->xDim; i++)
    {
        for (int j = 0; j < Manipulator->yDim; j++)
        {
            for (int k = 0; k < Manipulator->zDim; k++)
            {
                if (inCircle(7.5, 7.5, i, j, 8.0))
                {
                    Manipulator->dVoxels[i][j][k].deformability = robot_deform;
                    Manipulator->dVoxels[i][j][k].sensitivity = robot_cost;
                    Manipulator->dVoxels[i][j][k].A = 128;
                    Manipulator->dVoxels[i][j][k].R = 255;
                }
            }
        }
    }
    return Manipulator;
}

dVoxelArray * MakeLowResHockeyPuck(float robot_cost, float robot_deform)
{
    printf("Building the hockey puck robot...\n");
    dVoxelArray * Manipulator = AllocateDVoxelArray(9, 9, 3, 5, 5, 1, 0.0, 1.0);
    //Color the manipulator red and make it rigid
    for (int i = 0; i < Manipulator->xDim; i++)
    {
        for (int j = 0; j < Manipulator->yDim; j++)
        {
            for (int k = 0; k < Manipulator->zDim; k++)
            {
                if (inCircle(5.0, 5.0, i, j, 4.5))
                {
                    Manipulator->dVoxels[i][j][k].deformability = robot_deform;
                    Manipulator->dVoxels[i][j][k].sensitivity = robot_cost;
                    Manipulator->dVoxels[i][j][k].A = 128;
                    Manipulator->dVoxels[i][j][k].R = 255;
                }
            }
        }
    }
    return Manipulator;
}

dVoxelArray * MakeLShape(float robot_cost, float robot_deform)
{
    printf("Building the L-shaped robot...\n");
    dVoxelArray * Manipulator = AllocateDVoxelArray(15, 15, 5, 2, 2, 2, 0.0, 1.0);
    //Color the manipulator red and make it rigid
    for (int i = 0; i < Manipulator->xDim; i++)
    {
        for (int j = 0; j < Manipulator->yDim; j++)
        {
            for (int k = 0; k < Manipulator->zDim; k++)
            {
                if (i < 5 || j < 5)
                {
                    Manipulator->dVoxels[i][j][k].deformability = robot_deform;
                    Manipulator->dVoxels[i][j][k].sensitivity = robot_cost;
                    Manipulator->dVoxels[i][j][k].A = 128;
                    Manipulator->dVoxels[i][j][k].R = 255;
                }
            }
        }
    }
    return Manipulator;
}

dVoxelArray * MakeLowResLShape(float robot_cost, float robot_deform)
{
    printf("Building the L-shaped robot...\n");
    dVoxelArray * Manipulator = AllocateDVoxelArray(9, 9, 3, 5, 5, 1, 0.0, 1.0);
    //Color the manipulator red and make it rigid
    for (int i = 0; i < Manipulator->xDim; i++)
    {
        for (int j = 0; j < Manipulator->yDim; j++)
        {
            for (int k = 0; k < Manipulator->zDim; k++)
            {
                if (i < 3 || j < 3)
                {
                    Manipulator->dVoxels[i][j][k].deformability = robot_deform;
                    Manipulator->dVoxels[i][j][k].sensitivity = robot_cost;
                    Manipulator->dVoxels[i][j][k].A = 128;
                    Manipulator->dVoxels[i][j][k].R = 255;
                }
            }
        }
    }
    return Manipulator;
}

int IrosSimulationTest(int Xdim, int Ydim, int Zdim, float Xstart, float Ystart, float Zstart, float Xgoal, float Ygoal, float Zgoal, int options, float pareto, char * path_save)
{
    float solid_cost = 1.0;
    float soft_cost = 0.5;
    float solid_deform = 0.0;
    float soft_deform = 0.5;
    float robot_cost = 0.0;
    float robot_deform = 0.0;
    float env_cost = 0.0;
    float env_deform = 0.0;
    printf("Running with pareto value of %f\n", pareto);
    if (options == 0)
    {
        printf("Running with ENV:[SOLID] | ROBOT:[SOLID]\n");
        env_deform = solid_deform;
        env_cost = solid_cost;
        robot_deform = solid_deform;
        robot_cost = solid_cost;
    }
    else if (options == 1)
    {
        printf("Running with ENV:[SOFT] | ROBOT:[SOLID]\n");
        env_deform = soft_deform;
        env_cost = soft_cost;
        robot_deform = solid_deform;
        robot_cost = solid_cost;
    }
    else if (options == 2)
    {
        printf("Running with ENV:[SOLID] | ROBOT:[SOFT]\n");
        env_deform = solid_deform;
        env_cost = solid_cost;
        robot_deform = soft_deform;
        robot_cost = soft_cost;
    }
    else
    {
        printf("Running with ENV:[SOFT] | ROBOT:[SOFT]\n");
        env_deform = soft_deform;
        env_cost = soft_cost;
        robot_deform = soft_deform;
        robot_cost = soft_cost;
    }
    //Create environment and robot
    //dVoxelArray * Environment = MakeTripleWallEnv(env_cost, env_deform);
    //dVoxelArray * Robot = MakeHockeyPuck(robot_cost, robot_deform);
    dVoxelArray * Environment = MakeMultiCostEnv(env_cost, env_deform);
    dVoxelArray * Robot = MakeLShape(robot_cost, robot_deform);
    //Create start and goal states
    printf("Building test and goal states...\n");
    state6D * startState = NewState6D(NULL, Robot, Environment, Xstart, Ystart, Zstart, 0.0, 0.0, 0.0, 0.0);
    state6D * goalState = NewState6D(NULL, Robot, Environment, Xgoal, Ygoal, Zgoal, 0.0, 0.0, 0.0, 0.0);
    printf("Running sanity checks...\n");
    if (EvaluateState(startState, goalState, 0.5, IrosDistanceHeuristic3D, 3) == NULL)
    {
        printf("FATAL ERROR - start state is not feasible! - exiting\n");
        return -1;
    }
    if (EvaluateState(goalState, goalState, 0.5, IrosDistanceHeuristic3D, 3) == NULL)
    {
        printf("FATAL ERROR - goal state is not feasible! - exiting\n");
        return -1;
    }
    //printf("Press ENTER to run the planner...\n");
    //getchar();
    printf("Running full-connected Astar\n");
    int length = IrosAstar3D(startState, goalState, pareto, path_save);
    DestroyDVoxelArray(Environment);
    DestroyDVoxelArray(Robot);
    DestroyState6D(startState);
    DestroyState6D(goalState);
    return length;
}

int LowResIrosSimulationTest(int Xdim, int Ydim, int Zdim, float Xstart, float Ystart, float Zstart, float Xgoal, float Ygoal, float Zgoal, int options, float pareto, char * path_save)
{
    float solid_cost = 1.0;
    float soft_cost = 0.5;
    float solid_deform = 0.0;
    float soft_deform = 0.5;
    float robot_cost = 0.0;
    float robot_deform = 0.0;
    float env_cost = 0.0;
    float env_deform = 0.0;
    printf("Running with pareto value of %f\n", pareto);
    if (options == 0)
    {
        printf("Running with ENV:[SOLID] | ROBOT:[SOLID]\n");
        env_deform = solid_deform;
        env_cost = solid_cost;
        robot_deform = solid_deform;
        robot_cost = solid_cost;
    }
    else if (options == 1)
    {
        printf("Running with ENV:[SOFT] | ROBOT:[SOLID]\n");
        env_deform = soft_deform;
        env_cost = soft_cost;
        robot_deform = solid_deform;
        robot_cost = solid_cost;
    }
    else if (options == 2)
    {
        printf("Running with ENV:[SOLID] | ROBOT:[SOFT]\n");
        env_deform = solid_deform;
        env_cost = solid_cost;
        robot_deform = soft_deform;
        robot_cost = soft_cost;
    }
    else
    {
        printf("Running with ENV:[SOFT] | ROBOT:[SOFT]\n");
        env_deform = soft_deform;
        env_cost = soft_cost;
        robot_deform = soft_deform;
        robot_cost = soft_cost;
    }
    //Create environment and robot
    //dVoxelArray * Environment = MakeLowResTripleWallEnv(env_cost, env_deform);
    //dVoxelArray * Robot = MakeLowResHockeyPuck(robot_cost, robot_deform);
    dVoxelArray * Environment = MakeLowResMultiCostEnv(env_cost, env_deform);
    dVoxelArray * Robot = MakeLowResLShape(robot_cost, robot_deform);
    //Create start and goal states
    printf("Building test and goal states...\n");
    state6D * startState = NewState6D(NULL, Robot, Environment, Xstart, Ystart, Zstart, 0.0, 0.0, 0.0, 0.0);
    state6D * goalState = NewState6D(NULL, Robot, Environment, Xgoal, Ygoal, Zgoal, 0.0, 0.0, 0.0, 0.0);
    printf("Running sanity checks...\n");
    if (EvaluateState(startState, goalState, 0.5, IrosDistanceHeuristic3D, 3) == NULL)
    {
        printf("FATAL ERROR - start state is not feasible! - exiting\n");
        return -1;
    }
    if (EvaluateState(goalState, goalState, 0.5, IrosDistanceHeuristic3D, 3) == NULL)
    {
        printf("FATAL ERROR - goal state is not feasible! - exiting\n");
        return -1;
    }
    //printf("Press ENTER to run the planner...\n");
    //getchar();
    printf("Running full-connected Astar\n");
    int length = IrosAstar3D(startState, goalState, pareto, path_save);
    DestroyDVoxelArray(Environment);
    DestroyDVoxelArray(Robot);
    DestroyState6D(startState);
    DestroyState6D(goalState);
    return length;
}

int LowResTriplewallSimulationTest(int options, float pareto, char* path_save)
{
    float Xstart = 24.0;
    float Ystart = 6.0;
    float Zstart = 24.0;
    float Xgoal = 6.0;
    float Ygoal = 60.0;
    float Zgoal = 24.0;
    float solid_cost = 1.0;
    float soft_cost = 0.5;
    float solid_deform = 0.0;
    float soft_deform = 0.5;
    float robot_cost = 0.0;
    float robot_deform = 0.0;
    float env_cost = 0.0;
    float env_deform = 0.0;
    printf("Running with pareto value of %f\n", pareto);
    if (options == 0)
    {
        printf("Running with ENV:[SOLID] | ROBOT:[SOLID]\n");
        env_deform = solid_deform;
        env_cost = solid_cost;
        robot_deform = solid_deform;
        robot_cost = solid_cost;
    }
    else if (options == 1)
    {
        printf("Running with ENV:[SOFT] | ROBOT:[SOLID]\n");
        env_deform = soft_deform;
        env_cost = soft_cost;
        robot_deform = solid_deform;
        robot_cost = solid_cost;
    }
    else if (options == 2)
    {
        printf("Running with ENV:[SOLID] | ROBOT:[SOFT]\n");
        env_deform = solid_deform;
        env_cost = solid_cost;
        robot_deform = soft_deform;
        robot_cost = soft_cost;
    }
    else
    {
        printf("Running with ENV:[SOFT] | ROBOT:[SOFT]\n");
        env_deform = soft_deform;
        env_cost = soft_cost;
        robot_deform = soft_deform;
        robot_cost = soft_cost;
    }
    //Create environment and robot
    dVoxelArray * Environment = MakeLowResTripleWallEnv(env_cost, env_deform);
    dVoxelArray * Robot = MakeLowResHockeyPuck(robot_cost, robot_deform);
    //Create start and goal states
    printf("Building test and goal states...\n");
    state6D * startState = NewState6D(NULL, Robot, Environment, Xstart, Ystart, Zstart, 0.0, 0.0, 0.0, 0.0);
    state6D * goalState = NewState6D(NULL, Robot, Environment, Xgoal, Ygoal, Zgoal, 0.0, 0.0, 0.0, 0.0);
    printf("Running sanity checks...\n");
    if (EvaluateState(startState, goalState, 0.5, IrosDistanceHeuristic3D, 3) == NULL)
    {
        printf("FATAL ERROR - start state is not feasible! - exiting\n");
        return -1;
    }
    if (EvaluateState(goalState, goalState, 0.5, IrosDistanceHeuristic3D, 3) == NULL)
    {
        printf("FATAL ERROR - goal state is not feasible! - exiting\n");
        return -1;
    }
    //printf("Press ENTER to run the planner...\n");
    //getchar();
    printf("Running full-connected Astar\n");
    int length = IrosAstar3D(startState, goalState, pareto, path_save);
    DestroyDVoxelArray(Environment);
    DestroyDVoxelArray(Robot);
    DestroyState6D(startState);
    DestroyState6D(goalState);
    return length;
}

int LowResMulticostSimulationTest(int options, float pareto, char* path_save)
{
    float Xstart = 21.0;
    float Ystart = 6.0;
    float Zstart = 6.0;
    float Xgoal = 6.0;
    float Ygoal = 57.0;
    float Zgoal = 6.0;
    float solid_cost = 1.0;
    float soft_cost = 0.5;
    float solid_deform = 0.0;
    float soft_deform = 0.5;
    float robot_cost = 0.0;
    float robot_deform = 0.0;
    float env_cost = 0.0;
    float env_deform = 0.0;
    printf("Running with pareto value of %f\n", pareto);
    if (options == 0)
    {
        printf("Running with ENV:[SOLID] | ROBOT:[SOLID]\n");
        env_deform = solid_deform;
        env_cost = solid_cost;
        robot_deform = solid_deform;
        robot_cost = solid_cost;
    }
    else if (options == 1)
    {
        printf("Running with ENV:[SOFT] | ROBOT:[SOLID]\n");
        env_deform = soft_deform;
        env_cost = soft_cost;
        robot_deform = solid_deform;
        robot_cost = solid_cost;
    }
    else if (options == 2)
    {
        printf("Running with ENV:[SOLID] | ROBOT:[SOFT]\n");
        env_deform = solid_deform;
        env_cost = solid_cost;
        robot_deform = soft_deform;
        robot_cost = soft_cost;
    }
    else
    {
        printf("Running with ENV:[SOFT] | ROBOT:[SOFT]\n");
        env_deform = soft_deform;
        env_cost = soft_cost;
        robot_deform = soft_deform;
        robot_cost = soft_cost;
    }
    //Create environment and robot
    dVoxelArray * Environment = MakeLowResMultiCostEnv(env_cost, env_deform);
    dVoxelArray * Robot = MakeLowResLShape(robot_cost, robot_deform);
    //Create start and goal states
    printf("Building test and goal states...\n");
    state6D * startState = NewState6D(NULL, Robot, Environment, Xstart, Ystart, Zstart, 0.0, 0.0, 0.0, 0.0);
    state6D * goalState = NewState6D(NULL, Robot, Environment, Xgoal, Ygoal, Zgoal, 0.0, 0.0, 0.0, 0.0);
    printf("Running sanity checks...\n");
    if (EvaluateState(startState, goalState, 0.5, IrosDistanceHeuristic3D, 3) == NULL)
    {
        printf("FATAL ERROR - start state is not feasible! - exiting\n");
        return -1;
    }
    if (EvaluateState(goalState, goalState, 0.5, IrosDistanceHeuristic3D, 3) == NULL)
    {
        printf("FATAL ERROR - goal state is not feasible! - exiting\n");
        return -1;
    }
    //printf("Press ENTER to run the planner...\n");
    //getchar();
    printf("Running full-connected Astar\n");
    int length = IrosAstar3D(startState, goalState, pareto, path_save);
    DestroyDVoxelArray(Environment);
    DestroyDVoxelArray(Robot);
    DestroyState6D(startState);
    DestroyState6D(goalState);
    return length;
}

int main(int argc, char** argv)
{
    //printf("Testing the rotation/transform system...\n");
    //TransformTest();
    //printf("Testing the hashtable system...\n");
    //HashTest(1000);
    //printf("Press ENTER to run A* search...");
    //SearchTest(5.0,5.0,5.0,17.0,17.0,17.0,0);
    //getchar();
    printf("Starting a test search of the Iros planner...\n");
    //IrosTest(90, 115, 5, 71.0, 18.0, 2.0, 71.0, 96.0, 2.0, 1);
    float pareto = 0.5;
    int ctrl = 0;
    std::string file_prefix("test");
    printf("Running version with %d args\n", argc);
    if (argc == 3)
    {
        pareto = atof(argv[1]);
        ctrl = atoi(argv[2]);
        printf("Args: %s [0], %s [1], (none) [2]\n", argv[1], argv[2]);
    }
    else if (argc >= 4)
    {
        pareto = atof(argv[1]);
        ctrl = atoi(argv[2]);
        file_prefix = std::string(argv[3]);
        printf("Args: %s [0], %s [1], %s [2]\n", argv[1], argv[2], argv[3]);
    }
    else
    {
        printf("Error - not enough arguments!\nRun this program with the following arguments:\n./planner3d <float>pareto_weight <int>ctrl\nOR\n./planner3d <float>pareto_weight <int>ctrl <char*>file_prefix\n");
        return 0;
    }
    //IrosSimulationTest(50, 110, 50, 40, 10, 40, 10, 100, 40, ctrl, pareto, path_save);
    //IrosSimulationTest(50,110,15,35,10,6,10,95,6,ctrl,pareto,path_save);
    //LowResIrosSimulationTest(30,66,12,21,6,6,6,57,6,ctrl,pareto,path_save);
    printf("- TripleWall simulation test -\n");
    std::string save_name = file_prefix + std::string("_triplewall");
    LowResTriplewallSimulationTest(ctrl, pareto, (char*)save_name.c_str());
    printf("- Multicost simulation test -\n");
    save_name = file_prefix + std::string("_multicost");
    LowResMulticostSimulationTest(ctrl, pareto, (char*)save_name.c_str());
    return 0;
}
