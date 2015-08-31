#include <iostream>
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "deformable_astar/IrosPlanner.h"

#define _USE_MATH_DEFINES

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
            }
            for (int j = 50; j < 60; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = env_cost;
                Environment->dVoxels[i][j][k].deformability = env_deform;
                Environment->dVoxels[i][j][k].A = 255;
            }
            for (int j = 80; j < 90; j++)
            {
                Environment->dVoxels[i][j][k].sensitivity = env_cost;
                Environment->dVoxels[i][j][k].deformability = env_deform;
                Environment->dVoxels[i][j][k].A = 255;
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

float CalcStateDeform(int options, float xt, float yt, float zt, float xr, float yr, float zr)
{
    float solid_cost = 1.0;
    float soft_cost = 0.5;
    float solid_deform = 0.0;
    float soft_deform = 0.5;
    float robot_cost = 0.0;
    float robot_deform = 0.0;
    float env_cost = 0.0;
    float env_deform = 0.0;
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
    dVoxelArray * Environment = MakeTripleWallEnv(env_cost, env_deform);
    dVoxelArray * Robot = MakeHockeyPuck(robot_cost, robot_deform);
    //dVoxelArray * Environment = MakeMultiCostEnv(env_cost, env_deform);
    //dVoxelArray * Robot = MakeLShape(robot_cost, robot_deform);
    //Create start and goal states
    printf("Building test and goal states...\n");
    state6D * startState = NewState6D(NULL, Robot, Environment, xt, yt, zt, xr, yr, zr, 0.0);
    printf("Running sanity check...\n");
    state6D * assessed = EvaluateState(startState, startState, 0.5, IrosDistanceHeuristic3D, 3);
    if (assessed == NULL)
    {
        printf("FATAL ERROR - start state is not feasible! - exiting\n");
        return -1.0;
    }
    return assessed->COST;
}

int main(int argc, char** argv)
{
    printf("Running the standalone state deformation evaluator...\n");
    int ctrl = 0;
    float xt = 0.0;
    float yt = 0.0;
    float zt = 0.0;
    float xr = 0.0;
    float yr = 0.0;
    float zr = 0.0;
    printf("Running version with %d args\n", argc);
    if (argc == 5)
    {
        ctrl = atoi(argv[1]);
        xt = atof(argv[2]);
        yt = atof(argv[3]);
        zt = atof(argv[4]);
        printf("Args: %s [0], %s [1], (none) [2]\n", argv[1], argv[2]);
    }
    else if (argc >= 8)
    {
        ctrl = atoi(argv[1]);
        xt = atof(argv[2]);
        yt = atof(argv[3]);
        zt = atof(argv[4]);
        xr = atof(argv[5]);
        yr = atof(argv[6]);
        zr = atof(argv[7]);
        printf("Args: %s [0], %s [1], %s [2]\n", argv[1], argv[2], argv[3]);
    }
    else
    {
        printf("Error - not enough arguments!\nRun this program with the following arguments:\n./statechecker <int>ctrl <float>x <float>y <float>z\nOR\n./statechecker <int>ctrl <float>x <float>y <float>z <float>xr <float>yr <float>zr\n");
        return 0;
    }
    struct timespec st, et;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &st);
    float assessed_deformation = CalcStateDeform(ctrl, xt, yt, zt, xr, yr, zr);
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &et);
    float secs = (float)(et.tv_sec - st.tv_sec);
    secs = secs + (float)(et.tv_nsec - st.tv_nsec) / 1000000000.0;
    printf("Assessed deformation = %f in %f seconds\n", assessed_deformation, secs);
    return 0;
}

