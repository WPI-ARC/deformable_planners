#include <iostream>
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "time.h"
#include <cstdlib>
#include "deformable_astar/IrosPlanner.h"
#include <arc_utilities/pretty_print.hpp>
#include "xtf/xtf.hpp"

#define _USE_MATH_DEFINES
#define ROTATION_STEP (M_PI_4 / 2.0)
#define CLOSE_ENOUGH(a, b) (fabs(a - b)<=0.0001?(1):(0))

void AddObstacleBox(dVoxelArray* environment, int xstart, int ystart, int xextent, int yextent, float deformability, float sensitivity, uint8_t r, uint8_t g, uint8_t b, uint8_t a=0xff)
{
    int rsx = xstart;
    if (rsx >= environment->xDim)
    {
        return;
    }
    if (rsx < 0)
    {
        rsx = 0;
    }
    int rex = xstart + abs(xextent);
    if (rex >= environment->xDim)
    {
        rex = environment->xDim;
    }
    int rsy = ystart;
    if (rsy >= environment->yDim)
    {
        return;
    }
    if (rsy < 0)
    {
        rsy = 0;
    }
    int rey = ystart + abs(yextent);
    if (rey >= environment->yDim)
    {
        rey = environment->yDim;
    }
    for (int x = rsx; x < rex; x++)
    {
        for (int y = rsy; y < rey; y++)
        {
            for (int z = 0; z < environment->zDim; z++)
            {
                environment->dVoxels[x][y][0].deformability = deformability;
                environment->dVoxels[x][y][0].sensitivity += sensitivity;
                environment->dVoxels[x][y][0].R = r;
                environment->dVoxels[x][y][0].G = g;
                environment->dVoxels[x][y][0].B = b;
                environment->dVoxels[x][y][0].A = a;
            }
        }
    }
}

inline int random_in_range(int start, int end)
{
    double randd = (double)rand() / (double)RAND_MAX;
    double rand_offset = (double)(end - start) * randd;
    return start + (int)rand_offset;
}

void OffsetEnvironment(std::string in_filename, std::string out_filename, std::string out_csv_filename, int xshift, int yshift)
{
    dVoxelArray* Reference = LoadDVoxelArrayFromFile((char*)in_filename.c_str());
    dVoxelArray* Modified = AllocateDVoxelArray(Reference->xDim, Reference->yDim, Reference->zDim, Reference->xCenter, Reference->yCenter, Reference->zCenter, 1.0, 0.0);
    for (int i = 0; i < Reference->xDim; i++)
    {
        for (int j = 0; j < Reference->yDim; j++)
        {
            int new_x = i + xshift;
            int new_y = j + yshift;
            if (new_x >= 0 && new_y >= 0 && new_x < Modified->xDim && new_y < Modified->yDim)
            {
                Modified->dVoxels[new_x][new_y][0] = Reference->dVoxels[i][j][0];
            }
        }
    }
    UpdateSDF(Modified);
    WriteDVoxelArrayToFile(Modified, (char*)out_filename.c_str());
    WriteDVoxelArrayToCSVFile(Modified, (char*)out_csv_filename.c_str());
}

void OffsetEnvironmentBlocks(std::string in_filename, std::string out_filename, std::string out_csv_filename, int xshift, int yshift)
{
    dVoxelArray* Reference = LoadDVoxelArrayFromFile((char*)in_filename.c_str());
    dVoxelArray* Modified = AllocateDVoxelArray(Reference->xDim, Reference->yDim, Reference->zDim, Reference->xCenter, Reference->yCenter, Reference->zCenter, 0.0, 1.0);

    for (int i = 0; i < Reference->xDim; i++)
    {
        for (int j = 0; j < Reference->yDim; j++)
        {
            int new_x = i + random_in_range(-xshift, xshift);
            int new_y = j + random_in_range(-yshift, yshift);
            if (new_x >= 0 && new_y >= 0 && new_x < Modified->xDim && new_y < Modified->yDim)
            {
                Modified->dVoxels[new_x][new_y][0].deformability = Reference->dVoxels[i][j][0].deformability;
                Modified->dVoxels[new_x][new_y][0].sensitivity = Reference->dVoxels[i][j][0].sensitivity;
                Modified->dVoxels[new_x][new_y][0].SDF = Reference->dVoxels[i][j][0].SDF;
                Modified->dVoxels[new_x][new_y][0].R = Reference->dVoxels[i][j][0].R;
                Modified->dVoxels[new_x][new_y][0].G = Reference->dVoxels[i][j][0].G;
                Modified->dVoxels[new_x][new_y][0].B = Reference->dVoxels[i][j][0].B;
                Modified->dVoxels[new_x][new_y][0].A = Reference->dVoxels[i][j][0].A;
            }
        }
    }
    /*
    // Step through the grid one 10x10 block at a time
    int block_size = 10;
    for (int bsi = 0; bsi < Reference->xDim; bsi+=block_size)
    {
        for (int bsj = 0; bsj < Reference->yDim; bsj+=block_size)
        {
            // Step through the current block and shift it into the modified version
            int block_offset_x = random_in_range(-xshift, xshift);
            int block_offset_y = random_in_range(-yshift, yshift);
            printf("Shifting block at (%d,%d) with offsets %d,%d\n", bsi, bsj, block_offset_x, block_offset_y);
            for (int i = bsi; i < (bsi + block_size); i++)
            {
                for (int j = bsj; j < (bsj + block_size); j++)
                {
                    int new_x = i + block_offset_x;
                    int new_y = j + block_offset_y;
                    if (new_x >= 0 && new_y >= 0 && new_x < Modified->xDim && new_y < Modified->yDim)
                    {
                        Modified->dVoxels[new_x][new_y][0] = Reference->dVoxels[i][j][0];
                    }
                }
            }
        }
    }
    */
    UpdateSDF(Modified);
    WriteDVoxelArrayToFile(Modified, (char*)out_filename.c_str());
    WriteDVoxelArrayToCSVFile(Modified, (char*)out_csv_filename.c_str());
}

void GenerateIROSRobot()
{
    std::cout << "Making IROS Robot..." << std::endl;
    dVoxelArray * Robot = AllocateDVoxelArray(16, 16, 1, 11, 4, 0, 1.0, 0.0);
    for (int i = 0; i < Robot->xDim; i++)
    {
        for (int j = 0; j < Robot->yDim; j++)
        {
            if (i < 8 && j >= 8)
            {
                Robot->dVoxels[i][j][0].deformability = 1.0;
                Robot->dVoxels[i][j][0].sensitivity = 0.0;
            }
            else
            {
                Robot->dVoxels[i][j][0].A = 0xff;
            }
        }
    }
    WriteDVoxelArrayToFile(Robot, (char*)"IROSbot.dvxl");
    WriteDVoxelArrayToCSVFile(Robot, (char*)"IROSbot.csv");
    std::cout << "...Done Making IROS Robot" << std::endl;
}

void MakeTestEnvironment1()
{
    std::cout << "Making Test Environment 1..." << std::endl;
    dVoxelArray * Environment = AllocateDVoxelArray(84, 116, 1, 0, 0, 0, 0.0, 1.0);
    ;
    ;
    UpdateSDF(Environment);
    WriteDVoxelArrayToFile(Environment, (char*)"TestEnv1.dvxl");
    WriteDVoxelArrayToCSVFile(Environment, (char*)"TestEnv1.csv");
    std::cout << "...Done Making Test Environment 1" << std::endl;
}

void MakeTestEnvironment2()
{
    std::cout << "Making Test Environment 2..." << std::endl;
    dVoxelArray * Environment = AllocateDVoxelArray(84, 116, 1, 0, 0, 0, 0.0, 1.0);
    // Make the 8 obstacles
    int sx, sy;
    // First set
    sx = 0;
    sy = 17;
    for (int i = sx; i < sx + 16; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    sx = 30;
    sy = 17;
    for (int i = sx; i < sx + 20; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    sx = 68;
    sy = 17;
    for (int i = sx; i < sx + 16; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    // Second set
    sx = 0;
    sy = 50;
    for (int i = sx; i < sx + 35; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    sx = 49;
    sy = 50;
    for (int i = sx; i < sx + 35; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    // Third set
    sx = 0;
    sy = 83;
    for (int i = sx; i < sx + 16; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    sx = 34;
    sy = 83;
    for (int i = sx; i < sx + 20; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    sx = 68;
    sy = 83;
    for (int i = sx; i < sx + 16; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    UpdateSDF(Environment);
    WriteDVoxelArrayToFile(Environment, (char*)"TestEnv2.dvxl");
    WriteDVoxelArrayToCSVFile(Environment, (char*)"TestEnv2.csv");
    std::cout << "...Done Making Test Environment 2" << std::endl;
}

void MakeTestEnvironment3()
{
    std::cout << "Making Test Environment 3..." << std::endl;
    dVoxelArray * Environment = AllocateDVoxelArray(84, 116, 1, 0, 0, 0, 0.0, 1.0);
    ;
    ;
    UpdateSDF(Environment);
    WriteDVoxelArrayToFile(Environment, (char*)"TestEnv3.dvxl");
    WriteDVoxelArrayToCSVFile(Environment, (char*)"TestEnv3.csv");
    std::cout << "...Done Making Test Environment 3" << std::endl;
}

void MakeTestEnvironment4()
{
    std::cout << "Making Test Environment 4..." << std::endl;
    dVoxelArray * Environment = AllocateDVoxelArray(84, 116, 1, 0, 0, 0, 0.0, 1.0);
    ;
    ;
    UpdateSDF(Environment);
    WriteDVoxelArrayToFile(Environment, (char*)"TestEnv4.dvxl");
    WriteDVoxelArrayToCSVFile(Environment, (char*)"TestEnv4.csv");
    std::cout << "...Done Making Test Environment 4" << std::endl;
}

void MakeTestEnvironmentShifted()
{
    std::cout << "Making Test Environment Shifted..." << std::endl;
    dVoxelArray * Environment = AllocateDVoxelArray(84, 116, 1, 0, 0, 0, 0.0, 1.0);
    // Make the 8 obstacles
    int sx, sy;
    // First set
    sx = 0;
    sy = 17;
    for (int i = sx; i < sx + 15; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    sx = 29;
    sy = 17;
    for (int i = sx; i < sx + 20; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    sx = 67;
    sy = 17;
    for (int i = sx; i < sx + 17; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    // Second set
    sx = 0;
    sy = 50;
    for (int i = sx; i < sx + 34; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    sx = 48;
    sy = 50;
    for (int i = sx; i < sx + 36; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    // Third set
    sx = 0;
    sy = 83;
    for (int i = sx; i < sx + 15; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    sx = 33;
    sy = 83;
    for (int i = sx; i < sx + 20; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    sx = 67;
    sy = 83;
    for (int i = sx; i < sx + 17; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    UpdateSDF(Environment);
    WriteDVoxelArrayToFile(Environment, (char*)"TestEnvShifted.dvxl");
    WriteDVoxelArrayToCSVFile(Environment, (char*)"TestEnvShifted.csv");
    std::cout << "...Done Making Test Environment Shifted" << std::endl;
}

void MakeTestEnvironmentSpecialShifted()
{
    std::cout << "Making Test Environment Shifted..." << std::endl;
    dVoxelArray * Environment = AllocateDVoxelArray(84, 116, 1, 0, 0, 0, 0.0, 1.0);
    // Make the 8 obstacles
    int sx, sy;
    // First set (offset -1 cell in X)
    sx = 0;
    sy = 17;
    for (int i = sx; i < sx + 15; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    sx = 29;
    sy = 17;
    for (int i = sx; i < sx + 20; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    sx = 67;
    sy = 17;
    for (int i = sx; i < sx + 17; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    // Second set (left unchanged)
    sx = 0;
    sy = 50;
    for (int i = sx; i < sx + 35; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    sx = 49;
    sy = 50;
    for (int i = sx; i < sx + 35; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    // Third set (offset +1 cell in X)
    sx = 0;
    sy = 83;
    for (int i = sx; i < sx + 17; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    sx = 35;
    sy = 83;
    for (int i = sx; i < sx + 20; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    sx = 69;
    sy = 83;
    for (int i = sx; i < sx + 15; i++)
    {
        for (int j = sy; j < sy + 16; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    UpdateSDF(Environment);
    WriteDVoxelArrayToFile(Environment, (char*)"TestEnvSpecialShifted.dvxl");
    WriteDVoxelArrayToCSVFile(Environment, (char*)"TestEnvSpecialShifted.csv");
    std::cout << "...Done Making Test Environment Special Shifted" << std::endl;
}

void MakeIOCEnvironment1(float s1=3.0, float s2=2.0, float s3=1.0)
{
    std::cout << "Making IOC test environment 1..." << std::endl;
    dVoxelArray * Environment = AllocateDVoxelArray(84, 116, 1, 0, 0, 0, 0.0, 1.0);
    // First set
    for (int i = 0; i < 14; i++)
    {
        for (int j = 40; j < 76; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = s1;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0x00;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 28; i < 56; i++)
    {
        for (int j = 40; j < 76; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = s2;
            Environment->dVoxels[i][j][0].R = 0x00;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 70; i < 84; i++)
    {
        for (int j = 40; j < 76; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = s3;
            Environment->dVoxels[i][j][0].R = 0x00;
            Environment->dVoxels[i][j][0].G = 0xff;
            Environment->dVoxels[i][j][0].B = 0x00;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    UpdateSDF(Environment);
    WriteDVoxelArrayToFile(Environment, (char*)"IOC_test_environment1.dvxl");
    WriteDVoxelArrayToCSVFile(Environment, (char*)"IOC_test_environment1.csv");
    std::cout << "...Done Making IOC test environment 1" << std::endl;
}

void MakeIOCEnvironment2(float s1=3.0, float s2=2.0, float s3=1.0, float s4=1.0, float s5=2.0, float s6=3.0)
{
    std::cout << "Making IOC test environment 2..." << std::endl;
    dVoxelArray * Environment = AllocateDVoxelArray(84, 116, 1, 0, 0, 0, 0.0, 1.0);
    // First set
    for (int i = 0; i < 14; i++)
    {
        for (int j = 20; j < 40; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = s1;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0x00;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 28; i < 56; i++)
    {
        for (int j = 20; j < 40; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = s2;
            Environment->dVoxels[i][j][0].R = 0x00;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 70; i < 84; i++)
    {
        for (int j = 20; j < 40; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = s3;
            Environment->dVoxels[i][j][0].R = 0x00;
            Environment->dVoxels[i][j][0].G = 0xff;
            Environment->dVoxels[i][j][0].B = 0x00;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    // Second set
    for (int i = 0; i < 14; i++)
    {
        for (int j = 76; j < 96; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = s4;
            Environment->dVoxels[i][j][0].R = 0x00;
            Environment->dVoxels[i][j][0].G = 0xff;
            Environment->dVoxels[i][j][0].B = 0x00;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 28; i < 56; i++)
    {
        for (int j = 76; j < 96; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = s5;
            Environment->dVoxels[i][j][0].R = 0x00;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 70; i < 84; i++)
    {
        for (int j = 76; j < 96; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = s6;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0x00;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    UpdateSDF(Environment);
    WriteDVoxelArrayToFile(Environment, (char*)"IOC_test_environment2.dvxl");
    WriteDVoxelArrayToCSVFile(Environment, (char*)"IOC_test_environment2.csv");
    std::cout << "...Done Making IOC test environment 2" << std::endl;
}

void MakeIOCEnvironment3(float s1, float s2, float s3, float s4)
{
    std::cout << "Making IOC test environment 3..." << std::endl;
    dVoxelArray * Environment = AllocateDVoxelArray(84, 116, 1, 0, 0, 0, 0.0, 1.0);
    // First set
    for (int i = 15; i < 35; i++)
    {
        for (int j = 20; j < 50; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = s1;
            Environment->dVoxels[i][j][0].R = 0x36;
            Environment->dVoxels[i][j][0].G = 0xfb;
            Environment->dVoxels[i][j][0].B = 0xfb;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 49; i < 69; i++)
    {
        for (int j = 20; j < 50; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = s2;
            Environment->dVoxels[i][j][0].R = 0x36;
            Environment->dVoxels[i][j][0].G = 0xfb;
            Environment->dVoxels[i][j][0].B = 0xfb;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    // Second set
    for (int i = 15; i < 35; i++)
    {
        for (int j = 66; j < 96; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = s3;
            Environment->dVoxels[i][j][0].R = 0x36;
            Environment->dVoxels[i][j][0].G = 0xfb;
            Environment->dVoxels[i][j][0].B = 0xfb;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 49; i < 69; i++)
    {
        for (int j = 66; j < 96; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = s4;
            Environment->dVoxels[i][j][0].R = 0x36;
            Environment->dVoxels[i][j][0].G = 0xfb;
            Environment->dVoxels[i][j][0].B = 0xfb;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    UpdateSDF(Environment);
    WriteDVoxelArrayToFile(Environment, (char*)"IOC_test_environment3.dvxl");
    WriteDVoxelArrayToCSVFile(Environment, (char*)"IOC_test_environment3.csv");
    std::cout << "...Done Making IOC test environment 3" << std::endl;
}

void MakeDebugIOCEnvironment1()
{
    std::cout << "Making IOC test environment 1 [debug]..." << std::endl;
    dVoxelArray * Environment = AllocateDVoxelArray(84, 116, 1, 0, 0, 0, 0.0, 1.0);
    // First set
    for (int i = 0; i < 14; i++)
    {
        for (int j = 40; j < 76; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0x00;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 28; i < 56; i++)
    {
        for (int j = 40; j < 76; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 2.0;
            Environment->dVoxels[i][j][0].R = 0x00;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 70; i < 84; i++)
    {
        for (int j = 40; j < 76; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 3.0;
            Environment->dVoxels[i][j][0].R = 0x00;
            Environment->dVoxels[i][j][0].G = 0xff;
            Environment->dVoxels[i][j][0].B = 0x00;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    UpdateSDF(Environment);
    WriteDVoxelArrayToFile(Environment, (char*)"IOC_test_environment1_debug.dvxl");
    WriteDVoxelArrayToCSVFile(Environment, (char*)"IOC_test_environment1_debug.csv");
    std::cout << "...Done Making IOC test environment 1 [debug]" << std::endl;
}

void MakeDebugIOCEnvironment2()
{
    std::cout << "Making IOC test environment 2 [debug]..." << std::endl;
    dVoxelArray * Environment = AllocateDVoxelArray(84, 116, 1, 0, 0, 0, 0.0, 1.0);
    // First set
    for (int i = 0; i < 14; i++)
    {
        for (int j = 20; j < 40; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0x00;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 28; i < 56; i++)
    {
        for (int j = 20; j < 40; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 2.0;
            Environment->dVoxels[i][j][0].R = 0x00;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 70; i < 84; i++)
    {
        for (int j = 20; j < 40; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 3.0;
            Environment->dVoxels[i][j][0].R = 0x00;
            Environment->dVoxels[i][j][0].G = 0xff;
            Environment->dVoxels[i][j][0].B = 0x00;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    // Second set
    for (int i = 0; i < 14; i++)
    {
        for (int j = 76; j < 96; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 4.0;
            Environment->dVoxels[i][j][0].R = 0x00;
            Environment->dVoxels[i][j][0].G = 0xff;
            Environment->dVoxels[i][j][0].B = 0x00;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 28; i < 56; i++)
    {
        for (int j = 76; j < 96; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 5.0;
            Environment->dVoxels[i][j][0].R = 0x00;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0xff;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 70; i < 84; i++)
    {
        for (int j = 76; j < 96; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 6.0;
            Environment->dVoxels[i][j][0].R = 0xff;
            Environment->dVoxels[i][j][0].G = 0x00;
            Environment->dVoxels[i][j][0].B = 0x00;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    UpdateSDF(Environment);
    WriteDVoxelArrayToFile(Environment, (char*)"IOC_test_environment2_debug.dvxl");
    WriteDVoxelArrayToCSVFile(Environment, (char*)"IOC_test_environment2_debug.csv");
    std::cout << "...Done Making IOC test environment 2 [debug]" << std::endl;
}

void MakeDebugIOCEnvironment3()
{
    std::cout << "Making IOC test environment 3 [debug]..." << std::endl;
    dVoxelArray * Environment = AllocateDVoxelArray(84, 116, 1, 0, 0, 0, 0.0, 1.0);
    // First set
    for (int i = 15; i < 35; i++)
    {
        for (int j = 14; j < 34; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 1.0;
            Environment->dVoxels[i][j][0].R = 0x36;
            Environment->dVoxels[i][j][0].G = 0xfb;
            Environment->dVoxels[i][j][0].B = 0xfb;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 49; i < 69; i++)
    {
        for (int j = 14; j < 34; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 2.0;
            Environment->dVoxels[i][j][0].R = 0x36;
            Environment->dVoxels[i][j][0].G = 0xfb;
            Environment->dVoxels[i][j][0].B = 0xfb;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    // Second set
    for (int i = 15; i < 35; i++)
    {
        for (int j = 48; j < 68; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 3.0;
            Environment->dVoxels[i][j][0].R = 0x36;
            Environment->dVoxels[i][j][0].G = 0xfb;
            Environment->dVoxels[i][j][0].B = 0xfb;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    for (int i = 49; i < 69; i++)
    {
        for (int j = 48; j < 68; j++)
        {
            Environment->dVoxels[i][j][0].deformability = 0.5;
            Environment->dVoxels[i][j][0].sensitivity = 4.0;
            Environment->dVoxels[i][j][0].R = 0x36;
            Environment->dVoxels[i][j][0].G = 0xfb;
            Environment->dVoxels[i][j][0].B = 0xfb;
            Environment->dVoxels[i][j][0].A = 0xff;
        }
    }
    UpdateSDF(Environment);
    WriteDVoxelArrayToFile(Environment, (char*)"IOC_test_environment3_debug.dvxl");
    WriteDVoxelArrayToCSVFile(Environment, (char*)"IOC_test_environment3_debug.csv");
    std::cout << "...Done Making IOC test environment 3 [debug]" << std::endl;
}

void GenerateIOCRobot()
{
    std::cout << "Making IOC Robot..." << std::endl;
    dVoxelArray * Robot = AllocateDVoxelArray(20, 20, 1, 0, 0, 0, 1.0, 0.0);
    WriteDVoxelArrayToFile(Robot, (char*)"IOCbot.dvxl");
    WriteDVoxelArrayToCSVFile(Robot, (char*)"IOCbot.csv");
    std::cout << "...Done Making IOC Robot" << std::endl;
}

void GenerateRandomCostEnvironment(std::string filename, std::string csv_filename, int num_obstacle_blocks)
{
    srand(time(NULL));
    dVoxelArray * Environment = AllocateDVoxelArray(100, 100, 1, 0, 0, 0, 0.0, 1.0);
    // Add "random" obstacles to the environment to make it interesting
    // [0,0] to [19,99] and [80,0] to [99,99] are reserved as start and end zones
    // All of our obstacles are 10x10 blocks, we randomly pic start indices
    for (int i = 0; i < num_obstacle_blocks; i++)
    {
        //int randx = random_in_range(20, 70);
        int randx = random_in_range(-10, 100);
        int randy = random_in_range(-10, 100);
        AddObstacleBox(Environment, randx, randy, 10, 10, 0.5, 1.0, 0xff, 0x00, 0x00);
    }
    WriteDVoxelArrayToFile(Environment, (char*)filename.c_str());
    // Make simpler costmap variant of the dvxl grid
    WriteDVoxelArrayToCSVFile(Environment, (char*)csv_filename.c_str());
}

void GenerateRandomNarrowPassageEnvironment(std::string filename, std::string csv_filename, int num_narrow_passages)
{
    srand(time(NULL));
    dVoxelArray * Environment = AllocateDVoxelArray(100, 100, 1, 0, 0, 0, 0.0, 1.0);
    // Add "random" obstacles to the environment to make it interesting
    // [0,0] to [19,99] and [80,0] to [99,99] are reserved as start and end zones
    // All of our obstacles are 10x10 blocks, we randomly pic start indices
    std::vector<std::pair<int,int> > blacklist;
    int i = 0;
    int minimum_buffer = 5;
    while (i < num_narrow_passages)
    {
        // Sample 4 parameters that we can use to define narrow passages
        int startingX = random_in_range(20, 70);
        int centerY = random_in_range(0, 100);
        int depth = random_in_range(4, 20);
        int width = random_in_range(6, 20); //used to be 2, but we want to avoid infeasibility!
        // Check if this would overlap with an earlier narrow passage
        bool overlap = false;
        std::cout << "New passage with <" << startingX << "," << depth << ">" << std::endl;
        for (int idx = 0; idx < (int)blacklist.size(); idx++)
        {
            std::pair<int, int> passage_signature = blacklist[idx];
            std::cout << "Checking against narrow passage <" << passage_signature.first << "," << passage_signature.second << ">" << std::endl;
            if ((startingX >= (passage_signature.first - minimum_buffer)) && (startingX <= (passage_signature.first + passage_signature.second + minimum_buffer)))
            {
                std::cout << "Rejecting due to start inside another passage" << std::endl;
                overlap = true;
                break;
            }
            else if (((startingX + depth) >= (passage_signature.first - minimum_buffer)) && ((startingX + depth) <= (passage_signature.first + passage_signature.second + minimum_buffer)))
            {
                std::cout << "Rejecting due to end inside another passage" << std::endl;
                overlap = true;
                break;
            }
            else if ((passage_signature.first >= (startingX - minimum_buffer)) && (passage_signature.first <= (startingX + depth + minimum_buffer)))
            {
                std::cout << "Rejecting due to ahother passage starting inside us" << std::endl;
                overlap = true;
                break;
            }
            else if (((passage_signature.first + passage_signature.second) >= (startingX - minimum_buffer)) && ((passage_signature.first + passage_signature.second) <= (startingX + depth + minimum_buffer)))
            {
                std::cout << "Rejecting due to another passage ending inside us" << std::endl;
                overlap = true;
                break;
            }
        }
        if (overlap == false)
        {
            std::cout << "Adding passage with <" << startingX << "," << depth << ">" << std::endl;
            // Save the current passage
            std::pair<int, int> passage_signature(startingX, depth);
            blacklist.push_back(passage_signature);
            // These parameters define two boxes that create the narrow passage
            int s1x = startingX;
            int s1y = 0;
            int e1x = depth;
            int e1y = centerY - int(width / 2.0);
            int s2x = startingX;
            int s2y = centerY + int(width / 2.0);
            int e2x = depth;
            int e2y = 100 - s2y;
            AddObstacleBox(Environment, s1x, s1y, e1x, e1y, 0.0, 1.0, 0xff, 0x00, 0x00);
            AddObstacleBox(Environment, s2x, s2y, e2x, e2y, 0.0, 1.0, 0xff, 0x00, 0x00);
            i++;
        }
    }
    // Compute the SDF for the environment
    struct timespec st, et;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &st);
    UpdateSDF(Environment);
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &et);
    float secs = (float)(et.tv_sec - st.tv_sec);
    secs = secs + (float)(et.tv_nsec - st.tv_nsec) / 1000000000.0;
    std::cout << "SDF computation took " << secs << " seconds" << std::endl;
    WriteDVoxelArrayToFile(Environment, (char*)filename.c_str());
    // Make simpler costmap variant of the dvxl grid
    WriteDVoxelArrayToCSVFile(Environment, (char*)csv_filename.c_str());
}

void GenerateRobot(std::string filename, std::string csv_filename)
{
    dVoxelArray * Robot = AllocateDVoxelArray(9, 9, 1, 4, 4, 0, 1.0, 0.5);
    // Make the center rigid
    Robot->dVoxels[4][4][0].deformability = 0.0;
    // Set sensitivity to increase towards the center
    for (int i = 0; i < Robot->xDim; i++)
    {
        for (int j = 0; j < Robot->yDim; j++)
        {
            // Check the edges to set sensitivity to 1.0
            if (i == 0 || i == (Robot->xDim - 1) || j == 0 || j == (Robot->yDim - 1))
            {
                Robot->dVoxels[i][j][0].sensitivity = 1.0;
            }
            // Check one in from the edges to set sensitivity to 3.0
            else if (i == 1 || i == (Robot->xDim - 2) || j == 1 || j == (Robot->yDim - 2))
            {
                Robot->dVoxels[i][j][0].sensitivity = 3.0;
            }
            // Check two in from the edges to set sensitivity to 5.0
            else if (i == 2 || i == (Robot->xDim - 3) || j == 2 || j == (Robot->yDim - 3))
            {
                Robot->dVoxels[i][j][0].sensitivity = 5.0;
            }
            // Check two in from the edges to set sensitivity to 5.0
            else if (i == 3 || i == (Robot->xDim - 4) || j == 3 || j == (Robot->yDim - 4))
            {
                Robot->dVoxels[i][j][0].sensitivity = 7.0;
            }
        }
    }
    // Compute the SDF for the environment
    struct timespec st, et;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &st);
    UpdateSDF(Robot);
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &et);
    float secs = (float)(et.tv_sec - st.tv_sec);
    secs = secs + (float)(et.tv_nsec - st.tv_nsec) / 1000000000.0;
    std::cout << "SDF computation took " << secs << " seconds" << std::endl;
    WriteDVoxelArrayToFile(Robot, (char*)filename.c_str());
    WriteDVoxelArrayToCSVFile(Robot, (char*)csv_filename.c_str());
}

float DistanceHeuristic2D(state6D * stateToEval, state6D * goalState, int debug_level)
{
    float deltaX = goalState->XT - stateToEval->XT;
    float deltaY = goalState->YT - stateToEval->YT;
    float distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
    debug(debug_level, 3, "Assigned heuristic value: %f\n", distance);
    return distance;
}

float DistanceHeuristic2DR(state6D * stateToEval, state6D * goalState, int debug_level)
{
    float deltaX = goalState->XT - stateToEval->XT;
    float deltaY = goalState->YT - stateToEval->YT;
    float deltaR = goalState->ZR - stateToEval->ZR;
    float distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaR, 2));
    debug(debug_level, 3, "Assigned heuristic value: %f\n", distance);
    return distance;
}

StateList * GenerateChildren2D(state6D * state, int debug_level)
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
    float k = state->ZT;
    for (float i = (state->XT - 1.0); i <= (state->XT + 1.0); i+=1.0)
    {
        for (float j = (state->YT - 1.0); j <= (state->YT + 1.0); j+=1.0)
        {
            //Check if we're within the acceptable bounds of the environment
            if (i >= 0.0 && j >= 0.0 && i < Xmax && j < Ymax)
            {
                //Check to make sure we're not duplicating the parent
                if (i != state->XT || j != state->YT)
                {
                    float xdiff = i - state->XT;
                    float ydiff = j - state->YT;
                    float diff = sqrt(pow(xdiff, 2) + pow(ydiff, 2));
                    state6D * childState = NewState6D(state, state->RefShape, state->Environment, i, j, k, 0.0, 0.0, 0.0, diff);
                    StateAppend(children, childState);
                }
            }
        }
    }
    debug(debug_level, 2, "Generated %d possible new children\n", children->Length);
    return children;
}

StateList * GenerateChildren2DR(state6D * state, int debug_level)
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
    float k = state->ZT;
    for (float i = (state->XT - 1.0); i <= (state->XT + 1.0); i+=1.0)
    {
        for (float j = (state->YT - 1.0); j <= (state->YT + 1.0); j+=1.0)
        {
            for (float zr = (state->ZR - ROTATION_STEP); zr <= (state->ZR + ROTATION_STEP + 0.1); zr+=ROTATION_STEP)
            {
                //Check if we're within the acceptable bounds of the environment
                if (i >= 0.0 && j >= 0.0 && i < Xmax && j < Ymax && zr >= -(M_PI_2 + 0.1) && zr <= (M_PI_2 + 0.1))
                {
                    //Check to make sure we're not duplicating the parent
                    if (i != state->XT || j != state->YT || (CLOSE_ENOUGH(zr, state->ZR) == 0))
                    {
                        float xdiff = i - state->XT;
                        float ydiff = j - state->YT;
                        float rdiff = zr - state->ZR;
                        float diff = sqrt(pow(xdiff, 2) + pow(ydiff, 2) + pow(rdiff, 2));
                        state6D * childState = NewState6D(state, state->RefShape, state->Environment, i, j, k, 0.0, 0.0, zr, diff);
                        StateAppend(children, childState);
                    }
                }
            }
        }
    }
    debug(debug_level, 2, "Generated %d possible new children\n", children->Length);
    return children;
}

void PlanIOCPath(float xstart, float ystart, float xgoal, float ygoal, float p, std::string environment, std::string robot, std::string filename, int debug=0)
{
    //Create environment and robot
    dVoxelArray * Environment = LoadDVoxelArrayFromFile((char*)environment.c_str());
    dVoxelArray * Robot = LoadDVoxelArrayFromFile((char*)robot.c_str());
    //Create start and goal states
    state6D * startState = NewState6D(NULL, Robot, Environment, xstart, ystart, 0.0, 0.0, 0.0, 0.0, 0.0);
    state6D * goalState = NewState6D(NULL, Robot, Environment, xgoal, ygoal, 0.0, 0.0, 0.0, 0.0, 0.0);
    if (EvaluateState(startState, goalState, 0.5, DistanceHeuristic2D, 3) == NULL)
    {
        std::cout << "FATAL ERROR - start state is not feasible! - exiting" << std::endl;
        exit(1);
    }
    if (EvaluateState(goalState, goalState, 0.5, DistanceHeuristic2D, 3) == NULL)
    {
        std::cout << "FATAL ERROR - goal state is not feasible! - exiting" << std::endl;
        exit(1);
    }
    std::cout << "Running Astar" << std::endl;
    float ParetoWeight = p;
    int debug_level = debug;
    StateList * path = AstarSearch(startState, goalState, ParetoWeight, DistanceHeuristic2D, GenerateChildren2D, debug_level);
    if (path == NULL)
    {
        std::cout << "FATAL ERROR - unable to plan! - exiting" << std::endl;
        exit(1);
    }
    std::cout << "Path planned with " << path->Length << " states" << std::endl;
    // Convert to XTF trajectory
    std::vector<std::string> joints;
    joints.push_back("X");
    joints.push_back("Y");
    std::vector<std::string> tags;
    tags.push_back(environment);
    tags.push_back(robot);
    double prev_cost = 0.0;
    XTF::Trajectory stored_path(filename, XTF::Trajectory::GENERATED, XTF::Trajectory::UNTIMED, robot, "ioc_planner2d", joints, tags);
    for (int i = 0; i < path->Length; i++)
    {
        state6D * PN = GetIndex(path, i);
        std::vector<double> dp;
        dp.push_back(PN->XT);
        dp.push_back(PN->YT);
        std::vector<double> dv;
        std::vector<double> da;
        std::vector<double> ap;
        std::vector<double> av;
        std::vector<double> aa;
        struct timespec timing;
        XTF::State pstate(dp, dv, da, ap, av, aa, i, timing);
        XTF::KeyValue costextra(PN->COST);
        XTF::KeyValue coststep(PN->COST - prev_cost);
        XTF::KeyValue distance(PN->DIST);
        prev_cost = PN->COST;
        pstate.extras_["deformcost"] = costextra;
        pstate.extras_["coststep"] = coststep;
        pstate.extras_["distance"] = distance;
        stored_path.push_back(pstate);
    }
    // Save the XTF trajectory
    XTF::Parser parser;
    parser.ExportTraj(stored_path, filename);
    // Cleanup
    DestroyDVoxelArray(Environment);
    DestroyDVoxelArray(Robot);
    DestroyState6D(startState);
    DestroyState6D(goalState);
}

void PlanPath(float xstart, float ystart, float zrstart, float xgoal, float ygoal, float zrgoal, std::string environment, std::string robot, std::string filename, int debug=0)
{
    //Create environment and robot
    dVoxelArray * Environment = LoadDVoxelArrayFromFile((char*)environment.c_str());
    dVoxelArray * Robot = LoadDVoxelArrayFromFile((char*)robot.c_str());
    //Create start and goal states
    state6D * startState = NewState6D(NULL, Robot, Environment, xstart, ystart, 0.0, 0.0, 0.0, zrstart, 0.0);
    state6D * goalState = NewState6D(NULL, Robot, Environment, xgoal, ygoal, 0.0, 0.0, 0.0, zrgoal, 0.0);
    if (EvaluateState(startState, goalState, 0.5, DistanceHeuristic2D, 3) == NULL)
    {
        std::cout << "FATAL ERROR - start state is not feasible! - exiting" << std::endl;
        exit(1);
    }
    if (EvaluateState(goalState, goalState, 0.5, DistanceHeuristic2D, 3) == NULL)
    {
        std::cout << "FATAL ERROR - goal state is not feasible! - exiting" << std::endl;
        exit(1);
    }
    std::cout << "Running Astar" << std::endl;
    float ParetoWeight = 0.5;
    int debug_level = debug;
    StateList * path = AstarSearch(startState, goalState, ParetoWeight, DistanceHeuristic2DR, GenerateChildren2DR, debug_level);
    if (path == NULL)
    {
        std::cout << "FATAL ERROR - unable to plan! - exiting" << std::endl;
        exit(1);
    }
    std::cout << "Path planned with " << path->Length << " states" << std::endl;
    // Convert to XTF trajectory
    std::vector<std::string> joints;
    joints.push_back("X");
    joints.push_back("Y");
    joints.push_back("ZR");
    std::vector<std::string> tags;
    tags.push_back(environment);
    double prev_cost = 0.0;
    XTF::Trajectory stored_path(filename, XTF::Trajectory::GENERATED, XTF::Trajectory::UNTIMED, robot, "planner2d", joints, tags);
    for (int i = 0; i < path->Length; i++)
    {
        state6D * PN = GetIndex(path, i);
        std::vector<double> dp;
        dp.push_back(PN->XT);
        dp.push_back(PN->YT);
        dp.push_back(PN->ZR);
        std::vector<double> dv;
        std::vector<double> da;
        std::vector<double> ap;
        std::vector<double> av;
        std::vector<double> aa;
        struct timespec timing;
        XTF::State pstate(dp, dv, da, ap, av, aa, i, timing);
        XTF::KeyValue costextra(PN->COST);
        XTF::KeyValue coststep(PN->COST - prev_cost);
        prev_cost = PN->COST;
        pstate.extras_["deformcost"] = costextra;
        pstate.extras_["coststep"] = coststep;
        stored_path.push_back(pstate);
    }
    // Save the XTF trajectory
    XTF::Parser parser;
    parser.ExportTraj(stored_path, filename);
    // Cleanup
    DestroyDVoxelArray(Environment);
    DestroyDVoxelArray(Robot);
    DestroyState6D(startState);
    DestroyState6D(goalState);
}

void ComputeStateCost(float x, float y, float zr, std::string environment, std::string robot)
{
    //Create environment and robot
    dVoxelArray * Environment = LoadDVoxelArrayFromFile((char*)environment.c_str());
    dVoxelArray * Robot = LoadDVoxelArrayFromFile((char*)robot.c_str());
    state6D * State = NewState6D(NULL, Robot, Environment, x, y, 0.0, 0.0, 0.0, zr, 0.0);
    state6D * assessed = EvaluateState(State, State, 0.5, DistanceHeuristic2D, 3);
    if (assessed == NULL)
    {
        std::cout << "FATAL ERROR - state is not feasible! - exiting" << std::endl;
        exit(1);
    }
    else
    {
        std::cout << "COST: " << assessed->COST << std::endl;
    }
}

void ComputeStateGradient(float xt, float yt, float zr, std::string environment, std::string robot)
{
    //Create environment and robot
    dVoxelArray * Environment = LoadDVoxelArrayFromFile((char*)environment.c_str());
    dVoxelArray * Robot = LoadDVoxelArrayFromFile((char*)robot.c_str());
    state6D * State = NewState6D(NULL, Robot, Environment, xt, yt, 0.0, 0.0, 0.0, zr, 0.0);
    state6D * assessed = EvaluateState(State, State, 0.5, DistanceHeuristic2D, 3);
    if (assessed == NULL)
    {
        std::cout << "FATAL ERROR - state is not feasible! - exiting" << std::endl;
        exit(1);
    }
    else
    {
        // Actually compute the gradient from the SDF
        Eigen::Vector3d gradient;
        for (int x = 0; x < Robot->xDim; x++)
        {
            for (int y = 0; y < Robot->yDim; y++)
            {
                for (int z = 0; z < Robot->zDim; z++)
                {
                    double* rpoint = TransformPoint(xt, yt, 0.0, 0.0, 0.0, zr, x - Robot->xCenter, y - Robot->yCenter, z - Robot->zCenter);
                    int rx = snap(rpoint[0]);
                    int ry = snap(rpoint[1]);
                    int rz = snap(rpoint[2]);
                    free(rpoint);
                    Eigen::Vector3d vxl_gradient = Get2DGradientFromSDF(Environment, rx, ry, rz);
                    gradient += vxl_gradient;
                }
            }
        }
        // Any additional processing of the gradient
        double cx = gradient.x();
        if (fabs(cx) < 0.00001)
        {
            cx = 0.0;
        }
        double cy = gradient.y();
        if (fabs(cy) < 0.00001)
        {
            cy = 0.0;
        }
        double cz = gradient.z();
        if (fabs(cz) < 0.00001)
        {
            cz = 0.0;
        }
        std::cout << "GRADIENT: " << cx << "|" << cy << "|" << cz << std::endl;
    }
}

void ComputeStateOverlap(float xt, float yt, float zr, std::string environment, std::string robot)
{
    //Create environment and robot
    dVoxelArray * Environment = LoadDVoxelArrayFromFile((char*)environment.c_str());
    dVoxelArray * Robot = LoadDVoxelArrayFromFile((char*)robot.c_str());
    state6D * State = NewState6D(NULL, Robot, Environment, xt, yt, 0.0, 0.0, 0.0, zr, 0.0);
    state6D * assessed = EvaluateState(State, State, 0.5, DistanceHeuristic2D, 3);
    if (assessed == NULL)
    {
        std::cout << "FATAL ERROR - state is not feasible! - exiting" << std::endl;
        exit(1);
    }
    else
    {
        std::map<float, uint32_t> overlap = EvaluateStateOverlap(State, 3);
        std::cout << "OVERLAP: " << PrettyPrint::PrettyPrint(overlap, true) << std::endl;
    }
}

void ComputeStateProximity(float xt, float yt, float zr, std::string environment, std::string robot)
{
    //Create environment and robot
    dVoxelArray * Environment = LoadDVoxelArrayFromFile((char*)environment.c_str());
    dVoxelArray * Robot = LoadDVoxelArrayFromFile((char*)robot.c_str());
    state6D * State = NewState6D(NULL, Robot, Environment, xt, yt, 0.0, 0.0, 0.0, zr, 0.0);
    state6D * assessed = EvaluateState(State, State, 0.5, DistanceHeuristic2D, 3);
    if (assessed == NULL)
    {
        std::cout << "FATAL ERROR - state is not feasible! - exiting" << std::endl;
        exit(1);
    }
    else
    {
        // Find the closest point of the robot (from the SDF)
        float proximity = INFINITY;
        for (int x = 0; x < Robot->xDim; x++)
        {
            for (int y = 0; y < Robot->yDim; y++)
            {
                for (int z = 0; z < Robot->zDim; z++)
                {
                    double* rpoint = TransformPoint(xt, yt, 0.0, 0.0, 0.0, zr, x - Robot->xCenter, y - Robot->yCenter, z - Robot->zCenter);
                    int rx = snap(rpoint[0]);
                    int ry = snap(rpoint[1]);
                    int rz = snap(rpoint[2]);
                    free(rpoint);
                    float sdf_val = Environment->dVoxels[rx][ry][rz].SDF;
                    if (sdf_val < proximity)
                    {
                        proximity = sdf_val;
                    }
                }
            }
        }
        std::cout << "PROXIMITY: " << proximity << std::endl;
    }
}

int main(int argc, char** argv)
{
    // Check the arguments first
    if (argc < 2)
    {
        dVoxelArray* Environment = AllocateDVoxelArray(10, 10, 1, 0, 0, 0, 0.0, 1.0);
        AddObstacleBox(Environment, 4, 0, 2, 4, 0.0, 1.0, 0xff, 0x00, 0x00);
        AddObstacleBox(Environment, 4, 6, 4, 4, 0.0, 1.0, 0xff, 0x00, 0x00);
        UpdateSDF(Environment);
        WriteDVoxelArrayToFile(Environment, (char*)"rotationTestEnv.dvxl");
        WriteDVoxelArrayToCSVFile(Environment, (char*)"rotationTestEnv.csv");
        dVoxelArray* Robot = AllocateDVoxelArray(2,3,1,0,0,0,1.0,0.0);
        WriteDVoxelArrayToFile(Robot, (char*)"rotationTestBot.dvxl");
        WriteDVoxelArrayToCSVFile(Robot, (char*)"rotationTestBot.csv");
        PlanPath(0.0, 0.0, 0.0, 8.0, 6.0, 0.0, "rotationTestEnv.dvxl", "rotationTestBot.dvxl", "rotationTest.xtf", 0);
        /*
        double sx = 10.0;
        double sy = 10.0;
        double sz = 10.0;
        double tx = 10.0;
        double ty = 10.0;
        double tz = 10.0;
        double rx = 0.0;
        double ry = 0.0;
        double rz = M_PI_4;
        double* test = TransformPoint(tx, ty, tz, rx, ry, rz, sx, sy, sz);
        printf("Transformed point (%f,%f,%f) to (%f,%f,%f)\n", sx, sy, sz, test[0], test[1], test[2]);
        free(test);
        std::cout << "Size of dVoxel struct: " << sizeof(dVoxel) << std::endl;
        struct timespec st, et;
        dVoxelArray * Environment = AllocateDVoxelArray(100, 100, 1, 0, 0, 0, 0.0, 1.0);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &st);
        UpdateSDF(Environment);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &et);
        float secs = (float)(et.tv_sec - st.tv_sec);
        secs = secs + (float)(et.tv_nsec - st.tv_nsec) / 1000000000.0;
        std::cout << "SDF computation took " << secs << " seconds" << std::endl;
        WriteDVoxelArrayToCSVFile(Environment, (char*)"example_w_SDF.csv");
        */
        std::cout << "This program must be run with at least 1 argument(s) - none provided!" << std::endl;
        exit(1);
    }
    else
    {
        if (std::string(argv[1]) == std::string("-e"))
        {
            if (argc >= 5)
            {
                std::string filename(argv[2]);
                std::string csv_filename(argv[3]);
                int num_obstacles = atoi(argv[4]);
                GenerateRandomNarrowPassageEnvironment(filename, csv_filename, num_obstacles);
                return 0;
            }
            else
            {
                std::cout << "Too few arguments provided for environment generation mode" << std::endl;
                exit(1);
            }
        }
        else if (std::string(argv[1]) == std::string("-s"))
        {
            if (argc >= 7)
            {
                std::string in_filename(argv[2]);
                std::string out_filename(argv[3]);
                std::string out_csv_filename(argv[4]);
                int xshift = atoi(argv[5]);
                int yshift = atoi(argv[6]);
                OffsetEnvironment(in_filename, out_filename, out_csv_filename, xshift, yshift);
                return 0;
            }
            else
            {
                std::cout << "Too few arguments provided for environment shift mode" << std::endl;
                exit(1);
            }
        }
        else if (std::string(argv[1]) == std::string("-sb"))
        {
            if (argc >= 7)
            {
                std::string in_filename(argv[2]);
                std::string out_filename(argv[3]);
                std::string out_csv_filename(argv[4]);
                int xshift = atoi(argv[5]);
                int yshift = atoi(argv[6]);
                OffsetEnvironmentBlocks(in_filename, out_filename, out_csv_filename, xshift, yshift);
                return 0;
            }
            else
            {
                std::cout << "Too few arguments provided for environment shift (block) mode" << std::endl;
                exit(1);
            }
        }
        else if (std::string(argv[1]) == std::string("-r"))
        {
            if (argc >= 4)
            {
                std::string filename(argv[2]);
                std::string csv_filename(argv[3]);
                GenerateRobot(filename, csv_filename);
                return 0;
            }
            else
            {
                std::cout << "Too few arguments provided for robot generation mode" << std::endl;
                exit(1);
            }
        }
        else if (std::string(argv[1]) == std::string("-p"))
        {
            if (argc == 11)
            {
                float xs = atof(argv[2]);
                float ys = atof(argv[3]);
                float zrs = atof(argv[4]);
                float xg = atof(argv[5]);
                float yg = atof(argv[6]);
                float zrg = atof(argv[7]);
                std::string environment(argv[8]);
                std::string robot(argv[9]);
                std::string filename(argv[10]);
                PlanPath(xs, ys, zrs, xg, yg, zrg, environment, robot, filename, 0);
                return 0;
            }
            else if (argc >= 12)
            {
                float xs = atof(argv[2]);
                float ys = atof(argv[3]);
                float zrs = atof(argv[4]);
                float xg = atof(argv[5]);
                float yg = atof(argv[6]);
                float zrg = atof(argv[7]);
                std::string environment(argv[8]);
                std::string robot(argv[9]);
                std::string filename(argv[10]);
                int debug_level = atoi(argv[11]);
                PlanPath(xs, ys, zrs, xg, yg, zrg, environment, robot, filename, debug_level);
                return 0;
            }
            else
            {
                std::cout << "Too few arguments provided for planning mode" << std::endl;
                exit(1);
            }
        }
        else if (std::string(argv[1]) == std::string("-c"))
        {
            if (argc >= 7)
            {
                float x = atof(argv[2]);
                float y = atof(argv[3]);
                float zr = atof(argv[4]);
                std::string environment(argv[5]);
                std::string robot(argv[6]);
                ComputeStateCost(x, y, zr, environment, robot);
                return 0;
            }
            else
            {
                std::cout << "Too few arguments provided for cost assessment mode" << std::endl;
                exit(1);
            }
        }
        else if (std::string(argv[1]) == std::string("-g"))
        {
            if (argc >= 7)
            {
                float x = atof(argv[2]);
                float y = atof(argv[3]);
                float zr = atof(argv[4]);
                std::string environment(argv[5]);
                std::string robot(argv[6]);
                ComputeStateGradient(x, y, zr, environment, robot);
                return 0;
            }
            else
            {
                std::cout << "Too few arguments provided for gradient assessment mode" << std::endl;
                exit(1);
            }
        }
        else if (std::string(argv[1]) == std::string("-ov"))
        {
            if (argc >= 7)
            {
                float x = atof(argv[2]);
                float y = atof(argv[3]);
                float zr = atof(argv[4]);
                std::string environment(argv[5]);
                std::string robot(argv[6]);
                ComputeStateOverlap(x, y, zr, environment, robot);
                return 0;
            }
            else
            {
                std::cout << "Too few arguments provided for overlap assessment mode" << std::endl;
                exit(1);
            }
        }
        else if (std::string(argv[1]) == std::string("-px"))
        {
            if (argc >= 7)
            {
                float x = atof(argv[2]);
                float y = atof(argv[3]);
                float zr = atof(argv[4]);
                std::string environment(argv[5]);
                std::string robot(argv[6]);
                ComputeStateProximity(x, y, zr, environment, robot);
                return 0;
            }
            else
            {
                std::cout << "Too few arguments provided for proximity assessment mode" << std::endl;
                exit(1);
            }
        }
        else if (std::string(argv[1]) == std::string("-mioc"))
        {
            if (argc >= 3)
            {
                int env = atoi(argv[2]);
                if (env == 1)
                {
                    if (argc >= 6)
                    {
                        float s1 = atof(argv[3]);
                        float s2 = atof(argv[4]);
                        float s3 = atof(argv[5]);
                        MakeIOCEnvironment1(s1, s2, s3);
                    }
                    else
                    {
                        std::cout << "Making IOC environment 1 with default values" << std::endl;
                        MakeIOCEnvironment1();
                    }
                }
                else if (env == 2)
                {
                    if (argc >= 9)
                    {
                        float s1 = atof(argv[3]);
                        float s2 = atof(argv[4]);
                        float s3 = atof(argv[5]);
                        float s4 = atof(argv[6]);
                        float s5 = atof(argv[7]);
                        float s6 = atof(argv[8]);
                        MakeIOCEnvironment2(s1, s2, s3, s4, s5, s6);
                    }
                    else
                    {
                        std::cout << "Making IOC environment 2 with default values" << std::endl;
                        MakeIOCEnvironment2();
                    }
                }
                else if (env == 3)
                {
                    if (argc >= 7)
                    {
                        float s1 = atof(argv[3]);
                        float s2 = atof(argv[4]);
                        float s3 = atof(argv[5]);
                        float s4 = atof(argv[6]);
                        MakeIOCEnvironment3(s1, s2, s3, s4);
                    }
                    else
                    {
                        std::cout << "Making IOC environment 3 with default values" << std::endl;
                        MakeIOCEnvironment3(1.0, 1.0, 1.0, 1.0);
                    }
                }
                else
                {
                    std::cout << "Only 1 to 3 are valid IOC environment options" << std::endl;
                }
                return 0;
            }
            else
            {
                std::cout << "Too few arguments provided for IOC environment generation mode" << std::endl;
                exit(1);
            }
        }
        else if (std::string(argv[1]) == std::string("-mdioc"))
        {
            if (argc >= 3)
            {
                int env = atoi(argv[2]);
                if (env == 1)
                {
                    MakeDebugIOCEnvironment1();
                }
                else if (env == 2)
                {
                    MakeDebugIOCEnvironment2();
                }
                else if (env == 3)
                {
                    MakeDebugIOCEnvironment3();
                }
                else
                {
                    std::cout << "Only 1 to 3 are valid IOC debug environment options" << std::endl;
                }
                return 0;
            }
            else
            {
                std::cout << "Too few arguments provided for IOC debug environment generation mode" << std::endl;
                exit(1);
            }
        }
        else if (std::string(argv[1]) == std::string("-rioc"))
        {
            GenerateIOCRobot();
        }
        else if (std::string(argv[1]) == std::string("-pioc"))
        {
            if (argc == 11)
            {
                float xs = atof(argv[2]);
                float ys = atof(argv[3]);
                float xg = atof(argv[4]);
                float yg = atof(argv[5]);
                float p = atof(argv[6]);
                std::string environment(argv[7]);
                std::string robot(argv[8]);
                std::string filename(argv[9]);
                int debug_level = atoi(argv[10]);
                PlanIOCPath(xs, ys, xg, yg, p, environment, robot, filename, debug_level);
                return 0;
            }
            else
            {
                std::cout << "Too few arguments provided for IOC planning mode" << std::endl;
                exit(1);
            }
        }
        else if (std::string(argv[1]) == std::string("-m"))
        {
            if (argc >= 3)
            {
                int env = atoi(argv[2]);
                if (env == 1)
                {
                    MakeTestEnvironment1();
                }
                else if (env == 2)
                {
                    MakeTestEnvironment2();
                }

                else if (env == 3)
                {
                    MakeTestEnvironment3();
                }
                else if (env == 4)
                {
                    MakeTestEnvironment4();
                }
                else if (env == 5)
                {
                    MakeTestEnvironmentShifted();
                }
                else if (env == 6)
                {
                    MakeTestEnvironmentSpecialShifted();
                }
                else
                {
                    std::cout << "Only 1 to 6 are valid TestEnv options" << std::endl;
                }
                return 0;
            }
            else
            {
                std::cout << "Too few arguments provided for TestEnv generation mode" << std::endl;
                exit(1);
            }
        }
        else if (std::string(argv[1]) == std::string("-b"))
        {
            GenerateIROSRobot();
            return 0;
        }
        else
        {
            std::cout << "Invalid operation flag: " << argv[1] << std::endl;
            exit(1);
        }
    }
}
