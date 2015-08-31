#include "stdlib.h"
#include "stdio.h"
#include "stdint.h"
#include "math.h"
#include "vector"
#include "Eigen/Geometry"

#ifndef D_VOXEL_H
#define D_VOXEL_H

#define CLOSE_ENOUGH(a, b) (fabs(a - b)<=0.0001?(1):(0))
#define snap(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

typedef struct d_voxel {
    float deformability;
    float sensitivity;
    float SDF;
    uint8_t R;
    uint8_t G;
    uint8_t B;
    uint8_t A;
} dVoxel;

typedef struct d_voxel_array {
    dVoxel ***dVoxels;
    int xDim;
    int yDim;
    int zDim;
    int xCenter;
    int yCenter;
    int zCenter;
    float defaultSensitivity;
    float defaultDeformability;
} dVoxelArray;

typedef dVoxelArray* dVoxelArrayPtr;

typedef struct field_cell {
    float dx;
    float dy;
    float dz;
} cell;

typedef struct distance_field {
    cell ***cells;
    cell empty_cell;
    int xDim;
    int yDim;
    int zDim;
} field;

typedef struct sdf_bucket_cell {
    int location[3];
    int closest_point[3];
    double distance_square;
    int update_direction;
} bucket_cell;

typedef struct bucket_distance_field {
    bucket_cell ***cells;
    int xDim;
    int yDim;
    int zDim;
} bucket_field;

dVoxelArray * AllocateDVoxelArray(int Xdim, int Ydim, int Zdim, int Xcenter, int Ycenter, int Zcenter, float defaultSensitivity, float defaultDeformability);

int DestroyDVoxelArray(dVoxelArray * arrayToDestroy);

int WriteDVoxelArrayToFile(dVoxelArray * arrayToWrite, char* filename);

int WriteDVoxelArrayToCSVFile(dVoxelArray * arrayToWrite, char* filename);

dVoxelArray * LoadDVoxelArrayFromFile(char* filename);

dVoxelArray * CloneDVoxelArray(dVoxelArray * arrayToClone);

int UpdateSDF(dVoxelArray* arrayToUpdate);

Eigen::Vector3d GetGradientFromSDF(dVoxelArray * environment, int x, int y, int z);

Eigen::Vector3d Get2DGradientFromSDF(dVoxelArray * environment, int x, int y, int z);

void UpdateIntermediateField(field* fieldToUpdate);

double * TransformPoint(double tx, double ty, double tz, double rx, double ry, double rz, double px, double py, double pz);

double * RotatePoint(double X, double Y, double Z, double XR, double YR, double ZR, double XC, double YC, double ZC);

dVoxelArray * TransformMergeDVoxelArrays(dVoxelArray * world, dVoxelArray * object, float Xt, float Yt, float Zt, float Xr, float Yr, float Zr, uint8_t alpha_threshold);

dVoxelArray * Recolor(dVoxelArray * object, uint8_t R, uint8_t G, uint8_t B, uint8_t alpha_threshold, int copy_first);

bucket_field * BuildPropagationField(std::vector<Eigen::Vector3i>& points, int xDim, int yDim, int zDim);

field * CreateField(int xDim, int yDim, int zDim);

int DestroyField(field * fieldToDestroy);

bucket_field * CreateBucketField(int xDim, int yDim, int zDim);

int DestroyBucketField(bucket_field * fieldToDestroy);

std::vector< std::vector< std::vector< std::vector<int> > > > MakeNeighborhoods();

inline int GetDirectionNumber(int dx, int dy, int dz)
{
    return ((dx + 1) * 9) + ((dy + 1) * 3) + (dz + 1);
}

inline bucket_cell * GetBucketCell(bucket_field* grid, int x, int y, int z)
{
    if (x >= 0 && y >= 0 && z >= 0 && x < grid->xDim && y < grid->yDim && z < grid->zDim)
    {
        return &(grid->cells[x][y][z]);
    }
    else
    {
        return NULL;
    }
}

inline cell * Get(field* grid, int x, int y, int z)
{
    if (x >= 0 && y >= 0 && z >= 0 && x < grid->xDim && y < grid->yDim && z < grid->zDim)
    {
        return &(grid->cells[x][y][z]);
    }
    else
    {
        return &(grid->empty_cell);
    }
}

inline void Put(field* grid, cell* new_cell, int x, int y, int z)
{
    if (x >= 0 && y >= 0 && z >= 0 && x < grid->xDim && y < grid->yDim && z < grid->zDim)
    {
        grid->cells[x][y][z].dx = new_cell->dx;
        grid->cells[x][y][z].dy = new_cell->dy;
        grid->cells[x][y][z].dz = new_cell->dz;
    }
}

inline void Compare(field* grid, cell* cur_cell, int x, int y, int z, int xo, int yo, int zo)
{
    cell* other_cell = Get(grid, x + xo, y + yo, z + zo);
    float tdx = other_cell->dx + xo;
    float tdy = other_cell->dy + yo;
    float tdz = other_cell->dz + zo;
    if ((pow(tdx, 2) + pow(tdy, 2) + pow(tdz, 2)) < (pow(cur_cell->dx, 2) + pow(cur_cell->dy, 2) + pow(cur_cell->dz, 2)))
    {
        cur_cell->dx = tdx;
        cur_cell->dy = tdy;
        cur_cell->dz = tdz;
    }
}

inline int max(int a, int b)
{
    if (a > b)
    {
        return a;
    }
    else
    {
        return b;
    }
}

inline int CheckOccupancy(dVoxelArray* environment, int i, int j, int k)
{
    if (i < 0 || j < 0 || k < 0)
    {
        return -1;
    }
    if (i >= environment->xDim || j >= environment->yDim || k >= environment->zDim)
    {
        return -1;
    }
    if (environment->dVoxels[i][j][k].sensitivity > 0.0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

inline double DistanceSquared(int x1, int y1, int z1, int x2, int y2, int z2)
{
    return pow(x2 - x1, 2.0) + pow(y2 - y1, 2.0) + pow(z2 - z1, 2.0);
}

inline float Distance(int x1, int y1, int z1, int x2, int y2, int z2)
{
    return (float)sqrt(pow(x2 - x1, 2.0) + pow(y2 - y1, 2.0) + pow(z2 - z1, 2.0));
}

inline int DVoxelEqual(dVoxel one, dVoxel two)
{
    if (one.deformability == two.deformability)
    {
        if (one.sensitivity == two.sensitivity)
        {
            if (one.R == two.R && one.G == two.G && one.B == two.B && one.A == two.A)
            {
                return 1;
            }
        }
    }
    return 0;
}

#endif // D_VOXEL_H
