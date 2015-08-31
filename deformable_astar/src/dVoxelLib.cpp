#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "deformable_astar/dVoxelLib.h"
#include "omp.h"
#include "Eigen/Geometry"
#include "vector"

dVoxelArray * AllocateDVoxelArray(int Xdim, int Ydim, int Zdim, int Xcenter, int Ycenter, int Zcenter, float defaultSensitivity, float defaultDeformability)
{
    //Allocates a dVoxelArray with the given parameters
    dVoxelArray *new_dvoxel_array = (dVoxelArray*) malloc(sizeof(dVoxelArray));
    if (new_dvoxel_array == NULL)
    {
        return NULL;
    }
    //printf("Allocation start\n");
    new_dvoxel_array->xDim = Xdim;
    new_dvoxel_array->yDim = Ydim;
    new_dvoxel_array->zDim = Zdim;
    new_dvoxel_array->xCenter = Xcenter;
    new_dvoxel_array->yCenter = Ycenter;
    new_dvoxel_array->zCenter = Zcenter;
    new_dvoxel_array->defaultSensitivity = defaultSensitivity;
    new_dvoxel_array->defaultDeformability = defaultDeformability;
    //printf("Array allocation start\n");
    //Allocate the 3D dvoxel grid
    new_dvoxel_array->dVoxels = (dVoxel***) malloc(Xdim * sizeof(dVoxel**));
    //Safety check
    if (new_dvoxel_array->dVoxels == NULL)
    {
        return NULL;
    }
    //printf("Multidim allocation start\n");
    for (int i = 0; i < Xdim; i++)
    {
        new_dvoxel_array->dVoxels[i] = (dVoxel**) malloc(Ydim * sizeof(dVoxel*));
        //Safety check
        if (new_dvoxel_array->dVoxels[i] == NULL)
        {
            return NULL;
        }
        for (int j = 0; j < Ydim; j++)
        {
            new_dvoxel_array->dVoxels[i][j] = (dVoxel*) malloc(Zdim * sizeof(dVoxel));
            if (new_dvoxel_array->dVoxels[i][j] == NULL)
            {
                return NULL;
            }
            //Populate individual dvoxel
            for (int k = 0; k < Zdim; k++)
            {
                new_dvoxel_array->dVoxels[i][j][k].deformability = defaultDeformability;
                new_dvoxel_array->dVoxels[i][j][k].sensitivity = defaultSensitivity;
                new_dvoxel_array->dVoxels[i][j][k].SDF = NAN;
                new_dvoxel_array->dVoxels[i][j][k].R = 0x00;
                new_dvoxel_array->dVoxels[i][j][k].G = 0x00;
                new_dvoxel_array->dVoxels[i][j][k].B = 0x00;
                new_dvoxel_array->dVoxels[i][j][k].A = 0x00;
            }
        }
    }
    //printf("Ready to return assembled array\n");
    return new_dvoxel_array;
}

int DestroyDVoxelArray(dVoxelArray * arrayToDestroy)
{
    //Deallocates a dVoxelArray
    int Xdim = arrayToDestroy->xDim;
    int Ydim = arrayToDestroy->yDim;
    for (int i = 0; i < Xdim; i++)
    {
        for (int j = 0; j < Ydim; j++)
        {
            //printf("Free level 2\n");
            free(arrayToDestroy->dVoxels[i][j]);
        }
        //printf("Free level 1\n");
        free(arrayToDestroy->dVoxels[i]);
    }
    //printf("Free level 0\n");
    free(arrayToDestroy->dVoxels);
    //printf("Free remainder\n");
    free(arrayToDestroy);
    return 0;
}

int WriteDVoxelArrayToFile(dVoxelArray * arrayToWrite, char* filename)
{
    FILE *array_file = fopen(filename, "w");
    //Write file header
    fprintf(array_file, "D-Voxel Array Storage File\nSize: %d|%d|%d\nCenter: %d|%d|%d\nDefaults: %f|%f\nDATA", arrayToWrite->xDim, arrayToWrite->yDim, arrayToWrite->zDim, arrayToWrite->xCenter, arrayToWrite->yCenter, arrayToWrite->zCenter, arrayToWrite->defaultSensitivity, arrayToWrite->defaultDeformability);
    //Write file data
    for (int i = 0; i < arrayToWrite->xDim; i++)
    {
        for (int j = 0; j < arrayToWrite->yDim; j++)
        {
            for (int k = 0; k < arrayToWrite->zDim; k++)
            {
                dVoxel temp = arrayToWrite->dVoxels[i][j][k];
                fwrite(&temp, sizeof(dVoxel), 1, array_file);
            }
        }
    }
    //Flush & close file
    return fclose(array_file);
}

int WriteDVoxelArrayToCSVFile(dVoxelArray * arrayToWrite, char* filename)
{
    FILE *array_file = fopen(filename, "w");
    //Write file header
    fprintf(array_file, "DVXL Array CSV File\nSize,%d,%d,%d\nCenter,%d,%d,%d\nDefaults,%f,%f\nDATA\n", arrayToWrite->xDim, arrayToWrite->yDim, arrayToWrite->zDim, arrayToWrite->xCenter, arrayToWrite->yCenter, arrayToWrite->zCenter, arrayToWrite->defaultSensitivity, arrayToWrite->defaultDeformability);
    //Write file data
    for (int i = 0; i < arrayToWrite->xDim; i++)
    {
        for (int j = 0; j < arrayToWrite->yDim; j++)
        {
            for (int k = 0; k < arrayToWrite->zDim; k++)
            {
                dVoxel temp = arrayToWrite->dVoxels[i][j][k];
                fprintf(array_file, "%d,%d,%d,%f,%f,%f,%d,%d,%d,%d\n", i, j, k, temp.deformability, temp.sensitivity, temp.SDF, temp.R, temp.G, temp.B, temp.A);
            }
        }
    }
    //Flush & close file
    return fclose(array_file);
}

dVoxelArray * LoadDVoxelArrayFromFile(char* filename)
{
    FILE *array_file = fopen(filename, "r");
    if (array_file == NULL)
    {
        printf("*** Unable to find DVXL file! ***\n");
        fflush(stdout);
        exit(1);
    }
    //Read file header
    int Xdim, Ydim, Zdim, Xcenter, Ycenter, Zcenter;
    float defCost, defDeform;
    int read = fscanf(array_file, "D-Voxel Array Storage File\nSize: %d|%d|%d\nCenter: %d|%d|%d\nDefaults: %f|%f\nDATA", &Xdim, &Ydim, &Zdim, &Xcenter, &Ycenter, &Zcenter, &defCost, &defDeform);
    if (read != 8)
    {
        printf("*** Broken or corrupted DVXL file! - Check the file manually to make sure the header is correct :: Got %d parameters, expected %d! ***\n", read, 8);
        fflush(stdout);
        exit(1);
    }
    //Make an empty DVOXEL array
    dVoxelArray *new_dvoxel_array = AllocateDVoxelArray(Xdim, Ydim, Zdim, Xcenter, Ycenter, Zcenter, defCost, defDeform);
    //Safety check
    if (new_dvoxel_array == NULL)
    {
        return NULL;
    }
    //Read file data
    char * temp_read = (char *) malloc(sizeof(dVoxel));
    for (int i = 0; i < Xdim; i++)
    {
        for (int j = 0; j < Ydim; j++)
        {
            for (int k = 0; k < Zdim; k++)
            {
                int read = fread(temp_read, sizeof(dVoxel), 1, array_file);
                if (read != 1)
                {
                    printf("*** Broken or corrupted DVXL file! - Check the file manually to make sure the header is correct :: Current index: [%d,%d,%d] :: Read %d elements [%s], expected %d! ***\n", i, j, k, read, temp_read, 1);
                    fflush(stdout);
                    exit(1);
                }
                new_dvoxel_array->dVoxels[i][j][k].deformability = *( (float*)(temp_read) );
                new_dvoxel_array->dVoxels[i][j][k].sensitivity = *( (float*)(temp_read + 4) );
                new_dvoxel_array->dVoxels[i][j][k].SDF = *( (float*)(temp_read + 8) );
                new_dvoxel_array->dVoxels[i][j][k].R = temp_read[12];
                new_dvoxel_array->dVoxels[i][j][k].G = temp_read[13];
                new_dvoxel_array->dVoxels[i][j][k].B = temp_read[14];
                new_dvoxel_array->dVoxels[i][j][k].A = temp_read[15];
            }
        }
    }
    //Flush & close file
    fclose(array_file);
    free(temp_read);
    //Return data
    return new_dvoxel_array;
}

dVoxelArray * CloneDVoxelArray(dVoxelArray * arrayToClone)
{
    //Make an empty DVOXEL array
    dVoxelArray *new_dvoxel_array = AllocateDVoxelArray(arrayToClone->xDim, arrayToClone->yDim, arrayToClone->zDim, arrayToClone->xCenter, arrayToClone->yCenter, arrayToClone->zCenter, arrayToClone->defaultSensitivity, arrayToClone->defaultDeformability);
    //Safety check
    if (new_dvoxel_array == NULL)
    {
        return NULL;
    }
    //Copy data
    for (int i = 0; i < (arrayToClone->xDim); i++)
    {
        for (int j = 0; j < (arrayToClone->yDim); j++)
        {
            for (int k = 0; k < (arrayToClone->zDim); k++)
            {
                new_dvoxel_array->dVoxels[i][j][k].deformability = arrayToClone->dVoxels[i][j][k].deformability;
                new_dvoxel_array->dVoxels[i][j][k].sensitivity = arrayToClone->dVoxels[i][j][k].sensitivity;
                new_dvoxel_array->dVoxels[i][j][k].R = arrayToClone->dVoxels[i][j][k].R;
                new_dvoxel_array->dVoxels[i][j][k].G = arrayToClone->dVoxels[i][j][k].G;
                new_dvoxel_array->dVoxels[i][j][k].B = arrayToClone->dVoxels[i][j][k].B;
                new_dvoxel_array->dVoxels[i][j][k].A = arrayToClone->dVoxels[i][j][k].A;
            }
        }
    }
    //Return data
    return new_dvoxel_array;
}

field * CreateField(int xDim, int yDim, int zDim)
{
    //Allocates a dVoxelArray with the given parameters
    field *new_field = (field*)malloc(sizeof(field));
    if (new_field == NULL)
    {
        return NULL;
    }
    //printf("Allocation start\n");
    new_field->xDim = xDim;
    new_field->yDim = yDim;
    new_field->zDim = zDim;
    new_field->empty_cell.dx = INFINITY;
    new_field->empty_cell.dy = INFINITY;
    new_field->empty_cell.dz = INFINITY;
    //printf("Array allocation start\n");
    //Allocate the 3D field
    new_field->cells = (cell***)malloc(xDim * sizeof(cell**));
    //Safety check
    if (new_field->cells == NULL)
    {
        return NULL;
    }
    //printf("Multidim allocation start\n");
    for (int i = 0; i < xDim; i++)
    {
        new_field->cells[i] = (cell**)malloc(yDim * sizeof(cell*));
        //Safety check
        if (new_field->cells[i] == NULL)
        {
            return NULL;
        }
        for (int j = 0; j < yDim; j++)
        {
            new_field->cells[i][j] = (cell*)malloc(zDim * sizeof(cell));
            if (new_field->cells[i][j] == NULL)
            {
                return NULL;
            }
        }
    }
    //printf("Ready to return assembled array\n");
    return new_field;
}

int DestroyField(field * fieldToDestroy)
{
    //Deallocates a field
    int Xdim = fieldToDestroy->xDim;
    int Ydim = fieldToDestroy->yDim;
    for (int i = 0; i < Xdim; i++)
    {
        for (int j = 0; j < Ydim; j++)
        {
            //printf("Free level 2\n");
            free(fieldToDestroy->cells[i][j]);
        }
        //printf("Free level 1\n");
        free(fieldToDestroy->cells[i]);
    }
    //printf("Free level 0\n");
    free(fieldToDestroy->cells);
    //printf("Free remainder\n");
    free(fieldToDestroy);
    return 0;
}

bucket_field * CreateBucketField(int xDim, int yDim, int zDim)
{
    //Allocates a dVoxelArray with the given parameters
    bucket_field *new_field = (bucket_field*)malloc(sizeof(bucket_field));
    if (new_field == NULL)
    {
        return NULL;
    }
    //printf("Allocation start\n");
    new_field->xDim = xDim;
    new_field->yDim = yDim;
    new_field->zDim = zDim;
    //printf("Array allocation start\n");
    //Allocate the 3D field
    new_field->cells = (bucket_cell***)malloc(xDim * sizeof(bucket_cell**));
    //Safety check
    if (new_field->cells == NULL)
    {
        return NULL;
    }
    //printf("Multidim allocation start\n");
    for (int i = 0; i < xDim; i++)
    {
        new_field->cells[i] = (bucket_cell**)malloc(yDim * sizeof(bucket_cell*));
        //Safety check
        if (new_field->cells[i] == NULL)
        {
            return NULL;
        }
        for (int j = 0; j < yDim; j++)
        {
            new_field->cells[i][j] = (bucket_cell*)malloc(zDim * sizeof(bucket_cell));
            if (new_field->cells[i][j] == NULL)
            {
                return NULL;
            }
            //Initialize individual buckets
            for (int k = 0; k < zDim; k++)
            {
                new_field->cells[i][j][k].distance_square = INFINITY;
            }
        }
    }
    //printf("Ready to return assembled array\n");
    return new_field;
}

int DestroyBucketField(bucket_field * fieldToDestroy)
{
    //Deallocates a field
    int Xdim = fieldToDestroy->xDim;
    int Ydim = fieldToDestroy->yDim;
    for (int i = 0; i < Xdim; i++)
    {
        for (int j = 0; j < Ydim; j++)
        {
            //printf("Free level 2\n");
            free(fieldToDestroy->cells[i][j]);
        }
        //printf("Free level 1\n");
        free(fieldToDestroy->cells[i]);
    }
    //printf("Free level 0\n");
    free(fieldToDestroy->cells);
    //printf("Free remainder\n");
    free(fieldToDestroy);
    return 0;
}

bucket_field * BuildPropagationField(std::vector<Eigen::Vector3i>& points, int xDim, int yDim, int zDim)
{
    // Make container
    bucket_field* field = CreateBucketField(xDim, yDim, zDim);
    // Compute the maximum distance square
    int max_distance_square = (xDim * xDim) + (yDim * yDim) + (zDim * zDim);
    // Initialize bucket queue
    std::vector< std::vector<bucket_cell*> > bucket_queue;
    bucket_queue.resize(max_distance_square + 1);
    bucket_queue[0].reserve(points.size());
    // Set the initial update direction
    int initial_update_direction = GetDirectionNumber(0, 0, 0);
    // Mark all points with distance zero and add to queue
    for (size_t i = 0; i < points.size(); i++)
    {
        bucket_cell* cur_cell = GetBucketCell(field, points[i].x(), points[i].y(), points[i].z());
        if (cur_cell == NULL)
        {
            continue;
        }
        else
        {
            cur_cell->location[0] = points[i].x();
            cur_cell->location[1] = points[i].y();
            cur_cell->location[2] = points[i].z();
            cur_cell->closest_point[0] = points[i].x();
            cur_cell->closest_point[1] = points[i].y();
            cur_cell->closest_point[2] = points[i].z();
            cur_cell->distance_square = 0.0;
            cur_cell->update_direction = initial_update_direction;
            bucket_queue[0].push_back(cur_cell);
        }
    }
    // Process the queue
    std::vector< std::vector< std::vector< std::vector<int> > > > neighborhoods = MakeNeighborhoods();
    for (size_t i = 0; i < bucket_queue.size(); i++)
    {
        std::vector<bucket_cell*>::iterator list_it = bucket_queue[i].begin();
        while (list_it != bucket_queue[i].end())
        {
            // Get the current location
            bucket_cell* cur_cell = *list_it;
            double x = cur_cell->location[0];
            double y = cur_cell->location[1];
            double z = cur_cell->location[2];
            // Pick the update direction
            int D = i;
            if (D > 1)
            {
                D = 1;
            }
            // Make sure the update direction is valid
            if (cur_cell->update_direction < 0 || cur_cell->update_direction > 26)
            {
                ++list_it;
                continue;
            }
            // Get the neighborhood list
            std::vector< std::vector<int> >& neighborhood = neighborhoods[D][cur_cell->update_direction];
            // Update distance from the neighboring cells
            for (size_t n = 0; n < neighborhood.size(); n++)
            {
                // Get the direction to check
                int dx = neighborhood[n][0];
                int dy = neighborhood[n][1];
                int dz = neighborhood[n][2];
                int nx = x + dx;
                int ny = y + dy;
                int nz = z + dz;
                bucket_cell* neighbor_cell = GetBucketCell(field, nx, ny, nz);
                if (neighbor_cell == NULL)
                {
                    continue;
                }
                // Update the neighbor's distance based on current
                int new_distance_square = DistanceSquared(nx, ny, nz, cur_cell->closest_point[0], cur_cell->closest_point[1], cur_cell->closest_point[2]);
                if (new_distance_square > max_distance_square)
                {
                    // Skip this
                    continue;
                }
                if (new_distance_square < neighbor_cell->distance_square)
                {
                    // If the distance is better, time to update our neighbor
                    neighbor_cell->distance_square = new_distance_square;
                    neighbor_cell->closest_point[0] = cur_cell->closest_point[0];
                    neighbor_cell->closest_point[1] = cur_cell->closest_point[1];
                    neighbor_cell->closest_point[2] = cur_cell->closest_point[2];
                    neighbor_cell->location[0] = nx;
                    neighbor_cell->location[1] = ny;
                    neighbor_cell->location[2] = nz;
                    neighbor_cell->update_direction = GetDirectionNumber(dx, dy, dz);
                    // Add the neighbor to the queue
                    bucket_queue[new_distance_square].push_back(neighbor_cell);
                }
            }
            ++list_it;
        }
        bucket_queue[i].clear();
    }
    return field;
}

std::vector< std::vector< std::vector< std::vector<int> > > > MakeNeighborhoods()
{
    std::vector< std::vector< std::vector< std::vector<int> > > > neighborhoods;
    neighborhoods.resize(2);
    for (size_t n = 0; n < neighborhoods.size(); n++)
    {
        neighborhoods[n].resize(27);
        // Loop through the source directions
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dz = -1; dz <= 1; dz++)
                {
                    int direction_number = GetDirectionNumber(dx, dy, dz);
                    // Loop through the target directions
                    for (int tdx = -1; tdx <= 1; tdx++)
                    {
                        for (int tdy = -1; tdy <= 1; tdy++)
                        {
                            for (int tdz = -1; tdz <= 1; tdz++)
                            {
                                if (tdx == 0 && tdy == 0 && tdz == 0)
                                {
                                    continue;
                                }
                                if (n >= 1)
                                {
                                    if ((abs(tdx) + abs(tdy) + abs(tdz)) != 1)
                                    {
                                        continue;
                                    }
                                    if ((dx * tdx) < 0 || (dy * tdy) < 0 || (dz * tdz) < 0)
                                    {
                                        continue;
                                    }
                                }
                                std::vector<int> new_point;
                                new_point.resize(3);
                                new_point[0] = tdx;
                                new_point[1] = tdy;
                                new_point[2] = tdz;
                                neighborhoods[n][direction_number].push_back(new_point);
                            }
                        }
                    }
                }
            }
        }
    }
    return neighborhoods;
}

int UpdateSDF(dVoxelArray* arrayToUpdate)
{
    printf("Computing SDF\n");
    //fflush(stdout);
    if (arrayToUpdate->zDim > 1)
    {
        printf("Computing 3D SDF with bucket queue\n");
        fflush(stdout);
        // Build vectors of voxels inside and outside of obstacles
        std::vector<Eigen::Vector3i> free_voxels;
        std::vector<Eigen::Vector3i> collision_voxels;
        // Loop through the voxel grid
        for (int i = 0; i < arrayToUpdate->xDim; i++)
        {
            for (int j = 0; j < arrayToUpdate->yDim; j++)
            {
                //#pragma omp parallel for // OpenMP overhead results in worse performance!
                for (int k = 0; k < arrayToUpdate->zDim; k++)
                {
                    if (arrayToUpdate->dVoxels[i][j][k].sensitivity == 0.0)
                    {
                        free_voxels.push_back(Eigen::Vector3i(i,j,k));
                    }
                    else
                    {
                        collision_voxels.push_back(Eigen::Vector3i(i,j,k));
                    }
                }
            }
        }
        // Process each vector into a propagation field
        bucket_field* free_propagation_field = BuildPropagationField(free_voxels, arrayToUpdate->xDim, arrayToUpdate->yDim, arrayToUpdate->zDim);
        bucket_field* collision_propagation_field = BuildPropagationField(collision_voxels, arrayToUpdate->xDim, arrayToUpdate->yDim, arrayToUpdate->zDim);
        // Loop through the array and update the SDF values from the fields
        for (int i = 0; i < arrayToUpdate->xDim; i++)
        {
            for (int j = 0; j < arrayToUpdate->yDim; j++)
            {
                //#pragma omp parallel for // OpenMP overhead results in worse performance!
                for (int k = 0; k < arrayToUpdate->zDim; k++)
                {
                    double distance1 = sqrt(collision_propagation_field->cells[i][j][k].distance_square);
                    double distance2 = sqrt(free_propagation_field->cells[i][j][k].distance_square);
                    arrayToUpdate->dVoxels[i][j][k].SDF = distance1 - distance2;
                }
            }
        }
        DestroyBucketField(free_propagation_field);
        DestroyBucketField(collision_propagation_field);
        printf("SDF computation complete\n");
        fflush(stdout);
        return 0;
    }
    else if (arrayToUpdate->zDim == 1)
    {
        printf("Computing 2D SDF with 8SSEDT\n");
        fflush(stdout);
        // Make temporary storage for computing the SDF
        field* temp_field1 = CreateField(arrayToUpdate->xDim, arrayToUpdate->yDim, arrayToUpdate->zDim);
        field* temp_field2 = CreateField(arrayToUpdate->xDim, arrayToUpdate->yDim, arrayToUpdate->zDim);
        // Initialize the two temp fields
        cell empty_cell;
        empty_cell.dx = INFINITY;
        empty_cell.dy = INFINITY;
        empty_cell.dz = INFINITY;
        cell full_cell;
        full_cell.dx = 0.0;
        full_cell.dy = 0.0;
        full_cell.dz = 0.0;
        for (int i = 0; i < arrayToUpdate->xDim; i++)
        {
            for (int j = 0; j < arrayToUpdate->yDim; j++)
            {
                //#pragma omp parallel for // OpenMP overhead results in worse performance!
                for (int k = 0; k < arrayToUpdate->zDim; k++)
                {
                    if (arrayToUpdate->dVoxels[i][j][k].sensitivity == 0.0)
                    {
                        Put(temp_field1, &empty_cell, i, j, k);
                        Put(temp_field2, &full_cell, i, j, k);
                    }
                    else
                    {
                        Put(temp_field2, &empty_cell, i, j, k);
                        Put(temp_field1, &full_cell, i, j, k);
                    }
                }
            }
        }
        // Run propagation algorithm through the fields
        UpdateIntermediateField(temp_field1);
        UpdateIntermediateField(temp_field2);
        // Loop through the array and update the SDF values from the fields
        for (int i = 0; i < arrayToUpdate->xDim; i++)
        {
            for (int j = 0; j < arrayToUpdate->yDim; j++)
            {
                //#pragma omp parallel for // OpenMP overhead results in worse performance!
                for (int k = 0; k < arrayToUpdate->zDim; k++)
                {
                    float distance1 = sqrt(pow(temp_field1->cells[i][j][k].dx, 2) + pow(temp_field1->cells[i][j][k].dy, 2) + pow(temp_field1->cells[i][j][k].dz, 2));
                    float distance2 = sqrt(pow(temp_field2->cells[i][j][k].dx, 2) + pow(temp_field2->cells[i][j][k].dy, 2) + pow(temp_field2->cells[i][j][k].dz, 2));
                    arrayToUpdate->dVoxels[i][j][k].SDF = distance1 - distance2;
                }
            }
        }
        DestroyField(temp_field1);
        DestroyField(temp_field2);
        printf("SDF computation complete\n");
        fflush(stdout);
        return 0;
    }
    else
    {
        printf("DVoxelArray has invalid dimensions!\n");
        return -1;
    }
}

Eigen::Vector3d GetGradientFromSDF(dVoxelArray * environment, int x, int y, int z)
{
    // Make sure the point is at least 1 in from the edge, or we won't be able to compute a gradient
    if (x >= 1 && y >= 1 && z >= 1 && x < (environment->xDim - 1) && y < (environment->yDim - 1) && z < (environment->zDim - 1))
    {
        double gx = (environment->dVoxels[x + 1][y][z].SDF - environment->dVoxels[x - 1][y][z].SDF) * 0.5;
        double gy = (environment->dVoxels[x][y + 1][z].SDF - environment->dVoxels[x][y - 1][z].SDF) * 0.5;
        double gz = (environment->dVoxels[x][y][z + 1].SDF - environment->dVoxels[x][y][z - 1].SDF) * 0.5;
        //printf("Returning gradient (%f,%f,%f) for position %d|%d|%d\n", gx, gy, gz, x, y, z);
        return Eigen::Vector3d(gx, gy, gz);
    }
    else
    {
        //printf("Query %d|%d|%d out of bounds - returning zero gradient\n", x, y, z);
        // If it's on the edge, return 0 gradient
        return Eigen::Vector3d(0.0, 0.0, 0.0);
    }
}

Eigen::Vector3d Get2DGradientFromSDF(dVoxelArray * environment, int x, int y, int z)
{
    // Make sure the point is at least 1 in from the edge, or we won't be able to compute a gradient
    if (x >= 1 && y >= 1 && z >= 0 && x < (environment->xDim - 1) && y < (environment->yDim - 1) && z < (environment->zDim))
    {
        double gx = (environment->dVoxels[x + 1][y][z].SDF - environment->dVoxels[x - 1][y][z].SDF) * 0.5;
        double gy = (environment->dVoxels[x][y + 1][z].SDF - environment->dVoxels[x][y - 1][z].SDF) * 0.5;
        double gz = 0.0;
        //printf("Returning gradient (%f,%f,%f) for position %d|%d|%d\n", gx, gy, gz, x, y, z);
        return Eigen::Vector3d(gx, gy, gz);
    }
    else
    {
        //printf("Query %d|%d|%d out of bounds - returning zero gradient\n", x, y, z);
        // If it's on the edge, return 0 gradient
        return Eigen::Vector3d(0.0, 0.0, 0.0);
    }
}

void UpdateIntermediateField(field* fieldToUpdate)
{
    // This only does a 2D SDF! (assumes zd == 1)
    // Pass 1.0.0
    for (int z = 0; z < fieldToUpdate->zDim; z++)
    {
        // Pass 1.1.0
        for (int y = 0; y < fieldToUpdate->yDim; y++)
        {
            // Pass 1.1.1
            for (int x = 0; x < fieldToUpdate->xDim; x++)
            {
                // Get value from grid
                cell* point = Get(fieldToUpdate, x, y, z);
                // Do work
                Compare(fieldToUpdate, point, x, y, z, -1, 0, 0);
                Compare(fieldToUpdate, point, x, y, z, 0, -1, 0);
                Compare(fieldToUpdate, point, x, y, z, -1, -1, 0);
                Compare(fieldToUpdate, point, x, y, z, 1, -1, 0);
                // Store back into grid
                Put(fieldToUpdate, point, x, y, z);
            }
            // Pass 1.1.2
            for (int x = (fieldToUpdate->xDim - 1); x >= 0; x--)
            {
                // Get value from grid
                cell* point = Get(fieldToUpdate, x, y, z);
                // Do work
                Compare(fieldToUpdate, point, x, y, z, 1, 0, 0);
                // Store back into grid
                Put(fieldToUpdate, point, x, y, z);
            }
        }
        // Pass 1.2.0
        for (int y = (fieldToUpdate->yDim - 1); y >= 0; y--)
        {
            // Pass 1.1.1
            for (int x = (fieldToUpdate->xDim - 1); x >= 0; x--)
            {
                // Get value from grid
                cell* point = Get(fieldToUpdate, x, y, z);
                // Do work
                Compare(fieldToUpdate, point, x, y, z, 1, 0, 0);
                Compare(fieldToUpdate, point, x, y, z, 0, 1, 0);
                Compare(fieldToUpdate, point, x, y, z, -1, 1, 0);
                Compare(fieldToUpdate, point, x, y, z, 1, 1, 0);
                // Store back into grid
                Put(fieldToUpdate, point, x, y, z);
            }
            // Pass 1.1.2
            for (int x = 0; x < fieldToUpdate->xDim; x++)
            {
                // Get value from grid
                cell* point = Get(fieldToUpdate, x, y, z);
                // Do work
                Compare(fieldToUpdate, point, x, y, z, -1, 0, 0);
                // Store back into grid
                Put(fieldToUpdate, point, x, y, z);
            }
        }
    }
    /*
    // Pass 2.0.0
    for (int z = (fieldToUpdate->zDim - 1); z >= 0; z--)
    {
        // Pass 2.1.0
        for (int y = 0; y < yd; y++)
        {
            // Pass 1.1.1
            for (int x = 0; x < fieldToUpdate->xDim; x++)
            {
                // Get value from grid
                float* point = Get(grid, fieldToUpdate->xDim, yd, zd, x, y, z);
                // Do work
                ;
                // Store back into grid
                Put(grid, point, fieldToUpdate->xDim, yd, zd, x, y, z);
                free(point);
            }
            // Pass 1.1.2
            for (int x = (fieldToUpdate->xDim - 1); x >= 0; x--)
            {
                // Get value from grid
                float* point = Get(grid, fieldToUpdate->xDim, yd, zd, x, y, z);
                // Do work
                ;
                // Store back into grid
                Put(grid, point, fieldToUpdate->xDim, yd, zd, x, y, z);
                free(point);
            }
        }
        // Pass 2.2.0
        for (int y = (yd - 1); y >= 0; y--)
        {
            // Pass 1.1.1
            for (int x = (fieldToUpdate->xDim - 1); x >= 0; x--)
            {
                // Get value from grid
                float* point = Get(grid, fieldToUpdate->xDim, yd, zd, x, y, z);
                // Do work
                ;
                // Store back into grid
                Put(grid, point, fieldToUpdate->xDim, yd, zd, x, y, z);
                free(point);
            }
            // Pass 1.1.2
            for (int x = 0; x < fieldToUpdate->xDim; x++)
            {
                // Get value from grid
                float* point = Get(grid, fieldToUpdate->xDim, yd, zd, x, y, z);
                // Do work
                ;
                // Store back into grid
                Put(grid, point, fieldToUpdate->xDim, yd, zd, x, y, z);
                free(point);
            }
        }
    }
    */
}

double * TransformPoint(double tx, double ty, double tz, double rx, double ry, double rz, double px, double py, double pz)
{
    double * transformed = (double*)malloc(sizeof(double) * 3);
    // Make translation from (tx,ty,tz) and rotation from (rx,ry,rz)
    Eigen::Translation3d translation(tx, ty, tz);
    Eigen::Quaternion<double> rotation;
    rotation = Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());
    // Combine into a transform
    Eigen::Transform<double,3,Eigen::Affine> transform = translation * rotation;
    // Make vector from (px,py,pz)
    Eigen::Vector3d point(px, py, pz);
    // Multiply matrix and vector to get the transformed point
    Eigen::Vector3d transformed_point = transform * point;
    // Extract x,y,z and store in return array
    transformed[0] = transformed_point.x();
    transformed[1] = transformed_point.y();
    transformed[2] = transformed_point.z();
    return transformed;
}

double * RotatePoint(double X, double Y, double Z, double XR, double YR, double ZR, double XC, double YC, double ZC)
{
    double px = X - XC;
    double py = Y - YC;
    double pz = Z - ZC;
    double * transformed = TransformPoint(0.0, 0.0, 0.0, XR, YR, ZR, px, py, pz);
    double * rotated = (double*)malloc(sizeof(double) * 3);
    rotated[0] = transformed[0] + XC;
    rotated[1] = transformed[1] + YC;
    rotated[2] = transformed[2] + ZC;
    free(transformed);
    return rotated;
}

dVoxelArray * TransformMergeDVoxelArrays(dVoxelArray * world, dVoxelArray * object, float Xt, float Yt, float Zt, float Xr, float Yr, float Zr, uint8_t alpha_threshold)
{
    double new_center_x = Xt;
    double new_center_y = Yt;
    double new_center_z = Zt;
    for (int i = 0; i < object->xDim; i++)
    {
        for (int j = 0; j < object->yDim; j++)
        {
            for (int k = 0; k < object->zDim; k++)
            {
                //Find the translated absolute position of the current index
                float Xoffset = (float)(i - object->xCenter);
                float Yoffset = (float)(j - object->yCenter);
                float Zoffset = (float)(k - object->zCenter);
                //Compute the rotated position of the current point
                double * rotated_point = RotatePoint(Xoffset, Yoffset, Zoffset, Xr, Yr, Zr, 0.0, 0.0, 0.0);
                rotated_point[0] = rotated_point[0] + new_center_x;
                rotated_point[1] = rotated_point[1] + new_center_y;
                rotated_point[2] = rotated_point[2] + new_center_z;
                int new_x = snap(rotated_point[0]);
                int new_y = snap(rotated_point[1]);
                int new_z = snap(rotated_point[2]);
                //int new_x = snap(new_center_x + Xoffset);
                //int new_y = snap(new_center_y + Yoffset);
                //int new_z = snap(new_center_z + Zoffset);
                //Safety check
                if ((new_x >= 0 && new_y >= 0 && new_z >= 0) && (new_x < world->xDim && new_y < world->yDim && new_z < world->zDim))
                {
                    if (world->dVoxels[new_x][new_y][new_z].A < alpha_threshold)
                    {
                        //Merge the new point into the world
                        world->dVoxels[new_x][new_y][new_z].deformability = object->dVoxels[i][j][k].deformability;
                        world->dVoxels[new_x][new_y][new_z].sensitivity = object->dVoxels[i][j][k].sensitivity;
                        world->dVoxels[new_x][new_y][new_z].R = object->dVoxels[i][j][k].R;
                        world->dVoxels[new_x][new_y][new_z].G = object->dVoxels[i][j][k].G;
                        world->dVoxels[new_x][new_y][new_z].B = object->dVoxels[i][j][k].B;
                        world->dVoxels[new_x][new_y][new_z].A = object->dVoxels[i][j][k].A;
                    }
                }
                else
                {
                    printf("Attempted to transform to an illegal state!\n");
                    printf("Attempted to transform index to %d|%d|%d, but the boundaries are 0|0|0 and %d|%d|%d!\n", new_x, new_y, new_z, world->xDim, world->yDim, world->zDim);
                }
                free(rotated_point);
            }
        }
    }
    return world;
}

dVoxelArray * Recolor(dVoxelArray * object, uint8_t R, uint8_t G, uint8_t B, uint8_t alpha_threshold, int copy_first)
{
    dVoxelArray * to_recolor = object;
    if (copy_first == 1)
    {
        to_recolor = CloneDVoxelArray(object);
    }
    for (int i = 0; i < object->xDim; i++)
    {
        for (int j = 0; j < object->yDim; j++)
        {
            for (int k = 0; k < object->zDim; k++)
            {
                if (to_recolor->dVoxels[i][j][k].A >= alpha_threshold)
                to_recolor->dVoxels[i][j][k].R = R;
                to_recolor->dVoxels[i][j][k].G = G;
                to_recolor->dVoxels[i][j][k].B = B;
            }
        }
    }
    return to_recolor;
}
