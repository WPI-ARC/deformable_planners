#include <stdio.h>
#include <stdlib.h>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <omp.h>
#include <time.h>

int omp_test(void)
{
    int th_id, nthreads;
    #pragma omp parallel private(th_id)
    {
        th_id = omp_get_thread_num();
        #pragma omp barrier
        if (th_id == 0)
        {
            nthreads = omp_get_num_threads();
        }
    }
    return nthreads;
}

int check_fives_neighbors(const VoxelGrid::VoxelGrid<int>& source, const int64_t x_index, const int64_t y_index, const int64_t z_index, VoxelGrid::VoxelGrid<int>& output)
{
    // Grab the six neighbors
    std::pair<const int&, bool> xmyz = source.GetImmutable(x_index - 1, y_index, z_index);
    std::pair<const int&, bool> xpyz = source.GetImmutable(x_index + 1, y_index, z_index);
    std::pair<const int&, bool> xymz = source.GetImmutable(x_index, y_index - 1, z_index);
    std::pair<const int&, bool> xypz = source.GetImmutable(x_index, y_index + 1, z_index);
    std::pair<const int&, bool> xyzm = source.GetImmutable(x_index, y_index, z_index - 1);
    std::pair<const int&, bool> xyzp = source.GetImmutable(x_index, y_index, z_index + 1);
    // If one of the neighbors % 5, mark the fill grid
    int fives_neighbors = 0;
    if (xmyz.second && (xmyz.first % 5) == 0)
    {
        fives_neighbors++;
    }
    if (xpyz.second && (xpyz.first % 5) == 0)
    {
        fives_neighbors++;
    }
    if (xymz.second && (xymz.first % 5) == 0)
    {
        fives_neighbors++;
    }
    if (xypz.second && (xypz.first % 5) == 0)
    {
        fives_neighbors++;
    }
    if (xyzm.second && (xyzm.first % 5) == 0)
    {
        fives_neighbors++;
    }
    if (xyzp.second && (xyzp.first % 5) == 0)
    {
        fives_neighbors++;
    }
    output.SetValue(x_index, y_index, z_index, fives_neighbors);
    if (fives_neighbors > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void test_voxel_grid_omp()
{
    VoxelGrid::VoxelGrid<int> test_grid(0.25, 100.0, 100.0, 100.0, 0);
    VoxelGrid::VoxelGrid<int> fill_grid(0.25, 100.0, 100.0, 100.0, 0);
    struct timespec st, mt, et;
    clock_gettime(CLOCK_MONOTONIC, &st);
    // Load with special values
    int check_val = 1;
    #pragma omp parallel for schedule(guided)
    for (int64_t x_index = 0; x_index < test_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < test_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < test_grid.GetNumZCells(); z_index++)
            {
                test_grid.SetValue(x_index, y_index, z_index, check_val);
                #pragma omp atomic
                check_val++;
            }
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &mt);
    // Check the values
    int64_t sum = 0;
    #pragma omp parallel
    {
        #pragma omp single
        {
            for (int64_t x_index = 0; x_index < test_grid.GetNumXCells(); x_index++)
            {
                for (int64_t y_index = 0; y_index < test_grid.GetNumYCells(); y_index++)
                {
                    for (int64_t z_index = 0; z_index < test_grid.GetNumZCells(); z_index++)
                    {
                        int fives = 0;
                        #pragma omp task
                        fives = check_fives_neighbors(test_grid, x_index, y_index, z_index, fill_grid);
                        #pragma omp atomic
                        sum += fives;
                    }
                }
            }
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &et);
    // Compute times
    double fill_secs = (double)(mt.tv_sec - st.tv_sec);
    fill_secs += ((double)(mt.tv_nsec - st.tv_nsec) / 1000000000.0);
    double check_secs = (double)(et.tv_sec - mt.tv_sec);
    check_secs += ((double)(et.tv_nsec - mt.tv_nsec) / 1000000000.0);
    std::cout << "OPENMP TEST -- Sum: " << sum << " Fill took " << fill_secs << " seconds, check took " << check_secs << " seconds" << std::endl;
}

void test_voxel_grid()
{
    VoxelGrid::VoxelGrid<int> test_grid(0.25, 100.0, 100.0, 100.0, 0);
    VoxelGrid::VoxelGrid<int> fill_grid(0.25, 100.0, 100.0, 100.0, 0);
    struct timespec st, mt, et;
    clock_gettime(CLOCK_MONOTONIC, &st);
    // Load with special values
    int check_val = 1;
    for (int64_t x_index = 0; x_index < test_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < test_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < test_grid.GetNumZCells(); z_index++)
            {
                test_grid.SetValue(x_index, y_index, z_index, check_val);
                check_val++;
            }
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &mt);
    // Check the values
    int64_t sum = 0;
    for (int64_t x_index = 0; x_index < test_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < test_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < test_grid.GetNumZCells(); z_index++)
            {
                int fives = check_fives_neighbors(test_grid, x_index, y_index, z_index, fill_grid);
                sum += fives;
            }
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &et);
    // Compute times
    double fill_secs = (double)(mt.tv_sec - st.tv_sec);
    fill_secs += ((double)(mt.tv_nsec - st.tv_nsec) / 1000000000.0);
    double check_secs = (double)(et.tv_sec - mt.tv_sec);
    check_secs += ((double)(et.tv_nsec - mt.tv_nsec) / 1000000000.0);
    std::cout << "SERIAL TEST -- Sum: " << sum << " Fill took " << fill_secs << " seconds, check took " << check_secs << " seconds" << std::endl;
}

int main()
{
    std::cout << "OpenMP context has " << omp_test() << " threads" << std::endl;
    test_voxel_grid();
    test_voxel_grid();
    test_voxel_grid_omp();
    test_voxel_grid_omp();
    return 0;
}
