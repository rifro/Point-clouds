#include "voxelprocessor.h"

namespace Bocari
{
    namespace Voxelprocessing
    {
        // Explicit instantiations
        template void buildStartIndices<c_voxel>(const std::span<u32>, Voxelprocessing::GridUnion<c_voxel>&, u32);
        template void buildCumSum<c_voxel>(const Voxelprocessing::GridUnion<c_voxel>&,
                                           Voxelprocessing::GridUnion<c_voxel>&);
        template u32  countPointsInBox<c_voxel>(const Voxelprocessing::GridUnion<c_voxel>&, u32, u32, u32, u32, u32,
                                                u32);

#if c_voxel != c_subvoxel
        template void buildStartIndices<c_subvoxel>(const std::span<u32>, Voxelprocessing::GridUnion<c_voxel>&, u32);
        template void buildCumSum<c_subvoxel>(const Voxelprocessing::GridUnion<c_subvoxel>&,
                                              Voxelprocessing::GridUnion<c_subvoxel>&);
        template u32  countPointsInBox<c_subvoxel>(const Voxelprocessing::GridUnion<c_voxel>&, u32, u32, u32, u32, u32,
                                                   u32);
#endif
    } // namespace Voxelprocessing
} // namespace Bocari
