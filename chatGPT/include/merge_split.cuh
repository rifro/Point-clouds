#pragma once
#include "types.cuh"

// ====== Doel ======
// Host levert drie afzonderlijke arrays (x[], y[], z[]) in "nieuwe as-volgorde"
// (na AxisPermutatie op de host).
// GPU merge't naar één Vec3f-buffer voor kernels die coalesced 3D willen.
// Later kan GPU weer splitten naar (x[],y[],z[]) zodat host makkelijk terug-permuteert.

// Merge: (x,y,z) → Vec3f
void h_merge_xyz_naar_vec3(const DeviceBuffer<float>& d_x,
const DeviceBuffer<float>& d_y,
const DeviceBuffer<float>& d_z,
DeviceBuffer<Vec3f>&       d_pts);

// Split: Vec3f → (x,y,z)
void h_split_vec3_naar_xyz(const DeviceBuffer<Vec3f>& d_pts,
DeviceBuffer<float>&       d_x,
DeviceBuffer<float>&       d_y,
DeviceBuffer<float>&       d_z);

// ====== Helpers voor AxisPermutatie ======
// Bouw hostzijde AxisPermutatie-array vanuit drie kolommen (x,y,z) die al
// hernoemd/gepermutateerd zijn.
inline void h_axis_permutatie_vul(AxisPermutatie (&ap)[3],
const float* nieuwX, const float* nieuwY, const float* nieuwZ,
char naamX, char naamY, char naamZ,
uint32_t Vx, uint32_t Vy, uint32_t Vz)
{
ap[0] = AxisPermutatie{ nieuwX, naamX, Vx };
ap[1] = AxisPermutatie{ nieuwY, naamY, Vy };
ap[2] = AxisPermutatie{ nieuwZ, naamZ, Vz };
// Volgorde ap[0]=X, ap[1]=Y, ap[2]=Z is wat GPU verwacht.
}