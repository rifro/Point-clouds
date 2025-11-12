#pragma once
#include &lt;vector&gt;
#include &lt;string&gt;
#include "types.cuh"

struct AxisPermutatie {
// host arrays (na permutatie)
std::vector<float> x;
std::vector<float> y;
std::vector<float> z;
char naamX='x', naamY='y', naamZ='z'; // oorspronkelijke namen
uint32_t Vx=0, Vy=0, Vz=0;            // #voxels per as (voor morton)
};

struct OrchestratieResultaat {
// hoofdrichtingen (na stemmen; niet-orthonormaal)
float3 richtA=make_float3(0,0,1);
float3 richtB=make_float3(1,0,0);
float3 richtC=make_float3(0,1,0);
bool   frameGevonden=false;
};

void doOptimizeFrame2(/* CloudCompare hook: m_selectedCloud etc. worden intern opgehaald */);

// helpers (geëxporteerd voor tests)
AxisPermutatie h_bereken_en_permuteer(const std::vector<float>& X,
const std::vector<float>& Y,
const std::vector<float>& Z,
float3 bbMin, float3 bbMax);