#pragma once
#include <stdint.h>

// Simpele 3D vector (host-kant)
struct Vec3f {
    float x, y, z;
};

// Resultaat van OTNDC: tot 3 assen + hun "sterkte"
struct AssenResultaat {
    float v[3][3];   // v[i][0..2] = richtingvector as i
    float score[3];  // hun gewichten / counts
    int   dimensie;  // 0..3 (1D/2D/3D)
};

// Config voor OTNDC op host
struct OTConfig {
    int   maxSlots;       // max aantal stralenkrans-vlakken (bv. 8 of 12)
    float maxDotPlane;    // |dot(n, vlakNormal)| <= deze => n ligt "in vlak" (cos(80°) ~ 0.1736)
    float burstCos;       // cos(hoekBurst); bv. cos(5°) ~ 0.9962 
    float alpha;          // EMA-basisfactor voor nudging (bv. 0.02f)
    float minCountAxis;   // minimale count om een as "significant" te noemen
};

// Hoofdingang: voer OTNDC uit op een array van genormaliseerde stralen (normals)
void runOTNDC_fromNormals(
    const Vec3f* normals,
    int N,
    const OTConfig& cfg,
    AssenResultaat* out);
