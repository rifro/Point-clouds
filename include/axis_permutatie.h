#pragma once
#include <algorithm>
#include <cstdint>
#include <array>
#include <iostream>
#include "types.h"          // Vec3f, u32
#include "cc_adapter.h"     // CCVector3, CCPointCloud (forward wrappers)

/*
Doel
----

* Bepaal een as-permutatie zodat intern voor de GPU altijd Vx ≥ Vy ≥ Vz geldt
  (waar V* = aantal voxel-cellen / effectieve resolutie per as).
* Bewaar de *namen* van de originele assen zodat we na afloop labels
  en (optioneel) geroteerde punten terug kunnen mappen.

## Afspraak

* Identifiers NL, behalve seed/triangle/forest/tree/vote/count/slab.
* Deze header is *__host__-only* hulplogica.
  */

namespace Bocari {

struct AsInfo {
char   naam;   // 'x', 'y' of 'z' (originele naam)
float* data;   // __host__ buffer (x[], y[], z[]) — eigendom buiten deze struct
u32    V;      // effectieve voxelverdeling langs deze as (bv. Vx/Vy/Vz)
};

struct AxisPermutatie {
// volgorde newX,newY,newZ na sorteren op V aflopend
AsInfo nieuwX{};
AsInfo nieuwY{};
AsInfo nieuwZ{};


// inverse naam-mapping om later terug te mappen (‘x’->positie etc.)
// bijv: invNaam['x'] == 'z' betekent: originele ‘x’ kwam op plek ‘z’ terecht.
std::array<char, 256> invNaam{};


};

inline AxisPermutatie maak_axis_permutatie(AsInfo ax, AsInfo ay, AsInfo az)
{
// Sorteer aflopend op V (grotere resolutie eerst).
std::array<AsInfo,3> v = {ax, ay, az};
std::sort(v.begin(), v.end(), [](const AsInfo&a, const AsInfo&b){ return a.V > b.V; });


AxisPermutatie perm;
perm.nieuwX = v[0];
perm.nieuwY = v[1];
perm.nieuwZ = v[2];

// invNaam: voor elke originele naam, waar is hij terechtgekomen?
// newX.naam was bv ‘y’ → invNaam['y'] = 'x'
for (auto& r : perm.invNaam) r = 0;
perm.invNaam[(unsigned char)perm.nieuwX.naam] = 'x';
perm.invNaam[(unsigned char)perm.nieuwY.naam] = 'y';
perm.invNaam[(unsigned char)perm.nieuwZ.naam] = 'z';

std::cout << "[AxisPermutatie] volgorde: "
          << perm.nieuwX.naam << "→x, "
          << perm.nieuwY.naam << "→y, "
          << perm.nieuwZ.naam << "→z (op V)\n";
return perm;


}

// Hulpfunctie: druk mapping af
inline void toon_axis_mapping(const AxisPermutatie& P)
{
std::cout << "  nieuwX komt van '" << P.nieuwX.naam << "' (V=" << P.nieuwX.V << ")\n";
std::cout << "  nieuwY komt van '" << P.nieuwY.naam << "' (V=" << P.nieuwY.V << ")\n";
std::cout << "  nieuwZ komt van '" << P.nieuwZ.naam << "' (V=" << P.nieuwZ.V << ")\n";
std::cout << "  inverse naam: "
<< "orig 'x'→'" << P.invNaam[(unsigned char)'x'] << "', "
<< "orig 'y'→'" << P.invNaam[(unsigned char)'y'] << "', "
<< "orig 'z'→'" << P.invNaam[(unsigned char)'z'] << "'\n";
}

} // namespace Bocari