#include &lt;iostream&gt;
#include &lt;vector&gt;
#include &lt;cstdint&gt;
#include &lt;memory&gt;

#include "cc_adapter.h"            // CCVector3, CCPointCloud*
#include "device_buffer.cuh"       // DeviceBuffer<T>
#include "types.h"                 // Vec3f, u32
#include "axis_permutatie.h"       // AxisPermutatie
#include "orchestratie.h"          // NL orchestratie-functies
#include "consts.cuh"              // thresholds/constanten
#include "merge_split.cuh"         // h_merge_xyz_naar_vec3 / h_split_vec3_naar_xyz

// Kernels/host-stappen (geleverde modules)
extern void h_voxeliseer_morton(/*in*/const Bocari::DeviceBuffer<Bocari::Vec3f>& d_pts,
/*io*/Bocari::DeviceBuffer<u64>& d_keys,
/*io*/Bocari::DeviceBuffer<u32>& d_index,
/*out*/Bocari::DeviceBuffer<Bocari::Vec3f>& d_pts_sorted);

extern void h_ruis_boeren(/*io*/Bocari::DeviceBuffer<Bocari::Vec3f>& d_pts_sorted,
/*io*/Bocari::DeviceBuffer<u8>& d_labels);

extern void h_vlak_vote_current_frame(/*in*/const Bocari::DeviceBuffer<Bocari::Vec3f>& d_pts_sorted,
/*io*/Bocari::DeviceBuffer<u8>& d_labels,
/*out*/Bocari::KandidaatAssen& uit);

extern void h_vlak_vote_bruteforce(/*in*/const Bocari::DeviceBuffer<Bocari::Vec3f>& d_pts_sorted,
/*io*/Bocari::DeviceBuffer<u8>& d_labels,
/*out*/Bocari::KandidaatAssen& uit);

// NB: buis_vote happy path pas inzetten als nodig
extern void h_buis_vote_bruteforce(/*in*/const Bocari::DeviceBuffer<Bocari::Vec3f>& d_pts_sorted,
/*io*/Bocari::DeviceBuffer<u8>& d_labels,
/*out*/Bocari::KandidaatAssen& uit);

extern void h_rotatie_quat_apply(/*io*/Bocari::DeviceBuffer<Bocari::Vec3f>& d_pts_sorted,
/*in*/const Bocari::Quat& q);

extern void h_fine_slabs_cyl_label(/*io*/Bocari::DeviceBuffer<Bocari::Vec3f>& d_pts_sorted,
/*io*/Bocari::DeviceBuffer<u8>& d_labels);

// ============= HOOFDFLOW =============

void AutofitImpl::doOptimizeFrame3()
{
using namespace Bocari;

```
if (!m_selectedCloud || m_selectedCloud-&gt;size() == 0)
{
    std::cout &lt;&lt; "[Autofit] Geen point cloud of leeg.\n";
    return;
}

// 1) Host: lees CC-punten en bepaal as-verdeling (Vx/Vy/Vz) + permutatie
CCVector3 bbMin, bbMax;
m_selectedCloud-&gt;getBoundingBox(bbMin, bbMax);

const size_t N = m_selectedCloud-&gt;size();
std::vector&lt;float&gt; h_x(N), h_y(N), h_z(N);

for (size_t i=0; i&lt;N; ++i)
{
    const CCVector3* p = m_selectedCloud-&gt;getPoint(i);
    if (!p) continue;
    h_x[i] = p-&gt;x;
    h_y[i] = p-&gt;y;
    h_z[i] = p-&gt;z;
}

// Bepaal Vx,Vy,Vz (bijv via morton-driven voxelisatie ontwerp; hier simpele placeholder:
// neem verhouding van BB-lengtes t.o.v. target gridgrootte)
const float Lx = bbMax.x - bbMin.x;
const float Ly = bbMax.y - bbMin.y;
const float Lz = bbMax.z - bbMin.z;

// Ruime schatting: grotere lengte → grotere V
u32 Vx = std::max(1u, (u32)(Lx &gt; 0 ? 1024.f * (Lx / std::max({Lx,Ly,Lz})) : 1));
u32 Vy = std::max(1u, (u32)(Ly &gt; 0 ? 1024.f * (Ly / std::max({Lx,Ly,Lz})) : 1));
u32 Vz = std::max(1u, (u32)(Lz &gt; 0 ? 1024.f * (Lz / std::max({Lx,Ly,Lz})) : 1));

Bocari::AsInfo ax{ 'x', h_x.data(), Vx };
Bocari::AsInfo ay{ 'y', h_y.data(), Vy };
Bocari::AsInfo az{ 'z', h_z.data(), Vz };

AxisPermutatie perm = maak_axis_permutatie(ax, ay, az);
toon_axis_mapping(perm);

// 2) Upload nieuwe x/y/z volgens permutatie
DeviceBuffer&lt;float&gt; d_x(N), d_y(N), d_z(N);
d_x.upload(perm.nieuwX.data, N);
d_y.upload(perm.nieuwY.data, N);
d_z.upload(perm.nieuwZ.data, N);

// 3) Merge naar vec3f (device) → d_pts
DeviceBuffer&lt;Vec3f&gt; d_pts(N);
h_merge_xyz_naar_vec3(d_x, d_y, d_z, d_pts);

std::cout &lt;&lt; "[Autofit] Merge x/y/z → vec3f gedaan. Start pipeline…\n";

// 4) Morton-driven sort + boeren ruisfilter
DeviceBuffer&lt;u64&gt; d_keys(N);
DeviceBuffer&lt;u32&gt; d_index(N);
DeviceBuffer&lt;Vec3f&gt; d_pts_sorted(N);
DeviceBuffer&lt;u8&gt;  d_labels(N); d_labels.memset(0);

h_voxeliseer_morton(d_pts, d_keys, d_index, d_pts_sorted);
h_ruis_boeren(d_pts_sorted, d_labels);

// 5) Probeer huidig frame (3 richtingen) → vlakken
KandidaatAssen assen{};
h_vlak_vote_current_frame(d_pts_sorted, d_labels, assen);

if (!assen.heeftMinstensEenAs())
{
    std::cout &lt;&lt; "[Autofit] Geen vlak in huidig frame. Brute planes…\n";
    h_vlak_vote_bruteforce(d_pts_sorted, d_labels, assen);
}

if (!assen.heeftMinstensEenAs())
{
    std::cout &lt;&lt; "[Autofit] Geen vlak gevonden. Brute buizen (annulus/wedge)…\n";
    h_buis_vote_bruteforce(d_pts_sorted, d_labels, assen);
}

if (!assen.heeftMinstensEenAs())
{
    std::cout &lt;&lt; "[Autofit] Geen eerste as te vinden (vlakken/buizen). Stop.\n";
    return;
}

// 6) Orthonormaliseer 2e/3e as (JBF → Gram–Schmidt), maak quaternion
Quat q = maak_quat_naar_ideaal_frame(assen); // in orchestratie.h/.cpp

std::cout &lt;&lt; "[Autofit] Roteer naar ideaal frame (quaternion)…\n";
h_rotatie_quat_apply(d_pts_sorted, q);

// 7) Fine slabs + (simpele) radiale buislabeling in ideaal frame
h_fine_slabs_cyl_label(d_pts_sorted, d_labels);

// 8) Split terug: vec3f → x/y/z
h_split_vec3_naar_xyz(d_pts_sorted, d_x, d_y, d_z);

// 9) Download en TERUG-PERMUTEER naar oorspronkelijke as-namen
std::vector&lt;float&gt; rx(N), ry(N), rz(N);
d_x.download(rx.data(), N);
d_y.download(ry.data(), N);
d_z.download(rz.data(), N);

// Terug-mapping: we hebben invNaam: orig 'x'→'?' (x/y/z)
auto zetTerug = [&](char orig, const std::vector&lt;float&gt;&amp; nx,
                    const std::vector&lt;float&gt;&amp; ny,
                    const std::vector&lt;float&gt;&amp; nz, float&amp; out)
{
    char nieuwe = perm.invNaam[(unsigned char)orig];
    if (nieuwe == 'x') out = nx[&amp;out - &amp;out]; // niet bruikbaar zo; we schrijven rechtstreeks per punt
};

// Simpeler: schrijf direct de juiste component per punt
for (size_t i=0; i&lt;N; ++i)
{
    float X = rx[i], Y = ry[i], Z = rz[i];
    // welke originele as kwam op nieuwX?
    // perm.nieuwX.naam == 'y' betekent: oude 'y' staat nu in X
    float out_x=0, out_y=0, out_z=0;
    // inv map: welke nieuwe letter hoort bij orig?
    // We bepalen per nieuwe component naar welke originele naam die hoort:
    // nieuwX kwam van naamA → schrijf X terug naar die orig naamA
    auto schrijf = [&](char kwamVan, float waarde) {
        if (kwamVan=='x') out_x = waarde;
        else if (kwamVan=='y') out_y = waarde;
        else if (kwamVan=='z') out_z = waarde;
    };
    schrijf(perm.nieuwX.naam, X);
    schrijf(perm.nieuwY.naam, Y);
    schrijf(perm.nieuwZ.naam, Z);

    // Update terug in CC cloud (alleen pos; labels kun je separaat ophalen)
    CCVector3* p = m_selectedCloud-&gt;getPointMutable(i);
    if (p) { p-&gt;x = out_x; p-&gt;y = out_y; p-&gt;z = out_z; }
}

std::cout &lt;&lt; "[Autofit] Klaar: punten teruggezet in originele as-namen.\n";
```

}