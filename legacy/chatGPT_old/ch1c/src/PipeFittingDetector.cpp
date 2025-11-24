#include "PipeFittingDetector.h"

#include <unordered_map>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <optional>

namespace Bocari::PipeFitting {

using std::optional;
using std::nullopt;

struct PipeFittingDetector::Segment
{
    Point3mm a;
    Point3mm b;
    u16      buisStraal_mm{};
    u8       axis : 2;       // 0=X,1=Y,2=Z
    bool     hypothetical:1; // gap-fill/extensie
};

std::vector<Fitting> PipeFittingDetector::process(
    std::span<const Pipe> pipes,
    std::span<const Plane> planes,
    int max_gap_fill_mm,
    int max_extension_mm)
{
    std::vector<Segment> segs;
    segs.reserve(pipes.size());

    // Pipes normaliseren tot [a,b] lexicografisch en as bepalen
    for(const auto& p : pipes)
    {
        Point3mm a = p.start;
        Point3mm b = p.end;

        if( (a.x > b.x) ||
            (a.x == b.x && a.y > b.y) ||
            (a.x == b.x && a.y == b.y && a.z > b.z) )
        {
            std::swap(a, b);
        }

        u8 axis;
        if(a.x != b.x) axis = 0;
        else if(a.y != b.y) axis = 1;
        else axis = 2;

        Segment s;
        s.a = a;
        s.b = b;
        s.buisStraal_mm = p.buisStraal_mm;
        s.axis = axis;
        s.hypothetical = false;
        segs.push_back(s);
    }

    // Snap endpoints naar opgegeven vlakken (optioneel)
    snap_to_planes(segs, planes, max_extension_mm);

    // Segmenten per as mergen (aaneengesloten stukken)
    merge_overlapping(segs);

    // Gaten vullen (gap-fill, hypothetische pipes)
    fill_gaps(segs, max_gap_fill_mm);

    // Nodes clusteren + fittings bepalen
    std::vector<Fitting> fittings;
    cluster_nodes(segs, fittings);

    return fittings;
}

void PipeFittingDetector::snap_to_planes(std::vector<Segment>& segs,
                                         std::span<const Plane> planes,
                                         int max_ext_mm)
{
    if(planes.empty()) return;

    constexpr int MIN_GAP_UNITS = 10;       // 10 * 3mm = 30mm
    const int MAX_EXT_UNITS = max_ext_mm / 3;

    for(auto& s : segs)
    {
        for(bool start : {true,false})
        {
            Point3mm& p = start ? s.a : s.b;
            int bestDist = MAX_EXT_UNITS + 1;
            optional<Plane> bestPlane = nullopt;

            for(const auto& pl : planes)
            {
                i32 coord = (pl.axis == 0 ? p.x : (pl.axis == 1 ? p.y : p.z));
                int dist  = std::abs(pl.coord - coord);
                if(dist >= MIN_GAP_UNITS && dist < bestDist)
                {
                    bestDist  = dist;
                    bestPlane = pl;
                }
            }

            if(bestPlane)
            {
                const auto& pl = *bestPlane;
                if(pl.axis == 0)      p.x = pl.coord;
                else if(pl.axis == 1) p.y = pl.coord;
                else                  p.z = pl.coord;
                s.hypothetical = true;
            }
        }
    }
}

void PipeFittingDetector::merge_overlapping(std::vector<Segment>& segs)
{
    // Eenvoudige sort + merge per as
    std::sort(segs.begin(), segs.end(),
              [](const Segment& a, const Segment& b){
                  if(a.axis != b.axis) return a.axis < b.axis;
                  // sorteer op a.x+a.y+a.z als eenvoudige key
                  int sa = a.a.x + a.a.y + a.a.z;
                  int sb = b.a.x + b.a.y + b.a.z;
                  return sa < sb;
              });

    for(size_t i=0; i+1<segs.size();)
    {
        auto& s1 = segs[i];
        auto& s2 = segs[i+1];
        if(s1.axis == s2.axis &&
           s1.b.x == s2.a.x &&
           s1.b.y == s2.a.y &&
           s1.b.z == s2.a.z)
        {
            // merge s2 in s1
            s1.b = s2.b;
            s1.hypothetical = s1.hypothetical || s2.hypothetical;
            segs.erase(segs.begin() + (i+1));
        }
        else
        {
            ++i;
        }
    }
}

void PipeFittingDetector::fill_gaps(std::vector<Segment>& segs,
                                    int max_gap_mm)
{
    const int MAX_GAP_UNITS = max_gap_mm / 3;

    for(u8 axis=0; axis<3; ++axis)
    {
        std::vector<Segment*> line;
        for(auto& s : segs)
            if(s.axis == axis)
                line.push_back(&s);

        if(line.size() < 2) continue;

        std::sort(line.begin(), line.end(),
                  [](Segment* a, Segment* b){
                      int sa = a->a.x + a->a.y + a->a.z;
                      int sb = b->a.x + b->a.y + b->a.z;
                      return sa < sb;
                  });

        for(size_t i=0; i+1<line.size(); ++i)
        {
            Segment* s1 = line[i];
            Segment* s2 = line[i+1];

            int dx = s2->a.x - s1->b.x;
            int dy = s2->a.y - s1->b.y;
            int dz = s2->a.z - s1->b.z;
            int manhattanGap = std::abs(dx) + std::abs(dy) + std::abs(dz);

            if(manhattanGap > 10 && manhattanGap <= MAX_GAP_UNITS)
            {
                Segment gap;
                gap.a = s1->b;
                gap.b = s2->a;
                gap.buisStraal_mm = s1->buisStraal_mm; // zelfde diameter als hoofd
                gap.axis = axis;
                gap.hypothetical = true;
                segs.push_back(gap);
            }
        }
    }
}

u8 PipeFittingDetector::determine_elbow_plane(u8 axis1, u8 axis2) const noexcept
{
    // 0+1→2 (XZ), 0+2→1 (XY), 1+2→0 (YZ)
    return static_cast<u8>(3 - (axis1 + axis2));
}

Point3mm PipeFittingDetector::calculate_torus_center(const Segment* s1,
                                                     const Segment* s2,
                                                     u16 buisStraal_mm) const
{
    // Neem de node als gemeenschappelijk punt
    Point3mm node = s1->a; // aanname: a ligt dicht bij node

    u8 plane = determine_elbow_plane(s1->axis, s2->axis);

    // Buitenste punten (uiteinden van de segments)
    Point3mm outer1 = (s1->a == node ? s1->b : s1->a);
    Point3mm outer2 = (s2->a == node ? s2->b : s2->a);

    i32 sum_dx = outer1.x + outer2.x - 2 * node.x;
    i32 sum_dy = outer1.y + outer2.y - 2 * node.y;
    i32 sum_dz = outer1.z + outer2.z - 2 * node.z;

    // Ruwe schatting torusStraal: buisStraal + marge
    i32 offset_units = static_cast<i32>((buisStraal_mm + 44) / 3);

    Point3mm center = node;
    if(plane == 0)      center.y += (sum_dy > 0 ? offset_units : -offset_units);
    else if(plane == 1) center.z += (sum_dz > 0 ? offset_units : -offset_units);
    else                center.x += (sum_dx > 0 ? offset_units : -offset_units);

    return center;
}

// Hash voor Point3mm zodat we het als key in unordered_map kunnen gebruiken
struct Point3mmHash
{
    std::size_t operator()(const Point3mm& p) const noexcept
    {
        std::size_t h = static_cast<std::size_t>(p.x);
        h = h * 1315423911u + static_cast<std::size_t>(p.y);
        h = h * 1315423911u + static_cast<std::size_t>(p.z);
        return h;
    }
};

void PipeFittingDetector::cluster_nodes(const std::vector<Segment>& segs,
                                        std::vector<Fitting>& fittings)
{
    // tolerantie 12mm → 4 grid units → 4^2 = 16
    constexpr i32 MAX_DRIFT_SQ = 16;

    struct Cluster
    {
        Point3mm avg{};
        int      count = 0;
        std::vector<Point3mm> pts;

        void add(const Point3mm& p)
        {
            pts.push_back(p);
            avg.x += p.x;
            avg.y += p.y;
            avg.z += p.z;
            ++count;
        }
        void finalize()
        {
            if(count > 0)
            {
                avg.x /= count;
                avg.y /= count;
                avg.z /= count;
            }
        }
    };

    // Start met clusters per uniek punt
    std::unordered_map<Point3mm, Cluster, Point3mmHash> nodeMap;
    nodeMap.reserve(segs.size() * 2);

    for(const auto& s : segs)
    {
        nodeMap[s.a].add(s.a);
        nodeMap[s.b].add(s.b);
    }

    // Keys list voor pairwise merge
    std::vector<Point3mm> keys;
    keys.reserve(nodeMap.size());
    for(const auto& kv : nodeMap)
        keys.push_back(kv.first);

    // Union binnen 12mm
    for(size_t i=0; i<keys.size(); ++i)
    {
        for(size_t j=i+1; j<keys.size(); ++j)
        {
            auto it1 = nodeMap.find(keys[i]);
            auto it2 = nodeMap.find(keys[j]);
            if(it1 == nodeMap.end() || it2 == nodeMap.end())
                continue;

            const auto& p1 = it1->first;
            const auto& p2 = it2->first;
            i32 dx = p1.x - p2.x;
            i32 dy = p1.y - p2.y;
            i32 dz = p1.z - p2.z;
            i32 d2 = dx*dx + dy*dy + dz*dz;
            if(d2 <= MAX_DRIFT_SQ)
            {
                Cluster& big   = (it1->second.count >= it2->second.count) ? it1->second : it2->second;
                Cluster& small = (it1->second.count <  it2->second.count) ? it1->second : it2->second;
                for(const auto& p : small.pts)
                    big.add(p);
                nodeMap.erase( ( &small == &it1->second ) ? it1 : it2 );
            }
        }
    }

    for(auto& kv : nodeMap)
        kv.second.finalize();

    fittings.reserve(nodeMap.size() * 2);

    // Helper: alle segment-verbindingen voor een cluster
    auto collectConnections = [&](const Cluster& c) {
        std::vector<const Segment*> conns;
        for(const auto& s : segs)
        {
            for(const auto& p : c.pts)
            {
                if( (s.a == p) || (s.b == p) )
                {
                    conns.push_back(&s);
                    break;
                }
            }
        }
        return conns;
    };

    for(const auto& kv : nodeMap)
    {
        const Cluster& c = kv.second;
        auto conns = collectConnections(c);

        Fitting f{};
        f.pos = c.avg;

        if(conns.size() < 2)
        {
            f.setType(PT_Blind);
            fittings.push_back(f);
            continue;
        }

        // Grootste en kleinste straal
        u16 maxR = 0;
        u16 minR = std::numeric_limits<u16>::max();
        bool anyHyp = false;

        for(const Segment* s : conns)
        {
            maxR = std::max(maxR, s->buisStraal_mm);
            minR = std::min(minR, s->buisStraal_mm);
            if(s->hypothetical) anyHyp = true;
        }
        f.buisStraal_mm = maxR;
        f.setHyp(anyHyp);

        // Richtingen coderen (max 4)
        f.direction_count = static_cast<u8>(std::min<size_t>(conns.size(), 4));
        u8 axisMask = 0;
        for(size_t i=0; i<f.direction_count; ++i)
        {
            const Segment* s = conns[i];
            // bepaal of c.avg dichter bij a of b ligt
            i32 da = (c.avg.x - s->a.x)*(c.avg.x - s->a.x)
                   + (c.avg.y - s->a.y)*(c.avg.y - s->a.y)
                   + (c.avg.z - s->a.z)*(c.avg.z - s->a.z);
            i32 db = (c.avg.x - s->b.x)*(c.avg.x - s->b.x)
                   + (c.avg.y - s->b.y)*(c.avg.y - s->b.y)
                   + (c.avg.z - s->b.z)*(c.avg.z - s->b.z);

            Point3mm from = (da <= db ? s->a : s->b);
            Point3mm to   = (da <= db ? s->b : s->a);

            i32 vx = to.x - from.x;
            i32 vy = to.y - from.y;
            i32 vz = to.z - from.z;

            u8 axis = s->axis;
            int sign = 0;
            if(axis == 0) sign = (vx >= 0 ? 0 : 1);
            else if(axis == 1) sign = (vy >= 0 ? 0 : 1);
            else               sign = (vz >= 0 ? 0 : 1);

            u8 packed = static_cast<u8>((axis << 1) | (sign & 1));
            f.packed_directions |= static_cast<u16>(packed) << (i*3);
            axisMask |= static_cast<u8>(1u << axis);
        }

        // Type bepalen obv aantal richtingen en assen
        int axisCount = __builtin_popcount(axisMask);

        if(conns.size() == 2 && axisCount == 2)
        {
            f.setType(PT_Elbow);
            f.elbow_plane = determine_elbow_plane(conns[0]->axis, conns[1]->axis);
            f.elbow_center = calculate_torus_center(conns[0], conns[1], maxR);
            f.torusStraal_mm = static_cast<u16>(maxR + 50);
            f.elbow_corner = f.pos;
        }
        else if(conns.size() == 3 && axisCount == 2)
        {
            f.setType(PT_Tee);
        }
        else if(conns.size() == 4 && axisCount == 2)
        {
            f.setType(PT_Cross);
        }
        else if(anyHyp)
        {
            // puur topologische gapfill
            f.setType(PT_GapFill);
        }
        else
        {
            f.setType(PT_Blind);
        }

        // Variant 2 voor reducers:
        // Voor elke aansluiting met kleinere straal: apart Reducer-fitting
        if(f.type() == PT_Tee || f.type() == PT_Cross)
        {
            for(const Segment* s : conns)
            {
                if(s->buisStraal_mm < maxR)
                {
                    Fitting red = f;
                    red.setType(PT_Reducer);
                    red.reducer_to_buisStraal_mm = s->buisStraal_mm;

                    // Plaats reducer op uiteinde van dunne buis
                    // Neem het punt dat niet dicht bij f.pos ligt
                    i32 da = (f.pos.x - s->a.x)*(f.pos.x - s->a.x)
                           + (f.pos.y - s->a.y)*(f.pos.y - s->a.y)
                           + (f.pos.z - s->a.z)*(f.pos.z - s->a.z);
                    i32 db = (f.pos.x - s->b.x)*(f.pos.x - s->b.x)
                           + (f.pos.y - s->b.y)*(f.pos.y - s->b.y)
                           + (f.pos.z - s->b.z)*(f.pos.z - s->b.z);

                    red.pos = (da < db ? s->b : s->a);
                    fittings.push_back(red);
                }
            }
        }

        fittings.push_back(f);
    }
}

} // namespace Bocari::PipeFitting
