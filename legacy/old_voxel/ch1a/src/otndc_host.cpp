#include "otndc.h"
#include <cmath>
#include <algorithm>
#include <vector>

// Kleine helpers
static inline float dot3(const Vec3f& a, const Vec3f& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

static inline Vec3f add3(const Vec3f& a, const Vec3f& b) {
    return {a.x+b.x, a.y+b.y, a.z+b.z};
}

static inline Vec3f mul3(const Vec3f& a, float s) {
    return {a.x*s, a.y*s, a.z*s};
}

static inline float len2(const Vec3f& v) {
    return dot3(v, v);
}

static inline Vec3f norm3(const Vec3f& v) {
    float l2 = len2(v);
    if (l2 <= 1e-24f) return {0.f,0.f,0.f};
    float inv = 1.0f / std::sqrt(l2);
    return {v.x*inv, v.y*inv, v.z*inv};
}

static inline Vec3f cross3(const Vec3f& a, const Vec3f& b) {
    return {
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    };
}

static inline float clamp01(float x) {
    if (x < 0.f) return 0.f;
    if (x > 1.f) return 1.f;
    return x;
}

// Eén slot in de stralenkrans-vlak-cache
struct VlakSlot {
    Vec3f normal;  // genormaliseerde normaal van stralenkransvlak
    float count;   // gewogen stemmen (float, i.v.m. sin^16 en multipliers)
    bool  valid;
};

// Hoofd OTNDC implementatie
void runOTNDC_fromNormals(
    const Vec3f* normals,
    int N,
    const OTConfig& cfg,
    AssenResultaat* out)
{
    if (!out) return;

    // Init resultaat
    for (int i=0;i<3;i++) {
        out->v[i][0] = out->v[i][1] = out->v[i][2] = 0.f;
        out->score[i] = 0.f;
    }
    out->dimensie = 0;

    if (!normals || N <= 0 || cfg.maxSlots <= 0) {
        return;
    }

    // Vlak-cache
    const int S = cfg.maxSlots;
    std::vector<VlakSlot> slots(S);
    for (int i=0;i<S;i++) {
        slots[i].valid = false;
        slots[i].count = 0.f;
        slots[i].normal = {0.f,0.f,0.f};
    }

    // Vorige straal voor burst-detectie
    Vec3f prevN = {0.f,0.f,0.f};
    bool  prevValid = false;

    auto findMatchingSlot = [&](const Vec3f& n) -> int {
        // Zoek vlak waarvan normal bijna loodrecht op n staat:
        // |dot(n, nv)| <= maxDotPlane
        // Als er meerdere zijn, kies vlak met kleinste |dot|.
        int bestIdx = -1;
        float bestAbsDot = 1e9f;

        for (int i=0;i<S;i++) {
            if (!slots[i].valid) continue;
            float d = dot3(n, slots[i].normal);
            float ad = std::fabs(d);
            if (ad <= cfg.maxDotPlane) {
                if (ad < bestAbsDot) {
                    bestAbsDot = ad;
                    bestIdx = i;
                }
            }
        }
        return bestIdx;
    };

    auto planeExistsCloseTo = [&](const Vec3f& nNew, float cosThresh) -> bool {
        // Kijk of er al een vlak met vergelijkbare normaal is
        for (int i=0;i<S;i++) {
            if (!slots[i].valid) continue;
            float d = dot3(nNew, slots[i].normal);
            if (d > cosThresh) {
                return true; // hoek < arccos(cosThresh)
            }
        }
        return false;
    };

    int usedSlots = 0;

    for (int idx=0; idx<N; ++idx) {
        Vec3f n = normals[idx];
        // Normaliseer defensief (mocht inkomend niet exact unit zijn)
        n = norm3(n);
        if (len2(n) < 1e-12f) continue;

        // 1) Burst multiplier op basis van hoek met vorige straal
        float multiplier = 0.5f;
        if (prevValid) {
            float d = dot3(n, prevN);
            // d = cos(hoek(n, prevN))
            if (d > cfg.burstCos) {
                // hoek < burstAngle => vlak-burst: multiplier = 1.0
                multiplier = 1.0f;
            }
        }

        // 2) Zoek matchend stralenkransvlak
        int match = findMatchingSlot(n);

        if (match >= 0) {
            // ===== Case A: matchend vlak =====
            VlakSlot& v = slots[match];

            // Gewicht via sin^16:
            float cosNP = dot3(n, v.normal);
            cosNP = clamp01(std::fabs(cosNP)); // we gebruiken |cos|; symmetrisch
            float sin2 = 1.0f - cosNP*cosNP;   // sin^2
            if (sin2 < 0.f) sin2 = 0.f;

            float sin4 = sin2 * sin2;
            float sin8 = sin4 * sin4;
            float sin16 = sin8 * sin8;

            float w = sin16 * multiplier;

            // Voeg stem toe
            v.count += w;

            // Nudging via EMA richting n
            float beta = cfg.alpha * w;
            if (beta > 0.5f) beta = 0.5f; // veiligheid
            float oneMinus = 1.0f - beta;

            Vec3f newN = {
                oneMinus*v.normal.x + beta*n.x,
                oneMinus*v.normal.y + beta*n.y,
                oneMinus*v.normal.z + beta*n.z
            };
            v.normal = norm3(newN);

            // Bubble-achtig omhoog schuiven: alleen dit vlak kan "te groot" zijn
            int i = match;
            while (i > 0 && slots[i].valid && slots[i-1].valid &&
                   slots[i].count > slots[i-1].count)
            {
                std::swap(slots[i], slots[i-1]);
                --i;
            }
        } else {
            // ===== Case B: geen matchend vlak =====
            if (usedSlots < 3) {
                // In beginfase: direct vlak toevoegen met normal = n
                int pos = usedSlots;
                slots[pos].normal = n;
                slots[pos].count  = 1.0f;
                slots[pos].valid  = true;
                usedSlots++;
            } else {
                // Genereer nieuwe vlakken uit kruisen met top-3
                Vec3f cands[3];
                bool  candValid[3];

                for (int k=0;k<3;k++) {
                    candValid[k] = false;
                    if (!slots[k].valid) continue;
                    Vec3f nv = cross3(n, slots[k].normal);
                    float l2 = len2(nv);
                    if (l2 < 1e-10f) continue; // bijna parallel, negeren
                    nv = norm3(nv);
                    cands[k] = nv;
                    candValid[k] = true;
                }

                // Cosinusdrempel om duplicaten te vermijden: bv. cos(15°)
                const float cosDupl = std::cos(15.0f * 3.1415926535f / 180.0f);

                for (int k=0;k<3;k++) {
                    if (!candValid[k]) continue;

                    if (planeExistsCloseTo(cands[k], cosDupl)) {
                        // lijkt te veel op bestaand vlak → sla over
                        continue;
                    }

                    // Zoek plek: of eerste vrije slot, of evict laatste
                    int pos = -1;
                    for (int i=0;i<S;i++) {
                        if (!slots[i].valid) { pos = i; break; }
                    }
                    if (pos < 0) {
                        // cache vol → evict laatste
                        pos = S-1;
                    }

                    slots[pos].normal = cands[k];
                    slots[pos].count  = 1.0f;
                    slots[pos].valid  = true;
                    if (usedSlots < S) usedSlots++;

                    // NIEUW vlak komt achteraan / onderaan; geen bubble-sort nodig,
                    // want count=1 is nagenoeg minimum.
                }
            }
        }

        // update prevN
        prevN = n;
        prevValid = true;
    }

    // ===== Einde: bepaal assen uit slots =====
    // Verzamel alle geldige slots in een lijst, sorteer op count aflopend
    std::vector<VlakSlot> used;
    used.reserve(S);
    for (int i=0;i<S;i++) {
        if (slots[i].valid && slots[i].count >= cfg.minCountAxis) {
            used.push_back(slots[i]);
        }
    }

    if (used.empty()) {
        out->dimensie = 0;
        return;
    }

    std::sort(used.begin(), used.end(),
              [](const VlakSlot& a, const VlakSlot& b){
                  return a.count > b.count;
              });

    // Haal top 3 richtingen (de normals van stralenkransvlakken zijn de asrichtingen)
    int D = std::min(3, (int)used.size());
    out->dimensie = D;
    for (int i=0;i<D;i++) {
        Vec3f n = used[i].normal;
        out->v[i][0] = n.x;
        out->v[i][1] = n.y;
        out->v[i][2] = n.z;
        out->score[i] = used[i].count;
    }
}
