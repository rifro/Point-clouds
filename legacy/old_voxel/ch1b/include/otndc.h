#pragma once
#include <cuda_runtime.h>
#include <cstdint>

#include "types.cuh"   // verwacht Vec3f met { float x,y,z; }
#include "otndc_utils.cuh"  // helpers voor gewicht / vectoren

// ========================
// Resultaatstructuren
// ========================

// Resultaat van de assen-detectie (in wereldcoördinaten van de huidige cloud).
struct AssenResultaat {
    // 3 assen, unit vectoren: v[i][0..2] = (x,y,z)
    float v[3][3];
    // "sterkte" (gewogen stemmen) per as
    float score[3];
    // 1, 2 of 3 (hoeveel significante richtingen gevonden)
    int   dimensie;
};

// ========================
// OT-configuratie
// ========================
//
// Dit is de OTNDC-config zoals we 'm nu willen:
//
//  - maxSlots:     maximaal aantal richting-slots (typisch 8–12)
//  - maxDotAxis:   |dot(n, as)|-drempel voor "bijna loodrecht":
//                  bv cos(80°) ≈ 0.173 → normals bijna ⟂ as
//  - alpha:        basis EMA-factor voor bijsturen van de asrichting
//  - minCountAxis: minimale "sterkte" (gewogen stemmen) om een as te rapporteren
//  - sinPowK:      exponent voor sin²(θ) → gewicht:
//
//        sin²   → k = 1
//        sin⁴   → k = 2
//        sin⁸   → k = 3
//        sin¹⁶  → k = 4
//
//  Gewicht per straal wordt:
//      w = (sin²(θ))^(sinPowK)
//  met θ de hoek tussen inkomende normaal n en asrichting a.
//

struct OTConfig {
    int   maxSlots;       // aantal slots in de cache (max 16 in huidige kernel)
    float maxDotAxis;     // |dot(n, as)| ≤ maxDotAxis → kandidaat voor dit slot
    float alpha;          // basis EMA-factor voor nudge van asrichting
    float minCountAxis;   // minimale "sterkte" om een as te rapporteren
    int   sinPowK;        // exponent voor sin², typisch 3 of 4 (sin⁸ / sin¹⁶)
};

// ========================
// Device slot-structuur
// ========================
//
// Elk slot stelt één kandidaat-hoofdrichting voor:
//   n     = unit-vector (assenrichting)
//   count = gewogen stemmen (float wegens sin^k)

struct OTSlot {
    float3 n;     // asrichting
    float  count; // gewogen stemmen
};

// ========================
// Host-API voor OTNDC
// ========================
//
// Verwacht dat "normals" raakvlaknormals zijn (nog niet per se genormaliseerd).
// De host-variant kopieert de data naar device, roept de kernel aan,
// en vult AssenResultaat in.
//
// N: aantal normals
//
// runOTNDC_fromNormals:
//   - neemt host-array normals
//   - gebruikt OT-config
//   - schrijft top 1–3 assen + scores in 'out'

void runOTNDC_fromNormals(
    const Vec3f* normals,
    int N,
    const OTConfig& cfg,
    AssenResultaat* out);

// Lage-level variant: normals staan al op device (handig voor integratie
// met bestaande GPU-pipeline). De implementatie staat in otndc_host.cu.
void runOTNDC_fromNormals_device(
    const Vec3f* dNormals,
    int N,
    const OTConfig& cfg,
    AssenResultaat* out);
