#include "otndc.h"
#include <iostream>
#include <vector>
#include <cmath>

// simpele helper om "ruwe" buisnormals te synthetiseren
static Vec3f make_normal(float yawDeg, float pitchDeg) {
    float yaw   = yawDeg   * 3.1415926535f / 180.0f;
    float pitch = pitchDeg * 3.1415926535f / 180.0f;
    float cy = std::cos(yaw);
    float sy = std::sin(yaw);
    float cp = std::cos(pitch);
    float sp = std::sin(pitch);
    // spherical -> cartesiaans
    Vec3f v { cp*cy, cp*sy, sp };
    // voor demo is normalisatie niet strikt nodig; runOTNDC_fromNormals normaliseert
    return v;
}

int main() {
    // Demo: normals rond een buis met as ongeveer langs X,
    // + wat noise normwalen in andere richtingen.
    std::vector<Vec3f> normals;
    normals.reserve(1000);

    // Een "buis" langs X → raakvlaknormals liggen ongeveer in YZ-vlak (straalkransvlak ⟂ as)
    for (int i=0;i<600;i++) {
        float angle = (float)i / 600.0f * 360.0f;
        // maak een cirkel in YZ vlak
        float rad = angle * 3.1415926535f / 180.0f;
        float cy = std::cos(rad);
        float sy = std::sin(rad);
        // normaal naar buis-as: (0, cy, sy) ± kleine storing
        Vec3f n { 0.01f*((float)std::sin(rad*3.1f)),
                  cy + 0.02f*((float)std::sin(rad*7.7f)),
                  sy + 0.02f*((float)std::cos(rad*5.3f)) };
        normals.push_back(n);
    }

    // Wat random noise-richtingen
    for (int i=0;i<200;i++) {
        float yaw   = (float)(i * 57 % 360);
        float pitch = (float)(i * 131 % 60) - 30.0f;
        normals.push_back(make_normal(yaw, pitch));
    }

    OTConfig cfg;
    cfg.maxSlots      = 8;
    cfg.maxDotPlane   = std::cos(80.0f * 3.1415926535f / 180.0f); // |dot| <= cos80°
    cfg.burstCos      = std::cos(5.0f  * 3.1415926535f / 180.0f); // cos5°
    cfg.alpha         = 0.02f;   // EMA-factor
    cfg.minCountAxis  = 5.0f;    // minimale "sterkte" om een as te rapporteren

    AssenResultaat out{};
    runOTNDC_fromNormals(normals.data(), (int)normals.size(), cfg, &out);

    std::cout << "Dimensie: " << out.dimensie << "\n";
    for (int i=0;i<out.dimensie;i++) {
        std::cout << "As " << i
                  << " = (" << out.v[i][0]
                  << ", "   << out.v[i][1]
                  << ", "   << out.v[i][2]
                  << "), score=" << out.score[i] << "\n";
    }

    return 0;
}
