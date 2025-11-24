#pragma once

// Pas onderstaande includes aan naar jouw lokale CloudCompare pad.
// Deze adapter biedt minimale typedefs/bridge die in h_do_optimize_frame3.cpp worden gebruikt.

#include <vector>
#include <string>

struct CCVector3 {
float x, y, z;
};

struct CCPointCloud {
std::vector<CCVector3> punten;
size_t size() const { return punten.size(); }
const CCVector3* getPoint(size_t i) const { return &punten[i]; }
void getBoundingBox(CCVector3& mn, CCVector3& mx) const {
if (punten.empty()){ mn={0,0,0}; mx={0,0,0}; return; }
mn = mx = punten[0];
for (auto &p: punten){
if (p.x<mn.x) mn.x=p.x; if (p.y<mn.y) mn.y=p.y; if (p.z<mn.z) mn.z=p.z;
if (p.x>mx.x) mx.x=p.x; if (p.y>mx.y) mx.y=p.y; if (p.z>mx.z) mx.z=p.z;
}
}
};

// In jouw plugin: vervang bovenstaande dummy door echte CC includes,
// of maak een converteerfunctie die jouw CC cloud omzet naar deze dummy-structs.