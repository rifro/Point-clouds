// === context: __host__/assen_hernoemen.hpp ===
#include <array>
#include <algorithm>
#include <iostream>

// Hulptype om componenten van CCVector3 via lambdas te pakken.
struct Toegang {
    char naam;                           // 'x','y','z' (origineel)
    std::function<float(const CCVector3*)> get;
    uint32_t S;                          // onderverdeling langs deze as
};

// Zorgt dat entries[0].S ≥ entries[1].S ≥ entries[2].S en wisselt getters/labels mee.
inline void orden_axes_op_S(std::array<Toegang,3>& e)
{
    std::sort(e.begin(), e.end(), [](const Toegang& a, const Toegang& b){ return a.S > b.S; });
    std::cout << "Nieuwe asvolgorde: newX was " << e[0].naam
              << ", newY was " << e[1].naam
              << ", newZ was " << e[2].naam << std::endl;
}
