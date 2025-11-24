#include <iostream>
#include <vector>
#include <cuda_runtime.h>
#include "orchestratie.h"
#include "types.h"

template <typename T>
static T* gpu_malloc_copy(const std::vector<T>& h){
T* d=nullptr;
cudaMalloc(&d, h.size()*sizeof(T));
if(!h.empty()) cudaMemcpy(d, h.data(), h.size()*sizeof(T), cudaMemcpyHostToDevice);
return d;
}

int main()
{
// Demo: lege stub — in de echte plugin geef jij d_punten/d_mask/d_labels door.
std::vector<Vec3f> h_pts; // vul met jouw cloud
u32 N = (u32)h_pts.size();


Vec3f* d_punten = gpu_malloc_copy(h_pts);
u8* d_mask=nullptr;  // alles actief
PuntLabel* d_labels=nullptr;
cudaMalloc(&d_labels, N*sizeof(PuntLabel));
cudaMemset(d_labels, 0, N*sizeof(PuntLabel));

OrkestratieParams p;
FrameStatus frame;
DiscoverLog log;

bool ok = h_vind_ideaal_frame(d_punten, N, d_mask, d_labels, p, frame, log);
if(!ok){
    std::cout << "[demo] geen ideaal frame gevonden: " << log.verslag << "\n";
    return 0;
}
h_print_frame(frame);

h_rotatie_naar_ideaal(d_punten, N, frame);

h_fine_labeling(d_punten, N, d_labels, frame, p);

cudaFree(d_punten);
cudaFree(d_labels);
std::cout << "[demo] klaar.\n";
return 0;


}