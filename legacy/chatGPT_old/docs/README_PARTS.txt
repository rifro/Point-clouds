# GPU-autofit — Part 08

Deze part voegt toe:

* `include/axis_permutatie.h` (as-permutatie host, NL identifiers).
* Integratie merge/split:

  * `h_merge_xyz_naar_vec3()` vóór de pipeline,
  * `h_split_vec3_naar_xyz()` na labeling/rotatie.
* Update van `h_do_optimize_frame3.cpp` met duidelijke statusregels (S0 happy path):

  1. CC cloud → host x/y/z
  2. as-permutatie zodat Vx ≥ Vy ≥ Vz
  3. upload d_x/d_y/d_z → merge naar d_pts
  4. morton sort → boeren ruisfilter
  5. vlak-check huidig frame; zonodig brute planes; zonodig brute buizen
  6. quaternion-rotatie naar ideaal frame
  7. fine slabs + simpele cilinderlabel in ideaal frame
  8. split vec3→x/y/z → download → terug-permuteer namen

Build:

```
bash scripts/build_nv.sh
```

Volgende parts:

* Vervang placeholder `maak_quat_naar_ideaal_frame()` door jouw mat→quat logica (consistent met `k_rotatie_quat.cu`).
* Werk S1 (langzame, algemene aanpak) verder in h_orchestratie en h_vlak_vote/h_buis_vote.