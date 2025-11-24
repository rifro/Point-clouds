# 🚀 **Bocari A–Z Pipeline: Definitieve Architectuur & Documentatie (Markdown)**

### **Voor gebruik door toekomstige LLM’s, developers, GPU-engineers & jezelf (host/GPU co-design).**

### *Met alle ontwerpbeslissingen, rationale, nomenclatuur, dataflows en semantiek.*

---

# 0. **Probleemdefinitie (in mensentaal + formeel)**

We willen uit een ruwe LiDAR puntenwolk:

* hoofdassen (ideaal frame) detecteren
* buizen, vlakken, t-stukken, kruizen, bochten, verloopstukken vinden
* consistent labelen
* alles robuust tegen ruis, gaten, occlusie, jitter
* alles GPU-vriendelijk (massaal parallelliseerbaar)
* alles snappen op 3 mm grid (fine) en 24 mm grid (coarse)
* CPU beslist topologische plausibiliteit
* GPU bevestigt materialen → labels

Het systeem moet betrouwbaar zijn voor:

* gladde metalen buizen
* isolatie (licht kreukelig → zwaar kreukelig → extreem elastisch foil)
* houten balken
* wanden/vlakken
* complexe knooppunten in technische ruimtes
* plekken met misregistraties of rare local distortions

En… **het moet allemaal *heel snel* en *heel stabiel* zijn.**

---

# 1. **Units & Numerieke Grondslag**

Het systeem werkt met **drie tegelijk gebruikte schalen**, voor maximale robuustheid:

| Domein                              | Eenheid           | Waarom                                                   |
| ----------------------------------- | ----------------- | -------------------------------------------------------- |
| **CUDA kernels / geometry kernels** | **meter (float)** | hoogste stabiliteit, geen overflow, float3 standaard     |
| **Raster-logica (host + GPU)**      | **3 mm (int32)**  | dichtst bij LiDAR-resolutie, integer exact               |
| **Radiale parameters**              | **mm (uint16)**   | buisdiameters, torusstralen, standards uit de industrie  |
| **Spatial hashing / morton**        | **mm (int)**      | integer precies, maximale ordening zonder floating drift |

**Dit is cruciaal.**
Meters → geometrische stabiliteit.
Integers → topologische stabiliteit.

---

# 2. **Dataflow Overzicht (macro)**

```
LiDAR cloud (float3 meters)
       │
       ▼
Morton sort (mm)
       │
       ▼
1D region slicing via Vx≥Vy≥Vz (vierkant-methode)
       │
       ▼
Boeren-ring filter (denoising) → triangels
       │
       ▼
Normals buffer (contiguous, mm-scale)
       │
       ▼
OTNDC (Prominent Directions Detection via Normal Position Vector Voting)
       │
       ▼
Ideaal frame (orthogonaal, 3 assen)
       │
       ▼
Rotate point cloud → ideal frame (meters)
       │
       ▼
Snap → 3mm grid (int32)
       │
       ▼
GPU: vind gladde buizen (slabs, radial fit)
       │
       ▼
Host: run topologie (PipeFittingDetector)
       │
       ▼
Hypothetical fittings → GPU confirm
       │
       ▼
GPU labeling per point
```

---

# 3. **De 1D Region Slicing: Vierkant-Methode (Vx≥Vy≥Vz)**

## 3.1 Waarom?

Een 3D volumetrische sortering (zoals morton) is snel, maar het is **veel efficiënter** als je daarna werkt in **1D slices** die “zo vierkant mogelijk” zijn.

De regels:

1. Bepaal bounding box in mm.
2. Bereken Δx, Δy, Δz.
3. Sorteer deze op grootte → Vx ≥ Vy ≥ Vz.
4. De eerste as wordt je **lange as** voor slicing.
5. We maken regions langs Vx, maar de doorsnede Vy×Vz moet “zo vierkant mogelijk” zijn.

### Worst-case aspect ratio = 1:2

→ Dit is een topologisch perfecte tradeoff:
voldoende klein voor local coherence, groot genoeg voor farming normals.

---

# 4. **Boeren-Ring Filter (Sliding Window Triangle Denoiser)**

## Boeren-ring filter – precieze JBF-versie

We werken per centrumpunt (p), met twee stralen:

* (r_1) = binnenstraal (te dichtbij → duplicaatachtig)
* (r_2) = buitenstraal (max zoekradius)

We gebruiken (r_1^2) en (r_2^2) (distance²).

### 1. Eén sweep over alle buren binnen (r_2)

Voor elk centrumpunt (p):

1. Initialisatie:

   ```text
   ringCount = 0
   ```

2. Loop over alle buren (q) met (|q - p|^2 <= r_2^2)
   (via sliding window op de Morton-gesorteerde lijst):

   * **Als (d^2 <= r_1^2)**
     → (q) is **ruis** (te dicht op het centrumpunt, conceptueel duplicaat van p).
     → Markeer (q) meteen als `LABEL_RUIS`.

   * **Als (r_1^2 < d^2 <= r_2^2)**
     → (q) valt in de **ring** [r_1, r_2]. ringCount++;

   * **Early exit mogelijkheid**
     Zodra `ringCount == 2` weten we:

     * p is **géén ruis**
     * p_1 is **géén ruis**
     * p_2 is **géén ruis**

     We **mogen** dan stoppen met zoeken voor dit centrumpunt p, omdat het filter zijn beslissing al genomen heeft: dit is lokaal een stukje oppervlak.

	 Als er geen twee punten p_1 en p_2 zijn met r_1 < d(p, p_1) <= r_2 en r_1 < d(p, p_2) <= r_2:
   * markeer p als ruis: geen samenhang met lokaal oppervlak.
	 * als er een p_1 is,markeer p_1 als ruis,
     Alles wat we tot nu toe binnen (r_1) hebben gezien is al als ruis gemarkeerd; 

  Als we **minder dan twee ringpunten** vinden:
  → alles binnen (r_2) is conceptueel ruis, maar praktisch hoeven we alleen:

  * (p)  en eventuele ene punt (p_1)als ruis te markeren,


Normaalgeneratie:

* Alleen in het geval `ringCount = 2` en alsde punten niet te colineair zijn, dan maken we een driehoek ((p,p_1,p_2)) en een normaal.
* Die normaal vormt de inputstroom voor OTNDC, via een buffer?

---

# 5. **Normals → OTNDC**

OTNDC = **Prominent Directions Detection via Normal Position Vector Voting**

(oude naam: Origin-Translated Normals Direction Consensus)

## Wat OTNDC doet:

* Neemt alle normals
* Clustert ze in 0-3+ grote “stralenkransvlakken”:

  * vlak A: normals die loodrecht staan op x
  * vlak B: normals die loodrecht staan op y
  * vlak C: normals die loodrecht staan op z
  * bv vlak met hoek van 45/135 graden

Maar dat gebeurt **zonder** te weten wat x,y,z zijn — de richtingen emergen uit de data!

## 5.1 De Radial (Normal Position) Vector Disk

(DeepSeek’s betere vertaling van StralenKransVlak)

Voor elk hoek-domein:

```
struct DiskSlot {
    float3 dir;     // gemiddelde normal
    float  count;   // weight
}
```

De slots worden geordend op descending count.

---

# 6. **Waarom Bubble Swaps?**

OTNDC update gebeurt realtime tijdens streaming of block-wise merging.

Wanneer een nieuw normal arrives:

1. We vinden welk slot i het beste past
2. Verhogen count[i] += weight
3. **Bubble swap**:

```
Vergelijk slot[i] met slot[i-1]
    - indien count[i] > count[i-1], swap
Vergelijk slot[i-1] met slot[i-2]
    - idem
```

→ Meer is **nooit** nodig.

### Waarom 2 swaps?

* Weight is klein.
* Een slot springt nooit meer dan 2 plaatsen omhoog.
* Je behoudt O(1) update per normal.
* Sortering blijft quasi-perfect.
* Helemaal GPU-vriendelijk.

---

# 7. **Sin-k tolerantie voor kreukeligheid**

Wij gebruiken:

```
sin(x)^(2*k)
```

waar k = 3,4,8 ter keuze:

| Kreukel-type      | k | Opmerking   |
| ----------------- | - | ----------- |
| glad              | 8 | heel streng |
| normaal           | 4 | aanbevolen  |
| extreem kreukelig | 3 | tolerant    |

Specifieerbaar in config.

---

# 8. **Rotatie naar Ideaal Frame**

OTNDC levert drie orthogonale richtingen (x*,y*,z*).

We bouwen hier een exacte rotatiematrix:

```
R = [ x* y* z* ]
```

en draaien de hele point cloud in **1 GPU pass**.

Daarna:

* 3D wereld = 3D ideaal frame
* Geen dubbel coordinate system meer
* Translatie: zet bbMin naar (0,0,0)

---

# 9. **Snap naar 3mm Grid (int32)**

Alle verdere logic werkt in integer 3mm units:

* P0.x = round(X / 0.003)
* P0.y = round(Y / 0.003)
* P0.z = round(Z / 0.003)

Voordelen:

* absolute stabiliteit
* geen floating rounding drift
* buizen precies colinair
* PipeFittingDetector werkt hier perfect samen

---

# 10. **GPU: vind gladde buizen**

We hebben nu:

* direction a*
* 3mm punten
* full cloud in ideal frame

Per as:

1. Zoek clusters via slab:

```
| (P - M) × a | <  (r + eps)
```

2. R bepalen:

   * via EMA op lokale slab-radiale afstand
3. M (steunpunt) bepalen:

   * via EMA op buckled surface points
4. P0,P1 bepalen:

   * projecteer extreme slab hits op as
   * gebruik radiale coherence check (stddev)

**Output = InputPipe (3mm grid)**

Direct door naar CPU.

---

# 11. **CPU Topologie: PipeFittingDetector (Grok)**

De detector werkt in **exact dezelfde struct** als onze buizen:

```
struct Pipe {
    Point3mm start;
    Point3mm end;
    uint16_t radius_mm;
    uint8_t  type;
    uint8_t  flags;
    ...
};
```

Dankzij jouw unified enum:

```
namespace PipeType {
    enum PipeTypeEnum : uint8_t {
        Unlabeled = 0,
        Noise     = 1,
        Plane     = 2,
        Pipe      = 3,
        Elbow     = 4,
        Tee       = 5,
        Cross     = 6,
        Reducer   = 7,
        Blind     = 8,

        Hypothetical = 128,
        GapFill      = Pipe | Hypothetical
    };
}
```

### Grok maakt nodes:

* Elbows
* Tees
* Crosses
* Reducers (variant 2: één per aftakking)
* GapFill (hypothetical pipes)
* Blind (dead end)

3D positions in 3mm grid.

### En allemaal in 44 bytes.

---

# 12. **Host stuurt hypothetische fittings terug naar GPU**

Host weet nu:

* welke buizen bestaan
* welke fitting-nodes er *moeten* zijn

GPU krijgt:

* alle fitting positions (3mm)
* type (Elbow, Tee, …)
* buisstraal
* torusstraal (ruw, door GPU te verfijnen)

---

# 13. **GPU bevestigt fitting**

Slab tests:

* voor Elbow: 1/4 torus detectie
* voor Tee: orthogonale slab intersecties
* voor Cross: dubbele orthogonale slabs
* voor Reducer: radiale binnen/buiten shells
* voor Blind: dead-end criteria

Als fitting 3D consistent is:

```
clear hypothetical bit
```

als fitting inconsistent:

```
laat hypothetical staan
```

---

# 14. **GPU labeling van alle punten**

Per punt:

1. Check of het in slab van buis valt
2. Check fitting shells
3. Check plane slabs
4. Check relative distances

Label:

```
label = (PipeTypeEnum) node.type;
cluster_id = node.id;
```

---

# 15. **Samenvattende Namen**
originele naam: boerenvlakstraal methode of boeren stralenvlak/stralenkrans methode.
originele "officiele" naam: OTNDC = Origin Translated Normals Direction Consensus

Best: Prominent Directions Detection via Voting on Radial Normal Position Vector Disks
of: Prominent Directions Detection via Radial Normal Position Vector Disks Voting

Alternatieven:

* Emergent Significant Directions from Normal Position Vector Voting
* Multiple Direction Detection via Normal Position Vector Voting

De juiste (door ons gekozen):

### ✔ **Prominent Directions Detection via Normal Position Vector Voting**

---

# 16. **Slotbeschouwing (voor toekomstige LLMs)**

Deze pipeline:

* is GPU-optimaal
* is industry-grade
* gebruikt integer topologie voor betrouwbaarheid
* is floating-metervoorbereidend voor geometrie
* is extreem tolerant voor ruis
* detecteert assen, buizen, fittings met minimale aanname
* laat de CPU de semantiek bepalen
* laat de GPU de geometrie bevestigen

**Dit document is alles wat een LLM nodig heeft om jouw werk direct voort te zetten.**

---

# 🟢 EINDE DOCUMENT

===
4. Jouw idee om GPU-confirmed → host zet hypothetical bit af

Ja, perfect systeem.

CPU → initieel:

buizen en nodes die pure topology zijn → hyp=1
GPU → bevestigt via slab/geometry:

host zet hyp=0 op basis van bit van GPU, dat moet een andere bit zijn dan we al hebben. Misschien bit 6 van PipeType?

Het omgekeerde gebeurt nooit:

een confirmed fysiek object wordt nooit hypothetisch gemaakt.

Dit maakt jouw pipeline herstelbaar en monotonic:
Alleen omhoog in zekerheid.

