# Safe Semantic Planner (LPA*)

**PCCST503 — Machine Learning, Assignment 1**
*Design of a Safe Semantic Planner in a Finite Cartesian State Space*

![Overview](docs/images/overview.png)

A safety-aware path planner over a finite set of semantic states embedded in
2D Cartesian space. The planner is implemented twice from the same design:

| Implementation | Location | Purpose |
|---|---|---|
| **C++17 reference implementation** | [`cpp/main.cpp`](cpp/main.cpp) | Self-contained console program that implements the required `Planner` interface, runs all six assignment test cases, and reports timing/memory metrics using real process measurements (`getrusage`/`GetProcessMemoryInfo`). |
| **JavaScript port + interactive visualizer** | [`web/`](web/) | A pixel-for-pixel algorithmic port of the C++ planner (`planner.js`), driving a draggable, zoomable 2D canvas UI (`app.js`, `index.html`, `style.css`) so the search can be inspected and manipulated live in a browser. |

Both implementations share the same algorithm — **LPA\* (Lifelong Planning A\*)**
— and the same safety-aware edge-weight formulation, so results from one can
be used to sanity-check the other. See [`docs/EXPERIMENTAL_RESULTS.md`](docs/EXPERIMENTAL_RESULTS.md)
for a side-by-side comparison.

## Why LPA\*

The assignment's environment has a **fixed start state** and a **goal /
transition set that changes over time** (transitions can be added, removed,
or become unavailable; the goal can move). LPA\* searches forward from a
fixed start, so its `g`-values are goal-independent and survive a goal
change; only the priority-queue keys need to be recomputed. D\* Lite, by
contrast, searches backward from the goal, making its `g`-values
goal-dependent and therefore more expensive to reuse when the goal moves.
For this problem shape, LPA\* is the cheaper choice for incremental
replanning. The full justification is in [`docs/DESIGN_REPORT.md`](docs/DESIGN_REPORT.md).

## Safety objective

Every transition's effective search weight is:

```
w(e) = cost(e) + kGeom * (1 / d_safe(to)) + kTrans * (1 - safety(e))
```

- `d_safe(to)` — Euclidean distance from the destination state to the
  nearest bad (hazardous) state; the geometric clearance penalty grows as a
  path passes closer to a hazard.
- `safety(e)` — a per-transition safety attribute in `[0, 1]`; the penalty
  grows as a transition's own reliability/safety score drops.
- `kGeom`, `kTrans` — tunable weights (both default to `2.0`, and are
  live-adjustable sliders in the web UI).

Bad states are a **hard constraint**: they are structurally excluded from
the search (forced `g = rhs = ∞`, never enqueued), and a final pass
double-checks that no returned path visits one.

## Repository layout

```
.
├── cpp/
│   └── main.cpp          # C++17 reference implementation + 6 test cases
├── web/
│   ├── index.html        # UI shell
│   ├── style.css         # Visual styling (dark "glass" theme)
│   ├── planner.js        # LPA* planner — JS port of cpp/main.cpp
│   └── app.js            # Canvas rendering, presets, interaction wiring
└── docs/
    ├── DESIGN_REPORT.md          # Algorithm choice, math, correctness argument
    ├── EXPERIMENTAL_RESULTS.md   # Measured results for all 6 test cases
    ├── USER_MANUAL.md            # How to build/run/use both implementations
    ├── DEMONSTRATION.md          # Walkthrough with annotated screenshots
    ├── experimental_output.txt   # Raw stdout captured from cpp/main.cpp
    └── images/                   # Screenshots used across the docs
```

## Quick start

### C++ reference implementation

```bash
g++ -std=c++17 -O2 -o planner cpp/main.cpp
./planner
```

This prints all six test cases plus four additional safety edge cases, each
with the resulting path, total cost, minimum safety clearance, explored
state count, planning time (µs), and planner memory footprint — see
[`docs/EXPERIMENTAL_RESULTS.md`](docs/EXPERIMENTAL_RESULTS.md) for the
captured output.

### Interactive web visualizer

No build step or dependencies — it's static HTML/CSS/JS.

```bash
cd web
python3 -m http.server 8000
# then open http://localhost:8000 in a browser
```

(Opening `web/index.html` directly with a `file://` URL also works, since
there are no external module imports — only Google Fonts are loaded over
the network.)

Full usage instructions are in [`docs/USER_MANUAL.md`](docs/USER_MANUAL.md).

## Pushing to GitHub

```bash
git init
git add .
git commit -m "Safe Semantic Planner: LPA* implementation, visualizer, and docs"
git branch -M main
git remote add origin <your-repo-url>
git push -u origin main
```

## License

Coursework submission for PCCST503. Add a license of your choice before
making the repository public if required by your institution's policy.
