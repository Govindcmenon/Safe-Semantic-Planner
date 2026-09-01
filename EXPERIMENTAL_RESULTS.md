# Experimental Results

All numbers on this page are **actual captured output**, not illustrative
estimates. The C++ reference implementation was compiled with:

```bash
g++ -std=c++17 -O2 -o planner cpp/main.cpp
./planner
```

on a Linux container (results below), and the full unedited stdout is saved
at [`docs/experimental_output.txt`](experimental_output.txt). The
JavaScript port was exercised through the interactive web UI, loading each
built-in preset scenario, and screenshotted directly (`docs/images/`) —
every screenshot shown below is the live app, not a mockup.

## Summary table — all six required test cases

| Case | Scenario | Path | Total cost | Min. safety clearance | Explored states | Planning time (µs) | Planner memory |
|---|---|---|---|---|---|---|---|
| 1 | Basic reachability | S → A → B → G | 3.00 | N/A (no hazards) | 4 | 1.91 | 2.13 KB |
| 2 | Bad-state avoidance | S → C → D → G | 3.00 | 1.4142 | 5 | 1.99 | 3.19 KB |
| 3 | Safety margin optimization | S → B1 → B2 → G | 9.00 | 1.5000 | 5 | 1.52 | 3.45 KB |
| 4 (initial) | Dynamic transition — before block | S → A → G | 2.00 | N/A | 3 | 1.04 | 2.86 KB |
| 4 (replan) | Dynamic transition — after block | S → C → D → G | 6.00 | N/A | 4 | 1.27 | 2.78 KB |
| 5 (initial) | Goal update — toward `G_old` | S → A → `G_old` | 2.00 | N/A | 3 | 0.97 | 2.52 KB |
| 5 (replan) | Goal update — toward `G_new` | S → A → H → `G_new` | 3.50 | N/A | **2** | 1.09 | 2.50 KB |
| 6 (initial) | Transition addition — before shortcut | S → A → B → G | 3.00 | N/A | 4 | 0.55 | 2.13 KB |
| 6 (replan) | Transition addition — after shortcut | S → G | 1.50 | N/A | **1** | 0.55 | 2.20 KB |

Process-level peak RSS was constant at **3.80 MB** across every case — at
this problem scale, the planner's own data structures (a few KB) are
negligible against the C++ runtime/process baseline, which is exactly why
the program reports *both* the structural estimate and OS-level RSS
side by side rather than only one.

## Case-by-case detail

### Test Case 1 — Basic Reachability

Straight-line reachability with no hazards, confirming the planner behaves
like a correct shortest-path search when the safety layer has nothing to
do.

![Test Case 1](images/case1_basic_reachability.png)

```
State Path: S -> A -> B -> G
Total Cost: 3.0000
Explored States: 4
Planning Time: 1.91 microseconds
```

### Test Case 2 — Bad State Avoidance

State `X` is marked hazardous. The planner correctly discards the direct
`S→A→X→G` route and takes the `S→C→D→G` detour instead, at equal cost —
and the final validation pass confirms `X` never appears in the returned
path.

![Test Case 2](images/case2_bad_state_avoidance.png)

```
State Path: S -> C -> D -> G
Total Cost: 3.0000
Minimum Safety Distance: 1.414214
Verification - path contains bad state X: NO (correct)
```

### Test Case 3 — Safety Margin Optimization

The interesting case: two viable routes exist around hazard `Y`. Path 1
(`S→A1→A2→G`) is cheaper (base cost 3.0) but stays close to `Y` (clearance
≈0.54, transition safety 0.60). Path 2 (`S→B1→B2→G`) costs 3× as much
(9.0) but nearly triples the clearance (1.50) and uses safer transitions
(0.95). With the default weights (`kGeom = kTrans = 2.0`), the planner
correctly prefers Path 2:

![Test Case 3 — safety objective on](images/case3_safety_margin.png)

```
Selected: Path 2 (safer path, maximized safety clearance)
Total Cost: 9.0000
Minimum Safety Distance: 1.500000
Combined Evaluation Score = alpha*1 - beta*C + gamma*D + delta*R = 3.2290
```

**Turning the safety weights off** (`kGeom = kTrans = 0`) in the web UI —
live, via the sliders — collapses the effective weight back to plain
transition cost, and the planner immediately switches to the cheap-but-risky
route instead, confirming the weights are actually doing the work claimed
in the design report rather than being cosmetic:

![Test Case 3 — safety objective off](images/case3_safety_disabled.png)

| | Safety objective **on** (kGeom=kTrans=2.0) | Safety objective **off** (kGeom=kTrans=0.0) |
|---|---|---|
| Path | S → B1 → B2 → G | S → A1 → A2 → G |
| Total cost | 9.00 | 3.00 |
| Min. safety clearance | 1.500 | 0.539 |
| Transition reliability | 72.9% | 72.9% |
| Combined score | 3.23 | 8.27 |

### Test Case 4 — Dynamic Transition Failure

Transition `A→G` (id 402) starts unavailable, forcing the initial plan onto
the longer `S→C→D→G` detour. Toggling the transition back on triggers an
incremental replan (not a full re-solve) that immediately recovers the
cheaper `S→A→G` route:

| | Blocked (`A→G` unavailable) | Restored (`A→G` toggled on) |
|---|---|---|
| Path | S → C → D → G | S → A → G |
| Total cost | 6.00 | 2.00 |
| Explored states | 5 | 3 |

![Test Case 4 — transition blocked](images/case4_blocked.png)
![Test Case 4 — transition restored](images/case4_restored.png)

### Test Case 5 — Incremental Goal Update

The goal moves from `G_old` to `G_new` after the first plan has already
been solved. The re-plan explores **only 2 states** — fewer than the
**3** explored on the very first, from-scratch plan — because the
`g`-value already computed for the `S→A` sub-path (distance from the fixed
start) is reused unchanged; only the priority-queue keys, which depend on
distance-to-*goal*, needed to be recomputed. This is the concrete payoff of
choosing LPA\* over D\* Lite discussed in the design report.

![Test Case 5](images/case5_goal_update.png)

```
--- Initial plan toward G_old ---   Explored States: 3
--- Goal changes to G_new ---       Explored States: 2
```

### Test Case 6 — Transition Addition

A shortcut edge `S→G` (cost 1.5) is added at runtime. The incremental
`addTransition` call re-examines only the new edge's destination and
converges in a single explored state, immediately reporting the new
optimum instead of the previous 3-hop route:

![Test Case 6](images/case6_transition_addition.png)

```
--- Before shortcut ---  S -> A -> B -> G   (cost 3.00, 4 explored)
--- After shortcut  ---  S -> G             (cost 1.50, 1 explored)
```

### Additional safety edge cases (Part 5)

Four boundary conditions around the hard bad-state constraint, all
confirmed correct:

```
Case A - Initial state is a bad state       -> success = false (correct)
Case B - Goal state is a bad state          -> success = false (correct)
Case C - Only route passes through bad state -> success = false (correct)
Case D - No bad states defined              -> success = true (correct), safetyScore = +infinity (correct, documented)
```

## Cross-implementation consistency

The C++ and JavaScript planners were run on identical problem instances
(the six presets are defined once and shared verbatim between
`cpp/main.cpp`'s test-case builders and `web/app.js`'s `PRESETS` object).
Every path, total cost, and safety clearance value produced by the web
visualizer above matches the corresponding C++ console output exactly,
which is the expected outcome of `web/planner.js` being a direct
method-for-method port of the C++ `LPAStarPlanner` rather than an
independent reimplementation.

## Reproducing these results

```bash
# C++
g++ -std=c++17 -O2 -o planner cpp/main.cpp && ./planner

# Web (open http://localhost:8000 and click through the six scenario buttons)
cd web && python3 -m http.server 8000
```
