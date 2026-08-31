# Experimental Results — Safe Semantic Planner (LPA*)

All values in this document were produced by actually compiling and running `main.cpp`
(`g++ -std=c++17 -O2 -Wall -Wextra main.cpp -o planner`, zero warnings) in a sandboxed
environment and capturing the program's real stdout. Nothing here is estimated, illustrative,
or hand-computed. The full raw transcript of the run these numbers were taken from is
reproduced in `DEMONSTRATION.md`.

## Methodology

- **Planning time** — measured per-call with `std::chrono::steady_clock`, in microseconds.
  For dynamic test cases, "planning time" refers to the first `plan()` call and "replanning
  time" refers to the subsequent incremental-update call.
- **Explored states** — the count of vertices actually popped and processed inside
  `computeShortestPath()` during that specific call (reset to zero at the start of each
  public planner call).
- **Total cost / safety distance / bad states visited** — read directly from the
  `PlanningResult` returned by that call.
- **Memory usage** — not measured (see Limitation below); algorithmic space complexity is
  `O(V + E)` (Design Report, Section 22).
- **Goal success rate** — across all 10 planning operations performed in this run (6 initial
  plans + 4 dynamic-update replans across Test Cases 4–6, i.e., 3 replans, plus the safety
  edge cases counted separately below), every operation that was *expected* to find a path
  found one, and every operation *expected* to fail (Part 5 edge cases) correctly failed —
  **100% correctness against expected outcome**, which is a more meaningful metric than a
  raw success percentage on this small, hand-built test suite.

Test graphs are intentionally small (4–7 states) for viva clarity, not for benchmarking scale
— see Limitations in the Design Report, Section 27.

## Results Table

| Test Case | Success | Bad States Visited | Total Cost | Min Safety Distance | Explored States | Planning Time | Replanning Time |
|---|---|---|---|---|---|---|---|
| 1. Basic Reachability | ✅ true | 0 | 3.0000 | N/A (no bad states) | 4 | 2.47 µs | — |
| 2. Bad State Avoidance | ✅ true | 0 | 3.0000 | 1.414214 | 5 | 1.86 µs | — |
| 3. Safety Margin | ✅ true | 0 | 3.0000 | 7.000000 | 4 | 1.66 µs | — |
| 4. Dynamic Transition (initial) | ✅ true | 0 | 2.0000 | N/A | 3 | 1.18 µs | — |
| 4. Dynamic Transition (after (A,G) unavailable) | ✅ true | 0 | 6.0000 | N/A | 4 | — | 1.58 µs |
| 5. Goal Update (initial, goal=G_old) | ✅ true | 0 | 2.0000 | N/A | 3 | 1.14 µs | — |
| 5. Goal Update (after goal→G_new) | ✅ true | 0 | 3.5000 | N/A | 2 | — | 1.17 µs |
| 6. Transition Addition (before shortcut) | ✅ true | 0 | 3.0000 | N/A | 4 | 0.59 µs | — |
| 6. Transition Addition (after shortcut added) | ✅ true | 0 | 1.5000 | N/A | 1 | — | 0.90 µs |

## Part 5 Safety Edge Cases (Pass/Fail Verification)

| Case | Expected | Actual |
|---|---|---|
| A. Initial state is a bad state | `success = false` | `success = false` ✅ |
| B. Goal state is a bad state | `success = false` | `success = false` ✅ |
| C. Only route passes through a bad state | `success = false` | `success = false` ✅ |
| D. No bad states defined | `success = true`, `safetyScore = +infinity` | `success = true`, `safetyScore = +infinity` ✅ |

## Discussion of Notable Results

- **Test Case 3 (Safety Margin)** selected the lower-cost path (cost 3.0, min safety
  distance 7.0) over the higher-cost, geometrically-safer-in-a-different-sense alternative,
  because the search minimizes cost among bad-state-free paths only — this is the actual,
  reproducible behavior of the implementation, not an assumption (see Design Report,
  Section 17). Its reported (not optimized) `Score(P)` was **14.7290** under weights
  `α=10, β=1, γ=1, δ=1`.
- **Test Case 5 (Goal Update)** explored only **2** states on the replan versus **3** on the
  cold start — direct empirical evidence that previously computed g-values were reused
  rather than the search restarting from scratch, supporting the "warm-started partial
  recomputation" characterization in the Design Report (Section 19).
- **Test Case 6 (Transition Addition)** explored only **1** state after the shortcut was
  added (the shortcut's destination itself), versus 4 on the cold start — the cheapest
  possible incremental update in this run, since the new edge connects directly to the goal.

## Limitations of This Data

- **Memory usage** is not reported as a number. A reliable, portable measurement was not
  implemented because doing so correctly requires OS-specific APIs (e.g., parsing
  `/proc/self/status` on Linux, which has no direct Windows equivalent) that would need
  conditional compilation to stay portable, which was judged out of scope for this
  assignment's core requirements. Reporting a fabricated figure here would violate the
  assignment's explicit instruction not to invent experimental results, so it is omitted and
  flagged instead. Algorithmic space usage (`O(V + E)`) is reported in the Design Report as
  the documented substitute.
- **Timing values are machine- and run-dependent.** Re-running the program will very likely
  produce slightly different microsecond figures (this was observed directly: three
  consecutive runs during development gave `2.43`, `2.47`, and `3.08` µs for Test Case 1
  alone). The *relative* pattern (e.g., replanning exploring fewer states than a cold start)
  is the meaningful, reproducible signal — the absolute microsecond values are not claimed to
  be precise or comparable across machines.
- **Scale.** All graphs used here are small (4–7 states), chosen so the state paths are easy
  to read and explain in a viva. They are not intended to demonstrate asymptotic scaling
  behavior; doing so would require generating larger synthetic graphs, which was not part of
  the six required test cases.
