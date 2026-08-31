# Safe Semantic Planner — PCCST503 Assignment 1

A C++17 implementation of a safe path planner over a finite Cartesian state space, built
around **LPA\* (Lifelong Planning A\*)**, for the assignment
*"Design of a Safe Semantic Planner in a Finite Cartesian State Space."*

## What's in this submission

| File | Contents |
|---|---|
| `main.cpp` | Complete, single-file C++17 implementation: required interfaces, the LPA* planner, all six required test cases, and the Part 5 safety edge cases. Compiles cleanly with `-Wall -Wextra` (zero warnings). |
| `DESIGN_REPORT.md` | The full 28-section design report: problem definition, architecture, data structures, algorithm, heuristic, safety handling, dynamic-environment strategy, complexity analysis, discussion, limitations. |
| `EXPERIMENTAL_RESULTS.md` | The results table and discussion, built entirely from a real, captured run of the compiled program — no fabricated or estimated figures. |
| `USER_MANUAL.md` | How to compile, run, and interpret the program's output, plus troubleshooting. |
| `DEMONSTRATION.md` | A scripted walkthrough of all six test cases with real output and viva talking points, for use as the "demonstration" deliverable. |
| `README.md` | This file. |

## Quick start

```
g++ -std=c++17 -O2 -Wall -Wextra main.cpp -o planner
./planner
```

No external libraries or build system are required.

## One-paragraph summary

The planner models states as vectors in `ℝᵈ`, transitions as directed edges carrying cost,
safety, reliability, and an availability flag, and computes the minimum-cost path from a
start state to a goal state that never touches a designated "bad" state — enforced as a
true hard constraint, not a penalty. **LPA\*** was chosen over the initially-considered
**D\* Lite** after identifying that this assignment's fixed start / changing goal setup
favors LPA*'s forward-search, goal-independent g-values over D\* Lite's backward-search,
goal-dependent ones — see `DESIGN_REPORT.md`, Section 26 for the full reasoning, and
`EXPERIMENTAL_RESULTS.md` for the empirical evidence (fewer explored states on the Test
Case 5 replan than on the cold start).

## Honesty notes carried through every document

- The search optimizes **total cost only**, among bad-state-free paths. Safety-distance and
  reliability are computed and reported, not folded into the search's edge weight — Test
  Case 3 demonstrates this concretely rather than asserting an outcome the code doesn't
  actually produce.
- Dynamic updates are explicitly split into **truly local/incremental** (transition
  availability, addition, removal, bad-state changes) versus **partial, warm-started
  recomputation, not free** (goal changes) — see `DESIGN_REPORT.md` Section 19.
- **Memory usage is not measured** and is not reported as an invented number; only
  algorithmic space complexity (`O(V + E)`) is given, with the limitation stated plainly in
  `EXPERIMENTAL_RESULTS.md`.
- All numbers in `EXPERIMENTAL_RESULTS.md` and `DEMONSTRATION.md` came from one actual
  compiled run, reproduced verbatim — timing values will vary slightly on re-run (expected
  and discussed); path/cost/explored-state values are deterministic and should match
  exactly.

## Suggested reading order

1. `README.md` (this file) — orientation.
2. `main.cpp` — the implementation itself; read top-to-bottom, it's organized in the same
   order as the assignment's own sections.
3. `USER_MANUAL.md` — compile and run it yourself.
4. `DEMONSTRATION.md` — walk through the six test cases with commentary.
5. `EXPERIMENTAL_RESULTS.md` — the results table and what it does and doesn't show.
6. `DESIGN_REPORT.md` — the full write-up, for submission or deep-dive review.
