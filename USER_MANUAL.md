# User Manual — Safe Semantic Planner (LPA*)

## Requirements

- A C++17-compliant compiler (GCC ≥ 7, Clang ≥ 5, or MSVC 2017+).
- No external libraries — only the C++ standard library is used. Nothing else needs to be
  installed.
- Tested with `g++ 13.3.0` on Ubuntu 24.04; expected to work unmodified on Windows (MinGW),
  macOS, and other Linux distributions, since no OS-specific APIs are used.

## Installation / Setup

1. Save `main.cpp` into an empty folder.
2. No build system, package manager, or configuration file is required — it is a single
   translation unit.

## Compilation

**Linux / macOS:**
```
g++ -std=c++17 -O2 -Wall -Wextra main.cpp -o planner
```

**Windows (MinGW / MSYS2):**
```
g++ -std=c++17 -O2 -Wall -Wextra main.cpp -o planner.exe
```

**Windows (MSVC, Developer Command Prompt):**
```
cl /std:c++17 /O2 /EHsc main.cpp /Fe:planner.exe
```

**Antigravity / any generic C++17 environment:** the same `g++` command above works
unmodified — the program has no environment-specific dependencies.

The code compiles cleanly with `-Wall -Wextra` and produces **zero warnings**.

## Execution

**Linux / macOS:**
```
./planner
```

**Windows:**
```
planner.exe
```

The program takes no command-line arguments and requires no input files — it runs all six
required test cases plus the Part 5 safety edge cases automatically and prints results to
standard output. Redirect to a file if you want to save the transcript:
```
./planner > results.txt
```

## Program Structure

The single file `main.cpp` is organized top-to-bottom as:

1. **Required interfaces** — `State`, `Transition`, `PlanningProblem`, `PlanningResult`,
   `Planner` (abstract base class).
2. **`euclideanDistance()`** — the one place geometric distance is computed.
3. **`LPAStarPlanner`** — the full algorithm implementation: `plan()` (the required
   `Planner` interface method) plus five incremental-update methods
   (`setTransitionAvailability`, `addTransition`, `removeTransition`, `updateGoal`,
   `updateBadStates`) and two measurement getters (`lastExploredStates()`,
   `lastOperationMicros()`).
4. **`reportedScore()`** — the informational (not optimized) `Score(P)` metric.
5. **Output-formatting helpers** — `printHeader`, `pathToString`, `transitionsToString`,
   `printResult`.
6. **Test case builders** — `runTestCase1()` … `runTestCase6()`, plus
   `runSafetyEdgeCases()`.
7. **`main()`** — runs every test case in order.

## Running Test Cases

All six test cases and the safety edge cases run automatically every time you execute the
compiled program — there is nothing to select or configure. Each test case prints its own
clearly delimited section, in the order: Test Case 1 → 2 → 3 → 4 → 5 → 6 → Safety Edge
Cases.

To inspect or modify a single scenario, locate its `runTestCaseN()` function in `main.cpp`
and edit the `PlanningProblem` it constructs (states, transitions, `initialState`,
`goalState`, `badStates`).

## Understanding the Output

For each successful plan, the program prints:

- **State Path** — the sequence of state names/ids from start to goal.
- **Transition Path** — the sequence of transition ids used, in order.
- **Total Cost** — sum of the `cost` field over the transitions used; this is the exact
  quantity the search minimized.
- **Minimum Safety Distance** — the smallest Euclidean distance, over all states in the
  path, to the nearest bad state; `N/A` when no bad states are defined for that problem.
- **Bad States Visited** — always `0` for a successful result (this is checked and printed
  explicitly as a direct verification of the hard safety constraint).
- **Explored States** — how many states the search actually processed during that specific
  call (useful for comparing a cold plan against a subsequent incremental replan).
- **Planning Time** — wall-clock microseconds for that specific call.

For a failed plan (`Result: FAILURE`), only `Explored States` and `Planning Time` are
printed — there is no path, cost, or safety score to report, since none exists.

## Dynamic Replanning Demonstration

Test Cases 4, 5, and 6 each perform an initial `plan()` call and then one incremental
update call, printing both results back-to-back so you can directly compare:

- **Test Case 4** calls `setTransitionAvailability(402, false)` after the first plan.
- **Test Case 5** calls `updateGoal(4)` after the first plan.
- **Test Case 6** calls `addTransition(shortcut)` after the first plan.

In each case, compare the `Explored States` count between the "before" and "after" blocks —
a lower count on the "after" block is direct evidence that the incremental update reused
previously computed search state rather than restarting from scratch.

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `error: 'uint64_t' was not declared` | Compiler is not being invoked in C++17 mode — make sure `-std=c++17` (or later) is passed. |
| Compiler warnings about narrowing conversions | Not expected with the provided code as-is; if you've edited a `PlanningProblem` builder and see this, check that literal numbers match the declared field types (e.g., use `1.0` not `1` for `double` fields where the compiler is strict). |
| Program prints `FAILURE` where you expected `SUCCESS` | Check that every state referenced by a transition's `from`/`to` actually appears in `PlanningProblem::states`, and that the transition's `available` flag is `true`. |
| Timing values look inconsistent between runs | Expected — see the Limitations section in `EXPERIMENTAL_RESULTS.md`; microsecond timings on small graphs are noisy and machine-dependent. Path, cost, and explored-state counts are deterministic and should match exactly. |
| Want to test a larger/custom graph | Write a new function following the pattern of `runTestCase1()` … `runTestCase6()`, build your own `PlanningProblem`, and call it from `main()`. |
