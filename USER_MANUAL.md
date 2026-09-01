# User Manual — Safe Semantic Planner

This manual covers both provided interfaces: the C++ console program and
the interactive web visualizer.

## 1. C++ reference implementation

### 1.1 Requirements

- A C++17 compiler (`g++` or `clang++`). No external dependencies — the
  entire program is one self-contained file, `cpp/main.cpp`.
- Linux/macOS get process memory reporting via `getrusage`; Windows gets it
  via `GetProcessMemoryInfo` (both code paths are compiled in already,
  selected automatically by preprocessor checks).

### 1.2 Build and run

```bash
g++ -std=c++17 -O2 -o planner cpp/main.cpp
./planner
```

There are no command-line arguments — running the program prints all six
required test cases plus four additional safety edge cases in sequence,
then exits. Each case reports:

- the resulting state path and transition path (or failure),
- total cost and minimum safety clearance,
- how many states were explored during that particular `plan`/incremental
  update call,
- planning time in microseconds,
- the planner's own estimated memory footprint, and
- the process's peak resident set size (RSS).

Redirect stdout to a file if you want to keep a record, e.g.
`./planner > results.txt`. See `docs/experimental_output.txt` for a
reference capture.

### 1.3 Using the `Planner` interface in your own code

The required interfaces are declared at the top of `main.cpp` and can be
lifted into another program:

```cpp
LPAStarPlanner planner;                 // default kGeom = kTrans = 2.0
PlanningResult result = planner.plan(myProblem);

if (result.success) {
  // result.statePath, result.transitionPath, result.totalCost, result.safetyScore
}
```

For incremental updates without a full re-plan, use the dedicated methods
instead of calling `plan()` again:

| Situation | Call |
|---|---|
| A transition becomes available/unavailable | `planner.setTransitionAvailability(transitionId, available)` |
| A new transition appears | `planner.addTransition(newTransition)` |
| The goal changes | `planner.updateGoal(newGoalId)` |
| The bad-state set changes | `planner.updateBadStates(newBadStateIds)` |
| The safety weights change | `planner.setSafetyWeights(kGeom, kTrans)` |

Each of these returns a fresh `PlanningResult`, computed incrementally from
the planner's existing internal state rather than from scratch.

## 2. Interactive web visualizer

### 2.1 Running it

No build step, no npm install — it's static HTML/CSS/JS with one external
resource (Google Fonts, loaded over the network; everything else is local).

```bash
cd web
python3 -m http.server 8000
```

Then open `http://localhost:8000` in a browser. Opening `web/index.html`
directly via a `file://` URL also works.

### 2.2 Interface overview

![Interface overview](images/overview.png)

The screen is split into three panels:

**Left — Test Scenarios & Controls**
- Six preset buttons load the six assignment test cases (`case1`–`case6`).
  Clicking one replaces the current problem and immediately re-solves it.
- **Safety Objective Mode** toggle — flips both safety weights between
  their current values and `0`, letting you instantly compare "safety-aware"
  vs. "cost-only" planning on the loaded scenario.
- **Clearance Weight (kGeom)** and **Safety Weight (kTrans)** sliders —
  live-adjust the two safety-weight terms from the effective edge-weight
  formula (0–10, step 0.5). Moving a slider re-solves immediately.
- **Toggle Transition Availability** — flips one designated transition
  on/off (used for Test Case 4's dynamic-blocking demonstration) and
  triggers an incremental replan.

**Center — Interactive canvas**
- States are drawn at their Cartesian embedding coordinates; the start
  state is cyan, the goal is gold, hazardous ("bad") states are rose, and
  the currently optimal path is highlighted in green with an animated
  particle flowing along it.
- **Drag any node** to move it in Cartesian space — the planner re-solves
  in real time as you drag, since clearance-to-hazard and heuristic
  distances depend on the embeddings.
- **Scroll to zoom**, or use the `+`/`−`/`⟲` buttons in the bottom-right of
  the canvas (zoom in, zoom out, reset view).
- **Click-and-drag empty canvas space** to pan.
- The status badge (top-right of the canvas) reads `SUCCESS` or
  `NO PATH FOUND` depending on the current solve result.

**Right — Planning Diagnostics**
- **Optimal State Trajectory** — the solved path as a pill sequence.
- **Metric grid** — total base cost, minimum safety clearance, transition
  reliability, combined score, explored-state count, and planning time.
- **Planner Memory Footprint** — the same structural memory estimate the
  C++ program reports.
- **LPA\* Node States table** — every state's current `g(s)` and `rhs(s)`
  values and consistency status (Consistent / Overconsistent /
  Underconsistent / Bad State), for inspecting the search's internal state
  directly rather than only its final output.

### 2.3 Recommended walkthrough

1. Load **Test Case 3** (loaded by default) and note the selected path and
   its cost/clearance.
2. Drag both safety sliders to `0` and watch the planner switch to the
   cheaper, less-safe route — then drag them back to `2.0` to restore the
   original behavior.
3. Load **Test Case 4**, click **Toggle Transition Availability**, and
   watch the path reroute around the now-unavailable edge; click it again
   to restore the shortcut and see the path snap back.
4. Load **Test Case 2** and drag hazard state `X` closer to the safe path —
   watch the minimum safety clearance value and the path itself update live.
5. Open the **LPA\* Node States** table on any scenario and confirm every
   non-hazard state settles to `Consistent` (`g(s) == rhs(s)`) once the
   solve completes.

A fully worked version of this walkthrough, with screenshots at each step,
is in [`docs/DEMONSTRATION.md`](DEMONSTRATION.md).

## 3. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `g++: command not found` | Install a C++17-capable toolchain (e.g. `sudo apt install g++`, or Xcode Command Line Tools on macOS). |
| Web page loads with no fonts | The Google Fonts `<link>` tags require network access; the app still functions fully with system fallback fonts. |
| Canvas is blank after loading a scenario | Resize the browser window once, or click a scenario button again — `resizeCanvas()` runs on load and on window resize. |
| Dragging a node does nothing | Make sure you're grabbing within ~20px of the node center (the hit-test radius) rather than the label text below it. |
