# Design Report — Safe Semantic Planner in a Finite Cartesian State Space

**Course:** PCCST503 — Machine Learning
**Assignment 1:** Design of a Safe Semantic Planner in a Finite Cartesian State Space
**Selected Algorithm:** LPA* (Lifelong Planning A*)

---

## 1. Title

Design and Implementation of a Safe Semantic Planner in a Finite Cartesian State Space using LPA*.

## 2. Objective

The objective, as stated in the assignment, is to design and implement a generic planning
algorithm that computes a safe path in a finite Cartesian state space, emphasizing graph
search, heuristic design, optimization, software engineering, and experimental evaluation.

## 3. Problem Definition

The planner operates over a finite state set `S = {s₁, ..., sₙ}`, each state embedded in
`ℝᵈ` as a vector `sᵢ = (x₁, ..., x_d)`. The planner is given an initial state `sI`, a goal
state `sG`, a set of bad states `B = {b₁, ..., bₖ}` that must never be visited, and a set of
directed transitions `T = {(sᵢ, sⱼ)}`. The planner must compute a sequence of transitions
from `sI` to `sG` that avoids every state in `B`.

## 4. State Representation

Implemented exactly as specified:

```cpp
struct State {
    uint64_t id;
    std::vector<double> embedding;
};
```

`id` uniquely identifies the state; `embedding` holds its coordinate vector in `ℝᵈ`. No
additional fields were added to this struct — dimensionality `d` is implicit in the length
of `embedding` and is never hard-coded, so the same code works for any `d`.

## 5. Cartesian State Space

States exist in `ℝᵈ`. Geometric distance between any two states is computed once, in one
place, by `euclideanDistance()`:

```
d(a, b) = sqrt( Σ (aᵢ − bᵢ)² )
```

This function is reused for two distinct purposes in the implementation: (a) the safety
distance metric (Section 15 below), and (b) the heuristic scale factor (Section 13). If two
states are given embeddings of different lengths, the function compares only the common
prefix of dimensions rather than crashing — a defensive fallback, not the expected path,
since a correctly formed problem instance gives every state the same dimensionality.

## 6. Transition Representation

Implemented exactly as specified, with all four required attributes:

```cpp
struct Transition {
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;
    double reliability;
    bool available;
};
```

`available` is the field toggled by dynamic-environment updates (Section 18). `safety` is
retained as specified even though the search does not use it as an edge weight (see
Section 16) — it is preserved for completeness and potential future use, exactly as
instructed ("do not remove required fields just because the algorithm can work without
them").

## 7. Planning Problem

```cpp
struct PlanningProblem {
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};
```

This is the sole input to `LPAStarPlanner::plan()`, matching the `Planner` interface's
`plan(const PlanningProblem&)` signature from the PDF.

## 8. Planning Result

```cpp
struct PlanningResult {
    bool success;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost;
    double safetyScore;
};
```

`safetyScore` holds the minimum Euclidean distance from any state on the returned path to
the nearest bad state (Part 6 of the assignment), or `+infinity` if no bad states exist —
this convention is documented at the point of definition and used consistently everywhere
the value is read or printed.

## 9. System Architecture

The implementation follows the layered architecture proposed in the design phase, collapsed
into a single class (`LPAStarPlanner`) for the parts that are always used together, to avoid
over-engineering a project of this size:

- **Graph storage** — `statesById_`, `transitions_`, `outIdx_`/`inIdx_` (adjacency by
  transition index).
- **Safety filter** — `badStates_` plus the exclusion logic embedded in `updateVertex()` and
  `reconstructResult()`.
- **Heuristic evaluator** — `heuristicScale_` and `heuristic()`.
- **LPA\* search core** — `g_`, `rhs_`, `openList_`, `calculateKey()`, `updateVertex()`,
  `computeShortestPath()`.
- **Update manager** — the five public incremental-update methods
  (`setTransitionAvailability`, `addTransition`, `removeTransition`, `updateGoal`,
  `updateBadStates`).
- **Path reconstruction and validation** — `reconstructResult()`.
- **Evaluation** — `lastExploredStates()`, `lastOperationMicros()`, and the free function
  `reportedScore()`.

These are implemented as clearly delimited private sections and public methods of one
class rather than as separate classes/files, since splitting them further would add
indirection without a real benefit at this problem size — consistent with the instruction
not to over-engineer the project.

## 10. Data Structures

| Purpose | Structure used | Why |
|---|---|---|
| State lookup | `unordered_map<uint64_t, State>` | O(1) lookup by id. |
| Transition storage | `vector<Transition>` | Master list; attributes stay together. |
| Outgoing/incoming adjacency | `unordered_map<uint64_t, vector<size_t>>` (id → indices into `transitions_`) | LPA* needs predecessor access (`inIdx_`) for `rhs` computation and successor access (`outIdx_`) for propagation — both O(1) to fetch. |
| Bad-state lookup | `unordered_set<uint64_t>` | O(1) membership test on the search hot path. |
| Open list (priority queue) | `std::set<tuple<double,double,uint64_t>>` + `unordered_map<uint64_t, pair<double,double>>` for current-key lookup | LPA* requires decrease/increase-key semantics that `std::priority_queue` does not support directly; an ordered `set` keyed by `(k1, k2, id)` gives O(log n) insert/erase/find-min, and the side map lets `updateVertex` find and remove a vertex's old entry before reinserting it with a new key. |
| g / rhs values | Two `unordered_map<uint64_t, double>` | Directly mirrors the algorithm's own definition of per-vertex g and rhs. |
| Path reconstruction | Backward walk using `inIdx_` at each step, no separate parent map | The predecessor that achieves `rhs(u) = g(p) + cost(p,u)` is recomputed on demand during reconstruction rather than stored, keeping the search core free of a redundant parent map that would need to be kept in sync on every update. |

## 11. Selected Algorithm

**LPA\*** (Lifelong Planning A*), not D\* Lite. See Section 26 (Discussion) for the full
justification and the correction of the earlier D\* Lite recommendation.

## 12. Algorithm Working

LPA* maintains two values per vertex: `g(s)` (the current best known cost from `start` to
`s`) and `rhs(s)` (a one-step lookahead value based on `g` of `s`'s predecessors). A vertex
is *consistent* when `g(s) == rhs(s)`; inconsistent vertices are kept in a priority queue
ordered by key `(min(g,rhs) + h(s), min(g,rhs))`.

`computeShortestPath()` repeatedly pops the minimum-key vertex `u` and:
- If `g(u) > rhs(u)` (u was too expensive), sets `g(u) = rhs(u)` and calls `updateVertex`
  on every successor of `u` (their `rhs` may now improve).
- Otherwise (`g(u) < rhs(u)`, `u` was optimistic), sets `g(u) = ∞` and calls `updateVertex`
  on `u` itself and every successor (their `rhs` may now need to worsen).

The loop terminates once the goal is locally consistent and no queued vertex has a smaller
key than the goal's key — at that point `g(goal)` holds the true shortest safe-path cost.

`updateVertex(u)` recomputes `rhs(u)` from `u`'s available, non-bad incoming edges, removes
`u` from the open list, and reinserts it only if it is now inconsistent. Every dynamic update
in this implementation is expressed as one or more calls to `updateVertex` followed by a
resumed `computeShortestPath()` — this is what makes the updates incremental rather than a
restart.

## 13. Heuristic Function

Raw Euclidean distance is **not** assumed admissible, because the PDF permits arbitrary
transition costs with no stated relationship to geometric distance. Instead:

```
cMin = min over all known transitions t of ( t.cost / euclideanDistance(from, to) )
h(s) = cMin * euclideanDistance(s, goal)
```

Since `cMin` is the smallest cost-per-unit-distance ever observed on any transition,
`cMin * d(s, goal)` can never exceed the true cheapest possible cost to the goal — this is
exactly the admissibility condition A*/LPA* require. If no transition yields a usable ratio
(e.g., an empty transition set, or all-zero geometric distances), the implementation falls
back to `h = 0`, which is trivially admissible (this degrades LPA* to Dijkstra-like
behavior — correct, just less informed).

`cMin` is computed once from the full transition set in `plan()`, and is refreshed
(tightened, never loosened) whenever `addTransition()` introduces a new edge, so the
heuristic remains admissible after dynamic updates as well.

## 14. Safety Computation

Handled in two independent places, deliberately overlapping for robustness:

1. **Structural exclusion during search** — `updateVertex()` forces any vertex in
   `badStates_` to `g = rhs = ∞` and never inserts it into the open list. Incoming-edge
   scanning also skips any predecessor that is itself a bad state. This means a bad state
   cannot acquire a finite cost and cannot become part of any candidate path the search
   ever considers.
2. **Final validation before returning success** — `reconstructResult()` walks the
   completed path and checks every state against `badStates_` a second time, independent of
   how the path was built, before setting `success = true`. This satisfies the assignment's
   explicit instruction to "perform a final safety validation of the complete path" before
   returning success.

No penalty term of any kind is used — bad states are excluded, not discouraged.

## 15. Cost Handling

`totalCost` in the returned `PlanningResult` is the sum of the `cost` field of every
transition on the reconstructed path. This is the same quantity LPA*'s `g`/`rhs` values are
defined over, so `totalCost == g(goal)` for a successful plan by construction — the reported
cost is exactly the value the search actually minimized, not a separately computed
approximation.

## 16. Reliability Handling

Reliability is **not** part of the search's edge weight. It is accumulated only after a
result exists, by `reportedScore()`, as the product of the `reliability` values of the
transitions on the path (a natural composition rule for independent-probability-like
quantities, though the PDF does not mandate a specific combination rule, so this choice is
documented as a design decision, not a stated requirement). Reliability therefore currently
has no influence over which path LPA* returns — it is an evaluation/reporting quantity only.

## 17. Safety/Cost Trade-off

The implemented objective minimizes total transition cost **among bad-state-free paths
only**. The geometric minimum-safety-distance objective (objective 4 in the PDF) is computed
and reported per result, but is not part of the edge weight the search minimizes. Test
Case 3 makes this trade-off concrete and is the authoritative demonstration of this
behavior — see Section 23 and the Experimental Results document.

This is a deliberate, documented scope decision: folding safety-distance into the edge
weight (e.g., as a secondary weighted term, as sketched during the design phase) is possible
future work, but doing so without care would risk breaking the heuristic's admissibility or
the algorithm's stated optimality guarantee — so it was left out rather than implemented
partially and described as something it is not.

## 18. Dynamic Environment

Five kinds of environment change are supported, via five public methods on
`LPAStarPlanner`. See Section 19 for which are truly local and which are not.

| Change | Method | Mechanism |
|---|---|---|
| Transition availability flips | `setTransitionAvailability(id, bool)` | Flags the transition, calls `updateVertex` on its `to` endpoint. |
| New transition added | `addTransition(Transition)` | Appends to adjacency, refreshes `cMin`, calls `updateVertex` on the new edge's `to` endpoint. |
| Transition removed | `removeTransition(id)` | Marks unavailable, strips from adjacency, calls `updateVertex` on the former `to` endpoint. |
| Goal changes | `updateGoal(newGoal)` | Recomputes every queued vertex's key under the new heuristic target, then resumes the search. |
| Bad-state set changes | `updateBadStates(vector<uint64_t>)` | For every state whose bad/safe status changed, calls `updateVertex` on it and on its immediate neighbors. |

## 19. Replanning Strategy

Every update above ends with a call to `computeShortestPath()`, which resumes processing the
open list from wherever it currently stands — it never clears `g_`/`rhs_` and starts over.
Honesty about cost, as required by the assignment:

- **Truly local / incremental**: transition availability, addition, and removal. Each
  touches only the vertices adjacent to the changed edge; cost scales with local degree, not
  graph size.
- **Local but scales with the number of changed states**: bad-state updates. Cost scales
  with the number of states whose bad/safe status actually changed (plus their immediate
  neighbors), not the whole graph.
- **Partial, warm-started recomputation — not local, not a full rebuild**: goal updates.
  `g_`/`rhs_` values computed so far remain valid (they represent distance from the fixed
  start, which does not depend on the goal), but every vertex currently in the open list has
  its key recalculated for the new goal, and the search may need to expand vertices it had
  not touched before. Test Case 5's actual run (Section 23) shows this in practice: the
  second plan explored **2** states versus **3** for the first cold plan — cheaper than a
  cold start, but not free.

## 20. Algorithm / Pseudocode

```
function Initialize(start, goal):
    g(s) = inf, rhs(s) = inf for all s
    rhs(start) = 0
    U.insert(start, CalculateKey(start))

function CalculateKey(s):
    m = min(g(s), rhs(s))
    return (m + h(s, goal), m)

function UpdateVertex(u):
    if u is a bad state: g(u) = rhs(u) = inf; U.remove(u); return
    if u != start:
        rhs(u) = min over available, non-bad-predecessor edges (p, u) of ( g(p) + cost(p,u) )
    U.remove(u)
    if g(u) != rhs(u): U.insert(u, CalculateKey(u))

function ComputeShortestPath():
    while U not empty and (U.topKey() < CalculateKey(goal) or rhs(goal) != g(goal)):
        u = U.popMin()
        if g(u) > rhs(u):
            g(u) = rhs(u)
            for each successor s of u: UpdateVertex(s)
        else:
            g(u) = inf
            UpdateVertex(u)
            for each successor s of u: UpdateVertex(s)
```

The C++ implementation follows this pseudocode directly — see `updateVertex()` and
`computeShortestPath()` in `main.cpp`.

## 21. Time Complexity

Let `V` = number of states, `E` = number of transitions.

- **First (cold) `plan()` call**: `O((V + E) log V)`, the standard bound for a
  heap/tree-based A*-family search — every vertex may be inserted/removed from the open
  list at most a bounded number of times, each operation `O(log V)`.
- **Local incremental updates** (availability / add / remove / bad-state changes): bounded
  by `O(k log V)`, where `k` is the number of vertices that actually become inconsistent as
  a result of the change — typically small and local, not proportional to `V`. In the worst
  case (a change that propagates through a long dependency chain), this can still degrade to
  `O((V+E) log V)`.
- **Goal updates**: `O(|U| log |U|)` to recompute keys for the current open list, plus
  whatever further expansion (bounded by the same `O((V+E) log V)` worst case) is needed to
  reach and settle the new goal.

## 22. Space Complexity

`O(V + E)`: the state map, the transition vector, two adjacency-index maps, two g/rhs maps,
and the open list all scale linearly with the number of states and transitions.

## 23. Test Cases

All six required test cases are implemented as executable scenarios in `main.cpp`
(`runTestCase1()` … `runTestCase6()`), plus a seventh block (`runSafetyEdgeCases()`) covering
the explicit edge cases from Part 5 of the assignment (bad initial state, bad goal state, no
safe path exists, no bad states defined). Full actual output is captured in
`EXPERIMENTAL_RESULTS.md` and `DEMONSTRATION.md`. Summary of what each test demonstrates:

1. **Basic Reachability** — correctness of the core search and path reconstruction on a
   trivial linear graph.
2. **Bad State Avoidance** — the hard constraint holds even when the bad-state path would
   otherwise be viable; verified programmatically (path does not contain the bad state).
3. **Safety Margin** — demonstrates, honestly, that the implemented objective picks the
   lower-cost path over the geometrically safer one, since safety-distance is reported, not
   optimized.
4. **Dynamic Transition** — availability flip triggers a correct, local replan to an
   alternative route.
5. **Goal Update** — demonstrates warm-started replanning with fewer explored states than a
   cold start.
6. **Transition Addition** — a new shortcut is correctly discovered and adopted since it
   strictly lowers total cost.

## 24. Experimental Methodology

Each test case builds a small, hand-constructed `PlanningProblem`, invokes the planner via
`plan()` and/or the incremental update methods, and reads back: `success`, `statePath`,
`transitionPath`, `totalCost`, `safetyScore`, plus two measurement hooks —
`lastExploredStates()` (count of vertices popped and processed inside
`computeShortestPath()` during that specific call) and `lastOperationMicros()`
(`std::chrono::steady_clock` wall time for that call only). All values reported in
`EXPERIMENTAL_RESULTS.md` come directly from one recorded run of the compiled program; none
are hand-computed or estimated.

**Memory usage** is explicitly *not* fabricated: no reliable, portable, dependency-free
cross-platform memory measurement was implemented (this would require OS-specific APIs —
e.g., `/proc/self/status` on Linux, which is not portable to Windows without conditional
compilation). Instead, algorithmic space usage is reported analytically (Section 22), and
this limitation is stated plainly rather than filled with an invented number.

## 25. Experimental Results

See `EXPERIMENTAL_RESULTS.md` for the full results table and discussion.

## 26. Discussion

The most consequential design decision in this project was the mid-project correction from
D\* Lite to LPA\*. D\* Lite's efficient handling of change is built around a **moving start**
(searching backward from a fixed goal, so that g-values are goal-independent and
start-position changes are cheap). This assignment's start never moves; instead the *goal*
changes (Test Case 5) and the graph's edges change (Test Cases 4, 6). LPA\*'s forward search
from a fixed start makes its g-values goal-independent instead, which is the property this
assignment's Test Case 5 actually needs. The corrected choice is validated empirically: Test
Case 5's second `plan()` call explored fewer states than its first cold call, confirming that
previously computed g-values were genuinely reused, not discarded.

A second notable decision is keeping the search's optimization target narrowly defined as
"minimum cost among bad-state-free paths," rather than folding safety-distance or
reliability into the edge weight. This keeps the optimality guarantee simple, well-defined,
and easy to state precisely in a viva ("shortest safe path by cost") — at the cost of not
directly optimizing objective 4. Test Case 3 makes this trade-off, and its consequence,
explicit rather than hiding it.

## 27. Limitations

- The search optimizes total cost only; the geometric safety-distance objective (objective
  4) and reliability are computed and reported but not enforced as soft optimization
  targets. A combined weighted objective was deliberately not implemented (see Section 17)
  to avoid an unsupported optimality claim.
- Memory usage is not measured at runtime (Section 24); only algorithmic space complexity is
  reported.
- The heuristic's admissibility guarantee depends on `cMin` being computed from *all*
  transitions ever supplied to the planner (available or not). If a transition is removed
  and later a *cheaper-per-distance* transition is added, `cMin` correctly tightens; but
  `cMin` is never *loosened* on removal, so it can become more conservative (less
  informative, but still admissible) over a long sequence of removals — a soundness-safe,
  efficiency-only limitation.
- Test cases use small, hand-built graphs (4–7 states) chosen for illustrative clarity
  during a viva, not for statistically meaningful performance benchmarking at scale;
  microsecond-level timings on graphs this small are noisy and are presented as a
  demonstration of the measurement mechanism working, not as scalability results.
- Path reconstruction re-derives the best predecessor at each step from `inIdx_` rather than
  maintaining a dedicated parent map; this is simpler and always consistent with the current
  g-values, at the cost of doing a small amount of redundant work at reconstruction time
  (bounded by path length × average in-degree, not significant at this scale).

## 28. Conclusion

The implementation satisfies every mandatory requirement identified in the Part 1
requirements checklist: the required interfaces are implemented without modification to
their specified fields; bad-state avoidance is enforced as a true hard constraint with a
final validation pass; the Cartesian safety distance is computed exactly as specified; the
heuristic is kept provably admissible rather than assumed so; all five dynamic-environment
change types are supported with an honest distinction between local and partial-recompute
updates; and all six required test cases, plus the explicit Part 5 edge cases, pass and are
demonstrated with real, compiler-verified output rather than illustrative or invented
figures.
