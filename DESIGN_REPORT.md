# Design Report — Safe Semantic Planner

**PCCST503, Assignment 1 — Design of a Safe Semantic Planner in a Finite
Cartesian State Space**

## 1. Problem restatement

The planner operates over a `PlanningProblem`:

- a finite set of **states**, each with a unique id and a Cartesian
  **embedding** (a point in ℝⁿ, used only for geometric distance — not for
  state identity or transition cost);
- a finite set of directed **transitions**, each with `cost`, `safety` (a
  score in `[0, 1]`), `reliability`, and an `available` flag;
- a fixed **initial state**;
- a **goal state** that may change between planning calls;
- a set of **bad states** that a valid path must never enter.

The planner must return a `PlanningResult` — success flag, state path,
transition path, total cost, and a safety score — and must support cheap
**incremental replanning** as the goal, transition availability, transition
set, or bad-state set change, rather than recomputing from scratch every
time.

## 2. Algorithm choice: LPA\* over D\* Lite

Both LPA\* (Lifelong Planning A\*) and D\* Lite are incremental variants of
A\* built around the same consistent/overconsistent/underconsistent vertex
bookkeeping (`g` vs `rhs`, and a priority queue ordered by a two-part key).
They differ in search direction:

| | LPA\* | D\* Lite |
|---|---|---|
| Search direction | Forward, from the fixed start | Backward, from the goal |
| `g`-value meaning | Cost from **start** to `s` | Cost from `s` to **goal** |
| Cheap to update when... | the goal moves, transitions change | the **agent's position** moves (as in robot navigation) |
| Expensive to update when... | the agent's position moves | the goal moves |

This assignment's environment moves the **goal** and the **transition
set**, but the **start state never changes** (there is no "agent moving
through the world" — a single planning problem is solved and then
re-solved incrementally). Under LPA\*, `g(s)` is "cost from the fixed
start," so it stays valid across a goal change or a transition-availability
change; only the heuristic component of each vertex's key (which depends on
distance-to-goal) needs to be recomputed, and `computeShortestPath` only
needs to re-expand the vertices whose consistency actually changed. Under
D\* Lite the reverse would be true — `g`-values are distance-to-goal, so a
goal change would invalidate most of the existing search state and force
something close to a full re-search.

**Conclusion:** LPA\* is the correct fit for *this* assignment's dynamics
(fixed start, moving goal/graph), and was implemented in both the C++ and
JavaScript versions. Test Case 5 in the experimental results specifically
exercises and confirms the cheap-goal-update property (2 states explored on
re-plan vs. 3 on the initial plan, with the `S→A` sub-path's `g`-value
reused unchanged).

## 3. Architecture

Both implementations follow the same four-layer structure:

1. **Data model** — `State { id, embedding }`, `Transition { id, from, to,
   cost, safety, reliability, available }`, `PlanningProblem`,
   `PlanningResult`.
2. **Planner interface** — an abstract `Planner` with a single `plan(problem)`
   entry point, implemented by `LPAStarPlanner`. This keeps the required
   interface decoupled from the LPA\*-specific incremental API
   (`setTransitionAvailability`, `addTransition`, `updateGoal`,
   `updateBadStates`), which callers use only when they want to reuse
   search state between calls.
3. **LPA\* core** — the `g`/`rhs` maps, the sorted open list (a
   `std::vector` kept in key order in C++; an equivalent sorted array with
   binary-search insertion in JavaScript, for a matching complexity
   profile), `calculateKey`, `updateVertex`, and `computeShortestPath`.
4. **Safety layer** — `computeEdgeCost` (the effective weight formula, §4),
   `getDistanceToNearestBadState`, and `computeHeuristicScale`
   (admissibility scaling, §5), all of which sit between the raw graph and
   the LPA\* core so that safety never has to be special-cased inside the
   search loop itself — it is entirely encoded in the edge weights it
   consumes.

The JavaScript port (`web/planner.js`) mirrors this structure class-for-class
and method-for-method against `cpp/main.cpp`'s `LPAStarPlanner`, which is
why the two implementations produce identical paths, costs, and safety
scores on every test case (see `docs/EXPERIMENTAL_RESULTS.md`). `web/app.js`
is purely a presentation layer on top of it — a canvas renderer, six preset
scenarios, and UI event wiring (draggable nodes trigger a live `solve()`,
sliders live-update `kGeom`/`kTrans`) — it contains no planning logic.

## 4. Safety objective and effective edge weight

```
w(e) = cost(e) + kGeom * (1 / d_safe(to)) + kTrans * (1 − safety(e))
```

- **`d_safe(to)`** is the Euclidean distance, in embedding space, from the
  destination state to the nearest bad state (∞ if there are no bad
  states, so the clearance term vanishes exactly rather than only
  numerically).
- The **`kTrans`** term penalizes transitions whose own `safety` attribute
  is below `1.0`, independent of geometry — a transition can be
  intrinsically risky (e.g. an unreliable actuator) even far from any
  hazard.
- **`kGeom`** and **`kTrans`** are exposed as live parameters (sliders in
  the web UI; `setSafetyWeights` in both implementations) so a user can
  trade safety against cost interactively. Test Case 3 and
  `docs/images/case3_safety_disabled.png` demonstrate this directly: with
  both weights at `2.0` the planner prefers a longer, safer route (cost
  9.0, clearance 1.50); with both weights at `0.0` it collapses to the
  cheapest route regardless of hazard proximity (cost 3.0, clearance
  0.539).

**Hard bad-state constraint.** Independently of the soft weighting above,
any state in the bad-state set is forced to `g = rhs = ∞` and is never
inserted into the open list (`updateVertex` short-circuits for bad states);
no transition into or out of a bad state can contribute to another
vertex's `rhs`. `reconstructResult` performs a final linear pass over the
returned path and refuses to report success if a bad state is present,
so the constraint is enforced both structurally (during search) and
defensively (after reconstruction). This is validated by the four
additional edge cases in Part 5 of the C++ program (start-is-bad,
goal-is-bad, only-route-passes-through-bad, no-bad-states-defined — all
four resolve correctly, see `docs/EXPERIMENTAL_RESULTS.md`).

## 5. Heuristic admissibility and consistency

The heuristic is Euclidean distance to the goal, scaled by a single
constant:

```
cMin = min over all valid transitions e of ( w(e) / geometric_distance(e) )
h(s) = cMin * euclidean_distance(s, goal)
```

Because `cMin` is the minimum ratio of effective weight to geometric
distance over the whole graph, for any transition `(u, v)`:

```
h(u) ≤ cMin * dist(u, v) + h(v) ≤ w(u, v) + h(v)
```

by the triangle inequality on the Cartesian embeddings. This is exactly the
consistency condition A\*/LPA\* requires, and consistency implies
admissibility. Consequently:

- `w(e) ≥ 0` always (cost, `kGeom`, `kTrans` are non-negative; the
  penalties only add), so Bellman's principle of optimality holds and the
  additive cost model has well-defined subpath optimality.
- The scaled heuristic never overestimates true cost-to-go, so LPA\* is
  guaranteed to terminate with a globally cost-optimal path with respect to
  `w(e)`, not merely a locally greedy one.
- When a transition's weight or the bad-state set changes,
  `computeHeuristicScale` / `refreshHeuristicScaleForTransition` recompute
  `cMin`, preserving the consistency guarantee under incremental updates
  rather than only at the first `plan()` call.

## 6. Incremental update handling

Four kinds of environment change are supported without a full graph
rebuild, each reusing as much of the previous search state as its nature
allows:

| Change | Method | What's reused |
|---|---|---|
| Transition availability flips | `setTransitionAvailability` | Everything except the `rhs` of the transition's destination and any vertex reachable from it whose consistency that flip breaks. |
| New transition added | `addTransition` | All existing `g`/`rhs`; only the new edge's destination is re-evaluated, then the queue is re-drained. |
| Goal changes | `updateGoal` | All `g`-values (they are start-relative, not goal-relative — this is the LPA\*-over-D\*-Lite payoff from §2); only queue keys (which embed the heuristic to the *new* goal) are recomputed. |
| Bad-state set changes | `updateBadStates` | The graph topology and existing costs; every vertex is re-checked against the new set and re-enqueued only if its consistency actually changed. |

In all four cases the shared `computeShortestPath` loop is what actually
converges the search — the incremental methods only seed it with the
minimal set of "dirty" vertices, rather than reinitializing `g`/`rhs` for
every state as a batch `plan()` call does. Test Cases 4–6 in
`docs/EXPERIMENTAL_RESULTS.md` exercise exactly these three paths (minus
bad-state updates, which are covered by the Part 5 edge cases instead) and
show materially fewer explored states on the incremental re-plan than on
the initial batch plan.

## 7. Complexity

With `V` states and `E` transitions, and the open list implemented as a
priority structure with `O(log V)` insert/decrease-key/extract-min
(a binary heap in a from-scratch implementation; both provided
implementations use a sorted list with binary-search insertion, which is
`O(log V)` to locate but `O(V)` to shift — acceptable at the assignment's
scale, and noted here as the concrete tradeoff versus a heap):

- A full `plan()` call is `O((V + E) log V)`, matching standard A\*/Dijkstra
  bounds, since each vertex is settled at most a small constant number of
  times.
- An incremental update touches only the vertices whose `g`/`rhs`
  consistency actually changes as a result of the edit, which in practice
  is a small local neighborhood of the change rather than the whole graph —
  this is the entire point of using LPA\* instead of re-running `plan()`
  from scratch on every environment change.
- Memory is `O(V + E)`: one `g` and one `rhs` entry per state, one open-list
  entry per currently-inconsistent state, and `O(1)` per transition for the
  adjacency indices. Both implementations report an explicit byte-level
  estimate of this footprint at runtime (`getEstimatedMemoryBytes` /
  `getEstimatedMemoryBytes`), in addition to OS-level process memory
  (`getrusage` in C++).

## 8. Known limitations

- The open list is a sorted array, not a binary/Fibonacci heap — fine for
  the assignment's small test graphs, but insertion becomes `O(V)` due to
  array shifting at larger scale.
- The heuristic scale `cMin` is a single global constant. It remains
  admissible under any edge-weight change (recomputing the minimum can only
  keep it a valid lower bound), but it is not as tight as a per-edge or
  per-region heuristic could be, which trades a small amount of search
  efficiency for a much simpler admissibility argument.
- `getEstimatedMemoryBytes` is a structural estimate of the planner's own
  containers, not a substitute for OS-reported RSS; the C++ program reports
  both side by side for exactly this reason (see
  `docs/EXPERIMENTAL_RESULTS.md`).
