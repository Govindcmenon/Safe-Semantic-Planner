# Demonstration

This is a screenshot-driven walkthrough of the live web visualizer
(`web/`), captured directly from a running instance — every image below is
the real application, not a mockup. It's organized to demonstrate, in
order: correctness on a trivial case, hard bad-state avoidance, the safety
objective actually changing behavior, and the three incremental-replanning
mechanisms (transition blocking, goal change, transition addition).

## 1. Basic reachability

Loading **Test Case 1** confirms the planner behaves as a correct
shortest-path search when there's nothing hazardous to avoid: a straight
line, S → A → B → G, at the obvious cost of 3.0.

![Basic reachability](images/case1_basic_reachability.png)

## 2. Bad-state avoidance

**Test Case 2** places hazard `X` directly on the shortest route. The
diagnostics panel shows `X` marked **Bad State** in the LPA\* node table
(bottom-right), and the solved path detours through `C → D` instead,
at equal cost:

![Bad state avoidance](images/case2_bad_state_avoidance.png)

## 3. The safety objective, switched on and off

This is the core demonstration that the safety weighting is real, not
cosmetic. **Test Case 3** offers two routes around hazard `Y`: a cheap one
close to the hazard, and an expensive one that stays well clear.

With the default weights (**kGeom = kTrans = 2.0**, sliders on the left),
the planner picks the expensive-but-safe route — cost 9.00, clearance
1.500:

![Safety objective on](images/case3_safety_margin.png)

Dragging both sliders down to **0.0** removes the safety terms from the
effective edge weight entirely. The planner immediately re-solves (no page
reload, no button press needed — the slider's `input` event triggers
`solve()` directly) and switches to the cheap route — cost 3.00, clearance
drops to 0.539:

![Safety objective off](images/case3_safety_off.png)

Side by side, everything else about the graph is unchanged — only the
`kGeom`/`kTrans` sliders moved — which is the cleanest possible evidence
that the weighting term in `computeEdgeCost` is doing exactly what the
design report claims.

## 4. Incremental replanning — transition blocking

**Test Case 4** starts with transition `A→G` unavailable (shown as a dashed
red edge), forcing the initial solve onto the longer `S→C→D→G` detour:

![Transition blocked](images/case4_blocked.png)

Clicking **Toggle Transition Availability** flips that edge back on. The
planner doesn't re-solve from scratch — it calls
`setTransitionAvailability`, which only re-examines the vertices whose
consistency the flip could have affected — and the path snaps back to the
cheaper `S→A→G` route:

![Transition restored](images/case4_restored.png)

## 5. Incremental replanning — goal update

**Test Case 5** demonstrates the specific payoff of choosing LPA\* over
D\* Lite discussed in the design report: because `g`-values are
start-relative (not goal-relative), moving the goal from `G_old` to
`G_new` lets the planner reuse the already-computed `S→A` sub-path
untouched. The re-plan explores *fewer* states (2) than the original
from-scratch solve (3):

![Goal update](images/case5_goal_update.png)

## 6. Incremental replanning — transition addition

**Test Case 6** adds a brand-new shortcut edge `S→G` (cost 1.5) at
runtime via `addTransition`. The planner re-examines only the new edge's
destination and converges in a single explored state, immediately
reporting the new 1-hop optimum instead of the previous 3-hop route:

![Transition addition](images/case6_transition_addition.png)

## Summary

| Demonstration | What it proves |
|---|---|
| §1 Basic reachability | Correctness on the trivial case |
| §2 Bad-state avoidance | The hard bad-state constraint is enforced |
| §3 Safety objective on/off | `kGeom`/`kTrans` genuinely change planning behavior |
| §4 Transition blocking | Incremental replanning on availability changes |
| §5 Goal update | Incremental replanning reuses start-relative `g`-values |
| §6 Transition addition | Incremental replanning on graph growth |

All measured values shown above (cost, clearance, explored-state counts)
match the independently-run C++ console output exactly — see
[`docs/EXPERIMENTAL_RESULTS.md`](EXPERIMENTAL_RESULTS.md) for the raw
numbers and a direct cross-implementation comparison.
