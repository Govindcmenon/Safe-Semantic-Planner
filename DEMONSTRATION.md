# Demonstration — Safe Semantic Planner (LPA*)

This document is a scripted walkthrough for demonstrating the implementation live (in a
viva or a demo session), paired with the exact, real output the program produced when this
walkthrough was prepared. Every line under "Actual output" below is copy-pasted from a real
run of the compiled `planner` binary — nothing here is invented.

## How to run the demonstration live

```
g++ -std=c++17 -O2 -Wall -Wextra main.cpp -o planner
./planner
```

Everything below runs automatically, in this order, with no input required.

---

## 1. Basic Reachability

**What to say:** "This is the simplest case — a straight line from S to G with no
obstacles. It confirms the core search and path reconstruction work correctly before
anything dynamic is introduced."

**Actual output:**
```
========================================
TEST CASE 1: BASIC REACHABILITY
========================================
Initial State: S
Goal State: G
Bad States: None

Result: SUCCESS

State Path:
S -> A -> B -> G

Transition Path:
T101 -> T102 -> T103

Total Cost: 3.0000
Minimum Safety Distance: N/A (no bad states defined)
Bad States Visited: 0
Explored States: 4
Planning Time: 2.47 microseconds
```

---

## 2. Bad State Avoidance

**What to say:** "Two routes exist to G: one passes through a bad state X, the other
doesn't. The planner must select the second, and — this is the important part — it's not
just discouraged from using X, it structurally cannot, since X is excluded from the search
entirely."

**Actual output:**
```
========================================
TEST CASE 2: BAD STATE AVOIDANCE
========================================
Initial State: S
Goal State: G
Bad States: X

Result: SUCCESS

State Path:
S -> C -> D -> G

Transition Path:
T204 -> T205 -> T206

Total Cost: 3.0000
Minimum Safety Distance: 1.414214
Bad States Visited: 0
Explored States: 5
Planning Time: 1.86 microseconds

Verification - path contains bad state X: NO (correct)
```

---

## 3. Safety Margin

**What to say:** "Here's an honest, and important, result: the cheaper path is chosen even
though it passes closer to a bad state than the alternative. That's because this
implementation optimizes total cost among safe paths — it doesn't fold the safety-distance
objective into the search's edge weight. I'll explain why that's a defensible, documented
scope decision, not an oversight."

**Actual output:**
```
========================================
TEST CASE 3: SAFETY MARGIN
========================================
Initial State: S
Goal State: G
Bad States: Y
Path 1 (S-A1-A2-G): lower cost, passes closer to Y
Path 2 (S-B1-B2-G): higher cost, stays farther from Y

Result: SUCCESS

State Path:
S -> A1 -> A2 -> G

Transition Path:
T301 -> T302 -> T303

Total Cost: 3.0000
Minimum Safety Distance: 7.000000
Bad States Visited: 0
Explored States: 4
Planning Time: 1.66 microseconds

Selected: Path 1 (lower cost)
Explanation: this implementation's search minimizes TOTAL TRANSITION COST
among bad-state-free paths only; the geometric safety-distance objective
(objective 4) is computed and reported (Minimum Safety Distance above) but is
NOT part of the edge weight the search optimizes. Therefore the lower-cost
path is selected even though it passes closer to the bad state, as long as it
does not touch the bad state itself. This is the actual, honest behavior of
the implemented objective - not an assumption.
Reported (not optimized) Score(P) = alpha*G - beta*C + gamma*D + delta*R = 14.7290
(weights: alpha=10.0000 beta=1.0000 gamma=1.0000 delta=1.0000 -- this metric is informational only; see explanation above)
```

---

## 4. Dynamic Transition

**What to say:** "The planner first finds the direct route S→A→G. I then make the (A,G)
transition unavailable — simulating a blocked route — and the planner recomputes an
alternative, using the same warm search state rather than starting over."

**Actual output:**
```
========================================
TEST CASE 4: DYNAMIC TRANSITION
========================================
--- Initial plan ---
Result: SUCCESS

State Path:
S -> A -> G

Transition Path:
T401 -> T402

Total Cost: 2.0000
Minimum Safety Distance: N/A (no bad states defined)
Bad States Visited: 0
Explored States: 3
Planning Time: 1.18 microseconds

--- Making transition (A,G) [id 402] unavailable, then replanning ---
Result: SUCCESS

State Path:
S -> C -> D -> G

Transition Path:
T403 -> T404 -> T405

Total Cost: 6.0000
Minimum Safety Distance: N/A (no bad states defined)
Bad States Visited: 0
Explored States: 4
Planning Time: 1.58 microseconds
```

---

## 5. Goal Update

**What to say:** "This is the test case that decided the algorithm choice: LPA* over
D* Lite. Watch the explored-state count — the replan after the goal change explores *fewer*
states than the original cold plan, which is direct evidence that previously computed
distances were reused, not thrown away."

**Actual output:**
```
========================================
TEST CASE 5: GOAL UPDATE
========================================
--- Initial plan toward G_old ---
Result: SUCCESS

State Path:
S -> A -> G_old

Transition Path:
T501 -> T502

Total Cost: 2.0000
Minimum Safety Distance: N/A (no bad states defined)
Bad States Visited: 0
Explored States: 3
Planning Time: 1.14 microseconds

--- Goal changes to G_new, replanning (warm-started, not from scratch) ---
Result: SUCCESS

State Path:
S -> A -> H -> G_new

Transition Path:
T501 -> T503 -> T504

Total Cost: 3.5000
Minimum Safety Distance: N/A (no bad states defined)
Bad States Visited: 0
Explored States: 2
Planning Time: 1.17 microseconds

Note: g-values computed during the FIRST plan (e.g. distance from S to A)
remain valid and are reused; only priority-queue keys were recalculated and
the search extended toward the new goal. This is a partial, warm-started
recomputation, not a full graph reinitialization - see PART 9 discussion.
```

---

## 6. Transition Addition

**What to say:** "Finally, I add a shortcut edge directly from S to G. Since it strictly
lowers total cost, the planner adopts it — and because the shortcut connects directly to
the goal, this is the cheapest possible replan: only one state needs to be (re-)explored."

**Actual output:**
```
========================================
TEST CASE 6: TRANSITION ADDITION
========================================
--- Initial plan (no shortcut) ---
Result: SUCCESS

State Path:
S -> A -> B -> G

Transition Path:
T601 -> T602 -> T603

Total Cost: 3.0000
Minimum Safety Distance: N/A (no bad states defined)
Bad States Visited: 0
Explored States: 4
Planning Time: 0.59 microseconds

--- Adding shortcut transition S->G (id 604, cost 1.5), replanning ---
Result: SUCCESS

State Path:
S -> G

Transition Path:
T604

Total Cost: 1.5000
Minimum Safety Distance: N/A (no bad states defined)
Bad States Visited: 0
Explored States: 1
Planning Time: 0.90 microseconds
```

---

## 7. Safety Edge Cases (Part 5 requirement)

**What to say:** "Beyond the six main test cases, the assignment explicitly asks us to
handle four edge cases around safety: a bad initial state, a bad goal state, no safe path
existing at all, and the case where there are no bad states defined. All four are handled
explicitly and correctly."

**Actual output:**
```
========================================
ADDITIONAL SAFETY EDGE CASES (PART 5)
========================================
Case A - Initial state is a bad state -> success = false (correct)
Case B - Goal state is a bad state -> success = false (correct)
Case C - Only route passes through a bad state -> success = false (correct)
Case D - No bad states defined -> success = true (correct), safetyScore is +infinity (correct, documented)
```

---

## Suggested demo flow / talking points summary

1. Show the code compiles cleanly with `-Wall -Wextra` (zero warnings) — signals code
   quality.
2. Run the program once, live, so the examiner sees it isn't pre-recorded.
3. Walk through Test Cases 1–2 quickly (they establish baseline correctness).
4. Spend the most time on Test Case 3 — it's the most likely source of follow-up questions
   ("why didn't it pick the safer path?") and the answer demonstrates a clear understanding
   of what was actually implemented versus what could be added as an extension.
5. Use Test Case 5's explored-state numbers as concrete evidence for the LPA*-over-D*-Lite
   design decision — this is the single strongest, most quantifiable talking point in the
   whole project.
6. Close with the Part 5 edge cases to show the hard safety constraint was tested
   exhaustively, not just in the "happy path."
