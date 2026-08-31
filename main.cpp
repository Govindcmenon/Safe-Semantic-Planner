// =====================================================================
// PCCST503 - Machine Learning, Assignment 1
// "Design of a Safe Semantic Planner in a Finite Cartesian State Space"
//
// Selected algorithm: LPA* (Lifelong Planning A*)
//   (See accompanying report for the justification of LPA* over D* Lite
//    for THIS assignment: the initial state is fixed and never moves,
//    while the goal state and transition set change over time. LPA*'s
//    forward search from a fixed start keeps g-values goal-independent,
//    which makes goal updates cheaper to handle than in D* Lite, whose
//    backward search makes g-values goal-dependent.)
//
// This file is a single, self-contained C++17 program implementing:
//   - The required interfaces (State, Transition, PlanningProblem,
//     PlanningResult, Planner)
//   - The LPA* planning algorithm with a hard bad-state constraint
//   - Cartesian safety-distance computation
//   - A conservative, admissibility-aware heuristic
//   - Incremental update handling for dynamic environment changes
//   - All six illustrative test cases from the assignment
// =====================================================================

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <map>
#include <cstdint>
#include <cmath>
#include <limits>
#include <chrono>
#include <algorithm>
#include <string>
#include <iomanip>
#include <sstream>

// ---------------------------------------------------------------------
// PART 4.1 - Required interfaces, exactly as specified in the PDF
// ---------------------------------------------------------------------

struct State {
    uint64_t id;
    std::vector<double> embedding;
};

struct Transition {
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;       // per-transition safety score (reported, see report Part 3.5)
    double reliability;
    bool available;
};

struct PlanningProblem {
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};

struct PlanningResult {
    bool success = false;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost = 0.0;
    // Minimum Euclidean distance from any visited state to the nearest bad
    // state (objective 4 in the PDF). If there are no bad states at all,
    // this is defined as +infinity (no safety constraint is active, so the
    // path is maximally safe by definition - see PART 6 in the report).
    double safetyScore = 0.0;
};

class Planner {
public:
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
    virtual ~Planner() = default;
};

// ---------------------------------------------------------------------
// Small numeric helpers
// ---------------------------------------------------------------------

static const double INF = std::numeric_limits<double>::infinity();
static const double EPS = 1e-9;

static double euclideanDistance(const std::vector<double>& a, const std::vector<double>& b) {
    // Handle mismatched dimensionality defensively: compare over the
    // common prefix of dimensions rather than crashing. In a correctly
    // formed problem instance all embeddings share the same dimension d,
    // so this branch is a safety net, not the expected path.
    size_t n = std::min(a.size(), b.size());
    double sumSq = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double diff = a[i] - b[i];
        sumSq += diff * diff;
    }
    return std::sqrt(sumSq);
}

// ---------------------------------------------------------------------
// PART 4/5/6/7/8/9 - LPA* Planner implementation
// ---------------------------------------------------------------------
//
// Design notes (kept honest, matching the PDF's "never visit a bad
// state" hard-constraint requirement):
//
//  * Bad states are structurally excluded from the search: any vertex
//    in the bad-state set is forced to rhs = g = infinity and is never
//    inserted into the priority queue. No edge that starts or ends at a
//    bad state ever contributes to another vertex's rhs computation.
//    This means bad states cannot appear in ANY path that the search
//    considers -- not "discouraged via a penalty", but structurally
//    unreachable. A final validation pass double-checks this before a
//    successful result is returned (PART 5 requirement).
//
//  * The algorithm's optimality guarantee is: "the returned path is the
//    minimum TOTAL-COST path among all bad-state-free paths from start
//    to goal." Reliability and the geometric safety-distance objective
//    (objective 4) are NOT folded into the edge weight used by the
//    search; they are computed and reported for evaluation purposes
//    only (see PART 7 below and the design report, Part 3.5/17). This
//    is a deliberate, documented choice so that no false optimality
//    claim is made about a combined objective the algorithm does not
//    actually optimize.
//
//  * Heuristic: raw Euclidean distance is NOT assumed admissible,
//    because the PDF allows arbitrary transition costs with no stated
//    relationship to geometric distance. Instead we scale Euclidean
//    distance by cMin = min over all known transitions of
//    (cost / geometric_distance). This guarantees h(s) <= true cost to
//    goal for any transition ever supplied to the planner. If no valid
//    ratio can be established (e.g. no transitions yet), h = 0 is used
//    (Dijkstra-equivalent, always admissible).
//
//  * Dynamic updates are handled with explicit honesty about which are
//    truly local/incremental and which require a broader (but still
//    warm-started) recomputation. See comments on each update method
//    and PART 9 in the assignment prompt.

class LPAStarPlanner : public Planner {
public:
    // ---- Public incremental-update surface -----------------------------
    // plan() performs a full (cold) initialization + search - this is the
    // Planner interface method required by the PDF.
    PlanningResult plan(const PlanningProblem& problem) override {
        loadProblem(problem);
        initializeSearch();
        auto start = std::chrono::steady_clock::now();
        exploredThisCall_ = 0;
        computeShortestPath();
        auto end = std::chrono::steady_clock::now();
        lastOperationMicros_ = std::chrono::duration<double, std::micro>(end - start).count();
        return reconstructResult();
    }

    // TRUE INCREMENTAL: flipping availability only affects the two
    // endpoints of the changed transition. UpdateVertex is called on the
    // 'to' endpoint (its rhs may change) and, defensively, we also touch
    // its own successors is NOT needed because 'from' side rhs values are
    // unaffected by outgoing-edge availability. Bounded by local degree.
    PlanningResult setTransitionAvailability(uint64_t transitionId, bool available) {
        timedOperation([&]() {
            auto it = transitionIndexById_.find(transitionId);
            if (it == transitionIndexById_.end()) return;
            Transition& t = transitions_[it->second];
            if (t.available == available) return;
            t.available = available;
            updateVertex(t.to);
            computeShortestPath();
        });
        return reconstructResult();
    }

    // TRUE INCREMENTAL: a new edge only ever affects the rhs of its 'to'
    // endpoint (a new possible predecessor appeared for that vertex).
    // We also refresh the heuristic scale factor cMin, since a new edge
    // could otherwise make the existing heuristic inadmissible.
    PlanningResult addTransition(const Transition& t) {
        timedOperation([&]() {
            transitions_.push_back(t);
            size_t idx = transitions_.size() - 1;
            transitionIndexById_[t.id] = idx;
            outIdx_[t.from].push_back(idx);
            inIdx_[t.to].push_back(idx);
            refreshHeuristicScaleForTransition(t);
            updateVertex(t.to);
            computeShortestPath();
        });
        return reconstructResult();
    }

    // TRUE INCREMENTAL: removing an edge can only raise the rhs of its
    // 'to' endpoint (one fewer candidate predecessor). Local update.
    PlanningResult removeTransition(uint64_t transitionId) {
        timedOperation([&]() {
            auto it = transitionIndexById_.find(transitionId);
            if (it == transitionIndexById_.end()) return;
            size_t idx = it->second;
            uint64_t to = transitions_[idx].to;
            transitions_[idx].available = false; // logically remove: excluded from all searches
            removeFromAdjacency(idx);
            transitionIndexById_.erase(it);
            updateVertex(to);
            computeShortestPath();
        });
        return reconstructResult();
    }

    // PARTIAL RE-COMPUTATION, NOT A LOCAL UPDATE, NOT A FULL REBUILD.
    // g/rhs values computed so far remain valid because they represent
    // distance-from-a-fixed-start, which does not depend on the goal.
    // However every vertex currently in the open list has a key that
    // depends on h(v, goal), so all of those keys must be recalculated
    // for the new goal, and the search must then continue (possibly
    // expanding previously-unvisited vertices) until the NEW goal is
    // locally consistent. This is honestly reported as more expensive
    // than an edge-availability update, but still reuses all previously
    // computed g-values rather than rebuilding the graph from scratch.
    PlanningResult updateGoal(uint64_t newGoal) {
        timedOperation([&]() {
            goal_ = newGoal;
            if (!g_.count(goal_)) g_[goal_] = INF;
            if (!rhs_.count(goal_)) rhs_[goal_] = INF;
            // Recompute keys for every vertex currently queued, since the
            // heuristic term depends on the (now different) goal.
            std::vector<uint64_t> queued;
            queued.reserve(openList_.size());
            for (auto& entry : openList_) queued.push_back(std::get<2>(entry));
            openList_.clear();
            openKeyOf_.clear();
            for (uint64_t v : queued) insertIntoOpenList(v, calculateKey(v));
            // The goal itself may not have been discovered yet - make sure
            // it is (re)evaluated so the search can extend toward it.
            updateVertex(goal_);
            computeShortestPath();
        });
        return reconstructResult();
    }

    // MIXED: for each state that newly becomes bad, this is a TRUE LOCAL
    // update bounded by that vertex's degree (it and its successors are
    // touched). For each state that stops being bad, likewise local. The
    // overall call is therefore a sequence of local repairs, not a full
    // graph rebuild, but its total cost scales with the number of states
    // whose bad/safe status actually changed (which is honest and
    // expected: PART 9 asks for support, not zero-cost updates).
    PlanningResult updateBadStates(const std::vector<uint64_t>& newBadStates) {
        timedOperation([&]() {
            std::unordered_set<uint64_t> newSet(newBadStates.begin(), newBadStates.end());
            std::unordered_set<uint64_t> changed;
            for (uint64_t s : badStates_) if (!newSet.count(s)) changed.insert(s); // became safe
            for (uint64_t s : newSet) if (!badStates_.count(s)) changed.insert(s); // became bad
            badStates_ = newSet;
            for (uint64_t s : changed) {
                updateVertex(s);
                for (size_t idx : outIdx_[s]) updateVertex(transitions_[idx].to);
                for (size_t idx : inIdx_[s]) updateVertex(transitions_[idx].from);
            }
            computeShortestPath();
        });
        return reconstructResult();
    }

    size_t lastExploredStates() const { return exploredThisCall_; }
    double lastOperationMicros() const { return lastOperationMicros_; }
    const std::unordered_set<uint64_t>& badStates() const { return badStates_; }

private:
    // ---- Graph storage (Section 2.4 of the architecture) ---------------
    std::unordered_map<uint64_t, State> statesById_;
    std::vector<Transition> transitions_;
    std::unordered_map<uint64_t, size_t> transitionIndexById_;
    std::unordered_map<uint64_t, std::vector<size_t>> outIdx_; // from -> transition indices
    std::unordered_map<uint64_t, std::vector<size_t>> inIdx_;  // to   -> transition indices
    std::unordered_set<uint64_t> badStates_;
    uint64_t start_ = 0, goal_ = 0;

    // ---- LPA* search state ----------------------------------------------
    std::unordered_map<uint64_t, double> g_, rhs_;
    // Open list emulated with an ordered set of (k1, k2, id) plus a lookup
    // map so we can test membership / current key in O(log n).
    std::set<std::tuple<double, double, uint64_t>> openList_;
    std::unordered_map<uint64_t, std::pair<double, double>> openKeyOf_;

    double heuristicScale_ = 0.0; // cMin: keeps h(s) admissible (see class comment)
    size_t exploredThisCall_ = 0;
    double lastOperationMicros_ = 0.0;

    // ---- Setup -----------------------------------------------------------
    void loadProblem(const PlanningProblem& problem) {
        statesById_.clear();
        for (const auto& s : problem.states) statesById_[s.id] = s;

        transitions_.clear();
        transitionIndexById_.clear();
        outIdx_.clear();
        inIdx_.clear();
        for (const auto& t : problem.transitions) {
            transitions_.push_back(t);
            size_t idx = transitions_.size() - 1;
            transitionIndexById_[t.id] = idx;
            outIdx_[t.from].push_back(idx);
            inIdx_[t.to].push_back(idx);
        }

        badStates_.clear();
        for (uint64_t b : problem.badStates) badStates_.insert(b);

        start_ = problem.initialState;
        goal_ = problem.goalState;

        computeHeuristicScale();
    }

    void computeHeuristicScale() {
        // cMin = min over all transitions of (cost / geometric distance),
        // considering ONLY pairs with a positive geometric distance (a
        // zero-distance edge places no constraint on the scale factor).
        // This is recomputed from scratch whenever the full problem is
        // (re)loaded via plan(); addTransition() refreshes it
        // incrementally afterwards (see refreshHeuristicScaleForTransition).
        double m = INF;
        for (const auto& t : transitions_) {
            if (!statesById_.count(t.from) || !statesById_.count(t.to)) continue;
            double d = euclideanDistance(statesById_[t.from].embedding, statesById_[t.to].embedding);
            if (d > EPS && t.cost >= 0.0) m = std::min(m, t.cost / d);
        }
        heuristicScale_ = (m == INF) ? 0.0 : m; // 0.0 => fall back to h = 0 (always admissible)
    }

    void refreshHeuristicScaleForTransition(const Transition& t) {
        if (!statesById_.count(t.from) || !statesById_.count(t.to)) return;
        double d = euclideanDistance(statesById_[t.from].embedding, statesById_[t.to].embedding);
        if (d > EPS && t.cost >= 0.0) {
            double ratio = t.cost / d;
            heuristicScale_ = (heuristicScale_ <= 0.0) ? ratio : std::min(heuristicScale_, ratio);
        }
    }

    // ---- LPA* core --------------------------------------------------------
    double getG(uint64_t s) const {
        auto it = g_.find(s);
        return it == g_.end() ? INF : it->second;
    }
    double getRhs(uint64_t s) const {
        auto it = rhs_.find(s);
        return it == rhs_.end() ? INF : it->second;
    }

    double heuristic(uint64_t s) const {
        if (heuristicScale_ <= 0.0) return 0.0; // conservative fallback: always admissible
        if (!statesById_.count(s) || !statesById_.count(goal_)) return 0.0;
        double d = euclideanDistance(statesById_.at(s).embedding, statesById_.at(goal_).embedding);
        return heuristicScale_ * d;
    }

    std::pair<double, double> calculateKey(uint64_t s) const {
        double gv = getG(s), rv = getRhs(s);
        double m = std::min(gv, rv);
        double k1 = (m >= INF) ? INF : m + heuristic(s);
        return {k1, m};
    }

    void insertIntoOpenList(uint64_t s, std::pair<double, double> key) {
        openList_.insert({key.first, key.second, s});
        openKeyOf_[s] = key;
    }

    void removeFromOpenList(uint64_t s) {
        auto it = openKeyOf_.find(s);
        if (it == openKeyOf_.end()) return;
        openList_.erase({it->second.first, it->second.second, s});
        openKeyOf_.erase(it);
    }

    void initializeSearch() {
        g_.clear();
        rhs_.clear();
        openList_.clear();
        openKeyOf_.clear();
        exploredThisCall_ = 0;
        for (const auto& kv : statesById_) g_[kv.first] = INF;
        rhs_[start_] = 0.0;
        g_[start_] = INF; // start becomes locally consistent once processed
        if (badStates_.count(start_)) {
            // Bad initial state: it can never legally be "at" this state,
            // so rhs stays infinite - the search will correctly fail.
            rhs_[start_] = INF;
        } else {
            insertIntoOpenList(start_, calculateKey(start_));
        }
    }

    // The central local-repair routine used both by plan() (indirectly,
    // via computeShortestPath) and by every incremental update method.
    void updateVertex(uint64_t u) {
        if (badStates_.count(u)) {
            // HARD CONSTRAINT: a bad state can never be part of any valid
            // path, so it is forced permanently unreachable and is never
            // placed in the open list.
            g_[u] = INF;
            rhs_[u] = INF;
            removeFromOpenList(u);
            return;
        }
        if (u != start_) {
            double best = INF;
            for (size_t idx : inIdx_[u]) {
                const Transition& t = transitions_[idx];
                if (!t.available) continue;
                if (badStates_.count(t.from)) continue; // never route through a bad predecessor
                double gp = getG(t.from);
                if (gp < INF) best = std::min(best, gp + t.cost);
            }
            rhs_[u] = best;
        }
        removeFromOpenList(u);
        if (std::fabs(getG(u) - getRhs(u)) > EPS) {
            insertIntoOpenList(u, calculateKey(u));
        }
    }

    void computeShortestPath() {
        while (!openList_.empty()) {
            auto top = *openList_.begin();
            std::pair<double, double> topKey = {std::get<0>(top), std::get<1>(top)};
            std::pair<double, double> goalKey = calculateKey(goal_);
            bool goalConsistent = std::fabs(getRhs(goal_) - getG(goal_)) <= EPS;
            if (!(topKey < goalKey) && goalConsistent) break;

            uint64_t u = std::get<2>(top);
            openList_.erase(openList_.begin());
            openKeyOf_.erase(u);
            exploredThisCall_++;

            double gu = getG(u), ru = getRhs(u);
            if (gu > ru + EPS) {
                g_[u] = ru;
                for (size_t idx : outIdx_[u]) {
                    const Transition& t = transitions_[idx];
                    if (!t.available) continue;
                    updateVertex(t.to);
                }
            } else {
                g_[u] = INF;
                updateVertex(u);
                for (size_t idx : outIdx_[u]) {
                    const Transition& t = transitions_[idx];
                    if (!t.available) continue;
                    updateVertex(t.to);
                }
            }
        }
    }

    void removeFromAdjacency(size_t idx) {
        uint64_t from = transitions_[idx].from, to = transitions_[idx].to;
        auto& outs = outIdx_[from];
        outs.erase(std::remove(outs.begin(), outs.end(), idx), outs.end());
        auto& ins = inIdx_[to];
        ins.erase(std::remove(ins.begin(), ins.end(), idx), ins.end());
    }

    template <typename F>
    void timedOperation(F&& f) {
        exploredThisCall_ = 0;
        auto t0 = std::chrono::steady_clock::now();
        f();
        auto t1 = std::chrono::steady_clock::now();
        lastOperationMicros_ = std::chrono::duration<double, std::micro>(t1 - t0).count();
    }

    // ---- PART 5 / 6 - result construction with final safety validation ---
    PlanningResult reconstructResult() {
        PlanningResult result;

        if (badStates_.count(start_) || badStates_.count(goal_)) {
            result.success = false;
            return result; // explicit, documented handling of bad start/goal
        }
        if (getG(goal_) >= INF) {
            result.success = false;
            return result; // no safe path exists
        }

        std::vector<uint64_t> statePath;
        std::vector<uint64_t> transitionPath;
        uint64_t cur = goal_;
        statePath.push_back(cur);
        std::unordered_set<uint64_t> guard;
        guard.insert(cur);

        bool ok = true;
        while (cur != start_) {
            double bestVal = INF;
            uint64_t bestPred = 0;
            size_t bestIdx = SIZE_MAX;
            for (size_t idx : inIdx_[cur]) {
                const Transition& t = transitions_[idx];
                if (!t.available) continue;
                if (badStates_.count(t.from)) continue;
                double gp = getG(t.from);
                if (gp >= INF) continue;
                double val = gp + t.cost;
                if (val < bestVal - EPS) {
                    bestVal = val;
                    bestPred = t.from;
                    bestIdx = idx;
                }
            }
            if (bestIdx == SIZE_MAX) { ok = false; break; }
            transitionPath.push_back(transitions_[bestIdx].id);
            cur = bestPred;
            if (guard.count(cur)) { ok = false; break; } // cycle guard
            guard.insert(cur);
            statePath.push_back(cur);
            if (statePath.size() > statesById_.size() + 1) { ok = false; break; }
        }

        if (!ok) {
            result.success = false;
            return result;
        }

        std::reverse(statePath.begin(), statePath.end());
        std::reverse(transitionPath.begin(), transitionPath.end());

        // FINAL SAFETY VALIDATION (PART 5 requirement): re-check the
        // complete path against the current bad-state set before ever
        // reporting success, independent of how it was constructed.
        for (uint64_t s : statePath) {
            if (badStates_.count(s)) {
                result.success = false;
                return result;
            }
        }

        double totalCost = 0.0;
        for (uint64_t tid : transitionPath) {
            const Transition& t = transitions_[transitionIndexById_[tid]];
            totalCost += t.cost;
        }

        result.success = true;
        result.statePath = statePath;
        result.transitionPath = transitionPath;
        result.totalCost = totalCost;
        result.safetyScore = computeSafetyScore(statePath);
        return result;
    }

    // PART 6: minimum Euclidean distance, over all states in the path, to
    // the nearest bad state. If there are no bad states, this is +infinity
    // by definition (no proximity constraint is violated by anything).
    double computeSafetyScore(const std::vector<uint64_t>& statePath) const {
        if (badStates_.empty()) return INF;
        double minOverPath = INF;
        for (uint64_t s : statePath) {
            if (!statesById_.count(s)) continue;
            double nearest = INF;
            for (uint64_t b : badStates_) {
                if (!statesById_.count(b)) continue;
                double d = euclideanDistance(statesById_.at(s).embedding, statesById_.at(b).embedding);
                nearest = std::min(nearest, d);
            }
            minOverPath = std::min(minOverPath, nearest);
        }
        return minOverPath;
    }
};

// =====================================================================
// PART 7 - reported (not optimized) combined score, computed AFTER a
// planning result exists, purely for evaluation / Test Case 3 discussion.
// Score(P) = alphaG - betaC + gammaD + deltaR
// =====================================================================
struct ScoreWeights { double alpha = 10.0, beta = 1.0, gamma = 1.0, delta = 1.0; };

static double reportedScore(const PlanningResult& r, const std::vector<Transition>& allTransitions,
                             const std::unordered_map<uint64_t, size_t>& idxById, const ScoreWeights& w) {
    double G = r.success ? 1.0 : 0.0;
    double C = r.totalCost;
    double D = std::isinf(r.safetyScore) ? 0.0 : r.safetyScore; // treat "no bad states" as no bonus/penalty
    double R = 1.0;
    for (uint64_t tid : r.transitionPath) {
        auto it = idxById.find(tid);
        if (it != idxById.end()) R *= allTransitions[it->second].reliability;
    }
    if (r.transitionPath.empty()) R = 1.0;
    return w.alpha * G - w.beta * C + w.gamma * D + w.delta * R;
}

// =====================================================================
// Formatting helpers for PART 12 output style
// =====================================================================

static void printHeader(const std::string& title) {
    std::cout << "========================================\n";
    std::cout << title << "\n";
    std::cout << "========================================\n";
}

static std::string pathToString(const std::vector<uint64_t>& path, const std::unordered_map<uint64_t, std::string>& names) {
    std::ostringstream oss;
    for (size_t i = 0; i < path.size(); ++i) {
        auto it = names.find(path[i]);
        oss << (it != names.end() ? it->second : std::to_string(path[i]));
        if (i + 1 < path.size()) oss << " -> ";
    }
    return oss.str();
}

static std::string transitionsToString(const std::vector<uint64_t>& path) {
    std::ostringstream oss;
    for (size_t i = 0; i < path.size(); ++i) {
        oss << "T" << path[i];
        if (i + 1 < path.size()) oss << " -> ";
    }
    return oss.str();
}

static void printResult(const PlanningResult& r, const std::unordered_map<uint64_t, std::string>& names,
                         size_t explored, double micros, const std::unordered_set<uint64_t>& badStates) {
    std::cout << "Result: " << (r.success ? "SUCCESS" : "FAILURE") << "\n\n";
    if (r.success) {
        std::cout << "State Path:\n" << pathToString(r.statePath, names) << "\n\n";
        std::cout << "Transition Path:\n" << transitionsToString(r.transitionPath) << "\n\n";
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Total Cost: " << r.totalCost << "\n";
        std::cout << "Minimum Safety Distance: "
                   << (std::isinf(r.safetyScore) ? std::string("N/A (no bad states defined)") : std::to_string(r.safetyScore))
                   << "\n";
        int badVisited = 0;
        for (uint64_t s : r.statePath) if (badStates.count(s)) badVisited++;
        std::cout << "Bad States Visited: " << badVisited << "\n";
    }
    std::cout << "Explored States: " << explored << "\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Planning Time: " << micros << " microseconds\n";
    std::cout << "\n";
}

// =====================================================================
// Test case scenario builders
// =====================================================================

static State mkState(uint64_t id, std::vector<double> emb) { return State{id, std::move(emb)}; }
static Transition mkT(uint64_t id, uint64_t from, uint64_t to, double cost, double safety, double rel, bool avail) {
    return Transition{id, from, to, cost, safety, rel, avail};
}

// ---------------------------------------------------------------------
// TEST CASE 1: Basic Reachability   S -> A -> B -> G
// ---------------------------------------------------------------------
static void runTestCase1() {
    printHeader("TEST CASE 1: BASIC REACHABILITY");

    PlanningProblem p;
    p.states = {
        mkState(1, {0, 0}), mkState(2, {1, 0}), mkState(3, {2, 0}), mkState(4, {3, 0})
    };
    p.transitions = {
        mkT(101, 1, 2, 1.0, 1.0, 0.99, true),
        mkT(102, 2, 3, 1.0, 1.0, 0.99, true),
        mkT(103, 3, 4, 1.0, 1.0, 0.99, true),
    };
    p.initialState = 1; p.goalState = 4; p.badStates = {};

    std::unordered_map<uint64_t, std::string> names = {{1, "S"}, {2, "A"}, {3, "B"}, {4, "G"}};

    std::cout << "Initial State: S\nGoal State: G\nBad States: None\n\n";

    LPAStarPlanner planner;
    PlanningResult r = planner.plan(p);
    printResult(r, names, planner.lastExploredStates(), planner.lastOperationMicros(), planner.badStates());
}

// ---------------------------------------------------------------------
// TEST CASE 2: Bad State Avoidance
// S -> A -> X(bad) -> G   and   S -> C -> D -> G
// ---------------------------------------------------------------------
static void runTestCase2() {
    printHeader("TEST CASE 2: BAD STATE AVOIDANCE");

    PlanningProblem p;
    p.states = {
        mkState(1, {0, 0}),  // S
        mkState(2, {1, 1}),  // A
        mkState(3, {2, 1}),  // X (bad)
        mkState(4, {1, -1}), // C
        mkState(5, {2, -1}), // D
        mkState(6, {3, 0}),  // G
    };
    p.transitions = {
        mkT(201, 1, 2, 1.0, 1.0, 0.95, true), // S->A
        mkT(202, 2, 3, 1.0, 1.0, 0.95, true), // A->X (bad)
        mkT(203, 3, 6, 1.0, 1.0, 0.95, true), // X->G
        mkT(204, 1, 4, 1.0, 1.0, 0.95, true), // S->C
        mkT(205, 4, 5, 1.0, 1.0, 0.95, true), // C->D
        mkT(206, 5, 6, 1.0, 1.0, 0.95, true), // D->G
    };
    p.initialState = 1; p.goalState = 6; p.badStates = {3};

    std::unordered_map<uint64_t, std::string> names = {
        {1, "S"}, {2, "A"}, {3, "X"}, {4, "C"}, {5, "D"}, {6, "G"}
    };

    std::cout << "Initial State: S\nGoal State: G\nBad States: X\n\n";

    LPAStarPlanner planner;
    PlanningResult r = planner.plan(p);
    printResult(r, names, planner.lastExploredStates(), planner.lastOperationMicros(), planner.badStates());

    bool containsX = std::find(r.statePath.begin(), r.statePath.end(), 3) != r.statePath.end();
    std::cout << "Verification - path contains bad state X: " << (containsX ? "YES (FAULT)" : "NO (correct)") << "\n\n";
}

// ---------------------------------------------------------------------
// TEST CASE 3: Safety Margin
// Path 1: S->A1->A2->G   (lower cost, close to bad state Y)
// Path 2: S->B1->B2->G   (higher cost, far from Y)
// ---------------------------------------------------------------------
static void runTestCase3() {
    printHeader("TEST CASE 3: SAFETY MARGIN");

    PlanningProblem p;
    p.states = {
        mkState(1, {0, 0}),   // S
        mkState(2, {1, 0.2}), // A1 (near bad state)
        mkState(3, {2, 0.2}), // A2 (near bad state)
        mkState(4, {3, 0}),   // G
        mkState(5, {1, 5}),   // B1 (far)
        mkState(6, {2, 5}),   // B2 (far)
        mkState(7, {10, 0}),  // Y (bad state, geometrically near A1/A2)
    };
    p.transitions = {
        mkT(301, 1, 2, 1.0, 0.6, 0.9, true),  // S->A1  cheap
        mkT(302, 2, 3, 1.0, 0.6, 0.9, true),  // A1->A2
        mkT(303, 3, 4, 1.0, 0.6, 0.9, true),  // A2->G   Path 1 total cost = 3
        mkT(304, 1, 5, 3.0, 0.95, 0.9, true), // S->B1  expensive
        mkT(305, 5, 6, 3.0, 0.95, 0.9, true), // B1->B2
        mkT(306, 6, 4, 3.0, 0.95, 0.9, true), // B2->G   Path 2 total cost = 9
    };
    p.initialState = 1; p.goalState = 4; p.badStates = {7};

    std::unordered_map<uint64_t, std::string> names = {
        {1, "S"}, {2, "A1"}, {3, "A2"}, {4, "G"}, {5, "B1"}, {6, "B2"}, {7, "Y"}
    };

    std::cout << "Initial State: S\nGoal State: G\nBad States: Y\n";
    std::cout << "Path 1 (S-A1-A2-G): lower cost, passes closer to Y\n";
    std::cout << "Path 2 (S-B1-B2-G): higher cost, stays farther from Y\n\n";

    LPAStarPlanner planner;
    PlanningResult r = planner.plan(p);
    printResult(r, names, planner.lastExploredStates(), planner.lastOperationMicros(), planner.badStates());

    bool tookPath1 = !r.statePath.empty() && r.statePath.size() >= 2 && r.statePath[1] == 2;
    std::cout << "Selected: " << (tookPath1 ? "Path 1 (lower cost)" : "Path 2 (higher cost)") << "\n";
    std::cout << "Explanation: this implementation's search minimizes TOTAL TRANSITION COST\n"
                 "among bad-state-free paths only; the geometric safety-distance objective\n"
                 "(objective 4) is computed and reported (Minimum Safety Distance above) but is\n"
                 "NOT part of the edge weight the search optimizes. Therefore the lower-cost\n"
                 "path is selected even though it passes closer to the bad state, as long as it\n"
                 "does not touch the bad state itself. This is the actual, honest behavior of\n"
                 "the implemented objective - not an assumption.\n";

    std::unordered_map<uint64_t, size_t> idxById;
    for (size_t i = 0; i < p.transitions.size(); ++i) idxById[p.transitions[i].id] = i;
    ScoreWeights w;
    double score = reportedScore(r, p.transitions, idxById, w);
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Reported (not optimized) Score(P) = alpha*G - beta*C + gamma*D + delta*R = " << score
              << "\n(weights: alpha=" << w.alpha << " beta=" << w.beta << " gamma=" << w.gamma
              << " delta=" << w.delta << " -- this metric is informational only; see explanation above)\n\n";
}

// ---------------------------------------------------------------------
// TEST CASE 4: Dynamic Transition  (S->A->G, then (A,G) unavailable)
// ---------------------------------------------------------------------
static void runTestCase4() {
    printHeader("TEST CASE 4: DYNAMIC TRANSITION");

    PlanningProblem p;
    p.states = { mkState(1, {0,0}), mkState(2, {1,0}), mkState(3, {2,0}), mkState(4, {1,-1}), mkState(5, {2,-1}) };
    // S=1 A=2 G=3, alternate C=4 D=5
    p.transitions = {
        mkT(401, 1, 2, 1.0, 1.0, 0.9, true),  // S->A
        mkT(402, 2, 3, 1.0, 1.0, 0.9, true),  // A->G
        mkT(403, 1, 4, 2.0, 1.0, 0.9, true),  // S->C
        mkT(404, 4, 5, 2.0, 1.0, 0.9, true),  // C->D
        mkT(405, 5, 3, 2.0, 1.0, 0.9, true),  // D->G
    };
    p.initialState = 1; p.goalState = 3; p.badStates = {};

    std::unordered_map<uint64_t, std::string> names = {{1,"S"},{2,"A"},{3,"G"},{4,"C"},{5,"D"}};

    LPAStarPlanner planner;
    std::cout << "--- Initial plan ---\n";
    PlanningResult r1 = planner.plan(p);
    printResult(r1, names, planner.lastExploredStates(), planner.lastOperationMicros(), planner.badStates());

    std::cout << "--- Making transition (A,G) [id 402] unavailable, then replanning ---\n";
    PlanningResult r2 = planner.setTransitionAvailability(402, false);
    printResult(r2, names, planner.lastExploredStates(), planner.lastOperationMicros(), planner.badStates());
}

// ---------------------------------------------------------------------
// TEST CASE 5: Goal Update
// ---------------------------------------------------------------------
static void runTestCase5() {
    printHeader("TEST CASE 5: GOAL UPDATE");

    PlanningProblem p;
    p.states = { mkState(1,{0,0}), mkState(2,{1,0}), mkState(3,{2,0}), mkState(4,{3,0}), mkState(5,{2,2}) };
    // S=1 -> A=2 -> G_old=3 -> G_new=4 ; also A -> H=5
    p.transitions = {
        mkT(501, 1, 2, 1.0, 1.0, 0.9, true), // S->A
        mkT(502, 2, 3, 1.0, 1.0, 0.9, true), // A->G_old
        mkT(503, 2, 5, 1.5, 1.0, 0.9, true), // A->H
        mkT(504, 5, 4, 1.0, 1.0, 0.9, true), // H->G_new
    };
    p.initialState = 1; p.goalState = 3; p.badStates = {};

    std::unordered_map<uint64_t, std::string> names = {{1,"S"},{2,"A"},{3,"G_old"},{4,"G_new"},{5,"H"}};

    LPAStarPlanner planner;
    std::cout << "--- Initial plan toward G_old ---\n";
    PlanningResult r1 = planner.plan(p);
    printResult(r1, names, planner.lastExploredStates(), planner.lastOperationMicros(), planner.badStates());

    std::cout << "--- Goal changes to G_new, replanning (warm-started, not from scratch) ---\n";
    PlanningResult r2 = planner.updateGoal(4);
    printResult(r2, names, planner.lastExploredStates(), planner.lastOperationMicros(), planner.badStates());

    std::cout << "Note: g-values computed during the FIRST plan (e.g. distance from S to A)\n"
                 "remain valid and are reused; only priority-queue keys were recalculated and\n"
                 "the search extended toward the new goal. This is a partial, warm-started\n"
                 "recomputation, not a full graph reinitialization - see PART 9 discussion.\n\n";
}

// ---------------------------------------------------------------------
// TEST CASE 6: Transition Addition
// ---------------------------------------------------------------------
static void runTestCase6() {
    printHeader("TEST CASE 6: TRANSITION ADDITION");

    PlanningProblem p;
    p.states = { mkState(1,{0,0}), mkState(2,{1,0}), mkState(3,{2,0}), mkState(4,{3,0}) };
    p.transitions = {
        mkT(601, 1, 2, 1.0, 1.0, 0.9, true), // S->A
        mkT(602, 2, 3, 1.0, 1.0, 0.9, true), // A->B
        mkT(603, 3, 4, 1.0, 1.0, 0.9, true), // B->G   total = 3
    };
    p.initialState = 1; p.goalState = 4; p.badStates = {};

    std::unordered_map<uint64_t, std::string> names = {{1,"S"},{2,"A"},{3,"B"},{4,"G"}};

    LPAStarPlanner planner;
    std::cout << "--- Initial plan (no shortcut) ---\n";
    PlanningResult r1 = planner.plan(p);
    printResult(r1, names, planner.lastExploredStates(), planner.lastOperationMicros(), planner.badStates());

    std::cout << "--- Adding shortcut transition S->G (id 604, cost 1.5), replanning ---\n";
    Transition shortcut = mkT(604, 1, 4, 1.5, 1.0, 0.9, true);
    PlanningResult r2 = planner.addTransition(shortcut);
    printResult(r2, names, planner.lastExploredStates(), planner.lastOperationMicros(), planner.badStates());
}

// ---------------------------------------------------------------------
// Additional safety edge cases required by PART 5 of the assignment:
// bad initial state, bad goal state, and "no safe path exists".
// ---------------------------------------------------------------------
static void runSafetyEdgeCases() {
    printHeader("ADDITIONAL SAFETY EDGE CASES (PART 5)");

    // Case A: bad initial state
    {
        PlanningProblem p;
        p.states = { mkState(1, {0,0}), mkState(2, {1,0}) };
        p.transitions = { mkT(701, 1, 2, 1.0, 1.0, 0.9, true) };
        p.initialState = 1; p.goalState = 2; p.badStates = {1}; // start itself is bad
        LPAStarPlanner planner;
        PlanningResult r = planner.plan(p);
        std::cout << "Case A - Initial state is a bad state -> success = "
                  << (r.success ? "true (FAULT)" : "false (correct)") << "\n";
    }
    // Case B: bad goal state
    {
        PlanningProblem p;
        p.states = { mkState(1, {0,0}), mkState(2, {1,0}) };
        p.transitions = { mkT(702, 1, 2, 1.0, 1.0, 0.9, true) };
        p.initialState = 1; p.goalState = 2; p.badStates = {2}; // goal itself is bad
        LPAStarPlanner planner;
        PlanningResult r = planner.plan(p);
        std::cout << "Case B - Goal state is a bad state -> success = "
                  << (r.success ? "true (FAULT)" : "false (correct)") << "\n";
    }
    // Case C: no safe path exists (only route passes through a bad state)
    {
        PlanningProblem p;
        p.states = { mkState(1, {0,0}), mkState(2, {1,0}), mkState(3, {2,0}) };
        p.transitions = { mkT(703, 1, 2, 1.0, 1.0, 0.9, true), mkT(704, 2, 3, 1.0, 1.0, 0.9, true) };
        p.initialState = 1; p.goalState = 3; p.badStates = {2}; // only route blocked
        LPAStarPlanner planner;
        PlanningResult r = planner.plan(p);
        std::cout << "Case C - Only route passes through a bad state -> success = "
                  << (r.success ? "true (FAULT)" : "false (correct)") << "\n";
    }
    // Case D: no bad states at all
    {
        PlanningProblem p;
        p.states = { mkState(1, {0,0}), mkState(2, {1,0}) };
        p.transitions = { mkT(705, 1, 2, 1.0, 1.0, 0.9, true) };
        p.initialState = 1; p.goalState = 2; p.badStates = {};
        LPAStarPlanner planner;
        PlanningResult r = planner.plan(p);
        std::cout << "Case D - No bad states defined -> success = " << (r.success ? "true (correct)" : "false (FAULT)")
                  << ", safetyScore is " << (std::isinf(r.safetyScore) ? "+infinity (correct, documented)" : "finite (FAULT)") << "\n";
    }
    std::cout << "\n";
}

int main() {
    std::cout << "PCCST503 Assignment 1 - Safe Semantic Planner (LPA*)\n";
    std::cout << "Illustrative test case run - values below are ACTUAL program output\n\n";

    runTestCase1();
    runTestCase2();
    runTestCase3();
    runTestCase4();
    runTestCase5();
    runTestCase6();
    runSafetyEdgeCases();

    return 0;
}
