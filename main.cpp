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
//   - Safety-aware optimization balancing transition cost, geometric clearance,
//     and transition safety attributes
//   - Cartesian safety-distance computation
//   - A conservative, admissibility-aware heuristic
//   - Incremental update handling for dynamic environment changes
//   - Memory-usage measurement (planner data structures & process RSS)
//   - All six illustrative test cases from the assignment
// =====================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#if defined(_WIN32) || defined(_WIN64)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <psapi.h>
#elif defined(__linux__) || defined(__APPLE__)
#include <sys/resource.h>
#include <unistd.h>
#endif

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
  double safety; // per-transition safety score (in [0, 1])
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
  virtual PlanningResult plan(const PlanningProblem &problem) = 0;
  virtual ~Planner() = default;
};

// ---------------------------------------------------------------------
// Small numeric helpers
// ---------------------------------------------------------------------

static const double INF = std::numeric_limits<double>::infinity();
static const double EPS = 1e-9;

static double euclideanDistance(const std::vector<double> &a,
                                const std::vector<double> &b) {
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
// System & Process Memory Usage Measurement
// ---------------------------------------------------------------------
// Measures physical memory (Resident Set Size / Working Set) used by the
// process. On Windows, uses GetProcessMemoryInfo; on POSIX systems, uses
// getrusage().

static size_t getProcessPeakRSSBytes() {
#if defined(_WIN32) || defined(_WIN64)
  PROCESS_MEMORY_COUNTERS pmc;
  if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
    return pmc.PeakWorkingSetSize;
  }
  return 0;
#elif defined(__linux__) || defined(__APPLE__)
  struct rusage rusage;
  if (getrusage(RUSAGE_SELF, &rusage) == 0) {
#if defined(__APPLE__)
    return (size_t)rusage.ru_maxrss;
#else
    return (size_t)rusage.ru_maxrss * 1024; // Linux returns KB
#endif
  }
  return 0;
#else
  return 0;
#endif
}

static size_t getProcessCurrentRSSBytes() {
#if defined(_WIN32) || defined(_WIN64)
  PROCESS_MEMORY_COUNTERS pmc;
  if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
    return pmc.WorkingSetSize;
  }
  return 0;
#elif defined(__linux__) || defined(__APPLE__)
  return getProcessPeakRSSBytes();
#else
  return 0;
#endif
}

static std::string formatBytes(size_t bytes) {
  if (bytes == 0) return "0 B";
  const char *units[] = {"B", "KB", "MB", "GB"};
  int unitIdx = 0;
  double dBytes = static_cast<double>(bytes);
  while (dBytes >= 1024.0 && unitIdx < 3) {
    dBytes /= 1024.0;
    unitIdx++;
  }
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << dBytes << " " << units[unitIdx]
      << " (" << bytes << " bytes)";
  return oss.str();
}

// ---------------------------------------------------------------------
// Safety Configuration
// ---------------------------------------------------------------------
// Effective edge cost w(e) = cost + kGeom * (1 / d_safe(to)) + kTrans * (1 - safety)
//   - kGeom: geometric proximity penalty weight for clearance to bad states
//   - kTrans: transition safety penalty weight for low per-transition safety
struct SafetyWeights {
  double kGeom = 2.0;    // Geometric proximity penalty weight
  double kTrans = 2.0;   // Transition safety attribute penalty weight
};

// ---------------------------------------------------------------------
// PART 4/5/6/7/8/9 - LPA* Planner implementation
// ---------------------------------------------------------------------
//
// Design notes & Safety Objective formulation:
//
//  * Hard Bad-State Constraint: Bad states are structurally excluded from
//    the search: any vertex in the bad-state set is forced to rhs = g = infinity
//    and is never inserted into the priority queue. No edge that starts or ends
//    at a bad state ever contributes to another vertex's rhs computation.
//    A final validation pass double-checks this before a successful result
//    is returned (PART 5 requirement).
//
//  * Safety Objective & Effective Edge Weights:
//    To maximize safety clearance and prioritize safer paths (as required in
//    Test Case 3), the planner computes an effective edge weight:
//      w(e) = cost(e) + kGeom * (1.0 / d_safe(to)) + kTrans * (1.0 - safety(e))
//    where d_safe(v) is the Euclidean distance from state v to the nearest bad
//    state. When no bad states exist, d_safe(v) = infinity, so the geometric
//    penalty vanishes.
//
//  * Mathematical Optimality & Correctness Guarantees:
//    1. Non-negativity: Since cost >= 0, kGeom >= 0, d_safe > 0 for valid states,
//       and safety <= 1.0, w(e) is strictly non-negative.
//    2. Monotonicity & Subpath Optimality: The additive cost model preserves
//       Bellman's principle of optimality.
//    3. Admissible & Consistent Heuristic: We scale Euclidean distance by
//       cMin = min over all transitions of (w(e) / geometric_distance).
//       By the triangle inequality on Cartesian embeddings:
//         h(u) <= cMin * dist(u, v) + h(v) <= w(u, v) + h(v)
//       guaranteeing that h(s) is strictly admissible and consistent.
//    4. Dynamic Replanning: LPA*'s incremental vertex-update mechanism
//       correctly propagates weight and graph changes without full rebuilds.
//

class LPAStarPlanner : public Planner {
public:
  explicit LPAStarPlanner(SafetyWeights weights = SafetyWeights{})
      : safetyWeights_(weights) {}

  void setSafetyWeights(double kGeom, double kTrans) {
    safetyWeights_.kGeom = kGeom;
    safetyWeights_.kTrans = kTrans;
    computeHeuristicScale();
  }

  const SafetyWeights &safetyWeights() const { return safetyWeights_; }

  // Memory usage estimator for all internal dynamic data structures
  size_t getEstimatedMemoryBytes() const {
    size_t bytes = sizeof(*this);
    // State hash map & embedding storage
    bytes += statesById_.size() * (sizeof(uint64_t) + sizeof(State) + 32);
    for (const auto &kv : statesById_) {
      bytes += kv.second.embedding.capacity() * sizeof(double);
    }
    // Transitions vector & index map
    bytes += transitions_.capacity() * sizeof(Transition);
    bytes += transitionIndexById_.size() * (sizeof(uint64_t) + sizeof(size_t) + 32);
    // Adjacency lists
    bytes += outIdx_.size() * (sizeof(uint64_t) + sizeof(std::vector<size_t>) + 32);
    for (const auto &kv : outIdx_) bytes += kv.second.capacity() * sizeof(size_t);
    bytes += inIdx_.size() * (sizeof(uint64_t) + sizeof(std::vector<size_t>) + 32);
    for (const auto &kv : inIdx_) bytes += kv.second.capacity() * sizeof(size_t);
    // Bad states set
    bytes += badStates_.size() * (sizeof(uint64_t) + 32);
    // LPA* search structures
    bytes += g_.size() * (sizeof(uint64_t) + sizeof(double) + 32);
    bytes += rhs_.size() * (sizeof(uint64_t) + sizeof(double) + 32);
    bytes += openList_.size() * (sizeof(std::tuple<double, double, uint64_t>) + 48); // red-black tree node
    bytes += openKeyOf_.size() * (sizeof(uint64_t) + sizeof(std::pair<double, double>) + 32);
    return bytes;
  }

  // ---- Public incremental-update surface -----------------------------
  // plan() performs a full (cold) initialization + search - this is the
  // Planner interface method required by the PDF.
  PlanningResult plan(const PlanningProblem &problem) override {
    loadProblem(problem);
    initializeSearch();
    auto start = std::chrono::steady_clock::now();
    exploredThisCall_ = 0;
    computeShortestPath();
    auto end = std::chrono::steady_clock::now();
    lastOperationMicros_ =
        std::chrono::duration<double, std::micro>(end - start).count();
    return reconstructResult();
  }

  // TRUE INCREMENTAL: flipping availability only affects the two
  // endpoints of the changed transition. UpdateVertex is called on the
  // 'to' endpoint (its rhs may change). Bounded by local degree.
  PlanningResult setTransitionAvailability(uint64_t transitionId,
                                           bool available) {
    timedOperation([&]() {
      auto it = transitionIndexById_.find(transitionId);
      if (it == transitionIndexById_.end())
        return;
      Transition &t = transitions_[it->second];
      if (t.available == available)
        return;
      t.available = available;
      updateVertex(t.to);
      computeShortestPath();
    });
    return reconstructResult();
  }

  // TRUE INCREMENTAL: a new edge only ever affects the rhs of its 'to'
  // endpoint (a new possible predecessor appeared for that vertex).
  // We also refresh the heuristic scale factor cMin.
  PlanningResult addTransition(const Transition &t) {
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
      if (it == transitionIndexById_.end())
        return;
      size_t idx = it->second;
      uint64_t to = transitions_[idx].to;
      transitions_[idx].available =
          false; // logically remove: excluded from all searches
      removeFromAdjacency(idx);
      transitionIndexById_.erase(it);
      updateVertex(to);
      computeShortestPath();
    });
    return reconstructResult();
  }

  // PARTIAL RE-COMPUTATION, NOT A FULL REBUILD.
  // g/rhs values computed so far remain valid because they represent
  // distance-from-a-fixed-start, which does not depend on the goal.
  // We recalculate priority queue keys for the new goal and continue search.
  PlanningResult updateGoal(uint64_t newGoal) {
    timedOperation([&]() {
      goal_ = newGoal;
      if (!g_.count(goal_))
        g_[goal_] = INF;
      if (!rhs_.count(goal_))
        rhs_[goal_] = INF;
      std::vector<uint64_t> queued;
      queued.reserve(openList_.size());
      for (auto &entry : openList_)
        queued.push_back(std::get<2>(entry));
      openList_.clear();
      openKeyOf_.clear();
      for (uint64_t v : queued)
        insertIntoOpenList(v, calculateKey(v));
      updateVertex(goal_);
      computeShortestPath();
    });
    return reconstructResult();
  }

  // DYNAMIC BAD STATES UPDATE:
  // Refreshes safety distances and updates affected vertices.
  PlanningResult updateBadStates(const std::vector<uint64_t> &newBadStates) {
    timedOperation([&]() {
      std::unordered_set<uint64_t> newSet(newBadStates.begin(),
                                          newBadStates.end());
      std::unordered_set<uint64_t> changed;
      for (uint64_t s : badStates_)
        if (!newSet.count(s))
          changed.insert(s); // became safe
      for (uint64_t s : newSet)
        if (!badStates_.count(s))
          changed.insert(s); // became bad
      badStates_ = newSet;

      // Refresh heuristic scale for updated safety landscape
      computeHeuristicScale();

      // Update all vertices whose proximity to bad states or reachable predecessor status changed
      for (const auto &kv : statesById_) {
        updateVertex(kv.first);
      }
      computeShortestPath();
    });
    return reconstructResult();
  }

  size_t lastExploredStates() const { return exploredThisCall_; }
  double lastOperationMicros() const { return lastOperationMicros_; }
  const std::unordered_set<uint64_t> &badStates() const { return badStates_; }

  double getDistanceToNearestBadState(uint64_t s) const {
    if (badStates_.empty() || !statesById_.count(s))
      return INF;
    double minD = INF;
    const auto &emb = statesById_.at(s).embedding;
    for (uint64_t b : badStates_) {
      if (!statesById_.count(b))
        continue;
      double d = euclideanDistance(emb, statesById_.at(b).embedding);
      minD = std::min(minD, d);
    }
    return minD;
  }

  double computeEdgeCost(const Transition &t) const {
    if (badStates_.count(t.from) || badStates_.count(t.to)) {
      return INF;
    }
    double w = t.cost;
    // 1. Transition safety attribute penalty: (1.0 - safety) * kTrans
    if (safetyWeights_.kTrans > 0.0 && t.safety < 1.0) {
      w += safetyWeights_.kTrans * std::max(0.0, 1.0 - t.safety);
    }
    // 2. Geometric clearance penalty: (1.0 / d_safe(to)) * kGeom
    if (safetyWeights_.kGeom > 0.0 && !badStates_.empty() && statesById_.count(t.to)) {
      double dSafe = getDistanceToNearestBadState(t.to);
      if (dSafe > EPS) {
        w += safetyWeights_.kGeom / dSafe;
      } else {
        return INF; // Exactly on or infinitely close to bad state
      }
    }
    return w;
  }

private:
  SafetyWeights safetyWeights_;

  // ---- Graph storage (Section 2.4 of the architecture) ---------------
  std::unordered_map<uint64_t, State> statesById_;
  std::vector<Transition> transitions_;
  std::unordered_map<uint64_t, size_t> transitionIndexById_;
  std::unordered_map<uint64_t, std::vector<size_t>>
      outIdx_; // from -> transition indices
  std::unordered_map<uint64_t, std::vector<size_t>>
      inIdx_; // to   -> transition indices
  std::unordered_set<uint64_t> badStates_;
  uint64_t start_ = 0, goal_ = 0;

  // ---- LPA* search state ----------------------------------------------
  std::unordered_map<uint64_t, double> g_, rhs_;
  // Open list emulated with an ordered set of (k1, k2, id) plus a lookup
  // map so we can test membership / current key in O(log n).
  std::set<std::tuple<double, double, uint64_t>> openList_;
  std::unordered_map<uint64_t, std::pair<double, double>> openKeyOf_;

  double heuristicScale_ = 0.0; // cMin: keeps h(s) admissible
  size_t exploredThisCall_ = 0;
  double lastOperationMicros_ = 0.0;

  // ---- Setup -----------------------------------------------------------
  void loadProblem(const PlanningProblem &problem) {
    statesById_.clear();
    for (const auto &s : problem.states)
      statesById_[s.id] = s;

    transitions_.clear();
    transitionIndexById_.clear();
    outIdx_.clear();
    inIdx_.clear();
    for (const auto &t : problem.transitions) {
      transitions_.push_back(t);
      size_t idx = transitions_.size() - 1;
      transitionIndexById_[t.id] = idx;
      outIdx_[t.from].push_back(idx);
      inIdx_[t.to].push_back(idx);
    }

    badStates_.clear();
    for (uint64_t b : problem.badStates)
      badStates_.insert(b);

    start_ = problem.initialState;
    goal_ = problem.goalState;

    computeHeuristicScale();
  }

  void computeHeuristicScale() {
    // cMin = min over all transitions of (effective_cost / geometric distance),
    // considering ONLY pairs with a positive geometric distance.
    double m = INF;
    for (const auto &t : transitions_) {
      if (!statesById_.count(t.from) || !statesById_.count(t.to))
        continue;
      if (badStates_.count(t.from) || badStates_.count(t.to))
        continue;
      double d = euclideanDistance(statesById_[t.from].embedding,
                                   statesById_[t.to].embedding);
      double effCost = computeEdgeCost(t);
      if (d > EPS && effCost >= 0.0 && effCost < INF)
        m = std::min(m, effCost / d);
    }
    heuristicScale_ = (m == INF) ? 0.0 : m; // 0.0 => fall back to h = 0 (admissible)
  }

  void refreshHeuristicScaleForTransition(const Transition &t) {
    if (!statesById_.count(t.from) || !statesById_.count(t.to))
      return;
    if (badStates_.count(t.from) || badStates_.count(t.to))
      return;
    double d = euclideanDistance(statesById_[t.from].embedding,
                                 statesById_[t.to].embedding);
    double effCost = computeEdgeCost(t);
    if (d > EPS && effCost >= 0.0 && effCost < INF) {
      double ratio = effCost / d;
      heuristicScale_ =
          (heuristicScale_ <= 0.0) ? ratio : std::min(heuristicScale_, ratio);
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
    if (heuristicScale_ <= 0.0)
      return 0.0; // conservative fallback: always admissible
    if (!statesById_.count(s) || !statesById_.count(goal_))
      return 0.0;
    double d = euclideanDistance(statesById_.at(s).embedding,
                                 statesById_.at(goal_).embedding);
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
    if (it == openKeyOf_.end())
      return;
    openList_.erase({it->second.first, it->second.second, s});
    openKeyOf_.erase(it);
  }

  void initializeSearch() {
    g_.clear();
    rhs_.clear();
    openList_.clear();
    openKeyOf_.clear();
    exploredThisCall_ = 0;
    for (const auto &kv : statesById_)
      g_[kv.first] = INF;
    rhs_[start_] = 0.0;
    g_[start_] = INF;
    if (badStates_.count(start_)) {
      rhs_[start_] = INF;
    } else {
      insertIntoOpenList(start_, calculateKey(start_));
    }
  }

  void updateVertex(uint64_t u) {
    if (badStates_.count(u)) {
      g_[u] = INF;
      rhs_[u] = INF;
      removeFromOpenList(u);
      return;
    }
    if (u != start_) {
      double best = INF;
      for (size_t idx : inIdx_[u]) {
        const Transition &t = transitions_[idx];
        if (!t.available)
          continue;
        if (badStates_.count(t.from))
          continue; // never route through a bad predecessor
        double gp = getG(t.from);
        if (gp < INF) {
          double effCost = computeEdgeCost(t);
          if (effCost < INF) {
            best = std::min(best, gp + effCost);
          }
        }
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
      if (!(topKey < goalKey) && goalConsistent)
        break;

      uint64_t u = std::get<2>(top);
      openList_.erase(openList_.begin());
      openKeyOf_.erase(u);
      exploredThisCall_++;

      double gu = getG(u), ru = getRhs(u);
      if (gu > ru + EPS) {
        g_[u] = ru;
        for (size_t idx : outIdx_[u]) {
          const Transition &t = transitions_[idx];
          if (!t.available)
            continue;
          updateVertex(t.to);
        }
      } else {
        g_[u] = INF;
        updateVertex(u);
        for (size_t idx : outIdx_[u]) {
          const Transition &t = transitions_[idx];
          if (!t.available)
            continue;
          updateVertex(t.to);
        }
      }
    }
  }

  void removeFromAdjacency(size_t idx) {
    uint64_t from = transitions_[idx].from, to = transitions_[idx].to;
    auto &outs = outIdx_[from];
    outs.erase(std::remove(outs.begin(), outs.end(), idx), outs.end());
    auto &ins = inIdx_[to];
    ins.erase(std::remove(ins.begin(), ins.end(), idx), ins.end());
  }

  template <typename F> void timedOperation(F &&f) {
    exploredThisCall_ = 0;
    auto t0 = std::chrono::steady_clock::now();
    f();
    auto t1 = std::chrono::steady_clock::now();
    lastOperationMicros_ =
        std::chrono::duration<double, std::micro>(t1 - t0).count();
  }

  // ---- PART 5 / 6 - result construction with final safety validation ---
  PlanningResult reconstructResult() {
    PlanningResult result;

    if (badStates_.count(start_) || badStates_.count(goal_)) {
      result.success = false;
      return result;
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
        const Transition &t = transitions_[idx];
        if (!t.available)
          continue;
        if (badStates_.count(t.from))
          continue;
        double gp = getG(t.from);
        if (gp >= INF)
          continue;
        double effCost = computeEdgeCost(t);
        if (effCost >= INF)
          continue;
        double val = gp + effCost;
        if (val < bestVal - EPS) {
          bestVal = val;
          bestPred = t.from;
          bestIdx = idx;
        }
      }
      if (bestIdx == SIZE_MAX) {
        ok = false;
        break;
      }
      transitionPath.push_back(transitions_[bestIdx].id);
      cur = bestPred;
      if (guard.count(cur)) {
        ok = false;
        break;
      } // cycle guard
      guard.insert(cur);
      statePath.push_back(cur);
      if (statePath.size() > statesById_.size() + 1) {
        ok = false;
        break;
      }
    }

    if (!ok) {
      result.success = false;
      return result;
    }

    std::reverse(statePath.begin(), statePath.end());
    std::reverse(transitionPath.begin(), transitionPath.end());

    // FINAL SAFETY VALIDATION (PART 5 requirement): re-check the
    // complete path against the current bad-state set before reporting success.
    for (uint64_t s : statePath) {
      if (badStates_.count(s)) {
        result.success = false;
        return result;
      }
    }

    double totalCost = 0.0;
    for (uint64_t tid : transitionPath) {
      const Transition &t = transitions_[transitionIndexById_[tid]];
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
  // the nearest bad state. If there are no bad states, this is +infinity.
  double computeSafetyScore(const std::vector<uint64_t> &statePath) const {
    if (badStates_.empty())
      return INF;
    double minOverPath = INF;
    for (uint64_t s : statePath) {
      if (!statesById_.count(s))
        continue;
      double nearest = getDistanceToNearestBadState(s);
      minOverPath = std::min(minOverPath, nearest);
    }
    return minOverPath;
  }
};

// =====================================================================
// PART 7 - reported combined score, computed AFTER a planning result exists
// Score(P) = alpha*G - beta*C + gamma*D + delta*R
// =====================================================================
struct ScoreWeights {
  double alpha = 10.0, beta = 1.0, gamma = 1.0, delta = 1.0;
};

static double reportedScore(const PlanningResult &r,
                            const std::vector<Transition> &allTransitions,
                            const std::unordered_map<uint64_t, size_t> &idxById,
                            const ScoreWeights &w) {
  double G = r.success ? 1.0 : 0.0;
  double C = r.totalCost;
  double D = std::isinf(r.safetyScore)
                 ? 0.0
                 : r.safetyScore; // treat "no bad states" as 0 bonus
  double R = 1.0;
  for (uint64_t tid : r.transitionPath) {
    auto it = idxById.find(tid);
    if (it != idxById.end())
      R *= allTransitions[it->second].reliability;
  }
  if (r.transitionPath.empty())
    R = 1.0;
  return w.alpha * G - w.beta * C + w.gamma * D + w.delta * R;
}

// =====================================================================
// Formatting helpers for output
// =====================================================================

static void printHeader(const std::string &title) {
  std::cout << "========================================\n";
  std::cout << title << "\n";
  std::cout << "========================================\n";
}

static std::string
pathToString(const std::vector<uint64_t> &path,
             const std::unordered_map<uint64_t, std::string> &names) {
  std::ostringstream oss;
  for (size_t i = 0; i < path.size(); ++i) {
    auto it = names.find(path[i]);
    oss << (it != names.end() ? it->second : std::to_string(path[i]));
    if (i + 1 < path.size())
      oss << " -> ";
  }
  return oss.str();
}

static std::string transitionsToString(const std::vector<uint64_t> &path) {
  std::ostringstream oss;
  for (size_t i = 0; i < path.size(); ++i) {
    oss << "T" << path[i];
    if (i + 1 < path.size())
      oss << " -> ";
  }
  return oss.str();
}

static void printResult(const PlanningResult &r,
                        const std::unordered_map<uint64_t, std::string> &names,
                        size_t explored, double micros,
                        const std::unordered_set<uint64_t> &badStates,
                        size_t plannerHeapBytes = 0) {
  std::cout << "Result: " << (r.success ? "SUCCESS" : "FAILURE") << "\n\n";
  if (r.success) {
    std::cout << "State Path:\n" << pathToString(r.statePath, names) << "\n\n";
    std::cout << "Transition Path:\n"
              << transitionsToString(r.transitionPath) << "\n\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Total Cost: " << r.totalCost << "\n";
    std::cout << "Minimum Safety Distance: "
              << (std::isinf(r.safetyScore)
                      ? std::string("N/A (no bad states defined)")
                      : std::to_string(r.safetyScore))
              << "\n";
    int badVisited = 0;
    for (uint64_t s : r.statePath)
      if (badStates.count(s))
        badVisited++;
    std::cout << "Bad States Visited: " << badVisited << "\n";
  }
  std::cout << "Explored States: " << explored << "\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "Planning Time: " << micros << " microseconds\n";
  if (plannerHeapBytes > 0) {
    std::cout << "Planner Memory Footprint: " << formatBytes(plannerHeapBytes) << "\n";
  }
  size_t peakRSS = getProcessPeakRSSBytes();
  if (peakRSS > 0) {
    std::cout << "Process Peak Working Set (RSS): " << formatBytes(peakRSS) << "\n";
  }
  std::cout << "\n";
}

// =====================================================================
// Test case scenario builders
// =====================================================================

static State mkState(uint64_t id, std::vector<double> emb) {
  return State{id, std::move(emb)};
}
static Transition mkT(uint64_t id, uint64_t from, uint64_t to, double cost,
                      double safety, double rel, bool avail) {
  return Transition{id, from, to, cost, safety, rel, avail};
}

// ---------------------------------------------------------------------
// TEST CASE 1: Basic Reachability   S -> A -> B -> G
// ---------------------------------------------------------------------
static void runTestCase1() {
  printHeader("TEST CASE 1: BASIC REACHABILITY");

  PlanningProblem p;
  p.states = {mkState(1, {0, 0}), mkState(2, {1, 0}), mkState(3, {2, 0}),
              mkState(4, {3, 0})};
  p.transitions = {
      mkT(101, 1, 2, 1.0, 1.0, 0.99, true),
      mkT(102, 2, 3, 1.0, 1.0, 0.99, true),
      mkT(103, 3, 4, 1.0, 1.0, 0.99, true),
  };
  p.initialState = 1;
  p.goalState = 4;
  p.badStates = {};

  std::unordered_map<uint64_t, std::string> names = {
      {1, "S"}, {2, "A"}, {3, "B"}, {4, "G"}};

  std::cout << "Initial State: S\nGoal State: G\nBad States: None\n\n";

  LPAStarPlanner planner;
  PlanningResult r = planner.plan(p);
  printResult(r, names, planner.lastExploredStates(),
              planner.lastOperationMicros(), planner.badStates(),
              planner.getEstimatedMemoryBytes());
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
  p.initialState = 1;
  p.goalState = 6;
  p.badStates = {3};

  std::unordered_map<uint64_t, std::string> names = {
      {1, "S"}, {2, "A"}, {3, "X"}, {4, "C"}, {5, "D"}, {6, "G"}};

  std::cout << "Initial State: S\nGoal State: G\nBad States: X\n\n";

  LPAStarPlanner planner;
  PlanningResult r = planner.plan(p);
  printResult(r, names, planner.lastExploredStates(),
              planner.lastOperationMicros(), planner.badStates(),
              planner.getEstimatedMemoryBytes());

  bool containsX =
      std::find(r.statePath.begin(), r.statePath.end(), 3) != r.statePath.end();
  std::cout << "Verification - path contains bad state X: "
            << (containsX ? "YES (FAULT)" : "NO (correct)") << "\n\n";
}

// ---------------------------------------------------------------------
// TEST CASE 3: Safety Margin & Minimum Safety Distance Optimization
// Path 1: S->A1->A2->G   (lower transition cost 3.0, close to bad state Y, clearance ~0.54)
// Path 2: S->B1->B2->G   (higher transition cost 9.0, far from Y, clearance 1.50)
// ---------------------------------------------------------------------
static void runTestCase3() {
  printHeader("TEST CASE 3: SAFETY MARGIN");

  PlanningProblem p;
  p.states = {
      mkState(1, {0, 0}),   // S
      mkState(2, {1, 0.2}), // A1 (near bad state Y)
      mkState(3, {2, 0.2}), // A2 (near bad state Y)
      mkState(4, {3, 0}),   // G
      mkState(5, {1, 5}),   // B1 (far from Y)
      mkState(6, {2, 5}),   // B2 (far from Y)
      mkState(7, {1.5, 0.0}), // Y (bad state, geometrically near A1/A2)
  };
  p.transitions = {
      mkT(301, 1, 2, 1.0, 0.6, 0.9, true),  // S->A1  cheap, low safety
      mkT(302, 2, 3, 1.0, 0.6, 0.9, true),  // A1->A2
      mkT(303, 3, 4, 1.0, 0.6, 0.9, true),  // A2->G   Path 1 total base cost = 3
      mkT(304, 1, 5, 3.0, 0.95, 0.9, true), // S->B1  higher cost, high safety
      mkT(305, 5, 6, 3.0, 0.95, 0.9, true), // B1->B2
      mkT(306, 6, 4, 3.0, 0.95, 0.9, true), // B2->G   Path 2 total base cost = 9
  };
  p.initialState = 1;
  p.goalState = 4;
  p.badStates = {7};

  std::unordered_map<uint64_t, std::string> names = {
      {1, "S"}, {2, "A1"}, {3, "A2"}, {4, "G"}, {5, "B1"}, {6, "B2"}, {7, "Y"}};

  std::cout << "Initial State: S\nGoal State: G\nBad States: Y\n";
  std::cout << "Path 1 (S-A1-A2-G): lower cost (3.0), but passes close to Y (clearance ~0.54, trans safety 0.60)\n";
  std::cout << "Path 2 (S-B1-B2-G): higher cost (9.0), but stays far from Y (clearance 1.50, trans safety 0.95)\n\n";

  LPAStarPlanner planner;
  PlanningResult r = planner.plan(p);
  printResult(r, names, planner.lastExploredStates(),
              planner.lastOperationMicros(), planner.badStates(),
              planner.getEstimatedMemoryBytes());

  bool tookPath2 =
      !r.statePath.empty() && r.statePath.size() >= 2 && r.statePath[1] == 5;
  std::cout << "Selected: "
            << (tookPath2 ? "Path 2 (safer path, maximized safety clearance)"
                          : "Path 1 (lower cost path)")
            << "\n\n";
  std::cout << "Explanation & Optimality Guarantee:\n"
               "  1. Safety Objective: The planner integrates both geometric clearance\n"
               "     (1/d_safe penalty) and per-transition safety (1 - safety) into the\n"
               "     effective edge weight w(e) = cost + kGeom*(1/d_safe) + kTrans*(1-safety).\n"
               "  2. Maximizing Safety Distance: Path 1 has a minimum safety clearance of 0.5385,\n"
               "     while Path 2 provides a minimum safety clearance of 1.5000 (nearly 3x safer).\n"
               "     Path 2 also features higher transition safety (0.95 vs 0.60).\n"
               "  3. LPA* Correctness: Because w(e) >= 0 and the heuristic h(s) is scaled\n"
               "     by cMin = min(w(e)/d(e)), h(s) remains strictly admissible and consistent.\n"
               "     LPA* is guaranteed to find the globally optimal safe path and preserve all\n"
               "     incremental replanning guarantees.\n\n";

  std::unordered_map<uint64_t, size_t> idxById;
  for (size_t i = 0; i < p.transitions.size(); ++i)
    idxById[p.transitions[i].id] = i;
  ScoreWeights w;
  double score = reportedScore(r, p.transitions, idxById, w);
  std::cout << std::fixed << std::setprecision(4);
  std::cout
      << "Combined Evaluation Score(P) = alpha*G - beta*C + gamma*D + delta*R = "
      << score << "\n(weights: alpha=" << w.alpha << " beta=" << w.beta
      << " gamma=" << w.gamma << " delta=" << w.delta << ")\n\n";
}

// ---------------------------------------------------------------------
// TEST CASE 4: Dynamic Transition  (S->A->G, then (A,G) unavailable)
// ---------------------------------------------------------------------
static void runTestCase4() {
  printHeader("TEST CASE 4: DYNAMIC TRANSITION");

  PlanningProblem p;
  p.states = {mkState(1, {0, 0}), mkState(2, {1, 0}), mkState(3, {2, 0}),
              mkState(4, {1, -1}), mkState(5, {2, -1})};
  // S=1 A=2 G=3, alternate C=4 D=5
  p.transitions = {
      mkT(401, 1, 2, 1.0, 1.0, 0.9, true), // S->A
      mkT(402, 2, 3, 1.0, 1.0, 0.9, true), // A->G
      mkT(403, 1, 4, 2.0, 1.0, 0.9, true), // S->C
      mkT(404, 4, 5, 2.0, 1.0, 0.9, true), // C->D
      mkT(405, 5, 3, 2.0, 1.0, 0.9, true), // D->G
  };
  p.initialState = 1;
  p.goalState = 3;
  p.badStates = {};

  std::unordered_map<uint64_t, std::string> names = {
      {1, "S"}, {2, "A"}, {3, "G"}, {4, "C"}, {5, "D"}};

  LPAStarPlanner planner;
  std::cout << "--- Initial plan ---\n";
  PlanningResult r1 = planner.plan(p);
  printResult(r1, names, planner.lastExploredStates(),
              planner.lastOperationMicros(), planner.badStates(),
              planner.getEstimatedMemoryBytes());

  std::cout << "--- Making transition (A,G) [id 402] unavailable, then "
               "replanning ---\n";
  PlanningResult r2 = planner.setTransitionAvailability(402, false);
  printResult(r2, names, planner.lastExploredStates(),
              planner.lastOperationMicros(), planner.badStates(),
              planner.getEstimatedMemoryBytes());
}

// ---------------------------------------------------------------------
// TEST CASE 5: Goal Update
// ---------------------------------------------------------------------
static void runTestCase5() {
  printHeader("TEST CASE 5: GOAL UPDATE");

  PlanningProblem p;
  p.states = {mkState(1, {0, 0}), mkState(2, {1, 0}), mkState(3, {2, 0}),
              mkState(4, {3, 0}), mkState(5, {2, 2})};
  // S=1 -> A=2 -> G_old=3 -> G_new=4 ; also A -> H=5
  p.transitions = {
      mkT(501, 1, 2, 1.0, 1.0, 0.9, true), // S->A
      mkT(502, 2, 3, 1.0, 1.0, 0.9, true), // A->G_old
      mkT(503, 2, 5, 1.5, 1.0, 0.9, true), // A->H
      mkT(504, 5, 4, 1.0, 1.0, 0.9, true), // H->G_new
  };
  p.initialState = 1;
  p.goalState = 3;
  p.badStates = {};

  std::unordered_map<uint64_t, std::string> names = {
      {1, "S"}, {2, "A"}, {3, "G_old"}, {4, "G_new"}, {5, "H"}};

  LPAStarPlanner planner;
  std::cout << "--- Initial plan toward G_old ---\n";
  PlanningResult r1 = planner.plan(p);
  printResult(r1, names, planner.lastExploredStates(),
              planner.lastOperationMicros(), planner.badStates(),
              planner.getEstimatedMemoryBytes());

  std::cout << "--- Goal changes to G_new, replanning (warm-started, not from "
               "scratch) ---\n";
  PlanningResult r2 = planner.updateGoal(4);
  printResult(r2, names, planner.lastExploredStates(),
              planner.lastOperationMicros(), planner.badStates(),
              planner.getEstimatedMemoryBytes());

  std::cout << "Note: g-values computed during the FIRST plan (e.g. distance "
               "from S to A)\n"
               "remain valid and are reused; only priority-queue keys were "
               "recalculated and\n"
               "the search extended toward the new goal. This is a partial, "
               "warm-started\n"
               "recomputation, not a full graph reinitialization - see PART 9 "
               "discussion.\n\n";
}

// ---------------------------------------------------------------------
// TEST CASE 6: Transition Addition
// ---------------------------------------------------------------------
static void runTestCase6() {
  printHeader("TEST CASE 6: TRANSITION ADDITION");

  PlanningProblem p;
  p.states = {mkState(1, {0, 0}), mkState(2, {1, 0}), mkState(3, {2, 0}),
              mkState(4, {3, 0})};
  p.transitions = {
      mkT(601, 1, 2, 1.0, 1.0, 0.9, true), // S->A
      mkT(602, 2, 3, 1.0, 1.0, 0.9, true), // A->B
      mkT(603, 3, 4, 1.0, 1.0, 0.9, true), // B->G   total = 3
  };
  p.initialState = 1;
  p.goalState = 4;
  p.badStates = {};

  std::unordered_map<uint64_t, std::string> names = {
      {1, "S"}, {2, "A"}, {3, "B"}, {4, "G"}};

  LPAStarPlanner planner;
  std::cout << "--- Initial plan (no shortcut) ---\n";
  PlanningResult r1 = planner.plan(p);
  printResult(r1, names, planner.lastExploredStates(),
              planner.lastOperationMicros(), planner.badStates(),
              planner.getEstimatedMemoryBytes());

  std::cout << "--- Adding shortcut transition S->G (id 604, cost 1.5), "
               "replanning ---\n";
  Transition shortcut = mkT(604, 1, 4, 1.5, 1.0, 0.9, true);
  PlanningResult r2 = planner.addTransition(shortcut);
  printResult(r2, names, planner.lastExploredStates(),
              planner.lastOperationMicros(), planner.badStates(),
              planner.getEstimatedMemoryBytes());
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
    p.states = {mkState(1, {0, 0}), mkState(2, {1, 0})};
    p.transitions = {mkT(701, 1, 2, 1.0, 1.0, 0.9, true)};
    p.initialState = 1;
    p.goalState = 2;
    p.badStates = {1}; // start itself is bad
    LPAStarPlanner planner;
    PlanningResult r = planner.plan(p);
    std::cout << "Case A - Initial state is a bad state -> success = "
              << (r.success ? "true (FAULT)" : "false (correct)") << "\n";
  }
  // Case B: bad goal state
  {
    PlanningProblem p;
    p.states = {mkState(1, {0, 0}), mkState(2, {1, 0})};
    p.transitions = {mkT(702, 1, 2, 1.0, 1.0, 0.9, true)};
    p.initialState = 1;
    p.goalState = 2;
    p.badStates = {2}; // goal itself is bad
    LPAStarPlanner planner;
    PlanningResult r = planner.plan(p);
    std::cout << "Case B - Goal state is a bad state -> success = "
              << (r.success ? "true (FAULT)" : "false (correct)") << "\n";
  }
  // Case C: no safe path exists (only route passes through a bad state)
  {
    PlanningProblem p;
    p.states = {mkState(1, {0, 0}), mkState(2, {1, 0}), mkState(3, {2, 0})};
    p.transitions = {mkT(703, 1, 2, 1.0, 1.0, 0.9, true),
                     mkT(704, 2, 3, 1.0, 1.0, 0.9, true)};
    p.initialState = 1;
    p.goalState = 3;
    p.badStates = {2}; // only route blocked
    LPAStarPlanner planner;
    PlanningResult r = planner.plan(p);
    std::cout << "Case C - Only route passes through a bad state -> success = "
              << (r.success ? "true (FAULT)" : "false (correct)") << "\n";
  }
  // Case D: no bad states at all
  {
    PlanningProblem p;
    p.states = {mkState(1, {0, 0}), mkState(2, {1, 0})};
    p.transitions = {mkT(705, 1, 2, 1.0, 1.0, 0.9, true)};
    p.initialState = 1;
    p.goalState = 2;
    p.badStates = {};
    LPAStarPlanner planner;
    PlanningResult r = planner.plan(p);
    std::cout << "Case D - No bad states defined -> success = "
              << (r.success ? "true (correct)" : "false (FAULT)")
              << ", safetyScore is "
              << (std::isinf(r.safetyScore) ? "+infinity (correct, documented)"
                                            : "finite (FAULT)")
              << "\n";
  }
  std::cout << "\n";
}

int main() {
  std::cout << "PCCST503 Assignment 1 - Safe Semantic Planner (LPA*)\n";
  std::cout << "Illustrative test case run - values below are ACTUAL program "
               "output\n\n";

  runTestCase1();
  runTestCase2();
  runTestCase3();
  runTestCase4();
  runTestCase5();
  runTestCase6();
  runSafetyEdgeCases();

  return 0;
}