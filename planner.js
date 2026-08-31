/**
 * Safe Semantic Planner (LPA* Implementation in JavaScript)
 * Fully compliant with the C++17 SafeSemanticPlanner implementation.
 */

class LPAStarPlanner {
  constructor(options = {}) {
    this.kGeom = options.kGeom ?? 2.0;
    this.kTrans = options.kTrans ?? 2.0;
    this.alpha = options.alpha ?? 10.0;
    this.beta = options.beta ?? 1.0;
    this.gamma = options.gamma ?? 1.0;
    this.delta = options.delta ?? 1.0;

    this.statesById = new Map();
    this.transitions = [];
    this.transitionIndexById = new Map();
    this.outIdx = new Map();
    this.inIdx = new Map();
    this.badStates = new Set();
    this.start = 0;
    this.goal = 0;

    this.g = new Map();
    this.rhs = new Map();
    this.openList = []; // sorted array of { k1, k2, id }
    this.openKeyOf = new Map();

    this.heuristicScale = 0.0;
    this.exploredThisCall = 0;
    this.lastOperationMicros = 0;
  }

  setSafetyWeights(kGeom, kTrans) {
    this.kGeom = kGeom;
    this.kTrans = kTrans;
    this.computeHeuristicScale();
  }

  static euclideanDistance(a, b) {
    const n = Math.min(a.length, b.length);
    let sumSq = 0;
    for (let i = 0; i < n; i++) {
      const d = a[i] - b[i];
      sumSq += d * d;
    }
    return Math.sqrt(sumSq);
  }

  getDistanceToNearestBadState(sId) {
    if (this.badStates.size === 0 || !this.statesById.has(sId)) return Infinity;
    let minD = Infinity;
    const emb = this.statesById.get(sId).embedding;
    for (const bId of this.badStates) {
      if (!this.statesById.has(bId)) continue;
      const d = LPAStarPlanner.euclideanDistance(emb, this.statesById.get(bId).embedding);
      if (d < minD) minD = d;
    }
    return minD;
  }

  computeEdgeCost(t) {
    if (this.badStates.has(t.from) || this.badStates.has(t.to)) {
      return Infinity;
    }
    let w = t.cost;
    // 1. Transition safety attribute penalty
    if (this.kTrans > 0 && t.safety < 1.0) {
      w += this.kTrans * Math.max(0, 1.0 - t.safety);
    }
    // 2. Geometric clearance penalty
    if (this.kGeom > 0 && this.badStates.size > 0 && this.statesById.has(t.to)) {
      const dSafe = this.getDistanceToNearestBadState(t.to);
      if (dSafe > 1e-9) {
        w += this.kGeom / dSafe;
      } else {
        return Infinity;
      }
    }
    return w;
  }

  loadProblem(problem) {
    this.statesById.clear();
    for (const s of problem.states) {
      this.statesById.set(s.id, { ...s, embedding: [...s.embedding] });
    }

    this.transitions = [];
    this.transitionIndexById.clear();
    this.outIdx.clear();
    this.inIdx.clear();

    for (let i = 0; i < problem.transitions.length; i++) {
      const t = { ...problem.transitions[i] };
      this.transitions.push(t);
      this.transitionIndexById.set(t.id, i);

      if (!this.outIdx.has(t.from)) this.outIdx.set(t.from, []);
      this.outIdx.get(t.from).push(i);

      if (!this.inIdx.has(t.to)) this.inIdx.set(t.to, []);
      this.inIdx.get(t.to).push(i);
    }

    this.badStates = new Set(problem.badStates);
    this.start = problem.initialState;
    this.goal = problem.goalState;

    this.computeHeuristicScale();
  }

  computeHeuristicScale() {
    let m = Infinity;
    for (const t of this.transitions) {
      if (!this.statesById.has(t.from) || !this.statesById.has(t.to)) continue;
      if (this.badStates.has(t.from) || this.badStates.has(t.to)) continue;
      const d = LPAStarPlanner.euclideanDistance(
        this.statesById.get(t.from).embedding,
        this.statesById.get(t.to).embedding
      );
      const effCost = this.computeEdgeCost(t);
      if (d > 1e-9 && effCost >= 0 && effCost < Infinity) {
        m = Math.min(m, effCost / d);
      }
    }
    this.heuristicScale = (m === Infinity) ? 0.0 : m;
  }

  refreshHeuristicScaleForTransition(t) {
    if (!this.statesById.has(t.from) || !this.statesById.has(t.to)) return;
    if (this.badStates.has(t.from) || this.badStates.has(t.to)) return;
    const d = LPAStarPlanner.euclideanDistance(
      this.statesById.get(t.from).embedding,
      this.statesById.get(t.to).embedding
    );
    const effCost = this.computeEdgeCost(t);
    if (d > 1e-9 && effCost >= 0 && effCost < Infinity) {
      const ratio = effCost / d;
      this.heuristicScale = (this.heuristicScale <= 0) ? ratio : Math.min(this.heuristicScale, ratio);
    }
  }

  getG(s) {
    return this.g.has(s) ? this.g.get(s) : Infinity;
  }

  getRhs(s) {
    return this.rhs.has(s) ? this.rhs.get(s) : Infinity;
  }

  heuristic(s) {
    if (this.heuristicScale <= 0) return 0.0;
    if (!this.statesById.has(s) || !this.statesById.has(this.goal)) return 0.0;
    const d = LPAStarPlanner.euclideanDistance(
      this.statesById.get(s).embedding,
      this.statesById.get(this.goal).embedding
    );
    return this.heuristicScale * d;
  }

  calculateKey(s) {
    const gv = this.getG(s);
    const rv = this.getRhs(s);
    const m = Math.min(gv, rv);
    const k1 = (m >= Infinity) ? Infinity : m + this.heuristic(s);
    return [k1, m];
  }

  static compareKeys(k1, k2) {
    if (Math.abs(k1[0] - k2[0]) > 1e-9) {
      return k1[0] - k2[0];
    }
    return k1[1] - k2[1];
  }

  insertIntoOpenList(s, key) {
    this.removeFromOpenList(s);
    this.openKeyOf.set(s, key);
    // Maintain sorted order
    const entry = { k1: key[0], k2: key[1], id: s };
    let low = 0, high = this.openList.length;
    while (low < high) {
      const mid = (low + high) >>> 1;
      if (LPAStarPlanner.compareKeys([this.openList[mid].k1, this.openList[mid].k2], key) < 0) {
        low = mid + 1;
      } else {
        high = mid;
      }
    }
    this.openList.splice(low, 0, entry);
  }

  removeFromOpenList(s) {
    if (!this.openKeyOf.has(s)) return;
    this.openKeyOf.delete(s);
    const idx = this.openList.findIndex(e => e.id === s);
    if (idx !== -1) {
      this.openList.splice(idx, 1);
    }
  }

  initializeSearch() {
    this.g.clear();
    this.rhs.clear();
    this.openList = [];
    this.openKeyOf.clear();
    this.exploredThisCall = 0;

    for (const [id] of this.statesById) {
      this.g.set(id, Infinity);
    }
    this.rhs.set(this.start, 0.0);
    this.g.set(this.start, Infinity);

    if (this.badStates.has(this.start)) {
      this.rhs.set(this.start, Infinity);
    } else {
      this.insertIntoOpenList(this.start, this.calculateKey(this.start));
    }
  }

  updateVertex(u) {
    if (this.badStates.has(u)) {
      this.g.set(u, Infinity);
      this.rhs.set(u, Infinity);
      this.removeFromOpenList(u);
      return;
    }
    if (u !== this.start) {
      let best = Infinity;
      const ins = this.inIdx.get(u) || [];
      for (const idx of ins) {
        const t = this.transitions[idx];
        if (!t.available) continue;
        if (this.badStates.has(t.from)) continue;
        const gp = this.getG(t.from);
        if (gp < Infinity) {
          const effCost = this.computeEdgeCost(t);
          if (effCost < Infinity) {
            best = Math.min(best, gp + effCost);
          }
        }
      }
      this.rhs.set(u, best);
    }
    this.removeFromOpenList(u);
    if (Math.abs(this.getG(u) - this.getRhs(u)) > 1e-9) {
      this.insertIntoOpenList(u, this.calculateKey(u));
    }
  }

  computeShortestPath() {
    const EPS = 1e-9;
    while (this.openList.length > 0) {
      const top = this.openList[0];
      const topKey = [top.k1, top.k2];
      const goalKey = this.calculateKey(this.goal);
      const goalConsistent = Math.abs(this.getRhs(this.goal) - this.getG(this.goal)) <= EPS;

      if (!(LPAStarPlanner.compareKeys(topKey, goalKey) < 0) && goalConsistent) {
        break;
      }

      const u = top.id;
      this.openList.shift();
      this.openKeyOf.delete(u);
      this.exploredThisCall++;

      const gu = this.getG(u);
      const ru = this.getRhs(u);

      if (gu > ru + EPS) {
        this.g.set(u, ru);
        const outs = this.outIdx.get(u) || [];
        for (const idx of outs) {
          const t = this.transitions[idx];
          if (!t.available) continue;
          this.updateVertex(t.to);
        }
      } else {
        this.g.set(u, Infinity);
        this.updateVertex(u);
        const outs = this.outIdx.get(u) || [];
        for (const idx of outs) {
          const t = this.transitions[idx];
          if (!t.available) continue;
          this.updateVertex(t.to);
        }
      }
    }
  }

  plan(problem) {
    this.loadProblem(problem);
    this.initializeSearch();
    const t0 = performance.now();
    this.exploredThisCall = 0;
    this.computeShortestPath();
    const t1 = performance.now();
    this.lastOperationMicros = (t1 - t0) * 1000.0;
    return this.reconstructResult();
  }

  setTransitionAvailability(transitionId, available) {
    const t0 = performance.now();
    this.exploredThisCall = 0;
    const idx = this.transitionIndexById.get(transitionId);
    if (idx !== undefined) {
      const t = this.transitions[idx];
      if (t.available !== available) {
        t.available = available;
        this.updateVertex(t.to);
        this.computeShortestPath();
      }
    }
    const t1 = performance.now();
    this.lastOperationMicros = (t1 - t0) * 1000.0;
    return this.reconstructResult();
  }

  addTransition(t) {
    const t0 = performance.now();
    this.exploredThisCall = 0;
    this.transitions.push({ ...t });
    const idx = this.transitions.length - 1;
    this.transitionIndexById.set(t.id, idx);
    if (!this.outIdx.has(t.from)) this.outIdx.set(t.from, []);
    this.outIdx.get(t.from).push(idx);
    if (!this.inIdx.has(t.to)) this.inIdx.set(t.to, []);
    this.inIdx.get(t.to).push(idx);
    this.refreshHeuristicScaleForTransition(t);
    this.updateVertex(t.to);
    this.computeShortestPath();
    const t1 = performance.now();
    this.lastOperationMicros = (t1 - t0) * 1000.0;
    return this.reconstructResult();
  }

  updateGoal(newGoal) {
    const t0 = performance.now();
    this.exploredThisCall = 0;
    this.goal = newGoal;
    if (!this.g.has(this.goal)) this.g.set(this.goal, Infinity);
    if (!this.rhs.has(this.goal)) this.rhs.set(this.goal, Infinity);

    const queued = this.openList.map(e => e.id);
    this.openList = [];
    this.openKeyOf.clear();
    for (const v of queued) {
      this.insertIntoOpenList(v, this.calculateKey(v));
    }
    this.updateVertex(this.goal);
    this.computeShortestPath();
    const t1 = performance.now();
    this.lastOperationMicros = (t1 - t0) * 1000.0;
    return this.reconstructResult();
  }

  updateBadStates(newBadStates) {
    const t0 = performance.now();
    this.exploredThisCall = 0;
    this.badStates = new Set(newBadStates);
    this.computeHeuristicScale();
    for (const [id] of this.statesById) {
      this.updateVertex(id);
    }
    this.computeShortestPath();
    const t1 = performance.now();
    this.lastOperationMicros = (t1 - t0) * 1000.0;
    return this.reconstructResult();
  }

  reconstructResult() {
    const result = {
      success: false,
      statePath: [],
      transitionPath: [],
      totalCost: 0.0,
      safetyScore: 0.0,
      reliability: 1.0,
      score: 0.0,
      explored: this.exploredThisCall,
      timeMicros: this.lastOperationMicros,
      memoryBytes: this.getEstimatedMemoryBytes()
    };

    if (this.badStates.has(this.start) || this.badStates.has(this.goal)) {
      return result;
    }
    if (this.getG(this.goal) >= Infinity) {
      return result;
    }

    const statePath = [];
    const transitionPath = [];
    let cur = this.goal;
    statePath.push(cur);
    const guard = new Set([cur]);
    const EPS = 1e-9;
    let ok = true;

    while (cur !== this.start) {
      let bestVal = Infinity;
      let bestPred = 0;
      let bestIdx = -1;

      const ins = this.inIdx.get(cur) || [];
      for (const idx of ins) {
        const t = this.transitions[idx];
        if (!t.available) continue;
        if (this.badStates.has(t.from)) continue;
        const gp = this.getG(t.from);
        if (gp >= Infinity) continue;
        const effCost = this.computeEdgeCost(t);
        if (effCost >= Infinity) continue;
        const val = gp + effCost;
        if (val < bestVal - EPS) {
          bestVal = val;
          bestPred = t.from;
          bestIdx = idx;
        }
      }

      if (bestIdx === -1) {
        ok = false;
        break;
      }

      transitionPath.push(this.transitions[bestIdx].id);
      cur = bestPred;
      if (guard.has(cur)) {
        ok = false;
        break;
      }
      guard.add(cur);
      statePath.push(cur);

      if (statePath.length > this.statesById.size + 1) {
        ok = false;
        break;
      }
    }

    if (!ok) return result;

    statePath.reverse();
    transitionPath.reverse();

    // Final safety check
    for (const s of statePath) {
      if (this.badStates.has(s)) {
        result.success = false;
        return result;
      }
    }

    let totalCost = 0.0;
    let totalReliability = 1.0;
    for (const tid of transitionPath) {
      const t = this.transitions[this.transitionIndexById.get(tid)];
      totalCost += t.cost;
      totalReliability *= t.reliability;
    }

    const safetyScore = this.computeSafetyScore(statePath);
    const D = isFinite(safetyScore) ? safetyScore : 0.0;
    const score = this.alpha * 1.0 - this.beta * totalCost + this.gamma * D + this.delta * totalReliability;

    result.success = true;
    result.statePath = statePath;
    result.transitionPath = transitionPath;
    result.totalCost = totalCost;
    result.safetyScore = safetyScore;
    result.reliability = totalReliability;
    result.score = score;
    return result;
  }

  computeSafetyScore(statePath) {
    if (this.badStates.size === 0) return Infinity;
    let minOverPath = Infinity;
    for (const s of statePath) {
      const d = this.getDistanceToNearestBadState(s);
      if (d < minOverPath) minOverPath = d;
    }
    return minOverPath;
  }

  getEstimatedMemoryBytes() {
    let bytes = 128; // base planner instance
    bytes += this.statesById.size * 96;
    bytes += this.transitions.length * 64;
    bytes += this.badStates.size * 32;
    bytes += this.g.size * 48;
    bytes += this.rhs.size * 48;
    bytes += this.openList.length * 64;
    bytes += this.openKeyOf.size * 48;
    return bytes;
  }
}
