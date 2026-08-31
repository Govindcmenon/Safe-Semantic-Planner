/**
 * Safe Semantic Planner - Frontend Visualizer & Interactive App
 */

// ==========================================================================
// Preset Test Scenarios
// ==========================================================================

const PRESETS = {
  case1: {
    id: "case1",
    name: "Test Case 1: Basic Reachability",
    desc: "Straightforward linear path from Start to Goal with no obstacles.",
    states: [
      { id: 1, name: "S", embedding: [0, 0] },
      { id: 2, name: "A", embedding: [1, 0] },
      { id: 3, name: "B", embedding: [2, 0] },
      { id: 4, name: "G", embedding: [3, 0] }
    ],
    transitions: [
      { id: 101, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 0.99, available: true },
      { id: 102, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 0.99, available: true },
      { id: 103, from: 3, to: 4, cost: 1.0, safety: 1.0, reliability: 0.99, available: true }
    ],
    initialState: 1,
    goalState: 4,
    badStates: []
  },
  case2: {
    id: "case2",
    name: "Test Case 2: Bad State Avoidance",
    desc: "Planner must bypass bad state X and route safely through alternate nodes.",
    states: [
      { id: 1, name: "S", embedding: [0, 0] },
      { id: 2, name: "A", embedding: [1, 1] },
      { id: 3, name: "X", embedding: [2, 1] },
      { id: 4, name: "C", embedding: [1, -1] },
      { id: 5, name: "D", embedding: [2, -1] },
      { id: 6, name: "G", embedding: [3, 0] }
    ],
    transitions: [
      { id: 201, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 0.95, available: true },
      { id: 202, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 0.95, available: true },
      { id: 203, from: 3, to: 6, cost: 1.0, safety: 1.0, reliability: 0.95, available: true },
      { id: 204, from: 1, to: 4, cost: 1.0, safety: 1.0, reliability: 0.95, available: true },
      { id: 205, from: 4, to: 5, cost: 1.0, safety: 1.0, reliability: 0.95, available: true },
      { id: 206, from: 5, to: 6, cost: 1.0, safety: 1.0, reliability: 0.95, available: true }
    ],
    initialState: 1,
    goalState: 6,
    badStates: [3]
  },
  case3: {
    id: "case3",
    name: "Test Case 3: Safety Margin Optimization",
    desc: "Path 1 is cheap (cost 3) near hazard Y vs Path 2 which is safer (cost 9, clearance 1.50).",
    states: [
      { id: 1, name: "S", embedding: [0, 0] },
      { id: 2, name: "A1", embedding: [1, 0.2] },
      { id: 3, name: "A2", embedding: [2, 0.2] },
      { id: 4, name: "G", embedding: [3, 0] },
      { id: 5, name: "B1", embedding: [1, 2.8] },
      { id: 6, name: "B2", embedding: [2, 2.8] },
      { id: 7, name: "Y", embedding: [1.5, 0.0] }
    ],
    transitions: [
      { id: 301, from: 1, to: 2, cost: 1.0, safety: 0.6, reliability: 0.9, available: true },
      { id: 302, from: 2, to: 3, cost: 1.0, safety: 0.6, reliability: 0.9, available: true },
      { id: 303, from: 3, to: 4, cost: 1.0, safety: 0.6, reliability: 0.9, available: true },
      { id: 304, from: 1, to: 5, cost: 3.0, safety: 0.95, reliability: 0.9, available: true },
      { id: 305, from: 5, to: 6, cost: 3.0, safety: 0.95, reliability: 0.9, available: true },
      { id: 306, from: 6, to: 4, cost: 3.0, safety: 0.95, reliability: 0.9, available: true }
    ],
    initialState: 1,
    goalState: 4,
    badStates: [7]
  },
  case4: {
    id: "case4",
    name: "Test Case 4: Dynamic Transition Failure",
    desc: "Initial optimal route (A,G) gets blocked in real-time, triggering fast incremental replanning.",
    states: [
      { id: 1, name: "S", embedding: [0, 0] },
      { id: 2, name: "A", embedding: [1, 0] },
      { id: 3, name: "G", embedding: [2, 0] },
      { id: 4, name: "C", embedding: [1, -1] },
      { id: 5, name: "D", embedding: [2, -1] }
    ],
    transitions: [
      { id: 401, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 0.9, available: true },
      { id: 402, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 0.9, available: false }, // toggled in demo
      { id: 403, from: 1, to: 4, cost: 2.0, safety: 1.0, reliability: 0.9, available: true },
      { id: 404, from: 4, to: 5, cost: 2.0, safety: 1.0, reliability: 0.9, available: true },
      { id: 405, from: 5, to: 3, cost: 2.0, safety: 1.0, reliability: 0.9, available: true }
    ],
    initialState: 1,
    goalState: 3,
    badStates: []
  },
  case5: {
    id: "case5",
    name: "Test Case 5: Incremental Goal Update",
    desc: "Goal shifts to new position; g-values are reused with warm-started heuristic updates.",
    states: [
      { id: 1, name: "S", embedding: [0, 0] },
      { id: 2, name: "A", embedding: [1, 0] },
      { id: 3, name: "G_old", embedding: [2, 0] },
      { id: 4, name: "G_new", embedding: [3, 0] },
      { id: 5, name: "H", embedding: [2, 1.5] }
    ],
    transitions: [
      { id: 501, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 0.9, available: true },
      { id: 502, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 0.9, available: true },
      { id: 503, from: 2, to: 5, cost: 1.5, safety: 1.0, reliability: 0.9, available: true },
      { id: 504, from: 5, to: 4, cost: 1.0, safety: 1.0, reliability: 0.9, available: true }
    ],
    initialState: 1,
    goalState: 4, // switched to G_new
    badStates: []
  },
  case6: {
    id: "case6",
    name: "Test Case 6: Transition Addition",
    desc: "A shortcut edge S->G appears dynamically, immediately optimizing path cost.",
    states: [
      { id: 1, name: "S", embedding: [0, 0] },
      { id: 2, name: "A", embedding: [1, 0] },
      { id: 3, name: "B", embedding: [2, 0] },
      { id: 4, name: "G", embedding: [3, 0] }
    ],
    transitions: [
      { id: 601, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 0.9, available: true },
      { id: 602, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 0.9, available: true },
      { id: 603, from: 3, to: 4, cost: 1.0, safety: 1.0, reliability: 0.9, available: true },
      { id: 604, from: 1, to: 4, cost: 1.5, safety: 1.0, reliability: 0.9, available: true } // Shortcut
    ],
    initialState: 1,
    goalState: 4,
    badStates: []
  }
};

// ==========================================================================
// Application State & Controller
// ==========================================================================

class PlannerApp {
  constructor() {
    this.canvas = document.getElementById("plannerCanvas");
    this.ctx = this.canvas.getContext("2d");
    this.planner = new LPAStarPlanner({ kGeom: 2.0, kTrans: 2.0 });

    this.currentProblem = JSON.parse(JSON.stringify(PRESETS.case3));
    this.currentResult = null;

    // Viewport & Coordinate Transforms
    this.panX = 0;
    this.panY = 0;
    this.scale = 120; // pixels per Cartesian unit
    this.isDragging = false;
    this.draggedStateId = null;
    this.lastMousePos = { x: 0, y: 0 };
    this.hoveredStateId = null;
    this.hoveredTransitionId = null;

    // Particle Animation on Path
    this.particles = [];
    this.particleTimer = 0;

    this.init();
  }

  init() {
    this.bindEvents();
    this.resizeCanvas();
    window.addEventListener("resize", () => this.resizeCanvas());

    this.renderPresetButtons();
    this.loadScenario("case3");

    requestAnimationFrame(() => this.animationLoop());
  }

  resizeCanvas() {
    const rect = this.canvas.parentElement.getBoundingClientRect();
    this.canvas.width = rect.width * window.devicePixelRatio;
    this.canvas.height = rect.height * window.devicePixelRatio;
    this.ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
    this.canvasWidth = rect.width;
    this.canvasHeight = rect.height;
    this.centerView();
    this.redraw();
  }

  centerView() {
    if (!this.currentProblem || !this.currentProblem.states.length) return;
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const s of this.currentProblem.states) {
      minX = Math.min(minX, s.embedding[0]);
      maxX = Math.max(maxX, s.embedding[0]);
      minY = Math.min(minY, s.embedding[1]);
      maxY = Math.max(maxY, s.embedding[1]);
    }
    const midX = (minX + maxX) / 2;
    const midY = (minY + maxY) / 2;
    this.panX = this.canvasWidth / 2 - midX * this.scale;
    this.panY = this.canvasHeight / 2 + midY * this.scale;
  }

  worldToScreen(x, y) {
    return {
      x: this.panX + x * this.scale,
      y: this.panY - y * this.scale // Cartesian Y is up
    };
  }

  screenToWorld(px, py) {
    return {
      x: (px - this.panX) / this.scale,
      y: (this.panY - py) / this.scale
    };
  }

  loadScenario(presetKey) {
    const preset = PRESETS[presetKey];
    if (!preset) return;
    this.currentProblem = JSON.parse(JSON.stringify(preset));

    document.querySelectorAll(".scenario-btn").forEach(btn => {
      btn.classList.toggle("active", btn.dataset.key === presetKey);
    });

    this.centerView();
    this.solve();
  }

  solve() {
    this.currentResult = this.planner.plan(this.currentProblem);
    this.updateUI();
    this.redraw();
  }

  toggleSafetyMode(enabled) {
    if (enabled) {
      this.planner.setSafetyWeights(2.0, 2.0);
      document.getElementById("kGeomVal").innerText = "2.0";
      document.getElementById("kTransVal").innerText = "2.0";
      document.getElementById("sliderKGeom").value = 2.0;
      document.getElementById("sliderKTrans").value = 2.0;
    } else {
      this.planner.setSafetyWeights(0.0, 0.0);
      document.getElementById("kGeomVal").innerText = "0.0";
      document.getElementById("kTransVal").innerText = "0.0";
      document.getElementById("sliderKGeom").value = 0.0;
      document.getElementById("sliderKTrans").value = 0.0;
    }
    this.solve();
  }

  // ==========================================================================
  // UI & Metrics Sync
  // ==========================================================================

  renderPresetButtons() {
    const container = document.getElementById("scenarioList");
    if (!container) return;
    container.innerHTML = "";

    for (const [key, preset] of Object.entries(PRESETS)) {
      const btn = document.createElement("button");
      btn.className = `scenario-btn ${key === "case3" ? "active" : ""}`;
      btn.dataset.key = key;
      btn.innerHTML = `
        <div class="scenario-title">
          <span>${preset.name}</span>
        </div>
        <div class="scenario-desc">${preset.desc}</div>
      `;
      btn.addEventListener("click", () => this.loadScenario(key));
      container.appendChild(btn);
    }
  }

  updateUI() {
    const r = this.currentResult;
    if (!r) return;

    // Status Badge
    const statusBadge = document.getElementById("resultStatus");
    if (statusBadge) {
      statusBadge.innerText = r.success ? "SUCCESS" : "NO PATH FOUND";
      statusBadge.className = `status-badge ${r.success ? "consistent" : "bad"}`;
    }

    // Metric Cards
    document.getElementById("valCost").innerText = r.totalCost.toFixed(2);
    document.getElementById("valSafety").innerText = isFinite(r.safetyScore) ? r.safetyScore.toFixed(3) : "N/A (No Hazards)";
    document.getElementById("valReliability").innerText = (r.reliability * 100).toFixed(1) + "%";
    document.getElementById("valScore").innerText = r.score.toFixed(2);
    document.getElementById("valExplored").innerText = r.explored;
    document.getElementById("valTime").innerText = r.timeMicros.toFixed(1) + " μs";
    document.getElementById("valMemory").innerText = (r.memoryBytes / 1024).toFixed(2) + " KB";

    // Path sequence display
    const pathContainer = document.getElementById("pathSequence");
    if (pathContainer) {
      pathContainer.innerHTML = "";
      if (r.success && r.statePath.length) {
        r.statePath.forEach((sId, idx) => {
          const s = this.currentProblem.states.find(st => st.id === sId);
          const name = s ? s.name : sId;
          const isStart = sId === this.currentProblem.initialState;
          const isGoal = sId === this.currentProblem.goalState;
          const isBad = this.currentProblem.badStates.includes(sId);

          const pill = document.createElement("span");
          pill.className = `node-pill ${isStart ? 'start' : isGoal ? 'goal' : isBad ? 'bad' : ''}`;
          pill.innerText = name;
          pathContainer.appendChild(pill);

          if (idx < r.statePath.length - 1) {
            const arrow = document.createElement("span");
            arrow.className = "arrow-separator";
            arrow.innerText = "→";
            pathContainer.appendChild(arrow);
          }
        });
      } else {
        pathContainer.innerHTML = `<span style="color: var(--text-muted); font-size: 0.85rem;">No valid path</span>`;
      }
    }

    // LPA* Table
    this.renderLPATable();
  }

  renderLPATable() {
    const tbody = document.getElementById("lpaTableBody");
    if (!tbody) return;
    tbody.innerHTML = "";

    for (const s of this.currentProblem.states) {
      const g = this.planner.getG(s.id);
      const rhs = this.planner.getRhs(s.id);
      const isBad = this.currentProblem.badStates.includes(s.id);

      let status = "Consistent";
      let statusClass = "consistent";
      if (isBad) {
        status = "Bad State";
        statusClass = "bad";
      } else if (Math.abs(g - rhs) <= 1e-9) {
        status = "Consistent";
        statusClass = "consistent";
      } else if (g > rhs) {
        status = "Overconsistent";
        statusClass = "overconsistent";
      } else {
        status = "Underconsistent";
        statusClass = "underconsistent";
      }

      const row = document.createElement("tr");
      row.innerHTML = `
        <td><strong>${s.name}</strong> (${s.id})</td>
        <td>(${s.embedding[0].toFixed(1)}, ${s.embedding[1].toFixed(1)})</td>
        <td>${g === Infinity ? '∞' : g.toFixed(2)}</td>
        <td>${rhs === Infinity ? '∞' : rhs.toFixed(2)}</td>
        <td><span class="status-badge ${statusClass}">${status}</span></td>
      `;
      tbody.appendChild(row);
    }
  }

  // ==========================================================================
  // Canvas Rendering Engine
  // ==========================================================================

  redraw() {
    const ctx = this.ctx;
    ctx.clearRect(0, 0, this.canvasWidth, this.canvasHeight);

    this.drawGrid();
    this.drawSafetyFields();
    this.drawTransitions();
    this.drawPathHighlights();
    this.drawStates();
  }

  drawGrid() {
    const ctx = this.ctx;
    const step = this.scale;
    ctx.strokeStyle = "rgba(255, 255, 255, 0.04)";
    ctx.lineWidth = 1;

    const startX = this.panX % step;
    for (let x = startX; x < this.canvasWidth; x += step) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, this.canvasHeight);
      ctx.stroke();
    }

    const startY = this.panY % step;
    for (let y = startY; y < this.canvasHeight; y += step) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(this.canvasWidth, y);
      ctx.stroke();
    }

    // Axes
    ctx.strokeStyle = "rgba(255, 255, 255, 0.15)";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(0, this.panY);
    ctx.lineTo(this.canvasWidth, this.panY);
    ctx.moveTo(this.panX, 0);
    ctx.lineTo(this.panX, this.canvasHeight);
    ctx.stroke();
  }

  drawSafetyFields() {
    const ctx = this.ctx;
    if (!this.currentProblem.badStates.length) return;

    for (const bId of this.currentProblem.badStates) {
      const bState = this.currentProblem.states.find(s => s.id === bId);
      if (!bState) continue;
      const screenPos = this.worldToScreen(bState.embedding[0], bState.embedding[1]);

      // Radial safety clearance field
      const radiusMax = 2.0 * this.scale;
      const grad = ctx.createRadialGradient(screenPos.x, screenPos.y, 10, screenPos.x, screenPos.y, radiusMax);
      grad.addColorStop(0, "rgba(255, 51, 102, 0.35)");
      grad.addColorStop(0.4, "rgba(255, 51, 102, 0.12)");
      grad.addColorStop(1, "rgba(255, 51, 102, 0.0)");

      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(screenPos.x, screenPos.y, radiusMax, 0, Math.PI * 2);
      ctx.fill();

      // Proximity dashed boundary
      ctx.strokeStyle = "rgba(255, 51, 102, 0.4)";
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.arc(screenPos.x, screenPos.y, 1.2 * this.scale, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  drawTransitions() {
    const ctx = this.ctx;
    for (const t of this.currentProblem.transitions) {
      const fromState = this.currentProblem.states.find(s => s.id === t.from);
      const toState = this.currentProblem.states.find(s => s.id === t.to);
      if (!fromState || !toState) continue;

      const p1 = this.worldToScreen(fromState.embedding[0], fromState.embedding[1]);
      const p2 = this.worldToScreen(toState.embedding[0], toState.embedding[1]);

      const dx = p2.x - p1.x;
      const dy = p2.y - p1.y;
      const dist = Math.hypot(dx, dy);
      if (dist < 1e-3) continue;

      const angle = Math.atan2(dy, dx);
      const nodeR = 18;
      const startX = p1.x + Math.cos(angle) * nodeR;
      const startY = p1.y + Math.sin(angle) * nodeR;
      const endX = p2.x - Math.cos(angle) * (nodeR + 6);
      const endY = p2.y - Math.sin(angle) * (nodeR + 6);

      ctx.beginPath();
      ctx.moveTo(startX, startY);
      ctx.lineTo(endX, endY);

      if (!t.available) {
        ctx.strokeStyle = "rgba(255, 51, 102, 0.5)";
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 5]);
        ctx.stroke();
        ctx.setLineDash([]);
      } else {
        ctx.strokeStyle = "rgba(255, 255, 255, 0.25)";
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      // Arrow head
      ctx.save();
      ctx.translate(endX, endY);
      ctx.rotate(angle);
      ctx.fillStyle = t.available ? "rgba(255, 255, 255, 0.5)" : "rgba(255, 51, 102, 0.7)";
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(-10, -5);
      ctx.lineTo(-10, 5);
      ctx.closePath();
      ctx.fill();
      ctx.restore();

      // Transition Badge label (Cost / Safety)
      const midX = (p1.x + p2.x) / 2;
      const midY = (p1.y + p2.y) / 2;
      const offset = 14;
      const labelX = midX - Math.sin(angle) * offset;
      const labelY = midY + Math.cos(angle) * offset;

      ctx.font = "10px JetBrains Mono, monospace";
      ctx.fillStyle = t.available ? "rgba(148, 163, 184, 0.9)" : "rgba(255, 51, 102, 0.9)";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      const badgeText = `c:${t.cost} s:${t.safety}`;
      ctx.fillText(badgeText, labelX, labelY);
    }
  }

  drawPathHighlights() {
    const ctx = this.ctx;
    const r = this.currentResult;
    if (!r || !r.success || r.statePath.length < 2) return;

    ctx.save();
    ctx.strokeStyle = "#00ff88";
    ctx.lineWidth = 4;
    ctx.shadowColor = "rgba(0, 255, 136, 0.8)";
    ctx.shadowBlur = 16;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";

    ctx.beginPath();
    for (let i = 0; i < r.statePath.length; i++) {
      const s = this.currentProblem.states.find(st => st.id === r.statePath[i]);
      if (!s) continue;
      const p = this.worldToScreen(s.embedding[0], s.embedding[1]);
      if (i === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
    ctx.restore();
  }

  drawStates() {
    const ctx = this.ctx;
    for (const s of this.currentProblem.states) {
      const p = this.worldToScreen(s.embedding[0], s.embedding[1]);
      const isStart = s.id === this.currentProblem.initialState;
      const isGoal = s.id === this.currentProblem.goalState;
      const isBad = this.currentProblem.badStates.includes(s.id);
      const isHovered = s.id === this.hoveredStateId;

      ctx.save();
      const nodeR = isHovered ? 22 : 18;

      // Glow Halo
      if (isStart) {
        ctx.shadowColor = "rgba(0, 240, 255, 0.9)";
        ctx.shadowBlur = 20;
        ctx.fillStyle = "#00f0ff";
      } else if (isGoal) {
        ctx.shadowColor = "rgba(255, 190, 11, 0.9)";
        ctx.shadowBlur = 20;
        ctx.fillStyle = "#ffbe0b";
      } else if (isBad) {
        ctx.shadowColor = "rgba(255, 51, 102, 0.9)";
        ctx.shadowBlur = 20;
        ctx.fillStyle = "#ff3366";
      } else {
        ctx.shadowColor = "rgba(99, 102, 241, 0.6)";
        ctx.shadowBlur = 12;
        ctx.fillStyle = "#6366f1";
      }

      ctx.beginPath();
      ctx.arc(p.x, p.y, nodeR, 0, Math.PI * 2);
      ctx.fill();

      // Inner Core
      ctx.shadowBlur = 0;
      ctx.fillStyle = "#0c121e";
      ctx.beginPath();
      ctx.arc(p.x, p.y, nodeR - 3, 0, Math.PI * 2);
      ctx.fill();

      // State Label
      ctx.font = "bold 12px Outfit, sans-serif";
      ctx.fillStyle = "#fff";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(s.name, p.x, p.y);

      // Coordinate text below
      ctx.font = "10px JetBrains Mono, monospace";
      ctx.fillStyle = "rgba(148, 163, 184, 0.8)";
      ctx.fillText(`(${s.embedding[0].toFixed(1)}, ${s.embedding[1].toFixed(1)})`, p.x, p.y + nodeR + 12);

      ctx.restore();
    }
  }

  animationLoop() {
    this.particleTimer++;
    if (this.currentResult && this.currentResult.success && this.currentResult.statePath.length >= 2) {
      if (this.particleTimer % 10 === 0) {
        this.particles.push({
          segmentIdx: 0,
          progress: 0.0,
          speed: 0.02
        });
      }
    }

    this.updateParticles();
    this.redraw();
    this.drawParticles();

    requestAnimationFrame(() => this.animationLoop());
  }

  updateParticles() {
    const path = this.currentResult ? this.currentResult.statePath : [];
    if (path.length < 2) {
      this.particles = [];
      return;
    }

    for (let i = this.particles.length - 1; i >= 0; i--) {
      const pt = this.particles[i];
      pt.progress += pt.speed;
      if (pt.progress >= 1.0) {
        pt.progress = 0;
        pt.segmentIdx++;
        if (pt.segmentIdx >= path.length - 1) {
          this.particles.splice(i, 1);
        }
      }
    }
  }

  drawParticles() {
    const ctx = this.ctx;
    const path = this.currentResult ? this.currentResult.statePath : [];
    if (path.length < 2) return;

    ctx.save();
    for (const pt of this.particles) {
      if (pt.segmentIdx >= path.length - 1) continue;
      const s1 = this.currentProblem.states.find(s => s.id === path[pt.segmentIdx]);
      const s2 = this.currentProblem.states.find(s => s.id === path[pt.segmentIdx + 1]);
      if (!s1 || !s2) continue;

      const p1 = this.worldToScreen(s1.embedding[0], s1.embedding[1]);
      const p2 = this.worldToScreen(s2.embedding[0], s2.embedding[1]);

      const px = p1.x + (p2.x - p1.x) * pt.progress;
      const py = p1.y + (p2.y - p1.y) * pt.progress;

      ctx.fillStyle = "#ffffff";
      ctx.shadowColor = "#00ff88";
      ctx.shadowBlur = 12;
      ctx.beginPath();
      ctx.arc(px, py, 4, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();
  }

  // ==========================================================================
  // Mouse & Touch Interaction
  // ==========================================================================

  bindEvents() {
    const canvas = this.canvas;

    canvas.addEventListener("mousedown", e => {
      const rect = canvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;
      this.lastMousePos = { x: mx, y: my };

      const hitState = this.hitTestState(mx, my);
      if (hitState) {
        this.draggedStateId = hitState.id;
      } else {
        this.isDragging = true;
      }
    });

    window.addEventListener("mousemove", e => {
      const rect = canvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;
      const dx = mx - this.lastMousePos.x;
      const dy = my - this.lastMousePos.y;
      this.lastMousePos = { x: mx, y: my };

      if (this.draggedStateId) {
        const state = this.currentProblem.states.find(s => s.id === this.draggedStateId);
        if (state) {
          const world = this.screenToWorld(mx, my);
          state.embedding[0] = Math.round(world.x * 10) / 10;
          state.embedding[1] = Math.round(world.y * 10) / 10;
          this.solve();
        }
      } else if (this.isDragging) {
        this.panX += dx;
        this.panY += dy;
        this.redraw();
      } else {
        const hovered = this.hitTestState(mx, my);
        this.hoveredStateId = hovered ? hovered.id : null;
      }
    });

    window.addEventListener("mouseup", () => {
      this.isDragging = false;
      this.draggedStateId = null;
    });

    canvas.addEventListener("wheel", e => {
      e.preventDefault();
      const zoomFactor = e.deltaY < 0 ? 1.1 : 0.9;
      const rect = canvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;

      const world = this.screenToWorld(mx, my);
      this.scale = Math.min(Math.max(40, this.scale * zoomFactor), 300);

      this.panX = mx - world.x * this.scale;
      this.panY = my + world.y * this.scale;
      this.redraw();
    });

    // Control buttons
    document.getElementById("btnZoomIn")?.addEventListener("click", () => {
      this.scale = Math.min(300, this.scale * 1.2);
      this.centerView();
      this.redraw();
    });
    document.getElementById("btnZoomOut")?.addEventListener("click", () => {
      this.scale = Math.max(40, this.scale * 0.8);
      this.centerView();
      this.redraw();
    });
    document.getElementById("btnResetView")?.addEventListener("click", () => {
      this.scale = 120;
      this.centerView();
      this.redraw();
    });

    // Toggle Transition Button (For Test Case 4 demonstration)
    document.getElementById("btnToggleTransition")?.addEventListener("click", () => {
      if (!this.currentProblem.transitions.length) return;
      // Toggle first available or unavailable transition
      const t = this.currentProblem.transitions.find(tr => tr.id === 402 || tr.id === 301 || tr.id === 201) || this.currentProblem.transitions[0];
      if (t) {
        t.available = !t.available;
        this.solve();
      }
    });

    // Safety Weight Sliders
    const sliderKGeom = document.getElementById("sliderKGeom");
    const sliderKTrans = document.getElementById("sliderKTrans");
    const kGeomVal = document.getElementById("kGeomVal");
    const kTransVal = document.getElementById("kTransVal");

    sliderKGeom?.addEventListener("input", e => {
      const v = parseFloat(e.target.value);
      kGeomVal.innerText = v.toFixed(1);
      this.planner.setSafetyWeights(v, parseFloat(sliderKTrans.value));
      this.solve();
    });

    sliderKTrans?.addEventListener("input", e => {
      const v = parseFloat(e.target.value);
      kTransVal.innerText = v.toFixed(1);
      this.planner.setSafetyWeights(parseFloat(sliderKGeom.value), v);
      this.solve();
    });

    // Safety Mode Toggle Switch
    const switchSafety = document.getElementById("switchSafetyMode");
    switchSafety?.addEventListener("change", e => {
      this.toggleSafetyMode(e.target.checked);
    });
  }

  hitTestState(mx, my) {
    for (const s of this.currentProblem.states) {
      const p = this.worldToScreen(s.embedding[0], s.embedding[1]);
      const dist = Math.hypot(mx - p.x, my - p.y);
      if (dist <= 20) return s;
    }
    return null;
  }
}

// Instantiate application on page load
window.addEventListener("DOMContentLoaded", () => {
  window.app = new PlannerApp();
});
