/**
 * Solver scaling benchmark. Builds three mechanism topologies at growing sizes,
 * perturbs the starting pose deterministically, and times solve()-to-convergence.
 *
 * Topologies:
 *   - open-chain:   N bars pinned end-to-end, one ground. Tree (0 closed loops).
 *   - closed-chain: same chain, both ends grounded (within reach). 1 closed loop.
 *   - ladder:       M cross-rungs between two parallel rails, rail-1 fully grounded. M-1 loops.
 *
 * For each size, runs the solver R times from the same perturbed pose and reports
 * mean / p95 wall-clock per solve, plus the final residual. Pose is restored between
 * runs so each timed solve has the same starting work.
 */
import { Scene } from "../src/model";
import { solve } from "../src/solver";
import { Vec2, sub, len } from "../src/geometry";

const BAR_W = 100;
const BAR_H = 8;

interface Bar {
  bodyId: number;
  leftId: number;
  rightId: number;
}

function makeBar(scene: Scene, x: number, y: number, w = BAR_W): Bar {
  const b = scene.addBody([
    { x, y: y - BAR_H / 2 },
    { x: x + w, y: y - BAR_H / 2 },
    { x: x + w, y: y + BAR_H / 2 },
    { x, y: y + BAR_H / 2 },
  ]);
  const left = scene.addJoint(b.id, { x, y });
  const right = scene.addJoint(b.id, { x: x + w, y });
  return { bodyId: b.id, leftId: left.id, rightId: right.id };
}

function openChain(N: number): Scene {
  const s = new Scene();
  const bars: Bar[] = [];
  for (let i = 0; i < N; i++) bars.push(makeBar(s, i * BAR_W, 0));
  s.addGround(bars[0].leftId, { x: 0, y: 0 });
  for (let i = 0; i < N - 1; i++) s.addPin(bars[i].rightId, bars[i + 1].leftId);
  return s;
}

function closedChain(N: number): Scene {
  const s = new Scene();
  const bars: Bar[] = [];
  for (let i = 0; i < N; i++) bars.push(makeBar(s, i * BAR_W, 0));
  s.addGround(bars[0].leftId, { x: 0, y: 0 });
  // Ground the far end inside the chain's reach so the loop is solvable.
  const farX = N * BAR_W * 0.7;
  const farY = N * BAR_W * 0.2;
  s.addGround(bars[N - 1].rightId, { x: farX, y: farY });
  for (let i = 0; i < N - 1; i++) s.addPin(bars[i].rightId, bars[i + 1].leftId);
  return s;
}

function ladder(M: number): Scene {
  // Two parallel rails (top and bottom horizontal bars) connected by M short cross-rungs
  // at evenly spaced positions. Both ends of the bottom rail are grounded, locking it.
  // The top rail + rungs form M-1 closed loops (M rungs minus 1 spanning tree edge).
  const s = new Scene();
  const span = M * 60;
  const rungLen = 80;
  const bot = makeBar(s, 0, 0, span);
  const top = makeBar(s, 0, rungLen, span);
  s.addGround(bot.leftId, { x: 0, y: 0 });
  s.addGround(bot.rightId, { x: span, y: 0 });
  for (let i = 0; i < M; i++) {
    const x = (i + 0.5) * (span / M);
    // Attach points on the rails at column x
    const botPt = s.addJoint(bot.bodyId, { x, y: 0 });
    const topPt = s.addJoint(top.bodyId, { x, y: rungLen });
    // Vertical rung body between (x, 0) and (x, rungLen)
    const rung = s.addBody([
      { x: x - BAR_H / 2, y: 0 },
      { x: x + BAR_H / 2, y: 0 },
      { x: x + BAR_H / 2, y: rungLen },
      { x: x - BAR_H / 2, y: rungLen },
    ]);
    const rBot = s.addJoint(rung.id, { x, y: 0 });
    const rTop = s.addJoint(rung.id, { x, y: rungLen });
    s.addPin(rBot.id, botPt.id);
    s.addPin(rTop.id, topPt.id);
  }
  return s;
}

function snapshotPoses(scene: Scene): { pos: Vec2; angle: number }[] {
  return scene.bodies.map((b) => ({ pos: { x: b.pos.x, y: b.pos.y }, angle: b.angle }));
}

function restorePoses(scene: Scene, snap: { pos: Vec2; angle: number }[]): void {
  scene.bodies.forEach((b, i) => {
    b.pos.x = snap[i].pos.x;
    b.pos.y = snap[i].pos.y;
    b.angle = snap[i].angle;
  });
}

function perturb(scene: Scene, amount: number, seed: number): void {
  // Deterministic LCG so every run sees the same perturbation.
  let r = seed >>> 0;
  const rand = () => {
    r = (Math.imul(r, 1664525) + 1013904223) >>> 0;
    return r / 0x100000000;
  };
  for (const b of scene.bodies) {
    b.pos.x += (rand() - 0.5) * 2 * amount;
    b.pos.y += (rand() - 0.5) * 2 * amount;
    b.angle += (rand() - 0.5) * 0.3;
  }
}

function residual(scene: Scene): number {
  let max = 0;
  for (const c of scene.constraints) {
    if (c.kind === "pin") {
      const ja = scene.getJoint(c.jointA);
      const jb = scene.getJoint(c.jointB);
      if (ja && jb) max = Math.max(max, len(sub(scene.jointWorld(ja), scene.jointWorld(jb))));
    } else if (c.kind === "ground") {
      const j = scene.getJoint(c.joint);
      if (j) max = Math.max(max, len(sub(scene.jointWorld(j), c.anchor)));
    }
  }
  return max;
}

interface Row {
  topology: string;
  N: number;
  bodies: number;
  constraints: number;
  meanMs: number;
  p95Ms: number;
  residual: number;
}

function bench(name: string, N: number, build: () => Scene, runs = 20): Row {
  const scene = build();
  perturb(scene, 5, 0xc0ffee ^ N);
  const start = snapshotPoses(scene);

  // One untimed warm-up so JIT decisions settle.
  solve(scene, null, 100, 1);
  restorePoses(scene, start);

  const samples: number[] = [];
  let finalRes = 0;
  for (let r = 0; r < runs; r++) {
    restorePoses(scene, start);
    const t0 = performance.now();
    solve(scene, null, 100, 1);
    const t1 = performance.now();
    samples.push(t1 - t0);
    finalRes = residual(scene);
  }
  samples.sort((a, b) => a - b);
  const mean = samples.reduce((s, x) => s + x, 0) / samples.length;
  const p95 = samples[Math.min(samples.length - 1, Math.floor(samples.length * 0.95))];

  return {
    topology: name,
    N,
    bodies: scene.bodies.length,
    constraints: scene.constraints.length,
    meanMs: mean,
    p95Ms: p95,
    residual: finalRes,
  };
}

const sizes = [4, 8, 16, 32, 64, 128];
const rows: Row[] = [];

console.log("Building and benching... (each row = 20 timed solves from a perturbed pose)\n");

for (const N of sizes) rows.push(bench("open-chain", N, () => openChain(N)));
for (const N of sizes) rows.push(bench("closed-chain", N, () => closedChain(N)));
for (const N of sizes) rows.push(bench("ladder", N, () => ladder(N)));

// Pretty-print
const headers = ["topology", "N", "bodies", "cons", "mean ms", "p95 ms", "residual"];
const cells = rows.map((r) => [
  r.topology,
  String(r.N),
  String(r.bodies),
  String(r.constraints),
  r.meanMs.toFixed(3),
  r.p95Ms.toFixed(3),
  r.residual.toExponential(2),
]);
const widths = headers.map((h, i) =>
  Math.max(h.length, ...cells.map((row) => row[i].length))
);
const fmtRow = (row: string[]) =>
  row.map((c, i) => c.padStart(widths[i])).join("  ");
console.log(fmtRow(headers));
console.log(widths.map((w) => "-".repeat(w)).join("  "));
let lastTopo = "";
for (const row of cells) {
  if (lastTopo && row[0] !== lastTopo) console.log("");
  console.log(fmtRow(row));
  lastTopo = row[0];
}

// Crude scaling exponent per topology: log-log slope of mean-ms vs constraints.
console.log("\nScaling exponent (log-log slope of mean-ms vs constraint count):");
for (const topo of ["open-chain", "closed-chain", "ladder"]) {
  const xs = rows.filter((r) => r.topology === topo);
  const xy = xs.map((r) => ({ x: Math.log(r.constraints), y: Math.log(Math.max(r.meanMs, 1e-3)) }));
  const n = xy.length;
  const sx = xy.reduce((s, p) => s + p.x, 0);
  const sy = xy.reduce((s, p) => s + p.y, 0);
  const sxx = xy.reduce((s, p) => s + p.x * p.x, 0);
  const sxy = xy.reduce((s, p) => s + p.x * p.y, 0);
  const slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
  console.log(`  ${topo.padEnd(14)} slope ≈ ${slope.toFixed(2)}`);
}
console.log("\n(slope ~1 = linear, ~2 = quadratic, etc.)");
