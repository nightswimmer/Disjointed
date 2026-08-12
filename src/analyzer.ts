/**
 * Topology diagnostic for the scene (Stage 1 of the propagation-solver exploration).
 *
 * Builds a constraint graph (bodies + free joints as nodes; pins + slider-rider couplings
 * as edges), finds connected components, and per-component reports a Grübler-Kutzbach DOF
 * estimate, cyclomatic complexity (closed-loop count), a BFS-from-anchors propagation
 * order, and the back-edges that close those loops. Nothing here mutates the scene or
 * affects the solver — it just describes how the assembly is wired so we can judge how
 * much would decompose cleanly under a dyad-by-dyad propagation strategy.
 */
import { Scene } from "./model";

/** One edge in the topology graph — a single coincidence/coupling between two nodes. */
export interface AnalyzerEdge {
  /** Constraint kind that produced the coupling. */
  via: "pin" | "slider";
  /** Source constraint id (a pin id, or a slider id). */
  constraintId: number;
  /**
   * Joint ids the edge touches. For a pin: `[jointA, jointB]`. For a slider rider:
   * `[riderJointId]` — the rail is identified by the constraint, not a single joint.
   */
  joints: number[];
  /** Endpoint node keys. A body node's key is its body id; a free joint's key is the joint id. */
  a: number;
  b: number;
}

/** One node visited during the propagation walk: the node, its BFS depth, and the edge used to reach it. */
export interface BfsStep {
  distance: number;
  /** Node key (body id or free joint id). */
  node: number;
  /** Edge taken from a previously-visited node to first reach this one; null for seeds. */
  via: AnalyzerEdge | null;
}

/**
 * Augmented edge for bridge / BCC analysis. Unlike `AnalyzerEdge`, this includes synthetic
 * ground edges (anchored node ↔ virtual world) so the loop topology through world is visible
 * to the algorithms. Used internally; surfaced indirectly via `bccs` on `ComponentReport`.
 */
export interface AugEdge {
  via: "pin" | "slider" | "ground";
  constraintId: number;
  a: number;
  b: number;
  /** Stable per-edge id for parent-edge skipping during DFS. */
  key: string;
}

/**
 * One biconnected component of the augmented topology graph. A BCC is a maximal set of edges
 * such that any two edges in it lie on a common cycle. In mechanism terms: the bodies in a
 * BCC form one inseparable solve unit — you can't peel any single body off without breaking
 * the loop. BCCs are joined at articulation nodes (bodies/free-joints/world).
 */
export interface BccReport {
  /** Edges belonging to this BCC (pins, sliders, and/or ground edges through world). */
  edges: AugEdge[];
  /** Distinct nodes touched by those edges (body ids, free-joint ids, or the world sentinel). */
  nodes: number[];
  /** Just the real bodies in `nodes` — what you'd report as "the bodies in this block". */
  bodies: number[];
  /** Whether this BCC includes the virtual world node (i.e., the loop closes through ground). */
  includesWorld: boolean;
  /** Cyclomatic complexity of this BCC alone (edges − nodes + 1). 0 for a single-edge bridge. */
  internalCycles: number;
}

/** A connected component of the topology graph (one kinematic "island"). */
export interface ComponentReport {
  bodies: number[];          // body ids belonging to this component
  freeJoints: number[];      // free-joint node keys (body-less joints) belonging to this component
  edges: AnalyzerEdge[];     // pin/slider edges fully internal to this component
  anchoredNodes: number[];   // node keys with ≥1 ground constraint (or world-fixed-rail equivalent)
  groundCount: number;       // total ground constraints touching this component
  cycles: number;            // cyclomatic complexity (edges − nodes + 1); 0 = tree, ≥1 = closed loops
  dofEstimate: number;       // Grübler-Kutzbach M (planar): 3·B + 2·F − 2·(pins + grounds) − sliderRiders
  /** Propagation order (distance from anchors, BFS). Seeds get distance 0. */
  bfsOrder: BfsStep[];
  /** Edges that close cycles — not part of the BFS tree. Empty for a tree component. */
  backEdges: AnalyzerEdge[];
  /**
   * Body-level decomposition (per body, not per island): which bodies sit inside a closed-loop
   * core (touch a non-bridge edge — must be solved with the loop) vs which sit on a tree branch
   * off the cores or off the anchor (touch only bridge edges — propagatable in one pass once
   * any upstream loop is solved). For unanchored islands, all bodies fall into `floating`.
   */
  loopCoreBodies: number[];
  propagatableBodies: number[];
  /**
   * Biconnected-component decomposition of the augmented graph (pin/slider + ground edges).
   * Non-trivial BCCs (`edges.length ≥ 2`) are independent loop cores — each can in principle
   * be solved separately once articulation bodies are pinned by neighbour-block solves. A
   * single big BCC of N bodies means the whole island is one inseparable solve.
   */
  bccs: BccReport[];
  /** Bodies appearing in 2+ BCCs (cut vertices joining independent solve blocks). */
  articulationBodies: number[];
}

export interface AnalysisReport {
  totals: {
    bodies: number;
    freeJoints: number;
    bodyJoints: number;
    pins: number;
    grounds: number;
    sliders: number;
    sliderRiders: number;
  };
  components: ComponentReport[];
}

/** Stable per-edge key for tree/back-edge tracking. */
function edgeKey(e: AnalyzerEdge): string {
  return `${e.constraintId}:${e.joints[0]}`;
}

/** Sentinel for the virtual world node used during bridge analysis (real ids are positive). */
const WORLD = -1;

/**
 * Tarjan's bridge DFS on an undirected graph. Returns the set of edge keys whose removal
 * would disconnect the graph. Handles parallel edges correctly via per-edge keys: two
 * parallel edges between the same pair of nodes are both *not* bridges (they form a
 * 2-edge cycle). This is what lets two grounds on one body register as a loop-through-world.
 */
function findBridges(
  nodes: Iterable<number>,
  adj: Map<number, { neighbor: number; key: string }[]>
): Set<string> {
  const disc = new Map<number, number>();
  const low = new Map<number, number>();
  const bridges = new Set<string>();
  let timer = 0;

  const dfs = (u: number, parentEdgeKey: string | null): void => {
    disc.set(u, timer);
    low.set(u, timer);
    timer++;
    for (const e of adj.get(u) ?? []) {
      // Skip exactly the edge we descended through. Edge keys are unique, so at most one
      // edge matches; parallel edges to the parent have different keys and fall through to
      // the back-edge branch below, correctly registering as cycle closures.
      if (e.key === parentEdgeKey) continue;
      if (!disc.has(e.neighbor)) {
        dfs(e.neighbor, e.key);
        low.set(u, Math.min(low.get(u)!, low.get(e.neighbor)!));
        if (low.get(e.neighbor)! > disc.get(u)!) bridges.add(e.key);
      } else {
        // Back-edge to an ancestor (or cross-edge): use its discovery time as a candidate
        // for `low(u)`, lowering it and disqualifying ancestor-to-u edges as bridges.
        low.set(u, Math.min(low.get(u)!, disc.get(e.neighbor)!));
      }
    }
  };

  for (const n of nodes) if (!disc.has(n)) dfs(n, null);
  return bridges;
}

/**
 * Tarjan's biconnected-component DFS. Returns BCCs as arrays of `AugEdge` (each BCC is a
 * maximal set of edges sharing a common cycle, or a single bridge edge). The adjacency must
 * be the augmented one — i.e., pin/slider/ground edges — and must carry the full `AugEdge`
 * objects (not just keys) so we can return them grouped per BCC.
 *
 * Algorithm: standard DFS with `disc`/`low` arrays + an edge stack. When we finish a child
 * `v` and find `low(v) ≥ disc(u)`, every edge pushed since we descended into v belongs to a
 * BCC rooted at u — we pop them off the stack (down to and including the tree edge u→v).
 * Parallel edges are handled by per-edge keys: only the actual parent edge is skipped, and
 * other parallel edges fall through as back-edges that correctly create 2-edge cycles.
 */
function findBccs(
  nodes: Iterable<number>,
  adj: Map<number, { neighbor: number; key: string; edge: AugEdge }[]>
): AugEdge[][] {
  const disc = new Map<number, number>();
  const low = new Map<number, number>();
  const stack: AugEdge[] = [];
  const bccs: AugEdge[][] = [];
  let timer = 0;

  const dfs = (u: number, parentEdgeKey: string | null): void => {
    disc.set(u, timer);
    low.set(u, timer);
    timer++;
    for (const e of adj.get(u) ?? []) {
      if (e.key === parentEdgeKey) continue; // skip the one edge we came in on
      if (!disc.has(e.neighbor)) {
        // Tree edge — push, descend, then pop a BCC if v's subtree can't escape u.
        stack.push(e.edge);
        dfs(e.neighbor, e.key);
        low.set(u, Math.min(low.get(u)!, low.get(e.neighbor)!));
        if (low.get(e.neighbor)! >= disc.get(u)!) {
          const bcc: AugEdge[] = [];
          // Pop edges off the stack until we pop the tree edge u→v itself.
          // The stack is non-empty here because we just pushed e.edge before recursing.
          while (stack.length > 0) {
            const top = stack.pop()!;
            bcc.push(top);
            if (top.key === e.key) break;
          }
          bccs.push(bcc);
        }
      } else if (disc.get(e.neighbor)! < disc.get(u)!) {
        // Back-edge to a strict ancestor: push (it's in u's BCC) and update low.
        stack.push(e.edge);
        low.set(u, Math.min(low.get(u)!, disc.get(e.neighbor)!));
      }
      // disc(neighbor) > disc(u) on an already-visited neighbour means we're looking at the
      // *other side* of a back-edge we already pushed when we descended past it; skip it.
    }
  };

  for (const n of nodes) if (!disc.has(n)) dfs(n, null);
  return bccs;
}

export function analyzeScene(scene: Scene): AnalysisReport {
  // Joint → owner node key (body id, or the joint's own id if the joint is body-less).
  const ownerOf = new Map<number, number>();
  for (const j of scene.joints) ownerOf.set(j.id, j.bodyId ?? j.id);

  // Anchored = at least one ground constraint pins this node to the world.
  const groundsByNode = new Map<number, number>();
  for (const c of scene.constraints) {
    if (c.kind !== "ground") continue;
    const owner = ownerOf.get(c.joint);
    if (owner === undefined) continue;
    groundsByNode.set(owner, (groundsByNode.get(owner) ?? 0) + 1);
  }

  // Edges: pins always couple two distinct nodes; slider riders couple the rider's owner
  // to the rail's representative owner (`railA`). A rider that lives on the same body as
  // its rail wouldn't move along the rail anyway, so we drop the self-loop.
  const edges: AnalyzerEdge[] = [];
  for (const c of scene.constraints) {
    if (c.kind === "pin") {
      const a = ownerOf.get(c.jointA);
      const b = ownerOf.get(c.jointB);
      if (a === undefined || b === undefined || a === b) continue;
      edges.push({ via: "pin", constraintId: c.id, joints: [c.jointA, c.jointB], a, b });
    } else if (c.kind === "slider") {
      const railOwner = ownerOf.get(c.railA);
      if (railOwner === undefined) continue;
      for (const riderId of c.riders) {
        const r = ownerOf.get(riderId);
        if (r === undefined || r === railOwner) continue;
        edges.push({ via: "slider", constraintId: c.id, joints: [riderId], a: r, b: railOwner });
      }
    }
  }

  // Union-find over the node universe (every body + every free joint).
  const parent = new Map<number, number>();
  const find = (x: number): number => {
    if (parent.get(x) === undefined) parent.set(x, x);
    let root = x;
    while (parent.get(root)! !== root) root = parent.get(root)!;
    // path compression
    let cur = x;
    while (parent.get(cur)! !== root) {
      const next = parent.get(cur)!;
      parent.set(cur, root);
      cur = next;
    }
    return root;
  };
  const union = (a: number, b: number): void => {
    const ra = find(a);
    const rb = find(b);
    if (ra !== rb) parent.set(ra, rb);
  };
  for (const b of scene.bodies) find(b.id);
  for (const j of scene.joints) if (j.bodyId === null) find(j.id);
  for (const e of edges) union(e.a, e.b);

  // Bucket nodes + edges by their component root.
  interface Bucket {
    bodies: number[];
    freeJoints: number[];
    edges: AnalyzerEdge[];
  }
  const buckets = new Map<number, Bucket>();
  const getBucket = (root: number): Bucket => {
    let b = buckets.get(root);
    if (!b) {
      b = { bodies: [], freeJoints: [], edges: [] };
      buckets.set(root, b);
    }
    return b;
  };
  for (const b of scene.bodies) getBucket(find(b.id)).bodies.push(b.id);
  for (const j of scene.joints) if (j.bodyId === null) getBucket(find(j.id)).freeJoints.push(j.id);
  for (const e of edges) getBucket(find(e.a)).edges.push(e);

  // Per-component: BFS from anchored seeds, count cycles, estimate DOF.
  const components: ComponentReport[] = [];
  for (const bucket of buckets.values()) {
    const nodeKeys = [...bucket.bodies, ...bucket.freeJoints];
    const nodeCount = nodeKeys.length;
    const edgeCount = bucket.edges.length;

    // Anchored nodes (with ground constraints).
    const anchoredNodes: number[] = [];
    let groundCount = 0;
    for (const n of nodeKeys) {
      const g = groundsByNode.get(n);
      if (g) {
        anchoredNodes.push(n);
        groundCount += g;
      }
    }

    // Cyclomatic complexity counts mechanism loops, not raw graph cycles — two bodies
    // grounded separately and pinned together form one closed loop *through the world*,
    // which a pin-only edge count misses. Treat ground as a virtual world node with an
    // edge per ground constraint: V += 1 (when any ground is present), E += groundCount.
    const worldNode = groundCount > 0 ? 1 : 0;
    const cycles = Math.max(0, edgeCount + groundCount - (nodeCount + worldNode) + 1);

    // DOF: planar Grübler-Kutzbach. Pins and grounds each remove 2 DOF (a point coincidence /
    // a point locked to world). A slider rider is a point-on-line contact — it slides AND
    // rotates freely (one equality constraint in the solver) — so it removes only 1 DOF.
    const pinEdges = bucket.edges.filter((e) => e.via === "pin").length;
    const sliderEdges = bucket.edges.filter((e) => e.via === "slider").length;
    const dofEstimate =
      3 * bucket.bodies.length +
      2 * bucket.freeJoints.length -
      2 * (pinEdges + groundCount) -
      sliderEdges;

    // Adjacency for BFS.
    const adj = new Map<number, AnalyzerEdge[]>();
    for (const e of bucket.edges) {
      if (!adj.has(e.a)) adj.set(e.a, []);
      if (!adj.has(e.b)) adj.set(e.b, []);
      adj.get(e.a)!.push(e);
      adj.get(e.b)!.push(e);
    }

    // BFS from anchored seeds (one per ground node). With no anchor, pick any node so the
    // floating component still gets a propagation order — but flag it via dofEstimate >= 3.
    const seeds = anchoredNodes.length > 0 ? [...anchoredNodes] : nodeKeys.slice(0, 1);
    const visited = new Set<number>();
    const queue: BfsStep[] = [];
    const order: BfsStep[] = [];
    const treeEdgeKeys = new Set<string>();
    for (const s of seeds) {
      if (!visited.has(s)) {
        visited.add(s);
        queue.push({ distance: 0, node: s, via: null });
      }
    }
    while (queue.length > 0) {
      const step = queue.shift()!;
      order.push(step);
      for (const e of adj.get(step.node) ?? []) {
        const other = e.a === step.node ? e.b : e.a;
        if (!visited.has(other)) {
          visited.add(other);
          treeEdgeKeys.add(edgeKey(e));
          queue.push({ distance: step.distance + 1, node: other, via: e });
        }
      }
    }

    // Back-edges = every edge not in the BFS spanning tree.
    const backEdges = bucket.edges.filter((e) => !treeEdgeKeys.has(edgeKey(e)));

    // ----- Augmented graph for bridge + BCC analysis ------------------------------------
    // Pin/slider edges + one synthetic edge per ground constraint (anchored body ↔ virtual
    // world node). This is what the cycle-aware algorithms see: world ties together the
    // anchored nodes, so two grounded bodies pinned together form a true loop-through-world.
    const augAdj = new Map<number, { neighbor: number; key: string; edge: AugEdge }[]>();
    const addAugEdge = (e: AugEdge): void => {
      if (!augAdj.has(e.a)) augAdj.set(e.a, []);
      if (!augAdj.has(e.b)) augAdj.set(e.b, []);
      augAdj.get(e.a)!.push({ neighbor: e.b, key: e.key, edge: e });
      augAdj.get(e.b)!.push({ neighbor: e.a, key: e.key, edge: e });
    };
    // Seed every node so isolated bodies still appear as DFS roots.
    for (const n of nodeKeys) if (!augAdj.has(n)) augAdj.set(n, []);
    // Pin/slider edges from the bucket.
    for (const e of bucket.edges) {
      addAugEdge({ via: e.via, constraintId: e.constraintId, a: e.a, b: e.b, key: edgeKey(e) });
    }
    // Synthetic ground edges. Each ground constraint = a unique edge node↔world; two grounds
    // on one body therefore form a 2-edge parallel cycle (correctly: that body is over-locked
    // and counts as in a loop core).
    for (const c of scene.constraints) {
      if (c.kind !== "ground") continue;
      const owner = ownerOf.get(c.joint);
      if (owner === undefined || !nodeKeys.includes(owner)) continue;
      addAugEdge({ via: "ground", constraintId: c.id, a: owner, b: WORLD, key: `g${c.id}` });
    }
    const dfsRoots = [...nodeKeys, ...(groundCount > 0 ? [WORLD] : [])];

    // Bridges → body-level "loop core vs tree branch" classification.
    const bridgeOnlyAdj = new Map<number, { neighbor: number; key: string }[]>();
    for (const [k, v] of augAdj) bridgeOnlyAdj.set(k, v.map((x) => ({ neighbor: x.neighbor, key: x.key })));
    const bridges = findBridges(dfsRoots, bridgeOnlyAdj);
    const inLoopCore = new Set<number>();
    for (const [node, neighbors] of augAdj) {
      if (node === WORLD) continue;
      for (const { key } of neighbors) {
        if (!bridges.has(key)) {
          inLoopCore.add(node);
          break;
        }
      }
    }
    const loopCoreBodies = bucket.bodies.filter((id) => inLoopCore.has(id));
    const propagatableBodies = anchoredNodes.length > 0
      ? bucket.bodies.filter((id) => !inLoopCore.has(id))
      : [];

    // BCCs → independent solve blocks. Each BCC is one inseparable unit; multiple BCCs in
    // one island means the island decomposes into smaller solve problems joined at
    // articulation bodies (which appear in 2+ BCCs).
    const bccEdgeGroups = findBccs(dfsRoots, augAdj);
    const bccs: BccReport[] = bccEdgeGroups.map((edges) => {
      const nodeSet = new Set<number>();
      for (const ed of edges) {
        nodeSet.add(ed.a);
        nodeSet.add(ed.b);
      }
      const nodes = [...nodeSet];
      const includesWorld = nodeSet.has(WORLD);
      const bodySet = new Set(bucket.bodies);
      const bodies = nodes.filter((n) => bodySet.has(n));
      const internalCycles = Math.max(0, edges.length - nodes.length + 1);
      return { edges, nodes, bodies, includesWorld, internalCycles };
    });
    // Articulation bodies: bodies appearing in 2+ BCCs. Each appearance ≠ each edge — we
    // count distinct BCC membership per body.
    const bccCountByBody = new Map<number, number>();
    for (const bcc of bccs) {
      for (const b of bcc.bodies) bccCountByBody.set(b, (bccCountByBody.get(b) ?? 0) + 1);
    }
    const articulationBodies = bucket.bodies.filter((id) => (bccCountByBody.get(id) ?? 0) >= 2);

    components.push({
      bodies: bucket.bodies,
      freeJoints: bucket.freeJoints,
      edges: bucket.edges,
      anchoredNodes,
      groundCount,
      cycles,
      dofEstimate,
      bfsOrder: order,
      backEdges,
      loopCoreBodies,
      propagatableBodies,
      bccs,
      articulationBodies,
    });
  }

  // Sort: anchored components first (largest first), floating components last.
  components.sort((a, b) => {
    if ((a.anchoredNodes.length > 0) !== (b.anchoredNodes.length > 0)) {
      return a.anchoredNodes.length > 0 ? -1 : 1;
    }
    return b.bodies.length + b.freeJoints.length - (a.bodies.length + a.freeJoints.length);
  });

  // Totals.
  let pins = 0;
  let grounds = 0;
  let sliders = 0;
  let sliderRiders = 0;
  for (const c of scene.constraints) {
    if (c.kind === "pin") pins++;
    else if (c.kind === "ground") grounds++;
    else if (c.kind === "slider") {
      sliders++;
      sliderRiders += c.riders.length;
    }
  }
  const bodyJoints = scene.joints.filter((j) => j.bodyId !== null).length;
  const freeJointCount = scene.joints.length - bodyJoints;

  return {
    totals: { bodies: scene.bodies.length, freeJoints: freeJointCount, bodyJoints, pins, grounds, sliders, sliderRiders },
    components,
  };
}

/** Render a node key as a display label (bodies have no user-facing name, so use the id). */
function nodeLabel(nodeKey: number, scene: Scene): string {
  if (scene.getBody(nodeKey)) return `Body #${nodeKey}`;
  // Otherwise it's a free joint node.
  return `Free joint #${nodeKey}`;
}

/** Format the analysis as a human-readable string for the console. */
export function formatReport(report: AnalysisReport, scene: Scene): string {
  const lines: string[] = [];
  const t = report.totals;
  lines.push("=== Scene Analysis ===");
  lines.push("");
  lines.push(
    `Totals: ${t.bodies} bodies, ${t.freeJoints} free joints, ${t.bodyJoints} joints on bodies`
  );
  lines.push(
    `        ${t.pins} pin(s), ${t.grounds} ground(s), ${t.sliders} slider(s) with ${t.sliderRiders} rider(s)`
  );
  lines.push(`Kinematic islands: ${report.components.length}`);
  lines.push("");

  report.components.forEach((c, i) => {
    const size = c.bodies.length + c.freeJoints.length;
    const anchored = c.anchoredNodes.length > 0;
    const loopWord = c.cycles === 1 ? "1 closed loop" : `${c.cycles} closed loops`;
    const dofWord =
      c.dofEstimate === 0
        ? "fully constrained (0 DOF)"
        : c.dofEstimate < 0
        ? `over-constrained (${c.dofEstimate} DOF — ${-c.dofEstimate} redundant constraint${-c.dofEstimate === 1 ? "" : "s"})`
        : `${c.dofEstimate} DOF`;
    lines.push(
      `--- Island ${i + 1}: ${size} node(s), ${loopWord}, ${dofWord}${anchored ? "" : " — FLOATING (no anchor)"} ---`
    );
    if (c.bodies.length > 0) {
      const names = c.bodies.map((id) => `#${id}`).join(", ");
      lines.push(`  Bodies (${c.bodies.length}): ${names}`);
    }
    if (c.freeJoints.length > 0) {
      lines.push(`  Free joints (${c.freeJoints.length}): ${c.freeJoints.map((id) => `#${id}`).join(", ")}`);
    }
    if (anchored) {
      const anchorLabels = c.anchoredNodes.map((id) => nodeLabel(id, scene)).join(", ");
      lines.push(`  Anchored to world (${c.groundCount} ground(s)): ${anchorLabels}`);
    }

    // Body-level decomposition for this island: how the solver could split it.
    if (c.bodies.length > 0) {
      const coreNames = c.loopCoreBodies.map((id) => `#${id}`);
      const propNames = c.propagatableBodies.map((id) => `#${id}`);
      lines.push("");
      lines.push("  Body decomposition:");
      if (coreNames.length > 0) {
        lines.push(`    Loop core (${coreNames.length}, must be solved together): ${coreNames.join(", ")}`);
      } else {
        lines.push(`    Loop core (0): —`);
      }
      if (propNames.length > 0) {
        lines.push(`    Tree branches (${propNames.length}, propagatable in one pass): ${propNames.join(", ")}`);
      } else if (anchored) {
        lines.push(`    Tree branches (0): —`);
      }
    }

    // Block decomposition (biconnected components). The headline answer to "does this big
    // loop core split into independent sub-solves?" — if there are multiple non-trivial
    // BCCs, yes (joined at articulation bodies); if there's only one, no.
    const nonTrivial = c.bccs.filter((b) => b.edges.length >= 2);
    const trivial = c.bccs.length - nonTrivial.length;
    if (c.bccs.length > 0) {
      lines.push("");
      lines.push("  Block decomposition (biconnected components):");
      if (nonTrivial.length === 0) {
        lines.push(`    No non-trivial blocks — the island is a tree of ${trivial} bridge edge(s).`);
      } else {
        // Sort blocks largest-first so the big ones lead the report.
        const sorted = [...nonTrivial].sort((a, b) => b.bodies.length - a.bodies.length);
        lines.push(
          `    Non-trivial blocks (closed-loop cores, each is one independent solve): ${nonTrivial.length}`
        );
        sorted.forEach((b, i) => {
          const names = b.bodies.map((id) => `#${id}`).join(", ");
          const through = b.includesWorld ? " (through ground)" : "";
          const cyc = b.internalCycles === 1 ? "1 internal loop" : `${b.internalCycles} internal loops`;
          lines.push(
            `      Block ${i + 1}: ${b.bodies.length} body/bodies, ${b.edges.length} edges, ${cyc}${through} — ${names || "(no bodies, world-only)"}`
          );
        });
        lines.push(`    Trivial blocks (single bridge edges): ${trivial}`);
      }
      if (c.articulationBodies.length > 0) {
        const names = c.articulationBodies.map((id) => `#${id}`).join(", ");
        lines.push(
          `    Articulation bodies (in ${c.articulationBodies.length === 1 ? "this block boundary" : "multiple block boundaries"}): ${names}`
        );
      }
    }

    // Propagation order — group by distance.
    if (c.bfsOrder.length > 0) {
      lines.push("");
      lines.push("  Propagation order (BFS from anchor):");
      const byDist = new Map<number, BfsStep[]>();
      for (const s of c.bfsOrder) {
        if (!byDist.has(s.distance)) byDist.set(s.distance, []);
        byDist.get(s.distance)!.push(s);
      }
      const dists = [...byDist.keys()].sort((a, b) => a - b);
      for (const d of dists) {
        const steps = byDist.get(d)!;
        steps.forEach((step, idx) => {
          const me = nodeLabel(step.node, scene);
          const tag = idx === 0 ? `    d=${d}` : `       `;
          if (step.via) {
            const parent = step.via.a === step.node ? step.via.b : step.via.a;
            const parentLabel = nodeLabel(parent, scene);
            const via =
              step.via.via === "pin"
                ? `pin (joints #${step.via.joints[0]}↔#${step.via.joints[1]})`
                : `slider rider (joint #${step.via.joints[0]})`;
            lines.push(`${tag}  ${me}  ← from ${parentLabel} via ${via}`);
          } else {
            const tagText = c.anchoredNodes.includes(step.node) ? "[anchor]" : "[seed]";
            lines.push(`${tag}  ${me}  ${tagText}`);
          }
        });
      }
    }

    // Back-edges — list the loop closures.
    if (c.backEdges.length > 0) {
      lines.push("");
      lines.push("  Back-edges (close the loops — must be solved together, not propagated):");
      for (const e of c.backEdges) {
        const aLabel = nodeLabel(e.a, scene);
        const bLabel = nodeLabel(e.b, scene);
        if (e.via === "pin") {
          lines.push(`    ${aLabel}  ↔  ${bLabel}   pin (joints #${e.joints[0]}↔#${e.joints[1]})`);
        } else {
          lines.push(`    ${aLabel}  ↔  ${bLabel}   slider rider (joint #${e.joints[0]})`);
        }
      }
    }
    lines.push("");
  });

  // Body-level decomposition summary. The buckets are mutually exclusive:
  //  - propagatable: anchored island AND body touches only bridge edges (a tree branch off
  //    either an anchor or a loop core — solvable in one pass once any upstream loop is done)
  //  - loopCore:     body touches at least one non-bridge edge (sits inside a closed loop
  //                  that must be solved as a unit, not propagated)
  //  - floating:     body is in an island with no ground anchor (mechanism with rigid-body
  //                  freedom; nothing pins it to world)
  let propagatable = 0;
  let loopCore = 0;
  let floating = 0;
  for (const c of report.components) {
    if (c.anchoredNodes.length === 0) {
      floating += c.bodies.length;
    } else {
      propagatable += c.propagatableBodies.length;
      loopCore += c.loopCoreBodies.length;
    }
  }
  lines.push("Body-level decomposition summary:");
  lines.push(`  Propagatable (tree branch, one-pass):  ${propagatable} / ${t.bodies} bodies`);
  lines.push(`  In loop core (must solve together):    ${loopCore} / ${t.bodies} bodies`);
  lines.push(`  Floating (no anchor at all):           ${floating} / ${t.bodies} bodies`);
  if (loopCore > 0) {
    lines.push("");
    lines.push(
      `  Optimization headroom: ${propagatable} / ${t.bodies} bodies can be solved with closed-form`
    );
    lines.push(
      `  propagation once the ${loopCore}-body loop core is settled. Per-island breakdown above.`
    );
  }

  // Block-decomposition headline: how big is the largest inseparable solve unit, and how
  // many such units does the scene have? If the largest BCC is small relative to the total
  // body count, the mechanism decomposes well; if one BCC dominates, it doesn't.
  let nonTrivialBccCount = 0;
  let largestBccBodies = 0;
  let totalBccBodyCount = 0;
  for (const c of report.components) {
    for (const bcc of c.bccs) {
      if (bcc.edges.length < 2) continue;
      nonTrivialBccCount++;
      largestBccBodies = Math.max(largestBccBodies, bcc.bodies.length);
      totalBccBodyCount += bcc.bodies.length;
    }
  }
  if (nonTrivialBccCount > 0) {
    lines.push("");
    lines.push("Block-decomposition headline (biconnected components):");
    lines.push(`  Independent loop-core blocks: ${nonTrivialBccCount}`);
    lines.push(`  Largest single block: ${largestBccBodies} body/bodies`);
    if (nonTrivialBccCount > 1) {
      lines.push(
        `  Sum of block sizes: ${totalBccBodyCount} (articulation bodies count once per block they join)`
      );
    }
  }

  return lines.join("\n");
}
