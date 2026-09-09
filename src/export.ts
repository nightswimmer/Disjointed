/**
 * Export bodies as flat 2D cutting outlines for CNC / laser / plasma work — DXF (R12
 * ASCII) and SVG. Pure, DOM-free; `main.ts` only picks the bodies, calls one of the
 * writers, and hands the text to a download.
 *
 * Geometry is collected first into `CutLoop`s (in scene coordinates, y-down): a closed
 * ring of vertices where each vertex carries the *bulge* of the edge leaving it (the
 * DXF convention: `tan(sweep / 4)`, 0 = straight, sign = sweep direction in the ring's
 * own coordinate system), or a plain circle. Fillet corners are emitted as exact
 * tangent arcs via `filletCornerArcs`, offset-mode disks as circles, and uniform-margin
 * offset hulls as exact arcs + tangent lines. Anything else (offset hulls with mixed
 * margins) falls back to the sampled outline, which is what the canvas draws anyway.
 *
 * Both writers translate the geometry so the bounding box's corner lands at the
 * origin (positive quadrant — friendlier to bed-relative laser software) and:
 * - DXF flips y (DXF is y-up) — bulges change sign with it — writes POLYLINE /
 *   VERTEX / SEQEND + CIRCLE entities (bulges keep the arcs true) in the scene's
 *   working units with a matching `$INSUNITS`, body outlines + cut-outs on layer
 *   `CUT` and optional joint drill holes on layer `JOINTS`.
 * - SVG keeps y-down, converts to millimetres (`width`/`height` in mm with a matching
 *   `viewBox`, so 1 user unit = 1 mm and nothing gets rescaled on import), one `<path>`
 *   per body (outer + holes as subpaths, arcs via the `A` command) and `<circle>`s for
 *   joint holes, grouped by id.
 */
import { Vec2, vec, add, sub, scale, normalize, cross, dist, filletCornerArcs, convexHull, polygonArea } from "./geometry";
import { Body, Scene, Unit, UNIT_TO_MM } from "./model";

/** One ring vertex: its point plus the bulge (`tan(sweep/4)`) of the edge leaving it. */
export interface BulgeVertex {
  p: Vec2;
  bulge: number;
}

export type CutLoop = { kind: "ring"; verts: BulgeVertex[] } | { kind: "circle"; c: Vec2; r: number };

/** One exported body: its outer loop plus its cut-out loops. */
export interface CutPart {
  outer: CutLoop;
  holes: CutLoop[];
}

export interface CutSheet {
  parts: CutPart[];
  /** Joint drill holes (circles), when requested. */
  joints: CutLoop[];
  unit: Unit;
}

export interface ExportOptions {
  /** Diameter of the drill hole exported at every attached joint of an exported body; ≤ 0 / absent = none. */
  jointHoleDiameter?: number;
}

/** Two consecutive ring points closer than this collapse into one (zero-length edges). */
const MERGE_EPS = 1e-7;

// --- geometry collection --------------------------------------------------------

/**
 * Exact fillet ring of a control polygon (world coordinates): sharp corners keep their
 * vertex, rounded corners become their tangent arc (start point carrying the bulge,
 * end point straight to the next corner).
 */
function filletRing(ctrl: Vec2[], radii: number[]): BulgeVertex[] {
  const arcs = filletCornerArcs(ctrl, radii);
  const out: BulgeVertex[] = [];
  for (let i = 0; i < ctrl.length; i++) {
    const arc = arcs[i];
    if (!arc || arc.r <= 0 || Math.abs(arc.da) < 1e-9) {
      out.push({ p: vec(ctrl[i].x, ctrl[i].y), bulge: 0 });
      continue;
    }
    const a2 = arc.a1 + arc.da;
    out.push({ p: add(arc.center, vec(arc.r * Math.cos(arc.a1), arc.r * Math.sin(arc.a1))), bulge: Math.tan(arc.da / 4) });
    out.push({ p: add(arc.center, vec(arc.r * Math.cos(a2), arc.r * Math.sin(a2))), bulge: 0 });
  }
  return compactRing(out);
}

/**
 * Exact outline of a uniform-margin offset hull (the Minkowski sum of the points' convex
 * hull with a disk of radius `m`): an arc of radius `m` about every hull vertex, joined
 * by straight tangents. Returns null when the shape isn't representable this way (fewer
 * than two distinct hull points).
 */
function offsetHullRing(points: Vec2[], m: number): BulgeVertex[] | null {
  const hull = convexHull(points);
  if (hull.length < 2) return null;
  if (hull.length === 2) {
    // A stadium: two half-circles about the ends, joined by two parallel tangents.
    const [a, b] = hull;
    const d = sub(b, a);
    if (dist(a, b) < MERGE_EPS) return null;
    const dh = normalize(d);
    const n0 = vec(dh.y, -dh.x); // one side normal
    // Half circle about `b` from b+m·n0 to b−m·n0, bulging away from `a` (through b+m·dh):
    // sweep = ±π with the sign that puts the arc's midpoint along +dh.
    const sweepB = Math.PI * Math.sign(cross(n0, dh));
    const bulge = Math.tan(sweepB / 4);
    return compactRing([
      { p: add(a, scale(n0, m)), bulge: 0 },
      { p: add(b, scale(n0, m)), bulge },
      { p: sub(b, scale(n0, m)), bulge: 0 },
      { p: sub(a, scale(n0, m)), bulge },
    ]);
  }
  // Outward edge normals: for a ring with positive signed area (in its own coordinates)
  // the outward normal of edge direction d is (d.y, −d.x); negative area flips it.
  const n = hull.length;
  const sgn = polygonArea(hull) > 0 ? 1 : -1;
  const normals = hull.map((v, i) => {
    const d = normalize(sub(hull[(i + 1) % n], v));
    return scale(vec(d.y, -d.x), sgn);
  });
  const out: BulgeVertex[] = [];
  for (let i = 0; i < n; i++) {
    const nPrev = normals[(i - 1 + n) % n];
    const nNext = normals[i];
    // Sweep from the incoming edge's normal to the outgoing one (the exterior angle;
    // always < π for a strictly convex hull, so the short way is unambiguous).
    let da = Math.atan2(nNext.y, nNext.x) - Math.atan2(nPrev.y, nPrev.x);
    while (da > Math.PI) da -= 2 * Math.PI;
    while (da < -Math.PI) da += 2 * Math.PI;
    out.push({ p: add(hull[i], scale(nPrev, m)), bulge: Math.tan(da / 4) });
    out.push({ p: add(hull[i], scale(nNext, m)), bulge: 0 });
  }
  return compactRing(out);
}

/** Drop zero-length edges: a point coinciding with its predecessor replaces it (keeping the later bulge). */
function compactRing(ring: BulgeVertex[]): BulgeVertex[] {
  const out: BulgeVertex[] = [];
  for (const v of ring) {
    const prev = out[out.length - 1];
    if (prev && dist(prev.p, v.p) < MERGE_EPS) out[out.length - 1] = v;
    else out.push(v);
  }
  while (out.length > 1 && dist(out[0].p, out[out.length - 1].p) < MERGE_EPS) {
    // Closing duplicate: the last vertex's (zero-length) edge is dropped; the first
    // vertex keeps its own bulge since its edge is the real one.
    out.pop();
  }
  return out;
}

/** Sampled fallback: the polygon as straight edges. */
function polygonRing(pts: Vec2[]): BulgeVertex[] {
  return compactRing(pts.map((p) => ({ p: vec(p.x, p.y), bulge: 0 })));
}

/**
 * The cut loop of one outline (the body's outer shape, or one of its holes) in world
 * coordinates: exact arcs where the shape is parametric, the sampled polygon otherwise.
 */
function outlineLoop(
  ctrlWorld: Vec2[],
  radii: number[],
  round: "fillet" | "offset",
  sampledWorld: Vec2[]
): CutLoop {
  if (round === "offset") {
    const positive = radii.filter((r) => r > 0);
    if (ctrlWorld.length === 1 && radii[0] > 0) return { kind: "circle", c: vec(ctrlWorld[0].x, ctrlWorld[0].y), r: radii[0] };
    const uniform = positive.length === radii.length && positive.every((r) => Math.abs(r - positive[0]) < 1e-9);
    if (uniform && positive.length > 0) {
      const ring = offsetHullRing(ctrlWorld, positive[0]);
      if (ring && ring.length >= 2) return { kind: "ring", verts: ring };
    } else if (positive.length === 0) {
      const ring = polygonRing(convexHull(ctrlWorld));
      if (ring.length >= 3) return { kind: "ring", verts: ring };
    }
    return { kind: "ring", verts: polygonRing(sampledWorld) };
  }
  return { kind: "ring", verts: filletRing(ctrlWorld, radii) };
}

/** Collect the cut geometry of `bodies` (world coordinates, scene units). */
export function collectCutSheet(scene: Scene, bodies: Body[], opts: ExportOptions = {}): CutSheet {
  const parts: CutPart[] = [];
  for (const body of bodies) {
    const outer = outlineLoop(scene.bodyControlWorld(body), scene.bodyCornerRadii(body), body.round, scene.bodyWorldVerts(body));
    const holesWorld = scene.bodyHolesWorld(body);
    const holes = (body.holes ?? []).map((h, hi) =>
      outlineLoop(scene.bodyHoleControlWorld(body, hi), scene.bodyCornerRadii(body, hi), h.round ?? "fillet", holesWorld[hi] ?? [])
    );
    parts.push({ outer, holes });
  }
  const joints: CutLoop[] = [];
  const d = opts.jointHoleDiameter ?? 0;
  if (d > 0) {
    const ids = new Set(bodies.map((b) => b.id));
    const seen: Vec2[] = [];
    for (const j of scene.joints) {
      if (j.bodyId === null || !ids.has(j.bodyId)) continue;
      const p = scene.jointWorld(j);
      if (seen.some((q) => dist(p, q) < 1e-6)) continue; // two joints of exported bodies at one spot: one hole
      seen.push(p);
      joints.push({ kind: "circle", c: p, r: d / 2 });
    }
  }
  return { parts, joints, unit: scene.unit };
}

// --- shared helpers -------------------------------------------------------------

function loopBounds(loop: CutLoop, acc: { min: Vec2; max: Vec2 }): void {
  const take = (p: Vec2) => {
    acc.min = vec(Math.min(acc.min.x, p.x), Math.min(acc.min.y, p.y));
    acc.max = vec(Math.max(acc.max.x, p.x), Math.max(acc.max.y, p.y));
  };
  if (loop.kind === "circle") {
    take(vec(loop.c.x - loop.r, loop.c.y - loop.r));
    take(vec(loop.c.x + loop.r, loop.c.y + loop.r));
    return;
  }
  const n = loop.verts.length;
  for (let i = 0; i < n; i++) {
    const v = loop.verts[i];
    take(v.p);
    if (v.bulge === 0) continue;
    // A bulging edge can poke past its endpoints: include the arc's extreme points.
    const arc = arcOf(v.p, loop.verts[(i + 1) % n].p, v.bulge);
    for (const a of [0, Math.PI / 2, Math.PI, -Math.PI / 2]) {
      if (angleInSweep(a, arc.a1, arc.da)) take(add(arc.c, vec(arc.r * Math.cos(a), arc.r * Math.sin(a))));
    }
  }
}

/** Bounding box of everything in the sheet (scene units). */
export function sheetBounds(sheet: CutSheet): { min: Vec2; max: Vec2 } {
  const acc = { min: vec(Infinity, Infinity), max: vec(-Infinity, -Infinity) };
  for (const part of sheet.parts) {
    loopBounds(part.outer, acc);
    part.holes.forEach((h) => loopBounds(h, acc));
  }
  sheet.joints.forEach((j) => loopBounds(j, acc));
  if (!isFinite(acc.min.x)) return { min: vec(0, 0), max: vec(0, 0) };
  return acc;
}

/** Centre, radius, start angle and signed sweep of the arc from `p1` to `p2` with `bulge`. */
export function arcOf(p1: Vec2, p2: Vec2, bulge: number): { c: Vec2; r: number; a1: number; da: number } {
  const theta = 4 * Math.atan(bulge);
  const chord = sub(p2, p1);
  const c = dist(p1, p2);
  const r = c / (2 * Math.sin(Math.abs(theta) / 2));
  // The centre sits on the chord's perpendicular bisector, on the side the sweep turns
  // toward: rotate the chord by +90° for a positive sweep.
  const mid = scale(add(p1, p2), 0.5);
  const h = Math.sqrt(Math.max(0, r * r - (c * c) / 4)) * (Math.abs(theta) > Math.PI ? -1 : 1);
  const perp = normalize(vec(-chord.y, chord.x)); // +90° from the chord
  const centre = add(mid, scale(perp, h * Math.sign(theta)));
  const a1 = Math.atan2(p1.y - centre.y, p1.x - centre.x);
  return { c: centre, r, a1, da: theta };
}

function angleInSweep(a: number, a1: number, da: number): boolean {
  let rel = a - a1;
  const twoPi = 2 * Math.PI;
  rel = ((rel % twoPi) + twoPi) % twoPi; // 0..2π, measured the positive way
  if (da >= 0) return rel <= da;
  return twoPi - rel <= -da || rel === 0;
}

/** Compact numeric formatting (up to 6 decimals, no trailing zeros). */
function num(v: number): string {
  const s = Number(v.toFixed(6)).toString();
  return s === "-0" ? "0" : s;
}

/** Working unit → DXF `$INSUNITS` code (the same table the importer reads). */
const UNIT_TO_INSUNITS: Record<Unit, number> = { mm: 4, cm: 5, m: 6, in: 1 };

// --- DXF ------------------------------------------------------------------------

/**
 * DXF R12 (AC1009) ASCII text of the sheet in the scene's working units. Y is flipped
 * (DXF is y-up) and geometry is translated so its bounding box starts at the origin.
 */
export function toDxf(sheet: CutSheet): string {
  const b = sheetBounds(sheet);
  // Scene (x, y-down) → DXF (x − minX, maxY − y): a flip about the box's bottom edge.
  const xf = (p: Vec2): Vec2 => vec(p.x - b.min.x, b.max.y - p.y);
  const lines: string[] = [];
  const put = (code: number, value: string | number) => {
    lines.push(String(code), typeof value === "number" ? num(value) : value);
  };
  const circle = (layer: string, c: Vec2, r: number) => {
    put(0, "CIRCLE");
    put(8, layer);
    const q = xf(c);
    put(10, q.x);
    put(20, q.y);
    put(30, 0);
    put(40, r);
  };
  const ring = (layer: string, verts: BulgeVertex[]) => {
    put(0, "POLYLINE");
    put(8, layer);
    put(66, 1); // vertices follow
    put(70, 1); // closed
    put(10, 0);
    put(20, 0);
    put(30, 0);
    for (const v of verts) {
      put(0, "VERTEX");
      put(8, layer);
      const q = xf(v.p);
      put(10, q.x);
      put(20, q.y);
      put(30, 0);
      if (v.bulge !== 0) put(42, -v.bulge); // the y-flip mirrors every sweep
    }
    put(0, "SEQEND");
    put(8, layer);
  };
  const loop = (layer: string, l: CutLoop) => {
    if (l.kind === "circle") circle(layer, l.c, l.r);
    else if (l.verts.length >= 2) ring(layer, l.verts);
  };

  // HEADER
  put(0, "SECTION");
  put(2, "HEADER");
  put(9, "$ACADVER");
  put(1, "AC1009");
  put(9, "$INSUNITS");
  put(70, UNIT_TO_INSUNITS[sheet.unit]);
  put(9, "$EXTMIN");
  put(10, 0);
  put(20, 0);
  put(30, 0);
  put(9, "$EXTMAX");
  put(10, b.max.x - b.min.x);
  put(20, b.max.y - b.min.y);
  put(30, 0);
  put(0, "ENDSEC");
  // TABLES: the layers we reference.
  const layers = sheet.joints.length ? ["CUT", "JOINTS"] : ["CUT"];
  put(0, "SECTION");
  put(2, "TABLES");
  put(0, "TABLE");
  put(2, "LTYPE");
  put(70, 1);
  put(0, "LTYPE");
  put(2, "CONTINUOUS");
  put(70, 0);
  put(3, "Solid line");
  put(72, 65);
  put(73, 0);
  put(40, 0);
  put(0, "ENDTAB");
  put(0, "TABLE");
  put(2, "LAYER");
  put(70, layers.length);
  layers.forEach((name, i) => {
    put(0, "LAYER");
    put(2, name);
    put(70, 0);
    put(62, i === 0 ? 7 : 1); // white / red
    put(6, "CONTINUOUS");
  });
  put(0, "ENDTAB");
  put(0, "ENDSEC");
  // ENTITIES
  put(0, "SECTION");
  put(2, "ENTITIES");
  for (const part of sheet.parts) {
    loop("CUT", part.outer);
    part.holes.forEach((h) => loop("CUT", h));
  }
  sheet.joints.forEach((j) => loop("JOINTS", j));
  put(0, "ENDSEC");
  put(0, "EOF");
  return lines.join("\n") + "\n";
}

// --- SVG ------------------------------------------------------------------------

/**
 * SVG text of the sheet in millimetres: `width`/`height` carry the mm suffix and the
 * `viewBox` matches, so one user unit is one millimetre. Geometry is translated so the
 * bounding box starts at the origin; y stays down (SVG's own convention).
 */
export function toSvg(sheet: CutSheet): string {
  const k = UNIT_TO_MM[sheet.unit];
  const b = sheetBounds(sheet);
  const xf = (p: Vec2): Vec2 => vec((p.x - b.min.x) * k, (p.y - b.min.y) * k);
  const w = (b.max.x - b.min.x) * k;
  const h = (b.max.y - b.min.y) * k;
  const pathOf = (verts: BulgeVertex[]): string => {
    const n = verts.length;
    if (n < 2) return "";
    const parts: string[] = [];
    const p0 = xf(verts[0].p);
    parts.push(`M${num(p0.x)} ${num(p0.y)}`);
    for (let i = 0; i < n; i++) {
      const v = verts[i];
      const q = xf(verts[(i + 1) % n].p);
      if (v.bulge === 0) {
        if (i === n - 1) break; // the closing edge is implied by Z
        parts.push(`L${num(q.x)} ${num(q.y)}`);
        continue;
      }
      const theta = 4 * Math.atan(v.bulge); // sweep in the y-down frame
      const r = arcOf(v.p, verts[(i + 1) % n].p, v.bulge).r * k;
      const large = Math.abs(theta) > Math.PI ? 1 : 0;
      // SVG's sweep-flag 1 = increasing angle in the y-down frame, exactly our positive bulge.
      const sweep = theta > 0 ? 1 : 0;
      parts.push(`A${num(r)} ${num(r)} 0 ${large} ${sweep} ${num(q.x)} ${num(q.y)}`);
    }
    parts.push("Z");
    return parts.join("");
  };
  const circleOf = (c: Vec2, r: number, attrs = ""): string => {
    const q = xf(c);
    return `<circle cx="${num(q.x)}" cy="${num(q.y)}" r="${num(r * k)}"${attrs}/>`;
  };
  const out: string[] = [];
  out.push(`<?xml version="1.0" encoding="UTF-8"?>`);
  out.push(
    `<svg xmlns="http://www.w3.org/2000/svg" width="${num(w)}mm" height="${num(h)}mm" viewBox="0 0 ${num(w)} ${num(h)}" fill="none" stroke="#000000" stroke-width="0.1">`
  );
  out.push(`  <g id="bodies">`);
  sheet.parts.forEach((part, i) => {
    // A body is one path (outer + holes as subpaths) unless its outer is a circle,
    // in which case the circle and any hole paths sit together in a group.
    const holePaths = part.holes.map((h) => (h.kind === "circle" ? null : pathOf(h.verts)));
    const holeCircles = part.holes.filter((h): h is Extract<CutLoop, { kind: "circle" }> => h.kind === "circle");
    const d = [part.outer.kind === "ring" ? pathOf(part.outer.verts) : "", ...holePaths.filter((s): s is string => !!s)].join(" ").trim();
    if (part.outer.kind === "circle" || holeCircles.length) {
      out.push(`    <g id="body-${i + 1}">`);
      if (part.outer.kind === "circle") out.push(`      ${circleOf(part.outer.c, part.outer.r)}`);
      if (d) out.push(`      <path fill-rule="evenodd" d="${d}"/>`);
      holeCircles.forEach((c) => out.push(`      ${circleOf(c.c, c.r)}`));
      out.push(`    </g>`);
    } else {
      out.push(`    <path id="body-${i + 1}" fill-rule="evenodd" d="${d}"/>`);
    }
  });
  out.push(`  </g>`);
  if (sheet.joints.length) {
    out.push(`  <g id="joints">`);
    sheet.joints.forEach((j) => {
      if (j.kind === "circle") out.push(`    ${circleOf(j.c, j.r)}`);
    });
    out.push(`  </g>`);
  }
  out.push(`</svg>`);
  return out.join("\n") + "\n";
}
