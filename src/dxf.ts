/**
 * Minimal ASCII DXF reader for importing flat 2D outlines as bodies.
 *
 * Scope (v1): the ENTITIES section only — LWPOLYLINE, POLYLINE/VERTEX (2D),
 * LINE, ARC and CIRCLE. Closed polylines and circles become loops directly;
 * open polylines, lines and arcs are chained end-to-end (within a tolerance)
 * into closed loops. Arcs and vertex bulges are sampled into polyline points,
 * since bodies are control polygons. Everything else (INSERT, SPLINE, TEXT,
 * dimensions, 3D meshes, ...) is counted and skipped.
 *
 * Coordinates are returned exactly as stored in the file (DXF is y-up; the
 * importer flips and scales them). `$INSUNITS` from the HEADER section is
 * reported as millimetres-per-drawing-unit so the importer can convert into
 * the document's working units.
 */
import { Vec2, vec, dist, pointInPolygon } from "./geometry";

export interface DxfResult {
  /** Closed loops (≥ 3 points, no closing duplicate), in raw DXF coordinates (y-up). */
  loops: Vec2[][];
  /** Millimetres per drawing unit from `$INSUNITS`, or null when absent / unitless / unknown. */
  unitToMm: number | null;
  /** Open paths that could not be chained into a closed loop (dropped). */
  skippedPaths: number;
  /** Entity records in ENTITIES this reader does not understand (dropped). */
  skippedEntities: number;
}

/** `$INSUNITS` code → millimetres per drawing unit (unlisted codes are treated as unitless). */
const INSUNITS_TO_MM: Record<number, number> = {
  1: 25.4, // inches
  2: 304.8, // feet
  3: 1609344, // miles
  4: 1, // millimetres
  5: 10, // centimetres
  6: 1000, // metres
  7: 1e6, // kilometres
  10: 914.4, // yards
  13: 1e-3, // microns
  14: 100, // decimetres
};

/** Max sweep per sampled arc segment: a full circle becomes 48 polygon edges. */
const ARC_SEG = (2 * Math.PI) / 48;

interface Pair {
  code: number;
  value: string;
}

/** A polyline vertex as stored: its point plus the bulge of the edge leaving it. */
interface BulgeVertex {
  p: Vec2;
  bulge: number;
}

export function parseDxf(text: string): DxfResult {
  if (text.startsWith("AutoCAD Binary DXF")) {
    throw new Error("Binary DXF files are not supported — re-export as ASCII DXF.");
  }
  const ps = toPairs(text);
  if (!ps.some((p) => p.code === 0 && p.value === "SECTION")) {
    throw new Error("Not a DXF file (no SECTION records found).");
  }

  let unitToMm: number | null = null;
  const loops: Vec2[][] = [];
  const open: Vec2[][] = [];
  let skippedEntities = 0;

  let section = "";
  let i = 0;
  while (i < ps.length) {
    const p = ps[i];
    if (p.code === 0 && p.value === "SECTION") {
      section = ps[i + 1]?.code === 2 ? ps[i + 1].value : "";
      i += 2;
      continue;
    }
    if (p.code === 0 && (p.value === "ENDSEC" || p.value === "EOF")) {
      section = "";
      i++;
      continue;
    }
    if (section === "HEADER" && p.code === 9 && p.value === "$INSUNITS") {
      for (let j = i + 1; j < ps.length && ps[j].code !== 9 && ps[j].code !== 0; j++) {
        if (ps[j].code === 70) {
          unitToMm = INSUNITS_TO_MM[parseInt(ps[j].value, 10)] ?? null;
          break;
        }
      }
      i++;
      continue;
    }
    if (section === "ENTITIES" && p.code === 0) {
      i = parseEntity(ps, i, loops, open, () => skippedEntities++);
      continue;
    }
    i++;
  }

  const chained = chainPaths(open);
  const all = [...loops, ...chained.closed];
  const cleaned: Vec2[][] = [];
  let degenerate = 0;
  for (const loop of all) {
    const c = cleanLoop(loop);
    if (c) cleaned.push(c);
    else degenerate++;
  }
  return {
    loops: cleaned,
    unitToMm,
    skippedPaths: chained.skippedPaths + degenerate,
    skippedEntities,
  };
}

/** Split the file into (group code, value) pairs; DXF is strictly line-paired. */
function toPairs(text: string): Pair[] {
  const lines = text.split(/\r\n|\r|\n/);
  const out: Pair[] = [];
  for (let i = 0; i + 1 < lines.length; i += 2) {
    const code = parseInt(lines[i].trim(), 10);
    if (Number.isNaN(code)) continue;
    out.push({ code, value: lines[i + 1].trim() });
  }
  return out;
}

/**
 * Parse one entity starting at the `0/<TYPE>` pair at `i`; append its geometry to
 * `loops` (already closed) or `open` (paths for chaining). Returns the index of the
 * next unconsumed pair (a POLYLINE consumes its VERTEX/SEQEND sub-entities too).
 */
function parseEntity(
  ps: Pair[],
  i: number,
  loops: Vec2[][],
  open: Vec2[][],
  skip: () => void
): number {
  const type = ps[i].value;
  // Collect this entity's own pairs (up to the next 0 code).
  let end = i + 1;
  while (end < ps.length && ps[end].code !== 0) end++;
  const own = ps.slice(i + 1, end);
  const num = (code: number): number | null => {
    const p = own.find((q) => q.code === code);
    if (!p) return null;
    const v = parseFloat(p.value);
    return Number.isNaN(v) ? null : v;
  };

  switch (type) {
    case "LWPOLYLINE": {
      const verts: BulgeVertex[] = [];
      let flags = 0;
      for (const q of own) {
        if (q.code === 70) flags = parseInt(q.value, 10) || 0;
        else if (q.code === 10) verts.push({ p: vec(parseFloat(q.value), 0), bulge: 0 });
        else if (q.code === 20 && verts.length) verts[verts.length - 1].p.y = parseFloat(q.value);
        else if (q.code === 42 && verts.length) verts[verts.length - 1].bulge = parseFloat(q.value);
      }
      emitPolyline(verts, (flags & 1) !== 0, loops, open);
      return end;
    }
    case "POLYLINE": {
      const flags = (() => {
        const p = own.find((q) => q.code === 70);
        return p ? parseInt(p.value, 10) || 0 : 0;
      })();
      // Consume the VERTEX sub-entities up to SEQEND regardless, so an unsupported
      // 3D/mesh polyline is skipped as one unit.
      const verts: BulgeVertex[] = [];
      let j = end;
      while (j < ps.length && !(ps[j].code === 0 && ps[j].value !== "VERTEX")) {
        if (ps[j].code === 0 && ps[j].value === "VERTEX") {
          let ve = j + 1;
          while (ve < ps.length && ps[ve].code !== 0) ve++;
          const vp = ps.slice(j + 1, ve);
          const get = (code: number): number => {
            const q = vp.find((r) => r.code === code);
            return q ? parseFloat(q.value) || 0 : 0;
          };
          const vflags = (() => {
            const q = vp.find((r) => r.code === 70);
            return q ? parseInt(q.value, 10) || 0 : 0;
          })();
          // Skip spline-frame control points (bit 16); keep ordinary / fitted vertices.
          if ((vflags & 16) === 0) verts.push({ p: vec(get(10), get(20)), bulge: get(42) });
          j = ve;
        } else {
          j++;
        }
      }
      if (j < ps.length && ps[j].code === 0 && ps[j].value === "SEQEND") {
        let se = j + 1;
        while (se < ps.length && ps[se].code !== 0) se++;
        j = se;
      }
      if ((flags & (16 | 64)) !== 0) skip(); // 3D polygon mesh / polyface mesh
      else emitPolyline(verts, (flags & 1) !== 0, loops, open);
      return j;
    }
    case "LINE": {
      const x1 = num(10), y1 = num(20), x2 = num(11), y2 = num(21);
      if (x1 !== null && y1 !== null && x2 !== null && y2 !== null) {
        const a = vec(x1, y1);
        const b = vec(x2, y2);
        if (dist(a, b) > 0) open.push([a, b]);
      }
      return end;
    }
    case "CIRCLE": {
      const cx = num(10), cy = num(20), r = num(40);
      if (cx !== null && cy !== null && r !== null && r > 0) {
        const n = Math.ceil((2 * Math.PI) / ARC_SEG);
        const pts: Vec2[] = [];
        for (let k = 0; k < n; k++) {
          const a = (k / n) * 2 * Math.PI;
          pts.push(vec(cx + r * Math.cos(a), cy + r * Math.sin(a)));
        }
        loops.push(pts);
      }
      return end;
    }
    case "ARC": {
      const cx = num(10), cy = num(20), r = num(40), a0d = num(50), a1d = num(51);
      if (cx !== null && cy !== null && r !== null && r > 0 && a0d !== null && a1d !== null) {
        const a0 = (a0d * Math.PI) / 180;
        let sweep = ((a1d - a0d) * Math.PI) / 180;
        while (sweep <= 0) sweep += 2 * Math.PI; // DXF arcs always run CCW start → end
        const n = Math.max(1, Math.ceil(sweep / ARC_SEG));
        const pts: Vec2[] = [];
        for (let k = 0; k <= n; k++) {
          const a = a0 + (sweep * k) / n;
          pts.push(vec(cx + r * Math.cos(a), cy + r * Math.sin(a)));
        }
        open.push(pts);
      }
      return end;
    }
    default:
      skip();
      return end;
  }
}

/** Expand a polyline's bulges into sampled points and file it as a loop or an open path. */
function emitPolyline(verts: BulgeVertex[], closed: boolean, loops: Vec2[][], open: Vec2[][]): void {
  if (verts.length < 2) return;
  // A "closed" flag missing but last point on first point is treated as closed too.
  const n = verts.length;
  const effectiveClosed =
    closed || (n >= 4 && dist(verts[0].p, verts[n - 1].p) === 0);
  const pts: Vec2[] = [];
  const last = effectiveClosed ? n : n - 1;
  for (let k = 0; k < n; k++) {
    pts.push(verts[k].p);
    if (k < last) {
      const next = verts[(k + 1) % n].p;
      if (verts[k].bulge !== 0) sampleBulge(verts[k].p, next, verts[k].bulge, pts);
    }
  }
  if (effectiveClosed) loops.push(pts);
  else open.push(pts);
}

/**
 * Append the intermediate points of the bulge arc from `p1` to `p2` (endpoints excluded —
 * the caller adds them). Bulge = tan(sweep/4); positive runs CCW (in DXF's y-up frame).
 */
function sampleBulge(p1: Vec2, p2: Vec2, bulge: number, out: Vec2[]): void {
  const theta = 4 * Math.atan(bulge);
  const c = dist(p1, p2);
  if (c < 1e-12 || Math.abs(theta) < 1e-9) return;
  const r = c / (2 * Math.sin(Math.abs(theta) / 2));
  // Centre sits at h = (c/2)/tan(θ/2) along the chord's CCW normal from its midpoint
  // (the sign of tan places arcs > 180° on the far side).
  const nx = -(p2.y - p1.y) / c;
  const ny = (p2.x - p1.x) / c;
  const h = c / (2 * Math.tan(theta / 2));
  const cx = (p1.x + p2.x) / 2 + nx * h;
  const cy = (p1.y + p2.y) / 2 + ny * h;
  const a0 = Math.atan2(p1.y - cy, p1.x - cx);
  const n = Math.max(1, Math.ceil(Math.abs(theta) / ARC_SEG));
  for (let k = 1; k < n; k++) {
    const a = a0 + (theta * k) / n;
    out.push(vec(cx + r * Math.cos(a), cy + r * Math.sin(a)));
  }
}

/**
 * Greedily chain open paths end-to-end (either direction) into closed loops. The join
 * tolerance scales with the drawing's extent, so exact CAD exports and lightly-sloppy
 * ones both chain, while genuinely separate geometry doesn't.
 */
function chainPaths(open: Vec2[][]): { closed: Vec2[][]; skippedPaths: number } {
  const pool = open.filter((p) => p.length >= 2);
  let skipped = open.length - pool.length;
  const closed: Vec2[][] = [];
  if (!pool.length) return { closed, skippedPaths: skipped };

  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const path of pool)
    for (const p of path) {
      minX = Math.min(minX, p.x); minY = Math.min(minY, p.y);
      maxX = Math.max(maxX, p.x); maxY = Math.max(maxY, p.y);
    }
  const tol = Math.max(1e-6, Math.hypot(maxX - minX, maxY - minY) * 1e-4);

  const work = pool.map((p) => p.slice());
  while (work.length) {
    const cur = work.pop()!;
    for (;;) {
      if (cur.length >= 4 && dist(cur[0], cur[cur.length - 1]) <= tol) {
        cur.pop(); // drop the closing duplicate
        closed.push(cur);
        break;
      }
      const endPt = cur[cur.length - 1];
      let found = -1;
      let reversed = false;
      for (let k = 0; k < work.length; k++) {
        if (dist(work[k][0], endPt) <= tol) { found = k; break; }
        if (dist(work[k][work[k].length - 1], endPt) <= tol) { found = k; reversed = true; break; }
      }
      if (found < 0) {
        skipped++;
        break;
      }
      const next = work.splice(found, 1)[0];
      if (reversed) next.reverse();
      cur.push(...next.slice(1));
    }
  }
  return { closed, skippedPaths: skipped };
}

/** One importable solid: an outer outline plus the hole loops cut out of it. */
export interface NestedSolid {
  outer: Vec2[];
  holes: Vec2[][];
}

/**
 * Nest closed loops by containment, even-odd style: a loop inside another is a hole
 * of it; a loop inside a hole is a new solid (an island), and so on. Assumes loops
 * don't intersect each other (true of sane CAD exports) — containment is tested with
 * one representative vertex, and a loop's parent is its smallest enclosing loop.
 */
export function nestLoops(loops: Vec2[][]): NestedSolid[] {
  const areas = loops.map((l) => Math.abs(loopSignedArea(l)));
  const depth: number[] = [];
  const parent: number[] = [];
  for (let i = 0; i < loops.length; i++) {
    let d = 0;
    let par = -1;
    let parArea = Infinity;
    for (let j = 0; j < loops.length; j++) {
      if (j === i || areas[j] <= areas[i]) continue; // a container is strictly bigger
      if (!pointInPolygon(loops[i][0], loops[j])) continue;
      d++;
      if (areas[j] < parArea) {
        parArea = areas[j];
        par = j;
      }
    }
    depth.push(d);
    parent.push(par);
  }
  const solids = new Map<number, NestedSolid>();
  loops.forEach((l, i) => {
    if (depth[i] % 2 === 0) solids.set(i, { outer: l, holes: [] });
  });
  loops.forEach((l, i) => {
    if (depth[i] % 2 === 0) return;
    // Proper nesting puts a hole's parent at even depth; walk up just in case.
    let p = parent[i];
    while (p >= 0 && depth[p] % 2 === 1) p = parent[p];
    const s = p >= 0 ? solids.get(p) : undefined;
    if (s) s.holes.push(l);
  });
  return [...solids.values()];
}

/** Shoelace signed area (positive = CCW in a y-up frame). */
export function loopSignedArea(pts: Vec2[]): number {
  let a = 0;
  for (let i = 0; i < pts.length; i++) {
    const p = pts[i];
    const q = pts[(i + 1) % pts.length];
    a += p.x * q.y - q.x * p.y;
  }
  return a / 2;
}

/** Drop consecutive duplicates + a closing duplicate; reject loops with no real area. */
function cleanLoop(pts: Vec2[]): Vec2[] | null {
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const p of pts) {
    minX = Math.min(minX, p.x); minY = Math.min(minY, p.y);
    maxX = Math.max(maxX, p.x); maxY = Math.max(maxY, p.y);
  }
  const diag = Math.hypot(maxX - minX, maxY - minY);
  if (!(diag > 0)) return null;
  const tol = diag * 1e-9;
  const out: Vec2[] = [];
  for (const p of pts) {
    if (!out.length || dist(out[out.length - 1], p) > tol) out.push(vec(p.x, p.y));
  }
  while (out.length > 1 && dist(out[0], out[out.length - 1]) <= tol) out.pop();
  if (out.length < 3) return null;
  if (Math.abs(loopSignedArea(out)) < diag * diag * 1e-10) return null;
  return out;
}
