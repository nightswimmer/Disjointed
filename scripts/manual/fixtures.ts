/**
 * Fixture mechanisms for the manual's illustrations, built with the Scene API so they
 * never go stale with the file format (the generator serialises them fresh each run).
 * Coordinates are world units (screen y points down, so negative y is "up").
 *
 * Constraints added here are not solved - the geometry is drawn already satisfied, so
 * every fixture must place its elements exactly where its constraints want them.
 */
import { Scene, SceneData, MeasureRef, PatternSeed, BooleanOp } from "../../src/model";
import { Vec2 } from "../../src/geometry";

const v = (x: number, y: number): Vec2 => ({ x, y });

/** A link: a rounded bar of width `w` whose ends are centred on `a` and `b`. */
function bar(scene: Scene, a: Vec2, b: Vec2, w = 16, color?: string) {
  const dx = b.x - a.x, dy = b.y - a.y;
  const l = Math.hypot(dx, dy) || 1;
  const nx = (-dy / l) * (w / 2), ny = (dx / l) * (w / 2); // half-width normal
  const ex = (dx / l) * (w / 2), ey = (dy / l) * (w / 2); // half-width end extension
  const body = scene.addBody(
    [
      v(a.x - ex + nx, a.y - ey + ny),
      v(b.x + ex + nx, b.y + ey + ny),
      v(b.x + ex - nx, b.y + ey - ny),
      v(a.x - ex - nx, a.y - ey - ny),
    ],
    w / 2,
    "fillet"
  );
  if (color) body.color = color;
  return body;
}

/** A rectangular block centred on `c`. Corner 0 is the top-left, edges run clockwise on screen. */
function block(scene: Scene, c: Vec2, w: number, h: number, radius = 3, color?: string) {
  const body = scene.addBody(
    [v(c.x - w / 2, c.y - h / 2), v(c.x + w / 2, c.y - h / 2), v(c.x + w / 2, c.y + h / 2), v(c.x - w / 2, c.y + h / 2)],
    radius,
    "fillet"
  );
  if (color) body.color = color;
  return body;
}

/** A disk body: a one-point offset outline (what the Circle tool draws in the Body role). */
function disk(scene: Scene, c: Vec2, r: number, color?: string) {
  const body = scene.addBody([c], r, "offset");
  if (color) body.color = color;
  return body;
}

/** A plate whose top-left corner is at `o`; corner 0 top-left, edge 0 the top edge. */
function plateAt(scene: Scene, o: Vec2, w: number, h: number, radius: number, color: string, holes?: Parameters<Scene["addBody"]>[3]) {
  const body = scene.addBody([o, v(o.x + w, o.y), v(o.x + w, o.y + h), v(o.x, o.y + h)], radius, "fillet", holes);
  body.color = color;
  return body;
}

const vtx = (bodyId: number, index: number, hole?: number): MeasureRef =>
  hole === undefined ? { kind: "vertex", bodyId, index } : { kind: "vertex", bodyId, index, hole };
const edge = (bodyId: number, index: number): MeasureRef => ({ kind: "edge", bodyId, index });
const guideLine = (guideId: number): MeasureRef => ({ kind: "guideLine", guideId, edge: 0 });
const guidePoint = (guideId: number, which: string): MeasureRef => ({ kind: "guidePoint", guideId, which });

function must<T>(x: T | null | undefined, what: string): T {
  if (x === null || x === undefined) throw new Error(`fixture: ${what} was refused`);
  return x;
}

/** Ids the shots may want to point the mouse at. */
export interface Fixture {
  data: SceneData;
  /** Named joints (world ids) for scripted gestures. */
  joints: Record<string, number>;
  bodies: Record<string, number>;
}

/** A four-bar linkage: crank, coupler, rocker between two grounded pivots. */
export function fourBar(): Fixture {
  const scene = new Scene();
  const A = v(0, 0), B = v(30, -70), C = v(150, -120), D = v(200, 0);
  const crank = bar(scene, A, B, 16, "#ff4dac");
  const coupler = bar(scene, B, C, 14, "#4f9dff");
  const rocker = bar(scene, C, D, 16, "#5bd6a6");
  const jA = scene.addJoint(crank.id, A);
  const jB1 = scene.addJoint(crank.id, B);
  const jB2 = scene.addJoint(coupler.id, B);
  const jC1 = scene.addJoint(coupler.id, C);
  const jC2 = scene.addJoint(rocker.id, C);
  const jD = scene.addJoint(rocker.id, D);
  scene.addPin(jB1.id, jB2.id);
  scene.addPin(jC1.id, jC2.id);
  scene.addGround(jA.id, A);
  scene.addGround(jD.id, D);
  return {
    data: scene.serialize(),
    joints: { A: jA.id, B: jB1.id, C: jC1.id, D: jD.id },
    bodies: { crank: crank.id, coupler: coupler.id, rocker: rocker.id },
  };
}

/** A slider-crank: a crank and connecting rod driving a block on a fixed track. */
export function sliderCrank(): Fixture {
  const scene = new Scene();
  const O = v(0, 0), P = v(35, -35), Q = v(150, 0);
  const crank = bar(scene, O, P, 16, "#ff4dac");
  const rod = bar(scene, P, Q, 14, "#4f9dff");
  const piston = block(scene, v(150, 0), 50, 30, 4, "#f0b441");
  const jO = scene.addJoint(crank.id, O);
  const jP1 = scene.addJoint(crank.id, P);
  const jP2 = scene.addJoint(rod.id, P);
  const jQ1 = scene.addJoint(rod.id, Q);
  const jQ2 = scene.addJoint(piston.id, Q);
  scene.addPin(jP1.id, jP2.id);
  scene.addPin(jQ1.id, jQ2.id);
  scene.addGround(jO.id, O);
  const made = scene.createSlider(piston.id, v(100, 0), v(230, 0))!;
  return {
    data: scene.serialize(),
    joints: { O: jO.id, P: jP1.id, Q: jQ1.id, rider: made.rider.id },
    bodies: { crank: crank.id, rod: rod.id, piston: piston.id },
  };
}

/** A lone plate with a couple of holes: for tools that work on one body. */
export function plate(): Fixture {
  const scene = new Scene();
  const body = plateAt(scene, v(0, 0), 160, 100, 12, "#4f9dff", [
    { control: [v(40, 50)], radius: 12, round: "offset" },
    { control: [v(120, 50)], radius: 12, round: "offset" },
  ]);
  return { data: scene.serialize(), joints: {}, bodies: { plate: body.id } };
}

/**
 * A legend of canvas elements in a row: a free joint, a body with a plain joint, a pinned
 * pair, a grounded joint, a welded pair; below, a reference line, a dimension and a
 * constraint badge.
 */
export function legend(): Fixture {
  const scene = new Scene();
  const free = scene.addFreeJoint(v(0, 0));
  const plain = block(scene, v(70, 0), 50, 30, 4, "#4f9dff");
  const jPlain = scene.addJoint(plain.id, v(70, 0));
  const barA = bar(scene, v(140, 10), v(200, -15), 14, "#ff4dac");
  const barB = bar(scene, v(200, -15), v(260, 10), 14, "#5bd6a6");
  const p1 = scene.addJoint(barA.id, v(200, -15));
  const p2 = scene.addJoint(barB.id, v(200, -15));
  scene.addPin(p1.id, p2.id);
  const grounded = block(scene, v(330, 0), 50, 30, 4, "#f0b441");
  const jG = scene.addJoint(grounded.id, v(330, 0));
  scene.addGround(jG.id, v(330, 0));
  const wA = block(scene, v(410, -8), 44, 28, 4, "#4f9dff");
  const wB = block(scene, v(432, 8), 44, 28, 4, "#ff4dac");
  const w1 = scene.addJoint(wA.id, v(421, 0));
  const w2 = scene.addJoint(wB.id, v(421, 0));
  scene.addPin(w1.id, w2.id, true);
  // Second row: a reference segment, a dimension between two free joints (its label
  // above the pair makes it a horizontal one), and a horizontal-constraint badge on a
  // block's top edge.
  scene.addGuidePoly([v(0, 45), v(120, 45)], false);
  const dA = scene.addFreeJoint(v(200, 80));
  const dB = scene.addFreeJoint(v(300, 80));
  scene.addMeasurement("draw", { kind: "joint", jointId: dA.id }, { kind: "joint", jointId: dB.id }, v(250, 60));
  const hz = block(scene, v(410, 80), 60, 30, 4, "#5bd6a6");
  scene.addSketchConstraint("horizontal", edge(hz.id, 0));
  return {
    data: scene.serialize(),
    joints: { free: free.id, plain: jPlain.id, pin: p1.id, ground: jG.id, weld: w1.id, dimA: dA.id, dimB: dB.id },
    bodies: { plain: plain.id, barA: barA.id, barB: barB.id, grounded: grounded.id, weldA: wA.id, weldB: wB.id, horizontal: hz.id },
  };
}

/** A regular hexagon body (selected by the shot: handles, centre crosshair, "6 sides" tag). */
export function polygon(): Fixture {
  const scene = new Scene();
  const n = 6, R = 60;
  const pts = Array.from({ length: n }, (_, i) => {
    const a = -Math.PI / 2 + (i * 2 * Math.PI) / n;
    return v(R * Math.cos(a), R * Math.sin(a));
  });
  const body = scene.addBody(pts, 0, "fillet", undefined, undefined, n);
  body.color = "#5bd6a6";
  return { data: scene.serialize(), joints: {}, bodies: { hex: body.id } };
}

/**
 * Point-point dimensions between a plate's corners: horizontal (label above), vertical
 * (label beside), direct (label in a diagonal zone), and a horizontal one whose label was
 * dragged past the end of its line, so it gets a leader.
 */
export function dimensions(): Fixture {
  const scene = new Scene();
  const p = plateAt(scene, v(0, 0), 300, 120, 10, "#4f9dff");
  must(scene.addMeasurement("draw", vtx(p.id, 0), vtx(p.id, 1), v(150, -45)), "horizontal dimension");
  must(scene.addMeasurement("draw", vtx(p.id, 1), vtx(p.id, 2), v(350, 60)), "vertical dimension");
  must(scene.addMeasurement("draw", vtx(p.id, 0), vtx(p.id, 2), v(345, -40)), "direct dimension");
  const m = must(scene.addMeasurement("draw", vtx(p.id, 3), vtx(p.id, 2), v(150, 165)), "bottom dimension");
  scene.setMeasurementLabel(m.id, v(400, 165));
  return { data: scene.serialize(), joints: {}, bodies: { plate: p.id } };
}

/** A diameter dimension on a round hole and a radius dimension on a rounded corner. */
export function sizes(): Fixture {
  const scene = new Scene();
  const p = plateAt(scene, v(0, 0), 160, 100, 14, "#4f9dff", [{ control: [v(48, 50)], radius: 14, round: "offset" }]);
  must(scene.addMeasurement("draw", vtx(p.id, 0, 0), vtx(p.id, 0, 0), v(48, -40)), "diameter dimension");
  must(scene.addMeasurement("draw", vtx(p.id, 1), vtx(p.id, 1), v(205, -30)), "radius dimension");
  return { data: scene.serialize(), joints: {}, bodies: { plate: p.id } };
}

/** A bar with a locked corner and a locked reference line (padlock badges). */
export function fixedFx(): Fixture {
  const scene = new Scene();
  const b = block(scene, v(70, 0), 140, 40, 6, "#4f9dff");
  must(scene.addSketchConstraint("fixed", vtx(b.id, 0)), "fixed corner");
  const line = must(scene.addGuidePoly([v(-10, 60), v(190, 60)], false), "reference line");
  must(scene.addSketchConstraint("fixed", guideLine(line.id)), "fixed line");
  return { data: scene.serialize(), joints: {}, bodies: { bar: b.id } };
}

/** Two disks symmetrical about a vertical reference line. */
export function symmetricFx(): Fixture {
  const scene = new Scene();
  const mirror = must(scene.addGuidePoly([v(100, -70), v(100, 70)], false), "mirror line");
  const a = disk(scene, v(40, 0), 22, "#ff4dac");
  const b = disk(scene, v(160, 0), 22, "#ff4dac");
  must(scene.addSketchConstraint("symmetric", vtx(a.id, 0), vtx(b.id, 0), guideLine(mirror.id)), "symmetric");
  return { data: scene.serialize(), joints: {}, bodies: { a: a.id, b: b.id } };
}

/** A disk tangent to the top edge of a plate. */
export function tangentFx(): Fixture {
  const scene = new Scene();
  const p = plateAt(scene, v(0, 0), 200, 60, 6, "#4f9dff");
  const d = disk(scene, v(70, -30), 30, "#ff4dac");
  must(scene.addSketchConstraint("tangent", { kind: "disk", bodyId: d.id }, edge(p.id, 0)), "tangent");
  return { data: scene.serialize(), joints: {}, bodies: { plate: p.id, disk: d.id } };
}

/** Two reference lines blending into a reference arc: tangent + coincident at each end. */
export function blendFx(): Fixture {
  const scene = new Scene();
  const arc = must(scene.addGuideArc(v(-60, 0), v(0, -60), v(60, 0)), "arc");
  const left = must(scene.addGuidePoly([v(-60, 0), v(-60, 110)], false), "left line");
  const right = must(scene.addGuidePoly([v(60, 0), v(60, 110)], false), "right line");
  const circle: MeasureRef = { kind: "guideCircle", guideId: arc.id };
  must(scene.addSketchConstraint("tangent", circle, guideLine(left.id)), "left tangent");
  must(scene.addSketchConstraint("tangent", circle, guideLine(right.id)), "right tangent");
  must(scene.addSketchConstraint("coincident", guidePoint(arc.id, "a"), guidePoint(left.id, "0")), "left blend");
  must(scene.addSketchConstraint("coincident", guidePoint(arc.id, "b"), guidePoint(right.id, "0")), "right blend");
  return { data: scene.serialize(), joints: {}, bodies: {} };
}

/** A two-bar group (an L-shaped compound link) next to a plate that is not in it. */
export function groupFx(): Fixture {
  const scene = new Scene();
  const barA = bar(scene, v(0, 0), v(120, 0), 18, "#ff4dac");
  const barB = bar(scene, v(120, 0), v(120, -90), 18, "#f0b441");
  const jA = scene.addJoint(barA.id, v(0, 0));
  const jB = scene.addJoint(barB.id, v(120, -90));
  must(scene.addGroup([barA.id, barB.id]), "group");
  const outside = block(scene, v(240, -45), 90, 70, 6, "#4f9dff");
  const jO = scene.addJoint(outside.id, v(240, -45));
  scene.addGround(jO.id, v(240, -45));
  return {
    data: scene.serialize(),
    joints: { a: jA.id, b: jB.id, outside: jO.id },
    bodies: { barA: barA.id, barB: barB.id, outside: outside.id },
  };
}

/** A plate with a two-seed linear pattern: a round hole and a smaller one, repeated four times. */
export function patternLinearFx(): Fixture {
  const scene = new Scene();
  const p = plateAt(scene, v(0, 0), 260, 100, 10, "#4f9dff", [
    { control: [v(35, 36)], radius: 9, round: "offset" },
    { control: [v(35, 70)], radius: 5, round: "offset" },
  ]);
  const seeds: PatternSeed[] = [
    { kind: "hole", bodyId: p.id, hole: 0 },
    { kind: "hole", bodyId: p.id, hole: 1 },
  ];
  must(scene.createLinearPattern(seeds, v(95, 36), 4), "linear pattern");
  return { data: scene.serialize(), joints: {}, bodies: { plate: p.id } };
}

/** A flange with a bolt hole repeated six times around its centre bore. */
export function patternCircularFx(): Fixture {
  const scene = new Scene();
  const p = plateAt(scene, v(-90, -90), 180, 180, 40, "#5bd6a6", [
    { control: [v(0, 0)], radius: 22, round: "offset" },
    { control: [v(58, 0)], radius: 7, round: "offset" },
  ]);
  must(scene.createCircularPattern({ kind: "hole", bodyId: p.id, hole: 1 }, v(0, 0), 6), "circular pattern");
  return { data: scene.serialize(), joints: {}, bodies: { flange: p.id } };
}

/**
 * The Boolean examples: a plate (selected first), a disk lying inside it and a disk
 * crossing its right edge - as drawn (`null`), after Subtract, or after Intersect (of
 * the plate and the crossing disk; the inner disk is dropped first, since it shares no
 * area with the crossing one).
 */
export function booleanFx(op: BooleanOp | null): Fixture {
  const scene = new Scene();
  const p = plateAt(scene, v(0, 0), 180, 110, 10, "#4f9dff");
  const inner = disk(scene, v(60, 55), 22, "#ff4dac");
  const outer = disk(scene, v(180, 55), 34, "#f0b441");
  if (op === "subtract") {
    const r = scene.booleanBodies("subtract", [p.id, inner.id, outer.id]);
    if (!r.ok) throw new Error(`fixture: subtract refused - ${r.reason}`);
  } else if (op === "intersect") {
    scene.removeBody(inner.id);
    const r = scene.booleanBodies("intersect", [p.id, outer.id]);
    if (!r.ok) throw new Error(`fixture: intersect refused - ${r.reason}`);
  }
  return { data: scene.serialize(), joints: {}, bodies: { plate: p.id, inner: inner.id, outer: outer.id } };
}

/** A plate and a loose free joint: the shot drags the joint into an alignment with a corner. */
export function implicitFx(): Fixture {
  const scene = new Scene();
  const p = plateAt(scene, v(0, 0), 160, 100, 8, "#4f9dff");
  const j = scene.addFreeJoint(v(240, -70));
  return { data: scene.serialize(), joints: { free: j.id }, bodies: { plate: p.id } };
}

export const FIXTURES: Record<string, () => Fixture> = {
  fourBar,
  sliderCrank,
  plate,
  legend,
  polygon,
  dimensions,
  sizes,
  fixed: fixedFx,
  symmetric: symmetricFx,
  tangent: tangentFx,
  blend: blendFx,
  group: groupFx,
  patternLinear: patternLinearFx,
  patternCircular: patternCircularFx,
  booleanBefore: () => booleanFx(null),
  booleanSubtract: () => booleanFx("subtract"),
  booleanIntersect: () => booleanFx("intersect"),
  implicit: implicitFx,
};
