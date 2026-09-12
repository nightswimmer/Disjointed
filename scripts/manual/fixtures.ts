/**
 * Fixture mechanisms for the manual's illustrations, built with the Scene API so they
 * never go stale with the file format (the generator serialises them fresh each run).
 * Coordinates are world units (screen y points down).
 */
import { Scene, SceneData } from "../../src/model";
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

/** A rectangular block centred on `c`. */
function block(scene: Scene, c: Vec2, w: number, h: number, radius = 3, color?: string) {
  const body = scene.addBody(
    [v(c.x - w / 2, c.y - h / 2), v(c.x + w / 2, c.y - h / 2), v(c.x + w / 2, c.y + h / 2), v(c.x - w / 2, c.y + h / 2)],
    radius,
    "fillet"
  );
  if (color) body.color = color;
  return body;
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
  const body = scene.addBody(
    [v(0, 0), v(160, 0), v(160, 100), v(0, 100)],
    12,
    "fillet",
    [
      { control: [v(40, 50)], radius: 12, round: "offset" },
      { control: [v(120, 50)], radius: 12, round: "offset" },
    ]
  );
  body.color = "#4f9dff";
  return { data: scene.serialize(), joints: {}, bodies: { plate: body.id } };
}

/**
 * A legend of canvas elements in a row: a free joint, a body with a plain joint, a pinned
 * pair, a grounded joint, a welded pair; below, a guideline, a dimension and a constraint badge.
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
  // Second row: a guideline through two points, a dimension between two free joints, and
  // a horizontal constraint badge on a block's top edge.
  scene.addGuide(v(0, 45), v(120, 45));
  const dA = scene.addFreeJoint(v(200, 80));
  const dB = scene.addFreeJoint(v(300, 80));
  scene.addMeasurement("draw", { kind: "joint", jointId: dA.id }, { kind: "joint", jointId: dB.id }, v(250, 60));
  const hz = block(scene, v(410, 80), 60, 30, 4, "#5bd6a6");
  scene.addSketchConstraint("horizontal", { kind: "edge", bodyId: hz.id, index: 0 });
  return {
    data: scene.serialize(),
    joints: { free: free.id, plain: jPlain.id, pin: p1.id, ground: jG.id, weld: w1.id, dimA: dA.id, dimB: dB.id },
    bodies: { plain: plain.id, barA: barA.id, barB: barB.id, grounded: grounded.id, weldA: wA.id, weldB: wB.id, horizontal: hz.id },
  };
}

export const FIXTURES: Record<string, () => Fixture> = { fourBar, sliderCrank, plate, legend };
