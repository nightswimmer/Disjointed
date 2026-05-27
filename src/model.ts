/**
 * Scene data model: rigid bodies (polygons), joint points attached to bodies,
 * and the constraints that couple them. All world geometry is derived from a
 * body's pose (centroid position + angle) plus each joint's local offset.
 */
import {
  Vec2,
  add,
  rotate,
  vec,
  sub,
  dist,
  polygonCentroid,
  polygonArea,
  polygonInertiaAboutCentroid,
  pointInPolygon,
} from "./geometry";

export interface Body {
  id: number;
  /** Polygon vertices relative to the centroid, in the body's local frame. */
  local: Vec2[];
  /** World position of the centroid (the body's local origin). */
  pos: Vec2;
  angle: number;
  invMass: number;
  invInertia: number;
  color: string;
}

export interface Joint {
  id: number;
  bodyId: number;
  /** Offset from the owning body's centroid, in the body's local frame. */
  local: Vec2;
}

/** Two joints (on different bodies) share a world position; free relative rotation. */
export interface PinConstraint {
  kind: "pin";
  id: number;
  jointA: number;
  jointB: number;
}

/** A joint is locked to a fixed world point; the body may rotate about it. */
export interface GroundConstraint {
  kind: "ground";
  id: number;
  joint: number;
  anchor: Vec2;
}

/** A joint is constrained to lie on a fixed world line (origin + unit direction). */
export interface SliderConstraint {
  kind: "slider";
  id: number;
  joint: number;
  origin: Vec2;
  dir: Vec2;
}

export type Constraint = PinConstraint | GroundConstraint | SliderConstraint;

/** Serializable snapshot of an entire scene (for save / load / autosave). */
export interface SceneData {
  version: number;
  bodies: Body[];
  joints: Joint[];
  constraints: Constraint[];
}

const FORMAT_VERSION = 1;

const PALETTE = [
  "#4f9dff",
  "#ff7b54",
  "#5bd6a6",
  "#c98bff",
  "#ffd166",
  "#ff6b9d",
  "#46c2cb",
];

export class Scene {
  bodies: Body[] = [];
  joints: Joint[] = [];
  constraints: Constraint[] = [];
  private nextId = 1;

  private id(): number {
    return this.nextId++;
  }

  /** Create a body from a polygon given in world coordinates. */
  addBody(worldVerts: Vec2[]): Body {
    const centroid = polygonCentroid(worldVerts);
    const local = worldVerts.map((p) => sub(p, centroid));
    const area = Math.max(Math.abs(polygonArea(worldVerts)), 1);
    const inertia = Math.max(polygonInertiaAboutCentroid(worldVerts, centroid), 1);
    const body: Body = {
      id: this.id(),
      local,
      pos: centroid,
      angle: 0,
      invMass: 1 / area,
      invInertia: 1 / inertia,
      color: PALETTE[this.bodies.length % PALETTE.length],
    };
    this.bodies.push(body);
    return body;
  }

  addJoint(bodyId: number, worldPos: Vec2): Joint {
    const body = this.getBody(bodyId)!;
    const offset = sub(worldPos, body.pos);
    const local = rotate(offset, -body.angle);
    const joint: Joint = { id: this.id(), bodyId, local };
    this.joints.push(joint);
    return joint;
  }

  addPin(jointA: number, jointB: number): PinConstraint {
    const c: PinConstraint = { kind: "pin", id: this.id(), jointA, jointB };
    this.constraints.push(c);
    return c;
  }

  addGround(joint: number, anchor: Vec2): GroundConstraint {
    const c: GroundConstraint = { kind: "ground", id: this.id(), joint, anchor };
    this.constraints.push(c);
    return c;
  }

  addSlider(joint: number, origin: Vec2, dir: Vec2): SliderConstraint {
    const c: SliderConstraint = { kind: "slider", id: this.id(), joint, origin, dir };
    this.constraints.push(c);
    return c;
  }

  getBody(id: number): Body | undefined {
    return this.bodies.find((b) => b.id === id);
  }

  getJoint(id: number): Joint | undefined {
    return this.joints.find((j) => j.id === id);
  }

  /** World position of a joint, derived from its body's current pose. */
  jointWorld(joint: Joint): Vec2 {
    const body = this.getBody(joint.bodyId)!;
    return add(body.pos, rotate(joint.local, body.angle));
  }

  /** World-space polygon of a body. */
  bodyWorldVerts(body: Body): Vec2[] {
    return body.local.map((p) => add(body.pos, rotate(p, body.angle)));
  }

  /** Topmost body whose polygon contains the point, or undefined. */
  bodyAt(p: Vec2): Body | undefined {
    for (let i = this.bodies.length - 1; i >= 0; i--) {
      if (pointInPolygon(p, this.bodyWorldVerts(this.bodies[i]))) return this.bodies[i];
    }
    return undefined;
  }

  /** Nearest joint within `radius` world units of the point, or undefined. */
  jointAt(p: Vec2, radius: number): Joint | undefined {
    let best: Joint | undefined;
    let bestD = radius;
    for (const j of this.joints) {
      const d = dist(this.jointWorld(j), p);
      if (d <= bestD) {
        bestD = d;
        best = j;
      }
    }
    return best;
  }

  /** True if a joint participates in any grounding or slider constraint. */
  isAnchored(jointId: number): boolean {
    return this.constraints.some(
      (c) =>
        (c.kind === "ground" && c.joint === jointId) ||
        (c.kind === "slider" && c.joint === jointId)
    );
  }

  clear(): void {
    this.bodies = [];
    this.joints = [];
    this.constraints = [];
    this.nextId = 1;
  }

  /** Plain-data snapshot of the scene; safe to JSON.stringify. */
  serialize(): SceneData {
    return {
      version: FORMAT_VERSION,
      bodies: this.bodies,
      joints: this.joints,
      constraints: this.constraints,
    };
  }

  /** Replace the scene's contents from a snapshot. Throws on malformed data. */
  load(data: SceneData): void {
    if (
      !data ||
      !Array.isArray(data.bodies) ||
      !Array.isArray(data.joints) ||
      !Array.isArray(data.constraints)
    ) {
      throw new Error("Not a valid Disjointed file.");
    }
    // Deep-clone so loaded data is independent of the parsed JSON object.
    this.bodies = data.bodies.map((b) => ({ ...b, pos: vec(b.pos.x, b.pos.y), local: b.local.map((p) => vec(p.x, p.y)) }));
    this.joints = data.joints.map((j) => ({ ...j, local: vec(j.local.x, j.local.y) }));
    this.constraints = data.constraints.map((c) => ({ ...c }));
    const ids = [
      ...this.bodies.map((b) => b.id),
      ...this.joints.map((j) => j.id),
      ...this.constraints.map((c) => c.id),
    ];
    this.nextId = (ids.length ? Math.max(...ids) : 0) + 1;
  }

  /** Snapshot of every body's pose, for save/restore around a simulation run. */
  snapshotPoses(): Map<number, { pos: Vec2; angle: number }> {
    const m = new Map<number, { pos: Vec2; angle: number }>();
    for (const b of this.bodies) m.set(b.id, { pos: vec(b.pos.x, b.pos.y), angle: b.angle });
    return m;
  }

  restorePoses(snapshot: Map<number, { pos: Vec2; angle: number }>): void {
    for (const b of this.bodies) {
      const s = snapshot.get(b.id);
      if (s) {
        b.pos = vec(s.pos.x, s.pos.y);
        b.angle = s.angle;
      }
    }
  }
}
