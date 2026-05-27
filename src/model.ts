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
  clone,
  sub,
  dist,
  normalize,
  distToLine,
  filletPolygon,
  roundedConvexBody,
  polygonCentroid,
  polygonArea,
  polygonInertiaAboutCentroid,
  pointInPolygon,
} from "./geometry";

/** How a body's `radius` shapes it: round the corners in place, or offset the hull outward. */
export type RoundMode = "fillet" | "offset";

export interface Body {
  id: number;
  /** Editable control polygon (the corners), relative to the centroid, in the local frame. */
  controlLocal: Vec2[];
  /** Corner radius (fillet) or outward margin (offset) applied to the control polygon. 0 = sharp. */
  radius: number;
  round: RoundMode;
  /** Derived render/physics polygon, relative to the centroid — rebuilt from control + radius. */
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
  /** Owning body, or `null` for a free joint (a body-less movable point). */
  bodyId: number | null;
  /**
   * For an attached joint: offset from the body's centroid, in the body's local
   * frame. For a free joint (`bodyId === null`): the joint's own world position.
   */
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

/**
 * A slider / prismatic rail: the segment between `railA` and `railB` — two joints
 * on one body. Any joint in `riders` (on other bodies) is confined to that segment
 * and slides along it. The rail moves with its body, so it couples the rail's body
 * to each rider's body. (For a world-fixed track, put the rail joints on a grounded
 * body.) A rail with no riders is just a (selectable, deletable) guide.
 */
export interface SliderConstraint {
  kind: "slider";
  id: number;
  railA: number;
  railB: number;
  riders: number[];
}

export type Constraint = PinConstraint | GroundConstraint | SliderConstraint;

/** Serializable snapshot of an entire scene (for save / load / autosave). */
export interface SceneData {
  version: number;
  bodies: Body[];
  joints: Joint[];
  constraints: Constraint[];
}

/**
 * A self-contained snapshot of a body for copy/paste: its control polygon, the joints
 * attached to it, and the constraints that reference *only* those joints (grounds,
 * fully-internal sliders, intra-body pins). Everything is stored in world coordinates
 * relative to the original centroid; pasting translates the whole fragment so the
 * centroid lands at the drop point. `tmp` ids are the original joint ids, remapped to
 * fresh joints on paste. Cross-body pins can't be reproduced and are dropped.
 */
export interface BodyClip {
  controlWorld: Vec2[];
  radius: number;
  round: RoundMode;
  centroid: Vec2;
  joints: { tmp: number; world: Vec2 }[];
  grounds: { joint: number; anchor: Vec2 }[];
  sliders: { railA: number; railB: number; riders: number[] }[];
  pins: { a: number; b: number }[];
}

const FORMAT_VERSION = 5;

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

  /**
   * Create a body from a control polygon (world coords). `radius` rounds it: `fillet`
   * rounds the corners in place (keeps concavity); `offset` grows the convex hull outward.
   */
  addBody(worldVerts: Vec2[], radius = 0, round: RoundMode = "fillet"): Body {
    const body: Body = {
      id: this.id(),
      controlLocal: worldVerts.map((p) => vec(p.x, p.y)), // world for now; rebuild re-centers it
      radius,
      round,
      local: [],
      pos: vec(0, 0),
      angle: 0,
      invMass: 1,
      invInertia: 1,
      color: PALETTE[this.bodies.length % PALETTE.length],
    };
    this.bodies.push(body);
    this.rebuildBody(body);
    return body;
  }

  /**
   * Recompute a body's render/physics polygon, centroid and mass from its control
   * polygon + radius, keeping attached joints anchored in world space (the centroid,
   * and thus the local frame, shifts when the shape changes).
   */
  private rebuildBody(body: Body): void {
    const ctrlWorld = body.controlLocal.map((p) => add(body.pos, rotate(p, body.angle)));
    const finalWorld =
      body.round === "offset"
        ? roundedConvexBody(ctrlWorld, body.radius)
        : filletPolygon(ctrlWorld, body.radius);
    const centroid = polygonCentroid(finalWorld);
    const attached = this.joints.filter((j) => j.bodyId === body.id);
    const jointWorlds = attached.map((j) => this.jointWorld(j));
    body.pos = centroid;
    body.local = finalWorld.map((p) => rotate(sub(p, centroid), -body.angle));
    body.controlLocal = ctrlWorld.map((p) => rotate(sub(p, centroid), -body.angle));
    body.invMass = 1 / Math.max(Math.abs(polygonArea(finalWorld)), 1);
    body.invInertia = 1 / Math.max(polygonInertiaAboutCentroid(finalWorld, centroid), 1);
    attached.forEach((j, i) => {
      j.local = rotate(sub(jointWorlds[i], centroid), -body.angle);
    });
  }

  /** Move a control vertex of a body by a world-space delta, then rebuild its shape. */
  moveBodyVertex(bodyId: number, index: number, delta: Vec2): void {
    const body = this.getBody(bodyId);
    if (!body || index < 0 || index >= body.controlLocal.length) return;
    body.controlLocal[index] = add(body.controlLocal[index], rotate(delta, -body.angle));
    this.rebuildBody(body);
  }

  /**
   * Insert a new control vertex at `index` (world position → local), then rebuild.
   * Used to add a node on a polygon edge: pass the index it should occupy (i.e. the
   * later endpoint of the clicked edge).
   */
  insertBodyVertex(bodyId: number, index: number, worldPos: Vec2): void {
    const body = this.getBody(bodyId);
    if (!body) return;
    const clamped = Math.min(Math.max(index, 0), body.controlLocal.length);
    body.controlLocal.splice(clamped, 0, rotate(sub(worldPos, body.pos), -body.angle));
    this.rebuildBody(body);
  }

  /**
   * Remove a control vertex, then rebuild. No-op if it would leave fewer than 3
   * vertices (the minimum to define a polygon) or the index is out of range.
   */
  removeBodyVertex(bodyId: number, index: number): void {
    const body = this.getBody(bodyId);
    if (!body || body.controlLocal.length <= 3) return;
    if (index < 0 || index >= body.controlLocal.length) return;
    body.controlLocal.splice(index, 1);
    this.rebuildBody(body);
  }

  /** Set a body's corner radius / margin (clamped ≥ 0), then rebuild its shape. */
  setBodyRadius(bodyId: number, radius: number): void {
    const body = this.getBody(bodyId);
    if (!body) return;
    body.radius = Math.max(0, radius);
    this.rebuildBody(body);
  }

  addJoint(bodyId: number, worldPos: Vec2): Joint {
    const body = this.getBody(bodyId)!;
    const offset = sub(worldPos, body.pos);
    const local = rotate(offset, -body.angle);
    const joint: Joint = { id: this.id(), bodyId, local };
    this.joints.push(joint);
    return joint;
  }

  /** Create a free joint: a body-less point at `worldPos` that the solver can move. */
  addFreeJoint(worldPos: Vec2): Joint {
    const joint: Joint = { id: this.id(), bodyId: null, local: vec(worldPos.x, worldPos.y) };
    this.joints.push(joint);
    return joint;
  }

  /**
   * Build a body from existing joints, with its polygon the rounded convex hull of
   * those joints expanded outward by `margin`. Free joints are absorbed into the new
   * body; joints on other bodies stay put and get a new joint here pinned to them.
   * Returns the new body, or null if the joints can't form an area.
   */
  buildBodyFromJoints(jointIds: number[], margin: number): Body | null {
    const joints = jointIds
      .map((id) => this.getJoint(id))
      .filter((j): j is Joint => j !== undefined);
    if (joints.length < 2) return null;
    const worlds = joints.map((j) => this.jointWorld(j));
    // Store one control point per joint; the rounded outline (hull + outward offset) is
    // recomputed from these on every rebuild, rather than baking the hull in.
    const body = this.addBody(worlds, margin, "offset");
    if (body.local.length < 3) return null;
    joints.forEach((j, i) => {
      const w = worlds[i];
      if (j.bodyId === null) {
        // Absorb the free joint: it now belongs to the new body (angle 0 at creation).
        j.bodyId = body.id;
        j.local = sub(w, body.pos);
      } else {
        // Joint on another body: add a coincident joint here and pin them together.
        const nj = this.addJoint(body.id, w);
        this.addPin(nj.id, j.id);
      }
    });
    return body;
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

  /**
   * Create a slider rail from two joints (`railA`/`railB`). Normally these are two joints
   * on the same body (a rail that moves with it). They may instead be two free joints, which
   * define a track fixed in world space — in that case each free rail joint must be anchored,
   * so any that isn't already grounded gets grounded here at its current position.
   */
  addSlider(railA: number, railB: number): SliderConstraint {
    for (const id of [railA, railB]) {
      const j = this.getJoint(id);
      if (
        j &&
        j.bodyId === null &&
        !this.constraints.some((c) => c.kind === "ground" && c.joint === id)
      ) {
        this.addGround(id, this.jointWorld(j));
      }
    }
    const c: SliderConstraint = { kind: "slider", id: this.id(), railA, railB, riders: [] };
    this.constraints.push(c);
    return c;
  }

  /** Attach a joint as a rider of an existing slider (confined to its rail segment). */
  attachSliderRider(sliderId: number, jointId: number): void {
    const c = this.constraints.find((x) => x.id === sliderId && x.kind === "slider") as
      | SliderConstraint
      | undefined;
    if (c && !c.riders.includes(jointId)) c.riders.push(jointId);
  }

  getBody(id: number): Body | undefined {
    return this.bodies.find((b) => b.id === id);
  }

  getJoint(id: number): Joint | undefined {
    return this.joints.find((j) => j.id === id);
  }

  /** World position of a joint: its body's pose + local offset, or its own point if free. */
  jointWorld(joint: Joint): Vec2 {
    if (joint.bodyId === null) return vec(joint.local.x, joint.local.y);
    const body = this.getBody(joint.bodyId)!;
    return add(body.pos, rotate(joint.local, body.angle));
  }

  /** World-space polygon of a body. */
  bodyWorldVerts(body: Body): Vec2[] {
    return body.local.map((p) => add(body.pos, rotate(p, body.angle)));
  }

  /** World-space control-polygon vertices (the editable corners) of a body. */
  bodyControlWorld(body: Body): Vec2[] {
    return body.controlLocal.map((p) => add(body.pos, rotate(p, body.angle)));
  }

  /** Topmost body whose polygon contains the point, or undefined. */
  bodyAt(p: Vec2): Body | undefined {
    for (let i = this.bodies.length - 1; i >= 0; i--) {
      if (pointInPolygon(p, this.bodyWorldVerts(this.bodies[i]))) return this.bodies[i];
    }
    return undefined;
  }

  /** Every body whose polygon contains the point (topmost first). */
  bodiesAt(p: Vec2): Body[] {
    const hits: Body[] = [];
    for (let i = this.bodies.length - 1; i >= 0; i--) {
      if (pointInPolygon(p, this.bodyWorldVerts(this.bodies[i]))) hits.push(this.bodies[i]);
    }
    return hits;
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
        (c.kind === "slider" &&
          (c.railA === jointId || c.railB === jointId || c.riders.includes(jointId)))
    );
  }

  /** Slider whose rail line passes within `radius` (world units) of the point. */
  sliderAt(p: Vec2, radius: number): SliderConstraint | undefined {
    for (let i = this.constraints.length - 1; i >= 0; i--) {
      const c = this.constraints[i];
      if (c.kind !== "slider") continue;
      const ja = this.getJoint(c.railA);
      const jb = this.getJoint(c.railB);
      if (!ja || !jb) continue;
      const a = this.jointWorld(ja);
      const dir = normalize(sub(this.jointWorld(jb), a));
      if (dir.x === 0 && dir.y === 0) continue;
      if (distToLine(p, a, dir) <= radius) return c;
    }
    return undefined;
  }

  /**
   * Translate the body's pose by `delta`, carrying along the world-space ground
   * anchors of any grounded joints on it (so the ground moves with the part).
   */
  moveBody(bodyId: number, delta: Vec2): void {
    const body = this.getBody(bodyId);
    if (!body) return;
    body.pos = add(body.pos, delta);
    const owned = new Set(this.joints.filter((j) => j.bodyId === bodyId).map((j) => j.id));
    for (const c of this.constraints) {
      if (c.kind === "ground" && owned.has(c.joint)) c.anchor = add(c.anchor, delta);
    }
  }

  /**
   * Reposition a joint by a world-space `delta`. A free joint's world point moves; a
   * body joint's local offset shifts (the body stays put). Any ground anchor on the
   * joint follows, so it stays grounded where it now sits.
   */
  moveJoint(id: number, delta: Vec2): void {
    const j = this.getJoint(id);
    if (!j) return;
    if (j.bodyId === null) {
      j.local = add(j.local, delta);
    } else {
      const body = this.getBody(j.bodyId)!;
      j.local = add(j.local, rotate(delta, -body.angle));
    }
    const w = this.jointWorld(j);
    for (const c of this.constraints) {
      if (c.kind === "ground" && c.joint === id) c.anchor = vec(w.x, w.y);
    }
  }

  /**
   * Rigidly rotate a body by `delta` radians about a fixed world `pivot`: its pose
   * (centroid + angle) turns about the pivot and attached joints follow automatically
   * (they live in the local frame). Ground anchors on the body's joints rotate too, so
   * a grounded part stays grounded where it now sits. No shape rebuild is needed.
   */
  rotateBody(bodyId: number, pivot: Vec2, delta: number): void {
    const body = this.getBody(bodyId);
    if (!body || delta === 0) return;
    body.pos = add(pivot, rotate(sub(body.pos, pivot), delta));
    body.angle += delta;
    const owned = new Set(this.joints.filter((j) => j.bodyId === bodyId).map((j) => j.id));
    for (const c of this.constraints) {
      if (c.kind === "ground" && owned.has(c.joint)) {
        c.anchor = add(pivot, rotate(sub(c.anchor, pivot), delta));
      }
    }
  }

  /**
   * Mirror a body across a line through its centroid: `"h"` flips it left↔right (reflect
   * x), `"v"` flips it top↔bottom (reflect y). The control polygon and every attached
   * joint (and its ground anchor) are reflected; the polygon winding is reversed so the
   * fillet/offset rounding stays correct. The centroid is fixed by the reflection, so
   * the body doesn't move — it just turns into its mirror image in place.
   */
  mirrorBody(bodyId: number, axis: "h" | "v"): void {
    const body = this.getBody(bodyId);
    if (!body) return;
    const c = body.pos;
    const reflect = (w: Vec2): Vec2 =>
      axis === "h" ? vec(2 * c.x - w.x, w.y) : vec(w.x, 2 * c.y - w.y);
    // Reflect the control polygon in world space; reverse it to preserve the winding.
    const ctrlWorld = this.bodyControlWorld(body).map(reflect).reverse();
    const attached = this.joints.filter((j) => j.bodyId === bodyId);
    const jointWorlds = new Map(attached.map((j) => [j.id, reflect(this.jointWorld(j))]));
    const owned = new Set(attached.map((j) => j.id));
    for (const con of this.constraints) {
      if (con.kind === "ground" && owned.has(con.joint)) con.anchor = reflect(con.anchor);
    }
    // Bake the reflected world geometry back in at angle 0 (a reflection isn't a rotation,
    // so the prior angle no longer applies), then let rebuildBody re-derive shape/mass and
    // re-anchor joints to their now-reflected world positions.
    body.angle = 0;
    body.controlLocal = ctrlWorld.map((p) => sub(p, c));
    for (const j of attached) j.local = sub(jointWorlds.get(j.id)!, c);
    this.rebuildBody(body);
  }

  /**
   * Snapshot a body and its dependent features into a copy/paste clip (see `BodyClip`).
   * Returns null if the body doesn't exist.
   */
  extractBody(bodyId: number): BodyClip | null {
    const body = this.getBody(bodyId);
    if (!body) return null;
    const attached = this.joints.filter((j) => j.bodyId === bodyId);
    const owned = new Set(attached.map((j) => j.id));
    const grounds: BodyClip["grounds"] = [];
    const sliders: BodyClip["sliders"] = [];
    const pins: BodyClip["pins"] = [];
    for (const c of this.constraints) {
      if (c.kind === "ground" && owned.has(c.joint)) {
        grounds.push({ joint: c.joint, anchor: clone(c.anchor) });
      } else if (c.kind === "slider" && owned.has(c.railA) && owned.has(c.railB)) {
        sliders.push({
          railA: c.railA,
          railB: c.railB,
          riders: c.riders.filter((r) => owned.has(r)),
        });
      } else if (c.kind === "pin" && owned.has(c.jointA) && owned.has(c.jointB)) {
        pins.push({ a: c.jointA, b: c.jointB });
      }
    }
    return {
      controlWorld: this.bodyControlWorld(body).map(clone),
      radius: body.radius,
      round: body.round,
      centroid: clone(body.pos),
      joints: attached.map((j) => ({ tmp: j.id, world: this.jointWorld(j) })),
      grounds,
      sliders,
      pins,
    };
  }

  /**
   * Paste a `BodyClip` so its original centroid lands at `at` (the whole fragment is
   * translated by `at − clip.centroid`). Recreates the body, its joints, and the clipped
   * constraints with fresh ids. Returns the new body's id, or null on failure.
   */
  insertBody(clip: BodyClip, at: Vec2): number | null {
    const offset = sub(at, clip.centroid);
    const body = this.addBody(
      clip.controlWorld.map((p) => add(p, offset)),
      clip.radius,
      clip.round
    );
    if (body.local.length < 3) return null;
    const idMap = new Map<number, number>(); // tmp id → new joint id
    for (const j of clip.joints) {
      idMap.set(j.tmp, this.addJoint(body.id, add(j.world, offset)).id);
    }
    for (const g of clip.grounds) {
      const id = idMap.get(g.joint);
      if (id !== undefined) this.addGround(id, add(g.anchor, offset));
    }
    for (const s of clip.sliders) {
      const a = idMap.get(s.railA);
      const b = idMap.get(s.railB);
      if (a === undefined || b === undefined) continue;
      const sl = this.addSlider(a, b);
      for (const r of s.riders) {
        const nr = idMap.get(r);
        if (nr !== undefined) this.attachSliderRider(sl.id, nr);
      }
    }
    for (const p of clip.pins) {
      const a = idMap.get(p.a);
      const b = idMap.get(p.b);
      if (a !== undefined && b !== undefined) this.addPin(a, b);
    }
    return body.id;
  }

  /** Remove a body along with its joints, pruning the constraints that used them. */
  removeBody(id: number): void {
    const removed = new Set(this.joints.filter((j) => j.bodyId === id).map((j) => j.id));
    this.bodies = this.bodies.filter((b) => b.id !== id);
    this.joints = this.joints.filter((j) => j.bodyId !== id);
    this.pruneConstraints(removed);
  }

  /** Remove a single joint, pruning the constraints that used it. */
  removeJoint(id: number): void {
    this.joints = this.joints.filter((j) => j.id !== id);
    this.pruneConstraints(new Set([id]));
  }

  /**
   * Drop or trim constraints after some joints are removed: a pin/ground that uses
   * a gone joint is dropped; a slider is dropped if a *rail* joint is gone, but only
   * loses the affected *riders* otherwise (the rail itself survives).
   */
  private pruneConstraints(removed: Set<number>): void {
    this.constraints = this.constraints
      .map((c) => pruneConstraint(c, removed))
      .filter((c): c is Constraint => c !== null);
  }

  /** Remove a single constraint (e.g. a slider) by id, leaving its joints intact. */
  removeConstraint(id: number): void {
    this.constraints = this.constraints.filter((c) => c.id !== id);
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
    this.bodies = data.bodies.map((b) => {
      const local = b.local.map((p) => vec(p.x, p.y));
      // Older files (< v5) have no control polygon: treat the saved polygon as a sharp control.
      const src = b as Body & { controlLocal?: Vec2[]; radius?: number; round?: RoundMode };
      const controlLocal = src.controlLocal ? src.controlLocal.map((p) => vec(p.x, p.y)) : local.map((p) => vec(p.x, p.y));
      return { ...b, pos: vec(b.pos.x, b.pos.y), local, controlLocal, radius: src.radius ?? 0, round: src.round ?? "fillet" };
    });
    this.joints = data.joints.map((j) => ({ ...j, local: vec(j.local.x, j.local.y) }));
    // Sliders: drop the legacy origin+dir form (no railA); migrate the earlier
    // single-`slider` rider field to the `riders` array; normalize riders to an array.
    this.constraints = data.constraints
      .filter((c) => c.kind !== "slider" || (c as { railA?: number }).railA !== undefined)
      .map((c) => {
        if (c.kind !== "slider") return { ...c };
        const s = c as SliderConstraint & { slider?: number };
        const riders = Array.isArray(s.riders)
          ? s.riders.slice()
          : typeof s.slider === "number"
          ? [s.slider]
          : [];
        return { kind: "slider", id: s.id, railA: s.railA, railB: s.railB, riders };
      });
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

/**
 * Given a set of removed joint ids, return the constraint to keep — possibly a
 * trimmed copy — or `null` to drop it. Pins/grounds drop if any referenced joint is
 * gone; sliders drop only if a rail joint is gone, otherwise they shed dead riders.
 */
function pruneConstraint(c: Constraint, removed: Set<number>): Constraint | null {
  if (c.kind === "pin") return removed.has(c.jointA) || removed.has(c.jointB) ? null : c;
  if (c.kind === "ground") return removed.has(c.joint) ? null : c;
  if (removed.has(c.railA) || removed.has(c.railB)) return null;
  const riders = c.riders.filter((r) => !removed.has(r));
  return riders.length === c.riders.length ? c : { ...c, riders };
}
