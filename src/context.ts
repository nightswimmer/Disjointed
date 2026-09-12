/**
 * Context ghost — the enclosing assembly shown faded, in a definition's own frame,
 * while that component definition is being edited (v21).
 *
 * A definition is edited in its own coordinate frame; an instance places that frame
 * into its owning context by an `InstanceTransform` T (reflect if mirrored, rotate,
 * translate). Entering the definition *through* a particular instance lets us show
 * everything else in the owning context where it sits relative to that instance: each
 * enclosing context's material is mapped by T⁻¹ into the definition frame, so the
 * reference instance lands exactly on the live definition and the world around it
 * appears rotated / shifted (mirrored, for a mirrored instance) accordingly. Nested
 * levels compose: the root's material passes through every placement on the way in.
 *
 * The ghost is built from **scratch Scenes** — one per enclosing context — so the
 * existing Scene queries (outline geometry, hit-testing, `resolveMeasureRef`) serve
 * rendering, object snapping and temporary context dimensions unchanged. It is never
 * part of the document: nothing in it is selectable, draggable or constrainable, and it
 * is rebuilt whenever the enclosing data changes (a definition edit cascades into the
 * sibling instances it shows).
 *
 * What a level keeps: every body / joint / rail outside the reference instance, plus
 * the joints that sit **on** the reference instance's bodies from outside (assembly
 * pins, neighbour links — the spots one most wants to align the definition to), kept
 * as free points. The reference instance's own material is dropped (the live
 * definition is drawn there), and so are the context's guides, dimensions, sketch
 * constraints and pattern metadata (drawing aids of *that* context, not geometry).
 */
import { ComponentDef, InstanceTransform, Scene, SceneData } from "./model";
import { Vec2, add, rotate, sub, vec } from "./geometry";

/** One enclosing context of the definition being edited. */
export interface GhostSource {
  /** Snapshot of the context (the root assembly, or an ancestor definition's data). */
  data: SceneData;
  /** The instance of the next-inner definition this context was entered through (its
   *  id in `data`), or null when unknown — the chain of placements breaks there and
   *  this context (and everything outside it) cannot be placed. */
  via: number | null;
}

/**
 * Build the ghost for an editing path. `sources[k]` is context k (0 = the root, the
 * last = the immediate parent of the definition being edited); the result is aligned
 * with it — `levels[k]` is context k's material as a scratch Scene already expressed in
 * the innermost definition's frame, or null when that level isn't shown: beyond
 * `depth` enclosing levels (counted from the immediate parent outward; 0 = no ghost),
 * or outside a break in the placement chain (a level entered with no known instance).
 */
export function buildContextGhost(
  sources: GhostSource[],
  components: ComponentDef[],
  depth: number
): (Scene | null)[] {
  const n = sources.length;
  const levels: (Scene | null)[] = sources.map(() => null);
  // Placements of the levels already processed, innermost last: level k's material goes
  // through its own T_k⁻¹ first, then T_{k+1}⁻¹ … T_{n−1}⁻¹ on the way into the innermost frame.
  const inner: InstanceTransform[] = [];
  for (let k = n - 1; k >= 0 && n - k <= depth; k--) {
    const src = sources[k];
    if (src.via === null) break;
    const s = new Scene();
    s.loadContext(src.data); // deep-clones: the stored snapshot is never touched
    s.components = components;
    const t = s.instancePlacement(src.via);
    if (!t) break;
    stripReferenceInstance(s, src.via);
    applyInversePlacement(s, t);
    for (const ti of inner) applyInversePlacement(s, ti);
    inner.unshift(t);
    levels[k] = s;
  }
  return levels;
}

/**
 * Remove the reference instance's own material from a scratch context, keeping the
 * joints other elements placed on its bodies as free points (their world positions),
 * and drop the context's drawing aids (guides, dimensions, sketch constraints).
 */
function stripReferenceInstance(s: Scene, instanceId: number): void {
  const inst = s.instances.find((i) => i.id === instanceId);
  if (inst) {
    const own = new Set([...inst.jointMap, ...inst.anchorMap].map((e) => e.id));
    const bodies = new Set(inst.bodyMap.map((e) => e.id));
    for (const j of s.joints) {
      if (j.bodyId !== null && bodies.has(j.bodyId) && !own.has(j.id)) {
        j.local = s.jointWorld(j);
        j.bodyId = null;
      }
    }
    s.removeInstance(instanceId);
  }
  s.measurements = [];
  s.sketch = [];
  s.guides = [];
}

/**
 * Map a whole context by the inverse of an instance placement — context → definition
 * frame: translate by −pos, rotate by −angle about the origin, then (for a mirrored
 * instance) reflect across the definition's x-axis (y → −y). Uses the Scene's own
 * rigid-motion primitives so attached material, ground anchors and body-owned rail
 * tracks ride along exactly as they do for user edits.
 */
export function applyInversePlacement(s: Scene, t: InstanceTransform): void {
  const bodyIds = s.bodies.map((b) => b.id);
  const carried = s.ownedTrackJointsOf(bodyIds); // moved by their body's moveBody / rotateBody
  const freeJoints = s.joints.filter((j) => j.bodyId === null && !carried.has(j.id)).map((j) => j.id);
  const origin = vec(0, 0);
  // Translate.
  const shift = vec(-t.pos.x, -t.pos.y);
  for (const id of bodyIds) s.moveBody(id, shift);
  for (const id of freeJoints) s.moveJoint(id, shift);
  // Rotate about the origin.
  if (t.angle !== 0) {
    for (const id of bodyIds) s.rotateBody(id, origin, -t.angle);
    for (const id of freeJoints) {
      const j = s.getJoint(id);
      if (!j) continue;
      const w = s.jointWorld(j);
      s.moveJoint(id, sub(rotate(w, -t.angle), w));
    }
  }
  // Reflect y → −y. mirrorBodies reflects about the material's own bounding-box centre
  // line y = c (y' = 2c − y); a follow-up shift by −2c makes it the axis reflection.
  if (t.mirrored) {
    const probeId = bodyIds[0] ?? null;
    const probeJoint = probeId === null ? freeJoints[0] ?? null : null;
    const probe = probePoint(s, probeId, probeJoint);
    if (probe) {
      s.mirrorBodies(bodyIds, freeJoints, "v");
      const after = probePoint(s, probeId, probeJoint)!;
      const c = (probe.y + after.y) / 2;
      const down = vec(0, -2 * c);
      for (const id of bodyIds) s.moveBody(id, down);
      for (const id of s.joints.filter((j) => j.bodyId === null && !carried.has(j.id)).map((j) => j.id)) {
        s.moveJoint(id, down);
      }
    }
  }
}

/** A point that moves with the context's material (a body's centroid, else a free
 *  joint — by id, so it survives a re-expansion), used to measure what mirrorBodies
 *  reflected about. */
function probePoint(s: Scene, bodyId: number | null, jointId: number | null): Vec2 | null {
  const b = bodyId !== null ? s.getBody(bodyId) : undefined;
  if (b) return add(b.pos, vec(0, 0));
  const j = jointId !== null ? s.getJoint(jointId) : undefined;
  return j ? s.jointWorld(j) : null;
}
