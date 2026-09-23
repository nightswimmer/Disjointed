# Disjointed — Project State

Session-to-session handover notes for Claude. Keep this file to what is **not** derivable from the
code, git history, README.md or the manual: goals, design decisions and their reasons, invariants,
pitfalls, and what is in flight. Anything else belongs in code comments or README.md.

Where things are documented:
- **README.md** — the complete user-facing reference (every tool, gesture, shortcut, panel). When a
  behaviour question comes up, read it there; do not duplicate it here.
- **public/help/** — the in-app manual (same content as README, illustrated). Brought fully up
  to date on 2026-09-23 and kept in step with UI changes since (see *Manual* under *UI
  conventions*).
- **HANDOFF.md** — unfinished work: the rest of the shortcut remap (next up), the shape-tools
  roadmap (phases 2–4) and smaller follow-ups.
- **public/keymap.json** — every shortcut, as data, and **the shipped keymap itself** since
  2026-09-15: `src/commands.ts` imports it at build time, so replacing the file remaps the app.
- Code comments carry the local *why* for most non-obvious branches; `scripts/*.ts` are the
  executable spec.

## Goal
A single-page webapp to draw and simulate 2D planar mechanisms. **Draw mode**: bodies (editable
control polygon + corner rounding), joints (attached or free), constraints (pin / weld, ground, rail
+ riders / sliders, actuators, motors), CAD-style sketch constraints and dimensions, groups,
reusable components, reference geometry, patterns; DXF import and DXF/SVG cut-file export.
**Simulate mode**: drag any joint or body point; a position-based constraint solver propagates the
motion; actuators / motors animate.

## Tech stack & commands
- Vite + TypeScript + HTML5 Canvas, no UI framework, no runtime dependencies. Node 24.
- `npm run dev` / `build` / `preview`; `npm test` runs every `scripts/*.ts` (tsx, headless, no DOM);
  `npm run manual` regenerates the manual's SVG illustrations, UI markup snapshots and glyph tables
  with Playwright driving the **installed Chrome** (`channel: "chrome"`), and fails if a topic the
  app can ask for is missing or a referenced image / snapshot was not generated.
- Keymap scripts: `npm run keymap:file` rewrites `public/keymap.json`'s **metadata** from the
  registry and keeps every binding — needed only after *adding a command*, which it appends with
  an empty binding list; it refuses to write over a file it cannot parse, since that would throw
  the bindings away. `npm run keymap:docs` rewrites the shortcut lists in README.md and the
  manual (`--check` reports staleness instead; `npm test` runs `--check --readme`, so a keymap
  edit that leaves the README behind fails the suite). `npm run keymap:live` presses every
  shortcut in the real app under Chrome.
- The interactive canvas is not covered by `npm test`: confirm UI changes by eye (or with a
  throwaway Playwright script against `npm run dev` with `?automation`, which exposes
  `window.__disjointed` — `scripts/keymap-live.ts` is a worked example of that pattern).
- Field-repro scenes in the repo root: `FrontPanelHinge.json`, `gate hinge tests.json` (the
  "Door Assembly 6" pattern/dimension case). The tangent / line-blends-into-arc repro is
  **not** kept as a scene: its geometry is embedded in `scripts/tangent-constraint.ts`.

## Module map (`src/`)
| Module | Role |
|---|---|
| `geometry.ts` | Vec2, polygon properties, fillet (`filletPolygon` / `filletCornerArcs`), offset hulls, regular polygons, arcs |
| `model.ts` | `Scene`: bodies, joints, constraints, groups, components, guides, patterns, measurements, sketch constraints; serialization (`FORMAT_VERSION = 22`); all editing primitives |
| `solver.ts` | Sim solver (Gauss-Seidel positional impulses), groups/welds as rigid composites, break-and-exclude |
| `sketch.ts` | Draw-mode shape solver for sketch constraints + driving dimensions |
| `pose.ts` | Pose-level dimensions / constraints on component instances (rigid moves, not shape) |
| `boolean.ts` | Polygon union (Combine), difference (Cut role, Subtract) and intersection (Intersect) over a planar graph |
| `context.ts` | Context ghost: the enclosing assembly drawn faded inside a definition |
| `analyzer.ts` | Topology diagnostic (islands, DOF, loop cores). Not wired to any UI |
| `dxf.ts` / `export.ts` | DXF reader with fillet reconstruction; DXF R12 / SVG cut-file writer with exact arcs |
| `renderer.ts` | Canvas drawing from a `RenderInput`; theme palette; screen-space labels |
| `view.ts` | Camera (`screen = R(angle)·world·scale + t`) |
| `main.ts` | Everything UI: tools, drags, snapping, selection, history, persistence, backup, component contexts, animation loop; the command **actions** and the key handler |
| `commands.ts` | The command registry as data (id, label, description, group, context, tool) + the `Tool` / `ShapeRole` vocabulary; imports `public/keymap.json` for the shipped bindings. DOM-free |
| `keymap.ts` | The `keymap/1` format: chord spelling, event → slot normalization, validation, conflicts, the user's overrides. DOM-free |
| `notify.ts` | Toasts — the project-wide replacement for `alert` |
| `filestore.ts` | File System Access API wrappers + IndexedDB handle storage |
| `toolbar.ts` | Toolbar section rack: drag a group by its caption to reorder (live FLIP reflow), order in localStorage |
| `public/ui.css` / `style.css` | The **look** of the controls and panels (palette, buttons, fields, combos, sections, toggles, crumb bar, panels), linked from `index.html` before the bundle and shared with the manual / the **page**: layout, where overlays sit, drag states, editors, toasts, help drawer |
| `help.ts` / `helpmap.ts` | Help drawer + help mode; DOM-free UI→topic map shared with the manual generator |
| `automation.ts` / `svgcontext.ts` | Playwright hook; canvas-API-shaped SVG recorder (manual illustrations) |
| `public/help/`, `scripts/manual/` | The manual and its generator (`shoot.ts`, `shots.ts`, `fixtures.ts`) |

## Domain model — semantics worth remembering
- **Body**: pose = centroid `pos` + `angle`; shape = `controlLocal` + `radius` (+ per-corner
  `radii`, `round: "fillet" | "offset"`). The rendered polygon is derived. Holes are editable
  outlines of the same kind; `holesLocal` is derived. A 1-point offset outline is a **disk**
  (parametric); `regular` marks a regular polygon kept as an invariant.
- **Joint**: `bodyId` + `local`; `bodyId === null` = free joint (a point particle).
- **Pin** (`rigid` = weld), **ground**, **slider** = a rail (`railA`/`railB`) + `riders`, with
  `locked` (orientation-locked = "slider" in the UI) and `startRiders` (home at `railA`) sublists.
  A rail is either two joints on one body or two free joints (auto-grounded). Powered:
  `linearActuator` (rider on a rail), `motor` (pivot + crank on one body) — inert off-animation,
  driven through the solver's `anchors` map (treated exactly like moving grounds).
- **Groups** (`BodyGroup { bodyIds, jointIds }`): one rigid body in sim, move together in draw.
- **Components**: `ComponentDef { data: SceneData }` (a DAG, cycles rejected); instances are
  **materialized** real bodies/joints/constraints tagged with provenance maps, so solver, renderer
  and hit-testing know nothing about hierarchy. Grounding inside a def = "fixed to the component
  frame" and expands into a rigid **chassis group**, never a world ground.
- **Guides** (reference geometry): union of `poly` / `circle` / `arc` / `text`, all finite; never
  simulated, invisible in sim; usable as measurement / constraint refs.
- **Patterns**: `slots[]` (each a seed hole/joint + its members) sharing one layout anchored on the
  **first** slot's seed; members are real holes/joints **re-derived** from their seed on every change,
  every instance applying one rigid motion to the whole seed group.
- **Measurements** and **sketch constraints** share `MeasureRef` (joint, vertex, edge, bodyPoint,
  rail, guidePoint, guideLine, patternAxis, centre, midpoint; vertex/edge refs carry an optional `hole`).
  Refs always name **elements**, never coordinates. A **`midpoint` nests its line's own ref**
  (`of`: edge / rail / guideLine) instead of copying its fields, and every index remap
  (node insert/remove, mirror, split, combine/cut, hole removal, copy/paste) works on
  `refHost(ref)` — so a midpoint follows its line without any remap knowing about midpoints.
  Keep that invariant: a new remap site must go through `refHost`. Dimensions may be `driving` (with `target` and
  a held `side`). The **one** exception to "never coordinates" is the `fixed` constraint: a lock
  in place *is* a coordinate and there is no element to name it with, so it carries `at` (the
  point, or a point on the locked line) and — line form only — `angle`. `angle !== undefined`
  is what tells the two forms apart. The `symmetric` constraint is the one with **three**
  references: `refA` / `refB` (both points or both lines) plus `mirror` (a line). Every site
  that remaps, prunes, clones or ownership-tests a constraint's refs iterates
  **`sketchRefs(c)`**, never `[refA, refB]` — keep that invariant when adding a site.
  Two ref kinds name a **circle** rather than a point or line: `disk` (a disk body or a
  circular hole: `bodyId` + optional `hole`) and `guideCircle` (a reference circle, or the
  circle a reference arc lies on). `isCircleRef` tells them apart; **they resolve to their
  centre point** through `resolveMeasureRef`, so label anchors, badges, hover, pruning and
  every remap treat them like a point, and `circleOfRef` hands the radius to the few places
  that need the circle itself (the `tangent` items, the rim highlight
  `circleHighlightOfRef`). `tangent` takes one (stored as `refA`, the line as `refB`); the
  **radius form of `equal`** takes two *radius refs* — a `disk`, a `guideCircle`, or a rounded
  corner's `vertex` — told from the length form by `isEqualRadiusConstraint` (kind-based: no
  line ref), and read / written through `radiusOfRef` / `setRadiusOfRef` (`radiusSettable` is
  false only for a reference arc).
  A `disk` ref rides every remap site `vertex` / `edge` / `centre` ride on holes, and
  `shiftMeasureIndices` also drops it when a node added to the outline stops it being a disk
  (an outer disk cut, split or combined is a polygon afterwards: its refs go stale).
- The `bodies` array **is** the z-order (drawn first→last, picked last→first).

## Design decisions and invariants (with reasons)

### Sim solver
- **Converge to tolerance, never a fixed sweep count** — fixed counts drifted on closed loops.
- **Grounds are sacred**: projected exactly every sweep, never disabled; any grounded joint is a
  fixed host for whatever pins to it (`pinHostFor`), so a pin can't drag the body a grounded joint
  sits on. Grounded bodies and frozen material are immovable fixed hosts.
- **Structural constraints outrank the mouse driver**; the driver is step-limited so unreachable
  targets walk stably to the nearest reachable point.
- **Impossible assemblies → break-and-exclude**: Phase A solves; if unconverged, Phase B disables
  the worst non-ground units until the rest converges, Phase C closes them as far as freedom allows,
  and the breaks are reported (red dotted lines, red joints, banner). Runs only for unconverged scenes.
- **Phase-A early exit only when no driver is present** — with a driver the step limit keeps the
  residual ≈ 0 and an early exit starves the drag.
- **Groups and welds are rigid composites**, not iterated constraints: `mergeWeldComposites`
  union-finds weld-connected units into one `RigidGroup` per solve with exact snap-assembly at the
  drawn angle. Two earlier weld designs (phantom second pin; per-sweep angular projection) converged
  one link per sweep and caused false breaks / 100 % CPU on chains — don't go back. Intra-unit pins
  and riders are inert. Impossible welds surface via `weldConflictBreaks`, never Phase B.
- **Slider locks** use the **wrapped** relative angle (unique zero) — the old `sin`-based error was
  also zero at a 180° flip.
- **"As drawn" is the reference**: slider-lock and weld baselines are captured from the drawn pose
  at sim entry / rigid-drag start (`resetPoseBaselines` on every `markDirty`); no stored angle.
- **Scoped solves** (`SolveFreeze`) power the rigid Shift-drag: frozen material is immovable and
  constraints entirely inside the frozen world are out of scope (not solved, not reported).
- Drag-yield caveat: "a drag never breaks constraints" is enforced by convergence, not by
  construction. If a slow-converging mechanism ever false-breaks under a drag, the candidate
  hardening is to skip Phase B for pure mouse-driver solves.

### Sketch solver (draw mode)
- Variables are **world positions** of control vertices, joints, guide points and regular-polygon
  centres; a joint coincident with a control vertex maps onto that vertex (the node↔joint link).
- **Reject semantics**: an unconverged solve never touches the scene; the conflicting items flash
  red. Multi-pass write-back (`APPLY_PASSES`) re-solves from the applied state before rejecting,
  because the rigid carry itself can move items the iteration left satisfied.
- **Mobility ranks** decide who moves: 0 construction (guide points) < 1 geometry < 1.5 tied guides
  < 2 drag-anchored < 3 immovable (component-instance geometry — its shape belongs to the def —
  and points held by a `fixed` lock).
  Corrections flow to the lowest rank. Consequences: a guide with one demand moves alone; a guide
  tied to geometry or carrying several demands becomes a reference (geometry moves to it); a
  guides-as-reference fallback pass runs when the construction-first pass fails.
- **Drags are anchored**; when the anchored solve is infeasible the live solve falls back to a
  symmetric solve **the same frame**, so constraints never look broken mid-drag.
- **Whole-body rigid carry** (`applyRigidParts`): when a solve moves every outer vertex, the body
  moves as a rigid unit so holes, joints and anchors ride along. Body-owned variables the solve left
  unchanged are mapped through the same motion.
- Distances off an H/V-constrained line shift along the exact world axis, not the momentary normal
  (a mid-sweep tilt otherwise leaks motion sideways).
- **Driving dimensions hold their drawn `side`** so a one-frame overshoot can't flip the two sides
  through each other (direct point–point distances have no side by nature).
- First driving dimension on an otherwise-unconstrained body **scales it uniformly**; later ones move
  nodes. **A dimension with an end on a hole never scales** (`refOnHole` in `scaleEligibleBody`,
  2026-09-22): it is about that hole, not the plate — before, the first hole-width dimension on a
  plate scaled the whole plate, pattern spacing included. Size dimensions (diameter, radius,
  regular-polygon size) are not solver items: they set the parameter directly and are re-applied
  by `enforceSizeDims`. A direct resize (rim drag, `[`/`]`) demotes conflicting driving size
  dims to driven.
- **Pattern members ride with their seed** through rigid-offset couplings (an immovable rank
  deadlocked any solve that needed a patterned body to shift). A dimension is refused only when it
  **spans two instances** of one pattern (`patternSpannedBy`: seed ↔ member or member ↔ member —
  that distance is the layout, edited on the labels); both ends on one instance (a seed's
  diameter, a slot's own width) is ordinary shape material and the members copy the result. Until
  2026-09-22 the check compared *patterns* instead of instances, so nothing on a patterned seed
  could be dimensioned at all — the first reported patterns bug.
- **Multi-seed patterns (v22, 2026-09-22)**: `patternInstanceOfRef` → instance −1 for any seed,
  else the copy index, so seed ↔ seed dims drive (the copies reproduce the new placement) and
  seed ↔ copy dims are refused. The anchor is slot 0's seed; a circular `centre` is relative to it,
  so `reanchorLayout` shifts it when slot 0 goes (anchors read *before* removal — `rebuildBody`
  moves the local frame with the composite centroid). `syncPattern` trims every slot first (one
  `dropHoles`), then writes all members in one frame and rebuilds once — never write geometry
  after a rebuild with motions computed before it. A feature clip carries a pattern only when
  every seed is copied (all-or-nothing; partial slot carrying was not built).
- **Derived points are `PointHandle`s** (sketch.ts `acquirePoint`): the point items
  (coincident / H / V pairs, point-on-line, the point form of `fixed`) read and push a point
  through a handle, which is one variable or — for a `midpoint` — the mean of its line's two
  end variables. A midpoint's rank is its most mobile end's; a push is spread over the ends by
  rank (an end held by the drag or a lock stays put, the other swings twice as far), so a
  midpoint never tugs the drag and never stalls on a locked end. `pointVarKey` stays null for
  a midpoint: it is not a variable, so a `fixed` midpoint freezes neither end (dimensions
  don't take midpoints — the Measure tool can't pick them).
- **`fixed` is enforced unconditionally, not by rank**: its item writes the lock back every sweep
  whatever the ranks say, so a locked point resists even the drag and two locks that disagree
  never settle — the edit is then rejected, which is what an impossible lock should do (rank 3
  is there only so corrections flow away from a locked point in one step). A locked **line**
  stores the whole infinite line and projects *both* ends onto it: the ends keep their normal
  rank, so they stay free to slide along the line and stretch it, and only the line is frozen.
  Deliberate whole-element transforms that don't live-solve (`mirrorBody` / `mirrorBodies`)
  call `recaptureFixed` — the reflection is a move the user asked for, so the lock re-anchors
  instead of fighting it; drags and rotations need nothing, they solve live and the lock wins.
- **`symmetric` shares its correction mirror-first, by rank** (`mirrorShare` in sketch.ts):
  the mirror line moves onto the pair's bisector (perpendicular bisector for points, the
  angle bisector nearest its current direction — or the midline — for lines) as far as its
  share goes, then the two objects move toward each other's image (reflection is affine, so
  any split lands on exact images). Consequences that follow from the guide rules: a **free
  reference line is re-placed** by its first symmetry (one demand → the guide moves alone);
  a guide mirroring **two or more** symmetries joins the tied set (`tiedGuideVars`) and is the
  reference; a `fixed` line reads as rank 3 for the mirror. **Immovable participants are
  never written** (rank ≥ 3: instance geometry, locked points): pair immovable → the mirror
  takes it all; mirror immovable → the pair; both → the residual stands and the edit rejects
  (before this rule an item splitting 50/50 between two rank-3 instance points deformed the
  instances — the only place two rank-3 variables can meet in one item outside the pose
  route). Lines move as rigid pieces (`moveLineOnto`: turn about the midpoint, then shift), so
  lengths survive; projecting the ends along the normal shortened them by the cosine.
  On the pose route (every ref instance-owned) the A side is **not** the negated B move:
  `PoseMove` grew `deltaA` / `angleA`, and a side that carries the mirror moves by the
  *reflected* shift (moving it moves the mirror, which moves the image it chases) with the
  turn's sign flipped. Instance points about a non-instance mirror are a *sketch* case: the
  mirror moves (free) or the edit rejects (locked).
- **`tangent` is a point–line distance whose target is the radius** (`acquireCircle` + the
  tangent item in sketch.ts): the circle's centre — a disk's vertex 0, a reference circle's
  `c`; a reference arc has no centre variable, so the circle through its `a` / `m` / `b`
  variables is re-derived every sweep — is pushed along the line's normal together with the
  line, shared by rank like any pair. A `fixed` line reads as rank 3, as a symmetric's mirror
  does (splitting with it and letting the lock push the ends back left a sub-tolerance
  wobble on the plate). **The radius is never a variable**: disks are sized by size
  dimensions / the rim handle, and an arc shifts its three points as a rigid piece (points
  above the arc's lowest rank — one held by the drag — stay put, the arc reshapes a little
  and the next sweep corrects the rest). **No stored side**: the momentary sign decides, so
  a circle dragged through its line re-attaches on the far side (nothing was *drawn* to
  hold, unlike a driving dimension). Consequence: `applyDrivingDimension`'s diameter branch
  now runs `solveAndApply` after `setDiskRadius` (snapshot / restore), so a tangent line
  follows the new rim — before, a diameter edit never solved anything. On the pose route a
  tangent is a translation of one side along the line's normal (pose.ts).
  **Who moves within an arc / a line** (`System.named`): a point some other item names (a
  coincident, H/V pair, point-on-line, lock, driving-dimension end) is left where that
  item puts it; an arc reshapes through its remaining free points by a **Newton step**
  along the normal (`CircleHandle.correct`), and a line with one held end **turns about
  it** instead of shifting (`turnOrShiftLine`; both held → nothing; an H/V-held line only
  shifts). Rationale: a tangent translating the whole arc while a coincident pulled one
  end back chased itself into ever bigger circles (the field repro — its geometry is the
  blend case in `scripts/tangent-constraint.ts`).
  **A tangent pinned to an arc end is an angle condition** (`pinnedArcEnd`): when a
  coincident glues an arc end onto the tangent's line (its end, its midpoint, or
  point-on-line), the residual becomes the centre's offset *along the line* from that
  end and the corrections are turns — the arc about the end, the line about its held end
  (`newtonTurn`; an axis-held line leaves the whole turn to the arc). The distance form
  is second-order in that offset there, and Gauss-Seidel on it converged like 1/n² (400
  sweeps left 0.002 on 6000-unit geometry); the angle form settles in ~6 sweeps. That is
  the "line blends smoothly into an arc" CAD idiom. `sketchConfig.trace` is the per-sweep
  residual hook that found it — use it before guessing.
- **Equal between radii is a parameter pass, not a solver item** (`enforceEqualRadii`,
  2026-09-22). Radii are never variables, so an `equal` whose refs are circles / rounded
  corners builds no item; `enforceSizeDims` re-applies the **equality classes** after every
  solve (union-find over `radiusKey`: A = B, B = C is one class; a disk and its centre vertex —
  how a diameter dim names it — are one key; a pattern member keys as its seed). The class
  value comes from whoever cannot change, in order: a driving size dimension → a member
  nothing can set (a reference arc is three points the solver reshapes; instance geometry is
  the def's) → the `source` the caller names (the first pick on creation, the element just
  resized) → the first-named member. Same-rank disagreement is a **conflict**: the class is
  left alone and the breaks name the dims (plus the class's Equals when a dim asks an
  immovable to change). `tryAddConstraint` and the diameter / radius branches of
  `applyDrivingDimension` run the pass *before* their solve so tangents see the final rims,
  and snapshot / restore around it — the write has happened by the time a solve can fail.
  In main.ts a direct resize (`propagateEqualRadii`) demotes the **partners'** size dims like
  the resized outline's own, and a fillet drag / `[` `]` now settle-solve when the sketch has
  a tangent or a radius Equal (before, nothing solved after a rim drag, so a tangent line
  lagged until the next edit). Refused by `sketchConstraintProblem`: a line with a circle,
  two arcs (neither settable), seed ↔ member (redundant), two instance radii. Known
  ping-pong (accepted, same as radius dims): two classes each holding a different corner of
  one *uniform* outline, since a corner write there sets every corner.
- **Regular polygons are an invariant, not constraints**: rigid weighted fit per solve (a similarity
  fit let pinching corrections shrink it sweep after sweep); H/V/parallel/perpendicular on an edge
  turn it in one step; **a dimension never turns a regular polygon** (that solver item could stall
  and one stalled item fails every solve in the scene). Corner drags respect what the sketch leaves
  free (size driven → turn only; rotation locked → resize radially; both → move whole).
- Auto-constraints **while drawing**: near-H/V edges (±5°) get H/V; a vertex placed on a
  joint/corner/guide point/line midpoint gets a coincident. Always on (the switch below doesn't reach them).
- **Implicit constraints while dragging** (`DragAlign`, main.ts) arm after a 0.4 s hover and apply an
  exact alignment correction **before** placing the constraint — letting the solver close even a
  0.3 mm gap failed when the other side was dimension-pinned. Design points:
  - **Two candidates stay armed** (`ALIGN_MAX_CANDS`), so one drag can take a V off one reference
    and an H off another; a third hover drops the oldest, Esc drops the newest.
  - The correction is a **2-DOF linear solve**, not a per-kind special case: each match contributes
    `dot(n, delta) = d` (y axis for H, x for V, the line normal for point-on-line, **both** axes for
    a point-on-point coincident), one equation gives the perpendicular foot, two independent ones go
    through Cramer. A release therefore lands exactly on the intersection.
  - A match is kept only while the kept equations stay solvable: ≤ 2 total, and no two normals within
    `ALIGN_INDEPENDENT_TOL` (≈3°). Candidates are matched **newest first**, so a clash drops the
    older candidate's preview (it stays armed and returns when the drag stops matching the newer).
    A point-on-point coincident spends both DOF and so always previews alone.
  - Dropping a point **on** a candidate point is a coincident (it used to be "a placement, not an
    alignment"): you are on the candidate while hovering to arm it, so ◎ shows immediately.
  - Placement is per match, so one rejection doesn't lose the other.
  - `autoConstrain` (the Constraints group's switch) gates the whole thing at `newDragAlign` /
    `updateDragAlign`, so nothing is scanned when it's off.
  - **Point targets beat line targets** in every pick (object snap, the alignment hover,
    shape / reference-point placement): points are scanned first and a line only when no
    point is in range. Line **midpoints** (edge / rail / reference segment) are point targets
    with a real `midpoint` ref, so holding the middle of an edge arms the midpoint, not the
    edge. The constraint tools and the Measure tool deliberately don't pick midpoints
    (CAD convention: an inference, not a click target).

### Components
- **The definition is the pose reference**: any def edit re-expands every instance (fixpoint through
  the DAG) and snaps every part's pose to `T · defPose`; only placement and the instance-level
  grounded flag are instance state. Internally posed mechanisms therefore reset on def edits — accepted.
- Instances carry shapes, not design constraints (def sketch / dims / guides stay in the def), so
  instances rotate freely — the motivating pain with H/V constraints.
- Components are a **transparent grouping for dimensioning and constraining**: one free end → the
  free side moves; every end instance-owned → a **pose** dimension / constraint (translation or
  rotation of a rigid part, second-picked side moves by preference, grounded never moves); ends
  rigid to one another at a deeper level → rejected. h/v pose dims measure **world** axes (accepted caveat).
- Mirroring an instance is a placement flag (`mirrored`), expanded as reflect → rotate → translate;
  motors negate speed; re-expansion of a mirrored instance is a no-op.
- Deleting a def with instances **converts them to plain bodies** (chassis group survives).
  Forking (⊞ on one instance) deep-copies the def byte-identically so no re-expansion is needed.
- The **context ghost** is inert by design: object-snap targets only (refs stripped to null so
  nothing can bind to it); temporary context dimensions live in main, are never serialized, and
  their one-shot move emulates a drag of the picked live feature.

### Shapes, holes, roles
- **Picking and joint containment use the outer outline only** (a joint may sit at a shaft hole's
  centre; clicking in a cut-out still selects the body). Attached joints can never be placed or
  dragged outside their body (clamp to the outline); a shape change that strands one is **flagged,
  never auto-moved**.
- **Shape roles are explicit and sticky** (Body / Cut / Reference; Ctrl on the first click flips
  Body↔Cut once) — never inferred from where a click lands, because bodies routinely overlap.
- **Cut** = polygon difference: fast path (cutter fully inside, clear of holes) keeps the exact spec
  (disks stay parametric); the general path (`applyRegion`, shared with Combine) keeps unchanged
  corners' radii, new corners sharp, remaps refs by world geometry, and dissolves the body's
  patterns. A cut that would split or remove the body is refused ("use Split").
- **Subtract / Intersect** (`booleanBodies`, 2026-09-22) follow Combine's convention — the
  **first selected body survives**, the others are *tools* and are **consumed** (`removeBody`:
  their joints and constraints go) — rather than offering a keep-tools option; copy the tool
  first to keep it. A tool swallowed whole as a hole lends the hole its own spec (`applyRegion`
  matches result holes against the tools' outers), so a disk body punches a parametric disk. A
  tool that doesn't overlap the subject refuses the whole operation (it would be consumed for
  nothing); a subtract that would sever the body is refused like a Cut ("use Split"), and so is
  a disconnected intersection (no multi-body result).
- `boolean.ts` classifies each planar-graph edge by sampling just left / right of its midpoint.
  Since 2026-09-22 the sample offset is capped at half the edge's length and dangling boundary
  spurs are pruned before the pinch test: a body vertex a hair (between the merge tolerance and
  the sample offset) outside another body's edge makes a sliver edge that used to read as a
  boundary and refuse the operation as "pinched" (seen with a disk snapped onto a plate corner).
- Line, arc and text are reference-only; text is for labels (cut/engraved text is phase 4).
- Offset-mode bodies are **baked** to fillet control polygons before Split / Combine / general cuts
  (their rounded shape is larger than their control polygon).
- Fillet: centre on the bisector of the two edge directions (correct for convex and reflex corners);
  shared-edge budget + opposite-edge clamp so fillets never overlap or fold thin shapes. Arc sampling
  is 7.5°/segment everywhere (matches the DXF importer's reconstruction tolerance).
- DXF import reconstructs fillets (tangent arcs < 180° → sharp corner + per-corner radius) and
  circles → parametric disks; export emits exact arcs (bulges) and round-trips through the importer.
- Guides are finite since v21 (the infinite guideline is gone); guide constraints never move
  geometry unless the guide is tied / multiply demanded (see ranks).

### UI conventions
- **Sketch badges** are 11 px pills carrying one glyph; `SKETCH_SYMBOL` maps a kind to a
  character, or to `null` when the glyph is vector art (`fixed`'s padlock, `drawLockGlyph`,
  `symmetric`'s dashed mirror with a dot each side, `drawMirrorGlyph`, and `tangent`'s circle
  touching a line, `drawTangentGlyph` — each matches its
  toolbar icon, and no character reads right at that size: everything meaning "locked" is an
  emoji or a shape-in-a-shape like the coincident ◎, and ⇔ / ⋈ read as equivalence / join).
  `SvgRecorder` replays drawn glyphs fine. A symmetry badges all three of its elements; a
  tangent has **one** badge at the contact point (the centre's foot on the line, offset away
  from the circle — the one spot that names both elements), computed in `sketchGlyphsView`
  rather than per ref. Hovering a badge highlights a circle ref as its rim / arc
  (`highlightOfRef` in main.ts, `MeasureHighlight` in the renderer's `sketchDraft` and badge
  hover) and draws no link for a tangent (they touch by definition).
- **A dimension's direction is fixed at placement.** The h / v / direct choice of a point–point
  dimension is read from the label position only when it is created (`measureAxisForPlacement`);
  `setMeasurementLabel` and the context dimensions' `setTempDimLabel` move the label and nothing
  else (they used to re-derive the axis, which silently changed what a dragged dimension
  measured). Because the label can then sit anywhere, the pill leads with a direction glyph
  (`MeasureInfo.axis`, set only by `pointPointInfo`; `drawAxisGlyph` — vector art like the
  padlock, a fixed 45° diagonal for direct so it never looks like h on a level pair). The glyph
  is also the **only** control that changes the direction: a click cycles h → v → direct
  (`clickDimensionGlyph` in main.ts → `Scene.setMeasurementAxis`); a driving dimension keeps its
  target and re-solves along the new axis through `applyDimensionValue`, reverting on a reject.
  Pill layout lives in the renderer (`labelLayout`) and label hit-testing goes through
  `dimensionLabelHit` on the same layout, with the old 16 px pick circle kept as a floor. A
  label dragged past an end of its dimension line gets a dashed leader (`pushLeader`).
- Tools are **one-shot** (Rotate is a mode; the polyline/body draft spans clicks). Toolbar wiring is
  by id / `data-*` / class, never button text **and never by position** — the user reorders the
  groups. All user-facing warnings go through `notify`.
- **Toolbar = a rack of draggable groups** (`#tb-sections > .tb-sec`, `src/toolbar.ts`): each is a
  two-row grid filled *column-major* (`grid-auto-flow: column`, so markup order reads down-then-
  across; `.tb-tall` / `.tb-mid` span or centre a lone control) under a caption strip that is also
  its drag handle. One group opts out: **Grid** puts its four controls in two `.tb-row` flex rows
  (spacing + units, then line style + colour), because the column grid tied the spacing button's
  width to the wide line-style sample; each row now shares out the group's width itself, the
  sample taking whatever the square colour chip leaves. Dragging reorders the DOM live and FLIP-animates the others, deciding insertion
  on `offsetLeft`/`offsetTop` (layout values, immune to the drag transform and to a running
  slide); double-click a caption to reset. Per-mode visibility is the `draw-only` / `sim-only`
  classes, never a hard-coded group list. Selection- or tool-dependent fields live in `#tb-props`
  **outside** the rack so they can't reshuffle it; the armed-tool hint is the `#statusbar` line.
- **Group visibility toggles are eyes on the caption** (grid, constraint badges, dimensions) —
  the toggle applies to the whole group, and `.tb-cap .cap-eye` has to outweigh `button.active`
  or it inherits the accent fill. The mode switch is one big button showing the mode it switches
  *to* (the theme button's convention); its caption names the mode you are in.
- **A setting that lives among tools is a `.tb-switch`** (today: auto-constraints, `#autocon-btn`
  in the Constraints group): a `tb-tall` pill with a slider under its glyph. The pill stays
  **neutral when on** — it overrides `button.active`, because an accent-filled button means "this
  tool is armed" everywhere else — and the accent lives on the slider track instead.
- **Manual (`public/help/`, `scripts/manual/`)**: one topic per tool, control and canvas element —
  `npm run manual` fails on a topic the app can ask for that the page lacks, and on a referenced
  image it didn't generate. Fixtures are built with the Scene API, which does **not** solve:
  draw the geometry already satisfying its constraints. `fitView` frames bodies only and the
  SVG crop clamps to the canvas, so shots with annotations or reference geometry carry a
  `zoom` (≈0.4–0.5 with bodies; >1 for a reference-only scene) — which is also what keeps the
  11 px badges legible once the manual shows a capture at its own width. UI is shown as
  **markup snapshots** (`kind: "html"`: the generator copies the element's `outerHTML` into
  `public/help/ui.js`; `help.js` renders it `inert`, styled by `../ui.css`), so one snapshot
  follows both themes and every restyle; `png` shots stay as the fallback for anything CSS
  alone can't render (none today). A snapshot keeps its ids, so an element appears once per
  page. Glyph keys are `tool-x`, `role-x`, else
  the element id (`mode-toggle` carries both mode icons); `group:sec-*` rows expand a toolbar
  section. Run `npm run keymap:docs` before `npm run manual`: the shortcut list is generated,
  never edited.
- **Styles are split by role, not by feature** (2026-09-23): `public/ui.css` holds how a control
  or panel *looks*; `src/style.css` holds the page and where overlays *sit* (position, edges,
  z-index, max-height). Put a new rule on the right side — a look rule in style.css is missing
  from the manual's snapshots, a placement rule in ui.css would throw a snapshot out of its
  figure. `index.html` links ui.css root-absolute (`/ui.css`, how Vite addresses `public/`;
  the build rewrites it for the relative base) ahead of the bundled style.css, so the app's
  cascade order is unchanged; in dev Vite injects style.css at run time, also after it.
- Every mutation goes through `markDirty` → snapshot history + autosave + component-context sync +
  pose-baseline reset. `canonicalData()` is the root document without sim poses.
- Session-only state (grid, snap, osnap, visibility toggles, solver tuning) is not persisted;
  theme, help-drawer width, grid presets, the **grid's colour + line style**
  (`disjointed:gridLook`), backup settings and the toolbar group order live in localStorage.
  The grid colour is stored **per theme** (`{ dark, light }`, null = that theme's own tone):
  one colour can't read on both backgrounds. The renderer takes it as `RenderInput.gridColor`
  (absent = `theme.grid`) rather than through a mutated `Theme`, so the palettes stay constant.
- Selection kinds: single `selection` vs `multiSel` (groups and instances are selection-atomic)
  vs `featureSel` (vertices + joints of one body). Ctrl+G is a group toggle; plain G is Ground.
- **Group isolation** (`groupEdit`, main.ts): a double-click opens one group for editing from the
  inside. It is a *view* over the scene's own material — no context switch, nothing serialized,
  nothing in history (unlike `editPath`) — and it does exactly two things:
  - **suspends that one group's selection atomicity**: every site that expanded a group now asks
    `selGroupOf` / `selGroupOfJoint`, which return undefined for the open group. Keep new
    atomicity sites on that pair, never on `scene.groupOf`.
  - **scopes what is live**: `bodyInScope` / `jointInScope` for material, `refsInScope` for
    annotations (in scope = the item names **any** of the group's material, so a dimension from a
    member to the outside stays editable; guides belong to no group and are always outside).
    Faded ⇒ inert is the rule the status bar states — the pickers (`handleSelectClick`,
    `measurementLabelHitAt`, `sketchGlyphAt`, `patternLabelAt`) drop out-of-scope items, while
    ref *targets* (object snap, constraint / measure picks) deliberately still reach outside.
  It is draw-mode only, `leaveGroup()` runs before any mode / context switch, and the frame loop
  drops it when the group stops existing (ungrouped, deleted, undone, loaded over).
- The **isolation veil** is one `theme.surface` rectangle at `ISOLATE_VEIL_ALPHA` over the geometry
  layers, drawn *before* the annotations: they sit above it, so each fades itself by id against
  `isolate.items` (the ids main computed as in-scope). The **grid** is skipped on its usual pass and
  redrawn just after the veil — it is the canvas, not the drawing, so it never fades.
- **A refusal is always explained.** `Scene.sketchConstraintProblem` returns the user-facing reason
  a constraint can't exist on these references, and `addSketchConstraint` refuses **exactly** when
  it returns one — keep the pair in step (scripts/sketch.ts asserts it both ways). The UI's three
  failure shapes: a bad pick (the reason), a click the armed tool can't use at all
  (`reportUnusablePick` — the kind-aware picker used to swallow these), and an unsatisfiable solve
  (`describeBreaks` names the flashing items, identical ones collapsed into a count). Repeated
  identical warnings go through `notifyThrottled`. `SKETCH_FLASH_MS` is long enough (4 s) to find
  the flashing items after reading the toast that named them.
- Two-click slider start pair: a press grabs the **rail joint** in draw mode, the **rider** in sim.
- **A shortcut is data, written down once — and the bindings are written down in a file.**
  `src/commands.ts` is the registry (one entry per command: id, label, one-line description,
  group, context, the tool it arms) and `src/main.ts` holds the matching `COMMAND_ACTIONS`,
  because every `run()` closes over module state that no facade would carry cheaply. The two
  halves are keyed by id and `Record<CommandId, CommandAction>` makes the compiler demand an
  action for each — that type is the only thing keeping them in step, so don't widen it.
  The **keys** are in `public/keymap.json`, imported by `commands.ts`; the tooltips (a button's
  `title` in `index.html` carries **base text only**; `data-cmd` — or `data-tool` — names its
  commands and the key is appended at startup) and the README / manual lists are generated from
  the two together.
  - **`data-cmd` wires no click.** Every button has its own `addEventListener` in `main.ts`
    beside the others; filling in `COMMAND_ACTIONS` alone gives the key a target but leaves the
    button dead (how Subtract / Intersect first shipped on 2026-09-22).
  - **Precedence is `localStorage` → the file → nothing.** A command neither binds has no key,
    which is a legitimate state (23 of 81 ship that way) — there is no third fallback under it.
    Build-time import, not a fetch: no async start-up, no race, and `file://` still works. The
    cost is that a *deployed* copy's `dist/keymap.json` is only what KeyMapper fetches — the
    app's own defaults were inlined at build time.
  - A keymap file the app cannot parse leaves **every command unbound** rather than throwing:
    `keymapFileError` carries the reason, `main.ts` raises a toast at startup and the Shortcuts
    panel says so, because without keys that panel is pointer-only. `scripts/keymap.ts` holds
    the file and the registry to the same command list and the same metadata, and refuses two
    commands on one slot in overlapping contexts.
  - **Dispatch** is one map lookup on a *slot*: `KEY|mods`, mods always `ctrl+shift+alt`. Shift is
    part of the slot for letters and named keys, never for a character (`?` arrives as `"?"` with
    shiftKey true — recording Shift would build a slot nothing can produce). Exact slots are why
    ordering bugs are gone: `Shift+F` and `Shift+N` were unreachable under the old `if` ladder,
    which is why the remap could hand `Shift+F` to Fit view the same day.
    Resolution: first command on the slot whose `context` matches the mode (`any`
    matches both) and whose `enabled()` is happy; a refused command is **not** swallowed, so the
    keystroke still reaches the browser.
  - Five commands **branch internally and must stay one command each** — `Esc` (the layered
    cancel), `Delete` (measurement vs selection, both modes), `↑` / `↓` (armed polygon tool vs
    selected regular polygon), `Enter` (close a polyline vs commit a pattern count), `[` / `]`
    (disk vs corner radius). Modelling their states as separate bindings would put them back in
    a race.
  - Four commands carry `whileTyping` (save, save as, open, `F1`): they resolve *before* the
    focus-in-a-field bail-out. Note that `#export-panel` / `#backup-panel` / the grid menus stop
    propagation for their own fields, so a key pressed while focus is still inside one never
    reaches the handler at all.
  - A user's keymap is the whole `{commandId: bindings[]}` map in `localStorage`
    (`disjointed:keymap`), **not a diff** — a diff against a moving default is a migration problem.
    A command the map doesn't mention keeps its default, so adding one is always safe. The
    Shortcuts panel (File group) imports / exports / resets it; rearranging a layout is
    KeyMapper's job, not this app's.
- View rotation is purely visual (world axes for constraints, grid and snapping).

## Serialization history (`load` accepts everything ≤ 22)
v5 control polygons · v6 actuators/motors · v7 measurements · v8 sketch constraints + driving ·
v9 groups · v10 grounded bodies · v11 guides · v12 units · v13 holes (baked) · v14 components +
group joints · v15 per-corner radii · v16 editable holes · v17 locked riders · v18 welds ·
v19 patterns · v20 guide union + `startRiders` · v21 regular polygons, infinite guideline dropped ·
v22 pattern `slots` (several seeds per pattern; a v19–21 `seed`/`members` record migrates on load).
Optional fields added without a bump: `Measurement.side`, `mirrored`, `rigid`, the `fixed`
sketch-constraint kind with its `at` / `angle`, the `symmetric` kind with its `mirror` ref, the
`tangent` kind and the `disk` / `guideCircle` ref kinds. Load sanitizes
every list invariant (locked ⊆ riders, pattern members exist, mismatched regular counts dropped).

## Tests (`scripts/`, one line each)
solver-smoke (slider-crank + end-stops) · free-rail · ground-drag · impossible-assembly ·
persistence · build-body · shape-edit (fillet, containment, node↔joint link, radii, holes) ·
edit-utils (rotate/mirror/copy/z-order) · actuators · measurements (incl. diameter/radius) ·
sketch (constraints, dims, ranks, rigid carry, drift, refusal reasons ↔ refusals in step) ·
fixed-constraint (point / line locks,
conflict rejects, mirror re-capture) · symmetric-constraint (validation, point / line forms,
who moves — free / twice-demanded / tied / locked mirror, drag follow — rejects, remaps,
load, copy/paste, pose route incl. a mirror riding with the moved side) · tangent-constraint
(validation incl. problem ↔ refusal in step; who moves — locked edge / free reference circle /
rigid arc / locked disk / equal ranks; drag follow; a diameter edit re-solving the tangent;
conflict reject; remaps: node added to the hole, hole removal, cascades, copy/paste, load;
sketch vs pose route; the line-blends-into-arc idiom on the field-repro geometry — two long
reference lines and a ~154° arc on 6000-unit coordinates — incl.
sweep count, an H-held line, point-on-line pinning and a drag of the shared end) · equal-radius
(validation — lengths vs radii, arcs copy-only, seed ↔ member, instances; who follows — first
pick, dimension, arc, instance, direct resize; classes; uniform vs mixed corners; a follower
disk's tangent; conflicts untouched; the violated flag; remaps, load, copy/paste) · midpoint (midpoint refs: resolve, validate, solve, who
moves, follow-the-line remaps, load) · groups · group-joints · grounded-bodies ·
freeze-drag · slider-locks · two-click-slider · welds (incl. chain regression with solver stats) ·
components · pose-dims · pose-constraints · split-combine · boolean-ops (subtract / intersect:
primitive, holes, refs, refusals) · shapes (cut, references, v21 load) ·
regular · guides · patterns · features · context-ghost · view · dxf · export ·
keymap (key spelling, slots, conflicts, file parsing, overrides; then the three agreements:
the registry with itself, `public/keymap.json` with the registry — same command list, same
metadata, no slot wanted twice in one context — and `index.html`'s `data-cmd` / `data-tool` and
its shortcut-free tooltips with both) · keymap-docs `--check --readme`.
`solver-bench.ts` is a benchmark, not in `npm test`; `keymap-live.ts` (`npm run keymap:live`)
needs Chrome, so it isn't either — it is the only coverage **dispatch** can have, and its letters
are written out by hand on purpose: if it and the registry disagree, one of them is wrong.
Each script is a plain assertion list — read it for the exact cases.

## Lessons from past bugs (root causes to not repeat)
- Index-based refs must be remapped on every reordering of a control polygon (mirror reverses
  winding; insert/remove shifts; Split/Combine/Cut rebuild) — and keep the `hole` field.
- Anything that moves a body as a whole must carry holes, joints, ground anchors and owned rail
  tracks (`moveBody` / `rotateBody` / `mirrorBody` do; per-vertex writes don't).
- Hit tests on rails use the **segment**, not the infinite line.
- Never split a correction 50/50 with a dragged element and never keep a stored drag anchor: read the
  live scene each frame (the group-drift bug).
- A single stalled solver item rejects the whole scene's sketch — prefer invariants / direct
  parameter setting over items that can stall.
- **A line-vs-line conflict has an escape hatch: collapse.** `projectParallel` and the H/V line
  items are satisfied by a zero-length line, so "vertical on a horizontal edge" (and a parallel
  between two locked lines) *converges* by shrinking the edge to a point instead of rejecting.
  Long-standing, not specific to any one kind — don't write tests that assume such a pair
  rejects, and treat it as the candidate root cause if a degenerate edge ever appears.
- Gauss-Seidel propagates one link per sweep: chains of pairwise constraints must become composites.
- Errors with non-unique zeros (`sin`) allow flipped poses to read as satisfied.
- Guards that live only in one entry point (the instance-ownership check that lived in
  `setMeasurementDriving`) get bypassed — validate at the model / solver boundary.
- Solver instrumentation exists (`solverConfig`, `SolveStats`, sim-mode tuning sliders, per-run
  animation stats in the console); use it before guessing at convergence problems.
- Point-sampled classification in `boolean.ts` must probe at a distance the edge itself
  allows: a fixed offset read sub-tolerance sliver edges (a vertex a hair off another body's
  edge) as boundary spurs and refused valid cuts as "pinched". Reproduce such cases from the
  user's `test.json` headlessly before theorising — the synthetic exact-position cases all passed.

## In flight (2026-09-23)
- **The manual pass is done (2026-09-23).** Every topic was rewritten for the current UI and the
  pending list in HANDOFF.md was consumed: 22 shots (three of them UI markup snapshots), new topics `subtract`,
  `intersect`, `implicit-constraints`, `group-edit`, `shortcuts-panel`, the Guideline topic
  gone, the stale what's-this strings reworded. The UI-tweak deferral is over: a UI change now
  updates the manual in the same commit.
- **The shortcuts were remapped (2026-09-15) and the layout is not settled.** Fixed `F`, Circle
  `C`, Line `L`, Arc `A`, fit view `Shift+F`, view dial `Ctrl+R`, polyline-as-cut `Ctrl+U`,
  Connect `Ctrl+Shift+C`, actuator `Shift+A`, motor `Shift+M`, **Rail unassigned**. Two of those
  want a second opinion (HANDOFF.md): Rail lost its key where the earlier plan dropped the
  actuator's, and Connect became a three-key chord. Rearranging happens on the KeyMapper board,
  which must be **seeded from `public/keymap.json`** — the board's original hand-written file
  used its own ids and applies nothing. The next layout ships by replacing that file; no `src/`
  edit is involved any more.
- **KeyMapper now lives in its own repo** (`../KeyMapper`), and the empty `keymapper/` folder that
  staged it is gone. Its README specifies `keymap/1` normatively — the only thing the two projects
  share. `src/keymap.ts` implements that spec and must not restate it; if a rule needs changing,
  change it there first, in both projects.
- The **Tangential constraint shipped but needs another pass**: the user's field tests with
  a two-line-and-an-arc scene of their own found it still wanting. HANDOFF.md lists the known weak spots (three-point
  arcs, the line swinging in a blend, no held side, coverage); ask what was seen before
  changing anything.
- Shape-tools phase 1 shipped (v20/v21); phases 2–4 and follow-ups are in HANDOFF.md.
- **Subtract / Intersect shipped (2026-09-22)** with Combine's conventions (first selected
  survives, tools consumed, one-piece results only, no keys); the un-discussed choices are listed
  in HANDOFF.md as open questions. Manual topics: `subtract`, `intersect`.
- The toolbar was regrouped into draggable groups for **draw mode**; sim mode inherits the rack
  but its own grouping (Animation + solver tuning) has not been designed yet — see HANDOFF.md.

## Backlog (not built)
- **Shape tools phases 2–4** (HANDOFF.md): bulge arcs on outlines → arc edges, curved slots, exact
  arc export/import; ellipses (edge *kind*); regions from closed reference chains; cut/engraved text
  (stencil font). Smaller: role conversions, multi-body cuts, patterns surviving general cuts, text
  angle/font/alignment, point-on-circle / tangent / concentric constraints.
- **Auto-pause false positives**: still pauses on some solvable borderline closed loops. Next steps:
  use the tuning instrumentation to find the knob; wire `analyzer.ts` to a debug hook; prototype
  closed-form propagation for tree branches (loop cores stay iterative). Also the "Assembly
  impossible" flicker while dragging an actuator's rider in a closed loop (candidate: project the
  driver target onto the rail first).
- **Manual**: no component / context-ghost / Simulate-mode pictures yet (those fixtures are the
  next to write), no before/mid/after sequences for the shape tools, tutorial steps unillustrated;
  option to let controls act *and* navigate in help mode (one-line change in `help.ts`). The
  draw-mode toolbar is ~1480 px wide and wraps groups below that.
- **Patterns**: "make independent" UI undecided (`dissolvePattern` exists); labels not draggable;
  whole-body patterns; patterns inside defs don't expand as patterns. A dimension on a **turned
  circular member's** own corners is refused (the member couplings are translation-only, so the
  seed is pushed along the member's axis, not its own — dimension the seed instead); a
  rotation-aware coupling (`R_k` from the layout) would fix it for `rotate: true` patterns.
- **Components**: no per-instance scaling; no ports; per-instance actuator/motor speed overrides are
  lost on cascade; no thumbnails / drag-to-place in the browser; pose dims between instances aren't
  carried by copy/paste; a refused pose constraint gives no feedback (nothing to flash).
- **Sketch**: driving angle dimensions; radius dim on a sharp corner can't be picked (nor can Equal
  pick one — round it first; the model accepts a sharp corner's vertex as a radius of 0); Equal can
  copy a reference arc's radius but not set it — a real arc parametrisation (HANDOFF.md, the
  Tangential pass) would lift that. The implicit
  constraints' switch ships with no key (it is `sketch.autoConstraints` in the registry, so giving
  it one is now a line in the keymap rather than a code change), and there is no point-on-point coincident between
  a dragged *line* and a candidate (lines only take point-on-line). Midpoints are drag / placement
  inferences only: neither the constraint tools nor the Measure tool pick them (a Measure pick
  would need the dimension items to go through `PointHandle` too), and midpoints are not drawn
  until hovered / armed. **Fixed** takes points and
  lines only — there is no "lock this whole body" (two locks pin one rigidly, but a body click
  would be the obvious gesture); a locked element gets no styling of its own beyond its badge, so
  a body deforming around a lock under a drag is only explained by the badge.
- **Subtract / Intersect**: the tools are always consumed (no keep-tools modifier), a result in
  several pieces is refused rather than split into several bodies, and neither has a key.
  **Tangential** gaps: no circle–circle tangency; no tangent to a rounded corner's fillet arc (that arc is
  derived from the corner radius and its two edges — a different beast); no point-on-circle
  or concentric constraints (a circle ref *resolves* to its centre, but the constraint tools
  never pick one as a point — coincident onto a disk's centre goes through its vertex / `c`
  point as before); a reference arc is still three points, not centre / radius / angles —
  a held point makes the rest take a Newton step along the normal, which is a heuristic
  rather than a true arc parametrisation; a rail whose joint sits at the
  disk's own centre can never be tangent to it and is refused by the solve, not structurally.
  **Symmetrical** gaps: no symmetric between a point pair and a line
  pair at once, no "symmetric about a body's own axis" without a reference line, and the
  constraint tools still don't pick midpoints (so a midpoint can't be one of the pair).
- **Group isolation**: no keyboard way in (double-click only), no way to *add* an outside body to
  the open group without leaving, and Ctrl+G inside it dissolves the group (the selection is one
  group, so the toggle ungroups) — defensible, but a "group these members into a sub-group" would
  need nested groups, which the model doesn't have. Groups can't be named, so the crumb reads
  "Group (n parts)". An isolated group has no equivalent of the context ghost's eyes (the fade is
  all-or-nothing), and entering one in sim mode is deliberately impossible.
- **Refusal feedback**: pose-route failures (`applyPoseConstraint`) report the constraint itself as
  the conflict, so the toast can only say "the rest of the sketch" — naming the *pose* item that
  blocked it needs `settlePose` to return which item failed. A refused pose constraint between two
  rigid instance ends still has nothing to flash (see Components).
- **Context ghost**: can't switch reference instance; no hotkey for the eyes; ghost drops parent
  guides; temp dims are per session.
- **Sliders / welds**: Connect tool doesn't create welds; second click of a two-click slider snaps
  but records no coincident; shared / moving tracks are carried by nobody (symmetric "track body
  carries owners" rule is the natural extension); weld toggle on instance pins isn't blocked.
- **Holes**: hole-aware picking option; hole edges aren't containment-checked (vertices only).
- **View**: screen-aligned marquee when rotated; persistent angle badge; 5° nudge hotkey.
- **DXF**: SPLINE sampling, INSERT/blocks, ellipses. Copy/paste doesn't carry driven measurements.
- Actuator/motor speed editing while in sim (selection clears on mode change).
