# Disjointed — Project State

## Goal
A simple webapp to create and simulate 2D planar mechanisms (moving joints / linkages).
Two modes:
- **Drawing mode** — draw bodies, place joints (attached or free), and couple them with
  constraints (rotation/pin, ground, slider). Joints on the same body are rigid relative to
  each other. Bodies have an editable control polygon + corner radius; the rounded outline is
  derived from it.
- **Simulation mode** — drag any joint, or any point on a body, to drive the mechanism; the
  grabbed point follows the cursor and a constraint solver propagates the motion through
  everything connected to it.

## Current status
End-to-end draw → simulate works, with: select/move/delete editing; one-shot tools + keyboard
shortcuts; free (body-less) joints that can be grounded as anchors; bodies built two ways
(freehand polygon, or from existing joints); rounded corners (editable control polygon +
radius, re-editable by dragging corner handles, with vertices added/removed by double-click,
and a fillet that handles convex + concave corners without overlapping on thin shapes);
sliders with end-stops whose rail is either two
joints on one body (a moving rail) or two free joints (a world-fixed track, auto-grounded),
with joints auto-attached as riders when placed on a rail (Joint tool or body-from-joints);
a configurable, toggle-able grid with snap-to-grid for placement and dragging;
editing utilities (copy/paste a body+its joints/constraints, mirror H/V in place, and a rotate
tool that turns a body about its centroid or a node and snaps to 45°);
and a converge-to-tolerance solver with **impossible-assembly handling** (grounds are sacred;
unreachable pins/sliders are isolated, the rest still solves, and the breaks are drawn as red
dotted lines plus an on-canvas error banner). The solver and shape/edit logic are verified
by headless tests; the interactive canvas should be confirmed by eye via `npm run dev`.

### Tech stack
- **Vite + TypeScript + HTML5 Canvas** (no UI framework). Builds to static files.
- Constraint solver: **iterative position-based** (Gauss-Seidel projection).
- Node 24 / npm. Scripts: `npm run dev`, `npm run build`, `npm run preview`, `npm test`.

### Architecture (`src/`)
- **geometry.ts** — Vec2 math; polygon centroid / area / inertia; point-in-polygon;
  point-to-line distance; `distToSegment` (point to a clamped segment); `convexHull`;
  `roundedConvexBody` (hull + outward rounded offset = Minkowski sum with a disk, sampled
  adaptively to `margin`); `filletPolygon` (round a polygon's corners in place with tangent
  arcs — convex and concave/reflex corners). The fillet runs in passes: per-corner desired
  tangent length, a **shared-edge budget** (each edge split between its two corners in
  proportion to demand so neighbouring fillets can't overlap), an **opposite-edge clamp**
  (a few relaxation passes shrink any corner whose inscribed circle would poke through a
  non-adjacent edge — stops thin shapes folding at large radii), then arc emission. The
  fillet centre is always on the bisector of the two edge directions, which is the correct
  tangent side for both convex (into the body) and reflex (into the notch) corners.
- **model.ts** — `Scene` owning:
  - `Body` = rigid shape defined by an **editable control polygon** (`controlLocal`) + a
    corner `radius` + a `round` mode (`"fillet"` rounds corners in place; `"offset"` grows the
    hull outward). The render/physics polygon `local` is **derived** from these. Pose is `pos`
    (world centroid) + `angle`. `invMass`/`invInertia` from the derived polygon's area / second
    moment. `rebuildBody` regenerates `local`/centroid/mass and re-anchors attached joints when
    the centroid shifts.
  - `Joint` = a point with `bodyId` + `local`. If `bodyId` is a body, `local` is the offset
    from that body's centroid; if `bodyId === null` it is a **free joint** and `local` is its
    own world position (the solver treats it as a movable point particle).
  - Constraints: `pin` (two joints coincide, free rotation), `ground` (a joint locked to a
    fixed world point — grounding a free joint makes a body-less anchor), `slider` (a rail =
    two joints `railA`/`railB`, plus a `riders` list of joints confined to the segment between
    them, with end-stops). The rail is either **two joints on one body** (it moves with that
    body, coupling two bodies) or **two free joints** (a track fixed in world space — `addSlider`
    auto-grounds them). Riders may be body joints or free joints.
  - Body construction: `addBody(worldVerts, radius?, round?)` (freehand uses `fillet`);
    `buildBodyFromJoints(jointIds, margin)` stores **one control point per joint** with
    `round: "offset"`. Per joint: a **loose free joint (or a free slider rider) is absorbed**
    into the new body; a **joint on another body, or a grounded free joint (an anchor)**, gets a
    coincident new joint **pinned** to it (so the anchor stays independent) — this includes a
    **rider that belongs to another body**, where the pin joins the two bodies at that point so they
    ride the slider together through the shared pin; a **slider rail node** instead gets a coincident
    new joint **attached to that slider as its own rider** (the body connects to the slider track,
    not pinned to a rail endpoint). Editing: `moveBodyVertex`,
    `insertBodyVertex` (add a control vertex),
    `removeBodyVertex` (drop one, kept ≥ 3), `setBodyRadius`, `moveJoint`, `moveBody` — all go
    through `rebuildBody`, so attached joints stay anchored.
  - Edit utilities: `rotateBody(id, pivot, delta)` rigidly turns a body about a fixed world
    point (joints follow; ground anchors rotate too; no rebuild needed). `mirrorBody(id, "h"|"v")`
    reflects the control polygon + attached joints + their ground anchors across a centroid axis,
    reversing winding so the fillet/offset stays valid (centroid is fixed, so the body doesn't
    move). Copy/paste: `extractBody(id)` snapshots a `BodyClip` (control polygon + its joints +
    the constraints referencing *only* those joints — grounds, fully-internal sliders, intra-body
    pins; cross-body pins are dropped), stored in world coords relative to the original centroid;
    `insertBody(clip, at)` translates the whole fragment so the centroid lands at `at` and
    recreates everything with fresh ids.
  - Helpers: hit-testing (`bodyAt`, `bodiesAt`, `jointAt`, `sliderAt`), `bodyControlWorld`
    (corner handles), `addFreeJoint`, `attachSliderRider`, `removeBody`/`removeJoint`/
    `removeConstraint` + `pruneConstraint`, pose snapshot/restore, role queries.
  - `serialize()` / `load(SceneData)` for save / load / autosave (versioned plain-data
    snapshot, `FORMAT_VERSION = 5`; `load` deep-copies, recomputes `nextId`, drops legacy
    origin+dir sliders, migrates older single-`slider` → `riders`, and back-fills
    `controlLocal`/`radius`/`round` for pre-v5 bodies).
- **solver.ts** — `solve(scene, driver, iterations, relax): ConstraintBreak[]`. Operates on a
  **host abstraction** (`hostFor`): each constraint participant is a body, a free joint
  (translate-only point), or a fixed world point — which unifies pin/ground/slider/driver. A
  separate **`pinHostFor`** makes *any* grounded joint (free or on a body) a fixed point when
  something *pins* to it, so a pin can't drag the body a grounded joint sits on (the joint's own
  ground constraint still uses the body host, to lock multi-ground bodies' rotation). Returns the
  list of unsatisfiable **`ConstraintBreak`s** (empty when solvable). See "Solver notes" below.
- **view.ts** — camera transform `screen = world * scale + (tx, ty)`; `screenToWorld`,
  `worldToScreen`, cursor-anchored `zoomAt` (scale clamped to MIN_SCALE..MAX_SCALE = 0.2..5).
- **renderer.ts** — draws under the camera transform in world space: world-locked grid
  (spacing = `gridStep`, drawn only when `gridVisible`),
  bodies (selected/hovered highlighted), slider rails as bounded segments with end-caps,
  ground symbols, joints (color-coded: blue = pinned, yellow = grounded, green = slider rider;
  rail joints get a green ring; **a loose free joint gets a muted dashed ring — but a free joint
  that rides a slider or defines a rail drops the dashed ring and renders as anchored**), corner-handle squares
  for the selected body, plus draft overlays (freehand polygon, build-from-joints outline +
  expansion preview, slider rail preview) and a rotate-pivot crosshair. **Impossible-assembly
  breaks** are drawn as **red dotted lines** between the points that can't meet (`input.breaks`,
  sim mode only). Cosmetic sizes are divided by the zoom.
- **main.ts** — canvas/DPI setup, toolbar wiring, mode/tool state (tools are one-shot + have
  keyboard shortcuts), select-mode selection / move / delete / vertex-edit, the camera,
  pointer + key handling, persistence (save/load/autosave) plus a **snapshot undo/redo history**
  (`pushHistory`/`undo`/`redo`; `markDirty` records a step + autosaves), and the
  requestAnimationFrame render/solve loop. `timedSolve` captures the solver's `ConstraintBreak`s
  into `solveBreaks`; `updateSimError` shows/hides the red **"Assembly impossible"** banner
  (`#sim-error`), and the breaks are passed to the renderer in sim mode (cleared on leaving sim).
  The **Joint** tool auto-attaches a placed node to a slider when it lands on a rail/rail-node;
  the **body-from-joints** draft turns a bare slider-rail click into a grid-snapped rider joint
  (tracked in `jointDraftCreated`, removed if the draft is aborted). Also a `timedSolve` debug log.
  - **Edit utilities**: `copySelection`/`pasteAt` (clipboard is a `BodyClip` held in `main`;
    paste lands at the cursor, grid-snapped), `mirrorSelection("h"|"v")`, and a **rotate** tool.
    Rotate is a persistent mode (not one-shot): `startRotate` picks the pivot (a control node of
    the selected body if grabbed, else the centroid of the body under the cursor) and a
    `rotateDrag` tracks the pointer's swing about it — accumulated/unwrapped so it survives ±π,
    with the resulting absolute body angle snapped to 45° (`snapAngle`/`ROTATE_SNAP_TOL`) and only
    the incremental delta applied per move via `scene.rotateBody`. Copy/paste are also on
    `Ctrl/Cmd+C`/`V`; rotate on `R`.
  - **Grid / snapping** (session-only state, not persisted): `gridVisible`, `snapEnabled`,
    `gridStep` (clamped 1–200 via `parseGridSize`; number input + a preset `<select>`). `snap(p)`
    rounds a world point to the nearest grid intersection when enabled (identity otherwise) and is
    applied to placements (free/attached joints, freehand vertices) and to drags. Drags snap an
    **anchor** in absolute terms via a `grabOffset` captured at mousedown: a vertex reshape snaps
    the grabbed control vertex; a whole-body move snaps whichever of the centroid / control
    vertices is nearest the grab point (`bodyDragAnchor`), stored as a fixed `anchorOffset` from
    the centroid (`dragAnchorWorld` reconstructs its live position).

### Interaction model
Draw-mode tools are **one-shot**: arming a tool (toolbar button or first-letter shortcut —
`B`/`J`/`C`/`G`/`S`) lets you place one element, then it returns to **Select** mode. `Esc`
aborts the current placement.
- **Body** (`B`) — first click decides the mode. On **empty space**: freehand polygon (click
  vertices; click the first vertex / double-click / Enter to close; Esc cancels). On an
  **existing joint**: build a body *from joints* — click joints to outline (loose free joints
  absorbed; grounded free joints, joints on other bodies, and a **rider on another body** get a
  pinned twin — pinning to a cross-body rider joins the two bodies so they ride the slider together;
  a **slider rail node** gets a twin attached to the slider as its own rider; a bare **slider-rail**
  click drops a rider point there), click an already-added joint to finish, then move the cursor
  out to size the outward margin (live preview) and click to finalize.
- **Joint** (`J`) — click inside a body to attach a joint; click where bodies overlap to drop
  a joint in each and pin them together; click **empty space** to place a free (body-less) joint.
  A node placed on a **slider rail (or rail node)** is auto-attached to that slider as a rider.
- **Connect** (`C`) — click a joint, then another joint on a *different* body to pin them, or a
  *slider rail* to attach the joint to it as a rider.
- **Ground** (`G`) — click a joint to lock its world position (grounding a free joint makes a
  body-less anchor; a body can still rotate about a grounded joint).
- **Slider** (`S`) — click two joints on the *same body* (a rail that moves with it), or two
  *free joints* (a world-fixed track — they get grounded automatically), to create a slider rail
  (riders are attached later via Connect). A free+body or cross-body pair restarts the draft.

Select mode (default, no tool armed):
- Click a body, joint, or slider rail to **select** it (highlighted); **left-drag** moves the
  selected body or joint. A selected body shows **corner handles** — drag one to reshape it
  (`moveBodyVertex`); **`[` / `]`** decrease / increase its corner radius (round/un-round).
  **Double-click** an edge of the selected body to add a control node there (`insertBodyVertex`,
  grid-snapped), or a node to remove it (`removeBodyVertex`, kept ≥ 3). Clicking on/near the
  selected body's control polygon keeps it selected (so an edge double-click isn't lost).
- `Delete` removes the selection: a body takes its joints/constraints with it; a slider keeps
  its joints; a joint detaches from any rail and any constraints referencing it go.
- **Edit utilities** (toolbar edit group, act on a selected body): **Copy/Paste**
  (`Ctrl/Cmd+C`/`V` or buttons) duplicates a body with its joints + own constraints; the copy
  lands at the cursor (grid-snapped) and is selected. **Mirror H/V** reflects the body + joints
  in place about its centroid.

Rotate tool (`R`, draw mode — a mode, not one-shot):
- Drag a body to rotate it about its **centroid**; drag a **control node** of the already-selected
  body to rotate about that node. The body's absolute angle snaps to the nearest 45° when within
  ~2°. Joints and ground anchors rotate rigidly with the body. Esc / another tool exits.

Simulate mode:
- Drag any joint, or any part of a body. It becomes the driver; the grabbed point follows the
  cursor and connected bodies move with it. Joints take priority over the body underneath; with no
  joint under the cursor, `bodyAt` grabs the body and the grab point (a body-frame offset) is
  driven. Entering sim snapshots all poses and runs a settle solve; leaving sim restores the drawn
  layout so editing is non-destructive.
- **Impossible assemblies** are flagged, not faked: grounds never move, the solvable parts still
  solve, and every unsatisfiable pin/slider draws a **red dotted line** between the two points
  that can't meet (pulled as close as the assembly allows), with a red **"Assembly impossible"**
  banner. A connected impossible piece does not disturb the parts that can be solved.

Navigation (both modes):
- **Mouse wheel** zooms toward the cursor.
- **Right-drag always pans** the view (anywhere). Moving elements is left-drag in select mode
  (above); there is no right-drag-to-move.

Grid (toolbar grid group):
- **Grid** button toggles grid visibility; **Snap** button toggles snap-to-grid; a number field
  (1–200, with a preset dropdown) sets the spacing — both the drawn grid and the snap increment.
  Visibility and snapping are independent. See main.ts "Grid / snapping" above for what snaps.

Undo / redo (draw mode):
- `Ctrl/Cmd+Z` undo, `Ctrl/Cmd+Shift+Z` / `Ctrl/Cmd+Y` redo. Snapshot-based: every mutation
  records a (deduped) JSON snapshot of the drawn layout; undo/redo restore snapshots without
  recording new steps. History is seeded on startup and capped at `HISTORY_LIMIT` (100).

Persistence:
- **Save** downloads `mechanism-<timestamp>.json`; **Load** opens a `.json` via the file
  picker (invalid files alert instead of breaking).
- The drawn layout is autosaved to `localStorage` (debounced) on every mutation and restored
  on startup. Simulated poses are never saved — save/autosave use the canonical (pre-sim)
  poses, and loading exits sim and resets the view first.

### Solver notes (important design decisions)
- Each constraint is satisfied with effective-mass positional impulses:
  `pos += invMass·λ`, `angle += invInertia·cross(r, λ)`.
- **Converge to tolerance, not a fixed count.** After the main sweeps, the solver keeps
  running structural-only sweeps until the worst constraint error is below `STRUCTURAL_TOL`
  (capped at `MAX_CLEANUP_SWEEPS`), and bails early when already tight. This runs on every
  solve (settle and drive), so complex/closed-loop mechanisms hold without visible drift.
- **Structural constraints take strict priority over the mouse driver** — the convergence
  loop is structural-only, so the driver can never drag a ground point or break a pin/slider.
- The **mouse driver** (`Driver`) pulls a point toward the cursor target. The point is either an
  existing joint (`jointId`, via `pinHostFor` so a grounded joint stays put) or an arbitrary point
  fixed in a body's frame (`bodyId` + `local`, a body translate+rotate host built by `driverHost`) —
  the latter is what lets the user grab any part of a body. Grounds still win (projected every
  sweep), so dragging a grounded body's interior pivots it about its ground rather than tearing it
  loose, exactly as driving a non-grounded joint on that body does.
- The driver is **step-limited** (`DRIVER_MAX_STEP`): it pulls the point toward the cursor by
  at most a small amount per sweep. This keeps the linearized correction valid, so an
  unreachable target makes the joint walk stably along its feasible path to the nearest
  reachable point instead of overshooting.
- **Slider** = prismatic with end-stops. `solveAxis` applies a scalar impulse along a direction
  fixed in the rail's frame; the slider runs it for the perpendicular (stay on the line) plus a
  one-sided tangential limit at each rail endpoint. The rail side is a **`RailHost`** (like a
  `Host`, but the reaction acts at the rider's point via `applyAt`): either a rigid body
  (translate + rotate) or an **immovable world line** (zero mass/inertia, no-op `applyAt`). A
  `railKind` predicate classifies a rail as `"body"` (two joints on one body), `"fixed"` (two
  grounded free joints), or `null` (unsolvable) — the sweep and the residual check share it so
  they agree. The body-rail angular Jacobian reduces to `cross(u, pQ − posR)`; the fixed rail is
  just the zero-mass degenerate case (single-sided, like a grounded body). A rider may be a body
  joint or a free joint.
- **Host abstraction** (`hostFor`): every constraint participant is reduced to a `{ point, pos,
  invMass, invInertia, apply }` host — a body (translate + rotate), a free joint (translate
  only, zero inertia), or a fixed world point (immovable). This unifies pin/ground/slider/driver
  and lets the solver move free joints. A **grounded free joint is treated as a fixed host for
  every constraint**, so a heavy body pinned to it is pulled onto the anchor (rather than the
  light point being shoved around).
- **Grounds are sacred / inviolable.** `projectGrounds` runs at the end of every structural sweep
  and snaps each grounded joint exactly onto its anchor (a grounded *body* joint by translating
  its body — combined with the sweep's rotation that's a pivot about the anchor; a body with
  several grounds is moved by the *average* correction, so conflicting grounds settle
  deterministically rather than teleporting). Separately, **`pinHostFor`** makes *any* grounded
  joint a fixed point for pins/sliders/the driver, so a pin to a grounded joint-on-a-body pulls
  only the *other* side and never drags that body. (The joint's own ground constraint still uses
  `hostFor`'s body host, which is what locks rotation when a body has two grounds.)
- **Impossible assemblies → break-and-exclude.** `solve` returns `ConstraintBreak[]`. Phase A is
  the normal solve; if it converges, no breaks. If not, Phase B greedily disables the
  worst-violated *non-ground* unit (a pin, or a single slider rider — ids are globally unique)
  and re-settles (under-relaxed for stability) until the remaining active constraints can be
  satisfied — grounds are never disabled, so the disabled units are exactly the unreachable
  pins/sliders. Phase C (`closeBroken`) gently pulls each disabled unit shut using only the
  assembly's leftover freedom, fully re-tightening the active set after each nudge so solved
  parts are never disturbed. `breaksForBroken` reports the remaining gaps (plus any ground that
  genuinely can't be met). This whole path runs **only** for unconverged scenes, so solvable
  simulation is unaffected (and dragging an impossible scene costs a few ms).

### Tests (`scripts/`, run with `npm test`, executed via tsx)
- **solver-smoke.ts** — grounded slider-crank (coupler riding a grounded rail) driven through
  a full revolution; asserts ground / pin / slider stay satisfied (sub-micron). Plus an
  end-stop scene: a rider driven far past each rail end is clamped to the endpoints.
- **free-rail.ts** — a slider built from two **free** joints: asserts `addSlider` auto-grounds
  both, a free rider stays on the world-fixed line (zero offset), the rail joints never move,
  and the rider clamps at each grounded endpoint.
- **ground-drag.ts** — drags a joint on a grounded body to far/off-axis/unreachable targets;
  asserts the ground never moves and the joint snaps to the nearest reachable angle.
- **persistence.ts** — round-trips a scene through `serialize → JSON → load`; asserts counts,
  exact joint positions, constraint kinds, slider riders, `nextId` continuation, deep-copy
  independence, and rejection of malformed data.
- **build-body.ts** — `buildBodyFromJoints`: free joints absorbed, a coincident pinned joint
  added for a joint on another body, expanded body has area / contains the joints, min-joint
  rejection; a **grounded free joint** kept as an independent anchor (pinned twin, not absorbed);
  a **slider rail node** gets a rider twin (no pin); a **free slider rider** absorbed and stays a
  rider; a **rider that belongs to another body** gets a pinned twin (one pin, no second rider) so
  the two bodies are joined and ride the slider together.
- **impossible-assembly.ts** — over-constrained scenes: a grounded joint stays *exactly* fixed
  while an unreachable pin breaks; a solvable four-bar with an impossible pendant keeps the
  four-bar intact (only the pendant flagged); and the key case — two grounded pieces pinned at
  their free ends, plus a third piece pinned across both grounds at an impossible span — solves
  the good pin and flags only the third piece, with both grounds unmoved.
- **shape-edit.ts** — `filletPolygon` (convex + concave validity, radius 0 passthrough; a
  **reflex corner rounds into the notch, not the material**; a **narrow-neck shape stays
  simple — no self-intersection — across radii up to 200**) and body editing (`setBodyRadius` /
  `moveBodyVertex` keep attached joints anchored; `insertBodyVertex` / `removeBodyVertex` change
  the control-vertex count, keep joints anchored, and enforce the 3-vertex minimum).
- **edit-utils.ts** — `rotateBody` (90° about the centroid carries the joint + ground anchor; a
  pivot node stays fixed), `mirrorBody` (joint reflected, centroid + area preserved), and
  copy/paste (`extractBody`/`insertBody`: independent offset duplicate, joints/grounds/sliders
  duplicated with fresh ids, cross-body pins dropped).

## Bugs found & fixed so far
- **Slider correction sign was flipped** (`-c/w` → `c/w`); sliders pushed joints away from
  their rail and the solver diverged. Under-relaxation only masked it.
- **Driver could drag a ground point away** when the target was unreachable. Fixed with
  strict structural priority + a step-limited driver.
- **Pins drifted on complex mechanisms** because the solver ran a fixed sweep count. Fixed by
  converging structural constraints to a tolerance instead.
- **Ground symbol stayed behind when moving a body** in draw mode. Fixed: `moveBody` carries
  the body's ground anchors.
- **Grounded free joint drifted** when a heavy body was pinned to it (the light point got
  shoved around). Fixed: a grounded free joint is a fixed host for all constraints.
- **A free joint attached to a slider looked unconnected.** The attach worked all along (model
  + solver support a free rider), but the renderer kept drawing the rider with its "loose free
  joint" dashed ring, so it never read as connected. Fixed: a free joint that rides a slider is
  no longer treated as loose (`isFree = bodyId === null && !roles.slider.has(id)`), so it renders
  as a normal green rider. Also tightened the Connect tool: a second-pick click that lands on the
  already-selected joint now falls through to the slider underneath instead of cancelling (lets a
  free joint sitting right on the rail be attached by clicking it).
- **Fillet rounded concave (reflex) corners the wrong way.** The corner-rounding flipped the
  fillet centre's bisector for reflex corners, putting the arc on the material side (and breaking
  tangency) — so concave junctions like a neck/base "armpit" pinched inward instead of rounding
  out. Fixed: the centre always sits on the bisector of the two edge directions, which is the
  correct tangent side for both convex (into the body) and reflex (into the notch) corners.
- **Fillets overlapped / folded on thin shapes.** Each corner was clamped independently to half
  its shortest edge, so neighbouring fillets collided on short edges (and could fold a narrow
  neck). Fixed with a shared-edge budget (proportional split) plus an opposite-edge clamp.
- **Building a body from a *grounded* free joint absorbed the anchor.** A grounded free joint used
  in a body-from-joints build was folded into the body (losing the standalone anchor). Fixed: it
  now gets a coincident pinned twin like a joint on another body, keeping the anchor independent.
- **Impossible assemblies moved grounded joints and over-reported.** The old fixed-sweep solver
  let an unsatisfiable constraint shove grounded joints around, and in a connected assembly the
  error smeared across *every* constraint so solvable joints were flagged too. Root cause for the
  smearing: a pin to a **grounded joint that sits on a body** used a movable body host, so it
  dragged that body. Fixed with (a) `pinHostFor` — any grounded joint is a fixed point for
  pins/sliders/driver; (b) hard ground projection every sweep; and (c) break-and-exclude: only the
  genuinely unreachable pins/sliders are disabled and drawn as red dotted lines (grounds never
  move), with an on-canvas error banner.
- **Building two bodies on a shared slider rider left them independent.** Building a body from a
  joint that was already a **rider on another body** created a *new, separate* rider coincident with
  it, so the two bodies slid apart freely instead of staying joined. Root cause: `buildBodyFromJoints`
  lumped "rail node" and "rider on another body" into one rider-attach branch. Fixed: that branch now
  fires only for an actual **rail node**; a rider-on-another-body falls through to the pin branch, so
  the new body is **pinned** to the existing rider — joining the two bodies, which then ride the
  slider together through the shared pin.

## Backlog / next steps (not yet built)
- Live-link joint-built bodies to their joints (move a joint → body re-rounds) — currently the
  control polygon is a snapshot taken at build time.
- More joint types as needed.
- Optional: File System Access API for true "re-open the last file by path" (Chromium only).
  Current persistence is download/upload + localStorage autosave (restores the last session).

## Working conventions (from CLAUDE.md)
- Answer bug/feature questions first; confirm before changing code.
- Remind to push to GitHub before big structural changes.
- On finishing / commit: update this file and README.md, then summarize and suggest a
  commit message (the user runs the commit).
