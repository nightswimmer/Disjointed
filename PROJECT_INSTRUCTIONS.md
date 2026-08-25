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
**linear actuators** (a self-driving rider on a slider that travels back and forth in animation
at a configurable speed + motion profile) and **motors** (a pivot + crank pair on a body whose
crank pin orbits the pivot at a configurable angular speed in animation), with a sim-mode
**Run animation** toggle (▶/⏸ button or Space) that drives them all and **phase-fit on play** so
toggling pause/play resumes smoothly from the current pose;
a configurable, toggle-able grid with snap-to-grid for placement and dragging;
editing utilities (copy/paste a body + its joints/constraints — including its fully-internal
sketch constraints and driving dimensions — mirror H/V in place, and a rotate
tool that turns a body about its centroid or a node and snaps to 45°);
and a converge-to-tolerance solver with **impossible-assembly handling** (grounds are sacred;
unreachable pins/sliders are isolated, the rest still solves, and the breaks are drawn as red
dotted lines plus an on-canvas error banner). The solver and shape/edit logic are verified
by headless tests; the interactive canvas should be confirmed by eye via `npm run dev`.
**View navigation**: zoom range 0.05×–20×; a **fit-to-screen** button + `F` shortcut frames the
whole mechanism (bodies, joints, ground anchors) centered with a margin; **Tab** toggles
draw ↔ simulate mode.
**UI polish**: the toolbar is icon buttons with tooltips; a dark/light **theme toggle** (persisted)
themes both the chrome and the canvas; a **body-colour swatch** sets the new-body default or
recolours the selected body (paste keeps the source colour); an **inline speed/profile panel**
appears in the toolbar when an actuator's rider or a motor's body / pivot / crank is selected;
and in draw mode, **dotted connectors** mark constraints whose endpoints don't yet touch
(blue between pinned joints, green from a slider rider to its rail midpoint).
**Solver tuning / instrumentation** (aimed at the auto-pause false positives): the sim-mode
toolbar has four live tuning controls (Phase-A iteration cap, cleanup-sweep cap, structural
tolerance, break tolerance) backed by a mutable `solverConfig`; `solve` takes an optional
`SolveStats` out-param and the animation loop logs rolling per-run solve stats (time, sweep
counts, residuals, error-frame %). A new **`analyzer.ts`** (topology diagnostic, not yet wired
to any UI) reports kinematic islands, DOF, loops, propagation order, and bridge/BCC
decomposition — Stage 1 of exploring a propagation-based solver.
**Joint containment**: an attached joint can never be placed or dragged outside its body —
drags clamp to the nearest point on the outline (the joint slides along the edge), and a
grid-snapped placement that would land outside falls back to the exact click point.
**Node ↔ joint link**: a joint sitting exactly on one of its body's control vertices is
*stuck* to it (how joint-built bodies keep joints and nodes together) — dragging the node
carries the joint, and dragging the joint moves the node, reshaping/re-rounding the body
(the former "live-link joint-built bodies" backlog item). The link is coincidence-based
(no stored mapping), so it survives save/load, copy/paste, mirror, and rotate.
The "Assembly impossible" banner now overlays the **canvas area** (below the toolbar)
instead of covering the toolbar.
**Measurements** (`D`, works in **both modes** — each mode keeps its own set): pick two
references, then place the value label. References anchor to **elements** (joints, body
control vertices, slider rails, body control-polygon edges, or a point fixed in a body's
frame), never to bare coordinates — so in sim the values update live every frame as the
mechanism moves. Point+point measures h / v / direct distance (picked CAD-style from where
the label is placed); point+line measures perpendicular distance to the **infinite** line;
line+line measures the distance while (near-)parallel (< 0.5°) and the **angle** otherwise —
resolved dynamically per frame, so a line pair can flip between the two mid-simulation, with
the label's sector picking θ vs 180−θ. Labels are constant-size pills (cyan accent),
click-to-select, draggable (re-derives h/v/direct), Delete to remove — all in both modes.
Serialized (v7), cascade-removed with their elements, vertex/edge refs remapped across
control-node insert/remove.
**CAD-style sketch constraints** (draw mode, serialization v8): a constraint set —
**coincident** (point–point), **horizontal / vertical** (a line, or a point pair),
**parallel / perpendicular / equal-length** (line–line) — over joints, body control
vertices, body edges and slider rails (reusing the `MeasureRef` system, so constraints
remap/prune like measurements). A new **sketch solver** (`src/sketch.ts`, Gauss-Seidel
projection over *shape*: world positions of control vertices + joints) applies them.
Draw-mode distance dimensions can be **driving** (double-click the label → inline value
input): the *first* driving dimension on an otherwise-unconstrained body **scales it
uniformly about its centroid** (same form factor — internal constraints are
scale-invariant so they may exist); later dimensions move only the involved nodes while
holding every constraint + driving dimension. Driven (reference) dimensions render **in
parentheses** (CAD convention), driving ones plain/bold. Unsatisfiable edits are
**rejected**: the scene is left untouched and the conflicting items flash red.
Constraint badges (violet pills: ◎ H V ∥ ⊥ =) sit by their elements — click to select,
Delete to remove. **Sketch-aware dragging**: draw-mode drags (nodes, joints, bodies) and
rotates live-solve the sketch, CAD-style. **Auto-constraints** while drawing freehand: a
near-H/V edge (±5°) gets the H/V constraint; a vertex clicked on an existing joint/corner
lands exactly there and gets a coincident.

### Tech stack
- **Vite + TypeScript + HTML5 Canvas** (no UI framework). Builds to static files.
- Constraint solver: **iterative position-based** (Gauss-Seidel projection).
- Node 24 / npm. Scripts: `npm run dev`, `npm run build`, `npm run preview`, `npm test`.

### Architecture (`src/`)
- **geometry.ts** — Vec2 math; polygon centroid / area / inertia; point-in-polygon;
  point-to-line distance; `distToSegment` (point to a clamped segment);
  `closestPointOnPolygon` (nearest point on a closed polygon's boundary — used to clamp
  joints inside their body); `convexHull`;
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
    auto-grounds them). Riders may be body joints or free joints. Two new "powered" constraints
    layer on top of the above: `linearActuator` (a slider id + its driven rider id, `speed` in
    cycles/s, `profile: "triangle"|"sine"`) and `motor` (a body + pivot/crank joint ids on it,
    `speed` in revs/s). Off-animation they're inert (the actuator's rider is a normal slider
    rider; the motor's body is a normal body). With the sim-mode animation running, the main
    loop computes a world target per actuator/motor each frame and passes them to `solve` as
    `anchors`; the solver treats those targets like additional (moving) grounds — sacred, never
    disabled — so pins/sliders propagate the imposed motion through the rest of the assembly.
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
    `removeBodyVertex` (drop one, kept ≥ 3), `setBodyRadius`, `moveJoint`, `moveBody` — shape
    edits go through `rebuildBody`, so attached joints stay anchored. **Joint containment**:
    `pointInBody` / `clampIntoBody` test/clamp a world point against a body's rounded outline;
    `moveJoint` (via the private `shiftJoint`) clamps an attached joint's target inside its
    body, so a drag can't take it outside (free joints are unclamped; ground anchors follow).
    **Node ↔ joint link** (`VERTEX_LINK_EPS = 1e-6`): `moveBodyVertex` carries any joint of
    that body exactly coincident with the moved control vertex (moved *after* the rebuild, so
    every other joint stays anchored); `moveJoint` on a joint coincident with a control vertex
    delegates to `moveBodyVertex` — so the link is bidirectional and coincidence-based (no
    stored mapping; survives save/load, copy/paste, mirror, rotate). Edge cases: only the
    body's *own* joints follow a node (a pinned twin's partner on another body stays put — the
    pin shows as a dotted connector until sim closes it), and rounding a freehand body's
    corner after linking dissolves that link on the next drag (the control corner leaves the
    rounded outline, where a joint may not go).
  - Edit utilities: `rotateBody(id, pivot, delta)` rigidly turns a body about a fixed world
    point (joints follow; ground anchors rotate too; no rebuild needed). `mirrorBody(id, "h"|"v")`
    reflects the control polygon + attached joints + their ground anchors across a centroid axis,
    reversing winding so the fillet/offset stays valid (centroid is fixed, so the body doesn't
    move). Copy/paste: `extractBody(id)` snapshots a `BodyClip` (control polygon + its joints +
    the constraints referencing *only* those joints — grounds, fully-internal sliders, intra-body
    pins; cross-body pins are dropped), stored in world coords relative to the original centroid;
    the clip also carries the body's **`color`**, plus its **fully-internal sketch constraints
    and driving dimensions** (both refs on the body's own vertices/edges/frame, its joints, or a
    copied internal rail — anything referencing an outside element is dropped, like cross-body
    pins; driven reference dimensions are annotations and aren't copied). `insertBody(clip, at)`
    translates the whole fragment so the centroid lands at `at`, recreates everything with fresh
    ids (joint + slider ids remapped into the constraint/dimension refs), and restores the
    colour. No re-solve is needed on paste: everything carried is translation-invariant.
  - Helpers: hit-testing (`bodyAt`, `bodiesAt`, `jointAt`, `sliderAt`), `bodyControlWorld`
    (corner handles), `addFreeJoint`, `attachSliderRider`, `removeBody`/`removeJoint`/
    `removeConstraint` + `pruneConstraint`, pose snapshot/restore, role queries.
  - Construction helpers for the powered constraints: `addLinearActuator(sliderId, worldPos?)`
    drops a free joint on the rail (snapped to the click point, or rail midpoint), attaches
    it as a slider rider, and stores the actuator constraint that will drive it during
    animation. `addMotor(bodyId, pivotJointId, crankJointId)` validates that both joints
    belong to the body (and aren't the same joint) before storing the motor constraint.
    Cascade removal: dropping a slider removes any actuators on it (the rider survives as a
    free joint); removing a body's joints prunes any motor that referenced them.
  - **Measurements** (`measurements: Measurement[]` on the scene): a `Measurement` =
    `{ id, mode: "draw"|"sim", refA, refB, labelOffset, axis }`. A `MeasureRef` is a **point**
    (`joint` id / body `vertex` index / `bodyPoint` = a local offset fixed in a body's frame)
    or a **line** (slider `rail` id / body control-polygon `edge` index). `resolveMeasureRef`
    re-resolves a ref to world geometry every frame; `measureInfo(m)` computes the display —
    kind (`distance`/`angle`), value (world units / degrees), label position, arrowed
    dimension segment or arc, and dashed extension segments (`MeasureInfo`). Point+point uses
    the stored `axis` (`h`/`v`/`direct`, derived from label placement by
    `measureAxisForPlacement`, re-derived on label move); point+line is perpendicular distance
    to the infinite line; line+line is distance when within `MEASURE_PARALLEL_TOL` (0.5°) of
    parallel, else the angle of the sector the label sits in (of the four the two infinite
    lines make). `addMeasurement` / `removeMeasurement` / `setMeasurementLabel` /
    `measurementLabelPos` / `measurePreview` (placement preview without creating).
    `pruneMeasurements` drops measurements whose refs no longer resolve (called from the
    constraint-prune paths, so element deletion cascades); `shiftMeasureIndices` keeps
    vertex/edge refs pointing at the same geometry across `insertBodyVertex` /
    `removeBodyVertex` (a ref *on* a removed vertex/edge drops its measurement). Known
    limitations: `mirrorBody` doesn't remap vertex/edge/bodyPoint refs; copy/paste doesn't
    carry measurements.
  - **Sketch constraints** (`sketch: SketchConstraint[]` on the scene): a
    `SketchConstraint` = `{ kind, id, refA, refB }` where kind ∈ coincident / horizontal /
    vertical / parallel / perpendicular / equal and the refs are `MeasureRef`s (`refB` is
    null for H/V on a single line ref). `addSketchConstraint` validates ref kinds per
    constraint kind (points = joint/vertex — `bodyPoint` is measurement-only; lines =
    rail/edge), rejects duplicates of the same element and unresolvable refs.
    `removeSketchConstraint` / `getSketchConstraint`; `pruneSketch` runs alongside
    `pruneMeasurements` (element deletion cascades) and `shiftMeasureIndices` also remaps
    sketch vertex/edge refs. `sameMeasureRef` (exported) compares refs by element.
    **Driving dimensions**: draw-mode `Measurement`s gain optional `driving` + `target`;
    `setMeasurementDriving` / `clearMeasurementDriving`; `measureInfo` passes `driving`
    through to `MeasureInfo` for display. **`scaleBody(id, factor)`** scales a body
    uniformly about its centroid — control polygon, corner radius, attached joints, their
    ground anchors, and `bodyPoint` measurement refs all together.
  - `serialize()` / `load(SceneData)` for save / load / autosave (versioned plain-data
    snapshot, `FORMAT_VERSION = 8`; `load` deep-copies, recomputes `nextId`, drops legacy
    origin+dir sliders, migrates older single-`slider` → `riders`, and back-fills
    `controlLocal`/`radius`/`round` for pre-v5 bodies; pre-v6 files simply have no
    actuator/motor constraints, pre-v7 files no measurements, pre-v8 files no sketch
    constraints or driving flags — all load fine as-is).
- **solver.ts** — `solve(scene, driver, iterations, relax, anchors?, stats?): ConstraintBreak[]` (each
  `ConstraintBreak` carries `a`/`b`/`error` plus a `joints: number[]` list naming the joints
  involved — pin endpoints, the grounded joint, an unreachable slider rider, or the anchor's joint
  — so the UI can paint them red). Operates
  on a **host abstraction** (`hostFor`): each constraint participant is a body, a free joint
  (translate-only point), or a fixed world point — which unifies pin/ground/slider/driver. A
  separate **`pinHostFor`** makes *any* grounded joint (free or on a body) a fixed point when
  something *pins* to it, so a pin can't drag the body a grounded joint sits on (the joint's own
  ground constraint still uses the body host, to lock multi-ground bodies' rotation). The
  optional **`anchors`** map (joint id → world target) is threaded through every solver phase
  (sweep / residual / settle / phase A/B/C / projectGrounds / break reporting) and treated
  **exactly like ground constraints** — sacred, never disabled, projected at the end of each
  sweep — which is how linear-actuator riders and motor pivot/crank pairs are driven during
  animation without growing a new solver concept. Returns the list of unsatisfiable
  **`ConstraintBreak`s** (empty when solvable). The former module constants live in a mutable
  exported **`solverConfig`** (`structuralTol`, `breakTol`, `maxCleanupSweeps`) so the UI can
  tune them at runtime; an optional **`SolveStats`** out-param reports Phase-A sweeps run,
  cleanup sweeps run, and the final residual. Phase A **early-exits** once the structural
  residual is under tolerance — but **only when no driver is present**: the driver is
  step-limited (`DRIVER_MAX_STEP` per sweep), so a weakly-constrained grab keeps residual ≈ 0
  and an unconditional early-exit would starve the drag to one pull per solve. See "Solver
  notes" below.
- **sketch.ts** — the **sketch solver** (draw-mode only; the sim solver is untouched).
  Gauss-Seidel projection like solver.ts, but the variables are *shape*: the world
  positions of body **control vertices** (`v:body:index`) and **joints** (`j:id`) — a
  joint coincident with one of its body's control vertices maps onto the vertex variable
  (the node↔joint link), so constraints on it reshape the body. Each constraint /
  driving dimension becomes a projection item (coincident → midpoint; H/V → average the
  coordinate; parallel/perpendicular → rotate both lines half-way about their midpoints;
  equal → scale both lines to the mean length; distance dims → symmetric point/line
  moves along the axis/normal — a driving line–line distance also keeps the pair
  parallel). Sweeps run until every residual < `sketchConfig.tol` (mirrors
  `solverConfig`), then the solved positions are applied through `moveBodyVertex` /
  `moveJoint` (bodies rebuild, containment clamps apply) and **verified** against the
  actual scene; a failed verify reverts via a serialize snapshot. **Reject semantics
  throughout**: an unconverged solve never touches the scene and returns `SketchBreak[]`
  (ids of the unsatisfiable items). Public API: `solveSketch(scene)` (re-solve + apply),
  `applyDrivingDimension(scene, id, target)` (validates: draw-mode distance dims only,
  target > 0; picks **uniform scale** — both refs owned by one body, no other driving dim
  touches it, every sketch constraint touching it is fully internal — else the node
  solve; commits the driving flag only on success), `tryAddConstraint` (add + solve,
  remove again on conflict), `autoConstrainBody` (H/V inference within `AUTO_HV_TOL` =
  5°, each constraint solved in as it's added).
- **analyzer.ts** — read-only **topology diagnostic** (Stage 1 of the propagation-solver
  exploration; not yet imported by the app — call `formatReport(analyzeScene(scene), scene)`
  from a console/debug hook). Builds a constraint graph (bodies + free joints as nodes; pins +
  slider-rider couplings as edges), finds kinematic islands (union-find), and per island
  reports: Grübler-Kutzbach DOF (pins/grounds remove 2 DOF, a slider rider — point-on-line —
  removes 1), cyclomatic loop count (grounds modeled as edges to a virtual world node so
  loops-through-ground are counted), BFS propagation order from anchors, back-edges, and a
  Tarjan **bridge / biconnected-component decomposition** classifying bodies into loop cores
  (must be solved together) vs propagatable tree branches, with articulation bodies joining
  blocks. Labels are `#id`-based (bodies have no user-facing name).
- **view.ts** — camera transform `screen = world * scale + (tx, ty)`; `screenToWorld`,
  `worldToScreen`, cursor-anchored `zoomAt` (scale clamped to MIN_SCALE..MAX_SCALE = 0.05..20).
- **renderer.ts** — joints involved in any `ConstraintBreak.joints` are painted red (fill +
  stroke + slight size bump) so the stuck points stand out alongside the existing red dotted
  break lines. Draws under the camera transform in world space: world-locked grid
  (spacing = `gridStep`, drawn only when `gridVisible`),
  bodies (selected/hovered highlighted), slider rails as bounded segments with end-caps,
  ground symbols, joints (color-coded: blue = pinned, yellow = grounded, green = slider rider;
  rail joints get a green ring; **a loose free joint gets a muted dashed ring — but a free joint
  that rides a slider or defines a rail drops the dashed ring and renders as anchored**), corner-handle squares
  for the selected body, plus draft overlays (freehand polygon, build-from-joints outline +
  expansion preview, slider rail preview) and a rotate-pivot crosshair. **Powered constraints**:
  every motor draws a dashed yellow arm from its pivot to its crank, with a curved-arrow rotation
  badge centred on the pivot; every linear-actuator rider gets a green dashed outer ring on top
  of its normal slider-rider dot to badge it as self-driving. **Impossible-assembly breaks** are
  drawn as **red dotted lines** between the points that can't meet (`input.breaks`, sim mode only).
  **Draw-mode connectors**: a constraint whose endpoints sit apart is drawn dotted so the link
  still reads as connected — **blue** between two pinned joints (skipped when their dots overlap),
  and **green** from each slider rider to the **rail midpoint** (skipped when the rider is already
  on the rail, via `distToSegment`). Cosmetic sizes are divided by the zoom. Structural colours
  come from a **`Theme`** palette (`DARK_THEME`/`LIGHT_THEME`, in `input.theme`): only `ink`
  (highlights/draft/handle fill), `surface` (joint rings, pin centres, handle outline), `grid`, and
  `jointFill` flip between light/dark — the semantic accents (pin blue, slider/rail green,
  ground/rotate yellow, error red) and per-body colours stay fixed.
  **Measurements** are drawn last (annotations on top of everything): dashed extension lines,
  an arrowed dimension line or an angle arc (cyan accent `#46c2cb`; selected → theme ink),
  and the value as a constant-size pill + text rendered in **screen space** (via a transform
  reset per label, so text stays legible at any zoom; distances 1-decimal, angles with `°`).
  The measure-tool overlay highlights picked references (ring around a point, soft thick
  stroke over a line; hover ref dashed/lighter) and draws the placement preview as a dashed
  measurement. Inputs: `measurements: MeasureInfo[]` (already resolved by main),
  `measureDraft: { refs, hover, preview }`, and `selection` now includes kind `"measure"`.
  **Sketch constraints**: violet (`#b48cff`) constant-size badge pills (screen-space, like
  measurement labels) with a symbol per kind (◎ H V ∥ ⊥ =), drawn from
  `sketchGlyphs: SketchGlyphView[]` (positions computed by main so hit-testing matches);
  badges render **faded** (alpha 0.2) unless `faded: false` — main sets it when the cursor
  is over one of the constraint's elements or a badge — with selection / reject-flash
  always at full strength;
  `sketchDraft` highlights constraint-tool picks in violet (reusing
  `drawMeasureRefHighlight`, which took a colour param). In draw mode a **driven**
  dimension's value renders **in parentheses** and a **driving** one plain with a bolder
  pill border (`measureText(info, paren)`); `flash: Set<number>` paints the constraints /
  dimensions a rejected sketch edit named in the error red for a moment. Selection kind
  `"sketch"` highlights a badge in ink.
- **main.ts** — canvas/DPI setup, toolbar wiring (the toolbar is **icon buttons** — SVG glyphs with
  tooltips; wiring is by id/`data-*`/class, never button text), mode/tool state (tools are one-shot +
  have keyboard shortcuts), select-mode selection / move / delete / vertex-edit, the camera,
  pointer + key handling, persistence (save/load/autosave) plus a **snapshot undo/redo history**
  (`pushHistory`/`undo`/`redo`; `markDirty` records a step + autosaves), and the
  requestAnimationFrame render/solve loop. `timedSolve` captures the solver's `ConstraintBreak`s
  into `solveBreaks`; `updateSimError` shows/hides the red **"Assembly impossible"** banner
  (`#sim-error`), and the breaks are passed to the renderer in sim mode (cleared on leaving sim).
  The **Joint** tool auto-attaches a placed node to a slider when it lands on a rail/rail-node;
  the **body-from-joints** draft turns a bare slider-rail click into a grid-snapped rider joint
  (tracked in `jointDraftCreated`, removed if the draft is aborted). Also a `timedSolve` debug log.
  **Snap containment fallback**: both the Joint tool and body-from-joints joint minting hit-test
  against the raw click point but place at the snapped point — if snapping would land outside a
  body being attached to, they fall back to the exact click point (inside by hit-test), so a
  joint is never created outside its body. The `#sim-error` banner lives inside a
  **`#canvas-wrap`** container (index.html / style.css) wrapping the canvas, so it overlays the
  canvas top-center *below* the toolbar rather than on top of it.
  - **Actuator / motor tools** (`L` / `M`, draw mode, one-shot): **Linear actuator** is a single click
    on a slider rail → calls `addLinearActuator` (snapped to the click point). **Motor** is two clicks
    — first joint becomes the pivot (tracked in `motorPivotDraft`, highlighted via `activeJoints`),
    second joint on the **same body** becomes the crank pin (mismatched second click restarts the
    draft at the new joint). Both tools select the resulting element so the inline properties panel
    appears right away.
  - **Measure tool** (`D`, one-shot, the only tool that also works in **sim mode** — its
    toolbar group never hides): two reference picks then a label-placement click →
    `handleMeasureClick` / `measurePicks`. `measureRefAt(p)` picks by priority: joint →
    body control vertex → slider rail → body control-polygon edge → point inside a body
    (grid-snapped when snapping keeps it inside; empty space picks nothing). The draft view
    passes picked/hover refs + a live `measurePreview` to the renderer. Labels:
    `measurementLabelAt` hit-tests value pills (16 px radius, topmost overlay — checked
    *before* joints/bodies in select mode and in sim mousedown); click selects
    (`selection.kind === "measure"`, the one selection kind that stays live in sim), drag
    repositions via `leftDrag.kind === "measureLabel"` (unsnapped;
    `setMeasurementLabel` re-derives h/v/direct), Delete removes (works in both modes).
    Esc now calls `disarmTool()` in both modes. Adding/moving/deleting measurements marks
    dirty (persisted + undoable; `canonicalData` keeps sim poses out as always).
  - **Sketch-constraint tools** (`O`/`H`/`V`/`P`/`T`/`E`, draw mode, one-shot; tool name =
    constraint kind): pick the reference(s) — points via `constraintPointRefAt`
    (joint → body corner), lines via `constraintLineRefAt` (rail → body edge); H/V on a
    line commits on the first click, everything else on the second. `commitConstraint` →
    `tryAddConstraint`: the geometry solves to satisfy the new constraint immediately, or
    the add is rejected and the conflicting items **flash red** (`sketchFlash`, 1.2 s).
    Badge positions come from `sketchGlyphsView()` (one badge per referenced element,
    screen-offset beside a point / off a line midpoint, stacking sideways when an element
    has several; coincident gets a single badge), cached for `sketchGlyphAt` hit-testing;
    click selects (`selection.kind === "sketch"`), Delete removes. Badges are **faded by
    default** and shown full-strength only while the cursor hovers the constrained element
    (`refHovered`: the joint / corner / rail within pick range — or anywhere on the owning
    body for vertex/edge refs) or a badge itself.
  - **Visibility toggles** (session-only, like the grid): `#sketch-vis-btn` (sketch group)
    shows/hides all constraint badges; `#measure-vis-btn` (measure group, works in sim
    too) shows/hides all measurements. Hiding is purely visual — constraints still solve —
    but the hidden layer isn't hit-testable (no clicks, drags, or label edits), and a
    matching selection is cleared. Placing a new constraint/measurement or a reject-flash
    that names an item in a hidden layer **auto-reveals** that layer.
  - **Inline dimension editing**: double-click a draw-mode dimension label →
    `openDimEditor` positions the floating `#dim-edit` input over it (screen-space, inside
    `#canvas-wrap`). Enter (or blur) commits: a number → `applyDrivingDimension` (rejected
    edits flash red, nothing moves); an **empty value** → back to a driven reference
    (`clearMeasurementDriving`). Esc cancels. A global keydown guard ignores canvas
    shortcuts while any input/select has focus (so Delete/tool letters don't fire while
    typing — this also fixed a latent bug with the actuator speed fields).
  - **Sketch-aware dragging** (`solveSketchLive`): draw-mode select drags (vertex / body /
    joint) and rotate-tool drags re-solve the sketch after every move when any constraint
    or driving dimension exists — constraints hold while the dragged geometry follows,
    CAD-style. A solve that can't converge mid-drag is skipped (the next one re-tightens).
  - **Auto-constraints while drawing** (freehand body tool): a click that lands on an
    existing joint / body corner places the vertex **exactly there** and records the pick
    (`draftBodySnaps`); `finishBody` turns the picks into **coincident** constraints and
    runs `autoConstrainBody` (near-H/V edges → H/V), each solved in and skipped if
    unsatisfiable. Joint-built bodies are untouched (their shape comes from joints).
  - **Animation** (`#run-btn`, sim-mode only; Space toggles): `setAnimating(on)` flips a `running`
    flag and, on **play**, calls `fitPhases()` so each actuator/motor's `phaseAccum` matches the
    current pose — pressing play resumes from whatever the user (or the previous run) left in sim.
    Each frame, if `animating`, the loop advances every phase by `speed*dt` (clamped per frame to
    cap big jumps after the tab is backgrounded), `computeAnchors()` translates phases into world
    targets per actuator/motor (one per actuator rider, two per motor — pivot + crank), and the
    main solve passes those targets via the solver's `anchors` parameter. Animation defaults to
    **off** on entering sim so dragging-to-drive keeps working until the user starts it.
  - **Auto-pause on impossible** (`#autopause-btn`, sim-mode only): a toggle that stops the
    animation when the assembly can't be solved. Session-only state (`pauseOnImpossible`, not
    persisted). The check is debounced: `setAnimating(false)` fires only after
    `IMPOSSIBLE_PAUSE_FRAMES` (3) **consecutive** animation frames with non-empty `solveBreaks`,
    to filter out single-frame solver-convergence misses on complex closed loops. Visual indicators
    (red banner, red break lines, red joints) are *not* debounced — they reflect whatever the
    current solve found. Manual drag is unaffected (no pause concept). **Note**: the debounce
    helps but still false-positives on some borderline mechanisms; the next intended fix is on the
    solver side (more iterations / better convergence handling for animated anchors).
  - **Solver tuning controls** (sim-mode toolbar, next to Auto-pause): a Phase-A iteration
    slider (10–300; sets `animIterations`, the per-frame animation solve budget — was a
    hardcoded 60), a cleanup-sweep cap slider, and structural / break tolerance number inputs
    writing straight into `solverConfig`. The inputs are **seeded from the runtime values at
    startup** (the HTML carries no defaults, so they can't drift). Session-only, not persisted.
    `timedSolve("anim", …)` accumulates rolling stats across an animation run — solve time
    min/max/avg, Phase-A and cleanup sweep counts, final residual, and the % of frames that
    reported breaks — logged each frame, reset when the animation stops.
  - **Inline properties panel** (`#actuator-props` / `#motor-props`): mirrors the body-colour
    pattern — `syncPropsPanel()` runs each frame, hides both panels when neither element is
    selected, otherwise populates the speed field (and, for an actuator, the `/\` ↔ `~` profile
    toggle). Change-detected so editing the speed input mid-drag doesn't get clobbered.
    `selectedLinearActuator()` and `selectedMotor()` find the constraint the current `selection`
    identifies (rider / slider for actuators; body / pivot / crank for motors).
  - **Edit utilities**: `copySelection`/`pasteAt` (clipboard is a `BodyClip` held in `main`;
    paste lands at the cursor, grid-snapped), `mirrorSelection("h"|"v")`, and a **rotate** tool.
    Rotate is a persistent mode (not one-shot): `startRotate` picks the pivot (a control node of
    the selected body if grabbed, else the centroid of the body under the cursor) and a
    `rotateDrag` tracks the pointer's swing about it — accumulated/unwrapped so it survives ±π,
    with the resulting absolute body angle snapped to 45° (`snapAngle`/`ROTATE_SNAP_TOL`) and only
    the incremental delta applied per move via `scene.rotateBody`. Copy/paste are also on
    `Ctrl/Cmd+C`/`V`; rotate on `R`.
  - **Theme** (`#theme-btn`): a dark/light toggle that sets `data-theme` on `<html>` (CSS vars drive
    the chrome) and passes the matching `DARK_THEME`/`LIGHT_THEME` palette to the renderer; the
    choice persists in `localStorage` (`disjointed:theme`, separate from scene autosave).
  - **Body colour** (`#body-color`): a colour input that does double duty — with a body selected it
    recolours that body (live, `markDirty`); with nothing selected it sets `defaultBodyColor`, the
    colour given to newly drawn / built bodies (paste keeps the source colour instead). The swatch
    is synced to the selection each frame by `syncColorPicker` (change-detected so it never clobbers
    the picker mid-drag). The `#color-group` hides in sim mode like the tool/edit groups.
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
`B`/`J`/`C`/`G`/`S`/`L`/`M`) lets you place one element, then it returns to **Select** mode. `Esc`
aborts the current placement.
- **Body** (`B`) — first click decides the mode. On **empty space**: freehand polygon (click
  vertices; click the first vertex / double-click / Enter to close; Esc cancels). On an
  **existing joint**: build a body *from joints* — click joints to outline (loose free joints
  absorbed; grounded free joints, joints on other bodies, and a **rider on another body** get a
  pinned twin — pinning to a cross-body rider joins the two bodies so they ride the slider together;
  a **slider rail node** gets a twin attached to the slider as its own rider; a bare **slider-rail**
  click drops a rider point there; **a click on an existing body** mints a fresh joint on that body
  at the click point so the new body gets a pinned twin there — joining the two bodies; **a click
  on empty space** mints a free joint at the click point that gets absorbed into the new body),
  click an already-added joint to finish, then move the cursor out to size the outward margin
  (live preview) and click to finalize. Joints minted mid-draft are tracked so an aborted draft
  removes them.
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
- **Linear actuator** (`L`) — click a *slider rail* to drop a **self-driving rider** on it (a free
  joint attached as a rider, plus a `linearActuator` constraint that drives it during animation).
  Off-animation the rider is a normal slider rider (draggable / pinnable). Default speed 0.5 Hz,
  default profile `triangle`.
- **Motor** (`M`) — click a joint to set the **pivot** (must be on a body), then another joint on
  the *same body* for the **crank pin** (a cross-body second click restarts the draft at that
  joint). Creates a `motor` constraint that spins the body during animation. Default speed 0.25 Hz.
- **Measure** (`D`, also available in sim mode — sim keeps its own set) — click two references
  (joint, body corner, slider rail, body edge, or a point on a body), then click where the value
  should sit. Point+point: the label spot picks horizontal / vertical / direct. Point+line:
  perpendicular distance to the infinite line. Line+line: distance while parallel, angle
  otherwise (dynamic; the label's sector picks θ vs 180−θ). Values update live in sim.
- **Sketch constraints** (draw mode only): **Coincident** (`O`) — two points. **Horizontal**
  (`H`) / **Vertical** (`V`) — one edge/rail, or two points. **Parallel** (`P`) /
  **Perpendicular** (`T`) / **Equal length** (`E`) — two edges/rails. The geometry solves to
  satisfy the constraint the moment it's placed; an unsatisfiable one is rejected (conflicting
  items flash red). Badges are click-to-select, Delete to remove.
- **Driving dimensions** (draw mode): double-click a dimension's value → type a number →
  Enter. The first driving dimension on an otherwise-unconstrained body scales it uniformly;
  further ones move only the involved nodes while every constraint and driving dimension
  holds. Driven values show in parentheses; clear the field to make a driving dimension a
  reference again. Unsatisfiable targets are rejected (revert + red flash).

Select mode (default, no tool armed):
- **Sketch-aware dragging**: with any sketch constraints / driving dimensions present, every
  draw-mode drag (node, joint, body) and rotate re-solves the sketch live — the dragged
  geometry follows the cursor as far as the constraints allow and everything constrained to
  it comes along.
- Click a body, joint, or slider rail to **select** it (highlighted); **left-drag** moves the
  selected body or joint. An attached joint **can't leave its body**: dragging it past the edge
  clamps it to the outline (it slides along the edge). A joint sitting exactly on a control
  node (as in a joint-built body) is **stuck to that node** — dragging either one moves both,
  reshaping the body. A selected body shows **corner handles** — drag one to reshape it
  (`moveBodyVertex`, carrying any joint stuck to that node); **`[` / `]`** decrease / increase
  its corner radius (round/un-round).
  **Double-click** an edge of the selected body to add a control node there (`insertBodyVertex`,
  grid-snapped), or a node to remove it (`removeBodyVertex`, kept ≥ 3). Clicking on/near the
  selected body's control polygon keeps it selected (so an edge double-click isn't lost).
- `Delete` removes the selection: a body takes its joints/constraints with it; a slider keeps
  its joints; a joint detaches from any rail and any constraints referencing it go; a
  measurement just disappears (measurements are also cascade-removed with their elements).
- A **measurement's value label** is the topmost pick: click to select it, drag to reposition
  (a point–point measurement re-derives h/v/direct from the new spot), Delete to remove —
  this works in **sim mode** too, without disturbing the mechanism underneath.
- **Edit utilities** (act on a selected body): **Copy/Paste** (`Ctrl/Cmd+C`/`V`, **keyboard-only**
  now — no toolbar buttons) duplicates a body with its joints + own constraints — including its
  fully-internal sketch constraints and driving dimensions — **keeping its
  colour**; the copy lands at the cursor (grid-snapped) and is selected. **Mirror H/V** (toolbar,
  grouped with **Rotate**) reflects the body + joints in place about its centroid.

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
- **Run animation** (`▶`/`⏸` button in the sim-mode toolbar; **Space** toggles): drives every
  linear actuator and motor in the scene at their configured speed. Defaults to **off** on entering
  sim, so dragging-to-drive works first. Pressing play **phase-fits** every actuator/motor so the
  motion resumes smoothly from the current pose (the user can pause, drag a part somewhere new,
  and pressing play continues from there).
- **Impossible assemblies** are flagged, not faked: grounds never move, the solvable parts still
  solve, and every unsatisfiable pin/slider draws a **red dotted line** between the two points
  that can't meet (pulled as close as the assembly allows), with a red **"Assembly impossible"**
  banner. The joints involved in each break are also drawn **red** (red fill + red ring + slight
  size bump). A connected impossible piece does not disturb the parts that can be solved. The
  sim-mode toolbar has an **Auto-pause** toggle (warning-triangle icon) that halts the animation
  once the assembly stays impossible for a few frames in a row — useful for stopping motors /
  actuators before they push past a physically unreachable configuration.

Navigation (both modes):
- **Mouse wheel** zooms toward the cursor (0.05×–20×).
- **Right-drag always pans** the view (anywhere). Moving elements is left-drag in select mode
  (above); there is no right-drag-to-move.
- **Fit to screen** (`F`, or the toolbar button next to Save): frames the whole mechanism
  (body outlines, joints, ground anchors) centered with a 60 px margin (`fitView` in main.ts);
  an empty scene recenters the world origin at 1×.
- **Tab** toggles draw ↔ simulate mode (preventDefault'd away from the browser's focus cycle;
  like every shortcut it's inert while an input field has focus).

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
  the good pin and flags only the third piece, with both grounds unmoved. Also asserts each
  break's `joints` list names both stuck endpoints so the UI can paint them red.
- **shape-edit.ts** — `filletPolygon` (convex + concave validity, radius 0 passthrough; a
  **reflex corner rounds into the notch, not the material**; a **narrow-neck shape stays
  simple — no self-intersection — across radii up to 200**) and body editing (`setBodyRadius` /
  `moveBodyVertex` keep attached joints anchored; `insertBodyVertex` / `removeBodyVertex` change
  the control-vertex count, keep joints anchored, and enforce the 3-vertex minimum). Also
  **joint containment** (`moveJoint` inside the body lands where asked; a move past the edge /
  diagonal escape clamps to the outline; the ground anchor follows the clamped position; free
  joints stay unclamped) and the **node ↔ joint link** (moving a joint-built body's node carries
  its joint while the others stay anchored; moving the joint carries the node; a grounded linked
  joint keeps its anchor in step; a non-node joint moves without reshaping the body).
- **edit-utils.ts** — `rotateBody` (90° about the centroid carries the joint + ground anchor; a
  pivot node stays fixed), `mirrorBody` (joint reflected, centroid + area preserved), and
  copy/paste (`extractBody`/`insertBody`: independent offset duplicate, joints/grounds/sliders
  duplicated with fresh ids, cross-body pins dropped, **source colour preserved**; internal
  **sketch constraints** — H/V edges, joint↔vertex coincident, H on an internal rail — captured
  and recreated on the new elements with resolvable refs, cross-body constraints excluded;
  the internal **driving dimension** copied with its target + translated label while a driven
  reference dimension is left behind and the original stays untouched).
- **actuators.ts** — the two new powered constraints end-to-end: `addLinearActuator` places its
  rider on the rail and registers it with the slider; `addMotor` rejects pivot==crank, free-joint
  pivots, and cross-body cranks. The solver's `anchors` parameter drives an actuator rider to
  arbitrary on-rail targets (rider stays on rail, lands exactly on each anchor including the
  endpoints), and pivot+crank anchors on one body act as a motor (pivot fixed, crank orbits at
  constant radius, downstream pin propagates the rotation). Serialize/load round-trips both
  constraint kinds intact; removing a slider drops its actuator (rider survives as a free joint);
  removing a motor's body drops the motor (via joint-pruning).
- **sketch.ts (scripts)** — sketch constraints + driving dimensions end-to-end:
  `addSketchConstraint` validation (kind/ref mismatches, bodyPoint, same-element,
  unresolvable); H/V/coincident solving on free joints, on a body edge (reshapes the body,
  other joints anchored), and through the node↔joint link (a constraint on a linked joint
  drives the vertex); parallel / perpendicular / equal (lengths + midpoints preserved);
  driving dimensions on the node path (direct + h-axis; midpoint preserved, uninvolved
  nodes untouched); **scale-on-first-dimension** (centroid fixed, vertices/joints/ground
  anchors/radius/bodyPoint refs all scaled; a second dimension takes the node path and the
  first still holds; an external coincident disables the scale path); the parametric
  rectangle (H/V constraints + width scale + height node-solve); point-line and line-line
  driving; rejects (conflicting dims leave the scene byte-identical, angle dims and
  sim-mode dims can't drive, non-positive targets); `tryAddConstraint` rollback;
  `autoConstrainBody` H/V inference (diagonals left alone); cascade removal + vertex/edge
  index remapping; serialize/load v8 round-trip + pre-v8 files.
- **measurements.ts** — axis selection from label placement (h / v / direct zones);
  point-point values + dimension-line geometry + preview parity; label move re-deriving the
  axis; values and label positions tracking moving geometry; point-line using the infinite
  line (foot beyond the rail end + extension line); line-line parallel distance, the dynamic
  flip to angle when a rail tilts, and the label sector picking the obtuse angle; vertex/edge
  index remapping across `insertBodyVertex`/`removeBodyVertex`; a bodyPoint ref riding its
  body's frame; cascade removal (joint / slider / body); serialize/load round-trip (modes,
  values, deep copy, `nextId`) and pre-v7 files loading with no measurements.

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
- **Slider hit-test matched the infinite rail line, not the segment.** `sliderAt` used `distToLine`,
  so a click far past the rail's endpoints (but co-linear) still hit the slider — and the Joint tool
  auto-attached the placed joint as a rider even though it sat well outside the visible rail. Fixed:
  `sliderAt` now uses `distToSegment`, so the pick matches the rendered segment. Tightens the
  hit-test for the Joint placement, Connect attach, Linear actuator drop, Select pick, and the
  body-from-joints rail click — all of which use this helper.
- **A joint could be placed or dragged outside its body.** Select-mode drags had no containment
  check, and the Joint tool hit-tests the raw click but places at the *snapped* point, which
  near an edge could fall outside the clicked body. Fixed: `moveJoint` clamps an attached
  joint to the nearest point on its body's outline (it slides along the edge), and placement
  falls back to the unsnapped click point when snapping would exit the body.
- **The "Assembly impossible" banner covered the toolbar.** It was absolutely positioned at the
  top of `#app` (toolbar included) with a higher z-index than the toolbar. Fixed: the canvas +
  banner now live in a `#canvas-wrap` positioning context, so the banner overlays the canvas
  just below the toolbar at any toolbar height.
- **Phase-A early-exit starved the mouse driver.** The first version of the early-exit broke out
  of Phase A whenever the structural residual was under tolerance — but the driver is step-limited
  to `DRIVER_MAX_STEP` per sweep, so dragging something weakly constrained (a lone free joint, an
  unconstrained body) kept residual ≈ 0, exited after one sweep, and the dragged point crawled at
  8 units/frame instead of tracking the cursor. Fixed: the early-exit only fires when **no driver
  is present** (the pure-animation case it was built for); with a driver the loop runs the full
  budget, as before.

## Backlog / next steps (not yet built)
- **Sketch-constraint follow-ups**: driving *angle* dimensions (v1 is distances only);
  an auto-constraint on/off toggle in the toolbar; auto-coincident while *dragging* (today
  it's inferred only while drawing); constraint badges could use hover feedback; sketch
  constraints aren't remapped by `mirrorBody` (same as measurements) — copy/paste **does**
  carry the fully-internal ones (+ driving dimensions) since format-v8 follow-up work.
- Measurement follow-ups: `mirrorBody` doesn't remap vertex/edge/bodyPoint measurement refs;
  copy/paste doesn't carry *driven* (reference) measurements (consistent with
  actuators/motors; driving dimensions are carried, as constraints).
- More joint types as needed.
- Joint containment covers placement + drags only (by choice): **reshaping** a body (corner
  handles, radius shrink, vertex removal) can still strand an already-placed joint outside the
  new outline. If that bites, clamp stranded joints back in `rebuildBody`.
- **Actuator / motor follow-ups**: editing speed/profile while in sim (selection clears on mode
  change today, so the inline panel only appears in draw); copy/paste carrying actuators + motors
  in the `BodyClip` (today the body + its joints survive but the powered constraints don't).
- **Auto-pause false positives — investigation in progress.** The "auto-pause on impossible"
  feature works for genuinely unreachable configurations, and a 3-frame debounce filters obvious
  solver chatter — but the animation still pauses on some borderline complex closed-loop scenes
  that *are* solvable. The solver-side instrumentation for this is now in place: live tuning
  controls (iterations / cleanup cap / tolerances via `solverConfig`), `SolveStats`, and rolling
  per-run animation stats in the console — use them to find which knob actually clears a given
  false positive. **Stage 1 of a propagation-solver exploration is also done** (`analyzer.ts`:
  island / DOF / loop-core / BCC decomposition, not yet wired to UI); next stages would be a
  debug hook to run it on the live scene, then prototyping closed-form propagation for tree
  branches with the iterative solver kept for loop cores. Investigating a reported flicker of
  "Assembly impossible" while *dragging* an actuator's rider after stopping animation in a
  closed loop falls in the same bucket (fix candidate A is "project the driver target onto the
  rail before solving" for slider riders).
- Optional: File System Access API for true "re-open the last file by path" (Chromium only).
  Current persistence is download/upload + localStorage autosave (restores the last session).

## Working conventions (from CLAUDE.md)
- Answer bug/feature questions first; confirm before changing code.
- Remind to push to GitHub before big structural changes.
- On finishing / commit: update this file and README.md, then summarize and suggest a
  commit message (the user runs the commit).
