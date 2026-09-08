# Disjointed — Project State

## Goal
A simple webapp to create and simulate 2D planar mechanisms (moving joints / linkages).
Two modes:
- **Drawing mode** — draw bodies, place joints (attached or free), and couple them with
  constraints (rotation/pin, ground, rail + riders/sliders). Joints on the same body are rigid relative to
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
**rails** with end-stops (called "sliders" internally / pre-v17) — a rail is either two
joints on one body (a moving rail) or two free joints (a world-fixed track, auto-grounded),
with joints auto-attached as riders when placed on a rail (Joint tool or body-from-joints),
and riders optionally **orientation-locked** ("sliders", v17 — see the rails & sliders
paragraph below);
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
**View navigation**: zoom range 0.05×–200×; a **fit-to-screen** button + `F` shortcut frames the
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
**coincident** (point–point, or point-on-line — see below), **horizontal / vertical** (a line, or a point pair),
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
**Multi-selection & permanent groups** (draw mode, serialization v9): **Ctrl/Cmd+click**
toggles bodies (and free joints) in a multi-selection; **box select** (left-drag on empty
space) selects bodies fully inside the marquee plus free joints inside it (Ctrl+drag adds).
A multi-selection **moves together** (drag any member), deletes together, copies together.
**Ctrl/Cmd+G** is a group/ungroup **toggle**: 2+ selected bodies become a **permanent
group** (merging any groups touched) — unless the selection already is exactly one group,
which dissolves (a single selected grouped body also ungroups). Plain **G** is the Ground
tool only. Groups are **selection-atomic** (clicking any member selects — and drags /
rotates / mirrors / copies — the whole group) and behave as a **single rigid body in
simulation**: the solver treats a group as one composite (combined mass/centroid/inertia;
every impulse moves the whole group; intra-group pins/riders are inert). A group shows a
faint dashed convex hull **only while selected**. Copy/paste is generalized to whole
selections (`SelectionClip`): pins **between** copied bodies, group membership, free
joints, grounds, internal sliders, and internal sketch constraints / driving dimensions
all travel. Mirror reflects a multi-selection about its combined bbox centre; Rotate turns
it rigidly about that centre — and `mirrorBody` now **remaps vertex/edge/bodyPoint refs**
(the control-polygon reversal used to leave constraints pointing at the wrong corners).
**Grounded bodies & groups** (serialization v10): the Ground tool now also toggles on a
**body** (no joint under the cursor) — and, through a grouped body, on its **whole group**.
A grounded body is **completely fixed in simulation** (position *and* rotation, unlike a
joint ground which allows pivoting); draw mode still edits/moves it freely. In the solver
grounded bodies are immovable fixed hosts — sacred like ground anchors: pins/sliders/drags
can't budge them, a rail on one is a fixed track, and an unreachable pin onto one is
reported as a break. Rendered as the standard ground symbol at the body's centroid.
**Rigid (Shift) drag** (draw mode): holding **Shift** when starting a select-mode drag
moves the grabbed body / joint's body / free joint / multi-selection (groups always whole)
**like in simulation**: the sim solver drives the grab point toward the cursor each frame,
grounds hold exactly (a body pivots about its grounded joint; ground anchors never move),
pins/sliders to the rest of the scene constrain the motion — and the **rest of the scene
is frozen** in place. An open (dotted) pin from the dragged selection to the frozen world
**snaps closed** as the drag starts pulling (assemble-on-drag). The released poses become
the new drawn layout (one undo step). Implemented as an optional **`SolveFreeze`** scope
on `solve` — frozen bodies/free joints are immovable, and constraints entirely inside the
frozen world are **out of scope** (not solved, not reported as breaks). Rigid drags are
sim-like, so **sketch constraints are not applied** during them (a rigid rotation can
leave an H/V constraint unsatisfied until the next sketch solve). Plain drag, vertex
reshaping and label drags are unchanged (vertex handles win over Shift).
**Construction guidelines** (draw mode, serialization v11): **infinite** lines defined by
two points — pure drawing aids (invisible and unpickable in sim, never simulated). Tool
**`L`** (the linear actuator moved to **`A`**): two clicks; each click lands **exactly on**
a joint / body corner / another guide's point (recording a CAD-style **auto-coincident**),
projects onto a slider rail / body edge, or falls back to the grid/guide snap. Rendered as
muted dash-dot lines spanning the viewport with dots on the two defining points; click to
select (endpoints beat the line, the line beats body areas), **drag the line** to move it
whole (angle kept), **drag a defining point** to re-aim it (endpoint drags also land on
joints / corners / other guides' points), Delete removes (constraints + measurements on the
guide cascade away). With **snap on, guidelines beat the grid**: near one line the point
projects onto it, near two it lands on their **intersection**. Guides are first-class
`MeasureRef`s (`guidePoint` / `guideLine`), so **measurements** and **sketch constraints**
work on them — coincident on defining points; H/V, parallel, perpendicular on the lines
(`equal` is rejected: an infinite line has no length). The sketch solver ranks mobility
**construction < geometry < dragged** — guide constraints are satisfied by moving **only
free guide points, never joints or body nodes** (a guide fully bound to joints rejects an
unsatisfiable H/V with a red flash), while drags pin the dragged geometry so guides follow
it exactly; when a drag would break a constraint the solver falls back to a symmetric
re-solve **that same frame**, so dragging can only move things along the directions the
constraints leave free (a bound vertical guide's free point slides only vertically).
**Working units + DXF import + bodies with holes** (serialization v12 + v13): a **unit
dropdown** in the toolbar's grid group declares what one world unit means (mm / cm / m / in;
default mm) — purely declarative (nothing moves on change), saved with the file, undoable;
distance measurement labels show the unit suffix. **Drag-and-drop import onto the canvas**:
a `.dxf` file imports as bodies at the drop point (a `.json` loads as a scene, same as the
Load button). The importer (`src/dxf.ts`, hand-written ASCII DXF reader, zero deps) handles
LWPOLYLINE / POLYLINE (bulges sampled), CIRCLE, and loose LINE/ARC segments auto-chained
into closed loops; `$INSUNITS` converts coordinates into the working unit (a unitless file
is taken as already in working units), y is flipped (DXF is y-up), and the batch arrives
multi-selected. **Loops are nested by containment**: a loop inside another becomes a **hole**
of it (an island inside a hole starts a new solid), so a plate with cut-outs imports as ONE
body. Holes render as cut-outs (even-odd fill), subtract from mass/centroid/inertia
(composite properties via parallel-axis), and ride mirror / rotate / scale / copy-paste /
save-load; **picking + joint containment deliberately use the outer outline only** (so a
joint can sit at the centre of a shaft hole, and clicking in a cut-out still selects the
body). Originally baked loops — since **v16 holes are editable outlines** with their own
control polygon, rounding and measurement/sketch refs (see the v15/v16 paragraph below).
**Z-order reordering** (no format change): **Send to back / Bring to front** on the selected
body or multi-selection — toolbar buttons in the edit group (layer-stack icons, the moving
layer highlighted, with a down/up arrow) and **PageDown / PageUp**. The `bodies` array *is*
the z-order (rendered first→last, hit-tested last→first), so one reorder fixes both drawing
and picking — e.g. push a big imported DXF reference body behind the mechanism so it stops
covering it and stealing its clicks. Group-aware, undoable, order persists in save/load.
**Hierarchical components + groups with free-joint members** (serialization v14): reusable
**component definitions** (each a full sub-scene edited in its own context, breadcrumb-
navigated) placed as **materialized instances** — real bodies/joints/constraints in the
owning context, tagged with provenance maps, so the solver/renderer/hit-testing needed no
hierarchy concept. Grounding *inside* a definition means "fixed to the component frame" and
is **converted** on expansion: grounded bodies + grounded free joints become one rigid
**chassis group** per instance (never world-grounded); a joint-ground on a moving part
becomes a pin to a synthesized group-locked point (revolute to the frame); the def's sketch
constraints / dimensions / measurements / guides stay in the def (instances carry shapes,
not design constraints — so instances rotate freely, which was the motivating pain with
H/V constraints). Editing a definition **cascades** to every instance everywhere (other
defs that use it included — defs form a DAG, cycles rejected) via a reconciling re-expansion
that preserves each instance's placement and scene ids while snapping every part's pose
back to the definition layout (the def is the pose reference).
Prerequisite feature (also user-facing): **groups now lock free joints as members**
(`BodyGroup.jointIds`) — point masses riding the rigid composite; a ground on a locked joint
pivots its group about it; two locked joints can define a group-riding slider track. UI:
**⊞** create-component-from-selection, a **component browser** panel (rename / insert /
edit / delete), double-click an instance to edit its def, Esc / breadcrumbs to exit,
instance-atomic selection with a dashed hull, copy/paste of instances as new instances,
Shift-drag poses an instance's internal mechanism. `SelectionClip` now also carries
**actuators + motors** (an old backlog item, needed so components keep their powered
constraints).
**Per-corner radii + parametric DXF corners (v15) and editable holes (v16)**: every outline
corner can override the body-wide radius (`Body.radii`, null = default). A selected body
shows a **round radius handle** on each corner (at the fillet arc's midpoint, pushed to a
min screen offset when sharp; the squares still move vertices — nearest handle wins): drag
it along the bisector to round just that corner, drop it on the vertex for sharp,
double-click it to clear back to the outline default; `[`/`]` still set the body-wide
default (overridden corners keep their override). Offset-mode outlines take per-point
margins (a 1-point disk resizes by its rim handle). Arc sampling is unified at
**7.5°/segment** everywhere (fillet 24/180°, offset 48/circle — matching the DXF importer).
The **DXF importer reconstructs fillets**: a bulge arc sweeping < 180° and tangent to its
straight neighbours imports as a sharp control corner + per-corner radius (so imported
rounded corners stay parametric/editable); a CIRCLE imports as a 1-point offset disk;
non-tangent / ≥ 180° arcs stay sampled as before. **Holes are now first-class editable
outlines** (`Body.holes`: control polygon + radius / radii / round each; `holesLocal` is
derived): holes get the same vertex squares, radius handles, and double-click node
add/remove as the outer outline (double-clicking a hole's last removable node deletes the
whole hole, refs cascading); hole corners/edges are measurement + sketch-constraint
references (`MeasureRef.hole`) and full sketch-solver variables, the joint↔node stick-link
works on hole vertices, and circular DXF cut-outs arrive as resizable disk holes. Mirror
remaps hole refs within their own outline, scale / copy-paste / components / save-load all
carry hole shapes, and legacy (≤ v15) baked hole loops load as radius-0 editable holes.
Picking + joint containment still deliberately use the outer outline only.
**Hole tool** (`U`, no format change): holes can now be **drawn in the UI**, not only
imported — the first click picks the body to cut (topmost under the cursor; empty space is
ignored, an instance body is refused with an alert to edit the definition), later clicks add
cut-out vertices (grid-snapped with the usual containment fallback: a snap that would land
outside the body uses the exact click point, a click outside is ignored), and clicking the
first vertex / double-click / Enter closes the loop into an editable **radius-0 hole**
(`Scene.addBodyHole`, appended after existing holes so their refs keep their indices); the
body is then selected so the new hole's handles show immediately. Esc aborts the draft. The
in-progress polygon previews through the body tool's dashed-polyline render channel.
**Fork a component instance / "make unique"** (no format change): pressing **⊞** with exactly
one component instance selected no longer alerts — it **forks**: `Scene.makeInstanceUnique`
deep-copies the instance's definition into a new component (named `"<source> copy"`, deduped
with a counter) and re-points the instance's `defId` at the copy, so the two definitions
evolve independently from then on. Nothing moves — the copy is byte-identical, so every
provenance src id stays valid and no re-expansion is needed (a test verifies the reconcile
is a drift-0 no-op). The instance stays selected, the component browser opens on the new
definition, and the hint line announces the fork. Works inside definition-editing contexts
(no cycle risk: the copy uses exactly the sub-definitions the original did — nested defs
stay **shared**, the def DAG gains one node with the same out-edges). Undo/save needed no
work (`canonicalData` already persists `scene.components`). A selection mixing an instance
with other material still alerts, now explaining both options.
**Point-on-line coincident** (no format change): the Coincident constraint (`O`) now also
takes a **point + a line** (body edge, slider rail, or guideline — either pick order; the
point is normalized into `refA`, where the single badge anchors) and holds the point on the
**infinite** line — consistent with point+line measurements and H/V. The sketch solver zeroes
the signed perpendicular distance, split by the usual mobility ranks (a joint constrained
onto a guideline moves the guide, not the joint; a drag stays pinned). A point that *is* a
defining point of the line (an edge's end vertex, a rail's own joint, a guide's own point)
is rejected as a permanent no-op.
**Rails & sliders (v17)**: the old slider-line tool is now called **Rail** (`K`; the line
joints ride along — "track"/"rail" in CAD terms; the constraint keeps `kind: "slider"`
internally for format compatibility, but the tool id, selection kind `"rail"`, and all
user-facing text say rail).
New **Slider tool (`S`)**: click a rail to add a **slider** — an **orientation-locked
rider** (a prismatic joint, matching Fusion 360 / Onshape naming): it travels along the
rail but its body keeps its angle **relative to the rail** (a plain rider remains a
pin-in-slot: slides AND rotates). The click point projects onto the rail; the new joint
attaches to the topmost body under the cursor (excluding the rail's own body) or is free
otherwise — a lock on a free rider is **inert until the joint gains a body** (e.g. absorbed
by build-body-from-joints). Clicking an existing rider with the tool **toggles** its lock;
clicking a loose joint near a rail attaches it as a locked rider. The locked-in relative
angle is **captured from the drawn pose** when simulation starts (or a rigid Shift-drag
begins) — `resetPoseBaselines()` (named `resetSliderLockBaselines` pre-v18) is called from
`markDirty` / sim entry / `setDocument`, and the solver lazily captures missing baselines
per locked rider — so rotating the body in draw mode re-locks the new angle (the drawn
pose is the intended assembly, same philosophy as pins). Solver implementation (reworked
in v18 — see the welds paragraph): a **direct angular projection** rotates the rider's
unit against the rail's unit about the rider point until the wrapped relative angle
matches the baseline; the error metric stays positional — rider off-line distance plus
`(dl/2)·Δangle` — commensurate with positional tolerances; works on moving rails, groups,
fixed tracks, and under anchors. Each lock is
its own Phase-B unit keyed by the **negated rider id** (unique — scene ids are positive),
so an unreachable lock is disabled/reported independently of the rider's on-rail unit.
Rendering: a green rail-aligned **carriage rectangle** on each locked rider.
`SliderConstraint.locked: number[]` (⊆ riders, v17; legacy files load with none) persists
through save/load, copy/paste (`SelectionClip.sliders[].locked`), component expansion, and
cascades away with the rider; `attachSliderRider` gained a `locked` flag, plus
`sliderOfRider` / `setSliderRiderLocked`. The analyzer counts a locked rider as removing
2 DOF (vs 1). New test script `scripts/slider-locks.ts` (12 checks).
**Empty components + the definition as pose reference** (no format change): pressing **⊞
with nothing selected** now creates a **completely empty component** and opens it for
editing — the way to build a component made entirely of other components, no throwaway
body needed (`Scene.createEmptyComponent`; selecting only free joints still alerts, a
component needs ≥ 1 body). `instantiateComponent` **refuses a still-empty definition** (an
instance with no material would dissolve instantly) and the browser's ＋ insert button
alerts with a hint to fill the def first. And re-expansion now treats the **definition as
the pose reference**: on any def edit, every instance part — bodies AND free mechanism
joints, not just chassis material — snaps to `T·defPose` (`T` = the instance placement,
which is preserved), cascading through nested defs; a mechanism posed at the assembly
level resets whenever its definition is next touched (what's drawn in the def is what
instances show).
**Hotkey swap + joint-containment warning** (no format change): the Slider tool now owns
**`S`** and the Rail tool moved to **`K`** (S had kept the pre-v17 muscle memory; the
slider is the more-placed element, so it gets the natural letter). And the containment
invariant is now **watched, not just enforced at edit time**: direct edits can't put an
attached joint outside its body (placement + drags clamp), but a **shape change under the
joint can** — the canonical case is an assembly-level joint on a component instance whose
definition is then reshaped so the new outline no longer covers it (a removed control
vertex can do it too). `Scene.jointsOutsideBody()` reports such joints (outside the
rounded outline by more than `CONTAINMENT_WARN_EPS = 1e-6`, so a joint clamped exactly
onto the edge never counts); main re-checks every frame in draw mode, the renderer paints
the stranded joints **error-red with a dashed ring** (vs the solid red ring of a sim
break), and the hint line prepends a warning ("⚠ N joints lie outside their bodies…")
while any exist. Deliberately **nothing is auto-moved** — the user drags the joint back
(drags clamp to the outline) or fixes the shape / definition, and the flag clears itself.
**Welds — rigid joints (v18)**: a weld is a **pin with a `rigid` flag** (`PinConstraint.rigid`,
present only when true), so it rides every existing pin pathway — cascade deletion, undo,
copy/paste (`SelectionClip.pins[].rigid`), component expansion (the def is the reference:
toggling rigid in a definition cascades), save/load (pre-v18 pins load revolute). Besides the
pin's coincidence it locks the two bodies' **relative angle**, joining them completely; the
angle is captured **from the drawn pose** at sim entry / rigid drag (weld baselines live next
to the slider-lock baselines, both cleared by the renamed `resetPoseBaselines()`). A weld
with a free (body-less) joint on either side acts as a plain pin until the joint gains a
body. New **Weld tool (`W`)**: click where bodies overlap → a joint in each, welded together
(same placement mechanics as the Joint tool); click an **existing pinned joint** → toggles
its pin(s) weld ↔ revolute (mirroring the Slider tool's lock toggle). Rendering: welded
joints keep the pin blue but get a **square** center (no hollow revolute dot) plus a blue
square outline. Analyzer: a weld removes 3 DOF (pin 2). Solver: the weld's orientation is
its own Phase-B unit keyed by the **negated pin id** (like slider locks use negated rider
ids), solved by a **direct angular projection** — the two bodies (or groups) rotate about
the weld point, split by inverse inertia about it (a fixed side is one-sided) — after the
first phantom-pair implementation converged pathologically slowly (see the bug list: drags
could exhaust the cleanup budget and falsely break the assembly). The same drag-yield bug
hunt also made **slider locks flip-proof**: the lock error is now the **wrapped relative
angle** (unique zero — the old phantom-on-line `sin` error was also satisfied by a 180°
flip) enforced by the same angular projection (rider unit vs rail unit about the rider
point). New test script `scripts/welds.ts` (17 checks).
**Dimensions on components — pose-level driving dimensions (no format change)**:
components are now a *transparent grouping* for dimensioning — you can dimension and
drive anything to anything, as long as it doesn't fight what's already fixed at a deeper
level. Instance **shape** stays design-locked (the sketch solver gives instance-owned
variables a top mobility rank — immovable — and a dimension fully internal to one rigid
piece rejects), but a driving dimension can now move instance **poses**: with one end on
free geometry the sketch moves the free side; with **both ends on instance geometry**
it's a **pose dimension** (new `src/pose.ts`) — between two different instances one of
them **translates rigidly** to the target (a grounded instance never moves; conflicting
pose dims reject with the red flash), and between two mobile parts of ONE instance the
rigid-drag sim solver **re-poses the internal mechanism** (everything outside the
instance plus the other end's rigid unit frozen, pins intact; two ends actually rigid to
each other — same body / chassis / welded — can't reach the target and reject). Pose
dims are **enforced live during drags** (dragging a component pulls its dimensioned
partners along, the dragged instances anchored) and **re-asserted after definition-edit
cascades** (on def-context exit); one that can't hold (grounded partner, def edit)
renders **violated in error red** until re-applied (`MeasureInfo.violated`,
`DIM_VIOLATION_TOL`). And every driving dimension now holds its **drawn relative
direction**: an optional `Measurement.side` (1 | −1, captured when the dim starts
driving — same "as drawn" philosophy as weld/lock baselines) makes the h/v, point+line
and line+line corrections **signed** in both solvers, so a fast one-frame drag past the
partner can never flip the two sides through each other (direct point-point distances
have no side by nature — a pure distance is free to rotate). New test script
`scripts/pose-dims.ts` (36 checks).

### Tech stack
- **Vite + TypeScript + HTML5 Canvas** (no UI framework). Builds to static files.
- Constraint solver: **iterative position-based** (Gauss-Seidel projection).
- Node 24 / npm. Scripts: `npm run dev`, `npm run build`, `npm run preview`, `npm test`.

### Architecture (`src/`)
- **geometry.ts** — Vec2 math; polygon centroid / area / inertia; point-in-polygon;
  point-to-line distance; `distToSegment` (point to a clamped segment);
  `closestPointOnPolygon` (nearest point on a closed polygon's boundary — used to clamp
  joints inside their body); `convexHull`;
  `roundedConvexBody` (hull + outward rounded offset = Minkowski sum with disks; `margin`
  is uniform or **per-point** — the hull of different-size circles; 48 facets per circle);
  `filletPolygon` (round a polygon's corners in place with tangent arcs — convex and
  concave/reflex corners; `radius` is uniform or a **per-corner array**, ≤ 0 keeps a corner
  sharp; default sampling 24 segments per 180° = 7.5°/segment, matching the DXF importer).
  The fillet runs in passes: per-corner desired
  tangent length, a **shared-edge budget** (each edge split between its two corners in
  proportion to demand so neighbouring fillets can't overlap), an **opposite-edge clamp**
  (a few relaxation passes shrink any corner whose inscribed circle would poke through a
  non-adjacent edge — stops thin shapes folding at large radii), then arc emission. The
  fillet centre is always on the bisector of the two edge directions, which is the correct
  tangent side for both convex (into the body) and reflex (into the notch) corners.
  `filletCornerArcs` exposes each corner's solved arc (centre, actual radius, sweep)
  without sampling — the editor places the per-corner radius handles exactly on the arcs.
- **model.ts** — `Scene` owning:
  - `Body` = rigid shape defined by an **editable control polygon** (`controlLocal`) + a
    corner `radius` + optional per-corner overrides `radii: (number | null)[]` (v15;
    parallel to `controlLocal`, null = use `radius`, present only while ≥ 1 override) + a
    `round` mode (`"fillet"` rounds corners in place; `"offset"` grows the
    hull outward — with `radii`, offset takes per-point margins). The render/physics polygon
    `local` is **derived** from these. Pose is `pos`
    (world centroid) + `angle`. `invMass`/`invInertia` from the derived polygon's area / second
    moment. `rebuildBody` regenerates `local`/centroid/mass and re-anchors attached joints when
    the centroid shifts. A `grounded` flag (v10) fixes the body completely in simulation
    (see solver notes); `toggleBodyGround(bodyId)` toggles it — through a grouped body, on
    every member of the group at once (any-grounded → all off, else all on).
    **Holes** (v16): optional `holes: BodyHole[]` — each an **editable outline** exactly like
    the outer shape: `{ controlLocal, radius, radii?, round? }` in the body's local frame
    (`round: "offset"` with one control point = a perfect circular hole). `holesLocal`
    (parallel, present iff `holes` is) holds the **derived** sampled loops that the renderer
    and mass properties consume — `rebuildBody` re-derives them (`deriveHoleOutline`) and
    re-centers hole controls alongside the outer polygon. `addBody` takes `holesWorld?:
    HoleSpec[]` (a plain loop → radius-0 control polygon, or `{ control, radius?, radii?,
    round? }`). `rebuildBody` computes **composite mass properties** (net area, area-weighted
    composite centroid, inertia via per-loop moments + parallel-axis; falls back to
    outer-only if holes would outweigh the outer). Holes ride `mirrorBody` (controls
    reflected + winding and radii reversed, refs remapped per-outline), `scaleBody`
    (controls + radius/radii), rotate/move (local frame), copy/paste
    (`SelectionClip.bodies[].holes` carries the shapes) and component expansion.
    `addBodyHole(bodyId, spec)` cuts a new hole into an existing body (a world-coord
    `HoleSpec`, so a 1-point offset spec cuts a parametric disk; world → local under the
    body's pose; appended after existing holes so their refs keep their indices; returns
    the new hole's index, or null when rejected — an offset hole needs ≥ 1 control point,
    a fillet one ≥ 3). `removeBodyHole(bodyId, hole)` deletes one hole, dropping its refs
    and shifting later holes' refs down. `pointInBody`/`clampIntoBody`/`bodyAt` still use the **outer outline
    only** by design (joints may sit inside a hole, e.g. at a shaft's centre). Legacy
    (v13–v15) files carry only baked `holesLocal` loops — they load as radius-0 editable
    holes with identical geometry.
  - `Joint` = a point with `bodyId` + `local`. If `bodyId` is a body, `local` is the offset
    from that body's centroid; if `bodyId === null` it is a **free joint** and `local` is its
    own world position (the solver treats it as a movable point particle).
  - Constraints: `pin` (two joints coincide, free rotation — or a **weld** when `rigid`
    (v18): the relative angle locks too, joining the bodies completely; `addPin(a, b,
    rigid?)`, `setPinRigid(id, on)`, `pinsOfJoint(id)`; the flag is present only when
    true), `ground` (a joint locked to a
    fixed world point — grounding a free joint makes a body-less anchor), `slider` (a **rail**
    in the UI: two joints `railA`/`railB`, plus a `riders` list of joints confined to the
    segment between them, with end-stops, and a `locked` list (v17, ⊆ riders) naming the
    **orientation-locked riders** — "sliders" in the UI: prismatic, the rider's body keeps its
    drawn angle relative to the rail; a plain rider is a pin-in-slot). The rail is either
    **two joints on one body** (it moves with that body, coupling two bodies) or **two free
    joints** (a track fixed in world space — `addSlider` auto-grounds them). Riders may be
    body joints or free joints (a lock on a free rider is inert until it gains a body).
    `attachSliderRider(sliderId, jointId, locked?)`, `setSliderRiderLocked`, `sliderOfRider`. Two new "powered" constraints
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
    `removeBodyVertex` (drop one, kept ≥ 3 — ≥ 1 for an offset disk hole), `setBodyRadius`
    (the outline-wide default), `setBodyCornerRadius(bodyId, index, r | null)` (per-corner
    override / clear), `bodyCornerRadii` (effective per-corner list), `moveJoint`, `moveBody`
    — every vertex/radius method takes an optional trailing `hole` index (null/absent = the
    outer outline; `bodyHoleControlWorld(body, hi)` gives a hole's world control vertices),
    and shape edits go through `rebuildBody`, so attached joints stay anchored. **Joint containment**:
    `pointInBody` / `clampIntoBody` test/clamp a world point against a body's rounded outline;
    `moveJoint` (via the private `shiftJoint`) clamps an attached joint's target inside its
    body, so a drag can't take it outside (free joints are unclamped; ground anchors follow).
    `jointsOutsideBody()` reports attached joints a *shape change* stranded outside their
    body (beyond `CONTAINMENT_WARN_EPS = 1e-6`, so edge-clamped joints never count) — e.g.
    a component-definition edit cascading into instances, or a removed control vertex; the
    UI flags them (red dashed ring + hint warning) and never moves them.
    **Node ↔ joint link** (`VERTEX_LINK_EPS = 1e-6`): `moveBodyVertex` carries any joint of
    that body exactly coincident with the moved control vertex (moved *after* the rebuild, so
    every other joint stays anchored); `moveJoint` on a joint coincident with a control vertex
    — outer **or hole** (v16) — delegates to `moveBodyVertex` — so the link is bidirectional
    and coincidence-based (no stored mapping; survives save/load, copy/paste, mirror, rotate). Edge cases: only the
    body's *own* joints follow a node (a pinned twin's partner on another body stays put — the
    pin shows as a dotted connector until sim closes it), and rounding a freehand body's
    corner after linking dissolves that link on the next drag (the control corner leaves the
    rounded outline, where a joint may not go).
  - **Permanent groups** (`groups: BodyGroup[]`, a `BodyGroup` = `{ id, bodyIds, jointIds }`,
    v14): a member (body, or **free joint** locked to the group) belongs to at most one
    group; groups need ≥ 2 members total. `groupOf(bodyId)` / `groupOfJoint(jointId)`;
    `addGroup(bodyIds, jointIds?)` (absorbs/merges any group touching a member; attached
    joints are skipped); `ungroup(bodyIds, jointIds?)`; `pruneGroups()` (drops removed /
    absorbed members, dissolves < 2-member groups — called from `removeBody`, `removeJoint`
    and `load`). Groups make their members move together in draw mode (main.ts) and act as
    one rigid body in sim (solver.ts) — locked free joints ride the rigid motion like
    welded points (this is what a component chassis is made of). `addSlider` does **not**
    auto-ground a group-locked free rail joint (the rail rides the group instead), and
    `buildBodyFromJoints` never absorbs one (it gets a pinned twin, like an anchor).
  - **Components** (`components: ComponentDef[]` — document-level, kept across context
    switches; `instances: ComponentInstance[]` — per context): a `ComponentDef` =
    `{ id, name, data: SceneData }` (a full sub-scene in its own frame; may itself contain
    instances of other defs — a DAG). A `ComponentInstance` carries provenance maps
    (`bodyMap` — with each body's cached def-frame pose + `chassis` flag — `jointMap`,
    `constraintMap`, `anchorMap` for the joints synthesized from def joint-grounds,
    `groupMap` for groups recreated from the def's own groups, and `groupId` = the chassis
    group). Key methods: `createComponentFromSelection(name, bodyIds, freeJointIds)` (packs
    the selection — via `extractSelection` with `drivenDims: true` — into a new def and
    replaces it with one instance in place), `createEmptyComponent(name)` (a blank def, no
    instance placed — meant to be opened for editing and filled with bodies and/or
    instances of other defs), `instantiateComponent(defId, {pos, angle})` (refuses an
    unknown **or still-empty** definition),
    `makeInstanceUnique(instanceId)` (fork: deep-copies the instance's def into a new
    component — name `"<source> copy"` deduped — and re-points the instance's `defId`;
    the copy is identical so provenance maps stay valid and no re-expansion runs; nested
    sub-definitions stay shared),
    `reexpandInstances(changedDefIds)` (the reconciling re-expansion: surviving elements
    keep scene ids; design fields — shape, colour, joint locals, constraint wiring — AND
    **every part's pose** come from the def: bodies and free mechanism joints all snap to
    `T·defPose`, where `T` = the instance placement derived from a chassis body's scene
    pose vs its cached def pose — the definition is the pose reference; only the
    instance-level grounded flag is instance state), `removeInstance` / `dissolveInstance`
    (explode to plain elements) / `removeComponent` (refused while instances exist
    anywhere), `instanceOfBody/Joint/Constraint`, `instanceOfRef(ref)` /
    `refInstanceOwned(ref)` (sketch constraints on instance geometry are rejected — its
    *shape* belongs to the def — but dimensions may drive its *pose*, see pose.ts),
    `refRigidUnitKey(ref)` (the rigid unit a ref belongs to — group / lone body / lone
    free joint — two refs with the same key can never be pose-dimensioned apart),
    `moveInstance(id, delta)` (rigid translation of everything the instance expanded),
    `instancePlacement(id)`, `componentUses(defId)` (transitive, for cycle
    guards), `componentCenter(defId)`. **Ground conversion** in `expandInstance`: def
    grounded bodies → chassis members (flag stripped; the *instance's* grounded flag is
    user state and survives re-expansion); def grounded free joints → group-locked chassis
    points; def joint-grounds on non-grounded bodies → `addFreeJoint` at the anchor +
    group-lock + pin (revolute to the frame); grounds are never expanded as grounds.
    Exported helpers `reexpandData(data, components, changed)` (refresh a stored context
    snapshot in a scratch Scene) and `cascadeComponentChange(components, changedIds)`
    (fixpoint propagation through the def DAG). `serializeContext()` / `loadContext()`
    are the component-editing variants of `serialize()` / `load()` (they leave
    `components` untouched); `clear(dropComponents = true)`.
  - **Z-order** (`reorderBodies(bodyIds, "back" | "front")`): the `bodies` array *is* the
    z-order — rendered first→last (first = bottom), hit-tested last→first (`bodyAt` walks
    backwards, so the last body wins the click) — so one stable partition of the array fixes
    drawing and picking together. Group-aware (any id expands to its whole group), moved
    bodies keep their relative order, returns whether the order actually changed. Array
    order already survives `serialize()`/`load()`, so persistence is free (no format bump).
  - **Construction guidelines** (`guides: Guide[]`, a `Guide = { id, a, b }` — two world
    points defining an **infinite** line; drawing aids only, never simulated): `addGuide`
    (rejects spans < `GUIDE_MIN_SPAN`), `removeGuide` (cascades — prunes measurements and
    sketch constraints referencing the guide), `moveGuide` (translates both points, angle
    kept), `moveGuidePoint` (re-aims; refuses collapsing onto the twin point), hit tests
    `guideAt` (distance to the **infinite** line) and `guidePointAt` (defining points;
    optional `excludeGuide` so a dragged point can't pick itself). `MeasureRef` gained two
    kinds — `guidePoint` (a defining point) and `guideLine` (the line) — resolvable by
    measurements and sketch constraints alike; `addSketchConstraint` accepts them wherever
    points/lines go, except `equal` on a `guideLine` (rejected — no meaningful length).
    Copy/paste drops refs to guides (guides don't travel with a `SelectionClip`).
  - Edit utilities: `rotateBody(id, pivot, delta)` rigidly turns a body about a fixed world
    point (joints follow; ground anchors rotate too; no rebuild needed). `mirrorBody(id, "h"|"v")`
    reflects the control polygon + attached joints + their ground anchors across a centroid axis,
    reversing winding so the fillet/offset stays valid (centroid is fixed, so the body doesn't
    move) — and **remaps every vertex/edge sketch-constraint & measurement ref** to track the
    renumbered corners (vertex `i → n−1−i`, edge `i → n−2−i` wrapping), re-baking `bodyPoint`
    refs onto the reflected material, so constraints stay on the right elements (reflection
    preserves every constraint kind: H stays H, V stays V, the rest are reflection-invariant).
    `mirrorBodies(bodyIds, freeJointIds, axis)` mirrors a whole selection about the centre of
    its **combined bounding box**: each body mirrors in place + its centroid reflects across
    the shared axis, free joints reflect their position — cross-body pins stay coincident.
    Copy/paste: `extractSelection(bodyIds, freeJointIds?)` snapshots a **`SelectionClip`** —
    one or more bodies (control polygon, radius/round, **colour**), their joints, selected
    free joints, and every constraint whose joints all travel with the clip: grounds, internal
    sliders, and pins **including pins between copied bodies** (anything reaching outside the
    selection is dropped), plus **group membership** among the copied bodies and the
    fully-internal sketch constraints / driving dimensions (driven reference dimensions are
    annotations and aren't copied). `clip.center` is the mass-weighted centre of the copied
    bodies (= the centroid for a single body; the joints' average for a body-less clip).
    `insertSelection(clip, at)` translates the whole fragment so `center` lands at `at`,
    recreates everything with fresh ids (body + joint + slider ids remapped into the
    constraint/dimension refs; grounds created **before** sliders so `addSlider`'s
    auto-grounding doesn't double-ground a copied fixed track; groups recreated) and returns
    `{ bodyIds, freeJointIds }` for re-selection. No re-solve is needed on paste: everything
    carried is translation-invariant. `extractBody(id)` / `insertBody(clip, at)` remain as
    single-body convenience wrappers.
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
    or a **line** (slider `rail` id / body control-polygon `edge` index). Vertex/edge refs
    carry an optional `hole` index (v16) naming one of the body's hole outlines instead of
    the outer one (a 1-point disk hole has vertex refs but no edge refs). `resolveMeasureRef`
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
    rail/edge), rejects duplicates of the same element and unresolvable refs. Coincident
    takes two points, **or a point + a line** (point-on-line, normalized so the point is
    `refA`; `pointIsLineEndpoint` rejects a point that structurally *ends* the line — an
    edge's own vertices, a rail's own joints, a guide's own defining points — as a
    trivially-satisfied no-op).
    `removeSketchConstraint` / `getSketchConstraint`; `pruneSketch` runs alongside
    `pruneMeasurements` (element deletion cascades) and `shiftMeasureIndices` also remaps
    sketch vertex/edge refs. `sameMeasureRef` (exported) compares refs by element.
    **Driving dimensions**: draw-mode `Measurement`s gain optional `driving` + `target` +
    **`side`** (1 | −1 — the held relative direction, captured from the drawn geometry by
    `setMeasurementDriving` via `measurementSide(m)`; re-captured when a label drag
    changes the axis; cleared with the driving flag; sanitized + back-filled on load; no
    side for direct point-point dims). `setMeasurementDriving` rejects only a pair of
    instance-owned refs **rigid to one another** (`refRigidUnitKey` equal) — any other
    pairing may drive (pose or sketch). `measureInfo` passes `driving` through to
    `MeasureInfo` and sets **`violated`** when a driving dim's value is off its target by
    more than `DIM_VIOLATION_TOL` (2e-3) or sits on the flipped side of its held
    direction — the renderer paints those error-red. **`scaleBody(id, factor)`** scales a body
    uniformly about its centroid — control polygon, corner radius, attached joints, their
    ground anchors, and `bodyPoint` measurement refs all together.
  - **Working unit** (`unit: Unit` on the scene, v12): `Unit = "mm"|"cm"|"m"|"in"` with
    `UNIT_TO_MM` (both exported) — a declaration of what one world unit means; changing it
    never moves geometry. Used for measurement display and DXF import conversion.
  - `serialize()` / `load(SceneData)` for save / load / autosave (versioned plain-data
    snapshot, `FORMAT_VERSION = 18`; `load` deep-copies, recomputes `nextId`, drops legacy
    origin+dir sliders, migrates older single-`slider` → `riders`, and back-fills
    `controlLocal`/`radius`/`round` for pre-v5 bodies; pre-v6 files simply have no
    actuator/motor constraints, pre-v7 files no measurements, pre-v8 files no sketch
    constraints or driving flags, pre-v9 files no groups, pre-v10 bodies load ungrounded,
    pre-v11 files no guides, pre-v12 files default to `unit: "mm"`, pre-v13 bodies have no
    holes, pre-v14 files have no components/instances and their groups get empty
    `jointIds`, pre-v15 bodies have no per-corner `radii`, pre-v16 files carry only
    baked `holesLocal` loops (loaded as radius-0 editable holes; v16 loads sanitize `holes`
    and **re-derive** `holesLocal` from the controls, so the two can never disagree), and
    pre-v17 sliders have no `locked` riders (a hand-edited lock on a non-rider is dropped:
    locked ⊆ riders is a load invariant), and pre-v18 pins have no `rigid` flag (loaded
    revolute; a non-boolean hand-edited value is sanitized away) —
    all load fine as-is; loaded groups/instances are pruned against the loaded elements.
    `Measurement.side` is likewise an optional field (no format bump): load sanitizes
    non-±1 values, drops it from driven dims, and **back-fills** it on driving dims from
    the loaded — satisfied — geometry, so pre-side files gain flip protection on load).
    The `SelectionClip` (copy/paste) carries each body's `grounded` flag alongside its
    colour, per-corner `radii`, and **hole shapes** (`bodies[].holes`: world control +
    radius/radii/round each), pins/grounds/sliders internal to the selection (pins with
    their **rigid/weld flag**, rails with their riders **and locked lists**),
    **actuators + motors** (when their slider/body travels), groups (bodies + locked
    joints), internal sketch constraints and dimensions (driving always; driven only for
    component creation via `opts.drivenDims`).
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
  and an unconditional early-exit would starve the drag to one pull per solve.
  **Permanent groups as rigid composites**: `solve` builds a per-call `groupCtx`
  (`RigidGroup` = member bodies + **locked free joints**, indexed by body id and joint id;
  module-level since solve isn't reentrant); a grouped member's host (`bodyHostAt` /
  `groupHostAt`, used by `hostFor` / `driverHost` / rails) carries the group's **combined**
  mass, centroid, and inertia (parallel-axis; locked joints as point masses), and
  `groupImpulse` translates + rotates **all members about the combined centroid** — locked
  free joints included — so pins, sliders, grounds, the mouse driver, and motor/actuator
  anchors on any member move the whole group as one rigid body. A **grounded locked joint**
  behaves like a joint-ground on a body: its own ground uses the group host (the group
  pivots about the anchor; `projectGrounds` pools the correction per group and translates
  bodies *and* locked joints), while pins to it see a fixed point (`pinHostFor`).
  `projectGrounds` pools ground corrections **per rigid unit** (per group, or per lone
  body). `sameRigid(a, b)` compares **rigid-unit keys of joints** (same body, or members —
  body or locked joint — of one group) and makes **intra-unit pins and riders inert**:
  skipped in the sweeps and excluded from residuals/breaks (the unit is rigid — they could
  only fight it). **Rails** go through `railInfo`: a rail is solvable when its two joints
  are rigid to one unit — same body (as before) or same **group** (two locked free joints,
  or mixes: a chassis track that moves with a component instance) — or when both are
  grounded free joints (world-fixed track); the rail host is the body / group / immovable
  line accordingly, and a fixed group makes its rails immovable lines.
  **Slider orientation locks (v17, enforcement reworked in v18)**: a module-level
  `lockBaselines` map (rider id → the rail direction expressed in the rider body's frame
  at capture) is captured lazily by `captureLockBaselines` at the top of every `solve` and
  cleared by the exported `resetPoseBaselines()` (main calls it from `markDirty`, sim
  entry, and `setDocument`, so a sim session / rigid drag locks the relative angle **as
  drawn**; it also clears the weld baselines below). `solveLockedRider` is a **direct
  angular projection**: `unitAngHost` resolves each side's rigid unit (body / group /
  fixed) into an angular host about the rider point — inverse inertia about that pivot,
  parallel-axis — and the rider's unit rotates against the rail's unit until the
  **wrapped** relative angle matches the baseline (pivoting at the rider leaves the
  on-rail position untouched). `lockPhantom` remains as the error **metric**: rider
  off-line distance + `(dl/2)·wrapAngle(Δ)` — unlike the original phantom-on-rail-line
  `sin` error, zero ONLY at the locked angle, so a violently dragged carriage can no
  longer settle silently into a 180°-flipped pose. Each lock is a `StructuralUnit` keyed
  by the **negated rider id**, so Phase B can disable / Phase C can close a lock
  independently of its rider; a lock on a free rider, a degenerate rail, or a rider rigid
  to the rail unit is inert.
  **Welds (v18, `PinConstraint.rigid`)**: `weldBaselines` (pin id → the two bodies' angle
  offsets + a phantom arm length = mean characteristic body radius √(area/π)) is captured
  lazily by `captureWeldBaselines` next to the lock capture and cleared by the same
  `resetPoseBaselines()` — a weld locks the relative angle **as drawn**, and one with a
  free joint on either side has no baseline (plain pin until the joint gains a body).
  `solveWeld` is the same direct angular projection: both sides' units rotate about the
  weld point (midpoint of the two joints), split by inverse inertia about it, a fixed side
  one-sided, both fixed → left for break reporting. `weldPhantoms` is the error metric (two
  points rigid in the bodies, offset `arm` along the captured shared direction — coincident
  exactly at the welded angle, so the unit error ≈ `arm·Δangle`). Each weld's orientation
  is its own `StructuralUnit` keyed by the **negated pin id** (unique — scene ids are
  positive), disabled/closed independently of the pin itself; a weld inside one rigid unit
  is inert (`sameRigid`). The first implementation enforced the weld as a phantom **second
  pin** near the real one — two nearby point-coincidences between the same two bodies
  condition the rotation so badly that Gauss-Seidel needed hundreds of sweeps in benign
  poses, and an unreachable drag exhausted `maxCleanupSweeps`, letting Phase B misreport
  reachable riders/locks as breaks (the "drag pulls the rod off its rail" bug); the
  angular projection killed both.
  **Grounded bodies** (`fixedBodies` + `fixedGroups`, rebuilt per solve by `buildFixed`):
  every `grounded` body — expanded to whole groups, with the group's locked free joints
  becoming immovable too — is an **immovable fixed host** in `bodyHostAt` / `railInfo` and
  skipped by `projectGrounds`; like ground anchors they are sacred, so an unreachable
  pin/slider onto one is disabled and reported as a break.
  **Scoped solves** (`SolveFreeze`, optional last param of `solve`): extra bodies / free
  joints held immovable for one call (frozen free joints join the `grounded` set; frozen
  bodies join `fixedBodies`) — the engine behind draw mode's rigid (Shift) drag. Unlike
  grounding, freezing takes the frozen part's constraints **out of scope**: when a freeze
  is active, `eachUnit` skips any unit none of whose participants can move
  (`jointImmovable`: pin with both ends immovable, ground on a fixed body, immovable rider
  on an immovable rail) so a still-open pin inside the frozen world is neither counted in
  residuals nor reported as a break — this solve couldn't change it, and counting it would
  drive Phase B into disabling constraints the drag isn't touching. Without a freeze,
  behaviour is exactly as before (immovable-vs-immovable conflicts still report).
  See "Solver notes" below.
- **sketch.ts** — the **sketch solver** (draw-mode only; the sim solver is untouched).
  Gauss-Seidel projection like solver.ts, but the variables are *shape*: the world
  positions of body **control vertices** (`v:body:index`) and **joints** (`j:id`) — a
  joint coincident with one of its body's control vertices maps onto the vertex variable
  (the node↔joint link), so constraints on it reshape the body. Each constraint /
  driving dimension becomes a projection item (coincident → midpoint; point-on-line
  coincident → zero the signed perpendicular distance to the **infinite** line — the point
  moves along the normal, the line translates toward it, rank-weighted like a point+line
  driving dimension with target 0; H/V → average the
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
  target > 0, rejects a dim with both ends instance-owned — those are *pose* dims,
  pose.ts territory, and `buildSystem` excludes them from the shape system entirely;
  picks **uniform scale** — both refs owned by one body, no other driving dim
  touches it, every sketch constraint touching it is fully internal — else the node
  solve; commits the driving flag only on success), `tryAddConstraint` (add + solve,
  remove again on conflict), `autoConstrainBody` (H/V inference within `AUTO_HV_TOL` =
  5°, each constraint solved in as it's added). The h/v, point+line and line+line
  dimension items honour the dim's held **`side`** (the error is signed toward the drawn
  relative direction, so an overshooting drag reads as a large error back — the pair can
  never flip through each other; absent side falls back to the current sign, the old
  behaviour).
  The variables also include **guideline defining points** (`g:id:a` / `g:id:b`, applied
  back via `moveGuidePoint`), and every variable carries a **mobility rank** — 0
  construction (guide points), 1 geometry (vertices, joints), 2 drag-anchored,
  3 **component-instance geometry** (immovable: its shape belongs to the definition, so
  it outranks even the drag — dragging against it yields) — with each
  pairwise projection weighted so corrections flow entirely to the **lowest** rank (equal
  ranks split evenly, the old symmetric behaviour): guide constraints move only free guide
  points, never geometry, and a drag is never tugged back by its constraints.
  `solveSketch(scene, anchors?)` takes the drag-pinned variable keys, built in main via
  the exported `anchorVarsForBody` / `anchorVarsForJoint` / `anchorVarsForGuide` /
  `anchorVarForGuidePoint` / `anchorVarForVertex` helpers; main falls back to a
  symmetric re-solve whenever the anchored solve is infeasible, so live drags can never
  leave a constraint visibly broken.
- **pose.ts** — **pose-level driving dimensions**: draw-mode dims whose BOTH ends live on
  component-instance geometry (`isPoseDim`). They drive the *pose* of rigid parts, never
  shape. `applyDimensionValue(scene, id, target)` is the router main uses for every
  dimension-value commit (pose dims here, everything else to `applyDrivingDimension`).
  `applyPoseDimension`: snapshot → `setMeasurementDriving` (its rigid-unit backstop
  rejects same-body/chassis pairs) → for a same-instance dim `poseSolveIntra` re-poses
  the internal mechanism with the sim solver (driver on one end's ref point, target
  recomputed along the current direction each round; a `SolveFreeze` holds everything
  outside the instance + the other end's rigid unit; tries the refB side, then refA) →
  `enforcePoseDims` settles every pose dim together → `solveSketch` lets free geometry
  follow → any failure restores the snapshot and returns the conflicts (reject
  semantics, red flash). `enforcePoseDims(scene, anchoredInstances?)` is the live
  enforcement: Gauss-Seidel rounds of **closed-form rigid translations** (`poseCorrection`
  mirrors `measureInfo`'s value semantics per kind/axis and honours the held `side`;
  `moveInstance` applies them) — a side is movable unless its instance is grounded or
  drag-anchored; same-instance dims are only verified here (translation can't fix them);
  residuals come back as `SketchBreak`s and render violated. `poseDimError` is the
  side-aware residual (a flipped pose at the right absolute distance reads as
  value + target, never satisfied). main calls `enforcePoseDims` at the top of
  `solveSketchLive` (with `draggedInstanceIds()` anchored — dragging a component pulls
  its dimensioned partners along) and re-asserts pose dims on def-context exit
  (`exitComponent`), after the cascade snapped instance poses back to the defs.
  exploration; not yet imported by the app — call `formatReport(analyzeScene(scene), scene)`
  from a console/debug hook). Builds a constraint graph (bodies + free joints as nodes; pins +
  slider-rider couplings as edges), finds kinematic islands (union-find), and per island
  reports: Grübler-Kutzbach DOF (pins/grounds remove 2 DOF, a **weld** — rigid pin — 3, a
  slider rider — point-on-line — 1, an orientation-locked rider 2), cyclomatic loop count
  (grounds modeled as edges to a virtual world node so
  loops-through-ground are counted), BFS propagation order from anchors, back-edges, and a
  Tarjan **bridge / biconnected-component decomposition** classifying bodies into loop cores
  (must be solved together) vs propagatable tree branches, with articulation bodies joining
  blocks. Labels are `#id`-based (bodies have no user-facing name).
- **dxf.ts** — minimal **ASCII DXF reader** for the drag-and-drop import (no dependencies).
  `parseDxf(text)` reads `$INSUNITS` from the HEADER (reported as mm-per-drawing-unit, null
  when absent/unitless/unknown) and the ENTITIES section: LWPOLYLINE / POLYLINE+VERTEX
  (closed → loops; vertex bulges sampled — bulge = tan(θ/4), positive = CCW — at ≤ 7.5° per
  segment), CIRCLE (48-gon), LINE and ARC (sampled, always CCW start→end); open paths are
  **chained end-to-end** (either direction, extent-relative tolerance) into closed loops.
  Returns `{ loops, unitToMm, skippedPaths, skippedEntities }` in raw DXF coordinates (y-up;
  the importer in main flips/scales/translates). Each loop is a **`DxfLoop`**: the sampled
  `pts`, plus **`fillet`** — a reconstruction of the loop as a sparse control polygon +
  per-corner radii (`reconstructFillets`: a closed polyline's bulge arc sweeping < 180° and
  **tangent to the straight segments on both sides** (±0.01 rad) is replaced by the corner
  where those segments meet, carrying the arc's radius; non-fillet arcs stay sampled inline;
  null when no arc converts) — and **`circle`** (`{ c, r }` when the loop is a CIRCLE
  entity, importable as a parametric disk). `nestLoops(loops)` groups sampled loops even-odd
  style into `{ outer, holes }` solids (parent = smallest enclosing loop; an island inside a
  hole is a new solid). `loopSignedArea` exported for winding normalization. Binary DXF and
  non-DXF input throw a friendly error.
- **view.ts** — camera transform `screen = world * scale + (tx, ty)`; `screenToWorld`,
  `worldToScreen`, cursor-anchored `zoomAt` (scale clamped to MIN_SCALE..MAX_SCALE = 0.05..20).
- **renderer.ts** — **construction guidelines** (draw mode only) draw right after the grid:
  muted dash-dot lines (`GUIDE_COLOR` grey) clipped to the viewport via `drawGuideLine`
  (the segment is centred on the view, so the line always spans it), with dots on the two
  defining points; the selected guide uses theme ink; `guideDraft` previews the infinite
  line from the first placed point to the (element-snapped) cursor. Also: joints involved in any `ConstraintBreak.joints` are painted red (fill +
  stroke + slight size bump) so the stuck points stand out alongside the existing red dotted
  break lines; and joints in `containmentErrors: Set<number>` (draw mode — attached joints
  stranded outside their body's outline) are painted the same error red but with a
  **dashed** ring, so the two red states read differently. Draws under the camera transform in world space: world-locked grid
  (spacing = `gridStep`, drawn only when `gridVisible`),
  bodies (selected/hovered highlighted; a body's **hole loops** are added as subpaths and
  filled with the **even-odd rule**, so cut-outs show what's behind them, with the body
  outline stroked on the hole rims too), rails as bounded segments with end-caps — each
  **orientation-locked rider** additionally badged by a green rail-aligned **carriage
  rectangle** (`drawCarriage`, drawn with the rail so it exists in both modes),
  ground symbols (per ground constraint, plus one at the **centroid of every grounded
  body** — each member of a grounded group carries the flag, so each shows its own),
  joints (color-coded: blue = pinned, yellow = grounded, green = slider rider; a **welded**
  joint — on a `rigid` pin — keeps the pin blue but swaps the hollow revolute centre for a
  **square** centre plus a blue square outline;
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
  reset per label, so text stays legible at any zoom; distances 1-decimal **with the
  scene's working-unit suffix** (e.g. `120.5 mm`), angles with `°`).
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
  pill border (`measureText(info, paren)`); a **violated** driving dimension
  (`MeasureInfo.violated` — off its target, or on the flipped side of its held direction)
  renders in the error red persistently until re-applied; `flash: Set<number>` paints the
  constraints / dimensions a rejected sketch edit named in the error red for a moment.
  Selection kind `"sketch"` highlights a badge in ink.
  **Multi-selection & groups**: `multiSelected: { bodies, joints } | null` highlights every
  member like a normal selection (bodies get the ink outline, free joints the selected
  ring); `marquee: { a, b } | null` draws the in-progress box selection (dashed rect +
  translucent fill); each **permanent group** draws a faint dashed **convex hull** around
  its members — but **only while the group is selected** (any member in `multiSelected`;
  never in sim, where `multiSelected` is null).
- **main.ts** — canvas/DPI setup, toolbar wiring (the toolbar is **icon buttons** — SVG glyphs with
  tooltips; wiring is by id/`data-*`/class, never button text), mode/tool state (tools are one-shot +
  have keyboard shortcuts), select-mode selection / move / delete / vertex-edit, the camera,
  pointer + key handling, persistence (save/load/autosave) plus a **snapshot undo/redo history**
  (`pushHistory`/`undo`/`redo`; `markDirty` records a step + autosaves), and the
  requestAnimationFrame render/solve loop. `timedSolve` captures the solver's `ConstraintBreak`s
  into `solveBreaks`; `updateSimError` shows/hides the red **"Assembly impossible"** banner
  (`#sim-error`), and the breaks are passed to the renderer in sim mode (cleared on leaving sim).
  **Containment watch**: every draw-mode frame recomputes `containmentErrors` from
  `scene.jointsOutsideBody()` (cleared in sim), passes it to the renderer, and refreshes the
  hint when the count changes — `updateHint` prepends `containmentWarning()` ("⚠ N joints lie
  outside their bodies (red) — drag them back inside, or fix the body / component shape.")
  to the normal mode/tool hint while any exist.
  The **Joint** tool auto-attaches a placed node to a slider when it lands on a rail/rail-node;
  the **body-from-joints** draft turns a bare slider-rail click into a grid-snapped rider joint
  (tracked in `jointDraftCreated`, removed if the draft is aborted). Also a `timedSolve` debug log.
  **Snap containment fallback**: both the Joint tool and body-from-joints joint minting hit-test
  against the raw click point but place at the snapped point — if snapping would land outside a
  body being attached to, they fall back to the exact click point (inside by hit-test), so a
  joint is never created outside its body. The `#sim-error` banner lives inside a
  **`#canvas-wrap`** container (index.html / style.css) wrapping the canvas, so it overlays the
  canvas top-center *below* the toolbar rather than on top of it.
  - **Hole tool** (`U`, draw mode; spans many clicks like the body tool, disarming in
    `finishHole`): `handleHoleClick` — the first click sets `holeDraftBodyId` via `bodyAt`
    (instance bodies alert + disarm), later clicks push containment-checked vertices onto
    `holeDraft`, and closing (first-vertex click / dblclick / Enter) calls
    `scene.addBodyHole` + selects the body. The draft is passed to the renderer through
    the same `draftBody` input the body tool uses (dashed polyline to the cursor);
    `resetTransient` clears it, so Esc aborts.
  - **Actuator / motor tools** (`A` / `M`, draw mode, one-shot): **Linear actuator** is a single click
    on a slider rail → calls `addLinearActuator` (snapped to the click point). **Motor** is two clicks
    — first joint becomes the pivot (tracked in `motorPivotDraft`, highlighted via `activeJoints`),
    second joint on the **same body** becomes the crank pin (mismatched second click restarts the
    draft at the new joint). Both tools select the resulting element so the inline properties panel
    appears right away.
  - **Slider tool** (`S`, draw mode, one-shot): on an existing **rider** → toggles its
    orientation lock (`setSliderRiderLocked`); on another joint near a rail (not a rail
    endpoint) → attaches it as a locked rider; on a bare rail → mints a joint at the click
    projected onto the rail — attached to the topmost body under the cursor excluding the
    rail's own body (`bodiesAt` is topmost-first), else free — and attaches it locked
    (`attachSliderRider(id, jid, true)`). Selects the resulting joint. The rail tool is
    `K` (renamed from "Slider", which took the `S` key with it; tool id `"rail"`,
    selection kind `"rail"`, renderer input `railDraft`).
  - **Weld tool** (`W`, draw mode, one-shot): on an existing **joint** → toggles the
    rigidity of every pin it participates in (`pinsOfJoint` + `setPinRigid`; a mixed set
    goes all-rigid first, all-rigid goes back to revolute — mirrors the Slider tool's
    lock toggle); otherwise, where **≥ 2 bodies overlap** → mints a joint in each (same
    grid-snap + containment fallback as the Joint tool) and welds them together with
    rigid pins (`addPin(a, b, true)`). A single body / empty space does nothing (a lone
    joint has nothing to weld to). Selects the resulting joint.
  - **Guideline tool** (`L`, draw mode; the actuator's shortcut moved to `A`): two clicks
    through `guidePlacementAt` — exactly on a picked point element (joint / body corner /
    guide point; recorded and turned into an **auto-coincident** via `tryAddConstraint`),
    projected onto a picked rail / body edge, else `snap`. Selection kind `"guide"`;
    `LeftDrag` kinds `"guide"` (whole-line translate, anchor = point `a`) and
    `"guidePoint"` (endpoint re-aim with element snapping, own guide excluded from every
    pick). `snap(p, excludeGuide?)` prefers guidelines over the grid (projection; two
    near guides → their intersection). `dragAnchorVars()` maps the active drag / rotate to
    sketch anchor keys; `solveSketchLive` runs the anchored solve and **falls back to a
    symmetric solve when it's infeasible** (constraints win over the drag), plus a settle
    solve on mouseup. `measureRefAt` / `constraintPointRefAt` / `constraintLineRefAt` pick
    guide points and lines (draw mode only for measure — guides are invisible in sim).
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
    line commits on the first click, everything else on the second. Coincident picks
    accept a point *or* a line on either click (point preferred when both are under the
    cursor; once a line is picked the other pick must be a point). `commitConstraint` →
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
    `#canvas-wrap`). Enter (or blur) commits: a number → `applyDimensionValue` (the
    pose.ts router: pose dims re-pose rigid parts, everything else goes to the sketch's
    `applyDrivingDimension`; rejected edits flash red, nothing moves); an **empty
    value** → back to a driven reference
    (`clearMeasurementDriving`). Esc cancels. A global keydown guard ignores canvas
    shortcuts while any input/select has focus (so Delete/tool letters don't fire while
    typing — this also fixed a latent bug with the actuator speed fields).
  - **Sketch-aware dragging** (`solveSketchLive`): draw-mode select drags (vertex / body /
    joint) and rotate-tool drags re-solve the sketch after every move when any constraint
    or driving dimension exists — constraints hold while the dragged geometry follows,
    CAD-style. A solve that can't converge mid-drag is skipped (the next one re-tightens).
    `solveSketchLive` first runs `enforcePoseDims(scene, draggedInstanceIds())`, so
    dragging a component pulls its pose-dimensioned partner components along (the dragged
    instances anchored); `exitComponent` re-asserts pose dims after a definition cascade.
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
  - **Multi-selection** (`multiSel: { bodies, joints } | null`, draw mode, mutually exclusive
    with the single `selection`): **Ctrl/Cmd+click** toggles the body / free joint under the
    cursor (`toggleMultiAt`, seeded from the single selection; a grouped body toggles its
    whole group); **box select** (`boxSelect` state → `applyBoxSelect`) starts on a
    left-drag from empty space (Ctrl+drag = additive) and selects bodies **fully inside**
    the rectangle plus free joints inside it — a non-moved press stays a plain
    click-to-deselect. `setMulti` commits a selection: expands permanent groups
    (**selection-atomic** — `handleSelectClick` on a grouped body selects the whole group),
    prunes dead ids, and collapses a single ungrouped body / free joint back to a normal
    single selection. Clicking any member drags the whole set (`LeftDrag` kind `"multi"`:
    same delta via `moveBody`/`moveJoint` per member; snap anchor = nearest
    centroid/corner/joint to the grab). Delete removes every member; Esc / plain click
    elsewhere clears. **Ctrl/Cmd+G** → `toggleGroupSelection()`: ≥ 2 selected bodies →
    `groupSelection()` (`scene.addGroup`, merging groups touched) unless the selection
    already is exactly one group → `ungroupSelection()` (a single selected grouped body
    also ungroups). Plain **G** always arms the Ground tool. Renderer inputs
    `multiSelected` + `marquee`.
  - **Rigid (Shift) drag** (`LeftDrag` kind `"rigid"`, draw/select mode): Shift+mousedown
    selects what's under the cursor (`handleSelectClick`, unless the grab lands on the
    multi-selection) and `startRigidDrag` builds the movable set — the multi-selection, or
    the single body / joint's body (always whole groups), or a lone free joint — a `Driver`
    at the grab (a joint of the set, else a body-frame point like the sim body grab), and a
    `SolveFreeze` naming **everything else** (all other bodies + free joints). Mousemove
    only aims the driver (a grabbed joint's target grid-snaps; a body point follows the
    cursor exactly); the **frame loop** runs `timedSolve("rigidDrag", …, freeze)` each
    frame while the drag is live, and mouseup runs one final solve before `markDirty`.
    Sim-like, so `solveSketchLive` is **not** called; vertex-handle grabs win over Shift
    (reshape), and Shift+hover shows the sim `grab` cursor. `timedSolve` gained the
    optional `freeze` pass-through parameter. Breaks stay invisible in draw mode (the
    render input already gates them to sim).
  - **Ground tool on bodies**: a Ground-tool click with no joint under the cursor toggles
    `scene.toggleBodyGround` on the body there — grounding/ungrounding it, or its whole
    permanent group. Joint grounding is unchanged and takes pick priority.
  - **Edit utilities**: `copySelection`/`pasteAt` (clipboard is a `SelectionClip` held in
    `main` — a multi-selection copies via `extractSelection` with everything internal to it,
    incl. cross-body pins and group membership; a single body via `extractBody`; paste lands
    at the cursor, grid-snapped, and re-selects the copy via `setMulti`),
    `mirrorSelection("h"|"v")` (multi-selection / group → `scene.mirrorBodies` about the
    combined bbox centre; single body → `mirrorBody` about its centroid), and a **rotate**
    tool. Rotate is a persistent mode (not one-shot): `startRotate` picks the pivot — a
    control node of the singly-selected body if grabbed; the **centre of the combined bbox**
    when the grabbed body is part of the multi-selection (or of a permanent group, which
    gets auto-selected); else the body's centroid — and a `rotateDrag`
    (`bodyIds`/`jointIds`) tracks the pointer's swing about it — accumulated/unwrapped so it
    survives ±π, with the first body's absolute angle snapped to 45°
    (`snapAngle`/`ROTATE_SNAP_TOL`) and only the incremental delta applied per move:
    `scene.rotateBody` per body about the shared pivot + selected free joints orbiting it —
    a rigid rotation of the whole selection. Arming rotate keeps an existing single *or*
    multi selection. Copy/paste are also on `Ctrl/Cmd+C`/`V`; rotate on `R`.
  - **Z-order** (`reorderSelection("back" | "front")`, draw mode): sends the selected body /
    multi-selection (whole groups always) to the back or front of the drawing order via
    `scene.reorderBodies`; `markDirty` only when the order actually changed. Wired to the
    `#send-back-btn` / `#bring-front-btn` icon buttons in the edit toolbar group and to
    **PageDown / PageUp** (`[`/`]` were taken by corner radius).
  - **Component editing contexts** (`editPath: number[]` — def ids, outermost first; empty =
    the root assembly): the live `Scene` always holds the **innermost context**; `rootData`
    stashes the root snapshot while a def is open, ancestor defs live in `scene.components`.
    **Every `markDirty` syncs**: `syncComponentContext()` stores the live context into its
    def, runs `cascadeComponentChange` through the def DAG, and refreshes `rootData` — so
    saves/autosaves/undo always see a consistent document and instances follow the def live.
    `canonicalData()` returns the ROOT document (+ components); **history entries carry the
    edit path** (`{snap, path}`), and `setDocument(doc, path)` re-enters the recorded context
    on undo/redo (also used by load, which always opens at the root). `enterComponent(defId)`
    (double-click an instance body, or ✎ in the panel; re-entering the top def is a no-op) /
    `exitComponent(levels)` (breadcrumb clicks; **Esc with nothing armed/selected** exits one
    level); per-context camera saved/restored; entering fits the view. **Component browser**
    (`#comp-panel`, toggled by `#comp-panel-btn`): rename inline, **＋** arms `pendingInsert`
    (next canvas click instantiates at the snapped point, centered via `componentCenter`;
    cycle-guarded against `componentUses`; **refused with an alert for a still-empty def**),
    **✎** edits, **×** deletes (refused while instances exist anywhere or the def is on
    `editPath`). `#make-comp-btn` packs the selection (`makeComponentFromSelection`;
    refuses selections touching other instances; with exactly one instance selected forks
    it; **with nothing selected creates an empty component** via `createEmptyComponent`
    and enters it for editing straight away).
    **Instance interaction**: selection-atomic via `setMulti` (expands instances + groups —
    including group `jointIds` — to a fixpoint; a lone instance body stays multi-selected so
    it never grows edit handles); clicking an instance joint/body selects the whole instance;
    Delete removes whole instances (`removeInstance`; deleting an instance-owned slider alone
    is refused); copy/paste carries instances as **placements** (`clipboard = { clip,
    instances: {defId, t}[], center }`) and pastes new instances; mirror refuses instance
    material (v1); Ctrl+G refuses instance material; Ground tool on an instance body grounds
    its **chassis group**; Shift-rigid-drag of a grabbed member moves the whole instance
    sim-style (internal mechanism articulates). The breadcrumb bar (`#crumb-bar`) overlays
    the canvas top-left. The `#component-group` toolbar group hides in sim, like the others.
  - **Theme** (`#theme-btn`): a dark/light toggle that sets `data-theme` on `<html>` (CSS vars drive
    the chrome) and passes the matching `DARK_THEME`/`LIGHT_THEME` palette to the renderer; the
    choice persists in `localStorage` (`disjointed:theme`, separate from scene autosave).
  - **Body colour** (`#body-color`): a colour input that does double duty — with a body selected it
    recolours that body (live, `markDirty`); with nothing selected it sets `defaultBodyColor`, the
    colour given to newly drawn / built bodies (paste keeps the source colour instead). The swatch
    is synced to the selection each frame by `syncColorPicker` (change-detected so it never clobbers
    the picker mid-drag). The `#color-group` hides in sim mode like the tool/edit groups.
  - **Grid / snapping** (session-only state, not persisted): `gridVisible`, `snapEnabled`,
    `gridStep` (clamped 1–200 via `parseGridSize`, **decimals allowed** — the input uses
    `step="any"` and the value is no longer rounded; number input + a preset `<select>`). `snap(p)`
    rounds a world point to the nearest grid intersection when enabled (identity otherwise) and is
    applied to placements (free/attached joints, freehand vertices) and to drags. Drags snap an
    **anchor** in absolute terms via a `grabOffset` captured at mousedown: a vertex reshape snaps
    the grabbed control vertex; a whole-body move snaps whichever of the centroid / control
    vertices is nearest the grab point (`bodyDragAnchor`), stored as a fixed `anchorOffset` from
    the centroid (`dragAnchorWorld` reconstructs its live position).
  - **Working units + DXF import**: the `#unit-select` dropdown writes `scene.unit`
    (markDirty → saved/undoable) and re-syncs from the scene each frame (`syncUnitSelect`,
    change-detected — load/undo can change the unit). Canvas `dragover`/`drop` handlers:
    a dropped `.dxf` → `importDxfFile(file, eventWorld(e))`, a `.json` → `loadFromFile`,
    anything else alerts. `importDxfFile` parses (`parseDxf`), converts by
    `unitToMm / UNIT_TO_MM[scene.unit]` (factor 1 when the file is unitless), **flips y**
    (fillet controls / radii and circles transform alongside the sampled pts), centres the
    combined bbox at the (snapped) drop point, nests loops (`nestLoops` on the sampled pts,
    with a pts→`DxfLoop` map to recover each loop's arc metadata), and creates one body per
    solid, coloured `defaultBodyColor`: an outer **circle** → a 1-point offset disk body; an
    outer with a **fillet reconstruction** → `addBody(control, 0, "fillet", holes, radii)`
    (winding normalized with control + radii reversed together); else the sampled outline as
    before. Holes go through the same map: a circle cut-out → a parametric **disk hole**, a
    reconstructed one → `{ control, radii }`, else the sampled loop (winding normalized).
    The batch is selected via `setMulti` and marked dirty. No closed shapes → an explanatory alert; unchainable
    open paths → a post-import alert; unsupported entities → a console note.

### Interaction model
Draw-mode tools are **one-shot**: arming a tool (toolbar button or shortcut —
`B`/`U` hole/`J`/`W` weld/`C`/`G`/`K` rail/`S` slider/`L` guideline/`A` actuator/`M`) lets you
place one element, then it returns to **Select** mode. `Esc` aborts the current placement.
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
- **Hole** (`U` — `H` is the horizontal constraint) — cut a hole in a body: the **first click
  picks the body** (topmost under the cursor; empty space does nothing, a component-instance
  body alerts to edit the definition instead), later clicks add cut-out vertices (grid-snapped;
  a snap that would land outside the body falls back to the exact click point, a click outside
  is ignored), and clicking the **first vertex** / **double-click** / **Enter** closes the loop
  into an editable radius-0 hole (`addBodyHole`). The body is selected afterwards so the hole's
  vertex / radius handles show right away. No coincident snapping or auto-H/V on hole drafts
  (v1 scope; the body tool has both).
- **Joint** (`J`) — click inside a body to attach a joint; click where bodies overlap to drop
  a joint in each and pin them together; click **empty space** to place a free (body-less) joint.
  A node placed on a **slider rail (or rail node)** is auto-attached to that slider as a rider.
- **Weld** (`W`) — click where **bodies overlap** to weld them rigidly together at that point
  (a joint in each, joined with **rigid pins**: shared position AND locked relative angle — no
  relative motion at all; the angle is whatever you drew, re-captured on every sim entry /
  rigid drag like a slider's lock). Click an **existing pinned joint** to toggle its pin(s)
  weld ↔ revolute. Welded joints show a **square** centre instead of the revolute's hollow dot.
- **Connect** (`C`) — click a joint, then another joint on a *different* body to pin them, or a
  *slider rail* to attach the joint to it as a rider.
- **Ground** (`G`) — click a joint to lock its world position (grounding a free joint makes a
  body-less anchor; a body can still rotate about a grounded joint). **Clicking an
  already-grounded joint removes its ground** (toggle; duplicate grounds from earlier versions
  are all cleared). Exception: a free joint serving as a world-fixed slider-rail endpoint keeps
  its ground — the rail must stay anchored (`addSlider`'s invariant). With **no joint under
  the cursor**, clicking a **body** toggles grounding of the whole body — fixed position
  *and* rotation in sim — and, through a grouped body, of its **whole group** (any member
  grounded → all ungrounded, else all grounded).
- **Rail** (`K` — called "Slider" pre-v17) — click two joints on the *same body* (a rail that
  moves with it), or two *free joints* (a world-fixed track — they get grounded automatically),
  to create a rail (riders are attached later via Connect, the Joint tool, or the Slider tool).
  A free+body or cross-body pair restarts the draft.
- **Slider** (`S`) — click a **rail** to add a slider: an **orientation-locked rider** that
  travels along the rail while its body keeps its drawn angle relative to it (prismatic —
  a plain rider is a pin-in-slot). The click projects onto the rail; the joint attaches to
  the topmost body under the cursor (excluding the rail's own body), else it's free (the
  lock activates once the joint gains a body). Clicking an **existing rider** toggles its
  rotation lock; clicking a loose joint near a rail attaches it as a locked rider. The
  locked angle re-captures from the drawn pose on every sim entry / rigid drag.
- **Guideline** (`L`) — two clicks place an **infinite construction line**. Clicks land
  exactly on joints / body corners / other guides' points (with an auto-coincident), project
  onto rails / body edges, or grid/guide-snap. Select it to drag the whole line (angle kept)
  or either defining point (re-aims); Delete removes it with its constraints/measurements.
  With snap on, placements prefer guidelines (and guide intersections) over the grid.
  Constraints on guides move **only free guide points**, never geometry.
- **Linear actuator** (`A`) — click a *slider rail* to drop a **self-driving rider** on it (a free
  joint attached as a rider, plus a `linearActuator` constraint that drives it during animation).
  Off-animation the rider is a normal slider rider (draggable / pinnable). Default speed 0.5 Hz,
  default profile `triangle`.
- **Motor** (`M`) — click a joint to set the **pivot** (must be on a body), then another joint on
  the *same body* for the **crank pin** (a cross-body second click restarts the draft at that
  joint). Creates a `motor` constraint that spins the body during animation. Default speed 0.25 Hz.
- **Measure** (`D`, also available in sim mode — sim keeps its own set) — click two references
  (joint, body corner, slider rail, body edge — **hole corners/edges included** (v16) — or a
  point on a body), then click where the value
  should sit. Point+point: the label spot picks horizontal / vertical / direct. Point+line:
  perpendicular distance to the infinite line. Line+line: distance while parallel, angle
  otherwise (dynamic; the label's sector picks θ vs 180−θ). Values update live in sim.
- **Sketch constraints** (draw mode only): **Coincident** (`O`) — two points (joints, body
  corners — hole corners included — guide points), **or a point + a line** (body edge,
  slider rail, or guideline; either pick order) to hold the point on the **infinite**
  line. **Horizontal**
  (`H`) / **Vertical** (`V`) — one edge/rail/guideline, or two points. **Parallel** (`P`) /
  **Perpendicular** (`T`) — two lines (edges/rails/guidelines — hole edges included);
  **Equal length** (`E`) —
  two edges/rails (guidelines rejected — no length). The geometry solves to
  satisfy the constraint the moment it's placed; an unsatisfiable one is rejected (conflicting
  items flash red). Badges are click-to-select, Delete to remove.
- **Driving dimensions** (draw mode): double-click a dimension's value → type a number →
  Enter. The first driving dimension on an otherwise-unconstrained body scales it uniformly;
  further ones move only the involved nodes while every constraint and driving dimension
  holds. Driven values show in parentheses; clear the field to make a driving dimension a
  reference again. Unsatisfiable targets are rejected (revert + red flash).
  **On component instances** dimensions drive *poses*, never shape: between two different
  components one translates rigidly to the value (a grounded one never moves); between two
  mobile parts of one component the internal mechanism re-poses (pins intact); a dimension
  internal to one rigid piece (same body / chassis / welded — "already dimensioned at a
  deeper level") rejects. Dragging a component pulls its dimensioned partners along
  live; a dimension that can't hold (grounded partner, or a definition edit reset the
  poses) turns **error-red** until re-applied (double-click, Enter). Every driving
  dimension holds its **drawn relative direction** — a fast drag can never flip the two
  sides through each other; to re-side one, clear its value, move the part, re-enter it
  (direct unaligned distances have no side and rotate freely).

Select mode (default, no tool armed):
- **Sketch-aware dragging**: with any sketch constraints / driving dimensions present, every
  draw-mode drag (node, joint, body) and rotate re-solves the sketch live — the dragged
  geometry follows the cursor as far as the constraints allow and everything constrained to
  it comes along.
- Click a body, joint, or slider rail to **select** it (highlighted); **left-drag** moves the
  selected body or joint. An attached joint **can't leave its body**: dragging it past the edge
  clamps it to the outline (it slides along the edge). A joint sitting exactly on a control
  node (as in a joint-built body) is **stuck to that node** — dragging either one moves both,
  reshaping the body. A selected body shows **corner handles on the outer outline and every
  hole** — square handles move vertices (`moveBodyVertex`, carrying any joint stuck to that
  node) and **round radius handles** sit on each corner's fillet arc (drag along the bisector
  to set that corner's radius via `setBodyCornerRadius`; onto the vertex = sharp;
  double-click = back to the outline default; on an offset outline the handle rides the
  point's circle — a disk hole resizes by its rim). The nearest handle wins the grab.
  **`[` / `]`** decrease / increase the body-wide default radius (overridden corners keep
  their override).
  **Double-click** an edge of the selected body — outer or hole — to add a control node there
  (`insertBodyVertex`, grid-snapped), or a node to remove it (`removeBodyVertex`, kept ≥ 3 —
  ≥ 1 for a disk hole); double-clicking a **hole's last removable node deletes the whole
  hole** (`removeBodyHole`, refs cascading). Clicking on/near the
  selected body's control polygon keeps it selected (so an edge double-click isn't lost).
- **Multi-selection**: **Ctrl/Cmd+click** toggles bodies (and free joints) in and out of a
  multi-selection; **box select** — left-drag on empty space — selects every body fully
  inside the marquee plus the free joints inside it (Ctrl+drag adds to the selection).
  Dragging any selected element **moves the whole selection together**; Delete removes it
  all; a plain click elsewhere (or Esc) clears it. A single ungrouped body collapses back
  to a normal selection.
- **Permanent groups**: **Ctrl/Cmd+G** toggles grouping — with 2+ members multi-selected
  (bodies **and free joints**: joints become locked members that ride the group rigidly) it
  groups them permanently (grouping into an existing group merges); when the selection
  already is exactly one group (or a single grouped element) it ungroups. Groups are
  selection-atomic — clicking any member selects the whole group — and show a faint dashed
  hull **while selected**. In draw mode they move/rotate/mirror/copy as one; in **sim they
  are one rigid body** (see solver notes).
- **Components**: select bodies (+ free joints) and press **⊞** to pack them into a
  **component definition**, replaced in place by an **instance**. Instances are
  selection-atomic (dashed hull while selected), design-locked (no handles / sketch
  constraints; measurements allowed), drag/rotate/copy/delete as a unit (copies are new
  instances), and Shift-drag poses their internal mechanism sim-style. **Double-click an
  instance** (or ✎ in the component browser) to edit its definition in its own context —
  breadcrumbs at the top-left, every tool available, changes **cascade to all instances
  live**, Esc (idle) or a breadcrumb exits. Grounding inside a definition = "fixed to the
  component frame" (the rigid chassis); grounding an instance's body in the assembly fixes
  its chassis in the world. The **component browser** (toolbar toggle) renames / inserts /
  edits / deletes definitions. **⊞ with exactly one instance selected forks it** ("make
  unique"): the definition is copied into a new independent component and the selected
  instance re-pointed at the copy — nothing moves, the selection is kept, the browser opens
  on the new definition (`selectionInstance()` in main detects the pure single-instance
  selection; mixed selections still alert).
- **Rigid (Shift) drag**: hold **Shift** when starting a select-mode drag and the grabbed
  object — a body (with its whole group), the body a grabbed joint sits on, a lone free
  joint, or the whole multi-selection — moves **like in simulation** instead of being
  translated: grounds hold exactly (a body pivots about its grounded joint; ground anchors
  and grounded bodies never move), pins/sliders to the rest of the scene constrain the
  motion, and **everything not being dragged stays frozen**. An open dotted pin from the
  dragged object to the frozen world snaps closed as the drag pulls (assemble-on-drag).
  The poses you release at become the new drawn layout (one undo step). Sketch constraints
  are not applied during a rigid drag (sim-like); corner-handle reshaping wins over Shift.
- `Delete` removes the selection: a body takes its joints/constraints with it; a slider keeps
  its joints; a joint detaches from any rail and any constraints referencing it go; a
  measurement just disappears (measurements are also cascade-removed with their elements).
- A **measurement's value label** is the topmost pick: click to select it, drag to reposition
  (a point–point measurement re-derives h/v/direct from the new spot), Delete to remove —
  this works in **sim mode** too, without disturbing the mechanism underneath.
- **Edit utilities** (act on the selection — a single body, or a multi-selection / group):
  **Copy/Paste** (`Ctrl/Cmd+C`/`V`, **keyboard-only** — no toolbar buttons) duplicates the
  selection with its joints + every constraint internal to it — grounds, internal sliders,
  pins (including pins **between** the selected bodies), group membership, fully-internal
  sketch constraints and driving dimensions — **keeping colours**; the copy lands at the
  cursor (grid-snapped) and is selected. **Mirror H/V** (toolbar, grouped with **Rotate**)
  reflects a single body in place about its centroid, or a multi-selection / group about
  the centre of its combined bounding box (constraint refs are remapped, so mirrored
  geometry keeps its constraints on the right corners/edges). **Send to back / Bring to
  front** (toolbar buttons next to Mirror, or **PageDown / PageUp**) moves the selection to
  the bottom / top of the drawing order — clicks pick the topmost body, so a big imported
  reference body sent to the back stops covering the mechanism and stealing its clicks.

Rotate tool (`R`, draw mode — a mode, not one-shot):
- Drag a body to rotate it about its **centroid**; drag a **control node** of the already-selected
  body to rotate about that node. A body in the **multi-selection** (or a **permanent group**,
  which gets auto-selected) rotates the **whole selection rigidly** about the centre of its
  combined bounding box — selected free joints orbit with it. The (first) body's absolute angle
  snaps to the nearest 45° when within ~2°. Joints and ground anchors rotate rigidly with the
  body. Arming rotate keeps the current selection. Esc / another tool exits.

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
- **Impossible assemblies** are flagged, not faked: grounds (and grounded bodies) never move,
  the solvable parts still solve, and every unsatisfiable pin/slider draws a **red dotted line** between the two points
  that can't meet (pulled as close as the assembly allows), with a red **"Assembly impossible"**
  banner. The joints involved in each break are also drawn **red** (red fill + red ring + slight
  size bump). A connected impossible piece does not disturb the parts that can be solved. The
  sim-mode toolbar has an **Auto-pause** toggle (warning-triangle icon) that halts the animation
  once the assembly stays impossible for a few frames in a row — useful for stopping motors /
  actuators before they push past a physically unreachable configuration.

Navigation (both modes):
- **Mouse wheel** zooms toward the cursor (0.05×–200×).
- **Right-drag always pans** the view (anywhere). Moving elements is left-drag in select mode
  (above); there is no right-drag-to-move.
- **Fit to screen** (`F`, or the toolbar button next to Save): frames the whole mechanism
  (body outlines, joints, ground anchors) centered with a 60 px margin (`fitView` in main.ts);
  an empty scene recenters the world origin at 1×.
- **Tab** toggles draw ↔ simulate mode (preventDefault'd away from the browser's focus cycle;
  like every shortcut it's inert while an input field has focus).

Grid (toolbar grid group):
- **Grid** button toggles grid visibility; **Snap** button toggles snap-to-grid; a number field
  (1–200, decimals allowed, with a preset dropdown) sets the spacing — both the drawn grid and
  the snap increment.
  Visibility and snapping are independent. See main.ts "Grid / snapping" above for what snaps.

Undo / redo (draw mode):
- `Ctrl/Cmd+Z` undo, `Ctrl/Cmd+Shift+Z` / `Ctrl/Cmd+Y` redo. Snapshot-based: every mutation
  records a (deduped) JSON snapshot of the drawn layout; undo/redo restore snapshots without
  recording new steps. History is seeded on startup and capped at `HISTORY_LIMIT` (100).

Working units (toolbar grid group):
- A **unit dropdown** (mm / cm / m / in, default mm) declares what one world unit means.
  Declarative only — changing it moves nothing; distance measurements show the suffix, and
  DXF import converts into it. Saved with the file (v12) and undoable.

Import (drag-and-drop onto the canvas):
- Drop a **`.dxf`** to import its closed outlines as bodies at the drop point (at true scale
  via `$INSUNITS` + the working unit; a unitless file is assumed to be in working units).
  Loops nested inside others become **holes** (one plate with cut-outs = one body); loose
  lines/arcs are chained into loops; the imported batch arrives multi-selected. Open paths
  that can't close are skipped with a notice. **Rounded corners stay parametric**: a corner
  arc tangent to its neighbouring straight segments imports as a sharp control corner with a
  per-corner fillet radius (editable with the radius handles), and circles — standalone or
  cut-outs — import as 1-point offset disks (resize by the rim handle). Non-tangent or
  ≥ 180° arcs stay sampled points. Drop a **`.json`** to load it as a scene.
  `dxf import test.dxf` in the repo root is a hand-written sample (plate with two round
  cut-outs, circle, LINE-triangle, ARC+LINE D-shape, mm units); `Maze Base.dxf` is a real
  CAD export whose plate + 4 holes reconstruct fully (4 control corners each).

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
  joint or a free joint. An **orientation-locked rider** (v17) adds one more projection —
  since v18 a **direct angular one**: the rider's rigid unit rotates against the rail's unit
  about the rider point until the *wrapped* relative angle matches the value captured from
  the drawn pose (baselines live in the solver module and are reset by main whenever the
  drawn layout changes). **Welds** (`rigid` pins, v18) use the same angular projection about
  the weld point, locking two bodies' relative angle at the drawn value on top of the pin's
  coincidence. Angular errors are wrapped to [-π, π] with a unique zero, so neither a lock
  nor a weld can settle into a flipped pose; each is its own break-reportable unit (negated
  rider id / negated pin id).
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
- **slider-locks.ts** — orientation-locked riders (v17, 12 checks): a locked rider's body
  never rotates on a fixed track (and stays on the rail) while the same drag rotates it once
  unlocked; the baseline locks a tilted **drawn** angle; a locked body keeps its angle
  *relative to* a moving (pivoting) rail; save/load keeps `locked` and legacy sliders load
  with none; deleting the rider sheds rider + lock; copy/paste recreates the lock on the
  pasted rider; an impossible lock+ground reports a break with the ground unmoved.
- **welds.ts** — rigid pins / welds (v18, 17 checks): a welded pair keeps its drawn relative
  angle under dragging while still moving as one (and the same pin toggled back to revolute
  folds); the baseline re-captures a newly drawn relative angle; a weld to a free joint acts
  as a plain pin; persistence (rigid survives save/load, non-boolean values sanitized away);
  copy/paste carries the flag; an unreachable weld between two fixed bodies reports a break
  naming both joints with neither body moved; component expansion carries the flag and
  un-welding the definition cascades to the instance; the analyzer counts a weld as 3 DOF
  removed vs a pin's 2; and the **drag-yield regression** — unreachable drags (far
  perpendicular, past the end-stop, both) on a rod locked to a fixed track and welded to a
  second body never report breaks, never pull the rod off the rail line, and never rotate
  it off its lock (the flip check).
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
  **Per-corner radii (v15)**: array-radius `filletPolygon` rounds only named corners (others
  exactly sharp); per-point `roundedConvexBody` margins; `setBodyCornerRadius` override +
  clear semantics (all-null array dropped); insert/remove shifting the override with its
  corner; mirror reversing it onto the renumbered corner; scale scaling it; save/load and
  copy/paste round-trips. **Editable holes (v16)**: holes become shapes (+ derived loops), a
  1-point offset hole derives as a true circle, hole edits change mass but not the outer
  outline, hole vertex/edge refs resolve + track edits, insert/remove remap only same-hole
  refs, per-corner hole radii, mirror remapping hole refs within their own outline,
  copy/paste + save/load round-trips, legacy baked holes loading as radius-0 editable holes,
  and `removeBodyHole` cascading refs + shifting later holes' refs. **`addBodyHole`** (the
  hole tool's model op): returned index, derived loop, mass subtraction, joints staying
  anchored through the cut's centroid shift, < 3-point rejection, a 1-point offset spec
  cutting a disk appended after existing holes, and world → local conversion on a
  moved + rotated body.
- **edit-utils.ts** — `rotateBody` (90° about the centroid carries the joint + ground anchor; a
  pivot node stays fixed), `mirrorBody` (joint reflected, centroid + area preserved), and
  copy/paste (`extractBody`/`insertBody`: independent offset duplicate, joints/grounds/sliders
  duplicated with fresh ids, cross-body pins dropped, **source colour preserved**; internal
  **sketch constraints** — H/V edges, joint↔vertex coincident, H on an internal rail — captured
  and recreated on the new elements with resolvable refs, cross-body constraints excluded;
  the internal **driving dimension** copied with its target + translated label while a driven
  reference dimension is left behind and the original stays untouched). Also **z-order
  reordering** (`reorderBodies`: the newest body picked on top; pick falling through after a
  send-to-back; relative order kept; no-ops reported — already at the target end, or every
  body moved at once; a grouped member dragging its whole group; order surviving save/load).
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
  index remapping; serialize/load v8 round-trip + pre-v8 files. **Hole geometry (v16)**:
  a horizontal constraint on a hole edge solves by reshaping the hole (outer outline
  untouched), and joint ↔ hole-corner coincident solves to coincidence. **Point-on-line
  coincident**: validation (line-first pick order normalized to point-as-refA, line+line
  rejected, structural line endpoints — edge vertices incl. the wrapping one, a rail's own
  joint, a guide's own point — rejected); a joint beyond the segment lands on the
  *infinite* edge line with a purely perpendicular correction split evenly with the edge;
  save/load round-trip + re-solve; a free joint solved onto a rail line; a joint onto a
  guideline moving only the guide (mobility ranks).
- **measurements.ts** — axis selection from label placement (h / v / direct zones);
  point-point values + dimension-line geometry + preview parity; label move re-deriving the
  axis; values and label positions tracking moving geometry; point-line using the infinite
  line (foot beyond the rail end + extension line); line-line parallel distance, the dynamic
  flip to angle when a rail tilts, and the label sector picking the obtuse angle; vertex/edge
  index remapping across `insertBodyVertex`/`removeBodyVertex`; a bodyPoint ref riding its
  body's frame; cascade removal (joint / slider / body); serialize/load round-trip (modes,
  values, deep copy, `nextId`) and pre-v7 files loading with no measurements.
- **groups.ts** — permanent groups end-to-end (59 checks): group management (`addGroup`
  needs 2+ bodies, overlap-merge, `groupOf`, `ungroup`, prune on `removeBody`, dissolve
  < 2); serialize/load v9 round-trip + legacy files with no groups; **rigid sim behaviour**
  (driving any member carries the rest with the relative pose exact to ~1e-13; a ground on
  one member anchors the whole group, which pivots about it; an intra-group pin is inert —
  never a break, never moves anything; an external pin holds while the group is towed; a
  rider on a grouped body slides its whole group along a fixed track); **selection
  copy/paste** (`extractSelection`/`insertSelection`: cross-body pin kept, group + free
  joint carried, colours preserved, centre lands at the drop point, spacing/offsets exact);
  **multi mirror** (`mirrorBodies` reflects centroids + free joints about the bbox centre,
  cross-body pin stays coincident); **shared-pivot rotation** rigidity; and the **mirror
  ref-remap fix** (vertex/edge refs follow their corners/edges through a mirror; a sketch
  solve right after mirroring — or after mirroring + dragging the group — is a no-op).
- **group-joints.ts** — free joints as group members (v14): membership management (accepts
  free joints, rejects attached ones, merges through a shared joint, joints-only groups,
  prune on removeJoint, < 2-member dissolve); rigid sim behaviour (a locked joint rides a
  towed group exactly; a pin to it tows the whole group; a **grounded** locked joint makes
  the group pivot about the anchor; a grounded member body fixes the joint); a rail of two
  locked joints is a group-riding track (addSlider doesn't auto-ground them; riders slide
  and follow the towed group); serialize/load round-trip + legacy files; copy/paste carries
  group joints.
- **components.ts** — hierarchical components end-to-end (89 checks): creation from a
  selection (def carries sketch/measurements/grounded flags; assembly carries neither;
  instance replaces the originals exactly in place; sketch constraints on instance geometry
  rejected); joint-ground conversion (synthesized chassis anchor + pin; grounding the
  instance then dragging the arm pivots about the converted anchor exactly); multiple
  instances (rotated placement; chassis rigid under tow; other instances untouched);
  **cascade** (recolor/reshape/add/remove def bodies → every instance reconciles, surviving
  scene ids kept, placement — translation *and* rotation — preserved, new def material
  arrives in the instance's frame); **containment warning** (`jointsOutsideBody`: clean
  scene reports none, an edge-clamped joint is not flagged, a def-shrink cascade flags the
  stranded assembly joint *without moving it*, and dragging it back inside clears the
  flag); **pose snap** (a posed body / free joint snaps back to
  the def layout on re-expansion, instance placement kept — the def is the reference);
  **empty components** (`createEmptyComponent` stores a blank def; instantiation refused
  until it has content; fillable with bodies or with *only* an instance of another def —
  the wrapper appears in the def DAG); **nested defs** (editing the inner def ripples through
  the outer def to the root; componentUses DAG direction); powered constraints inside a
  component (slider + actuator expand with no world grounds; anchors drive the rider along
  the chassis track); removal / dissolve / removeComponent guards; **fork / make-unique**
  (`makeInstanceUnique`: new def, re-pointed instance, sibling untouched, drift-0 no-op
  reconcile, two-way edit independence after the fork, name dedup, nested defs stay shared
  in the DAG); serialize/load v14 round-trip, idempotent re-expansion after load,
  `reexpandData` on stored snapshots, and pre-v14 files loading with no components;
  **dimensions vs instance shape** (a dim internal to an instance can't drive and a
  rejected edit leaves the instance untouched; a mixed instance↔free dim drives by moving
  only the free side, and later sketch solves keep holding it without ever moving the
  instance).
- **pose-dims.ts** — pose-level driving dimensions (36 checks): an inter-instance dim
  drives by rigid translation (refB's side by preference, shapes untouched); a grounded
  partner never moves (the other side does), both grounded rejects untouched with the old
  target kept; h-axis dims translate along their axis only; live follow (`enforcePoseDims`
  with the dragged instance anchored — the partner follows, the drag stays where the user
  put it); conflicting pose dims reject; pose dims are excluded from the sketch (a plain
  `solveSketch` is a byte-identical no-op); an intra-instance dim re-poses the internal
  mechanism (target hit, held side fixed, arm pivoted, pin gap ~1e-9, shape + definition
  byte-identical) while a dim rigid at a deeper level (same instance body) rejects;
  **held-side flip protection** (a violent one-frame overshoot drag keeps the drawn
  relative direction — pose path and sketch path both; the side is captured on drive and
  dropped on clear); mixed dims still route to the sketch.
- **grounded-bodies.ts** — grounded bodies/groups (20 checks): `toggleBodyGround` toggle
  semantics (lone body on/off, whole-group grounding/ungrounding through any member); a
  grounded body immovable under drag; a body pinned to one pivots about the pin while the
  grounded body stays put; a rail on a grounded body is a fixed track (rider slides, clamps
  at the end-stops); an unreachable pin between two grounded bodies reports a break with
  neither moved; serialize/load v10 round-trip, pre-v10 files loading ungrounded, and
  copy/paste carrying the flag per body.
- **freeze-drag.ts** — scoped solves (`SolveFreeze`, the rigid-drag engine, 16 checks):
  frozen bodies / free joints never move; the movable selection pivots about a pin to a
  frozen body, keeps its ground anchors exactly, and a rider stays on a frozen rail; an
  open pin from the selection to the frozen world snaps closed (assemble-on-drag); an open
  pin **between two frozen bodies** is neither closed nor reported as a break — and the
  same pin without a freeze is closed by a normal solve (the skip is freeze-only).
- **guides.ts** — construction guidelines end-to-end (57 checks): CRUD + moves (whole-line
  translate preserves the angle; endpoint re-aim; collapse-onto-twin refused), hit tests
  (infinite line, defining points), serialize/load v11 round-trip + pre-v11 files + id
  continuity + deep copy; `guidePoint`/`guideLine` refs resolving; H solved on a free
  guide; coincident guide↔joint (guide follows, joint fixed) surviving a move + re-solve;
  `equal` with a guide rejected; parallel guide↔edge; constraint/measurement cascade on
  `removeGuide`; point↔guide measurement using the infinite line; **drag anchoring** (a
  rigid two-body move with a joint-bound guide leaves the bodies exactly rigid, guide
  following in full); **mobility ranks** (perpendicular between a joint-bound guide and a
  half-bound guide moves only the free point — body, joints and the bound guide untouched;
  H on a fully-bound guide rejected with the scene untouched; H on a half-bound guide
  solved by the free point); and the **drag fallback** (an infeasible anchored drag step
  followed by the symmetric solve restores V, keeps the drag's feasible component, joint
  untouched).
- **dxf.ts (scripts)** — the DXF importer + units + holes end-to-end: closed
  LWPOLYLINE / old-style POLYLINE+VERTEX loops; bulge sampling exactly on the arc's circle
  (positive bulge CCW); **fillet reconstruction** (a rounded rectangle reconstructs to its 4
  sharp corners with radius 5, regenerating the exact shape; a half-circle arc and
  non-tangent arcs don't reconstruct; arc-free loops report null; CIRCLE reports centre +
  radius); LINE chaining (unordered + reversed segments) and ARC+LINE chaining;
  CIRCLE sampling + CCW winding; `$INSUNITS` (mm / inches / unitless / absent); dangling
  open paths skipped + unsupported entities counted; non-DXF input rejected; the working
  unit surviving serialize → load with a pre-v12 mm default; `nestLoops` (plate + 4
  cut-outs → one solid, island-inside-a-hole → its own solid, disjoint loops separate); and
  bodies with holes: net mass, composite centroid, reduced inertia, `pointInBody` true
  inside a hole (joint placeable at a hole centre), holes round-tripping through save/load,
  mirror reflecting the hole, copy/paste carrying it, scale scaling it, rotate riding it.

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
- **Mirror left sketch constraints / measurements pointing at the wrong corners.**
  `mirrorBody` reverses the control polygon (to keep the winding valid for the rounding),
  but vertex/edge refs are index-based — after the reversal "vertex 0" / "edge 2" silently
  named a *different* corner/edge, so an H constraint could land on a vertical edge. The
  mirrored geometry looked right, but the next sketch solve (e.g. dragging the mirrored
  group) snapped everything to satisfy the mis-aimed constraints. Fixed in `mirrorBody`
  itself (covers single-body and group mirrors): vertex `i → n−1−i`, edge `i → n−2−i`
  (wrapping), and `bodyPoint` refs re-baked onto the reflected material. Reflection itself
  preserves every constraint kind, so after the remap everything stays satisfied.
- **A group drifted apart when dragged with a guide coincident to one member's joints.**
  Two causes: the sketch solver split every coincident correction 50/50 (the constrained
  body handed half of each increment to the guide), and the multi-drag's snap anchor was a
  *stored position* updated to the commanded target — unlike every other drag it never
  re-read the live scene, so the loss was never corrected and accumulated to half the drag
  distance. Fixed by **drag anchoring** (the dragged geometry is passed to `solveSketch`
  as pinned; free elements absorb the whole correction) and by making the multi-drag
  anchor a live-read landmark spec (`{bodyId, offset}` / `{jointId}`).
- **Guide constraints moved geometry.** A perpendicular between two guidelines rotated
  both lines' defining points — including ones coincident-bound to joints, dragging the
  joints along. Fixed with the **mobility ranks** (construction < geometry < dragged):
  guide constraints now move only free guide points; ones that would need geometry to
  move are rejected.
- **H/V on a guideline silently did nothing.** `handleConstraintClick`'s line test only
  knew rails and edges, so a guideline pick was stored as a bogus first *point* pick and
  the eventual commit failed model validation without feedback. Fixed by adding
  `guideLine` to that classification (parallel/perpendicular were unaffected — they use
  the line picker directly).
- **A drag could visibly break a guide constraint until release.** Dragging the free
  point of a joint-bound vertical guide sideways made the anchored live solve infeasible
  (neither the dragged point nor the joint may move), and the old policy just *skipped*
  failed mid-drag solves — the line went angled and only snapped back on mouseup. Fixed:
  `solveSketchLive` immediately falls back to a symmetric solve when the anchored one
  can't converge, so constraints hold every frame and the drag only moves things along
  the free directions.
- **Dragging a welded assembly could pull a slider off its rail (false breaks).** The first
  weld implementation (same-day fix, v18) enforced the locked angle as a phantom **second
  pin** right next to the real one — the worst conditioning for Gauss-Seidel (rotation is
  barely observable from two nearby points), so convergence took hundreds of sweeps in
  benign poses and an unreachable drag (grabbing the welded body at a lever and pulling
  perpendicular / past the end-stop) exhausted `maxCleanupSweeps`. The still-large residual
  sent the solver into Phase B, which **disabled perfectly reachable rider/lock units and
  reported them as breaks** — the rod visibly popped off its rail. The drag-yield design
  (driver yields via post-drive convergence) was sound; the weld just converged too slowly
  to fit the budget. Fixed by solving the weld's angle **directly**: rotate the two rigid
  units about the weld point by the wrapped angular error split by inverse inertia
  (`solveWeld` + `unitAngHost`); worst case dropped from 1000+ sweeps (failed) to ~270
  (converged), benign case 435 → 27. Regression-tested in `scripts/welds.ts`.
- **A violently dragged slider carriage could settle 180° flipped.** Pre-existing v17 flaw
  surfaced by the weld work: the lock's phantom-on-the-infinite-rail-line error is
  `(dl/2)·sin(Δ)` — zero at Δ = π too, so a wild drag could tunnel the locked body through
  a half-turn and the flipped pose reported **zero error** (no break, no snap-back). Fixed
  alongside the weld: the lock now measures the **wrapped** relative angle (unique zero)
  and enforces it with the same direct angular projection, so a flip reads as a large
  error and is pulled back.
- **A "driven" dimension on a component instance still reshaped it.** `applyDrivingDimension`
  moved geometry (scale or node solve) *before* the instance-ownership guard could run —
  the guard lived only inside `setMeasurementDriving`, whose `false` return was silently
  ignored. The instance body scaled while the label kept its driven parentheses, and the
  change silently diverged from the definition (the next cascade would discard it). Fixed
  by making instance geometry **rank-immovable** in the sketch solver and checking
  ownership up front — which then grew into the full pose-dimension feature (dims with
  both ends on instances rejected at first, later made to drive *poses* instead).
- **Fast drags could flip a driving dimension's direction.** Every dimension correction
  re-derived its sign from the *current* geometry (`sign(B − A)`), so a one-frame drag
  overshooting past the partner flipped the sign and the solver satisfied the constraint
  on the wrong side — two rectangles held apart by a short v distance could pass through
  each other. Fixed with the held **`Measurement.side`** (captured from the drawn
  geometry when the dim starts driving): corrections are now signed toward the drawn
  side in both solvers, side-aware residuals treat a flipped-but-equal-distance pose as
  unsatisfied, and the violated render flags it. Regression-tested with one-frame
  overshoot drags in `scripts/pose-dims.ts`.

## Backlog / next steps (not yet built)
- **Sketch-constraint follow-ups**: driving *angle* dimensions (v1 is distances only);
  an auto-constraint on/off toggle in the toolbar; auto-coincident while *dragging* (today
  it's inferred only while drawing); constraint badges could use hover feedback.
  (`mirrorBody` now remaps constraint/measurement refs; copy/paste carries the
  fully-internal ones + driving dimensions.)
- **Pose-dimension follow-ups**: sketch *constraints* (H/V, coincident, parallel…)
  between instance and free geometry are still blanket-rejected — the same
  anchor-the-instance treatment the dims got would apply naturally; a **direct**
  (unaligned) point-point dim has no side, so a one-frame jump straight through the
  partner can still tunnel (a circle constraint — continuous rotation around the partner
  is legitimate); pose dims between instances aren't carried by copy/paste (refs to
  re-instantiated material don't remap); h/v pose dims measure **world** axes — rotating
  a component changes what they hold (the user's accepted caveat, same as H/V constraints
  referencing component-local axes making no sense after rotation).
- Measurement follow-ups: copy/paste doesn't carry *driven* (reference) measurements
  (consistent with actuators/motors; driving dimensions are carried, as constraints).
- **Guideline follow-ups**: ~~no point-**on**-line constraint kind yet~~ (done — Coincident
  now takes point + line, so a guide point can be constrained to stay on an edge/rail —
  though placement-time snaps still don't record it automatically); guides don't join
  multi-selections / copy / mirror / rotate; the violet hover highlight for a guide-line
  ref covers only the defining segment (picking works anywhere on the infinite line);
  guides are deliberately invisible + unpickable in sim. If "level two joints through a
  guide" is ever wanted, constrain the joints directly (H on the point pair) — guide
  constraints never move geometry by design.
- **Group follow-ups**: ~~groups contain bodies only~~ (done in v14 — free joints can be
  locked members); reshaping a grouped body's outline in draw mode is still per-body
  (groups constrain sim + move-together, not draw-mode shape editing).
- **Component follow-ups (v1 scope decisions)**: no per-instance **mirroring** or scaling
  (mirror inside the definition instead); no explicit "ports" (every instance joint is
  connectable); editing an instance's actuator/motor speed at the assembly level works but
  is overwritten by the next definition cascade (per-instance overrides would need an
  override record); assembly measurements referencing an instance's *slider rail* survive
  re-expansion (constraints are reconciled in place) but die if the def removes the slider;
  a definition with nothing grounded has no chassis — its instance placement is derived
  from the first body (fine, by design decision); the component browser is minimal (no
  thumbnails / drag-to-place).
- **Hole follow-ups**: ~~editable hole vertices, measurements / sketch constraints on hole
  geometry, corner radius on holes~~ (all done in v16), ~~a UI to create holes~~ (done —
  the hole tool). Remaining: a hole-aware picking option (today clicking a cut-out
  deliberately still hits the body); hole-tool parity with the body tool (vertices snapping
  onto existing joints/corners with auto-coincident, auto-H/V on near-axis hole edges);
  no containment check of hole *edges* (only vertices are kept inside the body, so a
  cut-out hugging a concave outer outline can poke outside — harmless: even-odd rendering,
  still editable).
- **DXF follow-ups**: SPLINE sampling, INSERT/block instancing, ellipses; DXF *export*.
- More joint types as needed. (~~A prismatic / no-rotation slide joint~~ — done in v17: the
  Slider tool's orientation-locked riders. ~~A rigid / weld joint~~ — done in v18: the Weld
  tool's rigid pins.) **Weld follow-ups**: the Connect tool doesn't create welds directly
  (pin with `C`, then toggle with a `W` click on the joint); the weld toggle on an
  instance-owned pin isn't blocked (like the slider-lock toggle — a definition cascade
  overwrites it); a weld between a body and a *group-locked* free joint stays inert (the
  joint has a group orientation, but baselines only capture body pairs — weld a body of the
  group instead). **Drag-yield caveat**: "a drag never breaks constraints" is enforced by
  convergence, not by construction — a pathologically slow-converging mechanism could still
  exhaust `maxCleanupSweeps` under an unreachable drag and false-break like the phantom-pair
  weld did; if it ever recurs, the candidate hardening is to skip Phase B entirely for pure
  mouse-driver solves (breaks would then come only from sim-entry / animation solves).
  **Slider follow-ups**: the locked angle is always
  re-captured from the drawn pose (by design — no stored angle parameter); if an explicit,
  persistent relative angle is ever wanted, store a per-lock phase in the constraint instead
  of the solver-side baseline map. A group-locked *free* rider has no orientation, so its
  lock is inert (attach the slider to a body of the group instead).
- ~~Joint containment covers placement + drags only: reshaping a body can strand a joint
  outside the new outline with no feedback~~ (resolved — stranded joints are now **flagged**,
  deliberately not auto-clamped: `jointsOutsideBody()` + red dashed ring + hint warning; the
  user drags the joint back in or fixes the shape / component definition).
- **Actuator / motor follow-ups**: editing speed/profile while in sim (selection clears on mode
  change today, so the inline panel only appears in draw). ~~Copy/paste carrying actuators +
  motors~~ (done in v14 — `SelectionClip` carries both, needed so components keep their
  powered constraints).
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
