# Disjointed

A simple web app for creating and simulating **2D planar mechanisms** — bodies (with editable,
round-able shapes) coupled by joints (pins, grounds, sliders) that you can then drag and watch move.

> Status: working. Draw a mechanism (freehand or from joints), edit it, switch to simulate, and
> drag any part of it to drive it. The solver and shape/edit logic are covered by headless tests.

## Concepts

- **Body** — a rigid shape with rounded-able corners. It's defined by an editable **control
  polygon** plus a **corner radius**; the outline you see is derived from those, so you can
  reshape it (drag corners) or round it any time. A body can carry **holes** (inner cut-outs,
  e.g. from a DXF import): they render hollow and are subtracted from the body's mass and
  inertia, and they mirror / rotate / scale / copy with the body. Holes don't restrict where
  joints can go — a joint can sit anywhere inside the outer outline, including dead-centre
  of a shaft hole.
- **Joint** — a point. Either **attached** to a body (rigid with it) or **free** (a body-less
  point). A free joint can be grounded to make an anchor without needing a body.
- **Constraints**
  - **Pin** — connect two joints on different bodies; they share a position but can rotate freely.
  - **Ground** — lock a joint's position; its body can still rotate about it. A **body (or a
    whole group)** can also be grounded, which fixes it completely in simulation — position
    and rotation.
  - **Slider** — a rail defined by two joints. Joints attached to it (riders) slide along the
    segment **between** those two joints, with hard stops at each end. The rail can be two joints
    on one body (it moves with the body, coupling two bodies) or **two free joints**, which makes
    a track fixed in world space — the two free joints get grounded automatically.
  - **Linear actuator** — a special rider on a slider that travels back and forth along the rail
    automatically when animation runs. Configurable speed (Hz) and motion profile (triangle = constant
    speed, sine = smooth ease). Off-animation it behaves like any other rider — draggable, pinnable.
  - **Motor** — a pivot + crank pair on one body; the crank pin orbits the pivot at a configurable
    angular speed (Hz) when animation runs. Off-animation the body behaves normally.
- **Group** — a permanent set of bodies that acts as **one object**: selected, moved, rotated,
  mirrored and copied together in Draw mode, and simulated as a **single rigid body** (nothing
  inside a group can move relative to the rest). Made and dissolved with `Ctrl+G` (a toggle).
- **Guideline** — an **infinite construction line** through two points (Draw mode only; it
  never takes part in simulation). Placement, dragging and drawing snap onto guidelines in
  preference to the grid, and guidelines participate in sketch constraints and measurements
  like any other line — see *Construction guidelines* below.

## Usage

There are two modes, switched from the toolbar or by pressing **Tab**. The toolbar uses
**icon buttons** — hover any of
them for a tooltip naming the tool and its shortcut. A **theme toggle** (sun / moon) at the right
switches between **dark and light** themes; your choice is remembered across sessions.

### Draw
Tools are **one-shot**: pick a tool (or press its shortcut), place one element, and you return
to **Select** mode. Press **Esc** to abort the current placement.

| Tool | Shortcut | Action |
| --- | --- | --- |
| **Body** | `B` | **Empty space:** click to add vertices, then close (first vertex / double-click / Enter). **On a joint:** build a body *from joints* — click joints to outline, click a placed joint to finish, then move the cursor out to set the thickness and click. Joints on other bodies (and *grounded* free joints) get a coincident pinned joint so they stay put — including a **rider that belongs to another body**, which pins the two bodies together at that point so they ride the slider as one. A **slider rail node**, or a click on a bare **slider rail**, instead makes the new body its own **rider** of that slider. **Clicking on another body mid-draft** mints a fresh joint on that body and adds it to the outline (the two bodies get pinned together at that point); **clicking empty space mid-draft** mints a free joint and adds it to the outline (absorbed into the new body). |
| **Joint** | `J` | Click inside a body to attach a joint; click where bodies overlap to drop one in each (pinned together); click **empty space** for a free, body-less joint. Drop a joint on a **slider rail (or rail node)** and it's automatically attached to that slider as a rider. An attached joint always lands **inside** its body — if grid snapping would push it outside, it's placed at the exact click point instead. |
| **Connect** | `C` | Click a joint, then another joint on a different body to **pin** them — or click a **slider rail** to attach the joint to it as a rider. |
| **Ground** | `G` | Click a joint to lock its position (it can still rotate). Ground a free joint to make an anchor. Click a **body** (away from its joints) to ground the whole body — fixed position *and* rotation in Simulate; a grouped body grounds its **whole group**. Click an **already-grounded** joint or body to remove the ground (a free joint anchoring a world-fixed slider rail keeps its ground — the track must stay anchored). |
| **Slider** | `S` | Click two joints on the **same body** (a moving rail), or **two free joints** (a world-fixed track — they get grounded automatically), to create a slider rail. Attach riders later with Connect. |
| **Guideline** | `L` | Click **two points** to place an **infinite construction line**. Each click lands exactly on a joint / body corner / another guide's point (with an automatic **coincident** constraint), projects onto a rail or body edge, or snaps to the grid. See *Construction guidelines* below. |
| **Rotate** | `R` | A mode (not one-shot): **drag a body** to rotate it about its centroid, or **drag a control node** of the already-selected body to rotate about that node. A **multi-selection or group** rotates as one about the centre of its bounding box. The angle **snaps to 45°** when it's within ~2° of a multiple. Joints and ground anchors turn with the body. |
| **Linear actuator** | `A` | Click a **slider rail** to drop a self-driving rider on it. In Simulate mode with animation running, the rider travels back and forth along the rail. Off-animation it's just a normal rider you can pin to anything. |
| **Motor** | `M` | Click a joint to set the **pivot**, then another joint **on the same body** for the **crank pin**. In Simulate mode with animation running, the crank pin orbits the pivot at the motor's speed. |
| **Measure** | `D` | Click **two references**, then click where the value should sit. A reference is a **point** (a joint, a body corner node, a guide point, or any point inside a body) or a **line** (a slider rail, a body edge, or a guideline). Works in **both modes** — see *Measurements* below. |
| **Coincident** | `O` | Click **two points** (joints, body corners, or guide points) to make them share a position. |
| **Horizontal** / **Vertical** | `H` / `V` | Click a **body edge, slider rail or guideline** (one click), or **two points**, to make it horizontal / vertical. |
| **Parallel** / **Perpendicular** / **Equal** | `P` / `T` / `E` | Click **two lines** (body edges, slider rails or guidelines) to constrain their directions — or, for Equal, their lengths (Equal doesn't take guidelines: an infinite line has no length). |

**Select mode** (no tool active, the default): click a body, joint, or slider rail to select it.
**Drag** the selection to move it. An attached joint **can't leave its body** — dragging it past
the edge makes it slide along the outline instead. A joint sitting exactly on one of its body's
corner nodes (as in a body **built from joints**) is **stuck to that node**: dragging either one
moves both, reshaping the body around it. A selected body shows **corner handles** — drag one to
reshape it, and press **`[` / `]`** to decrease / increase its corner radius (this is how you
round a freehand polygon: draw it, select it, press `]`). With a body selected you can also edit its
outline by **double-click**: double-click an **edge** to add a node there (snapped to the grid
when Snap is on), or double-click a **node** to remove it (kept to a minimum of 3). Press
**Delete** to remove the selection: a body takes its joints and constraints with it; a slider rail
leaves its joints; a joint detaches from any rail.

**Rigid drag (Shift).** Hold **Shift** when you start a drag and the grabbed object moves the way
it would in Simulate mode instead of being translated: it behaves as a rigid body, **grounds hold
exactly** (a body with a grounded joint pivots about it; ground anchors and grounded bodies never
move), and its pins and slider connections to the rest of the scene constrain the motion — while
**everything you're not dragging stays frozen in place**. It's like simulating just the selected
object (or group, or multi-selection). A still-open dotted pin from the dragged object to the rest
of the scene snaps closed as the drag starts pulling, so you can use it to assemble a mechanism
piece by piece. The pose you release at becomes the new drawn layout (one undo step). Note: rigid
drags ignore sketch constraints (like simulation does), so a rigid rotation can leave an H/V
constraint unsatisfied until the next sketch edit re-solves it.

**Multi-select & groups.** **Ctrl/Cmd+click** bodies (or free joints) to build a multi-selection,
or **drag a box** on empty space to select everything fully inside it (Ctrl+drag adds). A
multi-selection **moves together** — drag any member — and Delete removes it all.
**`Ctrl/Cmd+G` toggles grouping**: with two or more bodies selected it makes them a **permanent
group** (grouping something already grouped merges); pressed on a selection that already *is* a
group, it dissolves it. A group behaves as **one object**: clicking any member selects the whole
group (shown with a dashed outline while selected), it drags, rotates, mirrors and copies as a
unit — and in **Simulate mode it moves as a single rigid body**.

**Editing utilities** (on the selection — a single body, or a multi-selection / group):
- **Copy / Paste** (`Ctrl/Cmd+C` / `Ctrl/Cmd+V`, keyboard only) — duplicate the selection with
  its joints and every constraint internal to it: grounds, internal sliders, **pins between the
  selected bodies**, **group membership**, and its **sketch constraints and driving dimensions**.
  The copy **keeps the original colours**, lands at the cursor (grid-snapped when Snap is on) and
  becomes the selection — pasting a group gives you a new, working group. Anything reaching
  outside the selection (e.g. a pin to an uncopied body) isn't reproduced.
- **Mirror H / V** — reflect a selected body left↔right or top↔bottom in place about its centroid;
  a multi-selection / group reflects about the centre of its combined bounding box. Constraints
  and dimensions follow their corners/edges through the flip. Grouped in the toolbar next to
  **Rotate**.

**Measurements.** The Measure tool (`D`) works in **both modes**, and each mode keeps its own
set of measurements. What gets measured follows from the two references you pick:

- **Two points** — where you place the value picks the dimension, CAD-style: above/below the
  pair → **horizontal** distance, beside it → **vertical**, in the diagonal zones → **direct**.
- **A point and a line** — the perpendicular distance to the (infinite) line.
- **Two lines** — the **distance** while they're parallel, the **angle** otherwise. This is
  re-evaluated live, so a line pair can flip between distance and angle mid-simulation, and
  the side you place the label on picks θ vs 180°−θ.

References anchor to the elements themselves, so in **Simulate mode the values update live**
as the mechanism moves — measure a stroke length by dimensioning two joints, or a transmission
angle by dimensioning two rails. Click a value pill to select it, drag it to reposition
(a point–point dimension re-derives h/v/direct), press **Delete** to remove it — all of this
works in both modes. Measurements are saved with the mechanism.

**Sketch constraints & driving dimensions** (draw mode). Draw mode works like a CAD sketch:

- The six **constraint tools** (table above) relate points and lines — the geometry moves to
  satisfy a constraint the moment you place it, and a constraint that *can't* be satisfied is
  rejected (the conflicting items flash red, nothing moves). Each constraint shows a small
  violet **badge** (◎ H V ∥ ⊥ =) beside its element — faded until you **hover the element**
  (or the badge): click to select, **Delete** to remove. A toolbar toggle next to the
  constraint tools **shows/hides all badges** (constraints keep working while hidden), and a
  matching toggle next to Measure shows/hides **all measurements** — in both modes.
- **Driving dimensions**: **double-click** a dimension's value, type a number, press Enter.
  The **first** driving dimension on an otherwise-unconstrained body **scales the whole body
  uniformly** (same shape, new size); further dimensions move **only the involved nodes**
  while every constraint and driving dimension holds. A **driven** (reference) dimension
  shows its value **in parentheses**; a driving one shows it plain. Clear the field to turn a
  driving dimension back into a reference. Impossible targets are rejected with a red flash.
- **Sketch-aware dragging**: with constraints or driving dimensions present, dragging a node,
  joint, or body (and rotating) **re-solves the sketch live** — what you drag follows the
  cursor as far as the constraints allow, and everything constrained to it comes along.
- **Auto-constraints while drawing**: a freehand edge drawn within ~5° of horizontal /
  vertical snaps straight and gets the H/V constraint; a vertex clicked **on an existing
  joint or corner** lands exactly there and gets a coincident constraint.

**Construction guidelines** (Draw mode). The Guideline tool (`L`) places **infinite lines**
through two points — CAD-style scaffolding for laying out a mechanism:

- **Placement snaps to existing elements**: a click lands exactly on a joint, body corner or
  another guide's defining point (and records an automatic **coincident** constraint so the
  guide stays attached when that point later moves), projects onto a slider rail or body
  edge, or falls back to the grid.
- **Snapping prefers guidelines over the grid**: with Snap on, anything you place or drag
  lands *on* a nearby guideline (projected onto it) — and where two guidelines cross, on
  their **intersection**. Great for laying out joints along a line or at a crossing.
- **Editing**: click to select (its two defining points show as dots); drag the **line** to
  move it whole (angle preserved), drag a **defining point** to re-aim it, **Delete** to
  remove it (its constraints and measurements go with it).
- **Constraints on guidelines** (H / V / parallel / perpendicular / coincident, and
  measurements — including driving dimensions) follow one strict rule: they are satisfied
  by moving **only free guide points — never joints or body nodes**. A constraint that
  would need geometry to move is rejected with a red flash (e.g. Horizontal on a guide
  whose both points are bound to joints at different heights). Constraints hold **during**
  drags too: dragging the free point of a joint-bound vertical guide slides it vertically —
  it can't be pulled off-axis even momentarily.
- Guidelines are drawing aids only: they're invisible (and unpickable) in Simulate mode,
  never affect the simulation, and don't travel with copy/paste.

**Body colour.** A colour swatch in the toolbar sets the active colour: with **nothing selected**
it's the colour given to newly drawn bodies; with a **body selected** it shows that body's colour
and editing it recolours the body.

**Actuator / motor speed.** Select an actuator's rider, the slider it rides, or a motor's body
(or its pivot / crank joint) and a small inline panel appears in the toolbar with a speed field
(in Hz) — and, for linear actuators, a `/\` ↔ `~` profile toggle (triangle for constant-speed
end-to-end travel, sine for smooth ease in/out at the endstops).

Joints are color-coded: **blue** = pinned, **yellow** = grounded, **green** = slider rider;
rail-defining joints get a **green ring**, and a **loose free joint a dashed ring**. Once a free
joint is attached to a slider it's no longer loose, so it drops the dashed ring and shows as a
normal (green) rider. While drawing, a constraint whose endpoints don't yet touch is drawn as a
**dotted connector** so the link still reads as connected: **blue** between two pinned joints, and
**green** from a slider rider to the **middle of its rail**.

**Undo / redo:** `Ctrl/Cmd+Z` undoes, `Ctrl/Cmd+Shift+Z` (or `Ctrl/Cmd+Y`) redoes — covering edits to the drawn layout.

### Simulate
Drag any joint, or **any part of a body**, to drive the mechanism. The grabbed point follows the
cursor and every connected body moves with it. Structural constraints always win over the cursor —
a body with a grounded joint can only rotate about it, a **grounded body or group doesn't move at
all**, and the grabbed point walks to the nearest position it can actually reach. A **permanent group** moves as one rigid body: grab any member and
the whole group translates and rotates together (a ground on one member anchors them all). Your
drawn layout is preserved when you switch back to Draw.

**Animate (▶ button or `Space`).** With any linear actuators / motors in the scene, press the
run-animation button in the sim-mode toolbar (or Spacebar) to drive them all at their configured
speeds. Press again to pause. Pressing play **resumes from the current pose** — phases auto-fit so
the motion picks up smoothly from wherever you (or the previous animation) left things.

**Impossible assemblies.** Grounded joints and grounded bodies are sacred — they never move. If a mechanism can't be
assembled (a constraint can't be satisfied), the solver keeps every solvable part working and
flags only the genuinely impossible connections: each shows a **red dotted line** between the two
points that can't meet (pulled as close together as the rest of the assembly allows), the joints
involved are drawn **red**, and a red **"Assembly impossible"** banner appears. A
connected-but-impossible piece won't disturb the parts that *can* be solved.

**Auto-pause on impossible** (warning-triangle button in the sim-mode toolbar). Toggle it on to
have the animation halt automatically when the assembly can't be assembled — useful when running
a motor or actuator into an unreachable configuration. A short debounce filters single-frame
solver chatter, so it only fires once the impossibility persists for a few frames.

**Solver tuning** (advanced, sim-mode toolbar). Four small controls let you trade solve accuracy
against per-frame cost live: the animation iteration cap (`it`), the convergence-sweep cap (`cl`),
and the structural / break tolerances (`st` / `br`). The defaults are right for most mechanisms —
these exist for dialing in complex closed-loop scenes where the animation occasionally flags a
solvable assembly as impossible. The browser console logs rolling solve statistics while the
animation runs.

### Units & DXF import
A **unit dropdown** in the toolbar's grid group declares what one world unit means — **mm, cm,
m or in** (default mm). It's purely a declaration (changing it never moves geometry): distance
measurements show the unit, and imports convert into it. The choice is saved with the file.

**Drag-and-drop a `.dxf` file onto the canvas** to import its shapes as bodies at the drop
point, at true scale — the file's `$INSUNITS` is converted into your working unit (a unitless
file is assumed to already be in working units). Closed polylines (arc bulges included),
circles, and loose lines/arcs that chain into closed loops all import; a loop **inside**
another becomes a **hole** in it, so a plate with cut-outs arrives as *one* body. The imported
shapes land multi-selected, ready to move or group. Dropping a `.json` file loads it as a
scene, same as the Load button. (`dxf import test.dxf` in the repo is a small sample to try.)

### Grid & snapping
The toolbar's grid group controls a world-locked grid: **Grid** toggles its visibility, **Snap**
toggles snap-to-grid, and the number field (with a preset dropdown) sets the spacing — any value
from **1 to 200** world units, decimals included. With Snap on, new joints and freehand vertices land on the grid,
and dragging snaps too: a per-vertex reshape snaps the grabbed corner, while moving a whole body
snaps whichever is nearest the grab point — the body's centroid or one of its corners. Visibility
and snapping are independent (you can snap to a hidden grid).

### Navigate
- **Mouse wheel** — zoom toward the cursor (0.05× to 200×).
- **Right-drag** — pan the view (anywhere). To move a body or joint, select it and left-drag (see Select mode).
- **Fit to screen** (`F`, or the toolbar button) — frame the whole mechanism centered in the canvas.
- **Tab** — switch between Draw and Simulate mode.

### Save & load
- **Save** downloads your mechanism as a `.json` file; **Load** opens one back up (you can
  also drag-and-drop a `.json` onto the canvas).
- Your work is also auto-saved in the browser and restored automatically the next time you
  open the app.

## Development

Requires Node.js (built with Node 24).

```bash
npm install      # install dependencies
npm run dev      # start the dev server (opens the app)
npm run build    # type-check + production build into dist/
npm run preview  # preview the production build
npm test         # headless tests: solver, persistence, body building, shape editing, edit utilities, actuators / motors, measurements, sketch constraints, groups, grounded bodies, rigid-drag scoped solves, construction guidelines, DXF import / units / holes
```

## How it works

A small **iterative position-based solver** (Gauss-Seidel projection) satisfies the
constraints. Each constraint participant is reduced to a uniform "host" — a rigid body, a free
joint (a movable point), or a fixed world anchor — so pins, grounds, sliders and the mouse
driver all share one routine. The driver can pull either a joint or an arbitrary point fixed in a
body's frame, which is what lets you grab anywhere on a body to drive it. After driving, the solver keeps sweeping the structural
constraints until the worst error is below a tolerance (capped), so complex or closed-loop
mechanisms converge tightly instead of drifting. The driver is step-limited and yields to
structural constraints, keeping dragging stable even when you pull toward a point the mechanism
can't reach. Grounds are inviolable: a grounded joint is treated as a fixed world point by every
pin/slider/driver, so pinning to it can never drag the body it sits on. When an assembly can't be
solved, the solver disables only the genuinely unreachable pins/sliders (never a ground),
re-solves the rest, then pulls the disabled ones as close as the freedom allows and reports them
as breaks — so a connected impossible piece doesn't corrupt the parts that can be solved. Sliders
are prismatic constraints with end-stops; the rail is either a body (which moves) or a world-fixed
line built from two grounded free joints. Body outlines are
generated from a control polygon + corner radius (rounded corners via fillet or outward offset).
The fillet rounds convex and concave (reflex) corners correctly, and splits each edge between its
two corners so neighbouring fillets never overlap or fold — even on thin shapes at large radii.
A body's **holes** are baked loops riding its frame: drawn with an even-odd fill and subtracted
from the mass properties (net area, composite centroid, parallel-axis inertia), while picking and
joint containment deliberately use the outer outline only.
**DXF import** is a small dependency-free reader (`dxf.ts`): it samples arcs/bulges into
polyline points, chains loose segments into closed loops, nests loops even-odd style into
solids-with-holes, and converts units via the file's `$INSUNITS` and the document's working unit.
**Actuators and motors** are layered on top of the same solver: while animation runs, each
actuator/motor computes a world target for its joint(s) from its phase + speed, and the solver
takes those targets as additional "moving grounds" — sacred just like a normal ground, so
pins/sliders propagate the imposed motion through the whole assembly.
**Permanent groups** are rigid composites in the solver: a grouped body's "host" carries the
group's combined mass, centroid and inertia, and every impulse translates + rotates **all**
members about the combined centroid — so pins, sliders, grounds, drags and motors on any member
move the group as one body, and constraints *between* members of one group are inert.
**Grounded bodies** are the degenerate case: an immovable host (zero mass and inertia), sacred
like a ground anchor — grounding any member fixes its whole group.
**Rigid (Shift) drags** reuse the same solver with a per-call *freeze scope*: everything outside
the dragged selection is held immovable, and constraints living entirely inside the frozen part
are taken out of scope (neither solved nor reported), so dragging one link of a mechanism poses
it kinematically without disturbing — or being spuriously blocked by — the rest of the drawing.
**Measurements** store references to elements (a joint, a body node or edge, a rail — never raw
coordinates) and re-resolve them to world geometry every frame, which is why their values track
the running simulation for free.
**Sketch constraints** get their own solver (`sketch.ts`): the same Gauss-Seidel projection
idea, but over *shape* — the world positions of body corner nodes, joints and guideline
defining points — rather than rigid poses. After a converged solve, bodies rebuild from
their new control polygons; an unsatisfiable solve never touches the scene (edits are
rejected, not approximated). Every solver variable carries a **mobility rank** —
construction (guide points) < geometry (nodes, joints) < actively-dragged — and each
correction flows entirely to the more mobile side (equals split evenly). That one rule
gives the CAD feel: guide constraints move guides rather than geometry, dragged geometry
is never tugged back by its constraints (guides follow it exactly, so groups stay rigid),
and when a drag would need the constraints to give way, a symmetric re-solve runs the same
frame so the constraint visibly holds and the drag slides along the directions left free.

Source lives in [`src/`](src/): `geometry.ts`, `model.ts`, `solver.ts`, `sketch.ts`,
`dxf.ts` (DXF import), `view.ts` (camera),
`renderer.ts`, `main.ts`, plus `analyzer.ts` — a standalone topology diagnostic (kinematic
islands, degrees of freedom, loop / block decomposition) groundwork for future solver
optimizations. Tests live in [`scripts/`](scripts/).

## License

See [LICENSE](LICENSE).
