# Disjointed

A simple web app for creating and simulating **2D planar mechanisms** — bodies (with editable,
round-able shapes) coupled by joints (pins, grounds, sliders) that you can then drag and watch move.

> Status: working. Draw a mechanism (freehand or from joints), edit it, switch to simulate, and
> drag any part of it to drive it. The solver and shape/edit logic are covered by headless tests.

## Concepts

- **Body** — a rigid shape with rounded-able corners. It's defined by an editable **control
  polygon** plus a **corner radius**; the outline you see is derived from those, so you can
  reshape it (drag corners) or round it any time.
- **Joint** — a point. Either **attached** to a body (rigid with it) or **free** (a body-less
  point). A free joint can be grounded to make an anchor without needing a body.
- **Constraints**
  - **Pin** — connect two joints on different bodies; they share a position but can rotate freely.
  - **Ground** — lock a joint's position; its body can still rotate about it.
  - **Slider** — a rail defined by two joints. Joints attached to it (riders) slide along the
    segment **between** those two joints, with hard stops at each end. The rail can be two joints
    on one body (it moves with the body, coupling two bodies) or **two free joints**, which makes
    a track fixed in world space — the two free joints get grounded automatically.
  - **Linear actuator** — a special rider on a slider that travels back and forth along the rail
    automatically when animation runs. Configurable speed (Hz) and motion profile (triangle = constant
    speed, sine = smooth ease). Off-animation it behaves like any other rider — draggable, pinnable.
  - **Motor** — a pivot + crank pair on one body; the crank pin orbits the pivot at a configurable
    angular speed (Hz) when animation runs. Off-animation the body behaves normally.

## Usage

There are two modes, switched from the toolbar. The toolbar uses **icon buttons** — hover any of
them for a tooltip naming the tool and its shortcut. A **theme toggle** (sun / moon) at the right
switches between **dark and light** themes; your choice is remembered across sessions.

### Draw
Tools are **one-shot**: pick a tool (or press its shortcut), place one element, and you return
to **Select** mode. Press **Esc** to abort the current placement.

| Tool | Shortcut | Action |
| --- | --- | --- |
| **Body** | `B` | **Empty space:** click to add vertices, then close (first vertex / double-click / Enter). **On a joint:** build a body *from joints* — click joints to outline, click a placed joint to finish, then move the cursor out to set the thickness and click. Joints on other bodies (and *grounded* free joints) get a coincident pinned joint so they stay put — including a **rider that belongs to another body**, which pins the two bodies together at that point so they ride the slider as one. A **slider rail node**, or a click on a bare **slider rail**, instead makes the new body its own **rider** of that slider. **Clicking on another body mid-draft** mints a fresh joint on that body and adds it to the outline (the two bodies get pinned together at that point); **clicking empty space mid-draft** mints a free joint and adds it to the outline (absorbed into the new body). |
| **Joint** | `J` | Click inside a body to attach a joint; click where bodies overlap to drop one in each (pinned together); click **empty space** for a free, body-less joint. Drop a joint on a **slider rail (or rail node)** and it's automatically attached to that slider as a rider. |
| **Connect** | `C` | Click a joint, then another joint on a different body to **pin** them — or click a **slider rail** to attach the joint to it as a rider. |
| **Ground** | `G` | Click a joint to lock its position (it can still rotate). Ground a free joint to make an anchor. |
| **Slider** | `S` | Click two joints on the **same body** (a moving rail), or **two free joints** (a world-fixed track — they get grounded automatically), to create a slider rail. Attach riders later with Connect. |
| **Rotate** | `R` | A mode (not one-shot): **drag a body** to rotate it about its centroid, or **drag a control node** of the already-selected body to rotate about that node. The angle **snaps to 45°** when it's within ~2° of a multiple. Joints and ground anchors turn with the body. |
| **Linear actuator** | `L` | Click a **slider rail** to drop a self-driving rider on it. In Simulate mode with animation running, the rider travels back and forth along the rail. Off-animation it's just a normal rider you can pin to anything. |
| **Motor** | `M` | Click a joint to set the **pivot**, then another joint **on the same body** for the **crank pin**. In Simulate mode with animation running, the crank pin orbits the pivot at the motor's speed. |

**Select mode** (no tool active, the default): click a body, joint, or slider rail to select it.
**Drag** the selection to move it. A selected body shows **corner handles** — drag one to reshape
it, and press **`[` / `]`** to decrease / increase its corner radius (this is how you round a
freehand polygon: draw it, select it, press `]`). With a body selected you can also edit its
outline by **double-click**: double-click an **edge** to add a node there (snapped to the grid
when Snap is on), or double-click a **node** to remove it (kept to a minimum of 3). Press
**Delete** to remove the selection: a body takes its joints and constraints with it; a slider rail
leaves its joints; a joint detaches from any rail.

**Editing utilities** (on a selected body):
- **Copy / Paste** (`Ctrl/Cmd+C` / `Ctrl/Cmd+V`, keyboard only) — duplicate a body together with
  its joints and the constraints that belong only to it (grounds, fully-internal sliders). The copy
  **keeps the original's colour**, lands at the cursor (grid-snapped when Snap is on) and becomes
  the selection. Cross-body pins aren't reproduced.
- **Mirror H / V** — reflect the selected body (and its joints) left↔right or top↔bottom, in place
  about its centroid. Grouped in the toolbar next to **Rotate**.

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
a grounded body can only rotate about its ground point, and the grabbed point walks to the nearest
position it can actually reach. Your drawn layout is preserved when you switch back to Draw.

**Animate (▶ button or `Space`).** With any linear actuators / motors in the scene, press the
run-animation button in the sim-mode toolbar (or Spacebar) to drive them all at their configured
speeds. Press again to pause. Pressing play **resumes from the current pose** — phases auto-fit so
the motion picks up smoothly from wherever you (or the previous animation) left things.

**Impossible assemblies.** Grounded joints are sacred — they never move. If a mechanism can't be
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

### Grid & snapping
The toolbar's grid group controls a world-locked grid: **Grid** toggles its visibility, **Snap**
toggles snap-to-grid, and the number field (with a preset dropdown) sets the spacing — any value
from **1 to 200** world units. With Snap on, new joints and freehand vertices land on the grid,
and dragging snaps too: a per-vertex reshape snaps the grabbed corner, while moving a whole body
snaps whichever is nearest the grab point — the body's centroid or one of its corners. Visibility
and snapping are independent (you can snap to a hidden grid).

### Navigate
- **Mouse wheel** — zoom toward the cursor.
- **Right-drag** — pan the view (anywhere). To move a body or joint, select it and left-drag (see Select mode).

### Save & load
- **Save** downloads your mechanism as a `.json` file; **Load** opens one back up.
- Your work is also auto-saved in the browser and restored automatically the next time you
  open the app.

## Development

Requires Node.js (built with Node 24).

```bash
npm install      # install dependencies
npm run dev      # start the dev server (opens the app)
npm run build    # type-check + production build into dist/
npm run preview  # preview the production build
npm test         # headless tests: solver, persistence, body building, shape editing, edit utilities, actuators / motors
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
**Actuators and motors** are layered on top of the same solver: while animation runs, each
actuator/motor computes a world target for its joint(s) from its phase + speed, and the solver
takes those targets as additional "moving grounds" — sacred just like a normal ground, so
pins/sliders propagate the imposed motion through the whole assembly.

Source lives in [`src/`](src/): `geometry.ts`, `model.ts`, `solver.ts`, `view.ts` (camera),
`renderer.ts`, `main.ts`, plus `analyzer.ts` — a standalone topology diagnostic (kinematic
islands, degrees of freedom, loop / block decomposition) groundwork for future solver
optimizations. Tests live in [`scripts/`](scripts/).

## License

See [LICENSE](LICENSE).
