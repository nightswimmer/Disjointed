# Disjointed

A simple web app for creating and simulating **2D planar mechanisms** — bodies (with editable,
round-able shapes) coupled by joints (pins, welds, grounds, rails with riders and sliders) that
you can then drag and watch move.

> Status: working. Draw a mechanism with the shape tools (polyline, rectangle, circle, polygon,
> slot — each as a **body**, a **cut** or **reference** geometry), edit it, switch to simulate,
> and drag any part of it to drive it. The solver and shape/edit logic are covered by headless tests.
> A built-in manual opens beside the canvas (the `?` button): click any button or anything on
> the canvas to read what it is and does.

## Concepts

- **Body** — a rigid shape with rounded-able corners. It's defined by an editable **control
  polygon** plus a **corner radius** — with optional **per-corner overrides**, so each corner
  can carry its own radius; the outline you see is derived from those, so you can reshape it
  (drag corners) or round it (drag a corner's round radius handle — **Ctrl+drag** rounds every
  corner at once) any time, and you can **dimension a corner's radius**. A body can carry
  **holes** (inner cut-outs — drawn with any shape tool in the **Cut** role, or imported from DXF) — each hole is a full editable outline of
  its own: drag its nodes, round its corners, and a **circular hole** (a Circle in the Cut
  role, or an imported DXF circle) is a true parametric disk: move it by its centre node,
  resize it by its rim, and **dimension its diameter** with the Measure tool (type a value to
  drive it). Holes render hollow, are subtracted from the body's mass and
  inertia, mirror / rotate / scale / copy with the body, and their corners and edges are
  **measurable and constrainable** just like the outer profile. Holes don't restrict where
  joints can go — a joint can sit anywhere inside the outer outline, including dead-centre
  of a shaft hole.
- **Joint** — a point. Either **attached** to a body (rigid with it) or **free** (a body-less
  point). A free joint can be grounded to make an anchor without needing a body.
- **Constraints**
  - **Pin** — connect two joints on different bodies; they share a position but can rotate freely.
  - **Weld** — a **rigid** pin: the two bodies share the position *and* keep their drawn relative
    angle, locking them completely together (no relative motion at all). Placed with the Weld
    tool (`W`) where bodies overlap, or by toggling an existing pinned joint; the welded-in
    angle is whatever you drew, re-captured every time simulation starts. Welded joints show a
    **square** centre instead of the revolute pin's hollow dot.
  - **Ground** — lock a joint's position; its body can still rotate about it. A **body (or a
    whole group)** can also be grounded, which fixes it completely in simulation — position
    and rotation.
  - **Rail** — a line defined by two joints. Joints attached to it (**riders**) slide along the
    segment **between** those two joints, with hard stops at each end. The rail can be two joints
    on one body (it moves with the body, coupling two bodies) or **two free joints**, which makes
    a track fixed in world space — the two free joints get grounded automatically. A plain rider
    is a **pin-in-slot**: it slides along the rail *and* rotates freely.
  - **Slider** — a rider whose rotation is **locked**: it travels along the rail while its body
    keeps its drawn angle relative to the rail (a prismatic joint, in CAD terms) — the way two
    parts move relative to each other along exactly one axis. The Slider tool (`S`) builds one
    in **two clicks**: click the body that moves where its travel starts, then click where the
    travel ends — the rail (a double-headed arrow), its two endpoints and the body's riding
    joint are all created for you. The locked-in angle is whatever you drew, re-captured every
    time simulation starts. Drawn as a small rail-aligned **carriage rectangle** on the rider.
  - **Linear actuator** — a special rider on a rail that travels back and forth along the rail
    automatically when animation runs. Configurable speed (Hz) and motion profile (triangle = constant
    speed, sine = smooth ease). Off-animation it behaves like any other rider — draggable, pinnable.
  - **Motor** — a pivot + crank pair on one body; the crank pin orbits the pivot at a configurable
    angular speed (Hz) when animation runs. Off-animation the body behaves normally.
- **Group** — a permanent set of bodies **and free joints** that acts as **one object**:
  selected, moved, rotated, mirrored and copied together in Draw mode, and simulated as a
  **single rigid body** (nothing inside a group can move relative to the rest — locked free
  joints ride the group like welded points). Made and dissolved with `Ctrl+G` (a toggle).
- **Component** — a reusable sub-mechanism designed in its **own editing context** and placed
  as **instances**. Editing the definition **cascades to every instance**. Grounding something
  *inside* a definition means "fixed to the component's frame": on placement that material
  becomes one rigid **chassis** (never world-grounded), a joint-ground becomes a **revolute to
  the frame**, and the definition's sketch constraints, dimensions and measurements stay inside
  the definition — instances carry the designed shapes without design-time constraints, so an
  instance can be placed at any rotation. Non-grounded parts keep moving relative to each
  other: components can contain working mechanisms (pins, rails, sliders, actuators, motors),
  and definitions can nest instances of other components.
- **Guideline** — an **infinite construction line** through two points (Draw mode only; it
  never takes part in simulation). Placement, dragging and drawing snap onto guidelines in
  preference to the grid, and guidelines participate in sketch constraints and measurements
  like any other line — see *Construction guidelines* below.
- **Pattern** — a **live array** of one hole or one joint on a body: a linear row, a two-direction
  **grid**, or a circular arrangement around a centre. The instances are real holes / joints, but
  they are **derived from the seed** — move or reshape the seed and every instance follows; edit
  the count, spacing or angle on the canvas labels and the array re-lays itself. Delete the seed
  to make the instances independent — see *Patterns* below.

## Usage

There are two modes, switched from the toolbar or by pressing **Tab**. The toolbar uses
compact **icon buttons** — hover any of
them for a tooltip naming the tool and its shortcut. When something can't be done (an invalid
selection, a rejected cut, a file that won't load) a **toast notification** slides in under the
toolbar and fades out on its own; click it or its ✕ to dismiss it sooner. A **theme toggle** (sun / moon) at the right
switches between **dark and light** themes; your choice is remembered across sessions.

### Draw
Tools are **one-shot**: pick a tool (or press its shortcut), place one element, and you return
to **Select** mode. Press **Esc** to abort the current placement.

| Tool | Shortcut | Action |
| --- | --- | --- |
| **Role switch** | `1` / `2` / `3` | Not a tool but the setting every shape tool obeys: **Body** (the shape becomes a rigid body), **Cut** (it is subtracted from a body — a hole when it lies inside, a **notch** when it crosses the outline; a cut that would sever a body is refused, use Split), **Reference** (construction geometry: dash-dot, snappable, measurable, never simulated or exported). Sticky — draw a row of holes without touching it. The preview shows the role while you draw (tint / orange hatching plus the highlighted target / dash-dot) and a badge beside the cursor names it. Hold **Ctrl** on a shape's **first click** to flip Body ↔ Cut for that one shape. A cut's target is the **selected body** if one is selected (select the plate, then cut), else the topmost body under the first click, else the body the finished shape lies in — so overlapping bodies are fine. |
| **Polyline** | `B` body · `U` cut | Click each corner, then close (first vertex / double-click / Enter). Clicks that land on joints, corners or edges are kept there by a coincident constraint; near-H/V edges are straightened. `B` arms it in the Body role, `U` in the Cut role. In the Reference role a closed polyline is a reference polygon, two points a segment. **Body role, on a joint:** build a body *from joints* — click joints to outline, click a placed joint to finish, then move the cursor out to set the thickness and click. Joints on other bodies (and *grounded* free joints) get a coincident pinned joint so they stay put — including a **rider that belongs to another body**, which pins the two bodies together at that point so they ride the slider as one. A **slider rail node**, or a click on a bare **slider rail**, instead makes the new body its own **rider** of that slider. **Clicking on another body mid-draft** mints a fresh joint on that body and adds it to the outline (the two bodies get pinned together at that point); **clicking empty space mid-draft** mints a free joint and adds it to the outline (absorbed into the new body). |
| **Rectangle** | `Shift+B` | Click one corner, then the opposite corner — or **press and drag**. **Shift** for a square, **Alt** to draw from the centre. Edges get H/V constraints. |
| **Circle** | `Shift+C` | Click the centre, then a point on the rim — or press and drag the radius out. As a body: a **disk** (a one-point rounded outline — move it by its centre node, resize by its rim handle). As a cut: a **round hole** (parametric, dimensionable by diameter, patternable). As a reference: a circle whose centre snaps and measures. |
| **Regular polygon** | `Shift+P` | Click the centre, then one corner — or press and drag. Sides from the toolbar field that appears while the tool is armed, or `↑` / `↓`. |
| **Slot** | `Shift+S` | Click where the slot starts and where it ends — or press and drag — then move out to set its **width** and click. A two-point rounded outline (a capsule): a link with rounded ends as a body, a slotted hole as a cut, the axis segment as a reference. |
| **Line** | `Shift+L` | A **reference segment**: two points, or press and drag. Endpoints that land on joints / corners / reference points are held there by a coincident; the segment takes H / V / parallel / perpendicular / **equal** constraints like a body edge. Reference only. |
| **Arc** | `Shift+A` | A **reference arc**: click its start and end, then a point it passes through. Its three points are handles and references; placements snap onto the arc. Reference only. |
| **Text** | `Shift+T` | A **label**: click where it goes, type, Enter. Clicked **on a body** it is anchored to that body and rides with it; on empty space it stays put. Height from the toolbar field. **Double-click** a label to edit it, drag to move, Delete to remove. Never simulated or exported. |
| **Linear pattern** | `I` | Repeat a **hole** (click inside its cut-out) or a **joint** along a line: click where the **next instance** should go — the row appears (3 instances) and its **×count** label opens, so type the count and press Enter. The tool stays armed for an optional **second direction** (click where the first instance of the other direction goes — a grid), or press Enter / Esc to keep a single row. A direction within ~5° of horizontal / vertical gets an **H / V constraint** automatically, like a body edge. |
| **Circular pattern** | `Q` | Repeat a hole or a joint around a **centre**: click the seed, then the centre (snaps to joints, hole centres, corners and the grid) — 6 instances spread evenly, each turned with the arc; the count label opens for typing. |
| **Split** | `X` | Cut a body in two. Click a point on a body's **outline** (a corner, or anywhere on an edge) to start the cut, click inside the body to route it (as many vertices as you like — each kept inside), then click the outline again to finish: the body splits along that path into two bodies (same colour, grounded flag and group; the new one sits right above the original in the stacking order). Existing corners keep their rounding, the cut corners start sharp. Holes stay whole on their side (a cut through a hole is refused); joints stay where they are and belong to the side they're on; a rail or motor whose two joints end up on different sides is dropped. Bodies built from joints are converted to an editable sharp outline first. Esc aborts. |
| **Joint** | `J` | Click inside a body to attach a joint; click where bodies overlap to drop one in each (pinned together); click **empty space** for a free, body-less joint. With **Object snap** on, the joint lands on the nearest corner / edge midpoint / centroid / **hole centre** / joint / guide point in range (else the grid). Drop a joint on a **rail (or rail node)** and it's automatically attached to that rail as a rider. An attached joint always lands **inside** its body — if grid snapping would push it outside, it's placed at the exact click point instead. |
| **Weld** | `W` | Click where **bodies overlap** to weld them rigidly together at that point — a joint in each, sharing the position **and** locked at the drawn relative angle (no relative rotation; the angle re-captures from the drawn pose on every sim entry, like a slider's lock). Click an **existing pinned joint** to toggle its pin(s) **weld ↔ revolute**. |
| **Connect** | `C` | Click a joint, then another joint on a different body to **pin** them — or click a **rail** to attach the joint to it as a rider. |
| **Ground** | `G` | Click a joint to lock its position (it can still rotate). Ground a free joint to make an anchor. Click a **body** (away from its joints) to ground the whole body — fixed position *and* rotation in Simulate; a grouped body grounds its **whole group**. Click an **already-grounded** joint or body to remove the ground (a free joint anchoring a world-fixed rail keeps its ground — the track must stay anchored). |
| **Rail** | `K` | Click two joints on the **same body** (a moving rail), or **two free joints** (a world-fixed track — they get grounded automatically), to create a bare rail for **pin-in-slot** riders (joints that slide *and* rotate). Attach riders with Connect / the Joint tool. Rails are drawn as **double-headed arrows** spanning the travel. |
| **Slider** | `S` | **Two clicks.** Click a **body** where the slider starts — that body is the part that will move — then click where the travel **ends**. The body gets a joint at the start that rides a new arrow from start to end **without rotating** (it keeps its drawn angle relative to the track). If the second click lands on **another body** that covers both points, the arrow rides *that* body (a moving track — e.g. a block in a slot); otherwise the arrow is fixed in the world. Clicking one of the body's **existing joints** starts the slider there (that joint becomes the rider). A world-fixed arrow ridden only by this body is **its own**: dragging, rotating, mirroring or copy/pasting the body carries the arrow along (an arrow shared by two bodies, or riding another body, stays put). Both endpoints are ordinary joints: drag them, snap them, snap other things to them — dragging the **start** endpoint in Draw mode re-places the body's riding joint under it wherever the start lies inside the body (take it outside and the joint stays behind; bring it back and it snaps home). On an **existing** arrow: click it to add another slider there, or click a **rider** to toggle its rotation lock on/off. |
| **Guideline** | `L` | Click **two points** to place an **infinite construction line**. Each click lands exactly on a joint / body corner / another guide's point (with an automatic **coincident** constraint), projects onto a rail or body edge, or snaps to the grid. See *Construction guidelines* below (finite reference geometry comes from the Line / Arc tools and the shape tools in the Reference role). |
| **Rotate** | `R` | A mode (not one-shot): **drag a body** to rotate it about its centroid, or **drag a control node** of the already-selected body to rotate about that node. A **multi-selection or group** rotates as one about the centre of its bounding box. The angle **snaps to 45°** when it's within ~2° of a multiple. Joints and ground anchors turn with the body. |
| **Linear actuator** | `A` | Click a **slider arrow** (or rail) to make it self-driving: the slider's own carriage — the body — travels back and forth along the arrow when animation runs in Simulate mode. A bare rail with no rider gets a free self-driving rider instead. Off-animation the rider is just a normal rider you can pin to anything. |
| **Motor** | `M` | Click a joint to set the **pivot**, then another joint **on the same body** for the **crank pin**. In Simulate mode with animation running, the crank pin orbits the pivot at the motor's speed. |
| **Measure** | `D` | Click **two references**, then click where the value should sit. A reference is a **point** (a joint, a body corner node — hole corners included — a guide point, or any point inside a body) or a **line** (a rail, a body edge or hole edge, or a guideline). Click the **rim of a round hole** (or disk body) as the first pick for a **diameter** dimension — it needs no second reference, the next click places the label. Likewise, click a **rounded corner's arc** as the first pick for a **radius** dimension. Works in **both modes** — see *Measurements* below. |
| **Coincident** | `O` | Click **two points** (joints, body corners, or guide points) to make them share a position — or a **point and a line** (body edge, rail or guideline, either order) to hold the point on the **infinite** line. |
| **Horizontal** / **Vertical** | `H` / `V` | Click a **body edge, rail or guideline** (one click), or **two points**, to make it horizontal / vertical. |
| **Parallel** / **Perpendicular** / **Equal** | `P` / `T` / `E` | Click **two lines** (body edges, rails, guidelines or reference segments) to constrain their directions — or, for Equal, their lengths (Equal doesn't take infinite guidelines: they have no length; reference segments do). |

**Select mode** (no tool active, the default): click a body, joint, or rail to select it.
**Drag** the selection to move it. An attached joint **can't leave its body** — dragging it past
the edge makes it slide along the outline instead. A joint sitting exactly on one of its body's
corner nodes (as in a body **built from joints**) is **stuck to that node**: dragging either one
moves both, reshaping the body around it. A selected body shows handles on its outer outline
**and on every hole**: **square handles** move vertices, and each corner also gets a **round
radius handle** sitting on its fillet arc — drag it away from the corner to round just that
corner, drop it onto the corner to make it sharp, **double-click** it to go back to the body's
default radius. Hold **Ctrl** (Cmd on Mac) while dragging a radius handle to round **every
corner of that outline together** (the outer outline, or that hole) — per-corner overrides are
dropped, so the corners are uniform again. **`[` / `]`** still decrease / increase the body-wide default radius (this is
how you round a freehand polygon: draw it, select it, press `]`; corners with their own radius
keep it). On a circular (disk) hole the round handle rides the rim — drag it to resize the
hole, drag the centre node to move it (with **Object snap** on, a dragged node — a hole
centre included — snaps onto other corners, centres, joints and edges). With a body selected you can also edit any outline by
**double-click**: double-click an **edge** (outer or hole) to add a node there (snapped to the
grid when Snap is on), or double-click a **node** to remove it (outer outlines keep a minimum
of 3) — double-clicking a hole's **last removable node deletes the whole hole**. Press
**Delete** to remove the selection: a body takes its joints and constraints with it; a slider
arrow takes its endpoints and riders with it (deleting an endpoint, the last rider or the
moving body removes the rest of the slider too — joints that still serve a pin, motor, other
rail, group or pattern are spared); a plain joint detaches from everything referencing it.

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

**Select parts of a body.** With one body selected, **Shift+drag a box from empty space** to
select several of its *features* at once: corners of the outline, holes (their corner handles —
a round hole's centre), and the joints sitting on it (**Ctrl+Shift+drag adds** to the set; a
pattern's members count as their seed, so the whole array is included). Selected corners fill
blue and selected joints get the selection ring. Then:
- **drag any selected corner or joint** to move them all together — the grabbed one snaps to
  the grid / other objects and can place implicit constraints, exactly like a single corner
  drag, and your sketch constraints keep holding on the rest of the body;
- **Delete** removes them (a hole left with too few corners disappears whole; the outline
  always keeps at least three corners);
- **`Ctrl/Cmd+C`** copies the **whole holes** (every corner selected) **and joints** in the set
  together with everything internal to them — grounds, a rail between two copied joints, a
  motor whose pivot and crank are both copied, sketch constraints and driving dimensions
  between them, and live patterns whose seed is copied — and **`Ctrl/Cmd+V`** pastes them
  **into whichever body is selected**, at the cursor. That's how you carry a hole pattern or a
  set of joints from one part to another (or duplicate them within the same part). Features
  that would land outside the target's outline are skipped and you're told how many. Outline
  corners never copy — they belong to the shape they're on.
Shift+drag *on* a body or joint is still the rigid drag described above.

**Editing utilities** (on the selection — a single body, or a multi-selection / group):
- **Copy / Paste** (`Ctrl/Cmd+C` / `Ctrl/Cmd+V`, keyboard only) — duplicate the selection with
  its joints and every constraint internal to it: grounds, internal sliders, **pins between the
  selected bodies**, **group membership**, and its **sketch constraints and driving dimensions**.
  The copy **keeps the original colours**, lands at the cursor (grid-snapped when Snap is on) and
  becomes the selection — pasting a group gives you a new, working group. Anything reaching
  outside the selection (e.g. a pin to an uncopied body) isn't reproduced. With a **feature
  selection** active (see *Select parts of a body*), the same keys copy its holes + joints and
  paste them into the selected body instead.
- **Mirror H / V** — reflect a selected body left↔right or top↔bottom in place about its centroid;
  a multi-selection / group reflects about the centre of its combined bounding box. Constraints
  and dimensions follow their corners/edges through the flip. **Component instances mirror
  too**: the instance becomes its definition's mirror image (motors spin the other way) while
  the definition and its other instances stay as they are — see *Components* below. Grouped
  in the toolbar next to **Rotate**.
- **Combine** (toolbar button next to **Split**, or **N**) — merge the **multi-selected bodies**
  (Ctrl+click or box-select two or more) into one body: the union of their shapes. They must
  **overlap or share an edge** (bodies that don't touch the rest, or touch only at a corner,
  are refused with a message saying why). The **first-selected** body survives — it keeps its
  colour and stacking position — and the others' joints move onto it; pins / welds *between*
  the combined bodies disappear (they'd be inside one body now), motors and rails carry over,
  groups merge. Corners that survive unchanged keep their rounding, new corners are sharp;
  holes not covered by the other bodies stay (a circular hole stays a true disk), and a region
  the union closes off becomes a new hole. Bodies built from joints are converted to an
  editable sharp outline first.
- **Send to back / Bring to front** (toolbar buttons next to Mirror, or **PageDown / PageUp**) —
  move the selection to the bottom / top of the stacking order. Clicks always pick the topmost
  body, so this also decides what a click lands on: send a big imported reference body to the
  back and it stops covering — and stealing clicks from — the mechanism drawn over it. The
  order is saved with the file.

**Patterns.** A pattern is edited **on the canvas**, like a dimension — there is no settings panel:

- **Labels**: a linear direction shows a **dotted line** from the seed through every instance,
  an arrowed **spacing** dimension beside the first step, and a **×count** label past the last
  instance; a circular pattern shows a crosshair at the centre with **×count**, the **angle**
  between instances (`even`, or a value in degrees — counter-clockwise on screen) and a
  **↻ turn / ↑ fixed** badge (click it: instances turn with the arc, or keep the seed's
  orientation). **Double-click** any label to type a new value (Enter commits, Esc cancels;
  type `even` or clear the angle to spread over the full circle).
- **Handles**: click a label or the dotted line to select the pattern, then drag the **square at
  the last instance** to re-aim and re-space that direction, or the **centre crosshair** to move
  the circle's centre.
- **Instances follow the seed**: drag the seed (dragging any instance does the same), reshape it,
  round its corners or resize a round hole — every instance copies it. Instances that would fall
  **outside the body** or **overlap** another hole / joint are ringed **red** so you can adjust
  the count or spacing (they are never moved for you).
- **Constraints on a direction**: the dotted line is a line reference — apply **Horizontal /
  Vertical / Parallel / Perpendicular** to it (or measure against it) like a body edge; the
  direction pivots about the seed to satisfy them, and stays constrained as the seed moves.
- **Delete** with the pattern selected removes every instance and keeps the seed. Delete the
  **seed** hole / joint instead and the pattern dissolves: the instances stay as ordinary,
  independent holes / joints. Deleting one instance removes them all (the seed stays). Patterns
  are saved with the file (format v19) and travel with copy / paste, mirror and scale.

**Measurements.** The Measure tool (`D`) works in **both modes**, and each mode keeps its own
set of measurements. What gets measured follows from the two references you pick:

- **Two points** — where you place the value picks the dimension, CAD-style: above/below the
  pair → **horizontal** distance, beside it → **vertical**, in the diagonal zones → **direct**.
- **A point and a line** — the perpendicular distance to the (infinite) line. The dimension
  line sits wherever you drag the label along the line, with dashed extensions back to the
  point (and to the line's end when you're beyond it).
- **Two lines** — the **distance** while they're parallel, the **angle** otherwise. This is
  re-evaluated live, so a line pair can flip between distance and angle mid-simulation, and
  the side you place the label on picks θ vs 180°−θ.
- **A disk** — click the rim of a round hole (or a disk body) and place the label: a
  **diameter** dimension (`⌀`), drawn through the centre towards the label. In draw mode you
  can type a value into it (double-click the label) and it **drives** the hole's diameter;
  resizing the disk directly afterwards (rim handle, `[` / `]`) turns it back into a
  reference dimension.
- **A rounded corner** — click its arc and place the label: a **radius** dimension (`R`),
  drawn from the arc centre (marked with a dot) out to the arc towards the label. In draw
  mode, typing a value **drives** the corner's radius: if the outline's corners are
  **uniform** (no corner has its own radius — e.g. after a Ctrl-drag of a radius handle or
  `[` / `]`), the value sets **every corner** of that outline; otherwise just that corner.
  Dragging a radius handle directly afterwards turns the affected dimension back into a
  reference. A sharp corner has no arc to click — round it a little first.

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
  (or the badge — hovering a badge also **highlights the elements it constrains**, with a
  dotted line between them when they're apart): click to select, **Delete** to remove. A toolbar toggle next to the
  constraint tools **shows/hides all badges** (constraints keep working while hidden), and a
  matching toggle next to Measure shows/hides **all measurements** — in both modes.
- **Constraints by dragging**: the common cases need no tool — mid-drag, hover a target to arm
  it, then release aligned with it (or on its line) and the H / V / coincident constraint is
  created. See *Implicit constraints while dragging* under Grid & snapping.
- **Driving dimensions**: **double-click** a dimension's value, type a number, press Enter.
  The **first** driving dimension on an otherwise-unconstrained body **scales the whole body
  uniformly** (same shape, new size); further dimensions move **only the involved nodes**
  while every constraint and driving dimension holds. A **driven** (reference) dimension
  shows its value **in parentheses**; a driving one shows it plain. Clear the field to turn a
  driving dimension back into a reference. Impossible targets are rejected with a red flash.
- **Dimensions on components drive poses, never shape**: a dimension between two different
  component instances moves one of them **rigidly** to the value (a grounded instance stays
  put — the other side moves); one between two **mobile parts of the same component**
  re-poses its internal mechanism (pins intact); one internal to a single rigid piece is
  rejected — that's already dimensioned at a deeper level (inside the definition). Dragging
  a component **pulls its dimensioned partners along**, and a dimension that can't hold
  (grounded partner, or a definition edit reset the poses) turns **red** until you re-apply
  it. Every driving dimension also **holds its drawn relative direction** — dragging one
  part fast past the other can never flip the two sides through each other.
- **Constraints on components work the same way**: a constraint between a component and free
  geometry moves the free side (component shapes are locked); one between two components
  moves a component **rigidly** — coincident, point-on-line and H/V point pairs slide the
  second-picked component into place, line H/V, parallel and perpendicular **rotate** it
  about the constrained edge (the first pick stays put; a grounded component never moves).
  Between two mobile parts of one component the internal mechanism re-poses. **Equal**
  between two component edges, or anything internal to one rigid piece, is refused. A
  constraint that can't hold (grounded partner, a definition edit, rotating a component
  against its H) shows its badges in **red** until it's satisfied again.
- **Sketch-aware dragging**: with constraints or driving dimensions present, dragging a node,
  joint, or body (and rotating) **re-solves the sketch live** — what you drag follows the
  cursor as far as the constraints allow, and everything constrained to it comes along. A
  body moved by a dimension or constraint moves **as a whole**: its holes and joints ride
  with the outline (holes are as rigid as the outline — even a plate that gets squashed
  against a dimension keeps its holes in place relative to it).
- **Auto-constraints while drawing**: a freehand edge drawn within ~5° of horizontal /
  vertical snaps straight and gets the H/V constraint; a vertex clicked **on an existing
  joint or corner** lands exactly there and gets a coincident constraint.

**Construction guidelines & reference geometry** (Draw mode). The Guideline tool (`L`) places **infinite lines**
through two points — CAD-style scaffolding for laying out a mechanism. The Line and Arc tools,
and any shape tool in the **Reference** role, add finite reference geometry (segments,
polylines and polygons, circles, arcs, text labels) that behaves the same way: dash-dot,
crosshair handles on the defining points (drag one to reshape, drag elsewhere to move it whole;
a selected circle shows a rim handle to resize it), snappable, measurable, constrainable
(a polyline's edges take line constraints, equal length included), hidden in Simulate, never
exported, Delete to remove. For infinite guidelines in particular:

- **Placement snaps to existing elements**: a click lands exactly on a joint, body corner or
  another guide's defining point (and records an automatic **coincident** constraint so the
  guide stays attached when that point later moves), projects onto a rail or body
  edge, or falls back to the grid.
- **Snapping prefers guidelines over the grid**: with Snap on, anything you place or drag
  lands *on* a nearby guideline (projected onto it) — and where two guidelines cross, on
  their **intersection**. Great for laying out joints along a line or at a crossing.
- **Editing**: click to select (its two defining points show as small **crosshairs**, so they
  never look like joints); drag the **line** to
  move it whole (angle preserved), drag a **defining point** to re-aim it, **Delete** to
  remove it (its constraints and measurements go with it).
- **Constraints on guidelines** (H / V / parallel / perpendicular / coincident — a guide
  point onto another point, or a point held **on** the guide's infinite line — and
  measurements, including driving dimensions): a guide with a **single** demand on it yields
  — it is satisfied by moving **only its free points, never joints or body nodes**. A guide
  that is **attached to geometry** (coincident with a corner or joint) or carries **several**
  demands (two driving dimensions, an attachment plus a dimension…) acts as a **reference**
  instead: the geometry moves to satisfy the dimension while the guide stays. So you can
  dimension any number of bodies to one guide, and dragging one of them pulls the guide and
  the others along. Likewise you can hold **any number of points on one guideline** (a quick
  way to align corners / joints): the first coincident brings the guide to the point, every
  further point moves straight onto the line, and dragging one aligned point across the line
  — or dragging the guide — carries all of them. Genuinely over-constrained edits are still rejected with a red flash
  (e.g. Horizontal on a guide whose both points are bound to joints at different heights).
  Constraints hold **during** drags too: dragging the free point of a joint-bound vertical
  guide slides it vertically — it can't be pulled off-axis even momentarily.
- **Picking a guideline** (Measure / constraint tools, object snap) highlights the **whole
  infinite line** and rings its two defining points.
- Guidelines are drawing aids only: they're invisible (and unpickable) in Simulate mode,
  never affect the simulation, and don't travel with copy/paste.

**Components** (Draw mode). Design once, place many:

- **Create**: select the bodies (and free joints) that should form the component and press the
  **⊞ button** — the selection is packed into a **definition** and replaced by one **instance**
  in place. Everything internal travels into the definition: joints, pins, grounds, sliders,
  actuators/motors, sketch constraints, dimensions and measurements. The definition's
  constraints **don't exist outside it** — instances show the designed shapes only.
- **Start empty**: press **⊞ with nothing selected** to create a completely **empty component**,
  opened for editing immediately — the natural way to build a component made entirely of other
  components (place instances inside it, no throwaway body needed). An empty definition can't be
  placed until it has content.
- **Instances are atomic**: clicking any part selects the whole instance (dashed outline);
  drag / rotate / mirror / copy / delete act on it whole, and pasting a copied instance creates a new
  instance of the same definition. Shapes are **design-locked** — no corner handles, and
  constraints / dimensions never reshape an instance: they move the free side, or **drive the
  instance's pose** (see *Driving dimensions* and *Constraints on components* above).
  **Shift-drag** a member to pose the
  instance's internal mechanism rigidly, exactly like simulating it.
- **Mirror an instance** (Mirror H / V with it selected): the instance becomes the
  **mirror image of its definition** — a left-hand part from a right-hand design — and stays
  linked: definition edits keep cascading into it, and copies of it paste mirrored. The
  definition and every other instance are untouched. Motors inside spin the other way, as a
  reflection should. Mirroring re-places the instance from its definition, so an internal
  mechanism you had posed with Shift-drag returns to the definition's layout.
- **Grounding in a definition = the chassis**: grounded bodies and grounded free joints become
  one rigid cluster per instance; a joint-ground on a moving part becomes a pivot fixed to that
  chassis. Ground an instance in the assembly (Ground tool on any of its bodies) to fix its
  chassis in the world — the internal mechanism keeps working.
- **Edit the definition**: double-click any instance (or use ✎ in the **component browser**,
  the panel behind the grid-of-squares toolbar button). A **breadcrumb bar** shows where you
  are (`Assembly ▸ Leg ▸ Foot` — definitions can nest); every tool works inside, including
  Simulate. Changes **cascade live to every instance everywhere** — each instance keeps its
  placement while its parts snap to the definition's layout (**the definition is the pose
  reference**: what you draw inside it is exactly what every instance shows, nested components
  included). Click a breadcrumb (or press **Esc** with nothing selected) to go back out.
  If a definition edit reshapes a body from under a joint you added **at the assembly level**
  (the new outline no longer covers it), the joint is flagged **red with a dashed ring** rather
  than moved — drag it back inside or adjust the definition.
- **Edit in context**: **Ctrl+double-click** an instance to open its definition with the
  **whole surrounding assembly drawn faded in the definition's own frame** — the instance you
  came through sits exactly on the live definition, everything else appears where it is
  relative to it (rotated, shifted, mirrored for a mirrored instance; nested components see
  through every level). The ghost is inert — nothing on it can be selected, dragged or
  constrained — but its corners, midpoints, edges, joints and rails are **snap targets** (with
  object snap on), so a hole lands exactly on the pin that will hold it, and the Measure tool
  can pick its joints / corners / edges / rails for **temporary dimensions**: shown in a muted
  tint, always driven, never saved, gone when you leave the definition. **Double-click a
  temporary dimension's value** to move your own geometry so it reads that value (a corner
  reshapes the body, an edge or body point carries the whole part) — the dimension stays a
  reference. The **eyes beside the breadcrumbs** (◉ / ◌) choose how far out the ghost reaches:
  click one to show up to that level, click a shown one to hide it and everything outside it.
  A plain double-click still opens the definition with the ghost off (turn it on with the
  eyes); a definition opened from the component browser has no instance to place it by, so its
  eye stays dimmed.
- The **component browser** lists definitions: rename inline, **＋** inserts an instance (click
  the canvas to place it), **✎** edits, **×** deletes — any instances of it become plain
  bodies (each keeps its rigid group, but they no longer share a definition).
  Circular references are rejected. Drag a row's **⋮⋮** grip to **reorder** the list (the
  other rows slide out of the way; the order is saved with the file). The list and the canvas
  **mirror each other**: hovering a row **fades everything else** and highlights every
  occurrence of that component — including ones nested inside other components — while
  hovering or selecting a component on the canvas highlights its row(s) in the list.
- **Fork an instance ("make unique")**: press **⊞** with a component instance selected and its
  definition is copied into a new independent component (named *“&lt;source&gt; copy”*), with the
  selected instance re-pointed at the copy — nothing moves, and from then on the two components
  are edited completely independently. Components nested inside the definition stay shared.

**Body colour.** A colour swatch in the toolbar sets the active colour: with **nothing selected**
it's the colour given to newly drawn bodies; with a **body selected** it shows that body's colour
and editing it recolours the body.

**Actuator / motor speed.** Select an actuator's rider, the rail it rides, or a motor's body
(or its pivot / crank joint) and a small inline panel appears in the toolbar with a speed field
(in Hz) — and, for linear actuators, a `/\` ↔ `~` profile toggle (triangle for constant-speed
end-to-end travel, sine for smooth ease in/out at the endstops).

Joints are color-coded: **blue** = pinned, **yellow** = grounded, **green** = rail rider;
rail-defining joints get a **green ring**, a **loose free joint a dashed ring**, a **welded**
joint swaps the revolute pin's hollow centre for a **square** one (plus a blue square
outline), and an
**orientation-locked rider (a slider)** additionally shows a green rail-aligned **carriage
rectangle**. Once a free joint is attached to a rail it's no longer loose, so it drops the
dashed ring and shows as a normal (green) rider. While drawing, a constraint whose endpoints
don't yet touch is drawn as a **dotted connector** so the link still reads as connected:
**blue** between two pinned joints, and **green** from a rider to the **middle of its rail**.
A joint stranded **outside its body** — a shape change happened under it, typically a component
edit cascading into an instance carrying an assembly-level joint — turns **red with a dashed
ring** and the hint bar warns until it's resolved. Nothing is moved automatically: drag the
joint back (drags clamp to the outline) or fix the shape / component definition.

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

### Units, DXF import & cut-file export
A **unit dropdown** in the toolbar's grid group declares what one world unit means — **mm, cm,
m or in** (default mm). It's purely a declaration (changing it never moves geometry): distance
measurements show the unit, and imports convert into it. The choice is saved with the file.

**Drag-and-drop a `.dxf` file onto the canvas** to import its shapes as bodies at the drop
point, at true scale — the file's `$INSUNITS` is converted into your working unit (a unitless
file is assumed to already be in working units). Closed polylines (arc bulges included),
circles, and loose lines/arcs that chain into closed loops all import; a loop **inside**
another becomes a **hole** in it, so a plate with cut-outs arrives as *one* body. **Rounded
corners stay editable**: an arc that fillets two straight segments imports as a sharp control
corner carrying that radius (grab its round handle to change it), and circles — bodies or
cut-outs — import as true parametric disks. The imported
shapes land multi-selected, ready to move or group. If an import covers your mechanism, press
**PageDown** (or the Send-to-back button) to push it behind everything. Dropping a `.json`
file loads it as a scene, same as the Load button. (`dxf import test.dxf` in the repo is a
small sample to try.)

**Export a cut file** with the Export button (next to Save): it writes the **selected body or
multi-selection — or every body when nothing is selected —** as a flat file ready for CNC /
laser / plasma work. Pick **DXF** (true arcs, in your working units, for CAM software such as
Fusion 360, VCarve, Carbide Create, Easel or SheetCAM) or **SVG** (in millimetres, for laser
software such as LightBurn, Glowforge, xTool or Inkscape). Rounded corners, disks and rounded
hulls export as exact arcs, every hole as its own loop, and geometry is placed in the positive
quadrant. Tick **Drill holes at joints** to add a hole of the given diameter at every joint of
the exported bodies (on its own layer / group), so a linkage can be cut and pinned straight
away. Files are named after the body (`body-…`, or `bodies-…` for several), or after its component
when it belongs to one — `Crank-…` for the whole component (a selected instance, or everything
while editing its definition), `Crank-body_2-…` for one body of several, `Crank-bodies-…` for
a partial selection — with the date and time appended. An exported DXF drops straight back onto the canvas with its fillets editable.

### Grid & snapping
The toolbar's grid group controls a world-locked grid: **Grid** toggles its visibility, **Snap**
toggles snap-to-grid, and the **grid-size selector** sets the spacing: click the value to open a
list of presets (1 to 200), pick one, or type your own in the **Custom…** field at the bottom and
press Enter / `+` — it applies straight away and joins the list (custom sizes are remembered in the
browser; the `×` next to one removes it). Decimals are allowed. The **units** dropdown sits right
after it. With Snap on, new joints and freehand vertices land on the grid,
and dragging snaps too: a per-vertex reshape snaps the grabbed corner, while moving a whole body
snaps whichever is nearest the grab point — the body's centroid or one of its corners. Visibility
and snapping are independent (you can snap to a hidden grid).

**Object snap** (the toolbar button next to Snap; independent of grid snap) makes drags snap
objects to each other. When you start dragging a body, joint or multi-selection, the feature of
it nearest the grab becomes the **reference** — a **corner**, then an **edge midpoint**, then an
**edge**, or the object's **centre** if nothing is close (holes count too). It's highlighted in
orange (and previewed while you hover, so you can see what you're about to grab). While dragging,
that reference snaps onto the same features of everything else — other bodies' corners, midpoints,
centres and edges, joints, rails and guidelines — with the target highlighted dashed. A corner or
midpoint can also land on another edge or guideline; an edge snaps onto **parallel** edges and
guidelines only, sliding sideways until the two are flush (it can't rotate the body), and the
target line is shown extended so you can line up bodies that don't overlap. With nothing in
range the drag falls back to the grid/guideline snap (or moves freely when Snap is off).
Object snap also applies to **reshaping**: a dragged corner node or hole centre is its own
reference and snaps onto the other features (its own body's other corners included, but not
the features that move with it), and to **placement**: a new joint, or the centre of a round
hole being dragged out, lands on the nearest corner / midpoint / centre / joint in range.

**Implicit constraints while dragging** let you place the common sketch constraints without
picking a tool. While dragging a body, joint, corner node or hole centre (with or without
object snap), **hold** the dragged point or edge over another element — a corner, hole centre,
joint, guide point, edge, rail or guideline — for about half a second: it becomes the
**alignment candidate** and lights up violet (hovering something else later replaces it;
**Esc** drops it and the drag goes on). Now move on to where you want the object. When the
dragged point lines up **horizontally or vertically** with a candidate point, or lands on the
**line** of a candidate edge / rail / guideline (or a candidate point lands on the dragged
edge's line), a dotted violet line with the constraint's badge (H, V or ◎) shows the
alignment. **Release while it shows** and the geometry is nudged exactly into alignment and the
constraint is created — the same H / V / coincident you'd get from the tools, badge and all.
Dropping a point right on top of a candidate point is a plain move (no constraint). If a
placement can't be satisfied, nothing is created and a message says why (the conflicting
items flash red).

### Navigate
- **Mouse wheel** — zoom toward the cursor (0.05× to 200×).
- **Right-drag** — pan the view (anywhere). To move a body or joint, select it and left-drag (see Select mode; turn on **Object snap** to drag by a corner / edge / centre and snap it onto other objects).
- **Fit to screen** (`F`, or the toolbar button) — frame the whole mechanism centered in the canvas (a rotated view fits the tilted picture).
- **Rotate the view** (`Shift+R`, or the toolbar button next to Fit; both modes) — a big ticked ring with a crosshair along the world X / Y axes appears over the canvas. Drag the ring or an arm to turn the whole picture about the screen centre; the angle snaps to 5° steps, hold **Shift** while dragging for any angle. The box under the centre shows the angle and takes an exact value (type it and press Enter, like a dimension). Double-click the centre (or press `0`) for 0°; Esc, `Shift+R` or a click elsewhere closes the dial. Pan and zoom keep working while it is open. The rotation is only how you look at the drawing: the mechanism, its horizontal / vertical constraints and the grid stay in their own axes (the grid turns with the drawing), and labels stay upright.
- **Tab** — switch between Draw and Simulate mode.
- `Ctrl/Cmd+S` save, `Ctrl/Cmd+Shift+S` save as, `Ctrl/Cmd+O` open (see below).

### Help
- **Help** (the `?` button next to the theme toggle, or the `?` key) opens the manual in a
  drawer beside the canvas. While it is open the pointer becomes a question mark and every
  click is a question: click a **toolbar button** to read about that tool, or click **anything
  on the canvas** — a body, a hole, a joint, a pin, a ground, a rail, a slider, a guideline, a
  dimension label, a constraint badge (each constraint kind has its own page), a pattern, a
  component instance — to read what that element is and does. Nothing is activated while the
  drawer is open; close it (the button, `?`, Esc or the ✕) to get back to work.
- `F1` opens the manual at its table of contents. The manual also has a short tutorial that
  builds a four-bar linkage, and a keyboard-shortcut reference.
- Drag the drawer's left edge to resize it; the arrow button opens the manual in its own tab.
  Its pictures are vector drawings made by the app's own renderer and follow the light / dark
  theme.

### Save, load & backups
- **Save** (`Ctrl/Cmd+S`, or the toolbar button) writes your mechanism as a `.json` file. In
  Chrome, Edge and other Chromium browsers the first save asks where to put it and every
  later save **overwrites that same file** silently; `Ctrl+Shift+S` (or Shift-click Save) saves
  as a new file. The tab title shows the file name, with a `•` while there are unsaved
  changes. Firefox and Safari can't write files in place, so there a save downloads a
  timestamped copy instead.
- **Load** (`Ctrl/Cmd+O`, or the toolbar button) opens a `.json` — you can also drag-and-drop
  one onto the canvas. The loaded file becomes the save target, and stays so after a reload
  (the browser may ask permission once).
- **Auto-backup** (the clock button next to Export) writes timestamped copies,
  `<name>-backup-<date-time>.json`, into a folder you choose. Pick how long after the first
  change a backup is written (1–30 min; the panel shows when the next one is due) and how many
  to keep (older ones are pruned). Saving cancels the pending backup, since the file is up to
  date. Needs a Chromium browser; the panel says so otherwise.
- Your work is also auto-saved in the browser and restored automatically the next time you
  open the app.

## Development

Requires Node.js (built with Node 24).

```bash
npm install      # install dependencies
npm run dev      # start the dev server (opens the app)
npm run build    # type-check + production build into dist/
npm run preview  # preview the production build
npm test         # headless tests: solver, persistence, body building, shape editing, edit utilities, actuators / motors, measurements, sketch constraints, groups (incl. free-joint members), grounded bodies, rigid-drag scoped solves, construction guidelines, DXF import / units / holes, DXF / SVG cut-file export, hierarchical components, slider orientation locks, two-click sliders (construction, whole-slider deletion, actuator on the body's own rider), welds (rigid joints), pose-level driving dimensions, sketch constraints on components, live hole / joint patterns (members riding with their seed), the context ghost (the enclosing assembly mapped into a definition's frame)
npm run manual   # regenerate the manual's illustrations (public/help/img) and toolbar glyph tables, and check that every help topic exists
```

The manual lives in `public/help/` and is a static page. Its illustrations are generated, not
drawn by hand: `scripts/manual/shoot.ts` drives the real app in your installed Chrome through
Playwright (no browser download), builds fixture mechanisms with the Scene API, performs the
gestures, and captures the canvas through the app's own renderer as SVG (theme colours become
CSS variables, so one file serves both themes). Add a shot in `scripts/manual/shots.ts` and a
`<section class="topic" id="…">` in `public/help/index.html`; the generator fails if a topic
the app can ask for is missing.

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
as breaks — so a connected impossible piece doesn't corrupt the parts that can be solved. Rails
are point-on-line constraints with end-stops; the rail is either a body (which moves) or a
world-fixed line built from two grounded free joints. An **orientation-locked rider (slider)** adds a
direct **angular projection**: the rider's rigid unit rotates against the rail's until their
wrapped relative angle matches the value captured from the drawn pose (re-captured whenever the
drawn layout changes) — the wrapped error has a unique zero, so a hard drag can't flip a carriage
180°. **Welds (rigid pins)** don't iterate at all: at the top of every solve, weld-connected
bodies (and groups) are merged into one **rigid composite**, snap-assembled exactly at the drawn
relative angle — so a welded chain moves as a single body and converges as fast as a group does,
no matter how long the chain; only a weld that genuinely can't hold (between two fixed bodies, or
an unclosable weld loop) is reported as a break. Permanent **groups** use the same mechanism,
which is why welding and grouping behave identically in simulation. Body outlines are
generated from a control polygon + corner radius (rounded corners via fillet or outward offset),
with optional **per-corner radii** overriding the body default; arcs sample at 7.5° per segment.
The fillet rounds convex and concave (reflex) corners correctly, and splits each edge between its
two corners so neighbouring fillets never overlap or fold — even on thin shapes at large radii.
A body's **holes** are outlines of the same kind (control polygon + rounding, or a one-point
disk), each with a derived loop drawn with an even-odd fill and subtracted from the mass
properties (net area, composite centroid, parallel-axis inertia); their corners and edges are
first-class measurement / constraint references, while picking and
joint containment deliberately use the outer outline only.
**DXF import** is a small dependency-free reader (`dxf.ts`): it samples arcs/bulges into
polyline points, chains loose segments into closed loops, nests loops even-odd style into
solids-with-holes, and converts units via the file's `$INSUNITS` and the document's working unit.
Corner arcs that are tangent fillets are **reconstructed** rather than baked — they come back as
sharp control corners with per-corner radii, and circles as parametric disks — so imported
rounded geometry stays editable.
**Actuators and motors** are layered on top of the same solver: while animation runs, each
actuator/motor computes a world target for its joint(s) from its phase + speed, and the solver
takes those targets as additional "moving grounds" — sacred just like a normal ground, so
pins/sliders propagate the imposed motion through the whole assembly.
**Permanent groups** are rigid composites in the solver: a grouped body's "host" carries the
group's combined mass, centroid and inertia, and every impulse translates + rotates **all**
members about the combined centroid — so pins, sliders, grounds, drags and motors on any member
move the group as one body, and constraints *between* members of one group are inert. Groups can
also lock **free joints** as members (point masses riding the rigid motion): a ground on a locked
joint pivots the whole group about it, and a rail between two locked joints is a track that moves
with the group.
**Components** are *materialized*: placing an instance expands the definition into real bodies,
joints and constraints tagged with provenance (def-local id → scene id), so the solver, renderer
and hit-testing need no hierarchy concept at all. Grounds inside a definition are *converted* on
expansion — grounded material becomes one rigid chassis group per instance, and a joint-ground
becomes a pin to a synthesized group-locked point (a revolute to the component's frame). Editing
a definition re-expands each instance by **reconciling** against the provenance maps: surviving
elements keep their scene ids while every part's pose snaps back to the definition's layout at
the instance's current placement (derived from a chassis body's pose vs its cached def-frame
pose — the definition is the pose reference), and added/removed definition elements
appear/disappear. Definitions can contain instances of other definitions (a DAG); a
change cascades through the definition graph and then into every open context.
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
idea, but over *shape* — the world positions of body corner nodes (outer and hole outlines
alike), joints and guideline
defining points — rather than rigid poses. After a converged solve, bodies rebuild from
their new control polygons; an unsatisfiable solve never touches the scene (edits are
rejected, not approximated). Every solver variable carries a **mobility rank** —
construction (guide points) < geometry (nodes, joints) < actively-dragged <
**component-instance geometry** (immovable: its shape belongs to the definition) — and each
correction flows entirely to the more mobile side (equals split evenly). That one rule
gives the CAD feel: guide constraints move guides rather than geometry, dragged geometry
is never tugged back by its constraints (guides follow it exactly, so groups stay rigid),
and when a drag would need the constraints to give way, a symmetric re-solve runs the same
frame so the constraint visibly holds and the drag slides along the directions left free.
**Pattern members** are derived geometry: each member variable is coupled to its seed by a
rigid offset, so a constraint or dimension on a member moves the whole array — seed, members
and their body together — instead of pinning it in place.
**Pose dimensions and pose constraints** (`pose.ts`) handle dimensions and sketch
constraints whose every end lives on component instances — not shape material at all:
between two instances the correction is a closed-form **rigid move** of one of them — a
translation for distances, coincident and H/V point pairs, a rotation about the constrained
edge's midpoint for line H/V, parallel and perpendicular (Gauss-Seidel over all pose items,
translations and rotations alternating, run live during drags with the dragged instances
anchored, so partners follow the drag); within one instance the **rigid-drag solver**
re-poses the internal mechanism with everything else frozen. Rejected edits restore a
snapshot, and unsatisfied items render red. Every driving dimension carries its captured **side** (the drawn relative
direction): corrections are signed toward it, so an overshooting drag reads as a large
error back — the two sides can never flip through each other.

**Cut-file export** (`export.ts`) is the importer's mirror image: bodies become closed rings whose
edges carry DXF-style *bulges* (so fillets, disks and rounded hulls stay true arcs) plus hole
loops and optional joint circles; the DXF writer flips y and emits R12 polylines with bulges in
the working units, the SVG writer converts to millimetres and emits `A` arcs in even-odd paths.

Source lives in [`src/`](src/): `geometry.ts`, `model.ts`, `solver.ts`, `sketch.ts`,
`pose.ts` (pose-level dimensions + constraints on components), `dxf.ts` (DXF import), `export.ts` (DXF / SVG cut-file export), `view.ts` (camera),
`renderer.ts`, `main.ts`, plus `analyzer.ts` — a standalone topology diagnostic (kinematic
islands, degrees of freedom, loop / block decomposition) groundwork for future solver
optimizations. Tests live in [`scripts/`](scripts/).

## License

See [LICENSE](LICENSE).
