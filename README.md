# Disjointed

A simple web app for creating and simulating **2D planar mechanisms** — bodies (with editable,
round-able shapes) coupled by joints (pins, grounds, sliders) that you can then drag and watch move.

> Status: working. Draw a mechanism (freehand or from joints), edit it, switch to simulate, and
> drag a joint to drive it. The solver and shape/edit logic are covered by headless tests.

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

## Usage

There are two modes, switched from the toolbar.

### Draw
Tools are **one-shot**: pick a tool (or press its shortcut), place one element, and you return
to **Select** mode. Press **Esc** to abort the current placement.

| Tool | Shortcut | Action |
| --- | --- | --- |
| **Body** | `B` | **Empty space:** click to add vertices, then close (first vertex / double-click / Enter). **On a joint:** build a body *from joints* — click joints to outline, click a placed joint to finish, then move the cursor out to set the thickness and click. |
| **Joint** | `J` | Click inside a body to attach a joint; click where bodies overlap to drop one in each (pinned together); click **empty space** for a free, body-less joint. |
| **Connect** | `C` | Click a joint, then another joint on a different body to **pin** them — or click a **slider rail** to attach the joint to it as a rider. |
| **Ground** | `G` | Click a joint to lock its position (it can still rotate). Ground a free joint to make an anchor. |
| **Slider** | `S` | Click two joints on the **same body** (a moving rail), or **two free joints** (a world-fixed track — they get grounded automatically), to create a slider rail. Attach riders later with Connect. |

**Select mode** (no tool active, the default): click a body, joint, or slider rail to select it.
**Drag** the selection to move it. A selected body shows **corner handles** — drag one to reshape
it, and press **`[` / `]`** to decrease / increase its corner radius (this is how you round a
freehand polygon: draw it, select it, press `]`). Press **Delete** to remove the selection:
a body takes its joints and constraints with it; a slider rail leaves its joints; a joint detaches
from any rail.

Joints are color-coded: **blue** = pinned, **yellow** = grounded, **green** = slider rider;
rail-defining joints get a **green ring**, and a **loose free joint a dashed ring**. Once a free
joint is attached to a slider it's no longer loose, so it drops the dashed ring and shows as a
normal (green) rider.

**Undo / redo:** `Ctrl/Cmd+Z` undoes, `Ctrl/Cmd+Shift+Z` (or `Ctrl/Cmd+Y`) redoes — covering edits to the drawn layout.

### Simulate
Drag any joint. It becomes the *driving joint*: its body follows the cursor and every
connected body moves with it. Structural constraints always win over the cursor — a grounded
body can only rotate about its ground point, and the dragged joint snaps to the nearest point
it can actually reach. Your drawn layout is preserved when you switch back to Draw.

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
npm test         # headless tests: solver, persistence, body building, shape editing
```

## How it works

A small **iterative position-based solver** (Gauss-Seidel projection) satisfies the
constraints. Each constraint participant is reduced to a uniform "host" — a rigid body, a free
joint (a movable point), or a fixed world anchor — so pins, grounds, sliders and the mouse
driver all share one routine. After driving, the solver keeps sweeping the structural
constraints until the worst error is below a tolerance (capped), so complex or closed-loop
mechanisms converge tightly instead of drifting. The driver is step-limited and yields to
structural constraints, keeping dragging stable even when you pull toward a point the mechanism
can't reach. Sliders are prismatic constraints with end-stops; the rail is either a body (which
moves) or a world-fixed line built from two grounded free joints. Body outlines are
generated from a control polygon + corner radius (rounded corners via fillet or outward offset).

Source lives in [`src/`](src/): `geometry.ts`, `model.ts`, `solver.ts`, `view.ts` (camera),
`renderer.ts`, `main.ts`. Tests live in [`scripts/`](scripts/).

## License

See [LICENSE](LICENSE).
