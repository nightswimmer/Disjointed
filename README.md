# Disjointed

A simple web app for creating and simulating **2D planar mechanisms** — polygon bodies
coupled by joints (pins, grounds, sliders) that you can then drag and watch move.

> Status: first working version. Draw a mechanism, switch to simulate, and drag a joint to
> drive it. The constraint solver is covered by headless tests.

## Concepts

- **Body** — a rigid polygon. Joints placed on the same body stay rigid relative to each other.
- **Joint** — a point attached to a body.
- **Constraints**
  - **Pin** — connect two joints on different bodies; they share a position but can rotate freely.
  - **Ground** — lock a joint's position; its body can still rotate about it.
  - **Slider** — a rail defined by two joints on one body. Joints attached to it (riders) slide
    along the segment **between** those two joints, with hard stops at each end. The rail moves
    with its body, so it couples two bodies (put the rail on a grounded body for a fixed track).

## Usage

There are two modes, switched from the toolbar.

### Draw
Tools are **one-shot**: pick a tool (or press its shortcut), place one element, and you return
to **Select** mode. Press **Esc** to abort the current placement.

| Tool | Shortcut | Action |
| --- | --- | --- |
| **Body** | `B` | Click to add vertices. Click the first vertex or double-click (or press Enter) to close. Esc cancels. |
| **Joint** | `J` | Click inside a body to attach a joint. Click where bodies overlap to drop a joint in each and pin them together. |
| **Connect** | `C` | Click a joint, then another joint on a different body to **pin** them — or click a **slider rail** to attach the joint to it as a rider. |
| **Ground** | `G` | Click a joint to lock its position (it can still rotate). |
| **Slider** | `S` | Click two joints on the **same body** to create a slider rail. Attach riders later with Connect. |

**Select mode** (no tool active, the default): click a body, joint, or slider rail to select it;
press **Delete** to remove it. Deleting a body removes its joints and their constraints; deleting
a slider rail leaves the joints in place; deleting a joint detaches it from any rail it rode.

Joints are color-coded: **blue** = pinned, **yellow** = grounded, **green** = slider rider;
rail-defining joints get a **green ring**.

### Simulate
Drag any joint. It becomes the *driving joint*: its body follows the cursor and every
connected body moves with it. Structural constraints always win over the cursor — a grounded
body can only rotate about its ground point, and the dragged joint snaps to the nearest point
it can actually reach. Your drawn layout is preserved when you switch back to Draw.

### Navigate
- **Mouse wheel** — zoom toward the cursor.
- **Right-drag empty space** — pan the view.
- **Right-drag a body** — move that body around (its ground anchors move with it).

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
npm test         # headless constraint-solver smoke test
```

## How it works

A small **iterative position-based solver** (Gauss-Seidel projection) satisfies the
constraints. Each body has a pose (centroid position + angle); each constraint nudges the
poses with effective-mass positional impulses. After driving, the solver keeps sweeping the
structural constraints until the worst error is below a tolerance (capped), so complex or
closed-loop mechanisms converge tightly instead of drifting. The mouse driver is step-limited
and yields to structural constraints, which keeps dragging stable even when you pull toward a
point the mechanism can't reach. Sliders are body-to-body prismatic constraints with end-stops.

Source lives in [`src/`](src/): `geometry.ts`, `model.ts`, `solver.ts`, `view.ts` (camera),
`renderer.ts`, `main.ts`. Tests live in [`scripts/`](scripts/).

## License

See [LICENSE](LICENSE).
