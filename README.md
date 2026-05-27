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
  - **Slider** — confine a joint to a straight rail.

## Usage

There are two modes, switched from the toolbar.

### Draw
| Tool | Action |
| --- | --- |
| **Polygon** | Click to add vertices. Click the first vertex or double-click (or press Enter) to close. Esc cancels. |
| **Joint** | Click inside a body to attach a joint point. |
| **Connect** | Click two joints on different bodies to pin them together. |
| **Ground** | Click a joint to lock its position (it can still rotate). |
| **Slider** | Click a joint, then click again to set the direction of its rail. |

Joints are color-coded: **blue** = pinned, **yellow** = grounded, **green** = on a slider.

### Simulate
Drag any joint. It becomes the *driving joint*: its body follows the cursor and every
connected body moves with it. Structural constraints always win over the cursor — a grounded
body can only rotate about its ground point, and the dragged joint snaps to the nearest point
it can actually reach. Your drawn layout is preserved when you switch back to Draw.

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
poses with effective-mass positional impulses until the mechanism is consistent. The mouse
driver is step-limited and yields to structural constraints, which keeps dragging stable even
when you pull toward a point the mechanism can't reach.

Source lives in [`src/`](src/): `geometry.ts`, `model.ts`, `solver.ts`, `renderer.ts`,
`main.ts`. Solver tests live in [`scripts/`](scripts/).

## License

See [LICENSE](LICENSE).
