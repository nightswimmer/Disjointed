# Disjointed — Project State

## Goal
A simple webapp to create and simulate 2D planar mechanisms (moving joints / linkages).
Two modes:
- **Drawing mode** — draw polygon objects, place joint points on them, and couple them
  with constraints (rotation/pin, ground, slider). Joints on the same object are rigid
  relative to each other.
- **Simulation mode** — pick a joint and drag it (the "driving joint"); the body it
  belongs to moves and a constraint solver propagates the motion through everything
  connected to it.

## Current status — first working vertical slice
End-to-end draw → simulate works. The constraint solver is verified by headless tests.
The interactive canvas (drawing/dragging) should be confirmed by eye via `npm run dev`.

### Tech stack
- **Vite + TypeScript + HTML5 Canvas** (no UI framework). Builds to static files.
- Constraint solver: **iterative position-based** (Gauss-Seidel projection).
- Node 24 / npm. Scripts: `npm run dev`, `npm run build`, `npm run preview`, `npm test`.

### Architecture (`src/`)
- **geometry.ts** — Vec2 math; polygon centroid / area / inertia; point-in-polygon;
  point-to-line distance.
- **model.ts** — `Scene` owning:
  - `Body` = rigid polygon. Pose is `pos` (world centroid) + `angle`. Vertices are stored
    in a local frame relative to the centroid. `invMass` / `invInertia` come from polygon
    area / second moment (density 1).
  - `Joint` = a point attached to a body, stored as a local-frame offset from the centroid.
  - Constraints: `pin` (two joints coincide, free rotation), `ground` (a joint locked to a
    fixed world point), `slider` (a joint confined to a fixed world line).
  - Helpers: hit-testing (`bodyAt`, `jointAt`), pose snapshot/restore, role queries.
- **solver.ts** — `solve(scene, driver, iterations, relax)`. Per sweep it projects each
  structural constraint (effective-mass positional impulses), then applies the optional
  mouse driver. See "Solver notes" below.
- **renderer.ts** — draws grid, bodies, slider rails, ground symbols, joints (color-coded:
  blue = pinned, yellow = grounded, green = slider), plus draft overlays.
- **main.ts** — canvas/DPI setup, toolbar wiring, mode/tool state, pointer + key handling,
  and the requestAnimationFrame render/solve loop.

### Interaction model
Draw-mode tools:
- **Polygon** — click to add vertices; click the first vertex or double-click (or Enter) to
  close; Esc cancels.
- **Joint** — click inside a body to attach a joint point to it.
- **Connect** — click two joints on *different* bodies to pin them together.
- **Ground** — click a joint to lock its world position (body can still rotate about it).
- **Slider** — click a joint, then click again to set the direction of its rail.

Simulate mode:
- Drag any joint. It becomes the driver; its body follows the cursor and connected bodies
  move with it. Entering sim snapshots all poses and runs a settle solve; leaving sim
  restores the drawn layout so editing is non-destructive.

### Solver notes (important design decisions)
- Each constraint is satisfied with effective-mass positional impulses:
  `pos += invMass·λ`, `angle += invInertia·cross(r, λ)`.
- **Structural constraints take strict priority over the mouse driver.** Each drag solve
  ends with structural-only cleanup sweeps (`CLEANUP_SWEEPS`) so ground/pin/slider hold
  exactly — the driver can never drag a ground point (or break a pin/slider) away.
- The driver is **step-limited** (`DRIVER_MAX_STEP`): it pulls the joint toward the cursor by
  at most a small amount per sweep. This keeps the linearized correction valid, so an
  unreachable target makes the joint walk stably along its feasible path to the nearest
  reachable point instead of overshooting.

### Tests (`scripts/`, run with tsx)
- **solver-smoke.ts** (`npm test`) — grounded slider-crank driven through a full revolution;
  asserts ground / pin / slider stay satisfied (sub-micron).
- **ground-drag.ts** — drags a joint on a grounded body to far/off-axis/unreachable targets;
  asserts the ground never moves and the joint snaps to the nearest reachable angle.

## Bugs found & fixed so far
- **Slider correction sign was flipped** (`-c/w` → `c/w`); sliders pushed joints away from
  their rail and the solver diverged. Under-relaxation only masked it.
- **Driver could drag a ground point away** when the target was unreachable. Fixed with
  strict structural priority (cleanup sweeps) + a step-limited driver.

## Backlog / next steps (not yet built)
- Edit/delete tools (remove bodies, joints, constraints; move vertices).
- Body-to-body sliders (currently sliders are grounded joint-to-fixed-line only).
- More joint types as needed.
- Save / load mechanisms (e.g. JSON), pan / zoom of the canvas.
- Visual confirmation of multi-ground bodies (should be fully locked) — untested interactively.

## Working conventions (from CLAUDE.md)
- Answer bug/feature questions first; confirm before changing code.
- Remind to push to GitHub before big structural changes.
- On finishing / commit: update this file and README.md, then summarize and suggest a
  commit message (the user runs the commit).
