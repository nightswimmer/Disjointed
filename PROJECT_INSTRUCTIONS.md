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

## Current status
End-to-end draw → simulate works, with editing (select/delete elements), one-shot tools +
keyboard shortcuts, body-to-body sliders (rail + riders, with end-stops), and a
converge-to-tolerance solver. The constraint solver is verified by headless tests; the
interactive canvas (drawing/dragging) should be confirmed by eye via `npm run dev`.

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
    fixed world point), `slider` (a rail = two joints `railA`/`railB` on one body, plus a
    `riders` list of joints on other bodies confined to the segment between them, with
    end-stops). The rail moves with its body, so a slider couples two bodies.
  - Helpers: hit-testing (`bodyAt`, `bodiesAt`, `jointAt`, `sliderAt`), `moveBody` (carries a
    body's ground anchors), `removeBody`/`removeJoint`/`removeConstraint` + `pruneConstraint`
    (deleting a rail joint kills the slider; deleting a rider just detaches it),
    `attachSliderRider`, pose snapshot/restore, role queries.
  - `serialize()` / `load(SceneData)` for save / load / autosave (versioned plain-data
    snapshot, `FORMAT_VERSION = 3`; `load` deep-copies, recomputes `nextId`, drops legacy
    origin+dir sliders, and migrates the older single-`slider` field to `riders`).
- **solver.ts** — `solve(scene, driver, iterations, relax)`. Per sweep it projects each
  structural constraint (effective-mass positional impulses), then applies the optional
  mouse driver. See "Solver notes" below.
- **view.ts** — camera transform `screen = world * scale + (tx, ty)`; `screenToWorld`,
  `worldToScreen`, cursor-anchored `zoomAt` (scale clamped to MIN_SCALE..MAX_SCALE = 0.2..5).
- **renderer.ts** — draws under the camera transform in world space: world-locked grid,
  bodies (selected/hovered highlighted), slider rails as bounded segments with end-caps,
  ground symbols, joints (color-coded: blue = pinned, yellow = grounded, green = slider rider;
  rail joints get a green ring), plus draft overlays. Cosmetic sizes (joint radius, line
  widths, ground symbol) are divided by the zoom so they stay constant on screen.
- **main.ts** — canvas/DPI setup, toolbar wiring, mode/tool state (tools are one-shot + have
  keyboard shortcuts), select-mode selection/deletion, the camera, pointer + key handling,
  persistence (save/load/autosave), and the requestAnimationFrame render/solve loop. Also a
  `timedSolve` debug helper that logs each solve's duration to the console.

### Interaction model
Draw-mode tools are **one-shot**: arming a tool (toolbar button or first-letter shortcut —
`B`/`J`/`C`/`G`/`S`) lets you place one element, then it returns to **Select** mode. `Esc`
aborts the current placement.
- **Body** (`B`) — click to add vertices; click the first vertex or double-click (or Enter)
  to close; Esc cancels.
- **Joint** (`J`) — click inside a body to attach a joint. Clicking where bodies overlap drops
  a joint in each overlapping body and pins them all together.
- **Connect** (`C`) — click a joint, then another joint on a *different* body to pin them, or a
  *slider rail* to attach the joint to it as a rider.
- **Ground** (`G`) — click a joint to lock its world position (body can still rotate about it).
- **Slider** (`S`) — click two joints on the *same body* to create a slider rail (riders are
  attached later via Connect).

Select mode (default, no tool armed):
- Click a body, joint, or slider rail to select (highlighted); `Delete` removes it. Removing a
  body deletes its joints/constraints; removing a slider keeps its joints; removing a joint
  detaches it from any rail.

Simulate mode:
- Drag any joint. It becomes the driver; its body follows the cursor and connected bodies
  move with it. Entering sim snapshots all poses and runs a settle solve; leaving sim
  restores the drawn layout so editing is non-destructive.

Navigation (both modes):
- **Mouse wheel** zooms toward the cursor.
- **Right-drag on empty space** pans the view.
- **Right-drag on a body** moves that body (its joints, and in draw mode its ground anchors,
  move with it). In simulate mode this is non-destructive — the drawn layout is restored on
  exit, and anchors are not moved.

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
- The driver is **step-limited** (`DRIVER_MAX_STEP`): it pulls the joint toward the cursor by
  at most a small amount per sweep. This keeps the linearized correction valid, so an
  unreachable target makes the joint walk stably along its feasible path to the nearest
  reachable point instead of overshooting.
- **Slider** = body-to-body prismatic with end-stops. `solveAxis` applies a scalar impulse
  along a direction fixed in the rail body's frame; the slider runs it for the perpendicular
  (stay on the line) plus a one-sided tangential limit at each rail endpoint. The rail-body
  angular Jacobian reduces to `cross(u, pQ − posR)`; with the rail body grounded it degenerates
  to a fixed-line constraint.

### Tests (`scripts/`, run with `npm test`, executed via tsx)
- **solver-smoke.ts** — grounded slider-crank (coupler riding a grounded rail) driven through
  a full revolution; asserts ground / pin / slider stay satisfied (sub-micron). Plus an
  end-stop scene: a rider driven far past each rail end is clamped to the endpoints.
- **ground-drag.ts** — drags a joint on a grounded body to far/off-axis/unreachable targets;
  asserts the ground never moves and the joint snaps to the nearest reachable angle.
- **persistence.ts** — round-trips a scene through `serialize → JSON → load`; asserts counts,
  exact joint positions, constraint kinds, slider riders, `nextId` continuation, deep-copy
  independence, and rejection of malformed data.

## Bugs found & fixed so far
- **Slider correction sign was flipped** (`-c/w` → `c/w`); sliders pushed joints away from
  their rail and the solver diverged. Under-relaxation only masked it.
- **Driver could drag a ground point away** when the target was unreachable. Fixed with
  strict structural priority + a step-limited driver.
- **Pins drifted on complex mechanisms** because the solver ran a fixed sweep count. Fixed by
  converging structural constraints to a tolerance instead.
- **Ground symbol stayed behind when moving a body** in draw mode. Fixed: `moveBody` carries
  the body's ground anchors.

## Backlog / next steps (not yet built)
- Move/edit individual vertices of an existing body.
- More joint types as needed.
- Optional: File System Access API for true "re-open the last file by path" (Chromium only).
  Current persistence is download/upload + localStorage autosave (restores the last session).
- Visual confirmation of multi-ground bodies (should be fully locked) — untested interactively.

## Working conventions (from CLAUDE.md)
- Answer bug/feature questions first; confirm before changing code.
- Remind to push to GitHub before big structural changes.
- On finishing / commit: update this file and README.md, then summarize and suggest a
  commit message (the user runs the commit).
