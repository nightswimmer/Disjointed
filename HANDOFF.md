# HANDOFF — Next feature: CAD-style sketch constraints in draw mode

This file captures the agreed direction and open questions for the **next** development
session. Read PROJECT_INSTRUCTIONS.md first for the current state of the project (the
measurements feature this builds on has just been committed).

## Goal (user's words, condensed)

Make draw mode feel like sketching in a CAD program:

- **Driving vs driven dimensions.** In draw mode a dimension can be *driving* — setting its
  value moves geometry to satisfy it. In simulate mode all dimensions are *driven*
  (read-only): the bodies are already fixed shapes.
- **A small set of typical CAD constraints** applicable to lines and points:
  **coincident, parallel, horizontal, vertical, perpendicular, equal**.
- **Dimension-edit behaviour:**
  - Setting the *first* dimension on a body **scales the whole body uniformly** (same form
    factor).
  - Setting a dimension on a body that *already has other dimensions* moves **only the nodes
    involved**, while keeping all defined constraints satisfied.
- **Auto-constraints:** some constraints are applied automatically while drawing (e.g. a
  near-horizontal / near-vertical drawn edge gets an H/V constraint).

## Proposed design (discussed, not yet confirmed in every detail)

### Data model (serialization v8)

- **Sketch constraints** stored in the scene alongside mechanism constraints, but they live
  purely in draw mode:
  - `coincident` (point–point)
  - `horizontal` / `vertical` (a line ref, or a point pair)
  - `parallel`, `perpendicular`, `equal` (line–line; `equal` = equal length)
- Points and lines **reuse the measurement reference system** (`MeasureRef` in model.ts:
  joints, body control vertices, body edges, slider rails), so constraints track their
  elements the same way measurements do — including the existing index-remap on control
  vertex insert/remove and prune-on-delete behaviour.
- **Dimensions:** draw-mode `Measurement`s gain `driving: boolean` + a target value. A
  driving dimension acts as a constraint; a driven one stays a read-only reference (display
  it differently — CAD convention is the value in parentheses). Sim-mode measurements stay
  driven-only, untouched.

### Sketch solver (new `src/sketch.ts`)

- Iterative projection (Gauss-Seidel style, same philosophy as `solver.ts`) over the world
  positions of **body control vertices and free joints** — the variables are *shape* nodes,
  not rigid poses.
- After each solve, bodies rebuild from their new control polygons (the existing
  `rebuildBody` path keeps attached joints anchored; the node↔joint coincidence link already
  behaves like a built-in coincident constraint).
- Convergence failures are reported like the sim solver's `ConstraintBreak`s → offending
  items paint red.

### Dimension-edit semantics

- Double-click a draw-mode dimension label → inline numeric input → typing a value makes it
  driving and triggers a sketch solve.
- If the dimension's references live on one body with **no other driving dimensions and no
  external constraints**: uniform scale of the whole body (control polygon + its joints +
  ground anchors) so the dimension hits the target.
- Otherwise: solve moves only the involved nodes, holding every other constraint and driving
  dimension fixed.
- On an unsatisfiable edit: **reject** (revert the edit, flash the conflicting items red)
  rather than leave a broken sketch. (Pending confirmation — question 4 below.)

### Auto-constraints while drawing

- A freehand polygon edge drawn within a few degrees of horizontal/vertical → automatic H/V
  constraint.
- Clicking on an existing node/joint while drawing → coincident.
- Small glyphs on the canvas show applied constraints (CAD-style ⊥ / ∥ / = badges),
  click-to-select, Delete to remove.

### Phasing (each phase ends working + tested)

1. **Model + solver:** constraint types, sketch solver, driving-dimension solve,
   scale-on-first-dimension — headless, fully test-covered (`scripts/` test in the npm test
   chain).
2. **UI:** constraint tools in the toolbar, glyph rendering/selection/deletion, inline
   dimension value editing.
3. **Sketch-aware dragging + auto-constraints:** select-mode drags solve constraints live;
   H/V and coincident inference while drawing.

## Open questions (answers pending — ask before starting phase 1)

1. **Sketch scope** — one canvas-wide sketch (constraints allowed across bodies, and between
   bodies and rails/free joints), or strictly per-body?
   *Recommendation: canvas-wide — simpler conceptually, and enables "this edge parallel to
   that rail".*
2. **Dragging** — should select-mode drags (nodes, joints, bodies) solve constraints live,
   CAD-style? Biggest behavioural change to draw mode, but it's what makes it "feel like a
   sketch". *Recommendation: yes, as phase 3.* The alternative: constraints enforced only
   when dimensions/constraints are edited, drags free to violate them.
3. **Driving angles** — can angle dimensions (between two lines) be driving too, or
   distances only for v1? *Recommendation: distances first, angles as a follow-up.*
4. **Conflicts / over-constraining** — reject the edit (revert + flash red) vs best-effort
   solve. *Recommendation: reject.*
5. **Scale anchor** — when the first dimension scales a whole body, scale about its
   **centroid** (body stays put) or about the dimension's first reference point (that point
   stays fixed)? *Recommendation: centroid.*

## Relevant code pointers

- `src/model.ts` — `MeasureRef` / `Measurement` / `MeasureInfo`, `resolveMeasureRef`,
  `measureInfo`, `shiftMeasureIndices` (vertex-index remapping), `pruneMeasurements`,
  `rebuildBody`, the node↔joint link (`VERTEX_LINK_EPS`). Serialization is at
  `FORMAT_VERSION = 7`; this feature bumps to 8.
- `src/main.ts` — measure tool (`measureRefAt`, `handleMeasureClick`,
  `measurementLabelAt`, label drag via `leftDrag.kind === "measureLabel"`), one-shot tool
  pattern, undo/redo snapshots (`markDirty`).
- `src/renderer.ts` — `drawMeasurement` (dimension lines/arcs/labels),
  `drawMeasureRefHighlight`; constraint glyphs would join this pass.
- `src/solver.ts` — the sim-time mechanism solver (untouched by this feature, but the
  sketch solver should mirror its converge-to-tolerance + break-reporting design).

## Known measurement limitations (noted at hand-off, may fold into this work)

- Mirroring a body doesn't remap vertex/edge/bodyPoint measurement refs (they can end up
  pointing at un-mirrored spots).
- Copy/paste doesn't carry measurements (consistent with actuators/motors today).
