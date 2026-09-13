# Disjointed — Project State

Session-to-session handover notes for Claude. Keep this file to what is **not** derivable from the
code, git history, README.md or the manual: goals, design decisions and their reasons, invariants,
pitfalls, and what is in flight. Anything else belongs in code comments or README.md.

Where things are documented:
- **README.md** — the complete user-facing reference (every tool, gesture, shortcut, panel). When a
  behaviour question comes up, read it there; do not duplicate it here.
- **public/help/** — the in-app manual (same content as README, illustrated). Its updates are
  currently **deferred** (see *In flight*).
- **HANDOFF.md** — the agreed roadmap for shape tools (phases 2–4), smaller follow-ups, and the
  list of pending manual edits.
- Code comments carry the local *why* for most non-obvious branches; `scripts/*.ts` are the
  executable spec.

## Goal
A single-page webapp to draw and simulate 2D planar mechanisms. **Draw mode**: bodies (editable
control polygon + corner rounding), joints (attached or free), constraints (pin / weld, ground, rail
+ riders / sliders, actuators, motors), CAD-style sketch constraints and dimensions, groups,
reusable components, reference geometry, patterns; DXF import and DXF/SVG cut-file export.
**Simulate mode**: drag any joint or body point; a position-based constraint solver propagates the
motion; actuators / motors animate.

## Tech stack & commands
- Vite + TypeScript + HTML5 Canvas, no UI framework, no runtime dependencies. Node 24.
- `npm run dev` / `build` / `preview`; `npm test` runs every `scripts/*.ts` (tsx, headless, no DOM);
  `npm run manual` regenerates the manual's SVG illustrations + glyph tables with Playwright driving
  the **installed Chrome** (`channel: "chrome"`), and fails if a topic the app can ask for is missing.
- The interactive canvas is not covered by tests: confirm UI changes by eye (or with a throwaway
  Playwright script against `npm run dev` with `?automation`, which exposes `window.__disjointed`).
- Field-repro scenes in the repo root: `FrontPanelHinge.json`, `gate hinge tests.json` (the
  "Door Assembly 6" pattern/dimension case).

## Module map (`src/`)
| Module | Role |
|---|---|
| `geometry.ts` | Vec2, polygon properties, fillet (`filletPolygon` / `filletCornerArcs`), offset hulls, regular polygons, arcs |
| `model.ts` | `Scene`: bodies, joints, constraints, groups, components, guides, patterns, measurements, sketch constraints; serialization (`FORMAT_VERSION = 21`); all editing primitives |
| `solver.ts` | Sim solver (Gauss-Seidel positional impulses), groups/welds as rigid composites, break-and-exclude |
| `sketch.ts` | Draw-mode shape solver for sketch constraints + driving dimensions |
| `pose.ts` | Pose-level dimensions / constraints on component instances (rigid moves, not shape) |
| `boolean.ts` | Polygon union (Combine) and difference (Cut role) over a planar graph |
| `context.ts` | Context ghost: the enclosing assembly drawn faded inside a definition |
| `analyzer.ts` | Topology diagnostic (islands, DOF, loop cores). Not wired to any UI |
| `dxf.ts` / `export.ts` | DXF reader with fillet reconstruction; DXF R12 / SVG cut-file writer with exact arcs |
| `renderer.ts` | Canvas drawing from a `RenderInput`; theme palette; screen-space labels |
| `view.ts` | Camera (`screen = R(angle)·world·scale + t`) |
| `main.ts` | Everything UI: tools, drags, snapping, selection, history, persistence, backup, component contexts, animation loop |
| `notify.ts` | Toasts — the project-wide replacement for `alert` |
| `filestore.ts` | File System Access API wrappers + IndexedDB handle storage |
| `toolbar.ts` | Toolbar section rack: drag a group by its caption to reorder (live FLIP reflow), order in localStorage |
| `help.ts` / `helpmap.ts` | Help drawer + help mode; DOM-free UI→topic map shared with the manual generator |
| `automation.ts` / `svgcontext.ts` | Playwright hook; canvas-API-shaped SVG recorder (manual illustrations) |
| `public/help/`, `scripts/manual/` | The manual and its generator (`shoot.ts`, `shots.ts`, `fixtures.ts`) |

## Domain model — semantics worth remembering
- **Body**: pose = centroid `pos` + `angle`; shape = `controlLocal` + `radius` (+ per-corner
  `radii`, `round: "fillet" | "offset"`). The rendered polygon is derived. Holes are editable
  outlines of the same kind; `holesLocal` is derived. A 1-point offset outline is a **disk**
  (parametric); `regular` marks a regular polygon kept as an invariant.
- **Joint**: `bodyId` + `local`; `bodyId === null` = free joint (a point particle).
- **Pin** (`rigid` = weld), **ground**, **slider** = a rail (`railA`/`railB`) + `riders`, with
  `locked` (orientation-locked = "slider" in the UI) and `startRiders` (home at `railA`) sublists.
  A rail is either two joints on one body or two free joints (auto-grounded). Powered:
  `linearActuator` (rider on a rail), `motor` (pivot + crank on one body) — inert off-animation,
  driven through the solver's `anchors` map (treated exactly like moving grounds).
- **Groups** (`BodyGroup { bodyIds, jointIds }`): one rigid body in sim, move together in draw.
- **Components**: `ComponentDef { data: SceneData }` (a DAG, cycles rejected); instances are
  **materialized** real bodies/joints/constraints tagged with provenance maps, so solver, renderer
  and hit-testing know nothing about hierarchy. Grounding inside a def = "fixed to the component
  frame" and expands into a rigid **chassis group**, never a world ground.
- **Guides** (reference geometry): union of `poly` / `circle` / `arc` / `text`, all finite; never
  simulated, invisible in sim; usable as measurement / constraint refs.
- **Patterns**: seed hole/joint + layout; members are real holes/joints **re-derived** from the seed
  on every change.
- **Measurements** and **sketch constraints** share `MeasureRef` (joint, vertex, edge, bodyPoint,
  rail, guidePoint, guideLine, patternAxis, centre; vertex/edge refs carry an optional `hole`).
  Refs always name **elements**, never coordinates. Dimensions may be `driving` (with `target` and
  a held `side`).
- The `bodies` array **is** the z-order (drawn first→last, picked last→first).

## Design decisions and invariants (with reasons)

### Sim solver
- **Converge to tolerance, never a fixed sweep count** — fixed counts drifted on closed loops.
- **Grounds are sacred**: projected exactly every sweep, never disabled; any grounded joint is a
  fixed host for whatever pins to it (`pinHostFor`), so a pin can't drag the body a grounded joint
  sits on. Grounded bodies and frozen material are immovable fixed hosts.
- **Structural constraints outrank the mouse driver**; the driver is step-limited so unreachable
  targets walk stably to the nearest reachable point.
- **Impossible assemblies → break-and-exclude**: Phase A solves; if unconverged, Phase B disables
  the worst non-ground units until the rest converges, Phase C closes them as far as freedom allows,
  and the breaks are reported (red dotted lines, red joints, banner). Runs only for unconverged scenes.
- **Phase-A early exit only when no driver is present** — with a driver the step limit keeps the
  residual ≈ 0 and an early exit starves the drag.
- **Groups and welds are rigid composites**, not iterated constraints: `mergeWeldComposites`
  union-finds weld-connected units into one `RigidGroup` per solve with exact snap-assembly at the
  drawn angle. Two earlier weld designs (phantom second pin; per-sweep angular projection) converged
  one link per sweep and caused false breaks / 100 % CPU on chains — don't go back. Intra-unit pins
  and riders are inert. Impossible welds surface via `weldConflictBreaks`, never Phase B.
- **Slider locks** use the **wrapped** relative angle (unique zero) — the old `sin`-based error was
  also zero at a 180° flip.
- **"As drawn" is the reference**: slider-lock and weld baselines are captured from the drawn pose
  at sim entry / rigid-drag start (`resetPoseBaselines` on every `markDirty`); no stored angle.
- **Scoped solves** (`SolveFreeze`) power the rigid Shift-drag: frozen material is immovable and
  constraints entirely inside the frozen world are out of scope (not solved, not reported).
- Drag-yield caveat: "a drag never breaks constraints" is enforced by convergence, not by
  construction. If a slow-converging mechanism ever false-breaks under a drag, the candidate
  hardening is to skip Phase B for pure mouse-driver solves.

### Sketch solver (draw mode)
- Variables are **world positions** of control vertices, joints, guide points and regular-polygon
  centres; a joint coincident with a control vertex maps onto that vertex (the node↔joint link).
- **Reject semantics**: an unconverged solve never touches the scene; the conflicting items flash
  red. Multi-pass write-back (`APPLY_PASSES`) re-solves from the applied state before rejecting,
  because the rigid carry itself can move items the iteration left satisfied.
- **Mobility ranks** decide who moves: 0 construction (guide points) < 1 geometry < 1.5 tied guides
  < 2 drag-anchored < 3 component-instance geometry (immovable — its shape belongs to the def).
  Corrections flow to the lowest rank. Consequences: a guide with one demand moves alone; a guide
  tied to geometry or carrying several demands becomes a reference (geometry moves to it); a
  guides-as-reference fallback pass runs when the construction-first pass fails.
- **Drags are anchored**; when the anchored solve is infeasible the live solve falls back to a
  symmetric solve **the same frame**, so constraints never look broken mid-drag.
- **Whole-body rigid carry** (`applyRigidParts`): when a solve moves every outer vertex, the body
  moves as a rigid unit so holes, joints and anchors ride along. Body-owned variables the solve left
  unchanged are mapped through the same motion.
- Distances off an H/V-constrained line shift along the exact world axis, not the momentary normal
  (a mid-sweep tilt otherwise leaks motion sideways).
- **Driving dimensions hold their drawn `side`** so a one-frame overshoot can't flip the two sides
  through each other (direct point–point distances have no side by nature).
- First driving dimension on an otherwise-unconstrained body **scales it uniformly**; later ones move
  nodes. Size dimensions (diameter, radius, regular-polygon size) are not solver items: they set the
  parameter directly and are re-applied by `enforceSizeDims`. A direct resize (rim drag, `[`/`]`)
  demotes conflicting driving size dims to driven.
- **Pattern members ride with their seed** through rigid-offset couplings (an immovable rank
  deadlocked any solve that needed a patterned body to shift).
- **Regular polygons are an invariant, not constraints**: rigid weighted fit per solve (a similarity
  fit let pinching corrections shrink it sweep after sweep); H/V/parallel/perpendicular on an edge
  turn it in one step; **a dimension never turns a regular polygon** (that solver item could stall
  and one stalled item fails every solve in the scene). Corner drags respect what the sketch leaves
  free (size driven → turn only; rotation locked → resize radially; both → move whole).
- Auto-constraints **while drawing**: near-H/V edges (±5°) get H/V; a vertex placed on a
  joint/corner/guide point gets a coincident. Always on (the switch below doesn't reach them).
- **Implicit constraints while dragging** (`DragAlign`, main.ts) arm after a 0.4 s hover and apply an
  exact alignment correction **before** placing the constraint — letting the solver close even a
  0.3 mm gap failed when the other side was dimension-pinned. Design points:
  - **Two candidates stay armed** (`ALIGN_MAX_CANDS`), so one drag can take a V off one reference
    and an H off another; a third hover drops the oldest, Esc drops the newest.
  - The correction is a **2-DOF linear solve**, not a per-kind special case: each match contributes
    `dot(n, delta) = d` (y axis for H, x for V, the line normal for point-on-line, **both** axes for
    a point-on-point coincident), one equation gives the perpendicular foot, two independent ones go
    through Cramer. A release therefore lands exactly on the intersection.
  - A match is kept only while the kept equations stay solvable: ≤ 2 total, and no two normals within
    `ALIGN_INDEPENDENT_TOL` (≈3°). Candidates are matched **newest first**, so a clash drops the
    older candidate's preview (it stays armed and returns when the drag stops matching the newer).
    A point-on-point coincident spends both DOF and so always previews alone.
  - Dropping a point **on** a candidate point is a coincident (it used to be "a placement, not an
    alignment"): you are on the candidate while hovering to arm it, so ◎ shows immediately.
  - Placement is per match, so one rejection doesn't lose the other.
  - `autoConstrain` (the Constraints group's switch) gates the whole thing at `newDragAlign` /
    `updateDragAlign`, so nothing is scanned when it's off.

### Components
- **The definition is the pose reference**: any def edit re-expands every instance (fixpoint through
  the DAG) and snaps every part's pose to `T · defPose`; only placement and the instance-level
  grounded flag are instance state. Internally posed mechanisms therefore reset on def edits — accepted.
- Instances carry shapes, not design constraints (def sketch / dims / guides stay in the def), so
  instances rotate freely — the motivating pain with H/V constraints.
- Components are a **transparent grouping for dimensioning and constraining**: one free end → the
  free side moves; every end instance-owned → a **pose** dimension / constraint (translation or
  rotation of a rigid part, second-picked side moves by preference, grounded never moves); ends
  rigid to one another at a deeper level → rejected. h/v pose dims measure **world** axes (accepted caveat).
- Mirroring an instance is a placement flag (`mirrored`), expanded as reflect → rotate → translate;
  motors negate speed; re-expansion of a mirrored instance is a no-op.
- Deleting a def with instances **converts them to plain bodies** (chassis group survives).
  Forking (⊞ on one instance) deep-copies the def byte-identically so no re-expansion is needed.
- The **context ghost** is inert by design: object-snap targets only (refs stripped to null so
  nothing can bind to it); temporary context dimensions live in main, are never serialized, and
  their one-shot move emulates a drag of the picked live feature.

### Shapes, holes, roles
- **Picking and joint containment use the outer outline only** (a joint may sit at a shaft hole's
  centre; clicking in a cut-out still selects the body). Attached joints can never be placed or
  dragged outside their body (clamp to the outline); a shape change that strands one is **flagged,
  never auto-moved**.
- **Shape roles are explicit and sticky** (Body / Cut / Reference; Ctrl on the first click flips
  Body↔Cut once) — never inferred from where a click lands, because bodies routinely overlap.
- **Cut** = polygon difference: fast path (cutter fully inside, clear of holes) keeps the exact spec
  (disks stay parametric); the general path (`applyRegion`, shared with Combine) keeps unchanged
  corners' radii, new corners sharp, remaps refs by world geometry, and dissolves the body's
  patterns. A cut that would split or remove the body is refused ("use Split").
- Line, arc and text are reference-only; text is for labels (cut/engraved text is phase 4).
- Offset-mode bodies are **baked** to fillet control polygons before Split / Combine / general cuts
  (their rounded shape is larger than their control polygon).
- Fillet: centre on the bisector of the two edge directions (correct for convex and reflex corners);
  shared-edge budget + opposite-edge clamp so fillets never overlap or fold thin shapes. Arc sampling
  is 7.5°/segment everywhere (matches the DXF importer's reconstruction tolerance).
- DXF import reconstructs fillets (tangent arcs < 180° → sharp corner + per-corner radius) and
  circles → parametric disks; export emits exact arcs (bulges) and round-trips through the importer.
- Guides are finite since v21 (the infinite guideline is gone); guide constraints never move
  geometry unless the guide is tied / multiply demanded (see ranks).

### UI conventions
- Tools are **one-shot** (Rotate is a mode; the polyline/body draft spans clicks). Toolbar wiring is
  by id / `data-*` / class, never button text **and never by position** — the user reorders the
  groups. All user-facing warnings go through `notify`.
- **Toolbar = a rack of draggable groups** (`#tb-sections > .tb-sec`, `src/toolbar.ts`): each is a
  two-row grid filled *column-major* (`grid-auto-flow: column`, so markup order reads down-then-
  across; `.tb-tall` / `.tb-mid` span or centre a lone control) under a caption strip that is also
  its drag handle. Dragging reorders the DOM live and FLIP-animates the others, deciding insertion
  on `offsetLeft`/`offsetTop` (layout values, immune to the drag transform and to a running
  slide); double-click a caption to reset. Per-mode visibility is the `draw-only` / `sim-only`
  classes, never a hard-coded group list. Selection- or tool-dependent fields live in `#tb-props`
  **outside** the rack so they can't reshuffle it; the armed-tool hint is the `#statusbar` line.
- **Group visibility toggles are eyes on the caption** (grid, constraint badges, dimensions) —
  the toggle applies to the whole group, and `.tb-cap .cap-eye` has to outweigh `button.active`
  or it inherits the accent fill. The mode switch is one big button showing the mode it switches
  *to* (the theme button's convention); its caption names the mode you are in.
- **A setting that lives among tools is a `.tb-switch`** (today: auto-constraints, `#autocon-btn`
  in the Constraints group): a `tb-tall` pill with a slider under its glyph. The pill stays
  **neutral when on** — it overrides `button.active`, because an accent-filled button means "this
  tool is armed" everywhere else — and the accent lives on the slider track instead.
- Every mutation goes through `markDirty` → snapshot history + autosave + component-context sync +
  pose-baseline reset. `canonicalData()` is the root document without sim poses.
- Session-only state (grid, snap, osnap, visibility toggles, solver tuning) is not persisted;
  theme, help-drawer width, grid presets, backup settings and the toolbar group order live in localStorage.
- Selection kinds: single `selection` vs `multiSel` (groups and instances are selection-atomic)
  vs `featureSel` (vertices + joints of one body). Ctrl+G is a group toggle; plain G is Ground.
- Two-click slider start pair: a press grabs the **rail joint** in draw mode, the **rider** in sim.
- Text shortcuts: plain letters in `TOOL_KEYS`, Shift+letter shape tools in `SHIFT_TOOL_KEYS`
  (main.ts); plain `L` is unbound; the whole shortcut map is due for a remap.
- View rotation is purely visual (world axes for constraints, grid and snapping).

## Serialization history (`load` accepts everything ≤ 21)
v5 control polygons · v6 actuators/motors · v7 measurements · v8 sketch constraints + driving ·
v9 groups · v10 grounded bodies · v11 guides · v12 units · v13 holes (baked) · v14 components +
group joints · v15 per-corner radii · v16 editable holes · v17 locked riders · v18 welds ·
v19 patterns · v20 guide union + `startRiders` · v21 regular polygons, infinite guideline dropped.
Optional fields added without a bump: `Measurement.side`, `mirrored`, `rigid`. Load sanitizes
every list invariant (locked ⊆ riders, pattern members exist, mismatched regular counts dropped).

## Tests (`scripts/`, one line each)
solver-smoke (slider-crank + end-stops) · free-rail · ground-drag · impossible-assembly ·
persistence · build-body · shape-edit (fillet, containment, node↔joint link, radii, holes) ·
edit-utils (rotate/mirror/copy/z-order) · actuators · measurements (incl. diameter/radius) ·
sketch (constraints, dims, ranks, rigid carry, drift) · groups · group-joints · grounded-bodies ·
freeze-drag · slider-locks · two-click-slider · welds (incl. chain regression with solver stats) ·
components · pose-dims · pose-constraints · split-combine · shapes (cut, references, v21 load) ·
regular · guides · patterns · features · context-ghost · view · dxf · export.
`solver-bench.ts` is a benchmark, not in `npm test`. Each script is a plain assertion list — read
it for the exact cases.

## Lessons from past bugs (root causes to not repeat)
- Index-based refs must be remapped on every reordering of a control polygon (mirror reverses
  winding; insert/remove shifts; Split/Combine/Cut rebuild) — and keep the `hole` field.
- Anything that moves a body as a whole must carry holes, joints, ground anchors and owned rail
  tracks (`moveBody` / `rotateBody` / `mirrorBody` do; per-vertex writes don't).
- Hit tests on rails use the **segment**, not the infinite line.
- Never split a correction 50/50 with a dragged element and never keep a stored drag anchor: read the
  live scene each frame (the group-drift bug).
- A single stalled solver item rejects the whole scene's sketch — prefer invariants / direct
  parameter setting over items that can stall.
- Gauss-Seidel propagates one link per sweep: chains of pairwise constraints must become composites.
- Errors with non-unique zeros (`sin`) allow flipped poses to read as satisfied.
- Guards that live only in one entry point (the instance-ownership check that lived in
  `setMeasurementDriving`) get bypassed — validate at the model / solver boundary.
- Solver instrumentation exists (`solverConfig`, `SolveStats`, sim-mode tuning sliders, per-run
  animation stats in the console); use it before guessing at convergence problems.

## In flight (2026-09-13)
- A run of **UI-tweak commits**. During it the in-app manual is deliberately **not** updated;
  every manual-relevant change is logged in HANDOFF.md ("pending manual updates") for one later pass.
  Currently pending there: guideline removal, parametric polygons and the "n sides" tag,
  projected corner-pair size dimensions, single-click line dimensions, the two-candidate /
  point-on-point implicit constraints with their new toolbar switch, and stale what's-this strings.
- Plain `L` is unbound; a full shortcut remap is planned.
- Shape-tools phase 1 shipped (v20/v21); phases 2–4 and follow-ups are in HANDOFF.md.
- The toolbar was regrouped into draggable groups for **draw mode**; sim mode inherits the rack
  but its own grouping (Animation + solver tuning) has not been designed yet — see HANDOFF.md.

## Backlog (not built)
- **Shape tools phases 2–4** (HANDOFF.md): bulge arcs on outlines → arc edges, curved slots, exact
  arc export/import; ellipses (edge *kind*); regions from closed reference chains; cut/engraved text
  (stencil font). Smaller: role conversions, multi-body cuts, patterns surviving general cuts, text
  angle/font/alignment, point-on-circle / tangent / concentric constraints.
- **Auto-pause false positives**: still pauses on some solvable borderline closed loops. Next steps:
  use the tuning instrumentation to find the knob; wire `analyzer.ts` to a debug hook; prototype
  closed-form propagation for tree branches (loop cores stay iterative). Also the "Assembly
  impossible" flicker while dragging an actuator's rider in a closed loop (candidate: project the
  driver target onto the rail first).
- **Manual**: richer topic text, gesture-sequence illustrations (before/mid/after via `run` steps),
  PNG panel shots (path exists, no shots), pattern/component/ghost fixtures; option to let controls
  act *and* navigate in help mode (one-line change in `help.ts`). The draw-mode toolbar is ~1480 px wide and wraps groups below that.
- **Patterns**: "make independent" UI undecided (`dissolvePattern` exists); labels not draggable;
  whole-body patterns; patterns inside defs don't expand as patterns.
- **Components**: no per-instance scaling; no ports; per-instance actuator/motor speed overrides are
  lost on cascade; no thumbnails / drag-to-place in the browser; pose dims between instances aren't
  carried by copy/paste; a refused pose constraint gives no feedback (nothing to flash).
- **Sketch**: driving angle dimensions; radius dim on a sharp corner can't be picked. Implicit
  constraints have no keyboard shortcut for their switch, and no point-on-point coincident between
  a dragged *line* and a candidate (lines only take point-on-line).
- **Context ghost**: can't switch reference instance; no hotkey for the eyes; ghost drops parent
  guides; temp dims are per session.
- **Sliders / welds**: Connect tool doesn't create welds; second click of a two-click slider snaps
  but records no coincident; shared / moving tracks are carried by nobody (symmetric "track body
  carries owners" rule is the natural extension); weld toggle on instance pins isn't blocked.
- **Holes**: hole-aware picking option; hole edges aren't containment-checked (vertices only).
- **View**: screen-aligned marquee when rotated; persistent angle badge; 5° nudge hotkey.
- **DXF**: SPLINE sampling, INSERT/blocks, ellipses. Copy/paste doesn't carry driven measurements.
- Actuator/motor speed editing while in sim (selection clears on mode change).
