# Handoff — open work

What has been **discussed and decided but not built yet**. Read with PROJECT_INSTRUCTIONS.md
(current state); this file holds only the unfinished part, so a future session can pick a piece
up without re-doing the brainstorm. In order: the rest of the **shortcut remap** (next up), the
**shape tools + roles** roadmap (phase 1 shipped 2026-09-13, format v20), the Tangential
follow-up and the sim-mode toolbar.

## The rest of the shortcut remap

The 2026-09-15 remap is **applied and shipped** — Fixed `F`, Circle `C`, Line `L`, Arc `A`,
Fit view `Shift+F`, Rotate view `Ctrl+R`, Polyline (cut) `Ctrl+U`, Connect `Ctrl+Shift+C`,
Linear actuator `Shift+A`, Motor `Shift+M`, Rail unassigned. What is left:

- **Rail lost its key and nothing took `K`.** The board did that; the earlier plan had *Linear
  actuator* as the one to drop. Ask before assuming it was meant — `K` is free either way.
- **Connect is now `Ctrl+Shift+C`**, a three-key chord for a tool used constantly. Worth a
  second look on the board.
- **23 commands have no key at all** (`public/keymap.json`, empty `bindings`): Export, the
  mirrors, the grid / snap / badge / measurement toggles, zoom in / out, theme, Create component,
  Component browser, Auto-pause, Delete everything, the two grid menus, Rail, and the Shortcuts
  panel itself. All have a working `run()` — Subtract and Intersect too, since 2026-09-22 — so
  the board can hand any of them a letter and it works.

### Working with the board

**Seed KeyMapper from `public/keymap.json`, never from an older capture.** The board's original
file was hand-written on 2026-09-14 by reading the old `if`-ladder (`"source":
"src/main.ts:8005-8283"`), and its ids were its own invention — `save`, `polybody`, `refline`,
`radplus`, `sidesup`. None of them match a registry id, so importing that file applies *nothing*
(the panel says "0 commands; 80 the app doesn't know, ignored"). The file in the repo carries
the real ids; load that, rearrange, bring it back — it is also what ships, so the round trip is
the whole job now. The artifact is
`cc79ffcf-efa4-4d38-9bd7-e2aade7901a4`.

### How to apply a layout

1. **To ship it:** drop the board's export in as `public/keymap.json` — that file *is* the
   shipped keymap since 2026-09-15, so nothing in `src/` is edited for a remap. Then
   `npm run keymap:docs` (it rewrites the README's tools table and the manual's shortcut list).
   Bring the letters in `scripts/keymap-live.ts` into line — they are written out by hand on
   purpose — and run `npm run keymap:live`. Also sweep README prose *outside* the tools table:
   the generator only owns the Shortcut column, and the Navigate / Help sections name keys in
   sentences. `npm test` fails if the file and the registry disagree about which commands exist,
   if the tooltips or the README table are behind, or if two commands want one slot in
   overlapping contexts, so a clash cannot ship. `npm run keymap:file` is only needed after
   *adding a command*: it rewrites the file's metadata and keeps every binding.
2. **To try one:** import it through the Shortcuts panel (File group). That writes
   `localStorage["disjointed:keymap"]` for this browser only; Reset undoes it.

An imported keymap that clashes is **accepted, not refused** — the toast counts the clashes, and
on a clashing slot the first command in registry order answers. Deliberate: refusing a file is
worse than showing the user what they did.

### Watch out

- **Ids are the contract.** Renaming a `CommandSpec.id` silently drops that command's binding
  from every saved keymap. Rename labels freely, never ids. (In `public/keymap.json` a rename is
  no longer silent — `npm test` reports the old id as one the app doesn't have — but the binding
  is lost all the same.)
- **Spelling rules are shared with KeyMapper** and specified in its README, not here: Shift is
  part of a letter's or named key's slot and never part of a character's (`?` arrives as `"?"`
  with shiftKey true). `src/keymap.ts` implements them; a change has to happen in both projects.
- Still out of scope, as decided: chords / key sequences, predicates expressed in the file (they
  stay in code as `enabled()`), and rebinding pointer gestures.

## Shape tools & roles — decisions already made (don't re-open)

- **Roles are explicit and sticky** (toolbar switch, keys 1 / 2 / 3), never inferred from
  where a click lands: bodies routinely overlap other bodies, so "first click inside a body
  = cut" is wrong. Mode errors are mitigated instead by the role-styled preview (tint /
  hatching + target highlight / dash-dot), the badge beside the cursor, and the one-shot
  **Ctrl on the first click** flip between Body and Cut.
- **Hole became Cut**: a polygon difference. Inside the material → a plain editable hole
  (exact spec, a disk stays parametric); crossing the outline → a notch; a cut that would
  sever the body is refused (use Split). Target = selected body, else topmost body under the
  first click, else the body the finished shape lies in.
- **Line, arc and text are reference-only** for now. Slots in the Reference role draw the
  axis segment only.
- **Text is for labels first** (anchored to a body when clicked on one; otherwise free).
  Engraving / cutting text is a later phase.
- Curved slots are a nice-to-have, not a priority.

## Not built yet — agreed phases, in order

### Phase 2 — arcs (and ellipses) on outlines
- Add a per-edge **bulge** (DXF convention `tan(sweep/4)`, already what the exporter writes
  and the importer reads) to body / hole control polygons, so an edge can be a true arc.
  Touches: fillet derivation, vertex/pose solvers (arc edges as constraint refs), hit-testing,
  export (exact `A` / bulge output), the DXF importer (stop sampling non-tangent / ≥180° arcs).
- Then: an **Arc edge** in the Body / Cut roles, **curved slots** (arc axis + width), D-shapes.
- **Ellipses**: not an arc — the edge record needs a *kind*, not just a bulge. Keep the edge
  representation open for that when bulges land. DXF has an ELLIPSE entity; export as such or
  sampled.
- Reference arcs / circles already exist (v20) and could become the pick targets for "arc
  through these reference points".

### Phase 3 — regions from reference segments
- Detect **closed chains** of reference segments / arcs (the DXF importer's LINE/ARC
  auto-chaining is reusable) and let the user click inside a region to make a body or a cut
  from it (sketch-profile workflow). Would make Line / Arc useful in the material roles.

### Phase 4 — cut / engraved text
- Needs an outline-font library (e.g. opentype.js) and a bundled **stencil** face: under the
  loop-nesting rule an island inside a hole starts a new solid, so the middle of an "O" cut
  into a plate would become loose material. Text as a *body* has the same problem plus the
  one-outline-per-body model (letters are disjoint → one body per letter, or welded).
- Engraving (a non-cutting layer for the laser) would need an export layer concept.

## Smaller follow-ups from phase 1

- **Role conversions**: promote a closed reference polygon / circle to a body; extract a hole
  as a body of the same shape (a peg for a hole). Both make the initial role choice
  non-binding. (The third one, a body as a cutter on another body, shipped as **Subtract** on
  2026-09-22.)
- **Cut several bodies at once** (a modifier on the closing click); today one target.
- **General-path cuts dissolve the body's patterns** (hole indices are rebuilt). Keeping
  patterns whose seed and members are untouched would need index remapping.
- A rounded cutter that crosses the outline arrives **sampled** (polygonal notch corners);
  with bulge edges (phase 2) the notch could keep exact arcs.
- **Text**: no angle of its own (free labels are horizontal; anchored ones follow the body),
  a fixed font, no alignment options, width estimated (`0.58 × size × chars`) for hit tests.
  Labels don't travel with copy/paste or into component definitions (guides never did).
- **Reference geometry as constraint targets**: tangent shipped (2026-09-13 — the
  `guideCircle` ref covers reference circles and arcs, `disk` covers disk bodies and round
  holes). Still missing: point-on-circle, concentric, circle–circle tangency, and an arc's
  centre as a reference *point* (derived, not a solver var — a `guideCircle` ref resolves to
  it, but nothing can be made coincident with it).
- **Shortcuts** for the new tools were Shift+letter picks; the 2026-09-15 remap moved Circle to
  `C`, Line to `L` and Arc to `A`, leaving Shift+B rectangle, Shift+P polygon, Shift+S slot,
  Shift+T text. Each is one edit in `keys` in `src/commands.ts`, everything else generated.
- Manual: the shape tools have no gesture illustrations (before / mid / after) yet; they
  would fit the `scripts/manual/shots.ts` pipeline (the role-styled preview is exposed
  through `RenderInput.shapeDraft`, and the `implicit-drag` shot is a worked mid-gesture
  example).

## Subtract / Intersect — open questions (2026-09-22, not decided)

Both shipped with Combine's conventions; none of these was discussed, they are the places a
user might push back:

## Tangential constraint — needs another pass (2026-09-13)

Shipped in the "Tangential constraint" commit (design in PROJECT_INSTRUCTIONS.md, tests in
`scripts/tangent-constraint.ts`), but the user's field tests with a scene of their own (two long
reference lines + a reference arc: the rounded end of a slot) found it still wanting. The
session closed before the specifics were written down — **ask what was seen first**, then
start from these known weak spots:

- **The arc is three points** (`a` / `m` / `b`), not centre / radius / angles. When a point is
  held, the others take a Newton step along the line's normal and the radius and sweep change
  as a side effect rather than being parameters the user controls. The structural fix is a
  real arc parametrisation in the sketch solver — either new variable kinds (centre, radius,
  start / end angle) or the regular-polygon route: keep the three points and add a coupling
  item that holds radius / sweep unless a size dimension says otherwise.
- **Who moves in a blend.** With everything free, a coincident that glues an arc end to a
  line end makes the *line* swing about the shared end (its far end moves) as much as the arc
  adapts — a 50/50 split by rank. CAD users expect the arc to absorb it. Options: a lower
  mobility preference for reference arcs, or the arc taking the whole turn whenever both are
  free guides (Fixed / H / V on the line already forces that).
- **No held side.** A circle dragged through its line re-attaches on the far side; a stored
  side (as a driving dimension keeps) would make it stick.
- **Coverage.** Circles are disks, round holes, reference circles and arcs only: no tangency
  to a body's rounded corner (the fillet arc), no circle–circle or arc–arc (G1 between two
  arcs), no point-on-circle / concentric.
- **Pinned-end detection** (`pinnedArcEnd`) recognises a coincident between an arc end and
  the line's end, its midpoint, or the line itself — not an arc end coincident with a joint
  that merely sits on the line.
- **Tuning.** Convergence rests on Newton probes with hand-set floors (0.1 · r for the arc,
  0.1 · L for the line) and a 0.5 rad cap per sweep. `sketchConfig.trace` prints the worst
  residual per sweep — run it on the field file before touching a number.

## Toolbar — next step: sim mode

The toolbar was regrouped into draggable groups **for draw mode only** (the harder half, by
agreement). Sim mode works and inherits the rack, but was not designed:

- Today it shows the mode-neutral groups (File, Mode, Grid, Measure, Snapping, View) plus
  **Animation** (run + auto-pause), with the four solver-tuning knobs in the properties strip.
  Per-mode visibility is the `draw-only` / `sim-only` class on a section, so adding or moving a
  sim group is markup, not code.
- Open questions: whether the solver knobs deserve their own (collapsible?) group rather than the
  properties strip; whether sim wants a selection / drag-behaviour group; whether the two modes
  should keep **separate saved orders** (one `disjointed:toolbarOrder` list today, shared).
- Not decided: whether the mode button should show the mode you are *in* instead of the one it
  switches *to* (the theme button's convention was followed).
